#include <DiMP/Graph/Avoid.h>
#include <DiMP/Graph/Geometry.h>
#include <DiMP/Graph/Object.h>
#include <DiMP/Graph/Graph.h>
#include <DiMP/Render/Config.h>

#include <sbtimer.h>
static Timer timer;

#include <omp.h>

namespace DiMP{;

const real_t inf = numeric_limits<real_t>::max();

//-------------------------------------------------------------------------------------------------
// AvoidKey

AvoidKey::AvoidKey(){

}

void AvoidKey::AddCon(Solver* solver){
	AvoidTask* task = (AvoidTask*)node;
	
	/*int ngeo0 = obj0->geoInfos.size();
	int ngeo1 = obj1->geoInfos.size();
	geoPairs.resize(ngeo0 * ngeo1);
	for(int i0 = 0; i0 < ngeo0; i0++)for(int i1 = 0; i1 < ngeo1; i1++){
		GeometryPair& gp = geoPairs[ngeo1*i0 + i1];
		gp.info0 = &obj0->geoInfos[i0];
		gp.info1 = &obj1->geoInfos[i1];
	}*/
	con_p.resize(task->param.nactive_max);
	con_v.resize(task->param.nactive_max);

	stringstream ss;
	for(int i = 0; i < task->param.nactive_max; i++){
		ss.str("");
		ss << name << "_p" << i;
		con_p[i] = new AvoidConP(solver, ss.str(), this, 0, node->graph->scale.pos_t);

		ss.str("");
		ss << name << "_v" << i;
		con_v[i] = new AvoidConV(solver, ss.str(), this, 0, node->graph->scale.vel_t);
		
		solver->AddCostCon(con_p[i], tick->idx);
		solver->AddCostCon(con_v[i], tick->idx);

		con_p[i]->enabled = task->param.avoid_p;
		con_v[i]->enabled = task->param.avoid_v;
	}
}

void AvoidKey::PrepareStep(){
	TaskKey::PrepareStep();

	AvoidTask* task = (AvoidTask*)node;

	if(!task->param.avoid_p && !task->param.avoid_v)
		return;

	// temporarily inactivate all constraints
	for(int i = 0; i < task->param.nactive_max; i++){
		con_p[i]->gp = 0;
		con_v[i]->gp = 0;
		con_p[i]->active = false;
		con_v[i]->active = false;
	}
	
	if( relation == Inside ){
		int nocttree = 0;
		int nsphere  = 0;
		int nbox     = 0;
		int ngjk     = 0;
		int nactive  = 0;
		//GeometryPair* gpmax = 0;
		//real_t        dmax  = 0.0;
		gpactive.clear();

		/*ptimer.CountUS();
		ExtractGeometryPairs(
			obj0->geoInfos, obj0->edgeInfos,
			obj1->geoInfos, obj1->edgeInfos,
			geoPairs);
		int timeExtract = ptimer.CountUS();
		*/
		//DSTR << "geo0: " << obj0->geoInfos.size() << " geo1: " << obj1->geoInfos.size() << " geo pair: " << geoPairs.size() << endl;

		timer.CountUS();
		for(int gp_idx = 0; gp_idx < geoPairs.size(); gp_idx++){
			GeometryPair& gp = geoPairs[gp_idx];
	
			// octtreeで判定
			//if( !gp.info0->octNode->IsAncestor(gp.info1->octNode) ){
			//	gp.cullOcttree = true;
			//	nocttree++;
			//	continue;
			//}

			// bsphereで判定
			real_t d    = (gp.info0->bsphereCenterAbs - gp.info1->bsphereCenterAbs).norm();
			real_t rsum = gp.info0->geo->bsphereRadius + gp.info1->geo->bsphereRadius;
			gp.cullSphere = (d - rsum > task->param.dmin);
			if(gp.cullSphere){
			    //printf("culled by bsphere: d=%f rsum=%f dmin=%f c0:%f %f %f  c1:%f %f %f\n",
			    // d, rsum, task->param.dmin,
			    // gp.info0->bsphereCenterAbs.x, gp.info0->bsphereCenterAbs.y, gp.info0->bsphereCenterAbs.z, 
			    // gp.info1->bsphereCenterAbs.x, gp.info1->bsphereCenterAbs.y, gp.info1->bsphereCenterAbs.z
			    // );
				nsphere++;
				continue;
			}
			// bboxで判定
			gp.cullBox = false;
			for(int i = 0; i < 3; i++){
				if( gp.info0->bbmin[i] > gp.info1->bbmax[i] + task->param.dmin || 
					gp.info1->bbmin[i] > gp.info0->bbmax[i] + task->param.dmin ){
					gp.cullBox = true;
					break;
				}
			}
			if(gp.cullBox){
				nbox++;
				continue;
			}

			CalcNearest(
				gp.info0->geo, gp.info1->geo,
				gp.info0->poseAbs, gp.info1->poseAbs,
				gp.sup0, gp.sup1, gp.dist
			);
			gp.cullGjk = (gp.dist > task->param.dmin);
			if(gp.cullGjk){
				ngjk++;
			}
			else{
				const real_t eps = 1.0e-10;
				vec3_t diff = gp.sup1 - gp.sup0;
				real_t dnorm = diff.norm();
				if(dnorm > eps){
					gp.normal = diff/dnorm;
					nactive++;
					gpactive.push_back(&gp);
					
					printf("active gp: %s - %s  sup0: %f %f %f  sup1: %f %f %f\n",
					  gp.info0->geo->name.c_str(),
					  gp.info1->geo->name.c_str(),
					  gp.sup0.x, gp.sup0.y, gp.sup0.z, 
					  gp.sup1.x, gp.sup1.y, gp.sup1.z
					);
					
					//if(dnorm > dmax){
					//	gpmax = &gp;
					//	dmax  = dnorm;
					//}
				}
			}
		}
		int timeEnum = timer.CountUS();

		//if(gpmax){
		//	con_p->gp     = gpmax;
		//	con_v->gp     = gpmax;
		//	con_p->active = true;
		//	con_v->active = true;
		//}
		//else{
		//	con_p->active = false;
		//	con_v->active = false;
		//}
		
		//DSTR << "bsphere: " << nsphere << " bbox: " << nbox << " gjk: " << ngjk << " active: " << nactive << endl;
		//if(gpmax){
		//	DSTR << " dist: " << gpmax->dist << " normal: " << gpmax->normal << endl;
		//}
		//DSTR << "timeExtract: " << timeExtract << " timeEnum: " << timeEnum << endl;
		
		//if(!geoPairs.empty())
		//    printf("bsphere:%d bbox:%d gjk:%d active:%d\n", nsphere, nbox, ngjk, nactive);
	}
	//else{
	//	con_p->active = false;
	//	con_v->active = false;
	//}

	// partially sort active geometry pairs in ascending order of signed distance
	int nactive = std::min((int)gpactive.size(), task->param.nactive_max);
	partial_sort(gpactive.begin(), gpactive.begin() + nactive, gpactive.end(),
		[](auto lhs, auto rhs){ return lhs->dist < rhs->dist; });
	
	for(int i = 0; i < nactive; i++){
		con_p[i]->gp = gpactive[i];
		con_v[i]->gp = gpactive[i];
		con_p[i]->active = true;
		con_v[i]->active = true;
	}
}

void AvoidKey::Draw(Render::Canvas* canvas, Render::Config* conf){
	Vec3f p0, p1;
	
	if(relation == Inside && conf->Set(canvas, Render::Item::Avoid, node)){
		for(auto& gp : geoPairs){
			// bsphereで枝刈りされておらず，かつ交差もしていない場合にsupport pointを結ぶ線を描画
			if(!gp.cullSphere && !gp.cullBox && !gp.cullGjk){
				p0 = gp.sup0;
				p1 = gp.sup1;
				canvas->Line(p0, p1);
			}
		}
	}
}

//-------------------------------------------------------------------------------------------------
// AvoidTask

AvoidTask::Param::Param(){
	avoid_p     = true;
	avoid_v     = true;
	dmin        = 0.0;
	nactive_max = 1;
}

AvoidTask::AvoidTask(Object* _obj0, Object* _obj1, TimeSlot* _time, const string& n)
	:Task(_obj0, _obj1, _time, n){
	
	graph->avoids.Add(this);
}

AvoidTask::~AvoidTask(){
	graph->avoids.Remove(this);
}

//-------------------------------------------------------------------------------------------------
// constructors

AvoidCon::AvoidCon(Solver* solver, ID id, AvoidKey* _key, GeometryPair* _gp, real_t _scale):Constraint(solver, 1, id, Constraint::Type::InequalityPenalty, _scale){
	key = _key;
	gp  = _gp;
}

AvoidConP::AvoidConP(Solver* solver, const string& _name, AvoidKey* _key, GeometryPair* _gp, real_t _scale):
	AvoidCon(solver, ID(ConTag::AvoidP, _key->node, _key->tick, _name), _key, _gp, _scale){
	// translational, position, scalar constraint
	ObjectKey::OptionS opt;
	opt.tp = true ;
	opt.rp = true ;
	opt.tv = false;
	opt.rv = false;
	key->obj0->AddLinks(this, opt);
	key->obj1->AddLinks(this, opt);
}

AvoidConV::AvoidConV(Solver* solver, const string& _name, AvoidKey* _key, GeometryPair* _gp, real_t _scale):
	AvoidCon(solver, ID(ConTag::AvoidV, _key->node, _key->tick, _name), _key, _gp, _scale){
	// translational, velocity, scalar constraint
	ObjectKey::OptionS opt;
	opt.tp = false;
	opt.rp = false;
	opt.tv = true ;
	opt.rv = true ;
	key->obj0->AddLinks(this, opt);
	key->obj1->AddLinks(this, opt);
}

//-------------------------------------------------------------------------------------------------
// CalcCoef

void AvoidConP::CalcCoef(){
	if(!active)
		return;
	uint i = 0;
	ObjectKey::OptionS opt;
	opt.tp = true ;
	opt.rp = true ;
	opt.tv = false;
	opt.rv = false;
	opt.k_tp = -gp->normal; opt.k_rp = -(gp->sup0 - key->obj0->pos_t->val) % gp->normal; key->obj0->CalcCoef(this, opt, i);
	opt.k_tp =  gp->normal, opt.k_rp =  (gp->sup1 - key->obj1->pos_t->val) % gp->normal; key->obj1->CalcCoef(this, opt, i);
}

void AvoidConV::CalcCoef(){
	if(!active)
		return;
	uint i = 0;
	ObjectKey::OptionS opt;
	opt.tp = true ;
	opt.rp = true ;
	opt.tv = false;
	opt.rv = false;
	opt.k_tv = -gp->normal; opt.k_rv = -(gp->sup0 - key->obj0->pos_t->val) % gp->normal; key->obj0->CalcCoef(this, opt, i);
	opt.k_tv =  gp->normal; opt.k_rv =  (gp->sup1 - key->obj1->pos_t->val) % gp->normal; key->obj1->CalcCoef(this, opt, i);
}

//-------------------------------------------------------------------------------------------------
// CalcDeviation

void AvoidConP::CalcDeviation(){
	if(!active){
		y[0] = 0.0;
		return;
	}
	y[0] = gp->dist - ((AvoidTask*)key->node)->param.dmin;
}

void AvoidConV::CalcDeviation(){
	if(!active){
		y[0] = 0.0;
		return;
	}
	Constraint::CalcDeviation();
}

//-------------------------------------------------------------------------------------------------
// Projection

void AvoidConP::Project(real_t& l, uint k){
	if(l < 0.0)
		l = 0.0;
}

void AvoidConV::Project(real_t& l, uint k){
	if(l < 0.0)
		l = 0.0;
}

}
