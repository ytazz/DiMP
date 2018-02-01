#include <DiMP/Graph/Avoid.h>
#include <DiMP/Graph/Geometry.h>
#include <DiMP/Graph/Object.h>
#include <DiMP/Graph/Graph.h>
#include <DiMP/Render/Config.h>

#include <Foundation/UTPreciseTimer.h>
static UTPreciseTimer ptimer;

namespace DiMP{;

const real_t inf = numeric_limits<real_t>::max();

//-------------------------------------------------------------------------------------------------
// GeometryPair

AvoidKey::GeometryPair::GeometryPair(){
	info0 = 0;
	info1 = 0;
	dmin  = inf; 
	dmax  = inf;
	dist  = inf;
	con_p = 0;
	con_v = 0;
}

//-------------------------------------------------------------------------------------------------
// AvoidKey

AvoidKey::AvoidKey(){
	//con_p = 0;
	//con_v = 0;
}

void AvoidKey::AddCon(Solver* solver){
	AvoidTask* task = (AvoidTask*)node;
	
	for(auto& g0 : obj0->geoInfos) for(auto& g1 : obj1->geoInfos){
		GeometryPair gp;
		gp.info0 = &g0;
		gp.info1 = &g1;
		geoPairs.push_back(gp);
	}
	for(auto& gp : geoPairs){
		gp.con_p = new AvoidConP(solver, name + "_p", this, &gp, node->graph->scale.pos_t);
		gp.con_v = new AvoidConV(solver, name + "_v", this, &gp, node->graph->scale.vel_t);
	}
}

void AvoidKey::Prepare(){
	TaskKey::Prepare();

	AvoidTask* task = (AvoidTask*)node;
	for(auto& gp : geoPairs){
		gp.con_p->enabled = task->param.avoid_p;
		gp.con_v->enabled = task->param.avoid_v;
	}

	if(relation == Inside){				
		ptimer.CountUS();
		int nsphere  = 0;
		int nbox     = 0;
		int ngjk     = 0;
		int nactive  = 0;
		for(auto& gp : geoPairs){
			if( gp.con_p->enabled == false &&
				gp.con_v->enabled == false )
				continue;

			// bsphere‚Å”»’è
			real_t d    = (gp.info0->bsphereCenterAbs - gp.info1->bsphereCenterAbs).norm();
			real_t rsum = gp.info0->geo->bsphereRadius + gp.info1->geo->bsphereRadius;
			if(d > rsum){
				gp.con_p->active = false;
				gp.con_v->active = false;
				nsphere++;
				continue;
			}
			// bbox‚Å”»’è
			bool intersect = true;
			for(int i = 0; i < 3; i++){
				if( gp.info0->bbmin.x > gp.info1->bbmax.x || 
					gp.info1->bbmin.x > gp.info0->bbmax.x ){
					intersect = false;
					break;
				}
			}
			if(!intersect){
				gp.con_p->active = false;
				gp.con_v->active = false;
				nbox++;
				continue;
			}

			CalcNearest(
				gp.info0->geo, gp.info1->geo,
				gp.info0->poseAbs, gp.info1->poseAbs,
				gp.sup0, gp.sup1, gp.dist
			);
			
			if(gp.dist > 0.0){
				gp.con_p->active = false;
				gp.con_v->active = false;
				ngjk++;
			}
			else{
				const real_t eps = 1.0e-10;
				vec3_t diff = gp.sup1 - gp.sup0;
				real_t dnorm = diff.norm();
				if(dnorm < eps){
					gp.con_p->active = false;
					gp.con_v->active = false;
				}
				gp.normal = diff/dnorm;
				gp.con_p->active = true;
				gp.con_v->active = true;
				nactive++;
			}
		}
		int timeGjk = ptimer.CountUS();

		//DSTR << "bsphere: " << nsphere << " bbox: " << nbox << " gjk: " << ngjk << " active: " << nactive << endl;
		//DSTR << "tgjk: " << timeGjk << endl;
	}
	else{
		for(auto& gp : geoPairs){
			gp.con_v->active = false;
			gp.con_p->active = false;
		}
	}
}

void AvoidKey::Draw(Render::Canvas* canvas, Render::Config* conf){
	Vec3f p0, p1;
	
	if(relation == Inside && conf->Set(canvas, Render::Item::Avoid, node)){
		for(auto& gp : geoPairs){
			// bsphere‚ÅŽ}Š ‚è‚³‚ê‚Ä‚¨‚ç‚¸C‚©‚ÂŒð·‚à‚µ‚Ä‚¢‚È‚¢ê‡‚Ésupport point‚ðŒ‹‚Ôü‚ð•`‰æ
			if(/*gp.con_p->active*/gp.dist != inf && gp.dist > 0.0){
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
	avoid_p = true;
	avoid_v = true;
	dmin    = 0.0;
}

AvoidTask::AvoidTask(Object* _obj0, Object* _obj1, TimeSlot* _time, const string& n)
	:Task(_obj0, _obj1, _time, n){
	
}

void AvoidTask::Prepare(){
	Task::Prepare();

	// Å¬Ú‹ß‹——£ŒÝ‚¢‚ÌŠOÚ‰~”¼Œa‚Ì˜a
	//dmin = obj0->bsphere + obj1->bsphere;
}

//-------------------------------------------------------------------------------------------------
// constructors

AvoidCon::AvoidCon(Solver* solver, ID id, AvoidKey* _key, AvoidKey::GeometryPair* _gp, real_t _scale):Constraint(solver, 1, id, _scale){
	key = _key;
	gp  = _gp;
}

AvoidConP::AvoidConP(Solver* solver, const string& _name, AvoidKey* _key, AvoidKey::GeometryPair* _gp, real_t _scale):
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

AvoidConV::AvoidConV(Solver* solver, const string& _name, AvoidKey* _key, AvoidKey::GeometryPair* _gp, real_t _scale):
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
	y[0] = gp->dist;
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