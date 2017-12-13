#include <DiMP/Graph/Avoid.h>
#include <DiMP/Graph/Geometry.h>
#include <DiMP/Graph/Object.h>
#include <DiMP/Graph/Graph.h>
#include <DiMP/Render/Config.h>

namespace DiMP{;

const real_t inf = numeric_limits<real_t>::max();

//-------------------------------------------------------------------------------------------------
// AvoidKey

AvoidKey::AvoidKey(){
	//con_p = 0;
	//con_v = 0;
}

void AvoidKey::AddCon(Solver* solver){
	AvoidTask* task = (AvoidTask*)node;
	
	for(int i0 = 0; i0 < (int)obj0->geoInfos.size(); i0++) for(int i1 = 0; i1 < (int)obj1->geoInfos.size(); i1++){
		GeometryPair gp;
		gp.info0 = &obj0->geoInfos[i0];
		gp.info1 = &obj1->geoInfos[i1];
		geoPairs.push_back(gp);
	}
	for(int i = 0; i < (int)geoPairs.size(); i++){
		GeometryPair& gp = geoPairs[i];
		gp.con_p = new AvoidConP(solver, name + "_p", this, &gp, node->graph->scale.pos_t);
		gp.con_v = new AvoidConV(solver, name + "_v", this, &gp, node->graph->scale.vel_t);
	}
}

void AvoidKey::Prepare(){
	TaskKey::Prepare();

	if(relation == Inside){
		AvoidTask* task = (AvoidTask*)node;
				
		// ‚·‚×‚Ä‚ÌŒ`ó‚Ì‘Î‚ğbsphere‚Ì‹——£‚Åƒ\[ƒg
		for(int i = 0; i < (int)geoPairs.size(); i++){
			GeometryPair& gp = geoPairs[i];
			real_t d    = (gp.info0->bsphereCenterAbs - gp.info1->bsphereCenterAbs).norm();
			real_t rsum = gp.info0->geo->bsphereRadius + gp.info1->geo->bsphereRadius;
			gp.dmin = d - rsum;
			gp.dmax = d + rsum;
			gp.dist = inf;
		}
		
		// ‹——£‚Ì‰ºŒÀ‚Ì¸‡‚ÉÕ“Ë”»’è
		for(int i = 0; i < (int)geoPairs.size(); i++){
			GeometryPair& gp = geoPairs[i];

			// ‰ºŒÀ‚ª³‚È‚çŒğ·‚È‚µ
			if(gp.dmin > 0.0){
				gp.con_p->active = false;
				gp.con_v->active = false;
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
			}
		}

		/*
		AvoidTask* task = (AvoidTask*)node;
		vec3_t r = obj1->pos_t->val - obj0->pos_t->val;
			
		real_t rnorm = r.norm();
		const real_t eps = 1.0e-10;
		if(rnorm < eps)
				normal = vec3_t(1.0, 0.0, 0.0);
		else normal = r/rnorm;

		prox0 = obj0->pos_t->val + task->con0->obj->bsphere * normal;
		prox1 = obj1->pos_t->val - task->con1->obj->bsphere * normal;
		depth = rnorm - (task->con0->obj->bsphere + task->con1->obj->bsphere);
		*/
	}
	else{
		for(int i = 0; i < (int)geoPairs.size(); i++){
			GeometryPair& gp = geoPairs[i];
			gp.con_v->active = false;
			gp.con_p->active = false;
		}
	}
}

void AvoidKey::Draw(Render::Canvas* canvas, Render::Config* conf){
	Vec3f p0, p1;
	
	if(relation == Inside && conf->Set(canvas, Render::Item::Avoid, node)){
		for(int i = 0; i < (int)geoPairs.size(); i++){
			GeometryPair& gp = geoPairs[i];
			// bsphere‚Å}Š ‚è‚³‚ê‚Ä‚¨‚ç‚¸C‚©‚ÂŒğ·‚à‚µ‚Ä‚¢‚È‚¢ê‡‚Ésupport point‚ğŒ‹‚Ôü‚ğ•`‰æ
			if(gp.dist != inf && gp.dist > 0.0){
				p0 = gp.sup0;
				p1 = gp.sup1;
				canvas->Line(p0, p1);
			}
		}
	}
}

//-------------------------------------------------------------------------------------------------
// AvoidTask

AvoidTask::AvoidTask(Object* _obj0, Object* _obj1, TimeSlot* _time, const string& n)
	:Task(_obj0, _obj1, _time, n){
	
	dmin = 0.0;
}

void AvoidTask::Prepare(){
	Task::Prepare();

	// Å¬Ú‹ß‹——£Œİ‚¢‚ÌŠOÚ‰~”¼Œa‚Ì˜a
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