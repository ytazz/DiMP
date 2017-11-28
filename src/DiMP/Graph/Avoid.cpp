#include <DiMP/Graph/Avoid.h>
#include <DiMP/Graph/Object.h>
#include <DiMP/Graph/Graph.h>
#include <DiMP/Render/Config.h>

namespace DiMP{;

//-------------------------------------------------------------------------------------------------
// AvoidKey

AvoidKey::AvoidKey(){
	con_p = 0;
	con_v = 0;
}

void AvoidKey::AddCon(Solver* solver){
	// create constraints
	con_p = new AvoidConP(solver, name + "_p", this, node->graph->scale.pos_t);
	con_v = new AvoidConV(solver, name + "_v", this, node->graph->scale.vel_t);
}

void AvoidKey::Prepare(){
	TaskKey::Prepare();

	if(relation == Inside){
		con_p->active = true;
		con_v->active = true;

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
	}
	else{
		con_v->active = false;
		con_p->active = false;
	}
}

//-------------------------------------------------------------------------------------------------
// AvoidTask

AvoidTask::AvoidTask(Connector* _con0, Connector* _con1, TimeSlot* _time, const string& n)
	:Task(_con0, _con1, _time, n){
	
	dmin = 0.0;
}

void AvoidTask::Prepare(){
	Task::Prepare();

	// Å¬Ú‹ß‹——£ŒÝ‚¢‚ÌŠOÚ‰~”¼Œa‚Ì˜a
	//dmin = obj0->bsphere + obj1->bsphere;
}

//-------------------------------------------------------------------------------------------------
// constructors

AvoidCon::AvoidCon(Solver* solver, ID id, AvoidKey* _key, real_t _scale):Constraint(solver, 1, id, _scale){
	key  = _key;
}

AvoidConP::AvoidConP(Solver* solver, const string& _name, AvoidKey* _key, real_t _scale):
	AvoidCon(solver, ID(ConTag::AvoidP, _key->node, _key->tick, _name), _key, _scale){
	// translational, position, scalar constraint
	ObjectKey::OptionS opt;
	opt.tp = true ;
	opt.rp = true ;
	opt.tv = false;
	opt.rv = false;
	key->obj0->AddLinks(this, opt);
	key->obj1->AddLinks(this, opt);
}

AvoidConV::AvoidConV(Solver* solver, const string& _name, AvoidKey* _key, real_t _scale):
	AvoidCon(solver, ID(ConTag::AvoidV, _key->node, _key->tick, _name), _key, _scale){
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
	opt.k_tp = -key->normal; opt.k_rp = -(key->prox0 - key->obj0->pos_t->val) % key->normal; key->obj0->CalcCoef(this, opt, i);
	opt.k_tp =  key->normal, opt.k_rp =  (key->prox1 - key->obj1->pos_t->val) % key->normal; key->obj1->CalcCoef(this, opt, i);
}

void AvoidConV::CalcCoef(){
	uint i = 0;
	ObjectKey::OptionS opt;
	opt.tp = true ;
	opt.rp = true ;
	opt.tv = false;
	opt.rv = false;
	opt.k_tv = -key->normal; opt.k_rv = -(key->prox0 - key->obj0->pos_t->val) % key->normal; key->obj0->CalcCoef(this, opt, i);
	opt.k_tv =  key->normal; opt.k_rv =  (key->prox1 - key->obj1->pos_t->val) % key->normal; key->obj1->CalcCoef(this, opt, i);
}

//-------------------------------------------------------------------------------------------------
// CalcDeviation

void AvoidConP::CalcDeviation(){
	y[0] = key->depth;
	if(y[0] < 0.0)
		active = true;
	else{
		active = false;
		y[0] = 0.0;
	}
}

void AvoidConV::CalcDeviation(){
	Constraint::CalcDeviation();

	// AvoidConP‚ªactive‚Ì‚Æ‚«‚Ì‚Ýactive
	if(key->depth <= 0.0 && y[0] < 0.0)
		active = true;
	else{
		active = false;
	}
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