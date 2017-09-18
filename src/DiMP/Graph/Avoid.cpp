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

	con_p->active = (relation == Inside);
	con_v->active = (relation == Inside);
}

//-------------------------------------------------------------------------------------------------
// AvoidTask

AvoidTask::AvoidTask(Object* _obj0, Object* _obj1, TimeSlot* _time, const string& n)
	:Task(_obj0, _obj1, _time, n){
	
	dmin = 0.0;
}

void AvoidTask::Prepare(){
	Task::Prepare();

	// Å¬Ú‹ß‹——£ŒÝ‚¢‚ÌŠOÚ‰~”¼Œa‚Ì˜a
	dmin = obj0->bsphere + obj1->bsphere;
}

//-------------------------------------------------------------------------------------------------
// constructors

AvoidCon::AvoidCon(Solver* solver, ID id, AvoidKey* _key, real_t _scale):Constraint(solver, 1, id, _scale){
	key  = _key;
}

AvoidConP::AvoidConP(Solver* solver, const string& _name, AvoidKey* _key, real_t _scale):
	AvoidCon(solver, ID(ConTag::AvoidP, _key->node, _key->tick, _name), _key, _scale){
	key->obj0->AddLinks(this, true, true, false);
	key->obj1->AddLinks(this, true, true, false);
}

AvoidConV::AvoidConV(Solver* solver, const string& _name, AvoidKey* _key, real_t _scale):
	AvoidCon(solver, ID(ConTag::AvoidV, _key->node, _key->tick, _name), _key, _scale){
	key->obj0->AddLinks(this, true, false, false);
	key->obj1->AddLinks(this, true, false, false);
}

//-------------------------------------------------------------------------------------------------
// CalcCoef

void AvoidCon::CalcCoef(){
	AvoidTask* task = (AvoidTask*)owner;
	
	// ˆÊ’u•Î·‚ð³‹K‰»‚µ‚½ƒxƒNƒgƒ‹
	vec3_t d = key->obj1->pos_t->val - key->obj0->pos_t->val;
	real_t dnorm = d.norm();
	const real_t eps = 1.0e-10;
	if(dnorm < eps)
		 d  = vec3_t(1.0, 0.0, 0.0);
	else d *= (1.0/dnorm);

	uint i = 0;
	key->obj0->CalcCoef(this, true, -d, i);
	key->obj1->CalcCoef(this, true,  d, i);
}

//-------------------------------------------------------------------------------------------------
// CalcDeviation

void AvoidConP::CalcDeviation(){
	Constraint::CalcDeviation();
	// S‘©•Î·„‘Ì‚ÌˆÊ’u•Î·‚Ìƒmƒ‹ƒ€[ÅÚ‹ß‹——£
	y[0] = (key->obj1->pos_t->val - key->obj0->pos_t->val).norm() - ((AvoidTask*)key->node)->dmin;
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
	real_t perror = (key->obj1->pos_t->val - key->obj0->pos_t->val).norm() - ((AvoidTask*)key->node)->dmin;	
	if(perror < 0.0 && y[0] < 0.0)
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