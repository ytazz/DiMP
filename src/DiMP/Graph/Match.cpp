#include <DiMP/Graph/Match.h>
#include <DiMP/Graph/Timing.h>
#include <DiMP/Graph/Object.h>
#include <DiMP/Graph/Graph.h>
#include <DiMP/Render/Config.h>

namespace DiMP{;

//-------------------------------------------------------------------------------------------------
// MatchTaskKey

MatchTaskKey::MatchTaskKey(){
	for(uint i = 0; i < 3; i++){
		con_tp[i] = 0;
		con_tv[i] = 0;
		con_rp[i] = 0;
		con_rv[i] = 0;
	}
}

void MatchTaskKey::AddCon(Solver* solver){
	// create constraints
	for(int i = 0; i < 3; i++){
		if((i == Start && !next) || (i == End && !prev)){
			con_tp[i] = 0;
			con_tv[i] = 0;
			con_rp[i] = 0;
			con_rv[i] = 0;
		}
		else{
			stringstream ss;
			ss << i;
			con_tp[i] = new MatchConTP(solver, name + "_tp" + ss.str(), this, i, node->graph->scale.pos_t);
			con_tv[i] = new MatchConTV(solver, name + "_tv" + ss.str(), this, i, node->graph->scale.vel_t);
			con_rp[i] = new MatchConRP(solver, name + "_rp" + ss.str(), this, i, node->graph->scale.pos_r);
			con_rv[i] = new MatchConRV(solver, name + "_rv" + ss.str(), this, i, node->graph->scale.vel_r);
		}
	}
}

void MatchTaskKey::Prepare(){
	TaskKey::Prepare();

	MatchTask::Param& param = ((MatchTask*)node)->param;

	// タイミング区間との関係に応じて拘束を有効化/無効化する
	for(int i = 0; i < 3; i++){
		if(i == Start && !next || i == End && !prev)
			continue;
		
		con_tp[i]->enabled = param.match_tp;
		con_tv[i]->enabled = param.match_tv;
		con_rp[i]->enabled = param.match_rp;
		con_rv[i]->enabled = param.match_rv;

		// 暫定処置
		if(i == Start || i == End){
			con_tp[i]->active = false;
			con_tv[i]->active = false;
			con_rp[i]->active = false;
			con_rv[i]->active = false;
		}
		else{
			con_tp[i]->active = (relation == i);
			con_tv[i]->active = (relation == i);
			con_rp[i]->active = (relation == i);
			con_rv[i]->active = (relation == i);
		}
	}
}

//-------------------------------------------------------------------------------------------------
// MatchTask

MatchTask::Param::Param(){
	spline   = false;
	match_tp = true;
	match_tv = true;
	match_rp = false;
	match_rv = false;
}

MatchTask::MatchTask(Object* _obj0, Object* _obj1, TimeSlot* _time, const string& n)
	:Task(_obj0, _obj1, _time, n){
}

void MatchTask::Draw(Render::Canvas* canvas){

}

//-------------------------------------------------------------------------------------------------
// constructors

MatchCon::MatchCon(Solver* solver, ID id, MatchTaskKey* _key, int _mode, real_t _scale)
	:Constraint(solver, 3, id, _scale){
	key  = _key;
	mode = _mode;
}

MatchConT::MatchConT(Solver* solver, ID id, MatchTaskKey* _key, int _mode, real_t _scale)
	:MatchCon(solver, id, _key, _mode, _scale){
}

MatchConR::MatchConR(Solver* solver, ID id, MatchTaskKey* _key, int _mode, real_t _scale)
	:MatchCon(solver, id, _key, _mode, _scale){
}

MatchConTP::MatchConTP(Solver* solver, const string& _name, MatchTaskKey* _key, int _mode, real_t _scale):
	MatchConT(solver, ID(ConTag::MatchTP, _key->node, _key->tick, _name), _key, _mode, _scale){
	AddLinks(true);
}

MatchConTV::MatchConTV(Solver* solver, const string& _name, MatchTaskKey* _key, int _mode, real_t _scale):
	MatchConT(solver, ID(ConTag::MatchTV, _key->node, _key->tick, _name), _key, _mode, _scale){
	AddLinks(false);
}

MatchConRP::MatchConRP(Solver* solver, const string& _name, MatchTaskKey* _key, int _mode, real_t _scale):
	MatchConR(solver, ID(ConTag::MatchRP, _key->node, _key->tick, _name), _key, _mode, _scale){
	AddLinks(true);
}

MatchConRV::MatchConRV(Solver* solver, const string& _name, MatchTaskKey* _key, int _mode, real_t _scale):
	MatchConR(solver, ID(ConTag::MatchRV, _key->node, _key->tick, _name), _key, _mode, _scale){
	AddLinks(false);
}

//-------------------------------------------------------------------------------------------------
// AddLinks

void MatchConT::AddLinks(bool pos_or_vel){
	MatchTask* task = (MatchTask*)key->node;

	if(mode == MatchTaskKey::Start){
		MatchTaskKey* next = (MatchTaskKey*)key->next;
		key ->obj0->AddLinks(this, true, true , true);
		key ->obj0->AddLinks(this, true, false, true);
		next->obj0->AddLinks(this, true, true , true);
		next->obj0->AddLinks(this, true, false, true);
		key ->obj1->AddLinks(this, true, true , true);
		key ->obj1->AddLinks(this, true, false, true);
		next->obj1->AddLinks(this, true, true , true);
		next->obj1->AddLinks(this, true, false, true);
		
		if(task->time)
			AddCLink(task->time->time_s);
	}
	else if(mode == MatchTaskKey::End){
		MatchTaskKey* prev = (MatchTaskKey*)key->prev;
		prev->obj0->AddLinks(this, true, true , true);
		prev->obj0->AddLinks(this, true, false, true);
		key ->obj0->AddLinks(this, true, true , true);
		key ->obj0->AddLinks(this, true, false, true);
		prev->obj1->AddLinks(this, true, true , true);
		prev->obj1->AddLinks(this, true, false, true);
		key ->obj1->AddLinks(this, true, true , true);
		key ->obj1->AddLinks(this, true, false, true);

		if(task->time)
			AddCLink(task->time->time_e);
	}
	else{
		if(pos_or_vel){
			key->obj0->AddLinks(this, true, true, true);
			key->obj1->AddLinks(this, true, true, true);
		}
		else{
			key->obj0->AddLinks(this, true, false, true);
			key->obj1->AddLinks(this, true, false, true);
		}
	}
}

void MatchConR::AddLinks(bool pos_or_vel){
	if(mode == MatchTaskKey::Start){
	}
	else if(mode == MatchTaskKey::End){
	}
	else{
		if(pos_or_vel){
			key->obj0->AddLinks(this, false, true, true);
			key->obj1->AddLinks(this, false, true, true);
		}
		else{
			key->obj0->AddLinks(this, false, false, true);
			key->obj1->AddLinks(this, false, false, true);
		}
	}
}

//-------------------------------------------------------------------------------------------------
// CalcCoef

void MatchConTP::CalcCoef(){
	if(!active)
		return;

	MatchTask* task = (MatchTask*)key->node;
	if(mode == MatchTaskKey::Start || mode == MatchTaskKey::End){
		real_t t, dt, h, s, s2, s3;
		if(mode == MatchTaskKey::Start){
			if(task->time)
				 t = task->time->time_s->val;
			else t = task->key_s->tick->time;
			dt = t - key->tick->time;
			h  = key->next->tick->time - key->tick->time;
		}
		else{
			if(task->time)
				 t = task->time->time_e->val;
			else t = task->key_e->tick->time;
			dt = t - key->prev->tick->time;
			h  = key->tick->time - key->prev->tick->time;
		}
		
		s  = dt/h;
		s2 = s*s;
		s3 = s2*s;

		// 係数
		real_t k_p0, k_v0, k_p1, k_v1;

		if(task->param.spline){
			k_p0 = (1.0 - 3.0*s2 + 2.0*s3);
			k_v0 = (s - 2.0*s2 + s3)*h;
			k_p1 = (3.0*s2 - 2.0*s3);
			k_v1 = (-s2 + s3)*h;
		}
		else{
			k_p0 = 1 - s2;
			k_v0 = (s - s2)*h;
			k_p1 = s2;
			k_v1 = 0;
		}
		
		vec3_t v0 = task->obj0->Vel(t, Interpolate::Quadratic);
		vec3_t v1 = task->obj1->Vel(t, Interpolate::Quadratic);

		uint i = 0;
		if(mode == MatchTaskKey::Start){
			MatchTaskKey* next = (MatchTaskKey*)key->next;
			key ->obj0->CalcCoef(this, true,  k_p0, i);
			key ->obj0->CalcCoef(this, true,  k_v0, i);
			next->obj0->CalcCoef(this, true,  k_p1, i);
			next->obj0->CalcCoef(this, true,  k_v1, i);
			key ->obj1->CalcCoef(this, true, -k_p0, i);
			key ->obj1->CalcCoef(this, true, -k_v0, i);
			next->obj1->CalcCoef(this, true, -k_p1, i);
			next->obj1->CalcCoef(this, true, -k_v1, i);
		}
		else{
			MatchTaskKey* prev = (MatchTaskKey*)key->prev;
			prev->obj0->CalcCoef(this, true,  k_p0, i);
			prev->obj0->CalcCoef(this, true,  k_v0, i);
			key ->obj0->CalcCoef(this, true,  k_p1, i);
			key ->obj0->CalcCoef(this, true,  k_v1, i);
			prev->obj1->CalcCoef(this, true, -k_p0, i);
			prev->obj1->CalcCoef(this, true, -k_v0, i);
			key ->obj1->CalcCoef(this, true, -k_p1, i);
			key ->obj1->CalcCoef(this, true, -k_v1, i);
		}

		if(task->time)
			((CLink*)links[i++])->SetCoef(v0 - v1);
	}
	else{
		uint i = 0;
		key->obj0->CalcCoef(this, true,  1.0, i);
		key->obj1->CalcCoef(this, true, -1.0, i);
	}
}

void MatchConTV::CalcCoef(){
	if(!active)
		return;

	MatchTask* task = (MatchTask*)key->node;
	if(mode == MatchTaskKey::Start || mode == MatchTaskKey::End){
		real_t t, dt, h, s, s2;
		if(mode == MatchTaskKey::Start){
			if(task->time)
				 t = task->time->time_s->val;
			else t = task->key_s->tick->time;
			dt = t - key->tick->time;
			h  = key->next->tick->time - key->tick->time;
		}
		else{
			if(task->time)
				 t = task->time->time_e->val;
			else t = task->key_e->tick->time;
			dt = t - key->prev->tick->time;
			h  = key->tick->time - key->prev->tick->time;
		}
		
		s  = dt/h;
		s2 = s*s;

		// 係数
		real_t k_p0, k_v0, k_p1, k_v1;

		if(task->param.spline){
			k_p0 = 6.0*( -s + s2);
			k_v0 = 1.0 - 4.0*s +3*s2;
			k_p1 = -k_p0;
			k_v1 =-2.0*s + 3.0*s2 ;
		}
		else{
			k_p0 = -2 * s/h ;
			k_v0 = 1.0 - 2.0*s;
			k_p1 = -k_p0;
			k_v1 = 0;
		}
		
		vec3_t a0 = task->obj0->Acc(t, Interpolate::Quadratic);
		vec3_t a1 = task->obj1->Acc(t, Interpolate::Quadratic);

		uint i = 0;
		if(mode == MatchTaskKey::Start){
			MatchTaskKey* next = (MatchTaskKey*)key->next;
			key ->obj0->CalcCoef(this, true,  k_p0, i);
			key ->obj0->CalcCoef(this, true,  k_v0, i);
			next->obj0->CalcCoef(this, true,  k_p1, i);
			next->obj0->CalcCoef(this, true,  k_v1, i);
			key ->obj1->CalcCoef(this, true, -k_p0, i);
			key ->obj1->CalcCoef(this, true, -k_v0, i);
			next->obj1->CalcCoef(this, true, -k_p1, i);
			next->obj1->CalcCoef(this, true, -k_v1, i);
		}
		else{
			MatchTaskKey* prev = (MatchTaskKey*)key->prev;
			prev->obj0->CalcCoef(this, true,  k_p0, i);
			prev->obj0->CalcCoef(this, true,  k_v0, i);
			key ->obj0->CalcCoef(this, true,  k_p1, i);
			key ->obj0->CalcCoef(this, true,  k_v1, i);
			prev->obj1->CalcCoef(this, true, -k_p0, i);
			prev->obj1->CalcCoef(this, true, -k_v0, i);
			key ->obj1->CalcCoef(this, true, -k_p1, i);
			key ->obj1->CalcCoef(this, true, -k_v1, i);
		}
	
		if(task->time)
			((CLink*)links[i++])->SetCoef(a0 - a1);
	}
	else{
		uint i = 0;
		key->obj0->CalcCoef(this, true,  1.0, i);
		key->obj1->CalcCoef(this, true, -1.0, i);
	}
}

void MatchConRP::CalcCoef(){
	if(!active)
		return;
	
	if(mode == MatchTaskKey::Start){
	}
	else if(mode == MatchTaskKey::End){
	}
	else{
		uint i = 0;
		key->obj0->CalcCoef(this, false,  1.0, i);
		key->obj1->CalcCoef(this, false, -1.0, i);
	}
}

void MatchConRV::CalcCoef(){
	if(!active)
		return;
	
	if(mode == MatchTaskKey::Start){
	}
	else if(mode == MatchTaskKey::End){
	}
	else{
		uint i = 0;
		key->obj0->CalcCoef(this, false,  1.0, i);
		key->obj1->CalcCoef(this, false, -1.0, i);
	}
}

//-------------------------------------------------------------------------------------------------
// CalcDeviation

void MatchConTP::CalcDeviation(){
	if(!active)
		return;

	MatchTask* task = (MatchTask*)key->node;

	// モード毎に誤差を計算
	real_t t;
	int type = Interpolate::Quadratic;

	// キーポイント上の誤差
	if(mode == MatchTaskKey::Inside){
		y = key->obj0->pos_t->val - key->obj1->pos_t->val;
	}
	// 始点時刻上の誤差
	else if(mode == MatchTaskKey::Start){
		if(task->time)
			 t = task->time->time_s->val;
		else t = task->key_s->tick->time;
		y = task->obj0->Pos(t, type) - task->obj1->Pos(t, type);
	}
	// 終点時刻上の誤差
	else if(mode == MatchTaskKey::End){
		if(task->time)
			 t = task->time->time_e->val;
		else t = task->key_e->tick->time;
		y = task->obj0->Pos(t, type) - task->obj1->Pos(t, type);
	}
}

void MatchConTV::CalcDeviation(){
	if(!active)
		return;

	MatchTask* task = (MatchTask*)key->node;

	// モード毎に誤差を計算
	real_t t;
	int type = Interpolate::Quadratic;

	// キーポイント上の誤差
	if(mode == MatchTaskKey::Inside){
		y = key->obj0->vel_t->val - key->obj1->vel_t->val;
	}
	// 始点時刻上の誤差
	else if(mode == MatchTaskKey::Start){
		if(task->time)
			 t = task->time->time_s->val;
		else t = task->key_s->tick->time;
		y = task->obj0->Vel(t, type) - task->obj1->Vel(t, type);
	}
	// 終点時刻上の誤差
	else if(mode == MatchTaskKey::End){
		if(task->time)
			 t = task->time->time_e->val;
		else t = task->key_e->tick->time;
		y = task->obj0->Vel(t, type) - task->obj1->Vel(t, type);
	}
}

void MatchConRP::CalcDeviation(){
	if(!active)
		return;

	MatchTask* task = (MatchTask*)key->node;

	if(mode == MatchTaskKey::Inside){
		quat_t q0 = key->obj0->pos_r->val;
		quat_t q1 = key->obj1->pos_r->val;
		quat_t qerror = q1.Conjugated() * q0;
		y = q1 * (qerror.Theta() * qerror.Axis());
	}
	else if(mode == MatchTaskKey::Start){
		y.clear();
	}
	else if(mode == MatchTaskKey::End){
		y.clear();
	}
}

void MatchConRV::CalcDeviation(){
	if(!active)
		return;

	MatchTask* task = (MatchTask*)key->node;

	if(mode == MatchTaskKey::Inside){
		y = key->obj0->vel_r->val - key->obj1->vel_r->val;
	}
	else if(mode == MatchTaskKey::Start){
		y.clear();
	}
	else if(mode == MatchTaskKey::End){
		y.clear();
	}
}

}