#include <DiMP2/Graph.h>
#include <DiMP2/Range.h>

#include <boost/lexical_cast.hpp>
using namespace boost;

namespace DiMP2{;

//-------------------------------------------------------------------------------------------------
// MCObjectKey

void MCObjectKey::AddVar(Solver* solver){
	ObjectBaseKey::AddVar(solver);

	MCObject* obj = GetNode();

	uint n;
	
	n = obj->NumJoints();
	joint.resize(n);
	for(uint j = 0; j < n; j++){
		real_t sp = (obj->IsRotational(j) ? obj->graph->scale.pos_r : obj->graph->scale.pos_t);
		real_t sv = (obj->IsRotational(j) ? obj->graph->scale.vel_r : obj->graph->scale.vel_t);
		
		string idx = lexical_cast<string>(j);
		joint[j].pos = new SVar(solver, ID(VarTag::JointP, node, tick, name + "_jpos" + idx), sp);
		joint[j].vel = new SVar(solver, ID(VarTag::JointV, node, tick, name + "_jvel" + idx), sv);	
	}

	n = obj->NumEndpoints();
	end.resize(n);
	for(uint i = 0; i < n; i++){
		string idx = lexical_cast<string>(i);
		end[i].pos = new V3Var(solver, ID(VarTag::JointP, node, tick, name + "_cpos" + idx), obj->graph->scale.pos_t  );
		end[i].vel = new V3Var(solver, ID(VarTag::JointV, node, tick, name + "_cvel" + idx), obj->graph->scale.vel_t  );
	}

	n = obj->NumContacts();
	for(uint i = 0; i < n; i++)
		contact.push_back(obj->contact[i]->GetKeypoint(tick));
}

void MCObjectKey::AddCon(Solver* solver){
	ObjectBaseKey::AddCon(solver);

	MCObjectKey* nextObj = (MCObjectKey*)next;
	
	uint n = GetNode()->NumEndpoints();
	for(uint i = 0; i < n; i++){
		string idx = lexical_cast<string>(i);
		end[i].con_tp = new MCConTP(solver, name + "_tp" + idx, this, i, node->graph->scale.pos_t);
		end[i].con_tv = new MCConTV(solver, name + "_tv" + idx, this, i, node->graph->scale.vel_t);
		if(next){
			end[i].con_c1 = new C1ConV3(solver, ID(ConTag::ObjectC1T, node, tick, name + "_c1"),
				end[i].pos, end[i].vel, nextObj->end[i].pos, nextObj->end[i].vel, node->graph->scale.pos_t);
			end[i].con_c1->h = hnext;
		}
	}

	if(next){
		con_force_t = new MCForceConT(solver, name + "_force_t", this, node->graph->scale.force_t);
		con_force_r = new MCForceConR(solver, name + "_force_r", this, node->graph->scale.force_r);
	}
}

void MCObjectKey::Prepare(){
	ObjectBaseKey::Prepare();

	// ì∑ëÃÇ…ëŒÇ∑ÇÈê⁄êGì_ÇÃà íuÇ∆ÉÑÉRÉrÉAÉìÇåvéZ
	uint nj = GetNode()->NumJoints();
	vector<real_t> p(nj);
	for(uint i = 0; i < nj; i++)
		p[i] = joint[i].pos->val;
	GetNode()->callback->Calc(&p[0]);
	
	uint ne = GetNode()->NumEndpoints();
	for(uint i = 0; i < ne; i++){
		// èâä˙éûçèÇÃí[ì_ÇÕå≈íË
		if(!prev)
			end[i].pos->Lock();

		// ê⁄êGì_ÇÃà íu
		GetNode()->callback->GetEndpointPos(end[i].prel, i);
		end[i].prel = pos_r->val * end[i].prel;
			
		// ê⁄êGì_ÇÃÉÑÉRÉrÉAÉì
		end[i].J.resize(nj);
		for(uint j = 0; j < nj; j++){
			GetNode()->callback->GetJacobian(end[i].J[j], i, j);
			end[i].J[j] = pos_r->val * end[i].J[j];
		}
	}
}

void MCObjectKey::Draw(GRRenderIf* render, DrawConfig* conf){
	ObjectBaseKey::Draw(render, conf);

	// ï`âÊÇÃÇΩÇﬂÇæÇØÇ…CalcÇåƒÇ‘ÇÃÇ‡ñ≥ë ÇæÇ™ÅEÅE
	uint nj = GetNode()->NumJoints();
	vector<real_t> p(nj);
	for(uint j = 0; j < nj; j++)
		p[j] = joint[j].pos->val;
	GetNode()->callback->Calc(&p[0]);

	// ÉRÅ[ÉãÉoÉbÉNÇ…ÇÊÇÈïtâ¡èÓïÒÅiê⁄êGì_Ç÷ÇÃÉäÉìÉNÇ»Ç«ÅjÇÃï`âÊ
	pose_t pose(pos_t->val, pos_r->val);
	Affinef aff;
	pose.ToAffine(aff);
	render->PushModelMatrix();
	render->MultModelMatrix(aff);
	
	GetNode()->callback->Draw(render, conf);

	render->PopModelMatrix();
}

//-------------------------------------------------------------------------------------------------
// MCObject

MCObject::MCObject(Graph* g, string n):ObjectBase(g, n){
}
	
void MCObject::SetCallback(MCCallback* cb){
	callback = cb;
	callback->node = this;
	jparam.SetDof(NumJoints());
}

uint MCObject::NumJoints(){
	return callback->NumJoints();
}

uint MCObject::NumEndpoints(){
	return callback->NumEndpoints();
}

uint MCObject::NumContacts(){
	return contact.size();
}

bool MCObject::IsRotational(uint i){
	return callback->IsRotational(i);
}

void MCObject::SetInitialJointPos(uint j, real_t pos){
	jparam.ini_p[j] = pos;
}
	
void MCObject::Init(){
	ObjectBase::Init();

	for(uint k = 0; k < graph->ticks.size(); k++){
		MCObjectKey* key = GetKeypoint(graph->ticks[k]);
		
		// for non-dynamical objects, sum-of-force constraint is disabled
		if(!param.dynamical && key->next){
			key->con_force_t->enabled = false;
		}

		// joint settings
		for(uint j = 0; j < NumJoints(); j++){
			// set initial values to positions and velocities
			key->joint[j].pos->val = jparam.ini_p[j];
			key->joint[j].vel->val = jparam.ini_v[j];
		}
	}
}

void MCObject::Prepare(){
	ObjectBase::Prepare();


}

void MCObject::Draw(GRRenderIf* render, DrawConfig* conf){
	ObjectBase::Draw(render, conf);
	
	// ê⁄êGì_ÇÃãOìπ
	if(conf->Set(render, DrawItem::ObjectTrajectory, this)){
		Vec3f p0, p1;
		for(uint i = 0; i < NumEndpoints(); i++){
			for(uint k = 0; k < graph->ticks.size()-1; k++){
				MCObjectKey* obj0 = GetKeypoint(graph->ticks[k]);
				MCObjectKey* obj1 = GetKeypoint(graph->ticks[k+1]);
				p0 = obj0->end[i].pos->val;
				p1 = obj1->end[i].pos->val;
				render->DrawLine(p0, p1);
			}
		}
	}
}

void MCObject::DrawSnapshot(real_t time, GRRenderIf* render, DrawConfig* conf){
}

void MCObject::DrawSnapshot(const pose_t& pose, GRRenderIf* render, DrawConfig* conf){
}

//-------------------------------------------------------------------------------------------------
// ContactTaskKey

ContactTaskKey::ContactTaskKey(){

}

void ContactTaskKey::AddVar(Solver* solver){
	obj = GetNode()->obj->GetKeypoint(tick);

	force_n    = new  SVar(solver, ID(VarTag::JointF, node, tick, name + "_cforce_n" ), node->graph->scale.force_t);
	force_t[0] = new  SVar(solver, ID(VarTag::JointF, node, tick, name + "_cforce_t0"), node->graph->scale.force_t);
	force_t[1] = new  SVar(solver, ID(VarTag::JointF, node, tick, name + "_cforce_t1"), node->graph->scale.force_t);
}

void ContactTaskKey::AddCon(Solver* solver){
	/*con_pos_dist = new DistanceConV3(solver, ID(ConTag::ContactP, node, tick, name + "_pos"),
	GetNode()->pos, obj->pos_t, node->graph->scale.pos_t);
	con_pos_dist->_min = GetNode()->param.dmin;
	con_pos_dist->_max = GetNode()->param.dmax;*/
	
	con_pos = new MatchConV3(solver, ID(ConTag::ContactP, node, tick, name + "_pos"),
		GetNode()->pos, obj->end[GetNode()->idx].pos, node->graph->scale.pos_t);
	con_vel = new FixConV3(solver, ID(ConTag::ContactV, node, tick, name + "_vel"),
		obj->end[GetNode()->idx].vel, node->graph->scale.vel_t);

	if(next){
		// ê⁄êGóÕÇÃñ@ê¸ê¨ï™ÇÕê≥
		con_force_n = new RangeConS(solver, ID(ConTag::ContactFN, node, tick, name + "_force_n"),
			force_n, node->graph->scale.force_t);
		con_force_n->_min = 0.0;
		con_force_n->_max = 1.0;
		// ñÄéCóÕÇ™ñÄéCâ~êçÇ…ä‹Ç‹ÇÍÇÈçSë©
		con_force_t = new FrictionCon(solver, ID(ConTag::ContactFT, node, tick, name + "_force_t"),
			force_n, force_t[0], force_t[1], GetNode()->param.mu, node->graph->scale.force_t);
	}
}

void ContactTaskKey::Prepare(){
	TaskBaseKey::Prepare();

	bool active = (relation != Outside);
	con_pos->active = active;
	con_vel->active = active;
	if(next){
		if(active){
			force_n   ->Lock(false);
			force_t[0]->Lock(false);
			force_t[1]->Lock(false);
		}
		else{
			force_n   ->val = 0.0;
			force_t[0]->val = 0.0;
			force_t[1]->val = 0.0;
			force_n   ->Lock(true);
			force_t[0]->Lock(true);
			force_t[1]->Lock(true);
		}
	//	con_pos_dist->active = active;
		con_force_n ->active = active;
		con_force_t ->active = active;
	}
}

void ContactTaskKey::Draw(GRRenderIf* render, DrawConfig* conf){
	ContactTask* con = GetNode();

	// ê⁄êGóÕÇÃï`âÊ
	if(conf->Set(render, DrawItem::JointForce, node)){
		real_t s = conf->Scale(DrawItem::JointForce, node);
		
		Vec3f p0, p1;
		p0 = con->pos->val;
		p1 = p0 + (s * force_n->val) * con->param.normal;
		render->DrawLine(p0, p1);
		p1 = p0 + (s * force_t[0]->val) * con->param.tangent[0];
		render->DrawLine(p0, p1);
		p1 = p0 + (s * force_t[1]->val) * con->param.tangent[1];
		render->DrawLine(p0, p1);
	}
}

//-------------------------------------------------------------------------------------------------
// ContactTask

ContactTask::Param::Param(){
	mu         = 0.3;
	normal     = vec3_t(0.0, 1.0, 0.0);
	tangent[0] = vec3_t(1.0, 0.0, 0.0);
	tangent[1] = vec3_t(0.0, 0.0, 1.0);
	dmin       = 0.5;
	dmax       = 0.8;
}

ContactTask::ContactTask(Graph* g, string name, MCObject* _obj, uint _idx, TimeSlot* _time):TaskBase(g, name, _time){
	obj = _obj;
	idx = _idx;
	obj->contact.push_back(this);
}

void ContactTask::AddVar(){
	TaskBase::AddVar();

	pos = new V3Var(&graph->solver, ID(VarTag::JointP, this, 0, name + "_cpos"), graph->scale.pos_t);	
}

void ContactTask::AddCon(){
	TaskBase::AddCon();

	con_pos_plane = new FixConPlane(&graph->solver, ID(ConTag::ContactP, this, 0, name + "_cpos"), pos, graph->scale.pos_t);
	con_pos_plane->normal = param.normal;
	con_pos_plane->origin = vec3_t();
}

void ContactTask::Prepare(){
	TaskBase::Prepare();

	// èâä˙éûçèÇ©ÇÁóLå¯Ç»èÍçáÅCê⁄êGì_ÇÃà íuÇå≈íË
	if(time->time_s->val == 0.0)
		pos->Lock();
}

void ContactTask::Draw(GRRenderIf* render, DrawConfig* conf){
	TaskBase::Draw(render, conf);

	// ê⁄êGì_ÇÃãOìπ
	if(conf->Set(render, DrawItem::ObjectPos, this))
		render->DrawPoint(pos->val);
}

//-------------------------------------------------------------------------------------------------
// BalanceTaskKey

BalanceTaskKey::BalanceTaskKey(){

}

void BalanceTaskKey::AddVar(Solver* solver){
	obj = GetNode()->obj->GetKeypoint(tick);
}

void BalanceTaskKey::AddCon(Solver* solver){
	con_fix_vel_t = new FixConV3(solver, ID(ConTag::ContactP, node, tick, name + "_vel_t"), obj->vel_t, node->graph->scale.vel_t);
}

void BalanceTaskKey::Prepare(){
	TaskBaseKey::Prepare();

	bool active = (relation != Outside);
	con_fix_vel_t->active = active;
}

//-------------------------------------------------------------------------------------------------
// BalanceTask

BalanceTask::BalanceTask(Graph* g, string name, MCObject* _obj, TimeSlot* _time):TaskBase(g, name, _time){
	obj = _obj;
}

//-------------------------------------------------------------------------------------------------
// Constructors

MCConTP::MCConTP(Solver* solver, string _name, MCObjectKey* _obj, uint i, real_t _scale):
	Constraint(solver, 3, ID(ConTag::JointTP, _obj->node, _obj->tick, _name), _scale){
	obj = _obj;
	idx = i;
	AddSLink(obj->pos_t, -1.0);
	AddXLink(obj->pos_r);
	
	uint nj = obj->GetNode()->NumJoints();
	for(uint j = 0; j < nj; j++)
		AddCLink(obj->joint[j].pos);

	AddSLink(obj->end[i].pos, 1.0);
}
MCConTV::MCConTV(Solver* solver, string _name, MCObjectKey* _obj, uint i, real_t _scale):
	Constraint(solver, 3, ID(ConTag::JointTV, _obj->node, _obj->tick, _name), _scale){
	obj = _obj;
	idx = i;
	AddSLink(obj->vel_t, -1.0);
	AddXLink(obj->vel_r);

	uint nj = obj->GetNode()->NumJoints();
	for(uint j = 0; j < nj; j++)
		AddCLink(obj->joint[j].vel);

	AddSLink(obj->end[i].vel, 1.0);
}
MCForceConT::MCForceConT(Solver* solver, string _name, MCObjectKey* _obj, real_t _scale):
	Constraint(solver, 3, ID(ConTag::ForceT, _obj->node, _obj->tick, _name), _scale){
	obj[0] = _obj;
	obj[1] = (MCObjectKey*)_obj->next;

	AddSLink(obj[0]->vel_t);
	AddSLink(obj[1]->vel_t);

	uint nc = obj[0]->GetNode()->contact.size();
	for(uint i = 0; i < nc; i++){
		AddCLink(obj[0]->contact[i]->force_n);
		AddCLink(obj[0]->contact[i]->force_t[0]);
		AddCLink(obj[0]->contact[i]->force_t[1]);
	}
}
MCForceConR::MCForceConR(Solver* solver, string _name, MCObjectKey* _obj, real_t _scale):
	Constraint(solver, 3, ID(ConTag::ForceR, _obj->node, _obj->tick, _name), _scale){
	obj = _obj;
	
	AddXLink(obj->pos_t);

	uint nc = obj->contact.size();
	for(uint i = 0; i < nc; i++){
		AddXLink(obj->contact[i]->GetNode()->pos);
		AddCLink(obj->contact[i]->force_n);
		AddCLink(obj->contact[i]->force_t[0]);
		AddCLink(obj->contact[i]->force_t[1]);
	}
}

//-------------------------------------------------------------------------------------------------
// CalcCoef

void MCConTP::CalcCoef(){
	((XLink*)links[1])->SetCoef(obj->end[idx].prel);
	
	uint nj = obj->GetNode()->NumJoints();
	for(uint j = 0; j < nj; j++)
		((CLink*)links[2+j])->SetCoef(-obj->end[idx].J[j]);
}
void MCConTV::CalcCoef(){
	((XLink*)links[1])->SetCoef(obj->end[idx].prel);
	
	uint nj = obj->GetNode()->NumJoints();
	for(uint j = 0; j < nj; j++)
		((CLink*)links[2+j])->SetCoef(-obj->end[idx].J[j]);
}
void MCForceConT::CalcCoef(){
	real_t h = obj[0]->hnext;
	real_t m = obj[0]->GetNode()->param.mass;

	((SLink*)links[0])->SetCoef(-m/h);
	((SLink*)links[1])->SetCoef( m/h);

	uint nc = obj[0]->GetNode()->contact.size();
	for(uint i = 0; i < nc; i++){
		ContactTask::Param& param = obj[0]->GetNode()->contact[i]->param;
		((CLink*)links[2+3*i+0])->SetCoef(-1.0 * param.normal);
		((CLink*)links[2+3*i+1])->SetCoef(-1.0 * param.tangent[0]);
		((CLink*)links[2+3*i+2])->SetCoef(-1.0 * param.tangent[1]);
	}
}
void MCForceConR::CalcCoef(){
	uint nc = obj->contact.size();

	// sum of all contact forces
	vector<vec3_t> f3(nc);
	vec3_t f3sum;
	for(uint i = 0; i < nc; i++){
		ContactTaskKey* con = obj->contact[i];
		ContactTask::Param& param = con->GetNode()->param;
		f3[i] = param.normal     * con->force_n   ->val
		      + param.tangent[0] * con->force_t[0]->val
			  + param.tangent[1] * con->force_t[1]->val;
		f3sum += f3[i];
	}

	((XLink*)links[0])->SetCoef(-f3sum);

	for(uint i = 0; i < nc; i++){
		ContactTaskKey* con = obj->contact[i];
		ContactTask::Param& param = con->GetNode()->param;
		((XLink*)links[1+4*i+0])->SetCoef(f3[i]);
		((CLink*)links[1+4*i+1])->SetCoef(param.normal     % (con->GetNode()->pos->val - obj->pos_t->val));
		((CLink*)links[1+4*i+2])->SetCoef(param.tangent[0] % (con->GetNode()->pos->val - obj->pos_t->val));
		((CLink*)links[1+4*i+3])->SetCoef(param.tangent[1] % (con->GetNode()->pos->val - obj->pos_t->val));
	}
}

//-------------------------------------------------------------------------------------------------
// CalcDeviation

void MCConTP::CalcDeviation(){
	y = obj->end[idx].pos->val - (obj->pos_t->val + obj->end[idx].prel);
}

void MCForceConT::CalcDeviation(){
	Constraint::CalcDeviation();
	y -= obj[0]->fext_t;	///< subtract external force
}

}
