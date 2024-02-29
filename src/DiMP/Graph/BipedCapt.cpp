#include <DiMP/Graph/BipedCapt.h>
#include <DiMP/Graph/Graph.h>
#include <DiMP/Render/Config.h>
#include <DiMP/Render/Canvas.h>

#include <sbrollpitchyaw.h>

namespace DiMP {;

const real_t  eps     = 1.0e-10;
const real_t   pi     = M_PI;
const real_t _2pi     = 2.0*pi;
const real_t  damping = 0.1;
const vec3_t one(1.0, 1.0, 1.0);

//-------------------------------------------------------------------------------------------------
// BipedCaptKey

BipedCaptKey::BipedCaptKey() {

}

// register variables for planning
void BipedCaptKey::AddVar(Solver* solver) {
	BipedCapt* obj = (BipedCapt*)node;

	// support foot indicator
	side = obj->param.steps[tick->idx].side;

	// support foot position
	var_sup_t = new V3Var(solver, ID(VarTag::BipedCaptSupT, node, tick, name + "_sup_t"), node->graph->scale.pos_t);
	var_sup_r = new SVar (solver, ID(VarTag::BipedCaptSupR, node, tick, name + "_sup_r"), node->graph->scale.pos_r);
	var_swg_t = new V3Var(solver, ID(VarTag::BipedCaptSwgT, node, tick, name + "_swg_t"), node->graph->scale.pos_t);
	var_swg_r = new SVar (solver, ID(VarTag::BipedCaptSwgR, node, tick, name + "_swg_r"), node->graph->scale.pos_r);
	var_icp   = new V3Var(solver, ID(VarTag::BipedCaptIcp , node, tick, name + "_icp"  ), node->graph->scale.pos_t);
	var_sup_t->weight    = damping*one;
	var_sup_r->weight[0] = damping;
	var_swg_t->weight    = damping*one;
	var_swg_r->weight[0] = damping;
	var_icp  ->weight    = damping*one;
	
	solver->AddStateVar(var_sup_t, tick->idx);
	solver->AddStateVar(var_sup_r, tick->idx);
	solver->AddStateVar(var_swg_t, tick->idx);
	solver->AddStateVar(var_swg_r, tick->idx);
	solver->AddStateVar(var_icp  , tick->idx);
	
	if(next){
		var_land_t   = new V3Var(solver, ID(VarTag::BipedCaptLandT   , node, tick, name + "_land_t"  ), node->graph->scale.pos_t);
		var_land_r   = new SVar (solver, ID(VarTag::BipedCaptLandR   , node, tick, name + "_land_r"  ), node->graph->scale.pos_r);
		var_cop      = new V3Var(solver, ID(VarTag::BipedCaptCop     , node, tick, name + "_cop"     ), node->graph->scale.pos_t);
		var_duration = new SVar (solver, ID(VarTag::BipedCaptDuration, node, tick, name + "_duration"), node->graph->scale.time);
		var_land_t  ->weight    = damping*one;
		var_land_r  ->weight[0] = damping;
		var_cop     ->weight    = damping*one;
		var_duration->weight[0] = damping;

		solver->AddInputVar(var_land_t  , tick->idx);
		solver->AddInputVar(var_land_r  , tick->idx);
		solver->AddInputVar(var_cop     , tick->idx);
		solver->AddInputVar(var_duration, tick->idx);
	}
}

// register constraints for planning
void BipedCaptKey::AddCon(Solver* solver) {
	BipedCapt* obj = (BipedCapt*)node;
	BipedCaptKey* nextObj = (BipedCaptKey*)next;

	if (next) {
		con_sup_t = new BipedCaptSupConT(solver, name + "_sup_t", this, node->graph->scale.pos_t);
		con_sup_r = new BipedCaptSupConR(solver, name + "_sup_r", this, node->graph->scale.pos_r);
		con_swg_t = new BipedCaptSwgConT(solver, name + "_swg_t", this, node->graph->scale.pos_t);
		con_swg_r = new BipedCaptSwgConR(solver, name + "_swg_r", this, node->graph->scale.pos_r);
		con_icp   = new BipedCaptIcpCon (solver, name + "_icp"  , this, node->graph->scale.pos_t);
		solver->AddTransitionCon(con_sup_t, tick->idx);
		solver->AddTransitionCon(con_sup_r, tick->idx);
		solver->AddTransitionCon(con_swg_t, tick->idx);
		solver->AddTransitionCon(con_swg_r, tick->idx);
		solver->AddTransitionCon(con_icp  , tick->idx);
		
		con_duration        = new BipedCaptDurationCon  (solver, name + "_duration"     , this, node->graph->scale.time);
		con_des_cop         = new BipedCaptDesCopCon    (solver, name + "_des_cop"      , this, node->graph->scale.pos_t);
		con_land_range_t[0] = new BipedCaptLandRangeConT(solver, name + "_land_range_t0", this, vec3_t( 1.0,  0.0, 0.0), node->graph->scale.pos_t);
		con_land_range_t[1] = new BipedCaptLandRangeConT(solver, name + "_land_range_t1", this, vec3_t(-1.0,  0.0, 0.0), node->graph->scale.pos_t);
		con_land_range_t[2] = new BipedCaptLandRangeConT(solver, name + "_land_range_t2", this, vec3_t( 0.0,  1.0, 0.0), node->graph->scale.pos_t);
		con_land_range_t[3] = new BipedCaptLandRangeConT(solver, name + "_land_range_t3", this, vec3_t( 0.0, -1.0, 0.0), node->graph->scale.pos_t);
		con_land_range_r[0] = new BipedCaptLandRangeConR(solver, name + "_land_range_r0", this,  1.0, node->graph->scale.pos_r);
		con_land_range_r[1] = new BipedCaptLandRangeConR(solver, name + "_land_range_r1", this, -1.0, node->graph->scale.pos_r);
		con_cop_range   [0] = new BipedCaptCopRangeCon  (solver, name + "_cop_range0", this, vec3_t( 1.0,  0.0, 0.0), node->graph->scale.pos_t);
		con_cop_range   [1] = new BipedCaptCopRangeCon  (solver, name + "_cop_range1", this, vec3_t(-1.0,  0.0, 0.0), node->graph->scale.pos_t);
		con_cop_range   [2] = new BipedCaptCopRangeCon  (solver, name + "_cop_range2", this, vec3_t( 0.0,  1.0, 0.0), node->graph->scale.pos_t);
		con_cop_range   [3] = new BipedCaptCopRangeCon  (solver, name + "_cop_range3", this, vec3_t( 0.0, -1.0, 0.0), node->graph->scale.pos_t);

		solver->AddCostCon(con_duration, tick->idx);
		solver->AddCostCon(con_des_cop , tick->idx);
		for(int i = 0; i < 4; i++)
			solver->AddCostCon(con_land_range_t[i], tick->idx);
		for(int i = 0; i < 2; i++)
			solver->AddCostCon(con_land_range_r[i], tick->idx);
		for(int i = 0; i < 4; i++)
			solver->AddCostCon(con_cop_range[i], tick->idx);
	}
	else{
		con_icp_range[0] = new BipedCaptIcpRangeCon(solver, name + "_icp_range0", this, vec3_t( 1.0,  0.0, 0.0), node->graph->scale.pos_t);
		con_icp_range[1] = new BipedCaptIcpRangeCon(solver, name + "_icp_range1", this, vec3_t(-1.0,  0.0, 0.0), node->graph->scale.pos_t);
		con_icp_range[2] = new BipedCaptIcpRangeCon(solver, name + "_icp_range2", this, vec3_t( 0.0,  1.0, 0.0), node->graph->scale.pos_t);
		con_icp_range[3] = new BipedCaptIcpRangeCon(solver, name + "_icp_range3", this, vec3_t( 0.0, -1.0, 0.0), node->graph->scale.pos_t);
		for(int i = 0; i < 4; i++)
			solver->AddCostCon(con_icp_range[i], tick->idx);
	}

	con_des_icp   = new FixConV3(solver, ID(ConTag::BipedCaptIcp , node, tick, name + "_des_icp"  ), var_icp  , node->graph->scale.pos_t);
	con_des_sup_t = new FixConV3(solver, ID(ConTag::BipedCaptSupT, node, tick, name + "_des_sup_t"), var_sup_t, node->graph->scale.pos_r);
	con_des_sup_r = new FixConS (solver, ID(ConTag::BipedCaptSupR, node, tick, name + "_des_sup_r"), var_sup_r, node->graph->scale.pos_t);
	con_des_swg_t = new FixConV3(solver, ID(ConTag::BipedCaptSwgT, node, tick, name + "_des_swg_t"), var_swg_t, node->graph->scale.pos_r);
	con_des_swg_r = new FixConS (solver, ID(ConTag::BipedCaptSwgR, node, tick, name + "_des_swg_r"), var_swg_r, node->graph->scale.pos_t);
	solver->AddCostCon(con_des_icp  , tick->idx);
	solver->AddCostCon(con_des_sup_t, tick->idx);
	solver->AddCostCon(con_des_sup_r, tick->idx);
	solver->AddCostCon(con_des_swg_t, tick->idx);
	solver->AddCostCon(con_des_swg_r, tick->idx);
}

void BipedCaptKey::Prepare() {

}

void BipedCaptKey::Finish(){

}

void BipedCaptKey::Draw(Render::Canvas* canvas, Render::Config* conf) {
	BipedCapt* obj = (BipedCapt*)node;

	Vec3f icp, cop;
	
	if (conf->Set(canvas, Render::Item::BipedCaptIcp, node)) {	
		icp.x = (float)var_icp->val.x;
		icp.y = (float)var_icp->val.y;
		icp.z = 0.0f;
		canvas->Point(icp);
	}

	if (conf->Set(canvas, Render::Item::BipedCaptCop, node)) {	
		if(next){
			cop.x = (float)var_cop->val.x;
			cop.y = (float)var_cop->val.y;
			cop.z = 0.0f;
			canvas->Point(cop);
		}
	}

	if (conf->Set(canvas, Render::Item::BipedCaptSup, node)) {		
		// foot print
		Vec3f cmin = obj->param.copMin[side];
		Vec3f cmax = obj->param.copMax[side];
		canvas->Push();
		canvas->Translate((float)var_sup_t->val.x, (float)var_sup_t->val.y, 0.0f);
		canvas->Rotate((float)var_sup_r->val, Vec3f(0.0f, 0.0f, 1.0f));
		canvas->Line(Vec3f(cmax.x, cmax.y, 0.0f), Vec3f(cmin.x, cmax.y, 0.0f));
		canvas->Line(Vec3f(cmin.x, cmax.y, 0.0f), Vec3f(cmin.x, cmin.y, 0.0f));
		canvas->Line(Vec3f(cmin.x, cmin.y, 0.0f), Vec3f(cmax.x, cmin.y, 0.0f));
		canvas->Line(Vec3f(cmax.x, cmin.y, 0.0f), Vec3f(cmax.x, cmax.y, 0.0f));
		canvas->Pop();
	}
	if (conf->Set(canvas, Render::Item::BipedCaptSwg, node)) {		
		// foot print
		Vec3f cmin = obj->param.copMin[!side];
		Vec3f cmax = obj->param.copMax[!side];
		canvas->Push();
		canvas->Translate((float)var_swg_t->val.x, (float)var_swg_t->val.y, 0.0f);
		canvas->Rotate((float)var_swg_r->val, Vec3f(0.0f, 0.0f, 1.0f));
		canvas->Line(Vec3f(cmax.x, cmax.y, 0.0f), Vec3f(cmin.x, cmax.y, 0.0f));
		canvas->Line(Vec3f(cmin.x, cmax.y, 0.0f), Vec3f(cmin.x, cmin.y, 0.0f));
		canvas->Line(Vec3f(cmin.x, cmin.y, 0.0f), Vec3f(cmax.x, cmin.y, 0.0f));
		canvas->Line(Vec3f(cmax.x, cmin.y, 0.0f), Vec3f(cmax.x, cmax.y, 0.0f));
		canvas->Pop();
	}

}

//-------------------------------------------------------------------------------------------------
// BipedCapt

BipedCapt::Param::Param() {
	gravity            = 9.8;
	comHeight          = 1.0;
	
	// range of foot position relative to com
	landPosMin[0] = vec3_t(-0.5, -0.2, 0.0);
	landPosMax[0] = vec3_t( 0.5, -0.0, 0.0);
	landPosMin[1] = vec3_t(-0.5,  0.0, 0.0);
	landPosMax[1] = vec3_t( 0.5,  0.2, 0.0);
	landOriMin[0] = Rad(-15.0);
	landOriMax[0] = Rad( 15.0);
	landOriMin[1] = Rad(-15.0);
	landOriMax[1] = Rad( 15.0);
	
	// range of cop relative to foot
	copMin[0] = vec3_t(-0.1, -0.05, 0.0);
	copMax[0] = vec3_t( 0.1,  0.05, 0.0);
	copMin[1] = vec3_t(-0.1, -0.05, 0.0);
	copMax[1] = vec3_t( 0.1,  0.05, 0.0);

	copMinDsp[0] = vec3_t(-0.1, -0.05, 0.0);
	copMaxDsp[0] = vec3_t( 0.1,  0.25, 0.0);
	copMinDsp[1] = vec3_t(-0.1, -0.25, 0.0);
	copMaxDsp[1] = vec3_t( 0.1,  0.05, 0.0);
}

BipedCapt::Step::Step(vec3_t _sup_t, real_t _sup_r, vec3_t _swg_t, real_t _swg_r, vec3_t _icp, vec3_t _cop, real_t _duration, int _side, bool _stepping){
	sup_t    = _sup_t;
	sup_r    = _sup_r;
	swg_t    = _swg_t;
	swg_r    = _swg_r;
	icp      = _icp;
	cop      = _cop;
	duration = _duration;
	side     = _side;
	stepping = _stepping;
}

//-------------------------------------------------------------------------------------------------
BipedCapt::Snapshot::Snapshot() {
	t = 0.0;
}

//-------------------------------------------------------------------------------------------------

BipedCapt::BipedCapt(Graph* g, string n) :TrajectoryNode(g, n) {
	type = Type::Object;
	graph->bipedCapts.Add(this);
}

BipedCapt::~BipedCapt() {
	graph->bipedCapts.Remove(this);
}

void BipedCapt::Init() {
	TrajectoryNode::Init();

	// time constant of inverted pendulum
	param.T = sqrt(param.comHeight / param.gravity);

	Reset();

	Prepare();
}

void BipedCapt::Reset(){
	int N = (int)graph->ticks.size()-1;
	for (int k = 0; k <= N; k++) {
		BipedCaptKey* key = (BipedCaptKey*)traj.GetKeypoint(graph->ticks[k]);

		key->side = param.steps[k].side;

		key->var_sup_t->val = key->con_des_sup_t->desired = param.steps[k].sup_t;
		key->var_sup_r->val = key->con_des_sup_r->desired = param.steps[k].sup_r;
		key->var_swg_t->val = key->con_des_swg_t->desired = param.steps[k].swg_t;
		key->var_swg_r->val = key->con_des_swg_r->desired = param.steps[k].swg_r;
		key->var_icp  ->val = key->con_des_icp  ->desired = param.steps[k].icp;

		key->con_des_sup_t->weight    = 0.01*one;
		key->con_des_sup_r->weight[0] = 0.01;
		key->con_des_swg_t->weight    = 0.01*one;
		key->con_des_swg_r->weight[0] = 0.01;
		key->con_des_icp  ->weight    = 0.01*one;

		if(key->next){
			key->var_land_t  ->val = param.steps[k+1].sup_t;
			key->var_land_r  ->val = param.steps[k+1].sup_r;
			key->var_cop     ->val = param.steps[k].cop;
			key->var_duration->val = param.steps[k].duration;
		}

		// desired cop weight
		if(key->next){
			key->con_des_cop->weight[0] = 0.01;
		}

		// set cop range
		if(key->next){
			if(param.steps[k].stepping){
				key->con_cop_range[0]->lim =  param.copMin[key->side].x;
				key->con_cop_range[1]->lim = -param.copMax[key->side].x;
				key->con_cop_range[2]->lim =  param.copMin[key->side].y;
				key->con_cop_range[3]->lim = -param.copMax[key->side].y;
			}
			else{
				key->con_cop_range[0]->lim =  param.copMinDsp[key->side].x;
				key->con_cop_range[1]->lim = -param.copMaxDsp[key->side].x;
				key->con_cop_range[2]->lim =  param.copMinDsp[key->side].y;
				key->con_cop_range[3]->lim = -param.copMaxDsp[key->side].y;
			}
		}

		// set landing range
		if(key->next){
			key->con_land_range_t[0]->lim =  param.landPosMin[!key->side].x;
			key->con_land_range_t[1]->lim = -param.landPosMax[!key->side].x;
			key->con_land_range_t[2]->lim =  param.landPosMin[!key->side].y;
			key->con_land_range_t[3]->lim = -param.landPosMax[!key->side].y;
			key->con_land_range_r[0]->lim =  param.landOriMin[!key->side];
			key->con_land_range_r[1]->lim = -param.landOriMax[!key->side];
		}

		// icp terminal constraint
		if(!key->next){
			key->con_icp_range[0]->lim =  param.copMin[key->side].x;
			key->con_icp_range[1]->lim = -param.copMax[key->side].x;
			key->con_icp_range[2]->lim =  param.copMin[key->side].y;
			key->con_icp_range[3]->lim = -param.copMax[key->side].y;
		}

		// fix initial state
		if(!key->prev){
			key->var_sup_t->locked = true;
			key->var_sup_r->locked = true;
			key->var_swg_t->locked = true;
			key->var_swg_r->locked = true;
			key->var_icp  ->locked = true;
		}
	}

	// initial guess of icp
	for(int k = N-1; k >= 0; k--){
		BipedCaptKey* key0 = (BipedCaptKey*)traj.GetKeypoint(graph->ticks[k+0]);
		BipedCaptKey* key1 = (BipedCaptKey*)traj.GetKeypoint(graph->ticks[k+1]);

		vec3_t cop0 = key0->var_cop->val;
		vec3_t icp0 = key0->var_icp->val;
		vec3_t icp1 = key1->var_icp->val;
		real_t ainv = exp(-key0->var_duration->val/param.T);
			
		if(k == 0){
			key0->var_cop->val = (icp0 - ainv*icp1)/(1.0-ainv);
		}
		else{
			key0->var_icp->val = key0->con_des_icp->desired = ainv*(icp1 - cop0) + cop0;
		}
	}

}

void BipedCapt::Prepare() {
	TrajectoryNode::Prepare();
	trajReady = false;
}

void BipedCapt::Finish() {
	TrajectoryNode::Finish();
	
	// update time
	real_t t = 0.0;
	for (int k = 0; k < graph->ticks.size(); k++) {
		BipedCaptKey* key = (BipedCaptKey*)traj.GetKeypoint(graph->ticks[k]);

		graph->ticks[k]->time = t;
		
		if(key->next)
			t += key->var_duration->val;
	}
}

//------------------------------------------------------------------------------------------------

void BipedCapt::Draw(Render::Canvas* canvas, Render::Config* conf) {
	TrajectoryNode::Draw(canvas, conf);
	
	if (!trajReady)
		CalcTrajectory();
	
	if (trajectory.empty())
		return;

	// icp
	if (conf->Set(canvas, Render::Item::BipedCaptIcp, this)) {
		canvas->BeginLayer("biped_icp", true);
		canvas->BeginPath();
		canvas->MoveTo(trajectory[0].icp);
		for (int i = 1; i < trajectory.size(); i++) {
			canvas->LineTo(trajectory[i].icp);
		}
		canvas->EndPath();
		canvas->EndLayer();
	}
}

void BipedCapt::CreateSnapshot(real_t t, BipedCapt::Snapshot& s){
	BipedCaptKey* key0 = (BipedCaptKey*)traj.GetSegment(t).first;
	BipedCaptKey* key1 = (BipedCaptKey*)traj.GetSegment(t).second;

	real_t dt = t - key0->tick->time;

	s.sup_t = key0->var_sup_t->val;
	s.sup_r = key0->var_sup_r->val;

	if(key1 == key0->next){
		vec3_t icp0 = key0->var_icp->val;
		vec3_t cop0 = key0->var_cop->val;
		
		real_t alpha = exp(dt/param.T);
		s.cop = cop0;
		s.icp = alpha*(icp0 - cop0) + cop0;
	}
	else{
		s.icp = key0->var_icp->val;
	}
}

void BipedCapt::CreateSnapshot(real_t t){
	CreateSnapshot(t, snapshot);
}

void BipedCapt::CalcTrajectory() {
	real_t tf = traj.back()->tick->time;
	real_t dt = 0.01;

	trajectory.clear();
	for (real_t t = 0.0; t <= tf; t += dt) {
		Snapshot s;
		CreateSnapshot(t, s);
		trajectory.push_back(s);
	}

	trajReady = true;
}

void BipedCapt::DrawSnapshot(Render::Canvas* canvas, Render::Config* conf) {

}

//-------------------------------------------------------------------------------------------------

// Constructors
BipedCaptIcpCon::BipedCaptIcpCon(Solver* solver, string _name, BipedCaptKey* _obj, real_t _scale) :
	Constraint(solver, 3, ID(ConTag::BipedCaptIcp, _obj->node, _obj->tick, _name), Constraint::Type::Equality, _scale) {
	obj[0] = _obj;
	obj[1] = (_obj->next ? (BipedCaptKey*)_obj->next : 0);

	AddSLink (obj[1]->var_icp);
	AddSLink (obj[0]->var_icp);
	AddSLink (obj[0]->var_cop);
	AddC3Link(obj[0]->var_duration);
}

BipedCaptSupConT::BipedCaptSupConT(Solver* solver, string _name, BipedCaptKey* _obj, real_t _scale) :
	Constraint(solver, 3, ID(ConTag::BipedCaptSupT, _obj->node, _obj->tick, _name), Constraint::Type::Equality, _scale) {
	obj[0] = _obj;
	obj[1] = (_obj->next ? (BipedCaptKey*)_obj->next : 0);

	AddSLink(obj[1]->var_sup_t );
	AddSLink(obj[0]->var_swg_t);
	AddSLink(obj[0]->var_land_t);
}

BipedCaptSupConR::BipedCaptSupConR(Solver* solver, string _name, BipedCaptKey* _obj, real_t _scale) :
	Constraint(solver, 1, ID(ConTag::BipedCaptSupR, _obj->node, _obj->tick, _name), Constraint::Type::Equality, _scale) {
	obj[0] = _obj;
	obj[1] = (_obj->next ? (BipedCaptKey*)_obj->next : 0);

	AddSLink(obj[1]->var_sup_r );
	AddSLink(obj[0]->var_swg_r);
	AddSLink(obj[0]->var_land_r);
}

BipedCaptSwgConT::BipedCaptSwgConT(Solver* solver, string _name, BipedCaptKey* _obj, real_t _scale) :
	Constraint(solver, 3, ID(ConTag::BipedCaptSwgT, _obj->node, _obj->tick, _name), Constraint::Type::Equality, _scale) {
	obj[0] = _obj;
	obj[1] = (_obj->next ? (BipedCaptKey*)_obj->next : 0);

	AddSLink(obj[1]->var_swg_t );
	AddSLink(obj[0]->var_sup_t);
}

BipedCaptSwgConR::BipedCaptSwgConR(Solver* solver, string _name, BipedCaptKey* _obj, real_t _scale) :
	Constraint(solver, 1, ID(ConTag::BipedCaptSwgR, _obj->node, _obj->tick, _name), Constraint::Type::Equality, _scale) {
	obj[0] = _obj;
	obj[1] = (_obj->next ? (BipedCaptKey*)_obj->next : 0);

	AddSLink(obj[1]->var_swg_r );
	AddSLink(obj[0]->var_sup_r);
}

BipedCaptDurationCon::BipedCaptDurationCon(Solver* solver, string _name, BipedCaptKey* _obj, real_t _scale) :
	Constraint(solver, 1, ID(ConTag::BipedCaptDuration, _obj->node, _obj->tick, _name), Constraint::Type::InequalityPenalty, _scale) {
	obj = _obj;

	AddR3Link(obj->var_swg_t   );
	AddSLink (obj->var_swg_r   );
	AddR3Link(obj->var_land_t  );
	AddSLink (obj->var_land_r  );
	AddSLink (obj->var_duration);
}

BipedCaptDesCopCon::BipedCaptDesCopCon(Solver* solver, string _name, BipedCaptKey* _obj, real_t _scale) :
	Constraint(solver, 3, ID(ConTag::BipedCaptCop, _obj->node, _obj->tick, _name), Constraint::Type::Equality, _scale) {
	obj = _obj;

	AddSLink(obj->var_cop);
	AddSLink(obj->var_sup_t);
}

BipedCaptLandRangeConT::BipedCaptLandRangeConT(Solver* solver, string _name, BipedCaptKey* _obj, vec3_t _dir, real_t _scale) :
	Constraint(solver, 1, ID(ConTag::BipedCaptLandRangeT, _obj->node, _obj->tick, _name), Constraint::Type::InequalityPenalty, _scale) {

	obj = _obj;
	dir = _dir;
	
	AddR3Link(obj->var_land_t);
	AddR3Link(obj->var_sup_t );
	AddSLink (obj->var_sup_r );
}

BipedCaptLandRangeConR::BipedCaptLandRangeConR(Solver* solver, string _name, BipedCaptKey* _obj, real_t _dir, real_t _scale) :
	Constraint(solver, 1, ID(ConTag::BipedCaptLandRangeR, _obj->node, _obj->tick, _name), Constraint::Type::InequalityPenalty, _scale) {

	obj = _obj;
	dir = _dir;

	AddSLink(obj->var_land_r);
	AddSLink(obj->var_sup_r );
}

BipedCaptCopRangeCon::BipedCaptCopRangeCon(Solver* solver, string _name, BipedCaptKey* _obj, vec3_t _dir, real_t _scale) :
	Constraint(solver, 1, ID(ConTag::BipedCaptCopRange, _obj->node, _obj->tick, _name), Constraint::Type::InequalityPenalty, _scale) {

	obj = _obj;
	dir = _dir;
	
	AddR3Link(obj->var_cop  );
	AddR3Link(obj->var_sup_t);
	AddSLink (obj->var_sup_r);
}

BipedCaptIcpRangeCon::BipedCaptIcpRangeCon(Solver* solver, string _name, BipedCaptKey* _obj, vec3_t _dir, real_t _scale) :
	Constraint(solver, 1, ID(ConTag::BipedCaptIcpRange, _obj->node, _obj->tick, _name), Constraint::Type::InequalityPenalty, _scale) {

	obj = _obj;
	dir = _dir;
	
	AddR3Link(obj->var_icp  );
	AddR3Link(obj->var_sup_t);
	AddSLink (obj->var_sup_r);
}

//-------------------------------------------------------------------------------------------------

void BipedCaptIcpCon::CalcLhs() {
	Prepare();
	obj[1]->var_icp->val = icp_rhs;
}

void BipedCaptSupConT::CalcLhs() {
	Prepare();
	obj[1]->var_sup_t->val = (stepping ? obj[0]->var_land_t->val : obj[0]->var_swg_t->val);
}

void BipedCaptSupConR::CalcLhs() {
	Prepare();
	obj[1]->var_sup_r->val = (stepping ? obj[0]->var_land_r->val : obj[0]->var_swg_r->val);
}

void BipedCaptSwgConT::CalcLhs() {
	Prepare();
	obj[1]->var_swg_t->val = obj[0]->var_sup_t->val;
}

void BipedCaptSwgConR::CalcLhs() {
	Prepare();
	obj[1]->var_swg_r->val = obj[0]->var_swg_r->val;
}

//-------------------------------------------------------------------------------------------------

void BipedCaptIcpCon::Prepare(){
	tau   = obj[0]->var_duration->val;
	T     = ((BipedCapt*)obj[0]->node)->param.T;
	alpha = exp(tau/T);

	cop = obj[0]->var_cop->val;
	icp = obj[0]->var_icp->val;

	icp_rhs = alpha*(icp - cop) + cop;
}

void BipedCaptSupConT::Prepare(){
	stepping = ((BipedCapt*)obj[0]->node)->param.steps[obj[0]->tick->idx].stepping;
}

void BipedCaptSupConR::Prepare(){
	stepping = ((BipedCapt*)obj[0]->node)->param.steps[obj[0]->tick->idx].stepping;
}

void BipedCaptSwgConT::Prepare(){
}

void BipedCaptSwgConR::Prepare(){
}

void BipedCaptDurationCon::Prepare(){
	dp  = obj->var_land_t->val - obj->var_swg_t->val;
	dr  = obj->var_land_r->val - obj->var_swg_r->val;

	dpnorm = dp.norm();
	drabs  = std::abs(dr);

	if(dpnorm > eps)
		 dpn = dp/dpnorm;
	else dpn.clear();

	if(drabs > eps)
		 drn = dr/drabs;
	else drn = 0.0;

	vmax      = ((BipedCapt*)obj->node)->param.vmax;
	wmax      = ((BipedCapt*)obj->node)->param.wmax;
	tau_const = ((BipedCapt*)obj->node)->param.tau_const;

	vmax_inv = 1.0/vmax;
	wmax_inv = 1.0/wmax;

	tau = obj->var_duration->val;
}

void BipedCaptLandRangeConT::Prepare(){
	r     = obj->var_land_t->val - obj->var_sup_t->val;
	theta = obj->var_sup_r->val;

	R       = mat3_t::Rot(theta, 'z');
	ez      = vec3_t(0.0, 0.0, 1.0);
	dir_abs = R*dir;
}

void BipedCaptLandRangeConR::Prepare(){
	theta_land = obj->var_land_r->val;
	theta_sup  = obj->var_sup_r ->val;

	r = theta_land - theta_sup;
	
	while(r >  pi) r -= _2pi;
	while(r < -pi) r += _2pi;
}

void BipedCaptIcpRangeCon::Prepare(){
	r     = obj->var_icp->val - obj->var_sup_t->val;
	theta = obj->var_sup_r->val;

	R       = mat3_t::Rot(theta, 'z');
	ez      = vec3_t(0.0, 0.0, 1.0);
	dir_abs = R*dir;
}

void BipedCaptCopRangeCon::Prepare(){
	r     = obj->var_cop->val - obj->var_sup_t->val;
	theta = obj->var_sup_r->val;

	R       = mat3_t::Rot(theta, 'z');
	ez      = vec3_t(0.0, 0.0, 1.0);
	dir_abs = R*dir;
}

//-------------------------------------------------------------------------------------------------

void BipedCaptIcpCon::CalcCoef() {
	Prepare();

	((SLink *)links[0])->SetCoef( 1.0);
	((SLink *)links[1])->SetCoef(-alpha);
	((SLink *)links[2])->SetCoef(-(1.0 - alpha));
	((C3Link*)links[3])->SetCoef(-(alpha/T)*(icp - cop));

}

void BipedCaptSupConT::CalcCoef() {
	Prepare();

	((SLink*)links[0])->SetCoef( 1.0);
	((SLink*)links[1])->SetCoef(stepping ?  0.0 : -1.0);
	((SLink*)links[2])->SetCoef(stepping ? -1.0 :  0.0);
}

void BipedCaptSupConR::CalcCoef() {
	Prepare();

	((SLink*)links[0])->SetCoef( 1.0);
	((SLink*)links[1])->SetCoef(stepping ?  0.0 : -1.0);
	((SLink*)links[2])->SetCoef(stepping ? -1.0 :  0.0);
}

void BipedCaptSwgConT::CalcCoef() {
	Prepare();

	((SLink*)links[0])->SetCoef( 1.0);
	((SLink*)links[1])->SetCoef(-1.0);
}

void BipedCaptSwgConR::CalcCoef() {
	Prepare();

	((SLink*)links[0])->SetCoef( 1.0);
	((SLink*)links[1])->SetCoef(-1.0);
}

void BipedCaptDesCopCon::CalcCoef(){
	((SLink*)links[0])->SetCoef( 1.0);
	((SLink*)links[1])->SetCoef(-1.0);
}

void BipedCaptDurationCon::CalcCoef(){
	Prepare();

	((R3Link*)links[0])->SetCoef( dpn*vmax_inv);
	((SLink *)links[1])->SetCoef( drn*wmax_inv);
	((R3Link*)links[2])->SetCoef(-dpn*vmax_inv);
	((SLink *)links[3])->SetCoef(-drn*wmax_inv);
	((SLink *)links[4])->SetCoef(1.0);
}

void BipedCaptLandRangeConT::CalcCoef(){
	Prepare();

	// (R eta)^T (pland - psup) >= lim
	((R3Link*)links[0])->SetCoef( dir_abs);
	((R3Link*)links[1])->SetCoef(-dir_abs);
	((SLink* )links[2])->SetCoef( (dir_abs % r)*ez);

}

void BipedCaptLandRangeConR::CalcCoef(){
	Prepare();

	((SLink*)links[0])->SetCoef( dir);
	((SLink*)links[1])->SetCoef(-dir);
	
}

void BipedCaptIcpRangeCon::CalcCoef(){
	Prepare();

	((R3Link*)links[0])->SetCoef( dir_abs);
	((R3Link*)links[1])->SetCoef(-dir_abs);
	((SLink* )links[2])->SetCoef( (dir_abs % r)*ez);
}

void BipedCaptCopRangeCon::CalcCoef(){
	Prepare();

	((R3Link*)links[0])->SetCoef( dir_abs);
	((R3Link*)links[1])->SetCoef(-dir_abs);
	((SLink* )links[2])->SetCoef( (dir_abs % r)*ez);
}

//-------------------------------------------------------------------------------------------------

void BipedCaptIcpCon::CalcDeviation() {
	y = obj[1]->var_icp->val - icp_rhs;

	//DSTR << "icp: " << y << endl;
}

void BipedCaptSupConT::CalcDeviation() {
	y = obj[1]->var_sup_t->val - (stepping ? obj[0]->var_land_t->val : obj[0]->var_swg_t->val);
	//DSTR << "sup_t: " << y << endl;
}

void BipedCaptSupConR::CalcDeviation() {
	y[0] = obj[1]->var_sup_r->val - (stepping ? obj[0]->var_land_r->val : obj[0]->var_swg_r->val);
	//DSTR << "sup_r: " << y << endl;
}

void BipedCaptSwgConT::CalcDeviation() {
	y = obj[1]->var_swg_t->val - obj[0]->var_sup_t->val;
	//DSTR << "swg_t: " << y << endl;
}

void BipedCaptSwgConR::CalcDeviation() {
	y[0] = obj[1]->var_swg_r->val - obj[0]->var_sup_r->val;
	//DSTR << "swg_r: " << y << endl;
}

void BipedCaptDesCopCon::CalcDeviation(){
	y = obj->var_cop->val - obj->var_sup_t->val;
}

void BipedCaptDurationCon::CalcDeviation(){
	real_t s = tau - (tau_const + dpnorm*vmax_inv + drabs*wmax_inv);

	//DSTR << tau << " " << dpnorm << " " << drabs << endl;

	if(s < 0.0){
		y[0] = s;
		active = true;
	}
	else{
		y[0] = 0.0;
		active = false;
	}
}

void BipedCaptLandRangeConT::CalcDeviation(){
	real_t s = dir_abs*r;
	
	if(s < lim) {
		y[0] = s - lim;
		active = true;
	}
	else{
		y[0] = 0.0;
		active = false;
	}
}

void BipedCaptLandRangeConR::CalcDeviation() {
	real_t s = dir*r;

	if(s < lim) {
		y[0] = s - lim;
		active = true;
	}
	else{
		y[0] = 0.0;
		active = false;
	}
}

void BipedCaptCopRangeCon::CalcDeviation(){
	real_t s = dir_abs*r;
	
	if(s < lim) {
		//DSTR << "cop range " << obj->tick->idx << " " << s - lim << endl;
		y[0] = s - lim;
		active = true;
	}
	else{
		y[0] = 0.0;
		active = false;
	}
}

void BipedCaptIcpRangeCon::CalcDeviation(){
	real_t s = dir_abs*r;
	
	if(s < lim) {
		y[0] = s - lim;
		active = true;
	}
	else{
		y[0] = 0.0;
		active = false;
	}
}

}
