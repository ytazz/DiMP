#include <DiMP/Graph/Biped.h>
#include <DiMP/Graph/Graph.h>
#include <DiMP/Render/Config.h>
#include <DiMP/Render/Canvas.h>

#include<stdio.h>

#include <sbrollpitchyaw.h>

namespace DiMP {;

const real_t   pi = M_PI;
const real_t _2pi = 2.0*pi;

const real_t eps = 0.1;

//-------------------------------------------------------------------------------------------------
// BipedLIPKey

BipedLIPKey::BipedLIPKey() {

}

// register variables for planning
void BipedLIPKey::AddVar(Solver* solver) {
	BipedLIP* obj = (BipedLIP*)node;

	vec3_t one(1.0, 1.0, 1.0);

	// torso position and velocity
	var_torso_pos_t = new V3Var(solver, ID(VarTag::BipedTorsoTP, node, tick, name + "_torso_tp"), node->graph->scale.pos_t);
	var_torso_pos_r = new SVar (solver, ID(VarTag::BipedTorsoRP, node, tick, name + "_torso_rp"), node->graph->scale.pos_r);
	var_torso_vel_t = new V3Var(solver, ID(VarTag::BipedTorsoTV, node, tick, name + "_torso_tv"), node->graph->scale.vel_t);
	solver->AddInputVar(var_torso_pos_t, tick->idx);
	solver->AddInputVar(var_torso_pos_r, tick->idx);
	solver->AddInputVar(var_torso_vel_t, tick->idx);
	var_torso_pos_t->weight    = eps*one;
	var_torso_pos_r->weight[0] = eps;
	var_torso_vel_t->weight    = eps*one;
	
	// foot position
	var_foot_pos_t[0] = new V3Var(solver, ID(VarTag::BipedFootT, node, tick, name + "_foot_r_t"), node->graph->scale.pos_t);
	var_foot_pos_r[0] = new SVar (solver, ID(VarTag::BipedFootR, node, tick, name + "_foot_r_r"), node->graph->scale.pos_r);
	var_foot_pos_t[1] = new V3Var(solver, ID(VarTag::BipedFootT, node, tick, name + "_foot_l_t"), node->graph->scale.pos_t);
	var_foot_pos_r[1] = new SVar (solver, ID(VarTag::BipedFootR, node, tick, name + "_foot_l_r"), node->graph->scale.pos_r);
	solver->AddInputVar(var_foot_pos_t[0], tick->idx);
	solver->AddInputVar(var_foot_pos_r[0], tick->idx);
	solver->AddInputVar(var_foot_pos_t[1], tick->idx);
	solver->AddInputVar(var_foot_pos_r[1], tick->idx);
	var_foot_pos_t[0]->weight    = eps*one;
	var_foot_pos_r[0]->weight[0] = eps;
	var_foot_pos_t[1]->weight    = eps*one;
	var_foot_pos_r[1]->weight[0] = eps;

	// CoM position and velocity
	var_com_pos = new V3Var(solver, ID(VarTag::BipedComP, node, tick, name + "_com_p"), node->graph->scale.pos_t);
	var_com_vel = new V3Var(solver, ID(VarTag::BipedComV, node, tick, name + "_com_v"), node->graph->scale.vel_t);
	var_com_pos->weight = eps*one;
	var_com_vel->weight = eps*one;
	
	// angular momentum
	var_mom     = new V3Var(solver, ID(VarTag::BipedMom, node, tick, name + "_mom"), node->graph->scale.vel_r);  //< no proper matching scale for angular momentum...
	var_mom->weight = eps*one;

	// CoP position
	var_cop_pos = new V3Var(solver, ID(VarTag::BipedCopP, node, tick, name + "_cop_p"), node->graph->scale.pos_t);
	var_cop_pos->weight = eps*one;

	// CMP offset
	var_cmp_pos = new V3Var(solver, ID(VarTag::BipedCmpP, node, tick, name + "_cmp_p"), node->graph->scale.pos_t);
	var_cmp_pos->weight = eps*one;

	// absolute time
	var_time = new SVar(solver, ID(VarTag::BipedTime, node, tick, name + "_time"), node->graph->scale.time);
	var_time->weight[0] = eps;

	// register state variables of ddp
	solver->AddStateVar(var_com_pos, tick->idx);
	solver->AddStateVar(var_com_vel, tick->idx);
	solver->AddStateVar(var_mom    , tick->idx);
	solver->AddStateVar(var_cop_pos, tick->idx);
	solver->AddStateVar(var_cmp_pos, tick->idx);
	solver->AddStateVar(var_time   , tick->idx);

	if(next){
		// CoP velocity
		var_cop_vel  = new V3Var(solver, ID(VarTag::BipedCopV, node, tick, name + "_cop_v"), node->graph->scale.vel_t);
		var_cop_vel->weight = eps*one;

		// CMP velocity
		var_cmp_vel  = new V3Var(solver, ID(VarTag::BipedCmpV, node, tick, name + "_cmp_v"), node->graph->scale.vel_t);
		var_cmp_vel->weight = eps*one;

		// step duration
		var_duration = new SVar(solver, ID(VarTag::BipedDuration, node, tick, name + "_duration"), node->graph->scale.time);
		var_duration->weight[0] = eps;

		// register input variables of ddp
		solver->AddInputVar(var_cop_vel , tick->idx);
		solver->AddInputVar(var_cmp_vel , tick->idx);
		solver->AddInputVar(var_duration, tick->idx);
	}
}

// register constraints for planning
void BipedLIPKey::AddCon(Solver* solver) {
	BipedLIP* obj = (BipedLIP*)node;
	BipedLIPKey* nextObj = (BipedLIPKey*)next;

	if (next) {
		con_lip_pos = new BipedLipPosCon(solver, name + "_lip_pos", this, node->graph->scale.pos_t);
		con_lip_vel = new BipedLipVelCon(solver, name + "_lip_vel", this, node->graph->scale.vel_t);
		con_lip_cop = new BipedLipCopCon(solver, name + "_lip_cop", this, node->graph->scale.pos_t);
		con_lip_cmp = new BipedLipCmpCon(solver, name + "_lip_cmp", this, node->graph->scale.pos_t);
		con_lip_mom = new BipedLipMomCon(solver, name + "_lip_mom", this, node->graph->scale.vel_r);

		solver->AddTransitionCon(con_lip_pos, tick->idx);
		solver->AddTransitionCon(con_lip_vel, tick->idx);
		solver->AddTransitionCon(con_lip_cop, tick->idx);
		solver->AddTransitionCon(con_lip_cmp, tick->idx);
		solver->AddTransitionCon(con_lip_mom, tick->idx);
	}

	con_foot_range_t[0][0] = new BipedFootRangeConT(solver, name + "_foot_range_r_t", this, 0, vec3_t(1.0, 0.0, 0.0), node->graph->scale.pos_t);
	con_foot_range_t[0][1] = new BipedFootRangeConT(solver, name + "_foot_range_r_t", this, 0, vec3_t(0.0, 1.0, 0.0), node->graph->scale.pos_t);
	con_foot_range_t[0][2] = new BipedFootRangeConT(solver, name + "_foot_range_r_t", this, 0, vec3_t(0.0, 0.0, 1.0), node->graph->scale.pos_t);
	con_foot_range_r[0]    = new BipedFootRangeConR(solver, name + "_foot_range_r_r", this, 0,                        node->graph->scale.pos_r);
	con_foot_range_t[1][0] = new BipedFootRangeConT(solver, name + "_foot_range_l_t", this, 1, vec3_t(1.0, 0.0, 0.0), node->graph->scale.pos_t);
	con_foot_range_t[1][1] = new BipedFootRangeConT(solver, name + "_foot_range_l_t", this, 1, vec3_t(0.0, 1.0, 0.0), node->graph->scale.pos_t);
	con_foot_range_t[1][2] = new BipedFootRangeConT(solver, name + "_foot_range_l_t", this, 1, vec3_t(0.0, 0.0, 1.0), node->graph->scale.pos_t);
	con_foot_range_r[1]    = new BipedFootRangeConR(solver, name + "_foot_range_l_r", this, 1,                        node->graph->scale.pos_r);
	solver->AddCostCon(con_foot_range_t[0][0], tick->idx);
	solver->AddCostCon(con_foot_range_t[0][1], tick->idx);
	solver->AddCostCon(con_foot_range_t[0][2], tick->idx);
	solver->AddCostCon(con_foot_range_r[0]   , tick->idx);
	solver->AddCostCon(con_foot_range_t[1][0], tick->idx);
	solver->AddCostCon(con_foot_range_t[1][1], tick->idx);
	solver->AddCostCon(con_foot_range_t[1][2], tick->idx);
	solver->AddCostCon(con_foot_range_r[1]   , tick->idx);

	// cop range constraint
	int phase = obj->phase[tick->idx];
	int side;
	if (phase == BipedLIP::Phase::R || phase == BipedLIP::Phase::RL)
		 side = 0;
	else side = 1;
	con_cop_range[0] = new BipedCopRangeCon(solver, name + "_cop_range", this, side, vec3_t(1.0, 0.0, 0.0), node->graph->scale.pos_t);
	con_cop_range[1] = new BipedCopRangeCon(solver, name + "_cop_range", this, side, vec3_t(0.0, 1.0, 0.0), node->graph->scale.pos_t);
	con_cop_range[2] = new BipedCopRangeCon(solver, name + "_cop_range", this, side, vec3_t(0.0, 0.0, 1.0), node->graph->scale.pos_t);
	solver->AddCostCon(con_cop_range[0], tick->idx);
	solver->AddCostCon(con_cop_range[1], tick->idx);
	solver->AddCostCon(con_cop_range[2], tick->idx);

	// cmp range constraint
	con_cmp_range[0] = new BipedCmpRangeCon(solver, name + "_cmp_range", this, vec3_t(1.0, 0.0, 0.0), node->graph->scale.pos_t);
	con_cmp_range[1] = new BipedCmpRangeCon(solver, name + "_cmp_range", this, vec3_t(0.0, 1.0, 0.0), node->graph->scale.pos_t);
	con_cmp_range[2] = new BipedCmpRangeCon(solver, name + "_cmp_range", this, vec3_t(0.0, 0.0, 1.0), node->graph->scale.pos_t);
	solver->AddCostCon(con_cmp_range[0], tick->idx);
	solver->AddCostCon(con_cmp_range[1], tick->idx);
	solver->AddCostCon(con_cmp_range[2], tick->idx);

	// acc range constraint
	con_acc_range[0] = new BipedAccRangeCon(solver, name + "_acc_range", this, vec3_t(1.0, 0.0, 0.0), node->graph->scale.acc_t);
	con_acc_range[1] = new BipedAccRangeCon(solver, name + "_acc_range", this, vec3_t(0.0, 1.0, 0.0), node->graph->scale.acc_t);
	con_acc_range[2] = new BipedAccRangeCon(solver, name + "_acc_range", this, vec3_t(0.0, 0.0, 1.0), node->graph->scale.acc_t);
	solver->AddCostCon(con_acc_range[0], tick->idx);
	solver->AddCostCon(con_acc_range[1], tick->idx);
	solver->AddCostCon(con_acc_range[2], tick->idx);

	// mom range constraint
	con_mom_range[0] = new BipedMomRangeCon(solver, name + "_mom_range", this, vec3_t(1.0, 0.0, 0.0), node->graph->scale.vel_t);
	con_mom_range[1] = new BipedMomRangeCon(solver, name + "_mom_range", this, vec3_t(0.0, 1.0, 0.0), node->graph->scale.vel_t);
	con_mom_range[2] = new BipedMomRangeCon(solver, name + "_mom_range", this, vec3_t(0.0, 0.0, 1.0), node->graph->scale.vel_t);
	solver->AddCostCon(con_mom_range[0], tick->idx);
	solver->AddCostCon(con_mom_range[1], tick->idx);
	solver->AddCostCon(con_mom_range[2], tick->idx);

	if (next) {
		con_duration_range = new RangeConS(solver, ID(ConTag::BipedDurationRange, node, tick, name + "_duration"), var_duration, node->graph->scale.time);
		solver->AddCostCon(con_duration_range, tick->idx);

		con_time = new BipedTimeCon(solver, name + "_time", this, node->graph->scale.time);
		solver->AddTransitionCon(con_time, tick->idx);
	}

	if (next) {
		con_foot_match_t[0] = new MatchConV3(solver, ID(ConTag::BipedFootMatchT, node, tick, name + "_foot_match_r_t"), var_foot_pos_t[0], nextObj->var_foot_pos_t[0], node->graph->scale.pos_t);
		con_foot_match_r[0] = new MatchConS (solver, ID(ConTag::BipedFootMatchR, node, tick, name + "_foot_match_r_r"), var_foot_pos_r[0], nextObj->var_foot_pos_r[0], node->graph->scale.pos_r);
		con_foot_match_t[1] = new MatchConV3(solver, ID(ConTag::BipedFootMatchT, node, tick, name + "_foot_match_l_t"), var_foot_pos_t[1], nextObj->var_foot_pos_t[1], node->graph->scale.pos_t);
		con_foot_match_r[1] = new MatchConS (solver, ID(ConTag::BipedFootMatchR, node, tick, name + "_foot_match_l_r"), var_foot_pos_r[1], nextObj->var_foot_pos_r[1], node->graph->scale.pos_r);
		solver->AddCostCon(con_foot_match_t[0], tick->idx);
		solver->AddCostCon(con_foot_match_r[0], tick->idx);
		solver->AddCostCon(con_foot_match_t[1], tick->idx);
		solver->AddCostCon(con_foot_match_r[1], tick->idx);

		con_foot_height [0] = new BipedFootHeightCon(solver, name + "_foot_height_r", this, 0, node->graph->scale.pos_t);
		con_foot_height [1] = new BipedFootHeightCon(solver, name + "_foot_height_l", this, 1, node->graph->scale.pos_t);
		solver->AddCostCon(con_foot_height [0], tick->idx);
		solver->AddCostCon(con_foot_height [1], tick->idx);
	}

	con_com_pos = new BipedComConP(solver, name + "_com_p", this, node->graph->scale.pos_t);
	con_com_vel = new BipedComConV(solver, name + "_com_v", this, node->graph->scale.vel_t);
	solver->AddCostCon(con_com_pos, tick->idx);
	solver->AddCostCon(con_com_vel, tick->idx);

	//con_lip_acc = new BipedLipAccCon(solver, name + "_com_acc", this, node->graph->scale.pos_t);
	//solver->AddCostCon(con_lip_acc, tick->idx);
	//
	//con_mom_zero = new FixConV2(solver, ID(ConTag::BipedLipMom, node, tick, name + "_mom_zero"), var_cmp_pos, node->graph->scale.pos_t);
	//con_mom_zero->desired.clear();
}

void BipedLIPKey::Prepare() {
	
}

void BipedLIPKey::Finish(){
	// tick's time is updated from time variable
	tick->time = var_time->val;
}

void BipedLIPKey::Draw(Render::Canvas* canvas, Render::Config* conf) {
	BipedLIP* obj = (BipedLIP*)node;

	Vec3f pcom, pcop, pf[2], pt;
	float thetaf[2];

	canvas->SetPointSize(5.0f);
	canvas->SetLineWidth(1.0f);

	pcom.x = (float)var_com_pos->val.x;
	pcom.y = (float)var_com_pos->val.y;
	pcom.z = (float)var_com_pos->val.z;
	//pcom.z = (float)((BipedLIP*)node)->param.comHeight;
	canvas->Point(pcom);

	// feet
	Vec3f cmin = obj->param.copMin; //CoPMinmum
	Vec3f cmax = obj->param.copMax; //CoPMax

	for (uint i = 0; i < 2; i++) {
		pf[i].x = (float)var_foot_pos_t[i]->val.x;
		pf[i].y = (float)var_foot_pos_t[i]->val.y;
		pf[i].z = (float)var_foot_pos_t[i]->val.z;
		canvas->Point(pf[i]);

		// foot print
		canvas->Push();
		thetaf[i] = (float)var_foot_pos_r[i]->val;
		canvas->Translate(pf[i].x, pf[i].y, pf[i].z);
		canvas->Rotate(thetaf[i], Vec3f(0.0f, 0.0f, 1.0f));
		canvas->Line(Vec3f(cmax.x, cmax.y, 0.0f), Vec3f(cmin.x, cmax.y, 0.0f));
		canvas->Line(Vec3f(cmin.x, cmax.y, 0.0f), Vec3f(cmin.x, cmin.y, 0.0f));
		canvas->Line(Vec3f(cmin.x, cmin.y, 0.0f), Vec3f(cmax.x, cmin.y, 0.0f));
		canvas->Line(Vec3f(cmax.x, cmin.y, 0.0f), Vec3f(cmax.x, cmax.y, 0.0f));
		canvas->Pop();
	}

	// torso
	pt = obj->TorsoPos(pcom, pf[0], pf[1]);
	canvas->Point(pt);

	// lines connecting torso and feet
	canvas->Line(pt, pf[0]);
	canvas->Line(pt, pf[1]);

	// cop
	pcop.x = (float)var_cop_pos->val.x;
	pcop.y = (float)var_cop_pos->val.y;
	pcop.z = (float)(var_foot_pos_t[0]->val.z + var_foot_pos_t[0]->val.z)/2.0f;
	canvas->Point(pcop);

}

//-------------------------------------------------------------------------------------------------
// BipedLIP

BipedLIP::Param::Param() {
	gravity          = 9.8;
	comHeight        = 1.0;
	torsoMass        = 10.0;
	footMass         = 5.0;
	swingProfile     = SwingProfile::Cycloid;
	swingHeight[0]   = 0.1; //0: maximum swing foot height
	swingHeight[1]   = 0.1; //1: swing foot height before landing (Wedge only)
	comHeightProfile = ComHeightProfile::Constant;
	durationMin[Phase::R ] = 0.1; // duration minimum at single support
	durationMax[Phase::R ] = 0.8; // duration maximum at single support
	durationMin[Phase::L ] = 0.1;
	durationMax[Phase::L ] = 0.8;
	durationMin[Phase::RL] = 0.1; // duration minimum at double support
	durationMax[Phase::RL] = 0.2; // duration maximum at double support
	durationMin[Phase::LR] = 0.1;
	durationMax[Phase::LR] = 0.2;
	durationMin[Phase::D ] = 0.1;
	durationMax[Phase::D ] = 0.2;

	// range of foot position relative to com
	footPosMin[0] = vec3_t(-0.5, -0.20, 0.0);
	footPosMax[0] = vec3_t( 0.5, -0.00, 0.0);
	footPosMin[1] = vec3_t(-0.5,  0.00, 0.0);
	footPosMax[1] = vec3_t( 0.5,  0.20, 0.0);
	footOriMin[0] = Rad(-15.0);
	footOriMax[0] = Rad( 15.0);
	footOriMin[1] = Rad(-15.0);
	footOriMax[1] = Rad( 15.0);

	// range of cop relative to support foot
	copMin    = vec3_t(-0.1, -0.05, 0.0);
	copMax    = vec3_t( 0.1,  0.05, 0.0);

	// range of cmp offset
	cmpMin    = vec3_t(-0.0, -0.0, 0.0);
	cmpMax    = vec3_t( 0.0,  0.0, 0.0);

	// range of com acceleration
	accMin    = vec3_t(-1.0, 1.0, 0.0);
	accMax    = vec3_t(-1.0, 1.0, 0.0);

	// range of angular momentum
	momMin    = vec3_t(-1.0, 1.0, 0.0);
	momMax    = vec3_t(-1.0, 1.0, 0.0);

	ankleToToe        = 0.1;
	ankleToHeel       = 0.1;
	toeCurvature      = 10.0;
	heelCurvature     = 10.0;
	toeCurvatureRate  = 0.0;
	heelCurvatureRate = 0.0;

	minSpacing  = 0.0;
	swingMargin = 1.0;
}

//-------------------------------------------------------------------------------------------------
// Waypoints
BipedLIP::Waypoint::Waypoint() {
	k             = 0;
	time          = 0.0;
	com_pos       = vec3_t();
	com_vel       = vec3_t();
	torso_pos_r   = 0.0;
	
	foot_pos_t[0] = vec3_t();
	foot_pos_r[0] = 0.0;
	foot_pos_t[1] = vec3_t();
	foot_pos_r[1] = 0.0;
	cop_pos       = vec3_t();

	fix_com_pos       = false;
	fix_com_vel       = false;
	fix_torso_pos_r   = false;
	
	fix_foot_pos_t[0] = false;
	fix_foot_pos_r[0] = false;
	fix_foot_pos_t[1] = false;
	fix_foot_pos_r[1] = false;
	fix_com_pos       = false;
	fix_com_vel       = false;
	fix_cop_pos       = false;
	fix_cmp_pos       = false;
	fix_mom           = false;
}

//-------------------------------------------------------------------------------------------------
BipedLIP::Snapshot::Snapshot() {
	t = 0.0;
	torso_pos_t = vec3_t();
	torso_pos_r = 0.0;
	foot_pos_t[0] = vec3_t();
	foot_pos_r[0] = quat_t();
	foot_pos_t[1] = vec3_t();
	foot_pos_r[1] = quat_t();
}

//-------------------------------------------------------------------------------------------------

BipedLIP::BipedLIP(Graph* g, string n) :TrajectoryNode(g, n) {
	type = Type::Object;
	graph->bipeds.Add(this);
}

BipedLIP::~BipedLIP() {
	graph->bipeds.Remove(this);
}

void BipedLIP::Init() {
	TrajectoryNode::Init();

	// time constant of inverted pendulum
	param.T = sqrt(param.comHeight / param.gravity);

	real_t durationAve[Phase::Num];
	for (int i = 0; i < Phase::Num; i++)
		durationAve[i] = (param.durationMin[i] + param.durationMax[i]) / 2.0;

	real_t t = 0.0;
	for (uint k = 0; k < graph->ticks.size(); k++) {
		BipedLIPKey* key = (BipedLIPKey*)traj.GetKeypoint(graph->ticks[k]);

		key->tick->time    = t;
		key->var_time->val = t;

		int ph = phase[key->tick->idx];
			
		if (!key->prev) {
			// initial time is fixed
			key->var_time->val = 0.0;
			key->var_time->locked = true;
		}

		if (key->next) {
			key->con_foot_match_t[0]->enabled = (ph != BipedLIP::Phase::L);
			key->con_foot_match_r[0]->enabled = (ph != BipedLIP::Phase::L);
			key->con_foot_match_t[1]->enabled = (ph != BipedLIP::Phase::R);
			key->con_foot_match_r[1]->enabled = (ph != BipedLIP::Phase::R);

			key->con_foot_height [0]->enabled = (ph == BipedLIP::Phase::R || ph == BipedLIP::Phase::D);
			key->con_foot_height [1]->enabled = (ph == BipedLIP::Phase::L || ph == BipedLIP::Phase::D);
		}

		// initial value of step duration is set as the average of minimum and maximum
		if (key->next) {
			key->var_duration->val        = durationAve[phase[k]];
			key->con_duration_range->_min = param.durationMin[phase[k]];
			key->con_duration_range->_max = param.durationMax[phase[k]];
			
			//key->var_duration->locked = true;
		}

		// range limit of foot position
		for (int i = 0; i < 2; i++) {
			for(int j = 0; j < 3; j++){
				key->con_foot_range_t[i][j]->_min = param.footPosMin[i][j];
				key->con_foot_range_t[i][j]->_max = param.footPosMax[i][j];
			}
			key->con_foot_range_r[i]   ->_min = param.footOriMin[i];
			key->con_foot_range_r[i]   ->_max = param.footOriMax[i];
		}

		for(int j = 0; j < 3; j++){
			// cop range
			if(j == 1){
				key->con_cop_range[j]->_min = (ph == BipedLIP::Phase::L || ph == BipedLIP::Phase::LR ? param.copMin[j] : -param.copMax[j]);
				key->con_cop_range[j]->_max = (ph == BipedLIP::Phase::L || ph == BipedLIP::Phase::LR ? param.copMax[j] : -param.copMin[j]);
			}
			else{
				key->con_cop_range[j]->_min = param.copMin[j];
				key->con_cop_range[j]->_max = param.copMax[j];
			}
		
			// cmp range
			key->con_cmp_range[j]->_min = param.cmpMin[j];
			key->con_cmp_range[j]->_max = param.cmpMax[j];
			
			// acceleration range
			key->con_acc_range[j]->_min = param.accMin[j];
			key->con_acc_range[j]->_max = param.accMax[j];
			
			// angular momentum range
			key->con_mom_range[j]->_min = param.momMin[j];
			key->con_mom_range[j]->_max = param.momMax[j];
		}

		// cop is unconstrained for D phase to enable it to be inside the convex hull of both feet
		if(ph == BipedLIP::Phase::D){
			for(int j = 0; j < 3; j++)
				key->con_cop_range[j]->enabled = false;
		}

		t += durationAve[phase[k]];
	}

	// initial value of com position, com velocity, and footsteps are calculated from a spline curve passing the waypoints
	Curve3d curve_com;
	Curved  curve_torso_r;
	Curve3d curve_foot_t[2];
	Curved  curve_foot_r[2];

	curve_com      .SetType(Interpolate::Cubic);
	curve_torso_r  .SetType(Interpolate::LinearDiff);
	curve_foot_t[0].SetType(Interpolate::Cubic);
	curve_foot_r[0].SetType(Interpolate::Cubic);
	curve_foot_t[1].SetType(Interpolate::Cubic);
	curve_foot_r[1].SetType(Interpolate::Cubic);

	for (uint i = 0; i < waypoints.size(); i++) {
		Waypoint& wp = waypoints[i];
		BipedLIPKey* key = (BipedLIPKey*)traj.GetKeypoint(graph->ticks[wp.k]);
		real_t t = key->var_time->val;

		curve_com.AddPoint(t);
		curve_com.SetPos(i, wp.com_pos);
		curve_com.SetVel(i, wp.com_vel);

		curve_torso_r.AddPoint(t);
		curve_torso_r.SetPos(i, wp.torso_pos_r);
		
		for (uint j = 0; j < 2; j++) {
			curve_foot_t[j].AddPoint(t);
			curve_foot_t[j].SetPos(i, wp.foot_pos_t[j]);
			curve_foot_t[j].SetVel(i, wp.com_vel);

			curve_foot_r[j].AddPoint(t);
			curve_foot_r[j].SetPos(i, wp.foot_pos_r[j]);
			curve_foot_r[j].SetVel(i, 0.0);
		}
	}

	for (uint k = 0; k < graph->ticks.size(); k++) {
		BipedLIPKey* key = (BipedLIPKey*)traj.GetKeypoint(graph->ticks[k]);
		real_t t = key->var_time->val;

		key->var_com_pos->val = curve_com.CalcPos(t);
		key->var_com_vel->val = curve_com.CalcVel(t);

		key->var_torso_pos_r->val = curve_torso_r.CalcPos(t);
		
		for (uint j = 0; j < 2; j++) {
			key->var_foot_pos_t[j]->val = curve_foot_t[j].CalcPos(t);
			key->var_foot_pos_r[j]->val = curve_foot_r[j].CalcPos(t);
		}

		// cop (actually vrp) is place at the same place as the com
		key->var_cop_pos->val = vec3_t(key->var_com_pos->val.x, key->var_com_pos->val.y, elevation[key->tick->idx]);

		// cop is initialized to be at the center of the support region
		//vec3_t c = (param.copMin + param.copMax)/2.0;
		//if (phase[k] == Phase::R || phase[k] == Phase::RL)
		//	 key->var_cop_pos->val = key->var_foot_pos_t[0]->val + mat3_t::Rot(key->var_foot_pos_r[0]->val, 'z')*c;
		//else key->var_cop_pos->val = key->var_foot_pos_t[1]->val + mat3_t::Rot(key->var_foot_pos_r[1]->val, 'z')*c;

		real_t mt = param.torsoMass;
		real_t mf = param.footMass;
		key->var_torso_pos_t->val = (1.0 + 2.0*(mf/mt))*key->var_com_pos->val - (mf/mt)*(key->var_foot_pos_t[0]->val + key->var_foot_pos_t[1]->val);
		key->var_torso_vel_t->val = (1.0 + 2.0*(mf/mt))*key->var_com_vel->val;
		//key->var_com_pos->val = (mt * key->var_torso_pos_t->val + mf * (key->var_foot_pos_t[0]->val + key->var_foot_pos_t[1]->val)) / (mt + 2.0*mf);
		//key->var_com_vel->val = (mt * key->var_torso_vel_t->val) / (mt + 2.0*mf);

		// angular momentum is initialized to zero
		key->var_mom->val.clear();

		// cmp is initialized to zero
		key->var_cmp_pos->val.clear();
	}

	// initial value of cop and cmp velocity
	for (uint k = 0; k < graph->ticks.size(); k++) {
		BipedLIPKey* key  = (BipedLIPKey*)traj.GetKeypoint(graph->ticks[k]);
		BipedLIPKey* key1 = (BipedLIPKey*)key->next;
		if(key1){
			key->var_cop_vel->val = (key1->var_cop_pos->val - key->var_cop_pos->val) / key->var_duration->val;
			key->var_cmp_vel->val.clear();
		}
	}

	// variables at waypoints are fixed
	for (uint i = 0; i < waypoints.size(); i++) {
		Waypoint& wp = waypoints[i];
		BipedLIPKey* key = (BipedLIPKey*)traj.GetKeypoint(graph->ticks[wp.k]);

		key->var_com_pos    ->locked = wp.fix_com_pos;
		key->var_com_vel    ->locked = wp.fix_com_vel;
		key->var_torso_pos_r->locked = wp.fix_torso_pos_r;
		key->var_cop_pos    ->locked = wp.fix_cop_pos;
		key->var_cmp_pos    ->locked = wp.fix_cmp_pos;
		key->var_mom        ->locked = wp.fix_mom;

		if(wp.fix_cop_pos)
			key->var_cop_pos->val = wp.cop_pos;

		for (uint j = 0; j < 2; j++) {
			key->var_foot_pos_t[j]->locked = wp.fix_foot_pos_t[j];
			key->var_foot_pos_r[j]->locked = wp.fix_foot_pos_r[j];
		}
	}
}

void BipedLIP::Prepare() {
	TrajectoryNode::Prepare();
	trajReady = false;
}

void BipedLIP::Finish() {
	TrajectoryNode::Finish();
}

int BipedLIP::Phase(real_t t) {
	BipedLIPKey* key = (BipedLIPKey*)traj.GetSegment(t).first;
	return phase[key->tick->idx];
}

void BipedLIP::ComState(real_t t, vec3_t& pos, vec3_t& vel, vec3_t& acc){
	BipedLIPKey* key0 = (BipedLIPKey*)traj.GetSegment(t).first;
	BipedLIPKey* key1 = (BipedLIPKey*)traj.GetSegment(t).second;

	real_t dt = t - key0->var_time->val;

	if(key1 == key0->next){
		real_t T    = param.T;
		real_t T2   = T * T;
		vec3_t p0   = key0->var_com_pos->val;
		vec3_t v0   = key0->var_com_vel->val;
		vec3_t c0   = key0->var_cop_pos->val + vec3_t(0.0, 0.0, param.comHeight);
		vec3_t cm0  = key0->var_cmp_pos->val;
		vec3_t cv0  = key0->var_cop_vel->val;
		vec3_t cmv0 = key0->var_cmp_vel->val;
	
		pos = (c0+cm0) + (cv0+cmv0)*dt + cosh(dt/T)*(p0 - (c0+cm0)) + T*sinh(dt/T)*(v0 - (cv0+cmv0));
		vel = (cv0+cmv0) + (1/T)*sinh(dt/T)*(p0 - (c0+cm0)) + cosh(dt/T)*(v0 - (cv0+cmv0));
		acc = (1/T2)*cosh(dt/T)*(p0 - (c0+cm0)) + (1/T)*sinh(dt/T)*(v0 - (cv0+cmv0));
	}
	else{
		pos = key0->var_com_pos->val;
		vel = key0->var_com_vel->val;
		acc = vec3_t();
	}
}

vec3_t BipedLIP::Momentum(real_t t){
	BipedLIPKey* key0 = (BipedLIPKey*)traj.GetSegment(t).first;
	BipedLIPKey* key1 = (BipedLIPKey*)traj.GetSegment(t).second;

	vec3_t Lt;

	real_t m = param.torsoMass + 2.0*param.footMass;
	real_t g = param.gravity;

	if(key1 == key0->next){
		real_t dt   = t - key0->var_time->val;
		vec3_t L0   = key0->var_mom    ->val;
		vec3_t cm0  = key0->var_cmp_pos->val;
		vec3_t cmv0 = key0->var_cmp_vel->val;
	
		Lt = L0 + vec3_t(0.0, 0.0, 1.0) % (cm0*dt + 0.5*dt*dt*cmv0);
	}
	else{
		Lt = vec3_t();
	}
	
	// momentum is internally normalized, so multiply by m*g to obtain actual momentum
	return (m*g)*Lt;
}

real_t BipedLIP::TorsoOri(real_t t) {
	BipedLIPKey* key0 = (BipedLIPKey*)traj.GetSegment(t).first;
	BipedLIPKey* key1 = (BipedLIPKey*)traj.GetSegment(t).second;

	real_t o;

	if (key1 == key0->next) {
		real_t t0 = key0->var_time->val;
		real_t t1 = key1->var_time->val;

		o = InterpolatePos(
			t,
			t0, key0->var_torso_pos_r->val, 0.0,
			t1, key1->var_torso_pos_r->val, 0.0,
			Interpolate::LinearDiff
		);
	}
	else {
		o = key0->var_torso_pos_r->val;
	}

	return o;
}

real_t BipedLIP::TorsoAngVel(real_t t) {
	BipedLIPKey* key0 = (BipedLIPKey*)traj.GetSegment(t).first;
	BipedLIPKey* key1 = (BipedLIPKey*)traj.GetSegment(t).second;

	real_t r;

	if (key1 == key0->next) {
		real_t t0 = key0->var_time->val;
		real_t t1 = key1->var_time->val;

		r = InterpolateVel(
			t,
			t0, key0->var_torso_pos_r->val, 0.0,
			t1, key1->var_torso_pos_r->val, 0.0,
			Interpolate::LinearDiff
		);
	}
	else {
		r = 0.0;
	}

	return r;
}

real_t BipedLIP::TorsoAngAcc(real_t t) {
	return 0.0;
}

real_t fresnelC(real_t d){
	return d - (1.0/10.0)*pow(d, 5.0);
}

real_t fresnelS(real_t d){
	return (1.0/3.0)*pow(d, 3.0) - (1.0/42.0)*pow(d, 7.0);
}

real_t clothoid_x(real_t d, real_t kappa){
	real_t tmp = sqrt(kappa/2.0);
	return (1.0/tmp)*fresnelC(tmp*d);
}

real_t clothoid_y(real_t d, real_t kappa){
	real_t tmp = sqrt(kappa/2.0);
	return (1.0/tmp)*fresnelS(tmp*d);
}

void BipedLIP::FootRotation(real_t px0, real_t pz0, real_t cp, real_t cv, vec3_t& pos, vec3_t& angle, vec3_t& vel, vec3_t& angvel, int& contact){
	// po : position of foot origin
	// cp : position of contact point
	// cv : velocity of contact point

	real_t l0     = param.ankleToToe ;
	real_t l1     = param.ankleToHeel;
	real_t rho0   = param.toeCurvature ;
	real_t rho1   = param.heelCurvature;
	real_t r0     = 1.0/rho0;
	real_t r1     = 1.0/rho1;
	real_t kappa0 = param.toeCurvatureRate ;
	real_t kappa1 = param.heelCurvatureRate;

	real_t d;          //< rolling distance
	vec2_t u, ud;      //< foot center to contact point on the ground, and its derivative
	vec2_t v, vd;      //< foot center to contact point on the foot, and its derivative
	real_t phi;
	real_t vphi;

	// current cop is on heel
	if(cp < px0 - l1){
		d = cp - (px0 - l1);
		if(param.footCurveType == FootCurveType::Arc){
			phi = d/r1;
			u   = vec2_t(-l1 + d, 0.0);
			v   = vec2_t(-l1 + r1*sin(phi), r1*(1.0 - cos(phi)));

			vphi = cv/r1;
			ud   = vec2_t(1.0, 0.0);
			vd   = vec2_t(cos(phi), sin(phi));
		}
		if(param.footCurveType == FootCurveType::Clothoid){
			phi = -0.5*kappa1*d*d;
			u   = vec2_t(-l1 + d, 0.0);
			v   = vec2_t(-l1 - clothoid_x(-d, kappa1), clothoid_y(-d, kappa1));

			vphi = -kappa1*d*cv;
			ud   = vec2_t(1.0, 0.0);
			vd   = vec2_t(cos(0.5*kappa1*d*d), -sin(0.5*kappa1*d*d));
		}

		contact = ContactState::Heel;
	}
	// current cop is on toe
	else if(cp > px0 + l0){
		d = cp - (px0 + l0);
		if(param.footCurveType == FootCurveType::Arc){
			phi = d/r0;
			u   = vec2_t(l0 + d, 0.0);
			v   = vec2_t(l0 + r0*sin(phi), r0*(1.0 - cos(phi)));

			vphi = cv/r0;
			ud   = vec2_t(1.0, 0.0);
			vd   = vec2_t(cos(phi), sin(phi));
		}
		if(param.footCurveType == FootCurveType::Clothoid){
			phi = 0.5*kappa0*d*d;
			u   = vec2_t(l0 + d, 0.0);
			v   = vec2_t(l0 + clothoid_x(d, kappa0), clothoid_y(d, kappa0));

			vphi = kappa0*d*cv;
			ud   = vec2_t(1.0, 0.0);
			vd   = vec2_t(cos(0.5*kappa0*d*d), sin(0.5*kappa0*d*d));
		}

		contact = ContactState::Toe;
	}
	// current cop is in the middle
	else{
		phi = 0.0;
		u   = vec2_t();
		v   = vec2_t();

		vphi = 0.0;
		ud   = vec2_t();
		vd   = vec2_t();

		contact = ContactState::Surface;
	}

	vec2_t pos2 = u - mat2_t::Rot(-phi)*v;
	vec2_t vel2 = (ud - mat2_t::Rot(-phi)*vd)*cv + mat2_t::Rot(-phi+ (pi/2.0))*v*vphi;

	pos.x     = pos2[0] + px0;
	pos.z     = pos2[1] + pz0;
	vel.x     = vel2[0];
	vel.z     = vel2[1];
	angle [1] = phi;
	angvel[1] = vphi;

}

void BipedLIP::FootPose(real_t t, int side, pose_t& pose, vec3_t& vel, vec3_t& angvel, vec3_t& acc, vec3_t& angacc, int& contact) {
	BipedLIPKey* keym2 = 0;
	BipedLIPKey* keym1 = 0;
	BipedLIPKey* key0 = (BipedLIPKey*)traj.GetSegment(t).first;
	BipedLIPKey* key1 = (BipedLIPKey*)traj.GetSegment(t).second;
	BipedLIPKey* key2  = 0;
	BipedLIPKey* key3  = 0;

	vec3_t pos;
	vec3_t angle;

	if(key1 != key0->next){
		pose.Pos() = key0->var_foot_pos_t[side]->val;
		pose.Ori() = FromRollPitchYaw(vec3_t(0.0, 0.0, key0->var_foot_pos_r[side]->val));
		
		vel   .clear();
		angvel.clear();
		acc   .clear();
		angacc.clear();

		contact = ContactState::Surface;

		return;
	}

	if(         key0 ->prev) keym1 = (BipedLIPKey*)key0 ->prev;
	if(keym1 && keym1->prev) keym2 = (BipedLIPKey*)keym1->prev;
	if(         key1 ->next) key2  = (BipedLIPKey*)key1 ->next;
	if(key2  && key2 ->next) key3  = (BipedLIPKey*)key2 ->next;
		
	// phase
	int ph = phase[key0->tick->idx];

	real_t t0   = key0->var_time->val;
	real_t tau  = key0->var_duration->val;  //< phase duration
	real_t t1   = t0 + tau;
	real_t dt   = std::max(t - t0, 0.0);    //< elapsed time since phase change
	real_t s    = dt/tau;                   //< normalized time
	vec3_t p0   = key0->var_foot_pos_t[side]->val;
	vec3_t p1   = key1->var_foot_pos_t[side]->val;
	real_t yaw0 = key0->var_foot_pos_r[side]->val;
	real_t yaw1 = key1->var_foot_pos_r[side]->val;
	real_t h0   = param.swingHeight[0];
	real_t h1   = param.swingHeight[1];
			
	if (param.swingProfile == SwingProfile::Wedge) {
		// double support
		if(ph == Phase::LR || ph == Phase::RL || ph == Phase::D){
			pos = p0;

			contact = ContactState::Surface;
		}
		// support foot of single support
		if( (ph == Phase::R && side == 0) ||
		    (ph == Phase::L && side == 1) ){
			pos = p0;
			
			contact = ContactState::Surface;
		}
		// swing foot of single support
		if( (ph == Phase::R && side == 1) ||
		    (ph == Phase::L && side == 0) ){
			pos.x = p0.x + (p1.x - p0.x)*s;
			pos.y = p0.y + (p1.y - p0.y)*s;
			vel.x = (p1.x - p0.x)/tau;
			vel.y = (p1.y - p0.y)/tau;

			if (dt < tau/2.0) {
				pos.z = p0.z + h0;
				vel.z = 0.0;
			}
			else{
				real_t a = dt/(tau/2.0) - 1.0;
				pos.z = (1 - a)*(p0.z + h0) + a*(p1.z + h1);
				vel.z = ((p1.z + h1) - (p0.z + h0))/(tau/2.0);
			}

			contact = ContactState::Float;
		}
	}
	if (param.swingProfile == SwingProfile::Cycloid){
		// double support
		if(ph == Phase::LR || ph == Phase::RL || ph == Phase::D){
			pos = p0;

			contact = ContactState::Surface;
		}
		// support foot of single support
		if( (ph == Phase::R && side == 0) ||
		    (ph == Phase::L && side == 1) ){
			pos = p0;

			contact = ContactState::Surface;
		}
		// swing foot of single support
		if( (ph == Phase::R && side == 1) ||
		    (ph == Phase::L && side == 0) ){
			real_t ch = (s - sin(_2pi*s)/_2pi);
			real_t cv = (1 - cos(_2pi*s))/2.0;
				
			pos = vec3_t(
				p0.x + (p1.x - p0.x)*ch,
				p0.y + (p1.y - p0.y)*ch,
				p0.z + (p1.z - p0.z)*ch + h0*cv
			);
			vel = vec3_t(
				((p1.x - p0.x)/tau)*(1.0 - cos(_2pi*s)),
				((p1.y - p0.y)/tau)*(1.0 - cos(_2pi*s)),
				((p1.z - p0.z)/tau)*(1.0 - cos(_2pi*s)) + (h0/(2.0*tau))*_2pi*sin(_2pi*s)
			);
			acc = vec3_t(
				((p1.x - p0.x)/(tau*tau))*_2pi*sin(_2pi*s),
				((p1.y - p0.y)/(tau*tau))*_2pi*sin(_2pi*s),
				((p1.z - p0.z)/(tau*tau))*_2pi*sin(_2pi*s) + (h0/(2.0*tau*tau))*(_2pi*_2pi)*cos(_2pi*s)
			);

			contact = ContactState::Float;
		}
	}
	if(param.swingProfile == SwingProfile::HeelToe){
		vec3_t c0 = key0->var_cop_pos->val;
		vec3_t c1 = key1->var_cop_pos->val;
		real_t ct = c0.x + (c1.x - c0.x)*s;
		real_t vc = (c1.x - c0.x)/tau;
		
		// support foot of single support phase
		if( (ph == Phase::R && side == 0) ||
			(ph == Phase::L && side == 1) ){

			FootRotation(p0.x, p0.z, ct, vc, pos, angle, vel, angvel, contact);

		}
		// swing foot of single support phase
		if( (ph == Phase::L && side == 0) ||
			(ph == Phase::R && side == 1) ){

			real_t cvm2   = 0.0;               //< cop velocity of previous single support phase
			real_t cv2    = 0.0;               //< cop velocity of next single support phase
			real_t c0     = 0.0;               //< cop position at lift-off
			real_t c1     = 0.0;               //< cop position at landing
			real_t lambda = 0.0;               //< offset of foot position in y axis
			vec3_t theta0;                     //< foot angle at lift-off
			vec3_t theta1;                     //< foot angle at landing
			vec3_t omega0;                     //< foot rotation speed at lift-off
			vec3_t omega1;                     //< foot rotation speed at landing
			vec3_t p00;
			vec3_t v00;
			vec3_t p11;
			vec3_t v11;
			int    con0;
			int    con1;

			if(keym2 && keym1) cvm2 = std::max(0.0, keym1->var_cop_pos->val.x - keym2->var_cop_pos->val.x) / (keym2->var_duration->val);
			if(key2  && key3 ) cv2  = std::max(0.0, key3 ->var_cop_pos->val.x - key2 ->var_cop_pos->val.x) / (key2 ->var_duration->val);
						
			if(keym1) c0 = std::max(p0.x, (keym1->var_cop_pos->val.x + cvm2*keym1->var_duration->val));
			if(key2 ) c1 = std::min(p1.x, (key2 ->var_cop_pos->val.x - cv2 *key1 ->var_duration->val));
			if(key0 ) lambda = std::min(std::abs(key0->var_foot_pos_t[!side]->val.y - p0.y), std::abs(key0->var_foot_pos_t[!side]->val.y - p1.y));

			FootRotation(p0.x, p0.z, c0, cvm2, p00, theta0, v00, omega0, con0);
			FootRotation(p1.x, p1.z, c1, cv2 , p11, theta1, v11, omega1, con1);
			
			// interpolate between endpoints with cubic polynomial
			pos.x    = InterpolatePos(t, t0, p00.x   , v00.x   , t1, p11.x   , v11.x   , Spr::Interpolate::Cubic);
			pos.z    = InterpolatePos(t, t0, p00.z   , v00.z   , t1, p11.z   , v11.z   , Spr::Interpolate::Cubic);
			angle.y  = InterpolatePos(t, t0, theta0.y, omega0.y, t1, theta1.y, omega1.y, Spr::Interpolate::Cubic);

			vel.x    = InterpolateVel(t, t0, p00.x   , v00.x   , t1, p11.x   , v11.x   , Spr::Interpolate::Cubic);
			vel.z    = InterpolateVel(t, t0, p00.z   , v00.z   , t1, p11.z   , v11.z   , Spr::Interpolate::Cubic);
			angvel.y = InterpolateVel(t, t0, theta0.y, omega0.y, t1, theta1.y, omega1.y, Spr::Interpolate::Cubic);

			acc.x    = InterpolateAcc(t, t0, p00.x   , v00.x   , t1, p11.x   , v11.x   , Spr::Interpolate::Cubic);
			acc.z    = InterpolateAcc(t, t0, p00.z   , v00.z   , t1, p11.z   , v11.z   , Spr::Interpolate::Cubic);
			angacc.y = InterpolateAcc(t, t0, theta0.y, omega0.y, t1, theta1.y, omega1.y, Spr::Interpolate::Cubic);

			// add cycloid movement to z to avoid scuffing the ground
			pos.z += h0*(1 - cos(_2pi*s))/2.0;
			vel.z += (h0/(2.0*tau))*_2pi*sin(_2pi*s);
			acc.z += (h0/(2.0*tau*tau))*(_2pi*_2pi)*cos(_2pi*s);

			// define movement in y to avoid scuffing support leg
			real_t avoid_y = (lambda < param.minSpacing) ? param.swingMargin + (param.minSpacing - lambda) : param.swingMargin;
			real_t sign = (ph == Phase::R ? 1.0 : -1.0);
			pos.y = p0.y + (p1.y - p0.y) * s + sign * avoid_y * (1 - cos(_2pi * s)) / 2.0;
			vel.y = (p1.y - p0.y) / tau + (sign * avoid_y / (2.0 * tau)) * _2pi * sin(_2pi * s);
			acc.y = (sign * avoid_y / (2.0 * tau * tau)) * (_2pi * _2pi) * cos(_2pi * s);
			
			contact = ContactState::Float;
		}
		// lifting-off foot of double support phase
		if ( (ph == Phase::RL && side == 0) ||
			 (ph == Phase::LR && side == 1) ){

			// cop velocity of previous single support phase
			real_t cv = 0.0;
			if(keym1) cv = std::max(0.0, key0->var_cop_pos->val.x - keym1->var_cop_pos->val.x) / (keym1->var_duration->val);

			real_t ct = c0.x + cv*dt;

			FootRotation(p0.x, p0.z, ct, cv, pos, angle, vel, angvel, contact);
		}
		// landed foot of double support phase
		if( (ph == Phase::LR && side == 0)||
			(ph == Phase::RL && side == 1) ){

			// cop velocity of next single support phase
			real_t cv = 0.0;
			if(key2) cv = std::max(0.0, key2->var_cop_pos->val.x - key1->var_cop_pos->val.x) / (key1->var_duration->val);

			real_t ct = c1.x - cv*(tau - dt);

			FootRotation(p0.x, p0.z, ct, cv, pos, angle, vel, angvel, contact);
		}
		// double support
		if(ph == Phase::D){
			pos = vec3_t(p0.x, p0.y, p0.z);
			contact = ContactState::Surface;
		}

		if ((ph != Phase::L || side == 1) &&
			(ph != Phase::R || side == 0)) {
			pos.y = p0.y + (p1.y - p0.y) * s;
			vel.y = (p1.y - p0.y) / tau;
			acc.y = 0;
		}
	}

	// yaw angle: linear interpolation
	angle [2] = (1.0-s)*yaw0 + s*yaw1;
	angvel[2] = (yaw1 - yaw0)/tau;
	angacc[2] = 0.0;

	pose.Pos() = pos;
	pose.Ori() = FromRollPitchYaw(angle);
}

real_t BipedLIP::TimeToLiftoff(real_t t, int side){
	BipedLIPKey* key = (BipedLIPKey*)traj.GetSegment(t).first;
	BipedLIPKey* keyNextFloat = key;

	int phase_float = (side == 0 ? Phase::L : Phase::R);
	while(keyNextFloat->next && phase[keyNextFloat->tick->idx] != phase_float)
		keyNextFloat = (BipedLIPKey*)keyNextFloat->next;
	
	// return 1.0 if there is no next float phase in the trajectory
	if(key == keyNextFloat && phase[key->tick->idx] != phase_float)
		return 1.0;

	return keyNextFloat->tick->time - t;
}

real_t BipedLIP::TimeToLanding(real_t t, int side){
	BipedLIPKey* key = (BipedLIPKey*)traj.GetSegment(t).first;
	BipedLIPKey* keyNextContact = key;

	int phase_float = (side == 0 ? Phase::L : Phase::R);
	while(keyNextContact->next && phase[keyNextContact->tick->idx] == phase_float)
		keyNextContact = (BipedLIPKey*)keyNextContact->next;

	// return 1.0 if there is no next contact phase in the trajectory
	if(key == keyNextContact && phase[key->tick->idx] == phase_float)
		return 1.0;

	return keyNextContact->tick->time - t;
}
		
vec3_t BipedLIP::CopPos(real_t t) {
	BipedLIPKey* key0 = (BipedLIPKey*)traj.GetSegment(t).first;
	BipedLIPKey* key1 = (BipedLIPKey*)traj.GetSegment(t).second;

	vec3_t ct;

	vec3_t c0  = key0->var_cop_pos->val;

	if (key1 == key0->next) {
		real_t dt  = t - key0->var_time->val;
		vec3_t cv0 = key0->var_cop_vel->val;

		ct = c0 + cv0*dt;
	}
	else {
		ct = c0;
	}

	return ct;
}

vec3_t BipedLIP::CmpPos(real_t t) {
	BipedLIPKey* key0 = (BipedLIPKey*)traj.GetSegment(t).first;
	BipedLIPKey* key1 = (BipedLIPKey*)traj.GetSegment(t).second;

	vec3_t cmt;

	vec3_t c0   = key0->var_cop_pos->val;
	vec3_t cm0  = key0->var_cmp_pos->val;

	if (key1 == key0->next) {
		real_t dt  = t - key0->var_time->val;
		vec3_t cv0  = key0->var_cop_vel->val;
		vec3_t cmv0 = key0->var_cmp_vel->val;
		
		cmt = (c0+cm0) + (cv0+cmv0)*dt;
	}
	else {
		cmt = (c0+cm0);
	}

	return cmt;
	//return vec3_t(cmt.x, cmt.y, 0.0);
}

vec3_t BipedLIP::TorsoPos(const vec3_t& pcom, const vec3_t& psup, const vec3_t& pswg) {
	real_t mt = param.torsoMass;
	real_t mf = param.footMass;
	vec3_t pt = ((mt + 2.0*mf)*pcom - mf * (psup + pswg)) / mt;
	return pt;
}

vec3_t BipedLIP::TorsoVel(const vec3_t& vcom, const vec3_t& vsup, const vec3_t& vswg) {
	real_t mt = param.torsoMass;
	real_t mf = param.footMass;
	vec3_t vt = ((mt + 2.0*mf)*vcom - mf * (vsup + vswg)) / mt;
	return vt;
}

vec3_t BipedLIP::TorsoAcc(const vec3_t& acom, const vec3_t& asup, const vec3_t& aswg) {
	real_t mt = param.torsoMass;
	real_t mf = param.footMass;
	vec3_t at = ((mt + 2.0*mf)*acom - mf * (asup + aswg)) / mt;
	return at;
}

//------------------------------------------------------------------------------------------------

void BipedLIP::Draw(Render::Canvas* canvas, Render::Config* conf) {
	TrajectoryNode::Draw(canvas, conf);

	if (!trajReady)
		CalcTrajectory();

	if (trajectory.empty())
		return;

	// com
	if (conf->Set(canvas, Render::Item::BipedCom, this)) {
		canvas->BeginLayer("biped_com", true);
		canvas->BeginPath();
		canvas->MoveTo(trajectory[0].com_pos);
		for (uint i = 1; i < trajectory.size(); i++) {
			canvas->LineTo(trajectory[i].com_pos);
		}
		canvas->EndPath();
		canvas->EndLayer();
	}

	// cop
	if (conf->Set(canvas, Render::Item::BipedCop, this)) {
		canvas->BeginLayer("biped_cop", true);
		canvas->BeginPath();
		canvas->MoveTo(trajectory[0].cop_pos);
		for (uint i = 1; i < trajectory.size(); i++) {
			canvas->LineTo(trajectory[i].cop_pos);
		}
		canvas->EndPath();
		canvas->EndLayer();
	}
	// cmp
	if (conf->Set(canvas, Render::Item::BipedCmp, this)) {
		canvas->BeginLayer("biped_cmp", true);
		canvas->BeginPath();
		canvas->MoveTo(trajectory[0].cmp_pos);
		for (uint i = 1; i < trajectory.size(); i++) {
			canvas->LineTo(trajectory[i].cmp_pos);
		}
		canvas->EndPath();
		canvas->EndLayer();
	}

	// torso
	if (conf->Set(canvas, Render::Item::BipedTorso, this)) {
		canvas->BeginLayer("biped_torso", true);
		canvas->BeginPath();
		canvas->MoveTo(trajectory[0].torso_pos_t);
		for (uint i = 1; i < trajectory.size(); i++) {
			canvas->LineTo(trajectory[i].torso_pos_t);
		}
		canvas->EndPath();
		canvas->EndLayer();
	}


	// swing foot
	if (conf->Set(canvas, Render::Item::BipedFoot, this)) {
		canvas->BeginLayer("biped_foot", true);
		canvas->BeginPath();
		canvas->MoveTo(trajectory[0].foot_pos_t[0]);
		for (uint i = 1; i < trajectory.size(); i++) {
			canvas->LineTo(trajectory[i].foot_pos_t[0]);
		}
		canvas->MoveTo(trajectory[0].foot_pos_t[1]);
		for (uint i = 1; i < trajectory.size(); i++) {
			canvas->LineTo(trajectory[i].foot_pos_t[1]);
		}
		canvas->EndPath();
		canvas->EndLayer();
	}
}

void BipedLIP::CreateSnapshot(real_t t, BipedLIP::Snapshot& s){
	pose_t pose;
	vec3_t vel, angvel, acc, angacc;
	int    contact;

	s.t = t;
	//s.com_pos = ComPos(t);
	ComState(t, s.com_pos, s.com_vel, s.com_acc);
	FootPose(t, 0, pose, vel, angvel, acc, angacc, contact);
	s.foot_pos_t[0] = pose.Pos();
	s.foot_pos_r[0] = pose.Ori();
	FootPose(t, 1, pose, vel, angvel, acc, angacc, contact);
	s.foot_pos_t[1] = pose.Pos();
	s.foot_pos_r[1] = pose.Ori();
	s.torso_pos_t = TorsoPos(s.com_pos, s.foot_pos_t[0], s.foot_pos_t[1]);
	s.torso_pos_r = TorsoOri(t);
	s.cop_pos = CopPos(t);
	s.cmp_pos = CmpPos(t);
}

void BipedLIP::CreateSnapshot(real_t t){
	CreateSnapshot(t, snapshot);
}

void BipedLIP::CalcTrajectory() {
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

void BipedLIP::DrawSnapshot(Render::Canvas* canvas, Render::Config* conf) {
	// lines connecting com and feet
	canvas->SetLineWidth(2.0f);
	canvas->BeginPath();
	for(int i = 0; i < 2; i++){
		canvas->MoveTo(snapshot.com_pos);
		canvas->LineTo(snapshot.foot_pos_t[i]);
	}
	canvas->EndPath();

	// foot outline
	vec3_t v[4];
	vec3_t p[4];
	v[0] = vec3_t(param.copMin.x, param.copMin.y, 0.0);
	v[1] = vec3_t(param.copMin.x, param.copMax.y, 0.0);
	v[2] = vec3_t(param.copMax.x, param.copMax.y, 0.0);
	v[3] = vec3_t(param.copMax.x, param.copMin.y, 0.0);
	for(int i = 0; i < 2; i++){
		for(int j = 0; j < 4; j++){
			p[j] = snapshot.foot_pos_t[i] + snapshot.foot_pos_r[i]*v[j];
		}
		canvas->SetLineWidth(2.0f);
		canvas->BeginPath();
		canvas->MoveTo(p[0]);
		canvas->LineTo(p[1]);
		canvas->LineTo(p[2]);
		canvas->LineTo(p[3]);
		canvas->LineTo(p[0]);
		canvas->EndPath();
	}
}

//-------------------------------------------------------------------------------------------------

// Constructors
BipedLipCon::BipedLipCon(Solver* solver, int _tag, string _name, BipedLIPKey* _obj, real_t _scale) :
	Constraint(solver, 3, ID(_tag, _obj->node, _obj->tick, _name), _scale) {
	obj[0] = _obj;
	obj[1] = (_obj->next ? (BipedLIPKey*)_obj->next : 0);
}

BipedLipPosCon::BipedLipPosCon(Solver* solver, string _name, BipedLIPKey* _obj, real_t _scale) :
	BipedLipCon(solver, ConTag::BipedLipPos, _name, _obj, _scale) {

	AddSLink (obj[1]->var_com_pos );
	AddSLink (obj[0]->var_com_pos );
	AddSLink (obj[0]->var_com_vel );
	AddSLink (obj[0]->var_cop_pos );
	AddSLink (obj[0]->var_cmp_pos );
	AddSLink (obj[0]->var_cop_vel );
	AddSLink (obj[0]->var_cmp_vel );
	AddC3Link(obj[0]->var_duration);
}

BipedLipVelCon::BipedLipVelCon(Solver* solver, string _name, BipedLIPKey* _obj, real_t _scale) :
	BipedLipCon(solver, ConTag::BipedLipVel, _name, _obj, _scale) {

	AddSLink (obj[1]->var_com_vel );
	AddSLink (obj[0]->var_com_pos );
	AddSLink (obj[0]->var_com_vel );
	AddSLink (obj[0]->var_cop_pos );
	AddSLink (obj[0]->var_cmp_pos );
	AddSLink (obj[0]->var_cop_vel );
	AddSLink (obj[0]->var_cmp_vel );
	AddC3Link(obj[0]->var_duration);
}

BipedLipCopCon::BipedLipCopCon(Solver* solver, string _name, BipedLIPKey* _obj, real_t _scale) :
	BipedLipCon(solver, ConTag::BipedLipCop, _name, _obj, _scale) {

	AddSLink (obj[1]->var_cop_pos );
	AddSLink (obj[0]->var_cop_pos );
	AddSLink (obj[0]->var_cop_vel );
	AddC3Link(obj[0]->var_duration);
}

BipedLipCmpCon::BipedLipCmpCon(Solver* solver, string _name, BipedLIPKey* _obj, real_t _scale) :
	BipedLipCon(solver, ConTag::BipedLipCmp, _name, _obj, _scale) {

	AddSLink (obj[1]->var_cmp_pos );
	AddSLink (obj[0]->var_cmp_pos );
	AddSLink (obj[0]->var_cmp_vel );
	AddC3Link(obj[0]->var_duration);
}

BipedLipMomCon::BipedLipMomCon(Solver* solver, string _name, BipedLIPKey* _obj, real_t _scale) :
	BipedLipCon(solver, ConTag::BipedLipMom, _name, _obj, _scale) {
	
	AddSLink (obj[1]->var_mom     );
	AddSLink (obj[0]->var_mom     );
	AddX3Link(obj[0]->var_cmp_pos );
	AddX3Link(obj[0]->var_cmp_vel );
	AddC3Link(obj[0]->var_duration);
}

BipedComConP::BipedComConP(Solver* solver, string _name, BipedLIPKey* _obj, real_t _scale) :
	Constraint(solver, 2, ID(ConTag::BipedComPos, _obj->node, _obj->tick, _name), _scale) {

	obj = _obj;

	AddSLink(obj->var_com_pos);
	AddSLink(obj->var_torso_pos_t);
	AddSLink(obj->var_foot_pos_t[0]);
	AddSLink(obj->var_foot_pos_t[1]);
}

BipedComConV::BipedComConV(Solver* solver, string _name, BipedLIPKey* _obj, real_t _scale) :
	Constraint(solver, 2, ID(ConTag::BipedComVel, _obj->node, _obj->tick, _name), _scale) {

	obj = _obj;

	AddSLink(obj->var_com_vel);
	AddSLink(obj->var_torso_vel_t);
}

BipedRangeCon::BipedRangeCon(Solver* solver, int _tag, string _name, BipedLIPKey* _obj, vec3_t _dir, real_t _scale) :
	Constraint(solver, 1, ID(_tag, _obj->node, _obj->tick, _name), _scale) {
	obj = _obj;
	dir = _dir;
}

BipedFootRangeConT::BipedFootRangeConT(Solver* solver, string _name, BipedLIPKey* _obj, uint _side, vec3_t _dir, real_t _scale) :
	BipedRangeCon(solver, ConTag::BipedFootRangeT, _name, _obj, _dir, _scale) {

	side = _side;

	AddR3Link(obj->var_foot_pos_t[side]);
	AddR3Link(obj->var_torso_pos_t);
	AddSLink (obj->var_torso_pos_r);
}

BipedFootRangeConR::BipedFootRangeConR(Solver* solver, string _name, BipedLIPKey* _obj, uint _side, real_t _scale) :
	BipedRangeCon(solver, ConTag::BipedFootRangeR, _name, _obj, vec3_t(), _scale) {

	side = _side;

	AddSLink(obj->var_foot_pos_r[side]);
	AddSLink(obj->var_torso_pos_r);
}

BipedCopRangeCon::BipedCopRangeCon(Solver* solver, string _name, BipedLIPKey* _obj, uint _side, vec3_t _dir, real_t _scale) :
	BipedRangeCon(solver, ConTag::BipedCopRange, _name, _obj, _dir, _scale) {

	side = _side;
	
	AddR3Link(obj->var_cop_pos);
	AddR3Link(obj->var_foot_pos_t[side]);
	AddSLink (obj->var_foot_pos_r[side]);
}

BipedCmpRangeCon::BipedCmpRangeCon(Solver* solver, string _name, BipedLIPKey* _obj, vec3_t _dir, real_t _scale) :
	BipedRangeCon(solver, ConTag::BipedCmpRange, _name, _obj, _dir, _scale) {

	AddR3Link(obj->var_cmp_pos);
	AddSLink (obj->var_torso_pos_r);
}

BipedAccRangeCon::BipedAccRangeCon(Solver* solver, string _name, BipedLIPKey* _obj, vec3_t _dir, real_t _scale) :
	BipedRangeCon(solver, ConTag::BipedAccRange, _name, _obj, _dir, _scale) {

	AddR3Link(obj->var_com_pos );
	AddR3Link(obj->var_cop_pos );
	AddR3Link(obj->var_cmp_pos );
	AddSLink (obj->var_torso_pos_r);
}

BipedMomRangeCon::BipedMomRangeCon(Solver* solver, string _name, BipedLIPKey* _obj, vec3_t _dir, real_t _scale) :
	BipedRangeCon(solver, ConTag::BipedMomRange, _name, _obj, _dir, _scale) {

	AddR3Link(obj->var_mom );
	AddSLink (obj->var_torso_pos_r);
}

BipedFootHeightCon::BipedFootHeightCon(Solver* solver, string _name, BipedLIPKey* _obj, uint _side, real_t _scale) :
	Constraint(solver, 1, ID(ConTag::BipedFootHeight, _obj->node, _obj->tick, _name), _scale) {

	obj  = _obj;
	side = _side;
	
	AddR3Link(obj->var_foot_pos_t[side]);
}

BipedTimeCon::BipedTimeCon(Solver* solver, string _name, BipedLIPKey* _obj, real_t _scale) :
	Constraint(solver, 1, ID(ConTag::BipedTime, _obj->node, _obj->tick, _name), _scale) {

	obj[0] = _obj;
	obj[1] = (BipedLIPKey*)_obj->next;

	AddSLink(obj[1]->var_time);
	AddSLink(obj[0]->var_time);
	AddSLink(obj[0]->var_duration);
}

//-------------------------------------------------------------------------------------------------

void BipedLipPosCon::CalcLhs() {
	Prepare();
	obj[1]->var_com_pos->val = (c0+cm0) + (cv0+cmv0)*tau + C*(p0 - (c0+cm0)) + (S*T)*(v0 - (cv0+cmv0));
}

void BipedLipVelCon::CalcLhs() {
	Prepare();
	obj[1]->var_com_vel->val = (cv0+cmv0) + (S/T)*(p0 - (c0+cm0)) + C*(v0 - (cv0+cmv0));
}

void BipedLipCopCon::CalcLhs() {
	Prepare();
	obj[1]->var_cop_pos->val = c0 + cv0*tau;
}

void BipedLipCmpCon::CalcLhs() {
	Prepare();
	obj[1]->var_cmp_pos->val = cm0 + cmv0*tau;
}

void BipedLipMomCon::CalcLhs(){
	Prepare();
	obj[1]->var_mom->val = L0 + ez % (cm0*tau + 0.5*tau*tau*cmv0);
}

void BipedTimeCon::CalcLhs(){
	obj[1]->var_time->val = obj[0]->var_time->val + obj[0]->var_duration->val;
}

//-------------------------------------------------------------------------------------------------

void BipedLipCon::Prepare() {
	BipedLIP::Param& param = ((BipedLIP*)obj[0]->node)->param;

	T    = param.T;
	ez   = vec3_t(0.0, 0.0, 1.0);
	L0   = obj[0]->var_mom    ->val;
	p0   = obj[0]->var_com_pos->val;
	v0   = obj[0]->var_com_vel->val;
	c0   = obj[0]->var_cop_pos->val + vec3_t(0.0, 0.0, param.comHeight);  //< c0 is vrp
	cm0  = obj[0]->var_cmp_pos->val;

	if(obj[1]){
		tau  = obj[0]->var_duration->val;
		C    = cosh(tau / T);
		S    = sinh(tau / T);

		cv0  = obj[0]->var_cop_vel->val;
		cmv0 = obj[0]->var_cmp_vel->val;

		L1   = obj[1]->var_mom    ->val;
		p1   = obj[1]->var_com_pos->val;
		v1   = obj[1]->var_com_vel->val;
		c1   = obj[1]->var_cop_pos->val + vec3_t(0.0, 0.0, param.comHeight);
		cm1  = obj[1]->var_cmp_pos->val;
	}
}

void BipedRangeCon::Prepare(){
	R       = mat3_t::Rot(theta, 'z');
	ez      = vec3_t(0.0, 0.0, 1.0);
	dir_abs = R*dir;
}

//-------------------------------------------------------------------------------------------------

void BipedLipPosCon::CalcCoef() {
	Prepare();

	((SLink *)links[0])->SetCoef( 1.0);
	((SLink *)links[1])->SetCoef(-C);
	((SLink *)links[2])->SetCoef(-T*S);
	((SLink *)links[3])->SetCoef(-1.0 + C);
	((SLink *)links[4])->SetCoef(-1.0 + C);
	((SLink *)links[5])->SetCoef(-tau + T*S);
	((SLink *)links[6])->SetCoef(-tau + T*S);
	((C3Link*)links[7])->SetCoef(-(cv0+cmv0) - (S/T)*(p0 - (c0+cm0)) - C*(v0 - (cv0+cmv0)));
}

void BipedLipVelCon::CalcCoef() {
	Prepare();

	((SLink *)links[0])->SetCoef( 1.0);
	((SLink *)links[1])->SetCoef(-S/T);
	((SLink *)links[2])->SetCoef(-C);
	((SLink *)links[3])->SetCoef( S/T);
	((SLink *)links[4])->SetCoef( S/T);
	((SLink *)links[5])->SetCoef(-1.0 + C);
	((SLink *)links[6])->SetCoef(-1.0 + C);
	((C3Link*)links[7])->SetCoef(-(C/(T*T))*(p0 - (c0+cm0)) - (S/T)*(v0 - (cv0+cmv0)));
}

void BipedLipCopCon::CalcCoef() {
	Prepare();

	((SLink *)links[0])->SetCoef( 1.0);
	((SLink *)links[1])->SetCoef(-1.0);
	((SLink *)links[2])->SetCoef(-tau);
	((C3Link*)links[3])->SetCoef(-cv0);
}

void BipedLipCmpCon::CalcCoef() {
	Prepare();

	((SLink *)links[0])->SetCoef( 1.0);
	((SLink *)links[1])->SetCoef(-1.0);
	((SLink *)links[2])->SetCoef(-tau);
	((C3Link*)links[3])->SetCoef(-cmv0);
}

void BipedLipMomCon::CalcCoef(){
	Prepare();

	((SLink *)links[0])->SetCoef( 1.0);
	((SLink *)links[1])->SetCoef(-1.0);
	((X3Link*)links[2])->SetCoef(-tau*ez);
	((X3Link*)links[3])->SetCoef(-(0.5*tau*tau)*ez);
	((C3Link*)links[4])->SetCoef(-(ez % (cm0 + cmv0*tau)));
}

void BipedComConP::CalcCoef() {
	BipedLIP::Param& param = ((BipedLIP*)obj->node)->param;

	real_t mt   = param.torsoMass;
	real_t mf   = param.footMass;
	real_t msum = mt + 2.0*mf;

	((SLink*)links[0])->SetCoef(1.0);
	((SLink*)links[1])->SetCoef(-mt/msum);
	((SLink*)links[2])->SetCoef(-mf/msum);
	((SLink*)links[3])->SetCoef(-mf/msum);

}

void BipedComConV::CalcCoef() {
	BipedLIP::Param& param = ((BipedLIP*)obj->node)->param;

	real_t mt   = param.torsoMass;
	real_t mf   = param.footMass;
	real_t msum = mt + 2.0*mf;

	((SLink*)links[0])->SetCoef(1.0);
	((SLink*)links[1])->SetCoef(-mt/msum);
}

void BipedFootRangeConT::CalcCoef() {
	r     = obj->var_foot_pos_t[side]->val - obj->var_torso_pos_t->val;
	theta = obj->var_torso_pos_r->val;

	Prepare();

	((R3Link*)links[0])->SetCoef( dir_abs);
	((R3Link*)links[1])->SetCoef(-dir_abs);
	((SLink* )links[2])->SetCoef( (ez % dir_abs)*r);
}

void BipedFootRangeConR::CalcCoef() {
	thetaf = obj->var_foot_pos_r[side]->val;
	thetat = obj->var_torso_pos_r->val;

	((SLink*)links[0])->SetCoef( 1.0);
	((SLink*)links[1])->SetCoef(-1.0);
}

void BipedCopRangeCon::CalcCoef() {
	r     = obj->var_cop_pos->val - obj->var_foot_pos_t[side]->val;
	theta = obj->var_foot_pos_r[side]->val;

	Prepare();
	
	((R3Link*)links[0])->SetCoef( dir_abs);
	((R3Link*)links[1])->SetCoef(-dir_abs);
	((SLink* )links[2])->SetCoef( (ez % dir_abs)*r);
}

void BipedCmpRangeCon::CalcCoef() {
	r     = obj->var_cmp_pos->val;
	theta = obj->var_torso_pos_r->val;
	
	Prepare();

	((R3Link*)links[0])->SetCoef(dir_abs);
	((SLink* )links[1])->SetCoef((ez % dir_abs)*r);
}

void BipedAccRangeCon::CalcCoef() {
	BipedLIP::Param& param = ((BipedLIP*)obj->node)->param;

	real_t T  = param.T;
	vec3_t p  = obj->var_com_pos->val;
	vec3_t c  = obj->var_cop_pos->val;
	vec3_t cm = obj->var_cmp_pos->val;
	r         = (1.0/(T*T))*(p - (c + vec3_t(0.0, 0.0, param.comHeight) + cm));
	theta     = obj->var_torso_pos_r->val;

	Prepare();
	
	((R3Link*)links[0])->SetCoef( dir_abs);
	((R3Link*)links[1])->SetCoef(-dir_abs);
	((R3Link*)links[2])->SetCoef(-dir_abs);
	((SLink *)links[3])->SetCoef( (ez % dir_abs)*r);
}

void BipedMomRangeCon::CalcCoef() {
	r      = obj->var_mom->val;
	theta  = obj->var_torso_pos_r->val;
	
	Prepare();

	((R3Link*)links[0])->SetCoef(dir_abs);
	((SLink *)links[1])->SetCoef((ez % dir_abs)*r);
}

void BipedFootHeightCon::CalcCoef(){
	((R3Link*)links[0])->SetCoef(vec3_t(0.0, 0.0, 1.0));
}

void BipedTimeCon::CalcCoef() {
	((SLink*)links[0])->SetCoef(1.0);
	((SLink*)links[1])->SetCoef(-1.0);
	((SLink*)links[2])->SetCoef(-1.0);
}

//-------------------------------------------------------------------------------------------------

void BipedLipPosCon::CalcDeviation() {
	y = p1 - ((c0+cm0) + (cv0+cmv0)*tau + C*(p0 - (c0+cm0)) + (T*S)*(v0 - (cv0+cmv0)));
}

void BipedLipVelCon::CalcDeviation() {
	y = v1 - ((cv0+cmv0) + (S/T)*(p0 - (c0+cm0)) + C*(v0 - (cv0+cmv0)));
}

void BipedLipCopCon::CalcDeviation() {
	y = c1 - (c0 + cv0*tau);
}

void BipedLipCmpCon::CalcDeviation() {
	y = cm1 - (cm0 + cmv0*tau);
}

void BipedLipMomCon::CalcDeviation(){
	y = L1 - (L0 + ez % (cm0*tau + 0.5*tau*tau*cmv0));
}

void BipedRangeCon::CalcDeviation(){
	real_t s = dir_abs*r;

	active = false;
	on_lower = (s < _min);
	on_upper = (s > _max);
	if (on_lower) {
		y[0] = (s - _min);
		active = true;
	}
	if (on_upper) {
		y[0] = (s - _max);
		active = true;
	}
}

void BipedFootRangeConR::CalcDeviation() {
	real_t s = thetaf - thetat;
	on_lower = (s < _min);
	on_upper = (s > _max);
	active = on_lower | on_upper;
	if (on_lower) y[0] = (s - _min);
	if (on_upper) y[0] = (s - _max);
}

void BipedFootHeightCon::CalcDeviation(){
	y[0] = obj->var_foot_pos_t[side]->val.z - ((BipedLIP*)obj->node)->elevation[obj->tick->idx];
}

//-------------------------------------------------------------------------------------------------

void BipedRangeCon::Project(real_t& l, uint k) {
	if ( on_upper && l > 0.0  ) l = 0.0;
	if ( on_lower && l < 0.0  ) l = 0.0;
	if (!on_upper && !on_lower) l = 0.0;
}

}
