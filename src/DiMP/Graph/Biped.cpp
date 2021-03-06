﻿#include <DiMP/Graph/Biped.h>
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

	// torso position and velocity
	var_torso_pos_t = new V2Var(solver, ID(VarTag::BipedTorsoTP, node, tick, name + "_torso_tp"), node->graph->scale.pos_t);
	var_torso_pos_r = new SVar (solver, ID(VarTag::BipedTorsoRP, node, tick, name + "_torso_rp"), node->graph->scale.pos_r);
	var_torso_vel_t = new V2Var(solver, ID(VarTag::BipedTorsoTV, node, tick, name + "_torso_tv"), node->graph->scale.vel_t);
	var_torso_vel_r = new SVar (solver, ID(VarTag::BipedTorsoRV, node, tick, name + "_torso_rv"), node->graph->scale.vel_r);
	solver->AddInputVar(var_torso_pos_t, tick->idx);
	solver->AddInputVar(var_torso_pos_r, tick->idx);
	solver->AddInputVar(var_torso_vel_t, tick->idx);
	solver->AddInputVar(var_torso_vel_r, tick->idx);
	var_torso_pos_t->weight = eps;
	var_torso_pos_r->weight = eps;
	var_torso_vel_t->weight = eps;
	var_torso_vel_r->weight = eps;

	// foot position
	var_foot_pos_t[0] = new V2Var(solver, ID(VarTag::BipedFootT, node, tick, name + "_foot_r_t"), node->graph->scale.pos_t);
	var_foot_pos_r[0] = new SVar (solver, ID(VarTag::BipedFootR, node, tick, name + "_foot_r_r"), node->graph->scale.pos_r);
	var_foot_pos_t[1] = new V2Var(solver, ID(VarTag::BipedFootT, node, tick, name + "_foot_l_t"), node->graph->scale.pos_t);
	var_foot_pos_r[1] = new SVar (solver, ID(VarTag::BipedFootR, node, tick, name + "_foot_l_r"), node->graph->scale.pos_r);
	solver->AddInputVar(var_foot_pos_t[0], tick->idx);
	solver->AddInputVar(var_foot_pos_r[0], tick->idx);
	solver->AddInputVar(var_foot_pos_t[1], tick->idx);
	solver->AddInputVar(var_foot_pos_r[1], tick->idx);
	var_foot_pos_t[0]->weight = eps;
	var_foot_pos_r[0]->weight = eps;
	var_foot_pos_t[1]->weight = eps;
	var_foot_pos_r[1]->weight = eps;

	// CoM position and velocity
	var_com_pos = new V2Var(solver, ID(VarTag::BipedComP, node, tick, name + "_com_p"), node->graph->scale.pos_t);
	var_com_vel = new V2Var(solver, ID(VarTag::BipedComV, node, tick, name + "_com_v"), node->graph->scale.vel_t);
	var_com_pos->weight = eps;
	var_com_vel->weight = eps;
	
	// angular momentum
	var_mom     = new V2Var(solver, ID(VarTag::BipedMom, node, tick, name + "_mom"), node->graph->scale.vel_r);  //< no proper matching scale for angular momentum...
	var_mom->weight = eps;

	// CoP position
	var_cop_pos = new V2Var(solver, ID(VarTag::BipedCopP, node, tick, name + "_cop_p"), node->graph->scale.pos_t);
	var_cop_pos->weight = eps;

	// CMP offset
	var_cmp_pos = new V2Var(solver, ID(VarTag::BipedCmpP, node, tick, name + "_cmp_p"), node->graph->scale.pos_t);
	var_cmp_pos->weight = eps;

	// absolute time
	var_time = new SVar(solver, ID(VarTag::BipedTime, node, tick, name + "_time"), node->graph->scale.time);
	var_time->weight = eps;

	// register state variables of ddp
	solver->AddStateVar(var_com_pos, tick->idx);
	solver->AddStateVar(var_com_vel, tick->idx);
	solver->AddStateVar(var_mom    , tick->idx);
	solver->AddStateVar(var_cop_pos, tick->idx);
	solver->AddStateVar(var_cmp_pos, tick->idx);
	solver->AddStateVar(var_time   , tick->idx);

	if(next){
		// CoP velocity
		var_cop_vel  = new V2Var(solver, ID(VarTag::BipedCopV, node, tick, name + "_cop_v"), node->graph->scale.vel_t);
		var_cop_vel->weight = eps;

		// CMP velocity
		var_cmp_vel  = new V2Var(solver, ID(VarTag::BipedCmpV, node, tick, name + "_cmp_v"), node->graph->scale.vel_t);
		var_cmp_vel->weight = eps;

		// step duration
		var_duration = new SVar(solver, ID(VarTag::BipedDuration, node, tick, name + "_duration"), node->graph->scale.time);
		var_duration->weight = eps;

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

	con_foot_t[0][0] = new BipedFootConT(solver, name + "_foot_range_r_t", this, 0, vec2_t(1.0, 0.0), node->graph->scale.pos_t);
	con_foot_t[0][1] = new BipedFootConT(solver, name + "_foot_range_r_t", this, 0, vec2_t(0.0, 1.0), node->graph->scale.pos_t);
	con_foot_r[0]    = new BipedFootConR(solver, name + "_foot_range_r_r", this, 0,                   node->graph->scale.pos_r);
	con_foot_t[1][0] = new BipedFootConT(solver, name + "_foot_range_l_t", this, 1, vec2_t(1.0, 0.0), node->graph->scale.pos_t);
	con_foot_t[1][1] = new BipedFootConT(solver, name + "_foot_range_l_t", this, 1, vec2_t(0.0, 1.0), node->graph->scale.pos_t);
	con_foot_r[1]    = new BipedFootConR(solver, name + "_foot_range_l_r", this, 1,                   node->graph->scale.pos_r);
	solver->AddCostCon(con_foot_t[0][0], tick->idx);
	solver->AddCostCon(con_foot_t[0][1], tick->idx);
	solver->AddCostCon(con_foot_r[0]   , tick->idx);
	solver->AddCostCon(con_foot_t[1][0], tick->idx);
	solver->AddCostCon(con_foot_t[1][1], tick->idx);
	solver->AddCostCon(con_foot_r[1]   , tick->idx);
	
	int phase = obj->phase[tick->idx];
	int side;
	if (phase == BipedLIP::Phase::R || phase == BipedLIP::Phase::RL)
		 side = 0;
	else side = 1;
	con_cop[0] = new BipedCopCon(solver, name + "_cop", this, side, vec2_t(1.0, 0.0), node->graph->scale.pos_t);
	con_cop[1] = new BipedCopCon(solver, name + "_cop", this, side, vec2_t(0.0, 1.0), node->graph->scale.pos_t);
	solver->AddCostCon(con_cop[0], tick->idx);
	solver->AddCostCon(con_cop[1], tick->idx);
	
	if (next) {
		con_duration = new RangeConS(solver, ID(ConTag::BipedDuration, node, tick, name + "_duration"), var_duration, node->graph->scale.time);
		solver->AddCostCon(con_duration, tick->idx);

		con_time = new BipedTimeCon(solver, name + "_time", this, node->graph->scale.time);
		solver->AddTransitionCon(con_time, tick->idx);
	}

	if (next) {
		con_foot_match_t[0] = new MatchConV2(solver, ID(ConTag::BipedFootMatchT, node, tick, name + "_foot_match_r_t"), var_foot_pos_t[0], nextObj->var_foot_pos_t[0], node->graph->scale.pos_t);
		con_foot_match_r[0] = new MatchConS (solver, ID(ConTag::BipedFootMatchR, node, tick, name + "_foot_match_r_r"), var_foot_pos_r[0], nextObj->var_foot_pos_r[0], node->graph->scale.pos_r);
		con_foot_match_t[1] = new MatchConV2(solver, ID(ConTag::BipedFootMatchT, node, tick, name + "_foot_match_l_t"), var_foot_pos_t[1], nextObj->var_foot_pos_t[1], node->graph->scale.pos_t);
		con_foot_match_r[1] = new MatchConS (solver, ID(ConTag::BipedFootMatchR, node, tick, name + "_foot_match_l_r"), var_foot_pos_r[1], nextObj->var_foot_pos_r[1], node->graph->scale.pos_r);
		solver->AddCostCon(con_foot_match_t[0], tick->idx);
		solver->AddCostCon(con_foot_match_r[0], tick->idx);
		solver->AddCostCon(con_foot_match_t[1], tick->idx);
		solver->AddCostCon(con_foot_match_r[1], tick->idx);
	}

	con_com_pos = new BipedComConP(solver, name + "_com_p", this, node->graph->scale.pos_t);
	con_com_vel = new BipedComConV(solver, name + "_com_v", this, node->graph->scale.vel_t);
	solver->AddCostCon(con_com_pos, tick->idx);
	solver->AddCostCon(con_com_vel, tick->idx);

	con_lip_acc = new BipedLipAccCon(solver, name + "_com_acc", this, node->graph->scale.pos_t);
	solver->AddCostCon(con_lip_acc, tick->idx);

	con_mom_zero = new FixConV2(solver, ID(ConTag::BipedLipMom, node, tick, name + "_mom_zero"), var_cmp_pos, node->graph->scale.pos_t);
	con_mom_zero->desired.clear();
}

void BipedLIPKey::Prepare() {
	BipedLIP* obj = (BipedLIP*)node;

}

void BipedLIPKey::Finish(){
	BipedLIP* obj = (BipedLIP*)node;
	
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
	pcom.z = (float)((BipedLIP*)node)->param.heightCoM;
	canvas->Point(pcom);

	// feet
	Vec2f cmin = obj->param.copPosMin; //CoPMinmum
	Vec2f cmax = obj->param.copPosMax; //CoPMax

	for (uint i = 0; i < 2; i++) {
		pf[i].x = (float)var_foot_pos_t[i]->val.x;
		pf[i].y = (float)var_foot_pos_t[i]->val.y;
		pf[i].z = 0.0f;
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
	pcop.z = 0.0f;
	canvas->Point(pcop);

}

//-------------------------------------------------------------------------------------------------
// BipedLIP

BipedLIP::Param::Param() {
	gravity        = 9.8;
	heightCoM      = 0.496;
	heightlow      = 0.45;
	heighthigh     = 0.51;
	heightmiddle   = (heightlow + heighthigh)/2;
	torsoMass      = 4.432*0.8;
	footMass       = (4.432 - torsoMass) / 2;
	swingProfile   = SwingProfile::Cycloid;
	swingHeight[0] = 0.1; //0: maximum swing foot height
	swingHeight[1] = 0.1; //1: swing foot height before landing (Wedge only)
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

	// allowable range of foot position relative to com
	footPosMin[0] = vec2_t(-0.2, -0.14);
	footPosMax[0] = vec2_t(0.2, -0.07);
	footPosMin[1] = vec2_t(-0.2, 0.07);
	footPosMax[1] = vec2_t(0.2, 0.14);
	footOriMin[0] = -Rad(15.0);
	footOriMax[0] = Rad(15.0);
	footOriMin[1] = -Rad(15.0);
	footOriMax[1] = Rad(15.0);

	copPosMin    = vec2_t(-0.1, -0.05 );
	copPosMax    = vec2_t( 0.1,  0.05 );

	angAccMax = 0.3;
	turnMax   = 0.5;

	ankleToToe  = 0.07;
	ankleToHeel = 0.07;
	toeRadius   = 0.03;
	heelRadius  = 0.03;

	accWeight = 0.0;
	momWeight = 0.0;
}

//-------------------------------------------------------------------------------------------------
// Waypoints
BipedLIP::Waypoint::Waypoint() {
	k             = 0;
	time          = 0.0;
	torso_pos_t   = vec2_t();
	torso_pos_r   = 0.0;
	torso_vel_t   = vec2_t();
	torso_vel_r   = 0.0;
	foot_pos_t[0] = vec2_t();
	foot_pos_r[0] = 0.0;
	foot_pos_t[1] = vec2_t();
	foot_pos_r[1] = 0.0;
	cop_pos       = vec2_t();

	fix_torso_pos_t   = false;
	fix_torso_pos_r   = false;
	fix_torso_vel_t   = false;
	fix_torso_vel_r   = false;
	fix_foot_pos_t[0] = false;
	fix_foot_pos_r[0] = false;
	fix_foot_pos_t[1] = false;
	fix_foot_pos_r[1] = false;
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
	param.T = sqrt(param.heightCoM / param.gravity);

	real_t durationAve[Phase::Num];
	for (int i = 0; i < Phase::Num; i++)
		durationAve[i] = (param.durationMin[i] + param.durationMax[i]) / 2.0;

	real_t t = 0.0;
	for (uint k = 0; k < graph->ticks.size(); k++) {
		BipedLIPKey* key = (BipedLIPKey*)traj.GetKeypoint(graph->ticks[k]);

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
		}

		// initial value of step duration is set as the average of minimum and maximum
		if (key->next) {
			key->var_duration->val = durationAve[phase[k]];
			key->con_duration->_min = param.durationMin[phase[k]];
			key->con_duration->_max = param.durationMax[phase[k]];
			
			//key->var_duration->locked = true;
		}

		/*
		// angular acceleration limit
		key->con_com_r[0]->_min = -param.angAccMax;
		key->con_com_r[0]->_max =  param.angAccMax;
		key->con_com_r[1]->_min = -param.angAccMax;
		key->con_com_r[1]->_max =  param.angAccMax;
		*/

		// range limit of foot position
		for (int i = 0; i < 2; i++) {
			key->con_foot_t[i][0]->_min = param.footPosMin[i][0];
			key->con_foot_t[i][1]->_min = param.footPosMin[i][1];
			key->con_foot_t[i][0]->_max = param.footPosMax[i][0];
			key->con_foot_t[i][1]->_max = param.footPosMax[i][1];
			key->con_foot_r[i]   ->_min = param.footOriMin[i];
			key->con_foot_r[i]   ->_max = param.footOriMax[i];
		}

		key->con_cop[0]->_min = param.copPosMin[0];
		key->con_cop[1]->_min = param.copPosMin[1];
		key->con_cop[0]->_max = param.copPosMax[0];
		key->con_cop[1]->_max = param.copPosMax[1];

		// cop is unconstrained for D phase to enable it to be inside the convex hull of both feet
		if(ph == BipedLIP::Phase::D){
			key->con_cop[0]->enabled = false;
			key->con_cop[1]->enabled = false;
		}

		// set weight of com acceleration constraint
		key->con_lip_acc->weight = param.accWeight;
		if(param.accWeight == 0.0)
			key->con_lip_acc->enabled = false;

		// set weight of cmp variable
		key->con_mom_zero->weight = param.momWeight;
		if(param.momWeight == 0.0)
			key->con_mom_zero->enabled = false;
		
		t += durationAve[phase[k]];
	}

	// initial value of com position, com velocity, and footsteps are calculated from a spline curve passing the waypoints
	Curve2d curve_torso_t, curve_foot_t[2];
	Curved  curve_torso_r, curve_foot_r[2];

	curve_torso_t  .SetType(Interpolate::Cubic);
	curve_torso_r  .SetType(Interpolate::Cubic);
	curve_foot_t[0].SetType(Interpolate::Cubic);
	curve_foot_r[0].SetType(Interpolate::Cubic);
	curve_foot_t[1].SetType(Interpolate::Cubic);
	curve_foot_r[1].SetType(Interpolate::Cubic);

	for (uint i = 0; i < waypoints.size(); i++) {
		Waypoint& wp = waypoints[i];
		BipedLIPKey* key = (BipedLIPKey*)traj.GetKeypoint(graph->ticks[wp.k]);
		real_t t = key->var_time->val;

		curve_torso_t.AddPoint(t);
		curve_torso_t.SetPos(i, wp.torso_pos_t);
		curve_torso_t.SetVel(i, wp.torso_vel_t);

		curve_torso_r.AddPoint(t);
		curve_torso_r.SetPos(i, wp.torso_pos_r);
		curve_torso_r.SetVel(i, wp.torso_vel_r);

		for (uint j = 0; j < 2; j++) {
			curve_foot_t[j].AddPoint(t);
			curve_foot_t[j].SetPos(i, wp.foot_pos_t[j]);
			curve_foot_t[j].SetVel(i, vec2_t());

			curve_foot_r[j].AddPoint(t);
			curve_foot_r[j].SetPos(i, wp.foot_pos_r[j]);
			curve_foot_r[j].SetVel(i, 0.0);
		}
	}

	for (uint k = 0; k < graph->ticks.size(); k++) {
		BipedLIPKey* key = (BipedLIPKey*)traj.GetKeypoint(graph->ticks[k]);
		real_t t = key->var_time->val;

		key->var_torso_pos_t->val = curve_torso_t.CalcPos(t);
		key->var_torso_pos_r->val = curve_torso_r.CalcPos(t);

		key->var_torso_vel_t->val = curve_torso_t.CalcVel(t);
		key->var_torso_vel_r->val = curve_torso_r.CalcVel(t);

		for (uint j = 0; j < 2; j++) {
			key->var_foot_pos_t[j]->val = curve_foot_t[j].CalcPos(t);
			key->var_foot_pos_r[j]->val = curve_foot_r[j].CalcPos(t);
		}

		real_t mt = param.torsoMass;
		real_t mf = param.footMass;
		key->var_com_pos->val = (mt * key->var_torso_pos_t->val + mf * (key->var_foot_pos_t[0]->val + key->var_foot_pos_t[1]->val)) / (mt + 2.0*mf);
		key->var_com_vel->val = (mt * key->var_torso_vel_t->val) / (mt + 2.0*mf);

		// angular momentum is initialized to zero
		key->var_mom->val.clear();

		// cmp is initialized to zero
		key->var_cmp_pos->val.clear();
		
		// cop is initialized to be at the center of the support region
		vec2_t c = (param.copPosMin + param.copPosMax)/2.0;
		if (phase[k] == Phase::R || phase[k] == Phase::RL)
			 key->var_cop_pos->val = key->var_foot_pos_t[0]->val + mat2_t::Rot(key->var_foot_pos_r[0]->val)*c;
		else key->var_cop_pos->val = key->var_foot_pos_t[1]->val + mat2_t::Rot(key->var_foot_pos_r[1]->val)*c;
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

		key->var_torso_pos_t->locked = wp.fix_torso_pos_t;
		key->var_torso_pos_r->locked = wp.fix_torso_pos_r;
		key->var_torso_vel_t->locked = wp.fix_torso_vel_t;
		key->var_torso_vel_r->locked = wp.fix_torso_vel_r;
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

vec3_t BipedLIP::ComPos(real_t t) {
	BipedLIPKey* key0 = (BipedLIPKey*)traj.GetSegment(t).first;
	BipedLIPKey* key1 = (BipedLIPKey*)traj.GetSegment(t).second;

	vec2_t pt;

	if(key1 == key0->next){
		real_t T    = param.T;
		real_t dt   = t - key0->var_time->val;
		vec2_t p0   = key0->var_com_pos->val;
		vec2_t v0   = key0->var_com_vel->val;
		vec2_t c0   = key0->var_cop_pos->val;
		vec2_t cm0  = key0->var_cmp_pos->val;
		vec2_t cv0  = key0->var_cop_vel->val;
		vec2_t cmv0 = key0->var_cmp_vel->val;

		pt = (c0+cm0) + (cv0+cmv0)*dt + cosh(dt/T)*(p0 - (c0+cm0)) + T*sinh(dt/T)*(v0 - (cv0+cmv0));
	}
	else{
		pt = key0->var_com_pos->val;
	}
	
	return vec3_t(pt.x, pt.y, param.heightCoM);
}

vec3_t BipedLIP::ComVel(real_t t) {
	BipedLIPKey* key0 = (BipedLIPKey*)traj.GetSegment(t).first;
	BipedLIPKey* key1 = (BipedLIPKey*)traj.GetSegment(t).second;

	vec2_t vt;

	if(key1 == key0->next){
		real_t T    = param.T;
		real_t dt   = t - key0->var_time->val;
		vec2_t p0   = key0->var_com_pos->val;
		vec2_t v0   = key0->var_com_vel->val;
		vec2_t c0   = key0->var_cop_pos->val;
		vec2_t cm0  = key0->var_cmp_pos->val;
		vec2_t cv0  = key0->var_cop_vel->val;
		vec2_t cmv0 = key0->var_cmp_vel->val;
		
		vt = (cv0+cmv0) + (1/T)*sinh(dt/T)*(p0 - (c0+cm0)) + cosh(dt/T)*(v0 - (cv0+cmv0));
	}
	else{
		vt = key0->var_com_vel->val;
	}
	
	return vec3_t(vt.x, vt.y, 0.0);
}

vec3_t BipedLIP::ComAcc(real_t t) {
	BipedLIPKey* key0 = (BipedLIPKey*)traj.GetSegment(t).first;
	BipedLIPKey* key1 = (BipedLIPKey*)traj.GetSegment(t).second;

	vec2_t at;

	if(key1 == key0->next){
		real_t T    = param.T;
		real_t T2   = T * T;
		real_t dt   = t - key0->var_time->val;
		vec2_t p0   = key0->var_com_pos->val;
		vec2_t v0   = key0->var_com_vel->val;
		vec2_t c0   = key0->var_cop_pos->val;
		vec2_t cm0  = key0->var_cmp_pos->val;
		vec2_t cv0  = key0->var_cop_vel->val;
		vec2_t cmv0 = key0->var_cmp_vel->val;
	
		at = (1/T2)*cosh(dt/T)*(p0 - (c0+cm0)) + (1/T)*sinh(dt/T)*(v0 - (cv0+cmv0));
	}
	else{
		at = vec2_t();
	}
	
	return vec3_t(at.x, at.y, 0.0);
}

vec3_t BipedLIP::Momentum(real_t t){
	BipedLIPKey* key0 = (BipedLIPKey*)traj.GetSegment(t).first;
	BipedLIPKey* key1 = (BipedLIPKey*)traj.GetSegment(t).second;

	vec2_t Lt;

	real_t m = param.torsoMass + 2.0*param.footMass;
	real_t g = param.gravity;

	if(key1 == key0->next){
		real_t dt   = t - key0->var_time->val;
		vec2_t L0   = key0->var_mom    ->val;
		vec2_t cm0  = key0->var_cmp_pos->val;
		vec2_t cmv0 = key0->var_cmp_vel->val;
	
		Lt.x = L0.x - (cm0.y*dt + 0.5*dt*dt*cmv0.y);
		Lt.y = L0.y + (cm0.x*dt + 0.5*dt*dt*cmv0.x);
	}
	else{
		Lt = vec2_t();
	}
	
	// momentum is internally normalized, so multiply by m*g to obtain actual momentum
	return (m*g)*vec3_t(Lt.x, Lt.y, 0.0);
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
			t0,
			key0->var_torso_pos_r->val,
			key0->var_torso_vel_r->val,
			t1,
			key1->var_torso_pos_r->val,
			key1->var_torso_vel_r->val,
			Interpolate::Cubic);
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
			t0,
			key0->var_torso_pos_r->val,
			key0->var_torso_vel_r->val,
			t1,
			key1->var_torso_pos_r->val,
			key1->var_torso_vel_r->val,
			Interpolate::Cubic);
	}
	else {
		r = key0->var_torso_vel_r->val;
	}

	return r;
}

real_t BipedLIP::TorsoAngAcc(real_t t) {
	BipedLIPKey* key0 = (BipedLIPKey*)traj.GetSegment(t).first;
	BipedLIPKey* key1 = (BipedLIPKey*)traj.GetSegment(t).second;

	real_t a;

	if (key1 == key0->next) {
		real_t t0 = key0->var_time->val;
		real_t t1 = key1->var_time->val;

		a = InterpolateAcc(
			t,
			t0,
			key0->var_torso_pos_r->val,
			key0->var_torso_vel_r->val,
			t1,
			key1->var_torso_pos_r->val,
			key1->var_torso_vel_r->val,
			Interpolate::Cubic);
	}
	else {
		a = 0.0;
	}

	return a;
}

pose_t BipedLIP::FootPose(real_t t, int side) {
	BipedLIPKey* keym2 = 0;
	BipedLIPKey* keym1 = 0;
	BipedLIPKey* key0 = (BipedLIPKey*)traj.GetSegment(t).first;
	BipedLIPKey* key1 = (BipedLIPKey*)traj.GetSegment(t).second;
	BipedLIPKey* key2  = 0;
	BipedLIPKey* key3  = 0;

	pose_t pose;
	vec2_t pos2;         //< pos x, y
	real_t z     = 0.0;  //< pos z
	real_t pitch = 0.0;  //< pitch angle
	real_t yaw   = 0.0;

	if(key1 != key0->next){
		pos2 = key0->var_foot_pos_t[side]->val;
		pose.Pos() = vec3_t(pos2.x, pos2.y, z);
		pose.Ori() = FromRollPitchYaw(vec3_t(0.0, 0.0, key0->var_foot_pos_r[side]->val));
		return pose;
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
	real_t dt   = t - t0;                   //< elapsed time since phase change
	real_t s    = dt/tau;                   //< normalized time
	vec2_t p0   = key0->var_foot_pos_t[side]->val;
	vec2_t p1   = key1->var_foot_pos_t[side]->val;
	real_t yaw0 = key0->var_foot_pos_r[side]->val;
	real_t yaw1 = key1->var_foot_pos_r[side]->val;
			
	if (param.swingProfile == SwingProfile::Wedge) {
		// double support
		if(ph == Phase::LR || ph == Phase::RL || ph == Phase::D){
			pos2 = key0->var_foot_pos_t[side]->val;
		}
		// support foot of single support
		if( (ph == Phase::R && side == 0) ||
		    (ph == Phase::L && side == 1) ){
			pos2 = key0->var_foot_pos_t[side]->val;
		}
		// swing foot of single support
		if( (ph == Phase::R && side == 1) ||
		    (ph == Phase::L && side == 0) ){
			if (dt < tau/2.0) {
				z = param.swingHeight[0];
			}
			else{
				real_t a = dt/(tau/2.0) - 1.0;
				z = (1 - a)*param.swingHeight[0] + a*param.swingHeight[1];
			}
			pos2 = p0 + (p1 - p0)*s;
		}
		pose.Pos() = vec3_t(pos2.x, pos2.y, z);
	}
	if (param.swingProfile == SwingProfile::Cycloid){
		// double support
		if(ph == Phase::LR || ph == Phase::RL || ph == Phase::D){
			pos2 = key0->var_foot_pos_t[side]->val;
		}
		// support foot of single support
		if( (ph == Phase::R && side == 0) ||
		    (ph == Phase::L && side == 1) ){
			pos2 = key0->var_foot_pos_t[side]->val;
		}
		// swing foot of single support
		if( (ph == Phase::R && side == 1) ||
		    (ph == Phase::L && side == 0) ){
			real_t ch = (s - sin(_2pi*s)/_2pi);
			real_t cv = (1 - cos(_2pi*s))/2.0;
				
			pos2 = p0 + (p1 - p0)*ch;
			z    = param.swingHeight[0]*cv;
		}
		pose.Pos() = vec3_t(pos2.x, pos2.y, z);
	}
	if(param.swingProfile == SwingProfile::HeelToe){
		real_t l0 = param.ankleToToe ;
		real_t l1 = param.ankleToHeel;
		real_t r0 = param.toeRadius  ;
		real_t r1 = param.heelRadius ;

		vec2_t c0 = key0->var_cop_pos->val;
		vec2_t c1 = key1->var_cop_pos->val;
		real_t ct = c0.x + (c1.x - c0.x)*s;
		
		// support foot of single support phase
		if( (ph == Phase::R && side == 0) ||
			(ph == Phase::L && side == 1) ){
			
			// current cop is on heel
			if(ct < p0.x - l1){
				pitch  = (ct - (p0.x - l1))/r1;
				pos2.x = p0.x + r1*(pitch - sin(pitch)) - l1*(1-cos(pitch));
				z      =        r1*(1.0   - cos(pitch)) - l1*sin(pitch);
			}
			// current cop is on toe
			else if(ct > p0.x + l0){
				pitch  = (ct - (p0.x + l0))/r0;
				pos2.x = p0.x + r0*(pitch - sin(pitch)) + l0*(1 - cos(pitch));
				z      =        r0*(1.0   - cos(pitch)) + l0*sin(pitch);
			}
			// current cop is in the middle
			else{
				pitch  = 0.0;
				pos2.x = p0.x;
				z      = 0.0;
			}
		}
		// swing foot of single support phase
		if( (ph == Phase::L && side == 0) ||
			(ph == Phase::R && side == 1) ){

			real_t cvm2   = 0.0;  //< cop velocity of previous single support phase
			real_t cv2    = 0.0;  //< cop velocity of next single support phase
			real_t theta0 = 0.0;  //< foot angle at lift-off
			real_t theta1 = 0.0;  //< foot angle at landing
			real_t omega0 = 0.0;  //< foot rotation speed at lift-off
			real_t omega1 = 0.0;  //< foot rotation speed at landing

			if(keym2 && keym1) cvm2 = std::max(0.0, keym1->var_cop_pos->val.x - keym2->var_cop_pos->val.x) / (keym2->var_duration->val);
			if(key2  && key3 ) cv2  = std::max(0.0, key3 ->var_cop_pos->val.x - key2 ->var_cop_pos->val.x) / (key2 ->var_duration->val);
						
			if(keym1){
				theta0 = std::max(0.0, (keym1->var_cop_pos->val.x + cvm2*keym1->var_duration->val) - (p0.x + l0))/r0;
				omega0 = cvm2/r0;
			}
			if(key2 ){
				theta1 = std::min(0.0, (key2 ->var_cop_pos->val.x - cv2 *key1 ->var_duration->val) - (p1.x - l1))/r1;
				omega1 = cv2/r1;
			}

			// foot position at lift-off
			real_t p00x = p0.x + r0*(theta0 - sin( theta0)) + l0*(1.0 - cos(theta0));
			real_t p00z =        r0*(1.0 - cos(theta0))     + l0*sin(theta0);
			// foot velocity at lift-off
			real_t v00x = (r0*(1.0 - cos(theta0)) + l0*sin(theta0))*omega0;
			real_t v00z = (r0*sin(theta0)         + l0*cos(theta0))*omega0;

			// foot position at landing
			real_t p11x = p1.x + r1*(theta1 - sin(theta1)) - l1*(1.0 - cos(theta1));
			real_t p11z =        r1*(1.0 - cos(theta1))    - l1*sin(theta1);
			// foot velocity at landing
			real_t v11x = (r1*(1.0 - cos(theta1)) - l1*sin(theta1))*omega1;
			real_t v11z = (r1*sin(theta1)         - l1*cos(theta1))*omega1;

			// interpolate between endpoints with cubic polynomial
			pos2.x = InterpolatePos(t, t0, p00x  , v00x  , t1, p11x  , v11x  , Spr::Interpolate::Cubic);
			z      = InterpolatePos(t, t0, p00z  , v00z  , t1, p11z  , v11z  , Spr::Interpolate::Cubic);
			pitch  = InterpolatePos(t, t0, theta0, omega0, t1, theta1, omega1, Spr::Interpolate::Cubic);

			// add cycloid movement to z to avoid scuffing the ground
			z += param.swingHeight[0]*(1 - cos(_2pi*s))/2.0;
		
			//vec2_t r(p11x - p00x, p11z - p00z);
			//real_t a     = r.norm();
			//real_t alpha = atan2(r[1], r[0]);
			//
			//real_t ch = (a    )*(s   - sin(_2pi*s)/_2pi);
			//real_t cv = (a/5.0)*(1.0 - cos(_2pi*s))/2.0;
			//
			//theta  = theta0 + (theta1 - theta0)*s;
			//pos2.x = p00x + cos(alpha)*ch - sin(alpha)*cv;
			//z      = p00z + sin(alpha)*ch + cos(alpha)*cv;
		}
		// lifting-off foot of double support phase
		if ( (ph == Phase::RL && side == 0) ||
			 (ph == Phase::LR && side == 1) ){

			// cop velocity of previous single support phase
			real_t cv = 0.0;
			if(keym1)
				cv = std::max(0.0, key0->var_cop_pos->val.x - keym1->var_cop_pos->val.x) / (keym1->var_duration->val);

			pitch  = std::max(0.0, (c0.x + cv*dt) - (p0.x + l0))/r0;		
			pos2.x = p0.x + r0*(pitch - sin(pitch)) + l0*(1.0 - cos(pitch));
			z      =        r0*(1.0   - cos(pitch)) + l0*sin(pitch);
		}
		// landed foot of double support phase
		if( (ph == Phase::LR && side == 0)||
			(ph == Phase::RL && side == 1) ){

			// cop velocity of next single support phase
			real_t cv = 0.0;
			if(key2)
				cv = std::max(0.0, key2->var_cop_pos->val.x - key1->var_cop_pos->val.x) / (key1->var_duration->val);

			pitch  = std::min(0.0, (c1.x - cv*(tau - dt)) - (p0.x - l1))/r1;
			pos2.x = p0.x + r1*(pitch - sin(pitch)) - l1*(1.0 - cos(pitch));
			z      =        r1*(1.0   - cos(pitch)) - l1*sin(pitch);
		}
		// double support
		if(ph == Phase::D){
			pitch  = 0.0;
			pos2.x = p0.x;
			z      = 0.0;
		}

		pos2.y = p0.y + (p1.y - p0.y)*s;
		pose.Pos() = vec3_t(pos2.x, pos2.y, z);
	}

	// yaw angle: linear interpolation
	yaw = (1.0-s)*yaw0 + s*yaw1;

	pose.Ori() = FromRollPitchYaw(vec3_t(0.0, pitch, yaw));

	return pose;
}

void BipedLIP::FootVel(real_t t, int side, vec3_t& v, vec3_t& w){
	// calculate velocity by forward difference
	const real_t dt = 0.001;
	const real_t dtinv = 1.0/dt;

	pose_t P0 = FootPose(t     , side);
	pose_t P1 = FootPose(t + dt, side);

	v = (P1.Pos() - P0.Pos())*dtinv;
	
	quat_t dp = P0.Ori().Conjugated() * P1.Ori();
	vec3_t axis   = dp.Axis ();
	real_t theta  = dp.Theta();
	if(theta > pi)
		theta -= 2*pi;
	w = (P0.Ori() * (theta * axis))*dtinv;
}

vec3_t BipedLIP::CopPos(real_t t) {
	BipedLIPKey* key0 = (BipedLIPKey*)traj.GetSegment(t).first;
	BipedLIPKey* key1 = (BipedLIPKey*)traj.GetSegment(t).second;

	vec2_t ct;

	vec2_t c0  = key0->var_cop_pos->val;

	if (key1 == key0->next) {
		real_t dt  = t - key0->var_time->val;
		vec2_t cv0 = key0->var_cop_vel->val;

		ct = c0 + cv0*dt;
	}
	else {
		ct = c0;
	}

	return vec3_t(ct.x, ct.y, 0.0);
}

vec3_t BipedLIP::CmpPos(real_t t) {
	BipedLIPKey* key0 = (BipedLIPKey*)traj.GetSegment(t).first;
	BipedLIPKey* key1 = (BipedLIPKey*)traj.GetSegment(t).second;

	vec2_t cmt;

	vec2_t c0   = key0->var_cop_pos->val;
	vec2_t cm0  = key0->var_cmp_pos->val;

	if (key1 == key0->next) {
		real_t dt  = t - key0->var_time->val;
		vec2_t cv0  = key0->var_cop_vel->val;
		vec2_t cmv0 = key0->var_cmp_vel->val;
		
		cmt = (c0+cm0) + (cv0+cmv0)*dt;
	}
	else {
		cmt = (c0+cm0);
	}

	return vec3_t(cmt.x, cmt.y, 0.0);
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
		canvas->SetLineWidth(3.0f);
		canvas->SetLineColor("black");
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
		canvas->SetLineWidth(3.0f);
		canvas->SetLineColor("magenta");
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
		canvas->SetLineWidth(1.0f);
		canvas->SetLineColor("green");
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
		canvas->SetLineWidth(2.0f);
		canvas->SetLineColor("blue");
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
		canvas->SetLineWidth(1.0f);
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

	s.t = t;
	s.com_pos = ComPos(t);
	pose = FootPose(t, 0);
	s.foot_pos_t[0] = pose.Pos();
	s.foot_pos_r[0] = pose.Ori();
	pose = FootPose(t, 1);
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
	v[0] = vec3_t(param.copPosMin.x, param.copPosMin.y, 0.0);
	v[1] = vec3_t(param.copPosMin.x, param.copPosMax.y, 0.0);
	v[2] = vec3_t(param.copPosMax.x, param.copPosMax.y, 0.0);
	v[3] = vec3_t(param.copPosMax.x, param.copPosMin.y, 0.0);
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

void BipedLIP::Save(const char* filename) {
	FILE* file = fopen(filename, "w");

	real_t dt = 0.0001;
	real_t tf = traj.back()->tick->time;
	for (real_t t = 0.0; t <= tf; t += dt) {

		vec3_t com_p   = ComPos(t);
		vec3_t lfoot_p = FootPose(t, 1).Pos();
		vec3_t rfoot_p = FootPose(t, 0).Pos();
		vec3_t cop_p   = CopPos(t);

		fprintf(file, "%3.4lf,", t);
		fprintf(file, "%3.4lf, %3.4lf, %3.4lf,", com_p.x, com_p.y, com_p.z);
		fprintf(file, "%3.4lf, %3.4lf, %3.4lf, %3.4lf, %3.4lf, %3.4lf,", lfoot_p.x, lfoot_p.y, lfoot_p.z, rfoot_p.x, rfoot_p.y, rfoot_p.z);
		fprintf(file, "%3.4lf, %3.4lf\n", cop_p.x, cop_p.y);
	}
	fclose(file);
}

void BipedLIP::Print()
{
	real_t dt = 0.0001;
	real_t tf = traj.back()->tick->time;
	for (real_t t = 0.0; t <= tf; t += dt) {
		printf("%3.4lf,\n", t);
	}
}

//-------------------------------------------------------------------------------------------------

// Constructors
BipedLipCon::BipedLipCon(Solver* solver, int _tag, string _name, BipedLIPKey* _obj, real_t _scale) :
	Constraint(solver, 2, ID(_tag, _obj->node, _obj->tick, _name), _scale) {
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
	AddC2Link(obj[0]->var_duration);
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
	AddC2Link(obj[0]->var_duration);
}

BipedLipAccCon::BipedLipAccCon(Solver* solver, string _name, BipedLIPKey* _obj, real_t _scale) :
	BipedLipCon(solver, ConTag::BipedLipAcc, _name, _obj, _scale) {

	AddSLink (obj[0]->var_com_pos );
	AddSLink (obj[0]->var_cop_pos );
	AddSLink (obj[0]->var_cmp_pos );
}

BipedLipCopCon::BipedLipCopCon(Solver* solver, string _name, BipedLIPKey* _obj, real_t _scale) :
	BipedLipCon(solver, ConTag::BipedLipCop, _name, _obj, _scale) {

	AddSLink (obj[1]->var_cop_pos );
	AddSLink (obj[0]->var_cop_pos );
	AddSLink (obj[0]->var_cop_vel );
	AddC2Link(obj[0]->var_duration);
}

BipedLipCmpCon::BipedLipCmpCon(Solver* solver, string _name, BipedLIPKey* _obj, real_t _scale) :
	BipedLipCon(solver, ConTag::BipedLipCmp, _name, _obj, _scale) {

	AddSLink (obj[1]->var_cmp_pos );
	AddSLink (obj[0]->var_cmp_pos );
	AddSLink (obj[0]->var_cmp_vel );
	AddC2Link(obj[0]->var_duration);
}

BipedLipMomCon::BipedLipMomCon(Solver* solver, string _name, BipedLIPKey* _obj, real_t _scale) :
	BipedLipCon(solver, ConTag::BipedLipMom, _name, _obj, _scale) {
	
	AddSLink (obj[1]->var_mom     );
	AddSLink (obj[0]->var_mom     );
	AddM2Link(obj[0]->var_cmp_pos );
	AddM2Link(obj[0]->var_cmp_vel );
	AddC2Link(obj[0]->var_duration);
}

/*
CoMConR::CoMConR(Solver* solver, string _name, BipedLIPKey* _obj, uint _idx, real_t _scale) :
Constraint(solver, 1, ID(ConTag::BipedCoMR, _obj->node, _obj->tick, _name), _scale) {
obj[0] = _obj;
obj[1] = (BipedLIPKey*)_obj->next;
idx = _idx;

AddSLink(obj[0]->com_pos_r);
AddSLink(obj[0]->com_vel_r);
AddSLink(obj[0]->period   );
AddSLink(obj[1]->com_pos_r);
AddSLink(obj[1]->com_vel_r);
}
*/

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

BipedFootConT::BipedFootConT(Solver* solver, string _name, BipedLIPKey* _obj, uint _side, vec2_t _dir, real_t _scale) :
	Constraint(solver, 1, ID(ConTag::BipedFootRangeT, _obj->node, _obj->tick, _name), _scale) {

	obj = _obj;
	side = _side;
	dir  = _dir;

	AddR2Link(obj->var_foot_pos_t[side]);
	AddR2Link(obj->var_torso_pos_t);
	AddSLink (obj->var_torso_pos_r);
}

BipedFootConR::BipedFootConR(Solver* solver, string _name, BipedLIPKey* _obj, uint _side, real_t _scale) :
	Constraint(solver, 1, ID(ConTag::BipedFootRangeR, _obj->node, _obj->tick, _name), _scale) {

	obj = _obj;
	side = _side;

	AddSLink(obj->var_foot_pos_r[side]);
	AddSLink(obj->var_torso_pos_r);
}

BipedCopCon::BipedCopCon(Solver* solver, string _name, BipedLIPKey* _obj, uint _side, vec2_t _dir, real_t _scale) :
	Constraint(solver, 1, ID(ConTag::BipedCop, _obj->node, _obj->tick, _name), _scale) {

	obj  = _obj;
	side = _side;
	dir  = _dir;

	AddR2Link(obj->var_cop_pos);
	AddR2Link(obj->var_foot_pos_t[side]);
	AddSLink (obj->var_foot_pos_r[side]);
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
	obj[1]->var_mom->val = L0 + H*(cm0*tau + 0.5*tau*tau*cmv0);
}

//-------------------------------------------------------------------------------------------------

void BipedLipCon::Prepare() {
	BipedLIP::Param& param = ((BipedLIP*)obj[0]->node)->param;

	T    = param.T;
	H[0][0] =  0.0;  H[0][1] = -1.0;
	H[1][0] =  1.0;  H[1][1] =  0.0;
	L0   = obj[0]->var_mom    ->val;
	p0   = obj[0]->var_com_pos->val;
	v0   = obj[0]->var_com_vel->val;
	c0   = obj[0]->var_cop_pos->val;
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
		c1   = obj[1]->var_cop_pos->val;
		cm1  = obj[1]->var_cmp_pos->val;
	}
}

void BipedLipPosCon::CalcCoef() {
	Prepare();

	((SLink *)links[0])->SetCoef( 1.0);
	((SLink *)links[1])->SetCoef(-C);
	((SLink *)links[2])->SetCoef(-T*S);
	((SLink *)links[3])->SetCoef(-1.0 + C);
	((SLink *)links[4])->SetCoef(-1.0 + C);
	((SLink *)links[5])->SetCoef(-tau + T*S);
	((SLink *)links[6])->SetCoef(-tau + T*S);
	((C2Link*)links[7])->SetCoef(-(cv0+cmv0) - (S/T)*(p0 - (c0+cm0)) - C*(v0 - (cv0+cmv0)));
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
	((C2Link*)links[7])->SetCoef(-(C/(T*T))*(p0 - (c0+cm0)) - (S/T)*(v0 - (cv0+cmv0)));
}

void BipedLipAccCon::CalcCoef() {
	Prepare();

	((SLink *)links[0])->SetCoef( 1.0);
	((SLink *)links[1])->SetCoef(-1.0);
	((SLink *)links[2])->SetCoef(-1.0);
}

void BipedLipCopCon::CalcCoef() {
	Prepare();

	((SLink *)links[0])->SetCoef( 1.0);
	((SLink *)links[1])->SetCoef(-1.0);
	((SLink *)links[2])->SetCoef(-tau);
	((C2Link*)links[3])->SetCoef(-cv0);
}

void BipedLipCmpCon::CalcCoef() {
	Prepare();

	((SLink *)links[0])->SetCoef( 1.0);
	((SLink *)links[1])->SetCoef(-1.0);
	((SLink *)links[2])->SetCoef(-tau);
	((C2Link*)links[3])->SetCoef(-cmv0);
}

void BipedLipMomCon::CalcCoef(){
	Prepare();

	((SLink *)links[0])->SetCoef( 1.0);
	((SLink *)links[1])->SetCoef(-1.0);
	((M2Link*)links[2])->SetCoef(tau*H);
	((M2Link*)links[3])->SetCoef((0.5*tau*tau)*H);
	((C2Link*)links[4])->SetCoef((H*(cm0 + cmv0*tau)));
}

/*
void CoMConR::CalcCoef() {
q0 = obj[0]->com_pos_r->val;
w0 = obj[0]->com_vel_r->val;
q1 = obj[1]->com_pos_r->val;
w1 = obj[1]->com_vel_r->val;
tau = obj[0]->period->val;
tau2 = tau*tau;
tau3 = tau*tau2;

if (idx == 0) {
((SLink*)links[0])->SetCoef(-6.0 / tau2);
((SLink*)links[1])->SetCoef(-4.0 / tau);
((SLink*)links[2])->SetCoef(-12.0*(q1 - q0) / tau3 + (4.0*w0 + 2.0*w1) / tau2);
((SLink*)links[3])->SetCoef(6.0 / tau2);
((SLink*)links[4])->SetCoef(-2.0 / tau);
}
else {
((SLink*)links[0])->SetCoef(6.0 / tau2);
((SLink*)links[1])->SetCoef(2.0 / tau);
((SLink*)links[2])->SetCoef(12.0*(q1 - q0) / tau3 - (2.0*w0 + 4.0*w1) / tau2);
((SLink*)links[3])->SetCoef(-6.0 / tau2);
((SLink*)links[4])->SetCoef(4.0 / tau);
}
}
*/

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

void BipedFootConT::CalcCoef() {
	pf = obj->var_foot_pos_t[side]->val;
	pt = obj->var_torso_pos_t->val;
	thetat = obj->var_torso_pos_r->val;
	R       = mat2_t::Rot(thetat);
	dR      = mat2_t::Rot(thetat + pi/2);
	dir_abs = R*dir;

	((R2Link*)links[0])->SetCoef( dir_abs);
	((R2Link*)links[1])->SetCoef(-dir_abs);
	((SLink* )links[2])->SetCoef( dir*(dR.trans()*(pf - pt)));
}

void BipedFootConR::CalcCoef() {
	thetaf = obj->var_foot_pos_r[side]->val;
	thetat = obj->var_torso_pos_r->val;

	((SLink*)links[0])->SetCoef(1.0);
	((SLink*)links[1])->SetCoef(-1.0);
}

void BipedCopCon::CalcCoef() {
	pc = obj->var_cop_pos->val;
	pf = obj->var_foot_pos_t[side]->val;
	thetaf = obj->var_foot_pos_r[side]->val;
	R       = mat2_t::Rot(thetaf);
	dR      = mat2_t::Rot(thetaf + pi/2);
	dir_abs = R*dir;

	((R2Link*)links[0])->SetCoef( dir_abs);
	((R2Link*)links[1])->SetCoef(-dir_abs);
	((SLink* )links[2])->SetCoef( dir*(dR.trans()*(pc - pf)));
}

void BipedTimeCon::CalcCoef() {
	((SLink*)links[0])->SetCoef(1.0);
	((SLink*)links[1])->SetCoef(-1.0);
	((SLink*)links[2])->SetCoef(-1.0);
}

void BipedTimeCon::CalcLhs(){
	obj[1]->var_time->val = obj[0]->var_time->val + obj[0]->var_duration->val;
}

//-------------------------------------------------------------------------------------------------

void BipedLipPosCon::CalcDeviation() {
	vec2_t y2 = p1 - ((c0+cm0) + (cv0+cmv0)*tau + C*(p0 - (c0+cm0)) + (T*S)*(v0 - (cv0+cmv0)));
	y[0] = y2[0];
	y[1] = y2[1];
}

void BipedLipVelCon::CalcDeviation() {
	vec2_t y2 = v1 - ((cv0+cmv0) + (S/T)*(p0 - (c0+cm0)) + C*(v0 - (cv0+cmv0)));
	y[0] = y2[0];
	y[1] = y2[1];
}

void BipedLipAccCon::CalcDeviation() {
	vec2_t y2 = p0 - (c0+cm0);
	y[0] = y2[0];
	y[1] = y2[1];
}

void BipedLipCopCon::CalcDeviation() {
	vec2_t y2 = c1 - (c0 + cv0*tau);
	y[0] = y2[0];
	y[1] = y2[1];
}

void BipedLipCmpCon::CalcDeviation() {
	vec2_t y2 = cm1 - (cm0 + cmv0*tau);
	y[0] = y2[0];
	y[1] = y2[1];
}

void BipedLipMomCon::CalcDeviation(){
	vec2_t y2 = L1 - (L0 + H*(cm0*tau + 0.5*tau*tau*cmv0));
}

/*
void CoMConR::CalcDeviation() {
real_t s;
if (idx == 0)
s =  6.0*(q1 - q0) / tau2 - (4.0*w0 + 2.0*w1) / tau;
else s = -6.0*(q1 - q0) / tau2 + (2.0*w0 + 4.0*w1) / tau;
on_lower = (s < _min);
on_upper = (s > _max);
active = on_lower | on_upper;
if (on_lower) y[0] = (s - _min);
if (on_upper) y[0] = (s - _max);
}
*/

void BipedFootConT::CalcDeviation() {
	real_t s = dir*(R.trans()*(pf - pt));

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

void BipedFootConR::CalcDeviation() {
	real_t s = thetaf - thetat;
	on_lower = (s < _min);
	on_upper = (s > _max);
	active = on_lower | on_upper;
	if (on_lower) y[0] = (s - _min);
	if (on_upper) y[0] = (s - _max);
}

void BipedCopCon::CalcDeviation() {
	real_t s = dir*(R.trans()*(pc - pf));

	if(_min == _max){
		active = true;
		y[0] = s - _min;
	}
	else{
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
}

//-------------------------------------------------------------------------------------------------

void BipedFootConT::Project(real_t& l, uint k) {
	if ( on_upper && l > 0.0  ) l = 0.0;
	if ( on_lower && l < 0.0  ) l = 0.0;
	if (!on_upper && !on_lower) l = 0.0;
}

void BipedFootConR::Project(real_t& l, uint k) {
	if ( on_upper && l > 0.0  ) l = 0.0;
	if ( on_lower && l < 0.0  ) l = 0.0;
	if (!on_upper && !on_lower) l = 0.0;
}

void BipedCopCon::Project(real_t& l, uint k) {
	if ( on_upper && l > 0.0  ) l = 0.0;
	if ( on_lower && l < 0.0  ) l = 0.0;
	if (!on_upper && !on_lower) l = 0.0;
}

}
