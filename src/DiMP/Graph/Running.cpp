#include <DiMP/Graph/Running.h>
#include <DiMP/Graph/Graph.h>
#include <DiMP/Render/Config.h>
#include <DiMP/Render/Canvas.h>

#include<stdio.h>

#include <sbrollpitchyaw.h>

namespace DiMP {
	;

	const real_t   pi = M_PI;
	const real_t _2pi = 2.0 * pi;

	const real_t eps = 0.1;

	//-------------------------------------------------------------------------------------------------
	// BipedLIPKey

	BipedRunKey::BipedRunKey() {

	}

	// register variables for planning
	void BipedRunKey::AddVar(Solver* solver) {
		BipedRunning* obj = (BipedRunning*)node;

		vec3_t one(1.0, 1.0, 1.0);

		// torso position and velocity
		var_torso_pos_t = new V3Var(solver, ID(VarTag::BipedTorsoTP, node, tick, name + "_torso_tp"), node->graph->scale.pos_t);
		var_torso_pos_r = new SVar(solver, ID(VarTag::BipedTorsoRP, node, tick, name + "_torso_rp"), node->graph->scale.pos_r);
		var_torso_vel_t = new V3Var(solver, ID(VarTag::BipedTorsoTV, node, tick, name + "_torso_tv"), node->graph->scale.vel_t);
		solver->AddInputVar(var_torso_pos_t, tick->idx);
		solver->AddInputVar(var_torso_pos_r, tick->idx);
		solver->AddInputVar(var_torso_vel_t, tick->idx);
		var_torso_pos_t->weight = eps * one;
		var_torso_pos_r->weight[0] = eps;
		var_torso_vel_t->weight = eps * one;

		// foot position
		var_foot_pos_t[0] = new V3Var(solver, ID(VarTag::BipedFootTP, node, tick, name + "_foot_r_t"), node->graph->scale.pos_t);
		var_foot_pos_r[0] = new SVar(solver, ID(VarTag::BipedFootRP, node, tick, name + "_foot_r_r"), node->graph->scale.pos_r);
		var_foot_pos_t[1] = new V3Var(solver, ID(VarTag::BipedFootTP, node, tick, name + "_foot_l_t"), node->graph->scale.pos_t);
		var_foot_pos_r[1] = new SVar(solver, ID(VarTag::BipedFootRP, node, tick, name + "_foot_l_r"), node->graph->scale.pos_r);
		solver->AddInputVar(var_foot_pos_t[0], tick->idx);
		solver->AddInputVar(var_foot_pos_r[0], tick->idx);
		solver->AddInputVar(var_foot_pos_t[1], tick->idx);
		solver->AddInputVar(var_foot_pos_r[1], tick->idx);
		var_foot_pos_t[0]->weight = eps * one;
		var_foot_pos_r[0]->weight[0] = eps;
		var_foot_pos_t[1]->weight = eps * one;
		var_foot_pos_r[1]->weight[0] = eps;

		// CoM position and velocity
		var_com_pos = new V3Var(solver, ID(VarTag::BipedComP, node, tick, name + "_com_p"), node->graph->scale.pos_t);
		var_com_vel = new V3Var(solver, ID(VarTag::BipedComV, node, tick, name + "_com_v"), node->graph->scale.vel_t);
		var_com_pos->weight = eps * one;
		var_com_vel->weight = eps * one;

		// CoP position
		var_cop_pos = new V3Var(solver, ID(VarTag::BipedFootCopP, node, tick, name + "_cop_p"), node->graph->scale.pos_t);
		var_cop_pos->weight = eps * one;

		// CMP offset
		var_cmp_pos = new V3Var(solver, ID(VarTag::CentroidTP, node, tick, name + "_cmp_p"), node->graph->scale.pos_t);
		var_cmp_pos->weight = eps * one;

		// absolute time
		var_time = new SVar(solver, ID(VarTag::BipedTime, node, tick, name + "_time"), node->graph->scale.time);
		var_time->weight[0] = eps;

		// register state variables of ddp
		solver->AddStateVar(var_com_pos, tick->idx);
		solver->AddStateVar(var_com_vel, tick->idx);
		solver->AddStateVar(var_mom, tick->idx);
		solver->AddStateVar(var_cop_pos, tick->idx);
		solver->AddStateVar(var_cmp_pos, tick->idx);
		solver->AddStateVar(var_time, tick->idx);

		if (next) {
			// CoP velocity
			var_cop_vel = new V3Var(solver, ID(VarTag::BipedFootCopV, node, tick, name + "_cop_v"), node->graph->scale.vel_t);
			var_cop_vel->weight = eps * one;

			// CMP velocity
			var_cmp_vel = new V3Var(solver, ID(VarTag::CentroidTV, node, tick, name + "_cmp_v"), node->graph->scale.vel_t);
			var_cmp_vel->weight = eps * one;

			// step duration
			var_duration = new SVar(solver, ID(VarTag::BipedDuration, node, tick, name + "_duration"), node->graph->scale.time);
			var_duration->weight[0] = eps;

			// register input variables of ddp
			solver->AddInputVar(var_cop_vel, tick->idx);
			solver->AddInputVar(var_cmp_vel, tick->idx);
			solver->AddInputVar(var_duration, tick->idx);
		}
	}

	// register constraints for planning
	void BipedRunKey::AddCon(Solver* solver) {
		BipedRunning* obj = (BipedRunning*)node;
		BipedRunKey* nextObj = (BipedRunKey*)next;

		if (next) {
			con_lip_pos = new RunnerLipPosCon(solver, name + "_lip_pos", this, node->graph->scale.pos_t);
			con_lip_vel = new RunnerLipVelCon(solver, name + "_lip_vel", this, node->graph->scale.vel_t);
			con_lip_cop = new RunnerLipCopCon(solver, name + "_lip_cop", this, node->graph->scale.pos_t);
			con_lip_cmp = new RunnerLipCmpCon(solver, name + "_lip_cmp", this, node->graph->scale.pos_t);
			//con_lip_mom = new RunnerLipMomCon(solver, name + "_lip_mom", this, node->graph->scale.vel_r);

			solver->AddTransitionCon(con_lip_pos, tick->idx);
			solver->AddTransitionCon(con_lip_vel, tick->idx);
			solver->AddTransitionCon(con_lip_cop, tick->idx);
			solver->AddTransitionCon(con_lip_cmp, tick->idx);
		}

		con_foot_range_t[0][0] = new RunnerFootRangeConT(solver, name + "_foot_range_r_t", this, 0, vec3_t(1.0, 0.0, 0.0), node->graph->scale.pos_t);
		con_foot_range_t[0][1] = new RunnerFootRangeConT(solver, name + "_foot_range_r_t", this, 0, vec3_t(0.0, 1.0, 0.0), node->graph->scale.pos_t);
		con_foot_range_t[0][2] = new RunnerFootRangeConT(solver, name + "_foot_range_r_t", this, 0, vec3_t(0.0, 0.0, 1.0), node->graph->scale.pos_t);
		con_foot_range_r[0] = new RunnerFootRangeConR(solver, name + "_foot_range_r_r", this, 0, node->graph->scale.pos_r);
		con_foot_range_t[1][0] = new RunnerFootRangeConT(solver, name + "_foot_range_l_t", this, 1, vec3_t(1.0, 0.0, 0.0), node->graph->scale.pos_t);
		con_foot_range_t[1][1] = new RunnerFootRangeConT(solver, name + "_foot_range_l_t", this, 1, vec3_t(0.0, 1.0, 0.0), node->graph->scale.pos_t);
		con_foot_range_t[1][2] = new RunnerFootRangeConT(solver, name + "_foot_range_l_t", this, 1, vec3_t(0.0, 0.0, 1.0), node->graph->scale.pos_t);
		con_foot_range_r[1] = new RunnerFootRangeConR(solver, name + "_foot_range_l_r", this, 1, node->graph->scale.pos_r);
		solver->AddCostCon(con_foot_range_t[0][0], tick->idx);
		solver->AddCostCon(con_foot_range_t[0][1], tick->idx);
		solver->AddCostCon(con_foot_range_t[0][2], tick->idx);
		solver->AddCostCon(con_foot_range_r[0], tick->idx);
		solver->AddCostCon(con_foot_range_t[1][0], tick->idx);
		solver->AddCostCon(con_foot_range_t[1][1], tick->idx);
		solver->AddCostCon(con_foot_range_t[1][2], tick->idx);
		solver->AddCostCon(con_foot_range_r[1], tick->idx);

		// cop range constraint
		int phase = obj->phase[tick->idx];
		int side;
		if (phase == BipedRunning::Phase::R || phase == BipedRunning::Phase::RL)
			side = 0;
		else side = 1;
		con_cop_range[0] = new RunnerCopRangeCon(solver, name + "_cop_range", this, side, vec3_t(1.0, 0.0, 0.0), node->graph->scale.pos_t);
		con_cop_range[1] = new RunnerCopRangeCon(solver, name + "_cop_range", this, side, vec3_t(0.0, 1.0, 0.0), node->graph->scale.pos_t);
		con_cop_range[2] = new RunnerCopRangeCon(solver, name + "_cop_range", this, side, vec3_t(0.0, 0.0, 1.0), node->graph->scale.pos_t);
		solver->AddCostCon(con_cop_range[0], tick->idx);
		solver->AddCostCon(con_cop_range[1], tick->idx);
		solver->AddCostCon(con_cop_range[2], tick->idx);

		// cmp range constraint
		con_cmp_range[0] = new RunnerCmpRangeCon(solver, name + "_cmp_range", this, vec3_t(1.0, 0.0, 0.0), node->graph->scale.pos_t);
		con_cmp_range[1] = new RunnerCmpRangeCon(solver, name + "_cmp_range", this, vec3_t(0.0, 1.0, 0.0), node->graph->scale.pos_t);
		con_cmp_range[2] = new RunnerCmpRangeCon(solver, name + "_cmp_range", this, vec3_t(0.0, 0.0, 1.0), node->graph->scale.pos_t);
		solver->AddCostCon(con_cmp_range[0], tick->idx);
		solver->AddCostCon(con_cmp_range[1], tick->idx);
		solver->AddCostCon(con_cmp_range[2], tick->idx);

		// acc range constraint
		con_acc_range[0] = new RunnerAccRangeCon(solver, name + "_acc_range", this, vec3_t(1.0, 0.0, 0.0), node->graph->scale.acc_t);
		con_acc_range[1] = new RunnerAccRangeCon(solver, name + "_acc_range", this, vec3_t(0.0, 1.0, 0.0), node->graph->scale.acc_t);
		con_acc_range[2] = new RunnerAccRangeCon(solver, name + "_acc_range", this, vec3_t(0.0, 0.0, 1.0), node->graph->scale.acc_t);
		solver->AddCostCon(con_acc_range[0], tick->idx);
		solver->AddCostCon(con_acc_range[1], tick->idx);
		solver->AddCostCon(con_acc_range[2], tick->idx);

		if (next) {
			con_duration_range = new RangeConS(solver, ID(ConTag::BipedDurationRange, node, tick, name + "_duration"), var_duration, node->graph->scale.time);
			solver->AddCostCon(con_duration_range, tick->idx);

			con_time = new RunnerTimeCon(solver, name + "_time", this, node->graph->scale.time);
			solver->AddTransitionCon(con_time, tick->idx);
		}

		if (next) {
			con_foot_match_t[0] = new MatchConV3(solver, ID(ConTag::BipedFootVelZeroT, node, tick, name + "_foot_match_r_t"), var_foot_pos_t[0], nextObj->var_foot_pos_t[0], node->graph->scale.pos_t);
			con_foot_match_r[0] = new MatchConS(solver, ID(ConTag::BipedFootVelZeroR, node, tick, name + "_foot_match_r_r"), var_foot_pos_r[0], nextObj->var_foot_pos_r[0], node->graph->scale.pos_r);
			con_foot_match_t[1] = new MatchConV3(solver, ID(ConTag::BipedFootVelZeroT, node, tick, name + "_foot_match_l_t"), var_foot_pos_t[1], nextObj->var_foot_pos_t[1], node->graph->scale.pos_t);
			con_foot_match_r[1] = new MatchConS(solver, ID(ConTag::BipedFootVelZeroR, node, tick, name + "_foot_match_l_r"), var_foot_pos_r[1], nextObj->var_foot_pos_r[1], node->graph->scale.pos_r);
			solver->AddCostCon(con_foot_match_t[0], tick->idx);
			solver->AddCostCon(con_foot_match_r[0], tick->idx);
			solver->AddCostCon(con_foot_match_t[1], tick->idx);
			solver->AddCostCon(con_foot_match_r[1], tick->idx);

			con_foot_height[0] = new RunnerFootHeightCon(solver, name + "_foot_height_r", this, 0, node->graph->scale.pos_t);
			con_foot_height[1] = new RunnerFootHeightCon(solver, name + "_foot_height_l", this, 1, node->graph->scale.pos_t);
			solver->AddCostCon(con_foot_height[0], tick->idx);
			solver->AddCostCon(con_foot_height[1], tick->idx);
		}

		con_com_pos = new RunnerComConP(solver, name + "_com_p", this, node->graph->scale.pos_t);
		con_com_vel = new RunnerComConV(solver, name + "_com_v", this, node->graph->scale.vel_t);
		solver->AddCostCon(con_com_pos, tick->idx);
		solver->AddCostCon(con_com_vel, tick->idx);

	}

	void BipedRunKey::Prepare() {

	}

	void BipedRunKey::Finish() {
		// tick's time is updated from time variable
		tick->time = var_time->val;
	}

	void BipedRunKey::Draw(Render::Canvas* canvas, Render::Config* conf) {
		BipedRunning* obj = (BipedRunning*)node;

		Vec3f pcom, pcop, pf[2], pt;
		float thetaf[2];

		canvas->SetPointSize(5.0f);
		canvas->SetLineWidth(1.0f);

		pcom.x = (float)var_com_pos->val.x;
		pcom.y = (float)var_com_pos->val.y;
		pcom.z = (float)var_com_pos->val.z;
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
		pcop.z = (float)(var_foot_pos_t[0]->val.z + var_foot_pos_t[0]->val.z) / 2.0f;
		canvas->Point(pcop);

	}

	//-------------------------------------------------------------------------------------------------
	// BipedLIP

	BipedRunning::Param::Param() {
		gravity = vec3_t(0.0, 0.0, 9.8);
		comHeight = 1.0;
		T[0] = sqrt(comHeight/gravity.z);
		T[1] = sqrt(comHeight/gravity.z);
		T[2] = sqrt(comHeight/gravity.z);
		torsoMass = 10.0;
		footMass = 5.0;
		swingProfile = SwingProfile::Cycloid;
		swingInterpolation = SwingInterpolation::Cubic;
		swingHeight[0] = 0.1; //0: maximum swing foot height
		swingHeight[1] = 0.1; //1: swing foot height before landing (Wedge only)
		durationMin[Phase::R] = 0.1; // duration minimum at single support
		durationMax[Phase::R] = 0.8; // duration maximum at single support
		durationMin[Phase::L] = 0.1;
		durationMax[Phase::L] = 0.8;
		durationMin[Phase::RL] = 0.1; // duration minimum at double support
		durationMax[Phase::RL] = 0.2; // duration maximum at double support
		durationMin[Phase::LR] = 0.1;
		durationMax[Phase::LR] = 0.2;
		durationMin[Phase::D] = 0.1;
		durationMax[Phase::D] = 0.2;

		// range of foot position relative to com
		footPosMin[0] = vec3_t(-0.5, -0.20, 0.0);
		footPosMax[0] = vec3_t(0.5, -0.00, 0.0);
		footPosMin[1] = vec3_t(-0.5, 0.00, 0.0);
		footPosMax[1] = vec3_t(0.5, 0.20, 0.0);
		footOriMin[0] = Rad(-15.0);
		footOriMax[0] = Rad(15.0);
		footOriMin[1] = Rad(-15.0);
		footOriMax[1] = Rad(15.0);

		// range of cop relative to support foot
		copMin = vec3_t(-0.1, -0.05, 0.0);
		copMax = vec3_t(0.1, 0.05, 0.0);

		// range of cmp offset
		cmpMin = vec3_t(-0.0, -0.0, 0.0);
		cmpMax = vec3_t(0.0, 0.0, 0.0);

		// range of com acceleration
		accMin = vec3_t(-1.0, 1.0, 0.0);
		accMax = vec3_t(-1.0, 1.0, 0.0);

		// range of angular momentum
		momMin = vec3_t(-1.0, 1.0, 0.0);
		momMax = vec3_t(-1.0, 1.0, 0.0);

	}

	//-------------------------------------------------------------------------------------------------
	// Waypoints
	BipedRunning::Waypoint::Waypoint() {
		k = 0;
		time = 0.0;
		com_pos = vec3_t();
		com_vel = vec3_t();
		torso_pos_r = 0.0;

		foot_pos_t[0] = vec3_t();
		foot_pos_r[0] = 0.0;
		foot_pos_t[1] = vec3_t();
		foot_pos_r[1] = 0.0;
		cop_pos = vec3_t();
		cop_min = vec3_t();
		cop_max = vec3_t();

		fix_com_pos = false;
		fix_com_vel = false;
		fix_torso_pos_r = false;

		fix_foot_pos_t[0] = false;
		fix_foot_pos_r[0] = false;
		fix_foot_pos_t[1] = false;
		fix_foot_pos_r[1] = false;
		fix_com_pos = false;
		fix_com_vel = false;
		fix_cop_pos = false;
		fix_cmp_pos = false;
		fix_mom = false;
		set_cop_range = false;
	}

	//-------------------------------------------------------------------------------------------------
	BipedRunning::Snapshot::Snapshot() {
		t = 0.0;
		torso_pos_t = vec3_t();
		torso_pos_r = 0.0;
		foot_pos_t[0] = vec3_t();
		foot_pos_r[0] = quat_t();
		foot_pos_t[1] = vec3_t();
		foot_pos_r[1] = quat_t();
	}

	//-------------------------------------------------------------------------------------------------

	BipedRunning::BipedRunning(Graph* g, string n) :TrajectoryNode(g, n) {
		type = Type::Object;
		graph->runners.Add(this);
	}

	BipedRunning::~BipedRunning() {
		graph->runners.Remove(this);
	}

	void BipedRunning::Init() {
		TrajectoryNode::Init();

		real_t durationAve[Phase::Num];
		for (int i = 0; i < Phase::Num; i++)
			durationAve[i] = (param.durationMin[i] + param.durationMax[i]) / 2.0;

		real_t t = 0.0;
		for (uint k = 0; k < graph->ticks.size(); k++) {
			BipedRunKey* key = (BipedRunKey*)traj.GetKeypoint(graph->ticks[k]);

			key->tick->time = t;
			key->var_time->val = t;

			int ph = phase[key->tick->idx];

			if (!key->prev) {
				// initial time is fixed
				key->var_time->val = 0.0;
				key->var_time->locked = true;
			}

			if (key->next) {
				key->con_foot_match_t[0]->enabled = (ph != BipedRunning::Phase::L);
				key->con_foot_match_r[0]->enabled = (ph != BipedRunning::Phase::L);
				key->con_foot_match_t[1]->enabled = (ph != BipedRunning::Phase::R);
				key->con_foot_match_r[1]->enabled = (ph != BipedRunning::Phase::R);

				key->con_foot_height[0]->enabled = (ph == BipedRunning::Phase::R || ph == BipedRunning::Phase::D);
				key->con_foot_height[1]->enabled = (ph == BipedRunning::Phase::L || ph == BipedRunning::Phase::D);
			}

			// initial value of step duration is set as the average of minimum and maximum
			if (key->next) {
				key->var_duration->val = durationAve[phase[k]];
				key->con_duration_range->_min = param.durationMin[phase[k]];
				key->con_duration_range->_max = param.durationMax[phase[k]];

				//key->var_duration->locked = true;
			}

			// range limit of foot position
			for (int i = 0; i < 2; i++) {
				for (int j = 0; j < 3; j++) {
					key->con_foot_range_t[i][j]->_min = param.footPosMin[i][j];
					key->con_foot_range_t[i][j]->_max = param.footPosMax[i][j];
				}
				key->con_foot_range_r[i]->_min = param.footOriMin[i];
				key->con_foot_range_r[i]->_max = param.footOriMax[i];
			}

			for (int j = 0; j < 3; j++) {
				// cop range: the sign of y range is flipped for left support
				if (j == 1) {
					key->con_cop_range[j]->_min = (ph == BipedRunning::Phase::L || ph == BipedRunning::Phase::LR ? param.copMin[j] : -param.copMax[j]);
					key->con_cop_range[j]->_max = (ph == BipedRunning::Phase::L || ph == BipedRunning::Phase::LR ? param.copMax[j] : -param.copMin[j]);
				}
				else {
					key->con_cop_range[j]->_min = param.copMin[j];
					key->con_cop_range[j]->_max = param.copMax[j];
				}

				// cmp range
				key->con_cmp_range[j]->_min = param.cmpMin[j];
				key->con_cmp_range[j]->_max = param.cmpMax[j];

				// acceleration range
				key->con_acc_range[j]->_min = param.accMin[j];
				key->con_acc_range[j]->_max = param.accMax[j];
			}

			// cop is unconstrained for D phase to enable it to be inside the convex hull of both feet
			if (ph == BipedRunning::Phase::D) {
				for (int j = 0; j < 3; j++)
					key->con_cop_range[j]->enabled = false;
			}

			t += durationAve[phase[k]];
		}

		// initial value of com position, com velocity, and footsteps are calculated from a spline curve passing the waypoints
		Curve3d curve_com;
		Curved  curve_torso_r;
		Curve3d curve_foot_t[2];
		Curved  curve_foot_r[2];

		curve_com.SetType(Interpolate::Cubic);
		curve_torso_r.SetType(Interpolate::LinearDiff);
		curve_foot_t[0].SetType(Interpolate::Cubic);
		curve_foot_r[0].SetType(Interpolate::Cubic);
		curve_foot_t[1].SetType(Interpolate::Cubic);
		curve_foot_r[1].SetType(Interpolate::Cubic);

		for (uint i = 0; i < waypoints.size(); i++) {
			Waypoint& wp = waypoints[i];
			BipedRunKey* key = (BipedRunKey*)traj.GetKeypoint(graph->ticks[wp.k]);
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
			BipedRunKey* key = (BipedRunKey*)traj.GetKeypoint(graph->ticks[k]);
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

			real_t mt = param.torsoMass;
			real_t mf = param.footMass;
			key->var_torso_pos_t->val = (1.0 + 2.0 * (mf / mt)) * key->var_com_pos->val - (mf / mt) * (key->var_foot_pos_t[0]->val + key->var_foot_pos_t[1]->val);
			key->var_torso_vel_t->val = (1.0 + 2.0 * (mf / mt)) * key->var_com_vel->val;
			//key->var_com_pos->val = (mt * key->var_torso_pos_t->val + mf * (key->var_foot_pos_t[0]->val + key->var_foot_pos_t[1]->val)) / (mt + 2.0*mf);
			//key->var_com_vel->val = (mt * key->var_torso_vel_t->val) / (mt + 2.0*mf);

			// cmp is initialized to zero
			key->var_cmp_pos->val.clear();
		}

		// initial value of cop and cmp velocity
		for (uint k = 0; k < graph->ticks.size(); k++) {
			BipedRunKey* key = (BipedRunKey*)traj.GetKeypoint(graph->ticks[k]);
			BipedRunKey* key1 = (BipedRunKey*)key->next;
			if (key1) {
				key->var_cop_vel->val = (key1->var_cop_pos->val - key->var_cop_pos->val) / key->var_duration->val;
				key->var_cmp_vel->val.clear();
			}
		}

		// variables at waypoints are fixed
		for (uint i = 0; i < waypoints.size(); i++) {
			Waypoint& wp = waypoints[i];
			BipedRunKey* key = (BipedRunKey*)traj.GetKeypoint(graph->ticks[wp.k]);

			key->var_com_pos->locked = wp.fix_com_pos;
			key->var_com_vel->locked = wp.fix_com_vel;
			key->var_torso_pos_r->locked = wp.fix_torso_pos_r;
			key->var_cop_pos->locked = wp.fix_cop_pos;
			key->var_cmp_pos->locked = wp.fix_cmp_pos;

			if (wp.fix_cop_pos) {
				key->var_cop_pos->val = wp.cop_pos;
			}

			// set cop range if specified (param setting is overridden)
			if (wp.set_cop_range) {
				for (int j = 0; j < 3; j++) {
					key->con_cop_range[j]->_min = wp.cop_min[j];
					key->con_cop_range[j]->_max = wp.cop_max[j];
				}
			}

			for (uint j = 0; j < 2; j++) {
				key->var_foot_pos_t[j]->locked = wp.fix_foot_pos_t[j];
				key->var_foot_pos_r[j]->locked = wp.fix_foot_pos_r[j];
			}
		}

		Prepare();
	}

	void BipedRunning::Prepare() {
		TrajectoryNode::Prepare();
		trajReady = false;
	}

	void BipedRunning::Finish() {
		TrajectoryNode::Finish();
	}

	int BipedRunning::Phase(real_t t) {
		BipedRunKey* key = (BipedRunKey*)traj.GetSegment(t).first;
		return phase[key->tick->idx];
	}

	void BipedRunning::ComState(real_t t, vec3_t& pos, vec3_t& vel, vec3_t& acc) {
		BipedRunKey* key0 = (BipedRunKey*)traj.GetSegment(t).first;
		BipedRunKey* key1 = (BipedRunKey*)traj.GetSegment(t).second;

		real_t dt = t - key0->var_time->val;
		//real_t T = param.T;
		real_t T;
		if (!key0->prev || !key1->next)
		{
			T = param.T[0]; //0.3316;
		}
		else if (!key0->prev->prev || !key1->next->next)
		{
			T = param.T[1]; //0.2659 0.2807;
		}
		else T = param.T[2];
		real_t T2 = T * T;

		if (key1 == key0->next) {
			vec3_t p0 = key0->var_com_pos->val;
			vec3_t v0 = key0->var_com_vel->val;
			vec3_t c0 = key0->var_cop_pos->val + vec3_t(0.0, 0.0, param.gravity.z * T * T);
			vec3_t cm0 = key0->var_cmp_pos->val;
			vec3_t cv0 = key0->var_cop_vel->val;
			vec3_t cmv0 = key0->var_cmp_vel->val;
			int ph = phase[key0->tick->idx];

			if (ph == Phase::LR || ph == Phase::RL) {
				pos = p0 + v0 * dt - 0.5 * param.gravity * dt * dt;
				vel = v0 - param.gravity * dt;
				acc = -param.gravity;
			}
			else {
				pos = (c0 + cm0) + (cv0 + cmv0) * dt + cosh(dt / T) * (p0 - (c0 + cm0)) + T * sinh(dt / T) * (v0 - (cv0 + cmv0));
				vel = (cv0 + cmv0) + (1 / T) * sinh(dt / T) * (p0 - (c0 + cm0)) + cosh(dt / T) * (v0 - (cv0 + cmv0));
				acc = (1 / T2) * cosh(dt / T) * (p0 - (c0 + cm0)) + (1 / T) * sinh(dt / T) * (v0 - (cv0 + cmv0));

			}
		}
		else {
			pos = key0->var_com_pos->val;
			vel = key0->var_com_vel->val;
			acc = vec3_t();
		}
	}

	vec3_t BipedRunning::Momentum(real_t t) {
		BipedRunKey* key0 = (BipedRunKey*)traj.GetSegment(t).first;
		BipedRunKey* key1 = (BipedRunKey*)traj.GetSegment(t).second;

		vec3_t Lt;

		real_t m = param.torsoMass + 2.0 * param.footMass;
		real_t g = param.gravity.z;

		if (key1 == key0->next) {
			real_t dt = t - key0->var_time->val;
			vec3_t L0 = key0->var_mom->val;
			vec3_t cm0 = key0->var_cmp_pos->val;
			vec3_t cmv0 = key0->var_cmp_vel->val;

			Lt = L0 + vec3_t(0.0, 0.0, 1.0) % (cm0 * dt + 0.5 * dt * dt * cmv0);
		}
		else {
			Lt = vec3_t();
		}

		// momentum is internally normalized, so multiply by m*g to obtain actual momentum
		return (m * g) * Lt;
	}

	real_t BipedRunning::TorsoOri(real_t t) {
		BipedRunKey* key0 = (BipedRunKey*)traj.GetSegment(t).first;
		BipedRunKey* key1 = (BipedRunKey*)traj.GetSegment(t).second;

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

	real_t BipedRunning::TorsoAngVel(real_t t) {
		BipedRunKey* key0 = (BipedRunKey*)traj.GetSegment(t).first;
		BipedRunKey* key1 = (BipedRunKey*)traj.GetSegment(t).second;

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

	real_t BipedRunning::TorsoAngAcc(real_t t) {
		return 0.0;
	}

	real_t fresnelC(real_t d) {
		return d - (1.0 / 10.0) * pow(d, 5.0);
	}

	real_t fresnelS(real_t d) {
		return (1.0 / 3.0) * pow(d, 3.0) - (1.0 / 42.0) * pow(d, 7.0);
	}

	real_t clothoid_x(real_t d, real_t kappa) {
		real_t tmp = sqrt(kappa / 2.0);
		return (1.0 / tmp) * fresnelC(tmp * d);
	}

	real_t clothoid_y(real_t d, real_t kappa) {
		real_t tmp = sqrt(kappa / 2.0);
		return (1.0 / tmp) * fresnelS(tmp * d);
	}


	// cubic or quintic interpolation
	void Interpolate(
		real_t t, real_t& p, real_t& v, real_t& a,
		real_t t0, real_t  p0, real_t  v0, real_t  a0,
		real_t t1, real_t  p1, real_t  v1, real_t  a1,
		int type)
	{
		real_t Kcubic[6][6] = {
			{ 1.0,  0.0, -3.0,  2.0,  0.0,  0.0},
			{ 0.0,  0.0,  3.0, -2.0,  0.0,  0.0},
			{ 0.0,  1.0, -2.0,  1.0,  0.0,  0.0},
			{ 0.0,  0.0, -1.0,  1.0,  0.0,  0.0},
			{ 0.0,  0.0,  0.0,  0.0,  0.0,  0.0},
			{ 0.0,  0.0,  0.0,  0.0,  0.0,  0.0}
		};
		real_t Kquintic[6][6] = {
			{ 1.0,  0.0,  0.0, -10.0,  15.0, -6.0},
			{ 0.0,  0.0,  0.0,  10.0, -15.0,  6.0},
			{ 0.0,  1.0,  0.0, -6.0,   8.0, -3.0},
			{ 0.0,  0.0,  0.0, -4.0,   7.0, -3.0},
			{ 0.0,  0.0,  0.5, -1.5,   1.5, -0.5},
			{ 0.0,  0.0,  0.0,   0.5, -1.0,  0.5}
		};

		real_t h = t1 - t0;
		real_t h2 = h * h;
		real_t s = (t - t0) / h;
		real_t s2 = s * s;
		real_t s3 = s * s2;
		real_t s4 = s * s3;
		real_t s5 = s * s4;

		real_t(*K)[6] = (type == BipedRunning::SwingInterpolation::Cubic ? Kcubic : Kquintic);

		vec6_t kp, kv, ka;
		for (int i = 0; i < 6; i++) {
			kp[i] = K[i][0] + K[i][1] * s + K[i][2] * s2 + K[i][3] * s3 + K[i][4] * s4 + K[i][5] * s5;
			kv[i] = K[i][1] + 2.0 * K[i][2] * s + 3.0 * K[i][3] * s2 + 4.0 * K[i][4] * s3 + 5.0 * K[i][5] * s4;
			ka[i] = 2.0 * K[i][2] + 6.0 * K[i][3] * s + 12.0 * K[i][4] * s2 + 20.0 * K[i][5] * s3;
		}

		p = kp[0] * p0 + kp[1] * p1 + kp[2] * h * v0 + kp[3] * h * v1 + kp[4] * h2 * a0 + kp[5] * h2 * a1;
		v = kv[0] / h * p0 + kv[1] / h * p1 + kv[2] * v0 + kv[3] * v1 + kv[4] * h * a0 + kv[5] * h * a1;
		a = ka[0] / h2 * p0 + ka[1] / h2 * p1 + ka[2] / h * v0 + ka[3] / h * v1 + ka[4] * a0 + ka[5] * a1;

	}

	// 4-th order or 6-th order spline
	// both ends are zero (up to 1st or 2nd derivative) and take pmid at midpoint
	void Interpolate2(real_t t, real_t& p, real_t& v, real_t& a, real_t t0, real_t t1, real_t pmid, int type) {
		real_t K6[7] = { 0.0, 0.0, 0.0,  1.0, -3.0, 3.0, -1.0 };
		real_t K4[7] = { 0.0, 0.0, 1.0, -2.0,  1.0, 0.0,  0.0 };

		real_t h = t1 - t0;
		real_t h2 = h * h;
		real_t s = (t - t0) / h;
		real_t s2 = s * s;
		real_t s3 = s * s2;
		real_t s4 = s * s3;
		real_t s5 = s * s4;
		real_t s6 = s * s5;

		real_t* K = (type == BipedRunning::SwingInterpolation::Cubic ? K4 : K6);

		real_t kp, kv, ka;
		kp = K[0] + K[1] * s + K[2] * s2 + K[3] * s3 + K[4] * s4 + K[5] * s5 + K[6] * s6;
		kv = K[1] + 2.0 * K[2] * s + 3.0 * K[3] * s2 + 4.0 * K[4] * s3 + 5.0 * K[5] * s4 + 6.0 * K[6] * s5;
		ka = 2.0 * K[2] + 6.0 * K[3] * s + 12.0 * K[4] * s2 + 20.0 * K[5] * s3 + 30.0 * K[6] * s4;

		real_t C = (type == BipedRunning::SwingInterpolation::Cubic ? 16.0 : 64.0) * pmid;
		p = kp * C;
		v = kv * C / h;
		a = ka * C / h2;
	}

	// swing leg trajectory
	void BipedRunning::FootPose(real_t t, int side, pose_t& pose, vec3_t& vel, vec3_t& angvel, vec3_t& acc, vec3_t& angacc, int& contact) {
		BipedRunKey* keym2 = 0;
		BipedRunKey* keym1 = 0;
		BipedRunKey* key0 = (BipedRunKey*)traj.GetSegment(t).first;
		BipedRunKey* key1 = (BipedRunKey*)traj.GetSegment(t).second;
		BipedRunKey* key2 = 0;
		BipedRunKey* key3 = 0;

		vec3_t pos;
		vec3_t angle;

		if (key1 != key0->next) {
			pose.Pos() = key0->var_foot_pos_t[side]->val;
			pose.Ori() = FromRollPitchYaw(vec3_t(0.0, 0.0, key0->var_foot_pos_r[side]->val));

			vel.clear();
			angvel.clear();
			acc.clear();
			angacc.clear();

			//contact = ContactState::Surface;

			return;
		}

		if (key0->prev) keym1 = (BipedRunKey*)key0->prev;
		if (keym1 && keym1->prev) keym2 = (BipedRunKey*)keym1->prev;
		if (key1->next) key2 = (BipedRunKey*)key1->next;
		if (key2 && key2->next) key3 = (BipedRunKey*)key2->next;

		// phase
		int ph = phase[key0->tick->idx];

		real_t t0 = key0->var_time->val;
		//real_t taum2 = keym2->var_duration->val;
		//real_t taum1 = keym1->var_duration->val;
		real_t tau0 = key0->var_duration->val;  //< phase duration
		//real_t tau1 = key1->var_duration->val;  //< phase duration of next phase
		real_t t1 = t0 + tau0;
		//real_t t3   = t0 + tau0 + tau1 + tau2;   
		real_t dt = std::max(t - t0, 0.0);    //< elapsed time since phase change
		real_t s = dt / tau0;                   //< normalized time
		vec3_t p0 = key0->var_foot_pos_t[side]->val;
		vec3_t p1 = key1->var_foot_pos_t[side]->val;
		real_t yaw0 = key0->var_foot_pos_r[side]->val;
		real_t yaw1 = key1->var_foot_pos_r[side]->val;
		real_t h0 = param.swingHeight[0];
		real_t h1 = param.swingHeight[1];

		if (param.swingProfile == SwingProfile::Cycloid) {

			vec3_t c0 = key0->var_cop_pos->val;
			vec3_t c1 = key1->var_cop_pos->val;
			real_t ct = c0.x + (c1.x - c0.x) * s;
			real_t cv = (c1.x - c0.x) / tau0;

			//double support
			if (ph == Phase::D) {
				pos = p0;

				contact = ContactState::Surface;
			}
			//support foot of single support
			if ((ph == Phase::R && side == 0) ||
				(ph == Phase::L && side == 1)) {
				pos = p0;
				contact = ContactState::Surface;
			}
			//first swing foot
			if ((ph == Phase::R && keym1 && phase[keym1->tick->idx] == Phase::D && side == 1) ||
				(ph == Phase::L && keym1 && phase[keym1->tick->idx] == Phase::D && side == 0) ||
				(ph == Phase::RL && keym2 && phase[keym2->tick->idx] == Phase::D && side == 1) ||
				(ph == Phase::LR && keym2 && phase[keym2->tick->idx] == Phase::D && side == 0)) {

				real_t tau1;
				if (ph == Phase::R || ph == Phase::L)
				{
					p1 = key2->var_foot_pos_t[side]->val;
					tau1 = key1->var_duration->val;
					s = dt / (tau0 + tau1);
				}
				if ((ph == Phase::RL || ph == Phase::LR) && keym1) {
					p0 = keym1->var_foot_pos_t[side]->val;
					dt = std::max(t - keym1->var_time->val, 0.0);
					tau1 = keym1->var_duration->val;
					s = dt / (tau0 + tau1);
				}

				real_t ds = 1 / (tau0 + tau1);
				real_t ch = (s - sin(_2pi * s) / _2pi);
				real_t cv = (1 - cos(_2pi * s)) / 2.0;

					pos = vec3_t(
						p0.x + (p1.x - p0.x) * ch,
						p0.y + (p1.y - p0.y) * ch,
						p0.z + (p1.z - p0.z) * ch + h0 * cv
					);
					vel = vec3_t(
						((p1.x - p0.x) * ds) * (1.0 - cos(_2pi * s)),
						((p1.y - p0.y) * ds) * (1.0 - cos(_2pi * s)),
						((p1.z - p0.z) * ds) * (1.0 - cos(_2pi * s)) + (h0 / (2.0 / ds)) * _2pi * sin(_2pi * s)
					);
					acc = vec3_t(
						((p1.x - p0.x) * (ds * ds)) * _2pi * sin(_2pi * s),
						((p1.y - p0.y) * (ds * ds)) * _2pi * sin(_2pi * s),
						((p1.z - p0.z) * (ds * ds)) * _2pi * sin(_2pi * s) + (h0 / (2.0 / ds / ds)) * (_2pi * _2pi) * cos(_2pi * s)
					);

				contact = ContactState::Float;
			}
			//last swing foot
			else if ((ph == Phase::RL && key2 && phase[key2->tick->idx] == Phase::D && side == 0) ||
				(ph == Phase::LR && key2 && phase[key2->tick->idx] == Phase::D && side == 1) ||
				(ph == Phase::L && phase[key1->tick->idx] == Phase::D && side == 0) ||
				(ph == Phase::R && phase[key1->tick->idx] == Phase::D && side == 1)) {

				real_t tau1;
				if (ph == Phase::RL || ph == Phase::LR) {
					p1 = key2->var_foot_pos_t[side]->val;
					tau1 = key1->var_duration->val;
					s = dt / (tau0 + tau1);
				}
				if ((ph == Phase::R || ph == Phase::L) && keym1) {
					p0 = keym1->var_foot_pos_t[side]->val;
					dt = std::max(t - keym1->var_time->val, 0.0);
					tau1 = keym1->var_duration->val;
					s = dt / (tau0 + tau1);
				}

				//real_t tau1 = (key2 ? key1->var_duration->val : keym1->var_duration->val);
				real_t ds = 1 / (tau0 + tau1);
				//real_t ds = s / dt;
				real_t ch = (s - sin(_2pi * s) / _2pi);
				real_t cv = (1 - cos(_2pi * s)) / 2.0;

				pos = vec3_t(
					p0.x + (p1.x - p0.x) * ch,
					p0.y + (p1.y - p0.y) * ch,
					p0.z + (p1.z - p0.z) * ch + h0 * cv
				);
				vel = vec3_t(
					((p1.x - p0.x) * ds) * (1.0 - cos(_2pi * s)),
					((p1.y - p0.y) * ds) * (1.0 - cos(_2pi * s)),
					((p1.z - p0.z) * ds) * (1.0 - cos(_2pi * s)) + (h0 / (2.0 / ds)) * _2pi * sin(_2pi * s)
				);
				acc = vec3_t(
					((p1.x - p0.x) * (ds * ds)) * _2pi * sin(_2pi * s),
					((p1.y - p0.y) * (ds * ds)) * _2pi * sin(_2pi * s),
					((p1.z - p0.z) * (ds * ds)) * _2pi * sin(_2pi * s) + (h0 / (2.0 / ds / ds)) * (_2pi * _2pi) * cos(_2pi * s)
				);

				contact = ContactState::Float;
			}
			//swing foot
			else if ((ph == Phase::RL) ||
				(ph == Phase::LR) ||
				(ph == Phase::L && side == 0) ||
				(ph == Phase::R && side == 1)) {

				if ((ph == Phase::RL && side == 0) ||
					(ph == Phase::LR && side == 1)) {
					if (key3) p1 = key3->var_foot_pos_t[side]->val;
				}
				if ((ph == Phase::L && side == 0) ||
					(ph == Phase::R && side == 1)) {
					if (keym1) {
						p0 = keym1->var_foot_pos_t[side]->val;
						dt = std::max(t - keym1->var_time->val, 0.0);
					}
					if (key2) p1 = key2->var_foot_pos_t[side]->val;
				}
				if ((ph == Phase::RL && side == 1) ||
					(ph == Phase::LR && side == 0)) {
					if (keym2) {
						p0 = keym2->var_foot_pos_t[side]->val;
						dt = std::max(t - keym2->var_time->val, 0.0);
					}

				}
				real_t tau1 = key1->var_duration->val;
				s = (ph == Phase::R || ph == Phase::L ? dt / (tau0 + tau1 * 2) : dt / (tau0 * 2 + tau1));
				real_t ds = (ph == Phase::R || ph == Phase::L ? 1 / (tau0 + tau1 * 2) : 1 / (tau0 * 2 + tau1));
				real_t ch = (s - sin(_2pi * s) / _2pi);
				real_t cv = (1 - cos(_2pi * s)) / 2.0;

				pos = vec3_t(
					p0.x + (p1.x - p0.x) * ch,
					p0.y + (p1.y - p0.y) * ch,
					p0.z + (p1.z - p0.z) * ch + h0 * cv
				);
				vel = vec3_t(
					((p1.x - p0.x) * ds) * (1.0 - cos(_2pi * s)),
					((p1.y - p0.y) * ds) * (1.0 - cos(_2pi * s)),
					((p1.z - p0.z) * ds) * (1.0 - cos(_2pi * s)) + (h0 / (2.0 / ds)) * _2pi * sin(_2pi * s)
				);
				acc = vec3_t(
					((p1.x - p0.x) * (ds * ds)) * _2pi * sin(_2pi * s),
					((p1.y - p0.y) * (ds * ds)) * _2pi * sin(_2pi * s),
					((p1.z - p0.z) * (ds * ds)) * _2pi * sin(_2pi * s) + (h0 / (2.0 / ds / ds)) * (_2pi * _2pi) * cos(_2pi * s)
				);

				contact = ContactState::Float;
			}
		}

		// experimental profile (may not work correctly)	
		if (param.swingProfile == SwingProfile::Experiment) {

			vec3_t c0 = key0->var_cop_pos->val;
			vec3_t c1 = key1->var_cop_pos->val;
			real_t ct = c0.x + (c1.x - c0.x) * s;
			real_t cv = (c1.x - c0.x) / tau0;

			//double support
			if (ph == Phase::D) {
				pos = p0;

				contact = ContactState::Surface;
			}
			//support foot of single support
			if ((ph == Phase::R && side == 0) ||
				(ph == Phase::L && side == 1)) {
				pos = p0;
				contact = ContactState::Surface;
			}
			//first swing foot
			if ((ph == Phase::R && keym1 && phase[keym1->tick->idx] == Phase::D && side == 1) ||
				(ph == Phase::L && keym1 && phase[keym1->tick->idx] == Phase::D && side == 0) ||
				(ph == Phase::RL && keym2 && phase[keym2->tick->idx] == Phase::D && side == 1) ||
				(ph == Phase::LR && keym2 && phase[keym2->tick->idx] == Phase::D && side == 0)) {

				real_t tau1;
				if (ph == Phase::R || ph == Phase::L)
				{
					p1 = key2->var_foot_pos_t[side]->val;
					tau1 = key1->var_duration->val;
					s = dt / (tau0 + tau1);
				}
				if ((ph == Phase::RL || ph == Phase::LR) && keym1) {
					p0 = keym1->var_foot_pos_t[side]->val;
					dt = std::max(t - keym1->var_time->val, 0.0);
					tau1 = keym1->var_duration->val;
					s = dt / (tau0 + tau1);
				}

				real_t ds = 1 / (tau0 + tau1);
				real_t ch = (s - sin(_2pi * s) / _2pi);
				real_t cv = (1 - cos(_2pi * s)) / 2.0;

				/*	pos = vec3_t(
						p0.x + (p1.x - p0.x) * ch,
						p0.y + (p1.y - p0.y) * ch,
						p0.z + (p1.z - p0.z) * ch + h0 * cv
					);
					vel = vec3_t(
						((p1.x - p0.x) * ds) * (1.0 - cos(_2pi * s)),
						((p1.y - p0.y) * ds) * (1.0 - cos(_2pi * s)),
						((p1.z - p0.z) * ds) * (1.0 - cos(_2pi * s)) + (h0 / (2.0 / ds)) * _2pi * sin(_2pi * s)
					);
					acc = vec3_t(
						((p1.x - p0.x) * (ds * ds)) * _2pi * sin(_2pi * s),
						((p1.y - p0.y) * (ds * ds)) * _2pi * sin(_2pi * s),
						((p1.z - p0.z) * (ds * ds)) * _2pi * sin(_2pi * s) + (h0 / (2.0 / ds / ds)) * (_2pi * _2pi) * cos(_2pi * s)
					);*/
				Interpolate(t, pos.x, vel.x, acc.x, t0, p0.x, 0.0, 0.0, t0 + tau0 + tau1, p1.x, 0.0, 0.0, param.swingInterpolation);
				Interpolate(t, pos.y, vel.y, acc.y, t0, p0.y, 0.0, 0.0, t0 + tau0 + tau1, p1.y, 0.0, 0.0, param.swingInterpolation);
				Interpolate(t, pos.z, vel.z, acc.z, t0, p0.z, 0.0, 0.0, t0 + tau0 + tau1, p1.z, 0.0, 0.0, param.swingInterpolation);

				//pos.z = 1.2922619 * ds - 4.81607143 * ds * ds + 6.202 * ds * ds * ds - 2.67857 * ds * ds * ds * ds;
				real_t dpos_z, dvel_z, dacc_z;
				Interpolate2(t, dpos_z, dvel_z, dacc_z, t0, t0 + tau0 + tau1, h0, param.swingInterpolation);
				pos.z += dpos_z;
				vel.z += dvel_z;
				acc.z += dacc_z;

				contact = ContactState::Float;
			}
			//last swing foot
			else if ((ph == Phase::RL && key2 && phase[key2->tick->idx] == Phase::D && side == 0) ||
				(ph == Phase::LR && key2 && phase[key2->tick->idx] == Phase::D && side == 1) ||
				(ph == Phase::L && phase[key1->tick->idx] == Phase::D && side == 0) ||
				(ph == Phase::R && phase[key1->tick->idx] == Phase::D && side == 1)) {

				real_t tau1;
				if (ph == Phase::RL || ph == Phase::LR) {
					p1 = key2->var_foot_pos_t[side]->val;
					tau1 = key1->var_duration->val;
					s = dt / (tau0 + tau1);
				}
				if ((ph == Phase::R || ph == Phase::L) && keym1) {
					p0 = keym1->var_foot_pos_t[side]->val;
					dt = std::max(t - keym1->var_time->val, 0.0);
					tau1 = keym1->var_duration->val;
					s = dt / (tau0 + tau1);
				}

				//real_t tau1 = (key2 ? key1->var_duration->val : keym1->var_duration->val);
				real_t ds = 1 / (tau0 + tau1);
				//real_t ds = s / dt;
				real_t ch = (s - sin(_2pi * s) / _2pi);
				real_t cv = (1 - cos(_2pi * s)) / 2.0;

				/*pos = vec3_t(
					p0.x + (p1.x - p0.x) * ch,
					p0.y + (p1.y - p0.y) * ch,
					p0.z + (p1.z - p0.z) * ch + h0 * cv
				);
				vel = vec3_t(
					((p1.x - p0.x) * ds) * (1.0 - cos(_2pi * s)),
					((p1.y - p0.y) * ds) * (1.0 - cos(_2pi * s)),
					((p1.z - p0.z) * ds) * (1.0 - cos(_2pi * s)) + (h0 / (2.0 / ds)) * _2pi * sin(_2pi * s)
				);
				acc = vec3_t(
					((p1.x - p0.x) * (ds * ds)) * _2pi * sin(_2pi * s),
					((p1.y - p0.y) * (ds * ds)) * _2pi * sin(_2pi * s),
					((p1.z - p0.z) * (ds * ds)) * _2pi * sin(_2pi * s) + (h0 / (2.0 / ds / ds)) * (_2pi * _2pi) * cos(_2pi * s)
				);*/

				Interpolate(t, pos.x, vel.x, acc.x, t0, p0.x, 0.0, 0.0, t0 + tau0 + tau1, p1.x, 0.0, 0.0, param.swingInterpolation);
				Interpolate(t, pos.y, vel.y, acc.y, t0, p0.y, 0.0, 0.0, t0 + tau0 + tau1, p1.y, 0.0, 0.0, param.swingInterpolation);
				Interpolate(t, pos.z, vel.z, acc.z, t0, p0.z, 0.0, 0.0, t0 + tau0 + tau1, p1.z, 0.0, 0.0, param.swingInterpolation);

				//pos.z = 1.2922619 * ds - 4.81607143 * ds * ds + 6.202 * ds * ds * ds - 2.67857 * ds * ds * ds * ds;
				real_t dpos_z, dvel_z, dacc_z;
				Interpolate2(t, dpos_z, dvel_z, dacc_z, t0, t0 + tau0 + tau1, h0, param.swingInterpolation);
				pos.z += dpos_z;
				vel.z += dvel_z;
				acc.z += dacc_z;

				contact = ContactState::Float;
			}
			//swing foot
			else if ((ph == Phase::RL) ||
				(ph == Phase::LR) ||
				(ph == Phase::L && side == 0) ||
				(ph == Phase::R && side == 1)) {

				if ((ph == Phase::RL && side == 0) ||
					(ph == Phase::LR && side == 1)) {
					if (key3) p1 = key3->var_foot_pos_t[side]->val;
				}
				if ((ph == Phase::L && side == 0) ||
					(ph == Phase::R && side == 1)) {
					if (keym1) {
						p0 = keym1->var_foot_pos_t[side]->val;
						dt = std::max(t - keym1->var_time->val, 0.0);
					}
					if (key2) p1 = key2->var_foot_pos_t[side]->val;
				}
				if ((ph == Phase::RL && side == 1) ||
					(ph == Phase::LR && side == 0)) {
					if (keym2) {
						p0 = keym2->var_foot_pos_t[side]->val;
						dt = std::max(t - keym2->var_time->val, 0.0);
					}

				}
				real_t tau1 = key1->var_duration->val;
				s = (ph == Phase::R || ph == Phase::L ? dt / (tau0 + tau1 * 2) : dt / (tau0 * 2 + tau1));
				real_t ds = (ph == Phase::R || ph == Phase::L ? 1 / (tau0 + tau1 * 2) : 1 / (tau0 * 2 + tau1));
				real_t ch = (s - sin(_2pi * s) / _2pi);
				real_t cv = (1 - cos(_2pi * s)) / 2.0;

				/*pos = vec3_t(
					p0.x + (p1.x - p0.x) * ch,
					p0.y + (p1.y - p0.y) * ch,
					p0.z + (p1.z - p0.z) * ch + h0 * cv
				);
				vel = vec3_t(
					((p1.x - p0.x) * ds) * (1.0 - cos(_2pi * s)),
					((p1.y - p0.y) * ds) * (1.0 - cos(_2pi * s)),
					((p1.z - p0.z) * ds) * (1.0 - cos(_2pi * s)) + (h0 / (2.0 / ds)) * _2pi * sin(_2pi * s)
				);
				acc = vec3_t(
					((p1.x - p0.x) * (ds * ds)) * _2pi * sin(_2pi * s),
					((p1.y - p0.y) * (ds * ds)) * _2pi * sin(_2pi * s),
					((p1.z - p0.z) * (ds * ds)) * _2pi * sin(_2pi * s) + (h0 / (2.0 / ds / ds)) * (_2pi * _2pi) * cos(_2pi * s)
				);*/
				real_t ptime = (ph == Phase::R || ph == Phase::L ? (tau0 + tau1 * 2) : (tau0 * 2 + tau1));
				Interpolate(t, pos.x, vel.x, acc.x, t0, p0.x, 0.0, 0.0, t0 + ptime, p1.x, 0.0, 0.0, param.swingInterpolation);
				Interpolate(t, pos.y, vel.y, acc.y, t0, p0.y, 0.0, 0.0, t0 + ptime, p1.y, 0.0, 0.0, param.swingInterpolation);
				Interpolate(t, pos.z, vel.z, acc.z, t0, p0.z, 0.0, 0.0, t0 + ptime, p1.z, 0.0, 0.0, param.swingInterpolation);

				//pos.z = 1.2922619 * ds - 4.81607143 * ds * ds + 6.202 * ds * ds * ds - 2.67857 * ds * ds * ds * ds;
				real_t dpos_z, dvel_z, dacc_z;
				Interpolate2(t, dpos_z, dvel_z, dacc_z, t0, t0 + ptime, h0, param.swingInterpolation);
				pos.z += dpos_z;
				vel.z += dvel_z;
				acc.z += dacc_z;

				contact = ContactState::Float;
			}
		}

		// yaw angle: linear interpolation
		angle[2] = (1.0 - s) * yaw0 + s * yaw1;
		angvel[2] = (yaw1 - yaw0) / tau0;
		angacc[2] = 0.0;

		pose.Pos() = pos;
		pose.Ori() = FromRollPitchYaw(angle);
	}

	real_t BipedRunning::TimeToLiftoff(real_t t, int side) {
		BipedRunKey* key = (BipedRunKey*)traj.GetSegment(t).first;
		BipedRunKey* keyNextFloat = key;

		int phase_firstfloat = (side == 0 ? Phase::RL : Phase::LR);
		while (keyNextFloat->next && phase[keyNextFloat->tick->idx] != phase_firstfloat)
			keyNextFloat = (BipedRunKey*)keyNextFloat->next;

		// return 1.0 if there is no next float phase in the trajectory
		if (key == keyNextFloat && phase[key->tick->idx] != phase_firstfloat)
			return 1.0;

		return keyNextFloat->tick->time - t;
	}

	real_t BipedRunning::TimeToLanding(real_t t, int side) {
		BipedRunKey* key = (BipedRunKey*)traj.GetSegment(t).first;
		BipedRunKey* keyNextContact = key;

		int phase_lastfloat = (side == 0 ? Phase::LR : Phase::RL);
		while (keyNextContact->next && phase[keyNextContact->tick->idx] != phase_lastfloat)
			keyNextContact = (BipedRunKey*)keyNextContact->next;

		// return 1.0 if there is no next contact phase in the trajectory
		if (key == keyNextContact && phase[key->tick->idx] != phase_lastfloat)
			return 1.0;
		if (keyNextContact->next) keyNextContact = (BipedRunKey*)keyNextContact->next;
		return keyNextContact->tick->time - t;
	}

	vec3_t BipedRunning::CopPos(real_t t) {
		BipedRunKey* key0 = (BipedRunKey*)traj.GetSegment(t).first;
		BipedRunKey* key1 = (BipedRunKey*)traj.GetSegment(t).second;

		vec3_t ct;

		vec3_t c0 = key0->var_cop_pos->val;

		if (key1 == key0->next) {
			real_t dt = t - key0->var_time->val;
			vec3_t cv0 = key0->var_cop_vel->val;

			ct = c0 + cv0 * dt;
		}
		else {
			ct = c0;
		}

		return ct;
	}

	vec3_t BipedRunning::CmpPos(real_t t) {
		BipedRunKey* key0 = (BipedRunKey*)traj.GetSegment(t).first;
		BipedRunKey* key1 = (BipedRunKey*)traj.GetSegment(t).second;

		vec3_t cmt;

		vec3_t c0 = key0->var_cop_pos->val;
		vec3_t cm0 = key0->var_cmp_pos->val;

		if (key1 == key0->next) {
			real_t dt = t - key0->var_time->val;
			vec3_t cv0 = key0->var_cop_vel->val;
			vec3_t cmv0 = key0->var_cmp_vel->val;

			cmt = (c0 + cm0) + (cv0 + cmv0) * dt;
		}
		else {
			cmt = (c0 + cm0);
		}

		return cmt;
		//return vec3_t(cmt.x, cmt.y, 0.0);
	}

	vec3_t BipedRunning::TorsoPos(const vec3_t& pcom, const vec3_t& psup, const vec3_t& pswg) {
		real_t mt = param.torsoMass;
		real_t mf = param.footMass;
		vec3_t pt = ((mt + 2.0 * mf) * pcom - mf * (psup + pswg)) / mt;
		return pt;
	}

	vec3_t BipedRunning::TorsoVel(const vec3_t& vcom, const vec3_t& vsup, const vec3_t& vswg) {
		real_t mt = param.torsoMass;
		real_t mf = param.footMass;
		vec3_t vt = ((mt + 2.0 * mf) * vcom - mf * (vsup + vswg)) / mt;
		return vt;
	}

	vec3_t BipedRunning::TorsoAcc(const vec3_t& acom, const vec3_t& asup, const vec3_t& aswg) {
		real_t mt = param.torsoMass;
		real_t mf = param.footMass;
		vec3_t at = ((mt + 2.0 * mf) * acom - mf * (asup + aswg)) / mt;
		return at;
	}

	//------------------------------------------------------------------------------------------------

	void BipedRunning::Draw(Render::Canvas* canvas, Render::Config* conf) {
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

	void BipedRunning::CreateSnapshot(real_t t, BipedRunning::Snapshot& s) {
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

	void BipedRunning::CreateSnapshot(real_t t) {
		CreateSnapshot(t, snapshot);
	}

	void BipedRunning::CalcTrajectory() {
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

	void BipedRunning::Save(const char* filename) {
		ofstream log(filename);
		log << "t com_pos.x com_pos.y com_pos.z com_vel.x com_vel.y com_vel.z cop_pos.x cop_pos.y cop_pos.z foot_pos0.x foot_pos0.y foot_pos0.z foot_pos1.x foot_pos1.y foot_pos1.z\n";
		for (int i = 0; i < trajectory.size(); i++)
		{
			log << trajectory[i].t << " ";
			log << trajectory[i].com_pos.x << " " << trajectory[i].com_pos.y << " " << trajectory[i].com_pos.z << " ";
			log << trajectory[i].com_vel.x << " " << trajectory[i].com_vel.y << " " << trajectory[i].com_vel.z << " ";
			log << trajectory[i].cop_pos.x << " " << trajectory[i].cop_pos.y << " " << trajectory[i].cop_pos.z << " ";
			log << trajectory[i].foot_pos_t[0].x << " " << trajectory[i].foot_pos_t[0].y << " " << trajectory[i].foot_pos_t[0].z << " ";
			log << trajectory[i].foot_pos_t[1].x << " " << trajectory[i].foot_pos_t[1].y << " " << trajectory[i].foot_pos_t[1].z;
			log << "\n";
		}
		log.close();
	}

	void BipedRunning::DrawSnapshot(Render::Canvas* canvas, Render::Config* conf) {
		// lines connecting com and feet
		canvas->SetLineWidth(2.0f);
		canvas->BeginPath();
		for (int i = 0; i < 2; i++) {
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
		for (int i = 0; i < 2; i++) {
			for (int j = 0; j < 4; j++) {
				p[j] = snapshot.foot_pos_t[i] + snapshot.foot_pos_r[i] * v[j];
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
	RunnerLipCon::RunnerLipCon(Solver* solver, int _tag, string _name, BipedRunKey* _obj, real_t _scale) :
		Constraint(solver, 3, ID(_tag, _obj->node, _obj->tick, _name), Constraint::Type::Equality, _scale) {
		obj[0] = _obj;
		obj[1] = (_obj->next ? (BipedRunKey*)_obj->next : 0);
	}

	RunnerLipPosCon::RunnerLipPosCon(Solver* solver, string _name, BipedRunKey* _obj, real_t _scale) :
		RunnerLipCon(solver, ConTag::BipedLipPos, _name, _obj, _scale) {

		AddSLink(obj[1]->var_com_pos);
		AddSLink(obj[0]->var_com_pos);
		AddSLink(obj[0]->var_com_vel);
		AddSLink(obj[0]->var_cop_pos);
		AddSLink(obj[0]->var_cmp_pos);
		AddSLink(obj[0]->var_cop_vel);
		AddSLink(obj[0]->var_cmp_vel);
		AddC3Link(obj[0]->var_duration);
	}

	RunnerLipVelCon::RunnerLipVelCon(Solver* solver, string _name, BipedRunKey* _obj, real_t _scale) :
		RunnerLipCon(solver, ConTag::BipedLipVel, _name, _obj, _scale) {

		AddSLink(obj[1]->var_com_vel);
		AddSLink(obj[0]->var_com_pos);
		AddSLink(obj[0]->var_com_vel);
		AddSLink(obj[0]->var_cop_pos);
		AddSLink(obj[0]->var_cmp_pos);
		AddSLink(obj[0]->var_cop_vel);
		AddSLink(obj[0]->var_cmp_vel);
		AddC3Link(obj[0]->var_duration);
	}

	RunnerLipCopCon::RunnerLipCopCon(Solver* solver, string _name, BipedRunKey* _obj, real_t _scale) :
		RunnerLipCon(solver, ConTag::BipedFootCop, _name, _obj, _scale) {

		AddSLink(obj[1]->var_cop_pos);
		AddSLink(obj[0]->var_cop_pos);
		AddSLink(obj[0]->var_cop_vel);
		AddC3Link(obj[0]->var_duration);
	}

	//BipedFootCopPosCon::BipedFootCopPosCon(Solver* solver, string _name, BipedRunKey* _obj, int _side, real_t _scale) :
	//	Constraint(solver, 3, ID(ConTag::BipedFootCop, _obj->node, _obj->tick, _name), Constraint::Type::Equality, _scale) {

	//	obj[0] = _obj;
	//	obj[1] = (_obj->next ? (BipedLIPKey*)_obj->next : 0);
	//	side = _side;

	//	AddSLink(obj[1]->foot[side].var_cop_pos);
	//	AddSLink(obj[0]->foot[side].var_cop_pos);
	//	AddSLink(obj[0]->foot[side].var_cop_vel);
	//	AddC3Link(obj[0]->var_duration);
	//}

	RunnerLipCmpCon::RunnerLipCmpCon(Solver* solver, string _name, BipedRunKey* _obj, real_t _scale) :
		RunnerLipCon(solver, ConTag::Any, _name, _obj, _scale) {

		AddSLink(obj[1]->var_cmp_pos);
		AddSLink(obj[0]->var_cmp_pos);
		AddSLink(obj[0]->var_cmp_vel);
		AddC3Link(obj[0]->var_duration);
	}

	RunnerLipMomCon::RunnerLipMomCon(Solver* solver, string _name, BipedRunKey* _obj, real_t _scale) :
		RunnerLipCon(solver, ConTag::Any, _name, _obj, _scale) {

		AddSLink(obj[1]->var_mom);
		AddSLink(obj[0]->var_mom);
		AddX3Link(obj[0]->var_cmp_pos);
		AddX3Link(obj[0]->var_cmp_vel);
		AddC3Link(obj[0]->var_duration);
	}

	RunnerComConP::RunnerComConP(Solver* solver, string _name, BipedRunKey* _obj, real_t _scale) :
		Constraint(solver, 2, ID(ConTag::BipedComPos, _obj->node, _obj->tick, _name), Constraint::Type::Equality, _scale) {

		obj = _obj;

		AddSLink(obj->var_com_pos);
		AddSLink(obj->var_torso_pos_t);
		AddSLink(obj->var_foot_pos_t[0]);
		AddSLink(obj->var_foot_pos_t[1]);
	}

	RunnerComConV::RunnerComConV(Solver* solver, string _name, BipedRunKey* _obj, real_t _scale) :
		Constraint(solver, 2, ID(ConTag::BipedComVel, _obj->node, _obj->tick, _name), Constraint::Type::Equality, _scale) {

		obj = _obj;

		AddSLink(obj->var_com_vel);
		AddSLink(obj->var_torso_vel_t);
	}

	RunnerRangeCon::RunnerRangeCon(Solver* solver, int _tag, string _name, BipedRunKey* _obj, vec3_t _dir, real_t _scale) :
		Constraint(solver, 1, ID(_tag, _obj->node, _obj->tick, _name), Constraint::Type::Equality, _scale) {
		obj = _obj;
		dir = _dir;
	}

	RunnerFootRangeConT::RunnerFootRangeConT(Solver* solver, string _name, BipedRunKey* _obj, uint _side, vec3_t _dir, real_t _scale) :
		RunnerRangeCon(solver, ConTag::BipedFootPosRangeT, _name, _obj, _dir, _scale) {

		side = _side;

		AddR3Link(obj->var_foot_pos_t[side]);
		AddR3Link(obj->var_torso_pos_t);
		AddSLink(obj->var_torso_pos_r);
	}

	RunnerFootRangeConR::RunnerFootRangeConR(Solver* solver, string _name, BipedRunKey* _obj, uint _side, real_t _scale) :
		RunnerRangeCon(solver, ConTag::BipedFootPosRangeR, _name, _obj, vec3_t(), _scale) {

		side = _side;

		AddSLink(obj->var_foot_pos_r[side]);
		AddSLink(obj->var_torso_pos_r);
	}

	RunnerCopRangeCon::RunnerCopRangeCon(Solver* solver, string _name, BipedRunKey* _obj, uint _side, vec3_t _dir, real_t _scale) :
		RunnerRangeCon(solver, ConTag::BipedFootCopRange, _name, _obj, _dir, _scale) {

		side = _side;

		AddR3Link(obj->var_cop_pos);
		AddR3Link(obj->var_foot_pos_t[side]);
		AddSLink(obj->var_foot_pos_r[side]);
	}

	RunnerCmpRangeCon::RunnerCmpRangeCon(Solver* solver, string _name, BipedRunKey* _obj, vec3_t _dir, real_t _scale) :
		RunnerRangeCon(solver, ConTag::Any, _name, _obj, _dir, _scale) {

		AddR3Link(obj->var_cmp_pos);
		AddSLink(obj->var_torso_pos_r);
	}

	RunnerAccRangeCon::RunnerAccRangeCon(Solver* solver, string _name, BipedRunKey* _obj, vec3_t _dir, real_t _scale) :
		RunnerRangeCon(solver, ConTag::Any, _name, _obj, _dir, _scale) {

		AddR3Link(obj->var_com_pos);
		AddR3Link(obj->var_cop_pos);
		AddR3Link(obj->var_cmp_pos);
		AddSLink(obj->var_torso_pos_r);
	}

	RunnerMomRangeCon::RunnerMomRangeCon(Solver* solver, string _name, BipedRunKey* _obj, vec3_t _dir, real_t _scale) :
		RunnerRangeCon(solver, ConTag::Any, _name, _obj, _dir, _scale) {

		AddR3Link(obj->var_mom);
		AddSLink(obj->var_torso_pos_r);
	}

	RunnerFootHeightCon::RunnerFootHeightCon(Solver* solver, string _name, BipedRunKey* _obj, uint _side, real_t _scale) :
		Constraint(solver, 1, ID(ConTag::Any, _obj->node, _obj->tick, _name), Constraint::Type::Equality, _scale) {

		obj = _obj;
		side = _side;

		AddR3Link(obj->var_foot_pos_t[side]);
	}

	RunnerTimeCon::RunnerTimeCon(Solver* solver, string _name, BipedRunKey* _obj, real_t _scale) :
		Constraint(solver, 1, ID(ConTag::BipedTime, _obj->node, _obj->tick, _name), Constraint::Type::Equality, _scale) {

		obj[0] = _obj;
		obj[1] = (BipedRunKey*)_obj->next;

		AddSLink(obj[1]->var_time);
		AddSLink(obj[0]->var_time);
		AddSLink(obj[0]->var_duration);
	}

	//-------------------------------------------------------------------------------------------------

	void RunnerLipPosCon::CalcLhs() {
		Prepare();
		if (ph == BipedRunning::Phase::LR || ph == BipedRunning::Phase::RL)
			obj[1]->var_com_pos->val = p0 + v0 * tau - 0.5 * g * tau * tau;
		else
			obj[1]->var_com_pos->val = (c0 + cm0) + (cv0 + cmv0) * tau + C * (p0 - (c0 + cm0)) + (S * T) * (v0 - (cv0 + cmv0));
	}

	void RunnerLipVelCon::CalcLhs() {
		Prepare();
		if (ph == BipedRunning::Phase::LR || ph == BipedRunning::Phase::LR)
			obj[1]->var_com_vel->val = v0 - g * tau;
		else
			obj[1]->var_com_vel->val = (cv0 + cmv0) + (S / T) * (p0 - (c0 + cm0)) + C * (v0 - (cv0 + cmv0));
	}

	void RunnerLipCopCon::CalcLhs() {
		Prepare();
		obj[1]->var_cop_pos->val = c0 + cv0 * tau;
	}

	void RunnerLipCmpCon::CalcLhs() {
		Prepare();
		obj[1]->var_cmp_pos->val = cm0 + cmv0 * tau;
	}

	void RunnerLipMomCon::CalcLhs() {
		Prepare();
		obj[1]->var_mom->val = L0 + ez % (cm0 * tau + 0.5 * tau * tau * cmv0);
	}

	void RunnerTimeCon::CalcLhs() {
		obj[1]->var_time->val = obj[0]->var_time->val + obj[0]->var_duration->val;
	}

	//-------------------------------------------------------------------------------------------------

	void RunnerLipCon::Prepare() {
		BipedRunning::Param& param = ((BipedRunning*)obj[0]->node)->param;
		std::vector<int>& phase = ((BipedRunning*)obj[0]->node)->phase;
		ph = phase[obj[0]->tick->idx];
		if (ph == BipedRunning::Phase::D)
		{
			T = param.T[0]; //0.3144 for h=0.9; // 0.3316 for h=1.0, tau0=0.30, tau1=0.10
		}
		else if (!obj[0]->prev->prev || !obj[1]->next->next)
		{
			T = param.T[1];//0.2659 for h=0.9; // 0.2807 for h=1.0, tau0=0.30, tau1=0.10
		}
		else T = param.T[2];
		g = param.gravity;
		mode = param.trajectoryMode;
		ez = vec3_t(0.0, 0.0, 1.0);
		//L0 = obj[0]->var_mom->val;
		p0 = obj[0]->var_com_pos->val;
		v0 = obj[0]->var_com_vel->val;
		c0 = obj[0]->var_cop_pos->val + vec3_t(0.0, 0.0, T * T * g.z);  //< c0 is vrp
		cm0 = obj[0]->var_cmp_pos->val;

		if (obj[1]) {
			tau = obj[0]->var_duration->val;
			C = cosh(tau / T);
			S = sinh(tau / T);

			cv0 = obj[0]->var_cop_vel->val;
			cmv0 = obj[0]->var_cmp_vel->val; 

			//L1 = obj[1]->var_mom->val;
			p1 = obj[1]->var_com_pos->val;
			v1 = obj[1]->var_com_vel->val;
			c1 = obj[1]->var_cop_pos->val + vec3_t(0.0, 0.0, T * T * g.z);
			cm1 = obj[1]->var_cmp_pos->val;
		}
	}

	void RunnerRangeCon::Prepare() {
		R = mat3_t::Rot(theta, 'z');
		ez = vec3_t(0.0, 0.0, 1.0);
		dir_abs = R * dir;
	}

	//-------------------------------------------------------------------------------------------------

	void RunnerLipPosCon::CalcCoef() {
		Prepare();
		if (ph == BipedRunning::Phase::LR || ph == BipedRunning::Phase::RL)
		{
			((SLink*)links[0])->SetCoef(1.0);
			((SLink*)links[1])->SetCoef(-1.0);
			((SLink*)links[2])->SetCoef(-tau);
			((SLink*)links[3])->SetCoef(0.0);
			((SLink*)links[4])->SetCoef(0.0);
			((SLink*)links[5])->SetCoef(0.0);
			((SLink*)links[6])->SetCoef(0.0);
			((C3Link*)links[7])->SetCoef(-(v0 - g * tau));

		}
		else
		{
			((SLink*)links[0])->SetCoef(1.0);
			((SLink*)links[1])->SetCoef(-C);
			((SLink*)links[2])->SetCoef(-T * S);
			((SLink*)links[3])->SetCoef(-1.0 + C);
			((SLink*)links[4])->SetCoef(-1.0 + C);
			((SLink*)links[5])->SetCoef(-tau + T * S);
			((SLink*)links[6])->SetCoef(-tau + T * S);
			((C3Link*)links[7])->SetCoef(-(cv0 + cmv0) - (S / T) * (p0 - (c0 + cm0)) - C * (v0 - (cv0 + cmv0)));
		}
	}

	void RunnerLipVelCon::CalcCoef() {
		Prepare();
		if (ph == BipedRunning::Phase::LR || ph == BipedRunning::Phase::RL)
		{
			((SLink*)links[0])->SetCoef(1.0);
			((SLink*)links[1])->SetCoef(0.0);
			((SLink*)links[2])->SetCoef(-1.0);
			((SLink*)links[3])->SetCoef(0.0);
			((SLink*)links[4])->SetCoef(0.0);
			((SLink*)links[5])->SetCoef(0.0);
			((SLink*)links[6])->SetCoef(0.0);
			((C3Link*)links[7])->SetCoef(g);
		}
		else
		{
			((SLink*)links[0])->SetCoef(1.0);
			((SLink*)links[1])->SetCoef(-S / T);
			((SLink*)links[2])->SetCoef(-C);
			((SLink*)links[3])->SetCoef(S / T);
			((SLink*)links[4])->SetCoef(S / T);
			((SLink*)links[5])->SetCoef(-1.0 + C);
			((SLink*)links[6])->SetCoef(-1.0 + C);
			((C3Link*)links[7])->SetCoef(-(C / (T * T)) * (p0 - (c0 + cm0)) - (S / T) * (v0 - (cv0 + cmv0)));
		}
	}

	void RunnerLipCopCon::CalcCoef() {
		Prepare();

		((SLink*)links[0])->SetCoef(1.0);
		((SLink*)links[1])->SetCoef(-1.0);
		((SLink*)links[2])->SetCoef(-tau);
		((C3Link*)links[3])->SetCoef(-cv0);
	}

	void RunnerLipCmpCon::CalcCoef() {
		Prepare();

		((SLink*)links[0])->SetCoef(1.0);
		((SLink*)links[1])->SetCoef(-1.0);
		((SLink*)links[2])->SetCoef(-tau);
		((C3Link*)links[3])->SetCoef(-cmv0);
	}

	void RunnerLipMomCon::CalcCoef() {
		Prepare();

		((SLink*)links[0])->SetCoef(1.0);
		((SLink*)links[1])->SetCoef(-1.0);
		((X3Link*)links[2])->SetCoef(-tau * ez);
		((X3Link*)links[3])->SetCoef(-(0.5 * tau * tau) * ez);
		((C3Link*)links[4])->SetCoef(-(ez % (cm0 + cmv0 * tau)));
	}

	void RunnerComConP::CalcCoef() {
		BipedRunning::Param& param = ((BipedRunning*)obj->node)->param;

		real_t mt = param.torsoMass;
		real_t mf = param.footMass;
		real_t msum = mt + 2.0 * mf;

		((SLink*)links[0])->SetCoef(1.0);
		((SLink*)links[1])->SetCoef(-mt / msum);
		((SLink*)links[2])->SetCoef(-mf / msum);
		((SLink*)links[3])->SetCoef(-mf / msum);

	}

	void RunnerComConV::CalcCoef() {
		BipedRunning::Param& param = ((BipedRunning*)obj->node)->param;

		real_t mt = param.torsoMass;
		real_t mf = param.footMass;
		real_t msum = mt + 2.0 * mf;

		((SLink*)links[0])->SetCoef(1.0);
		((SLink*)links[1])->SetCoef(-mt / msum);
	}

	void RunnerFootRangeConT::CalcCoef() {
		r = obj->var_foot_pos_t[side]->val - obj->var_torso_pos_t->val;
		theta = obj->var_torso_pos_r->val;

		Prepare();

		((R3Link*)links[0])->SetCoef(dir_abs);
		((R3Link*)links[1])->SetCoef(-dir_abs);
		((SLink*)links[2])->SetCoef((ez % dir_abs) * r);
	}

	void RunnerFootRangeConR::CalcCoef() {
		thetaf = obj->var_foot_pos_r[side]->val;
		thetat = obj->var_torso_pos_r->val;

		((SLink*)links[0])->SetCoef(1.0);
		((SLink*)links[1])->SetCoef(-1.0);
	}

	void RunnerCopRangeCon::CalcCoef() {
		r = obj->var_cop_pos->val - obj->var_foot_pos_t[side]->val;
		theta = obj->var_foot_pos_r[side]->val;

		Prepare();

		((R3Link*)links[0])->SetCoef(dir_abs);
		((R3Link*)links[1])->SetCoef(-dir_abs);
		((SLink*)links[2])->SetCoef((ez % dir_abs) * r);
	}

	void RunnerCmpRangeCon::CalcCoef() {
		r = obj->var_cmp_pos->val;
		theta = obj->var_torso_pos_r->val;

		Prepare();

		((R3Link*)links[0])->SetCoef(dir_abs);
		((SLink*)links[1])->SetCoef((ez % dir_abs) * r);
	}

	// TODO : support of changing time constant
	void RunnerAccRangeCon::CalcCoef() {
		BipedRunning::Param& param = ((BipedRunning*)obj->node)->param;

		real_t T = param.T[2];
		vec3_t p = obj->var_com_pos->val;
		vec3_t c = obj->var_cop_pos->val;
		vec3_t cm = obj->var_cmp_pos->val;
		r = (1.0 / (T * T)) * (p - (c + vec3_t(0.0, 0.0, param.comHeight) + cm));
		theta = obj->var_torso_pos_r->val;

		Prepare();

		((R3Link*)links[0])->SetCoef(dir_abs);
		((R3Link*)links[1])->SetCoef(-dir_abs);
		((R3Link*)links[2])->SetCoef(-dir_abs);
		((SLink*)links[3])->SetCoef((ez % dir_abs) * r);
	}

	void RunnerMomRangeCon::CalcCoef() {
		r = obj->var_mom->val;
		theta = obj->var_torso_pos_r->val;

		Prepare();

		((R3Link*)links[0])->SetCoef(dir_abs);
		((SLink*)links[1])->SetCoef((ez % dir_abs) * r);
	}

	void RunnerFootHeightCon::CalcCoef() {
		((R3Link*)links[0])->SetCoef(vec3_t(0.0, 0.0, 1.0));
	}

	void RunnerTimeCon::CalcCoef() {
		((SLink*)links[0])->SetCoef(1.0);
		((SLink*)links[1])->SetCoef(-1.0);
		((SLink*)links[2])->SetCoef(-1.0);
	}

	//-------------------------------------------------------------------------------------------------

	void RunnerLipPosCon::CalcDeviation() {
		if (ph == BipedRunning::Phase::LR || ph == BipedRunning::Phase::RL)
			y = p1 - (p0 + v0 * tau - 0.5 * g * tau * tau);
		else
			y = p1 - ((c0 + cm0) + (cv0 + cmv0) * tau + C * (p0 - (c0 + cm0)) + (T * S) * (v0 - (cv0 + cmv0)));
	}

	void RunnerLipVelCon::CalcDeviation() {
		if (ph == BipedRunning::Phase::LR || ph == BipedRunning::Phase::RL)
			y = v1 - (v0 - g * tau);
		else
			y = v1 - ((cv0 + cmv0) + (S / T) * (p0 - (c0 + cm0)) + C * (v0 - (cv0 + cmv0)));
	}

	void RunnerLipCopCon::CalcDeviation() {
		y = c1 - (c0 + cv0 * tau);
	}

	void RunnerLipCmpCon::CalcDeviation() {
		y = cm1 - (cm0 + cmv0 * tau);
	}

	void RunnerLipMomCon::CalcDeviation() {
		y = L1 - (L0 + ez % (cm0 * tau + 0.5 * tau * tau * cmv0));
	}

	void RunnerRangeCon::CalcDeviation() {
		real_t s = dir_abs * r;

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

	void RunnerFootRangeConR::CalcDeviation() {
		real_t s = thetaf - thetat;
		on_lower = (s < _min);
		on_upper = (s > _max);
		active = on_lower | on_upper;
		if (on_lower) y[0] = (s - _min);
		if (on_upper) y[0] = (s - _max);
	}

	void RunnerFootHeightCon::CalcDeviation() {
		y[0] = obj->var_foot_pos_t[side]->val.z - ((BipedRunning*)obj->node)->elevation[obj->tick->idx];
	}

	//-------------------------------------------------------------------------------------------------

	void RunnerRangeCon::Project(real_t& l, uint k) {
		if (on_upper && l > 0.0) l = 0.0;
		if (on_lower && l < 0.0) l = 0.0;
		if (!on_upper && !on_lower) l = 0.0;
	}

}
