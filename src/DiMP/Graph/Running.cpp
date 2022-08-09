#include <DiMP/Graph/Running.h>
#include <DiMP/Graph/Graph.h>
#include <DiMP/Render/Config.h>
#include <DiMP/Render/Canvas.h>

#include <stdio.h>

#include <sbrollpitchyaw.h>

namespace DiMP {
	;

	const real_t   pi = M_PI;
	const real_t _2pi = 2.0 * pi;

	const real_t damping = 0.1;

	//-------------------------------------------------------------------------------------------------
	// BipedRunKey

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

		// CoM position and velocity
		var_com_pos = new V3Var(solver, ID(VarTag::BipedComP, node, tick, name + "_com_p"), node->graph->scale.pos_t);
		var_com_vel = new V3Var(solver, ID(VarTag::BipedComV, node, tick, name + "_com_v"), node->graph->scale.vel_t);

		// absolute time
		var_time = new SVar(solver, ID(VarTag::BipedTime, node, tick, name + "_time"), node->graph->scale.time);

		var_torso_pos_t->weight = damping * one;
		var_torso_pos_r->weight[0] = damping;
		var_torso_vel_t->weight = damping * one;

		var_com_pos->weight = damping * one;
		var_com_vel->weight = damping * one;

		var_time->weight[0] = damping;

		// register state variables of ddp
		solver->AddInputVar(var_torso_pos_t, tick->idx);
		solver->AddInputVar(var_torso_pos_r, tick->idx);
		solver->AddInputVar(var_torso_vel_t, tick->idx);

		solver->AddStateVar(var_com_pos, tick->idx);
		solver->AddStateVar(var_com_vel, tick->idx);

		solver->AddStateVar(var_time, tick->idx);

		if (next) {
			// step duration
			var_duration = new SVar(solver, ID(VarTag::BipedDuration, node, tick, name + "_duration"), node->graph->scale.time);
			var_duration->weight[0] = damping;

			// register input variables of ddp
			solver->AddInputVar(var_duration, tick->idx);
		}

		// foot position
		for (int i = 0; i < 2; i++) {
			string prefix = (i == 0 ? "_foot_r" : "_foot_l");

			foot[i].var_pos_t = new V3Var(solver, ID(VarTag::BipedFootTP, node, tick, name + prefix + "_foot_t"), node->graph->scale.pos_t);
			foot[i].var_pos_r = new SVar(solver, ID(VarTag::BipedFootRP, node, tick, name + prefix + "_foot_r"), node->graph->scale.pos_r);
			foot[i].var_cop_pos = new V3Var(solver, ID(VarTag::BipedFootCopP, node, tick, name + prefix + "_cop_p"), node->graph->scale.pos_t);
			foot[i].var_cop_vel = new V3Var(solver, ID(VarTag::BipedFootCopV, node, tick, name + prefix + "_cop_v"), node->graph->scale.vel_t);

			foot[i].var_pos_t->weight = damping * one;
			foot[i].var_pos_r->weight[0] = damping;
			foot[i].var_cop_pos->weight = damping * one;
			foot[i].var_cop_vel->weight = damping * one;

			foot[i].var_pos_r->locked = true;

			solver->AddStateVar(foot[i].var_pos_t, tick->idx);
			solver->AddStateVar(foot[i].var_pos_r, tick->idx);
			solver->AddStateVar(foot[i].var_cop_pos, tick->idx);
			solver->AddStateVar(foot[i].var_cop_vel, tick->idx);

			if (next) {
				foot[i].var_vel_t = new V3Var(solver, ID(VarTag::BipedFootTV, node, tick, name + prefix + "_foot_t"), node->graph->scale.vel_t);
				foot[i].var_vel_r = new SVar(solver, ID(VarTag::BipedFootRV, node, tick, name + prefix + "_foot_r"), node->graph->scale.vel_r);
				foot[i].var_cop_vel_diff = new V3Var(solver, ID(VarTag::BipedFootCopV, node, tick, name + prefix + "_cop_vd"), node->graph->scale.vel_t);

				foot[i].var_vel_t->weight = damping * one;
				foot[i].var_vel_r->weight[0] = damping;
				foot[i].var_cop_vel_diff->weight = damping * one;

				solver->AddInputVar(foot[i].var_vel_t, tick->idx);
				solver->AddInputVar(foot[i].var_vel_r, tick->idx);
				solver->AddInputVar(foot[i].var_cop_vel_diff, tick->idx);
			}
		}
	}

	// register constraints for planning
	void BipedRunKey::AddCon(Solver* solver) {
		BipedRunning* obj = (BipedRunning*)node;
		BipedRunKey* nextObj = (BipedRunKey*)next;

		vec3_t one(1.0, 1.0, 1.0);

		if (next) {
			con_lip_pos = new BipedLipPosCon(solver, name + "_lip_pos", this, node->graph->scale.pos_t);
			con_lip_vel = new BipedLipVelCon(solver, name + "_lip_vel", this, node->graph->scale.vel_t);

			con_time = new BipedTimeCon(solver, name + "_time", this, node->graph->scale.time);

			con_duration_range = new RangeConS(solver, ID(ConTag::BipedDurationRange, node, tick, name + "_duration"), var_duration, node->graph->scale.time);

			//con_com_vel_zero = new FixConV3(solver, ID(ConTag::BipedFootVelZeroT, node, tick, name + "_com_vel_zero"), var_com_vel, node->graph->scale.vel_t);
			//con_com_vel_zero->weight = 0.01*one;

			solver->AddTransitionCon(con_lip_pos, tick->idx);
			solver->AddTransitionCon(con_lip_vel, tick->idx);

			solver->AddTransitionCon(con_time, tick->idx);

			solver->AddCostCon(con_duration_range, tick->idx);

			//solver->AddCostCon(con_com_vel_zero, tick->idx);
		}

		con_com_pos = new BipedComConP(solver, name + "_com_p", this, node->graph->scale.pos_t);
		solver->AddCostCon(con_com_pos, tick->idx);

		if (next) {
			con_com_vel = new BipedComConV(solver, name + "_com_v", this, node->graph->scale.vel_t);
			solver->AddCostCon(con_com_vel, tick->idx);
		}

		for (int i = 0; i < 2; i++) {
			string prefix = (i == 0 ? "_foot_r" : "_foot_l");

			if (next) {
				foot[i].con_pos_t = new BipedFootPosConT(solver, name + prefix + "_pos_t", this, i, node->graph->scale.pos_t);
				foot[i].con_pos_r = new BipedFootPosConR(solver, name + prefix + "_pos_r", this, i, node->graph->scale.pos_r);
				foot[i].con_cop_pos = new BipedFootCopPosCon(solver, name + prefix + "_cop_pos", this, i, node->graph->scale.pos_t);
				foot[i].con_cop_vel = new BipedFootCopVelCon(solver, name + prefix + "_cop_vel", this, i, node->graph->scale.vel_t);
				solver->AddTransitionCon(foot[i].con_pos_t, tick->idx);
				solver->AddTransitionCon(foot[i].con_pos_r, tick->idx);
				solver->AddTransitionCon(foot[i].con_cop_pos, tick->idx);
				solver->AddTransitionCon(foot[i].con_cop_vel, tick->idx);

				foot[i].con_vel_zero_t = new FixConV3(solver, ID(ConTag::BipedFootVelZeroT, node, tick, name + prefix + "_vel_zero_t"), foot[i].var_vel_t, node->graph->scale.vel_t);
				foot[i].con_vel_zero_r = new FixConS(solver, ID(ConTag::BipedFootVelZeroT, node, tick, name + prefix + "_vel_zero_r"), foot[i].var_vel_r, node->graph->scale.vel_r);
				foot[i].con_cop_vel_diff_zero = new FixConV3(solver, ID(ConTag::BipedFootVelZeroT, node, tick, name + prefix + "_cop_vel_diff_zero"), foot[i].var_cop_vel_diff, node->graph->scale.vel_t);

				foot[i].con_vel_zero_t->desired = vec3_t();
				foot[i].con_vel_zero_r->desired = 0.0;
				foot[i].con_cop_vel_diff_zero->desired = vec3_t();
				foot[i].con_cop_vel_diff_zero->weight = 0.1 * one;

				solver->AddCostCon(foot[i].con_vel_zero_t, tick->idx);
				solver->AddCostCon(foot[i].con_vel_zero_r, tick->idx);
				solver->AddCostCon(foot[i].con_cop_vel_diff_zero, tick->idx);
			}

			for (int d = 0; d < 2; d++) {
				real_t sign = (d == 0 ? 1.0 : -1.0);
				for (int j = 0; j < 3; j++) {
					vec3_t dir;
					dir[j] = sign;
					foot[i].con_pos_range_t[j][d] = new BipedFootPosRangeConT(solver, name + prefix + "_range_t", this, i, dir, node->graph->scale.pos_t);
					solver->AddCostCon(foot[i].con_pos_range_t[j][d], tick->idx);

					foot[i].con_cop_range[j][d] = new BipedFootCopRangeCon(solver, name + prefix + "_cop_range", this, i, dir, node->graph->scale.pos_t);
					solver->AddCostCon(foot[i].con_cop_range[j][d], tick->idx);
				}
				foot[i].con_pos_range_r[d] = new BipedFootPosRangeConR(solver, name + prefix + "_range_r", this, 0, sign, node->graph->scale.pos_r);
				solver->AddCostCon(foot[i].con_pos_range_r[d], tick->idx);
			}
		}
	}

	//-------------------------------------------------------------------------------------------------
	// BipedLIP

	BipedRunning::Param::Param() {
		gravity = vec3_t(0.0, 0.0, 9.8);
		comHeight = 1.0;
		T[0] = sqrt(comHeight/gravity.z);
		T[1] = sqrt(comHeight/gravity.z);
		T[2] = sqrt(comHeight/gravity.z);
		T[3] = sqrt(comHeight/gravity.z);
		torsoMass = 10.0;
		footMass = 5.0;
		//gaitType = GaitType::Walk;
		swingProfile = SwingProfile::Cycloid;
		swingInterpolation = SwingInterpolation::Cubic;
		swingHeight= 0.1; //0: maximum swing foot height
		durationMin[Phase::R] = 0.1; // duration minimum at single support
		durationMax[Phase::R] = 0.8; // duration maximum at single support
		durationMin[Phase::L] = 0.1;
		durationMax[Phase::L] = 0.8;
		durationMin[Phase::RL] = 0.1; // duration minimum at double support
		durationMax[Phase::RL] = 0.2; // duration maximum at double support
		durationMin[Phase::LR] = 0.1;
		durationMax[Phase::LR] = 0.2;
		durationMin[Phase::LRF] = 0.1;
		durationMin[Phase::RLF] = 0.1;
		durationMax[Phase::LRF] = 0.2;
		durationMax[Phase::RLF] = 0.2;
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

		// range of cop relative to foot
		footCopMin[0] = vec3_t(-0.1, -0.05, 0.0);
		footCopMax[0] = vec3_t(0.1, 0.05, 0.0);
		footCopMin[1] = vec3_t(-0.1, -0.05, 0.0);
		footCopMax[1] = vec3_t(0.1, 0.05, 0.0);

		// range of com acceleration
		/*accMin = vec3_t(-1.0, 1.0, 0.0);
		accMax = vec3_t(-1.0, 1.0, 0.0);*/

		// range of angular momentum
		/*momMin = vec3_t(-1.0, 1.0, 0.0);
		momMax = vec3_t(-1.0, 1.0, 0.0);*/

	}

	//-------------------------------------------------------------------------------------------------

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

	BipedRunning::BipedRunning(Graph* g, string n) : BipedLIP(g, n) {
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
		for (int k = 0; k < graph->ticks.size(); k++) {
			BipedLIPKey* key = (BipedLIPKey*)traj.GetKeypoint(graph->ticks[k]);

			key->tick->time = t;
			key->var_time->val = t;

			int ph = phase[key->tick->idx];

			if (!key->prev) {
				// initial time is fixed
				key->var_time->val = 0.0;
				key->var_time->locked = true;
			}

			if (key->next) {
				// foot velocity must be zero while in contact
				key->foot[0].con_vel_zero_t->enabled = (ph != BipedLIP::Phase::L);
				key->foot[0].con_vel_zero_r->enabled = (ph != BipedLIP::Phase::L);
				key->foot[1].con_vel_zero_t->enabled = (ph != BipedLIP::Phase::R);
				key->foot[1].con_vel_zero_r->enabled = (ph != BipedLIP::Phase::R);

				// cop velocity must be constant (i.e., acceleration must be zero) while in contact
				key->foot[0].con_cop_vel_diff_zero->enabled = (ph == BipedLIP::Phase::LR || ph == BipedLIP::Phase::R);
				key->foot[1].con_cop_vel_diff_zero->enabled = (ph == BipedLIP::Phase::RL || ph == BipedLIP::Phase::L);
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
					key->foot[i].con_pos_range_t[j][0]->bound = param.footPosMin[i][j];
					key->foot[i].con_pos_range_t[j][1]->bound = -param.footPosMax[i][j];
				}
				key->foot[i].con_pos_range_r[0]->bound = param.footOriMin[i];
				key->foot[i].con_pos_range_r[1]->bound = -param.footOriMax[i];

				for (int j = 0; j < 3; j++) {
					key->foot[i].con_cop_range[j][0]->bound = param.footCopMin[i][j];
					key->foot[i].con_cop_range[j][1]->bound = -param.footCopMax[i][j];
				}
			}

			t += durationAve[phase[k]];

			// set weight of feet
			if (key->next) {
				BipedLIPKey* key0 = key;
				BipedLIPKey* key1 = (BipedLIPKey*)key->next;

				if (phase[k] == Phase::R) {
					key0->foot[0].weight = 1.0;
					key1->foot[0].weight = 1.0;
					key0->foot[1].weight = 0.0;
					key1->foot[1].weight = 0.0;
				}
				if (phase[k] == Phase::L) {
					key0->foot[0].weight = 0.0;
					key1->foot[0].weight = 0.0;
					key0->foot[1].weight = 1.0;
					key1->foot[1].weight = 1.0;
				}
				if (phase[k] == Phase::D) {
					key0->foot[0].weight = 0.5;
					key1->foot[0].weight = 0.5;
					key0->foot[1].weight = 0.5;
					key1->foot[1].weight = 0.5;
				}
			}

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

		int idx = 0;
		for (Waypoint& wp : waypoints) {
			BipedLIPKey* key = (BipedLIPKey*)traj.GetKeypoint(graph->ticks[wp.k]);
			real_t t = key->var_time->val;

			curve_com.AddPoint(t);
			curve_com.SetPos(idx, wp.com_pos);
			curve_com.SetVel(idx, wp.com_vel);

			curve_torso_r.AddPoint(t);
			curve_torso_r.SetPos(idx, wp.torso_pos_r);

			for (int i = 0; i < 2; i++) {
				curve_foot_t[i].AddPoint(t);
				curve_foot_t[i].SetPos(idx, wp.foot_pos_t[i]);
				curve_foot_t[i].SetVel(idx, wp.com_vel);

				curve_foot_r[i].AddPoint(t);
				curve_foot_r[i].SetPos(idx, wp.foot_pos_r[i]);
				curve_foot_r[i].SetVel(idx, 0.0);
			}

			idx++;
		}

		for (int k = 0; k < graph->ticks.size(); k++) {
			BipedLIPKey* key = (BipedLIPKey*)traj.GetKeypoint(graph->ticks[k]);
			real_t t = key->var_time->val;

			key->var_com_pos->val = curve_com.CalcPos(t);
			key->var_com_vel->val = curve_com.CalcVel(t);

			key->var_torso_pos_r->val = curve_torso_r.CalcPos(t);

			for (int i = 0; i < 2; i++) {
				key->foot[i].var_pos_t->val = curve_foot_t[i].CalcPos(t);
				key->foot[i].var_pos_r->val = curve_foot_r[i].CalcPos(t);
				key->foot[i].var_cop_pos->val = key->foot[i].var_pos_t->val;

				if (key->next) {
					key->foot[i].var_vel_t->val = curve_foot_t[i].CalcVel(t);
					key->foot[i].var_vel_r->val = curve_foot_r[i].CalcVel(t);
					key->foot[i].var_cop_vel->val = curve_foot_t[i].CalcVel(t);
				}
			}

			real_t mt = param.torsoMass;
			real_t mf = param.footMass;
			key->var_torso_pos_t->val = (1.0 + 2.0 * (mf / mt)) * key->var_com_pos->val - (mf / mt) * (key->foot[0].var_pos_t->val + key->foot[1].var_pos_t->val);
			key->var_torso_vel_t->val = (1.0 + 2.0 * (mf / mt)) * key->var_com_vel->val;
		}

		// variables at waypoints are fixed
		for (Waypoint& wp : waypoints) {
			BipedLIPKey* key = (BipedLIPKey*)traj.GetKeypoint(graph->ticks[wp.k]);

			key->var_com_pos->locked = wp.fix_com_pos;
			key->var_com_vel->locked = wp.fix_com_vel;
			key->var_torso_pos_r->locked = wp.fix_torso_pos_r;

			for (int i = 0; i < 2; i++) {
				key->foot[i].var_pos_t->locked = wp.fix_foot_pos_t[i];
				key->foot[i].var_pos_r->locked = wp.fix_foot_pos_r[i];
				key->foot[i].var_cop_pos->locked = wp.fix_foot_cop[i];

				if (wp.set_cop_range[i]) {
					for (int j = 0; j < 3; j++) {
						key->foot[i].con_cop_range[j][0]->bound = wp.foot_cop_min[i][j];
						key->foot[i].con_cop_range[j][1]->bound = -wp.foot_cop_max[i][j];
					}
				}
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

	bool BipedRunning::OnTransition(real_t t) {
		BipedRunKey* key0 = (BipedRunKey*)traj.GetSegment(t).first;
		BipedRunKey* key1 = (BipedRunKey*)traj.GetSegment(t).second;
		BipedRunKey* keym1 = 0;

		if (key0->prev) {
			keym1 = (BipedRunKey*)key0->prev;
			if (gaittype[key0->tick->idx] != gaittype[key1->tick->idx] || gaittype[key0->tick->idx] != gaittype[keym1->tick->idx]) {
				return true;
			}
		}
		return false;
	}

	void BipedRunning::ComState(real_t t, vec3_t& pos, vec3_t& vel, vec3_t& acc) {
		BipedRunKey* key0 = (BipedRunKey*)traj.GetSegment(t).first;
		BipedRunKey* key1 = (BipedRunKey*)traj.GetSegment(t).second;
		int gtype = gaittype[key0->tick->idx];
		int ph = phase[key0->tick->idx];

		real_t dt = t - key0->var_time->val;
		real_t T;
		if (OnTransition(t) && gtype == GaitType::Walk)
			T = param.T[0]; //0.3316;
		else if (OnTransition(t) && gtype == GaitType::Run)
			T = param.T[1]; //0.2659 0.2807;
		else if (gtype == GaitType::Run)
			T = param.T[2];
		else
			T = param.T[3];

		real_t T2 = T * T;

		if (key1 == key0->next) {
			vec3_t p0 = key0->var_com_pos->val;
			vec3_t v0 = key0->var_com_vel->val;
			vec3_t c0 = key0->cop_pos + vec3_t(0.0, 0.0, param.gravity.z * T * T);
			vec3_t cv0 = key0->cop_vel;

			if (ph == Phase::LRF || ph == Phase::RLF) {
				pos = p0 + v0 * dt - 0.5 * param.gravity * dt * dt;
				vel = v0 - param.gravity * dt;
				acc = -param.gravity;
			}
			else {
				pos = c0 + cv0 * dt + cosh(dt / T) * (p0 - c0) + T * sinh(dt / T) * (v0 - cv0);
				vel = cv0 + (1 / T) * sinh(dt / T) * (p0 - c0) + cosh(dt / T) * (v0 - cv0);
				acc = (1 / T2) * cosh(dt / T) * (p0 - c0) + (1 / T) * sinh(dt / T) * (v0 - cv0);

			}
		}
		else {
			pos = key0->var_com_pos->val;
			vel = key0->var_com_vel->val;
			acc = vec3_t();
		}
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
			pose.Pos() = key0->foot[side].var_pos_t->val;
			pose.Ori() = FromRollPitchYaw(vec3_t(0.0, 0.0, key0->foot[side].var_pos_r->val));

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
		real_t tau0 = key0->var_duration->val;  //< phase duration
		real_t t1 = t0 + tau0;
		real_t dt = std::max(t - t0, 0.0);    //< elapsed time since phase change
		real_t s = dt / tau0;                   //< normalized time
		vec3_t p0 = key0->foot[side].var_pos_t->val;
		vec3_t v0 = key0->var_com_vel->val;
		vec3_t p1 = key1->foot[side].var_pos_t->val;
		real_t yaw0 = key0->foot[side].var_pos_r->val;
		real_t yaw1 = key1->foot[side].var_pos_r->val;
		real_t h0 = param.swingHeight;

		if (param.swingProfile == SwingProfile::Cycloid) {
			WALK:
			if (gaittype[key0->tick->idx] == GaitType::Walk) {
				real_t tau = key0->var_duration->val;

				if (ph == Phase::LR || ph == Phase::RL || ph == Phase::D) {
					pos = p0;

					contact = ContactState::Surface;
				}
				// support foot of single support
				if ((ph == Phase::R && side == 0) ||
					(ph == Phase::L && side == 1)) {
					pos = p0;

					contact = ContactState::Surface;
				}
				// swing foot of single support
				if ((ph == Phase::R && side == 1) ||
					(ph == Phase::L && side == 0)) {
					real_t ch = (s - sin(_2pi * s) / _2pi);
					real_t cv = (1 - cos(_2pi * s)) / 2.0;

					pos = vec3_t(
						p0.x + (p1.x - p0.x) * ch,
						p0.y + (p1.y - p0.y) * ch,
						p0.z + (p1.z - p0.z) * ch + h0 * cv
					);
					vel = vec3_t(
						((p1.x - p0.x) / tau) * (1.0 - cos(_2pi * s)),
						((p1.y - p0.y) / tau) * (1.0 - cos(_2pi * s)),
						((p1.z - p0.z) / tau) * (1.0 - cos(_2pi * s)) + (h0 / (2.0 * tau)) * _2pi * sin(_2pi * s)
					);
					acc = vec3_t(
						((p1.x - p0.x) / (tau * tau)) * _2pi * sin(_2pi * s),
						((p1.y - p0.y) / (tau * tau)) * _2pi * sin(_2pi * s),
						((p1.z - p0.z) / (tau * tau)) * _2pi * sin(_2pi * s) + (h0 / (2.0 * tau * tau)) * (_2pi * _2pi) * cos(_2pi * s)
					);

					contact = ContactState::Float;
				}
			}

			else {
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
				if ((ph == Phase::R   && keym1 && gaittype[keym1->tick->idx] == GaitType::Walk && side == 1) ||
					(ph == Phase::L   && keym1 && gaittype[keym1->tick->idx] == GaitType::Walk && side == 0) ||
					(ph == Phase::RLF && keym2 && gaittype[keym2->tick->idx] == GaitType::Walk && side == 1) ||
					(ph == Phase::LRF && keym2 && gaittype[keym2->tick->idx] == GaitType::Walk && side == 0)) {

					real_t tau1;
					if (ph == Phase::R || ph == Phase::L)
					{
						p1 = key2->foot[side].var_pos_t->val;
						tau1 = key1->var_duration->val;
						s = dt / (tau0 + tau1);
					}
					if ((ph == Phase::RLF || ph == Phase::LRF) && keym1) {
						p0 = keym1->foot[side].var_pos_t->val;
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
				else if ((ph == Phase::RLF && key2 && gaittype[key2->tick->idx] == GaitType::Walk && side == 0) ||
					     (ph == Phase::LRF && key2 && gaittype[key2->tick->idx] == GaitType::Walk && side == 1) ||
					     (ph == Phase::L   &&         gaittype[key1->tick->idx] == GaitType::Walk && side == 0) ||
					     (ph == Phase::R   &&         gaittype[key1->tick->idx] == GaitType::Walk && side == 1)) {

					real_t tau1 = 0;
					if (ph == Phase::RLF || ph == Phase::LRF) {
						p1 = key2->foot[side].var_pos_t->val;
						tau1 = key1->var_duration->val;
						s = dt / (tau0 + tau1);
					}
					if ((ph == Phase::R || ph == Phase::L) && keym1) {
						p0 = keym1->foot[side].var_pos_t->val;
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
				else if ((ph == Phase::RLF) || (ph == Phase::LRF) || (ph == Phase::L && side == 0) || (ph == Phase::R && side == 1)) {

					if ((ph == Phase::RLF && side == 0) ||
						(ph == Phase::LRF && side == 1)) {
						if (key3) p1 = key3->foot[side].var_pos_t->val;
					}
					if ((ph == Phase::L && side == 0) ||
						(ph == Phase::R && side == 1)) {
						if (keym1) {
							p0 = keym1->foot[side].var_pos_t->val;
							dt = std::max(t - keym1->var_time->val, 0.0);
						}
						if (key2) p1 = key2->foot[side].var_pos_t->val;
					}
					if ((ph == Phase::RLF && side == 1) ||
						(ph == Phase::LRF && side == 0)) {
						if (keym2) {
							p0 = keym2->foot[side].var_pos_t->val;
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
		}

		// experimental profile (may not work correctly)	
		else if (param.swingProfile == SwingProfile::Experiment) {
			if (gaittype[key0->tick->idx] == GaitType::Walk) goto WALK;

			// double support
			if (ph == Phase::D) {
				pos = p0;

				contact = ContactState::Surface;
			}
			// support foot of single support
			if ((ph == Phase::R && side == 0) ||
				(ph == Phase::L && side == 1)) {
				pos = p0;
				contact = ContactState::Surface;
			}
			// first swing foot
			if ((ph == Phase::R   && keym1 && gaittype[keym1->tick->idx] == GaitType::Walk && side == 1) ||
				(ph == Phase::L   && keym1 && gaittype[keym1->tick->idx] == GaitType::Walk && side == 0) ||
				(ph == Phase::RLF && keym2 && gaittype[keym2->tick->idx] == GaitType::Walk && side == 1) ||
				(ph == Phase::LRF && keym2 && gaittype[keym2->tick->idx] == GaitType::Walk && side == 0)) {

				real_t tau1;
				if (ph == Phase::R || ph == Phase::L)
				{
					p1 = key2->foot[side].var_pos_t->val;
					tau1 = key1->var_duration->val;
				}
				if ((ph == Phase::RLF || ph == Phase::LRF) && keym1) {
					p0 = keym1->foot[side].var_pos_t->val;
					dt = std::max(t - keym1->var_time->val, 0.0);
					tau1 = keym1->var_duration->val;
					t0 = keym1->var_time->val;
					//v0 = keym1->var_com_vel->val;
				}

				Interpolate(t, pos.x, vel.x, acc.x, t0, p0.x, 0.0, 0.0, t0 + tau0 + tau1, p1.x, 0.0, 0.0, param.swingInterpolation);
				Interpolate(t, pos.y, vel.y, acc.y, t0, p0.y, 0.0, 0.0, t0 + tau0 + tau1, p1.y, 0.0, 0.0, param.swingInterpolation);
				Interpolate2(t, pos.z, vel.z, acc.z, t0, t0 + tau0 + tau1, h0, param.swingInterpolation);

				contact = ContactState::Float;
			}
			//last swing foot
			else if ((ph == Phase::RLF && key2 && gaittype[key2->tick->idx] == GaitType::Walk && side == 0) ||
				     (ph == Phase::LRF && key2 && gaittype[key2->tick->idx] == GaitType::Walk && side == 1) ||
				     (ph == Phase::L   &&         gaittype[key1->tick->idx] == GaitType::Walk && side == 0) ||
				     (ph == Phase::R   &&         gaittype[key1->tick->idx] == GaitType::Walk && side == 1)) {

				real_t tau1;
				if (ph == Phase::RLF || ph == Phase::LRF) {
					p1 = key2->foot[side].var_pos_t->val;
					tau1 = key1->var_duration->val;
				}
				if ((ph == Phase::R || ph == Phase::L) && keym1) {
					p0 = keym1->foot[side].var_pos_t->val;
					dt = std::max(t - keym1->var_time->val, 0.0);
					tau1 = keym1->var_duration->val;
					t0 = keym1->var_time->val;
					v0 = keym1->var_com_vel->val;
				}

				Interpolate(t, pos.x, vel.x, acc.x, t0, p0.x, 0.0, 0.0, t0 + tau0 + tau1, p1.x, 0.0, 0.0, param.swingInterpolation);
				Interpolate(t, pos.y, vel.y, acc.y, t0, p0.y, 0.0, 0.0, t0 + tau0 + tau1, p1.y, 0.0, 0.0, param.swingInterpolation);
				Interpolate2(t, pos.z, vel.z, acc.z, t0, t0 + tau0 + tau1, h0, param.swingInterpolation);
				contact = ContactState::Float;
			}
			//swing foot
			else if ((ph == Phase::RLF) || (ph == Phase::LRF) || (ph == Phase::L && side == 0) || (ph == Phase::R && side == 1)) {
				if ((ph == Phase::RLF && side == 0) ||
					(ph == Phase::LRF && side == 1)) {
					Interpolate(t, pos.z, vel.z, acc.z, t0, p0.z, 0.0, 0.0, t0 + tau0, h0, 0.0, -9.8, param.swingInterpolation);
					if (key3) p1 = key3->foot[side].var_pos_t->val;
				}
				if ((ph == Phase::L && side == 0) ||
					(ph == Phase::R && side == 1)) {
					if (keym1) {
						Interpolate(t, pos.z, vel.z, acc.z, t0, h0, 0.0, -9.8, t0 + tau0, 0.7*h0, 0.0, 0.0, param.swingInterpolation);
						p0 = keym1->foot[side].var_pos_t->val;
						dt = std::max(t - keym1->var_time->val, 0.0);
						t0 = keym1->var_time->val;
						v0 = keym1->var_com_vel->val;
					}
					if (key2) p1 = key2->foot[side].var_pos_t->val;
				}
				if ((ph == Phase::RLF && side == 1) ||
					(ph == Phase::LRF && side == 0)) {
					if (keym2) {
						Interpolate(t, pos.z, vel.z, acc.z, t0, 0.7 * h0, 0.0, 0.0, t0 + tau0, p1.z, 0.0, 0.0, param.swingInterpolation);
						p0 = keym2->foot[side].var_pos_t->val;
						dt = std::max(t - keym2->var_time->val, 0.0);
						t0 = keym2->var_time->val;
						v0 = keym2->var_com_vel->val;
					}
				}
				real_t tau1 = key1->var_duration->val;
				real_t ptime = (ph == Phase::R || ph == Phase::L ? (tau0 + tau1 * 2) : (tau0 * 2 + tau1));
				Interpolate(t, pos.x, vel.x, acc.x, t0, p0.x, 0.0, 0.0, t0 + ptime, p1.x, 0.0, 0.0, param.swingInterpolation);
				Interpolate(t, pos.y, vel.y, acc.y, t0, p0.y, 0.0, 0.0, t0 + ptime, p1.y, 0.0, 0.0, param.swingInterpolation);
				//Interpolate(t, pos.z, vel.z, acc.z, t0, p0.z, v0.z, 0.0, t0 + ptime, p1.z, 0.0, 0.0, param.swingInterpolation);

				real_t dpos_z, dvel_z, dacc_z;
				//Interpolate2(t, dpos_z, dvel_z, dacc_z, t0, t0 + ptime, h0, param.swingInterpolation);
				/*pos.z += dpos_z;
				vel.z += dvel_z;
				acc.z += dacc_z;*/

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

		vec3_t c0 = key0->cop_pos;

		if (key1 == key0->next) {
			real_t dt = t - key0->var_time->val;
			vec3_t cv0 = key0->cop_vel;

			ct = c0 + cv0 * dt;
		}
		else {
			ct = c0;
		}

		return ct;
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
		log << "t com_pos.x com_pos.y com_pos.z com_vel.x com_vel.y com_vel.z cop_pos.x cop_pos.y cop_pos.z foot_pos0.x foot_pos0.y foot_pos0.z foot_pos1.x foot_pos1.y foot_pos1.z time_const\n";
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
		for (int i = 0; i < 2; i++) {
			v[0] = vec3_t(param.footCopMin[i].x, param.footCopMin[i].y, 0.0);
			v[1] = vec3_t(param.footCopMin[i].x, param.footCopMax[i].y, 0.0);
			v[2] = vec3_t(param.footCopMax[i].x, param.footCopMax[i].y, 0.0);
			v[3] = vec3_t(param.footCopMax[i].x, param.footCopMin[i].y, 0.0);
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
	BipedRunCon::BipedRunCon(Solver* solver, int _tag, string _name, BipedRunKey* _obj, real_t _scale) :
		BipedLipCon(solver, _tag, _name, _obj, _scale) {
		obj[0] = _obj;
		obj[1] = (_obj->next ? (BipedRunKey*)_obj->next : 0);
	}

	RunnerLipPosCon::RunnerLipPosCon(Solver* solver, string _name, BipedRunKey* _obj, real_t _scale) :
		BipedRunCon(solver, ConTag::BipedLipPos, _name, _obj, _scale) {

		AddSLink(obj[1]->var_com_pos);
		AddSLink(obj[0]->var_com_pos);
		AddSLink(obj[0]->var_com_vel);
		AddSLink(obj[0]->foot[0].var_cop_pos);
		AddSLink(obj[0]->foot[1].var_cop_pos);
		AddSLink(obj[0]->foot[0].var_cop_vel);
		AddSLink(obj[0]->foot[1].var_cop_vel);
		AddC3Link(obj[0]->var_duration);
	}

	RunnerLipVelCon::RunnerLipVelCon(Solver* solver, string _name, BipedRunKey* _obj, real_t _scale) :
		BipedRunCon(solver, ConTag::BipedLipVel, _name, _obj, _scale) {

		AddSLink(obj[1]->var_com_vel);
		AddSLink(obj[0]->var_com_pos);
		AddSLink(obj[0]->var_com_vel);
		AddSLink(obj[0]->foot[0].var_cop_pos);
		AddSLink(obj[0]->foot[1].var_cop_pos);
		AddSLink(obj[0]->foot[0].var_cop_vel);
		AddSLink(obj[0]->foot[1].var_cop_vel);
		AddC3Link(obj[0]->var_duration);
	}

	RunnerComConP::RunnerComConP(Solver* solver, string _name, BipedRunKey* _obj, real_t _scale) :
		Constraint(solver, 2, ID(ConTag::BipedComPos, _obj->node, _obj->tick, _name), Constraint::Type::Equality, _scale) {

		obj = _obj;

		AddSLink(obj->var_com_pos);
		AddSLink(obj->var_torso_pos_t);
		AddSLink(obj->foot[0].var_pos_t);
		AddSLink(obj->foot[1].var_pos_t);
	}

	RunnerComConV::RunnerComConV(Solver* solver, string _name, BipedRunKey* _obj, real_t _scale) :
		Constraint(solver, 2, ID(ConTag::BipedComVel, _obj->node, _obj->tick, _name), Constraint::Type::Equality, _scale) {

		obj = _obj;

		AddSLink(obj->var_com_vel);
		AddSLink(obj->var_torso_vel_t);
		AddSLink(obj->foot[0].var_vel_t);
		AddSLink(obj->foot[1].var_vel_t);
	}

	RunnerFootPosConT::RunnerFootPosConT(Solver* solver, string _name, BipedRunKey* _obj, int _side, real_t _scale) :
		Constraint(solver, 3, ID(ConTag::BipedFootPosT, _obj->node, _obj->tick, _name), Constraint::Type::Equality, _scale) {

		obj[0] = _obj;
		obj[1] = (_obj->next ? (BipedRunKey*)_obj->next : 0);
		side = _side;

		AddSLink(obj[1]->foot[side].var_pos_t);
		AddSLink(obj[0]->foot[side].var_pos_t);
		AddSLink(obj[0]->foot[side].var_vel_t);
		AddC3Link(obj[0]->var_duration);
	}

	RunnerFootPosConR::RunnerFootPosConR(Solver* solver, string _name, BipedRunKey* _obj, int _side, real_t _scale) :
		Constraint(solver, 1, ID(ConTag::BipedFootPosR, _obj->node, _obj->tick, _name), Constraint::Type::Equality, _scale) {

		obj[0] = _obj;
		obj[1] = (_obj->next ? (BipedRunKey*)_obj->next : 0);
		side = _side;

		AddSLink(obj[1]->foot[side].var_pos_r);
		AddSLink(obj[0]->foot[side].var_pos_r);
		AddSLink(obj[0]->foot[side].var_vel_r);
		AddSLink(obj[0]->var_duration);
	}

	RunnerFootCopPosCon::RunnerFootCopPosCon(Solver* solver, string _name, BipedRunKey* _obj, int _side, real_t _scale) :
		Constraint(solver, 3, ID(ConTag::BipedFootCop, _obj->node, _obj->tick, _name), Constraint::Type::Equality, _scale) {

		obj[0] = _obj;
		obj[1] = (_obj->next ? (BipedRunKey*)_obj->next : 0);
		side = _side;

		AddSLink(obj[1]->foot[side].var_cop_pos);
		AddSLink(obj[0]->foot[side].var_cop_pos);
		AddSLink(obj[0]->foot[side].var_cop_vel);
		AddC3Link(obj[0]->var_duration);
	}

	RunnerFootCopVelCon::RunnerFootCopVelCon(Solver* solver, string _name, BipedRunKey* _obj, int _side, real_t _scale) :
		Constraint(solver, 3, ID(ConTag::BipedFootCop, _obj->node, _obj->tick, _name), Constraint::Type::Equality, _scale) {

		obj[0] = _obj;
		obj[1] = (_obj->next ? (BipedRunKey*)_obj->next : 0);
		side = _side;

		AddSLink(obj[1]->foot[side].var_cop_vel);
		AddSLink(obj[0]->foot[side].var_cop_vel);
		AddSLink(obj[0]->foot[side].var_cop_vel_diff);
	}

	RunnerFootPosRangeConT::RunnerFootPosRangeConT(Solver* solver, string _name, BipedRunKey* _obj, int _side, vec3_t _dir, real_t _scale) :
		Constraint(solver, 1, ID(ConTag::BipedFootPosRangeT, _obj->node, _obj->tick, _name), Constraint::Type::InequalityPenalty, _scale) {

		obj = _obj;
		side = _side;
		dir = _dir;

		AddR3Link(obj->foot[side].var_pos_t);
		AddR3Link(obj->var_torso_pos_t);
		AddSLink(obj->var_torso_pos_r);
	}

	RunnerFootCopRangeCon::RunnerFootCopRangeCon(Solver* solver, string _name, BipedRunKey* _obj, int _side, vec3_t _dir, real_t _scale) :
		Constraint(solver, 1, ID(ConTag::BipedFootCopRange, _obj->node, _obj->tick, _name), Constraint::Type::InequalityPenalty, _scale) {

		obj = _obj;
		side = _side;
		dir = _dir;

		AddR3Link(obj->foot[side].var_cop_pos);
		AddR3Link(obj->foot[side].var_pos_t);
		AddSLink(obj->foot[side].var_pos_r);
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
		//int gtype = ((BipedRunning*)obj[0]->node)->gaittype;
		if (ph == BipedRunning::Phase::LRF || ph == BipedRunning::Phase::RLF)
			obj[1]->var_com_pos->val = p0 + v0 * tau - 0.5 * g * tau * tau;
		else
			obj[1]->var_com_pos->val = c0 + cv0 * tau + C * (p0 - c0) + (S * T) * (v0 - cv0);
	}

	void RunnerLipVelCon::CalcLhs() {
		Prepare();
		BipedRunning::Param& param = ((BipedRunning*)obj[0]->node)->param;
		if (ph == BipedRunning::Phase::LRF || ph == BipedRunning::Phase::RLF)
			obj[1]->var_com_vel->val = v0 - g * tau;
		else
			obj[1]->var_com_vel->val = cv0 + (S / T) * (p0 - c0) + C * (v0 - cv0);
	}

	void RunnerTimeCon::CalcLhs() {
		obj[1]->var_time->val = obj[0]->var_time->val + obj[0]->var_duration->val;
	}

	//-------------------------------------------------------------------------------------------------

	void BipedRunCon::Prepare() {
		BipedRunning::Param& param = ((BipedRunning*)obj[0]->node)->param;
		std::vector<int>& phase = ((BipedRunning*)obj[0]->node)->phase;
		std::vector<int>& gaittype = ((BipedRunning*)obj[0]->node)->gaittype;
		ph = phase[obj[0]->tick->idx];
		gtype = gaittype[obj[0]->tick->idx];
		int gtype1 = gaittype[obj[1]->tick->idx];
		int gtypem1 = gtype;
		if (obj[0]->prev) gtypem1 = gaittype[obj[0]->prev->tick->idx];

		bool transition = (gtype != gtype1 || gtype != gtypem1);
		if (transition && gtype == BipedRunning::GaitType::Walk)
		{
			T = param.T[0]; //0.3144 for h=0.9; // 0.3316 for h=1.0, tau0=0.30, tau1=0.10
		}
		else if (transition && gtype == BipedRunning::GaitType::Run)
		{
			T = param.T[1]; //0.2659 for h=0.9; // 0.2807 for h=1.0, tau0=0.30, tau1=0.10
		}
		else if (gtype == BipedRunning::GaitType::Run)
		{
			T = param.T[2];
		}
		else T = param.T[3];
		g = param.gravity;
		ez = vec3_t(0.0, 0.0, 1.0);
		//L0 = obj[0]->var_mom->val;
		p0 = obj[0]->var_com_pos->val;
		v0 = obj[0]->var_com_vel->val;
		c0 = obj[0]->cop_pos + vec3_t(0.0, 0.0, T * T * g.z);  //< c0 is vrp

		if (obj[1]) {
			tau = obj[0]->var_duration->val;
			C = cosh(tau / T);
			S = sinh(tau / T);

			cv0 = obj[0]->cop_vel;

			p1 = obj[1]->var_com_pos->val;
			v1 = obj[1]->var_com_vel->val;
			c1 = obj[1]->cop_pos + vec3_t(0.0, 0.0, T * T * g.z);
		}
	}

	//-------------------------------------------------------------------------------------------------

	void RunnerLipPosCon::CalcCoef() {
		Prepare();
		if (gtype == BipedRunning::GaitType::Run && (ph == BipedRunning::Phase::LRF || ph == BipedRunning::Phase::RLF))
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
			((C3Link*)links[7])->SetCoef(-cv0 - (S / T) * (p0 - c0) - C * (v0 - cv0));
		}
	}

	void RunnerLipVelCon::CalcCoef() {
		Prepare();
		if (gtype == BipedRunning::GaitType::Run && (ph == BipedRunning::Phase::LRF || ph == BipedRunning::Phase::RLF))
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
			((C3Link*)links[7])->SetCoef(-(C / (T * T)) * (p0 - c0) - (S / T) * (v0 - cv0));
		}
	}

	void RunnerLipCopCon::CalcCoef() {
		Prepare();

		((SLink*)links[0])->SetCoef(1.0);
		((SLink*)links[1])->SetCoef(-1.0);
		((SLink*)links[2])->SetCoef(-tau);
		((C3Link*)links[3])->SetCoef(-cv0);
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

	void RunnerTimeCon::CalcCoef() {
		((SLink*)links[0])->SetCoef(1.0);
		((SLink*)links[1])->SetCoef(-1.0);
		((SLink*)links[2])->SetCoef(-1.0);
	}

	//-------------------------------------------------------------------------------------------------

	void RunnerLipPosCon::CalcDeviation() {
		if (gtype == BipedRunning::GaitType::Run && (ph == BipedRunning::Phase::LRF || ph == BipedRunning::Phase::RLF))
			y = p1 - (p0 + v0 * tau - 0.5 * g * tau * tau);
		else
			y = p1 - (c0 + cv0 * tau + C * (p0 - c0) + (T * S) * (v0 - cv0));
	}

	void RunnerLipVelCon::CalcDeviation() {
		if (gtype == BipedRunning::GaitType::Run && (ph == BipedRunning::Phase::LRF || ph == BipedRunning::Phase::RLF))
			y = v1 - (v0 - g * tau);
		else
			y = v1 - (cv0 + (S / T) * (p0 - c0) + C * (v0 - cv0));
	}

	//-------------------------------------------------------------------------------------------------

}