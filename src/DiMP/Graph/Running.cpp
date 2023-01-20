#include <DiMP/Graph/Running.h>
#include <DiMP/Graph/Graph.h>
#include <DiMP/Render/Config.h>
#include <DiMP/Render/Canvas.h>

#include <stdio.h>

#include <sbrollpitchyaw.h>

namespace DiMP {;

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
		var_torso_pos_r = new SVar (solver, ID(VarTag::BipedTorsoRP, node, tick, name + "_torso_rp"), node->graph->scale.pos_r);
		var_torso_vel_t = new V3Var(solver, ID(VarTag::BipedTorsoTV, node, tick, name + "_torso_tv"), node->graph->scale.vel_t);

		// CoM position and velocity
		var_com_pos = new V3Var(solver, ID(VarTag::BipedComP, node, tick, name + "_com_p"), node->graph->scale.pos_t);
		var_com_vel = new V3Var(solver, ID(VarTag::BipedComV, node, tick, name + "_com_v"), node->graph->scale.vel_t);

		// absolute time
		var_time = new SVar(solver, ID(VarTag::BipedTime, node, tick, name + "_time"), node->graph->scale.time);

		var_torso_pos_t->weight    = damping*one;
		var_torso_pos_r->weight[0] = damping;
		var_torso_vel_t->weight    = damping*one;

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

			foot[i].var_pos_t   = new V3Var(solver, ID(VarTag::BipedFootTP  , node, tick, name + prefix + "_foot_t"), node->graph->scale.pos_t);
			foot[i].var_pos_r   = new SVar (solver, ID(VarTag::BipedFootRP  , node, tick, name + prefix + "_foot_r"), node->graph->scale.pos_r);
			foot[i].var_cop_pos = new V3Var(solver, ID(VarTag::BipedFootCopP, node, tick, name + prefix + "_cop_p" ), node->graph->scale.pos_t);
			foot[i].var_cop_vel = new V3Var(solver, ID(VarTag::BipedFootCopV, node, tick, name + prefix + "_cop_v" ), node->graph->scale.vel_t);

			foot[i].var_pos_t  ->weight    = damping*one;
			foot[i].var_pos_r  ->weight[0] = damping;
			foot[i].var_cop_pos->weight    = 10*damping*one;
			foot[i].var_cop_vel->weight    = damping*one;

			foot[i].var_pos_r->locked = true;

			solver->AddStateVar(foot[i].var_pos_t  , tick->idx);
			solver->AddStateVar(foot[i].var_pos_r  , tick->idx);
			solver->AddStateVar(foot[i].var_cop_pos, tick->idx);
			solver->AddStateVar(foot[i].var_cop_vel, tick->idx);

			if (next) {
				foot[i].var_vel_t        = new V3Var(solver, ID(VarTag::BipedFootTV  , node, tick, name + prefix + "_foot_t"), node->graph->scale.vel_t);
				foot[i].var_vel_r        = new SVar (solver, ID(VarTag::BipedFootRV  , node, tick, name + prefix + "_foot_r"), node->graph->scale.vel_r);
				foot[i].var_cop_vel_diff = new V3Var(solver, ID(VarTag::BipedFootCopV, node, tick, name + prefix + "_cop_vd"), node->graph->scale.vel_t);

				foot[i].var_vel_t       ->weight    = damping*one;
				foot[i].var_vel_r       ->weight[0] = damping;
				foot[i].var_cop_vel_diff->weight    = damping*one;

				solver->AddInputVar(foot[i].var_vel_t       , tick->idx);
				solver->AddInputVar(foot[i].var_vel_r       , tick->idx);
				solver->AddInputVar(foot[i].var_cop_vel_diff, tick->idx);
			}
		}

		// CMP position and velocity
		var_cmp_pos = new V3Var(solver, ID(VarTag::CentroidTP, node, tick, name + "_cmp_p"), node->graph->scale.pos_t);
		var_cmp_vel = new V3Var(solver, ID(VarTag::CentroidTV, node, tick, name + "_cmp_v"), node->graph->scale.vel_t);
		var_cmp_pos->weight = damping * one;
		var_cmp_vel->weight = damping * one;
		solver->AddStateVar(var_cmp_pos, tick->idx);
		solver->AddStateVar(var_cmp_vel, tick->idx);


		var_mom = new V3Var(solver, ID(VarTag::CentroidTP, node, tick, name + "_mom"), node->graph->scale.vel_t);
		var_mom->weight = damping * one;
		solver->AddStateVar(var_mom, tick->idx);
	}

	// register constraints for planning
	void BipedRunKey::AddCon(Solver* solver) {
		BipedRunning* obj = (BipedRunning*)node;
		BipedRunKey* nextObj = (BipedRunKey*)next;

		vec3_t one(1.0, 1.0, 1.0);

		if (next) {
			con_lip_pos = new RunnerLipPosCon(solver, name + "_lip_pos", this, node->graph->scale.pos_t);
			con_lip_vel = new RunnerLipVelCon(solver, name + "_lip_vel", this, node->graph->scale.vel_t);

			con_lip_cmp = new RunnerLipCmpCon(solver, name + "_lip_cmp", this, node->graph->scale.pos_t);
			con_lip_mom = new RunnerLipMomCon(solver, name + "_lip_mom", this, node->graph->scale.pos_t);

			con_time = new RunnerTimeCon(solver, name + "_time", this, node->graph->scale.time);

			con_duration_range = new RangeConS(solver, ID(ConTag::BipedDurationRange, node, tick, name + "_duration"), var_duration, node->graph->scale.time);

			//con_com_vel_zero = new FixConV3(solver, ID(ConTag::BipedFootVelZeroT, node, tick, name + "_com_vel_zero"), var_com_vel, node->graph->scale.vel_t);
			//con_com_vel_zero->weight = 0.01*one;

			solver->AddTransitionCon(con_lip_pos, tick->idx);
			solver->AddTransitionCon(con_lip_vel, tick->idx);
			solver->AddTransitionCon(con_lip_cmp, tick->idx);
			solver->AddTransitionCon(con_lip_mom, tick->idx);

			solver->AddTransitionCon(con_time, tick->idx);

			solver->AddCostCon(con_duration_range, tick->idx);

			//solver->AddCostCon(con_com_vel_zero, tick->idx);
		}

		con_com_pos = new RunnerComConP(solver, name + "_com_p", this, node->graph->scale.pos_t);
		solver->AddCostCon(con_com_pos, tick->idx);

		if (next) {
			con_com_vel = new RunnerComConV(solver, name + "_com_v", this, node->graph->scale.vel_t);
			solver->AddCostCon(con_com_vel, tick->idx);
		}

		for (int i = 0; i < 2; i++) {
			string prefix = (i == 0 ? "_foot_r" : "_foot_l");

			if (next) {
				foot[i].con_pos_t   = new RunnerFootPosConT  (solver, name + prefix + "_pos_t"  , this, i, node->graph->scale.pos_t);
				foot[i].con_pos_r   = new RunnerFootPosConR  (solver, name + prefix + "_pos_r"  , this, i, node->graph->scale.pos_r);
				foot[i].con_cop_pos = new RunnerFootCopPosCon(solver, name + prefix + "_cop_pos", this, i, node->graph->scale.pos_t);
				foot[i].con_cop_vel = new RunnerFootCopVelCon(solver, name + prefix + "_cop_vel", this, i, node->graph->scale.vel_t);
				solver->AddTransitionCon(foot[i].con_pos_t  , tick->idx);
				solver->AddTransitionCon(foot[i].con_pos_r  , tick->idx);
				solver->AddTransitionCon(foot[i].con_cop_pos, tick->idx);
				solver->AddTransitionCon(foot[i].con_cop_vel, tick->idx);

				foot[i].con_vel_zero_t        = new FixConV3(solver, ID(ConTag::BipedFootVelZeroT, node, tick, name + prefix + "_vel_zero_t"       ), foot[i].var_vel_t       , node->graph->scale.vel_t);
				foot[i].con_vel_zero_r        = new FixConS (solver, ID(ConTag::BipedFootVelZeroT, node, tick, name + prefix + "_vel_zero_r"       ), foot[i].var_vel_r       , node->graph->scale.vel_r);
				foot[i].con_cop_vel_diff_zero = new FixConV3(solver, ID(ConTag::BipedFootVelZeroT, node, tick, name + prefix + "_cop_vel_diff_zero"), foot[i].var_cop_vel_diff, node->graph->scale.vel_t);

				foot[i].con_vel_zero_t       ->desired = vec3_t();
				foot[i].con_vel_zero_r       ->desired = 0.0;
				foot[i].con_cop_vel_diff_zero->desired = vec3_t();
				foot[i].con_cop_vel_diff_zero->weight = 0.1*one;

				solver->AddCostCon(foot[i].con_vel_zero_t       , tick->idx);
				solver->AddCostCon(foot[i].con_vel_zero_r       , tick->idx);
				solver->AddCostCon(foot[i].con_cop_vel_diff_zero, tick->idx);
			}

			for (int d = 0; d < 2; d++) {
				real_t sign = (d == 0 ? 1.0 : -1.0);
				for (int j = 0; j < 3; j++) {
					vec3_t dir;
					dir[j] = sign;
					foot[i].con_pos_range_t[j][d] = new RunnerFootPosRangeConT(solver, name + prefix + "_range_t", this, i, dir, node->graph->scale.pos_t);
					solver->AddCostCon(foot[i].con_pos_range_t[j][d], tick->idx);

					foot[i].con_cop_range[j][d] = new RunnerFootCopRangeCon(solver, name + prefix + "_cop_range", this, i, dir, node->graph->scale.pos_t);
					solver->AddCostCon(foot[i].con_cop_range[j][d], tick->idx);
				}
				foot[i].con_pos_range_r[d] = new RunnerFootPosRangeConR(solver, name + prefix + "_range_r", this, 0, sign, node->graph->scale.pos_r);
				solver->AddCostCon(foot[i].con_pos_range_r[d], tick->idx);
			}
		}

		// cmp range constraint
		for (int d = 0; d < 2; d++) {
			real_t sign = (d == 0 ? 1.0 : -1.0);
			for (int j = 0; j < 3; j++) {
				vec3_t dir;
				dir[j] = sign;
				con_cmp_range[j][d] = new RunnerCmpRangeCon(solver, name + "_cmp_range", this, dir, node->graph->scale.pos_t);
				con_mom_range[j][d] = new RunnerMomRangeCon(solver, name + "_mom_range", this, dir, node->graph->scale.pos_t);
				solver->AddCostCon(con_cmp_range[j][d], tick->idx);
				solver->AddCostCon(con_mom_range[j][d], tick->idx);
			}
		}
	}

	void BipedRunKey::Prepare() {
		if (next) {
			// calc cop
			cop_pos.clear();
			cop_vel.clear();
			cop_acc.clear();

			real_t tau = var_duration->val;

			for (int i = 0; i < 2; i++) {
				real_t w0 = foot[i].weight;
				real_t w1 = ((BipedRunKey*)next)->foot[i].weight;
				vec3_t c0 = foot[i].var_cop_pos->val;
				vec3_t cv0 = foot[i].var_cop_vel->val;

				cop_pos += w0 * c0;
				cop_vel += w0 * cv0 + ((w1 - w0) / tau) * c0;
				cop_acc += (2.0 * (w1 - w0) / tau) * cv0;
			}
		}
	}

	void BipedRunKey::Finish() {
		// tick's time is updated from time variable
		tick->time = var_time->val;
	}

	void BipedRunKey::Draw(Render::Canvas* canvas, Render::Config* conf) {
		BipedRunning* obj = (BipedRunning*)node;

		Vec3f pcom, pcop, pf[2], pc[2], pt;
		float thetaf[2];

		canvas->SetPointSize(5.0f);
		canvas->SetLineWidth(1.0f);

		pcom.x = (float)var_com_pos->val.x;
		pcom.y = (float)var_com_pos->val.y;
		pcom.z = (float)var_com_pos->val.z;
		canvas->Point(pcom);

		// feet
		for (int i = 0; i < 2; i++) {
			// foot pos
			pf[i].x = (float)foot[i].var_pos_t->val.x;
			pf[i].y = (float)foot[i].var_pos_t->val.y;
			pf[i].z = (float)foot[i].var_pos_t->val.z;
			canvas->Point(pf[i]);

			// foot cop
			pc[i].x = (float)foot[i].var_cop_pos->val.x;
			pc[i].y = (float)foot[i].var_cop_pos->val.y;
			pc[i].z = (float)foot[i].var_cop_pos->val.z;
			canvas->Point(pc[i]);

			// foot print
			Vec3f cmin = obj->param.footCopMin[i];
			Vec3f cmax = obj->param.footCopMax[i];
			canvas->Push();
			thetaf[i] = (float)foot[i].var_pos_r->val;
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
		pcop = cop_pos;
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
		T[3] = sqrt(comHeight/gravity.z);
		torsoMass = 10.0;
		footMass = 5.0;
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
		footPosMax[0] = vec3_t( 0.5, -0.00, 0.0);
		footPosMin[1] = vec3_t(-0.5,  0.00, 0.0);
		footPosMax[1] = vec3_t( 0.5,  0.20, 0.0);
		footOriMin[0] = Rad(-15.0);
		footOriMax[0] = Rad( 15.0);
		footOriMin[1] = Rad(-15.0);
		footOriMax[1] = Rad( 15.0);

		// range of cop relative to foot
		footCopMin[0] = vec3_t(-0.1, -0.05, 0.0);
		footCopMax[0] = vec3_t( 0.1,  0.05, 0.0);
		footCopMin[1] = vec3_t(-0.1, -0.05, 0.0);
		footCopMax[1] = vec3_t( 0.1,  0.05, 0.0);

		// range of cmp relative to cop (zmp)
		cmpMin = vec3_t(0.0, 0.0, 0.0);
		cmpMax = vec3_t(0.0, 0.0, 0.0);

		// range of centroidal momentum
		momMin = vec3_t(0.0, 0.0, 0.0);
		momMax = vec3_t(0.0, 0.0, 0.0);

		ankleToToe        = 0.1;
		ankleToHeel       = 0.1;
		toeCurvature      = 10.0;
		heelCurvature     = 10.0;
		toeCurvatureRate  = 0.0;
		heelCurvatureRate = 0.0;

		minSpacing = 0.0;
		swingMargin = 0.0;
	}

	//-------------------------------------------------------------------------------------------------
	// Waypoints
	BipedRunning::Waypoint::Waypoint() {
		k           = 0;
		time        = 0.0;
		com_pos     = vec3_t();
		com_vel     = vec3_t();
		cmp_pos     = vec3_t();
		mom         = vec3_t();
		torso_pos_r = 0.0;

		fix_com_pos     = false;
		fix_com_vel     = false;
		fix_cmp_pos     = false;
		fix_mom         = false;
		fix_torso_pos_r = false;

		for (int i = 0; i < 2; i++) {
			foot_pos_t  [i] = vec3_t();
			foot_pos_r  [i] = 0.0;
			foot_cop    [i] = vec3_t();
			foot_cop_min[i] = vec3_t();
			foot_cop_max[i] = vec3_t();

			fix_foot_pos_t[i] = false;
			fix_foot_pos_r[i] = false;
			fix_foot_cop  [i] = false;
			set_cop_range [i] = false;
		}
	}
	//-------------------------------------------------------------------------------------------------
	BipedRunning::Snapshot::Snapshot() {
		t = 0.0;
		torso_pos_t = vec3_t();
		torso_pos_r = 0.0;
		com_pos = vec3_t();
		com_vel = vec3_t();
		com_acc = vec3_t();
		foot_pos_t[0] = vec3_t();
		foot_pos_r[0] = quat_t();
		foot_pos_t[1] = vec3_t();
		foot_pos_r[1] = quat_t();
	}

	//-------------------------------------------------------------------------------------------------

	BipedRunning::BipedRunning(Graph* g, string n) : TrajectoryNode(g, n) {
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
				// foot velocity must be zero while in contact
				key->foot[0].con_vel_zero_t->enabled = (ph != BipedRunning::Phase::L && ph != BipedRunning::Phase::LRF && ph != BipedRunning::Phase::RLF);
				key->foot[0].con_vel_zero_r->enabled = (ph != BipedRunning::Phase::L && ph != BipedRunning::Phase::LRF && ph != BipedRunning::Phase::RLF);
				key->foot[1].con_vel_zero_t->enabled = (ph != BipedRunning::Phase::R && ph != BipedRunning::Phase::LRF && ph != BipedRunning::Phase::RLF);
				key->foot[1].con_vel_zero_r->enabled = (ph != BipedRunning::Phase::R && ph != BipedRunning::Phase::LRF && ph != BipedRunning::Phase::RLF);

				// cop velocity must be constant (i.e., acceleration must be zero) while in contact
				key->foot[0].con_cop_vel_diff_zero->enabled = (ph == BipedRunning::Phase::LR || ph == BipedRunning::Phase::R);
				key->foot[1].con_cop_vel_diff_zero->enabled = (ph == BipedRunning::Phase::RL || ph == BipedRunning::Phase::L);
			}

			// initial value of step duration is set as the average of minimum and maximum
			if (key->next) {
				key->var_duration->val = durationAve[phase[k]];
				key->con_duration_range->_min = param.durationMin[phase[k]];
				key->con_duration_range->_max = param.durationMax[phase[k]];

				//key->var_duration->locked = true;
			}

			// range limit of foot position, CoP and CMP offset w.r.t. ZMP (not CoP of each foot)
			for (int i = 0; i < 2; i++) {
				for (int j = 0; j < 3; j++) {
					key->foot[i].con_pos_range_t[j][0]->bound =  param.footPosMin[i][j];
					key->foot[i].con_pos_range_t[j][1]->bound = -param.footPosMax[i][j];
				}
				key->foot[i].con_pos_range_r[0]->bound =  param.footOriMin[i];
				key->foot[i].con_pos_range_r[1]->bound = -param.footOriMax[i];

				for (int j = 0; j < 3; j++) {
					key->foot[i].con_cop_range[j][0]->bound =  param.footCopMin[i][j];
					key->foot[i].con_cop_range[j][1]->bound = -param.footCopMax[i][j];
				}
				for (int j = 0; j < 3; j++)
				{
					key->con_cmp_range[j][0]->bound =  param.cmpMin[j];
					key->con_cmp_range[j][1]->bound = -param.cmpMax[j];

					key->con_mom_range[j][0]->bound =  param.momMin[j];
					key->con_mom_range[j][1]->bound = -param.momMax[j];
				}
			}
		
			t += durationAve[phase[k]];

			// set weight of feet
			if (key->next) {
				BipedRunKey* key0 = key;
				BipedRunKey* key1 = (BipedRunKey*)key->next;

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

		curve_com      .SetType(Interpolate::Cubic);
		curve_torso_r  .SetType(Interpolate::LinearDiff);
		curve_foot_t[0].SetType(Interpolate::Cubic);
		curve_foot_r[0].SetType(Interpolate::Cubic);
		curve_foot_t[1].SetType(Interpolate::Cubic);
		curve_foot_r[1].SetType(Interpolate::Cubic);

		int idx = 0;
		for (Waypoint& wp : waypoints) {
			BipedRunKey* key = (BipedRunKey*)traj.GetKeypoint(graph->ticks[wp.k]);
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
			BipedRunKey* key = (BipedRunKey*)traj.GetKeypoint(graph->ticks[k]);
			real_t t = key->var_time->val;

			key->var_com_pos->val = curve_com.CalcPos(t);
			key->var_com_vel->val = curve_com.CalcVel(t);

			key->var_torso_pos_r->val = curve_torso_r.CalcPos(t);

			for (int i = 0; i < 2; i++) {
				key->foot[i].var_pos_t->val   = curve_foot_t[i].CalcPos(t);
				key->foot[i].var_pos_r->val   = curve_foot_r[i].CalcPos(t);
				key->foot[i].var_cop_pos->val = key->foot[i].var_pos_t->val;

				if (key->next) {
					key->foot[i].var_vel_t->val   = curve_foot_t[i].CalcVel(t);
					key->foot[i].var_vel_r->val   = curve_foot_r[i].CalcVel(t);
					key->foot[i].var_cop_vel->val = curve_foot_t[i].CalcVel(t);
				} 
			}

			key->var_cmp_vel->val.clear();

			real_t mt = param.torsoMass;
			real_t mf = param.footMass;
			key->var_torso_pos_t->val = (1.0 + 2.0 * (mf / mt)) * key->var_com_pos->val - (mf / mt) * (key->foot[0].var_pos_t->val + key->foot[1].var_pos_t->val);
			key->var_torso_vel_t->val = (1.0 + 2.0 * (mf / mt)) * key->var_com_vel->val;
		}

		// variables at waypoints are fixed
		for (Waypoint& wp : waypoints) {
			BipedRunKey* key = (BipedRunKey*)traj.GetKeypoint(graph->ticks[wp.k]);

			key->var_com_pos    ->locked = wp.fix_com_pos;
			key->var_com_vel    ->locked = wp.fix_com_vel;
			key->var_torso_pos_r->locked = wp.fix_torso_pos_r;

			for (int i = 0; i < 2; i++) {
				key->foot[i].var_pos_t  ->locked = wp.fix_foot_pos_t[i];
				key->foot[i].var_pos_r  ->locked = wp.fix_foot_pos_r[i];
				key->foot[i].var_cop_pos->locked = wp.fix_foot_cop[i];

				if (wp.set_cop_range[i]) {
					for (int j = 0; j < 3; j++) {
						key->foot[i].con_cop_range[j][0]->bound =  wp.foot_cop_min[i][j];
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
			T = param.T[0];
		else if (OnTransition(t) && gtype == GaitType::Run)
			T = param.T[1];
		else if (gtype == GaitType::Run)
			T = param.T[2];
		else
			T = param.T[3];

		real_t T2 = T * T;

		if (key1 == key0->next) {
			vec3_t p0 = key0->var_com_pos->val;
			vec3_t v0 = key0->var_com_vel->val;
			vec3_t c0 = key0->cop_pos + vec3_t(0.0, 0.0, param.gravity.z * T * T); // actually vrp
			vec3_t cv0 = key0->cop_vel;
			vec3_t ca0 = key0->cop_acc;
			vec3_t cm0 = key0->var_cmp_pos->val;
			vec3_t cmv0 = key0->var_cmp_vel->val;
			real_t C = cosh(dt / T);
			real_t S = sinh(dt / T);

			if (ph == Phase::LRF || ph == Phase::RLF) {
				pos = p0 + v0 * dt - 0.5 * param.gravity * dt * dt;
				vel = v0 - param.gravity * dt;
				acc = -param.gravity;
			}
			else {
				pos = c0 + cm0 + ca0*T2 + (cv0 + cmv0)*dt + (1.0 / 2.0)*ca0*dt*dt +        C*(p0 - (c0 + ca0*T2 + cm0)) +     T*S*(v0 - cv0 - cmv0);
				vel =               cv0 + cmv0            +             ca0*dt    + (1/T )*S*(p0 - (c0 + ca0*T2 + cm0)) +       C*(v0 - cv0 - cmv0);
				acc =                                                   ca0       + (1/T2)*C*(p0 - (c0 + ca0*T2 + cm0)) + (1/T)*S*(v0 - cv0 - cmv0);

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
			real_t dt   = t - key0->var_time->val;
			vec3_t L0   = key0->var_mom->val;
			vec3_t p0   = key0->var_com_pos->val;
	        vec3_t c0   = key0->cop_pos;
			vec3_t cm0  = key0->var_cmp_pos->val;
			vec3_t cmv0 = key0->var_cmp_vel->val;

			vec3_t pt, vt, at;
			ComState(t, pt, vt, at);

			//Lt = L0 + vec3_t(0.0, 0.0, 1.0) % (cm0 * dt + 0.5 * dt * dt * cmv0); // it works only if vertical acceleration is zero
			Lt = L0 + vec3_t(0.0, 0.0, 1.0) % ((g * cm0 + vt.z * cmv0) * dt + 0.5 * g * cmv0 * dt * dt);

			int ph = key0->var_duration->val;
			if (ph == Phase::RLF || ph == Phase::LRF) Lt = L0;
		}
		else {
			Lt = vec3_t();
		}

		// momentum is internally normalized, so multiply by m*g to obtain actual momentum
		return m * Lt;
	}

	vec3_t BipedRunning::CmpPos(real_t t) {
		BipedRunKey* key0 = (BipedRunKey*)traj.GetSegment(t).first;
		BipedRunKey* key1 = (BipedRunKey*)traj.GetSegment(t).second;

		vec3_t cmt;

		vec3_t c0 = key0->cop_pos;
		vec3_t cm0 = key0->var_cmp_pos->val;

		if (key1 == key0->next) {
			real_t dt = t - key0->var_time->val;
			vec3_t cv0 = key0->cop_vel;
			vec3_t ca0 = key0->cop_acc;
			vec3_t cmv0 = key0->var_cmp_vel->val;

			cmt = (c0 + cm0) + (cv0 + cmv0) * dt + 0.5 * ca0 * dt * dt;
		}
		else {
			cmt = (c0 + cm0);
		}

		return cmt;
		//return vec3_t(cmt.x, cmt.y, 0.0);
	}

	void BipedRunning::TorsoState(real_t t, real_t& ori, real_t& angvel, real_t& angacc) {
		BipedRunKey* key0 = (BipedRunKey*)traj.GetSegment(t).first;
		BipedRunKey* key1 = (BipedRunKey*)traj.GetSegment(t).second;

		if (key1 == key0->next) {
			real_t t0 = key0->var_time->val;
			real_t t1 = key1->var_time->val;

			ori = InterpolatePos(
				t,
				t0, key0->var_torso_pos_r->val, 0.0,
				t1, key1->var_torso_pos_r->val, 0.0,
				Interpolate::LinearDiff
			);
			angvel = InterpolateVel(
				t,
				t0, key0->var_torso_pos_r->val, 0.0,
				t1, key1->var_torso_pos_r->val, 0.0,
				Interpolate::LinearDiff
			);
		}
		else {
			ori = key0->var_torso_pos_r->val;
			angvel = 0.0;
		}
		angacc = 0.0;
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

	void BipedRunning::FootRotation(
		real_t cp, real_t cv, real_t ca,
		vec3_t& pos, vec3_t& vel, vec3_t& acc,
		vec3_t& ori, vec3_t& angvel, vec3_t& angacc,
		int& contact)
	{
		real_t l0 = param.ankleToToe;
		real_t l1 = param.ankleToHeel;
		real_t rho0 = param.toeCurvature;
		real_t rho1 = param.heelCurvature;
		real_t r0 = 1.0 / rho0;
		real_t r1 = 1.0 / rho1;
		real_t kappa0 = param.toeCurvatureRate;
		real_t kappa1 = param.heelCurvatureRate;

		real_t d;                 //< rolling distance
		vec2_t v, vd, vdd;        //< foot center to contact point on the foot, and its derivative w.r.t. d
		real_t phi, phid, phidd;  //< foot rotation and its derivative w.r.t. d

		phi = phid = phidd = 0.0;

		// current cop is on heel
		if (cp < -l1) {
			d = cp + l1;

			if (param.footCurveType == FootCurveType::Arc) {
				phi = d / r1;
				v = vec2_t(-l1 + r1 * sin(phi), r1 * (1.0 - cos(phi)));

				phid = 1.0 / r1;
				vd = vec2_t(cos(phi), sin(phi));

				phidd = 0.0;
				vdd = vec2_t(-sin(phi), cos(phi)) * phid;
			}
			if (param.footCurveType == FootCurveType::Clothoid) {
				phi = -0.5 * kappa1 * d * d;
				v = vec2_t(-l1 - clothoid_x(-d, kappa1), clothoid_y(-d, kappa1));

				phid = -kappa1 * d;
				vd = vec2_t(cos(phi), sin(phi));

				phidd = -kappa1;
				vdd = vec2_t(-sin(phi), cos(phi)) * phid;
			}

			contact = ContactState::Heel;
		}
		// current cop is on toe
		else if (cp > l0) {
			d = cp - l0;

			if (param.footCurveType == FootCurveType::Arc) {
				phi = d / r0;
				v = vec2_t(l0 + r0 * sin(phi), r0 * (1.0 - cos(phi)));

				phid = 1.0 / r0;
				vd = vec2_t(cos(phi), sin(phi));

				phidd = 0.0;
				vdd = vec2_t(-sin(phi), cos(phi)) * phid;
			}
			if (param.footCurveType == FootCurveType::Clothoid) {
				phi = 0.5 * kappa0 * d * d;
				v = vec2_t(l0 + clothoid_x(d, kappa0), clothoid_y(d, kappa0));

				phid = kappa0 * d;
				vd = vec2_t(cos(phi), sin(phi));

				phidd = kappa0;
				vdd = vec2_t(-sin(phi), cos(phi)) * phid;
			}

			contact = ContactState::Toe;
		}
		// current cop is in the middle
		else {
			v = vec2_t(cp, 0.0);
			vd = vec2_t(1.0, 0.0);
			vdd = vec2_t(0.0, 0.0);
			phi = phid = phidd = 0.0;

			contact = ContactState::Surface;
		}

		vec2_t pos2 = -mat2_t::Rot(-phi) * v;
		vec2_t vel2 = (-mat2_t::Rot(-phi) * vd + mat2_t::Rot(-phi + (pi / 2.0)) * v * phid) * cv;
		vec2_t acc2 = (-mat2_t::Rot(-phi) * vd + mat2_t::Rot(-phi + (pi / 2.0)) * v * phid) * ca;
		(-mat2_t::Rot(-phi) * (vdd - v * phid * phid) + mat2_t::Rot(-phi + (pi / 2.0)) * (2.0 * vd * phid + v * phidd))* (cv * cv);

		pos = vec3_t(pos2[0] + cp, 0.0, pos2[1]);
		vel = vec3_t(vel2[0] + cv, 0.0, vel2[1]);
		acc = vec3_t(acc2[0] + ca, 0.0, acc2[1]);
		ori = vec3_t(0.0, phi, 0.0);
		angvel = vec3_t(0.0, phid * cv, 0.0);
		angacc = vec3_t(0.0, phidd * cv * cv + phid * ca, 0.0);
	}

	void BipedRunning::FootRotationInv(
		real_t ori, real_t angvel, real_t angacc,
		real_t& cp, real_t& cv, real_t& ca) {

		real_t phi, phid, phidd;
		real_t d;
		real_t l0 = param.ankleToToe;
		real_t l1 = param.ankleToHeel;
		real_t rho0 = param.toeCurvature;
		real_t rho1 = param.heelCurvature;
		real_t r0 = 1.0 / rho0;
		real_t r1 = 1.0 / rho1;
		real_t kappa0 = param.toeCurvatureRate;
		real_t kappa1 = param.heelCurvatureRate;

		phi = ori;
		const real_t eps = 1.0e-2;
		if (std::abs(phi) < eps) {
			cp = 0.0;
			cv = 0.0;
			ca = 0.0;
		}
		else {
			if (phi < 0.0) {
				if (param.footCurveType == FootCurveType::Arc) {
					d = r1 * phi;
					phid = 1.0 / r1;
					phidd = 0.0;
				}
				if (param.footCurveType == FootCurveType::Clothoid) {
					d = -sqrt(-2.0 * phi / kappa1);
					phid = -kappa1 * d;
					phidd = -kappa1;
				}
				cp = -l1 + d;
				cv = angvel / phid;
				ca = (angacc - phidd * angvel * angvel) / phid;
			}
			else {
				if (param.footCurveType == FootCurveType::Arc) {
					d = r0 * phi;
					phid = 1.0 / r0;
					phidd = 0.0;
				}
				if (param.footCurveType == FootCurveType::Clothoid) {
					d = sqrt(2.0 * phi / kappa0);
					phid = kappa0 * d;
					phidd = kappa0;
				}
				cp = l0 + d;
				cv = angvel / phid;
				ca = (angacc - phidd * angvel * angvel) / phid;
			}
		}
	}

	// cubic or quintic interpolation
	template <typename T>
	void Interpolate(
		real_t t, T& p, T& v, T& a,
		real_t t0, const T& p0, const T& v0, const T& a0,
		real_t t1, const T& p1, const T& v1, const T& a1,
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
			kp[i] =       K[i][0] +       K[i][1] * s +      K[i][2]*s2 +      K[i][3]*s3 +     K[i][4]*s4 + K[i][5]*s5;
			kv[i] =       K[i][1] + 2.0 * K[i][2] * s +  3.0*K[i][3]*s2 +  4.0*K[i][4]*s3 + 5.0*K[i][5]*s4;
			ka[i] = 2.0 * K[i][2] + 6.0 * K[i][3] * s + 12.0*K[i][4]*s2 + 20.0*K[i][5]*s3;
		}

		p = kp[0]   *p0 + kp[1]   *p1 + kp[2]*h*v0 + kp[3]*h*v1 + kp[4]*h2*a0 + kp[5]*h2*a1;
		v = kv[0]/h *p0 + kv[1]/h *p1 + kv[2]  *v0 + kv[3]  *v1 + kv[4]*h *a0 + kv[5]*h *a1;
		a = ka[0]/h2*p0 + ka[1]/h2*p1 + ka[2]/h*v0 + ka[3]/h*v1 + ka[4]   *a0 + ka[5]   *a1;

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
		BipedRunKey* keym3 = 0;
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

			contact = ContactState::Surface;

			return;
		}

		if (key0->prev) keym1 = (BipedRunKey*)key0->prev;
		if (keym1 && keym1->prev) keym2 = (BipedRunKey*)keym1->prev;
		if (keym2 && keym2->prev) keym3 = (BipedRunKey*)keym2->prev;
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
		vec3_t p1 = key1->foot[side].var_pos_t->val;
		real_t yaw0 = key0->foot[side].var_pos_r->val;
		real_t yaw1 = key1->foot[side].var_pos_r->val;
		real_t h0 = param.swingHeight;

		vec3_t c0 = key0->foot[side].var_cop_pos->val;; // cop pos
		vec3_t c1 = key1->foot[side].var_cop_pos->val;; // cop pos
		vec3_t cv0, cv1; // cop vel

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

	else if (param.swingProfile == SwingProfile::HeelToe)
	{
		real_t tau1; // not always next duration!

		vec3_t f_tp, f_tv, f_ta; //< translational pos, vel and acc of foot
		vec3_t f_rp, f_rv, f_ra; //< rotational pos, vel and acc of foot
		vec3_t f_tp_roll, f_tv_roll, f_ta_roll, f_rp_roll, f_rv_roll, f_ra_roll;
		vec3_t f_tp_roll0, f_tv_roll0, f_ta_roll0, f_rp_roll0, f_rv_roll0, f_ra_roll0;
		vec3_t f_tp_roll1, f_tv_roll1, f_ta_roll1, f_rp_roll1, f_rv_roll1, f_ra_roll1;
		vec3_t cp, cv_v, ca;                     //< zmp in absolute coordinate
		vec3_t cp_local, cv_local, ca_local;   //< zmp in foot local

		if (keym1) cv0 = keym1->foot[side].var_cop_vel->val;
		cv1 = key1->foot[side].var_cop_vel->val;
		if (gaittype[key0->tick->idx] == GaitType::Run)
		{
			// first swing foot
			if ((ph == Phase::R && keym1 && gaittype[keym1->tick->idx] == GaitType::Walk && side == 1) ||
				(ph == Phase::L && keym1 && gaittype[keym1->tick->idx] == GaitType::Walk && side == 0) ||
				(ph == Phase::RLF && keym2 && gaittype[keym2->tick->idx] == GaitType::Walk && side == 1) ||
				(ph == Phase::LRF && keym2 && gaittype[keym2->tick->idx] == GaitType::Walk && side == 0))
			{
				if (ph == Phase::R || ph == Phase::L)
				{
					p1 = key2->foot[side].var_pos_t->val;
					c1 = key2->foot[side].var_cop_pos->val;
					cv1 = key2->foot[side].var_cop_vel->val;
					tau1 = key1->var_duration->val;
					t1 = key2->var_time->val;
					s = dt / (tau0 + tau1);
				}
				if (ph == Phase::RLF || ph == Phase::LRF)
				{
					p0 = keym1->foot[side].var_pos_t->val;
					c0 = keym1->foot[side].var_cop_pos->val;
					if (keym2) cv0 = keym2->foot[side].var_cop_vel->val;
					t0 = keym1->var_time->val;
					tau1 = keym1->var_duration->val;
					dt = std::max(t - keym1->var_time->val, 0.0);
					s = dt / (tau0 + tau1);
				}
			}

			// last swing foot
			else if ((ph == Phase::RLF && key2 && gaittype[key2->tick->idx] == GaitType::Walk && side == 0) ||
				     (ph == Phase::LRF && key2 && gaittype[key2->tick->idx] == GaitType::Walk && side == 1) ||
				     (ph == Phase::L && gaittype[key1->tick->idx]           == GaitType::Walk && side == 0) ||
				     (ph == Phase::R && gaittype[key1->tick->idx]           == GaitType::Walk && side == 1))
			{
				if (ph == Phase::RLF || ph == Phase::LRF)
				{
					p1 = key2->foot[side].var_pos_t->val;
					c1 = key2->foot[side].var_cop_pos->val;
					cv1 = key2->foot[side].var_cop_vel->val;
					tau1 = key1->var_duration->val;
					t1 = key2->var_time->val;
					s = dt / (tau0 + tau1);
				}
				if ((ph == Phase::R || ph == Phase::L) && keym1)
				{
					p0 = keym1->foot[side].var_pos_t->val;
					c0 = keym1->foot[side].var_cop_pos->val;
					if (keym2) cv0 = keym2->foot[side].var_cop_vel->val;
					tau1 = keym1->var_duration->val;
					t0 = keym1->var_time->val;
					dt = std::max(t - keym1->var_time->val, 0.0);
					s = dt / (tau0 + tau1);
				}
			}

			// swing foot
			else if ((ph == Phase::RLF) || (ph == Phase::LRF) || (ph == Phase::L && side == 0) || (ph == Phase::R && side == 1))
			{
				if ((ph == Phase::RLF && side == 0) || (ph == Phase::LRF && side == 1))
				{
					if (key3) p1 = key3->foot[side].var_pos_t->val;
					if (key3) c1 = key3->foot[side].var_cop_pos->val;
					if (key3) cv1 = key3->foot[side].var_cop_vel->val;
					if (key3) t1 = key3->var_time->val;
					if (key1) tau1 = key1->var_duration->val;
					s = dt / (tau0*2 + tau1);
				}
				if ((ph == Phase::R && side == 1) || (ph == Phase::L && side == 0))
				{
					if (keym1) p0 = keym1->foot[side].var_pos_t->val;
					if (keym1) c0 = keym1->foot[side].var_cop_pos->val;
					if (keym2) cv0 = keym2->foot[side].var_cop_vel->val;
					if (keym1) t0 = keym1->var_time->val;
					if (key2)  p1 = key2->foot[side].var_pos_t->val;
					if (key2)  c1 = key2->foot[side].var_cop_pos->val;
					if (key2)  cv1 = key2->foot[side].var_cop_vel->val;
					if (key2)  t1 = key2->var_time->val;
					if (key1) tau1 = key1->var_duration->val;
					if (keym1) dt = std::max(t - keym1->var_time->val, 0.0);
					s = dt / (tau0 + tau1*2);
				}
				if ((ph == Phase::RLF && side == 1) || (ph == Phase::LRF && side == 0))
				{
					if (keym2) p0 = keym2->foot[side].var_pos_t->val;
					if (keym2) c0 = keym2->foot[side].var_cop_pos->val;
					if (keym3) cv0 = keym3->foot[side].var_cop_vel->val;
					if (keym2) t0 = keym2->var_time->val;
					if (keym1) tau1 = keym1->var_duration->val;
					if (keym2) dt = std::max(t - keym2->var_time->val, 0.0);
					s = dt / (tau0*2 + tau1);
				}
			}
		}

		real_t ds = s / dt;
		real_t ch = (s - sin(_2pi * s) / _2pi);
		real_t chd = ((1.0 - cos(_2pi * s))*ds);
		real_t chdd = ((_2pi * sin(_2pi * s))*ds*ds);
		real_t cv = (1 - cos(_2pi * s)) / 2.0;
		real_t cvd = (_2pi * sin(_2pi * s)*ds/2);
		real_t cvdd = ((_2pi * _2pi) * cos(_2pi * s)*ds*ds/2);

		// preparation for foot print rotation while stance phase
		if (ph == Phase::LR || ph == Phase::RL || ph == Phase::D || (ph == Phase::R && side == 0) || (ph == Phase::L && side == 1))
		{
			//c0 = key0->foot[side].var_cop_pos->val;
			//c1 = key1->foot[side].var_cop_pos->val;
			cv0 = key0->foot[side].var_cop_vel->val;
			cv1 = key0->foot[side].var_cop_vel->val;

			f_tp = p0;
			f_tv = vec3_t(0.0, 0.0, 0.0);
			f_ta = vec3_t(0.0, 0.0, 0.0);
			f_rp = vec3_t(0.0, 0.0, yaw0);
			f_rv = vec3_t(0.0, 0.0, 0.0);
			f_ra = vec3_t(0.0, 0.0, 0.0);

			// zmp
			Interpolate(
				t, cp, cv_v, ca,
				t0, c0, cv0, vec3_t(0.0, 0.0, 0.0),
				t1, c1, cv1, vec3_t(0.0, 0.0, 0.0),
				param.swingInterpolation
			);

			// zmp in foot local (approximate assuming yaw rotation is slow enough)
			quat_t q = FromRollPitchYaw(f_rp);
			cp_local = q.Conjugated() * (cp - f_tp);
			cv_local = q.Conjugated() * (cv_v - f_tv);
			ca_local = q.Conjugated() * (ca - f_ta);
		}
		else
		{
			real_t yaw_diff = yaw1 - yaw0;
			while (yaw_diff > pi) yaw_diff -= 2.0 * pi;
			while (yaw_diff < -pi) yaw_diff += 2.0 * pi;

			/*f_tp = p0 + ch  *(p1 - p0) + cv  *vec3_t(0.0, 0.0, param.swingHeight);
			f_tv =      chd *(p1 - p0) + cvd *vec3_t(0.0, 0.0, param.swingHeight);
			f_ta =      chdd*(p1 - p0) + cvdd*vec3_t(0.0, 0.0, param.swingHeight);*/
			vec3_t pcom, vcom, acom;
			real_t dpos_z, dvel_z, dacc_z;    // additional z position
			ComState(t0, pcom, vcom, acom);
			Interpolate(t, f_tp.x, f_tv.x, f_ta.x, t0, p0.x, vcom.x, acom.x, t1, p1.x, 0.0, 0.0, param.swingInterpolation);
			Interpolate(t, f_tp.y, f_tv.y, f_ta.y, t0, p0.y, vcom.y, acom.y, t1, p1.y, 0.0, 0.0, param.swingInterpolation);
			Interpolate(t, f_tp.z, f_tv.z, f_ta.z, t0, p0.z, max(vcom.z, 0.0), acom.z, t1, p1.z, 0.0, 0.0, param.swingInterpolation);
			Interpolate2(t, dpos_z, dvel_z, dacc_z, t0, t1, param.swingHeight, param.swingInterpolation);
			f_tp.z += dpos_z;
			f_tv.z += dvel_z;
			f_ta.z += dacc_z;

			f_rp.z = yaw0 + ch * yaw_diff;
			f_rv.z =        chd * yaw_diff;
			f_ra.z =        chdd * yaw_diff;

			FootRotation(
				c0.x - p0.x, cv0.x, 0.0,
				f_tp_roll0, f_tv_roll0, f_ta_roll0,
				f_rp_roll0, f_rv_roll0, f_ra_roll0,
				contact);
			FootRotation(
				c1.x - p1.x, cv1.x, 0.0,
				f_tp_roll1, f_tv_roll1, f_ta_roll1,
				f_rp_roll1, f_rv_roll1, f_ra_roll1,
				contact);

			Interpolate(
				t, f_rp_roll.y, f_rv_roll.y, f_ra_roll.y,
				t0, f_rp_roll0.y, f_rv_roll0.y, 0.0,
				t1, f_rp_roll1.y, f_rv_roll1.y, 0.0,
				param.swingInterpolation
			);

			FootRotationInv(
				f_rp_roll.y, f_rv_roll.y, f_ra_roll.y,
				cp_local.x, cv_local.x, ca_local.x);

			contact = ContactState::Float; // TODO: –³ˆÓ–¡
		}

		// relative transform from footprint to foot
		FootRotation(
			cp_local.x, cv_local.x, ca_local.x,
			f_tp_roll, f_tv_roll, f_ta_roll,
			f_rp_roll, f_rv_roll, f_ra_roll,
			contact);

		// absolute pose of foot (approixmate)
		quat_t q = FromRollPitchYaw(f_rp);
		pose.Pos() = f_tp + q * f_tp_roll;
		pose.Ori() = q * FromRollPitchYaw(f_rp_roll);
		vel        = f_tv + q * f_tv_roll;
		angvel     = f_rv + q * f_rv_roll;
		acc        = f_ta + q * f_ta_roll;
		angacc     = f_ra + q * f_ra_roll;
	}

		// yaw angle: linear interpolation
		/*angle[2] = (1.0 - s) * yaw0 + s * yaw1;
		angvel[2] = (yaw1 - yaw0) / tau0;
		angacc[2] = 0.0;

		pose.Pos() = pos;
		pose.Ori() = FromRollPitchYaw(angle);*/
	}

	void BipedRunning::FootCopState(real_t t, int side, vec3_t& pos, vec3_t& vel, real_t& weight) {
		BipedRunKey* key0 = (BipedRunKey*)traj.GetSegment(t).first;
		BipedRunKey* key1 = (BipedRunKey*)traj.GetSegment(t).second;

		if (key0->next == key1) {
			real_t dt = t - key0->var_time->val;
			real_t tau = key0->var_duration->val;
			real_t s = dt / tau;

			vec3_t c0 = key0->foot[side].var_cop_pos->val;
			vec3_t c1 = key1->foot[side].var_cop_pos->val;

			pos = (1.0 - s) * c0 + s * c1;
			vel = (c1 - c0) / tau;
			weight = (1.0 - s) * key0->foot[side].weight + s * key1->foot[side].weight;
		}
		else {
			pos = key0->foot[side].var_cop_pos->val;
			vel.clear();
			weight = key0->foot[side].weight;
		}
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

	void BipedRunning::CopState(real_t t, vec3_t& pos, vec3_t& vel, vec3_t& acc) {
		BipedRunKey* key0 = (BipedRunKey*)traj.GetSegment(t).first;
		BipedRunKey* key1 = (BipedRunKey*)traj.GetSegment(t).second;

		real_t dt = t - key0->var_time->val;

		pos = key0->cop_pos + key0->cop_vel * dt + (1.0 / 2.0) * key0->cop_acc * (dt * dt);
		vel = key0->cop_vel + key0->cop_acc * dt;
		acc = key0->cop_acc;
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
		if (conf->Set(canvas, Render::Item::BipedFootCop, this)) {
			for (int i = 0; i < 2; i++) {
				canvas->BeginLayer("biped_foot_cop", true);
				canvas->BeginPath();
				canvas->MoveTo(trajectory[0].foot_cop[i]);
				for (int k = 1; k < trajectory.size(); k++) {
					canvas->LineTo(trajectory[k].foot_cop[i]);
				}
				canvas->EndPath();
				canvas->EndLayer();
			}
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
		real_t weight;
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
		TorsoState(t, s.torso_pos_r, angvel.z, angacc.z);
		CopState(t, s.cop_pos, vel, acc);
		s.cmp_pos = CmpPos(t);
		FootCopState(t, 0, s.foot_cop[0], vel, weight);
		FootCopState(t, 1, s.foot_cop[1], vel, weight);
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
		log << "t com_pos.x com_pos.y com_pos.z com_vel.x com_vel.y com_vel.z cop_pos.x cop_pos.y cop_pos.z foot_pos0.x foot_pos0.y foot_pos0.z foot_pos1.x foot_pos1.y foot_pos1.z cmp_pos.x cmp_pos.y cmp_pos.z\n";
		for (int i = 0; i < trajectory.size(); i++)
		{
			log << trajectory[i].t << " ";
			log << trajectory[i].com_pos.x << " " << trajectory[i].com_pos.y << " " << trajectory[i].com_pos.z << " ";
			log << trajectory[i].com_vel.x << " " << trajectory[i].com_vel.y << " " << trajectory[i].com_vel.z << " ";
			log << trajectory[i].cop_pos.x << " " << trajectory[i].cop_pos.y << " " << trajectory[i].cop_pos.z << " ";
			log << trajectory[i].foot_pos_t[0].x << " " << trajectory[i].foot_pos_t[0].y << " " << trajectory[i].foot_pos_t[0].z << " ";
			log << trajectory[i].foot_pos_t[1].x << " " << trajectory[i].foot_pos_t[1].y << " " << trajectory[i].foot_pos_t[1].z << " ";
			log << trajectory[i].cmp_pos.x << " " << trajectory[i].cmp_pos.y << " " << trajectory[i].cmp_pos.z;
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
		Constraint(solver, 3, ID(_tag, _obj->node, _obj->tick, _name), Constraint::Type::Equality, _scale) {
		obj[0] = _obj;
		obj[1] = (_obj->next ? (BipedRunKey*)_obj->next : 0);
	}

	RunnerLipPosCon::RunnerLipPosCon(Solver* solver, string _name, BipedRunKey* _obj, real_t _scale) :
		BipedRunCon(solver, ConTag::BipedLipPos, _name, _obj, _scale) {

		AddSLink (obj[1]->var_com_pos);
		AddSLink (obj[0]->var_com_pos);
		AddSLink (obj[0]->var_com_vel);
		AddC3Link(obj[0]->var_duration);
		AddSLink (obj[0]->var_cmp_pos);
		AddSLink (obj[0]->var_cmp_vel);
		
		for (int i = 0; i < 2; i++) {
			AddSLink(obj[0]->foot[i].var_cop_pos);
			AddSLink(obj[0]->foot[i].var_cop_vel);
		}
	}

	RunnerLipVelCon::RunnerLipVelCon(Solver* solver, string _name, BipedRunKey* _obj, real_t _scale) :
		BipedRunCon(solver, ConTag::BipedLipVel, _name, _obj, _scale) {

		AddSLink (obj[1]->var_com_vel);
		AddSLink (obj[0]->var_com_pos);
		AddSLink (obj[0]->var_com_vel);
		AddC3Link(obj[0]->var_duration);
		AddSLink (obj[0]->var_cmp_pos);
		AddSLink (obj[0]->var_cmp_vel);
		for (int i = 0; i < 2; i++) {
			AddSLink(obj[0]->foot[i].var_cop_pos);
			AddSLink(obj[0]->foot[i].var_cop_vel);
		}
	}

	RunnerLipCmpCon::RunnerLipCmpCon(Solver* solver, string _name, BipedRunKey* _obj, real_t _scale) :
		BipedRunCon(solver, ConTag::Any, _name, _obj, _scale) {

		AddSLink(obj[1]->var_cmp_pos);
		AddSLink(obj[0]->var_cmp_pos);
		AddSLink(obj[0]->var_cmp_vel);
		AddC3Link(obj[0]->var_duration);
	}

	RunnerLipMomCon::RunnerLipMomCon(Solver* solver, string _name, BipedRunKey* _obj, real_t _scale) :
		BipedRunCon(solver, ConTag::Any, _name, _obj, _scale) {

		AddSLink(obj[1]->var_mom);
		AddSLink(obj[0]->var_mom);
		AddX3Link(obj[0]->var_cmp_pos);
		AddX3Link(obj[0]->var_cmp_vel);
		AddC3Link(obj[0]->var_com_vel);
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

		AddSLink (obj[1]->foot[side].var_pos_t);
		AddSLink (obj[0]->foot[side].var_pos_t);
		AddSLink (obj[0]->foot[side].var_vel_t);
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

	RunnerFootPosRangeConR::RunnerFootPosRangeConR(Solver* solver, string _name, BipedRunKey* _obj, int _side, real_t _dir, real_t _scale) :
		Constraint(solver, 1, ID(ConTag::BipedFootPosRangeR, _obj->node, _obj->tick, _name), Constraint::Type::InequalityPenalty, _scale) {

		obj = _obj;
		side = _side;
		dir = _dir;

		AddSLink(obj->foot[side].var_pos_r);
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

	RunnerCmpRangeCon::RunnerCmpRangeCon(Solver* solver, string _name, BipedRunKey* _obj, vec3_t _dir, real_t _scale) :
		Constraint(solver, 1, ID(ConTag::BipedFootCopRange, _obj->node, _obj->tick, _name), Constraint::Type::InequalityPenalty, _scale) {

		obj = _obj;
		dir = _dir;

		AddR3Link(obj->var_cmp_pos);
		//AddR3Link(obj->foot[side].var_pos_t);
		AddSLink(obj->var_torso_pos_r);
	}

	RunnerMomRangeCon::RunnerMomRangeCon(Solver* solver, string _name, BipedRunKey* _obj, vec3_t _dir, real_t _scale) :
		Constraint(solver, 1, ID(ConTag::BipedFootCopRange, _obj->node, _obj->tick, _name), Constraint::Type::InequalityPenalty, _scale) {

		obj = _obj;
		dir = _dir;

		AddR3Link(obj->var_mom);
		AddSLink(obj->var_torso_pos_r);
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
		if (ph == BipedRunning::Phase::LRF || ph == BipedRunning::Phase::RLF)
			obj[1]->var_com_pos->val = p0 + v0 * tau - 0.5 * g * tau * tau;
		else
			obj[1]->var_com_pos->val = p_rhs;
	}

	void RunnerLipVelCon::CalcLhs() {
		Prepare();
		BipedRunning::Param& param = ((BipedRunning*)obj[0]->node)->param;
		if (ph == BipedRunning::Phase::LRF || ph == BipedRunning::Phase::RLF)
			obj[1]->var_com_vel->val = v0 - g * tau;
		else
			obj[1]->var_com_vel->val = v_rhs;
	}

	void RunnerLipCmpCon::CalcLhs() {
		Prepare();
		obj[1]->var_cmp_pos->val = cm0 + cmv0 * tau;
	}

	void RunnerFootPosConT::CalcLhs() {
		Prepare();
		obj[1]->foot[side].var_pos_t->val = p0 + v0*tau;
	}

	void RunnerFootPosConR::CalcLhs() {
		Prepare();
		obj[1]->foot[side].var_pos_r->val = theta0 + omega0*tau;
	}

	void RunnerFootCopPosCon::CalcLhs() {
		Prepare();
		obj[1]->foot[side].var_cop_pos->val = c0 + cv0*tau;
	}

	void RunnerFootCopVelCon::CalcLhs() {
		Prepare();
		obj[1]->foot[side].var_cop_vel->val = cv0 + cvd0;
	}

	void RunnerLipMomCon::CalcLhs() {
		Prepare();
		if (ph == BipedRunning::Phase::LRF || ph == BipedRunning::Phase::RLF)
			obj[1]->var_mom->val = L0;
		else
			obj[1]->var_mom->val = L0 + (g.z * cm0 + v1.z * cmv0) * tau + 0.5 * g.z * cmv0* tau * tau;
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

		T2 = T * T;
		g = param.gravity;
		ez = vec3_t(0.0, 0.0, 1.0);
		L0 = obj[0]->var_mom->val;
		p0 = obj[0]->var_com_pos->val;
		v0 = obj[0]->var_com_vel->val;
		c0 = obj[0]->cop_pos + vec3_t(0.0, 0.0, T2 * g.z);  //< c0 is vrp
		cm0 = obj[0]->var_cmp_pos->val;

		if (obj[1]) {
			tau = obj[0]->var_duration->val;
			tau2 = tau*tau;
			C = cosh(tau / T);
			S = sinh(tau / T);

			cv0 = obj[0]->cop_vel;
			cmv0 = obj[0]->var_cmp_vel->val;
			ca0 = obj[0]->cop_acc;

			p1 = obj[1]->var_com_pos->val;
			v1 = obj[1]->var_com_vel->val;
			c1 = obj[1]->cop_pos + vec3_t(0.0, 0.0, T2 * g.z);

			k_p_p = C;
			k_p_v = T * S;
			k_p_c = 1.0 - C;
			k_p_cv = tau - T * S;
			k_p_ca = T2 + (1.0 / 2.0) * tau2 - T2 * C;
			k_p_tau = cv0 + cmv0 + ca0 * tau + (S / T) * (p0 - (c0 + cm0 + ca0 * T2)) + C * (v0 - cv0 - cmv0);

			k_v_p = S / T;
			k_v_v = C;
			k_v_c = -S / T;
			k_v_cv = 1.0 - C;
			k_v_ca = tau - T * S;
			k_v_tau = ca0 + (C / T2) * (p0 - (c0 + cm0 + ca0 * T2)) + (S / T) * (v0 - cv0 - cmv0);

			for (int i = 0; i < 2; i++) {
				real_t w0 = obj[0]->foot[i].weight;
				real_t w1 = obj[1]->foot[i].weight;

				k_c_c[i] = w0;
				k_cv_c[i] = (w1 - w0) / tau;
				k_cv_cv[i] = w0;
				k_ca_cv[i] = 2.0 * (w1 - w0) / tau;
			}

			p_rhs = c0 + cm0 + ca0 * T2 + (cv0 + cmv0) * tau + (1.0 / 2.0) * ca0 * tau2 + C * (p0 - (c0 + cm0 + ca0 * T2)) + (S * T) * (v0 - cv0 - cmv0);
			v_rhs = cv0 + cmv0 + ca0 * tau + (S / T) * (p0 - (c0 + cm0 + ca0 * T2)) + C * (v0 - cv0 - cmv0);
		}
	}

	void RunnerFootPosConT::Prepare() {
		p1 = obj[1]->foot[side].var_pos_t->val;
		p0 = obj[0]->foot[side].var_pos_t->val;
		v0 = obj[0]->foot[side].var_vel_t->val;
		tau = obj[0]->var_duration->val;
	}

	void RunnerFootPosConR::Prepare() {
		theta1 = obj[1]->foot[side].var_pos_r->val;
		theta0 = obj[0]->foot[side].var_pos_r->val;
		omega0 = obj[0]->foot[side].var_vel_r->val;
		tau = obj[0]->var_duration->val;
	}

	void RunnerFootCopPosCon::Prepare() {
		c1 = obj[1]->foot[side].var_cop_pos->val;
		c0 = obj[0]->foot[side].var_cop_pos->val;
		cv0 = obj[0]->foot[side].var_cop_vel->val;
		tau = obj[0]->var_duration->val;
	}

	void RunnerFootCopVelCon::Prepare() {
		cv1 = obj[1]->foot[side].var_cop_vel->val;
		cv0 = obj[0]->foot[side].var_cop_vel->val;
		cvd0 = obj[0]->foot[side].var_cop_vel_diff->val;
	}

	void RunnerFootPosRangeConT::Prepare() {
		r = obj->foot[side].var_pos_t->val - obj->var_torso_pos_t->val;
		theta = obj->var_torso_pos_r->val;

		R = mat3_t::Rot(theta, 'z');
		ez = vec3_t(0.0, 0.0, 1.0);
		dir_abs = R * dir;
	}

	void RunnerFootPosRangeConR::Prepare() {
		thetaf = obj->foot[side].var_pos_r->val;
		thetat = obj->var_torso_pos_r->val;

		r = thetaf - thetat;

		while (r > pi) r -= _2pi;
		while (r < -pi) r += _2pi;
	}

	void RunnerFootCopRangeCon::Prepare() {
		r = obj->foot[side].var_cop_pos->val - obj->foot[side].var_pos_t->val;
		theta = obj->foot[side].var_pos_r->val;

		R = mat3_t::Rot(theta, 'z');
		ez = vec3_t(0.0, 0.0, 1.0);
		dir_abs = R * dir;
	}

	void RunnerCmpRangeCon::Prepare() {
		//theta = obj->foot[side].var_pos_r->val;

		R = mat3_t::Rot(theta, 'z');
		ez = vec3_t(0.0, 0.0, 1.0);
		dir_abs = R * dir;
	}

	void RunnerMomRangeCon::Prepare() {
		R = mat3_t::Rot(theta, 'z');
		ez = vec3_t(0.0, 0.0, 1.0);
		dir_abs = R * dir;
	}
	//-------------------------------------------------------------------------------------------------

	void RunnerLipPosCon::CalcCoef() {
		Prepare();
		if (ph == BipedRunning::Phase::LRF || ph == BipedRunning::Phase::RLF)
		{
			((SLink*)links[0])->SetCoef(1.0);
			((SLink*)links[1])->SetCoef(-1.0);
			((SLink*)links[2])->SetCoef(-tau);
			((C3Link*)links[3])->SetCoef(-(v0 - g * tau));
			((SLink*)links[4])->SetCoef(0.0);
			((SLink*)links[5])->SetCoef(0.0);
			((SLink*)links[6])->SetCoef(0.0);
			((SLink*)links[7])->SetCoef(0.0);
			((SLink*)links[8])->SetCoef(0.0);
			((SLink*)links[9])->SetCoef(0.0);

		}
		else
		{
			int idx = 0;
			((SLink*)links[idx++])->SetCoef(1.0);
			((SLink*)links[idx++])->SetCoef(-k_p_p);
			((SLink*)links[idx++])->SetCoef(-k_p_v);
			((C3Link*)links[idx++])->SetCoef(-k_p_tau);
			((SLink*)links[idx++])->SetCoef(-k_p_c);
			((SLink*)links[idx++])->SetCoef(-k_p_cv);

			for (int i = 0; i < 2; i++) {
				((SLink*)links[idx++])->SetCoef(-(k_p_c * k_c_c[i] + k_p_cv * k_cv_c[i]));
				((SLink*)links[idx++])->SetCoef(-(k_p_cv * k_cv_cv[i] + k_p_ca * k_ca_cv[i]));
			}
		}
	}

	void RunnerLipVelCon::CalcCoef() {
		Prepare();
		if (ph == BipedRunning::Phase::LRF || ph == BipedRunning::Phase::RLF)
		{
			((SLink*)links[0])->SetCoef(1.0);
			((SLink*)links[1])->SetCoef(0.0);
			((SLink*)links[2])->SetCoef(-1.0);
			((C3Link*)links[3])->SetCoef(g);
			((SLink*)links[4])->SetCoef(0.0);
			((SLink*)links[5])->SetCoef(0.0);
			((SLink*)links[6])->SetCoef(0.0);
			((SLink*)links[7])->SetCoef(0.0);
			((SLink*)links[8])->SetCoef(0.0);
			((SLink*)links[9])->SetCoef(0.0);
		}
		else
		{
			int idx = 0;
			((SLink*)links[idx++])->SetCoef(1.0);
			((SLink*)links[idx++])->SetCoef(-k_v_p);
			((SLink*)links[idx++])->SetCoef(-k_v_v);
			((C3Link*)links[idx++])->SetCoef(-k_v_tau);
			((SLink*)links[idx++])->SetCoef(-k_v_c);
			((SLink*)links[idx++])->SetCoef(-k_v_cv);

			for (int i = 0; i < 2; i++) {
				((SLink*)links[idx++])->SetCoef(-(k_v_c * k_c_c[i] + k_v_cv * k_cv_c[i]));
				((SLink*)links[idx++])->SetCoef(-(k_v_cv * k_cv_cv[i] + k_v_ca * k_ca_cv[i]));
			}
		}
	}

	void RunnerFootPosConT::CalcCoef() {
		Prepare();

		((SLink*)links[0])->SetCoef(1.0);
		((SLink*)links[1])->SetCoef(-1.0);
		((SLink*)links[2])->SetCoef(-tau);
		((C3Link*)links[3])->SetCoef(-v0);
	}

	void RunnerFootPosConR::CalcCoef() {
		Prepare();

		((SLink*)links[0])->SetCoef(1.0);
		((SLink*)links[1])->SetCoef(-1.0);
		((SLink*)links[2])->SetCoef(-tau);
		((SLink*)links[3])->SetCoef(-omega0);
	}

	void RunnerFootCopPosCon::CalcCoef() {
		Prepare();

		((SLink*)links[0])->SetCoef(1.0);
		((SLink*)links[1])->SetCoef(-1.0);
		((SLink*)links[2])->SetCoef(-tau);
		((C3Link*)links[3])->SetCoef(-cv0);
	}

	void RunnerFootCopVelCon::CalcCoef() {
		Prepare();

		((SLink*)links[0])->SetCoef(1.0);
		((SLink*)links[1])->SetCoef(-1.0);
		((SLink*)links[2])->SetCoef(-1.0);
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
		((X3Link*)links[2])->SetCoef(-g.z * tau * ez);
		((X3Link*)links[3])->SetCoef(-g.z * (0.5 * tau * tau) * ez);
		((C3Link*)links[4])->SetCoef(-(ez % (cmv0 * tau)));
		((C3Link*)links[5])->SetCoef(-(ez % (g.z * cm0 + v1.z * cmv0 + cmv0 * tau)));
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
		((SLink*)links[2])->SetCoef(-mf / msum);
		((SLink*)links[3])->SetCoef(-mf / msum);
	}

	void RunnerFootPosRangeConT::CalcCoef() {
		Prepare();

		((R3Link*)links[0])->SetCoef(dir_abs);
		((R3Link*)links[1])->SetCoef(-dir_abs);
		((SLink*)links[2])->SetCoef((ez % dir_abs) * r);
	}

	void RunnerFootPosRangeConR::CalcCoef() {
		Prepare();

		((SLink*)links[0])->SetCoef(dir);
		((SLink*)links[1])->SetCoef(-dir);
	}

	void RunnerFootCopRangeCon::CalcCoef() {
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

	void RunnerMomRangeCon::CalcCoef() {
		r = obj->var_mom->val;
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
		if (ph == BipedRunning::Phase::LRF || ph == BipedRunning::Phase::RLF)
			y = p1 - (p0 + v0 * tau - 0.5 * g * tau * tau);
		else
			y = p1 - p_rhs;

		//printf("y0 = (%lf, %lf, %lf)\n", y[0], y[1], y[2]);
	}

	void RunnerLipVelCon::CalcDeviation() {
		if (ph == BipedRunning::Phase::LRF || ph == BipedRunning::Phase::RLF)
			y = v1 - (v0 - g * tau);
		else
			y = v1 - v_rhs;
	}

	void RunnerLipCmpCon::CalcDeviation() {
		y = cm1 - (cm0 + cmv0 * tau);
	}

	void RunnerLipMomCon::CalcDeviation() {
		if (ph == BipedRunning::Phase::LRF || ph == BipedRunning::Phase::RLF)
			y = L1 - L0;
		else 
			y = L1 - (L0 + ((g.z*cm0 + v1.z*cmv0)*tau + 0.5 * tau * tau * cmv0 * g.z));
	}

	void RunnerFootPosConT::CalcDeviation() {
		y = p1 - (p0 + v0 * tau);

	}

	void RunnerFootPosConR::CalcDeviation() {
		y[0] = theta1 - (theta0 + omega0 * tau);
	}

	void RunnerFootCopPosCon::CalcDeviation() {
		y = c1 - (c0 + cv0 * tau);
	}

	void RunnerFootCopVelCon::CalcDeviation() {
		y = cv1 - (cv0 + cvd0);
	}

	void RunnerFootPosRangeConT::CalcDeviation() {
		real_t s = dir_abs * r;

		if (s < bound) {
			y[0] = s - bound;
			active = true;
		}
		else {
			y[0] = 0.0;
			active = false;
		}
	}

	void RunnerFootPosRangeConR::CalcDeviation() {
		real_t s = dir * r;

		if (s < bound) {
			y[0] = s - bound;
			active = true;
		}
		else {
			y[0] = 0.0;
			active = false;
		}
	}

	void RunnerFootCopRangeCon::CalcDeviation() {
		real_t s = dir_abs * r;

		if (s < bound) {
			y[0] = s - bound;
			active = true;
		}
		else {
			y[0] = 0.0;
			active = false;
		}
	}

	void RunnerCmpRangeCon::CalcDeviation() {
		real_t s = dir_abs * r;

		if (s < bound)
		{
			y[0] = s - bound;
			active = true;
		}
		else
		{
			y[0] = 0.0;
			active = false;
		}
	}

	void RunnerMomRangeCon::CalcDeviation() {
		real_t s = dir_abs * r;

		if (s < bound)
		{
			y[0] = s - bound;
			active = true;
		}
		else
		{
			y[0] = 0.0;
			active = false;
		}
	}

	//-------------------------------------------------------------------------------------------------

}