#include <DiMP/Graph/Biped.h>
#include <DiMP/Graph/Graph.h>
#include <DiMP/Render/Config.h>
#include <DiMP/Render/Canvas.h>

#include <sbrollpitchyaw.h>

namespace DiMP {;

const real_t   pi     = M_PI;
const real_t _2pi     = 2.0*pi;
const real_t  damping = 0.1;
const vec3_t  one(1.0, 1.0, 1.0);
	
//-------------------------------------------------------------------------------------------------
// BipedLIPKey

BipedLIPKey::BipedLIPKey() {

}

// register variables for planning
void BipedLIPKey::AddVar(Solver* solver) {
	BipedLIP* obj = (BipedLIP*)node;

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
	
	var_com_pos->weight = damping*one;
	var_com_vel->weight = damping*one;
	
	var_time->weight[0] = damping;

	// register state variables of ddp
	solver->AddInputVar(var_torso_pos_t, tick->idx);
	solver->AddInputVar(var_torso_pos_r, tick->idx);
	solver->AddInputVar(var_torso_vel_t, tick->idx);
	
	solver->AddStateVar(var_com_pos, tick->idx);
	solver->AddStateVar(var_com_vel, tick->idx);
	
	solver->AddStateVar(var_time   , tick->idx);

	if(next){
		// step duration
		var_duration = new SVar(solver, ID(VarTag::BipedDuration, node, tick, name + "_duration"), node->graph->scale.time);
		var_duration->weight[0] = damping;

		// T
		var_T = new SVar(solver, ID(VarTag::BipedT, node, tick, name + "_T"), node->graph->scale.time);
		var_T->weight[0] = damping;

		// register input variables of ddp
		solver->AddInputVar(var_duration, tick->idx);
		solver->AddInputVar(var_T       , tick->idx);
	}

	// foot position
	for(int i = 0; i < 2; i++){
		string prefix = (i == 0 ? "_foot_r" : "_foot_l");

		foot[i].var_pos_t   = new V3Var(solver, ID(VarTag::BipedFootTP  , node, tick, name + prefix + "_foot_t"), node->graph->scale.pos_t);
		foot[i].var_pos_r   = new SVar (solver, ID(VarTag::BipedFootRP  , node, tick, name + prefix + "_foot_r"), node->graph->scale.pos_r);
		foot[i].var_cop_pos = new V3Var(solver, ID(VarTag::BipedFootCopP, node, tick, name + prefix + "_cop_p" ), node->graph->scale.pos_t);
		foot[i].var_cop_vel = new V3Var(solver, ID(VarTag::BipedFootCopV, node, tick, name + prefix + "_cop_v" ), node->graph->scale.vel_t);
		
		foot[i].var_pos_t  ->weight    = damping*one;
		foot[i].var_pos_r  ->weight[0] = damping;
		foot[i].var_cop_pos->weight    = damping*one;
		foot[i].var_cop_vel->weight    = damping*one;

		//foot[i].var_pos_r  ->locked = true;
		
		solver->AddStateVar(foot[i].var_pos_t  , tick->idx);
		solver->AddStateVar(foot[i].var_pos_r  , tick->idx);
		solver->AddStateVar(foot[i].var_cop_pos, tick->idx);
		solver->AddStateVar(foot[i].var_cop_vel, tick->idx);
		
		if(next){
			foot[i].var_vel_t        = new V3Var(solver, ID(VarTag::BipedFootTV  , node, tick, name + prefix + "_foot_t" ), node->graph->scale.vel_t);
			foot[i].var_vel_r        = new SVar (solver, ID(VarTag::BipedFootRV  , node, tick, name + prefix + "_foot_r" ), node->graph->scale.vel_r);
			foot[i].var_cop_vel_diff = new V3Var(solver, ID(VarTag::BipedFootCopV, node, tick, name + prefix + "_cop_vd" ), node->graph->scale.vel_t);

			foot[i].var_vel_t       ->weight    = damping*one;
			foot[i].var_vel_r       ->weight[0] = damping;
			foot[i].var_cop_vel_diff->weight    = damping*one;

			solver->AddInputVar(foot[i].var_vel_t       , tick->idx);
			solver->AddInputVar(foot[i].var_vel_r       , tick->idx);
			solver->AddInputVar(foot[i].var_cop_vel_diff, tick->idx);
		}
	}
}

// register constraints for planning
void BipedLIPKey::AddCon(Solver* solver) {
	BipedLIP* obj = (BipedLIP*)node;
	BipedLIPKey* nextObj = (BipedLIPKey*)next;

	if (next) {
		con_lip_pos = new BipedLipPosCon(solver, name + "_lip_pos", this, node->graph->scale.pos_t);
		con_lip_vel = new BipedLipVelCon(solver, name + "_lip_vel", this, node->graph->scale.vel_t);
	
		con_time = new BipedTimeCon(solver, name + "_time", this, node->graph->scale.time);
		
		con_duration_range = new RangeConS(solver, ID(ConTag::BipedDurationRange, node, tick, name + "_duration_range"), var_duration, node->graph->scale.time);
		con_T_range        = new RangeConS(solver, ID(ConTag::BipedDurationRange, node, tick, name + "_T_range"       ), var_T       , node->graph->scale.time);

		con_des_duration = new FixConS(solver, ID(ConTag::BipedDurationRange, node, tick, name + "_des_duration"), var_duration, node->graph->scale.time);
		con_des_T        = new FixConS(solver, ID(ConTag::BipedDurationRange, node, tick, name + "_des_T"       ), var_T       , node->graph->scale.time);
		con_des_duration->weight[0] = 0.01;
		con_des_T       ->weight[0] = 0.01;
		
		//con_com_vel_zero = new FixConV3(solver, ID(ConTag::BipedFootVelZeroT, node, tick, name + "_com_vel_zero"), var_com_vel, node->graph->scale.vel_t);
		//con_com_vel_zero->weight = 0.01*one;

		solver->AddTransitionCon(con_lip_pos, tick->idx);
		solver->AddTransitionCon(con_lip_vel, tick->idx);
		
		solver->AddTransitionCon(con_time, tick->idx);
		
		solver->AddCostCon(con_duration_range, tick->idx);
		solver->AddCostCon(con_T_range       , tick->idx);
		solver->AddCostCon(con_des_duration  , tick->idx);
		solver->AddCostCon(con_des_T         , tick->idx);

		//solver->AddCostCon(con_com_vel_zero, tick->idx);
	}

	con_com_pos = new BipedComConP(solver, name + "_com_p", this, node->graph->scale.pos_t);
	solver->AddCostCon(con_com_pos, tick->idx);
	
	if(next){
		con_com_vel = new BipedComConV(solver, name + "_com_v", this, node->graph->scale.vel_t);
		solver->AddCostCon(con_com_vel, tick->idx);
	}

	con_des_com_pos     = new FixConV3(solver, ID(ConTag::BipedComPos, node, tick, name + "_des_com_pos"    ), var_com_pos    , node->graph->scale.pos_t);
	con_des_com_vel     = new FixConV3(solver, ID(ConTag::BipedComVel, node, tick, name + "_des_com_vel"    ), var_com_vel    , node->graph->scale.vel_t);
	con_des_torso_pos_r = new FixConS (solver, ID(ConTag::BipedTorso , node, tick, name + "_des_torso_pos_r"), var_torso_pos_r, node->graph->scale.pos_r);
	con_des_com_pos    ->weight.clear(); 
	con_des_com_vel    ->weight.clear(); 
	con_des_torso_pos_r->weight.clear(); 
	solver->AddCostCon(con_des_com_pos    , tick->idx);
	solver->AddCostCon(con_des_com_vel    , tick->idx);
	solver->AddCostCon(con_des_torso_pos_r, tick->idx);

	for(int i = 0; i < 2; i++){
		string prefix = (i == 0 ? "_foot_r" : "_foot_l");

		if (next) {
			foot[i].con_pos_t   = new BipedFootPosConT  (solver, name + prefix + "_pos_t"  , this, i, node->graph->scale.pos_t);
			foot[i].con_pos_r   = new BipedFootPosConR  (solver, name + prefix + "_pos_r"  , this, i, node->graph->scale.pos_r);
			foot[i].con_cop_pos = new BipedFootCopPosCon(solver, name + prefix + "_cop_pos", this, i, node->graph->scale.pos_t);
			foot[i].con_cop_vel = new BipedFootCopVelCon(solver, name + prefix + "_cop_vel", this, i, node->graph->scale.vel_t);
			solver->AddTransitionCon(foot[i].con_pos_t  , tick->idx);
			solver->AddTransitionCon(foot[i].con_pos_r  , tick->idx);
			solver->AddTransitionCon(foot[i].con_cop_pos, tick->idx);
			solver->AddTransitionCon(foot[i].con_cop_vel, tick->idx);

			foot[i].con_vel_zero_t        = new FixConV3(solver, ID(ConTag::BipedFootVelZeroT, node, tick, name + prefix + "_vel_zero_t"       ), foot[i].var_vel_t       , node->graph->scale.vel_t);
			foot[i].con_vel_zero_r        = new FixConS (solver, ID(ConTag::BipedFootVelZeroR, node, tick, name + prefix + "_vel_zero_r"       ), foot[i].var_vel_r       , node->graph->scale.vel_r);
			foot[i].con_cop_vel_diff_zero = new FixConV3(solver, ID(ConTag::BipedFootVelZeroT, node, tick, name + prefix + "_cop_vel_diff_zero"), foot[i].var_cop_vel_diff, node->graph->scale.vel_t);
			
			foot[i].con_vel_zero_t       ->desired = vec3_t();
			foot[i].con_vel_zero_r       ->desired = 0.0;
			foot[i].con_cop_vel_diff_zero->desired = vec3_t();
			foot[i].con_cop_vel_diff_zero->weight  = 0.1*one;
			
			solver->AddCostCon(foot[i].con_vel_zero_t       , tick->idx);
			solver->AddCostCon(foot[i].con_vel_zero_r       , tick->idx);
			solver->AddCostCon(foot[i].con_cop_vel_diff_zero, tick->idx);
		}

		foot[i].con_des_pos_t   = new FixConV3(solver, ID(ConTag::BipedFootPosT, node, tick, name + prefix + "_des_pos_t"  ), foot[i].var_pos_t  , node->graph->scale.pos_t);
		foot[i].con_des_pos_r   = new FixConS (solver, ID(ConTag::BipedFootPosR, node, tick, name + prefix + "_des_pos_r"  ), foot[i].var_pos_r  , node->graph->scale.pos_r);
		foot[i].con_des_cop_pos = new FixConV3(solver, ID(ConTag::BipedFootCop , node, tick, name + prefix + "_des_cop_pos"), foot[i].var_cop_pos, node->graph->scale.pos_t);
		foot[i].con_des_pos_t  ->weight.clear(); 
		foot[i].con_des_pos_r  ->weight.clear(); 
		foot[i].con_des_cop_pos->weight.clear(); 
		solver->AddCostCon(foot[i].con_des_pos_t  , tick->idx);
		solver->AddCostCon(foot[i].con_des_pos_r  , tick->idx);
		solver->AddCostCon(foot[i].con_des_cop_pos, tick->idx);

		for(int d = 0; d < 2; d++){
			real_t sign = (d == 0 ? 1.0 : -1.0);
			for(int j = 0; j < 3; j++){
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

		foot[i].con_contact = new BipedFootContactCon(solver, name + prefix + "_contact", this, i, node->graph->scale.pos_t);
		solver->AddCostCon(foot[i].con_contact, tick->idx);
	}
}

void BipedLIPKey::Prepare() {
	if(next){
		// calc cop
		cop_pos.clear();
		cop_vel.clear();
		cop_acc.clear();

		real_t tau = var_duration->val;
	
		for(int i = 0; i < 2; i++){
			real_t w0  = foot[i].weight;
			real_t w1  = ((BipedLIPKey*)next)->foot[i].weight;
			vec3_t c0  = foot[i].var_cop_pos->val;
			vec3_t cv0 = foot[i].var_cop_vel->val;

			cop_pos += w0*c0;
			cop_vel += w0*cv0 + ((w1 - w0)/tau)*c0;
			cop_acc += (2.0*(w1 - w0)/tau)*cv0;
		}
	}
}

void BipedLIPKey::Finish(){
	// tick's time is updated from time variable
	tick->time = var_time->val;
}

void BipedLIPKey::Draw(Render::Canvas* canvas, Render::Config* conf) {
	BipedLIP* obj = (BipedLIP*)node;

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

BipedLIP::Param::Param() {
	gravity            = 9.8;
	comHeight          = 1.0;
	torsoMass          = 10.0;
	footMass           = 5.0;
	swingProfile       = SwingProfile::Cycloid;
	swingInterpolation = SwingInterpolation::Cubic;
	swingHeight        = 0.1;
	durationMin[Phase::R ] = 0.1;
	durationMax[Phase::R ] = 0.8;
	durationMin[Phase::L ] = 0.1;
	durationMax[Phase::L ] = 0.8;
	durationMin[Phase::RL] = 0.1;
	durationMax[Phase::RL] = 0.2;
	durationMin[Phase::LR] = 0.1;
	durationMax[Phase::LR] = 0.2;
	durationMin[Phase::D ] = 0.1;
	durationMax[Phase::D ] = 0.2;
	durationMin[Phase::F ] = 0.1;
	durationMax[Phase::F ] = 0.3;

	TScaleMin[Phase::R ] = 0.5;
	TScaleMax[Phase::R ] = 1.0;
	TScaleMin[Phase::L ] = 0.5;
	TScaleMax[Phase::L ] = 1.0;
	TScaleMin[Phase::RL] = 1.0;
	TScaleMax[Phase::RL] = 1.5;
	TScaleMin[Phase::LR] = 1.0;
	TScaleMax[Phase::LR] = 1.5;
	TScaleMin[Phase::D ] = 1.0;
	TScaleMax[Phase::D ] = 1.0;
	TScaleMin[Phase::F ] = 1.0;
	TScaleMax[Phase::F ] = 1.0;

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

	ankleToToe        = 0.1;
	ankleToHeel       = 0.1;
	toeCurvature      = 10.0;
	heelCurvature     = 10.0;
	toeCurvatureRate  = 0.0;
	heelCurvatureRate = 0.0;
    
	minSpacing  = 0.0;
	swingMargin = 0.0;
}

//-------------------------------------------------------------------------------------------------
// Waypoints
BipedLIP::Waypoint::Waypoint() {
	k             = 0;
	time          = 0.0;
	duration      = 0.0;
	duration_min  = 0.0;
	duration_max  = 0.0;
	com_pos       = vec3_t();
	com_vel       = vec3_t();
	torso_pos_r   = 0.0;
	
	//fix_com_pos       = false;
	//fix_com_vel       = false;
	//fix_torso_pos_r   = false;
	set_duration_range = false;

	weight_duration    = 0.0;
	weight_com_pos     = vec3_t();
	weight_com_vel     = vec3_t();
	weight_torso_pos_r = 0.0;

	for(int i = 0; i < 2; i++){
		foot_pos_t  [i] = vec3_t();
		foot_pos_r  [i] = 0.0;
		foot_cop    [i] = vec3_t();
		foot_cop_min[i] = vec3_t();
		foot_cop_max[i] = vec3_t();

		weight_foot_pos_t[i] = vec3_t();
		weight_foot_pos_r[i] = 0.0;
		weight_foot_cop  [i] = vec3_t();

		set_cop_range [i] = false;
	}
}

//-------------------------------------------------------------------------------------------------
BipedLIP::Snapshot::Snapshot() {
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
	param.Tnominal = sqrt(param.comHeight / param.gravity);

	real_t durationAve[Phase::Num];
	for (int i = 0; i < Phase::Num; i++)
		durationAve[i] = (param.durationMin[i] + param.durationMax[i]) / 2.0;

	// set desired value and range of duration and T value
	for (int k = 0; k < graph->ticks.size(); k++) {
		BipedLIPKey* key = (BipedLIPKey*)traj.GetKeypoint(graph->ticks[k]);

		// initial value of step duration is set as the average of minimum and maximum
		if (key->next) {
			int ph = phase[key->tick->idx];
	
			key->var_duration->val = key->con_des_duration->desired = durationAve[phase[k]];
			key->con_duration_range->_min = param.durationMin[phase[k]];
			key->con_duration_range->_max = param.durationMax[phase[k]];
			key->var_T->val = key->con_des_T->desired = sqrt(param.comHeight/param.gravity);
			key->con_T_range       ->_min = param.Tnominal*param.TScaleMin[phase[k]];
			key->con_T_range       ->_max = param.Tnominal*param.TScaleMax[phase[k]];
		}
	}
	
	// set duration from waypoints (if set)
	for(Waypoint& wp : waypoints) {
		BipedLIPKey* key = (BipedLIPKey*)traj.GetKeypoint(graph->ticks[wp.k]);
	
		if(key->next){
			if(wp.set_duration_range){
				key->con_duration_range->_min = wp.duration_min;
				key->con_duration_range->_max = wp.duration_max;
			}
			if(wp.weight_duration != 0.0){
				key->var_duration->val           = wp.duration;
				key->con_des_duration->desired   = wp.duration;
				key->con_des_duration->weight[0] = wp.weight_duration;
			}
		}
	}

	real_t t = 0.0;
	for (int k = 0; k < graph->ticks.size(); k++) {
		BipedLIPKey* key = (BipedLIPKey*)traj.GetKeypoint(graph->ticks[k]);

		key->tick->time    = t;
		key->var_time->val = t;
			
		if (!key->prev) {
			// initial time is fixed
			key->var_time->val = 0.0;
			key->var_time->locked = true;
		}

		if (key->next) {
			// foot velocity must be zero while in contact
			key->foot[0].con_vel_zero_t->active = (phase[k] != BipedLIP::Phase::L);
			key->foot[0].con_vel_zero_r->active = (phase[k] != BipedLIP::Phase::L);
			key->foot[1].con_vel_zero_t->active = (phase[k] != BipedLIP::Phase::R);
			key->foot[1].con_vel_zero_r->active = (phase[k] != BipedLIP::Phase::R);

			// cop velocity must be constant (i.e., acceleration must be zero) while in contact
			key->foot[0].con_cop_vel_diff_zero->active = (phase[k] == BipedLIP::Phase::LR || phase[k] == BipedLIP::Phase::R);
			key->foot[1].con_cop_vel_diff_zero->active = (phase[k] == BipedLIP::Phase::RL || phase[k] == BipedLIP::Phase::L);
		}
		
		// range limit of foot position
		for (int i = 0; i < 2; i++) {
			for(int j = 0; j < 3; j++){
				key->foot[i].con_pos_range_t[j][0]->bound =  param.footPosMin[i][j];
				key->foot[i].con_pos_range_t[j][1]->bound = -param.footPosMax[i][j];
			}
			key->foot[i].con_pos_range_r[0]->bound =  param.footOriMin[i];
			key->foot[i].con_pos_range_r[1]->bound = -param.footOriMax[i];

			for(int j = 0; j < 3; j++){
				key->foot[i].con_cop_range[j][0]->bound =  param.footCopMin[i][j];
				key->foot[i].con_cop_range[j][1]->bound = -param.footCopMax[i][j];
			}
		}

		if(key->next)
			t += key->var_duration->val;

		// set weight of feet
		if(key->next){
			BipedLIPKey* key0 = key;
			BipedLIPKey* key1 = (BipedLIPKey*)key->next;

			if(phase[k] == Phase::R){
				key0->foot[0].weight = 1.0;
				key1->foot[0].weight = 1.0;
				key0->foot[1].weight = 0.0;
				key1->foot[1].weight = 0.0;
			}
			if(phase[k] == Phase::L){
				key0->foot[0].weight = 0.0;
				key1->foot[0].weight = 0.0;
				key0->foot[1].weight = 1.0;
				key1->foot[1].weight = 1.0;
			}
			if(phase[k] == Phase::D){
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
	for(Waypoint& wp : waypoints) {
		BipedLIPKey* key = (BipedLIPKey*)traj.GetKeypoint(graph->ticks[wp.k]);
		real_t t = key->var_time->val;

		curve_com.AddPoint(t);
		curve_com.SetPos(idx, wp.com_pos);
		curve_com.SetVel(idx, wp.com_vel);

		curve_torso_r.AddPoint(t);
		curve_torso_r.SetPos(idx, wp.torso_pos_r);
		
		for(int i = 0; i < 2; i++) {
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

		key->var_com_pos->val = key->con_des_com_pos->desired = curve_com.CalcPos(t);
		key->var_com_vel->val = key->con_des_com_vel->desired = curve_com.CalcVel(t);

		key->var_torso_pos_r->val = curve_torso_r.CalcPos(t);
		
		for (int i = 0; i < 2; i++) {
			key->foot[i].var_pos_t  ->val = key->foot[i].con_des_pos_t  ->desired = curve_foot_t[i].CalcPos(t);
			key->foot[i].var_pos_r  ->val = key->foot[i].con_des_pos_r  ->desired = curve_foot_r[i].CalcPos(t);
			key->foot[i].var_cop_pos->val = key->foot[i].con_des_cop_pos->desired = key->foot[i].var_pos_t->val;
			
			if(key->next){
				key->foot[i].var_vel_t  ->val = curve_foot_t[i].CalcVel(t);
				key->foot[i].var_vel_r  ->val = curve_foot_r[i].CalcVel(t);
				key->foot[i].var_cop_vel->val = curve_foot_t[i].CalcVel(t);
			}
		}

		real_t mt = param.torsoMass;
		real_t mf = param.footMass;
		key->var_torso_pos_t->val = (1.0 + 2.0*(mf/mt))*key->var_com_pos->val - (mf/mt)*(key->foot[0].var_pos_t->val + key->foot[1].var_pos_t->val);
		key->var_torso_vel_t->val = (1.0 + 2.0*(mf/mt))*key->var_com_vel->val;
	}

	// set weights and bounds
	for (Waypoint& wp : waypoints) {
		BipedLIPKey* key = (BipedLIPKey*)traj.GetKeypoint(graph->ticks[wp.k]);

		key->con_des_com_pos    ->weight    = wp.weight_com_pos;
		key->con_des_com_vel    ->weight    = wp.weight_com_vel;
		key->con_des_torso_pos_r->weight[0] = wp.weight_torso_pos_r;

		for(int i = 0; i < 2; i++){
			key->foot[i].con_des_pos_t  ->weight    = wp.weight_foot_pos_t[i];
			key->foot[i].con_des_pos_r  ->weight[0] = wp.weight_foot_pos_r[i];
			key->foot[i].con_des_cop_pos->weight    = wp.weight_foot_cop  [i];

			if(wp.set_cop_range[i]){
				for(int j = 0; j < 3; j++){
					key->foot[i].con_cop_range[j][0]->bound =  wp.foot_cop_min[i][j];
					key->foot[i].con_cop_range[j][1]->bound = -wp.foot_cop_max[i][j];
				}
			}
		}
	}
	
	Prepare();
}

void BipedLIP::Prepare() {
	TrajectoryNode::Prepare();
	trajReady = false;
}

void BipedLIP::Finish() {
	TrajectoryNode::Finish();

	/*
	for (int k = 0; k < graph->ticks.size(); k++) {
		BipedLIPKey* key = (BipedLIPKey*)traj.GetKeypoint(graph->ticks[k]);

		if(key->next)
			DSTR << "k: " << k
			     << " phase: " << phase[k]
			     << " c0: " << key->foot[0].var_cop_vel->val.x
			     << " c1: " << key->foot[1].var_cop_vel->val.x << endl;
	}
	*/
}

int BipedLIP::Phase(real_t t) {
	BipedLIPKey* key = (BipedLIPKey*)traj.GetSegment(t).first;
	return phase[key->tick->idx];
}

void BipedLIP::ComState(real_t t, vec3_t& pos, vec3_t& vel, vec3_t& acc){
	BipedLIPKey* key0 = (BipedLIPKey*)traj.GetSegment(t).first;
	BipedLIPKey* key1 = (BipedLIPKey*)traj.GetSegment(t).second;

	real_t dt  = t - key0->var_time->val;
	real_t dt2 = dt*dt;

	if(key1 == key0->next){
		//real_t T   = param.T;
		real_t T   = key0->var_T->val;
		real_t T2  = T*T;
		vec3_t p0  = key0->var_com_pos->val;
		vec3_t v0  = key0->var_com_vel->val;
		vec3_t c0  = key0->cop_pos;// + vec3_t(0.0, 0.0, param.comHeight);
		vec3_t cv0 = key0->cop_vel;
		vec3_t ca0 = key0->cop_acc;// + vec3_t(0.0, 0.0, param.gravity);
		vec3_t g   = vec3_t(0.0, 0.0, param.gravity);
		real_t C   = cosh(dt/T);
		real_t S   = sinh(dt/T);
		vec3_t dp  = p0 - c0 - (ca0 + g)*T2;
		vec3_t dv  = v0 - cv0;
	
		if(phase[key0->tick->idx] == Phase::F){
			pos = p0 + v0*dt - (1.0/2.0)*g*dt2;
			vel = v0 - g*dt;
			acc = -g;
		}
		else{
			pos = c0 + (ca0 + g)*T2 + cv0*dt + (1.0/2.0)*ca0*dt*dt +      C*dp + (S*T)*dv;
			vel =                     cv0    +           ca0*dt    + (S/T )*dp +     C*dv;
			acc =                                        ca0       + (C/T2)*dp + (S/T)*dv;
		}
	}
	else{
		pos = key0->var_com_pos->val;
		vel = key0->var_com_vel->val;
		acc = vec3_t();
	}
}

void BipedLIP::TorsoState(real_t t, real_t& ori, real_t& angvel, real_t& angacc) {
	BipedLIPKey* key0 = (BipedLIPKey*)traj.GetSegment(t).first;
	BipedLIPKey* key1 = (BipedLIPKey*)traj.GetSegment(t).second;

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
		ori    = key0->var_torso_pos_r->val;
		angvel = 0.0;
	}
	angacc = 0.0;
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

void BipedLIP::FootRotation(
    real_t cp, real_t cv, real_t ca,
    vec3_t& pos, vec3_t& vel, vec3_t& acc,
	vec3_t& ori, vec3_t& angvel, vec3_t& angacc,
    int& contact)
{
	real_t l0     = param.ankleToToe ;
	real_t l1     = param.ankleToHeel;
	real_t rho0   = param.toeCurvature ;
	real_t rho1   = param.heelCurvature;
	real_t r0     = 1.0/rho0;
	real_t r1     = 1.0/rho1;
	real_t kappa0 = param.toeCurvatureRate ;
	real_t kappa1 = param.heelCurvatureRate;

	real_t d;                 //< rolling distance
    vec2_t v, vd, vdd;        //< foot center to contact point on the foot, and its derivative w.r.t. d
	real_t phi, phid, phidd;  //< foot rotation and its derivative w.r.t. d

    phi = phid = phidd = 0.0;
	            
	// current cop is on heel
	if(cp < -l1){
		d = cp + l1;
        
		if(param.footCurveType == FootCurveType::Arc){
			phi = d/r1;
            v   = vec2_t(-l1 + r1*sin(phi), r1*(1.0 - cos(phi)));

            phid = 1.0/r1;
			vd   = vec2_t(cos(phi), sin(phi));

            phidd = 0.0;
            vdd   = vec2_t(-sin(phi), cos(phi))*phid;
		}
		if(param.footCurveType == FootCurveType::Clothoid){
			phi = -0.5*kappa1*d*d;
            v   = vec2_t(-l1 - clothoid_x(-d, kappa1), clothoid_y(-d, kappa1));

            phid = -kappa1*d;
			vd   = vec2_t(cos(phi), sin(phi));

            phidd = -kappa1;
            vdd   = vec2_t(-sin(phi), cos(phi))*phid;
		}

		contact = ContactState::Heel;
	}
	// current cop is on toe
	else if(cp > l0){
		d = cp - l0;
		
        if(param.footCurveType == FootCurveType::Arc){
			phi = d/r0;
            v   = vec2_t(l0 + r0*sin(phi), r0*(1.0 - cos(phi)));

            phid = 1.0/r0;
			vd   = vec2_t(cos(phi), sin(phi));

            phidd = 0.0;
            vdd   = vec2_t(-sin(phi), cos(phi))*phid;
		}
		if(param.footCurveType == FootCurveType::Clothoid){
			phi = 0.5*kappa0*d*d;
            v   = vec2_t(l0 + clothoid_x(d, kappa0), clothoid_y(d, kappa0));

            phid = kappa0*d;
			vd   = vec2_t(cos(phi), sin(phi));

            phidd = kappa0;
            vdd  = vec2_t(-sin(phi), cos(phi))*phid;
		}

		contact = ContactState::Toe;
	}
	// current cop is in the middle
	else{
		v   = vec2_t(cp , 0.0);
		vd  = vec2_t(1.0, 0.0);
		vdd = vec2_t(0.0, 0.0);
		phi = phid = phidd = 0.0;

		contact = ContactState::Surface;
	}

	vec2_t pos2 =  - mat2_t::Rot(-phi)*v;
	vec2_t vel2 = (- mat2_t::Rot(-phi)* vd                 + mat2_t::Rot(-phi + (pi/2.0))*v*phid)*cv;
    vec2_t acc2 = (- mat2_t::Rot(-phi)* vd                 + mat2_t::Rot(-phi + (pi/2.0))*v*phid)*ca;
                  (- mat2_t::Rot(-phi)*(vdd - v*phid*phid) + mat2_t::Rot(-phi + (pi/2.0))*(2.0*vd*phid + v*phidd))*(cv*cv);

	pos    = vec3_t(pos2[0] + cp, 0.0, pos2[1]);
	vel    = vec3_t(vel2[0] + cv, 0.0, vel2[1]);
    acc    = vec3_t(acc2[0] + ca, 0.0, acc2[1]);
	ori    = vec3_t(0.0, phi                  , 0.0);
	angvel = vec3_t(0.0, phid*cv              , 0.0);
    angacc = vec3_t(0.0, phidd*cv*cv + phid*ca, 0.0);
}

void BipedLIP::FootRotationInv(
    real_t ori, real_t angvel, real_t angacc,
	real_t& cp, real_t& cv, real_t& ca){

	real_t phi, phid, phidd;
	real_t d;
	real_t l0     = param.ankleToToe ;
	real_t l1     = param.ankleToHeel;
	real_t rho0   = param.toeCurvature ;
	real_t rho1   = param.heelCurvature;
	real_t r0     = 1.0/rho0;
	real_t r1     = 1.0/rho1;
	real_t kappa0 = param.toeCurvatureRate ;
	real_t kappa1 = param.heelCurvatureRate;

	phi = ori;
	const real_t eps = 1.0e-2;
	if(std::abs(phi) < eps){
		cp = 0.0;
		cv = 0.0;
		ca = 0.0;
	}
	else{
		if(phi < 0.0){
	        if(param.footCurveType == FootCurveType::Arc){
				d     = r1*phi;
				phid  = 1.0/r1;
				phidd = 0.0;
			}
			if(param.footCurveType == FootCurveType::Clothoid){
				d     = -sqrt(-2.0*phi/kappa1);
				phid  = -kappa1*d;
				phidd = -kappa1;
			}
			cp = -l1 + d;
			cv = angvel/phid;
			ca = (angacc - phidd*angvel*angvel)/phid;
		}
		else{
	        if(param.footCurveType == FootCurveType::Arc){
				d     = r0*phi;
				phid  = 1.0/r0;
				phidd = 0.0;
			}
			if(param.footCurveType == FootCurveType::Clothoid){
				d     = sqrt(2.0*phi/kappa0);
				phid  = kappa0*d;
				phidd = kappa0;
			}
			cp = l0 + d;
			cv = angvel/phid;
			ca = (angacc - phidd*angvel*angvel)/phid;
		}
	}
}

// cubic or quintic interpolation
template <typename T>
void Interpolate(
    real_t t ,       T& p ,       T& v ,       T& a ,
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
        { 0.0,  1.0,  0.0, - 6.0,   8.0, -3.0},
        { 0.0,  0.0,  0.0, - 4.0,   7.0, -3.0},
        { 0.0,  0.0,  0.5, - 1.5,   1.5, -0.5},
        { 0.0,  0.0,  0.0,   0.5, - 1.0,  0.5}
    };

    real_t h  = t1 - t0;
    real_t h2 = h*h;
    real_t s  = (t - t0)/h;
    real_t s2 = s*s;
    real_t s3 = s*s2;
    real_t s4 = s*s3;
    real_t s5 = s*s4;

    real_t (*K)[6] = (type == BipedLIP::SwingInterpolation::Cubic ? Kcubic : Kquintic);

    vec6_t kp, kv, ka;
    for(int i = 0; i < 6; i++){
        kp[i] =     K[i][0] +     K[i][1]*s +      K[i][2]*s2 +      K[i][3]*s3 +     K[i][4]*s4 + K[i][5]*s5;
        kv[i] =     K[i][1] + 2.0*K[i][2]*s +  3.0*K[i][3]*s2 +  4.0*K[i][4]*s3 + 5.0*K[i][5]*s4;
        ka[i] = 2.0*K[i][2] + 6.0*K[i][3]*s + 12.0*K[i][4]*s2 + 20.0*K[i][5]*s3;
    }

    p = kp[0]   *p0 + kp[1]   *p1 + kp[2]*h*v0 + kp[3]*h*v1 + kp[4]*h2*a0 + kp[5]*h2*a1;
    v = kv[0]/h *p0 + kv[1]/h *p1 + kv[2]  *v0 + kv[3]  *v1 + kv[4]*h *a0 + kv[5]*h *a1;
    a = ka[0]/h2*p0 + ka[1]/h2*p1 + ka[2]/h*v0 + ka[3]/h*v1 + ka[4]   *a0 + ka[5]   *a1;

}

// 4-th order or 6-th order spline
// both ends are zero (up to 1st or 2nd derivative) and take pmid at midpoint
template <typename T>
void Interpolate2(real_t t, T& p, T& v, T& a, real_t t0, real_t t1, const T& pmid, int type){
    real_t K6[7] = {0.0, 0.0, 0.0,  1.0, -3.0, 3.0, -1.0};
    real_t K4[7] = {0.0, 0.0, 1.0, -2.0,  1.0, 0.0,  0.0};

    real_t h  = t1 - t0;
    real_t h2 = h*h;
    real_t s  = (t - t0)/h;
    real_t s2 = s*s;
    real_t s3 = s*s2;
    real_t s4 = s*s3;
    real_t s5 = s*s4;
    real_t s6 = s*s5;

    real_t* K = (type == BipedLIP::SwingInterpolation::Cubic ? K4 : K6);

    real_t kp, kv, ka;
    kp =     K[0] +     K[1]*s +      K[2]*s2 +      K[3]*s3 +      K[4]*s4 +     K[5]*s5 + K[6]*s6;
    kv =     K[1] + 2.0*K[2]*s +  3.0*K[3]*s2 +  4.0*K[4]*s3 +  5.0*K[5]*s4 + 6.0*K[6]*s5;
    ka = 2.0*K[2] + 6.0*K[3]*s + 12.0*K[4]*s2 + 20.0*K[5]*s3 + 30.0*K[6]*s4;

    T C = (type == BipedLIP::SwingInterpolation::Cubic ? 16.0 : 64.0) * pmid;
    p = kp*C;
    v = kv*C/h;
    a = ka*C/h2;
}

void BipedLIP::FootPose(real_t t, int side, pose_t& pose, vec3_t& vel, vec3_t& angvel, vec3_t& acc, vec3_t& angacc, int& contact) {
	BipedLIPKey* key0  = (BipedLIPKey*)traj.GetSegment(t).first;
	BipedLIPKey* key1  = (BipedLIPKey*)traj.GetSegment(t).second;
	BipedLIPKey* keym1 = (key0->prev ? (BipedLIPKey*)key0->prev : (BipedLIPKey*)0);
	
	//BipedLIPKey* key_c0;   //< last contact phase
	//BipedLIPKey* key_nc0;  //< non-contact phase after c0
	//BipedLIPKey* key_c1;   //< next contact phase

	vec3_t pos;
	vec3_t angle;

	if(key1 != key0->next){
		pose.Pos() = key0->foot[side].var_pos_t->val;
		pose.Ori() = FromRollPitchYaw(vec3_t(0.0, 0.0, key0->foot[side].var_pos_r->val));
		
		vel   .clear();
		angvel.clear();
		acc   .clear();
		angacc.clear();

		contact = ContactState::Surface;

		return;
	}

	// phase
	int ph = phase[key0->tick->idx];

	if(!InContact(ph, side)){
		while(!InContact(phase[key0->tick->idx - 1], side))
			key0 = (BipedLIPKey*)key0->prev;
		keym1 = (BipedLIPKey*)key0->prev;
		while(!InContact(phase[key1->tick->idx], side))
			key1 = (BipedLIPKey*)key1->next;
	}

	real_t t0   = key0->var_time->val;
	real_t t1   = key1->var_time->val;
	real_t tau  = t1 - t0;//key0->var_duration->val;  //< phase duration
	//real_t t1   = t0 + tau;
	real_t dt   = std::max(t - t0, 0.0);    //< elapsed time since phase change
	real_t s    = dt/tau;                   //< normalized time
	vec3_t p0   = key0->foot[side].var_pos_t->val;
	vec3_t p1   = key1->foot[side].var_pos_t->val;
	real_t yaw0 = key0->foot[side].var_pos_r->val;
	real_t yaw1 = key1->foot[side].var_pos_r->val;
	real_t lift = param.swingHeight;
			
	vec3_t c0 = key0 ->foot[side].var_cop_pos->val;
	vec3_t c1 = key1 ->foot[side].var_cop_pos->val;
	vec3_t cv0, cv1;
	vec3_t f_tp, f_tv, f_ta; //< translational pos, vel and acc of foot
	vec3_t f_rp, f_rv, f_ra; //< rotational pos, vel and acc of foot
	vec3_t f_tp_lift , f_tv_lift , f_ta_lift;
	vec3_t f_tp_roll , f_tv_roll , f_ta_roll , f_rp_roll , f_rv_roll , f_ra_roll ;
	vec3_t f_tp_roll0, f_tv_roll0, f_ta_roll0, f_rp_roll0, f_rv_roll0, f_ra_roll0;
	vec3_t f_tp_roll1, f_tv_roll1, f_ta_roll1, f_rp_roll1, f_rv_roll1, f_ra_roll1;
	vec3_t cp, cv, ca;                     //< zmp in absolute coordinate
	vec3_t cp_local, cv_local, ca_local;   //< zmp in foot local

	if( !InContact(ph, side) ){
		// cycloid
		real_t ch   = (s - sin(_2pi*s)/_2pi);
		real_t chd  = ((1.0 - cos(_2pi*s))/tau);
		real_t chdd = ((_2pi*sin(_2pi*s ))/(tau*tau));
		real_t cv   = (1 - cos(_2pi*s))/2.0;
		real_t cvd  = ( _2pi      *sin(_2pi*s)/(2.0*tau)    );
		real_t cvdd = ((_2pi*_2pi)*cos(_2pi*s)/(2.0*tau*tau));
		
		// lateral swinging to avoid scuffing support leg
		vec3_t psup     = key0->foot[!side].var_pos_t->val;
		vec3_t tswg     = p1 - p0;
		real_t tswg_sqr = tswg.square();
		vec3_t nswg;

		const real_t eps = 1.0e-10;
		if(tswg_sqr > eps){
			real_t sproj = (tswg*(psup - p0))/tswg_sqr;
			sproj = std::min(std::max(0.0, sproj), 1.0);
			vec3_t pproj = p0 + sproj*tswg;
			nswg = pproj - psup;
		}
		else{
			nswg = p0 - psup;
		}
			
		real_t nswg_norm = nswg.norm();
		nswg *= (1.0/nswg_norm);

		real_t swing = (nswg_norm < param.minSpacing) ? (param.minSpacing - nswg_norm) : 0.0;
		vec3_t nlift = vec3_t(0.0, 0.0, 1.0);
		
		// footprint rotation
		real_t yaw_diff = yaw1 - yaw0;
		while(yaw_diff >  pi) yaw_diff -= 2.0*pi;
		while(yaw_diff < -pi) yaw_diff += 2.0*pi;

		f_tp = p0 + ch  *(p1 - p0) + cv  *(swing*nswg + lift*nlift);
		f_tv =      chd *(p1 - p0) + cvd *(swing*nswg + lift*nlift);
		f_ta =      chdd*(p1 - p0) + cvdd*(swing*nswg + lift*nlift);
		f_rp.z = yaw0 + ch  *yaw_diff;
		f_rv.z =        chd *yaw_diff;
		f_ra.z =        chdd*yaw_diff;

		contact = ContactState::Float;
	}
	else{
		f_tp = p0;
		f_tv = vec3_t(0.0, 0.0, 0.0);
		f_ta = vec3_t(0.0, 0.0, 0.0);
		f_rp = vec3_t(0.0, 0.0, yaw0);
		f_rv = vec3_t(0.0, 0.0, 0.0);
		f_ra = vec3_t(0.0, 0.0, 0.0);

		contact = ContactState::Surface;
	}
	
	if(param.swingProfile == SwingProfile::HeelToe){
		if( !InContact(ph, side) ){
			cv0  = keym1->foot[side].var_cop_vel->val;
			cv1  = key1 ->foot[side].var_cop_vel->val;

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
				t , f_rp_roll .y, f_rv_roll .y, f_ra_roll.y,
				t0, f_rp_roll0.y, f_rv_roll0.y, 0.0,
				t1, f_rp_roll1.y, f_rv_roll1.y, 0.0,
				param.swingInterpolation
			);
			
			FootRotationInv(
				f_rp_roll.y, f_rv_roll.y, f_ra_roll.y,
				cp_local.x, cv_local.x, ca_local.x);

		}
		else{
			cv0  = key0->foot[side].var_cop_vel->val;
			cv1  = key1->foot[side].var_cop_vel->val;

			// zmp
			Interpolate(
				t , cp, cv , ca,
				t0, c0, cv0, vec3_t(0.0, 0.0, 0.0),
				t1, c1, cv1, vec3_t(0.0, 0.0, 0.0),
				param.swingInterpolation
			);

			// zmp in foot local (approximate assuming yaw rotation is slow enough)
			quat_t q = FromRollPitchYaw(f_rp);
			cp_local = q.Conjugated()*(cp - f_tp);
			cv_local = q.Conjugated()*(cv - f_tv);
			ca_local = q.Conjugated()*(ca - f_ta);
		}
		
		// relative transform from footprint to foot
		FootRotation(
			cp_local.x, cv_local.x, ca_local.x, 
			f_tp_roll, f_tv_roll, f_ta_roll,
			f_rp_roll, f_rv_roll, f_ra_roll,
			contact);

	}

	if( !InContact(ph, side) ){
		contact = ContactState::Float;
	}

	// absolute pose of foot (approixmate)
	quat_t q = FromRollPitchYaw(f_rp);
	pose.Pos() = f_tp + q*f_tp_roll;
	pose.Ori() = q*FromRollPitchYaw(f_rp_roll);
	vel        = f_tv + q*f_tv_roll;
	angvel     = f_rv + q*f_rv_roll;
	acc        = f_ta + q*f_ta_roll;
	angacc     = f_ra + q*f_ra_roll;
}

void BipedLIP::FootCopState(real_t t, int side, vec3_t& pos, vec3_t& vel, real_t& weight) {
	BipedLIPKey* key0 = (BipedLIPKey*)traj.GetSegment(t).first;
	BipedLIPKey* key1 = (BipedLIPKey*)traj.GetSegment(t).second;
	
	if(key0->next == key1){
		real_t dt  = t - key0->var_time->val;
		real_t tau = key0->var_duration->val;
		real_t s   = dt/tau;

		vec3_t c0 = key0->foot[side].var_cop_pos->val;
		vec3_t c1 = key1->foot[side].var_cop_pos->val;
		
		pos    = (1.0 - s)*c0 + s*c1;
		vel    = (c1 - c0)/tau;
		weight = (1.0 - s)*key0->foot[side].weight + s*key1->foot[side].weight;
	}
	else{
		pos = key0->foot[side].var_cop_pos->val;
		vel.clear();
		weight = key0->foot[side].weight;
	}
}

real_t BipedLIP::TValue(real_t t){
	BipedLIPKey* key = (BipedLIPKey*)traj.GetSegment(t).first;

	return key->var_T->val;
}

bool BipedLIP::InContact(int phase, int side){
	return !(              phase == Phase::F  || 
			 (side == 0 && phase == Phase::L) ||
			 (side == 1 && phase == Phase::R));
}

real_t BipedLIP::TimeToLiftoff(real_t t, int side){
	BipedLIPKey* key = (BipedLIPKey*)traj.GetSegment(t).first;
	BipedLIPKey* keyNextFloat = key;

	while(keyNextFloat->next && InContact(phase[keyNextFloat->tick->idx], side))
		keyNextFloat = (BipedLIPKey*)keyNextFloat->next;
	
	// return 1.0 if there is no next float phase in the trajectory
	if(key == keyNextFloat && InContact(phase[key->tick->idx], side))
		return 1.0;

	return keyNextFloat->tick->time - t;
}

real_t BipedLIP::TimeToLanding(real_t t, int side){
	BipedLIPKey* key = (BipedLIPKey*)traj.GetSegment(t).first;
	BipedLIPKey* keyNextContact = key;

	while( keyNextContact->next && !InContact(phase[keyNextContact->tick->idx], side))
		keyNextContact = (BipedLIPKey*)keyNextContact->next;

	// return 1.0 if there is no next contact phase in the trajectory
	if(key == keyNextContact && !InContact(phase[key->tick->idx], side))
		return 1.0;

	return keyNextContact->tick->time - t;
}
		
void BipedLIP::CopState(real_t t, vec3_t& pos, vec3_t& vel, vec3_t& acc) {
	BipedLIPKey* key0 = (BipedLIPKey*)traj.GetSegment(t).first;
	BipedLIPKey* key1 = (BipedLIPKey*)traj.GetSegment(t).second;
	
	real_t dt = t - key0->var_time->val;

	pos = key0->cop_pos + key0->cop_vel*dt + (1.0/2.0)*key0->cop_acc*(dt*dt);
	vel = key0->cop_vel + key0->cop_acc*dt;
	acc = key0->cop_acc;
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
		for (int i = 1; i < trajectory.size(); i++) {
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
		for (int i = 1; i < trajectory.size(); i++) {
			canvas->LineTo(trajectory[i].cop_pos);
		}
		canvas->EndPath();
		canvas->EndLayer();
	}
	if (conf->Set(canvas, Render::Item::BipedFootCop, this)) {
		for(int i = 0; i < 2; i++){
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

	// torso
	if (conf->Set(canvas, Render::Item::BipedTorso, this)) {
		canvas->BeginLayer("biped_torso", true);
		canvas->BeginPath();
		canvas->MoveTo(trajectory[0].torso_pos_t);
		for (int i = 1; i < trajectory.size(); i++) {
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
		for (int i = 1; i < trajectory.size(); i++) {
			canvas->LineTo(trajectory[i].foot_pos_t[0]);
		}
		canvas->MoveTo(trajectory[0].foot_pos_t[1]);
		for (int i = 1; i < trajectory.size(); i++) {
			canvas->LineTo(trajectory[i].foot_pos_t[1]);
		}
		canvas->EndPath();
		canvas->EndLayer();
	}
}

void BipedLIP::CreateSnapshot(real_t t, BipedLIP::Snapshot& s){
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
	FootCopState(t, 0, s.foot_cop[0], vel, weight);
	FootCopState(t, 1, s.foot_cop[1], vel, weight);
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
	for(int i = 0; i < 2; i++){
		v[0] = vec3_t(param.footCopMin[i].x, param.footCopMin[i].y, 0.0);
		v[1] = vec3_t(param.footCopMin[i].x, param.footCopMax[i].y, 0.0);
		v[2] = vec3_t(param.footCopMax[i].x, param.footCopMax[i].y, 0.0);
		v[3] = vec3_t(param.footCopMax[i].x, param.footCopMin[i].y, 0.0);
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
	Constraint(solver, 3, ID(_tag, _obj->node, _obj->tick, _name), Constraint::Type::Equality, _scale) {
	obj[0] = _obj;
	obj[1] = (_obj->next ? (BipedLIPKey*)_obj->next : 0);
}

BipedLipPosCon::BipedLipPosCon(Solver* solver, string _name, BipedLIPKey* _obj, real_t _scale) :
	BipedLipCon(solver, ConTag::BipedLipPos, _name, _obj, _scale) {

	AddSLink (obj[1]->var_com_pos );
	AddSLink (obj[0]->var_com_pos );
	AddSLink (obj[0]->var_com_vel );
	AddC3Link(obj[0]->var_duration);
	AddC3Link(obj[0]->var_T       );

	for(int i = 0; i < 2; i++){
		AddSLink(obj[0]->foot[i].var_cop_pos);
		AddSLink(obj[0]->foot[i].var_cop_vel);
	}
}

BipedLipVelCon::BipedLipVelCon(Solver* solver, string _name, BipedLIPKey* _obj, real_t _scale) :
	BipedLipCon(solver, ConTag::BipedLipVel, _name, _obj, _scale) {

	AddSLink (obj[1]->var_com_vel );
	AddSLink (obj[0]->var_com_pos );
	AddSLink (obj[0]->var_com_vel );
	AddC3Link(obj[0]->var_duration);
	AddC3Link(obj[0]->var_T       );

	for(int i = 0; i < 2; i++){
		AddSLink(obj[0]->foot[i].var_cop_pos);
		AddSLink(obj[0]->foot[i].var_cop_vel);
	}
}

BipedComConP::BipedComConP(Solver* solver, string _name, BipedLIPKey* _obj, real_t _scale) :
	Constraint(solver, 2, ID(ConTag::BipedComPos, _obj->node, _obj->tick, _name), Constraint::Type::Equality, _scale) {

	obj = _obj;

	AddSLink(obj->var_com_pos);
	AddSLink(obj->var_torso_pos_t);

	for(int i = 0; i < 2; i++){
		AddSLink(obj->foot[i].var_pos_t);
	}
}

BipedComConV::BipedComConV(Solver* solver, string _name, BipedLIPKey* _obj, real_t _scale) :
	Constraint(solver, 2, ID(ConTag::BipedComVel, _obj->node, _obj->tick, _name), Constraint::Type::Equality, _scale) {

	obj = _obj;

	AddSLink(obj->var_com_vel);
	AddSLink(obj->var_torso_vel_t);

	for(int i = 0; i < 2; i++){
		AddSLink(obj->foot[i].var_vel_t);
	}
}

BipedFootPosConT::BipedFootPosConT(Solver* solver, string _name, BipedLIPKey* _obj, int _side, real_t _scale) :
	Constraint(solver, 3, ID(ConTag::BipedFootPosT, _obj->node, _obj->tick, _name), Constraint::Type::Equality, _scale) {

	obj[0] = _obj;
	obj[1] = (_obj->next ? (BipedLIPKey*)_obj->next : 0);
	side = _side;

	AddSLink (obj[1]->foot[side].var_pos_t);
	AddSLink (obj[0]->foot[side].var_pos_t);
	AddSLink (obj[0]->foot[side].var_vel_t);
	AddC3Link(obj[0]->var_duration);
}

BipedFootPosConR::BipedFootPosConR(Solver* solver, string _name, BipedLIPKey* _obj, int _side, real_t _scale) :
	Constraint(solver, 1, ID(ConTag::BipedFootPosR, _obj->node, _obj->tick, _name), Constraint::Type::Equality, _scale) {

	obj[0] = _obj;
	obj[1] = (_obj->next ? (BipedLIPKey*)_obj->next : 0);
	side = _side;

	AddSLink(obj[1]->foot[side].var_pos_r);
	AddSLink(obj[0]->foot[side].var_pos_r);
	AddSLink(obj[0]->foot[side].var_vel_r);
	AddSLink(obj[0]->var_duration);
}

BipedFootCopPosCon::BipedFootCopPosCon(Solver* solver, string _name, BipedLIPKey* _obj, int _side, real_t _scale) :
	Constraint(solver, 3, ID(ConTag::BipedFootCop, _obj->node, _obj->tick, _name), Constraint::Type::Equality, _scale) {

	obj[0] = _obj;
	obj[1] = (_obj->next ? (BipedLIPKey*)_obj->next : 0);
	side = _side;

	AddSLink (obj[1]->foot[side].var_cop_pos);
	AddSLink (obj[0]->foot[side].var_cop_pos);
	AddSLink (obj[0]->foot[side].var_cop_vel);
	AddC3Link(obj[0]->var_duration);
}

BipedFootCopVelCon::BipedFootCopVelCon(Solver* solver, string _name, BipedLIPKey* _obj, int _side, real_t _scale) :
	Constraint(solver, 3, ID(ConTag::BipedFootCop, _obj->node, _obj->tick, _name), Constraint::Type::Equality, _scale) {

	obj[0] = _obj;
	obj[1] = (_obj->next ? (BipedLIPKey*)_obj->next : 0);
	side = _side;

	AddSLink (obj[1]->foot[side].var_cop_vel);
	AddSLink (obj[0]->foot[side].var_cop_vel);
	AddSLink (obj[0]->foot[side].var_cop_vel_diff);
}

BipedFootPosRangeConT::BipedFootPosRangeConT(Solver* solver, string _name, BipedLIPKey* _obj, int _side, vec3_t _dir, real_t _scale) :
	Constraint(solver, 1, ID(ConTag::BipedFootPosRangeT, _obj->node, _obj->tick, _name), Constraint::Type::InequalityPenalty, _scale) {

	obj  = _obj;
	side = _side;
	dir  = _dir;

	AddR3Link(obj->foot[side].var_pos_t);
	AddR3Link(obj->var_torso_pos_t);
	AddSLink (obj->var_torso_pos_r);
}

BipedFootPosRangeConR::BipedFootPosRangeConR(Solver* solver, string _name, BipedLIPKey* _obj, int _side, real_t _dir, real_t _scale) :
	Constraint(solver, 1, ID(ConTag::BipedFootPosRangeR, _obj->node, _obj->tick, _name), Constraint::Type::InequalityPenalty, _scale) {

	obj  = _obj;
	side = _side;
	dir  = _dir;

	AddSLink(obj->foot[side].var_pos_r);
	AddSLink(obj->var_torso_pos_r);
}

BipedFootContactCon::BipedFootContactCon(Solver* solver, string _name, BipedLIPKey* _obj, int _side, real_t _scale) :
	Constraint(solver, 1, ID(ConTag::BipedFootPosRangeT, _obj->node, _obj->tick, _name), Constraint::Type::Equality, _scale) {

	obj  = _obj;
	side = _side;

	AddR3Link(obj->foot[side].var_pos_t);
}

BipedFootCopRangeCon::BipedFootCopRangeCon(Solver* solver, string _name, BipedLIPKey* _obj, int _side, vec3_t _dir, real_t _scale) :
	Constraint(solver, 1, ID(ConTag::BipedFootCopRange, _obj->node, _obj->tick, _name), Constraint::Type::InequalityPenalty, _scale) {

	obj  = _obj;
	side = _side;
	dir  = _dir;
	
	AddR3Link(obj->foot[side].var_cop_pos);
	AddR3Link(obj->foot[side].var_pos_t);
	AddSLink (obj->foot[side].var_pos_r);
}

BipedTimeCon::BipedTimeCon(Solver* solver, string _name, BipedLIPKey* _obj, real_t _scale) :
	Constraint(solver, 1, ID(ConTag::BipedTime, _obj->node, _obj->tick, _name), Constraint::Type::Equality, _scale) {

	obj[0] = _obj;
	obj[1] = (BipedLIPKey*)_obj->next;

	AddSLink(obj[1]->var_time);
	AddSLink(obj[0]->var_time);
	AddSLink(obj[0]->var_duration);
}

//-------------------------------------------------------------------------------------------------

void BipedLipPosCon::CalcLhs() {
	Prepare();
	obj[1]->var_com_pos->val = p_rhs;
}

void BipedLipVelCon::CalcLhs() {
	Prepare();
	obj[1]->var_com_vel->val = v_rhs;
}

void BipedFootPosConT::CalcLhs() {
	Prepare();
	obj[1]->foot[side].var_pos_t->val = p0 + v0*tau;
}

void BipedFootPosConR::CalcLhs() {
	Prepare();
	obj[1]->foot[side].var_pos_r->val = theta0 + omega0*tau;
}

void BipedFootCopPosCon::CalcLhs() {
	Prepare();
	obj[1]->foot[side].var_cop_pos->val = c0 + cv0*tau;
}

void BipedFootCopVelCon::CalcLhs() {
	Prepare();
	obj[1]->foot[side].var_cop_vel->val = cv0 + cvd0;
}

void BipedTimeCon::CalcLhs(){
	obj[1]->var_time->val = obj[0]->var_time->val + obj[0]->var_duration->val;
}

//-------------------------------------------------------------------------------------------------

void BipedLipCon::Prepare() {
	BipedLIP::Param& param = ((BipedLIP*)obj[0]->node)->param;
	int phase = ((BipedLIP*)obj[0]->node)->phase[obj[0]->tick->idx];

	p0 = obj[0]->var_com_pos->val;
	v0 = obj[0]->var_com_vel->val;
	c0 = obj[0]->cop_pos;// + vec3_t(0.0, 0.0, param.comHeight);  //< c0 is vrp
	
	if(obj[1]){
		//T  = param.T;
		T  = obj[0]->var_T->val;
		T2 = T*T;
		T3 = T2*T;

		tau  = obj[0]->var_duration->val;
		tau2 = tau*tau;
		C    = cosh(tau/T);
		S    = sinh(tau/T);

		cv0  = obj[0]->cop_vel;
		ca0  = obj[0]->cop_acc;
		g    = vec3_t(0.0, 0.0, param.gravity);
		
		p1   = obj[1]->var_com_pos->val;
		v1   = obj[1]->var_com_vel->val;
		c1   = obj[1]->cop_pos;// + vec3_t(0.0, 0.0, param.comHeight);

		dp = (p0 - c0 - (ca0 + g)*T2);
		dv = (v0 - cv0);

		if(phase == BipedLIP::Phase::F){
			k_p_p   = 1.0;
			k_p_v   = tau;
			k_p_c   = 0.0;
			k_p_cv  = 0.0;
			k_p_ca  = 0.0;
			k_p_tau = v0 - g*tau;
			k_p_T   = vec3_t();

			k_v_p   = 0.0;
			k_v_v   = 1.0;
			k_v_c   = 0.0;
			k_v_cv  = 0.0;
			k_v_ca  = 0.0;
			k_v_tau = -g;
			k_v_T   = vec3_t();
		}
		else{
			k_p_p   = C;
			k_p_v   = T*S;
			k_p_c   = 1.0 - C;
			k_p_cv  = tau - T*S;
			k_p_ca  = T2 + (1.0/2.0)*tau2 - T2*C;
			k_p_tau = cv0 + ca0*tau + (S/T)*dp + C*dv;
			k_p_T   = (2*(1-C)*T)*(ca0 + g) - ((tau*S)/T2)*dp + (S - ((tau*C)/T))*dv;

			k_v_p   = S/T;
			k_v_v   = C;
			k_v_c   = -S/T;
			k_v_cv  = 1.0 - C;
			k_v_ca  = tau - T*S;
			k_v_tau = ca0 + (C/T2)*dp + (S/T)*dv;
			k_v_T   = -(S/T2)*(p0 - c0 + (ca0 + g)*T2) - ((tau*C)/T3)*dp - ((tau*S)/T2)*dv;
		}

		for(int i = 0; i < 2; i++){
			real_t w0 = obj[0]->foot[i].weight;
			real_t w1 = obj[1]->foot[i].weight;

			k_c_c  [i] = w0;
			k_cv_c [i] = (w1 - w0)/tau;
			k_cv_cv[i] = w0;
			k_ca_cv[i] = 2.0*(w1 - w0)/tau;
		}

		if(phase == BipedLIP::Phase::F){
			p_rhs = p0 + v0*tau - (1.0/2.0)*g*tau2;
			v_rhs = v0 - g*tau;
		}
		else{
			p_rhs = c0 + (ca0 + g)*T2 + cv0*tau + (1.0/2.0)*ca0*tau2 + C*dp + (S*T)*dv;
			v_rhs = cv0 + ca0*tau + (S/T)*dp + C*dv;
		}
	}
}

void BipedFootPosConT::Prepare(){
	p1  = obj[1]->foot[side].var_pos_t->val;
	p0  = obj[0]->foot[side].var_pos_t->val;
	v0  = obj[0]->foot[side].var_vel_t->val;
	tau = obj[0]->var_duration->val;
}

void BipedFootPosConR::Prepare(){
	theta1 = obj[1]->foot[side].var_pos_r->val;
	theta0 = obj[0]->foot[side].var_pos_r->val;
	omega0 = obj[0]->foot[side].var_vel_r->val;
	tau    = obj[0]->var_duration->val;
}

void BipedFootCopPosCon::Prepare(){
	c1  = obj[1]->foot[side].var_cop_pos->val;
	c0  = obj[0]->foot[side].var_cop_pos->val;
	cv0 = obj[0]->foot[side].var_cop_vel->val;
	tau = obj[0]->var_duration->val;
}

void BipedFootCopVelCon::Prepare(){
	cv1  = obj[1]->foot[side].var_cop_vel->val;
	cv0  = obj[0]->foot[side].var_cop_vel->val;
	cvd0 = obj[0]->foot[side].var_cop_vel_diff->val;
}

void BipedFootPosRangeConT::Prepare(){
	r     = obj->foot[side].var_pos_t->val - obj->var_torso_pos_t->val;
	theta = obj->var_torso_pos_r->val;

	R       = mat3_t::Rot(theta, 'z');
	ez      = vec3_t(0.0, 0.0, 1.0);
	dir_abs = R*dir;
}

void BipedFootPosRangeConR::Prepare(){
	thetaf = obj->foot[side].var_pos_r->val;
	thetat = obj->var_torso_pos_r->val;

	r = thetaf - thetat;
	
	while(r >  pi) r -= _2pi;
	while(r < -pi) r += _2pi;
}

void BipedFootContactCon::Prepare(){
}

void BipedFootCopRangeCon::Prepare(){
	r     = obj->foot[side].var_cop_pos->val - obj->foot[side].var_pos_t->val;
	theta = obj->foot[side].var_pos_r->val;

	R       = mat3_t::Rot(theta, 'z');
	ez      = vec3_t(0.0, 0.0, 1.0);
	dir_abs = R*dir;
}

//-------------------------------------------------------------------------------------------------

void BipedLipPosCon::CalcCoef() {
	Prepare();

	int idx = 0;
	((SLink *)links[idx++])->SetCoef( 1.0);
	((SLink *)links[idx++])->SetCoef(-k_p_p);
	((SLink *)links[idx++])->SetCoef(-k_p_v);
	((C3Link*)links[idx++])->SetCoef(-k_p_tau);
	((C3Link*)links[idx++])->SetCoef(-k_p_T);

	for(int i = 0; i < 2; i++){
		((SLink*)links[idx++])->SetCoef(-(k_p_c *k_c_c  [i] + k_p_cv*k_cv_c [i]));
		((SLink*)links[idx++])->SetCoef(-(k_p_cv*k_cv_cv[i] + k_p_ca*k_ca_cv[i]));
	}
}

void BipedLipVelCon::CalcCoef() {
	Prepare();

	int idx = 0;
	((SLink *)links[idx++])->SetCoef( 1.0);
	((SLink *)links[idx++])->SetCoef(-k_v_p);
	((SLink *)links[idx++])->SetCoef(-k_v_v);
	((C3Link*)links[idx++])->SetCoef(-k_v_tau);
	((C3Link*)links[idx++])->SetCoef(-k_v_T);

	for(int i = 0; i < 2; i++){
		((SLink*)links[idx++])->SetCoef(-(k_v_c *k_c_c  [i] + k_v_cv*k_cv_c [i]));
		((SLink*)links[idx++])->SetCoef(-(k_v_cv*k_cv_cv[i] + k_v_ca*k_ca_cv[i]));
	}
}

void BipedFootPosConT::CalcCoef() {
	Prepare();

	((SLink *)links[0])->SetCoef( 1.0);
	((SLink *)links[1])->SetCoef(-1.0);
	((SLink *)links[2])->SetCoef(-tau);
	((C3Link*)links[3])->SetCoef(-v0 );
}

void BipedFootPosConR::CalcCoef() {
	Prepare();

	((SLink*)links[0])->SetCoef( 1.0   );
	((SLink*)links[1])->SetCoef(-1.0   );
	((SLink*)links[2])->SetCoef(-tau   );
	((SLink*)links[3])->SetCoef(-omega0);
}

void BipedFootCopPosCon::CalcCoef() {
	Prepare();

	((SLink *)links[0])->SetCoef( 1.0);
	((SLink *)links[1])->SetCoef(-1.0);
	((SLink *)links[2])->SetCoef(-tau);
	((C3Link*)links[3])->SetCoef(-cv0);
}

void BipedFootCopVelCon::CalcCoef() {
	Prepare();

	((SLink *)links[0])->SetCoef( 1.0);
	((SLink *)links[1])->SetCoef(-1.0);
	((SLink *)links[2])->SetCoef(-1.0);
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
	((SLink*)links[2])->SetCoef(-mf/msum);
	((SLink*)links[3])->SetCoef(-mf/msum);
}

void BipedFootPosRangeConT::CalcCoef() {
	Prepare();

	((R3Link*)links[0])->SetCoef( dir_abs);
	((R3Link*)links[1])->SetCoef(-dir_abs);
	//((SLink* )links[2])->SetCoef( (ez % dir_abs)*r);
	((SLink* )links[2])->SetCoef( (dir_abs % r)*ez);
}

void BipedFootPosRangeConR::CalcCoef() {
	Prepare();

	((SLink*)links[0])->SetCoef( dir);
	((SLink*)links[1])->SetCoef(-dir);
}

void BipedFootContactCon::CalcCoef() {
	Prepare();

	((R3Link*)links[0])->SetCoef(vec3_t(0.0, 0.0, 1.0));
}

void BipedFootCopRangeCon::CalcCoef() {
	Prepare();
	
	((R3Link*)links[0])->SetCoef( dir_abs);
	((R3Link*)links[1])->SetCoef(-dir_abs);
	//((SLink* )links[2])->SetCoef( (ez % dir_abs)*r);
	((SLink* )links[2])->SetCoef( (dir_abs % r)*ez);
}

void BipedTimeCon::CalcCoef() {
	((SLink*)links[0])->SetCoef(1.0);
	((SLink*)links[1])->SetCoef(-1.0);
	((SLink*)links[2])->SetCoef(-1.0);
}

//-------------------------------------------------------------------------------------------------

void BipedLipPosCon::CalcDeviation() {
	y = p1 - p_rhs;
}

void BipedLipVelCon::CalcDeviation() {
	y = v1 - v_rhs;
}

void BipedFootPosConT::CalcDeviation() {
	y = p1 - (p0 + v0*tau);
}

void BipedFootPosConR::CalcDeviation() {
	y[0] = theta1 - (theta0 + omega0*tau);
}

void BipedFootCopPosCon::CalcDeviation() {
	y = c1 - (c0 + cv0*tau);
}

void BipedFootCopVelCon::CalcDeviation() {
	y = cv1 - (cv0 + cvd0);
}

void BipedFootPosRangeConT::CalcDeviation(){
	real_t s = dir_abs*r;
	
	if(s < bound) {
		y[0] = s - bound;
		active = true;
	}
	else{
		y[0] = 0.0;
		active = false;
	}
}

void BipedFootPosRangeConR::CalcDeviation() {
	real_t s = dir*r;

	if(s < bound) {
		y[0] = s - bound;
		active = true;
	}
	else{
		y[0] = 0.0;
		active = false;
	}
}

void BipedFootContactCon::CalcDeviation(){
	y[0] = obj->foot[side].var_pos_t->val.z;
}

void BipedFootCopRangeCon::CalcDeviation(){
	real_t s = dir_abs*r;
	
	if(s < bound) {
		y[0] = s - bound;
		active = true;
	}
	else{
		y[0] = 0.0;
		active = false;
	}
}

}
