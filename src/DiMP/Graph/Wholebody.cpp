#include <DiMP/Graph/Wholebody.h>
#include <DiMP/Graph/Graph.h>
#include <DiMP/Render/Config.h>
#include <DiMP/Render/Canvas.h>

#include <omp.h>

#include <sbtimer.h>
static Timer timer;
static Timer timer2;

using namespace PTM;

namespace DiMP {;

const real_t _pi     = M_PI;
const real_t inf     = numeric_limits<real_t>::max();
const real_t damping = 0.1;
const vec3_t one(1.0, 1.0, 1.0);

//-------------------------------------------------------------------------------------------------
// IKData

WholebodyData::End::End(){
	pos_t_weight       = one;
	pos_r_weight       = one;
	vel_t_weight       = one;
	vel_r_weight       = one;
	acc_t_weight       = one;
	acc_r_weight       = one;
	force_t_weight     = one;
	force_r_weight     = one;

	state  = Wholebody::ContactState::Free;
	mu     = 0.5;
}

WholebodyData::Base::Base(){
	pos_r_weight = one;
	vel_r_weight = one;
	acc_r_weight = one;
}

WholebodyData::Centroid::Centroid(){
	pos_t_weight = one;
	vel_t_weight = one;
	pos_r_weight = one;
	vel_r_weight = one;
	//Ld_weight    = one;
	L_weight     = one;
}

void WholebodyData::Init(Wholebody* wb){
	int nlink  = (int)wb->links .size();
	int nend   = (int)wb->ends  .size();
	int nchain = (int)wb->chains.size();
	int nlimit = (int)wb->limits.size();
	int njoint = (int)wb->joints.size();

	links.resize(nlink );
	ends .resize(nend  );
	q    .resize(njoint, 0.0);
	qd   .resize(njoint, 0.0);
	qdd  .resize(njoint, 0.0);
	tau  .resize(njoint, 0.0);
	e    .resize(nlimit, 0.0);

	q_weight  .resize(njoint, 1.0);
	qd_weight .resize(njoint, 1.0);
	qdd_weight.resize(njoint, 1.0);

	q_min.resize(njoint, -inf);
	q_max.resize(njoint,  inf);

	Jq.resize(nchain);
	Je.resize(nchain);
	for(int i = 0; i < nchain; i++){
		Jq[i].resize(wb->chains[i].ilink .size(), 6, 0.0);
		Je[i].resize(wb->chains[i].ilimit.size(), 6, 0.0);
	}
}

void WholebodyData::InitJacobian(Wholebody* wb){
	int nlink  = (int)wb->links .size();
	int nend   = (int)wb->ends  .size();
	int nlimit = (int)wb->limits.size();
	int njoint = (int)wb->joints.size();
		
	Jik_e_v0.resize(nlimit, 6, 0.0);
	Jik_e_ve.resize(nend);
	for(int i = 0; i < nend; i++){
		Jik_e_ve[i].resize(nlimit, 6, 0.0);
	}
	Jik_q_v0.resize(njoint, 6, 0.0);
	Jik_q_ve.resize(nend);
	for(int i = 0; i < nend; i++){
		Jik_q_ve[i].resize(njoint, 6, 0.0);
	}
	Jfk_vi_v0.resize(nlink);
	Jfk_vi_q .resize(nlink);
	for(int i = 0; i < nlink; i++){
		Jfk_vi_v0[i].clear();
		Jfk_vi_q [i].resize(6, njoint, 0.0);
	}
	
	Jfkik_vi_v0.resize(nlink);
	Jfkik_vi_ve.resize(nlink);
	for(int i = 0; i < nlink; i++){
		Jfkik_vi_v0[i].clear();
		Jfkik_vi_ve[i].resize(nend);
		for(int j = 0; j < nend; j++){
			Jfkik_vi_ve[i][j].clear();
		}
	}

}

//-------------------------------------------------------------------------------------------------
// WholebodyKey

WholebodyKey::WholebodyKey() {
	
}

void WholebodyKey::AddVar(Solver* solver) {
	wb = (Wholebody*)node;

	int nend   = (int)wb->ends  .size();
	int nlink  = (int)wb->links .size();
	int njoint = (int)wb->joints.size();
	
	ends  .resize(nend);
	joints.resize(njoint);

	stringstream ss, ss2;

	// states
	centroid.var_pos_t = new V3Var(solver, ID(VarTag::WholebodyPosT, node, tick, name + "_centroid_pos_t"), wb->spt);
	centroid.var_pos_r = new QVar (solver, ID(VarTag::WholebodyPosR, node, tick, name + "_centroid_pos_r"), wb->spr);
	centroid.var_vel_t = new V3Var(solver, ID(VarTag::WholebodyVelT, node, tick, name + "_centroid_vel_t"), wb->svt);
	centroid.var_vel_r = new V3Var(solver, ID(VarTag::WholebodyVelR, node, tick, name + "_centroid_vel_r"), wb->svr);
	centroid.var_pos_t->weight = damping*one;
	centroid.var_pos_r->weight = damping*one;
	centroid.var_vel_t->weight = damping*one;
	centroid.var_vel_r->weight = damping*one;
	solver->AddStateVar(centroid.var_pos_t, tick->idx);
	solver->AddStateVar(centroid.var_pos_r, tick->idx);
	solver->AddStateVar(centroid.var_vel_t, tick->idx);
	solver->AddStateVar(centroid.var_vel_r, tick->idx);

	base.var_pos_r = new QVar (solver, ID(VarTag::WholebodyPosR, node, tick, name + "_base_pos_r"), wb->spr);
	base.var_vel_r = new V3Var(solver, ID(VarTag::WholebodyVelR, node, tick, name + "_base_vel_r"), wb->svr);
	base.var_pos_r->weight = damping*one;
	base.var_vel_r->weight = damping*one;
	solver->AddStateVar(base.var_pos_r, tick->idx);
	solver->AddStateVar(base.var_vel_r, tick->idx);

	if(wb->param.parametrization == Wholebody::Parametrization::Joint){
		for(int i = 0; i < njoint; i++){
			ss.str("");
			ss << name << "_joint" << i;

			joints[i].var_q  = new SVar(solver, ID(VarTag::WholebodyJointPos, node, tick, ss.str() + "_q" ), wb->spr);
			joints[i].var_qd = new SVar(solver, ID(VarTag::WholebodyJointVel, node, tick, ss.str() + "_qd"), wb->svr);
			joints[i].var_q ->weight[0] = damping;
			joints[i].var_qd->weight[0] = damping;
			solver->AddStateVar(joints[i].var_q , tick->idx);
			solver->AddStateVar(joints[i].var_qd, tick->idx);
		}
	}

	if(wb->param.parametrization == Wholebody::Parametrization::End){
		for(int i = 0; i < nend; i++){
			ss.str("");
			ss << name << "_end" << i;

			ends[i].var_pos_t   = new V3Var(solver, ID(VarTag::WholebodyPosT, node, tick, ss.str() + "_pos_t"), wb->spt);
			ends[i].var_pos_r   = new QVar (solver, ID(VarTag::WholebodyPosR, node, tick, ss.str() + "_pos_r"), wb->spr);
			ends[i].var_vel_t   = new V3Var(solver, ID(VarTag::WholebodyVelT, node, tick, ss.str() + "_vel_t"), wb->svt);
			ends[i].var_vel_r   = new V3Var(solver, ID(VarTag::WholebodyVelR, node, tick, ss.str() + "_vel_r"), wb->svr);
			ends[i].var_pos_t  ->weight = damping*one;
			ends[i].var_pos_r  ->weight = damping*one;
			ends[i].var_vel_t  ->weight = damping*one;
			ends[i].var_vel_r  ->weight = damping*one;
			solver->AddStateVar(ends[i].var_pos_t  , tick->idx);
			solver->AddStateVar(ends[i].var_pos_r  , tick->idx);
			solver->AddStateVar(ends[i].var_vel_t  , tick->idx);	
			solver->AddStateVar(ends[i].var_vel_r  , tick->idx);
		}
	}

	// inputs
	if(next){
		base.var_acc_r = new V3Var(solver, ID(VarTag::WholebodyAccR, node, tick, name + "_base_acc_r"), wb->sar);
		base.var_acc_r->weight = damping*one;
		solver->AddInputVar(base.var_acc_r, tick->idx);

		if(wb->param.parametrization == Wholebody::Parametrization::Joint){
			for(int i = 0; i < njoint; i++){
				ss.str("");
				ss << name << "_joint" << i;

				joints[i].var_qdd = new SVar(solver, ID(VarTag::WholebodyJointAcc, node, tick, ss.str() + "_qdd" ), wb->sar);
				joints[i].var_qdd->weight[0] = damping;
				solver->AddInputVar(joints[i].var_qdd , tick->idx);
			}
		}

		if(wb->param.parametrization == Wholebody::Parametrization::End){
			for(int i = 0; i < nend; i++){
				ss.str("");
				ss << name << "_end" << i;

				ends[i].var_acc_t   = new V3Var(solver, ID(VarTag::WholebodyAccT  , node, tick, ss.str() + "_acc_t"  ), wb->sat);
				ends[i].var_acc_r   = new V3Var(solver, ID(VarTag::WholebodyAccR  , node, tick, ss.str() + "_acc_r"  ), wb->sar);
				ends[i].var_acc_t  ->weight = damping*one;
				ends[i].var_acc_r  ->weight = damping*one;
				solver->AddInputVar(ends[i].var_acc_t  , tick->idx);
				solver->AddInputVar(ends[i].var_acc_r  , tick->idx);
			}
		}
		for(int i = 0; i < nend; i++){
			ss.str("");
			ss << name << "_end" << i;

			ends[i].var_force_t = new V3Var(solver, ID(VarTag::WholebodyForceT, node, tick, ss.str() + "_force_t"), wb->sft);
			ends[i].var_force_r = new V3Var(solver, ID(VarTag::WholebodyForceR, node, tick, ss.str() + "_force_r"), wb->sfr);
			ends[i].var_force_t->weight = damping*one;
			ends[i].var_force_r->weight = damping*one;
			solver->AddInputVar(ends[i].var_force_t, tick->idx);
			solver->AddInputVar(ends[i].var_force_r, tick->idx);
		}
	}

	data.Init(wb);
	data.InitJacobian(wb);
	data_des.Init(wb);
	data_tmp.resize(3 + 6*nend);
	for(auto& d : data_tmp){
		d.Init(wb);
	}	
}

void WholebodyKey::AddCon(Solver* solver) {
	WholebodyKey* nextObj = (WholebodyKey*)next;

    int nend   = (int)wb->ends  .size();
	int njoint = (int)wb->joints.size();
	int nchain = (int)wb->chains.size();
	int nlimit = (int)wb->limits.size();
	int k = tick->idx;
    
	stringstream ss;

	if(next){
		centroid.con_pos_t = new WholebodyCentroidPosConT(solver, name + "_centroid_pos_t", this, wb->spt);
		centroid.con_vel_t = new WholebodyCentroidVelConT(solver, name + "_centroid_vel_t", this, wb->svt);
		centroid.con_pos_r = new WholebodyCentroidPosConR(solver, name + "_centroid_pos_r", this, wb->spr);
		centroid.con_vel_r = new WholebodyCentroidVelConR(solver, name + "_centroid_vel_r", this, wb->svr);
		solver->AddTransitionCon(centroid.con_pos_t, tick->idx);
		solver->AddTransitionCon(centroid.con_vel_t, tick->idx);
		solver->AddTransitionCon(centroid.con_pos_r, tick->idx);
		solver->AddTransitionCon(centroid.con_vel_r, tick->idx);

		base.con_pos_r = new WholebodyPosConR(solver, name + "_base_pos_r", this, -1, wb->spr);
		base.con_vel_r = new WholebodyVelConR(solver, name + "_base_vel_r", this, -1, wb->svr);        
		solver->AddTransitionCon(base.con_pos_r, tick->idx);
		solver->AddTransitionCon(base.con_vel_r, tick->idx);
	}

	if(prev){
		centroid.con_des_pos_t = new FixConV3(solver, ID(ConTag::WholebodyPosT, node, tick, name + "_des_centroid_pos_t"), centroid.var_pos_t, wb->spt);
		centroid.con_des_vel_t = new FixConV3(solver, ID(ConTag::WholebodyVelT, node, tick, name + "_des_centroid_vel_t"), centroid.var_vel_t, wb->svt);
		centroid.con_des_pos_r = new FixConQ (solver, ID(ConTag::WholebodyPosR, node, tick, name + "_des_centroid_pos_r"), centroid.var_pos_r, wb->spr);
		centroid.con_des_vel_r = new FixConV3(solver, ID(ConTag::WholebodyVelR, node, tick, name + "_des_centroid_vel_r"), centroid.var_vel_r, wb->svr);
		solver->AddCostCon(centroid.con_des_pos_t, tick->idx);
		solver->AddCostCon(centroid.con_des_vel_t, tick->idx);
		solver->AddCostCon(centroid.con_des_pos_r, tick->idx);
		solver->AddCostCon(centroid.con_des_vel_r, tick->idx);
	
		base.con_des_pos_r = new FixConQ (solver, ID(ConTag::WholebodyPosR, node, tick, name + "_des_base_pos_r"), base.var_pos_r, wb->spr);
		base.con_des_vel_r = new FixConV3(solver, ID(ConTag::WholebodyVelR, node, tick, name + "_des_base_vel_r"), base.var_vel_r, wb->svr);
		solver->AddCostCon(base.con_des_pos_r, tick->idx);
		solver->AddCostCon(base.con_des_vel_r, tick->idx);
	}

	if(next){
		base.con_des_acc_r = new FixConV3(solver, ID(ConTag::WholebodyAccR, node, tick, name + "_des_base_acc_r"), base.var_acc_r, wb->sar);
		solver->AddCostCon(base.con_des_acc_r, tick->idx);
	}

	if(wb->param.parametrization == Wholebody::Parametrization::End && prev){	
		con_limit.resize(nlimit);
		for(int j = 0; j < nlimit; j++){
			ss.str("");
			ss << name << "_lim" << j;
			
			con_limit[j] = new WholebodyLimitCon(solver, ss.str(), this, j, wb->limits[j].type, wb->limits[j].scale);
			solver->AddCostCon(con_limit[j], tick->idx);
		}
	}

	if(next && prev){
		//centroid.con_Ld = new WholebodyLdCon(solver, name + "_Ld", this, wb->sfr);
		//solver->AddCostCon(centroid.con_Ld, tick->idx);

		centroid.con_L = new WholebodyLCon(solver, name + "_L", this, wb->sL);
		solver->AddCostCon(centroid.con_L, tick->idx);
	}

	if(wb->param.parametrization == Wholebody::Parametrization::Joint){
		for(int i = 0; i < njoint; i++){
			ss.str("");
			ss << name << "_joint" << i;

			if(next){
				joints[i].con_q  = new WholebodyJointPosCon(solver, ss.str() + "_q" , this, i, wb->spr);
				joints[i].con_qd = new WholebodyJointVelCon(solver, ss.str() + "_qd", this, i, wb->svr);        
				solver->AddTransitionCon(joints[i].con_q , tick->idx);
				solver->AddTransitionCon(joints[i].con_qd, tick->idx);

				joints[i].con_des_qdd = new FixConS(solver, ID(ConTag::WholebodyJointAcc, node, tick, ss.str() + "_des_qdd"), joints[i].var_qdd, wb->sar);
				solver->AddCostCon(joints[i].con_des_qdd, tick->idx);
			}
			if(prev){
				joints[i].con_des_q  = new FixConS(solver, ID(ConTag::WholebodyJointPos, node, tick, ss.str() + "_des_q" ), joints[i].var_q , wb->spr);
				joints[i].con_des_qd = new FixConS(solver, ID(ConTag::WholebodyJointVel, node, tick, ss.str() + "_des_qd"), joints[i].var_qd, wb->svr);
				solver->AddCostCon(joints[i].con_des_q , tick->idx);
				solver->AddCostCon(joints[i].con_des_qd, tick->idx);

				joints[i].con_range_q = new RangeConS(solver, ID(ConTag::WholebodyJointPos, node, tick, ss.str() + "_range_q"), joints[i].var_q, wb->spr);
				solver->AddCostCon(joints[i].con_range_q, tick->idx);
			}
		}
	}
	
	for(int i = 0; i < nend; i++){
		ss.str("");
		ss << name << "_end" << i;

		if(wb->param.parametrization == Wholebody::Parametrization::End && next){
			ends[i].con_pos_t   = new WholebodyPosConT  (solver, ss.str() + "_pos_t"  , this, i, wb->spt);
			ends[i].con_pos_r   = new WholebodyPosConR  (solver, ss.str() + "_pos_r"  , this, i, wb->spr);
			ends[i].con_vel_t   = new WholebodyVelConT  (solver, ss.str() + "_vel_t"  , this, i, wb->svt);
			ends[i].con_vel_r   = new WholebodyVelConR  (solver, ss.str() + "_vel_r"  , this, i, wb->svr);        
			solver->AddTransitionCon(ends[i].con_pos_t  , tick->idx);
			solver->AddTransitionCon(ends[i].con_pos_r  , tick->idx);
			solver->AddTransitionCon(ends[i].con_vel_t  , tick->idx);
			solver->AddTransitionCon(ends[i].con_vel_r  , tick->idx);
		}
		
		if(prev){
			ends[i].con_des_pos_t   = new WholebodyDesPosConT(solver, ss.str() + "_des_pos_t", this, i, wb->spt);
			ends[i].con_des_vel_t   = new WholebodyDesVelConT(solver, ss.str() + "_des_vel_t", this, i, wb->svt);
			ends[i].con_des_pos_r   = new WholebodyDesPosConR(solver, ss.str() + "_des_pos_r", this, i, wb->spr);
			ends[i].con_des_vel_r   = new WholebodyDesVelConR(solver, ss.str() + "_des_vel_r", this, i, wb->svr);
			solver->AddCostCon(ends[i].con_des_pos_t  , tick->idx);
			solver->AddCostCon(ends[i].con_des_vel_t  , tick->idx);
			solver->AddCostCon(ends[i].con_des_pos_r  , tick->idx);
			solver->AddCostCon(ends[i].con_des_vel_r  , tick->idx);
		}	
		if(next){
			ends[i].con_force_normal         = new WholebodyNormalForceCon  (solver, ss.str() + "_force_normal"  , this, i, wb->sft);
			ends[i].con_force_friction[0][0] = new WholebodyFrictionForceCon(solver, ss.str() + "_force_friction", this, i, 0, 0, wb->sft);
			ends[i].con_force_friction[0][1] = new WholebodyFrictionForceCon(solver, ss.str() + "_force_friction", this, i, 0, 1, wb->sft);
			ends[i].con_force_friction[1][0] = new WholebodyFrictionForceCon(solver, ss.str() + "_force_friction", this, i, 1, 0, wb->sft);
			ends[i].con_force_friction[1][1] = new WholebodyFrictionForceCon(solver, ss.str() + "_force_friction", this, i, 1, 1, wb->sft);
			ends[i].con_moment[0][0]         = new WholebodyMomentCon       (solver, ss.str() + "_moment"        , this, i, 0, 0, wb->sfr);
			ends[i].con_moment[0][1]         = new WholebodyMomentCon       (solver, ss.str() + "_moment"        , this, i, 0, 1, wb->sfr);
			ends[i].con_moment[1][0]         = new WholebodyMomentCon       (solver, ss.str() + "_moment"        , this, i, 1, 0, wb->sfr);
			ends[i].con_moment[1][1]         = new WholebodyMomentCon       (solver, ss.str() + "_moment"        , this, i, 1, 1, wb->sfr);
		    
			solver->AddCostCon(ends[i].con_force_normal        , tick->idx);
			solver->AddCostCon(ends[i].con_force_friction[0][0], tick->idx);
			solver->AddCostCon(ends[i].con_force_friction[0][1], tick->idx);
			solver->AddCostCon(ends[i].con_force_friction[1][0], tick->idx);
			solver->AddCostCon(ends[i].con_force_friction[1][1], tick->idx);
			solver->AddCostCon(ends[i].con_moment[0][0]        , tick->idx);
			solver->AddCostCon(ends[i].con_moment[0][1]        , tick->idx);
			solver->AddCostCon(ends[i].con_moment[1][0]        , tick->idx);
			solver->AddCostCon(ends[i].con_moment[1][1]        , tick->idx);
		}

		if(prev){
			ends[i].con_contact_pos_t = new WholebodyContactPosConT(solver, ss.str() + "_contact_pos_t", this, i, wb->spt);
			ends[i].con_contact_pos_r = new WholebodyContactPosConR(solver, ss.str() + "_contact_pos_r", this, i, wb->spr);
			ends[i].con_contact_vel_t = new WholebodyContactVelConT(solver, ss.str() + "_contact_vel_t", this, i, wb->svt);
			ends[i].con_contact_vel_r = new WholebodyContactVelConR(solver, ss.str() + "_contact_vel_r", this, i, wb->svr);
			solver->AddCostCon(ends[i].con_contact_pos_t, tick->idx);
			solver->AddCostCon(ends[i].con_contact_pos_r, tick->idx);
			solver->AddCostCon(ends[i].con_contact_vel_t, tick->idx);
			solver->AddCostCon(ends[i].con_contact_vel_r, tick->idx);
		}

		if(wb->param.parametrization == Wholebody::Parametrization::End && next){
			ends[i].con_des_acc_t   = new FixConV3(solver, ID(ConTag::WholebodyAccT      , node, tick, ss.str() + "_des_acc_t"      ), ends[i].var_acc_t      , wb->sat );
			ends[i].con_des_acc_r   = new FixConV3(solver, ID(ConTag::WholebodyAccR      , node, tick, ss.str() + "_des_acc_r"      ), ends[i].var_acc_r      , wb->sar );
			solver->AddCostCon(ends[i].con_des_acc_t      , tick->idx);
			solver->AddCostCon(ends[i].con_des_acc_r      , tick->idx);
        }

		if(next){
			ends[i].con_des_force_t = new FixConV3(solver, ID(ConTag::WholebodyForceT, node, tick, ss.str() + "_des_force_t"), ends[i].var_force_t, wb->sft);
			ends[i].con_des_force_r = new FixConV3(solver, ID(ConTag::WholebodyForceR, node, tick, ss.str() + "_des_force_r"), ends[i].var_force_r, wb->sfr);
			solver->AddCostCon(ends[i].con_des_force_t, tick->idx);
			solver->AddCostCon(ends[i].con_des_force_r, tick->idx);
		}
	}
}

void WholebodyKey::Prepare() {
	int nlink  = (int)wb->links .size();
	int nend   = (int)wb->ends  .size();
	int njoint = (int)wb->joints.size();

	// copy variables to data
	data.centroid.pos_t = centroid.var_pos_t->val;
	data.centroid.vel_t = centroid.var_vel_t->val;
	data.centroid.pos_r = centroid.var_pos_r->val;
	data.centroid.vel_r = centroid.var_vel_r->val;

	data.base.pos_r = base.var_pos_r->val;
	data.base.vel_r = base.var_vel_r->val;

	if(next){
		data.base.acc_r = base.var_acc_r->val;
	}

	if(wb->param.parametrization == Wholebody::Parametrization::Joint){
		for(int i = 0; i < njoint; i++){
			data.q [i] = joints[i].var_q ->val;
			data.qd[i] = joints[i].var_qd->val;

			if(next){
				data.qdd[i] = joints[i].var_qdd->val;
			}
		}
	}

	for(int i = 0; i < nend; i++){
		End& end = ends[i];

		WholebodyData::End & dend = data.ends [i];
		
		if(wb->param.parametrization == Wholebody::Parametrization::End){
			dend.pos_t = end.var_pos_t->val;
			dend.pos_r = end.var_pos_r->val;
			dend.vel_t = end.var_vel_t->val;
			dend.vel_r = end.var_vel_r->val;

			if(next){
				dend.acc_t = end.var_acc_t->val;
				dend.acc_r = end.var_acc_r->val;
			}
		}

		if(next){
			dend.force_t = end.var_force_t->val;
			dend.force_r = end.var_force_r->val;
		}
	}

	bool fk_or_ik = (wb->param.parametrization == Wholebody::Parametrization::Joint);
	wb->CalcPosition          (data, fk_or_ik);
	wb->CalcJacobian          (data, data_tmp);
	wb->CalcVelocity          (data, fk_or_ik);
	wb->CalcAcceleration      (data, fk_or_ik);
	wb->CalcMomentum          (data);
	wb->CalcMomentumDerivative(data);
	wb->CalcForce             (data);

	// working variables
	if(next){
		q0 = centroid.var_pos_r->val;
		q0.ToMatrix(R0);

		fe .resize(nend);
		me .resize(nend);
		re .resize(nend);
		rec.resize(nend);
		
		if(wb->param.parametrization == Wholebody::Parametrization::Joint){
			J_L_q   .resize(3, njoint, 0.0);
			J_L_qd  .resize(3, njoint, 0.0);
			J_Ld_q  .resize(3, njoint, 0.0);
			J_Ld_qdd.resize(3, njoint, 0.0);
		}
		else{
			J_L_pe.resize(nend);
			J_L_qe.resize(nend);
			J_L_ve.resize(nend);
			J_L_we.resize(nend);
			J_Ld_pe.resize(nend);
			J_Ld_qe.resize(nend);
			J_Ld_ae.resize(nend);
			J_Ld_ue.resize(nend);
		}

		fsum.clear();
		msum.clear();
		for(int i = 0; i < nend; i++){
			WholebodyData::End& dend = data.ends[i];
			
			re [i] = data.centroid.pos_r*dend.pos_t;
			rec[i] = mat3_t::Cross(re[i]);
			
			fe[i] = dend.force_t;
			me[i] = dend.force_r;
			
			fsum += fe[i];
			msum += me[i] + re[i] % fe[i];
		}

		if(wb->param.parametrization == Wholebody::Parametrization::Joint){
			J_L_qb.clear();
			J_L_wb.clear();
			J_Ld_qb.clear();
			J_Ld_ub.clear();
			for(int j = 0; j < nlink; j++){
				WholebodyData::Link& dlnk = data.links[j];

				real_t mj = wb->links[j].mass;
				mat3_t mj_ajc = mj*mat3_t::Cross(dlnk.acc_t);
				mat3_t mj_vjc = mj*mat3_t::Cross(dlnk.vel_t);
				mat3_t mj_pjc = mj*mat3_t::Cross(dlnk.pos_t);
				J_L_qb  -= mj_vjc*data.Jfk_vi_v0[j].sub_matrix(TSubMatrixDim<0,3,3,3>());
				J_L_wb  += mj_pjc*data.Jfk_vi_v0[j].sub_matrix(TSubMatrixDim<0,3,3,3>());
				J_Ld_qb -= mj_ajc*data.Jfk_vi_v0[j].sub_matrix(TSubMatrixDim<0,3,3,3>());
				J_Ld_ub += mj_pjc*data.Jfk_vi_v0[j].sub_matrix(TSubMatrixDim<0,3,3,3>());
			}
	
			J_L_q   .clear();
			J_L_qd  .clear();
			J_Ld_q  .clear();
			J_Ld_qdd.clear();
			for(int i = 0; i < njoint; i++){
				for(int j = 0; j < nlink; j++){
					WholebodyData::Link& dlnk = data.links[j];

					real_t mj = wb->links[j].mass;
					mat3_t mj_ajc = mj*mat3_t::Cross(dlnk.acc_t);
					mat3_t mj_vjc = mj*mat3_t::Cross(dlnk.vel_t);
					mat3_t mj_pjc = mj*mat3_t::Cross(dlnk.pos_t);
					mat3_t Ij     = dlnk.I;
					vec3_t Jv = data.Jfk_vi_q[j].col(i).sub_vector(TSubVectorDim<0,3>());
					vec3_t Jw = data.Jfk_vi_q[j].col(i).sub_vector(TSubVectorDim<3,3>());
					J_L_q   .col(i) -= mj_vjc*Jv;
					J_L_qd  .col(i) += mj_pjc*Jv + Ij*Jw;
					J_Ld_q  .col(i) -= mj_ajc*Jv;
					J_Ld_qdd.col(i) += mj_pjc*Jv + Ij*Jw;
				}
			}
		}
		else{
			J_L_qb.clear();
			J_L_wb.clear();
			J_Ld_qb.clear();
			J_Ld_ub.clear();
			for(int j = 0; j < nlink; j++){
				WholebodyData::Link& dlnk = data.links[j];

				real_t mj = wb->links[j].mass;
				mat3_t mj_ajc = mj*mat3_t::Cross(dlnk.acc_t);
				mat3_t mj_vjc = mj*mat3_t::Cross(dlnk.vel_t);
				mat3_t mj_pjc = mj*mat3_t::Cross(dlnk.pos_t);
				J_L_qb  -= mj_vjc*data.Jfkik_vi_v0[j].sub_matrix(TSubMatrixDim<0,3,3,3>());
				J_L_wb  += mj_pjc*data.Jfkik_vi_v0[j].sub_matrix(TSubMatrixDim<0,3,3,3>());
				J_Ld_qb -= mj_ajc*data.Jfkik_vi_v0[j].sub_matrix(TSubMatrixDim<0,3,3,3>());
				J_Ld_ub += mj_pjc*data.Jfkik_vi_v0[j].sub_matrix(TSubMatrixDim<0,3,3,3>());
			}
	
			for(int i = 0; i < nend; i++){
				J_L_pe[i].clear();
				J_L_qe[i].clear();
				J_L_ve[i].clear();
				J_L_we[i].clear();
				J_Ld_pe[i].clear();
				J_Ld_qe[i].clear();
				J_Ld_ae[i].clear();
				J_Ld_ue[i].clear();
				for(int j = 0; j < nlink; j++){
					WholebodyData::Link& dlnk = data.links[j];

					real_t mj = wb->links[j].mass;
					mat3_t mj_ajc = mj*mat3_t::Cross(dlnk.acc_t);
					mat3_t mj_vjc = mj*mat3_t::Cross(dlnk.vel_t);
					mat3_t mj_pjc = mj*mat3_t::Cross(dlnk.pos_t);
					mat3_t Ij     = dlnk.I;
					J_L_pe [i] -= mj_vjc*data.Jfkik_vi_ve[j][i].sub_matrix(TSubMatrixDim<0,0,3,3>());
					J_L_qe [i] -= mj_vjc*data.Jfkik_vi_ve[j][i].sub_matrix(TSubMatrixDim<0,3,3,3>());
					J_L_ve [i] += mj_pjc*data.Jfkik_vi_ve[j][i].sub_matrix(TSubMatrixDim<0,0,3,3>()) + Ij*data.Jfkik_vi_ve[j][i].sub_matrix(TSubMatrixDim<3,0,3,3>());
					J_L_we [i] += mj_pjc*data.Jfkik_vi_ve[j][i].sub_matrix(TSubMatrixDim<0,3,3,3>()) + Ij*data.Jfkik_vi_ve[j][i].sub_matrix(TSubMatrixDim<3,3,3,3>());
					J_Ld_pe[i] -= mj_ajc*data.Jfkik_vi_ve[j][i].sub_matrix(TSubMatrixDim<0,0,3,3>());
					J_Ld_qe[i] -= mj_ajc*data.Jfkik_vi_ve[j][i].sub_matrix(TSubMatrixDim<0,3,3,3>());
					J_Ld_ae[i] += mj_pjc*data.Jfkik_vi_ve[j][i].sub_matrix(TSubMatrixDim<0,0,3,3>()) + Ij*data.Jfkik_vi_ve[j][i].sub_matrix(TSubMatrixDim<3,0,3,3>());
					J_Ld_ue[i] += mj_pjc*data.Jfkik_vi_ve[j][i].sub_matrix(TSubMatrixDim<0,3,3,3>()) + Ij*data.Jfkik_vi_ve[j][i].sub_matrix(TSubMatrixDim<3,3,3,3>());
				}

				/*
				fe[i] = dend.force_t;
				me[i] = dend.force_r;
			
				fsum +=  Re[i]*fe[i];
				msum += (Re[i]*me[i] + re[i] % (Re[i]*fe[i]));

				R0*Re[i]*fe[i]
				R0*(Re[i]*me[i] + re[i] % (Re[i]*fe[i]))

				Omega0 % (R0*Re[i]*fe[i])
				 + R0*(Omega[i] % (Re[i]*fe[i])) + (R0*Re[i])*dfe[i]
				 =
				-(R0*Re[i]*fe[i])^x * Omega0
				-R0*(Re[i]*fe[i])^x * Omegai
				+R0*Re[i]           * dfe[i]
	
				Omega0 % (R0*(Re[i]*me[i] + re[i] % (Re[i]*fe[i]))
				 + R0*(Omega[i] % (Re[i]*me[i]) + dre[i] % (Re[i]*fe[i]) + re[i] % (Omega[i] % (Re[i]*fe[i])) + Re[i]*dme[i] + re[i] % (Re[i]*dfe[i]))
				 =
				-(R0*(Re[i]*me[i] + re[i] % (Re[i]*fe[i]))^x    * Omega0
				-R0*((Re[i]*me[i])^x + re[i]^x (Re[i]*fe[i])^x) * Omegai
				-R0*(Re[i]*fe[i])^x * dre[i]
				+R0*(re[i]^x Re[i]) * dfe[i]
				+R0*Re[i]           * dme[i]
				*/
			}
		}
	}

}

void WholebodyKey::PrepareStep(){
}

void WholebodyKey::Finish(){
}

void WholebodyKey::Draw(Render::Canvas* canvas, Render::Config* conf) {
}

//-------------------------------------------------------------------------------------------------
// Wholebody

Wholebody::Param::Param() {
	gravity = 9.8;
	analyticalJacobian = false;
	useLd = true;
	comIkNumIter = 10;
	comIkRatio   = 1.0;
	parametrization = Wholebody::Parametrization::End;
}

//-------------------------------------------------------------------------------------------------

Wholebody::Link::Link(real_t _mass, vec3_t _inertia, int _iend, int _iparent, int _ijoint, vec3_t _trn, vec3_t _axis){
	mass    = _mass;
	iend    = _iend;
	iparent = _iparent;
	ijoint  = _ijoint;
	trn     = _trn;
	axis    = _axis;

	inertia.clear();
	inertia[0][0] = _inertia[0];
	inertia[1][1] = _inertia[1];
	inertia[2][2] = _inertia[2];
}

//-------------------------------------------------------------------------------------------------

Wholebody::End::End(int _ilink, vec3_t _offset){
	ilink  = _ilink;
	offset = _offset;
}

//-------------------------------------------------------------------------------------------------

Wholebody::Limit::Limit(int _ichain, int _type, real_t _scale){
	ichain = _ichain;
	type   = _type;
	scale  = _scale;
}

//-------------------------------------------------------------------------------------------------

Wholebody::Chain::Chain(const vector<int>& _ilink, const vector<int>& _ilimit){
	ilink  = _ilink;
	ilimit = _ilimit;
}

//-------------------------------------------------------------------------------------------------

Wholebody::Snapshot::Snapshot() {

}

//-------------------------------------------------------------------------------------------------

Wholebody::Wholebody(Graph* g, string n) :TrajectoryNode(g, n) {
	type = Type::Object;
	graph->wholebodies.Add(this);
}

Wholebody::~Wholebody() {
	graph->wholebodies.Remove(this);
}

void Wholebody::SetScaling(){
	// calc scaling constants
	param.totalMass = 0.0;
	for(int i = 0; i < links.size(); i++){
		param.totalMass += links[i].mass;
	}
	for(int i = 0; i < links.size(); i++){
		links[i].mass_ratio = links[i].mass/param.totalMass;
	}

	sl   = 1.0;  //< unit length
	st   = graph->ticks[1]->time - graph->ticks[0]->time;
	sat  = param.gravity;
	svt  = sat*st;
	sft  = param.totalMass*param.gravity;
	spt  = svt*st;
	spr  = spt/sl;
	svr  = svt/sl;
	sar  = sat/sl;
	sfr  = sft*sl;
	sL   = sfr*st;
}

void Wholebody::Reset(){
	int nend   = (int)ends  .size();
	int njoint = (int)joints.size();
	int N = (int)graph->ticks.size()-1;

	for (int k = 0; k <= N; k++) {
		WholebodyKey* key = (WholebodyKey*)traj.GetKeypoint(graph->ticks[k]);
		WholebodyData& d = key->data_des;

		key->centroid.var_pos_t->val = d.centroid.pos_t;
		key->centroid.var_vel_t->val = d.centroid.vel_t;
		key->centroid.var_pos_r->val = d.centroid.pos_r;
		key->centroid.var_vel_r->val = d.centroid.vel_r;

		key->base.var_pos_r->val = d.base.pos_r;
		key->base.var_vel_r->val = d.base.vel_r;

		if(key->next){
			key->base.var_acc_r->val = d.base.acc_r;
		}

		for(int i = 0; i < njoint; i++){
			WholebodyKey::Joint& jnt = key->joints[i];

			if(param.parametrization == Parametrization::Joint){
				jnt.var_q ->val = d.q [i];
				jnt.var_qd->val = d.qd[i];

				if(key->next){
					jnt.var_qdd->val = d.qdd[i];
				}
			}
		}

		for(int i = 0; i < nend; i++){
			WholebodyKey ::End&  end  = key->ends[i];
			WholebodyData::End&  dend = d.ends [i];
			
			if(param.parametrization == Parametrization::End){
				end.var_pos_t->val = dend.pos_t;
				end.var_pos_r->val = dend.pos_r;
				end.var_vel_t->val = dend.vel_t;
				end.var_vel_r->val = dend.vel_r;
				
				if(key->next){
					end.var_acc_t->val = dend.acc_t;
					end.var_acc_r->val = dend.acc_r;
				}
			}
			
			if(key->next){
				end.var_force_t->val = dend.force_t;
				end.var_force_r->val = dend.force_r;
			}
		}
	}

	trajReady = false;
}

void Wholebody::Shift(){
	int nend   = (int)ends  .size();
	int njoint = (int)joints.size();
	int N = (int)graph->ticks.size()-1;

	for (int k = 0; k < N; k++) {
		WholebodyKey* key0 = (WholebodyKey*)traj.GetKeypoint(graph->ticks[k+0]);
		WholebodyKey* key1 = (WholebodyKey*)traj.GetKeypoint(graph->ticks[k+1]);
		
		key0->centroid.var_pos_t->val = key1->centroid.var_pos_t->val;
		key0->centroid.var_pos_r->val = key1->centroid.var_pos_r->val;
		key0->centroid.var_vel_t->val = key1->centroid.var_vel_t->val;
		key0->centroid.var_vel_r->val = key1->centroid.var_vel_r->val;
		key0->base.var_pos_r->val     = key1->base.var_pos_r->val;
		key0->base.var_vel_r->val     = key1->base.var_vel_r->val;

		if(key1->next)
			key0->base.var_acc_r->val = key1->base.var_acc_r->val;

		for(int i = 0; i < njoint; i++){
			WholebodyKey::Joint& jnt0 = key0->joints[i];
			WholebodyKey::Joint& jnt1 = key1->joints[i];

			if(param.parametrization == Parametrization::Joint){
				jnt0.var_q ->val = jnt1.var_q ->val;
				jnt0.var_qd->val = jnt1.var_qd->val;
				
				if(key1->next){
					jnt0.var_qdd->val = jnt1.var_qdd->val;
				}
			}
		}
		
		for(int i = 0; i < nend; i++){
			WholebodyKey ::End&  end0 = key0->ends[i];
			WholebodyKey ::End&  end1 = key1->ends[i];
			
			if(param.parametrization == Parametrization::End){
				end0.var_pos_t->val = end1.var_pos_t->val;
				end0.var_pos_r->val = end1.var_pos_r->val;
				end0.var_vel_t->val = end1.var_vel_t->val;
				end0.var_vel_r->val = end1.var_vel_r->val;
			
				if(key1->next){
					end0.var_acc_t->val = end1.var_acc_t->val;
					end0.var_acc_r->val = end1.var_acc_r->val;
				}
			}

			if(key1->next){
				end0.var_force_t->val = end1.var_force_t->val;
				end0.var_force_r->val = end1.var_force_r->val;
			}
		}
	}
}

void Wholebody::Setup(){
	int nend   = (int)ends  .size();
	int njoint = (int)joints.size();
	int N = (int)graph->ticks.size()-1;

	for (int k = 0; k <= N; k++) {
		WholebodyKey* key = (WholebodyKey*)traj.GetKeypoint(graph->ticks[k]);
		WholebodyData& d = key->data_des;

		real_t t = graph->ticks[k]->time;

		// need to get contact state as desired state
		callback->GetDesiredState(k, t, d);

		if(k == 0)
			callback->GetInitialState(d);

		if(k == 0){	
			key->centroid.var_pos_t->val = d.centroid.pos_t;
			key->centroid.var_vel_t->val = d.centroid.vel_t;
			key->centroid.var_pos_r->val = d.centroid.pos_r;
			key->centroid.var_vel_r->val = d.centroid.vel_r;
			key->centroid.var_pos_t->locked = true;
			key->centroid.var_vel_t->locked = true;
			key->centroid.var_pos_r->locked = true;
			key->centroid.var_vel_r->locked = true;
		
			key->base.var_pos_r->val = d.base.pos_r;
			key->base.var_vel_r->val = d.base.vel_r;
			key->base.var_pos_r->locked = true;
			key->base.var_vel_r->locked = true;
		}
		if(k > 0){
			key->centroid.con_des_pos_t->desired = d.centroid.pos_t;
			key->centroid.con_des_vel_t->desired = d.centroid.vel_t;
			key->centroid.con_des_pos_r->desired = d.centroid.pos_r;
			key->centroid.con_des_vel_r->desired = d.centroid.vel_r;
			key->centroid.con_des_pos_t->weight  = d.centroid.pos_t_weight;
			key->centroid.con_des_vel_t->weight  = d.centroid.vel_t_weight;
			key->centroid.con_des_pos_r->weight  = d.centroid.pos_r_weight;
			key->centroid.con_des_vel_r->weight  = d.centroid.vel_r_weight;

			key->base.con_des_pos_r->desired = d.base.pos_r;
			key->base.con_des_vel_r->desired = d.base.vel_r;
			key->base.con_des_pos_r->weight  = d.base.pos_r_weight;
			key->base.con_des_vel_r->weight  = d.base.vel_r_weight;
		}

		if(k == 0){
			key->base.var_acc_r->val = d.base.acc_r;
			key->base.var_acc_r->locked = true;
		}
		else if(key->next){
			key->base.con_des_acc_r->desired = d.base.acc_r;
			key->base.con_des_acc_r->weight  = d.base.acc_r_weight;
		}	

		if(key->next && key->prev){
			//key->centroid.con_Ld->weight = d.centroid.Ld_weight;
			key->centroid.con_L->weight = d.centroid.L_weight;
		}

		if(param.parametrization == Parametrization::Joint){
			for(int i = 0; i < njoint; i++){
				WholebodyKey::Joint& jnt = key->joints[i];
			
				if(k == 0){
					jnt.var_q ->val = d.q [i];
					jnt.var_qd->val = d.qd[i];
					jnt.var_q ->locked = true;
					jnt.var_qd->locked = true;

					jnt.var_qdd->val = d.qdd[i];
					jnt.var_qdd->locked = true;
				}
				else{
					jnt.con_des_q ->desired = d.q [i];
					jnt.con_des_qd->desired = d.qd[i];
					jnt.con_des_q ->weight[0] = d.q_weight [i];
					jnt.con_des_qd->weight[0] = d.qd_weight[i];

					jnt.con_range_q->_min = d.q_min[i];
					jnt.con_range_q->_max = d.q_max[i];

					if(key->next){
						jnt.con_des_qdd->desired = d.qdd[i];
						jnt.con_des_qdd->weight[0] = d.qdd_weight[i];
					}
				}
			}
		}

		for(int i = 0; i < nend; i++){
			WholebodyKey ::End&  end  = key->ends[i];
			WholebodyData::End&  dend = d.ends [i];

			key->data.ends[i].state = d.ends[i].state;

			if(k == 0){
				if(param.parametrization == Parametrization::End){
					end.var_pos_t->val = dend.pos_t;
					end.var_pos_r->val = dend.pos_r;
					end.var_vel_t->val = dend.vel_t;
					end.var_vel_r->val = dend.vel_r;
					end.var_pos_t->locked = true;
					end.var_pos_r->locked = true;
					end.var_vel_t->locked = true;
					end.var_vel_r->locked = true;
				}
			}
			else{
				// transform to absolute coordinate
				end.con_des_pos_t->desired = d.centroid.pos_t + d.centroid.pos_r*dend.pos_t;
				end.con_des_pos_r->desired = d.centroid.pos_r*dend.pos_r;
				end.con_des_vel_t->desired = d.centroid.vel_t + d.centroid.vel_r%(d.centroid.pos_r*dend.pos_t) + d.centroid.pos_r*dend.vel_t;
				end.con_des_vel_r->desired = d.centroid.vel_r + d.centroid.pos_r*dend.vel_r;
				end.con_des_pos_t->weight  = dend.pos_t_weight;
				end.con_des_pos_r->weight  = dend.pos_r_weight;
				end.con_des_vel_t->weight  = dend.vel_t_weight;
				end.con_des_vel_r->weight  = dend.vel_r_weight;
				
				if(dend.state == ContactState::Free){
					end.con_contact_pos_t->weight = vec3_t(0.0, 0.0, 0.0);
					end.con_contact_pos_r->weight = vec3_t(0.0, 0.0, 0.0);
					end.con_contact_vel_t->weight = vec3_t(0.0, 0.0, 0.0);
					end.con_contact_vel_r->weight = vec3_t(0.0, 0.0, 0.0);
				}
				if(dend.state == ContactState::Surface){
					end.con_contact_pos_t->weight = vec3_t(0.0, 0.0, 1.0);
					end.con_contact_pos_r->weight = vec3_t(1.0, 1.0, 0.0);
					end.con_contact_vel_t->weight = vec3_t(0.0, 0.0, 1.0);
					end.con_contact_vel_r->weight = vec3_t(1.0, 1.0, 1.0);
				}
				if(dend.state == ContactState::Line){
					end.con_contact_pos_t->weight = vec3_t(0.0, 0.0, 1.0);
					end.con_contact_pos_r->weight = vec3_t(0.0, 0.0, 0.0);
					end.con_contact_vel_t->weight = vec3_t(0.0, 0.0, 1.0);
					end.con_contact_vel_r->weight = vec3_t(0.0, 0.0, 1.0);
				}
				if(dend.state == ContactState::Point){
					end.con_contact_pos_t->weight = vec3_t(0.0, 0.0, 1.0);
					end.con_contact_pos_r->weight = vec3_t(0.0, 0.0, 0.0);
					end.con_contact_vel_t->weight = vec3_t(1.0, 1.0, 1.0);
					end.con_contact_vel_r->weight = vec3_t(0.0, 0.0, 0.0);
				}
			}

			if(k == 0){
				if(param.parametrization == Parametrization::End){
					end.var_acc_t->val = dend.acc_t;
					end.var_acc_r->val = dend.acc_r;
					end.var_acc_t->locked = true;
					end.var_acc_r->locked = true;
				}

				end.var_force_t->val = dend.force_t;
				end.var_force_r->val = dend.force_r;				
				end.var_force_t->locked = true;
				end.var_force_r->locked = true;
			}
			else if(key->next){
				if(param.parametrization == Parametrization::End){
					end.var_acc_t->val = dend.acc_t;
					end.var_acc_r->val = dend.acc_r;
					end.con_des_acc_t->desired = dend.acc_t;
					end.con_des_acc_r->desired = dend.acc_r;
					end.con_des_acc_t->weight = dend.acc_t_weight;
					end.con_des_acc_r->weight = dend.acc_r_weight;
				}

				if(dend.state == ContactState::Free){
					end.var_force_t->val.clear();
					end.var_force_r->val.clear();
					end.var_force_t->locked = true;
					end.var_force_r->locked = true;
					end.con_des_force_t->active = false;
					end.con_des_force_r->active = false;
				}
				else{
					end.var_force_t->val = dend.force_t;
					end.var_force_r->val = dend.force_r;
					end.var_force_t->locked = false;
					end.var_force_r->locked = false;
					end.con_des_force_t->active = true;
					end.con_des_force_r->active = true;
					end.con_des_force_t->desired = dend.force_t;
					end.con_des_force_r->desired = dend.force_r;
					end.con_des_force_t->weight = dend.force_t_weight;
					end.con_des_force_r->weight = dend.force_r_weight;
				}

				end.con_force_normal        ->active = (dend.state != ContactState::Free);
				end.con_force_friction[0][0]->active = (dend.state != ContactState::Free);
				end.con_force_friction[0][1]->active = (dend.state != ContactState::Free);
				end.con_force_friction[1][0]->active = (dend.state != ContactState::Free);
				end.con_force_friction[1][1]->active = (dend.state != ContactState::Free);
				end.con_moment[0][0]        ->active = (dend.state != ContactState::Free);
				end.con_moment[0][1]        ->active = (dend.state != ContactState::Free);
				end.con_moment[1][0]        ->active = (dend.state != ContactState::Free);
				end.con_moment[1][1]        ->active = (dend.state != ContactState::Free);

				end.con_force_normal        ->weight[0] = 1.0;
				end.con_force_friction[0][0]->weight[0] = 1.0;
				end.con_force_friction[0][1]->weight[0] = 1.0;
				end.con_force_friction[1][0]->weight[0] = 1.0;
				end.con_force_friction[1][1]->weight[0] = 1.0;
				end.con_moment[0][0]        ->weight[0] = 1.0;//0.01;
				end.con_moment[0][1]        ->weight[0] = 1.0;//0.01;
				end.con_moment[1][0]        ->weight[0] = 1.0;//0.01;
				end.con_moment[1][1]        ->weight[0] = 1.0;//0.01;
			}
		}
	}

	trajReady = false;
}

void Wholebody::Init() {
	TrajectoryNode::Init();

	int nlink  = (int)links .size();
	int nend   = (int)ends  .size();
	int nchain = (int)chains.size();
	
	for(int i = 0; i < nlink; i++){
		if(links[i].iparent != -1)
			links[links[i].iparent].ichildren.push_back(i);
	}
	
	// call prepare here so that initial trajectory is visualized properly
	Setup  ();
	Reset  ();
    Prepare();

    trajReady = false;
}

void Wholebody::Prepare() {
	trajReady = false;

	traj.Update();
	
	#pragma omp parallel for
	for(int k = 0; k < traj.size(); k++){
		traj[k]->Prepare();
	}
	//TrajectoryNode::Prepare();
}

void Wholebody::PrepareStep(){
	#pragma omp parallel for
	for(int k = 0; k < traj.size(); k++){
		traj[k]->PrepareStep();
	}
	//TrajectoryNode::PrepareStep();
}

void Wholebody::Finish(){
	TrajectoryNode::Finish();
}

void Wholebody::CalcIK(WholebodyData& d, int ichain, bool calc_jacobian){
	vec3_t pe_local;
	quat_t qe_local;
	vvec_t joint;
	vvec_t error;

	bool jacobian_by_diff = false;

	Chain& ch = chains[ichain];

	timer.CountUS();
    
	int ifront = ch.ilink.front();
	int iroot  = links[ifront].iparent;
		
	vec3_t pr = d.links[iroot].pos_t;
	quat_t qr = d.links[iroot].pos_r;
	vec3_t pe = d.ends [ichain].pos_t;
	quat_t qe = d.ends [ichain].pos_r;
	vec3_t tr = links[ifront].trn;
	vec3_t oe = ends [ichain].offset;

	//  pr + qr*(tr + pe_local) + qe*oe = pe
	//  qr*qe_local = qe
	pe_local = qr.Conjugated()*(pe - pr - qe*oe) - tr;
	qe_local = qr.Conjugated()*qe;

	callback->CalcIK(ichain, pe_local, qe_local, joint, error, d.Jq[ichain], d.Je[ichain], calc_jacobian/* && !jacobian_by_diff*/);
    	
	for(int j = 0; j < ch.ilink.size(); j++){
		int i = ch.ilink[j];
		d.q[links[i].ijoint] = joint[j];
	}

	for(int j = 0; j < ch.ilimit.size(); j++){
		d.e[ch.ilimit[j]] = error[j];
	}

	if(jacobian_by_diff){
		// calc IK jacobian by finite difference
		const real_t eps    = 1.0e-2;
		const real_t epsinv = 1.0/eps;
		vec3_t pe_local_tmp;
		quat_t qe_local_tmp;
		vvec_t joint_tmp;
		vvec_t error_tmp;
		vmat_t Jq_tmp;
		vmat_t Je_tmp;
		Jq_tmp.resize(joint.size(), 6);
		Je_tmp.resize(error.size(), 6);
		for(int j = 0; j < 6; j++){
			pe_local_tmp = pe_local;
			qe_local_tmp = qe_local;
			if(j < 3){
				pe_local_tmp[j] += eps;
			}
			else{
				qe_local_tmp = quat_t::Rot(eps, 'x' + (j%3))*qe_local_tmp;
				qe_local_tmp.unitize();
			}
			callback->CalcIK(ichain, pe_local_tmp, qe_local_tmp, joint_tmp, error_tmp, d.Jq[ichain], d.Je[ichain], false);
			Jq_tmp.col(j) = (joint_tmp - joint)*epsinv;
			Je_tmp.col(j) = (error_tmp - error)*epsinv;
		}

		// calc error between analytical jacobian
		//real_t diff = 0.0;
		//for(int r = 0; r < joint.size(); r++)for(int c = 0; c < 6; c++){
		//	diff += std::abs(Jq_tmp[r][c] - d.Jq[ichain][r][c]);
		//}
		//DSTR << "ichain: " << ichain << "  d: " << diff << endl;
	}
}

void Wholebody::CalcFK(WholebodyData& d, int ichain, bool calc_end){
	Chain& ch = chains[ichain];

	// calc fk
	for(int i : ch.ilink){
		int ip = links[i].iparent;
		WholebodyData::Link& dlnk  = d.links[i ];
		WholebodyData::Link& dlnkp = d.links[ip];

		dlnk.pos_t = dlnkp.pos_t + dlnkp.pos_r*links[i].trn;
		dlnk.pos_r = dlnkp.pos_r*quat_t::Rot(d.q[links[i].ijoint], links[i].axis);
		dlnk.pos_r.unitize();
	}

	// calc end pose
	if(calc_end){
		WholebodyData::Link& dlnk = d.links[ch.ilink.back()];
		WholebodyData::End&  dend = d.ends[ichain];
		dend.pos_t = dlnk.pos_t + dlnk.pos_r*ends[ichain].offset;
		dend.pos_r = dlnk.pos_r;
	}
}

void Wholebody::CalcPosition(WholebodyData& d, bool fk_or_ik){
	timer2.CountUS();
	int nlink  = (int)links.size();
	int nend   = (int)ends.size();
	int nchain = (int)chains.size();

	// initialize base link pose (com local)
	d.links[0].pos_t = vec3_t();
	d.links[0].pos_r = d.base.pos_r;

	vec3_t pc;

	// one iteration is enough in the case of FK
	int niter = (fk_or_ik ? 1 : param.comIkNumIter);
	for(int n = 0; n <= niter; n++){
		if(fk_or_ik){
			for(int ic = 0; ic < nchain; ic++)
				CalcFK(d, ic, true);
		}
		else{
			for(int ic = 0; ic < nchain; ic++){
				CalcIK(d, ic, false);
				CalcFK(d, ic, false);
			}
		}
		
		if(n == param.comIkNumIter)
			break;

		pc.clear();
		for(int i = 0; i < nlink; i++){
			pc += links[i].mass_ratio*d.links[i].pos_t;
		}
		
		d.links[0].pos_t -= param.comIkRatio*pc;
	}
	int T = timer2.CountUS();
}

void Wholebody::CalcComAcceleration (WholebodyData& d){
	vec3_t fsum;
	
	int nend = (int)ends.size();
    for(int i = 0; i < nend; i++){
        WholebodyData::End& dend = d.ends[i];
        fsum += dend.force_t;
    }
    d.centroid.acc_t = (1.0/param.totalMass)*fsum - vec3_t(0.0, 0.0, param.gravity);
}

void Wholebody::CalcBaseAcceleration(WholebodyData& d){
	vec3_t msum;
	
	int nend = (int)ends.size();
    for(int i = 0; i < nend; i++){
        WholebodyData::End& dend = d.ends[i];
        msum += dend.force_r + (d.centroid.pos_r*dend.pos_t) % dend.force_t;
    }
    d.centroid.acc_r = d.centroid.Iinv*(msum - d.centroid.pos_r*d.centroid.Ld);
}

void Wholebody::CalcJacobian(WholebodyData& d, vector<WholebodyData>& dtmp){
	int nlink  = (int)links .size();
	int nend   = (int)ends  .size();
	
	// clear
	d.Jik_e_v0.clear();
	for(int i = 0; i < nend; i++){
		d.Jik_e_ve[i].clear();
	}
	d.Jik_q_v0.clear();
	for(int i = 0; i < nend; i++){
		d.Jik_q_ve[i].clear();
	}
	for(int i = 0; i < nlink; i++){
		d.Jfkik_vi_v0[i].clear();
		for(int j = 0; j < nend; j++){
			d.Jfkik_vi_ve[i][j].clear();
		}
	}

	if(param.analyticalJacobian){
		CalcJacobianAnalytical(d);
	}
	else{
		CalcJacobianNumerical(d, dtmp);
	}
	TransformJacobian(d);

	//SaveJacobian(d);
}

void Wholebody::SaveJacobian(WholebodyData& d){
	int nlink  = (int)links .size();
	int nend   = (int)ends  .size();
	int njoint = (int)joints.size();
	FILE* file = fopen("jacobian.csv", "w");
	int nrow = nlink*6;
	int ncol = (1+nend)*6;
	real_t val;
	for(int r = 0; r < nrow; r++){
		int ilink = r/6;
		int jrow  = r%6;

		for(int c = 0; c < ncol; c++){
			int iend = c/6;
			int jcol = c%6;
			if(iend == 0){
				val = d.Jfkik_vi_v0[ilink][jrow][jcol];
			}
			else{
				val = d.Jfkik_vi_ve[ilink][iend-1][jrow][jcol];
			}
			fprintf(file, "%f, ", val);
		}
		fprintf(file, "\n");
	}
	fclose(file);

	file = fopen("jq.csv", "w");
	nrow = njoint;
	ncol = (1+nend)*6;
	for(int r = 0; r < nrow; r++){
		int jrow = r;
		for(int c = 0; c < ncol; c++){
			int iend = c/6;
			int jcol = c%6;
			if(iend == 0){
				val = d.Jik_q_v0[jrow][jcol];
			}
			else{
				val = d.Jik_q_ve[iend-1][jrow][jcol];
			}
			fprintf(file, "%f, ", val);
		}
		fprintf(file, "\n");
	}
	fclose(file);
}

void Wholebody::CalcJacobianAnalytical(WholebodyData& d){
	int nchain = (int)chains.size();
	int nlink  = (int)links .size();
	int njoint = (int)joints.size();
	int nlimit = (int)limits.size();

	if(param.parametrization == Parametrization::End){
		for(int ic = 0; ic < nchain; ic++){
			CalcIK(d, ic, true);
		}
	}
	for(int ic = 0; ic < nchain; ic++){
		CalcFK(d, ic, false);
	}

	// IK jacobian
	if(param.parametrization == Parametrization::End){
		vmat_t Jr, Je;
		for(int ic = 0; ic < nchain; ic++){
			Chain& ch = chains[ic];

			int ir = links[ch.ilink[0]].iparent;
			int ie = ends[ic].ilink;
			vec3_t pr = d.links[ir].pos_t;
			quat_t qr = d.links[ir].pos_r;
			mat3_t Rr;
			qr.ToMatrix(Rr);

			vec3_t pe  = d.links[ie].pos_t;
			quat_t qe  = d.links[ie].pos_r;
			quat_t qer = qr.Conjugated()*qe;
			vec3_t oe  = ends[ic].offset;

			// rotation to root local frame
			// spatial transform to consider end offset
			mat6_t Xr;
			Xr.sub_matrix(TSubMatrixDim<0,0,3,3>()) = Rr.trans();
			Xr.sub_matrix(TSubMatrixDim<0,3,3,3>()) = mat3_t::Cross(qer*oe)*Rr.trans();
			Xr.sub_matrix(TSubMatrixDim<3,0,3,3>()).clear();
			Xr.sub_matrix(TSubMatrixDim<3,3,3,3>()) = Rr.trans();

			mat6_t Xer;
			Xer.sub_matrix(TSubMatrixDim<0,0,3,3>()) = mat3_t();
			Xer.sub_matrix(TSubMatrixDim<0,3,3,3>()) = -mat3_t::Cross(pe - pr);
			Xer.sub_matrix(TSubMatrixDim<3,0,3,3>()).clear();
			Xer.sub_matrix(TSubMatrixDim<3,3,3,3>()) = mat3_t();

			d.Jq[ic] = d.Jq[ic]*Xr;
			d.Je[ic] = d.Je[ic]*Xr;

			int nq = (int)ch.ilink.size();
			Jr.resize(nq, 6);
			Je.resize(nq, 6);
			Je =  d.Jq[ic];
			Jr = -d.Jq[ic]*Xer;

			for(int j = 0; j < (int)ch.ilink.size(); j++){
				int iq = links[ch.ilink[j]].ijoint;

				if(ir == 0)
					 d.Jik_q_v0.row(iq)                 = Jr.row(j);
				else d.Jik_q_ve[links[ir].iend].row(iq) = Jr.row(j);
				d.Jik_q_ve[ic].row(iq) = Je.row(j);
			}

			int ne = (int)ch.ilimit.size();
			Jr.resize(ne, 6);
			Je.resize(ne, 6);
			Je =  d.Je[ic];
			Jr = -d.Je[ic]*Xer;

			for(int j = 0; j < (int)ch.ilimit.size(); j++){
				int ie = ch.ilimit[j];

				if(ir == 0)
					 d.Jik_e_v0.row(ie)                 = Jr.row(j);
				else d.Jik_e_ve[links[ir].iend].row(ie) = Jr.row(j);
				d.Jik_e_ve[ic].row(ie) = Je.row(j);
			}
		}
	}

	// FK jacobian
	for(int ic = 0; ic < nchain; ic++){
		Chain& ch = chains[ic];
		int ir = links[ch.ilink[0]].iparent;
		int ie = ends[ic].ilink;
		int nq = (int)ch.ilink .size();

		vec3_t pr = d.links[ir].pos_t;

		for(int j = 0; j < ch.ilink.size(); j++){
			int i = ch.ilink[j];

			vec3_t pi = d.links[i].pos_t;
			quat_t qi = d.links[i].pos_r;

			mat6_t Xir;
			Xir.sub_matrix(TSubMatrixDim<0,0,3,3>()) = mat3_t();
			Xir.sub_matrix(TSubMatrixDim<0,3,3,3>()) = -mat3_t::Cross(pi - pr);
			Xir.sub_matrix(TSubMatrixDim<3,0,3,3>()).clear();
			Xir.sub_matrix(TSubMatrixDim<3,3,3,3>()) = mat3_t();

			d.Jfk_vi_v0[i].clear();
			d.Jfk_vi_q [i].clear();
			if(ir == 0){
				d.Jfk_vi_v0[i] = Xir;
			}
			else{
				d.Jfk_vi_v0[i] = Xir*d.Jfk_vi_v0[ir];
				d.Jfk_vi_q [i] = Xir*d.Jfk_vi_q [ir];
			}

			for(int j0 = 0; j0 <= j; j0++){
				int i0 = ch.ilink[j0];
				int iq = links[i0].ijoint;

				vec3_t pi0 = d.links[i0].pos_t;
				quat_t qi0 = d.links[i0].pos_r;

				vec3_t eta = qi0*links[i0].axis;
				vec3_t r   = pi - pi0;
				
				d.Jfk_vi_q[i].col(iq).v_range(0,3) = eta % r;
				d.Jfk_vi_q[i].col(iq).v_range(3,3) = eta;
			}
		}
	}

	/*
	 qd = J_IK,0*V0 + J_IK,e*Ve
	 Vi = J_FK,0*V0 + J_FK,q*qd
	    = J_FK,0*V0 + J_FK,q*(J_IK,0*V0 + J_IK,e*Ve)
		= (J_FK,0 + J_FK,q*J_IK,0)*V0 + (J_FK,q*J_IK,e)*Ve
	*/
	if(param.parametrization == Parametrization::End){
		for(int j = 0; j < nlink; j++){
			d.Jfkik_vi_v0[j] = d.Jfk_vi_v0[j] + d.Jfk_vi_q[j]*d.Jik_q_v0;

			for(int i = 0; i < nchain; i++)
				d.Jfkik_vi_ve[j][i] = d.Jfk_vi_q[j]*d.Jik_q_ve[i];
		}
	}

	/*
	// jacobian from base link to base link is identity
	d.J_vi_v0[0].sub_matrix(TSubMatrixDim<0,0,3,3>()) = mat3_t();
	d.J_vi_v0[0].sub_matrix(TSubMatrixDim<3,3,3,3>()) = mat3_t();

	//
	for(int ic = 0; ic < nchain; ic++){
		Chain& ch = chains[ic];
		int ir = links[ch.ilink[0]].iparent;
		int ie = ends[ic].ilink;

		
		// error jacobian			
		for(int j = 0; j < ch.ilimit.size(); j++){
			int ie = ch.ilimit[j];

			// root link is base link
			if(ir == 0){
				d.J_e_v0    .row(ie) = -Xer.trans()*d.Je[ic].row(j);
				d.J_e_ve[ic].row(ie) =  d.Je[ic].row(j);
			}
			// otherwise:
			//  assume that ir belongs to 0th chain
			else{
				d.J_e_v0    .row(ie) = -(d.J_vi_v0[ir]   ).trans()*(Xer.trans()*d.Je[ic].row(j));
				d.J_e_ve[0 ].row(ie) = -(d.J_vi_ve[ir][0]).trans()*(Xer.trans()*d.Je[ic].row(j));
				d.J_e_ve[ic].row(ie) = d.Je[ic].row(j);
			}
		}

		// joint jacobian
		for(int j = 0; j < (int)ch.ilink.size(); j++){
			int iq = links[ch.ilink[j]].ijoint;

			// root link is base link
			if(ir == 0){
				d.J_q_v0    .row(iq) = -Xer.trans()*d.Jq[ic].row(j);
				d.J_q_ve[ic].row(iq) =  d.Jq[ic].row(j);
			}
			// otherwise:
			//  assume that ir belongs to 0th chain
			else{
				d.J_q_v0    .row(iq) = -(d.J_vi_v0[ir]   ).trans()*(Xer.trans()*d.Jq[ic].row(j));
				d.J_q_ve[0 ].row(iq) = -(d.J_vi_ve[ir][0]).trans()*(Xer.trans()*d.Jq[ic].row(j));
				d.J_q_ve[ic].row(iq) = d.Jq[ic].row(j);
			}
		}
	}
	*/
}

void Wholebody::CalcJacobianNumerical(WholebodyData& data, vector<WholebodyData>& data_tmp){
	int nend   = (int)ends .size ();
	int nlink  = (int)links.size ();
	int nchain = (int)chains.size();

	const real_t eps    = 1.0e-2;
	const real_t epsinv = 1.0/eps;

	vec6_t dv;

	// base link velocity
	for(int j = 0; j < 6; j++){
		WholebodyData& d = data_tmp[j];
		d = data;

		if(j < 3){
			d.links[0].pos_t[j] += eps;
		}
		else{
			d.links[0].pos_r = quat_t::Rot(eps, 'x' + (j%3))*d.links[0].pos_r;
			d.links[0].pos_r.unitize();
		}
		
		for(int ic = 0; ic < nchain; ic++){
			CalcIK(d, ic, false);
			CalcFK(d, ic, false);
		}
	
		for(int i = 0; i < nlink; i++){
			dv.sub_vector(TSubVectorDim<0,3>()) = (d.links[i].pos_t - data.links[i].pos_t)*epsinv;

			quat_t qdiff = d.links[i].pos_r*data.links[i].pos_r.Conjugated();
			real_t theta = qdiff.Theta();
			if(theta > _pi)
				theta -= 2*_pi;
			
			dv.sub_vector(TSubVectorDim<3,3>()) = qdiff.Axis()*(theta*epsinv);
			data.Jfkik_vi_v0[i].col(j) = dv;
		}
		data.Jik_e_v0.col(j) = (d.e - data.e)*epsinv;
		data.Jik_q_v0.col(j) = (d.q - data.q)*epsinv;
	}

	// end velocity
	for(int iend = 0; iend < nend; iend++){
		for(int j = 0; j < 6; j++){
			WholebodyData& d = data_tmp[3 + 6*iend + j];
			d = data;

			if(j < 3){
				d.ends[iend].pos_t[j] += eps;
			}
			else{
				d.ends[iend].pos_r = quat_t::Rot(eps, 'x' + (j%3))*d.ends[iend].pos_r;
				d.ends[iend].pos_r.unitize();
			}

			for(int ic = 0; ic < nchain; ic++){
				CalcIK(d, ic, false);
				CalcFK(d, ic, false);
			}

			for(int i = 0; i < nlink; i++){
				dv.sub_vector(TSubVectorDim<0,3>()) = (d.links[i].pos_t - data.links[i].pos_t)*epsinv;

				quat_t qdiff = d.links[i].pos_r*data.links[i].pos_r.Conjugated();
				real_t theta = qdiff.Theta();
				if(theta > _pi)
					theta -= 2*_pi;
				dv.sub_vector(TSubVectorDim<3,3>()) = qdiff.Axis()*(theta*epsinv);
				
				data.Jfkik_vi_ve[i][iend].col(j) = dv;
			}
			data.Jik_e_ve[iend].col(j) = (d.e - data.e)*epsinv;
			data.Jik_q_ve[iend].col(j) = (d.q - data.q)*epsinv;
		}
	}
}

void Wholebody::CalcJacobianNumerical2(WholebodyData& data, vector<WholebodyData>& data_tmp){
	int nend  = (int)ends .size();
	int nlink = (int)links.size();

	const real_t eps    = 1.0e-2;
	const real_t epsinv = 1.0/eps;

	vec6_t dv;

	for(int iend = 0; iend < nend; iend++){
		for(int j = 0; j < 6; j++){
			WholebodyData& d = data_tmp[6*iend + j];
			d = data;

			if(j < 3){
				d.ends[iend].pos_t[j] += eps;
			}
			else{
				d.ends[iend].pos_r = quat_t::Rot(eps, 'x' + (j % 3))*d.ends[iend].pos_r;
				d.ends[iend].pos_r.unitize();
			}

			CalcPosition(d, false);

			for(int i = 0; i < nlink; i++){
				dv.sub_vector(TSubVectorDim<0,3>()) = (d.links[i].pos_t - data.links[i].pos_t)*epsinv;

				quat_t qdiff = d.links[i].pos_r*data.links[i].pos_r.Conjugated();
				real_t theta = qdiff.Theta();
				if(theta > _pi)
					theta -= 2*_pi;
				dv.sub_vector(TSubVectorDim<3,3>()) = qdiff.Axis()*(theta*epsinv);
				
				data.Jfkik_vi_ve[i][iend].col(j) = dv;
			}
			data.Jik_e_ve[iend].col(j) = (d.e - data.e)*epsinv;
			data.Jik_q_ve[iend].col(j) = (d.q - data.q)*epsinv;
		}
	}
						
	int T = timer.CountUS();
	//DSTR << "prepare step: " << T << endl;
}

#define v()      sub_vector(TSubVectorDim<0,3>())
#define w()      sub_vector(TSubVectorDim<3,3>())
#define vv()     sub_matrix(TSubMatrixDim<0,0,3,3>())
#define vw()     sub_matrix(TSubMatrixDim<0,3,3,3>())
#define vrow()   sub_matrix(TSubMatrixDim<0,0,3,6>())
#define vcol()   sub_matrix(TSubMatrixDim<0,0,6,3>())
#define wcol()   sub_matrix(TSubMatrixDim<0,3,6,3>())
#define vrow_q() vsub_matrix(0,0,3,njoint)

void Wholebody::TransformJacobian(WholebodyData& d){
	int nlink  = (int)links .size();
	int nchain = (int)chains.size();
	int njoint = (int)joints.size();
	int nlimit = (int)limits.size();

	mat3_t X, Xinv;
	if(param.parametrization == Parametrization::Joint){
		X.clear();
		for(int i = 0; i < nlink; i++){
			X += links[i].mass*d.Jfk_vi_v0[i].vv();
		}
		Xinv = X.inv();

		d.Jfk_vi_v0[0].vw().clear();
		for(int i = 1; i < nlink; i++){
			d.Jfk_vi_v0[0].vw() += links[i].mass*d.Jfk_vi_v0[i].vw();
		}
		d.Jfk_vi_v0[0].vw() = -Xinv*d.Jfk_vi_v0[0].vw();

		d.Jfk_vi_q[0].vrow_q().clear();
		for(int i = 1; i < nlink; i++){
			d.Jfk_vi_q[0].vrow_q() += links[i].mass*d.Jfk_vi_q[i].vrow_q();
		}
		d.Jfk_vi_q[0].vrow_q() = -Xinv*d.Jfk_vi_q[0].vrow_q();

		for(int i = 1; i < nlink; i++){
			d.Jfk_vi_v0[i].wcol() += d.Jfk_vi_v0[i].vcol()*d.Jfk_vi_v0[0].vw();
			d.Jfk_vi_q[i]         += d.Jfk_vi_v0[i].vcol()*d.Jfk_vi_q[0].vrow_q();
		}
	}
	else{
		X.clear();
		for(int i = 0; i < nlink; i++){
			X += links[i].mass*d.Jfkik_vi_v0[i].vv();
		}
		Xinv = X.inv();

		d.Jfkik_vi_v0[0].vw().clear();
		for(int i = 1; i < nlink; i++){
			d.Jfkik_vi_v0[0].vw() += links[i].mass*d.Jfkik_vi_v0[i].vw();
		}
		d.Jfkik_vi_v0[0].vw() = -Xinv*d.Jfkik_vi_v0[0].vw();

		for(int ic = 0; ic < nchain; ic++){
			d.Jfkik_vi_ve[0][ic].vrow().clear();
		
			for(int i = 1; i < nlink; i++){
				d.Jfkik_vi_ve[0][ic].vrow() += links[i].mass*d.Jfkik_vi_ve[i][ic].vrow();
			}

			d.Jfkik_vi_ve[0][ic].vrow() = -Xinv*d.Jfkik_vi_ve[0][ic].vrow();
		}

		for(int i = 1; i < nlink; i++){
			d.Jfkik_vi_v0[i].wcol() += d.Jfkik_vi_v0[i].vcol()*d.Jfkik_vi_v0[0].vw();

			for(int ic = 0; ic < nchain; ic++){
				d.Jfkik_vi_ve[i][ic] += d.Jfkik_vi_v0[i].vcol()*d.Jfkik_vi_ve[0][ic].vrow();
			}
		}

		for(int ic = 0; ic < nchain; ic++){
			d.Jik_e_v0.vsub_matrix(0,3,nlimit,3) += d.Jik_e_v0.vsub_matrix(0,0,nlimit,3)*d.Jfkik_vi_v0[0].vw();
			d.Jik_e_ve[ic]                       += d.Jik_e_v0.vsub_matrix(0,0,nlimit,3)*d.Jfkik_vi_ve[0][ic].vrow();
		}	

		for(int ic = 0; ic < nchain; ic++){
			d.Jik_q_v0.vsub_matrix(0,3,njoint,3) += d.Jik_q_v0.vsub_matrix(0,0,njoint,3)*d.Jfkik_vi_v0[0].vw();
			d.Jik_q_ve[ic]                       += d.Jik_q_v0.vsub_matrix(0,0,njoint,3)*d.Jfkik_vi_ve[0][ic].vrow();
		}
	}
}

#undef v
#undef w
#undef vv
#undef vw
#undef vrow
#undef vcol
#undef wcol
#undef vrow_q

void Wholebody::CalcVelocity(WholebodyData& d, bool fk_or_ik){
	int nlink  = (int)links .size();
	int nend   = (int)ends  .size();
	int njoint = (int)joints.size();

	if(fk_or_ik){
		/*
		d.links[0].vel_t.clear();
		d.links[0].vel_r = d.base.vel_r;
		
		for(Chain& ch : chains){
			for(int i : ch.ilink){
				int ip = links[i].iparent;
				WholebodyData::Link& dlnk  = d.links[i];
				WholebodyData::Link& dlnkp = d.links[ip];

				dlnk.vel_t = dlnkp.vel_t + dlnkp.vel_r % (dlnkp.pos_r*links[i].trn);
				dlnk.vel_r = dlnkp.vel_r + dlnkp.pos_r*(links[i].axis*d.qd[links[i].ijoint]);
			}
		}

		// calc com velocity
		vec3_t vc;
		for(int i = 0; i < nlink; i++){
			vc += links[i].mass_ratio*d.links[i].vel_t;
		}
		
		// subtract calculated com vel
		for(int i = 0; i < nlink; i++){
			d.links[i].vel_t -= vc;
		}
		*/
		vec6_t vi;
		for(int i = 0; i < nlink; i++){
			vi.clear();
			vi += d.Jfk_vi_v0[i].sub_matrix(TSubMatrixDim<0,3,6,3>())*d.base.vel_r;
			for(int j = 0; j < njoint; j++){
				vi += d.Jfk_vi_q[i].col(j)*d.qd[j];
			}
			d.links[i].vel_t = vi.sub_vector(TSubVectorDim<0,3>());
			d.links[i].vel_r = vi.sub_vector(TSubVectorDim<3,3>());
		}

		// calc end vel
		for(int iend = 0; iend < nend; iend++){
			int i = ends[iend].ilink;
			d.ends[iend].vel_t = d.links[i].vel_t + d.links[i].vel_r%(d.links[i].pos_r*ends[iend].offset);
			d.ends[iend].vel_r = d.links[i].vel_r;
		}
	}
	else{
		vec6_t vi, ve;
		for(int i = 0; i < nlink; i++){
			vi.clear();
			vi += d.Jfkik_vi_v0[i].sub_matrix(TSubMatrixDim<0,3,6,3>())*d.base.vel_r;
			for(int iend = 0; iend < nend; iend++){
				ve.sub_vector(TSubVectorDim<0,3>()) = d.ends[iend].vel_t;
				ve.sub_vector(TSubVectorDim<3,3>()) = d.ends[iend].vel_r;
				vi += d.Jfkik_vi_ve[i][iend]*ve;
			}
			d.links[i].vel_t = vi.sub_vector(TSubVectorDim<0,3>());
			d.links[i].vel_r = vi.sub_vector(TSubVectorDim<3,3>());
		}

		// calc joint velocity
		d.qd.clear();
		d.qd += d.Jik_q_v0.vsub_matrix(0,3,njoint,3)*d.base.vel_r;
		for(int iend = 0; iend < nend; iend++){
			ve.sub_vector(TSubVectorDim<0,3>()) = d.ends[iend].vel_t;
			ve.sub_vector(TSubVectorDim<3,3>()) = d.ends[iend].vel_r;
			d.qd += d.Jik_q_ve[iend]*ve;
		}
	}
}

void Wholebody::CalcAcceleration(WholebodyData& d, bool fk_or_ik){
	int nlink  = (int)links.size();
	int nend   = (int)ends .size();
	int njoint = (int)joints.size();

	if(fk_or_ik){
		vec6_t ai;
		for(int i = 0; i < nlink; i++){
			ai.clear();
			ai += d.Jfk_vi_v0[i].sub_matrix(TSubMatrixDim<0,3,6,3>())*d.base.acc_r;
			for(int j = 0; j < njoint; j++){
				ai += d.Jfk_vi_q[i].col(j)*d.qdd[j];
			}
			d.links[i].acc_t = ai.sub_vector(TSubVectorDim<0,3>());
			d.links[i].acc_r = ai.sub_vector(TSubVectorDim<3,3>());
		}

		// calc end vel
		for(int iend = 0; iend < nend; iend++){
			int i = ends[iend].ilink;
			d.ends[iend].acc_t = d.links[i].acc_t + d.links[i].acc_r%(d.links[i].pos_r*ends[iend].offset);
			d.ends[iend].acc_r = d.links[i].acc_r;
		}
	}
	else{
		vec6_t ai, ae;
		for(int i = 0; i < nlink; i++){
			ai.clear();
			ai += d.Jfkik_vi_v0[i].sub_matrix(TSubMatrixDim<0,3,6,3>())*d.base.acc_r;
			for(int iend = 0; iend < nend; iend++){
				ae.sub_vector(TSubVectorDim<0,3>()) = d.ends[iend].acc_t;
				ae.sub_vector(TSubVectorDim<3,3>()) = d.ends[iend].acc_r;
				ai += d.Jfkik_vi_ve[i][iend]*ae;
			}
			d.links[i].acc_t = ai.sub_vector(TSubVectorDim<0,3>());
			d.links[i].acc_r = ai.sub_vector(TSubVectorDim<3,3>());
		}
	}
}

void Wholebody::CalcMomentum(WholebodyData& d){
	int nlink = (int)links.size();

	// calc inertial matrix
	d.centroid.I.clear();
	for(int j = 0; j < nlink; j++){
		vec3_t rj  = d.centroid.pos_r*d.links[j].pos_t;
		quat_t qj  = d.centroid.pos_r*d.links[j].pos_r;
		mat3_t Rj;
		qj.ToMatrix(Rj);
		mat3_t rjc = mat3_t::Cross(rj);
		mat3_t Ij  = Rj*links[j].inertia*Rj.trans();
		d.centroid.I += links[j].mass*(rjc*rjc.trans()) + Ij;
	}
	d.centroid.Iinv = d.centroid.I.inv();

	// calc momentum in local coordinate
	d.centroid.L.clear();
	for(int j = 0; j < nlink; j++){
		WholebodyData::Link& dlnk  = d.links[j];
		
		// local inertia
		mat3_t Rj_local;
		dlnk.pos_r.ToMatrix(Rj_local);
		dlnk.I = Rj_local*links[j].inertia*Rj_local.trans();

		d.centroid.L += dlnk.pos_t % (links[j].mass*dlnk.vel_t) + dlnk.I*dlnk.vel_r;
	}
}

void Wholebody::CalcMomentumDerivative(WholebodyData& d){
	int nlink = (int)links.size();

	// calc momentum derivative in local coordinate
	d.centroid.Ld.clear();
	for(int i = 0; i < nlink; i++){
		WholebodyData::Link& dlnk  = d.links[i];

		d.centroid.Ld += dlnk.pos_t % (links[i].mass*dlnk.acc_t) + dlnk.I*dlnk.acc_r;
	}
}
	
void Wholebody::CalcForce(WholebodyData & d){
	int nchain = (int)chains.size();
	int nend   = (int)ends.size();

	// transform and copy end forces to links
	for(int i = 0; i < nend; i++){
		WholebodyData::End&  dend = d.ends[i];
	    WholebodyData::Link& dlnk = d.links[ends[i].ilink];
		
		dlnk.force_t = dend.force_t;
		dlnk.force_r = dend.force_r + ends[i].offset % dend.force_t;
	}

	vec3_t ac = d.centroid.acc_t;
	quat_t q0 = d.centroid.pos_r;
	vec3_t w0 = d.centroid.vel_r;
	vec3_t u0 = d.centroid.acc_r;

	// trace chains in reverse order
	for(int ic = nchain-1; ic >= 0; ic--){
		int nlink = (int)chains[ic].ilink.size();
		for(int j = nlink-1; j >= 0; j--){
			int i = chains[ic].ilink[j];
			WholebodyData::Link& dlnk = d.links[i];
			
			dlnk.force_t_child.clear();
			dlnk.force_r_child.clear();

			for(int ic : links[i].ichildren){
				WholebodyData::Link& dlnkc = d.links[ic];
		
				vec3_t r = q0*(dlnkc.pos_t - dlnk.pos_t);
				dlnk.force_t_child -=  dlnkc.force_t_par;
				dlnk.force_r_child -= (dlnkc.force_r_par + r % dlnkc.force_t_par);
			}

			// pj = pc * q0 * pjhat;
			// vj = vc + w0 % (q0*pjhat) + q0*vjhat
			// aj = ac + u0 % (q0*pjhat) + w0 % (w0 % (q0*pjhat)) + 2*w0 % (q0*vjhat) + q0*ajhat;

			// qj = q0 * qjhat;
			// wj = w0 + q0*wjhat;
			// uj = u0 + q0*ujhat + w0 % (q0*wjhat);
			vec3_t acc_t_abs   = ac + u0 % (q0*dlnk.pos_t) + w0 % (w0 % (q0*dlnk.pos_t)) + 2.0*(w0 % (q0*dlnk.vel_t)) + q0*dlnk.acc_t;
			vec3_t vel_r_abs   = w0 + q0*dlnk.vel_r;
			vec3_t acc_r_abs   = u0 + w0 % (q0*dlnk.vel_r) + q0*dlnk.acc_r;
			
			quat_t qj = q0*dlnk.pos_r;
			vec3_t acc_r_local = qj.Conjugated()*acc_r_abs;
			vec3_t vel_r_local = qj.Conjugated()*vel_r_abs;
					
			dlnk.force_t_par = links[i].mass*(acc_t_abs + vec3_t(0.0, 0.0, param.gravity)) - dlnk.force_t - dlnk.force_t_child;
			dlnk.force_r_par = qj*(links[j].inertia*acc_r_local - vel_r_local % (links[j].inertia*vel_r_local)) - dlnk.force_r - dlnk.force_r_child;
				
			// calc joint torque
			d.tau[links[i].ijoint] = links[i].axis*dlnk.force_r_par;
		}
	}
}

void Wholebody::ComState (real_t t, vec3_t& pos, vec3_t& vel){
	if(traj.empty())
		return;

	KeyPair       kp = traj.GetSegment(t);
	WholebodyKey* k0 = (WholebodyKey*)kp.first;
	WholebodyKey* k1 = (WholebodyKey*)kp.second;

    if(k1 == k0->next){
        pos = InterpolatePos(
		    t,
		    k0->tick->time, k0->data.centroid.pos_t, k0->data.centroid.vel_t,
		    k1->tick->time, k1->data.centroid.pos_t, k1->data.centroid.vel_t,
		    Interpolate::LinearDiff);
        vel = InterpolateVel(
		    t,
		    k0->tick->time, k0->data.centroid.pos_t, k0->data.centroid.vel_t,
		    k1->tick->time, k1->data.centroid.pos_t, k1->data.centroid.vel_t,
		    Interpolate::LinearDiff);
    }
    else{
        pos = k0->data.centroid.pos_t;
		vel = k0->data.centroid.vel_t;
    }
}

void Wholebody::BaseState(real_t t, quat_t& ori, vec3_t& angvel) {
	if(traj.empty())
		return;

	KeyPair       kp = traj.GetSegment(t);
	WholebodyKey* k0 = (WholebodyKey*)kp.first;
	WholebodyKey* k1 = (WholebodyKey*)kp.second;

    if(k1 == k0->next){
        ori = InterpolateOri(
		    t,
		    k0->tick->time, k0->data.centroid.pos_r, k0->data.centroid.vel_r,
		    k1->tick->time, k1->data.centroid.pos_r, k1->data.centroid.vel_r,
		    Interpolate::SlerpDiff);
        angvel = k0->data.centroid.vel_r;
    }
    else{
        ori    = k0->data.centroid.pos_r;
        angvel = k0->data.centroid.vel_r;
    }
}

void Wholebody::LinkPose(real_t t, int i, vec3_t& pos, quat_t& ori) {
	if(traj.empty())
		return;

	KeyPair      kp = traj.GetSegment(t);
	WholebodyKey* k0 = (WholebodyKey*)kp.first;
	WholebodyKey* k1 = (WholebodyKey*)kp.second;

    if(k1 == k0->next){
        pos = InterpolatePos(
		    t,
		    k0->tick->time, k0->data.links[i].pos_t, k0->data.links[i].vel_t,
		    k1->tick->time, k1->data.links[i].pos_t, k1->data.links[i].vel_t,
		    Interpolate::LinearDiff);
        ori = InterpolateOri(
		    t,
		    k0->tick->time, k0->data.links[i].pos_r, k0->data.links[i].vel_r,
		    k1->tick->time, k1->data.links[i].pos_r, k1->data.links[i].vel_r,
		    Interpolate::SlerpDiff);
    }
    else{
        pos = k0->data.links[i].pos_t;
        ori = k0->data.links[i].pos_r;
    }
}

void Wholebody::LinkVelocity(real_t t, int i, vec3_t& vel, vec3_t& angvel) {
	if(traj.empty())
		return;

	KeyPair      kp = traj.GetSegment(t);
	WholebodyKey* k0 = (WholebodyKey*)kp.first;
	WholebodyKey* k1 = (WholebodyKey*)kp.second;

    vel    = k0->data.links[i].vel_t;
    angvel = k0->data.links[i].vel_r;
}

void Wholebody::LinkForce(real_t t, int i, vec3_t& force, vec3_t& moment){
	if(traj.empty())
		return;

	KeyPair      kp = traj.GetSegment(t);
	WholebodyKey* k0 = (WholebodyKey*)kp.first;
	WholebodyKey* k1 = (WholebodyKey*)kp.second;

    force  = k0->data.links[i].force_t;
    moment = k0->data.links[i].force_r;
}

void Wholebody::CalcTrajectory() {
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

void Wholebody::Draw(Render::Canvas* canvas, Render::Config* conf) {
	TrajectoryNode::Draw(canvas, conf);

	if (!trajReady)
		CalcTrajectory();

	if (trajectory.empty())
		return;

	stringstream ss;

	// link
	if (conf->Set(canvas, Render::Item::WholebodyLinkTraj, this)) {
		for(int i = 0; i < links.size(); i++){
			ss.str("");
			ss << i;
			canvas->BeginLayer("wholebody_link" + ss.str(), true);
			canvas->SetPointSize(5.0);
			vec3_t pc = trajectory[0].com_pos;
			quat_t q0 = trajectory[0].base_pos_r;
			vec3_t pi = trajectory[0].links[i].pos_t;
			canvas->Point(Vec3f(pc + q0*pi));
			canvas->BeginPath();
			canvas->MoveTo(pc + q0*pi);
			for (int k = 1; k < trajectory.size(); k++) {
				pc = trajectory[k].com_pos;
				q0 = trajectory[k].base_pos_r;
				pi = trajectory[k].links[i].pos_t;
				canvas->LineTo(pc + q0*pi);
			}
			canvas->EndPath();
			canvas->EndLayer();
		}
	}
}

void Wholebody::CreateSnapshot(real_t t, Wholebody::Snapshot& s){
	s.t = t;
    
	ComState (t, s.com_pos, s.com_vel);
	BaseState(t, s.base_pos_r, s.base_vel_r);

	s.links.resize(links.size());
	for(int i = 0; i < links.size(); i++){
        LinkPose    (t, i, s.links[i].pos_t  , s.links[i].pos_r  );
        LinkVelocity(t, i, s.links[i].vel_t  , s.links[i].vel_r  );
        LinkForce   (t, i, s.links[i].force_t, s.links[i].force_r);
	}
}

void Wholebody::CreateSnapshot(real_t t){
	CreateSnapshot(t, snapshot);
}

void Wholebody::DrawSnapshot(Render::Canvas* canvas, Render::Config* conf) {
	if (conf->Set(canvas, Render::Item::WholebodyLink, this)) {
		vec3_t pc = snapshot.com_pos;
		quat_t q0 = snapshot.base_pos_r;

		for(int i = 1; i < links.size(); i++){
			canvas->BeginLayer("wholebody_link_snapshot", true);
			canvas->SetLineColor(i % 2 == 0 ? "cyan" : "blue");

			int ipar = links[i].iparent;
			
			// line connecting each link and its parent
			canvas->BeginPath();
			canvas->MoveTo(pc + q0*snapshot.links[ipar].pos_t);
			canvas->LineTo(pc + q0*snapshot.links[i   ].pos_t);
			canvas->EndPath();
			
            // line indicating force
			canvas->SetLineColor("magenta");
			canvas->BeginPath();
			canvas->MoveTo(pc + q0*snapshot.links[i].pos_t);
			canvas->LineTo(pc + q0*snapshot.links[i].pos_t + 0.001*snapshot.links[i].force_t);
			canvas->EndPath();
			canvas->EndLayer();
		}

		for(int i = 0; i < ends.size(); i++){
			int ilink = ends[i].ilink;
			canvas->SetPointSize(5.0);
			canvas->Point(Vec3f(pc + q0*snapshot.links[ilink].pos_t));
		}
	}	
}

///////////////////////////////////////////////////////////////////////////////////////////////////

WholebodyCon::WholebodyCon(Solver* solver, int _dim, int _tag, string _name, WholebodyKey* _obj, real_t _scale):
	Constraint(solver, _dim, ID(_tag, _obj->node, _obj->tick, _name), Constraint::Type::Equality, _scale) {
	obj[0] = _obj;
	obj[1] = (WholebodyKey*)_obj->next;
}

WholebodyPosConT::WholebodyPosConT(Solver* solver, string _name, WholebodyKey* _obj, int _iend, real_t _scale):
	WholebodyCon(solver, 3, ConTag::WholebodyPosT, _name, _obj, _scale) {

	iend = _iend;

	AddSLink(obj[1]->ends[iend].var_pos_t);
	AddSLink(obj[0]->ends[iend].var_pos_t);
	AddSLink(obj[0]->ends[iend].var_vel_t);
	AddSLink(obj[0]->ends[iend].var_acc_t);
}

WholebodyPosConR::WholebodyPosConR(Solver* solver, string _name, WholebodyKey* _obj, int _iend, real_t _scale):
	WholebodyCon(solver, 3, ConTag::WholebodyPosR, _name, _obj, _scale) {

	iend = _iend;

	if(iend == -1){
		AddSLink(obj[1]->base.var_pos_r);
		AddSLink(obj[0]->base.var_pos_r);
		AddSLink(obj[0]->base.var_vel_r);
		AddSLink(obj[0]->base.var_acc_r);
	}
	else{
		AddSLink(obj[1]->ends[iend].var_pos_r);
		AddSLink(obj[0]->ends[iend].var_pos_r);
		AddSLink(obj[0]->ends[iend].var_vel_r);
		AddSLink(obj[0]->ends[iend].var_acc_r);
	}
}

WholebodyVelConT::WholebodyVelConT(Solver* solver, string _name, WholebodyKey* _obj, int _iend, real_t _scale):
	WholebodyCon(solver, 3, ConTag::WholebodyVelT, _name, _obj, _scale) {

	iend = _iend;

	AddSLink(obj[1]->ends[iend].var_vel_t);
	AddSLink(obj[0]->ends[iend].var_vel_t);
	AddSLink(obj[0]->ends[iend].var_acc_t);
}

WholebodyVelConR::WholebodyVelConR(Solver* solver, string _name, WholebodyKey* _obj, int _iend, real_t _scale):
	WholebodyCon(solver, 3, ConTag::WholebodyVelR, _name, _obj, _scale) {

	iend = _iend;

	if(iend == -1){
		AddSLink(obj[1]->base.var_vel_r);
		AddSLink(obj[0]->base.var_vel_r);
		AddSLink(obj[0]->base.var_acc_r);
	}
	else{
		AddSLink(obj[1]->ends[iend].var_vel_r);
		AddSLink(obj[0]->ends[iend].var_vel_r);
		AddSLink(obj[0]->ends[iend].var_acc_r);
	}
}

WholebodyJointPosCon::WholebodyJointPosCon(Solver* solver, string _name, WholebodyKey* _obj, int _ijoint, real_t _scale):
	WholebodyCon(solver, 1, ConTag::WholebodyJointPos, _name, _obj, _scale) {

	ijoint = _ijoint;

	AddSLink(obj[1]->joints[ijoint].var_q  );
	AddSLink(obj[0]->joints[ijoint].var_q  );
	AddSLink(obj[0]->joints[ijoint].var_qd );
	AddSLink(obj[0]->joints[ijoint].var_qdd);
}

WholebodyJointVelCon::WholebodyJointVelCon(Solver* solver, string _name, WholebodyKey* _obj, int _ijoint, real_t _scale):
	WholebodyCon(solver, 1, ConTag::WholebodyJointVel, _name, _obj, _scale) {

	ijoint = _ijoint;

	AddSLink(obj[1]->joints[ijoint].var_qd );
	AddSLink(obj[0]->joints[ijoint].var_qd );
	AddSLink(obj[0]->joints[ijoint].var_qdd);
}

WholebodyCentroidPosConT::WholebodyCentroidPosConT(Solver* solver, string _name, WholebodyKey* _obj, real_t _scale):
	WholebodyCon(solver, 3, ConTag::WholebodyPosT, _name, _obj, _scale) {

	AddSLink(obj[1]->centroid.var_pos_t);
	AddSLink(obj[0]->centroid.var_pos_t);
	AddSLink(obj[0]->centroid.var_vel_t);
	int nend   = (int)obj[0]->ends  .size();
	for(int i = 0; i < nend; i++){
		AddSLink(obj[0]->ends[i].var_force_t);
	}
}

WholebodyCentroidVelConT::WholebodyCentroidVelConT(Solver* solver, string _name, WholebodyKey* _obj, real_t _scale):
	WholebodyCon(solver, 3, ConTag::WholebodyVelT, _name, _obj, _scale) {

	AddSLink(obj[1]->centroid.var_vel_t);
	AddSLink(obj[0]->centroid.var_vel_t);
	int nend   = (int)obj[0]->ends  .size();
	for(int i = 0; i < nend; i++){
		AddSLink(obj[0]->ends[i].var_force_t);
	}
}

WholebodyCentroidPosConR::WholebodyCentroidPosConR(Solver* solver, string _name, WholebodyKey* _obj, real_t _scale):
	WholebodyCon(solver, 3, ConTag::WholebodyPosR, _name, _obj, _scale) {

	AddSLink(obj[1]->centroid.var_pos_r);
	AddSLink(obj[0]->centroid.var_pos_r);
	AddSLink(obj[0]->centroid.var_vel_r);

	AddM3Link(obj[0]->base.var_pos_r);
	AddM3Link(obj[0]->base.var_acc_r);
	
	int nend   = (int)obj[0]->ends  .size();
	int njoint = (int)obj[0]->joints.size();
	if(obj[0]->wb->param.useLd){
		if(obj[0]->wb->param.parametrization == Wholebody::Parametrization::Joint){
			for(int i = 0; i < njoint; i++)
				AddC3Link(obj[0]->joints[i].var_q);
			for(int i = 0; i < njoint; i++)
				AddC3Link(obj[0]->joints[i].var_qdd);
		}
		else{
			for(int i = 0; i < nend; i++){
				AddM3Link(obj[0]->ends[i].var_pos_t);
				AddM3Link(obj[0]->ends[i].var_pos_r);
				AddM3Link(obj[0]->ends[i].var_acc_t);
				AddM3Link(obj[0]->ends[i].var_acc_r);
			}
		}
	}
	for(int i = 0; i < nend; i++){
		AddM3Link(obj[0]->ends[i].var_force_t);
		AddM3Link(obj[0]->ends[i].var_force_r);
	}
}

WholebodyCentroidVelConR::WholebodyCentroidVelConR(Solver* solver, string _name, WholebodyKey* _obj, real_t _scale):
	WholebodyCon(solver, 3, ConTag::WholebodyVelR, _name, _obj, _scale) {

	AddSLink (obj[1]->centroid.var_vel_r);
	AddSLink (obj[0]->centroid.var_vel_r);

	AddM3Link(obj[0]->base.var_pos_r);
	AddM3Link(obj[0]->base.var_acc_r);

	int nend   = (int)obj[0]->ends  .size();
	int njoint = (int)obj[0]->joints.size();
	if(obj[0]->wb->param.useLd){
		if(obj[0]->wb->param.parametrization == Wholebody::Parametrization::Joint){
			for(int i = 0; i < njoint; i++)
				AddC3Link(obj[0]->joints[i].var_q);
			for(int i = 0; i < njoint; i++)
				AddC3Link(obj[0]->joints[i].var_qdd);
		}
		else{
			for(int i = 0; i < nend; i++){
				AddM3Link(obj[0]->ends[i].var_pos_t);
				AddM3Link(obj[0]->ends[i].var_pos_r);
				AddM3Link(obj[0]->ends[i].var_acc_t);
				AddM3Link(obj[0]->ends[i].var_acc_r);
			}
		}
	}
	for(int i = 0; i < nend; i++){
		AddM3Link(obj[0]->ends[i].var_force_t);
		AddM3Link(obj[0]->ends[i].var_force_r);
	}
}

WholebodyDesPosConT::WholebodyDesPosConT(Solver* solver, string _name, WholebodyKey* _obj, int _iend, real_t _scale):
	Constraint(solver, 3, ID(ConTag::WholebodyPosT, _obj->node, _obj->tick, _name), Constraint::Type::Equality, _scale){
	obj = _obj;
	iend = _iend;

	AddSLink (obj->centroid.var_pos_t);
	AddX3Link(obj->centroid.var_pos_r);

	if(obj->wb->param.parametrization == Wholebody::Parametrization::Joint){
		AddM3Link(obj->base.var_pos_r);

		int njoint = (int)obj->joints.size();
		for(int i = 0; i < njoint; i++)
			AddC3Link(obj->joints[i].var_q);
	}
	else{
		AddM3Link(obj->ends[iend].var_pos_t);
	}
}

WholebodyDesPosConR::WholebodyDesPosConR(Solver* solver, string _name, WholebodyKey* _obj, int _iend, real_t _scale):
	Constraint(solver, 3, ID(ConTag::WholebodyPosR, _obj->node, _obj->tick, _name), Constraint::Type::Equality, _scale){
	obj = _obj;
	iend = _iend;

	AddSLink (obj->centroid.var_pos_r);

	if(obj->wb->param.parametrization == Wholebody::Parametrization::Joint){
		AddM3Link(obj->base.var_pos_r);

		int njoint = (int)obj->joints.size();
		for(int i = 0; i < njoint; i++)
			AddC3Link(obj->joints[i].var_q);
	}
	else{
		AddM3Link(obj->ends[iend].var_pos_r);
	}
}

WholebodyDesVelConT::WholebodyDesVelConT(Solver* solver, string _name, WholebodyKey* _obj, int _iend, real_t _scale):
	Constraint(solver, 3, ID(ConTag::WholebodyVelT, _obj->node, _obj->tick, _name), Constraint::Type::Equality, _scale){
	obj = _obj;
	iend = _iend;

	AddSLink (obj->centroid.var_vel_t);
	AddX3Link(obj->centroid.var_vel_r);

	if(obj->wb->param.parametrization == Wholebody::Parametrization::Joint){
		AddM3Link(obj->base.var_vel_r);

		int njoint = (int)obj->joints.size();
		for(int i = 0; i < njoint; i++)
			AddC3Link(obj->joints[i].var_qd);
	}
	else{
		AddM3Link(obj->ends[iend].var_vel_t);
	}
}

WholebodyDesVelConR::WholebodyDesVelConR(Solver* solver, string _name, WholebodyKey* _obj, int _iend, real_t _scale):
	Constraint(solver, 3, ID(ConTag::WholebodyVelR, _obj->node, _obj->tick, _name), Constraint::Type::Equality, _scale){
	obj = _obj;
	iend = _iend;

	AddSLink (obj->centroid.var_vel_r);
	
	if(obj->wb->param.parametrization == Wholebody::Parametrization::Joint){
		AddM3Link(obj->base.var_vel_r);

		int njoint = (int)obj->joints.size();
		for(int i = 0; i < njoint; i++)
			AddC3Link(obj->joints[i].var_qd);
	}
	else{
		AddM3Link(obj->ends[iend].var_vel_r);
	}
}

WholebodyLimitCon::WholebodyLimitCon(Solver* solver, string _name, WholebodyKey* _obj, int _ierror, int _type, real_t _scale):
	Constraint(solver, 1, ID(ConTag::WholebodyLimit, _obj->node, _obj->tick, _name), _type, _scale){
	obj    = _obj;
	ierror = _ierror;
    
	int nend = (int)obj->ends.size();
	AddR3Link(obj->base.var_pos_r);
	for(int i = 0; i < nend; i++){
		AddR3Link(obj->ends[i].var_pos_t);
		AddR3Link(obj->ends[i].var_pos_r);
	}
}

/*WholebodyLdCon::WholebodyLdCon(Solver* solver, string _name, WholebodyKey* _obj, real_t _scale):
	Constraint(solver, 3, ID(ConTag::WholebodyMoment, _obj->node, _obj->tick, _name), Constraint::Type::Equality, _scale){
	obj = _obj;
	
	AddM3Link(obj->base.var_pos_r);
	AddM3Link(obj->base.var_acc_r);

	int nend = (int)obj->ends.size();
	for(int i = 0; i < nend; i++){
		AddM3Link(obj->ends[i].var_pos_t);
		AddM3Link(obj->ends[i].var_pos_r);
		AddM3Link(obj->ends[i].var_acc_t);
		AddM3Link(obj->ends[i].var_acc_r);
	}
}*/

WholebodyLCon::WholebodyLCon(Solver* solver, string _name, WholebodyKey* _obj, real_t _scale):
	Constraint(solver, 3, ID(ConTag::WholebodyMomentum, _obj->node, _obj->tick, _name), Constraint::Type::Equality, _scale){
	obj = _obj;
	
	AddM3Link(obj->base.var_pos_r);
	AddM3Link(obj->base.var_vel_r);

	if(obj->wb->param.parametrization == Wholebody::Parametrization::Joint){
		int njoint = (int)obj->joints.size();
		for(int i = 0; i < njoint; i++)
			AddC3Link(obj->joints[i].var_q);
		for(int i = 0; i < njoint; i++)
			AddC3Link(obj->joints[i].var_qd);
	}
	else{
		int nend = (int)obj->ends.size();
		for(int i = 0; i < nend; i++){
			AddM3Link(obj->ends[i].var_pos_t);
			AddM3Link(obj->ends[i].var_pos_r);
			AddM3Link(obj->ends[i].var_vel_t);
			AddM3Link(obj->ends[i].var_vel_r);
		}
	}
}

WholebodyContactPosConT::WholebodyContactPosConT(Solver* solver, string _name, WholebodyKey* _obj, int _iend, real_t _scale):
	Constraint(solver, 3, ID(ConTag::WholebodyContactPosT, _obj->node, _obj->tick, _name), Constraint::Type::Equality, _scale){
	obj  = _obj;
	iend = _iend;
    
	AddM3Link(obj->centroid.var_pos_t);
	AddM3Link(obj->centroid.var_pos_r);

	if(obj->wb->param.parametrization == Wholebody::Parametrization::Joint){
		int njoint = (int)obj->joints.size();
		for(int i = 0; i < njoint; i++)
			AddC3Link(obj->joints[i].var_q);
	}
	else{
		AddM3Link(obj->ends[iend].var_pos_t);
		AddM3Link(obj->ends[iend].var_pos_r);
	}
}

WholebodyContactPosConR::WholebodyContactPosConR(Solver* solver, string _name, WholebodyKey* _obj, int _iend, real_t _scale):
	Constraint(solver, 3, ID(ConTag::WholebodyContactPosR, _obj->node, _obj->tick, _name), Constraint::Type::Equality, _scale){
	obj  = _obj;
	iend = _iend;
    
	AddM3Link(obj->centroid.var_pos_r);

	if(obj->wb->param.parametrization == Wholebody::Parametrization::Joint){
		int njoint = (int)obj->joints.size();
		for(int i = 0; i < njoint; i++)
			AddC3Link(obj->joints[i].var_q);
	}
	else{
		AddM3Link(obj->ends[iend].var_pos_r);
	}
}

WholebodyContactVelConT::WholebodyContactVelConT(Solver* solver, string _name, WholebodyKey* _obj, int _iend, real_t _scale):
	Constraint(solver, 3, ID(ConTag::WholebodyContactVelT, _obj->node, _obj->tick, _name), Constraint::Type::Equality, _scale){
	obj  = _obj;
	iend = _iend;
    
	AddM3Link(obj->centroid.var_vel_t);
	AddM3Link(obj->centroid.var_vel_r);

	if(obj->wb->param.parametrization == Wholebody::Parametrization::Joint){
		int njoint = (int)obj->joints.size();
		for(int i = 0; i < njoint; i++)
			AddC3Link(obj->joints[i].var_qd);
	}
	else{
		AddM3Link(obj->ends[iend].var_vel_t);
		AddM3Link(obj->ends[iend].var_vel_r);
	}
}

WholebodyContactVelConR::WholebodyContactVelConR(Solver* solver, string _name, WholebodyKey* _obj, int _iend, /*int _dir, */real_t _scale):
	Constraint(solver, 3, ID(ConTag::WholebodyContactVelR, _obj->node, _obj->tick, _name), Constraint::Type::Equality, _scale){
	obj  = _obj;
	iend = _iend;
	
	AddM3Link(obj->centroid.var_vel_r);

	if(obj->wb->param.parametrization == Wholebody::Parametrization::Joint){
		int njoint = (int)obj->joints.size();
		for(int i = 0; i < njoint; i++)
			AddC3Link(obj->joints[i].var_qd);
	}
	else{
		AddM3Link(obj->ends[iend].var_vel_r);
	}
}

WholebodyNormalForceCon::WholebodyNormalForceCon(Solver* solver, string _name, WholebodyKey* _obj, int _iend, real_t _scale):
	Constraint(solver, 1, ID(ConTag::WholebodyNormalForce, _obj->node, _obj->tick, _name), Constraint::Type::InequalityPenalty, _scale){
	obj  = _obj;
	iend = _iend;
    
	AddR3Link(obj->ends[iend].var_force_t);
}

WholebodyFrictionForceCon::WholebodyFrictionForceCon(Solver* solver, string _name, WholebodyKey* _obj, int _iend, int _dir, int _side, real_t _scale):
	Constraint(solver, 1, ID(ConTag::WholebodyFrictionForce, _obj->node, _obj->tick, _name), Constraint::Type::InequalityPenalty, _scale){
	obj  = _obj;
	iend = _iend;
	dir  = _dir;
	side = _side;
    
	AddR3Link(obj->ends[iend].var_force_t);
}

WholebodyMomentCon::WholebodyMomentCon(Solver* solver, string _name, WholebodyKey* _obj, int _iend, int _dir, int _side, real_t _scale):
	Constraint(solver, 1, ID(ConTag::WholebodyMoment, _obj->node, _obj->tick, _name), Constraint::Type::InequalityPenalty, _scale){
	obj   = _obj;
	iend  = _iend;
	dir   = _dir;
	side  = _side;
    
	AddR3Link(obj->ends[iend].var_force_t);
	AddR3Link(obj->ends[iend].var_force_r);
}

///////////////////////////////////////////////////////////////////////////////////////////////////

void WholebodyPosConT::Prepare(){
	p0 = obj[0]->ends[iend].var_pos_t->val;
	v0 = obj[0]->ends[iend].var_vel_t->val;
	a0 = obj[0]->ends[iend].var_acc_t->val;
	p1 = obj[1]->ends[iend].var_pos_t->val;
	h  = obj[0]->hnext;
	h2 = h*h;
	
	p_rhs = p0 + h*v0 + (0.5*h2)*a0;
}

void WholebodyPosConR::Prepare(){
	if(iend == -1){
		q0 = obj[0]->base.var_pos_r->val;
		w0 = obj[0]->base.var_vel_r->val;
		u0 = obj[0]->base.var_acc_r->val;
		q1 = obj[1]->base.var_pos_r->val;
	}
	else{
		q0 = obj[0]->ends[iend].var_pos_r->val;
		w0 = obj[0]->ends[iend].var_vel_r->val;
		u0 = obj[0]->ends[iend].var_acc_r->val;
		q1 = obj[1]->ends[iend].var_pos_r->val;
	}
	h  = obj[0]->hnext;
	h2 = h*h;
	
	q_rhs = q0*quat_t::Rot(q0.Conjugated()*(h*w0 + (0.5*h2)*u0));
}

void WholebodyVelConT::Prepare(){
	v0 = obj[0]->ends[iend].var_vel_t->val;
	a0 = obj[0]->ends[iend].var_acc_t->val;
	v1 = obj[1]->ends[iend].var_vel_t->val;
	h  = obj[0]->hnext;

	v_rhs = v0 + h*a0;
}

void WholebodyVelConR::Prepare(){
	if(iend == -1){
		w0 = obj[0]->base.var_vel_r->val;
		u0 = obj[0]->base.var_acc_r->val;
		w1 = obj[1]->base.var_vel_r->val;
	}
	else{
		w0 = obj[0]->ends[iend].var_vel_r->val;
		u0 = obj[0]->ends[iend].var_acc_r->val;
		w1 = obj[1]->ends[iend].var_vel_r->val;
	}
	h  = obj[0]->hnext;

	w_rhs = w0 + h*u0;
}

void WholebodyJointPosCon::Prepare(){
	q0     = obj[0]->joints[ijoint].var_q  ->val;
	qd0    = obj[0]->joints[ijoint].var_qd ->val;
	qdd0   = obj[0]->joints[ijoint].var_qdd->val;
	q1     = obj[1]->joints[ijoint].var_q  ->val;
	h      = obj[0]->hnext;
	h2     = h*h;
	
	q_rhs = q0 + h*qd0 + (0.5*h2)*qdd0;
}

void WholebodyJointVelCon::Prepare(){
	qd0     = obj[0]->joints[ijoint].var_qd ->val;
	qdd0    = obj[0]->joints[ijoint].var_qdd->val;
	qd1     = obj[1]->joints[ijoint].var_qd ->val;
	h       = obj[0]->hnext;
	
	qd_rhs = qd0 + h*qdd0;
}

void WholebodyCentroidPosConT::Prepare(){
	pc0 = obj[0]->centroid.var_pos_t->val;
	vc0 = obj[0]->centroid.var_vel_t->val;
	pc1 = obj[1]->centroid.var_pos_t->val;
	h   = obj[0]->hnext;
	h2  = h*h;
	m   = obj[0]->wb->param.totalMass;
	g   = vec3_t(0.0, 0.0, -obj[0]->wb->param.gravity);
	ac0 = (1.0/m)*(obj[0]->fsum) + g;

	pc_rhs = pc0 + h*vc0 + ((1.0/2.0)*h2)*ac0;
}

void WholebodyCentroidVelConT::Prepare(){
	vc0 = obj[0]->centroid.var_vel_t->val;
	vc1 = obj[1]->centroid.var_vel_t->val;
	h   = obj[0]->hnext;
	m   = obj[0]->wb->param.totalMass;
	g   = vec3_t(0.0, 0.0, -obj[0]->wb->param.gravity);
	ac0 = (1.0/m)*(obj[0]->fsum) + g;

	vc_rhs = vc0 + h*ac0;
}

void WholebodyCentroidPosConR::Prepare(){
	w0   = obj[0]->centroid.var_vel_r->val;
	q1   = obj[1]->centroid.var_pos_r->val;
	h    = obj[0]->hnext;
	h2   = h*h;
	Ld   = obj[0]->data.centroid.Ld;
	Iinv = obj[0]->data.centroid.Iinv;
	if(obj[0]->wb->param.useLd)
		 u0 = Iinv*(obj[0]->msum - obj[0]->q0*Ld);
	else u0 = Iinv*(obj[0]->msum);
	
	q_rhs = obj[0]->q0*quat_t::Rot(obj[0]->q0.Conjugated()*(h*w0 + (0.5*h2)*u0));
}

void WholebodyCentroidVelConR::Prepare(){
	pc   = obj[0]->centroid.var_pos_t->val;
	w0   = obj[0]->centroid.var_vel_r->val;
	w1   = obj[1]->centroid.var_vel_r->val;
	Ld   = obj[0]->data.centroid.Ld;
	Iinv = obj[0]->data.centroid.Iinv;
	h    = obj[0]->hnext;
	if(obj[0]->wb->param.useLd)
		 u0 = Iinv*(obj[0]->msum - obj[0]->q0*Ld);
	else u0 = Iinv*(obj[0]->msum);
	
	w_rhs = w0 + h*u0;
}

void WholebodyDesPosConT::Prepare(){
	WholebodyKey::End& end = obj->ends[iend];
	WholebodyData::End& dend = obj->data.ends[iend];

	pc = obj->centroid.var_pos_t->val;
	q0 = obj->centroid.var_pos_r->val;
	q0.ToMatrix(R0);
	pi = dend.pos_t;
}

void WholebodyDesPosConR::Prepare(){
	WholebodyKey::End& end = obj->ends[iend];
	WholebodyData::End& dend = obj->data.ends[iend];

	q0 = obj->centroid.var_pos_r->val;
	q0.ToMatrix(R0);
	qi = dend.pos_r;
}

void WholebodyDesVelConT::Prepare(){
	WholebodyKey::End& end = obj->ends[iend];
	WholebodyData::End& dend = obj->data.ends[iend];

	vc = obj->centroid.var_vel_t->val;
	w0 = obj->centroid.var_vel_r->val;
	q0 = obj->centroid.var_pos_r->val;
	q0.ToMatrix(R0);
	pi = dend.pos_t;
	vi = dend.vel_t;
}

void WholebodyDesVelConR::Prepare(){
	WholebodyKey::End& end = obj->ends[iend];
	WholebodyData::End& dend = obj->data.ends[iend];

	w0 = obj->centroid.var_vel_r->val;
	q0 = obj->centroid.var_pos_r->val;
	q0.ToMatrix(R0);
	wi = dend.vel_r;
}

void WholebodyLimitCon::Prepare(){
	
}

/*void WholebodyLdCon::Prepare(){

}*/

void WholebodyLCon::Prepare(){

}

void WholebodyContactPosConT::Prepare(){
	WholebodyData::End&  dend  = obj->data.ends[iend];
	WholebodyData::End&  dend_des  = obj->data_des.ends[iend];
	
	po = dend_des.pos_tc;
	qo = dend_des.pos_rc;
	qo.ToMatrix(Ro);
	r  = dend_des.pos_te;
	pc = obj->centroid.var_pos_t->val;
	q0 = obj->centroid.var_pos_r->val;
	q0.ToMatrix(R0);
	pi = dend.pos_t;
	qi = dend.pos_r;
}

void WholebodyContactPosConR::Prepare(){
	WholebodyData::End& dend = obj->data.ends[iend];
	WholebodyData::End& dend_des = obj->data_des.ends[iend];
	
	qo = dend_des.pos_rc;
	qo.ToMatrix(Ro);
	q0 = obj->centroid.var_pos_r->val;
	q0.ToMatrix(R0);
	qi = dend.pos_r;
}

void WholebodyContactVelConT::Prepare(){
	WholebodyData::End& dend = obj->data.ends[iend];
	WholebodyData::End& dend_des = obj->data_des.ends[iend];
	
	qo = dend_des.pos_rc;
	qo.ToMatrix(Ro);
	r  = dend_des.pos_te;
	vc = obj->centroid.var_vel_t->val;
	q0 = obj->centroid.var_pos_r->val;
	q0.ToMatrix(R0);
	w0 = obj->centroid.var_vel_r->val;
	pi = dend.pos_t;
	qi = dend.pos_r;
	vi = dend.vel_t;
	wi = dend.vel_r;
}

void WholebodyContactVelConR::Prepare(){
	WholebodyData::End& dend = obj->data.ends[iend];
	WholebodyData::End& dend_des = obj->data_des.ends[iend];
	
	qo = dend_des.pos_rc;
	qo.ToMatrix(Ro);
	q0 = obj->centroid.var_pos_r->val;
	q0.ToMatrix(R0);
	w0 = obj->centroid.var_vel_r->val;
	qi = dend.pos_r;
	wi = dend.vel_r;
}

void WholebodyNormalForceCon::Prepare(){
	WholebodyKey ::End& end  = obj->ends[iend];
	WholebodyData::End& dend = obj->data.ends[iend];

	fn = end.var_force_t->val.z;
}

void WholebodyFrictionForceCon::Prepare(){
	WholebodyKey ::End& end  = obj->ends[iend];
	WholebodyData::End& dend = obj->data.ends[iend];
	WholebodyData::End& dend_des = obj->data_des.ends[iend];

	mu = dend_des.mu;
	vec3_t f = end.var_force_t->val;
	ft = (dir == 0 ? f.x : f.y);
	fn = f.z;
}

void WholebodyMomentCon::Prepare(){
	WholebodyKey ::End& end  = obj->ends[iend];
	WholebodyData::End& dend = obj->data.ends[iend];
	WholebodyData::End& dend_des = obj->data_des.ends[iend];

	fn   = end.var_force_t->val.z;
	m.x  = end.var_force_r->val.x;
	m.y  = end.var_force_r->val.y;
	cmin = dend_des.cop_min;
	cmax = dend_des.cop_max;
}

///////////////////////////////////////////////////////////////////////////////////////////////////

void WholebodyPosConT::CalcCoef(){
	Prepare();

	int idx = 0;
	((SLink*)links[idx++])->SetCoef( 1.0);
	((SLink*)links[idx++])->SetCoef(-1.0);
	((SLink*)links[idx++])->SetCoef(-h);
	((SLink*)links[idx++])->SetCoef(-(0.5*h2));
}

void WholebodyPosConR::CalcCoef(){
	Prepare();

	int idx = 0;
	((SLink*)links[idx++])->SetCoef( 1.0);
	((SLink*)links[idx++])->SetCoef(-1.0);
	((SLink*)links[idx++])->SetCoef(-h);
	((SLink*)links[idx++])->SetCoef(-(0.5*h2));
}

void WholebodyVelConT::CalcCoef(){
	Prepare();

	int idx = 0;
	((SLink*)links[idx++])->SetCoef( 1.0);
	((SLink*)links[idx++])->SetCoef(-1.0);
	((SLink*)links[idx++])->SetCoef(-h);
}

void WholebodyVelConR::CalcCoef(){
	Prepare();

	int idx = 0;
	((SLink*)links[idx++])->SetCoef( 1.0);
	((SLink*)links[idx++])->SetCoef(-1.0);
	((SLink*)links[idx++])->SetCoef(-h);
}

void WholebodyJointPosCon::CalcCoef(){
	Prepare();

	int idx = 0;
	((SLink*)links[idx++])->SetCoef( 1.0);
	((SLink*)links[idx++])->SetCoef(-1.0);
	((SLink*)links[idx++])->SetCoef(-h);
	((SLink*)links[idx++])->SetCoef(-(0.5*h2));
}

void WholebodyJointVelCon::CalcCoef(){
	Prepare();

	int idx = 0;
	((SLink*)links[idx++])->SetCoef( 1.0);
	((SLink*)links[idx++])->SetCoef(-1.0);
	((SLink*)links[idx++])->SetCoef(-h);
}

void WholebodyCentroidPosConT::CalcCoef(){
	Prepare();

	int idx = 0;
	((SLink*)links[idx++])->SetCoef( 1.0);
	((SLink*)links[idx++])->SetCoef(-1.0);
	((SLink*)links[idx++])->SetCoef(-h);

	int nend = (int)obj[0]->ends.size();
	for(int i = 0; i < nend; i++){
		((SLink*)links[idx++])->SetCoef(-(0.5*h2/m));
	}
}

void WholebodyCentroidVelConT::CalcCoef(){
	Prepare();

	int idx = 0;
	((SLink*)links[idx++])->SetCoef( 1.0);
	((SLink*)links[idx++])->SetCoef(-1.0);

	int nend = (int)obj[0]->ends.size();
	for(int i = 0; i < nend; i++){
		((SLink*)links[idx++])->SetCoef(-(h/m));
	}
}

void WholebodyCentroidPosConR::CalcCoef(){
	Prepare();

	mat3_t tmp = (0.5*h2)*Iinv;
	mat3_t tmpR0 = tmp*obj[0]->R0;
	
	int idx = 0;
	((SLink*)links[idx++])->SetCoef( 1.0);
	((SLink*)links[idx++])->SetCoef(-1.0);
	((SLink*)links[idx++])->SetCoef(-h);

	((M3Link*)links[idx++])->SetCoef( tmpR0*obj[0]->J_Ld_qb);
	((M3Link*)links[idx++])->SetCoef( tmpR0*obj[0]->J_Ld_ub);
	
	int nend = (int)obj[0]->ends.size();
	if(obj[0]->wb->param.useLd){
		if(obj[0]->wb->param.parametrization == Wholebody::Parametrization::Joint){
			int njoint = (int)obj[0]->joints.size();
			for(int i = 0; i < njoint; i++){
				((C3Link*)links[idx++])->SetCoef(tmpR0*obj[0]->J_Ld_q  .col(i));
			}
			for(int i = 0; i < njoint; i++){
				((C3Link*)links[idx++])->SetCoef(tmpR0*obj[0]->J_Ld_qdd.col(i));
			}
		}
		else{
			for(int i = 0; i < nend; i++){
				((M3Link*)links[idx++])->SetCoef( tmpR0*obj[0]->J_Ld_pe[i] + tmp*mat3_t::Cross(obj[0]->fe[i])*obj[0]->R0);
				((M3Link*)links[idx++])->SetCoef( tmpR0*obj[0]->J_Ld_qe[i]);
				((M3Link*)links[idx++])->SetCoef( tmpR0*obj[0]->J_Ld_ae[i]);
				((M3Link*)links[idx++])->SetCoef( tmpR0*obj[0]->J_Ld_ue[i]);
			}
		}
	}
	// (R0*re) x fe = -fe^x*R0*re
	for(int i = 0; i < nend; i++){
		//((M3Link*)links[idx++])->SetCoef(-tmp*obj[0]->J_mc_fe[i]);
		//((M3Link*)links[idx++])->SetCoef(-tmp*obj[0]->J_mc_me[i]);
		((M3Link*)links[idx++])->SetCoef(-tmp*obj[0]->rec[i]);
		((M3Link*)links[idx++])->SetCoef(-tmp);
	}
}

void WholebodyCentroidVelConR::CalcCoef(){
	Prepare();

	mat3_t tmp   = h*Iinv;
	mat3_t tmpR0 = tmp*obj[0]->R0;

	int idx = 0;
	((SLink*)links[idx++])->SetCoef( 1.0);
	((SLink*)links[idx++])->SetCoef(-1.0);

	((M3Link*)links[idx++])->SetCoef( tmpR0*obj[0]->J_Ld_qb);
	((M3Link*)links[idx++])->SetCoef( tmpR0*obj[0]->J_Ld_ub);

	int nend = (int)obj[0]->ends.size();
	if(obj[0]->wb->param.useLd){
		if(obj[0]->wb->param.parametrization == Wholebody::Parametrization::Joint){
			int njoint = (int)obj[0]->joints.size();
			for(int i = 0; i < njoint; i++){
				((C3Link*)links[idx++])->SetCoef(tmpR0*obj[0]->J_Ld_q  .col(i));
			}
			for(int i = 0; i < njoint; i++){
				((C3Link*)links[idx++])->SetCoef(tmpR0*obj[0]->J_Ld_qdd.col(i));
			}
		}
		else{
			for(int i = 0; i < nend; i++){
				((M3Link*)links[idx++])->SetCoef( tmpR0*obj[0]->J_Ld_pe[i] + tmp*mat3_t::Cross(obj[0]->fe[i])*obj[0]->R0);
				((M3Link*)links[idx++])->SetCoef( tmpR0*obj[0]->J_Ld_qe[i]);
				((M3Link*)links[idx++])->SetCoef( tmpR0*obj[0]->J_Ld_ae[i]);
				((M3Link*)links[idx++])->SetCoef( tmpR0*obj[0]->J_Ld_ue[i]);
			}
		}
	}
	for(int i = 0; i < nend; i++){
		//((M3Link*)links[idx++])->SetCoef(-tmp*obj[0]->J_mc_fe[i]);
		//((M3Link*)links[idx++])->SetCoef(-tmp*obj[0]->J_mc_me[i]);
		((M3Link*)links[idx++])->SetCoef(-tmp*obj[0]->rec[i]);
		((M3Link*)links[idx++])->SetCoef(-tmp);
	}
}

void WholebodyDesPosConT::CalcCoef(){
	Prepare();

	int idx = 0;
	((SLink *)links[idx++])->SetCoef(1.0);
	((X3Link*)links[idx++])->SetCoef(-(q0*pi));

	if(obj->wb->param.parametrization == Wholebody::Parametrization::Joint){
		int i = obj->wb->ends[iend].ilink;
		int njoint = (int)obj->joints.size();

		mat3_t Jvw = obj->data.Jfk_vi_v0[i].sub_matrix(TSubMatrixDim<0,3,3,3>());
		((M3Link*)links[idx++])->SetCoef(R0*Jvw);

		for(int j = 0; j < njoint; j++){
			vec3_t Jv = obj->data.Jfk_vi_q[i].col(j).sub_vector(TSubVectorDim<0,3>());
			((C3Link*)links[idx++])->SetCoef(R0*Jv);
		}
	}
	else{
		((M3Link*)links[idx++])->SetCoef(R0);
	}
}

void WholebodyDesPosConR::CalcCoef(){
	Prepare();

	int idx = 0;
	((SLink *)links[idx++])->SetCoef(1.0);

	if(obj->wb->param.parametrization == Wholebody::Parametrization::Joint){
		int njoint = (int)obj->joints.size();
		int i = obj->wb->ends[iend].ilink;

		mat3_t Jww = obj->data.Jfk_vi_v0[i].sub_matrix(TSubMatrixDim<3,3,3,3>());
		((M3Link*)links[idx++])->SetCoef(R0*Jww);

		for(int j = 0; j < njoint; j++){
			vec3_t Jw = obj->data.Jfk_vi_q[i].col(j).sub_vector(TSubVectorDim<3,3>());
			((C3Link*)links[idx++])->SetCoef(R0*Jw);
		}
	}
	else{
		((M3Link*)links[idx++])->SetCoef(R0);
	}
}

void WholebodyDesVelConT::CalcCoef(){
	Prepare();

	int idx = 0;
	((SLink *)links[idx++])->SetCoef(1.0);
	((X3Link*)links[idx++])->SetCoef(-(q0*pi));
	
	if(obj->wb->param.parametrization == Wholebody::Parametrization::Joint){
		int njoint = (int)obj->joints.size();
		int i = obj->wb->ends[iend].ilink;

		mat3_t Jvw = obj->data.Jfk_vi_v0[i].sub_matrix(TSubMatrixDim<0,3,3,3>());
		((M3Link*)links[idx++])->SetCoef(R0*Jvw);

		for(int j = 0; j < njoint; j++){
			vec3_t Jv = obj->data.Jfk_vi_q[i].col(j).sub_vector(TSubVectorDim<0,3>());
			((C3Link*)links[idx++])->SetCoef(R0*Jv);
		}
	}
	else{
		((M3Link*)links[idx++])->SetCoef(R0);
	}
}

void WholebodyDesVelConR::CalcCoef(){
	Prepare();

	int idx = 0;
	((SLink *)links[idx++])->SetCoef(1.0);

	if(obj->wb->param.parametrization == Wholebody::Parametrization::Joint){
		int njoint = (int)obj->joints.size();
		int i = obj->wb->ends[iend].ilink;

		mat3_t Jww = obj->data.Jfk_vi_v0[i].sub_matrix(TSubMatrixDim<3,3,3,3>());
		((M3Link*)links[idx++])->SetCoef(R0*Jww);

		for(int j = 0; j < njoint; j++){
			vec3_t Jw = obj->data.Jfk_vi_q[i].col(j).sub_vector(TSubVectorDim<3,3>());
			((C3Link*)links[idx++])->SetCoef(R0*Jw);
		}
	}
	else{
		((M3Link*)links[idx++])->SetCoef(R0);
	}
}

void WholebodyLimitCon::CalcCoef(){
	Prepare();

	int nend = (int)obj->ends.size();
	int idx = 0;

	vec6_t  Je_row = obj->data.Jik_e_v0.row(ierror);
	((R3Link*)links[idx++])->SetCoef(Je_row.sub_vector(TSubVectorDim<3,3>()));

	for(int i = 0; i < nend; i++){
		Je_row = obj->data.Jik_e_ve[i].row(ierror);
		((R3Link*)links[idx++])->SetCoef(Je_row.sub_vector(TSubVectorDim<0,3>()));
		((R3Link*)links[idx++])->SetCoef(Je_row.sub_vector(TSubVectorDim<3,3>()));
	}	
}

/*void WholebodyLdCon::CalcCoef(){
	Prepare();

	int nend = (int)obj->ends.size();

	int idx = 0;

	((M3Link*)links[idx++])->SetCoef(obj->J_Ld_qb);
	((M3Link*)links[idx++])->SetCoef(obj->J_Ld_ub);

	for(int i = 0; i < nend; i++){
		((M3Link*)links[idx++])->SetCoef(obj->J_Ld_pe[i]);
		((M3Link*)links[idx++])->SetCoef(obj->J_Ld_qe[i]);
		((M3Link*)links[idx++])->SetCoef(obj->J_Ld_ae[i]);
		((M3Link*)links[idx++])->SetCoef(obj->J_Ld_ue[i]);
	}
}*/

void WholebodyLCon::CalcCoef(){
	Prepare();

	int nend = (int)obj->ends.size();

	int idx = 0;
	((M3Link*)links[idx++])->SetCoef(obj->J_L_qb);
	((M3Link*)links[idx++])->SetCoef(obj->J_L_wb);

	if(obj->wb->param.parametrization == Wholebody::Parametrization::Joint){
		int njoint = (int)obj->joints.size();
		for(int j = 0; j < njoint; j++)
			((C3Link*)links[idx++])->SetCoef(obj->J_L_q .col(j));
		for(int j = 0; j < njoint; j++)
			((C3Link*)links[idx++])->SetCoef(obj->J_L_qd.col(j));
	}
	else{
		for(int i = 0; i < nend; i++){
			((M3Link*)links[idx++])->SetCoef(obj->J_L_pe[i]);
			((M3Link*)links[idx++])->SetCoef(obj->J_L_qe[i]);
			((M3Link*)links[idx++])->SetCoef(obj->J_L_ve[i]);
			((M3Link*)links[idx++])->SetCoef(obj->J_L_we[i]);
		}
	}

}

void WholebodyContactPosConT::CalcCoef(){
	/*
	y  = qo^T*(pc + q0*(pi + qi*r) - po)
	dy = qo^T*(dpc + Omega0 % q0*(pi + qi*r) + q0*(dpi + Omegai % qi*r))
	   = qo^T*dpc + qo^T (q0*(pi + qi*r)^x)^T Omega0 + (qo^T*q0)*dpi + qo^T q0 ((qi*r)^x)^T Omegai
    */
	Prepare();

	int idx = 0;
	((M3Link*)links[idx++])->SetCoef( Ro.trans());
	((M3Link*)links[idx++])->SetCoef(-Ro.trans()*mat3_t::Cross(q0*(pi + qi*r)));

	if(obj->wb->param.parametrization == Wholebody::Parametrization::Joint){
		int njoint = (int)obj->joints.size();
		int i = obj->wb->ends[iend].ilink;
		for(int j = 0; j < njoint; j++){
			vec3_t Jv = obj->data.Jfk_vi_q[i].col(j).sub_vector(TSubVectorDim<0,3>());
			vec3_t Jw = obj->data.Jfk_vi_q[i].col(j).sub_vector(TSubVectorDim<3,3>());
			((C3Link*)links[idx++])->SetCoef(Ro.trans()*R0*(Jv + Jw % (qi*r)));
		}
	}
	else{
		((M3Link*)links[idx++])->SetCoef( Ro.trans()*R0);
		((M3Link*)links[idx++])->SetCoef(-Ro.trans()*R0*mat3_t::Cross(qi*r));
	}
}

void WholebodyContactPosConR::CalcCoef(){
	Prepare();

	int idx = 0;
	((M3Link*)links[idx++])->SetCoef(Ro.trans());

	if(obj->wb->param.parametrization == Wholebody::Parametrization::Joint){
		int njoint = (int)obj->joints.size();
		int i = obj->wb->ends[iend].ilink;
		for(int j = 0; j < njoint; j++){
			vec3_t Jw = obj->data.Jfk_vi_q[i].col(j).sub_vector(TSubVectorDim<3,3>());
			((C3Link*)links[idx++])->SetCoef(Ro.trans()*R0*Jw);
		}
	}
	else{
		((M3Link*)links[idx++])->SetCoef(Ro.trans()*R0);
	}
}

void WholebodyContactVelConT::CalcCoef(){
	/*
	y = qo^T*(vc + w0 % q0*(pi + qi*r) + q0*(vi + wi % qi*r))
	dy = qo^T*dvc + qo^T*(q0*(pi + qi*r))^x^T * dw0 + qo^T*q0*dvi + qo^T*q0*(qi*r)^x^T * dwi
	*/
	Prepare();

	int idx = 0;
	((M3Link*)links[idx++])->SetCoef( Ro.trans());
	((M3Link*)links[idx++])->SetCoef(-Ro.trans()*mat3_t::Cross(q0*(pi + qi*r)));

	if(obj->wb->param.parametrization == Wholebody::Parametrization::Joint){
		int njoint = (int)obj->joints.size();
		int i = obj->wb->ends[iend].ilink;
		for(int j = 0; j < njoint; j++){
			vec3_t Jv = obj->data.Jfk_vi_q[i].col(j).sub_vector(TSubVectorDim<0,3>());
			vec3_t Jw = obj->data.Jfk_vi_q[i].col(j).sub_vector(TSubVectorDim<3,3>());
			((C3Link*)links[idx++])->SetCoef(Ro.trans()*R0*(Jv + Jw % (qi*r)));
		}
	}
	else{
		((M3Link*)links[idx++])->SetCoef( Ro.trans()*R0);
		((M3Link*)links[idx++])->SetCoef(-Ro.trans()*R0*mat3_t::Cross(qi*r));
	}
}

void WholebodyContactVelConR::CalcCoef(){
	Prepare();

	int idx = 0;
	((M3Link*)links[idx++])->SetCoef(Ro.trans());

	if(obj->wb->param.parametrization == Wholebody::Parametrization::Joint){
		int njoint = (int)obj->joints.size();
		int i = obj->wb->ends[iend].ilink;
		for(int j = 0; j < njoint; j++){
			vec3_t Jw = obj->data.Jfk_vi_q[i].col(j).sub_vector(TSubVectorDim<3,3>());
			((C3Link*)links[idx++])->SetCoef(Ro.trans()*R0*Jw);
		}
	}
	else{
		((M3Link*)links[idx++])->SetCoef(Ro.trans()*R0);
	}
}

void WholebodyNormalForceCon::CalcCoef(){
	Prepare();

	((R3Link*)links[0])->SetCoef(vec3_t(0.0, 0.0, 1.0));
}

void WholebodyFrictionForceCon::CalcCoef(){
	Prepare();

	// -mu*fn <= ft <= mu*fn
	// -ft + mu*fn >= 0
	//  ft + mu*fn >= 0

	((R3Link*)links[0])->SetCoef(
		vec3_t(
			(dir == 0 ? (side == 0 ? -1.0 : 1.0) : 0.0),
			(dir == 1 ? (side == 0 ? -1.0 : 1.0) : 0.0),
			mu));
}

void WholebodyMomentCon::CalcCoef(){
	Prepare();

	// cop_min.x <= -my/fz <= cop_max.x
	// cop_min.y <=  mx/fz <= cop_max.y
	//
	// cop_min.x fz <= -my <= cop_max.x fz
	// cop_min.y fz <=  mx <= cop_max.y fz
	//
	// -my - (cop_min.x)*fz >= 0
	//  my + (cop_max.x)*fz >= 0
	//  mx - (cop_min.y)*fz >= 0
	// -mx + (cop_max.y)*fz >= 0

	((R3Link*)links[0])->SetCoef(
		vec3_t(
			0.0,
			0.0,
			(dir == 0 ? (side == 0 ? -cmin.x : cmax.x) : (side == 0 ? -cmin.y : cmax.y)) ));
	((R3Link*)links[1])->SetCoef(
		vec3_t(
			(dir == 0 ? 0.0 : (side == 0 ? +1.0 : -1.0)),
			(dir == 1 ? 0.0 : (side == 0 ? -1.0 : +1.0)),
			0.0));
}

///////////////////////////////////////////////////////////////////////////////////////////////////

void WholebodyPosConT::CalcDeviation(){
	y = p1 - p_rhs;
}

void WholebodyPosConR::CalcDeviation(){
	quat_t qerror = q_rhs.Conjugated()*q1;
	vec3_t axis   = qerror.Axis ();
	real_t theta  = qerror.Theta();
	if(theta > _pi)
		theta -= 2*_pi;
	y = q_rhs*(theta*axis);
}

void WholebodyVelConT::CalcDeviation(){
	y = v1 - v_rhs;
}

void WholebodyVelConR::CalcDeviation(){
	y = w1 - w_rhs;
}

void WholebodyJointPosCon::CalcDeviation(){
	y[0] = q1 - q_rhs;
}

void WholebodyJointVelCon::CalcDeviation(){
	y[0] = qd1 - qd_rhs;
}

void WholebodyCentroidPosConT::CalcDeviation(){
	y = pc1 - pc_rhs;
}

void WholebodyCentroidVelConT::CalcDeviation(){
	y = vc1 - vc_rhs;
}

void WholebodyCentroidPosConR::CalcDeviation(){
	quat_t qerror = q_rhs.Conjugated()*q1;
	vec3_t axis   = qerror.Axis ();
	real_t theta  = qerror.Theta();
	if(theta > _pi)
		theta -= 2*_pi;
	y = q_rhs*(theta*axis);
}

void WholebodyCentroidVelConR::CalcDeviation(){
	y = w1 - w_rhs;
}

void WholebodyDesPosConT::CalcDeviation(){
	y = (pc + q0*pi) - desired;
}

void WholebodyDesPosConR::CalcDeviation(){
	quat_t qerror = desired.Conjugated()*(q0*qi);
	vec3_t axis   = qerror.Axis ();
	real_t theta  = qerror.Theta();
	if(theta > _pi)
		theta -= 2*_pi;
	y = desired*(theta*axis);
}

void WholebodyDesVelConT::CalcDeviation(){
	y = (vc + w0 % (q0*pi) + q0*vi) - desired;
}

void WholebodyDesVelConR::CalcDeviation(){
	y = (w0 + q0*wi) - desired;
}

void WholebodyLimitCon::CalcDeviation(){
	real_t e = obj->data.e[ierror];

	if(type == Constraint::Type::Equality){
		y[0]   = e;
		active = true;
	}
	if(type == Constraint::Type::InequalityPenalty){
		if(e > 0.0){
			y[0] = 0.0;
			active = false;
		}
		else{
			y[0] = e;
			active = true;
		}
	}
	//DSTR << "limit error: " << y[0] << endl;
}

/*void WholebodyLdCon::CalcDeviation(){
	y = obj->data.centroid.Ld;
}*/

void WholebodyLCon::CalcDeviation(){
	y = obj->data.centroid.L;
}

void WholebodyContactPosConT::CalcDeviation(){
	y = qo.Conjugated()*(pc + q0*(pi + qi*r) - po);
}

void WholebodyContactPosConR::CalcDeviation(){
	quat_t qerror = qo.Conjugated()*(q0*qi);
	vec3_t axis   = qerror.Axis ();
	real_t theta  = qerror.Theta();
	if(theta > _pi)
		theta -= 2*_pi;
	y = theta*axis;
}

void WholebodyContactVelConT::CalcDeviation(){
	y = qo.Conjugated()*(vc + w0 % (q0*(pi + qi*r)) + q0*vi);
}

void WholebodyContactVelConR::CalcDeviation(){
	y = qo.Conjugated()*(w0 + q0*wi);
}

void WholebodyNormalForceCon::CalcDeviation(){
	if(fn > 0.0){
		y[0] = 0.0;
		active = false;
	}
	else{
		y[0] = fn;
		active = true;
	}
}

void WholebodyFrictionForceCon::CalcDeviation(){
	real_t e = (side == 0 ? mu*fn - ft : ft + mu*fn);
	if(e > 0.0){
		y[0] = 0.0;
		active = false;
	}
	else{
		y[0] = e;
		active = true;
	}		
}

void WholebodyMomentCon::CalcDeviation(){
	// -ty*m - (cop_min.x)*fn >= 0
	//  ty*m + (cop_max.x)*fn >= 0
	//  tx*m - (cop_min.y)*fn >= 0
	// -tx*m + (cop_max.y)*fn >= 0
	real_t e = (dir == 0 ? 
		(side == 0 ? (-m.y - cmin.x*fn) : ( m.y + cmax.x*fn)) :
		(side == 0 ? ( m.x - cmin.y*fn) : (-m.x + cmax.y*fn))
		);

	// always active in barrier mode
	//y[0] = std::max(1.0, e);

	if(e > 0.0){
		y[0] = 0.0;
		active = false;
	}
	else{
		y[0] = e;
		active = true;
	}
	
}

}
