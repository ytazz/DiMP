#include <DiMP/Graph/Wholebody.h>
#include <DiMP/Graph/Graph.h>
#include <DiMP/Render/Config.h>
#include <DiMP/Render/Canvas.h>

namespace DiMP {;

const real_t pi      = M_PI;
const real_t inf     = numeric_limits<real_t>::max();
const real_t damping = 0.1;
const vec3_t one(1.0, 1.0, 1.0);

//-------------------------------------------------------------------------------------------------
// IKData

void WholebodyData::Init(Wholebody* wb){
	links.resize(wb->links .size());
	ends .resize(wb->ends  .size());
	q    .resize(wb->joints.size());
	e    .resize(wb->limits.size());
}

//-------------------------------------------------------------------------------------------------
// WholebodyKey

WholebodyKey::WholebodyKey() {
	
}

void WholebodyKey::AddVar(Solver* solver) {
	wb = (Wholebody*)node;

	int nend  = wb->ends .size();
	int nlink = wb->links.size();
	
	ends.resize(nend);
	stringstream ss, ss2;
	for(int i = 0; i < nend; i++){
		ss.str("");
		ss << name << "_end" << i;
		
		ends[i].var_pos_t = new V3Var(solver, ID(VarTag::WholebodyPosT, node, tick, ss.str() + "_pos_t"), wb->spt);
		ends[i].var_pos_r = new QVar (solver, ID(VarTag::WholebodyPosR, node, tick, ss.str() + "_pos_r"), wb->spr);
		ends[i].var_vel_t = new V3Var(solver, ID(VarTag::WholebodyVelT, node, tick, ss.str() + "_vel_t"), wb->svt);
		ends[i].var_vel_r = new V3Var(solver, ID(VarTag::WholebodyVelR, node, tick, ss.str() + "_vel_r"), wb->svr);
		ends[i].var_pos_t->weight = damping*one;
		ends[i].var_pos_r->weight = damping*one;
		ends[i].var_vel_t->weight = damping*one;
		ends[i].var_vel_r->weight = damping*one;
        solver->AddStateVar(ends[i].var_pos_t, tick->idx);
        solver->AddStateVar(ends[i].var_pos_r, tick->idx);
        solver->AddStateVar(ends[i].var_vel_t, tick->idx);
        solver->AddStateVar(ends[i].var_vel_r, tick->idx);
		
		if(next){
			for(int j = 0; j < 6; j++){
				ss2.str("");
				ss << ss.str() << "_compl" << j;
				// scale is set dynamically 
				ends[i].var_compl[j] = new SVar(solver, ID(VarTag::WholebodyCompl, node, tick, ss2.str()), 1.0);
				ends[i].var_compl[j]->weight[0] = damping;
				solver->AddInputVar(ends[i].var_compl[j], tick->idx);
			}
		}
	}

	var_com_pos = new V3Var(solver, ID(VarTag::WholebodyComPos  , node, tick, ss.str() + "_com_pos"), wb->spt);
	var_com_vel = new V3Var(solver, ID(VarTag::WholebodyComVel  , node, tick, ss.str() + "_com_vel"), wb->svt);
	var_mom     = new V3Var(solver, ID(VarTag::WholebodyMomentum, node, tick, ss.str() + "_mom"    ), wb->sL);
	var_com_pos->weight = damping*one;
	var_com_vel->weight = damping*one;
	var_mom    ->weight = damping*one;
	solver->AddStateVar(var_com_pos, tick->idx);
	solver->AddStateVar(var_com_vel, tick->idx);
	solver->AddStateVar(var_mom    , tick->idx);

	data      .Init(wb);
	data_plus .Init(wb);
	data_minus.Init(wb);
	data_tmp  .Init(wb);

	J_pcom_pe.resize(nend);
	J_pcom_qe.resize(nend);
	J_pcom_ve.resize(nend);
	J_pcom_we.resize(nend);
	J_vcom_pe.resize(nend);
	J_vcom_qe.resize(nend);
	J_vcom_ve.resize(nend);
	J_vcom_we.resize(nend);
	J_L_pe   .resize(nend);
	J_L_qe   .resize(nend);
	J_L_ve   .resize(nend);
	J_L_we   .resize(nend);
	J_q_pe   .resize(nend);
	J_q_qe   .resize(nend);
	J_q_ve   .resize(nend);
	J_q_we   .resize(nend);
	J_e_pe   .resize(nend);
	J_e_qe   .resize(nend);
	J_e_ve   .resize(nend);
	J_e_we   .resize(nend);

	int njoint = data.q.size();
	int nlimit = data.e.size();
	
	for(int i = 0; i < nend; i++){
		J_q_pe[i].resize(njoint, 3);
		J_q_qe[i].resize(njoint, 3);
		J_q_ve[i].resize(njoint, 3);
		J_q_we[i].resize(njoint, 3);
		J_e_pe[i].resize(nlimit, 3);
		J_e_qe[i].resize(nlimit, 3);
		J_e_ve[i].resize(nlimit, 3);
		J_e_we[i].resize(nlimit, 3);
	}
}

void WholebodyKey::AddCon(Solver* solver) {
	WholebodyKey* nextObj = (WholebodyKey*)next;

    int nend   = wb->ends  .size();
	int nchain = wb->chains.size();
	int nlimit = wb->limits.size();
    
	stringstream ss, ss2;

	for(int i = 0; i < nend; i++){
		ss.str("");
		ss << name << "_end" << i;

		if(next){
			ends[i].con_pos_t = new WholebodyPosConT(solver, name + "_pos_t", this, i, wb->spt);
			ends[i].con_pos_r = new WholebodyPosConR(solver, name + "_pos_r", this, i, wb->spr);
			ends[i].con_vel_t = new WholebodyVelConT(solver, name + "_vel_t", this, i, wb->svt);
			ends[i].con_vel_r = new WholebodyVelConR(solver, name + "_vel_r", this, i, wb->svr);
        
			solver->AddTransitionCon(ends[i].con_pos_t, tick->idx);
			solver->AddTransitionCon(ends[i].con_pos_r, tick->idx);
			solver->AddTransitionCon(ends[i].con_vel_t, tick->idx);
			solver->AddTransitionCon(ends[i].con_vel_r, tick->idx);
		}

		ends[i].con_des_pos_t = new FixConV3(solver, ID(ConTag::WholebodyPosT, node, tick, ss.str() + "_des_pos_t"), ends[i].var_pos_t, wb->spt);
		ends[i].con_des_pos_r = new FixConQ (solver, ID(ConTag::WholebodyPosR, node, tick, ss.str() + "_des_pos_r"), ends[i].var_pos_r, wb->spr);
		ends[i].con_des_vel_t = new FixConV3(solver, ID(ConTag::WholebodyVelT, node, tick, ss.str() + "_des_vel_t"), ends[i].var_vel_t, wb->svt);
		ends[i].con_des_vel_r = new FixConV3(solver, ID(ConTag::WholebodyVelR, node, tick, ss.str() + "_des_vel_r"), ends[i].var_vel_r, wb->svr);
		solver->AddCostCon(ends[i].con_des_pos_t, tick->idx);
		solver->AddCostCon(ends[i].con_des_pos_r, tick->idx);
		solver->AddCostCon(ends[i].con_des_vel_t, tick->idx);
		solver->AddCostCon(ends[i].con_des_vel_r, tick->idx);

		if(next){
			for(int j = 0; j < 6; j++){
				ss2.str("");
				ss << ss.str() << "_compl" << j;
				ends[i].con_des_compl[j] = new FixConS(solver, ID(ConTag::WholebodyCompl, node, tick, ss2.str()), ends[i].var_compl[j], 1.0);
			}
		
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
	}
	
	con_limit.resize(nlimit);
	for(int j = 0; j < nlimit; j++){
		ss.str("");
		ss << name << "_lim" << j;
			
		con_limit[j] = new WholebodyLimitCon(solver, ss.str(), this, j, wb->limits[j].type, wb->limits[j].scale);
		solver->AddCostCon(con_limit[j], tick->idx);
	}
	
	con_com_pos_match = new WholebodyComPosMatchCon(solver, name + "_com_pos_match", this, wb->spt);
	solver->AddCostCon(con_com_pos_match, tick->idx);

	con_des_com_pos = new FixConV3(solver, ID(ConTag::WholebodyComPos, node, tick, name + "_des_com_pos"), var_com_pos, wb->spt);
	solver->AddCostCon(con_des_com_pos, tick->idx);

	if(next){
		con_com_pos      = new WholebodyComPosCon     (solver, name + "_com_pos"     , this, wb->spt);
		con_total_force  = new WholebodyTotalForceCon (solver, name + "_total_force" , this, wb->svt);
		con_total_moment = new WholebodyTotalMomentCon(solver, name + "_total_moment", this, wb->sL);
		solver->AddTransitionCon(con_com_pos     , tick->idx);
		solver->AddTransitionCon(con_total_force , tick->idx);
		solver->AddTransitionCon(con_total_moment, tick->idx);

		con_com_vel_match = new WholebodyComVelMatchCon  (solver, name + "_com_vel_match", this, wb->svt);
		con_mom_match     = new WholebodyMomentumMatchCon(solver, name + "_mom_match"    , this, wb->sL);
		solver->AddCostCon(con_com_vel_match, tick->idx);
		solver->AddCostCon(con_mom_match    , tick->idx);

		con_des_com_vel = new FixConV3(solver, ID(ConTag::WholebodyComVel  , node, tick, name + "_des_com_vel"), var_com_vel, wb->svt);
		con_des_mom     = new FixConV3(solver, ID(ConTag::WholebodyMomentum, node, tick, name + "_des_mom"    ), var_mom    , wb->sL);
		solver->AddCostCon(con_des_com_vel, tick->idx);
		solver->AddCostCon(con_des_mom    , tick->idx);
	}
}

void WholebodyKey::CalcIK(WholebodyData& _data, bool calc_acc, bool calc_force){
	int nend  = wb->ends .size();
	int nlink = wb->links.size();

	wb->callback->CalcIK(_data);
	
	const real_t dt     = 0.01;
	const real_t dtinv  = 1.0/dt;
	const real_t dt2    = dt*dt;
	
	data_plus  = _data;
	if(calc_acc)
		data_minus = _data;

	for(int i = 0; i < nend; i++){
		WholebodyData::End&  dend       = _data.ends[i];
		WholebodyData::Link& dlnk       = _data     .links[wb->ends[i].ilink];
		WholebodyData::Link& dlnk_plus  = data_plus .links[wb->ends[i].ilink];
		WholebodyData::Link& dlnk_minus = data_minus.links[wb->ends[i].ilink];

		dlnk_plus.pos_t = dlnk.pos_t + dlnk.vel_t*dt + dlnk.acc_t*(0.5*dt2);
		dlnk_plus.pos_r = quat_t::Rot( dlnk.vel_r*dt + dlnk.acc_r*(0.5*dt2))*dlnk.pos_r;

		if(calc_acc){
			dlnk_minus.pos_t = dlnk.pos_t - dlnk.vel_t*dt + dlnk.acc_t*(0.5*dt2);
			dlnk_minus.pos_r = quat_t::Rot(-dlnk.vel_r*dt + dlnk.acc_r*(0.5*dt2))*dlnk.pos_r;
		}
	}

	wb->callback->CalcIK(data_plus );
	if(calc_acc)
		wb->callback->CalcIK(data_minus);

	for(int i = 0; i < nlink; i++){
		WholebodyData::Link& dlnk       = _data     .links[i];
		WholebodyData::Link& dlnk_plus  = data_plus .links[i];
		WholebodyData::Link& dlnk_minus = data_minus.links[i];

		if(!wb->links[i].is_end){
			dlnk.vel_t = (dlnk_plus.pos_t - dlnk.pos_t)*dtinv;
						
			quat_t qdiff = dlnk_plus.pos_r*dlnk.pos_r.Conjugated();
			dlnk.vel_r = qdiff.Axis()*(qdiff.Theta()*dtinv);

			if(calc_acc){
				dlnk_minus.vel_t = (dlnk.pos_t - dlnk_minus.pos_t)*dtinv;

				quat_t qdiff = dlnk.pos_r*dlnk_minus.pos_r.Conjugated();
				dlnk_minus.vel_r = qdiff.Axis()*(qdiff.Theta()*dtinv);

				dlnk.acc_t = (dlnk.vel_t - dlnk_minus.vel_t)*dtinv;
				dlnk.acc_r = (dlnk.vel_r - dlnk_minus.vel_r)*dtinv;
			}
		}
	}

	_data.com_pos.clear();
	_data.com_vel.clear();
	for(int i = 0; i < nlink; i++){
		WholebodyData::Link& dlnk  = _data.links[i];

		_data.com_pos += wb->links[i].mass_ratio*dlnk.pos_t;
		_data.com_vel += wb->links[i].mass_ratio*dlnk.vel_t;
	}
	
	_data.mom.clear();
	for(int i = 0; i < nlink; i++){
		WholebodyData::Link& dlnk  = _data.links[i];

		_data.mom += (dlnk.pos_t - _data.com_pos) % (wb->links[i].mass*dlnk.vel_t);
	}

	if(calc_force){
		for(int i : wb->idOrder){
			WholebodyData::Link& dlnk  = _data.links[i];
			
			vec3_t force_t_child;
			vec3_t force_r_child;
			for(int ic : wb->links[i].ichildren){
				vec3_t r = _data.links[ic].pos_t - dlnk.pos_t;
				force_t_child -=  _data.links[ic].force_t_par;
				force_r_child -= (_data.links[ic].force_r_par + r % _data.links[ic].force_t_par);
			}
					
			dlnk.force_t_par = wb->links[i].mass*(dlnk.acc_t + vec3_t(0.0, 0.0, wb->param.gravity)) - dlnk.force_t - force_t_child;
			dlnk.force_r_par = -dlnk.force_r - force_r_child;
				
			// calc joint torque
			_data.tau[wb->links[i].ijoint] = wb->links[i].axis*dlnk.force_r_par;
		}
	}
}

void WholebodyKey::Prepare() {
	int nlink = wb->links.size();
	int nend  = wb->ends .size();

	// calc IK
	
	for(int i = 0; i < nend; i++){
		End& end = ends[i];

		WholebodyData::End & dend = data.ends [i];
		WholebodyData::Link& dlnk = data.links[wb->ends[i].ilink];
		
		dlnk.pos_t = end.var_pos_t->val;
		dlnk.pos_r = end.var_pos_r->val;
		dlnk.vel_t = end.var_vel_t->val;
		dlnk.vel_r = end.var_vel_r->val;

		// update complimentarity

		if(next){
			dend.v_or_f[0] = ( dend.state == Wholebody::ContactState::Free );
			dend.v_or_f[1] = ( dend.state == Wholebody::ContactState::Free );
			dend.v_or_f[2] = ( dend.state == Wholebody::ContactState::Free );
			dend.v_or_f[3] = ( dend.state == Wholebody::ContactState::Free ||
		     		           dend.state == Wholebody::ContactState::Point );
			dend.v_or_f[4] = ( dend.state == Wholebody::ContactState::Free ||
				               dend.state == Wholebody::ContactState::Line ||
				               dend.state == Wholebody::ContactState::Point );
			dend.v_or_f[5] = ( dend.state == Wholebody::ContactState::Free ||
					           dend.state == Wholebody::ContactState::Point );

			// set scale
			for(int j = 0; j < 3; j++){
				end.var_compl    [j]->scale = (dend.v_or_f[j] ? wb->sat : wb->sft);
				end.con_des_compl[j]->scale = (dend.v_or_f[j] ? wb->sat : wb->sft);
			}
			for(int j = 3; j < 6; j++){
				end.var_compl    [j]->scale = (dend.v_or_f[j] ? wb->sar : wb->sfr);
				end.con_des_compl[j]->scale = (dend.v_or_f[j] ? wb->sar : wb->sfr);
			}

			dend.acc_tc  .clear();
			dend.acc_rc  .clear();
			dend.force_tc.clear();
			dend.force_rc.clear();

			(dend.v_or_f[0] ? dend.acc_tc.x : dend.force_tc.x) = end.var_compl[0]->val;
			(dend.v_or_f[1] ? dend.acc_tc.y : dend.force_tc.y) = end.var_compl[1]->val;
			(dend.v_or_f[2] ? dend.acc_tc.z : dend.force_tc.z) = end.var_compl[2]->val;
			(dend.v_or_f[3] ? dend.acc_rc.x : dend.force_rc.x) = end.var_compl[3]->val;
			(dend.v_or_f[4] ? dend.acc_rc.y : dend.force_rc.y) = end.var_compl[4]->val;
			(dend.v_or_f[5] ? dend.acc_rc.z : dend.force_rc.z) = end.var_compl[5]->val;
	
			// transform contact velocity/force to link velocity/force
	
			// wl = qc*wc
			// vl = qc*vc - qc*wc % rlc
			//    = qc*vc - wl % rlc
		
			// ul = qc*uc
			// al = qc*ac - qc*uc % rlc - qc*wc % (qc*vc - vl)
			//    = qc*ac - ul % rlc - wl % (wl % rlc)

			dlnk.acc_r = dend.pos_rc*dend.acc_rc;
			dlnk.acc_t = dend.pos_rc*dend.acc_tc - dlnk.acc_r % dend.rlc - dlnk.vel_r % (dlnk.vel_r % dend.rlc);

			dlnk.force_t = dend.pos_rc*dend.force_tc;
			dlnk.force_r = dend.pos_rc*dend.force_rc + dend.rlc % dend.force_tc;
		}		

		if(i == 4){
			DSTR << "k: " << tick->idx << " pos: " << dlnk.pos_t << " vel: " << dlnk.vel_t << " acc: " << dlnk.acc_t << endl;
		}
	}

	CalcIK(data, (next ? true : false), (next ? true : false));

	for(int i = 0; i < nend; i++){
		WholebodyData::End&  dend = data.ends[i];
		WholebodyData::Link& dlnk = data.links[wb->ends[i].ilink];

		dend.pos_rc.ToMatrix(dend.Rc);
		dend.rlc = dend.pos_tc - dlnk.pos_t;
		dend.rcc = dend.pos_tc - data.com_pos;
		
		dend.rlc_cross_Rc = mat3_t::Cross(dend.rlc)*dend.Rc;
		dend.rcc_cross_Rc = mat3_t::Cross(dend.rcc)*dend.Rc;
	}
}

void WholebodyKey::PrepareStep(){
	// calc Jacobian
	
	int nend  = wb->ends .size();

	data_tmp = data;

	const real_t eps    = 1.0e-2;
	const real_t epsinv = 1.0/eps;

	for(int j = 0; j < 12*nend; j++){
		int i0    = j % 12;
		int i1    = i0 % 3;
		int iend  = (j - i0)/12;
		int ilink = wb->ends[iend].ilink;

		WholebodyData::End & dend_tmp = data_tmp.ends [iend ];
		WholebodyData::Link& dlnk_tmp = data_tmp.links[ilink];

		if(i0 < 3){
			dlnk_tmp.pos_t[i1] += eps;
		}
		else if(i0 < 6){
			vec3_t axis;
			axis[i1] = 1.0;
			dlnk_tmp.pos_r = dlnk_tmp.pos_r*quat_t::Rot(eps, axis);
		}
		else if(i0 < 9){
			dlnk_tmp.vel_t[i1] += eps;
		}
		else if(i0 < 12){
			dlnk_tmp.vel_r[i1] += eps;
		}

		CalcIK(data_tmp, false, false);

		if(i0 < 3){
			J_pcom_pe[iend].col(i1) = (data_tmp.com_pos - data.com_pos)*epsinv;
			J_vcom_pe[iend].col(i1) = (data_tmp.com_vel - data.com_vel)*epsinv;
			J_L_pe   [iend].col(i1) = (data_tmp.mom     - data.mom    )*epsinv;
			J_q_pe   [iend].col(i1) = (data_tmp.q       - data.q      )*epsinv;
			J_e_pe   [iend].col(i1) = (data_tmp.e       - data.e      )*epsinv;
		}
		else if(i0 < 6){
			J_pcom_qe[iend].col(i1) = (data_tmp.com_pos - data.com_pos)*epsinv;
			J_vcom_qe[iend].col(i1) = (data_tmp.com_vel - data.com_vel)*epsinv;
			J_L_qe   [iend].col(i1) = (data_tmp.mom     - data.mom    )*epsinv;
			J_q_qe   [iend].col(i1) = (data_tmp.q       - data.q      )*epsinv;
			J_e_qe   [iend].col(i1) = (data_tmp.e       - data.e      )*epsinv;
		}
		else if(i0 < 9){
			J_pcom_ve[iend].col(i1).clear();
			J_vcom_ve[iend].col(i1) = (data_tmp.com_vel - data.com_vel)*epsinv;
			J_L_ve   [iend].col(i1) = (data_tmp.mom     - data.mom    )*epsinv;
			J_q_ve   [iend].col(i1).clear();
			J_e_ve   [iend].col(i1).clear();
		}
		else if(i0 < 12){
			J_pcom_we[iend].col(i1).clear();
			J_vcom_we[iend].col(i1) = (data_tmp.com_vel - data.com_vel)*epsinv;
			J_L_we   [iend].col(i1) = (data_tmp.mom     - data.mom    )*epsinv;
			J_q_we   [iend].col(i1).clear();
			J_e_we   [iend].col(i1).clear();
		}

		data_tmp = data;
	}

}

void WholebodyKey::Finish(){

}

void WholebodyKey::Draw(Render::Canvas* canvas, Render::Config* conf) {

}

//-------------------------------------------------------------------------------------------------
// Wholebody

Wholebody::Param::Param() {
	gravity = 9.8;
}

//-------------------------------------------------------------------------------------------------

Wholebody::Link::Link(real_t _mass, int _iparent, int _ijoint, vec3_t _axis){
	mass    = _mass;
	iparent = _iparent;
	ijoint  = _ijoint;
	axis    = _axis;
}

//-------------------------------------------------------------------------------------------------

Wholebody::End::End(){
	ilink = 0;
}

//-------------------------------------------------------------------------------------------------

Wholebody::Limit::Limit(int _ichain, int _type, real_t _scale){
	ichain = _ichain;
	type   = _type;
	scale  = _scale;
}

//-------------------------------------------------------------------------------------------------

Wholebody::Chain::Chain(int _ibase, int _iend, const vector<int>& _ilink, const vector<int>& _ilimit){
	ibase  = _ibase;
	iend   = _iend;
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
	param.total_mass = 0.0;
	for(int i = 0; i < links.size(); i++){
		param.total_mass += links[i].mass;
	}
	for(int i = 0; i < links.size(); i++){
		links[i].mass_ratio = links[i].mass/param.total_mass;
	}

	sl  = 1.0;  //< unit length
	st  = graph->ticks[1]->time - graph->ticks[0]->time;
	sat = param.gravity;
	svt = sat*st;
	sft = param.total_mass*param.gravity;
	spt = svt*st;
	spr = spt/sl;
	svr = svt/sl;
	sar = sat/sl;
	sfr = sft*sl;
	sL  = sfr*st;
}

void Wholebody::Setup(){
	int nend = ends.size();

	for (int k = 0; k < graph->ticks.size(); k++) {
		WholebodyKey* key = (WholebodyKey*)traj.GetKeypoint(graph->ticks[k]);

		real_t t = graph->ticks[k]->time;

		callback->Setup(t, key->data);

		for(int i = 0; i < nend; i++){
			WholebodyKey ::End&  end  = key->ends[i];
			WholebodyData::End&  dend = key->data.ends [i];
			
			if(k == 0){
				end.var_pos_t->val = dend.pos_t_ini;
				end.var_pos_r->val = dend.pos_r_ini;
				end.var_vel_t->val = dend.vel_t_ini;
				end.var_vel_r->val = dend.vel_r_ini;
				//end.var_pos_t->locked = true;
				//end.var_pos_r->locked = true;
				//end.var_vel_t->locked = true;
				//end.var_vel_r->locked = true;
				end.con_des_pos_t->desired = dend.pos_t_ini;
				end.con_des_pos_r->desired = dend.pos_r_ini;
				end.con_des_vel_t->desired = dend.vel_t_ini;
				end.con_des_vel_r->desired = dend.vel_r_ini;
				end.con_des_pos_t->weight = 100.0*one;
				end.con_des_pos_r->weight = 100.0*one;
				end.con_des_vel_t->weight = 100.0*one;
				end.con_des_vel_r->weight = 100.0*one;
				//end.con_des_pos_t->enabled = false;
				//end.con_des_pos_r->enabled = false;
				//end.con_des_vel_t->enabled = false;
				//end.con_des_vel_r->enabled = false;

				end.var_compl[0]->val = end.con_des_compl[0]->desired = (dend.v_or_f[0] ? dend.acc_tc_ini.x : dend.force_tc_ini.x);
				end.var_compl[1]->val = end.con_des_compl[1]->desired = (dend.v_or_f[1] ? dend.acc_tc_ini.y : dend.force_tc_ini.y);
				end.var_compl[2]->val = end.con_des_compl[2]->desired = (dend.v_or_f[2] ? dend.acc_tc_ini.z : dend.force_tc_ini.z);
				end.var_compl[3]->val = end.con_des_compl[3]->desired = (dend.v_or_f[3] ? dend.acc_rc_ini.x : dend.force_rc_ini.x);
				end.var_compl[4]->val = end.con_des_compl[4]->desired = (dend.v_or_f[4] ? dend.acc_rc_ini.y : dend.force_rc_ini.y);
				end.var_compl[5]->val = end.con_des_compl[5]->desired = (dend.v_or_f[5] ? dend.acc_rc_ini.z : dend.force_rc_ini.z);
				
				for(int j = 0; j < 6; j++){
					//end.var_compl[j]->locked = true;
					//end.con_des_compl[j]->enabled = false;
					end.con_des_compl[j]->weight[0] = 100.0;
				}
			}
			else{
				end.var_pos_t->val = dend.pos_t_des;
				end.var_pos_r->val = dend.pos_r_des;
				end.var_vel_t->val = dend.vel_t_des;
				end.var_vel_r->val = dend.vel_r_des;
				end.con_des_pos_t->desired = dend.pos_t_des;
				end.con_des_pos_r->desired = dend.pos_r_des;
				end.con_des_vel_t->desired = dend.vel_t_des;
				end.con_des_vel_r->desired = dend.vel_r_des;
				end.con_des_pos_t->weight = (dend.state == Wholebody::ContactState::Free ? 0.1 : 1.0)*one;
				end.con_des_pos_r->weight = (dend.state == Wholebody::ContactState::Free ? 0.1 : 1.0)*one;
				end.con_des_vel_t->weight = (dend.state == Wholebody::ContactState::Free ? 0.1 : 1.0)*one;
				end.con_des_vel_r->weight = (dend.state == Wholebody::ContactState::Free ? 0.1 : 1.0)*one;
			}
			
			if(key->next){				
				end.var_compl[0]->val = end.con_des_compl[0]->desired = (dend.v_or_f[0] ? dend.acc_tc_des.x : dend.force_tc_des.x);
				end.var_compl[1]->val = end.con_des_compl[1]->desired = (dend.v_or_f[1] ? dend.acc_tc_des.y : dend.force_tc_des.y);
				end.var_compl[2]->val = end.con_des_compl[2]->desired = (dend.v_or_f[2] ? dend.acc_tc_des.z : dend.force_tc_des.z);
				end.var_compl[3]->val = end.con_des_compl[3]->desired = (dend.v_or_f[3] ? dend.acc_rc_des.x : dend.force_rc_des.x);
				end.var_compl[4]->val = end.con_des_compl[4]->desired = (dend.v_or_f[4] ? dend.acc_rc_des.y : dend.force_rc_des.y);
				end.var_compl[5]->val = end.con_des_compl[5]->desired = (dend.v_or_f[5] ? dend.acc_rc_des.z : dend.force_rc_des.z);
			
				for(int j = 0; j < 6; j++){
					end.con_des_compl[j]->weight[0] = 0.1;
				}
			}
		}

		if(k == 0){
			// disable kinematic limit
			for(auto& con : key->con_limit)
				con->enabled = false;

			key->con_com_pos_match->enabled = false;
			key->con_mom_match    ->enabled = false;
		}

		key->var_com_pos->val = key->data.com_pos_des;
		
		key->con_des_com_pos->desired = key->data.com_pos_des;
		key->con_des_com_pos->weight  = 0.1*one;

		key->con_com_pos_match->weight = 1.0*one;
		  
		if(key->next){
			key->var_com_vel->val = key->data.com_vel_des;
			key->var_mom    ->val = key->data.mom_des;
			
			key->con_des_com_vel->desired = key->data.com_vel_des;
			key->con_des_mom    ->desired = key->data.mom_des;
			key->con_des_com_vel->weight = 0.1*one;
			key->con_des_mom    ->weight = 0.1*one;

			key->con_com_vel_match->weight = 1.0*one;
			key->con_mom_match    ->weight = 0.1*one;
		}
	}

	trajReady = false;
}

void Wholebody::Init() {
	TrajectoryNode::Init();

	int nlink  = links .size();
	int nend   = ends  .size();
	int nchain = chains.size();
	
	for(int i = 0; i < nlink; i++){
		links[i].is_end = false;
	}
	for(int i = 0; i < nend; i++){
		links[ends[i].ilink].is_end = true;
	}
	//for(int i = 0; i < nchain; i++){
	//	for(int j : chains[i].ilimit)
	//		limits[j].ichain = i;
	//}

	//int olink  = 0;
	//int ojoint = 0;
	//int olimit = 0;
	//for(int i = 0; i < chains.size(); i++){
	//	chains[i].link_offset  = olink ;
	//	chains[i].joint_offset = ojoint;
	//	chains[i].limit_offset = olimit;
	//	
	//	olink  += chains[i].num_links ;
	//	ojoint += chains[i].num_joints;
	//	olimit += chains[i].limits.size();
	//}

    // call prepare here so that initial trajectory is visualized properly
	Setup  ();
    Prepare();

    trajReady = false;
}

void Wholebody::Prepare() {
	trajReady = false;

	TrajectoryNode::Prepare();
}

void Wholebody::Finish(){
	TrajectoryNode::Finish();
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
		    k0->tick->time, k0->data.com_pos, k0->data.com_vel,
		    k1->tick->time, k1->data.com_pos, k1->data.com_vel,
		    Interpolate::LinearDiff);
        vel = InterpolateVel(
		    t,
		    k0->tick->time, k0->data.com_pos, k0->data.com_vel,
		    k1->tick->time, k1->data.com_pos, k1->data.com_vel,
		    Interpolate::LinearDiff);
    }
    else{
        pos = k0->data.com_pos;
		vel = k0->data.com_vel;
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
		    Interpolate::LinearDiff);
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
			canvas->Point(Vec3f(trajectory[0].links[i].pos_t));
			canvas->BeginPath();
			canvas->MoveTo(trajectory[0].links[i].pos_t);
			for (int k = 1; k < trajectory.size(); k++) {
				canvas->LineTo(trajectory[k].links[i].pos_t);
			}
			canvas->EndPath();
			canvas->EndLayer();
		}
	}
}

void Wholebody::CreateSnapshot(real_t t, Wholebody::Snapshot& s){
	s.t = t;
    
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
		for(int i = 0; i < links.size(); i++){
			canvas->BeginLayer("wholebody_link_snapshot", true);
			canvas->SetLineColor(i % 2 == 0 ? "cyan" : "blue");

			int ipar = links[i].iparent;
			if(ipar != -1){
				// line connecting each link and its parent
				canvas->BeginPath();
				canvas->MoveTo(snapshot.links[ipar].pos_t);
				canvas->LineTo(snapshot.links[i   ].pos_t);
				canvas->EndPath();
			}

            // line indicating force
			canvas->SetLineColor("magenta");
			canvas->BeginPath();
			canvas->MoveTo(snapshot.links[i].pos_t);
			canvas->LineTo(snapshot.links[i].pos_t + 0.001*snapshot.links[i].force_t);
			canvas->EndPath();
			canvas->EndLayer();
		}

		for(int i = 0; i < ends.size(); i++){
			int ilink = ends[i].ilink;
			canvas->SetPointSize(5.0);
			canvas->Point(Vec3f(snapshot.links[ilink].pos_t));
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
}

WholebodyPosConR::WholebodyPosConR(Solver* solver, string _name, WholebodyKey* _obj, int _iend, real_t _scale):
	WholebodyCon(solver, 3, ConTag::WholebodyPosR, _name, _obj, _scale) {

	iend = _iend;

	AddSLink(obj[1]->ends[iend].var_pos_r);
	AddSLink(obj[0]->ends[iend].var_pos_r);
	AddSLink(obj[0]->ends[iend].var_vel_r);
}

WholebodyVelConT::WholebodyVelConT(Solver* solver, string _name, WholebodyKey* _obj, int _iend, real_t _scale):
	WholebodyCon(solver, 3, ConTag::WholebodyVelT, _name, _obj, _scale) {

	iend = _iend;

	AddSLink(obj[1]->ends[iend].var_vel_t);
	AddSLink(obj[0]->ends[iend].var_vel_t);
	for(int j = 0; j < 6; j++){
		AddC3Link(obj[0]->ends[iend].var_compl[j]);
	}
}

WholebodyVelConR::WholebodyVelConR(Solver* solver, string _name, WholebodyKey* _obj, int _iend, real_t _scale):
	WholebodyCon(solver, 3, ConTag::WholebodyVelR, _name, _obj, _scale) {

	iend = _iend;

	AddSLink(obj[1]->ends[iend].var_vel_r);
	AddSLink(obj[0]->ends[iend].var_vel_r);
	for(int j = 3; j < 6; j++){
		AddC3Link(obj[0]->ends[iend].var_compl[j]);
	}
}

WholebodyComPosCon::WholebodyComPosCon(Solver* solver, string _name, WholebodyKey* _obj, real_t _scale):
	WholebodyCon(solver, 3, ConTag::WholebodyComPos, _name, _obj, _scale) {

	AddSLink(obj[1]->var_com_pos);
	AddSLink(obj[0]->var_com_pos);
	AddSLink(obj[0]->var_com_vel);
}

WholebodyTotalForceCon::WholebodyTotalForceCon(Solver* solver, string _name, WholebodyKey* _obj, real_t _scale):
	WholebodyCon(solver, 3, ConTag::WholebodyTotalForce, _name, _obj, _scale){
	
	AddSLink(obj[1]->var_com_vel);
	AddSLink(obj[0]->var_com_vel);

	int nend = obj[0]->ends.size();
	for(int i = 0; i < nend; i++){
		for(int j = 0; j < 3; j++){
			AddC3Link(obj[0]->ends[i].var_compl[j]);
		}
	}
    
}

WholebodyTotalMomentCon::WholebodyTotalMomentCon(Solver* solver, string _name, WholebodyKey* _obj, real_t _scale):
	WholebodyCon(solver, 3, ConTag::WholebodyTotalMoment, _name, _obj, _scale){
	
	AddSLink(obj[1]->var_mom);
	AddSLink(obj[0]->var_mom);

	int nend = obj[0]->ends.size();
	for(int i = 0; i < nend; i++){
		for(int j = 0; j < 6; j++){
			AddC3Link(obj[0]->ends[i].var_compl[j]);
		}
	}
    
}

WholebodyLimitCon::WholebodyLimitCon(Solver* solver, string _name, WholebodyKey* _obj, int _idx, int _type, real_t _scale):
	Constraint(solver, 1, ID(ConTag::WholebodyLimit, _obj->node, _obj->tick, _name), _type, _scale){
	obj = _obj;
	idx = _idx;
    
	Wholebody::Limit& lim = obj->wb->limits[idx];
	Wholebody::Chain& ch  = obj->wb->chains[lim.ichain];

	AddR3Link(obj->ends[ch.ibase].var_pos_t);
	AddR3Link(obj->ends[ch.ibase].var_pos_r);
	AddR3Link(obj->ends[ch.iend ].var_pos_t);
	AddR3Link(obj->ends[ch.iend ].var_pos_r);
}

WholebodyNormalForceCon::WholebodyNormalForceCon(Solver* solver, string _name, WholebodyKey* _obj, int _iend, real_t _scale):
	Constraint(solver, 1, ID(ConTag::WholebodyNormalForce, _obj->node, _obj->tick, _name), Constraint::Type::InequalityPenalty, _scale){
	obj  = _obj;
	iend = _iend;
    
	AddSLink(obj->ends[iend].var_compl[2]);

}

WholebodyFrictionForceCon::WholebodyFrictionForceCon(Solver* solver, string _name, WholebodyKey* _obj, int _iend, int _dir, int _side, real_t _scale):
	Constraint(solver, 1, ID(ConTag::WholebodyFrictionForce, _obj->node, _obj->tick, _name), Constraint::Type::InequalityPenalty, _scale){
	obj  = _obj;
	iend = _iend;
	dir  = _dir;
	side = _side;
    
	AddSLink(obj->ends[iend].var_compl[dir == 0 ? 0 : 1]);
	AddSLink(obj->ends[iend].var_compl[2]);

}

WholebodyMomentCon::WholebodyMomentCon(Solver* solver, string _name, WholebodyKey* _obj, int _iend, int _dir, int _side, real_t _scale):
	Constraint(solver, 1, ID(ConTag::WholebodyMoment, _obj->node, _obj->tick, _name), Constraint::Type::InequalityPenalty, _scale){
	obj   = _obj;
	iend  = _iend;
	dir   = _dir;
	side  = _side;
    
	AddSLink(obj->ends[iend].var_compl[2]);
	AddSLink(obj->ends[iend].var_compl[dir == 0 ? 4 : 3]);

}

WholebodyComPosMatchCon::WholebodyComPosMatchCon(Solver* solver, string _name, WholebodyKey* _obj, real_t _scale):
	Constraint(solver, 3, ID(ConTag::WholebodyComPosMatch, _obj->node, _obj->tick, _name), Constraint::Type::Equality, _scale){
	obj   = _obj;

	AddSLink(obj->var_com_pos);
    
	int nend = obj->ends.size();
	for(int i = 0; i < nend; i++){
		AddM3Link(obj->ends[i].var_pos_t);
		AddM3Link(obj->ends[i].var_pos_r);
	}
    
}

WholebodyComVelMatchCon::WholebodyComVelMatchCon(Solver* solver, string _name, WholebodyKey* _obj, real_t _scale):
	Constraint(solver, 3, ID(ConTag::WholebodyComVelMatch, _obj->node, _obj->tick, _name), Constraint::Type::Equality, _scale){
	obj   = _obj;

	AddSLink(obj->var_com_vel);
    
	int nend = obj->ends.size();
	for(int i = 0; i < nend; i++){
		AddM3Link(obj->ends[i].var_pos_t);
		AddM3Link(obj->ends[i].var_pos_r);
		AddM3Link(obj->ends[i].var_vel_t);
		AddM3Link(obj->ends[i].var_vel_r);
	}
}

WholebodyMomentumMatchCon::WholebodyMomentumMatchCon(Solver* solver, string _name, WholebodyKey* _obj, real_t _scale):
	Constraint(solver, 3, ID(ConTag::WholebodyMomentumMatch, _obj->node, _obj->tick, _name), Constraint::Type::Equality, _scale){
	obj   = _obj;

	AddSLink(obj->var_mom);
   
	int nend = obj->ends.size();
	for(int i = 0; i < nend; i++){
		AddM3Link(obj->ends[i].var_pos_t);
		AddM3Link(obj->ends[i].var_pos_r);
		AddM3Link(obj->ends[i].var_vel_t);
		AddM3Link(obj->ends[i].var_vel_r);
	}

}

///////////////////////////////////////////////////////////////////////////////////////////////////

void WholebodyPosConT::Prepare(){
	WholebodyData::Link& dlnk0 = obj[0]->data.links[obj[0]->wb->ends[iend].ilink];
	WholebodyData::Link& dlnk1 = obj[1]->data.links[obj[0]->wb->ends[iend].ilink];

	p0 = dlnk0.pos_t;
	v0 = dlnk0.vel_t;
	p1 = dlnk1.pos_t;
	
	p_rhs = p0 + v0*obj[0]->hnext;
}

void WholebodyPosConR::Prepare(){
	WholebodyData::Link& dlnk0 = obj[0]->data.links[obj[0]->wb->ends[iend].ilink];
	WholebodyData::Link& dlnk1 = obj[1]->data.links[obj[0]->wb->ends[iend].ilink];

	q0 = dlnk0.pos_r;
	w0 = dlnk0.vel_r;
	q1 = dlnk1.pos_r;
	
	q_rhs = q0*quat_t::Rot(q0.Conjugated()*(w0*obj[0]->hnext));
}

void WholebodyVelConT::Prepare(){
	WholebodyData::Link& dlnk0 = obj[0]->data.links[obj[0]->wb->ends[iend].ilink];
	WholebodyData::Link& dlnk1 = obj[1]->data.links[obj[0]->wb->ends[iend].ilink];

	v0 = dlnk0.vel_t;
	a0 = dlnk0.acc_t;
	v1 = dlnk1.vel_t;
	
	v_rhs = v0 + a0*obj[0]->hnext;
}

void WholebodyVelConR::Prepare(){
	WholebodyData::Link& dlnk0 = obj[0]->data.links[obj[0]->wb->ends[iend].ilink];
	WholebodyData::Link& dlnk1 = obj[1]->data.links[obj[0]->wb->ends[iend].ilink];

	w0 = dlnk0.vel_r;
	u0 = dlnk0.acc_r;
	w1 = dlnk1.vel_r;
	
	w_rhs = w0 + u0*obj[0]->hnext;
}

void WholebodyComPosCon::Prepare(){
	pc0 = obj[0]->var_com_pos->val;
	vc0 = obj[0]->var_com_vel->val;
	pc1 = obj[1]->var_com_pos->val;
	
	pc_rhs = pc0 + vc0*obj[0]->hnext;
}

void WholebodyTotalForceCon::Prepare(){
	int nend  = obj[0]->ends.size();
	int nlink = obj[0]->wb->links.size();
	
	fsum.clear();
	for(int i = 0; i < nend; i++){
		WholebodyData::End& dend = obj[0]->data.ends[i];

		fsum += dend.pos_rc*dend.force_tc;
	}

	vec3_t g(0.0, 0.0, -obj[0]->wb->param.gravity);
	vc_rhs = obj[0]->var_com_vel->val + (obj[0]->hnext/obj[0]->wb->param.total_mass)*fsum + obj[0]->hnext*g;
}

void WholebodyTotalMomentCon::Prepare(){
	int nend = obj[0]->ends.size();

	msum.clear();
	for(int i = 0; i < nend; i++){
		WholebodyData::End& dend = obj[0]->data.ends[i];

		msum += (dend.pos_rc*dend.force_rc) + (dend.pos_tc - obj[0]->data.com_pos) % (dend.pos_rc*dend.force_tc);
	}

	L_rhs = obj[0]->var_mom->val + obj[0]->hnext*msum;
}

void WholebodyLimitCon::Prepare(){
	
}

void WholebodyNormalForceCon::Prepare(){
	WholebodyKey ::End& end  = obj->ends[iend];
	WholebodyData::End& dend = obj->data.ends[iend];
	on = (dend.state != Wholebody::ContactState::Free);

	if(on){
		fn = end.var_compl[2]->val;
	}
}

void WholebodyFrictionForceCon::Prepare(){
	WholebodyKey ::End& end  = obj->ends[iend];
	WholebodyData::End& dend = obj->data.ends[iend];
	on = (dend.state != Wholebody::ContactState::Free);

	if(on){
		mu = dend.mu;
		ft = end.var_compl[dir == 0 ? 0 : 1]->val;
		fn = end.var_compl[2]->val;
	}
}

void WholebodyMomentCon::Prepare(){
	WholebodyKey ::End& end  = obj->ends[iend];
	WholebodyData::End& dend = obj->data.ends[iend];
	on = (dend.state != Wholebody::ContactState::Free);

	if(on){
		fn   = end.var_compl[2]->val;
		m.x  = end.var_compl[3]->val;
		m.y  = end.var_compl[4]->val;
		cmin = dend.cop_min;
		cmax = dend.cop_max;
	}
}

void WholebodyComPosMatchCon::Prepare(){
	
}

void WholebodyComVelMatchCon::Prepare(){
	
}

void WholebodyMomentumMatchCon::Prepare(){
	
}

///////////////////////////////////////////////////////////////////////////////////////////////////

void WholebodyPosConT::CalcCoef(){
	Prepare();

	int idx = 0;
	((SLink*)links[idx++])->SetCoef( 1.0);
	((SLink*)links[idx++])->SetCoef(-1.0);
	((SLink*)links[idx++])->SetCoef(-obj[0]->hnext);
}

void WholebodyPosConR::CalcCoef(){
	Prepare();

	int idx = 0;
	((SLink*)links[idx++])->SetCoef( 1.0);
	((SLink*)links[idx++])->SetCoef(-1.0);
	((SLink*)links[idx++])->SetCoef(-obj[0]->hnext);
}

void WholebodyVelConT::CalcCoef(){
	Prepare();

	int idx = 0;
	((SLink*)links[idx++])->SetCoef( 1.0);
	((SLink*)links[idx++])->SetCoef(-1.0);

	WholebodyData::End& dend = obj[0]->data.ends[iend];
	((C3Link*)links[idx++])->SetCoef(dend.v_or_f[0] ? -obj[0]->hnext*dend.Rc.col(0)           : vec3_t());
	((C3Link*)links[idx++])->SetCoef(dend.v_or_f[1] ? -obj[0]->hnext*dend.Rc.col(1)           : vec3_t());
	((C3Link*)links[idx++])->SetCoef(dend.v_or_f[2] ? -obj[0]->hnext*dend.Rc.col(2)           : vec3_t());
	((C3Link*)links[idx++])->SetCoef(dend.v_or_f[3] ? -obj[0]->hnext*dend.rlc_cross_Rc.col(0) : vec3_t());
	((C3Link*)links[idx++])->SetCoef(dend.v_or_f[4] ? -obj[0]->hnext*dend.rlc_cross_Rc.col(1) : vec3_t());
	((C3Link*)links[idx++])->SetCoef(dend.v_or_f[5] ? -obj[0]->hnext*dend.rlc_cross_Rc.col(2) : vec3_t());
}

void WholebodyVelConR::CalcCoef(){
	Prepare();

	int idx = 0;
	((SLink*)links[idx++])->SetCoef( 1.0);
	((SLink*)links[idx++])->SetCoef(-1.0);
	
	WholebodyData::End& dend = obj[0]->data.ends[iend];
	((C3Link*)links[idx++])->SetCoef(dend.v_or_f[3] ? -obj[0]->hnext*dend.Rc.col(0) : vec3_t());
	((C3Link*)links[idx++])->SetCoef(dend.v_or_f[4] ? -obj[0]->hnext*dend.Rc.col(1) : vec3_t());
	((C3Link*)links[idx++])->SetCoef(dend.v_or_f[5] ? -obj[0]->hnext*dend.Rc.col(2) : vec3_t());
}

void WholebodyComPosCon::CalcCoef(){
	Prepare();

	((SLink*)links[0])->SetCoef( 1.0);
	((SLink*)links[1])->SetCoef(-1.0);
	((SLink*)links[2])->SetCoef(-obj[0]->hnext);
}

void WholebodyTotalForceCon::CalcCoef(){
	Prepare();

	int nend = obj[0]->ends.size();
	int idx = 0;
	((SLink*)links[idx++])->SetCoef( 1.0);
	((SLink*)links[idx++])->SetCoef(-1.0);

	for(int i = 0; i < nend; i++){
		WholebodyData::End& dend = obj[0]->data.ends[i];
		((C3Link*)links[idx++])->SetCoef(dend.v_or_f[0] ? vec3_t() : -(obj[0]->hnext/obj[0]->wb->param.total_mass)*dend.Rc.col(0));
		((C3Link*)links[idx++])->SetCoef(dend.v_or_f[1] ? vec3_t() : -(obj[0]->hnext/obj[0]->wb->param.total_mass)*dend.Rc.col(1));
		((C3Link*)links[idx++])->SetCoef(dend.v_or_f[2] ? vec3_t() : -(obj[0]->hnext/obj[0]->wb->param.total_mass)*dend.Rc.col(2));
	}
}

void WholebodyTotalMomentCon::CalcCoef(){
	Prepare();

	int nend = obj[0]->ends.size();
	int idx = 0;
	((SLink*)links[idx++])->SetCoef( 1.0);
	((SLink*)links[idx++])->SetCoef(-1.0);
	
	for(int i = 0; i < nend; i++){
		//ml = qc*mc + rc % qc*fc;
		WholebodyData::End& dend = obj[0]->data.ends[i];
		((C3Link*)links[idx++])->SetCoef(dend.v_or_f[0] ? vec3_t() : -obj[0]->hnext*dend.rcc_cross_Rc.col(0));
		((C3Link*)links[idx++])->SetCoef(dend.v_or_f[1] ? vec3_t() : -obj[0]->hnext*dend.rcc_cross_Rc.col(1));
		((C3Link*)links[idx++])->SetCoef(dend.v_or_f[2] ? vec3_t() : -obj[0]->hnext*dend.rcc_cross_Rc.col(2));
		((C3Link*)links[idx++])->SetCoef(dend.v_or_f[3] ? vec3_t() : -obj[0]->hnext*dend.Rc.col(0)          );
		((C3Link*)links[idx++])->SetCoef(dend.v_or_f[4] ? vec3_t() : -obj[0]->hnext*dend.Rc.col(1)          );
		((C3Link*)links[idx++])->SetCoef(dend.v_or_f[5] ? vec3_t() : -obj[0]->hnext*dend.Rc.col(2)          );
	}
}

void WholebodyLimitCon::CalcCoef(){
	Prepare();

	Wholebody::Limit& lim = obj->wb->limits[idx];
	Wholebody::Chain& ch  = obj->wb->chains[lim.ichain];
	
	((R3Link*)links[0])->SetCoef(vec3_t(obj->J_e_pe[ch.ibase].row(idx)));
	((R3Link*)links[1])->SetCoef(vec3_t(obj->J_e_qe[ch.ibase].row(idx)));
	((R3Link*)links[2])->SetCoef(vec3_t(obj->J_e_pe[ch.iend ].row(idx)));
	((R3Link*)links[3])->SetCoef(vec3_t(obj->J_e_qe[ch.iend ].row(idx)));
	
}

void WholebodyNormalForceCon::CalcCoef(){
	Prepare();

	((SLink*)links[0])->SetCoef(on ? 0.0 : 1.0);
}

void WholebodyFrictionForceCon::CalcCoef(){
	Prepare();

	// -mu*fn <= ft <= mu*fn
	// -ft + mu*fn >= 0
	//  ft + mu*fn >= 0

	((SLink*)links[0])->SetCoef(on ? (side == 0 ? -1.0 : 1.0) : 0.0);
	((SLink*)links[1])->SetCoef(on ?  mu    : 0.0);
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

	if(on){
		((SLink*)links[0])->SetCoef( (dir == 0 ? (side == 0 ? -cmin.x : cmax.x) : (side == 0 ? -cmin.y : cmax.y)) );
		((SLink*)links[1])->SetCoef( (dir == 0 ? (side == 0 ? -1.0    : 1.0   ) : (side == 0 ?  1.0    : -1.0  )) );
	}
	else{
		((SLink*)links[0])->SetCoef(0.0);
		((SLink*)links[1])->SetCoef(0.0);
	}
}

void WholebodyComPosMatchCon::CalcCoef(){
	Prepare();

	int nend = obj->ends.size();
	int idx  = 0;

	((SLink*)links[idx++])->SetCoef(1.0);

	for(int i = 0; i < nend; i++){
		((M3Link*)links[idx++])->SetCoef(-obj->J_pcom_pe[i]);
		((M3Link*)links[idx++])->SetCoef(-obj->J_pcom_qe[i]);
	}
}

void WholebodyComVelMatchCon::CalcCoef(){
	Prepare();

	int nend = obj->ends.size();
	int idx = 0;

	((SLink*)links[idx++])->SetCoef(1.0);

	for(int i = 0; i < nend; i++){
		WholebodyData::End& dend = obj->data.ends[i];

		((M3Link*)links[idx++])->SetCoef(-obj->J_vcom_pe[i]);
		((M3Link*)links[idx++])->SetCoef(-obj->J_vcom_qe[i]);
		((M3Link*)links[idx++])->SetCoef(-obj->J_vcom_ve[i]);
		((M3Link*)links[idx++])->SetCoef(-obj->J_vcom_ve[i]);
	}
}

void WholebodyMomentumMatchCon::CalcCoef(){
	Prepare();

	int nend = obj->ends.size();
	int idx = 0;

	((SLink*)links[idx++])->SetCoef(1.0);

	for(int i = 0; i < nend; i++){
		WholebodyData::End& dend = obj->data.ends[i];

		((M3Link*)links[idx++])->SetCoef(-obj->J_L_pe[i]);
		((M3Link*)links[idx++])->SetCoef(-obj->J_L_qe[i]);
		((M3Link*)links[idx++])->SetCoef(-obj->J_L_ve[i]);
		((M3Link*)links[idx++])->SetCoef(-obj->J_L_ve[i]);
	}
}

///////////////////////////////////////////////////////////////////////////////////////////////////

void WholebodyPosConT::CalcDeviation(){
	y = p1 - p_rhs;
}

void WholebodyPosConR::CalcDeviation(){
	quat_t qerror = q_rhs.Conjugated()*q1;
	vec3_t axis   = qerror.Axis ();
	real_t theta  = qerror.Theta();
	if(theta > pi)
		theta -= 2*pi;
	y = q_rhs*(theta*axis);
}

void WholebodyVelConT::CalcDeviation(){
	y = v1 - v_rhs;
}

void WholebodyVelConR::CalcDeviation(){
	y = w1 - w_rhs;
}

void WholebodyComPosCon::CalcDeviation(){
	y = pc1 - pc_rhs;
}

void WholebodyTotalForceCon::CalcDeviation(){
	y = obj[1]->var_com_vel->val - vc_rhs;
}

void WholebodyTotalMomentCon::CalcDeviation(){
	y = obj[1]->var_mom->val - L_rhs;
}

void WholebodyLimitCon::CalcDeviation(){
	real_t e = obj->data.e[idx];

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

void WholebodyNormalForceCon::CalcDeviation(){
	if(!on || fn > 0.0){
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
	if(!on || e > 0.0){
		y[0] = 0.0;
		active = false;
	}
	else{
		y[0] = e;
		active = true;
		//printf("!\n");
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

	//DSTR << m.x << " " << m.y << " " << fn << " " << e << endl;
	
	if(!on || e > 0.0){
		y[0] = 0.0;
		active = false;
	}
	else{
		y[0] = e;
		active = true;
	}
	
}

void WholebodyComPosMatchCon::CalcDeviation(){
	y = obj->var_com_pos->val - obj->data.com_pos;
	//DSTR << "com pos error: " << y << endl;
}

void WholebodyComVelMatchCon::CalcDeviation(){
	y = obj->var_com_vel->val - obj->data.com_vel;	
	//DSTR << "com vel error: " << obj->var_com_vel->val << " " << obj->data.vcom << endl;
}

void WholebodyMomentumMatchCon::CalcDeviation(){
	y = obj->var_mom->val - obj->data.mom;
	//DSTR << "mom error: " << obj->var_mom->val << " " << obj->data.L << endl;
}

///////////////////////////////////////////////////////////////////////////////////////////////////

void WholebodyPosConT::CalcLhs(){
	obj[1]->ends[iend].var_pos_t->val = p_rhs;
}

void WholebodyPosConR::CalcLhs(){
	obj[1]->ends[iend].var_pos_r->val = q_rhs;
}

void WholebodyVelConT::CalcLhs(){
	obj[1]->ends[iend].var_vel_t->val = v_rhs;
}

void WholebodyVelConR::CalcLhs(){
	obj[1]->ends[iend].var_vel_r->val = w_rhs;
}

void WholebodyComPosCon::CalcLhs(){
	obj[1]->var_com_pos->val = pc_rhs;
}

void WholebodyTotalForceCon::CalcLhs(){
	obj[1]->var_com_vel->val = vc_rhs;
}

void WholebodyTotalMomentCon::CalcLhs(){
	obj[1]->var_mom->val = L_rhs;
}

///////////////////////////////////////////////////////////////////////////////////////////////////

}
