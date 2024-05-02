#include <DiMP/Graph/Centroid.h>
#include <DiMP/Graph/Graph.h>
#include <DiMP/Render/Config.h>
#include <DiMP/Render/Canvas.h>

#include <sbrollpitchyaw.h>
#include <sbtokenizer.h>

namespace DiMP {;

const real_t m_pi    = 3.1415926535;
const real_t inf     = numeric_limits<real_t>::max();
const int    infi    = numeric_limits<int>::max();
const vec2_t inf2    = vec2_t(inf, inf);
const vec3_t inf3    = vec3_t(inf, inf, inf);
const real_t eps     = 1.0e-10;
const real_t damping = 0.0;
const vec3_t one(1.0, 1.0, 1.0);
const vec3_t ex (1.0, 0.0, 0.0);
const vec3_t ey (0.0, 1.0, 0.0);

// validify of waypoint values
inline bool is_valid(int i){
	return i != infi;
}
inline bool is_valid(real_t v){
	return v != inf;
}
inline bool is_valid(vec2_t v){
	return (v.x != inf && v.y != inf);
}
inline bool is_valid(vec3_t v){
	return (v.x != inf && v.y != inf && v.z != inf);
}
inline bool is_valid(quat_t v){
	return (v.w != inf && v.x != inf && v.y != inf && v.z != inf);
}

//-------------------------------------------------------------------------------------------------
// CentroidKey

CentroidKey::CentroidKey() {
	
}

void CentroidKey::AddVar(Solver* solver) {
	cen = (Centroid*)node;
	int nend = (int)cen->ends.size();
	
	// position and velocity
	var_pos_t = new V3Var(solver, ID(VarTag::CentroidTP, node, tick, name + "_tp"), cen->scale.pt);
	var_vel_t = new V3Var(solver, ID(VarTag::CentroidTV, node, tick, name + "_tv"), cen->scale.vt);
	var_pos_t->weight = damping*one;
	var_vel_t->weight = damping*one;
    solver->AddStateVar(var_pos_t, tick->idx);
	solver->AddStateVar(var_vel_t, tick->idx);
	
	var_pos_r = new QVar(solver, ID(VarTag::CentroidRP, node, tick, name + "_rp"), cen->scale.pr);
	var_pos_r->weight = damping*one;
	solver->AddStateVar(var_pos_r, tick->idx);

	var_pos_rpy = new V3Var(solver, ID(VarTag::CentroidRP, node, tick, name + "_rp"), cen->scale.pr);
	var_pos_rpy->weight = damping*one;
	solver->AddStateVar(var_pos_rpy, tick->idx);

	var_vel_r = new V3Var(solver, ID(VarTag::CentroidRV, node, tick, name + "_rv"), cen->scale.vr);
	var_vel_r->weight = damping*one;
	solver->AddStateVar(var_vel_r, tick->idx);
	
	var_time     = new SVar (solver, ID(VarTag::CentroidTime    , node, tick, name + "_time"    ), cen->scale.t );
	var_duration = new SVar (solver, ID(VarTag::CentroidDuration, node, tick, name + "_duration"), cen->scale.t );
	var_time    ->weight[0] = damping;
	var_duration->weight[0] = damping;
	solver->AddStateVar(var_time , tick->idx);
    solver->AddInputVar(var_duration, tick->idx);
	
	ends.resize(nend);
	stringstream ss, ss2;
	for(int i = 0; i < nend; i++){
		ends[i].key = this;
		
		ss.str("");
		ss << name << "_end" << i;

		ends[i].var_pos_t  = new V3Var(solver, ID(VarTag::CentroidEndPos, node, tick, ss.str() + "_pos_t"  ), cen->scale.pt);
		ends[i].var_pos_t->weight = damping*one;
		solver->AddStateVar(ends[i].var_pos_t, tick->idx);

		ends[i].var_vel_t  = new V3Var(solver, ID(VarTag::CentroidEndVel, node, tick, ss.str() + "_vel_t"  ), cen->scale.vt);
		ends[i].var_vel_t->weight = damping*one;
		solver->AddInputVar(ends[i].var_vel_t, tick->idx);
		
		ends[i].var_pos_r  = new QVar (solver, ID(VarTag::CentroidEndPos   , node, tick, ss.str() + "_pos_r"  ), cen->scale.pr);
		ends[i].var_pos_r->weight = damping*one;
		solver->AddStateVar(ends[i].var_pos_r , tick->idx);

		ends[i].var_vel_r  = new V3Var(solver, ID(VarTag::CentroidEndVel   , node, tick, ss.str() + "_vel_r"  ), cen->scale.vr);
		ends[i].var_vel_r->weight = damping*one;
		solver->AddInputVar(ends[i].var_vel_r , tick->idx);
			
		ends[i].var_stiff  = new SVar (solver, ID(VarTag::CentroidEndStiff , node, tick, ss.str() + "_stiff"    ), cen->scale.tinv);
		ends[i].var_stiff->weight[0] = damping;
		solver->AddInputVar(ends[i].var_stiff , tick->idx);

		for(int j = 0; j < 2; j++){
			ends[i].var_cmp[j] = new SVar(solver, ID(VarTag::CentroidEndCmp, node, tick, ss.str() + "_cmp"      ), cen->scale.pt );
			ends[i].var_cmp[j]->weight[0] = damping;
			solver->AddInputVar(ends[i].var_cmp[j], tick->idx);
		}

		ends[i].var_moment = new V3Var(solver, ID(VarTag::CentroidEndMoment, node, tick, ss.str() + "_moment"   ), cen->scale.pt2);
		ends[i].var_moment->weight = damping*one;
		solver->AddInputVar(ends[i].var_moment, tick->idx);
	}

	li  .resize(nend);
	li2 .resize(nend);
	pi  .resize(nend);
	ri  .resize(nend);
	etai.resize(nend);

	pi_cross.resize(nend);
	ri_cross.resize(nend);
	
	int ndiv = cen->param.rotationResolution;
	t     .resize(ndiv+1);
	C     .resize(ndiv+1);
	S     .resize(ndiv+1);
	C_tau .resize(ndiv+1);
	S_tau .resize(ndiv+1);
	C_lbar.resize(ndiv+1);
	S_lbar.resize(ndiv+1);

	coef.v_rhs   .resize(ndiv+1);
	coef.w_rhs   .resize(ndiv+1);
	coef.q_rhs   .resize(ndiv+1);
	coef.R_omega .resize(ndiv+1);
	coef.v_p     .resize(ndiv+1);
	coef.v_v     .resize(ndiv+1);
	coef.v_pbar  .resize(ndiv+1);
	coef.v_rbar  .resize(ndiv+1);
	coef.v_lbar  .resize(ndiv+1);
	coef.v_tau   .resize(ndiv+1);
	coef.w_p     .resize(ndiv+1);
	coef.w_v     .resize(ndiv+1);
	coef.w_tau   .resize(ndiv+1);
	coef.w_rbar  .resize(ndiv+1);
	coef.w_etabar.resize(ndiv+1);
	coef.q_w1    .resize(ndiv+1);
		
	coef.lbar_li    .resize(nend);
	coef.pbar_li    .resize(nend);
	coef.pbar_pi    .resize(nend);
	coef.pbar_vi    .resize(nend);
	coef.rbar_li    .resize(nend);
	coef.rbar_ri    .resize(nend);
	coef.etabar_li  .resize(nend);
	coef.etabar_pi  .resize(nend);
	coef.etabar_vi  .resize(nend);
	coef.etabar_ri  .resize(nend);
	coef.etabar_etai.resize(nend);

	coef.p_li.resize(nend);
	coef.p_pi.resize(nend);
	coef.p_vi.resize(nend);
	coef.p_ri.resize(nend);

	coef.v_li.resize(nend);
	coef.v_pi.resize(nend);
	coef.v_vi.resize(nend);
	coef.v_ri.resize(nend);

	coef.w_li.resize  (nend);
	coef.w_pi.resize  (nend);
	coef.w_vi.resize  (nend);
	coef.w_ri.resize  (nend);
	coef.w_etai.resize(nend);

	coef.q_li.resize  (nend);
	coef.q_pi.resize  (nend);
	coef.q_vi.resize  (nend);
	coef.q_ri.resize  (nend);
	coef.q_etai.resize(nend);

	for(int iend = 0; iend < nend; iend++){
		coef.v_li[iend].resize(ndiv+1);
		coef.v_pi[iend].resize(ndiv+1);
		coef.v_vi[iend].resize(ndiv+1);
		coef.v_ri[iend].resize(ndiv+1);

		coef.w_li  [iend].resize(ndiv+1);
		coef.w_pi  [iend].resize(ndiv+1);
		coef.w_vi  [iend].resize(ndiv+1);
		coef.w_ri  [iend].resize(ndiv+1);
		coef.w_etai[iend].resize(ndiv+1);
	}		
}

void CentroidKey::AddCon(Solver* solver) {
	CentroidKey* nextObj = (CentroidKey*)next;

    int nend  = (int)cen->ends .size();
    int nface = (int)cen->faces.size();

    if(next){
		con_pos_t = new CentroidPosConT(solver, name + "_pos_t", this, cen->scale.pt);
		solver->AddTransitionCon       (con_pos_t, tick->idx);
		
		con_vel_t = new CentroidVelConT(solver, name + "_vel_t", this, cen->scale.vt);
		solver->AddTransitionCon       (con_vel_t, tick->idx);
        
		con_pos_r = new CentroidPosConR(solver, name + "_pos_r", this, cen->scale.pr);
		solver->AddTransitionCon       (con_pos_r, tick->idx);
		
		con_pos_rpy = new CentroidPosConRPY(solver, name + "_pos_r", this, cen->scale.pr);
		solver->AddTransitionCon       (con_pos_rpy, tick->idx);
		
		con_vel_r = new CentroidVelConR(solver, name + "_vel_r", this, cen->scale.vr);
		solver->AddTransitionCon       (con_vel_r, tick->idx);
		
		con_time  = new CentroidTimeCon(solver, name + "_time" , this, cen->scale.t );
		solver->AddTransitionCon(con_time , tick->idx);		
        
        con_duration_range[0] = new CentroidDurationRangeCon(solver, name + "_duration", this,  1.0, cen->scale.t);
        con_duration_range[1] = new CentroidDurationRangeCon(solver, name + "_duration", this, -1.0, cen->scale.t);
        solver->AddCostCon(con_duration_range[0], tick->idx);
        solver->AddCostCon(con_duration_range[1], tick->idx);
    }

    con_des_pos_t = new FixConV3(solver, ID(ConTag::CentroidDesPosT, node, tick, name + "_des_pos_t"), var_pos_t, cen->scale.pt);
	solver->AddCostCon(con_des_pos_t, tick->idx);

	con_des_vel_t = new FixConV3(solver, ID(ConTag::CentroidDesVelT, node, tick, name + "_des_vel_t"), var_vel_t, cen->scale.vt);
    solver->AddCostCon(con_des_vel_t, tick->idx);

	con_des_pos_r = new FixConQ (solver, ID(ConTag::CentroidDesPosR, node, tick, name + "_des_pos_r"), var_pos_r, cen->scale.pr);
	solver->AddCostCon(con_des_pos_r, tick->idx);
		
	con_des_pos_rpy = new FixConV3(solver, ID(ConTag::CentroidDesPosR, node, tick, name + "_des_pos_r"), var_pos_rpy, cen->scale.pr);
	solver->AddCostCon(con_des_pos_rpy, tick->idx);
	
	con_des_vel_r = new FixConV3(solver, ID(ConTag::CentroidDesVelR, node, tick, name + "_des_vel_r"), var_vel_r, cen->scale.vr);
	solver->AddCostCon(con_des_vel_r, tick->idx);

	con_des_time     = new FixConS (solver, ID(ConTag::CentroidDesTime, node, tick, name + "_des_time"    ), var_time    , cen->scale.t );
	con_des_duration = new FixConS (solver, ID(ConTag::CentroidDesDuration, node, tick, name + "_des_duration"), var_duration, cen->scale.t );
    solver->AddCostCon(con_des_time    , tick->idx);
    solver->AddCostCon(con_des_duration, tick->idx);

	stringstream ss;
	for(int i = 0; i < nend; i++){
		ss.str("");
		ss << name << "_end" << i;

        if(next){
            ends[i].con_pos_t = new CentroidEndPosConT(solver, ss.str() + "_pos_t", this, i, cen->scale.pt);
            solver->AddTransitionCon(ends[i].con_pos_t, tick->idx);

			ends[i].con_pos_r = new CentroidEndPosConR(solver, ss.str() + "_pos_r", this, i, cen->scale.pr);
			solver->AddTransitionCon(ends[i].con_pos_r, tick->idx);

            ends[i].con_stiff_range     = new RangeConS (solver, ID(ConTag::CentroidEndStiffRange, node, tick, ss.str() + "_stiff_range" ), ends[i].var_stiff, cen->scale.tinv);
			solver->AddCostCon(ends[i].con_stiff_range    , tick->idx);
        }
	
		for(int j = 0; j < 3; j++){
			ends[i].con_pos_range[j][0] = new CentroidEndPosRangeCon(solver, name + "_pos_range", this, i, j,  1.0, cen->scale.pt);
			ends[i].con_pos_range[j][1] = new CentroidEndPosRangeCon(solver, name + "_pos_range", this, i, j, -1.0, cen->scale.pt);
			solver->AddCostCon(ends[i].con_pos_range[j][0], tick->idx);
			solver->AddCostCon(ends[i].con_pos_range[j][1], tick->idx);
		}

		ends[i].con_des_pos_t = new FixConV3(solver, ID(ConTag::CentroidDesEndPosT, node, tick, name + "_des_pos"), ends[i].var_pos_t, cen->scale.pt);
		ends[i].con_des_vel_t = new FixConV3(solver, ID(ConTag::CentroidDesEndVelT, node, tick, name + "_des_vel"), ends[i].var_vel_t, cen->scale.vt);
		solver->AddCostCon(ends[i].con_des_pos_t , tick->idx);
		solver->AddCostCon(ends[i].con_des_vel_t , tick->idx);
		
		ends[i].con_des_pos_r  = new FixConQ (solver, ID(ConTag::CentroidDesEndPosR, node, tick, name + "_des_pos"   ), ends[i].var_pos_r , cen->scale.pr  );
		ends[i].con_des_vel_r  = new FixConV3(solver, ID(ConTag::CentroidDesEndVelR, node, tick, name + "_des_vel"   ), ends[i].var_vel_r , cen->scale.vr  );
		solver->AddCostCon(ends[i].con_des_pos_r , tick->idx);
		solver->AddCostCon(ends[i].con_des_vel_r , tick->idx);

		ends[i].con_des_stiff  = new FixConS (solver, ID(ConTag::CentroidDesEndStiff, node, tick, name + "_des_stiff" ), ends[i].var_stiff , cen->scale.tinv);
		solver->AddCostCon(ends[i].con_des_stiff , tick->idx);

		for(int j = 0; j < 2; j++){
			ends[i].con_des_cmp[j] = new FixConS(solver, ID(ConTag::CentroidDesEndCmp, node, tick, name + "_des_cmp"   ), ends[i].var_cmp[j]   , cen->scale.pt  );
			solver->AddCostCon(ends[i].con_des_cmp[j], tick->idx);
		}

		ends[i].con_des_moment = new FixConV3(solver, ID(ConTag::CentroidDesEndMoment, node, tick, name + "_des_moment"), ends[i].var_moment, cen->scale.pt2 );
		solver->AddCostCon(ends[i].con_des_moment, tick->idx);
		
		/// contact force constraints
		if(next){
			ends[i].con_friction = new CentroidEndFrictionCon(solver, name + "_friction", this, i, cen->scale.pt);
			solver->AddCostCon(ends[i].con_friction, tick->idx);
			
			//ends[i].con_moment[0][0] = new CentroidEndMomentCon  (solver, name + "_moment00", this, i, 0,  1.0, cen->scale.pt2);
			//ends[i].con_moment[1][0] = new CentroidEndMomentCon  (solver, name + "_moment10", this, i, 1,  1.0, cen->scale.pt2);
			//ends[i].con_moment[2][0] = new CentroidEndMomentCon  (solver, name + "_moment20", this, i, 2,  1.0, cen->scale.pt2);
			//ends[i].con_moment[0][1] = new CentroidEndMomentCon  (solver, name + "_moment01", this, i, 0, -1.0, cen->scale.pt2);
			//ends[i].con_moment[1][1] = new CentroidEndMomentCon  (solver, name + "_moment11", this, i, 1, -1.0, cen->scale.pt2);
			//ends[i].con_moment[2][1] = new CentroidEndMomentCon  (solver, name + "_moment21", this, i, 2, -1.0, cen->scale.pt2);

			//solver->AddCostCon(ends[i].con_moment[0][0], tick->idx);
			//solver->AddCostCon(ends[i].con_moment[1][0], tick->idx);
			//solver->AddCostCon(ends[i].con_moment[2][0], tick->idx);
			//solver->AddCostCon(ends[i].con_moment[0][1], tick->idx);
			//solver->AddCostCon(ends[i].con_moment[1][1], tick->idx);
			//solver->AddCostCon(ends[i].con_moment[2][1], tick->idx);
		}
        
        ends[i].con_contact.resize(nface);
		for(int j = 0; j < nface; j++){
		    ends[i].con_contact[j] = new CentroidEndContactCon (solver, name + "_contact" , this, i, j, cen->scale.pt);
            
			solver->AddCostCon(ends[i].con_contact[j], tick->idx);
        }
	}
}

inline mat3_t vvtrmat(vec3_t c, vec3_t r){
	mat3_t m;
	for(int i = 0; i < 3; i++) for(int j = 0; j < 3; j++)
		m[i][j] = c[i]*r[j];
	return m;
}

inline mat3_t RotJacobian(vec3_t omega){
	real_t theta = omega.norm();
	if(theta < eps)
		return mat3_t();

	vec3_t eta = omega/theta;

	mat3_t A = (sin(theta)/theta)*mat3_t() + (1.0 - (sin(theta)/theta))*vvtrmat(eta, eta) + ((cos(theta) - 1.0)/theta)*mat3_t::Cross(eta);
	
	return A;
}

inline mat3_t RpyJacobian(vec3_t rpy){
	mat3_t J;
	quat_t qz  =    quat_t::Rot(rpy.z, 'z');
	quat_t qzy = qz*quat_t::Rot(rpy.y, 'y');
	
	J.col(0) = qzy*vec3_t(1.0, 0.0, 0.0);
	J.col(1) = qz *vec3_t(0.0, 1.0, 0.0);
	J.col(2) =     vec3_t(0.0, 0.0, 1.0);
	
	return J;
}

void CentroidKey::Prepare() {
	if(!next)
		return;

    cen = (Centroid*)node;
	const real_t eps2 = 1.0e-06;

	int nend = (int)ends.size();
	int ndiv = cen->param.rotationResolution;
	
	tau  = var_duration->val;
	dtau = tau/(real_t)ndiv;
	for(int k = 0; k <= ndiv; k++){
		t[k] = k*dtau;
	}

	// to avoid singularity in flight phase
    l2sum = eps2*eps2;
	g = vec3_t(0.0, 0.0, cen->param.g);
	
	psum.clear();
	rsum.clear();
	etabar.clear();
	for(int i = 0; i < nend; i++){
		li  [i] = ends[i].var_stiff->val;
		li2 [i] = li[i]*li[i];
		pi  [i] = ends[i].var_pos_t->val + ends[i].var_vel_t->val*(tau/2.0);
		pi_cross[i] = mat3_t::Cross(pi[i]);
		
		ri  [i] = vec3_t(ends[i].var_cmp[0]->val, ends[i].var_cmp[1]->val, 0.0);
		ri_cross[i] = mat3_t::Cross(ri[i]);
		etai[i] = ends[i].var_moment->val;
		
		l2sum   += li2[i];
		psum    += li2[i]*pi[i];
		rsum    += li2[i]*ri[i];
		etabar  += li2[i]*(etai[i] - pi[i] % ri[i]);
	}

	lbar    = sqrt(l2sum);
	pbar    = (psum + g)/l2sum;
	rbar    = rsum/l2sum;
	etabar += l2sum*pbar % rbar;

	pbar_cross = mat3_t::Cross(pbar);
	rbar_cross = mat3_t::Cross(rbar);

	for(int k = 0; k <= ndiv; k++){
		C[k]      = cosh(lbar*t[k]);
		S[k]      = sinh(lbar*t[k]);
		C_tau [k] = ((real_t)k/(real_t)ndiv)*lbar*S[k];
		S_tau [k] = ((real_t)k/(real_t)ndiv)*lbar*C[k];
		C_lbar[k] = t[k]*S[k];
		S_lbar[k] = t[k]*C[k];
	}
	
	CentroidKey* obj[2];
	obj[0] = this;
	obj[1] = (CentroidKey*)next;

	Iinv_m = cen->param.Iinv*cen->param.m;
	
	coef.p   = obj[0]->var_pos_t->val;
	coef.v   = obj[0]->var_vel_t->val;
	coef.q   = obj[0]->var_pos_r->val;
	coef.rpy = obj[0]->var_pos_rpy->val;
	coef.w   = obj[0]->var_vel_r->val;
		
	coef.p_lhs   = obj[1]->var_pos_t->val;
	coef.v_lhs   = obj[1]->var_vel_t->val;
	coef.q_lhs   = obj[1]->var_pos_r->val;
	coef.rpy_lhs = obj[1]->var_pos_rpy->val;
	coef.w_lhs   = obj[1]->var_vel_r->val;
		
	coef.p_rhs = pbar + rbar + C[ndiv]*(coef.p - (pbar + rbar)) + (S[ndiv]/lbar)*coef.v;

	for(int k = 0; k <= ndiv; k++){
		coef.v_rhs[k] = (lbar*S[k])*(coef.p - (pbar + rbar)) + C[k]*coef.v;
		coef.w_rhs[k] = coef.w + Iinv_m*( (coef.v_rhs[k] - coef.v) % rbar + t[k]*etabar );
	}

	// q[k+1] = q(w(tk+dtau*(ndiv-1)))*q(w(tk+dtau*(ndiv-2))) * ... * q(w(tk)) * q[k]
	if(cen->param.enableQuaternion)
		 coef.q_rhs[0] = coef.q;
	else coef.q_rhs[0] = FromRollPitchYaw(coef.rpy);
		
	for(int k = 0; k < ndiv; k++){
		coef.q_rhs[k+1] = quat_t::Rot(coef.w_rhs[k]*dtau)*coef.q_rhs[k];
		//coef.q_rhs[k+1].unitize();
	}
	coef.rpy_rhs = ToRollPitchYaw(coef.q_rhs[ndiv]);
}

void CentroidKey::PrepareStep(){
	int nend = (int)ends.size();
	int ndiv = cen->param.rotationResolution;

	coef.R_omega[ndiv] = mat3_t();
	for(int k = ndiv-1; k >= 0; k--){
		//vec3_t omega = coef.w_rhs[k]*dtau;
		//mat3_t R = mat3_t::Rot(omega.norm(), omega.unit());
		mat3_t R;
		quat_t::Rot(coef.w_rhs[k]*dtau).ToMatrix(R);
		coef.R_omega[k] = coef.R_omega[k+1]*R;
		//coef.R_omega[k] = mat3_t();
	}
	if(!cen->param.enableQuaternion){
		mat3_t Jrpy0 = RpyJacobian(coef.rpy);
		mat3_t Jrpy1 = RpyJacobian(coef.rpy_rhs);
		mat3_t Jrpy1inv = Jrpy1.inv();

		coef.R_omega[0] = coef.R_omega[0]*Jrpy0;
		for(int k = 0; k <= ndiv; k++)
			coef.R_omega[k] = Jrpy1inv*coef.R_omega[k];
	}

	// calc direct coefficients

	// lbar coef
	for(int i = 0; i < nend; i++){
		coef.lbar_li[i] = li[i]/lbar;
	}
	// pbar coef
	coef.pbar_lbar = -2.0*(psum + g)/(l2sum*lbar);
	for(int i = 0; i < nend; i++){
		coef.pbar_li[i] = 2.0*li[i]*pi[i]/l2sum;
		coef.pbar_pi[i] = li2[i]/l2sum;
		coef.pbar_vi[i] = coef.pbar_pi[i]*(tau/2.0);
	}
	// rbar coef
	coef.rbar_lbar = -2.0*rsum/(l2sum*lbar);
	for(int i = 0; i < nend; i++){
		coef.rbar_li[i] = 2.0*li[i]*ri[i]/l2sum;
		coef.rbar_ri[i] = li2[i]/l2sum;
	}
	// etabar coef
	coef.etabar_lbar =  2.0*lbar*pbar % rbar;
	coef.etabar_pbar = -l2sum*rbar_cross;
	coef.etabar_rbar =  l2sum*pbar_cross;
	for(int i = 0; i < nend; i++){
		coef.etabar_li  [i] =  2.0*li[i]*(etai[i] - pi[i] % ri[i]);
		coef.etabar_pi  [i] =  li2[i]*ri_cross[i];
		coef.etabar_vi  [i] =  coef.etabar_pi[i]*(tau/2.0);
		coef.etabar_ri  [i] = -li2[i]*pi_cross[i];
		coef.etabar_etai[i] =  li2[i];
	}
	// p' coef
	coef.p_C    =  coef.p - (pbar + rbar);
	coef.p_S    =  (1.0/lbar)*coef.v;
	coef.p_p    =  C[ndiv];
	coef.p_v    =  (S[ndiv])/lbar;
	coef.p_lbar = -(S[ndiv]/l2sum)*coef.v;
	coef.p_pbar =  1.0 - C[ndiv];
	coef.p_rbar =  1.0 - C[ndiv];
	// v' coef
	coef.v_C = coef.v;
	coef.v_S = lbar*(coef.p - (pbar + rbar));
	for(int k = 0; k <= ndiv; k++){
		coef.v_p   [k] =  lbar*S[k];
		coef.v_v   [k] =  C[k];
		coef.v_lbar[k] =  S[k]*(coef.p - (pbar + rbar));
		coef.v_pbar[k] = -lbar*S[k];
		coef.v_rbar[k] = -lbar*S[k];
	}
	// w' coef
	coef.w_w    =  1;
	coef.w_v1   = -Iinv_m*rbar_cross;
	for(int k = 0; k <= ndiv; k++){
		coef.w_p     [k].clear();
		coef.w_v     [k] = Iinv_m*rbar_cross;
		coef.w_rbar  [k] = Iinv_m*(mat3_t::Cross(coef.v_rhs[k] - coef.v));
		coef.w_etabar[k] = Iinv_m*t[k];
		coef.w_tau   [k] = Iinv_m*((real_t)k/(real_t)ndiv)*etabar;
	}
	// q' coef
	coef.q_q = coef.R_omega[0];
	coef.q_w.clear();
	coef.q_p.clear();
	coef.q_v.clear();
	coef.q_tau.clear();
	for(int k = 0; k < ndiv; k++){
		mat3_t tmp = coef.R_omega[k+1]*RotJacobian(coef.w_rhs[k]*dtau);
		coef.q_w1[k] = tmp*dtau;
		coef.q_tau  += tmp*coef.w_rhs[k]/(real_t)ndiv;
	}
	for(int i = 0; i < nend; i++){
		coef.q_li  [i].clear();
		coef.q_pi  [i].clear();
		coef.q_vi  [i].clear();
		coef.q_ri  [i].clear();
		coef.q_etai[i].clear();
	}

	// calc dependent coefficients

	// p' coef
	coef.p_tau = coef.p_C*C_tau[ndiv] + coef.p_S*S_tau[ndiv];
	for(int i = 0; i < nend; i++){
		coef.p_li[i] = 
			(coef.p_C*C_lbar[ndiv] + coef.p_S*S_lbar[ndiv])*coef.lbar_li[i] +
			coef.p_pbar*(coef.pbar_lbar*coef.lbar_li[i] + coef.pbar_li[i]) +
			coef.p_rbar*(coef.rbar_lbar*coef.lbar_li[i] + coef.rbar_li[i]);
		coef.p_pi[i] = coef.p_pbar*coef.pbar_pi[i];
		coef.p_vi[i] = coef.p_pbar*coef.pbar_vi[i];
		coef.p_ri[i] = coef.p_rbar*coef.rbar_ri[i];
	}
	// v' coef
	for(int k = 0; k <= ndiv; k++){
		coef.v_tau[k]  = coef.v_C*C_tau[k] + coef.v_S*S_tau[k];
	}
	for(int i = 0; i < nend; i++){
		for(int k = 0; k <= ndiv; k++){
			coef.v_li[i][k] = 
				(coef.v_C*C_lbar[k] + coef.v_S*S_lbar[k])*coef.lbar_li[i] +
				coef.v_pbar[k]*(coef.pbar_lbar*coef.lbar_li[i] + coef.pbar_li[i]) +
				coef.v_rbar[k]*(coef.rbar_lbar*coef.lbar_li[i] + coef.rbar_li[i]);
			coef.v_pi[i][k] = coef.v_pbar[k]*coef.pbar_pi[i];
			coef.v_vi[i][k] = coef.v_pbar[k]*coef.pbar_vi[i];
			coef.v_ri[i][k] = coef.v_rbar[k]*coef.rbar_ri[i];
		}
	}
	// w' coef
	for(int k = 0; k <= ndiv; k++){
		coef.w_p  [k] += coef.w_v1*coef.v_p  [k];
		coef.w_v  [k] += coef.w_v1*coef.v_v  [k];
		coef.w_tau[k] += coef.w_v1*coef.v_tau[k];
	}
	for(int i = 0; i < nend; i++){
		for(int k = 0; k <= ndiv; k++){
			coef.w_li[i][k] = 
				coef.w_v1*coef.v_li[i][k] +
				coef.w_rbar  [k]*(coef.rbar_li[i] + coef.rbar_lbar*coef.lbar_li[i]);
				coef.w_etabar[k]*(
					coef.etabar_li[i] + 
					(coef.etabar_lbar + coef.etabar_pbar*coef.pbar_lbar + coef.etabar_rbar*coef.rbar_lbar)*coef.lbar_li[i] +
					coef.etabar_pbar*coef.pbar_li[i] +
					coef.etabar_rbar*coef.rbar_li[i]
					);
			coef.w_pi[i][k] = 
				coef.w_v1*coef.v_pi[i][k] +
				coef.w_etabar[k]*(coef.etabar_pi[i] + coef.etabar_pbar*coef.pbar_pi[i]);
			coef.w_vi[i][k] = 
				coef.w_v1*coef.v_vi[i][k] +
				coef.w_etabar[k]*(coef.etabar_vi[i] + coef.etabar_pbar*coef.pbar_vi[i]);
			coef.w_ri[i][k] = 
				coef.w_v1*coef.v_ri[i][k] +
				coef.w_rbar  [k]*coef.rbar_ri[i] +
				coef.w_etabar[k]*(coef.etabar_ri[i] + coef.etabar_rbar*coef.rbar_ri[i]);
			coef.w_etai[i][k] = 
				coef.w_etabar[k]*coef.etabar_etai[i];
		}
	}
	// q' coef
	for(int k = 0; k < ndiv; k++){
		coef.q_w   += coef.q_w1[k]*coef.w_w;
		coef.q_p   += coef.q_w1[k]*coef.w_p  [k];
		coef.q_v   += coef.q_w1[k]*coef.w_v  [k];
		coef.q_tau += coef.q_w1[k]*coef.w_tau[k];
	}
	for(int i = 0; i < nend; i++){
		for(int k = 0; k < ndiv; k++){
			coef.q_li  [i] += coef.q_w1[k]*coef.w_li  [i][k];
			coef.q_pi  [i] += coef.q_w1[k]*coef.w_pi  [i][k];
			coef.q_vi  [i] += coef.q_w1[k]*coef.w_vi  [i][k];
			coef.q_ri  [i] += coef.q_w1[k]*coef.w_ri  [i][k];
			coef.q_etai[i] += coef.q_w1[k]*coef.w_etai[i][k];
		}
	}
}

void CentroidKey::Finish(){
	//DSTR << tick->idx << endl;
	tick->time = var_time->val;
	
	var_duration->val = std::min(std::max(cen->param.durationMin, var_duration->val), cen->param.durationMax);
	//DSTR << "duration: " << var_duration->val << endl;
	//DSTR << "pos_r: " << var_pos_r->val << " vel_r: " << var_vel_r->val << " lbar: " << lbar << " pbar: " << pbar << " rbar: " << rbar << " etabar: " << etabar << endl;
	//DSTR << "rpy: " << var_pos_rpy->val << " vel_r: " << var_vel_r->val << " pbar: " << pbar << " rbar: " << rbar << " etabar: " << etabar << " lbar: " << lbar << " tau: " << tau << endl;
	//DSTR << "vel_r: " << var_vel_r->val << endl;
	//DSTR << "con_des_pos_r : " << con_des_pos_r->y << endl;

	real_t dmax = 0.0;
	for(int i = 0; i < ends.size(); i++){
		End& end = ends[i];
		//int njnt = end.var_joint_pos.size();
		end.var_stiff->val = std::min(std::max(0.0, end.var_stiff->val), 100.0);

		//DSTR << i
		//	 << " iface: "  << end.iface
		//	 << " pos_t: "  << end.var_pos_t->val
		//	 << " vel_t: "  << end.var_vel_r->val
		//	 << " vel_r: "  << end.var_vel_r->val
		//	 << " stiff: "  << end.var_stiff->val
		//	 << " cmp: "    << end.var_cmp[0]->val << " " << end.var_cmp[1]->val
		//     << " moment: " << end.var_moment->val
		//	 << " fric: "   << (next ? end.con_friction->y[0] : 0.0)
		//     << endl;
		vec3_t pe = end.var_pos_t->val - var_pos_t->val;
		for(int j = 0; j < 3; j++){
			dmax = std::max(dmax,   pe[j] - cen->ends[i].posMax[j] );
			dmax = std::max(dmax, -(pe[j] - cen->ends[i].posMin[j]));
			//DSTR << i << " " << j << " " << end.var_joint_pos[j]->val << endl;
			//end.var_joint_pos[j]->val = std::max(cen->ends[i].jointPosMin[j]/cen->L, end.var_joint_pos[j]->val);
			//end.var_joint_pos[j]->val = std::min(cen->ends[i].jointPosMax[j]/cen->L, end.var_joint_pos[j]->val);
			//end.var_joint_vel[j]->val = std::max(cen->ends[i].jointVelMin[j]/cen->V, end.var_joint_vel[j]->val);
			//end.var_joint_vel[j]->val = std::min(cen->ends[i].jointVelMax[j]/cen->V, end.var_joint_vel[j]->val);
		}
	}
	DSTR << "max violation: " << dmax << endl;
	//if(next){
	//	DSTR << var_duration->val;
	//	//DSTR << con_pos_t->y << " " << con_vel_t->y << endl;
		//DSTR << " " << ends[0].var_stiff->val << " " << ends[1].var_stiff->val << endl;
	//}
	//DSTR << endl;

	Prepare();
	
}

void CentroidKey::Draw(Render::Canvas* canvas, Render::Config* conf) {
	Vec3f  p;

	canvas->SetPointSize(5.0f);
	p = var_pos_t->val;
	canvas->Point(p);
	/*
	quat_t q;
	if(cen->param.enableQuaternion)
		 q = var_pos_r->val;
	else q = FromRollPitchYaw(var_pos_rpy->val);

	canvas->BeginPath();
	canvas->SetLineWidth(1.0f);
	p = var_pos_t->val;
	canvas->MoveTo(p);
	p = var_pos_t->val + q*vec3_t(0.0, 0.0, 0.1);
	canvas->LineTo(p);
	canvas->EndPath();
	*/
	int nend = (int)ends.size();
	for(int i = 0; i < nend; i++){
		if(ends[i].iface == -1)
			continue;

		canvas->SetPointSize(2.0f);
		canvas->SetPointColor("black");
		p = ends[i].var_pos_t->val;
		canvas->Point(p);

		canvas->SetPointColor("blue");
		p = ends[i].var_pos_t->val + vec3_t(ends[i].var_cmp[0]->val, ends[i].var_cmp[1]->val, 0.0);
		canvas->Point(p);
	}
}

//-------------------------------------------------------------------------------------------------
// Centroid

Centroid::Param::Param() {
	g  = 9.8;
	m  = 1.0;
	I  = mat3_t();
	mu = 1.0;

    durationMin = 0.1;
	durationMax = 1.0;

    bodyRangeMin = vec3_t(-0.1, -0.1, -0.1);
    bodyRangeMax = vec3_t( 0.1,  0.1,  0.1);

    swingSlope  = 1.0;
    swingHeight = 0.05;

	contactMargin = 0.0;
	contactSwitchCost = 0.0;
	contactFaceSwitchCost = 0.0;

	enableRotation   = true;
	enableQuaternion = true;
	lockRpy[0] = false;
	lockRpy[1] = false;
	lockRpy[2] = false;
	rotationResolution = 1;
}

//-------------------------------------------------------------------------------------------------

Centroid::End::End(){
    stiffnessMax    = 1.0;
	lockOri         = false;
	lockCmp         = false;
	lockMoment      = false;
}

//-------------------------------------------------------------------------------------------------

Centroid::Face::Face(){
	normal       = vec3_t(0.0, 0.0, 1.0);
    hull         = 0;
    numSwitchMax = 100;
}

//-------------------------------------------------------------------------------------------------

Centroid::Waypoint::End::Value::Value(){
	pos_t = inf3;
	pos_r = inf3;
	vel_t = inf3;
	vel_r = inf3;
	iface = infi;
}
Centroid::Waypoint::End::Value::Value(vec3_t _p, vec3_t _q, vec3_t _v, vec3_t _w, int _iface){
	pos_t = _p;
	pos_r = _q;
	vel_t = _v;
	vel_r = _w;
	iface = _iface;
}
Centroid::Waypoint::End::Weight::Weight(){
	pos_t  = inf3;
	pos_r  = inf3;
	vel_t  = inf3;
	vel_r  = inf3;
	stiff  = inf;
	cmp    = inf2;
	moment = inf3;
}
Centroid::Waypoint::End::Weight::Weight(vec3_t _p, vec3_t _q, vec3_t _v, vec3_t _w, real_t _l, vec2_t _r, vec3_t _eta){
	pos_t  = _p;
	pos_r  = _q;
	vel_t  = _v;
	vel_r  = _w;
	stiff  = _l;
	cmp    = _r;
	moment = _eta;
}
Centroid::Waypoint::End::End(){
}
Centroid::Waypoint::Value::Value(){
	time     = inf;
	duration = inf;
	pos_t    = vec3_t(inf, inf, inf);
	pos_r    = vec3_t(inf, inf, inf);
	vel_t    = vec3_t(inf, inf, inf);
	vel_r    = vec3_t(inf, inf, inf);
}
Centroid::Waypoint::Value::Value(real_t _t, real_t _tau, vec3_t _p, vec3_t _q, vec3_t _v, vec3_t _w){
	time     = _t;
	duration = _tau;
	pos_t    = _p;
	pos_r    = _q;
	vel_t    = _v;
	vel_r    = _w;
}
Centroid::Waypoint::Weight::Weight(){
	time     = inf;
	duration = inf;
	pos_t    = inf3;
	pos_r    = inf3;
	vel_t    = inf3;
	vel_r    = inf3;
}
Centroid::Waypoint::Weight::Weight(real_t _t, real_t _tau, vec3_t _p, vec3_t _q, vec3_t _v, vec3_t _w){
	time     = _t;
	duration = _tau;
	pos_t    = _p;
	pos_r    = _q;
	vel_t    = _v;
	vel_r    = _w;
}

Centroid::Waypoint::Waypoint() {

}

//-------------------------------------------------------------------------------------------------

Centroid::Snapshot::Snapshot() {
	t = 0.0;
}

//-------------------------------------------------------------------------------------------------

Centroid::Centroid(Graph* g, string n) :TrajectoryNode(g, n) {
	type = Type::Object;
	graph->centroids.Add(this);
}

Centroid::~Centroid() {
	graph->centroids.Remove(this);
}

void Centroid::SetScaling(){
	// moment of inertial of solid sphere = 0.4 m r^2
	// r = sqrt(I/(0.4m))
	scale.l    = sqrt(param.I[0][0]/(0.4*param.m));//0.5;  //< unit length
	scale.t    = sqrt(scale.l/param.g);//graph->ticks[1]->time - graph->ticks[0]->time;
	scale.tinv = 1.0/scale.t;
	scale.at   = param.g;
	scale.vt   = scale.at*scale.t;
	scale.ft   = param.m*param.g;
	scale.pt   = scale.vt*scale.t;
	scale.pt2  = scale.pt*scale.pt;
	scale.pr   = scale.pt/scale.l;
	scale.vr   = scale.vt/scale.l;
	scale.ar   = scale.at/scale.l;
	scale.fr   = scale.ft*scale.l;
	scale.L    = scale.fr*scale.t;

	//scale.l    = 1.0;
	//scale.t    = 1.0;
	//scale.tinv = 1.0;
	//scale.at   = 1.0;
	//scale.vt   = 1.0;
	//scale.ft   = 1.0;
	//scale.pt   = 1.0;
	//scale.pr   = 1.0;
	//scale.vr   = 1.0;
	//scale.ar   = 1.0;
	//scale.fr   = 1.0;
	//scale.L    = 1.0;
}

void Centroid::Init() {
	TrajectoryNode::Init();

	param.Iinv = param.I.inv();

    int nend  = (int)ends .size();
    int nface = (int)faces.size();
	int N     = (int)graph->ticks.size()-1;

	// set contact pattern from string
	if(!param.contactPattern.empty()){
		Tokenizer tok(param.contactPattern, " ", true);
		for(int k = 0; k <= N; k++){
			CentroidKey* key = (CentroidKey*)traj.GetKeypoint(graph->ticks[k]);

			for(int i = 0; i < nend; i++){
				string str = to_string(tok.GetToken());
				int iface;
				if(str == "-")
					iface = -1;
				else if(str == "*")
					iface = infi;
				else{
					Converter::FromString(str, iface);
				}

				key->ends[i].iface = iface;

				tok.Next();
			}
		}
	}

	// initialize position and velocity values by spline curve connecting the waypoints
	Curved              curve_time;
	Curve3d             curve_pos_t;
	Curve3d             curve_pos_r;
	Curved              curve_weight_time;
	Curved              curve_weight_duration;
	Curve3d             curve_weight_pos_t;
	Curve3d             curve_weight_pos_r;
	Curve3d             curve_weight_vel_t;
	Curve3d             curve_weight_vel_r;
	vector<Curve3d>     curve_end_pos_t;
	vector<Curve3d>     curve_end_pos_r;
	vector<Curve3d>     curve_end_weight_pos_t ;
	vector<Curve3d>     curve_end_weight_pos_r ;
	vector<Curve3d>     curve_end_weight_vel_t ;
	vector<Curve3d>     curve_end_weight_vel_r ;
	vector<Curved>      curve_end_weight_stiff ;
	vector<Curve3d>     curve_end_weight_moment;
	vector<Curve2d>     curve_end_weight_cmp   ;
	
	curve_time           .SetType(Interpolate::LinearDiff);
	curve_pos_t          .SetType(Interpolate::Cubic     );
	curve_pos_r          .SetType(Interpolate::Cubic     );
	curve_weight_time    .SetType(Interpolate::LinearDiff);
	curve_weight_duration.SetType(Interpolate::LinearDiff);
	curve_weight_pos_t   .SetType(Interpolate::LinearDiff);
	curve_weight_pos_r   .SetType(Interpolate::LinearDiff);
	curve_weight_vel_t   .SetType(Interpolate::LinearDiff);
	curve_weight_vel_r   .SetType(Interpolate::LinearDiff);

	curve_end_pos_t        .resize(ends.size());
	curve_end_pos_r        .resize(ends.size());
	curve_end_weight_pos_t .resize(ends.size());
	curve_end_weight_pos_r .resize(ends.size());
	curve_end_weight_vel_t .resize(ends.size());
	curve_end_weight_vel_r .resize(ends.size());
	curve_end_weight_stiff .resize(ends.size());
	curve_end_weight_cmp   .resize(ends.size());
	curve_end_weight_moment.resize(ends.size());

    for(int j = 0; j < nend; j++){
		curve_end_pos_t[j].SetType(Interpolate::Cubic);
		curve_end_pos_r[j].SetType(Interpolate::Cubic);
		curve_end_weight_pos_t [j].SetType(Interpolate::LinearDiff);
		curve_end_weight_pos_r [j].SetType(Interpolate::LinearDiff);
		curve_end_weight_vel_t [j].SetType(Interpolate::LinearDiff);
		curve_end_weight_vel_r [j].SetType(Interpolate::LinearDiff);
		curve_end_weight_stiff [j].SetType(Interpolate::LinearDiff);
		curve_end_weight_cmp   [j].SetType(Interpolate::LinearDiff);
		curve_end_weight_moment[j].SetType(Interpolate::LinearDiff);
	}

	// interpolation of time and weights
	for(int k = 0; k <= N; k++){
		Waypoint& wp = waypoints[k];
		if(is_valid(wp.value.time)){
			curve_time.AddPoint(k);
			curve_time.SetPos((int)curve_time.NPoints()-1, wp.value.time);
		}
		if(is_valid(wp.weight.time)){
			curve_weight_time.AddPoint(k);
			curve_weight_time.SetPos((int)curve_weight_time.NPoints()-1, wp.weight.time);
		}
		if(is_valid(wp.weight.duration)){
			curve_weight_duration.AddPoint(k);
			curve_weight_duration.SetPos((int)curve_weight_duration.NPoints()-1, wp.weight.duration);
		}
		if(is_valid(wp.weight.pos_t)){
			curve_weight_pos_t.AddPoint(k);
			curve_weight_pos_t.SetPos((int)curve_weight_pos_t.NPoints()-1, wp.weight.pos_t);
		}
		if(is_valid(wp.weight.pos_r)){
			curve_weight_pos_r.AddPoint(k);
			curve_weight_pos_r.SetPos((int)curve_weight_pos_r.NPoints()-1, wp.weight.pos_r);
		}
		if(is_valid(wp.weight.vel_t)){
			curve_weight_vel_t.AddPoint(k);
			curve_weight_vel_t.SetPos((int)curve_weight_vel_t.NPoints()-1, wp.weight.vel_t);
		}
		if(is_valid(wp.weight.vel_r)){
			curve_weight_vel_r.AddPoint(k);
			curve_weight_vel_r.SetPos((int)curve_weight_vel_r.NPoints()-1, wp.weight.vel_r);
		}

		for(int j = 0; j < nend; j++){
			if(wp.ends.size() <= j)
				continue;

			if(is_valid(wp.ends[j].weight.pos_t)){
				curve_end_weight_pos_t[j].AddPoint(k);
				curve_end_weight_pos_t[j].SetPos((int)curve_end_weight_pos_t[j].NPoints()-1, wp.ends[j].weight.pos_t);
			}
			if(is_valid(wp.ends[j].weight.pos_r)){
				curve_end_weight_pos_r[j].AddPoint(k);
				curve_end_weight_pos_r[j].SetPos((int)curve_end_weight_pos_r[j].NPoints()-1, wp.ends[j].weight.pos_r);
			}
			if(is_valid(wp.ends[j].weight.vel_t)){
				curve_end_weight_vel_t[j].AddPoint(k);
				curve_end_weight_vel_t[j].SetPos((int)curve_end_weight_vel_t[j].NPoints()-1, wp.ends[j].weight.vel_t);
			}
			if(is_valid(wp.ends[j].weight.vel_r)){
				curve_end_weight_vel_r[j].AddPoint(k);
				curve_end_weight_vel_r[j].SetPos((int)curve_end_weight_vel_r[j].NPoints()-1, wp.ends[j].weight.vel_r);
			}
			if(is_valid(wp.ends[j].weight.stiff)){
				curve_end_weight_stiff[j].AddPoint(k);
				curve_end_weight_stiff[j].SetPos((int)curve_end_weight_stiff[j].NPoints()-1, wp.ends[j].weight.stiff);
			}
			if(is_valid(wp.ends[j].weight.cmp)){
				curve_end_weight_cmp[j].AddPoint(k);
				curve_end_weight_cmp[j].SetPos((int)curve_end_weight_cmp[j].NPoints()-1, wp.ends[j].weight.cmp);
			}
			if(is_valid(wp.ends[j].weight.moment)){
				curve_end_weight_moment[j].AddPoint(k);
				curve_end_weight_moment[j].SetPos((int)curve_end_weight_moment[j].NPoints()-1, wp.ends[j].weight.moment);
			}
		}
	}

	// interpolation of trajectory
	for(int k = 0; k <= N; k++){
		Waypoint& wp = waypoints[k];
		real_t t = curve_time.CalcPos(k);

		if(is_valid(wp.value.pos_t)){
			curve_pos_t.AddPoint(t);
			curve_pos_t.SetPos((int)curve_pos_t.NPoints()-1, wp.value.pos_t);
			curve_pos_t.SetVel((int)curve_pos_t.NPoints()-1, is_valid(wp.value.vel_t) ? wp.value.vel_t : vec3_t());
		}
		if(is_valid(wp.value.pos_r)){
			curve_pos_r.AddPoint(t);
			curve_pos_r.SetPos((int)curve_pos_r.NPoints()-1, wp.value.pos_r);
			curve_pos_r.SetVel((int)curve_pos_r.NPoints()-1, is_valid(wp.value.vel_r) ? wp.value.vel_r : vec3_t());
		}

		for(int j = 0; j < nend; j++){
			if(wp.ends.size() <= j)
				continue;

			if(is_valid(wp.ends[j].value.pos_t)){
				curve_end_pos_t[j].AddPoint(t);
				curve_end_pos_t[j].SetPos((int)curve_end_pos_t[j].NPoints()-1, wp.ends[j].value.pos_t);
				curve_end_pos_t[j].SetVel((int)curve_end_pos_t[j].NPoints()-1, is_valid(wp.ends[j].value.vel_t) ? wp.ends[j].value.vel_t : vec3_t());
			}
			if(is_valid(wp.ends[j].value.pos_r)){
				curve_end_pos_r[j].AddPoint(t);
				curve_end_pos_r[j].SetPos((int)curve_end_pos_r[j].NPoints()-1, wp.ends[j].value.pos_r);
				curve_end_pos_r[j].SetVel((int)curve_end_pos_r[j].NPoints()-1, is_valid(wp.ends[j].value.vel_r) ? wp.ends[j].value.vel_r : vec3_t());
			}
		}
	}

	for (int k = 0; k <= N; k++) {
		CentroidKey* key = (CentroidKey*)traj.GetKeypoint(graph->ticks[k]);

		// initial setting of time
		real_t t  = curve_time.CalcPos(k);
		real_t dt = curve_time.CalcVel(k);

		graph->ticks[k]->time        = t;
		key->var_time->val           = t;
		key->con_des_time->desired   = t;
		key->con_des_time->weight[0] = curve_weight_time.CalcPos(k);

		if(key->next){
			key->var_duration->val           = dt;
			key->con_des_duration->desired   = dt;
			key->con_des_duration->weight[0] = curve_weight_duration.CalcPos(k);

            key->con_duration_range[0]->bound =  param.durationMin;
		    key->con_duration_range[1]->bound = -param.durationMax;
			key->con_duration_range[0]->weight[0] = 1.0;
			key->con_duration_range[1]->weight[0] = 1.0;
			key->con_duration_range[0]->barrier_margin = 0.0001;
			key->con_duration_range[1]->barrier_margin = 0.0001;
        }
		
		vec3_t pc = curve_pos_t.CalcPos(t);
		vec3_t vc = curve_pos_t.CalcVel(t);
		quat_t qc;
		vec3_t wc;

		key->var_pos_t->val = pc;
		key->var_vel_t->val = vc;
		key->con_des_pos_t->desired = pc;
        key->con_des_vel_t->desired = vc;
		key->con_des_pos_t->weight  = curve_weight_pos_t.CalcPos(k);
		key->con_des_vel_t->weight  = curve_weight_vel_t.CalcPos(k);

		vec3_t rpy = curve_pos_r.CalcPos(t);
		wc = RpyJacobian(rpy)*curve_pos_r.CalcVel(t);
		qc = FromRollPitchYaw(rpy);

		key->var_pos_r->val    = qc;
		key->var_pos_r->locked = !param.enableRotation || !param.enableQuaternion;
		key->con_des_pos_r->desired = qc;
		key->con_des_pos_r->weight  = curve_weight_pos_r.CalcPos(k);
		
		key->var_pos_rpy->val    = rpy;
		key->var_pos_rpy->locked = !param.enableRotation || param.enableQuaternion;
		key->con_des_pos_rpy->desired = rpy;
		key->con_des_pos_rpy->weight  = curve_weight_pos_r.CalcPos(k);
		
		key->var_vel_r->val    = wc;
		key->var_vel_r->locked = !param.enableRotation;
		key->con_des_vel_r->desired = wc;
		key->con_des_vel_r->weight  = curve_weight_vel_r.CalcPos(k);

		for(int i = 0; i < nend; i++){
			vec3_t pe = curve_end_pos_t[i].CalcPos(t);
			vec3_t ve = curve_end_pos_t[i].CalcVel(t);
			quat_t qe = FromRollPitchYaw(curve_end_pos_r[i].CalcPos(t));
			vec3_t we = curve_end_pos_r[i].CalcVel(t);

			//vec3_t pe_abs = pc + qc*(pe + ends[i].basePos);
			//vec3_t ve_abs = vc + wc % (qc*(pe + ends[i].basePos)) + qc*ve;
			//quat_t qe_abs = qc*qe;
			//vec3_t we_abs = wc + qc*we;
			
			key->ends[i].var_pos_t ->val = pe;
			key->ends[i].var_vel_t ->val = (key->ends[i].iface == -1 ? ve : vec3_t());
            key->ends[i].con_des_pos_t ->desired = pe;
            key->ends[i].con_des_vel_t ->desired = (key->ends[i].iface == -1 ? ve : vec3_t());
			key->ends[i].con_des_pos_t ->weight = curve_end_weight_pos_t[i].CalcPos(k);
			key->ends[i].con_des_vel_t ->weight = (key->ends[i].iface == -1 ? curve_end_weight_vel_t[i].CalcPos(k) : 100.0*one);

			key->ends[i].var_pos_r->val = qe;
			key->ends[i].var_vel_r->val = we;
			key->ends[i].var_pos_r->locked = !param.enableRotation || ends[i].lockOri;
			key->ends[i].var_vel_r->locked = !param.enableRotation || ends[i].lockOri;
			key->ends[i].con_des_pos_r ->desired = qe;
			key->ends[i].con_des_vel_r ->desired = (key->ends[i].iface == -1 ? we : vec3_t());
			key->ends[i].con_des_pos_r ->weight = curve_end_weight_pos_r[i].CalcPos(k);
			key->ends[i].con_des_vel_r ->weight = (key->ends[i].iface == -1 ? curve_end_weight_vel_r[i].CalcPos(k) : 100*one);
			

			for(int j = 0; j < 3; j++){
				key->ends[i].con_pos_range[j][0]->weight[0] = 10.0;
				key->ends[i].con_pos_range[j][1]->weight[0] = 10.0;
				key->ends[i].con_pos_range[j][0]->barrier_margin = 0.00001;
				key->ends[i].con_pos_range[j][1]->barrier_margin = 0.00001;
			}
    		
            if(key->next){
				key->ends[i].con_stiff_range->_min = 0.0;
                key->ends[i].con_stiff_range->_max = ends[i].stiffnessMax;
				key->ends[i].con_stiff_range->weight[0] = 1.0;
				key->ends[i].con_stiff_range->barrier_margin = 0.0001;
            
				key->ends[i].con_friction    ->weight[0] = 100;
				key->ends[i].con_friction    ->barrier_margin = 0.00001;
				//key->ends[i].con_moment[0][0]->weight[0] = 1.0;
				//key->ends[i].con_moment[1][0]->weight[0] = 1.0;
				//key->ends[i].con_moment[2][0]->weight[0] = 1.0;
				//key->ends[i].con_moment[0][1]->weight[0] = 1.0;
				//key->ends[i].con_moment[1][1]->weight[0] = 1.0;
				//key->ends[i].con_moment[2][1]->weight[0] = 1.0;
				//key->ends[i].con_moment[0][0]->barrier_margin = 0.0001;
				//key->ends[i].con_moment[1][0]->barrier_margin = 0.0001;
				//key->ends[i].con_moment[2][0]->barrier_margin = 0.0001;
				//key->ends[i].con_moment[0][1]->barrier_margin = 0.0001;
				//key->ends[i].con_moment[1][1]->barrier_margin = 0.0001;
				//key->ends[i].con_moment[2][1]->barrier_margin = 0.0001;
			}

            for(int j = 0; j < nface; j++){
		        key->ends[i].con_contact[j]->weight[0] = (key->ends[i].iface == j ? 100.0 : 0.0);
            }
		}

		/*
		 min    sum li^4
		 sub.to sum li^2 (pc - pi) = ac + g

		 li^2 = (pc - pi)^T mu
		 (sum (pc - pi)(pc - pi)^T) mu = g
		 */
		vector<vec3_t>  pl(nend);
        mat3_t A = eps*mat3_t();
        for(int i = 0; i < nend; i++){
		    pl[i] = key->var_pos_t->val - key->ends[i].var_pos_t->val;
            A += vvtrmat(pl[i], pl[i]);
        }
		mat3_t Ainv = A.inv();

		for(int i = 0; i < nend; i++){
			real_t li = sqrt(std::max(0.0, pl[i]*(Ainv*vec3_t(0.0, 0.0, param.g))));
			key->ends[i].var_stiff ->val          = li;
			key->ends[i].con_des_stiff->desired   = (key->ends[i].iface == -1 ? 0.0 : li);
			key->ends[i].con_des_stiff->weight[0] = (key->ends[i].iface == -1 ? 100.0 : curve_end_weight_stiff [i].CalcPos(k));

			for(int j = 0; j < 2; j++){
				key->ends[i].var_cmp[j]->val    = ends[i].cmpOffset[j];
				key->ends[i].var_cmp[j]->locked = ends[i].lockCmp;
				key->ends[i].con_des_cmp[j]->desired   = ends[i].cmpOffset[j];
				key->ends[i].con_des_cmp[j]->weight[0] = curve_end_weight_cmp[i].CalcPos(k)[j];
			}

			key->ends[i].var_moment->val    = vec3_t();
			key->ends[i].var_moment->locked = ends[i].lockMoment;
			key->ends[i].con_des_moment->desired = vec3_t();
			key->ends[i].con_des_moment->weight  = curve_end_weight_moment[i].CalcPos(k);
		}
	}

	// normalize face vertices
    for(Face& face : faces){
        face.hull->CalcBSphere();
    }

	// call prepare here so that initial trajectory is visualized properly
    Prepare();

	graph->solver->reg_x = 10.0;
	
    trajReady = false;
}

void Centroid::Prepare() {
	trajReady = false;

	traj.Update();
	
	#pragma omp parallel for if(graph->solver->param.parallelize)
	for(int k = 0; k < traj.size(); k++){
		traj[k]->Prepare();
	}
}

void Centroid::PrepareStep() {
	#pragma omp parallel for if(graph->solver->param.parallelize)
	for(int k = 0; k < traj.size(); k++){
		traj[k]->PrepareStep();
	}
}

void Centroid::Finish(){
	TrajectoryNode::Finish();
}

void Centroid::ComState(real_t t, vec3_t& pos, vec3_t& vel, vec3_t& acc) {
	if(traj.empty())
		return;

	KeyPair      kp = traj.GetSegment(t);
	CentroidKey* k0 = (CentroidKey*)kp.first;
	CentroidKey* k1 = (CentroidKey*)kp.second;

    vec3_t pt, vt, at;

    if(k1 == k0->next){
	    real_t dt = t - k0->var_time->val;
	    real_t Ct = cosh(k0->lbar*(dt));
	    real_t St = sinh(k0->lbar*(dt));
	
        pt = k0->pbar + k0->rbar + Ct*(k0->var_pos_t->val - (k0->pbar + k0->rbar)) + (St/k0->lbar)*k0->var_vel_t->val;
	    vt = k0->lbar * St*(k0->var_pos_t->val - (k0->pbar + k0->rbar)) + Ct*k0->var_vel_t->val;
	}
    else{
        pt = k0->var_pos_t->val;
        vt = k0->var_vel_t->val;
    }

    at = (k0->lbar*k0->lbar)*(pt - (k0->pbar + k0->rbar));
	
    pos = pt;
	vel = vt;
    acc = at;
}

void Centroid::TorsoState(real_t t, quat_t& ori, vec3_t& angvel, int type) {
	if(traj.empty())
		return;

	KeyPair      kp = traj.GetSegment(t);
	CentroidKey* k0 = (CentroidKey*)kp.first;
	CentroidKey* k1 = (CentroidKey*)kp.second;
	
	/*
	if(param.enableQuaternion){
		q0 = k0->var_pos_r->val;
		q1 = k1->var_pos_r->val;
	}
	else{
		q0 = FromRollPitchYaw(vec3_t(k0->var_pos_rpy[0]->val, k0->var_pos_rpy[1]->val, k0->var_pos_rpy[2]->val));
		q1 = FromRollPitchYaw(vec3_t(k1->var_pos_rpy[0]->val, k1->var_pos_rpy[1]->val, k1->var_pos_rpy[2]->val));
	}
	*/
	if(k1 == k0->next){
		real_t t0   = k0->var_time->val;
		real_t t1   = k1->var_time->val;
		real_t tau  = k0->var_duration->val;
		int ndiv    = param.rotationResolution;
		real_t dtau = tau/(real_t)ndiv;
		int idiv = std::min(std::max(0, (int)((t - t0)/dtau)), ndiv-1);
		
		quat_t q0, q1;
		vec3_t w0, w1;
		//q0 = k0->var_pos_r->val;
		//q1 = k1->var_pos_r->val;
		//w0 = k0->var_vel_r->val;
		//w1 = k1->var_vel_r->val;
		q0 = k0->coef.q_rhs[idiv+0];
		q1 = k0->coef.q_rhs[idiv+1];
		w0 = k0->coef.w_rhs[idiv+0];
		w1 = k0->coef.w_rhs[idiv+1];

		//ori = q0;
		ori = InterpolateOri(
		    t,
		    t0 + (idiv+0)*dtau, q0, w0,
		    t0 + (idiv+1)*dtau, q1, w1,
			//t0, q0, w0,
			//t1, q1, w1,
		    type);
		
		angvel = InterpolateAngvel(
		    t,
			//t0, q0, w0,
			//t1, q1, w1,
		    t0 + (idiv+0)*dtau, q0, w0,
		    t0 + (idiv+1)*dtau, q1, w1,
		    type);
    }
    else{
		if(param.enableQuaternion)
 			 ori = k0->var_pos_r->val;
		else ori = FromRollPitchYaw(k0->var_pos_rpy->val);
        angvel = k0->var_vel_r->val;
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

    real_t (*K)[6] = (type == 3 ? Kcubic : Kquintic);

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

void Centroid::EndState(real_t t, int index, vec3_t& pos, quat_t& ori, vec3_t& vel, vec3_t& angvel) {
	if(traj.empty())
		return;

	KeyPair      kp = traj.GetSegment(t);
	CentroidKey* k0 = (CentroidKey*)kp.first;
	CentroidKey* k1 = (CentroidKey*)kp.second;

	// in flight
    if(k1 == k0->next && k0->ends[index].iface == -1){
		// find lift-off and landing phase
		while(k0->prev && ((CentroidKey*)k0->prev)->ends[index].iface == -1)
            k0 = (CentroidKey*)k0->prev;
        while(k1->next && k1->ends[index].iface == -1)
            k1 = (CentroidKey*)k1->next;

		vec3_t pc0 = k0->var_pos_t->val;
		quat_t qc0 = k0->var_pos_r->val;
		vec3_t pc1 = k1->var_pos_t->val;
		quat_t qc1 = k1->var_pos_r->val;
		vec3_t pe0_abs = k0->ends[index].var_pos_t->val;
		vec3_t pe1_abs = k1->ends[index].var_pos_t->val;
		quat_t qe0_abs = k0->ends[index].var_pos_r->val;
		quat_t qe1_abs = k1->ends[index].var_pos_r->val;
		real_t t0 = k0->var_time->val;
		real_t t1 = k1->var_time->val;

		vec3_t pe0 = qc0.Conjugated()*(pe0_abs - pc0);
		quat_t qe0 = qc0.Conjugated()*qe0_abs;
		vec3_t pe1 = qc1.Conjugated()*(pe1_abs - pc1);
		quat_t qe1 = qc1.Conjugated()*qe1_abs;
			
		const real_t _2pi = 2.0*m_pi;
		real_t tau = t1 - t0;
		real_t s   = (t - t0)/tau;
		real_t ch  = (s - sin(_2pi*s)/_2pi);
		real_t chd = ((1.0 - cos(_2pi*s))/tau);
		real_t cv  = (1 - cos(_2pi*s))/2.0;
		real_t cvd = (_2pi*sin(_2pi*s)/(2.0*tau));

		real_t sw = param.swingHeight;
		vec3_t nx = pe1 - pe0;
		vec3_t ny;
		vec3_t nz(0.0, 0.0, 1.0);
		real_t nxnorm = nx.norm();

		if(nxnorm > eps){
			nx = nx/nxnorm;
			ny = nz % nx;
			nz = nx % ny;
		}
			
		vec3_t pe = pe0 + ch *(pe1 - pe0) + (cv *sw)*nz;
		vec3_t ve =       chd*(pe1 - pe0) + (cvd*sw)*nz;

		quat_t qrel = qe0.Conjugated()*qe1;
		vec3_t axis   = qrel.Axis ();
		real_t theta  = qrel.Theta();
		if(theta > m_pi)
			theta -= 2*m_pi;
				
		quat_t qe = qe0*quat_t::Rot((ch*theta)*axis);
		vec3_t we = qe0*((chd*theta)*axis);

		vec3_t pc, vc, ac, wc;
		quat_t qc;
		
		ComState(t, pc, vc, ac);
		TorsoState(t, qc, wc);

		pos    = pc + qc*pe;
		ori    = qc*qe;
		vel    = vc + qc*ve + wc % (qc*pe);
		angvel = wc + qc*we;
	}
    else{
		pos = k0->ends[index].var_pos_t->val;
		vel = k0->ends[index].var_vel_t->val;
		ori    = k0->ends[index].var_pos_r->val;
		angvel = k0->ends[index].var_vel_r->val;
    }
}

void Centroid::EndForce(real_t t, int index, real_t& stiff, vec2_t& cmp, vec3_t& moment, bool& contact){
    KeyPair      kp = traj.GetSegment(t);
	CentroidKey* k0 = (CentroidKey*)kp.first;
	CentroidKey* k1 = (CentroidKey*)kp.second;

    stiff   = k0->ends[index].var_stiff ->val;
	cmp     = vec2_t(k0->ends[index].var_cmp[0]->val, k0->ends[index].var_cmp[1]->val);
	moment  = k0->ends[index].var_moment->val;
    contact = (k0->ends[index].iface != -1);
}

void Centroid::EndSwitchTiming(real_t t, int index, real_t& tprev, real_t& tnext){
	if(traj.empty())
		return;

	KeyPair      kp = traj.GetSegment(t);
	CentroidKey* k0 = (CentroidKey*)kp.first;
	CentroidKey* k1 = (CentroidKey*)kp.second;

	int iface = k0->ends[index].iface;

	while(k0->prev && ((CentroidKey*)k0->prev)->ends[index].iface == iface)
        k0 = (CentroidKey*)k0->prev;
    while(k1->next && k1->ends[index].iface == iface)
        k1 = (CentroidKey*)k1->next;

	tprev = k0->var_time->val;
	tnext = k1->var_time->val;
}

void Centroid::CalcTrajectory() {
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

void Centroid::Draw(Render::Canvas* canvas, Render::Config* conf) {
	TrajectoryNode::Draw(canvas, conf);

	if (!trajReady)
		CalcTrajectory();

	if (trajectory.empty())
		return;

	// pos
	if (conf->Set(canvas, Render::Item::CentroidPos, this)) {
		canvas->BeginLayer("centroid_pos", true);
		canvas->BeginPath();
		canvas->MoveTo(trajectory[0].pos);
		for (uint i = 1; i < trajectory.size(); i++) {
			canvas->LineTo(trajectory[i].pos);
		}
		canvas->EndPath();
		canvas->EndLayer();
	}

	stringstream ss;

	// end
	if (conf->Set(canvas, Render::Item::CentroidEndTraj, this)) {
		for(int i = 0; i < ends.size(); i++){
			ss.str("");
			ss << i;
            canvas->SetLineColor(i % 2 ? "blue" : "magenta");
			canvas->BeginLayer("centroid_end" + ss.str(), true);
			canvas->BeginPath();
			canvas->MoveTo(trajectory[0].ends[i].pos);
			for (int k = 1; k < trajectory.size(); k++) {
				canvas->LineTo(trajectory[k].ends[i].pos);
			}
			canvas->EndPath();
			canvas->EndLayer();
		}
	}
    
    if(conf->Set(canvas, Render::Item::CentroidFace, this)){
	    for(int i = 0; i < faces.size(); i++){
			Face& f = faces[i];
			ss.str("");
			ss << i;
			canvas->BeginLayer("centroid_face" + ss.str(), true);
		    canvas->BeginPath();
		    canvas->MoveTo(f.hull->vertices[0]);
		    canvas->LineTo(f.hull->vertices[1]);
		    canvas->LineTo(f.hull->vertices[2]);
		    canvas->LineTo(f.hull->vertices[3]);
		    canvas->LineTo(f.hull->vertices[0]);
		    canvas->EndPath();
		    canvas->EndLayer();
	    }
    }
}

void Centroid::CreateSnapshot(real_t t, Centroid::Snapshot& s){
	s.t = t;
    ComState  (t, s.pos, s.vel, s.acc);
    TorsoState(t, s.ori, s.angvel);
	
	s.ends.resize(ends.size());
	for(int i = 0; i < ends.size(); i++){
        EndState(t, i, s.ends[i].pos, s.ends[i].ori, s.ends[i].vel, s.ends[i].angvel);
        EndForce(t, i, s.ends[i].stiffness, s.ends[i].cmp, s.ends[i].moment, s.ends[i].contact);
        real_t l = s.ends[i].stiffness;
        s.ends[i].force = param.m*l*l*(s.pos - s.ends[i].pos - vec3_t(s.ends[i].cmp.x, s.ends[i].cmp.y, 0.0));
	}
}

void Centroid::CreateSnapshot(real_t t){
	CreateSnapshot(t, snapshot);
}

void Centroid::DrawSnapshot(Render::Canvas* canvas, Render::Config* conf) {
	if (conf->Set(canvas, Render::Item::CentroidEnd, this)) {
		canvas->BeginPath();
		canvas->MoveTo(snapshot.pos);
		canvas->LineTo(snapshot.pos + snapshot.ori*vec3_t(0.0, 0.0, 0.3));
		canvas->EndPath();

		for(int i = 0; i < ends.size(); i++){
			canvas->BeginLayer("centroid_end_snapshot", true);
			canvas->SetLineColor("black");
            canvas->SetLineWidth(1.0f);

			// line connecting com and end
		    canvas->BeginPath();
			canvas->MoveTo(snapshot.pos);
			canvas->LineTo(snapshot.pos + snapshot.ori*ends[i].basePos);
			canvas->LineTo(snapshot.ends[i].pos);
			canvas->EndPath();

            // line indicating force
            canvas->SetLineColor("red");
            canvas->SetLineWidth(1.0f);
			canvas->BeginPath();
			canvas->MoveTo(snapshot.ends[i].pos);
			canvas->LineTo(snapshot.ends[i].pos + 0.001*snapshot.ends[i].force);
			canvas->EndPath();
			canvas->EndLayer();
		}

        // end rectangle
        for(int i = 0; i < ends.size(); i++){
            vec3_t vtx[4];
            vtx[0] = vec3_t(ends[i].copMin.x, ends[i].copMin.y, 0.0);
            vtx[1] = vec3_t(ends[i].copMin.x, ends[i].copMax.y, 0.0);
            vtx[2] = vec3_t(ends[i].copMax.x, ends[i].copMax.y, 0.0);
            vtx[3] = vec3_t(ends[i].copMax.x, ends[i].copMin.y, 0.0);
            canvas->SetLineColor("green");
            canvas->SetLineWidth(/*snapshot.ends[i].contact ? 2.0f : */1.0f);
			canvas->BeginPath();
			canvas->MoveTo(snapshot.ends[i].pos + snapshot.ends[i].ori*vtx[0]);
			canvas->LineTo(snapshot.ends[i].pos + snapshot.ends[i].ori*vtx[1]);
			canvas->LineTo(snapshot.ends[i].pos + snapshot.ends[i].ori*vtx[2]);
			canvas->LineTo(snapshot.ends[i].pos + snapshot.ends[i].ori*vtx[3]);
			canvas->LineTo(snapshot.ends[i].pos + snapshot.ends[i].ori*vtx[0]);
			canvas->EndPath();
		}
	}	
}

///////////////////////////////////////////////////////////////////////////////////////////////////

CentroidCon::CentroidCon(Solver* solver, int _dim, int _tag, string _name, CentroidKey* _obj, real_t _scale):
	Constraint(solver, _dim, ID(_tag, _obj->node, _obj->tick, _name), Constraint::Type::Equality, _scale) {
	obj[0] = _obj;
	obj[1] = (CentroidKey*)_obj->next;
}

CentroidComCon::CentroidComCon(Solver* solver, int _dim, int _tag, string _name, CentroidKey* _obj, real_t _scale):
	CentroidCon(solver, _dim, _tag, _name, _obj, _scale) {

}

CentroidPosConT::CentroidPosConT(Solver* solver, string _name, CentroidKey* _obj, real_t _scale):
	CentroidComCon(solver, 3, ConTag::CentroidPosT, _name, _obj, _scale) {

	AddSLink (obj[1]->var_pos_t);
	AddSLink (obj[0]->var_pos_t);
	AddSLink (obj[0]->var_vel_t);
	AddC3Link(obj[0]->var_duration);
	
	int nend = (int)obj[0]->ends.size();
	for(int i = 0; i < nend; i++){
		AddC3Link(obj[0]->ends[i].var_stiff);
		AddSLink (obj[0]->ends[i].var_pos_t);
		AddSLink (obj[0]->ends[i].var_vel_t);
		AddC3Link(obj[0]->ends[i].var_cmp[0]);
		AddC3Link(obj[0]->ends[i].var_cmp[1]);
	}
}

CentroidVelConT::CentroidVelConT(Solver* solver, string _name, CentroidKey* _obj, real_t _scale):
	CentroidComCon(solver, 3, ConTag::CentroidVelT, _name, _obj, _scale) {

	AddSLink (obj[1]->var_vel_t);
	AddSLink (obj[0]->var_pos_t);
	AddSLink (obj[0]->var_vel_t);
	AddC3Link(obj[0]->var_duration);

	int nend = (int)obj[0]->ends.size();
	for(int i = 0; i < nend; i++){
		AddC3Link(obj[0]->ends[i].var_stiff);
		AddSLink (obj[0]->ends[i].var_pos_t);
		AddSLink (obj[0]->ends[i].var_vel_t);
		AddC3Link(obj[0]->ends[i].var_cmp[0]);
		AddC3Link(obj[0]->ends[i].var_cmp[1]);
	}
}

CentroidPosConR::CentroidPosConR(Solver* solver, string _name, CentroidKey* _obj, real_t _scale):
	CentroidCon(solver, 3, ConTag::CentroidPosR, _name, _obj, _scale) {

	AddSLink (obj[1]->var_pos_r);
	AddM3Link(obj[0]->var_pos_r);
	AddM3Link(obj[0]->var_vel_r);
	AddM3Link(obj[0]->var_pos_t);
	AddM3Link(obj[0]->var_vel_t);
	AddC3Link(obj[0]->var_duration);

	int nend = (int)obj[0]->ends.size();
	for(int i = 0; i < nend; i++){
		AddC3Link(obj[0]->ends[i].var_stiff );
		AddM3Link(obj[0]->ends[i].var_pos_t );
		AddM3Link(obj[0]->ends[i].var_vel_t);
		AddC3Link(obj[0]->ends[i].var_cmp[0]);
		AddC3Link(obj[0]->ends[i].var_cmp[1]);
		AddM3Link(obj[0]->ends[i].var_moment);
	}
}

CentroidPosConRPY::CentroidPosConRPY(Solver* solver, string _name, CentroidKey* _obj, real_t _scale):
	CentroidCon(solver, 3, ConTag::CentroidPosR, _name, _obj, _scale) {

	AddSLink (obj[1]->var_pos_rpy);
	AddM3Link(obj[0]->var_pos_rpy);
	AddM3Link(obj[0]->var_vel_r);
	AddM3Link(obj[0]->var_pos_t);
	AddM3Link(obj[0]->var_vel_t);
	AddC3Link(obj[0]->var_duration);

	int nend = (int)obj[0]->ends.size();
	for(int i = 0; i < nend; i++){
		AddC3Link(obj[0]->ends[i].var_stiff );
		AddM3Link(obj[0]->ends[i].var_pos_t );
		AddM3Link(obj[0]->ends[i].var_vel_t);
		AddC3Link(obj[0]->ends[i].var_cmp[0]);
		AddC3Link(obj[0]->ends[i].var_cmp[1]);
		AddM3Link(obj[0]->ends[i].var_moment);
	}
}

CentroidVelConR::CentroidVelConR(Solver* solver, string _name, CentroidKey* _obj, real_t _scale):
	CentroidCon(solver, 3, ConTag::CentroidVelR, _name, _obj, _scale) {

	AddSLink (obj[1]->var_vel_r);
	AddSLink (obj[0]->var_vel_r);
	AddM3Link(obj[0]->var_pos_t);
	AddM3Link(obj[0]->var_vel_t);
	AddC3Link(obj[0]->var_duration);	

	int nend = (int)obj[0]->ends.size();
	for(int i = 0; i < nend; i++){
		AddC3Link(obj[0]->ends[i].var_stiff );
		AddM3Link(obj[0]->ends[i].var_pos_t );
		AddM3Link(obj[0]->ends[i].var_vel_t);
		AddC3Link(obj[0]->ends[i].var_cmp[0]);
		AddC3Link(obj[0]->ends[i].var_cmp[1]);
		AddM3Link(obj[0]->ends[i].var_moment);
	}
}

CentroidTimeCon::CentroidTimeCon(Solver* solver, string _name, CentroidKey* _obj, real_t _scale):
	CentroidCon(solver, 1, ConTag::CentroidTime, _name, _obj, _scale) {
	
	AddSLink(obj[1]->var_time);
	AddSLink(obj[0]->var_time);
	AddSLink(obj[0]->var_duration);
}

CentroidEndPosConT::CentroidEndPosConT(Solver* solver, string _name, CentroidKey* _obj, int _iend, real_t _scale):
	CentroidCon(solver, 3, ConTag::CentroidEndPosT, _name, _obj, _scale) {
	iend  = _iend;

	AddSLink (obj[1]->ends[iend].var_pos_t);
	AddSLink (obj[0]->ends[iend].var_pos_t);
	AddSLink (obj[0]->ends[iend].var_vel_t);
	AddC3Link(obj[0]->var_duration);
}

CentroidEndPosConR::CentroidEndPosConR(Solver* solver, string _name, CentroidKey* _obj, int _iend, real_t _scale):
	CentroidCon(solver, 3, ConTag::CentroidEndPosR, _name, _obj, _scale) {
	iend  = _iend;

	AddSLink (obj[1]->ends[iend].var_pos_r);
	AddM3Link(obj[0]->ends[iend].var_pos_r);
	AddM3Link(obj[0]->ends[iend].var_vel_r);
	AddC3Link(obj[0]->var_duration);
}

CentroidDurationRangeCon::CentroidDurationRangeCon(Solver* solver, string _name, CentroidKey* _obj, real_t _dir, real_t _scale):
	Constraint(solver, 1, ID(ConTag::CentroidDurationRange, _obj->node, _obj->tick, _name), Constraint::Type::InequalityBarrier, _scale){
	obj  = _obj;
	dir  = _dir;

	AddSLink(obj->var_duration);
}

CentroidEndPosRangeCon::CentroidEndPosRangeCon(Solver* solver, string _name, CentroidKey* _obj, int _iend, int _idx, real_t _dir, real_t _scale):
	Constraint(solver, 1, ID(ConTag::CentroidEndPosRange, _obj->node, _obj->tick, _name), Constraint::Type::InequalityPenalty, _scale){
	obj  = _obj;
	iend = _iend;
	idx  = _idx;
	dir  = _dir;

	AddR3Link(obj->var_pos_t);
	AddR3Link(obj->var_pos_r);
	AddR3Link(obj->ends[iend].var_pos_t);
}

CentroidEndContactCon::CentroidEndContactCon(Solver* solver, string _name, CentroidKey* _obj, int _iend, int _iface, real_t _scale):
	Constraint(solver, 1, ID(ConTag::CentroidEndContact, _obj->node, _obj->tick, _name), Constraint::Type::Equality, _scale){
	obj   = _obj;
	iend  = _iend;
    iface = _iface;
    point =  obj->cen->ends [iend].point;
    face  = &obj->cen->faces[_iface];

	AddR3Link(obj->ends[iend].var_pos_t);
}

CentroidEndFrictionCon::CentroidEndFrictionCon(Solver* solver, string _name, CentroidKey* _obj, int _iend, real_t _scale):
	Constraint(solver, 1, ID(ConTag::CentroidEndFriction, _obj->node, _obj->tick, _name), Constraint::Type::InequalityPenalty, _scale){
	obj   = _obj;
	iend  = _iend;
    
	AddR3Link(obj->var_pos_t);
	AddR3Link(obj->ends[iend].var_pos_t);
	AddR3Link(obj->ends[iend].var_vel_t);
	//AddR3Link(obj->ends[iend].var_pos_r);
	AddSLink(obj->ends[iend].var_cmp[0]);
	AddSLink(obj->ends[iend].var_cmp[1]);
}

CentroidEndMomentCon::CentroidEndMomentCon(Solver* solver, string _name, CentroidKey* _obj, int _iend, int _idx, int _dir, real_t _scale):
	Constraint(solver, 1, ID(ConTag::CentroidEndMomentRange, _obj->node, _obj->tick, _name), Constraint::Type::InequalityBarrier, _scale){
	obj   = _obj;
	iend  = _iend;
    idx   = _idx;
	dir   = _dir;
    
	AddR3Link(obj->var_pos_t);
	AddR3Link(obj->ends[iend].var_pos_t );
	AddR3Link(obj->ends[iend].var_pos_r );
	AddSLink (obj->ends[iend].var_cmp[0]);
	AddSLink (obj->ends[iend].var_cmp[1]);
	AddR3Link(obj->ends[iend].var_moment);
}

///////////////////////////////////////////////////////////////////////////////////////////////////

void CentroidCon::Prepare(){
}

void CentroidComCon::Prepare(){
	CentroidCon::Prepare();

}

void CentroidPosConT::Prepare(){
	CentroidComCon::Prepare();
}

void CentroidVelConT::Prepare(){
	CentroidComCon::Prepare();
}

void CentroidPosConR::Prepare(){
	CentroidCon::Prepare();
}

void CentroidPosConRPY::Prepare(){
	CentroidCon::Prepare();
}

void CentroidVelConR::Prepare(){
	CentroidCon::Prepare();
}

void CentroidTimeCon::Prepare(){
	CentroidCon::Prepare();

	t     = obj[0]->var_time->val;
	t_lhs = obj[1]->var_time->val;
	t_rhs = t + obj[0]->tau;
}

void CentroidEndPosConT::Prepare(){
	CentroidCon::Prepare();

	pe     = obj[0]->ends[iend].var_pos_t->val;
	pe_lhs = obj[1]->ends[iend].var_pos_t->val;
	ve     = obj[0]->ends[iend].var_vel_t->val;
	pe_rhs = pe + obj[0]->tau*ve;
}

void CentroidEndPosConR::Prepare(){
	CentroidCon::Prepare();

	qe     = obj[0]->ends[iend].var_pos_r->val;
	qe_lhs = obj[1]->ends[iend].var_pos_r->val;

	we    = obj[0]->ends[iend].var_vel_r->val;
	omega = obj[0]->tau*we;
	q_omega = quat_t::Rot(omega);
	q_omega.ToMatrix(R_omega);
	A_omega = RotJacobian(omega);
	qe_rhs  = q_omega*qe;
}

void CentroidEndPosRangeCon::Prepare(){
	p       = obj->var_pos_t->val;
	q       = obj->var_pos_r->val;
	pe      = obj->ends[iend].var_pos_t->val;
	pbase   = obj->cen->ends[iend].basePos;
	
	eta.clear();
	eta[idx] = dir;
	eta_abs = q*eta;

	if(dir == 1.0)
		 bound =  obj->cen->ends[iend].posMin[idx];
	else bound = -obj->cen->ends[iend].posMax[idx];

}

void CentroidEndContactCon::Prepare(){
    pe = obj->ends[iend].var_pos_t->val;
    point->position = pe;
    point->CalcBSphere();

    vec3_t sup0, sup1;
    real_t dist = inf;
    CalcNearest(point, face->hull, pose_t(), pose_t(), sup0, sup1, dist);

	//pf = vec3_t(pe.x, pe.y, 0.0);
	//nf = vec3_t(0.0, 0.0, 1.0);
    pf = sup1;
    vec3_t d = pe - pf;
    real_t dnorm = d.norm();
    if(dnorm < 1.0e-10)
         nf = face->normal;
    else nf = d/dnorm;
}

void CentroidEndFrictionCon::Prepare(){
	/*
	 f = p - pi - ri   // ignore  m*li^2
	 mu*fz - sqrt(fx^2 + fy^2) >= 0
	*/
	f  = obj->coef.p - obj->pi[iend] - obj->ri[iend];
	ft = vec2_t(f.x, f.y);
	ftnorm = ft.norm();
	if(ftnorm < eps)
		 ftn = vec2_t();
	else ftn = ft/ftnorm;

	mu = obj->cen->param.mu;

	d = vec3_t(-ftn.x, -ftn.y, mu);
	//DSTR << "d: " << d << endl;
}

void CentroidEndMomentCon::Prepare(){
	f   = obj->coef.p - obj->pi[iend] - obj->ri[iend];
	eta = obj->etai[iend];
	Rc  = mat3_t();

	if(dir == 1.0)
		 bound = vec3_t(obj->cen->ends[iend].copMin.y, -obj->cen->ends[iend].copMax.x, -obj->cen->param.mu);
	else bound = vec3_t(obj->cen->ends[iend].copMax.y, -obj->cen->ends[iend].copMin.x,  obj->cen->param.mu);
}

///////////////////////////////////////////////////////////////////////////////////////////////////

void CentroidPosConT::CalcCoef(){
	Prepare();

	int i = 0;
	dynamic_cast<SLink *>(links[i++])->SetCoef( 1.0    );
	dynamic_cast<SLink *>(links[i++])->SetCoef(-obj[0]->coef.p_p  );
	dynamic_cast<SLink *>(links[i++])->SetCoef(-obj[0]->coef.p_v  );
	dynamic_cast<C3Link*>(links[i++])->SetCoef(-obj[0]->coef.p_tau);
	
	int nend = (int)obj[0]->ends.size();
	for(int iend = 0; iend < nend; iend++){
		dynamic_cast<C3Link*>(links[i++])->SetCoef(-obj[0]->coef.p_li[iend]);
		dynamic_cast<SLink *>(links[i++])->SetCoef(-obj[0]->coef.p_pi[iend]);
		dynamic_cast<SLink *>(links[i++])->SetCoef(-obj[0]->coef.p_vi[iend]);
		dynamic_cast<C3Link*>(links[i++])->SetCoef(-obj[0]->coef.p_ri[iend]*ex);
		dynamic_cast<C3Link*>(links[i++])->SetCoef(-obj[0]->coef.p_ri[iend]*ey);
	}
}

void CentroidVelConT::CalcCoef(){
	Prepare();

	int i = 0;
	dynamic_cast<SLink *>(links[i++])->SetCoef( 1.0    );
	dynamic_cast<SLink *>(links[i++])->SetCoef(-obj[0]->coef.v_p  .back());
	dynamic_cast<SLink *>(links[i++])->SetCoef(-obj[0]->coef.v_v  .back());
	dynamic_cast<C3Link*>(links[i++])->SetCoef(-obj[0]->coef.v_tau.back());

	int nend = (int)obj[0]->ends.size();
	int ndiv = obj[0]->cen->param.rotationResolution;
	for(int iend = 0; iend < nend; iend++){
		dynamic_cast<C3Link*>(links[i++])->SetCoef(-obj[0]->coef.v_li[iend][ndiv]);
		dynamic_cast<SLink *>(links[i++])->SetCoef(-obj[0]->coef.v_pi[iend][ndiv]);
		dynamic_cast<SLink *>(links[i++])->SetCoef(-obj[0]->coef.v_vi[iend][ndiv]);
		dynamic_cast<C3Link*>(links[i++])->SetCoef(-obj[0]->coef.v_ri[iend][ndiv]*ex);
		dynamic_cast<C3Link*>(links[i++])->SetCoef(-obj[0]->coef.v_ri[iend][ndiv]*ey);
	}
}

void CentroidPosConR::CalcCoef(){
	Prepare();

	int i = 0;
	dynamic_cast<SLink* >(links[i++])->SetCoef( 1.0);
	dynamic_cast<M3Link*>(links[i++])->SetCoef(-obj[0]->coef.q_q  );
	dynamic_cast<M3Link*>(links[i++])->SetCoef(-obj[0]->coef.q_w  );
	dynamic_cast<M3Link*>(links[i++])->SetCoef(-obj[0]->coef.q_p  );
	dynamic_cast<M3Link*>(links[i++])->SetCoef(-obj[0]->coef.q_v  );
	dynamic_cast<C3Link*>(links[i++])->SetCoef(-obj[0]->coef.q_tau);

	int nend = (int)obj[0]->ends.size();
	for(int iend = 0; iend < nend; iend++){
		dynamic_cast<C3Link*>(links[i++])->SetCoef(-obj[0]->coef.q_li  [iend]);
		dynamic_cast<M3Link*>(links[i++])->SetCoef(-obj[0]->coef.q_pi  [iend]);
		dynamic_cast<M3Link*>(links[i++])->SetCoef(-obj[0]->coef.q_vi  [iend]);
		dynamic_cast<C3Link*>(links[i++])->SetCoef(-obj[0]->coef.q_ri  [iend].col(0));
		dynamic_cast<C3Link*>(links[i++])->SetCoef(-obj[0]->coef.q_ri  [iend].col(1));
		dynamic_cast<M3Link*>(links[i++])->SetCoef(-obj[0]->coef.q_etai[iend]);
	}	
}

void CentroidPosConRPY::CalcCoef(){
	Prepare();

	int nend = (int)obj[0]->ends.size();

	int i = 0;
	dynamic_cast<SLink *>(links[i++])->SetCoef( 1.0);
	dynamic_cast<M3Link*>(links[i++])->SetCoef(-obj[0]->coef.q_q  );
	dynamic_cast<M3Link*>(links[i++])->SetCoef(-obj[0]->coef.q_w  );
	dynamic_cast<M3Link*>(links[i++])->SetCoef(-obj[0]->coef.q_p  );
	dynamic_cast<M3Link*>(links[i++])->SetCoef(-obj[0]->coef.q_v  );
	dynamic_cast<C3Link*>(links[i++])->SetCoef(-obj[0]->coef.q_tau);

	for(int iend = 0; iend < nend; iend++){
		dynamic_cast<C3Link*>(links[i++])->SetCoef(-obj[0]->coef.q_li  [iend]);
		dynamic_cast<M3Link*>(links[i++])->SetCoef(-obj[0]->coef.q_pi  [iend]);
		dynamic_cast<M3Link*>(links[i++])->SetCoef(-obj[0]->coef.q_vi  [iend]);
		dynamic_cast<C3Link*>(links[i++])->SetCoef(-obj[0]->coef.q_ri  [iend].col(0));
		dynamic_cast<C3Link*>(links[i++])->SetCoef(-obj[0]->coef.q_ri  [iend].col(1));
		dynamic_cast<M3Link*>(links[i++])->SetCoef(-obj[0]->coef.q_etai[iend]);
	}	
}

void CentroidVelConR::CalcCoef(){
	Prepare();

	int nend = (int)obj[0]->ends.size();
	int ndiv = obj[0]->cen->param.rotationResolution;

	int i = 0;
	dynamic_cast<SLink* >(links[i++])->SetCoef( 1.0);
	dynamic_cast<SLink* >(links[i++])->SetCoef(-obj[0]->coef.w_w  );
	dynamic_cast<M3Link*>(links[i++])->SetCoef(-obj[0]->coef.w_p  [ndiv]);
	dynamic_cast<M3Link*>(links[i++])->SetCoef(-obj[0]->coef.w_v  [ndiv]);
	dynamic_cast<C3Link*>(links[i++])->SetCoef(-obj[0]->coef.w_tau[ndiv]);
	
	for(int iend = 0; iend < nend; iend++){
		dynamic_cast<C3Link*>(links[i++])->SetCoef(-obj[0]->coef.w_li  [iend][ndiv]);
		dynamic_cast<M3Link*>(links[i++])->SetCoef(-obj[0]->coef.w_pi  [iend][ndiv]);
		dynamic_cast<M3Link*>(links[i++])->SetCoef(-obj[0]->coef.w_vi  [iend][ndiv]);
		dynamic_cast<C3Link*>(links[i++])->SetCoef(-obj[0]->coef.w_ri  [iend][ndiv].col(0));
		dynamic_cast<C3Link*>(links[i++])->SetCoef(-obj[0]->coef.w_ri  [iend][ndiv].col(1));
		dynamic_cast<M3Link*>(links[i++])->SetCoef(-obj[0]->coef.w_etai[iend][ndiv]);
	}	
}
void CentroidTimeCon::CalcCoef(){
	Prepare();

	((SLink*)links[0])->SetCoef( 1.0);
	((SLink*)links[1])->SetCoef(-1.0);
	((SLink*)links[2])->SetCoef(-1.0);
}

void CentroidEndPosConT::CalcCoef(){
	Prepare();

	int i = 0;
	((SLink *)links[i++])->SetCoef( 1.0);
	((SLink *)links[i++])->SetCoef(-1.0);
	((SLink *)links[i++])->SetCoef(-obj[0]->tau);
	((C3Link*)links[i++])->SetCoef(-ve);
}

void CentroidEndPosConR::CalcCoef(){
	Prepare();

	int i = 0;
	((SLink *)links[i++])->SetCoef( 1.0);
	((M3Link*)links[i++])->SetCoef(-R_omega);
	((M3Link*)links[i++])->SetCoef(-A_omega*obj[0]->tau);
	((C3Link*)links[i++])->SetCoef(-A_omega*we);
}

void CentroidDurationRangeCon::CalcCoef(){
	((SLink*)links[0])->SetCoef(dir);
}

void CentroidEndPosRangeCon::CalcCoef(){
	Prepare();

	/* 
	(q*dir)^T *(pe - (p + q*pbase)) >= bound
	(q*dir)^T *(pe - p) - dir^T*pbase >= bound
	
	(pe - p)^T*(q*dir)
	  (pe - p)^T*(Omega % dir_abs)
	= (pe - p)^T*(dir_abs^xT)*Omega
	= (dir_abs % (pe - p))^T *Omega
	*/

	((R3Link*)links[0])->SetCoef(-eta_abs);
	((R3Link*)links[1])->SetCoef( eta_abs % (pe - p));
	((R3Link*)links[2])->SetCoef( eta_abs);
}

void CentroidEndContactCon::CalcCoef(){
	Prepare();

	((R3Link*)links[0])->SetCoef(nf);
}

void CentroidEndFrictionCon::CalcCoef(){
	Prepare();

	int idx = 0;
	dynamic_cast<R3Link*>(links[idx++])->SetCoef( d);
	dynamic_cast<R3Link*>(links[idx++])->SetCoef(-d);
	dynamic_cast<R3Link*>(links[idx++])->SetCoef(-d*(obj->tau/2.0));
	//((R3Link*)links[idx++])->SetCoef( d % f);
	dynamic_cast<SLink*>(links[idx++])->SetCoef(-d.x);
	dynamic_cast<SLink*>(links[idx++])->SetCoef(-d.y);
}

void CentroidEndMomentCon::CalcCoef(){
	Prepare();

	int idx = 0;
	dynamic_cast<R3Link*>(links[idx++])->SetCoef(-dir*bound[idx]*Rc.col(2));
	dynamic_cast<R3Link*>(links[idx++])->SetCoef( dir*bound[idx]*Rc.col(2));
	dynamic_cast<R3Link*>(links[idx++])->SetCoef( dir*(Rc.col(idx) % eta - bound[idx]*(Rc.col(2) % f)));
	dynamic_cast<SLink *>(links[idx++])->SetCoef( dir*bound[idx]*Rc.col(2)[0]);
	dynamic_cast<SLink *>(links[idx++])->SetCoef( dir*bound[idx]*Rc.col(2)[1]);
	dynamic_cast<R3Link*>(links[idx++])->SetCoef( dir*Rc.col(idx));
}

///////////////////////////////////////////////////////////////////////////////////////////////////

void CentroidPosConT::CalcDeviation(){
	y = obj[0]->coef.p_lhs - obj[0]->coef.p_rhs;
}

void CentroidPosConR::CalcDeviation(){
	quat_t qerror = obj[0]->coef.q_rhs.back().Conjugated()*obj[0]->coef.q_lhs;
	vec3_t axis   = qerror.Axis ();
	real_t theta  = qerror.Theta();
	if(theta > m_pi)
		theta -= 2*m_pi;
	y = (1.0/2.0)*(obj[0]->coef.q_rhs.back()*(theta*axis) + obj[0]->coef.q_lhs*(theta*axis));
	//y = obj[0]->coef[rev].q_rhs.back()*(theta*axis);
	//y.clear();
	//y = obj[0]->coef[rev].q_lhs - obj[0]->coef[rev].q_rhs;
}

void CentroidPosConRPY::CalcDeviation(){
	y = obj[0]->coef.rpy_lhs - obj[0]->coef.rpy_rhs;
}

void CentroidVelConT::CalcDeviation(){
	y = obj[0]->coef.v_lhs - obj[0]->coef.v_rhs.back();
}

void CentroidVelConR::CalcDeviation(){
	y = obj[0]->coef.w_lhs - obj[0]->coef.w_rhs.back();
	//y.clear();
}
void CentroidTimeCon::CalcDeviation(){
	y[0] = t_lhs - t_rhs;
}
void CentroidEndPosConT::CalcDeviation(){
	y = pe_lhs - pe_rhs;
}

void CentroidEndPosConR::CalcDeviation(){
	quat_t qerror = qe_rhs.Conjugated()*qe_lhs;
	vec3_t axis   = qerror.Axis ();
	real_t theta  = qerror.Theta();
	if(theta > m_pi)
		theta -= 2*m_pi;
	y = (1.0/2.0)*(qe_rhs*(theta*axis) + qe_lhs*(theta*axis));
	//y = qe_lhs - qe_rhs;
}

void CentroidDurationRangeCon::CalcDeviation(){
    y[0] = dir*obj->var_duration->val - bound;

	// set activity if penalty mode
	//active = (y[0] < 0.0);
}

void CentroidEndPosRangeCon::CalcDeviation(){
    y[0] = eta_abs*(pe - (p + q*pbase)) - bound;
	//y[0] = dir*q - bound;
	
	// set activity if penalty mode
	active = (y[0] < 0.0);

	//if(y[0] < 0.0){
	//	DSTR << "prc: " << y[0] << endl;
	//}
}

void CentroidEndContactCon::CalcDeviation(){
	y[0] = nf*(pe - pf) - obj->cen->param.contactMargin;
}

void CentroidEndFrictionCon::CalcDeviation(){
	y[0] = mu*f.z - ftnorm;
	active = (obj->ends[iend].iface != -1 && y[0] < 0.0);
}

void CentroidEndMomentCon::CalcDeviation(){
	y[0] = dir*(eta[idx] - bound[idx]*f.z);
}

}
