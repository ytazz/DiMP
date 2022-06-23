#include <DiMP/Graph/Centroid.h>
#include <DiMP/Graph/Graph.h>
#include <DiMP/Render/Config.h>
#include <DiMP/Render/Canvas.h>

#include <iomanip>


namespace DiMP {;

const real_t pi      = M_PI;
const real_t inf     = numeric_limits<real_t>::max();
const real_t eps     = 1.0e-10;
const real_t damping = 1.0;
const vec3_t one(1.0, 1.0, 1.0);

//-------------------------------------------------------------------------------------------------
// CentroidKey::End

//-------------------------------------------------------------------------------------------------
// CentroidKey

CentroidKey::CentroidKey() {
	
}

void CentroidKey::AddVar(Solver* solver) {
	cen = (Centroid*)node;
	int nend = cen->ends.size();
	
	// position and velocity
	var_pos_t    = new V3Var(solver, ID(VarTag::CentroidTP      , node, tick, name + "_tp"      ), node->graph->scale.pos_t);
	var_pos_r    = new QVar (solver, ID(VarTag::CentroidRP      , node, tick, name + "_rp"      ), node->graph->scale.pos_r);
	var_vel_t    = new V3Var(solver, ID(VarTag::CentroidTV      , node, tick, name + "_tv"      ), node->graph->scale.vel_t);
	var_vel_r    = new V3Var(solver, ID(VarTag::CentroidRV      , node, tick, name + "_rv"      ), node->graph->scale.vel_r);
	var_time     = new SVar (solver, ID(VarTag::CentroidTime    , node, tick, name + "_time"    ), 1.0);
	var_duration = new SVar (solver, ID(VarTag::CentroidDuration, node, tick, name + "_duration"), 1.0);
	var_pos_t   ->weight    = damping*one;
	var_pos_r   ->weight    = damping*one;
	var_vel_t   ->weight    = damping*one;
	var_vel_r   ->weight    = damping*one;
	var_time    ->weight[0] = damping;
	var_duration->weight[0] = damping;

    solver->AddStateVar(var_pos_t, tick->idx);
	solver->AddStateVar(var_pos_r, tick->idx);
	solver->AddStateVar(var_vel_t, tick->idx);
	solver->AddStateVar(var_vel_r, tick->idx);
	solver->AddStateVar(var_time , tick->idx);

    solver->AddInputVar(var_duration, tick->idx);
	
	ends.resize(nend);
	stringstream ss, ss2;
	for(int i = 0; i < nend; i++){
		ends[i].key = this;

		//int njnt = cen->ends[i].jointPosRange.size();

		ss.str("");
		ss << name << "_end" << i;
		//ends[i].var_joint_pos.resize(njnt);
		//ends[i].var_joint_vel.resize(njnt);
		for(int j = 0; j < 3; j++){
			ss2.str("");
			ss2 << j;
			ends[i].var_joint_pos[j] = new SVar(solver, ID(VarTag::CentroidEndPos, node, tick, ss.str() + "_pos_local" + ss2.str()), node->graph->scale.pos_t);
			ends[i].var_joint_vel[j] = new SVar(solver, ID(VarTag::CentroidEndVel, node, tick, ss.str() + "_vel_local" + ss2.str()), node->graph->scale.vel_t);

			ends[i].var_joint_pos[j]->weight[0] = damping;
			ends[i].var_joint_vel[j]->weight[0] = damping;

			solver->AddStateVar(ends[i].var_joint_pos[j], tick->idx);
			solver->AddInputVar(ends[i].var_joint_vel[j], tick->idx);
		}
		ends[i].var_pos    = new V3Var(solver, ID(VarTag::CentroidEndPos   , node, tick, ss.str() + "_pos_abs"  ), node->graph->scale.pos_t);
		ends[i].var_vel    = new V3Var(solver, ID(VarTag::CentroidEndVel   , node, tick, ss.str() + "_vel_abs"  ), node->graph->scale.vel_t);
		ends[i].var_stiff  = new SVar (solver, ID(VarTag::CentroidEndStiff , node, tick, ss.str() + "_stiff"    ), 1.0);
		ends[i].var_moment = new V3Var(solver, ID(VarTag::CentroidEndMoment, node, tick, ss.str() + "_moment"   ), 1.0);
		
		ends[i].var_pos   ->weight    = damping*one;
		ends[i].var_vel   ->weight    = damping*one;
		ends[i].var_stiff ->weight[0] = damping;
		ends[i].var_moment->weight    = damping*one;

        ends[i].subst_pos    = solver->AddStateVar(ends[i].var_pos   , tick->idx);
        ends[i].subin_vel    = solver->AddInputVar(ends[i].var_vel   , tick->idx);
        ends[i].subin_stiff  = solver->AddInputVar(ends[i].var_stiff , tick->idx);
        ends[i].subin_moment = solver->AddInputVar(ends[i].var_moment, tick->idx);
	}
}

void CentroidKey::AddCon(Solver* solver) {
	CentroidKey* nextObj = (CentroidKey*)next;

    int nend  = cen->ends .size();
    int nface = cen->faces.size();

    if(next){
		con_pos_t = new CentroidPosConT(solver, name + "_pos_t", this, node->graph->scale.pos_t);
		con_pos_r = new CentroidPosConR(solver, name + "_pos_r", this, node->graph->scale.pos_r);
		con_vel_t = new CentroidVelConT(solver, name + "_vel_t", this, node->graph->scale.vel_t);
		con_vel_r = new CentroidVelConR(solver, name + "_vel_r", this, node->graph->scale.vel_r);
		//con_acc_t = new CentroidAccConT(solver, name + "_acc_t", this, node->graph->scale.acc_t);
		con_time  = new CentroidTimeCon(solver, name + "_time" , this, 1.0);
		
		con_duration_range = new RangeConS (solver, ID(ConTag::CentroidTime, node, tick, name + "_duration" ), var_duration, 1.0);
        
        solver->AddTransitionCon(con_pos_t, tick->idx);
        solver->AddTransitionCon(con_pos_r, tick->idx);
        solver->AddTransitionCon(con_vel_t, tick->idx);
        solver->AddTransitionCon(con_vel_r, tick->idx);
        solver->AddTransitionCon(con_time , tick->idx);

        solver->AddCostCon(con_duration_range, tick->idx);
        //solver->AddCostCon(con_acc_t         , tick->idx);
    }

    con_des_pos_t = new FixConV3(solver, ID(ConTag::CentroidPosT, node, tick, name + "_des_pos_t"), var_pos_t, 1.0);
    con_des_pos_r = new FixConQ (solver, ID(ConTag::CentroidPosR, node, tick, name + "_des_pos_r"), var_pos_r, 1.0);
	con_des_vel_t = new FixConV3(solver, ID(ConTag::CentroidVelT, node, tick, name + "_des_vel_t"), var_vel_t, 1.0);

	solver->AddCostCon(con_des_pos_t, tick->idx);
    solver->AddCostCon(con_des_pos_r, tick->idx);
	solver->AddCostCon(con_des_vel_t, tick->idx);
    
	stringstream ss;
	for(int i = 0; i < nend; i++){
		ss.str("");
		ss << name << "_end" << i;

		//int njnt = cen->ends[i].jointPosRange.size();
		
        /// non-complementary constraints
		if(next){
			//ends[i].con_joint.resize(njnt);
			for(int j = 0; j < 3; j++){
				ends[i].con_joint[j] = new CentroidJointPosCon(solver, ss.str() + "_pos", this, i, j, 1.0);
				solver->AddTransitionCon(ends[i].con_joint[j], tick->idx);
			}

            ends[i].con_pos = new CentroidEndPosCon(solver, ss.str() + "_pos", this, i, 1.0);
            solver->AddTransitionCon(ends[i].con_pos, tick->idx);

            ends[i].con_stiff_range     = new RangeConS (solver, ID(ConTag::CentroidEndStiff , node, tick, name + "_stiff_range" ), ends[i].var_stiff, 1.0);		
		    ends[i].con_moment_range[0] = new RangeConV3(solver, ID(ConTag::CentroidEndMoment, node, tick, name + "_mom_range0"), ends[i].var_moment, vec3_t(1.0, 0.0, 0.0), 1.0);
		    ends[i].con_moment_range[1] = new RangeConV3(solver, ID(ConTag::CentroidEndMoment, node, tick, name + "_mom_range1"), ends[i].var_moment, vec3_t(0.0, 1.0, 0.0), 1.0);
		    ends[i].con_moment_range[2] = new RangeConV3(solver, ID(ConTag::CentroidEndMoment, node, tick, name + "_mom_range2"), ends[i].var_moment, vec3_t(0.0, 0.0, 1.0), 1.0);

            //ends[i].con_vel_range[0][0] = new CentroidEndVelRangeCon(solver, ss.str() + "_vel_range0_min", this, i, vec3_t( 1.0,  0.0,  0.0), node->graph->scale.vel_t);
            //ends[i].con_vel_range[0][1] = new CentroidEndVelRangeCon(solver, ss.str() + "_vel_range0_max", this, i, vec3_t(-1.0,  0.0,  0.0), node->graph->scale.vel_t);
		    //ends[i].con_vel_range[1][0] = new CentroidEndVelRangeCon(solver, ss.str() + "_vel_range1_min", this, i, vec3_t( 0.0,  1.0,  0.0), node->graph->scale.vel_t);
		    //ends[i].con_vel_range[1][1] = new CentroidEndVelRangeCon(solver, ss.str() + "_vel_range1_max", this, i, vec3_t( 0.0, -1.0,  0.0), node->graph->scale.vel_t);
		    //ends[i].con_vel_range[2][0] = new CentroidEndVelRangeCon(solver, ss.str() + "_vel_range2_min", this, i, vec3_t( 0.0,  0.0,  1.0), node->graph->scale.vel_t);
		    //ends[i].con_vel_range[2][1] = new CentroidEndVelRangeCon(solver, ss.str() + "_vel_range2_max", this, i, vec3_t( 0.0,  0.0, -1.0), node->graph->scale.vel_t);

			solver->AddCostCon(ends[i].con_stiff_range    , tick->idx);
            solver->AddCostCon(ends[i].con_moment_range[0], tick->idx);
            solver->AddCostCon(ends[i].con_moment_range[1], tick->idx);
            solver->AddCostCon(ends[i].con_moment_range[2], tick->idx);

            //subcost_u.push_back(solver->AddCostCon(ends[i].con_vel_range[0][0], tick->idx));
            //subcost_u.push_back(solver->AddCostCon(ends[i].con_vel_range[0][1], tick->idx));
            //subcost_u.push_back(solver->AddCostCon(ends[i].con_vel_range[1][0], tick->idx));
            //subcost_u.push_back(solver->AddCostCon(ends[i].con_vel_range[1][1], tick->idx));
            //subcost_u.push_back(solver->AddCostCon(ends[i].con_vel_range[2][0], tick->idx));
            //subcost_u.push_back(solver->AddCostCon(ends[i].con_vel_range[2][1], tick->idx));
        }
	
		//ends[i].con_joint_pos_range.resize(njnt);
		for(int j = 0; j < 3; j++){
			//ends[i].con_joint_pos_range[j] = new RangeConS(solver, ID(ConTag::CentroidEndPos, node, tick, name + "_joint_pos_range"), ends[i].var_joint_pos[j], 1.0);
			ends[i].con_joint_pos_range[j][0] = new CentroidJointPosRangeCon(solver, name + "_joint_pos_range", this, i, j,  1.0, 1.0);
			ends[i].con_joint_pos_range[j][1] = new CentroidJointPosRangeCon(solver, name + "_joint_pos_range", this, i, j, -1.0, 1.0);
			solver->AddCostCon(ends[i].con_joint_pos_range[j][0], tick->idx);
			solver->AddCostCon(ends[i].con_joint_pos_range[j][1], tick->idx);
		}
        //ends[i].con_pos_range[0][0] = new CentroidEndPosRangeCon(solver, ss.str() + "_pos_range0_min", this, i, vec3_t( 1.0,  0.0,  0.0), node->graph->scale.pos_t);
        //ends[i].con_pos_range[0][1] = new CentroidEndPosRangeCon(solver, ss.str() + "_pos_range0_max", this, i, vec3_t(-1.0,  0.0,  0.0), node->graph->scale.pos_t);
		//ends[i].con_pos_range[1][0] = new CentroidEndPosRangeCon(solver, ss.str() + "_pos_range1_min", this, i, vec3_t( 0.0,  1.0,  0.0), node->graph->scale.pos_t);
		//ends[i].con_pos_range[1][1] = new CentroidEndPosRangeCon(solver, ss.str() + "_pos_range1_max", this, i, vec3_t( 0.0, -1.0,  0.0), node->graph->scale.pos_t);
		//ends[i].con_pos_range[2][0] = new CentroidEndPosRangeCon(solver, ss.str() + "_pos_range2_min", this, i, vec3_t( 0.0,  0.0,  1.0), node->graph->scale.pos_t);
		//ends[i].con_pos_range[2][1] = new CentroidEndPosRangeCon(solver, ss.str() + "_pos_range2_max", this, i, vec3_t( 0.0,  0.0, -1.0), node->graph->scale.pos_t);
        
        //solver->AddCostCon(ends[i].con_pos_range[0][0], tick->idx);
        //solver->AddCostCon(ends[i].con_pos_range[0][1], tick->idx);
        //solver->AddCostCon(ends[i].con_pos_range[1][0], tick->idx);
        //solver->AddCostCon(ends[i].con_pos_range[1][1], tick->idx);
        //solver->AddCostCon(ends[i].con_pos_range[2][0], tick->idx);
        //solver->AddCostCon(ends[i].con_pos_range[2][1], tick->idx);

		ends[i].con_kin     = new CentroidEndKinematicsCon(solver, name + "_kin", this, i, 1.0);
        ends[i].con_des_pos = new FixConV3(solver, ID(ConTag::CentroidEndPos, node, tick, name + "_des_pos"), ends[i].var_pos, 1.0);
		ends[i].con_des_vel = new FixConV3(solver, ID(ConTag::CentroidEndVel, node, tick, name + "_des_vel"), ends[i].var_vel, 1.0);

		solver->AddCostCon(ends[i].con_kin    , tick->idx);
		solver->AddCostCon(ends[i].con_des_pos, tick->idx);
		solver->AddCostCon(ends[i].con_des_vel, tick->idx);

        /// complimentary constraints
        if(next){
			//ends[i].con_vel_zero    = new CentroidEndVelZeroCon(solver, name + "_vel_zero", this, i, 1.0);
            ends[i].con_vel_zero    = new FixConV3(solver, ID(ConTag::CentroidEndVel   , node, tick, name + "_vel_zero"   ), ends[i].var_vel   , 1.0);
            ends[i].con_stiff_zero  = new FixConS (solver, ID(ConTag::CentroidEndStiff , node, tick, name + "_stiff_zero" ), ends[i].var_stiff , 1.0);
            ends[i].con_moment_zero = new FixConV3(solver, ID(ConTag::CentroidEndMoment, node, tick, name + "_moment_zero"), ends[i].var_moment, 1.0);

            solver->AddCostCon(ends[i].con_vel_zero   , tick->idx);
            solver->AddCostCon(ends[i].con_stiff_zero , tick->idx);
            solver->AddCostCon(ends[i].con_moment_zero, tick->idx);

			//ends[i].con_joint_vel_zero.resize(njnt);
			for(int j = 0; j < 3; j++){
				ends[i].con_joint_vel_zero[j] = new FixConS(solver, ID(ConTag::CentroidEndVel   , node, tick, name + "_joint_vel_zero"   ), ends[i].var_joint_vel[j]   , 1.0);
				solver->AddCostCon(ends[i].con_joint_vel_zero[j], tick->idx);
			}
		}
        
        ends[i].con_contact.resize(nface);
        for(int j = 0; j < nface; j++){
		    ends[i].con_contact[j] = new CentroidEndContactCon(solver, name + "_contact", this, i, j, 1.0);
            
			solver->AddCostCon(ends[i].con_contact[j], tick->idx);
        }
        
	}
}		

void CentroidKey::Prepare() {
    const real_t eps2 = 1.0e-03;

    // to avoid singularity in flight phase
    real_t l2sum = eps2*eps2;
	vec3_t ez(0.0, 0.0, 1.0);
	vec3_t psum = eps2*eps2*var_pos_t->val;
	vec3_t msum;

	for(End& end : ends){
		real_t le  = end.var_stiff ->val;
        vec3_t me  = end.var_moment->val;
		vec3_t pe  = end.var_pos   ->val;

		real_t le2 = le*le;

		l2sum += le2;
		psum  += le2*pe;
		msum  += me;
	}

	p   = var_pos_t->val;
	q   = var_pos_r->val;
	v   = var_vel_t->val;
	w   = var_vel_r->val;
	tau = var_duration->val;
	//q.ToMatrix(R);

	lbar = sqrt(l2sum);
	pave = psum/l2sum;
    pbar = (psum + ez)/l2sum;

	C = cosh(lbar*tau);
	S = sinh(lbar*tau);

	k_p_p    =  C;
	k_p_v    =  S/lbar;
	k_p_tau  =  lbar*S*(p - pbar) + C*v;
	k_p_pbar =  (1.0 - C);
	k_p_lbar =  tau*S*(p - pbar) + ((lbar*tau*C - S)/l2sum)*v;
	
	k_v_p    =  lbar*S;
	k_v_v    =  C;
	k_v_tau  =  l2sum*C*(p - pbar) + lbar*S*v;
	k_v_pbar = -lbar*S;
	k_v_lbar =  (lbar*tau*C + S)*(p - pbar) + tau*S*v;
	
	for(End& end : ends){
		real_t le  = end.var_stiff->val;
		real_t le2 = le*le;
		vec3_t pe  = end.var_pos->val;

		end.k_pbar_pe = le2/l2sum;
		end.k_pbar_le = (2.0*le*(pe - pbar))/(l2sum);
		end.k_lbar_le = le/lbar;
	}

	p_rhs = pbar + C*(p - pbar) + (S/lbar)*v;
	v_rhs = lbar*S*(p - pbar) + C*v;
	
	q_rhs = q*quat_t::Rot(q.Conjugated()*(w*tau));
	w_rhs = w + msum*tau;
}

void CentroidKey::Finish(){
	tick->time = var_time->val*cen->T;
	
	real_t dmax = 0.0;
	for(int i = 0; i < ends.size(); i++){
		End& end = ends[i];
		//int njnt = end.var_joint_pos.size();
		for(int j = 0; j < 3; j++){
			dmax = std::max(dmax,   end.var_joint_pos[j]->val*cen->L - cen->ends[i].jointPosMax[j] );
			dmax = std::max(dmax, -(end.var_joint_pos[j]->val*cen->L - cen->ends[i].jointPosMin[j]));
			//DSTR << i << " " << j << " " << end.var_joint_pos[j]->val << endl;
			//end.var_joint_pos[j]->val = std::max(cen->ends[i].jointPosMin[j]/cen->L, end.var_joint_pos[j]->val);
			//end.var_joint_pos[j]->val = std::min(cen->ends[i].jointPosMax[j]/cen->L, end.var_joint_pos[j]->val);
			//end.var_joint_vel[j]->val = std::max(cen->ends[i].jointVelMin[j]/cen->V, end.var_joint_vel[j]->val);
			//end.var_joint_vel[j]->val = std::min(cen->ends[i].jointVelMax[j]/cen->V, end.var_joint_vel[j]->val);
		}
	}
	DSTR << "max violation: " << dmax << endl;
}

void CentroidKey::Draw(Render::Canvas* canvas, Render::Config* conf) {
	Vec3f p;

	canvas->SetPointSize(5.0f);
	canvas->SetLineWidth(1.0f);

	p = var_pos_t->val*cen->L;
	canvas->Point(p);
}

//-------------------------------------------------------------------------------------------------
// Centroid

Centroid::Param::Param() {
	g  = 9.8;
	m  = 1.0;
	I  = 1.0;

    complWeightMin  =  0.1;
    complWeightMax  =  1.0;
    complWeightRate = 10.0;

	durationMin = 0.1;
	durationMax = 1.0;

    bodyRangeMin = vec3_t(-0.1, -0.1, -0.1);
    bodyRangeMax = vec3_t( 0.1,  0.1,  0.1);

    swingSlope  = 1.0;
    swingHeight = 0.05;
}

//-------------------------------------------------------------------------------------------------

Centroid::End::End(){
    stiffnessMax    = 1.0;
}

//-------------------------------------------------------------------------------------------------

Centroid::Face::Face(){

}

//-------------------------------------------------------------------------------------------------

Centroid::Waypoint::Waypoint() {
	k    = 0;
	time = 0.0;
}

Centroid::Waypoint::Waypoint(
	int _k, 
	real_t _time, vec3_t _pos_t, quat_t _pos_r, vec3_t _vel_t, vec3_t _vel_r)
{
	k         = _k;
	time      = _time;
	pos_t     = _pos_t;
	pos_r     = _pos_r;
	vel_t     = _vel_t;
	vel_r     = _vel_r;
}

Centroid::Waypoint::End::End(){

}
			
Centroid::Waypoint::End::End(vec3_t _pos, vec3_t _vel){
	pos     = _pos;
	vel     = _vel;
}

//-------------------------------------------------------------------------------------------------

Centroid::Snapshot::Snapshot() {
	t = 0.0;
}

//-------------------------------------------------------------------------------------------------

CentroidDDPState::CentroidDDPState(Centroid* _cen, CustomSolver* _solver):DDPState(_solver){
	cen = _cen;
}

CentroidDDPState::~CentroidDDPState(){

}

bool CentroidDDPState::IsIdentical(const DDPState* _st){
	const CentroidDDPState* st = (const CentroidDDPState*)_st;
	return contact == st->contact;
}

bool CentroidDDPState::IsTerminal(){
	int nend = cen->ends.size();

	bool ret = true;
	for(int i = 0; i < nend; i++){
		ret &= (contact[i] == cen->ends[i].contactTerminal);
	}

	return ret;
}

void CentroidDDPState::CalcCost(){
	int k = stage->k;
	CentroidKey* key = (CentroidKey*)cen->traj.GetKeypoint(cen->graph->ticks[k]);

	L = solver->L[k];
	vec_copy(solver->Lx [k], Lx );
	mat_copy(solver->Lxx[k], Lxx);
	if(k < solver->N){
		vec_copy(solver->Lu [k], Lu );
		mat_copy(solver->Luu[k], Luu);
		mat_copy(solver->Lux[k], Lux);
	}

	int nend  = cen->ends .size();
	int nface = cen->faces.size();

	real_t w  = cen->complWeight;
	real_t w2 = w*w;

	for(int i = 0; i < nend; i++){
		CentroidKey::End& end = key->ends[i];
		
		if(contact[i] == -1){
			if(k < solver->N){
				real_t l  = end.var_stiff ->val;
				vec3_t m  = end.var_moment->val;
				int    il = end.subin_stiff ->index;
				int    im = end.subin_moment->index;
			
				L += (1.0/2.0)*w2*(l*l + m.square());

				Lu (il)    += w2*l;
				Luu(il,il) += w2;

				for(int j = 0; j < 3; j++){
					Lu (im+j)      += w2*m[j];
					Luu(im+j,im+j) += w2;
				}
			}
		}
		else{
			if(k < solver->N){				
				vec3_t ve  = end.var_vel->val;
				int    ive = end.subin_vel->index;
			
				L += (1.0/2.0)*w2*ve.square();

				for(int j = 0; j < 3; j++){
					Lu (ive+j)       += w2*ve[j];
					Luu(ive+j,ive+j) += w2;
				}
			}
			
			real_t d   = end.con_contact[contact[i]]->y[0];
			vec3_t nf  = end.con_contact[contact[i]]->nf;
			int    ipe = end.subst_pos->index;

			L += (1.0/2.0)*w2*(d*d);
			
			for(int j = 0; j < 3; j++){
				Lx(ipe+j)       += w2*d*nf[j];
			}
			for(int j0 = 0; j0 < 3; j0++)for(int j1 = 0; j1 < 3; j1++){
				Lxx(ipe+j0,ipe+j1) += w2*nf[j0]*nf[j1];
			}
		}
	}

	DDPState::CalcCost();
}

void CentroidDDPState::Finish(){
	int k = stage->k;
	CentroidKey* key = (CentroidKey*)cen->traj.GetKeypoint(cen->graph->ticks[k]);

	int nend = cen->ends.size();
	for(int i = 0; i < nend; i++)
		key->ends[i].iface = contact[i];

}

void CentroidDDPState::Print(){
	int nend = cen->ends.size();
	for(int i = 0; i < nend; i++)
		DSTR << contact[i] << " ";
	DSTR << endl;
}

//-------------------------------------------------------------------------------------------------

Centroid::Centroid(Graph* g, string n) :TrajectoryNode(g, n) {
	type = Type::Object;
	graph->centroids.Add(this);
}

Centroid::~Centroid() {
	graph->centroids.Remove(this);
}

void Centroid::Init() {
	TrajectoryNode::Init();

	// calc normalized params
	L = sqrt(param.I/param.m);
	T = sqrt(L/param.g);
	V = L/T;
    A = L/(T*T);
	F = param.m*param.g;
	M = F*L;
    S = sqrt(F/L);

    int nend  = ends .size();
    int nface = faces.size();

	// end effector range
	for (int k = 0; k < graph->ticks.size(); k++) {
		CentroidKey* key = (CentroidKey*)traj.GetKeypoint(graph->ticks[k]);

		for(int i = 0; i < nend; i++){
			//int njnt = ends[i].jointPosRange.size();

			for(int j = 0; j < 3; j++){
				key->ends[i].con_joint_pos_range[j][0]->bound =  ends[i].jointPosMin[j]/L;
				key->ends[i].con_joint_pos_range[j][1]->bound = -ends[i].jointPosMax[j]/L;
			}

    		for(int j = 0; j < 3; j++){
				//key->ends[i].con_pos_range[j][0]->bound =  ends[i].posRangeMin[j]/L;
				//key->ends[i].con_pos_range[j][1]->bound = -ends[i].posRangeMax[j]/L;

                if(key->next){
				    key->ends[i].con_moment_range[j]->_min = ends[i].momentRangeMin[j]/M;
				    key->ends[i].con_moment_range[j]->_max = ends[i].momentRangeMax[j]/M;
				    
                    //key->ends[i].con_vel_range[j][0]->bound =  ends[i].velRangeMin[j]/V;
				    //key->ends[i].con_vel_range[j][1]->bound = -ends[i].velRangeMax[j]/V;
                }
			}
		}
	}

	// initialize position and velocity values by spline curve connecting the waypoints
	Curve3d          curve_t;
	QuatCurved       curve_r;
	vector<Curve3d>  curve_end;
	
	curve_t.SetType(Interpolate::Cubic);
	curve_r.SetType(Interpolate::SlerpDiff);
	curve_end.resize(ends.size());

    for(int j = 0; j < nend; j++){
		curve_end[j].SetType(Interpolate::Cubic);
	}

	for (uint i = 0; i < waypoints.size(); i++) {
		Waypoint& wp = waypoints[i];

		curve_t.AddPoint(wp.time);
		curve_t.SetPos(i, wp.pos_t);
		curve_t.SetVel(i, wp.vel_t);

		curve_r.AddPoint(wp.time);
		curve_r.SetPos(i, wp.pos_r);
		curve_r.SetVel(i, wp.vel_r);

		for(int j = 0; j < nend; j++){
			curve_end[j].AddPoint(wp.time);
			curve_end[j].SetPos(i, wp.ends[j].pos);
			curve_end[j].SetVel(i, wp.ends[j].vel);
		}
	}

	// initial setting of time
	real_t dt = waypoints.back().time/(real_t)(graph->ticks.size() - 1);
	for (uint k = 0; k < graph->ticks.size(); k++) {
		graph->ticks[k]->time = k*dt;
	}
	
	for (uint k = 0; k < graph->ticks.size(); k++) {
		CentroidKey* key = (CentroidKey*)traj.GetKeypoint(graph->ticks[k]);
		real_t t = graph->ticks[k]->time;

		vec3_t pt = curve_t.CalcPos(t);
		quat_t qt = curve_r.CalcPos(t);
		vec3_t vt = curve_t.CalcVel(t);
		vec3_t wt = curve_r.CalcVel(t);

		key->var_pos_t->val = pt/L;
		key->var_pos_r->val = qt;
		key->var_vel_t->val = vt/V;
		key->var_vel_r->val = wt/(1/T);

        //if(k == 0){
        //    key->var_pos_t->locked = true;
        //    key->var_vel_t->locked = true;
        //}

        // rotation is fixed at the moment
        key->var_pos_r->locked = true;
	    key->var_vel_r->locked = true;

        key->con_des_pos_t->desired = pt/L;
        key->con_des_pos_r->desired = qt;
        key->con_des_vel_t->desired = vt/V;
		for(int j = 0; j < 3; j++){
			key->con_des_pos_t->weight[j] = (!key->prev ? 100.0 : (key->next ? 0.1 : 100.0));
			key->con_des_pos_r->weight[j] = (!key->prev ? 100.0 : (key->next ? 0.1 : 100.0));
			key->con_des_vel_t->weight[j] = (!key->prev ? 100.0 : (key->next ? 1.0 : 100.0));
		}

		key->var_time->val = t/T;
		if(key->next){
			key->var_duration->val = (key->next->tick->time - t)/T;

			//key->con_acc_t->weight = 0.1*one;

            // duration is fixed
            //key->var_duration->locked = true;

            //key->con_duration_range->enabled = false;
		    key->con_duration_range->_min =  param.durationMin/T;
		    key->con_duration_range->_max =  param.durationMax/T;
        }

        vec3_t psum;
        real_t l2sum = 0.0;
        vector<real_t>  zl(nend);
        real_t zlsum = 0.0;
        for(int i = 0; i < nend; i++){
		    zl[i]  = std::max(0.0, key->var_pos_t->val.z - key->ends[i].var_pos->val.z);
            zlsum += zl[i]*zl[i];
        }

        for(int i = 0; i < nend; i++){
			vec3_t pe = curve_end[i].CalcPos(t);
			vec3_t ve = curve_end[i].CalcVel(t);

			vec3_t pe_local = qt.Conjugated()*(pe - pt) -  ends[i].basePos;

			//int njnt = ends[i].jointPosRange.size();
			for(int j = 0; j < 3; j++){
				//key->ends[i].var_joint_pos[j]->val = 0.5*(ends[i].jointPosMin[j] + ends[i].jointPosMax[j])/L;
				key->ends[i].var_joint_pos[j]->val = pe_local[j]/L;
				key->ends[i].var_joint_vel[j]->val = 0.0;
			}

			key->ends[i].var_pos   ->val = pe/L;
			key->ends[i].var_vel   ->val = ve/V;
			key->ends[i].var_stiff ->val = sqrt(zl[i]/zlsum);
			key->ends[i].var_moment->val.clear();

			key->ends[i].con_kin->weight = 10.0*one;

            key->ends[i].con_des_pos->desired = pe/L;
            key->ends[i].con_des_vel->desired = ve/V;
            key->ends[i].con_des_pos->weight  = (!key->prev ? 100.0 : (key->next ? 0.01 : 100.0))*one;
			key->ends[i].con_des_vel->weight  = (!key->prev ? 100.0 : (key->next ? 0.01 : 100.0))*one;

			for(int j = 0; j < 3; j++){
				key->ends[i].con_joint_pos_range[j][0]->weight[0] = 100.0;
				key->ends[i].con_joint_pos_range[j][1]->weight[0] = 100.0;
			}
    		//for(int j = 0; j < 3; j++){
			//	key->ends[i].con_pos_range[j][0]->weight[0] = 1.0;
			//	key->ends[i].con_pos_range[j][1]->weight[0] = 1.0;
			//}
			//if(k == 0){
            //    key->ends[i].var_pos->locked = true;
            //    key->ends[i].var_vel->locked = true;
            //}

            if(key->next){
                key->ends[i].con_stiff_range->_min = 0.0;
                key->ends[i].con_stiff_range->_max = ends[i].stiffnessMax/S;

                key->ends[i].con_vel_zero   ->desired   = vec3_t();
                key->ends[i].con_stiff_zero ->desired   = 0.0;
                key->ends[i].con_moment_zero->desired   = vec3_t();

                // initial weight is zero
                key->ends[i].con_vel_zero   ->weight    = param.complWeightMin*one;
                key->ends[i].con_stiff_zero ->weight[0] = param.complWeightMin;
                key->ends[i].con_moment_zero->weight    = param.complWeightMin*one;

				for(int j = 0; j < 3; j++){
					key->ends[i].con_joint_vel_zero[j]->desired   = 0.0;
					key->ends[i].con_joint_vel_zero[j]->weight[0] = 0.1;
				}
            }

            for(int j = 0; j < nface; j++){
		        key->ends[i].con_contact[j]->weight[0] = param.complWeightMin;
            }
		}
	}

    // normalize face vertices
    for(Face& face : faces){
        for(vec3_t& v : face.hull->vertices)
            v *= (1.0/L);
        face.hull->CalcBSphere();
    }

	// create set of valid contacts
	CentroidDDPState* tmp = new CentroidDDPState(this, graph->solver);
	tmp->contact.resize(nend, -1);
	int idx = 0;
	validContacts.clear();
	while(true){
		if(callback->IsValidState(tmp))
			validContacts.push_back(tmp->contact);

		while(idx < nend){
			if(++tmp->contact[idx] == nface){
				tmp->contact[idx] = -1;
				idx++;
				continue;
			}
			idx = 0;
			break;
		}
		if(idx == nend)
			break;
	}

    // call prepare here so that initial trajectory is visualized properly
    Prepare();

    complWeight = param.complWeightMin;
	
    trajReady = false;
}

void Centroid::Prepare() {
	TrajectoryNode::Prepare();

	trajReady = false;
}

void Centroid::Finish(){
	complWeight = std::min(param.complWeightMax, complWeight*param.complWeightRate);

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
	    real_t dt = t/T - k0->var_time->val;
	    real_t Ct = cosh(k0->lbar*(dt));
	    real_t St = sinh(k0->lbar*(dt));
	
        pt = k0->pbar + Ct*(k0->p - k0->pbar) + (St/k0->lbar)*k0->v;
	    vt = k0->lbar * St*(k0->p - k0->pbar) + Ct*k0->v;
    }
    else{
        pt = k0->p;
        vt = k0->v;
    }

    at = (k0->lbar*k0->lbar)*(pt - k0->pbar);
	
    pos = L*pt;
	vel = V*vt;
    acc = A*at;
}

void Centroid::TorsoState(real_t t, quat_t& ori, vec3_t& angvel, int type) {
	if(traj.empty())
		return;

	KeyPair      kp = traj.GetSegment(t);
	CentroidKey* k0 = (CentroidKey*)kp.first;
	CentroidKey* k1 = (CentroidKey*)kp.second;

    if(k1 == k0->next){
        ori = InterpolateOri(
		    t,
		    k0->var_time->val*T, k0->var_pos_r->val, k0->var_vel_r->val/T,
		    k1->var_time->val*T, k1->var_pos_r->val, k1->var_vel_r->val/T,
		    type);

        angvel = InterpolateAngvel(
		    t,
		    k0->var_time->val*T, k0->var_pos_r->val, k0->var_vel_r->val/T,
		    k1->var_time->val*T, k1->var_pos_r->val, k1->var_vel_r->val/T,
		    type);
    }
    else{
        ori    = k0->var_pos_r->val;
        angvel = k0->var_vel_r->val/T;
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

void Centroid::EndState(real_t t, int index, vec3_t& pos, vec3_t& vel) {
	if(traj.empty())
		return;

	KeyPair      kp = traj.GetSegment(t);
	CentroidKey* k0 = (CentroidKey*)kp.first;
	CentroidKey* k1 = (CentroidKey*)kp.second;

    if(k1 == k0->next){
		// in flight
        if(k0->ends[index].iface == -1){
			// find lift-off and landing phase
			CentroidKey* km1 = k0;
			CentroidKey* k2  = k1;
			CentroidKey* ks  = k0;
			CentroidKey* kf  = k1;
            if(km1->prev && ((CentroidKey*)km1->prev)->ends[index].iface == -1)
                km1 = (CentroidKey*)km1->prev;
            if(k2->next && k2->ends[index].iface == -1)
                k2 = (CentroidKey*)k2->next;
			while(ks->prev && ((CentroidKey*)ks->prev)->ends[index].iface == -1)
                ks = (CentroidKey*)ks->prev;
            while(kf->next && kf->ends[index].iface == -1)
                kf = (CentroidKey*)kf->next;

		    real_t tm1 = km1->var_time->val*T;
			real_t t0  = k0 ->var_time->val*T;
			real_t t1  = k1 ->var_time->val*T;
			real_t t2  = k2 ->var_time->val*T;
			
			vec3_t pm1 = km1->ends[index].var_pos->val*L;
			vec3_t p0  = k0 ->ends[index].var_pos->val*L;
			vec3_t p1  = k1 ->ends[index].var_pos->val*L;
			vec3_t p2  = k2 ->ends[index].var_pos->val*L;
			vec3_t v0  = (k0 == ks) ? vec3_t() : (p1 - pm1)/(t1 - tm1);
			vec3_t v1  = (k1 == kf) ? vec3_t() : (p2 - p0 )/(t2 - t0 );
			//vec3_t v0 = k0->ends[index].var_vel->val*V;
			//vec3_t v1 = k1->ends[index].var_vel->val*V;
			
			//vec3_t ps = ks->ends[index].var_pos->val*L;
			//vec3_t pf = kf->ends[index].var_pos->val*L;

			pos = InterpolatePos(t, t0, p0, v0, t1, p1, v1, Interpolate::Cubic);
			vel = InterpolateVel(t, t0, p0, v0, t1, p1, v1, Interpolate::Cubic);

			real_t ts = ks->var_time->val*T;
			real_t tf = kf->var_time->val*T;
			
			const real_t _2pi = 2.0*3.1415;
			real_t tau = tf - ts;
			real_t s   = (t - ts)/tau;
			real_t ch  = (s - sin(_2pi*s)/_2pi);
			real_t chd = ((1.0 - cos(_2pi*s))/tau);
			real_t cv  = (1 - cos(_2pi*s))/2.0;
			real_t cvd = (_2pi*sin(_2pi*s)/(2.0*tau));

			real_t sw = param.swingHeight;
			real_t d  = (p1 - p0).norm();
			//if(d > 0.5){
			//	sw += (d - 0.5)*0.2;
			//}
			//pos += p0 + ch *(p1 - p0) + cv *vec3_t(0.0, 0.0, sw);
			//vel +=      chd*(p1 - p0) + cvd*vec3_t(0.0, 0.0, sw);
			pos += cv *vec3_t(0.0, 0.0, sw);
			vel += cvd*vec3_t(0.0, 0.0, sw);
			
			/*
			vec3_t pc0, vc0, ac0;
			quat_t qc0;
			vec3_t wc0;
			
			vec3_t pc1, vc1, ac1;
			quat_t qc1;
			vec3_t wc1;
			
			ComState  (t0+0.001, pc0, vc0, ac0);
			TorsoState(t0+0.001, qc0, wc0);

			ComState  (t1-0.001, pc1, vc1, ac1);
			TorsoState(t1-0.001, qc1, wc1);
			
			// project to movable range
			vec3_t pct, vct, act;
			quat_t qct;
			vec3_t wct;
			ComState  (t, pct, vct, act);
			TorsoState(t, qct, wct);

			vec3_t p0_local = qc0.Conjugated()*(p0 - pc0);
			vec3_t p1_local = qc1.Conjugated()*(p1 - pc1);
			vec3_t v0_local = qc0.Conjugated()*(   - vc0);
			vec3_t v1_local = qc1.Conjugated()*(   - vc1);
			vec3_t a0_local = qc0.Conjugated()*(   - ac0);
			vec3_t a1_local = qc1.Conjugated()*(   - ac1);

			vec3_t pt_local;
			vec3_t vt_local;
			vec3_t at_local;
			Interpolate(
				t, pt_local, vt_local, at_local,
				t0, p0_local, v0_local, a0_local, 
				t1, p1_local, v1_local, a1_local,
				5);

			pos = qct*pt_local + pct;
			vel = qct*vt_local + wct % (pos - pct) + vct;
			
			const real_t _2pi = 2.0*3.1415;
			real_t s   = (t - t0)/(t1 - t0);
			real_t cv  = (1 - cos(_2pi*s))/2.0;
			real_t cvd = (_2pi*sin(_2pi*s)/(2.0*(t1 - t0)));
		
			pos += vec3_t(0.0, 0.0, param.swingHeight*cv );
			vel += vec3_t(0.0, 0.0, param.swingHeight*cvd);
			*/
			/*
			vec3_t v0   = qc0*vt_local + wc0 % (p0 - pc0) + vc0;
			vec3_t v1   = qc1*vt_local + wc1 % (p1 - pc1) + vc1;
			vec3_t pacc = InterpolatePos(t, t0, vec3_t(), -v0, t1, vec3_t(), -v1, Interpolate::Cubic);
			vec3_t vacc = InterpolateVel(t, t0, vec3_t(), -v0, t1, vec3_t(), -v1, Interpolate::Cubic);

			pos += pacc;
			vel += vacc;
			*/	
			/*
			const real_t Tacc = 0.1;
			const real_t Tdec = 0.1;
			if(t < t0 + Tacc){
				vec3_t v0   = qc0*vt_local + wc0 % (p0 - pc0) + vc0;
				vec3_t pacc = InterpolatePos(t, t0, vec3_t(), -v0, t0 + Tacc, vec3_t(), vec3_t(), Interpolate::Cubic);
				vec3_t vacc = InterpolateVel(t, t0, vec3_t(), -v0, t0 + Tacc, vec3_t(), vec3_t(), Interpolate::Cubic);

				pos += pacc;
				vel += vacc;
			}
			if(t > t1 - Tdec){
				vec3_t v1   = qc1*vt_local + wc1 % (p1 - pc1) + vc1;
				vec3_t pdec = InterpolatePos(t, t1 - Tdec, vec3_t(), vec3_t(), t1, vec3_t(), -v1, Interpolate::Cubic);
				vec3_t vdec = InterpolateVel(t, t1 - Tdec, vec3_t(), vec3_t(), t1, vec3_t(), -v1, Interpolate::Cubic);

				pos += pdec;
				vel += vdec;
			}
			*/
			/*
			pos = InterpolatePos(t, t0, p0, vec3_t(), t1, p1, vec3_t(), Interpolate::Cubic);
			vel = InterpolateVel(t, t0, p0, vec3_t(), t1, p1, vec3_t(), Interpolate::Cubic);
        
			real_t d  = (p1  - p0 ).norm();
			real_t d0 = (pos - p0 ).norm();
			real_t d1 = (p1  - pos).norm();

			real_t zmax = std::max(p0.z, p1.z) + param.swingHeight;
			real_t s    = param.swingSlope;
			pos.z = std::min(zmax, std::min(p0.z + s*d0, p1.z + s*d1));

			real_t vh = sqrt(vel.x*vel.x + vel.y*vel.y);

			if(pos.z == zmax)
				vel.z =  0.0;
			if(pos.z == p0.z + s*d0)
				vel.z =  s*vh;
			if(pos.z == p1.z + s*d1)
				vel.z = -s*vh;
			*/
			/*
			// project to movable range
			vec3_t pct, vct, act;
			ComState(t, pct, vct, act);

			quat_t qct;
			vec3_t wct;
			TorsoState(t, qct, wct);

			vec3_t pos_local = qct.Conjugated()*(pos - pct);
			vec3_t vel_local = qct.Conjugated()*(vel - (vct + wct % (pos - pct)));

			for(int j = 0; j < 3; j++){
				if(pos_local[j] < ends[index].posRangeMin[j]){
					pos_local[j] = ends[index].posRangeMin[j];
					vel_local[j] = 0.0;
				}
				if(pos_local[j] > ends[index].posRangeMax[j]){
					pos_local[j] = ends[index].posRangeMax[j];
					vel_local[j] = 0.0;
				}
			}

			pos = qct*pos_local + pct;
			vel = qct*vel_local + vct + wct % (pos - pct);
			*/
		}
		else{
			pos = k0->ends[index].var_pos->val*L;
			vel = k0->ends[index].var_vel->val*V;
		}
    }
    else{
		pos = k0->ends[index].var_pos->val*L;
		vel = k0->ends[index].var_vel->val*V;
    }
}

void Centroid::EndForce(real_t t, int index, real_t& stiff, vec3_t& moment, bool& contact){
    KeyPair      kp = traj.GetSegment(t);
	CentroidKey* k0 = (CentroidKey*)kp.first;
	CentroidKey* k1 = (CentroidKey*)kp.second;

    stiff   = k0->ends[index].var_stiff ->val*S;
    moment  = k0->ends[index].var_moment->val*M;
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

	tprev = k0->var_time->val*T;
	tnext = k1->var_time->val*T;
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
		    canvas->MoveTo(f.hull->vertices[0]*L);
		    canvas->LineTo(f.hull->vertices[1]*L);
		    canvas->LineTo(f.hull->vertices[2]*L);
		    canvas->LineTo(f.hull->vertices[3]*L);
		    canvas->LineTo(f.hull->vertices[0]*L);
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
        EndState(t, i, s.ends[i].pos, s.ends[i].vel);
        EndForce(t, i, s.ends[i].stiffness, s.ends[i].moment, s.ends[i].contact);
        real_t l = s.ends[i].stiffness;
        s.ends[i].force = l*l*(s.pos - s.ends[i].pos);
	}
}

void Centroid::CreateSnapshot(real_t t){
	CreateSnapshot(t, snapshot);
}

void Centroid::DrawSnapshot(Render::Canvas* canvas, Render::Config* conf) {
    // body box
    /*
	if(conf->Set(canvas, Render::Item::CentroidTorso, this)){
        vec3_t vtx[8];
        vtx[0] = vec3_t(param.bodyRangeMin.x, param.bodyRangeMin.y, param.bodyRangeMin.z);
        vtx[1] = vec3_t(param.bodyRangeMin.x, param.bodyRangeMax.y, param.bodyRangeMin.z);
        vtx[2] = vec3_t(param.bodyRangeMax.x, param.bodyRangeMax.y, param.bodyRangeMin.z);
        vtx[3] = vec3_t(param.bodyRangeMax.x, param.bodyRangeMin.y, param.bodyRangeMin.z);
        vtx[4] = vec3_t(param.bodyRangeMin.x, param.bodyRangeMin.y, param.bodyRangeMax.z);
        vtx[5] = vec3_t(param.bodyRangeMin.x, param.bodyRangeMax.y, param.bodyRangeMax.z);
        vtx[6] = vec3_t(param.bodyRangeMax.x, param.bodyRangeMax.y, param.bodyRangeMax.z);
        vtx[7] = vec3_t(param.bodyRangeMax.x, param.bodyRangeMin.y, param.bodyRangeMax.z);
        for(int j = 0; j < 2; j++){
		    canvas->BeginPath();
		    canvas->MoveTo(snapshot.pos + snapshot.ori*vtx[4*j+0]);
		    canvas->LineTo(snapshot.pos + snapshot.ori*vtx[4*j+1]);
		    canvas->LineTo(snapshot.pos + snapshot.ori*vtx[4*j+2]);
		    canvas->LineTo(snapshot.pos + snapshot.ori*vtx[4*j+3]);
		    canvas->LineTo(snapshot.pos + snapshot.ori*vtx[4*j+0]);
		    canvas->EndPath();
        }
		for(int j = 0; j < 4; j++){
		    canvas->BeginPath();
		    canvas->MoveTo(snapshot.pos + snapshot.ori*vtx[0+j]);
		    canvas->LineTo(snapshot.pos + snapshot.ori*vtx[4+j]);
		    canvas->EndPath();
        }
    }
	*/
	if (conf->Set(canvas, Render::Item::CentroidEnd, this)) {
		for(int i = 0; i < ends.size(); i++){
			canvas->BeginLayer("centroid_end_snapshot", true);
			canvas->SetLineColor(i % 2 == 0 ? "magenta" : "blue");

			// line connecting com and end
		    canvas->BeginPath();
			canvas->MoveTo(snapshot.pos);
			canvas->LineTo(snapshot.pos + snapshot.ori*ends[i].basePos);
			canvas->LineTo(snapshot.ends[i].pos);
			canvas->EndPath();

            // line indicating force
            canvas->BeginPath();
			canvas->MoveTo(snapshot.ends[i].pos);
			canvas->LineTo(snapshot.ends[i].pos + 0.001*snapshot.ends[i].force);
			canvas->EndPath();
			canvas->EndLayer();
			
		}

        // end rectangle
        for(int i = 0; i < ends.size(); i++){
            vec3_t vtx[4];
            vtx[0] = vec3_t(ends[i].copRangeMin.x, ends[i].copRangeMin.y, 0.0);
            vtx[1] = vec3_t(ends[i].copRangeMin.x, ends[i].copRangeMax.y, 0.0);
            vtx[2] = vec3_t(ends[i].copRangeMax.x, ends[i].copRangeMax.y, 0.0);
            vtx[3] = vec3_t(ends[i].copRangeMax.x, ends[i].copRangeMin.y, 0.0);
            canvas->SetLineWidth(/*snapshot.ends[i].contact ? 2.0f : */1.0f);
			canvas->BeginPath();
			canvas->MoveTo(snapshot.ends[i].pos + vtx[0]);
			canvas->LineTo(snapshot.ends[i].pos + vtx[1]);
			canvas->LineTo(snapshot.ends[i].pos + vtx[2]);
			canvas->LineTo(snapshot.ends[i].pos + vtx[3]);
			canvas->LineTo(snapshot.ends[i].pos + vtx[0]);
			canvas->EndPath();
		}
	}	
}

DDPState* Centroid::CreateInitialState(){
	int nend  = ends .size();
    
	CentroidDDPState* st = new CentroidDDPState(this, graph->solver);
	st->contact.resize(nend);
	for(int i = 0; i < nend; i++){
		st->contact[i] = ends[i].contactInitial;
	}

	return st;
}

void Centroid::CreateNextStates(DDPState* _state, vector<DDPState*>& _next){
	int nend  = ends .size();
    int nface = faces.size();
    
    CentroidDDPState* st0 = (CentroidDDPState*)_state;
	CentroidDDPState* st1;

	CentroidKey* key = (CentroidKey*)traj.GetKeypoint(graph->ticks[st0->stage->k]);
	/*
	for(int i = 0; i < validContacts.size(); i++){
		st1 = new CentroidDDPState(this, graph->solver);
		st1->contact = validContacts[i];
	
		if( callback->IsValidTransition(st0, st1) ){
			_next.push_back(st1);
		}
	}
	*/
	// no contact change
	st1 = new CentroidDDPState(this, graph->solver);
	st1->contact = st0->contact;
	_next.push_back(st1);

	for(int i = 0; i < nend; i++){
		if(st0->contact[i] == -1){
			for(int j = 0; j < nface; j++){
				if(!key->ends[i].con_contact[j]->enabled)
					continue;

				st1 = new CentroidDDPState(this, graph->solver);
				st1->contact = st0->contact;
				st1->contact[i] = j;
				if( callback->IsValidState(st1) &&
					callback->IsValidTransition(st0, st1) ){
					_next.push_back(st1);
				}
			}
		}
		else{
			st1 = new CentroidDDPState(this, graph->solver);
			st1->contact = st0->contact;
			st1->contact[i] = -1;
			if( callback->IsValidState(st1) &&
				callback->IsValidTransition(st0, st1) ){
				_next.push_back(st1);
			}
		}
	}
}

void Centroid::OnThreadUpdate(DDPThread* _thread){
	static FILE* fileContact = 0;
	static int niter = 0;

	if(!fileContact)
		fileContact = fopen("contact.csv",  "w");

	for(int k = 0; k < _thread->steps.size(); k++){
		CentroidDDPState* st = (CentroidDDPState*)_thread->steps[k]->state;

		fprintf(fileContact, "%d, %d, ", niter, k);
		for(int i = 0; i < st->contact.size(); i++){
			fprintf(fileContact, "%d, ", st->contact[i]);
		}
		fprintf(fileContact, "\n");
	}

	niter++;

}

///////////////////////////////////////////////////////////////////////////////////////////////////

CentroidCon::CentroidCon(Solver* solver, int _dim, int _tag, string _name, CentroidKey* _obj, real_t _scale):
	Constraint(solver, _dim, ID(_tag, _obj->node, _obj->tick, _name), Constraint::Type::Equality, _scale) {
	obj[0] = _obj;
	obj[1] = (CentroidKey*)_obj->next;
}

CentroidPosConT::CentroidPosConT(Solver* solver, string _name, CentroidKey* _obj, real_t _scale):
	CentroidCon(solver, 3, ConTag::CentroidPosT, _name, _obj, _scale) {

	AddSLink (obj[1]->var_pos_t   );
	AddSLink (obj[0]->var_pos_t   );
	AddSLink (obj[0]->var_vel_t   );
	AddC3Link(obj[0]->var_duration);

	for(CentroidKey::End& end : obj[0]->ends){
		AddC3Link(end.var_stiff);
		AddSLink (end.var_pos  );
	}
}

CentroidPosConR::CentroidPosConR(Solver* solver, string _name, CentroidKey* _obj, real_t _scale):
	CentroidCon(solver, 3, ConTag::CentroidPosR, _name, _obj, _scale) {

	AddSLink(obj[1]->var_pos_r);
	AddSLink(obj[0]->var_pos_r);
	AddSLink(obj[0]->var_vel_r);
}

CentroidVelConT::CentroidVelConT(Solver* solver, string _name, CentroidKey* _obj, real_t _scale):
	CentroidCon(solver, 3, ConTag::CentroidVelT, _name, _obj, _scale) {

	AddSLink (obj[1]->var_vel_t   );
	AddSLink (obj[0]->var_pos_t   );
	AddSLink (obj[0]->var_vel_t   );
	AddC3Link(obj[0]->var_duration);

	for(CentroidKey::End& end : obj[0]->ends){
		AddC3Link(end.var_stiff);
		AddSLink (end.var_pos  );
	}
}

CentroidVelConR::CentroidVelConR(Solver* solver, string _name, CentroidKey* _obj, real_t _scale):
	CentroidCon(solver, 3, ConTag::CentroidVelR, _name, _obj, _scale) {

	AddSLink(obj[1]->var_vel_r);
	AddSLink(obj[0]->var_vel_r);
	
	for(CentroidKey::End& end : obj[0]->ends){
		AddSLink(end.var_moment);
	}
}
/*
CentroidAccConT::CentroidAccConT(Solver* solver, string _name, CentroidKey* _obj, real_t _scale):
	CentroidCon(solver, 3, ConTag::CentroidAccT, _name, _obj, _scale) {

	AddSLink (obj[0]->var_pos_t   );

	for(CentroidKey::End& end : obj[0]->ends){
		AddC3Link(end.var_stiff);
		AddM3Link(end.var_pos  );
	}
}
*/
CentroidTimeCon::CentroidTimeCon(Solver* solver, string _name, CentroidKey* _obj, real_t _scale):
	CentroidCon(solver, 1, ConTag::CentroidTime, _name, _obj, _scale) {
	
	AddSLink(obj[1]->var_time);
	AddSLink(obj[0]->var_time);
	AddSLink(obj[0]->var_duration);
}

CentroidJointPosCon::CentroidJointPosCon(Solver* solver, string _name, CentroidKey* _obj, int _iend, int _idx, real_t _scale):
	CentroidCon(solver, 1, ConTag::CentroidEndPos, _name, _obj, _scale) {
	iend  = _iend;
	idx   = _idx;

	AddSLink(obj[1]->ends[iend].var_joint_pos[idx]);
	AddSLink(obj[0]->ends[iend].var_joint_pos[idx]);
	AddSLink(obj[0]->ends[iend].var_joint_vel[idx]);
	AddSLink(obj[0]->var_duration);
}

CentroidEndPosCon::CentroidEndPosCon(Solver* solver, string _name, CentroidKey* _obj, int _iend, real_t _scale):
	CentroidCon(solver, 3, ConTag::CentroidEndPos, _name, _obj, _scale) {
	iend  = _iend;

	AddSLink (obj[1]->ends[iend].var_pos);
	AddSLink (obj[0]->ends[iend].var_pos);
	AddSLink (obj[0]->ends[iend].var_vel);
	AddC3Link(obj[0]->var_duration);
}

CentroidJointPosRangeCon::CentroidJointPosRangeCon(Solver* solver, string _name, CentroidKey* _obj, int _iend, int _idx, real_t _dir, real_t _scale):
	Constraint(solver, 1, ID(ConTag::CentroidEndRange, _obj->node, _obj->tick, _name), Constraint::Type::InequalityPenalty, _scale){
	obj  = _obj;
	iend = _iend;
	idx  = _idx;
	dir  = _dir;

	//AddR3Link(obj->var_pos_t);
	//AddR3Link(obj->var_pos_r);
	AddSLink(obj->ends[iend].var_joint_pos[idx]);
}

/*
CentroidEndVelRangeCon::CentroidEndVelRangeCon(Solver* solver, string _name, CentroidKey* _obj, int _iend, vec3_t _dir, real_t _scale):
	Constraint(solver, 1, ID(ConTag::CentroidEndRange, _obj->node, _obj->tick, _name), Constraint::Type::InequalityBarrier, _scale){
	obj  = _obj;
	iend = _iend;
	dir  = _dir;

	AddR3Link(obj->var_pos_r);
    AddR3Link(obj->var_vel_t);
	AddR3Link(obj->ends[iend].var_vel);
}

CentroidEndVelZeroCon::CentroidEndVelZeroCon(Solver* solver, string _name, CentroidKey* _obj, int _iend, real_t _scale):
	Constraint(solver, 1, ID(ConTag::CentroidEndRange, _obj->node, _obj->tick, _name), Constraint::Type::Equality, _scale){
	obj  = _obj;
	iend = _iend;
	
	AddSLink (obj->var_vel_t);
	AddX3Link(obj->var_vel_r);
	AddM3Link(obj->ends[iend].var_vel);
}
*/

CentroidEndKinematicsCon::CentroidEndKinematicsCon(Solver* solver, string _name, CentroidKey* _obj, int _iend, real_t _scale):
	Constraint(solver, 3, ID(ConTag::CentroidEndContact, _obj->node, _obj->tick, _name), Constraint::Type::Equality, _scale){
	obj   = _obj;
	iend  = _iend;
    
	AddSLink (obj->var_pos_t);
	AddX3Link(obj->var_pos_r);
	AddSLink (obj->ends[iend].var_pos);
	for(SVar* v : obj->ends[iend].var_joint_pos)
		AddC3Link(v);

	qj     .resize(3);
	dir    .resize(3);
	dir_abs.resize(3);
}

CentroidEndContactCon::CentroidEndContactCon(Solver* solver, string _name, CentroidKey* _obj, int _iend, int _iface, real_t _scale):
	Constraint(solver, 1, ID(ConTag::CentroidEndContact, _obj->node, _obj->tick, _name), Constraint::Type::Equality, _scale){
	obj   = _obj;
	iend  = _iend;
    iface = _iface;
    point =  obj->cen->ends [iend].point;
    face  = &obj->cen->faces[_iface];

	AddR3Link(obj->ends[iend].var_pos);
}

///////////////////////////////////////////////////////////////////////////////////////////////////

void CentroidJointPosRangeCon::Prepare(){
	//p       = obj->var_pos_t->val;
	//q       = obj->var_pos_r->val;
	//pbase   = obj->cen->ends[iend].basePos;
	//pend    = obj->ends[iend].var_pos->val;
	//dir_abs = q*dir;
	q = obj->ends[iend].var_joint_pos[idx]->val;
}

/*
void CentroidEndVelRangeCon::Prepare(){
	q       = obj->var_pos_r->val;
    v       = obj->var_vel_t->val;
	vend    = obj->ends[iend].var_vel->val;
	dir_abs = q*dir;
}

void CentroidEndVelZeroCon::Prepare(){
	q  = obj->var_pos_r->val;
    v  = obj->var_vel_t->val;
	w  = obj->var_vel_r->val;
	pe = obj->ends[iend].var_pos->val;
	ve = obj->ends[iend].var_vel->val;

	pe_abs = q*pe;
	q.ToMatrix(R);
}
*/

void CentroidJointPosCon::Prepare(){
	q1  = obj[1]->ends[iend].var_joint_pos[idx]->val;
	q0  = obj[0]->ends[iend].var_joint_pos[idx]->val;
	qd0 = obj[0]->ends[iend].var_joint_vel[idx]->val;
	tau = obj[0]->var_duration->val;

	//qmin = obj[0]->cen->ends[iend].jointPosRange[idx][0];
	//qmax = obj[0]->cen->ends[iend].jointPosRange[idx][1];
	
	//real_t x    = q0 + qd0*tau;
	//real_t a    = 10.0/(qmax - qmin);

	//sig_min = exp(a*(x - qmin))/(1.0 + exp(a*(x - qmin)));
	//sig_max = exp(a*(x - qmax))/(1.0 + exp(a*(x - qmax)));
	//sig     = sig_min - sig_max;
}

void CentroidEndPosCon::Prepare(){
	pe1 = obj[1]->ends[iend].var_pos->val;
	pe0 = obj[0]->ends[iend].var_pos->val;
	ve0 = obj[0]->ends[iend].var_vel->val;
	tau = obj[0]->var_duration->val;
}

void CentroidEndKinematicsCon::Prepare(){
	p = obj->var_pos_t->val;
	q = obj->var_pos_r->val;
	
	pe_base = obj->cen->ends[iend].basePos;
	pe_local.clear();
	for(int j = 0; j < 3; j++){
		qj[j] = obj->ends[iend].var_joint_pos[j]->val;
		dir[j].clear();
		dir[j][j] = 1.0;
		dir_abs[j] = q*dir[j];

		pe_local += dir[j]*qj[j];
	}

	pe = obj->ends[iend].var_pos->val;

}

void CentroidEndContactCon::Prepare(){
    pe = obj->ends[iend].var_pos->val;
    point->position = pe;
    point->CalcBSphere();

    vec3_t sup0, sup1;
    real_t dist = inf;
    CalcNearest(point, face->hull, pose_t(), pose_t(), sup0, sup1, dist);

    pf = sup1;
    vec3_t d = pe - pf;
    real_t dnorm = d.norm();
    if(dnorm < 1.0e-10)
         nf = face->normal;
    else nf = d/dnorm;
}

///////////////////////////////////////////////////////////////////////////////////////////////////

void CentroidPosConT::CalcCoef(){
	int i = 0;
	((SLink *)links[i++])->SetCoef( 1.0    );
	((SLink *)links[i++])->SetCoef(-obj[0]->k_p_p  );
	((SLink *)links[i++])->SetCoef(-obj[0]->k_p_v  );
	((C3Link*)links[i++])->SetCoef(-obj[0]->k_p_tau);

	for(CentroidKey::End& end : obj[0]->ends){
		((C3Link*)links[i++])->SetCoef(-(obj[0]->k_p_pbar*end.k_pbar_le + obj[0]->k_p_lbar*end.k_lbar_le));
		((SLink *)links[i++])->SetCoef(-(obj[0]->k_p_pbar*end.k_pbar_pe));
	}
	
}

void CentroidPosConR::CalcCoef(){
	((SLink*)links[0])->SetCoef( 1.0);
	((SLink*)links[1])->SetCoef(-1.0);
	((SLink*)links[2])->SetCoef(-obj[0]->tau);
}

void CentroidVelConT::CalcCoef(){
	int i = 0;
	((SLink *)links[i++])->SetCoef( 1.0    );
	((SLink *)links[i++])->SetCoef(-obj[0]->k_v_p  );
	((SLink *)links[i++])->SetCoef(-obj[0]->k_v_v  );
	((C3Link*)links[i++])->SetCoef(-obj[0]->k_v_tau);

	for(CentroidKey::End& end : obj[0]->ends){
		((C3Link*)links[i++])->SetCoef(-(obj[0]->k_v_pbar*end.k_pbar_le + obj[0]->k_v_lbar*end.k_lbar_le));
		((SLink *)links[i++])->SetCoef(-(obj[0]->k_v_pbar*end.k_pbar_pe));
	}
}

void CentroidVelConR::CalcCoef(){
	int i = 0;
	((SLink*)links[i++])->SetCoef( 1.0);
	((SLink*)links[i++])->SetCoef(-1.0);
	
	for(CentroidKey::End& end : obj[0]->ends){
		((SLink*)links[i++])->SetCoef(-obj[0]->tau);
	}	
}
/*
void CentroidAccConT::CalcCoef(){
	int i = 0;
	((SLink *)links[i++])->SetCoef(obj[0]->k_a_p);

	for(CentroidKey::End& end : obj[0]->ends){
		((C3Link*)links[i++])->SetCoef((obj[0]->k_a_pbar*end.k_pbar_le + obj[0]->k_a_lbar*end.k_lbar_le));
		((M3Link*)links[i++])->SetCoef((obj[0]->k_a_pbar*end.k_pbar_pe));
	}
	
}
*/
void CentroidTimeCon::CalcCoef(){
	((SLink*)links[0])->SetCoef( 1.0);
	((SLink*)links[1])->SetCoef(-1.0);
	((SLink*)links[2])->SetCoef(-1.0);
}

void CentroidJointPosCon::CalcCoef(){
	Prepare();

	//sig = std::min(std::max(0.1, sig), 1.0);

	((SLink*)links[0])->SetCoef( 1.0);
	((SLink*)links[1])->SetCoef(-/*sig**/1.0);
	((SLink*)links[2])->SetCoef(-/*sig**/obj[0]->tau);
	((SLink*)links[3])->SetCoef(-/*sig**/qd0);
}

void CentroidEndPosCon::CalcCoef(){
	Prepare();

	((SLink *)links[0])->SetCoef( 1.0);
	((SLink *)links[1])->SetCoef(-1.0);
	((SLink *)links[2])->SetCoef(-obj[0]->tau);
    ((C3Link*)links[3])->SetCoef(-ve0);
}

void CentroidJointPosRangeCon::CalcCoef(){
	Prepare();

	//((R3Link*)links[0])->SetCoef(-dir_abs           );
	//((R3Link*)links[1])->SetCoef( dir_abs%(pend - p));
	//((R3Link*)links[2])->SetCoef( dir_abs           );
	((SLink*)links[0])->SetCoef(dir);
}

/*
void CentroidEndVelRangeCon::CalcCoef(){
	Prepare();

    ((R3Link*)links[0])->SetCoef( dir_abs%(vend - v));
	((R3Link*)links[1])->SetCoef(-dir_abs           );
	((R3Link*)links[2])->SetCoef( dir_abs           );
}
*/
/*
void CentroidEndVelZeroCon::CalcCoef(){
	Prepare();

	//y = v + w % (q*pe) + q*ve;
	((SLink *)links[0])->SetCoef(1.0);
	((X3Link*)links[1])->SetCoef(-pe_abs);
	((M3Link*)links[2])->SetCoef(R);
}
*/

void CentroidEndKinematicsCon::CalcCoef(){
	Prepare();

	int i = 0;
	((SLink *)links[i++])->SetCoef(-1.0);
	((X3Link*)links[i++])->SetCoef( q*(pe_base + pe_local));
	((SLink *)links[i++])->SetCoef( 1.0);
	for(int j = 0; j < qj.size(); j++)
		((C3Link*)links[i++])->SetCoef(-dir_abs[j]);

}

void CentroidEndContactCon::CalcCoef(){
	Prepare();

	((R3Link*)links[0])->SetCoef(nf);
}

///////////////////////////////////////////////////////////////////////////////////////////////////

void CentroidPosConT::CalcDeviation(){
	y = obj[1]->var_pos_t->val - obj[0]->p_rhs;
}

void CentroidPosConR::CalcDeviation(){
	quat_t qerror = obj[0]->q_rhs.Conjugated()*obj[1]->var_pos_r->val;
	vec3_t axis   = qerror.Axis ();
	real_t theta  = qerror.Theta();
	if(theta > pi)
		theta -= 2*pi;
	y = obj[0]->q_rhs*(theta*axis);
}

void CentroidVelConT::CalcDeviation(){
	y = obj[1]->var_vel_t->val - obj[0]->v_rhs;
}

void CentroidVelConR::CalcDeviation(){
	y = obj[1]->var_vel_r->val - obj[0]->w_rhs;
}
/*
void CentroidAccConT::CalcDeviation(){
	y = obj[0]->a_rhs;
}
*/
void CentroidTimeCon::CalcDeviation(){
	y[0] = obj[1]->var_time->val - (obj[0]->var_time->val + obj[0]->var_duration->val);
}

void CentroidJointPosCon::CalcDeviation(){
	y[0] = q1 - (q0 + qd0*tau);
	//y[0] = q1 - ((1.0-sig_min)*qmin + (sig_min - sig_max)*(q0 + qd0*tau) + sig_max*qmax);
}

void CentroidEndPosCon::CalcDeviation(){
	y = pe1 - (pe0 + ve0*tau);
}

void CentroidJointPosRangeCon::CalcDeviation(){
    //y[0] = dir_abs*(pend - (p + q*pbase)) - bound;
	y[0] = dir*q - bound;
	active = (y[0] < 0.0);

	//if(y[0] < 0.0){
	//	DSTR << "prc: " << y[0] << endl;
	//}
}

/*
void CentroidEndVelRangeCon::CalcDeviation(){
    y[0] = dir_abs*(vend - v) - bound;
}

void CentroidEndVelZeroCon::CalcDeviation(){
	y = v + w % (q*pe) + q*ve;
}
*/

void CentroidEndKinematicsCon::CalcDeviation(){
	y = pe - (p + q*(pe_base + pe_local));
	//if(iend == 0)
	//	DSTR << y << endl;
}

void CentroidEndContactCon::CalcDeviation(){
	y[0] = nf*(pe - pf);
}

///////////////////////////////////////////////////////////////////////////////////////////////////

void CentroidPosConT::CalcLhs(){
	obj[1]->var_pos_t->val = obj[0]->p_rhs;
}

void CentroidPosConR::CalcLhs(){
	obj[1]->var_pos_r->val = obj[0]->q_rhs;
}

void CentroidVelConT::CalcLhs(){
	obj[1]->var_vel_t->val = obj[0]->v_rhs;
}

void CentroidVelConR::CalcLhs(){
	obj[1]->var_vel_r->val = obj[0]->w_rhs;
}

void CentroidTimeCon::CalcLhs(){
	obj[1]->var_time->val = obj[0]->var_time->val + obj[0]->var_duration->val;
}

void CentroidJointPosCon::CalcLhs(){
	obj[1]->ends[iend].var_joint_pos[idx]->val = q0 + qd0*tau;
}

void CentroidEndPosCon::CalcLhs(){
	obj[1]->ends[iend].var_pos->val = pe0 + ve0*tau;
}

///////////////////////////////////////////////////////////////////////////////////////////////////

/*
void CentroidEndPosRangeCon::Project(real_t& l, uint k) {
	if ( on_upper && l > 0.0  ) l = 0.0;
	if ( on_lower && l < 0.0  ) l = 0.0;
	if (!on_upper && !on_lower) l = 0.0;
}
*/
/*
void CentroidEndVelRangeCon::Project(real_t& l, uint k) {
	if ( on_upper && l > 0.0  ) l = 0.0;
	if ( on_lower && l < 0.0  ) l = 0.0;
	if (!on_upper && !on_lower) l = 0.0;
}
*/
}
