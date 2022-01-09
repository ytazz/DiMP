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
	
	ends.resize(cen->ends.size());
	stringstream ss;
	for(int i = 0; i < ends.size(); i++){
		ends[i].key = this;

		ss.str("");
		ss << name << "_end" << i;
		ends[i].var_pos    = new V3Var(solver, ID(VarTag::CentroidEndPos   , node, tick, ss.str() + "_pos"   ), node->graph->scale.pos_t);
		ends[i].var_vel    = new V3Var(solver, ID(VarTag::CentroidEndVel   , node, tick, ss.str() + "_vel"   ), node->graph->scale.vel_t);
		ends[i].var_stiff  = new SVar (solver, ID(VarTag::CentroidEndStiff , node, tick, ss.str() + "_stiff" ), 1.0);
		ends[i].var_moment = new V3Var(solver, ID(VarTag::CentroidEndMoment, node, tick, ss.str() + "_moment"), 1.0);
		ends[i].var_pos   ->weight    = damping*one;
		ends[i].var_vel   ->weight    = damping*one;
		ends[i].var_stiff ->weight[0] = damping;
		ends[i].var_moment->weight    = damping*one;

        solver->AddStateVar(ends[i].var_pos, tick->idx);

        solver->AddInputVar(ends[i].var_vel   , tick->idx);
        solver->AddInputVar(ends[i].var_stiff , tick->idx);
        solver->AddInputVar(ends[i].var_moment, tick->idx);
	}
}

void CentroidKey::AddCon(Solver* solver) {
	CentroidKey* nextObj = (CentroidKey*)next;

    subcost_u.clear();
    subcost_x.clear();

    int nend  = cen->ends .size();
    int nface = cen->faces.size();

    if(next){
		con_pos_t = new CentroidPosConT(solver, name + "_pos_t", this, node->graph->scale.pos_t);
		con_pos_r = new CentroidPosConR(solver, name + "_pos_r", this, node->graph->scale.pos_r);
		con_vel_t = new CentroidVelConT(solver, name + "_vel_t", this, node->graph->scale.vel_t);
		con_vel_r = new CentroidVelConR(solver, name + "_vel_r", this, node->graph->scale.vel_r);
		con_time  = new CentroidTimeCon(solver, name + "_time" , this, 1.0);
		
		//con_duration_range = new RangeConS (solver, ID(ConTag::CentroidTime, node, tick, name + "_duration" ), var_duration, 1.0);
        
        solver->AddTransitionCon(con_pos_t, tick->idx);
        solver->AddTransitionCon(con_pos_r, tick->idx);
        solver->AddTransitionCon(con_vel_t, tick->idx);
        solver->AddTransitionCon(con_vel_r, tick->idx);
        solver->AddTransitionCon(con_time , tick->idx);

        //subcost_u.push_back(solver->AddCostCon(con_duration_range, tick->idx));
    }

    con_vel_zero  = new FixConV3(solver, ID(ConTag::CentroidVelT, node, tick, name + "_vel_zero" ), var_vel_t, 1.0);
    con_des_pos_t = new FixConV3(solver, ID(ConTag::CentroidPosT, node, tick, name + "_des_pos_t"), var_pos_t, 1.0);
    con_des_pos_r = new FixConQ (solver, ID(ConTag::CentroidPosR, node, tick, name + "_des_pos_r"), var_pos_r, 1.0);

    subcost_x.push_back(solver->AddCostCon(con_vel_zero , tick->idx));
    subcost_x.push_back(solver->AddCostCon(con_des_pos_t, tick->idx));
    subcost_x.push_back(solver->AddCostCon(con_des_pos_r, tick->idx));

	stringstream ss;
	for(int i = 0; i < nend; i++){
		ss.str("");
		ss << name << "_end" << i;

        ends[i].subcost_c   .clear();
        ends[i].subcost_nc  .clear();
        ends[i].subcost_face.resize(nface);

        /// non-complementary constraints
		if(next){
			ends[i].con_pos    = new CentroidEndPosCon(solver, ss.str() + "_pos", this, i, 1.0);
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

            subcost_u.push_back(solver->AddCostCon(ends[i].con_stiff_range    , tick->idx));
            subcost_u.push_back(solver->AddCostCon(ends[i].con_moment_range[0], tick->idx));
            subcost_u.push_back(solver->AddCostCon(ends[i].con_moment_range[1], tick->idx));
            subcost_u.push_back(solver->AddCostCon(ends[i].con_moment_range[2], tick->idx));

            //subcost_u.push_back(solver->AddCostCon(ends[i].con_vel_range[0][0], tick->idx));
            //subcost_u.push_back(solver->AddCostCon(ends[i].con_vel_range[0][1], tick->idx));
            //subcost_u.push_back(solver->AddCostCon(ends[i].con_vel_range[1][0], tick->idx));
            //subcost_u.push_back(solver->AddCostCon(ends[i].con_vel_range[1][1], tick->idx));
            //subcost_u.push_back(solver->AddCostCon(ends[i].con_vel_range[2][0], tick->idx));
            //subcost_u.push_back(solver->AddCostCon(ends[i].con_vel_range[2][1], tick->idx));
        }
	
        //ends[i].con_pos_range[0][0] = new CentroidEndPosRangeCon(solver, ss.str() + "_pos_range0_min", this, i, vec3_t( 1.0,  0.0,  0.0), node->graph->scale.pos_t);
        //ends[i].con_pos_range[0][1] = new CentroidEndPosRangeCon(solver, ss.str() + "_pos_range0_max", this, i, vec3_t(-1.0,  0.0,  0.0), node->graph->scale.pos_t);
		//ends[i].con_pos_range[1][0] = new CentroidEndPosRangeCon(solver, ss.str() + "_pos_range1_min", this, i, vec3_t( 0.0,  1.0,  0.0), node->graph->scale.pos_t);
		//ends[i].con_pos_range[1][1] = new CentroidEndPosRangeCon(solver, ss.str() + "_pos_range1_max", this, i, vec3_t( 0.0, -1.0,  0.0), node->graph->scale.pos_t);
		//ends[i].con_pos_range[2][0] = new CentroidEndPosRangeCon(solver, ss.str() + "_pos_range2_min", this, i, vec3_t( 0.0,  0.0,  1.0), node->graph->scale.pos_t);
		//ends[i].con_pos_range[2][1] = new CentroidEndPosRangeCon(solver, ss.str() + "_pos_range2_max", this, i, vec3_t( 0.0,  0.0, -1.0), node->graph->scale.pos_t);
        
        //subcost_x.push_back(solver->AddCostCon(ends[i].con_pos_range[0][0], tick->idx));
        //subcost_x.push_back(solver->AddCostCon(ends[i].con_pos_range[0][1], tick->idx));
        //subcost_x.push_back(solver->AddCostCon(ends[i].con_pos_range[1][0], tick->idx));
        //subcost_x.push_back(solver->AddCostCon(ends[i].con_pos_range[1][1], tick->idx));
        //subcost_x.push_back(solver->AddCostCon(ends[i].con_pos_range[2][0], tick->idx));
        //subcost_x.push_back(solver->AddCostCon(ends[i].con_pos_range[2][1], tick->idx));

        ends[i].con_des_pos = new FixConV3(solver, ID(ConTag::CentroidEndPos, node, tick, name + "_des_pos"), ends[i].var_pos, 1.0);

        subcost_x.push_back(solver->AddCostCon(ends[i].con_des_pos, tick->idx));

        /// complimentary constraints
        if(next){
            ends[i].con_vel_zero    = new FixConV3(solver, ID(ConTag::CentroidEndVel   , node, tick, name + "_vel_zero"   ), ends[i].var_vel   , 1.0);
            ends[i].con_stiff_zero  = new FixConS (solver, ID(ConTag::CentroidEndStiff , node, tick, name + "_stiff_zero" ), ends[i].var_stiff , 1.0);
            ends[i].con_moment_zero = new FixConV3(solver, ID(ConTag::CentroidEndMoment, node, tick, name + "_moment_zero"), ends[i].var_moment, 1.0);

            ends[i].subcost_c .push_back(solver->AddCostCon(ends[i].con_vel_zero   , tick->idx));
            ends[i].subcost_nc.push_back(solver->AddCostCon(ends[i].con_stiff_zero , tick->idx));
            ends[i].subcost_nc.push_back(solver->AddCostCon(ends[i].con_moment_zero, tick->idx));
        }
        
        ends[i].con_contact.resize(nface);
        for(int j = 0; j < nface; j++){
		    ends[i].con_contact[j] = new CentroidEndContactCon(solver, name + "_contact", this, i, j, 1.0);
            
            ends[i].subcost_face[j] = solver->AddCostCon(ends[i].con_contact[j], tick->idx);
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
	k_v_tau  =  (l2sum*C*(p - pbar) + lbar*S*v);
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

	p_rhs = (pbar + C*(p - pbar) + (S/lbar)*v);
	v_rhs = (lbar*S*(p - pbar) + C*v);

	q_rhs = q*quat_t::Rot(q.Conjugated()*(w*tau));
	w_rhs = w + msum*tau;

    // contact filtering
	for(End& end : ends){
        for(int j = 0; j < cen->faces.size(); j++){
            end.con_contact[j]->Prepare();
            cen->callback->EnableContact(end.con_contact[j]);
        }

		DSTR << end.var_stiff->val << " ";
    }
	DSTR << endl;
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

	L = 0.0;
	Lx .clear();
	Lu .clear();
	Lxx.clear();
	Luu.clear();
	Lux.clear();
    
	// weights on variables
	for(Solver::SubState* subst : solver->state[k]->substate){
		Lxx += subst->Lxx;
	}
	if(k < solver->N){
		for(Solver::SubInput* subin : solver->input[k]->subinput){
			Luu += subin->Luu;
		}
	}

	for(Solver::SubCost* subcost : key->subcost_x){
		if(subcost->con->enabled && subcost->con->active){
			L   += subcost->L;
			Lx  += subcost->Lx;
			Lxx += subcost->Lxx;
		}
	}
	for(Solver::SubCost* subcost : key->subcost_u){
		if(subcost->con->enabled && subcost->con->active){
			L   += subcost->L;
			Lu  += subcost->Lu;
			Luu += subcost->Luu;
		}
	}

	int nend  = cen->ends .size();
	int nface = cen->faces.size();
	//real_t wr = cen->complWeight/cen->param.complWeightMin;
	real_t wr = cen->param.complWeightMax/cen->param.complWeightMin;

	for(int i = 0; i < nend; i++){
		real_t w_c , w_c2 ;
		real_t w_nc, w_nc2;
		w_c   = (contact[i] == -1 ? 1.0 : wr );
		w_nc  = (contact[i] == -1 ? wr  : 1.0);
		w_c2  = w_c*w_c;
		w_nc2 = w_nc*w_nc;

		for(Solver::SubCost* subcost : key->ends[i].subcost_c){
			L   += w_c2 * subcost->L;
			Lu  += w_c2 * subcost->Lu;
			Luu += w_c2 * subcost->Luu;
		}
		for(Solver::SubCost* subcost : key->ends[i].subcost_nc){
			L   += w_nc2 * subcost->L;
			Lu  += w_nc2 * subcost->Lu;
			Luu += w_nc2 * subcost->Luu;
		}

		for(int j = 0; j < nface; j++){
			real_t w_f, w_f2;
			w_f  = (contact[i] == j ? wr : 1.0);
			w_f2 = w_f*w_f;
			L   += w_f2 * key->ends[i].subcost_face[j]->L;
			Lx  += w_f2 * key->ends[i].subcost_face[j]->Lx;
			Lxx += w_f2 * key->ends[i].subcost_face[j]->Lxx;
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

		key->var_pos_t->val = curve_t.CalcPos(t)/L;
		key->var_pos_r->val = curve_r.CalcPos(t);
		key->var_vel_t->val = curve_t.CalcVel(t)/V;
		key->var_vel_r->val = curve_r.CalcVel(t)/(1/T);

        //if(k == 0){
        //    key->var_pos_t->locked = true;
        //    key->var_vel_t->locked = true;
        //}

        // rotation is fixed at the moment
        key->var_pos_r->locked = true;
	    key->var_vel_r->locked = true;

        key->con_des_pos_t->desired = curve_t.CalcPos(t)/L;
        key->con_des_pos_r->desired = curve_r.CalcPos(t);
        key->con_vel_zero ->desired = vec3_t();
        key->con_des_pos_t->weight = (key->next ? 1.0 : 10.0)*one;
        key->con_des_pos_r->weight = (key->next ? 1.0 : 10.0)*one;
        key->con_vel_zero ->weight = (key->next ? 1.0 : 10.0)*one;

		key->var_time->val = t/T;
		if(key->next){
			key->var_duration->val = (key->next->tick->time - t)/T;

            // duration is fixed
            key->var_duration->locked = true;

            //key->con_duration_range->enabled = false;
		    //key->con_duration_range->_min =  0.0;
		    //key->con_duration_range->_max =  inf;
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
			key->ends[i].var_pos  ->val = curve_end[i].CalcPos(t)/L;
			key->ends[i].var_vel  ->val = curve_end[i].CalcVel(t)/V;
			key->ends[i].var_stiff->val = sqrt(zl[i]/zlsum);
			key->ends[i].var_moment->val.clear();

            key->ends[i].con_des_pos->desired = curve_end[i].CalcPos(t)/L;
            key->ends[i].con_des_pos->weight  = (key->next ? 1.0 : 10.0)*one;

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
	for (uint k = 0; k < graph->ticks.size(); k++) {
		CentroidKey* key = (CentroidKey*)traj.GetKeypoint(graph->ticks[k]);
		key->tick->time = key->var_time->val*T;
	}

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

void Centroid::EndState(real_t t, int index, vec3_t& pos, vec3_t& vel) {
	if(traj.empty())
		return;

	KeyPair      kp = traj.GetSegment(t);
	CentroidKey* k0 = (CentroidKey*)kp.first;
	CentroidKey* k1 = (CentroidKey*)kp.second;

    if(k1 == k0->next){
        if(k0->ends[index].iface == -1){
            while(k0->prev && ((CentroidKey*)k0->prev)->ends[index].iface == -1)
                k0 = (CentroidKey*)k0->prev;
            while(k1->next && k1->ends[index].iface == -1)
                k1 = (CentroidKey*)k1->next;
        }

        real_t t0 = k0->var_time->val*T;
        real_t t1 = k1->var_time->val*T;

        vec3_t p0 = k0->ends[index].var_pos->val*L;
        vec3_t p1 = k1->ends[index].var_pos->val*L;

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

	// end
	if (conf->Set(canvas, Render::Item::CentroidEnd, this)) {
		for(int i = 0; i < ends.size(); i++){
			canvas->BeginLayer("centroid_end", true);
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
	    for(Face& f : faces){
		    canvas->BeginLayer("centroid_face", true);
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

    if (conf->Set(canvas, Render::Item::CentroidEnd, this)) {
		for(int i = 0; i < ends.size(); i++){
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
				_next.push_back(st1);
			}
		}
		else{
			st1 = new CentroidDDPState(this, graph->solver);
			st1->contact = st0->contact;
			st1->contact[i] = -1;
			_next.push_back(st1);
		}
	}
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

CentroidTimeCon::CentroidTimeCon(Solver* solver, string _name, CentroidKey* _obj, real_t _scale):
	CentroidCon(solver, 1, ConTag::CentroidTime, _name, _obj, _scale) {
	//obj[1] = obj[0]->endNext;

	AddSLink(obj[1]->var_time);
	AddSLink(obj[0]->var_time);
	AddSLink(obj[0]->var_duration);
}

CentroidEndPosCon::CentroidEndPosCon(Solver* solver, string _name, CentroidKey* _obj, int _iend, real_t _scale):
	CentroidCon(solver, 3, ConTag::CentroidEndPos, _name, _obj, _scale) {
	iend = _iend;

	AddSLink (obj[1]->ends[iend].var_pos);
	AddSLink (obj[0]->ends[iend].var_pos);
	AddSLink (obj[0]->ends[iend].var_vel);
	AddC3Link(obj[0]->var_duration);
}

CentroidEndPosRangeCon::CentroidEndPosRangeCon(Solver* solver, string _name, CentroidKey* _obj, int _iend, vec3_t _dir, real_t _scale):
	Constraint(solver, 1, ID(ConTag::CentroidEndRange, _obj->node, _obj->tick, _name), Constraint::Type::InequalityBarrier, _scale){
	obj  = _obj;
	iend = _iend;
	dir  = _dir;

	AddR3Link(obj->var_pos_t);
	AddR3Link(obj->var_pos_r);
	AddR3Link(obj->ends[iend].var_pos);
}

CentroidEndVelRangeCon::CentroidEndVelRangeCon(Solver* solver, string _name, CentroidKey* _obj, int _iend, vec3_t _dir, real_t _scale):
	Constraint(solver, 1, ID(ConTag::CentroidEndRange, _obj->node, _obj->tick, _name), Constraint::Type::InequalityBarrier, _scale){
	obj  = _obj;
	iend = _iend;
	dir  = _dir;

	AddR3Link(obj->var_pos_r);
    AddR3Link(obj->var_vel_t);
	AddR3Link(obj->ends[iend].var_vel);
}
/*
CentroidEndVelZeroCon::CentroidEndVelZeroCon(Solver* solver, string _name, CentroidKey* _obj, int _iend, vec3_t _dir, real_t _scale):
	Constraint(solver, 1, ID(ConTag::CentroidEndRange, _obj->node, _obj->tick, _name), _scale){
	obj  = _obj;
	iend = _iend;
	dir  = _dir;

	AddR3Link(obj->var_pos_r);
	AddR3Link(obj->var_vel_t);
	AddR3Link(obj->ends[iend].var_vel);
}
*/
CentroidEndContactCon::CentroidEndContactCon(Solver* solver, string _name, CentroidKey* _obj, int _iend, int _iface, real_t _scale):
	Constraint(solver, 1, ID(ConTag::CentroidEndContact, _obj->node, _obj->tick, _name), Constraint::Type::Equality, _scale){
	obj   = _obj;
	iend  = _iend;
    iface = _iface;
    point =  obj->cen->ends [iend].point;
    face  = &obj->cen->faces[_iface];
    // = _ihull;
    //face  = &obj->cen->faces[iface];

	AddR3Link(obj->ends[iend].var_pos);
}

///////////////////////////////////////////////////////////////////////////////////////////////////

void CentroidEndPosRangeCon::Prepare(){
	p       = obj->var_pos_t->val;
	q       = obj->var_pos_r->val;
	pbase   = obj->cen->ends[iend].basePos;
	pend    = obj->ends[iend].var_pos->val;
	dir_abs = q*dir;
}

void CentroidEndVelRangeCon::Prepare(){
	q       = obj->var_pos_r->val;
    v       = obj->var_vel_t->val;
	vend    = obj->ends[iend].var_vel->val;
	dir_abs = q*dir;
}
/*
void CentroidEndVelZeroCon::Prepare(){
	q       = obj->var_pos_r->val;
    v       = obj->var_vel_t->val;
	vend    = obj->ends[iend].var_vel->val;
	dir_abs = q*dir;
}
*/
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

    //nf = face->normal;
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

void CentroidTimeCon::CalcCoef(){
	((SLink*)links[0])->SetCoef( 1.0);
	((SLink*)links[1])->SetCoef(-1.0);
	((SLink*)links[2])->SetCoef(-1.0);
}

void CentroidEndPosCon::CalcCoef(){
	//if(obj[0]->ends[iend].iface != -1){
	//	((SLink *)links[0])->SetCoef( 1.0);
	//	((SLink *)links[1])->SetCoef(-1.0);
	//	((SLink *)links[2])->SetCoef( 0.0);
	//	((C3Link*)links[3])->SetCoef( vec3_t());
	//}
	//else{
	((SLink *)links[0])->SetCoef( 1.0);
	((SLink *)links[1])->SetCoef(-1.0);
	((SLink *)links[2])->SetCoef(-obj[0]->tau);
	((C3Link*)links[3])->SetCoef(-obj[0]->ends[iend].var_vel->val);
	//}
}

void CentroidEndPosRangeCon::CalcCoef(){
	Prepare();

	((R3Link*)links[0])->SetCoef(-dir_abs           );
	((R3Link*)links[1])->SetCoef( dir_abs%(pend - p));
	((R3Link*)links[2])->SetCoef( dir_abs           );
}

void CentroidEndVelRangeCon::CalcCoef(){
	Prepare();

    ((R3Link*)links[0])->SetCoef( dir_abs%(vend - v));
	((R3Link*)links[1])->SetCoef(-dir_abs           );
	((R3Link*)links[2])->SetCoef( dir_abs           );
}
/*
void CentroidEndVelZeroCon::CalcCoef(){
	Prepare();

	((R3Link*)links[0])->SetCoef( dir_abs % (vend - v));
	((R3Link*)links[1])->SetCoef(-dir_abs );
	((R3Link*)links[2])->SetCoef( dir_abs );
}
*/
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

void CentroidTimeCon::CalcDeviation(){
	y[0] = obj[1]->var_time->val - (obj[0]->var_time->val + obj[0]->var_duration->val);
}

void CentroidEndPosCon::CalcDeviation(){
	//if(obj[0]->ends[iend].iface != -1){
	//	y = obj[1]->ends[iend].var_pos->val - (obj[0]->ends[iend].var_pos->val);
	//}
	//else{
	y = obj[1]->ends[iend].var_pos->val - (obj[0]->ends[iend].var_pos->val + obj[0]->ends[iend].var_vel->val*obj[0]->var_duration->val);
	//}
}

void CentroidEndPosRangeCon::CalcDeviation(){
    y[0] = dir_abs*(pend - (p + q*pbase)) - bound;

	/*
    y[0]   = 0.0;
	on_lower = on_upper = active = false;

	real_t r = dir_abs*(pend - (p + q*pbase));
	if(r < _min){
		y[0]     = r - _min;
		on_lower = true;
		active   = true;
	}
	else if(r > _max){
		y[0]     = r - _max;
		on_upper = true;
		active   = true;
	}
    */
}

void CentroidEndVelRangeCon::CalcDeviation(){
    y[0] = dir_abs*(vend - v) - bound;

    /*
    y[0]   = 0.0;
	on_lower = on_upper = active = false;

	real_t r = dir_abs*(vend - v);
	if(r < _min){
		y[0]     = r - _min;
		on_lower = true;
		active   = true;
	}
	else if(r > _max){
		y[0]     = r - _max;
		on_upper = true;
		active   = true;
	}
    */
}
/*
void CentroidEndVelZeroCon::CalcDeviation(){
	y[0] = dir_abs*(vend - v);
}
*/
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

void CentroidEndPosCon::CalcLhs(){
	//if(obj[0]->ends[iend].iface != -1){
	//	obj[1]->ends[iend].var_pos->val = obj[0]->ends[iend].var_pos->val;
	//}
	//else{
    obj[1]->ends[iend].var_pos->val = obj[0]->ends[iend].var_pos->val + obj[0]->ends[iend].var_vel->val*obj[0]->var_duration->val;
	//}
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
