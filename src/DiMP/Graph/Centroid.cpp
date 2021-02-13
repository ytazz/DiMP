#include <DiMP/Graph/Centroid.h>
#include <DiMP/Graph/Graph.h>
#include <DiMP/Render/Config.h>
#include <DiMP/Render/Canvas.h>

namespace DiMP {;

const real_t pi      = M_PI;
const real_t inf     = numeric_limits<real_t>::max();
const real_t eps     = 1.0e-10;
const real_t damping = 0.1;
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
	
	ends.resize(cen->param.ends.size());
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
	}
}

void CentroidKey::AddCon(Solver* solver) {
	CentroidKey* nextObj = (CentroidKey*)next;

	if(next){
		con_pos_t = new CentroidPosConT(solver, name + "_pos_t", this, node->graph->scale.pos_t);
		con_pos_r = new CentroidPosConR(solver, name + "_pos_r", this, node->graph->scale.pos_r);
		con_vel_t = new CentroidVelConT(solver, name + "_vel_t", this, node->graph->scale.vel_t);
		con_vel_r = new CentroidVelConR(solver, name + "_vel_r", this, node->graph->scale.vel_r);
		con_time  = new CentroidTimeCon(solver, name + "_time" , this, 1.0);
		
		con_duration_range = new RangeConS(solver, ID(ConTag::CentroidTime, node, tick, name + "_duration"), var_duration, 1.0);

		con_vel_match = new MatchConV3(solver, ID(ConTag::CentroidVelT, node, tick, name + "_vel_match"), var_vel_t, ((CentroidKey*)next)->var_vel_t, 1.0);
	}

	stringstream ss;
	for(int i = 0; i < ends.size(); i++){
		ss.str("");
		ss << name << "_end" << i;

		if(next){
			ends[i].con_pos    = new CentroidEndPosCon(solver, ss.str() + "_pos", this, i, 1.0);
		}
		//ends[i].con_stiff  = new CentroidEndStiffCon (solver, ConTag::CentroidEndStiff , ss.str() + "_stiff" , this, i, 1.0);
		//ends[i].con_moment = new CentroidEndMomentCon(solver, ConTag::CentroidEndMoment, ss.str() + "_moment", this, i, 1.0);

		//if(next){
		//	ends[i].con_pos_match    = new MatchConV3(solver, ID(ConTag::CentroidEndPos   , node, tick, name + "_pos_match"   ), ends[i].var_pos   , ((CentroidKey*)next)->ends[i].var_pos   , 1.0);
		//	ends[i].con_stiff_match  = new MatchConS (solver, ID(ConTag::CentroidEndStiff , node, tick, name + "_stiff_match" ), ends[i].var_stiff , ((CentroidKey*)next)->ends[i].var_stiff , 1.0);
		//	ends[i].con_moment_match = new MatchConV3(solver, ID(ConTag::CentroidEndMoment, node, tick, name + "_moment_match"), ends[i].var_moment, ((CentroidKey*)next)->ends[i].var_moment, 1.0);
		//}

		ends[i].con_pos_range[0] = new CentroidEndPosRangeCon(solver, ss.str() + "_pos_range0", this, i, vec3_t(1.0, 0.0, 0.0), node->graph->scale.pos_t);
		ends[i].con_pos_range[1] = new CentroidEndPosRangeCon(solver, ss.str() + "_pos_range1", this, i, vec3_t(0.0, 1.0, 0.0), node->graph->scale.pos_t);
		ends[i].con_pos_range[2] = new CentroidEndPosRangeCon(solver, ss.str() + "_pos_range2", this, i, vec3_t(0.0, 0.0, 1.0), node->graph->scale.pos_t);

		ends[i].con_vel_range[0] = new CentroidEndVelRangeCon(solver, ss.str() + "_vel_range0", this, i, vec3_t(1.0, 0.0, 0.0), node->graph->scale.pos_t);
		ends[i].con_vel_range[1] = new CentroidEndVelRangeCon(solver, ss.str() + "_vel_range1", this, i, vec3_t(0.0, 1.0, 0.0), node->graph->scale.pos_t);
		ends[i].con_vel_range[2] = new CentroidEndVelRangeCon(solver, ss.str() + "_vel_range2", this, i, vec3_t(0.0, 0.0, 1.0), node->graph->scale.pos_t);

		//ends[i].con_force_range[0] = new CentroidEndForceRangeCon(solver, ConTag::CentroidEndRange, ss.str() + "_force_range0", this, i, vec3_t(1.0, 0.0, 0.0), node->graph->scale.force_t);
		//ends[i].con_force_range[1] = new CentroidEndForceRangeCon(solver, ConTag::CentroidEndRange, ss.str() + "_force_range1", this, i, vec3_t(0.0, 1.0, 0.0), node->graph->scale.force_t);
		//ends[i].con_force_range[2] = new CentroidEndForceRangeCon(solver, ConTag::CentroidEndRange, ss.str() + "_force_range2", this, i, vec3_t(0.0, 0.0, 1.0), node->graph->scale.force_t);

		ends[i].con_stiff_range     = new RangeConS (solver, ID(ConTag::CentroidEndStiff , node, tick, name + "_stiff_range" ), ends[i].var_stiff, 1.0);
		
		ends[i].con_moment_range[0] = new RangeConV3(solver, ID(ConTag::CentroidEndMoment, node, tick, name + "_mom_range0"), ends[i].var_moment, vec3_t(1.0, 0.0, 0.0), 1.0);
		ends[i].con_moment_range[1] = new RangeConV3(solver, ID(ConTag::CentroidEndMoment, node, tick, name + "_mom_range1"), ends[i].var_moment, vec3_t(0.0, 1.0, 0.0), 1.0);
		ends[i].con_moment_range[2] = new RangeConV3(solver, ID(ConTag::CentroidEndMoment, node, tick, name + "_mom_range2"), ends[i].var_moment, vec3_t(0.0, 0.0, 1.0), 1.0);

		ends[i].con_contact = new CentroidEndContactCon(solver, name + "_contact", this, i, 1.0);
		
		//ends[i].con_stiff_zero = new FixConS(solver, ID(ConTag::CentroidEndStiff, node,tick, name + "_stiff_zero"), ends[i].var_stiff, 1.0);
	}
}		

void CentroidKey::Prepare() {
	real_t l2sum = 0.0;
	vec3_t ez(0.0, 0.0, 1.0);
	vec3_t psum = ez;
	vec3_t msum;

	ncon = 0;
	for(End& end : ends){
		if(end.contact){
			real_t le  = end.var_stiff ->val;
			vec3_t pe  = end.var_pos->val;
			vec3_t me  = end.var_moment->val;

			real_t le2 = le*le;

			l2sum += le2;
			psum  += le2*pe;
			msum  += me;
			ncon++;
		}
	}

	p   = var_pos_t->val;
	q   = var_pos_r->val;
	v   = var_vel_t->val;
	w   = var_vel_r->val;
	tau = var_duration->val;

	if(ncon > 0){
		lbar = sqrt(l2sum);
		pbar = psum/l2sum;

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
			if(end.contact){
				real_t le  = end.var_stiff->val;
				real_t le2 = le*le;
				vec3_t pe  = end.var_pos->val;

				end.k_pbar_pe = le2/l2sum;
				end.k_pbar_le = (2.0*le*(pe - pbar))/(l2sum);
				end.k_lbar_le = le/lbar;
			}
			else{
				end.k_pbar_pe = 0.0;
				end.k_pbar_le = vec3_t();
				end.k_lbar_le = 0.0;
			}
		}

		p_rhs = (pbar + C*(p - pbar) + (S/lbar)*v);
		v_rhs = (lbar*S*(p - pbar) + C*v);
	}
	else{
		k_p_p    = 1.0;
		k_p_v    = tau;
		k_p_tau  = v - ez*tau;
		k_p_pbar = 0.0;
		k_p_lbar = vec3_t();

		k_v_p    = 0.0;
		k_v_v    = 1.0;
		k_v_tau  = -ez;
		k_v_pbar = 0.0;
		k_p_lbar = vec3_t();

		for(End& end : ends){
			end.k_pbar_pe = 0.0;
			end.k_pbar_le = vec3_t();
			end.k_lbar_le = 0.0;
		}

		p_rhs = p + v*tau - (1.0/2.0)*ez*tau*tau;
		v_rhs = v - ez*tau;
	}

	q_rhs = q*quat_t::Rot(q.Conjugated()*(w*tau));
	w_rhs = w + msum*tau;
	
	var_pos_r->locked = true;
	var_vel_r->locked = true;

	// set range constraint of duration
	if(next){
		CentroidKey* key1 = (CentroidKey*)next;
		if(tick->idx == 0 || iend == key1->iend){
			con_duration_range->_min =  0.0;
			con_duration_range->_max =  inf;
		}
		else{
			con_duration_range->_min = -inf;
			con_duration_range->_max =  inf;
		}

		con_vel_match->weight = 0.1 * one;
	
		// set stiffness and moment variables
		for(int i = 0; i < ends.size(); i++){
			End& end = ends[i];

			if(end.contact){
				end.con_contact->_min = 0.0;
				end.con_contact->_max = 0.0;
			}
			else{
				end.con_contact->_min = 0.0;
				end.con_contact->_max = inf;
			}

			//end.con_pos->enabled = end.contact;

			/*if(!end.contact){
				end.var_stiff ->val = 0.0;
				end.var_moment->val.clear();
			}*/

			//end.var_stiff ->locked = !end.contact;
			//end.var_moment->locked = !end.contact;
			//end.var_vel   ->locked =  end.contact;

			/*if(end.contact){
				end.var_vel->val = vec3_t();
			}*/

			//for(int j = 0; j < 3; j++)
			//	end.con_vel_range[j]->enabled = !end.contact;

			//end.con_pos_match->enabled    = (end.contact);
			//end.con_stiff_match->enabled  = /*false;*/(end.contact && key1->ends[i].contact);
			//end.con_moment_match->enabled = /*false;*/(end.contact && key1->ends[i].contact);

			//end.con_force_range[0]->weight[0] = 0.001;
			//end.con_force_range[1]->weight[0] = 0.001;
			//end.con_force_range[2]->weight[0] = 0.001;
			//end.con_stiff_zero->desired   = 0.0;
			//end.con_stiff_zero->weight[0] = 0.0;
		}
	}
}

void CentroidKey::Swap(CentroidKey* key){
	swap(iend         , key->iend         );
	swap(var_time->val, key->var_time->val);
}

void CentroidKey::Draw(Render::Canvas* canvas, Render::Config* conf) {
	Vec3f p;

	canvas->SetPointSize(5.0f);
	canvas->SetLineWidth(1.0f);

	p = var_pos_t->val*cen->L;
	canvas->Point(p);

	p = pbar*cen->L;
	canvas->Point(p);
}

//-------------------------------------------------------------------------------------------------
// Centroid

Centroid::Face::Face(const vec2_t _rmin, const vec2_t _rmax, real_t _h){
	rangeMin = _rmin;
	rangeMax = _rmax;
	height   = _h;
}

Centroid::Param::Param() {
	g  = 9.8;
	m  = 1.0;
	I  = 1.0;
}

//-------------------------------------------------------------------------------------------------

Centroid::Waypoint::Waypoint() {
	k    = 0;
	time = 0.0;
}

Centroid::Waypoint::Waypoint(
	int _k, 
	real_t _time, vec3_t _pos_t, quat_t _pos_r, vec3_t _vel_t, vec3_t _vel_r, 
	bool _fix_pos_t, bool _fix_pos_r, bool _fix_vel_t, bool _fix_vel_r)
{
	k         = _k;
	time      = _time;
	pos_t     = _pos_t;
	pos_r     = _pos_r;
	vel_t     = _vel_t;
	vel_r     = _vel_r;
	fix_pos_t = _fix_pos_t;
	fix_pos_r = _fix_pos_r;
	fix_vel_t = _fix_vel_t;
	fix_vel_r = _fix_vel_r;
}

Centroid::Waypoint::End::End(vec3_t _pos, vec3_t _vel, bool _fix_pos, bool _fix_vel){
	pos     = _pos;
	vel     = _vel;
	fix_pos = _fix_pos;
	fix_vel = _fix_vel;
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

void Centroid::Init() {
	TrajectoryNode::Init();

	// calc normalized params
	L = sqrt(param.I/param.m);
	T = sqrt(L/param.g);
	V = L/T;
	F = param.m*param.g;
	M = F*L;

	// end effector range
	for (int k = 0; k < graph->ticks.size(); k++) {
		CentroidKey* key = (CentroidKey*)traj.GetKeypoint(graph->ticks[k]);

		key->iend = (k % key->ends.size());

		for(int i = 0; i < key->ends.size(); i++){
			for(int j = 0; j < 3; j++){
				key->ends[i].con_pos_range[j]->_min = param.ends[i].posRangeMin[j]/L;
				key->ends[i].con_pos_range[j]->_max = param.ends[i].posRangeMax[j]/L;

				key->ends[i].con_vel_range[j]->_min = param.ends[i].velRangeMin[j]/V;
				key->ends[i].con_vel_range[j]->_max = param.ends[i].velRangeMax[j]/V;

				key->ends[i].con_moment_range[j]->_min = param.ends[i].momentRangeMin[j]/M;
				key->ends[i].con_moment_range[j]->_max = param.ends[i].momentRangeMax[j]/M;
			}
		}
	}

	// initialize position and velocity values by spline curve connecting the waypoints
	Curve3d          curve_t;
	QuatCurved       curve_r;
	vector<Curve3d>  curve_end;
	
	curve_t.SetType(Interpolate::Cubic);
	curve_r.SetType(Interpolate::Cubic);
	curve_end.resize(param.ends.size());
	for(int j = 0; j < param.ends.size(); j++){
		curve_end[j].SetType(Interpolate::Cubic);
	}

	for (uint i = 0; i < waypoints.size(); i++) {
		Waypoint& wp = waypoints[i];
		//CentroidKey* key = (CentroidKey*)traj.GetKeypoint(graph->ticks[wp.k]);
		//real_t t = key->tick->time;

		curve_t.AddPoint(wp.time);
		curve_t.SetPos(i, wp.pos_t);
		curve_t.SetVel(i, wp.vel_t);

		curve_r.AddPoint(wp.time);
		curve_r.SetPos(i, wp.pos_r);
		curve_r.SetVel(i, wp.vel_r);

		for(int j = 0; j < param.ends.size(); j++){
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
		key->var_pos_r->val = curve_r.CalcPos(t)/L;
		key->var_vel_t->val = curve_t.CalcVel(t)/V;
		key->var_vel_r->val = curve_r.CalcVel(t)/V;

		key->var_time->val = t/T;
		if(key->next){
			key->var_duration->val = (key->next->tick->time - t)/T;
		}

		for(int i = 0; i < key->ends.size(); i++){
			key->ends[i].var_pos->val = curve_end[i].CalcPos(t)/L;
			key->ends[i].var_vel->val = curve_end[i].CalcVel(t)/V;
			key->ends[i].var_stiff->val = 1.0/(key->var_pos_t->val.z - key->ends[i].var_pos->val.z);

			key->ends[i].var_moment->val.clear();
			//key->ends[i].SetVel(curve_end[i].CalcVel(t));
		}

		// fix initial state
		if(k == 0){
			key->var_pos_t->locked = true;
			key->var_pos_r->locked = true;
			key->var_vel_t->locked = true;
			key->var_vel_r->locked = true;
			key->var_time ->locked = true;
		}
	}

	// fix variables on waypoints
	for (uint i = 0; i < waypoints.size(); i++) {
		Waypoint& wp = waypoints[i];
		CentroidKey* key = (CentroidKey*)traj.GetKeypoint(graph->ticks[wp.k]);

		key->var_pos_t->locked = wp.fix_pos_t;
		key->var_pos_r->locked = wp.fix_pos_r;
		key->var_vel_t->locked = wp.fix_vel_t;
		key->var_vel_r->locked = wp.fix_vel_r;

		for(int j = 0; j < key->ends.size(); j++){
			key->ends[j].var_pos->locked = wp.ends[j].fix_pos;
			//key->ends[j].var_vel[0]->locked = wp.ends[j].fix_vel;
			//key->ends[j].var_vel[1]->locked = wp.ends[j].fix_vel;
			//key->ends[j].var_vel[2]->locked = wp.ends[j].fix_vel;
		}
	}
}
/*
struct CompByTime{
	bool operator()(const Keypoint* lhs, const Keypoint* rhs){
		CentroidKey* key0 = (CentroidKey*)lhs;
		CentroidKey* key1 = (CentroidKey*)rhs;

		// ordering of keypoints of the same end-effector shall not be changed
		if(key0->iend == key1->iend)
			return key0->tick->idx < key1->tick->idx;

		return  (key0->var_time->val <  key1->var_time->val) ||
			   ((key0->var_time->val == key1->var_time->val) && (key0->iend < key1->iend));
	}
};

struct CompByIndex{
	bool operator()(const Tick* lhs, const Tick* rhs){
		return lhs->idx < rhs->idx;
	}
};
*/
void Centroid::Prepare() {
	for (uint k = 0; k < graph->ticks.size(); k++) {
		CentroidKey* key = (CentroidKey*)traj.GetKeypoint(graph->ticks[k]);
		key->var_time->val = std::max(0.0, key->var_time->val);
	}

	while(true){
		bool swapped = false;
		for (uint k = 0; k < graph->ticks.size()-1; k++) {
			CentroidKey* key0 = (CentroidKey*)traj.GetKeypoint(graph->ticks[k]);
			CentroidKey* key1 = (CentroidKey*)key0->next;

			if(key0->var_time->val > key1->var_time->val){
				key0->Swap(key1);
				swapped = true;
			}
		}	
		if(!swapped)
			break;
	}

	/*
	// sort keypoints
	sort(traj.begin(), traj.end(), CompByTime());
	// reassign indices
	for(int k = 0; k < traj.size(); k++){
		if(traj[k]->tick->idx != k)
			DSTR << "order changed!" << endl;
		traj[k]->tick->idx = k;
		traj[k]->prev = (k > 0             ? traj[k-1] : 0);
		traj[k]->next = (k < traj.size()-1 ? traj[k+1] : 0);
	}
	sort(graph->ticks.begin(), graph->ticks.end(), CompByIndex());
	*/

	// reset time values
	real_t t = 0.0;
	for (uint k = 0; k < graph->ticks.size(); k++) {
		CentroidKey* key = (CentroidKey*)traj.GetKeypoint(graph->ticks[k]);
		key->tick->time = key->var_time->val*T;
		if(key->next){
			key->var_duration->val = ((CentroidKey*)key->next)->var_time->val - key->var_time->val;
		}

		DSTR << k << " " << key->var_time->val << " " << key->var_duration->val << endl;
	}

	// update contact state
	vector<bool> contact(param.ends.size(), false);
	bool contact_map[][2] = {{1,1}, {1,0}, {1,1}, {0,1}};
	for (uint k = 0; k < graph->ticks.size(); k++) {
		CentroidKey* key = (CentroidKey*)traj.GetKeypoint(graph->ticks[k]);
		
		contact[key->iend] = !contact[key->iend];
		for(int i = 0; i < key->ends.size(); i++){
			//key->ends[i].contact = contact_map[k%4][i]; 
			key->ends[i].contact = contact[i];
			//key->ends[i].contact = !(k%2)&&(i==0);
		}
	}

	TrajectoryNode::Prepare();
	trajReady = false;

}

void Centroid::Finish(){
	for (uint k = 0; k < graph->ticks.size(); k++) {
		CentroidKey* key = (CentroidKey*)traj.GetKeypoint(graph->ticks[k]);
		key->tick->time = key->var_time->val*T;
	}

	TrajectoryNode::Finish();
}

Centroid::Face* Centroid::FindFace(const vec3_t& p){
	Face* fnear = 0;
	for(Face& f : faces){
		if( (f.rangeMin.x <= p.x && p.x <= f.rangeMax.x) &&
			(f.rangeMin.y <= p.y && p.y <= f.rangeMax.y) ){
			if(!fnear || fnear->height < f.height)
				fnear = &f;
		}
	}

	return fnear;
}	

vec3_t Centroid::ComPos(real_t t, int type) {
	if(traj.empty())
		return vec3_t();

	KeyPair      kp = traj.GetSegment(t);
	CentroidKey* k0 = (CentroidKey*)kp.first;
	CentroidKey* k1 = (CentroidKey*)kp.second;

	real_t dt = t/T - k0->var_time->val;
	vec3_t pt;
	if(k0->ncon > 0){
		real_t Ct = cosh(k0->lbar*(dt));
		real_t St = sinh(k0->lbar*(dt));
		pt = k0->pbar + Ct*(k0->p - k0->pbar) + (St/k0->lbar)*k0->v;
	}
	else{
		pt = k0->p + k0->v*dt - (1.0/2.0)*vec3_t(0.0, 0.0, 1.0)*(dt*dt);
	}

	return L*pt;
	//return InterpolatePos(
	//	t,
	//	k0->var_time->val*T, k0->var_pos_t->val*L, k0->var_vel_t->val*V,
	//	k1->var_time->val*T, k1->var_pos_t->val*L, k1->var_vel_t->val*V,
	//	type);
}

vec3_t Centroid::ComVel(real_t t, int type) {
	if(traj.empty())
		return vec3_t();

	KeyPair      kp = traj.GetSegment(t);
	CentroidKey* k0 = (CentroidKey*)kp.first;
	CentroidKey* k1 = (CentroidKey*)kp.second;

	real_t dt = t/T - k0->var_time->val;
	vec3_t vt;
	if(k0->ncon > 0){
		real_t Ct = cosh(k0->lbar*(dt));
		real_t St = sinh(k0->lbar*(dt));
		vt = k0->pbar + Ct*(k0->p - k0->pbar) + (St/k0->lbar)*k0->v;
	}
	else{
		vt = k0->v - vec3_t(0.0, 0.0, 1.0)*dt;
	}

	return V*vt;
	//return InterpolateVel(
	//	t,
	//	k0->var_time->val*T, k0->var_pos_t->val*L, k0->var_vel_t->val*V,
	//	k1->var_time->val*T, k1->var_pos_t->val*L, k1->var_vel_t->val*V,
	//	type);
}

quat_t Centroid::ComOri(real_t t, int type) {
	if(traj.empty())
		return quat_t();

	KeyPair      kp = traj.GetSegment(t);
	CentroidKey* k0 = (CentroidKey*)kp.first;
	CentroidKey* k1 = (CentroidKey*)kp.second;

	return InterpolateOri(
		t,
		k0->var_time->val*T, k0->var_pos_r->val, k0->var_vel_r->val/T,
		k1->var_time->val*T, k1->var_pos_r->val, k1->var_vel_r->val/T,
		type);
}

vec3_t Centroid::ComAngVel(real_t t, int type) {
	if(traj.empty())
		return vec3_t();

	KeyPair      kp = traj.GetSegment(t);
	CentroidKey* k0 = (CentroidKey*)kp.first;
	CentroidKey* k1 = (CentroidKey*)kp.second;

	return InterpolateAngvel(
		t,
		k0->var_time->val*T, k0->var_pos_r->val, k0->var_vel_r->val/T,
		k1->var_time->val*T, k1->var_pos_r->val, k1->var_vel_r->val/T,
		type);
}

vec3_t Centroid::EndPos(real_t t, int index, int type) {
	if(traj.empty())
		return vec3_t();

	KeyPair      kp = traj.GetSegment(t);
	CentroidKey* k0 = (CentroidKey*)kp.first;
	CentroidKey* k1 = (CentroidKey*)kp.second;

	return InterpolatePos(
		t,
		k0->var_time->val*T, k0->ends[index].var_pos->val*L, vec3_t()/*k0->ends[index].var_vel->val*/,
		k1->var_time->val*T, k1->ends[index].var_pos->val*L, vec3_t()/*k1->ends[index].var_vel->val*/,
		type);
}
/*
vec3_t Centroid::EndForce(real_t t, int index, int type) {
	if(traj.empty())
		return vec3_t();

	KeyPair      kp = traj.GetSegment(t);
	CentroidKey* k0 = (CentroidKey*)kp.first;
	CentroidKey* k1 = (CentroidKey*)kp.second;

	return InterpolatePos(
		t,
		k0->tick->time, k0->ends[index].GetForce()*F, vec3_t(),
		k1->tick->time, k1->ends[index].GetForce()*F, vec3_t(),
		type);
}
*/
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
		for(int i = 0; i < param.ends.size(); i++){
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

	for(Face& f : faces){
		canvas->BeginLayer("centroid_face", true);
		canvas->BeginPath();
		canvas->MoveTo(vec3_t(f.rangeMin.x, f.rangeMin.y, f.height));
		canvas->LineTo(vec3_t(f.rangeMax.x, f.rangeMin.y, f.height));
		canvas->LineTo(vec3_t(f.rangeMax.x, f.rangeMax.y, f.height));
		canvas->LineTo(vec3_t(f.rangeMin.x, f.rangeMax.y, f.height));
		canvas->LineTo(vec3_t(f.rangeMin.x, f.rangeMin.y, f.height));
		canvas->EndPath();
		canvas->EndLayer();
	}
}

void Centroid::CreateSnapshot(real_t t, Centroid::Snapshot& s){
	s.t = t;
	s.pos = ComPos(t);
	s.vel = ComVel(t);

	s.ends.resize(param.ends.size());
	for(int i = 0; i < param.ends.size(); i++){
		s.ends[i].pos   = EndPos  (t, i);
		//s.ends[i].force = EndForce(t, i);
	}
}

void Centroid::CreateSnapshot(real_t t){
	CreateSnapshot(t, snapshot);
}

void Centroid::DrawSnapshot(Render::Canvas* canvas, Render::Config* conf) {
	if (conf->Set(canvas, Render::Item::CentroidEnd, this)) {
		// lines connecting com and feet
		for(int i = 0; i < param.ends.size(); i++){
			canvas->BeginPath();
			canvas->MoveTo(snapshot.pos);
			canvas->LineTo(snapshot.ends[i].pos);
			canvas->EndPath();
		}
	}
	/*if (conf->Set(canvas, Render::Item::CentroidForce, this)) {
		for(int i = 0; i < param.ends.size(); i++){
			canvas->BeginPath();
			canvas->MoveTo(snapshot.ends[i].pos);
			canvas->LineTo(snapshot.ends[i].pos + snapshot.ends[i].force);
			canvas->EndPath();
		}
	}*/
	
}

///////////////////////////////////////////////////////////////////////////////////////////////////

CentroidCon::CentroidCon(Solver* solver, int _dim, int _tag, string _name, CentroidKey* _obj, real_t _scale):
	Constraint(solver, _dim, ID(_tag, _obj->node, _obj->tick, _name), _scale) {
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

/*
CentroidEndStiffCon::CentroidEndStiffCon(Solver* solver, int _tag, string _name, CentroidKey* _obj, int _iend, real_t _scale):
	CentroidCon(solver, 1, ConTag::CentroidTime, _name, _obj, _scale) {
	iend = _iend;

	AddSLink(obj[1]->ends[iend].var_stiff);
	AddSLink(obj[0]->ends[iend].var_stiff);
	AddSLink(obj[0]->ends[iend].var_stiff_diff);
}

CentroidEndMomentCon::CentroidEndMomentCon(Solver* solver, int _tag, string _name, CentroidKey* _obj, int _iend, real_t _scale):
	CentroidCon(solver, 1, ConTag::CentroidTime, _name, _obj, _scale) {
	iend = _iend;

	AddSLink(obj[1]->ends[iend].var_moment);
	AddSLink(obj[0]->ends[iend].var_moment);
	AddSLink(obj[0]->ends[iend].var_moment_diff);
}
*/
CentroidEndPosRangeCon::CentroidEndPosRangeCon(Solver* solver, string _name, CentroidKey* _obj, int _iend, vec3_t _dir, real_t _scale):
	Constraint(solver, 1, ID(ConTag::CentroidEndRange, _obj->node, _obj->tick, _name), _scale){
	obj  = _obj;
	iend = _iend;
	dir  = _dir;

	AddR3Link(obj->var_pos_t);
	AddR3Link(obj->var_pos_r);
	AddR3Link(obj->ends[iend].var_pos);
}

CentroidEndVelRangeCon::CentroidEndVelRangeCon(Solver* solver, string _name, CentroidKey* _obj, int _iend, vec3_t _dir, real_t _scale):
	Constraint(solver, 1, ID(ConTag::CentroidEndRange, _obj->node, _obj->tick, _name), _scale){
	obj  = _obj;
	iend = _iend;
	dir  = _dir;

	AddR3Link(obj->var_pos_r);
	AddR3Link(obj->ends[iend].var_vel);
}

CentroidEndContactCon::CentroidEndContactCon(Solver* solver, string _name, CentroidKey* _obj, int _iend, real_t _scale):
	Constraint(solver, 1, ID(ConTag::CentroidEndContact, _obj->node, _obj->tick, _name), _scale){
	obj  = _obj;
	iend = _iend;

	AddR3Link(obj->ends[iend].var_pos);
}
/*
CentroidEndForceRangeCon::CentroidEndForceRangeCon(Solver* solver, int _tag, string _name, CentroidKey* _obj, int _iend, vec3_t _dir, real_t _scale):
	Constraint(solver, 1, ID(_tag, _obj->node, _obj->tick, _name), _scale){
	obj  = _obj;
	iend = _iend;
	dir  = _dir;
	
	AddR3Link(obj->var_pos_t);
	AddR3Link(obj->ends[iend].var_pos);
	AddSLink (obj->ends[iend].var_stiff);
}
*/
///////////////////////////////////////////////////////////////////////////////////////////////////

void CentroidEndPosRangeCon::Prepare(){
	p       = obj->var_pos_t->val;
	q       = obj->var_pos_r->val;
	pend    = obj->ends[iend].var_pos->val;
	dp      = pend - p;
	dir_abs = q*dir;
}

void CentroidEndVelRangeCon::Prepare(){
	q       = obj->var_pos_r->val;
	vend    = obj->ends[iend].var_vel->val;
	dir_abs = q*dir;
}

void CentroidEndContactCon::Prepare(){
	face = obj->cen->FindFace(obj->ends[iend].var_pos->val);
}
/*
void CentroidEndForceRangeCon::Prepare(){
	le = obj->ends[iend].var_stiff->val;
	p  = obj->var_pos_t->val;
	pe = obj->ends[iend].var_pos->val;	
}
*/
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
	if(obj[0]->ends[iend].contact){
		((SLink *)links[0])->SetCoef( 1.0);
		((SLink *)links[1])->SetCoef(-1.0);
		((SLink *)links[2])->SetCoef( 0.0);
		((C3Link*)links[3])->SetCoef( vec3_t());
	}
	else{
		((SLink *)links[0])->SetCoef( 1.0);
		((SLink *)links[1])->SetCoef(-1.0);
		((SLink *)links[2])->SetCoef(-obj[0]->tau);
		((C3Link*)links[3])->SetCoef(-obj[0]->ends[iend].var_vel->val);
	}
}

/*
void CentroidEndStiffCon::CalcCoef(){
	((SLink*)links[0])->SetCoef( 1.0);
	((SLink*)links[1])->SetCoef(-1.0);
	((SLink*)links[2])->SetCoef(-1.0);
}

void CentroidEndMomentCon::CalcCoef(){
	((SLink*)links[0])->SetCoef( 1.0);
	((SLink*)links[1])->SetCoef(-1.0);
	((SLink*)links[2])->SetCoef(-1.0);
}
*/
void CentroidEndPosRangeCon::CalcCoef(){
	Prepare();

	((R3Link*)links[0])->SetCoef(-dir_abs     );
	((R3Link*)links[1])->SetCoef( dir_abs % dp);
	((R3Link*)links[2])->SetCoef( dir_abs     );
}

void CentroidEndVelRangeCon::CalcCoef(){
	Prepare();

	((R3Link*)links[0])->SetCoef( dir_abs % vend);
	((R3Link*)links[1])->SetCoef( dir_abs       );
}

void CentroidEndContactCon::CalcCoef(){
	Prepare();

	((R3Link*)links[0])->SetCoef(vec3_t(0.0, 0.0, 1.0));
}
/*
void CentroidEndForceRangeCon::CalcCoef(){
	Prepare();

	((R3Link*)links[0])->SetCoef( le*dir);
	((R3Link*)links[1])->SetCoef(-le*dir);
	((SLink *)links[2])->SetCoef( dir*(p - pe));
}
*/
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
	if(obj[0]->ends[iend].contact){
		y = obj[1]->ends[iend].var_pos->val - (obj[0]->ends[iend].var_pos->val);
	}
	else{
		y = obj[1]->ends[iend].var_pos->val - (obj[0]->ends[iend].var_pos->val + obj[0]->ends[iend].var_vel->val*obj[0]->var_duration->val);
	}
}
/*

void CentroidEndStiffCon::CalcDeviation(){
	y[0] = obj[1]->ends[iend].var_stiff->val - (obj[0]->ends[iend].var_stiff->val + obj[0]->ends[iend].var_stiff_diff->val);
}

void CentroidEndMomentCon::CalcDeviation(){
	y = obj[1]->ends[iend].var_moment->val - (obj[0]->ends[iend].var_moment->val + obj[0]->ends[iend].var_moment_diff->val);
}
*/
void CentroidEndPosRangeCon::CalcDeviation(){
	y[0]   = 0.0;
	on_lower = on_upper = active = false;

	real_t r = dir_abs*dp;
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
}

void CentroidEndVelRangeCon::CalcDeviation(){
	y[0]   = 0.0;
	on_lower = on_upper = active = false;

	real_t r = dir_abs*vend;
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
}

void CentroidEndContactCon::CalcDeviation(){
	y[0]   = 0.0;
	on_lower = on_upper = active = false;

	real_t h = (face ? face->height : 0.0);
	real_t r = obj->ends[iend].var_pos->val.z - h;
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
}
/*
void CentroidEndForceRangeCon::CalcDeviation(){
	y[0] = dir*(le*(p - pe));
}
*/
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
	obj[1]->ends[iend].var_pos->val = obj[0]->ends[iend].var_pos->val + obj[0]->ends[iend].var_vel->val*obj[0]->var_duration->val;
}
/*
void CentroidEndStiffCon::CalcLhs(){
	obj[1]->ends[iend].var_stiff->val = obj[0]->ends[iend].var_stiff->val + obj[0]->ends[iend].var_stiff_diff->val;
}

void CentroidEndMomentCon::CalcLhs(){
	obj[1]->ends[iend].var_moment->val = obj[0]->ends[iend].var_moment->val + obj[0]->ends[iend].var_moment_diff->val;
}
*/
///////////////////////////////////////////////////////////////////////////////////////////////////

void CentroidEndPosRangeCon::Project(real_t& l, uint k) {
	if ( on_upper && l > 0.0  ) l = 0.0;
	if ( on_lower && l < 0.0  ) l = 0.0;
	if (!on_upper && !on_lower) l = 0.0;
}

void CentroidEndVelRangeCon::Project(real_t& l, uint k) {
	if ( on_upper && l > 0.0  ) l = 0.0;
	if ( on_lower && l < 0.0  ) l = 0.0;
	if (!on_upper && !on_lower) l = 0.0;
}

void CentroidEndContactCon::Project(real_t& l, uint k) {
	if ( on_upper && l > 0.0  ) l = 0.0;
	if ( on_lower && l < 0.0  ) l = 0.0;
	if (!on_upper && !on_lower) l = 0.0;
}

}
