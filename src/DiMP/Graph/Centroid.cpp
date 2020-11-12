#include <DiMP/Graph/Centroid.h>
#include <DiMP/Graph/Graph.h>
#include <DiMP/Render/Config.h>
#include <DiMP/Render/Canvas.h>

namespace DiMP {;

const real_t pi  = M_PI;
const real_t inf = numeric_limits<real_t>::max();
const real_t eps = 1.0e-10;

//-------------------------------------------------------------------------------------------------
// CentroidKey

CentroidKey::CentroidKey() {
	
}

void CentroidKey::AddVar(Solver* solver) {
	Centroid* obj = (Centroid*)node;
	
	// position and velocity
	var_pos_t = new V3Var(solver, ID(VarTag::CentroidTP, node, tick, name + "_tp"), node->graph->scale.pos_t);
	var_pos_r = new QVar (solver, ID(VarTag::CentroidRP, node, tick, name + "_rp"), node->graph->scale.pos_r);
	var_vel_t = new V3Var(solver, ID(VarTag::CentroidTV, node, tick, name + "_tv"), node->graph->scale.vel_t);
	var_vel_r = new V3Var(solver, ID(VarTag::CentroidRV, node, tick, name + "_rv"), node->graph->scale.vel_r);
	
	ends    .resize(obj->param.ends    .size());
	contacts.resize(obj->param.contacts.size());

	stringstream ss;

	for(int i = 0; i < ends.size(); i++){
		ss.str("");
		ss << name << "_end" << i;
		ends[i].var_pos = new V3Var(solver, ID(VarTag::CentroidEndPos, node, tick, ss.str() + "_pos" ), node->graph->scale.pos_t);
		ends[i].var_vel = new V3Var(solver, ID(VarTag::CentroidEndVel, node, tick, ss.str() + "_vel" ), node->graph->scale.vel_t);
	}

	for(int i = 0; i < contacts.size(); i++){
		ss.str("");
		ss << name << "_con" << i;
		contacts[i].var_active[0] = new SVar(solver, ID(VarTag::CentroidContactCmpl , node, tick, ss.str() + "_cmpl0" ), 1.0);
		contacts[i].var_active[1] = new SVar(solver, ID(VarTag::CentroidContactCmpl , node, tick, ss.str() + "_cmpl1" ), 1.0);
		contacts[i].var_force [0] = new SVar(solver, ID(VarTag::CentroidContactForce, node, tick, ss.str() + "_force0"), node->graph->scale.force_t);
		contacts[i].var_force [1] = new SVar(solver, ID(VarTag::CentroidContactForce, node, tick, ss.str() + "_force1"), node->graph->scale.force_t);
		contacts[i].var_force [2] = new SVar(solver, ID(VarTag::CentroidContactForce, node, tick, ss.str() + "_force1"), node->graph->scale.force_t);
	}
}

void CentroidKey::AddCon(Solver* solver) {
	Centroid* obj = (Centroid*)node;
	CentroidKey* nextObj = (CentroidKey*)next;

	if(next){
		con_pos_t = new CentroidPosConT(solver, ConTag::CentroidPosT, name + "_pos_t", this, node->graph->scale.pos_t);
		con_pos_r = new CentroidPosConR(solver, ConTag::CentroidPosR, name + "_pos_r", this, node->graph->scale.pos_r);
		con_vel_t = new CentroidVelConT(solver, ConTag::CentroidVelT, name + "_vel_t", this, node->graph->scale.vel_t);
		con_vel_r = new CentroidVelConR(solver, ConTag::CentroidVelR, name + "_vel_r", this, node->graph->scale.vel_r);
	}

	stringstream ss;

	for(int i = 0; i < ends.size(); i++){
		ss.str("");
		ss << name << "_end" << i;
		ends[i].con_range[0] = new CentroidEndRangeCon(solver, ConTag::CentroidEndRange, ss.str() + "_range0", this, i, vec3_t(1.0, 0.0, 0.0), node->graph->scale.pos_t);
		ends[i].con_range[1] = new CentroidEndRangeCon(solver, ConTag::CentroidEndRange, ss.str() + "_range1", this, i, vec3_t(0.0, 1.0, 0.0), node->graph->scale.pos_t);
		ends[i].con_range[2] = new CentroidEndRangeCon(solver, ConTag::CentroidEndRange, ss.str() + "_range2", this, i, vec3_t(0.0, 0.0, 1.0), node->graph->scale.pos_t);
		ends[i].con_cmpl     = new CentroidEndCmplCon (solver, ConTag::CentroidEndCmpl , ss.str() + "_cmpl"  , this, i, 1.0);

		if(next){
			ends[i].con_pos = new CentroidEndPosCon  (solver, ConTag::CentroidEndPos  , ss.str() + "_pos"   , this, i, node->graph->scale.pos_t);
		}
	}

	for(int i = 0; i < contacts.size(); i++){
		ss.str("");
		ss << name << "_con" << i;

		contacts[i].con_active[0] = new RangeConS(solver, ID(ConTag::CentroidContactCmpl, node, tick, ss.str() + "_active0"), contacts[i].var_active[0], 1.0);
		contacts[i].con_active[1] = new RangeConS(solver, ID(ConTag::CentroidContactCmpl, node, tick, ss.str() + "_active1"), contacts[i].var_active[1], 1.0);
		contacts[i].con_active[0]->_min = 0.0;
		contacts[i].con_active[0]->_max = 1.0;
		contacts[i].con_active[1]->_min = 0.0;
		contacts[i].con_active[1]->_max = 1.0;

		contacts[i].con_cmpl      = new CentroidContactCmplCon(solver, ConTag::CentroidContactCmpl, ss.str() + "_cmpl", this, i, 1.0);
		contacts[i].con_gap       = new CentroidContactGapCon (solver, ConTag::CentroidContactGap , ss.str() + "_gap" , this, i, node->graph->scale.pos_t);

		if(next){
			contacts[i].con_force[0] = new CentroidContactForceCon(solver, ConTag::CentroidContactForce, ss.str() + "_force0", this, i, 0, node->graph->scale.force_t);
			contacts[i].con_force[1] = new CentroidContactForceCon(solver, ConTag::CentroidContactForce, ss.str() + "_force1", this, i, 1, node->graph->scale.force_t);
			contacts[i].con_force[1] = new CentroidContactForceCon(solver, ConTag::CentroidContactForce, ss.str() + "_force2", this, i, 2, node->graph->scale.force_t);
		}
	}
}		

void CentroidKey::Prepare() {
	Centroid* obj = (Centroid*)node;

	real_t mu = obj->param.mu;
	
	for(Contact& con : contacts){
		if(next){
			con.f = vec3_t(
				con.var_force[0]->val,
				con.var_force[1]->val,
				con.var_force[2]->val);
		}

		DSTR << tick->idx << " " << con.f << " " << con.var_active[0]->val << " " << con.var_active[1]->val << endl;
	}
}

void CentroidKey::Draw(Render::Canvas* canvas, Render::Config* conf) {
	Centroid* obj = (Centroid*)node;

	Vec3f p;

	canvas->SetPointSize(5.0f);
	canvas->SetLineWidth(1.0f);

	p = var_pos_t->val;
	canvas->Point(p);
}

//-------------------------------------------------------------------------------------------------
// Centroid

Centroid::Param::Param() {
	g  = 9.8;
	m  = 1.0;
	I  = 1.0;
	h  = 0.1;
}
/*
void Centroid::Param::Face::CalcNearest(const vec3_t& p, vec3_t& pc, int& iedge, int& ivtx){
	iedge = -1;
	ivtx  = -1;

	bool inside = true;
	for(Edge& e : edges){
		if(e.n*(p - e.v[0]) > 0.0)
			inside = false;
	}

	// projected on face
	if(inside){
		vec3_t n = R.col(2);
		pc = p - (n*(p - origin))*n;
		return;
	}

	for(int i = 0; i < edges.size(); i++){
		Edge& e = edges[i];
		vec3_t d = e.v[1] - e.v[0];
		real_t s = ((p - e.v[0])*d)/(d.square());
		if( 0.0 <= s && s <= 1.0 ){
			pc = e.v[0] + s*d;
			iedge = i;
			return;
		}
	}

	real_t d2min = inf;
	vec3_t vmin;
	for(int i = 0; i < edges.size(); i++){
		Edge& e = edges[i];
		for(int j = 0; j < 2; j++){
			real_t d2 = (p - e.v[j]).square();
			if(d2 < d2min){
				d2min = d2;
				vmin  = e.v[j];
				iedge = i;
				ivtx  = j;
			}
		}
	}
	pc = vmin;
}
*/
//-------------------------------------------------------------------------------------------------

Centroid::Waypoint::Waypoint() {
	k = 0;
	time = 0.0;
}

Centroid::Waypoint::Waypoint(
	int _k, real_t _time, vec3_t _pos_t, quat_t _pos_r, vec3_t _vel_t, vec3_t _vel_r, 
	bool _fix_pos_t, bool _fix_pos_r, bool _fix_vel_t, bool _fix_vel_r)
{
	k = _k;
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

void Centroid::AddVar(){
	stringstream ss;

	contacts.resize(param.contacts.size());
	for(int i = 0; i < contacts.size(); i++){
		ss.str("");
		ss << name << "_con" << i;
		contacts[i].var_pos = new V3Var(graph->solver, ID(VarTag::CentroidContactPos, this, 0, ss.str() + "_pos"), graph->scale.pos_t);
	}

	TrajectoryNode::AddVar();
}

void Centroid::AddCon(){
	TrajectoryNode::AddCon();
}

void Centroid::Init() {
	TrajectoryNode::Init();

	// calc normalized params
	L     = sqrt(param.I/param.m);
	T     = sqrt(L/param.g);
	hnorm = param.h/T;

	/*
	// create face edges
	for(Param::Face& f : param.faces){
		int nvtx = f.vertices.size();
		
		f.origin.clear();
		for(int i = 0; i < nvtx; i++)
			f.origin += f.vertices[i];
		f.origin *= (1.0/(real_t)nvtx);

		vec3_t nf = (f.vertices[0] - f.origin) % (f.vertices[1] - f.origin);
		nf.unitize();
		vec3_t tf0 = vec3_t(1.0, 0.0, 0.0);
		vec3_t tf1 = nf % tf0;
		tf0 = tf1 % nf;
		f.R.col(0) = tf0;
		f.R.col(1) = tf1;
		f.R.col(2) = nf;

		f.edges.resize(nvtx);
		for(int i = 0; i < nvtx; i++){
			Param::Edge& e = f.edges[i];
			e.v[0] = f.vertices[(i+0)%nvtx];
			e.v[1] = f.vertices[(i+1)%nvtx];
			e.n    = (e.v[1] - e.v[0]) % nf;
			if(e.n*(e.v[0] - f.origin) < 0.0)
				e.n = -e.n;
		}
	}
	*/
	// end effector range
	for (int k = 0; k < graph->ticks.size(); k++) {
		CentroidKey* key = (CentroidKey*)traj.GetKeypoint(graph->ticks[k]);

		for(int i = 0; i < key->ends.size(); i++){
			for(int j = 0; j < 3; j++){
				key->ends[i].con_range[j]->_min = param.ends[i].rangeMin[j]/L;
				key->ends[i].con_range[j]->_max = param.ends[i].rangeMax[j]/L;
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
		CentroidKey* key = (CentroidKey*)traj.GetKeypoint(graph->ticks[wp.k]);
		real_t t = key->tick->time;

		curve_t.AddPoint(t);
		curve_t.SetPos(i, wp.pos_t);
		curve_t.SetVel(i, wp.vel_t);

		curve_r.AddPoint(t);
		curve_r.SetPos(i, wp.pos_r);
		curve_r.SetVel(i, wp.vel_r);

		for(int j = 0; j < param.ends.size(); j++){
			curve_end[j].AddPoint(t);
			curve_end[j].SetPos(i, wp.ends[j].pos);
			curve_end[j].SetVel(i, wp.ends[j].vel);
		}
	}

	for (uint k = 0; k < graph->ticks.size(); k++) {
		CentroidKey* key = (CentroidKey*)traj.GetKeypoint(graph->ticks[k]);
		real_t t = graph->ticks[k]->time;

		key->var_pos_t->val = curve_t.CalcPos(t)/L;
		key->var_pos_r->val = curve_r.CalcPos(t);
		key->var_vel_t->val = curve_t.CalcVel(t)*(T/L);
		key->var_vel_r->val = curve_r.CalcVel(t)*(T);

		for(int i = 0; i < key->ends.size(); i++){
			key->ends[i].var_pos->val = curve_end[i].CalcPos(t)/L;
			key->ends[i].var_vel->val = curve_end[i].CalcVel(t)*(T/L);
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
			key->ends[j].var_vel->locked = wp.ends[j].fix_vel;
		}
	}

	// set initial position to contact points
	for(int i = 0; i < contacts.size(); i++){
		contacts[i].var_pos->val = param.contacts[i].pos/L;
	}
	
	// set initial values of cmpl variables
	for (uint k = 0; k < graph->ticks.size(); k++) {
		CentroidKey* key = (CentroidKey*)traj.GetKeypoint(graph->ticks[k]);
	
		for(CentroidKey::Contact& con : key->contacts){
			//con.var_cmpl->val = (i == 0 ? sin(20*con.var_pos[0]->val) : -sin(20*con.var_pos[0]->val));
			con.var_active[0]->val = 0.5;
			con.var_active[1]->val = 0.5;

			//vec3_t p = end.var_pos->val;
			//for(int i = 0; i < end.faces.size(); i++){
			//	param.faces[i].CalcNearest(p, end.faces[i].pc, end.faces[i].iedge, end.faces[i].ivtx);
			//
			//	real_t d = (p - end.faces[i].pc).norm();
			//	end.faces[i].var_cmpl->val = log((d + rel)/sqrt_rel);
			//
			//}
		}
	}
}

void Centroid::Prepare() {
	TrajectoryNode::Prepare();
	trajReady = false;
}

void Centroid::Finish(){
	TrajectoryNode::Finish();
}

vec3_t Centroid::ComPos(real_t t, int type) {
	if(traj.empty())
		return vec3_t();

	KeyPair      kp = traj.GetSegment(t);
	CentroidKey* k0 = (CentroidKey*)kp.first;
	CentroidKey* k1 = (CentroidKey*)kp.second;

	return InterpolatePos(
		t,
		k0->tick->time, k0->var_pos_t->val*L, k0->var_vel_t->val*(L/T),
		k1->tick->time, k1->var_pos_t->val*L, k1->var_vel_t->val*(L/T),
		type);
}

vec3_t Centroid::ComVel(real_t t, int type) {
	if(traj.empty())
		return vec3_t();

	KeyPair      kp = traj.GetSegment(t);
	CentroidKey* k0 = (CentroidKey*)kp.first;
	CentroidKey* k1 = (CentroidKey*)kp.second;

	return InterpolateVel(
		t,
		k0->tick->time, k0->var_pos_t->val*L, k0->var_vel_t->val*(L/T),
		k1->tick->time, k1->var_pos_t->val*L, k1->var_vel_t->val*(L/T),
		type);
}

quat_t Centroid::ComOri(real_t t, int type) {
	if(traj.empty())
		return quat_t();

	KeyPair      kp = traj.GetSegment(t);
	CentroidKey* k0 = (CentroidKey*)kp.first;
	CentroidKey* k1 = (CentroidKey*)kp.second;

	return InterpolateOri(
		t,
		k0->tick->time, k0->var_pos_r->val, k0->var_vel_r->val/T,
		k1->tick->time, k1->var_pos_r->val, k1->var_vel_r->val/T,
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
		k0->tick->time, k0->var_pos_r->val, k0->var_vel_r->val/T,
		k1->tick->time, k1->var_pos_r->val, k1->var_vel_r->val/T,
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
		k0->tick->time, k0->ends[index].var_pos->val*L, k0->ends[index].var_vel->val*(L/T),
		k1->tick->time, k1->ends[index].var_pos->val*L, k1->ends[index].var_vel->val*(L/T),
		type);
}

vec3_t Centroid::EndVel(real_t t, int index, int type) {
	if(traj.empty())
		return vec3_t();

	KeyPair      kp = traj.GetSegment(t);
	CentroidKey* k0 = (CentroidKey*)kp.first;
	CentroidKey* k1 = (CentroidKey*)kp.second;

	return InterpolateVel(
		t,
		k0->tick->time, k0->ends[index].var_pos->val*L, k0->ends[index].var_vel->val*(L/T),
		k1->tick->time, k1->ends[index].var_pos->val*L, k1->ends[index].var_vel->val*(L/T),
		type);
}

vec3_t Centroid::ContactForce(real_t t, int index, int type) {
	if(traj.empty())
		return vec3_t();

	KeyPair      kp = traj.GetSegment(t);
	CentroidKey* k0 = (CentroidKey*)kp.first;
	CentroidKey* k1 = (CentroidKey*)kp.second;

	return InterpolatePos(
		t,
		k0->tick->time, k0->contacts[index].f, vec3_t(),
		k1->tick->time, k1->contacts[index].f, vec3_t(),
		type);
}

vec3_t Centroid::ContactPos(int index){
	return contacts[index].var_pos->val*L;
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
}

void Centroid::CreateSnapshot(real_t t, Centroid::Snapshot& s){
	s.t = t;
	s.pos = ComPos(t);

	s.ends    .resize(param.ends    .size());
	s.contacts.resize(param.contacts.size());
	for(int i = 0; i < param.ends.size(); i++){
		s.ends[i].pos = EndPos(t, i);
	}
	for(int i = 0; i < param.contacts.size(); i++){
		s.contacts[i].pos   = ContactPos  (i);
		s.contacts[i].force = ContactForce(t, i);
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
	if (conf->Set(canvas, Render::Item::CentroidForce, this)) {
		for(int i = 0; i < param.contacts.size(); i++){
			canvas->BeginPath();
			canvas->MoveTo(snapshot.contacts[i].pos);
			canvas->LineTo(snapshot.contacts[i].pos + snapshot.contacts[i].force);
			canvas->EndPath();
		}
	}
	
}

///////////////////////////////////////////////////////////////////////////////////////////////////

CentroidCon::CentroidCon(Solver* solver, int _dim, int _tag, string _name, CentroidKey* _obj, real_t _scale):
	Constraint(solver, _dim, ID(_tag, _obj->node, _obj->tick, _name), _scale) {
	obj[0] = _obj;
	obj[1] = (CentroidKey*)_obj->next;
}

CentroidPosConT::CentroidPosConT(Solver* solver, int _tag, string _name, CentroidKey* _obj, real_t _scale):
	CentroidCon(solver, 3, ConTag::CentroidPosT, _name, _obj, _scale) {

	AddSLink(obj[1]->var_pos_t);
	AddSLink(obj[0]->var_pos_t);
	AddSLink(obj[0]->var_vel_t);
}

CentroidPosConR::CentroidPosConR(Solver* solver, int _tag, string _name, CentroidKey* _obj, real_t _scale):
	CentroidCon(solver, 3, ConTag::CentroidPosR, _name, _obj, _scale) {

	AddSLink(obj[1]->var_pos_r);
	AddSLink(obj[0]->var_pos_r);
	AddSLink(obj[0]->var_vel_r);
}

CentroidVelConT::CentroidVelConT(Solver* solver, int _tag, string _name, CentroidKey* _obj, real_t _scale):
	CentroidCon(solver, 3, ConTag::CentroidVelT, _name, _obj, _scale) {

	AddSLink(obj[1]->var_vel_t);
	AddSLink(obj[0]->var_vel_t);
	for(CentroidKey::Contact& con : obj[0]->contacts){
		AddC3Link(con.var_force[0]);
		AddC3Link(con.var_force[1]);
		AddC3Link(con.var_force[2]);
	}
}

CentroidVelConR::CentroidVelConR(Solver* solver, int _tag, string _name, CentroidKey* _obj, real_t _scale):
	CentroidCon(solver, 3, ConTag::CentroidVelR, _name, _obj, _scale) {

	Centroid* cen = (Centroid*)obj[0]->node;

	AddSLink (obj[1]->var_vel_r);
	AddSLink (obj[0]->var_vel_r);
	AddX3Link(obj[0]->var_pos_t);
	for(int i = 0; i < cen->contacts.size(); i++){
		AddX3Link(cen->contacts[i].var_pos);
	}
	for(int i = 0; i < obj[0]->contacts.size(); i++){
		AddC3Link(obj[0]->contacts[i].var_force[0]);
		AddC3Link(obj[0]->contacts[i].var_force[1]);
		AddC3Link(obj[0]->contacts[i].var_force[2]);
	}
}

CentroidEndPosCon::CentroidEndPosCon(Solver* solver, int _tag, string _name, CentroidKey* _obj, int _iend, real_t _scale):
	CentroidCon(solver, 3, ConTag::CentroidEndPos, _name, _obj, _scale){
	iend = _iend;
	
	AddSLink(obj[1]->ends[iend].var_pos);
	AddSLink(obj[0]->ends[iend].var_pos);
	AddSLink(obj[0]->ends[iend].var_vel);
}

CentroidEndRangeCon::CentroidEndRangeCon(Solver* solver, int _tag, string _name, CentroidKey* _obj, int _iend, vec3_t _dir, real_t _scale):
	Constraint(solver, 1, ID(_tag, _obj->node, _obj->tick, _name), _scale){
	obj  = _obj;
	iend = _iend;
	dir  = _dir;

	AddR3Link(obj->var_pos_t);
	AddR3Link(obj->var_pos_r);
	AddR3Link(obj->ends[iend].var_pos);
}

CentroidEndCmplCon::CentroidEndCmplCon(Solver* solver, int _tag, string _name, CentroidKey* _obj, int _iend, real_t _scale):
	Constraint(solver, 1, ID(_tag, _obj->node, _obj->tick, _name), _scale){
	obj  = _obj;
	iend = _iend;

	Centroid* cen = (Centroid*)obj->node;
	
	for(int j = 0; j < obj->contacts.size(); j++){
		if(cen->param.contacts[j].iend == iend){
			AddSLink(obj->contacts[j].var_active[0]);
		}
	}
}

CentroidContactGapCon::CentroidContactGapCon(Solver* solver, int _tag, string _name, CentroidKey* _obj, int _icon, real_t _scale):
	Constraint(solver, 1, ID(_tag, _obj->node, _obj->tick, _name), _scale){

	obj  = _obj;
	icon = _icon;
	
	Centroid* cen = (Centroid*)_obj->node;
	iend = cen->param.contacts[icon].iend;
	
	AddR3Link(obj->ends    [iend].var_pos);
	AddR3Link(cen->contacts[icon].var_pos);
	AddSLink (obj->contacts[icon].var_active[1]);
}

CentroidContactForceCon::CentroidContactForceCon(Solver* solver, int _tag, string _name, CentroidKey* _obj, int _icon, int _dir, real_t _scale):
	Constraint(solver, 1, ID(_tag, _obj->node, _obj->tick, _name), _scale){
	
	obj  = _obj;
	icon = _icon;
	dir  = _dir;

	if(dir == 0){
		AddSLink(obj->contacts[icon].var_force[0]);
		AddSLink(obj->contacts[icon].var_force[2]);
	}
	if(dir == 1){
		AddSLink(obj->contacts[icon].var_force[1]);
		AddSLink(obj->contacts[icon].var_force[2]);
	}
	if(dir == 2){
		AddSLink(obj->contacts[icon].var_force [2]);
		AddSLink(obj->contacts[icon].var_active[0]);
	}
}

CentroidContactCmplCon::CentroidContactCmplCon(Solver* solver, int _tag, string _name, CentroidKey* _obj, int _icon, real_t _scale):
	Constraint(solver, 1, ID(_tag, _obj->node, _obj->tick, _name), _scale){
	
	obj  = _obj;
	icon = _icon;

	AddSLink(obj->contacts[icon].var_active[0]);
	AddSLink(obj->contacts[icon].var_active[1]);
}

///////////////////////////////////////////////////////////////////////////////////////////////////

void CentroidCon::Prepare(){
	Centroid* cen = (Centroid*)obj[0]->node;

	hnorm    = cen->hnorm;
}

void CentroidEndRangeCon::Prepare(){
	p       = obj->var_pos_t->val;
	q       = obj->var_pos_r->val;
	pend    = obj->ends[iend].var_pos->val;
	dp      = pend - p;
	dir_abs = q*dir;
}

void CentroidContactGapCon::Prepare(){
	Centroid* cen = (Centroid*)obj->node;
	
	dmax   = cen->param.dmax;
	pe     = obj->ends[iend].var_pos->val;
	pc     = cen->contacts[icon].var_pos->val;
	ac     = obj->contacts[icon].var_active[1]->val;
	dp     = pe - pc;
	dpnorm = dp.norm();
	if(dpnorm > eps)
		 dpn = dp/dpnorm;
	else dpn = vec3_t(1.0, 0.0, 0.0);

	/*
	//dpnorm = dp.norm();
	dpn = vec3_t(0.0, 0.0, 1.0);
	dpnorm = dpn*dp;
	
	if(dpnorm > eps)
		 dpn = dp/dpnorm;
	else dpn.clear();

	if(face.iedge == -1 && face.ivtx == -1){
		vec3_t nf = face.R.col(2);
		
		dpn = (nf*dpn)*nf;
	}
	if(face.iedge != -1 && face.ivtx == -1){
		vec3_t nf = face.R.col(2);
		vec3_t ne = cen->param.faces[iface].edges[face.iedge].n;

		dpn = (nf*dpn)*nf + (ne*dpn)*ne;
	}
	if(face.iedge != -1 && face.ivtx != -1){
	
	}
	*/
}

void CentroidContactForceCon::Prepare(){
	Centroid* cen = (Centroid*)obj->node;

	mu   = cen->param.mu;
	fmax = cen->param.fmax;
	f    = obj->contacts[icon].f;
	a    = obj->contacts[icon].var_active[0]->val;

	//ft = vec2_t(
	//	obj->ends[iend].faces[iface].var_fric[0]->val,
	//	obj->ends[iend].faces[iface].var_fric[1]->val);
	//ftnorm = ft.norm();
	//if(ftnorm < eps)
	//	 ftn.clear();
	//else ftn = ft/ftnorm;
}

void CentroidContactCmplCon::Prepare(){
	a  = obj->contacts[icon].var_active[0]->val;
	ac = obj->contacts[icon].var_active[1]->val;
}
/*
void CentroidEndVelCon::Prepare(){
	CentroidCmplCon::Prepare();

	Centroid* cen = (Centroid*)obj->node;

	eta   = cen->param.faces[iface].eta;
	v     = dir*obj->ends[iend].var_vel->val;
	//vnorm = v.norm();
	//if(vnorm < eps)
	//	 vn.clear();
	//else vn = v/vnorm;
}
*/
///////////////////////////////////////////////////////////////////////////////////////////////////

void CentroidPosConT::CalcCoef(){
	Prepare();

	((SLink*)links[0])->SetCoef( 1.0);
	((SLink*)links[1])->SetCoef(-1.0);
	((SLink*)links[2])->SetCoef(-hnorm);
}

void CentroidPosConR::CalcCoef(){
	Prepare();

	((SLink*)links[0])->SetCoef( 1.0);
	((SLink*)links[1])->SetCoef(-1.0);
	((SLink*)links[2])->SetCoef(-hnorm);
}

void CentroidVelConT::CalcCoef(){
	Prepare();

	Centroid* cen = (Centroid*)obj[0]->node;

	int i = 0;
	((SLink*)links[i++])->SetCoef( 1.0);
	((SLink*)links[i++])->SetCoef(-1.0);
	for(CentroidKey::Contact& con : obj[0]->contacts){
		((C3Link*)links[i++])->SetCoef(-hnorm * vec3_t(1.0, 0.0, 0.0));
		((C3Link*)links[i++])->SetCoef(-hnorm * vec3_t(0.0, 1.0, 0.0));
		((C3Link*)links[i++])->SetCoef(-hnorm * vec3_t(0.0, 0.0, 1.0));
	}
}

void CentroidVelConR::CalcCoef(){
	Prepare();

	Centroid* cen = (Centroid*)obj[0]->node;

	int i = 0;
	((SLink*)links[i++])->SetCoef( 1.0);
	((SLink*)links[i++])->SetCoef(-1.0);

	//real_t Iinv = 1.0/((Centroid*)obj[0]->node)->Inorm;
	vec3_t fsum;
	for(CentroidKey::Contact& con : obj[0]->contacts){
		fsum += con.f;
	}
	((X3Link*)links[i++])->SetCoef(-hnorm*fsum);

	for(int j = 0; j < obj[0]->contacts.size(); j++){
		((X3Link*)links[i++])->SetCoef(hnorm*obj[0]->contacts[j].f);
	}
	for(int j = 0; j < obj[0]->contacts.size(); j++){
		vec3_t r = cen->contacts[j].var_pos->val - obj[0]->var_pos_t->val;

		((C3Link*)links[i++])->SetCoef(-hnorm*(r % vec3_t(1.0, 0.0, 0.0)));
		((C3Link*)links[i++])->SetCoef(-hnorm*(r % vec3_t(0.0, 1.0, 0.0)));
		((C3Link*)links[i++])->SetCoef(-hnorm*(r % vec3_t(0.0, 0.0, 1.0)));
	}
}

void CentroidEndPosCon::CalcCoef(){
	Prepare();

	((SLink*)links[0])->SetCoef( 1.0);
	((SLink*)links[1])->SetCoef(-1.0);
	((SLink*)links[2])->SetCoef(-hnorm);
}

void CentroidEndRangeCon::CalcCoef(){
	Prepare();

	((R3Link*)links[0])->SetCoef(-dir_abs);
	((R3Link*)links[1])->SetCoef( dir_abs % dp );
	((R3Link*)links[2])->SetCoef( dir_abs);
}

void CentroidEndCmplCon::CalcCoef(){
	Centroid* cen = (Centroid*)obj->node;
	
	int i = 0;
	for(int j = 0; j < obj->contacts.size(); j++){
		if(cen->param.contacts[j].iend == iend){
			((SLink*)links[i++])->SetCoef(1.0);
		}
	}
}

void CentroidContactGapCon::CalcCoef(){
	Prepare();

	((R3Link*)links[0])->SetCoef( dpn );
	((R3Link*)links[1])->SetCoef(-dpn );
	((SLink* )links[2])->SetCoef(-dmax);
}

void CentroidContactForceCon::CalcCoef(){
	Prepare();

	if(dir == 0 || dir == 1){
		((SLink*)links[0])->SetCoef(1.0);
		((SLink*)links[1])->SetCoef((on_upper ? -1.0 : 1.0)*mu);
	}
	if(dir == 2){
		((SLink*)links[0])->SetCoef(1.0);
		((SLink*)links[1])->SetCoef((on_upper ? -fmax : 0.0));
	}
}

void CentroidContactCmplCon::CalcCoef(){
	Prepare();

	((SLink*)links[0])->SetCoef(1.0);
	((SLink*)links[1])->SetCoef(1.0);
}

/*
void CentroidEndVelCon::CalcCoef(){
	Prepare();

	((R3Link*)links[0])->SetCoef(-(v >= 0.0 ? 1.0 : -1.0)*dir);	
	((SLink *)links[1])->SetCoef( eta*sqrt_rel*exp(s));
}
*/
///////////////////////////////////////////////////////////////////////////////////////////////////

void CentroidPosConT::CalcDeviation(){
	y = obj[1]->var_pos_t->val - (obj[0]->var_pos_t->val + hnorm*obj[0]->var_vel_t->val);
}

void CentroidPosConR::CalcDeviation(){
	quat_t q0     = obj[0]->var_pos_r->val;
	quat_t q1     = obj[1]->var_pos_r->val;
	quat_t q      = q0*quat_t::Rot(q0.Conjugated()*(hnorm*obj[0]->var_vel_r->val));
	quat_t qerror = q.Conjugated()*q1;
	vec3_t axis   = qerror.Axis ();
	real_t theta  = qerror.Theta();
	if(theta > pi)
		theta -= 2*pi;
	y = q*(theta*axis);
}

void CentroidVelConT::CalcDeviation(){
	y = obj[1]->var_vel_t->val - obj[0]->var_vel_t->val;
	
	for(CentroidKey::Contact& con : obj[0]->contacts){
		y -= hnorm*con.f;
	}

	//y -= ((Centroid*)obj[0]->node)->gnorm;
	// normalized gravity
	y -= hnorm*vec3_t(0.0, 0.0, -1.0);
}

void CentroidVelConR::CalcDeviation(){
	Centroid* cen = (Centroid*)obj[0]->node;

	y = obj[1]->var_vel_r->val - obj[0]->var_vel_r->val;

	//real_t Iinv = 1.0/((Centroid*)obj[0]->node)->Inorm;
	for(int j = 0; j < obj[0]->contacts.size(); j++){
		y -= hnorm*( (cen->contacts[j].var_pos->val - obj[0]->var_pos_t->val) % obj[0]->contacts[j].f );
	}
}

void CentroidEndPosCon::CalcDeviation(){
	y = obj[1]->ends[iend].var_pos->val - (obj[0]->ends[iend].var_pos->val + hnorm*obj[0]->ends[iend].var_vel->val);
}

void CentroidEndRangeCon::CalcDeviation(){
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

void CentroidEndCmplCon::CalcDeviation(){
	y[0] = 0.0;

	Centroid* cen = (Centroid*)obj->node;
	
	for(int j = 0; j < obj->contacts.size(); j++){
		if(cen->param.contacts[j].iend == iend){
			y[0] += obj->contacts[j].var_active[0]->val;
		}
	}

	y[0] -= 1.0;
}

void CentroidContactGapCon::CalcDeviation(){
	y[0] = 0.0;
	active = false;

	if(dpnorm > dmax*ac){
		y[0]   = dpnorm - dmax*ac;
		active = true;
	}
}

void CentroidContactForceCon::CalcDeviation(){
	y[0] = 0.0;
	on_upper = on_lower = active = false;

	if(dir == 0 || dir == 1){
		if(f[dir] >  mu*f[2]){
			y[0]     = f[dir] - mu*f[2];
			on_upper = true;
			active   = true;
		}
		if(f[dir] < -mu*f[2]){
			y[0]     = f[dir] + mu*f[2];
			on_lower = true;
			active   = true;
		}
	}
	if(dir == 2){
		if(f[2] > fmax*a){
			y[0]     = f[2] - fmax*a;
			on_upper = true;
			active   = true;
		}
		if(f[2] < 0.0){
			y[0]     = f[2];
			on_lower = true;
			active   = true;
		}
	}
}

void CentroidContactCmplCon::CalcDeviation(){
	y[0] = a + ac - 1.0;
}

/*
void CentroidEndVelCon::CalcDeviation(){
	y[0] = eta*(sqrt_rel*exp(s) - rel) - std::abs(v);
	if(y[0] < 0.0){
		active = true;
	}
	else{
		y[0] = 0.0;
		active = false;
	}
}
*/

///////////////////////////////////////////////////////////////////////////////////////////////////

void CentroidPosConT::CalcLhs(){
	obj[1]->var_pos_t->val = obj[0]->var_pos_t->val + hnorm*obj[0]->var_vel_t->val;
}

void CentroidPosConR::CalcLhs(){
	obj[1]->var_pos_r->val = obj[0]->var_pos_r->val * quat_t(obj[0]->var_pos_r->val.Conjugated()*(hnorm*obj[0]->var_vel_r->val));
}

void CentroidVelConT::CalcLhs(){
	obj[1]->var_vel_t->val = obj[0]->var_vel_t->val;
	for(CentroidKey::Contact& con : obj[0]->contacts){
		obj[1]->var_vel_t->val += hnorm*con.f;
	}
	//obj[1]->var_vel_t->val += ((Centroid*)obj[0]->node)->gnorm;
	obj[1]->var_vel_t->val += hnorm*vec3_t(0.0, 0.0, -1.0);
}

void CentroidVelConR::CalcLhs(){
	Centroid* cen = (Centroid*)obj[0]->node;
	
	obj[1]->var_vel_r->val = obj[0]->var_vel_r->val;
	//real_t Iinv = 1.0/((Centroid*)obj[0]->node)->Inorm;
	for(int j = 0; j < obj[0]->contacts.size(); j++){
		obj[1]->var_vel_r->val += hnorm*( (cen->contacts[j].var_pos->val - obj[0]->var_pos_t->val) % obj[0]->contacts[j].f );
	}
}

void CentroidEndPosCon::CalcLhs(){
	obj[1]->ends[iend].var_pos->val = obj[0]->ends[iend].var_pos->val + hnorm*obj[0]->ends[iend].var_vel->val;
}

///////////////////////////////////////////////////////////////////////////////////////////////////

void CentroidEndRangeCon::Project(real_t& l, uint k) {
	if ( on_upper && l > 0.0  ) l = 0.0;
	if ( on_lower && l < 0.0  ) l = 0.0;
	if (!on_upper && !on_lower) l = 0.0;
}

void CentroidContactGapCon::Project(real_t& l, uint k) {
	if (l > 0.0) l = 0.0;
}

void CentroidContactForceCon::Project(real_t& l, uint k) {
	if ( on_upper && l > 0.0  ) l = 0.0;
	if ( on_lower && l < 0.0  ) l = 0.0;
	if (!on_upper && !on_lower) l = 0.0;
}

/*
void CentroidEndVelCon::Project(real_t& l, uint k){
	if(l < 0.0) l = 0.0;
}

void CentroidFrictionCon::Project(real_t& l, uint k){
	if(l < 0.0) l = 0.0;
}
*/
}
