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

vec3_t CentroidKey::End::GetPos(){
	return vec3_t(
		var_pos[0]->val,
		var_pos[1]->val,
		var_pos[2]->val
	);
}

vec3_t CentroidKey::End::GetVel(){
	return vec3_t(
		var_vel[0]->val,
		var_vel[1]->val,
		var_vel[2]->val
	);
}

vec3_t CentroidKey::End::GetForce(){
	return vec3_t(
		var_force[0]->val,
		var_force[1]->val,
		var_force[2]->val
	);
}

void CentroidKey::End::SetPos(const vec3_t& p){
	var_pos[0]->val = p[0];
	var_pos[1]->val = p[1];
	var_pos[2]->val = p[2];
}

void CentroidKey::End::SetVel(const vec3_t& v){
	var_vel[0]->val = v[0];
	var_vel[1]->val = v[1];
	var_vel[2]->val = v[2];
}

void CentroidKey::End::SetForce(const vec3_t& f){
	var_force[0]->val = f[0];
	var_force[1]->val = f[1];
	var_force[2]->val = f[2];
}

//-------------------------------------------------------------------------------------------------
// CentroidKey

CentroidKey::CentroidKey() {
	
}

void CentroidKey::AddVar(Solver* solver) {
	cen = (Centroid*)node;
	
	// position and velocity
	var_pos_t = new V3Var(solver, ID(VarTag::CentroidTP, node, tick, name + "_tp"), node->graph->scale.pos_t);
	var_pos_r = new QVar (solver, ID(VarTag::CentroidRP, node, tick, name + "_rp"), node->graph->scale.pos_r);
	var_vel_t = new V3Var(solver, ID(VarTag::CentroidTV, node, tick, name + "_tv"), node->graph->scale.vel_t);
	var_vel_r = new V3Var(solver, ID(VarTag::CentroidRV, node, tick, name + "_rv"), node->graph->scale.vel_r);
	var_pos_t->weight = damping*one;
	var_pos_r->weight = damping*one;
	var_vel_t->weight = damping*one;
	var_vel_r->weight = damping*one;
	
	ends.resize(cen->param.ends.size());
	stringstream ss;
	for(int i = 0; i < ends.size(); i++){
		ends[i].key = this;

		for(int j = 0; j < 3; j++){
			ss.str("");
			ss << name << "_end" << i << "_pos" << j;
			ends[i].var_pos  [j] = new SVar(solver, ID(VarTag::CentroidEndPos  , node, tick, ss.str()), node->graph->scale.pos_t);
			
			ss.str("");
			ss << name << "_end" << i << "_vel" << j;
			ends[i].var_vel  [j] = new SVar(solver, ID(VarTag::CentroidEndVel  , node, tick, ss.str()), node->graph->scale.vel_t);			
			
			ss.str("");
			ss << name << "_end" << i << "_force" << j;
			ends[i].var_force[j] = new SVar(solver, ID(VarTag::CentroidEndForce, node, tick, ss.str()), node->graph->scale.force_t);

			ends[i].var_pos  [j]->weight[0] = damping;
			ends[i].var_vel  [j]->weight[0] = damping;
			ends[i].var_force[j]->weight[0] = damping;
		}
	}
}

void CentroidKey::AddCon(Solver* solver) {
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

		if(next){
			ends[i].con_pos[0]  = new CentroidEndPosCon  (solver, ConTag::CentroidEndPos  , ss.str() + "_pos0" , this, i, 0, node->graph->scale.pos_t  );
			ends[i].con_pos[1]  = new CentroidEndPosCon  (solver, ConTag::CentroidEndPos  , ss.str() + "_pos1" , this, i, 1, node->graph->scale.pos_t  );
			ends[i].con_pos[2]  = new CentroidEndPosCon  (solver, ConTag::CentroidEndPos  , ss.str() + "_pos2" , this, i, 2, node->graph->scale.pos_t  );
			ends[i].con_vel     = new CentroidEndVelCon  (solver, ConTag::CentroidEndVel  , ss.str() + "_vel"  , this, i,    node->graph->scale.force_t);
			ends[i].con_force   = new CentroidEndForceCon(solver, ConTag::CentroidEndForce, ss.str() + "_force", this, i,    node->graph->scale.force_t);
		}
		ends[i].con_pos_z   = new RangeConS(solver, ID(ConTag::CentroidEndPos  , node, tick, ss.str() + "_pos_z"  ), ends[i].var_pos  [2], 1.0);
		ends[i].con_force_z = new RangeConS(solver, ID(ConTag::CentroidEndForce, node, tick, ss.str() + "_force_z"), ends[i].var_force[2], 1.0);
	}
}		

void CentroidKey::Prepare() {
	
}

void CentroidKey::Draw(Render::Canvas* canvas, Render::Config* conf) {
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
	k    = 0;
	time = 0.0;
}

Centroid::Waypoint::Waypoint(
	int _k, real_t _time, vec3_t _pos_t, quat_t _pos_r, vec3_t _vel_t, vec3_t _vel_r, 
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
	L     = sqrt(param.I/param.m);
	T     = sqrt(L/param.g);
	V     = L/T;
	F     = param.m*param.g;
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
			key->ends[i].con_pos_z  ->_min = 0.0;
			key->ends[i].con_pos_z  ->_max = inf;
			key->ends[i].con_force_z->_min = 0.0;
			key->ends[i].con_force_z->_max = inf;
		}
	}
	/*
	for (uint k = 0; k < graph->ticks.size(); k++) {
		CentroidKey* key = (CentroidKey*)traj.GetKeypoint(graph->ticks[k]);
		real_t t = graph->ticks[k]->time;

		key->var_pos_t->val = vec3_t(0.0, 0.0, 1.0)/L;
		key->var_pos_r->val = quat_t();
		key->var_vel_t->val = vec3_t(0.0, 0.0, 0.0)/V;
		key->var_vel_r->val = vec3_t(0.0, 0.0, 0.0);

		for(int i = 0; i < key->ends.size(); i++){
			key->ends[i].SetPos(vec3_t(0.0, 0.0, 0.0)/L);
			key->ends[i].SetVel(vec3_t(0.0, 0.0, 0.0)/V);
		}

		if(k == 0){
			key->var_pos_t->locked = true;
			key->var_pos_r->locked = true;
			key->var_vel_t->locked = true;
			key->var_vel_r->locked = true;

			for(int i = 0; i < key->ends.size(); i++){
				for(int j = 0; j < 3; j++){
					key->ends[i].var_pos[j]->locked = true;
					key->ends[i].var_vel[j]->locked = true;
				}
			}
		}
	}
	*/
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
			key->ends[i].SetPos(curve_end[i].CalcPos(t)/L);
			key->ends[i].SetVel(curve_end[i].CalcVel(t)/V);
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
			key->ends[j].var_pos[0]->locked = wp.ends[j].fix_pos;
			key->ends[j].var_pos[1]->locked = wp.ends[j].fix_pos;
			key->ends[j].var_pos[2]->locked = wp.ends[j].fix_pos;
			key->ends[j].var_vel[0]->locked = wp.ends[j].fix_vel;
			key->ends[j].var_vel[1]->locked = wp.ends[j].fix_vel;
			key->ends[j].var_vel[2]->locked = wp.ends[j].fix_vel;
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
		k0->tick->time, k0->ends[index].GetPos()*L, k0->ends[index].GetVel()*V,
		k1->tick->time, k1->ends[index].GetPos()*L, k1->ends[index].GetVel()*V,
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
		k0->tick->time, k0->ends[index].GetPos()*L, k0->ends[index].GetVel()*V,
		k1->tick->time, k1->ends[index].GetPos()*L, k1->ends[index].GetVel()*V,
		type);
}

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

	s.ends.resize(param.ends.size());
	for(int i = 0; i < param.ends.size(); i++){
		s.ends[i].pos   = EndPos  (t, i);
		s.ends[i].force = EndForce(t, i);
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
		for(int i = 0; i < param.ends.size(); i++){
			canvas->BeginPath();
			canvas->MoveTo(snapshot.ends[i].pos);
			canvas->LineTo(snapshot.ends[i].pos + snapshot.ends[i].force);
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
	for(CentroidKey::End& end : obj[0]->ends){
		AddC3Link(end.var_force[0]);
		AddC3Link(end.var_force[1]);
		AddC3Link(end.var_force[2]);
	}
}

CentroidVelConR::CentroidVelConR(Solver* solver, int _tag, string _name, CentroidKey* _obj, real_t _scale):
	CentroidCon(solver, 3, ConTag::CentroidVelR, _name, _obj, _scale) {

	AddSLink (obj[1]->var_vel_r);
	AddSLink (obj[0]->var_vel_r);
	AddX3Link(obj[0]->var_pos_t);
	
	for(CentroidKey::End& end : obj[0]->ends){
		AddC3Link(end.var_pos  [0]);
		AddC3Link(end.var_pos  [1]);
		AddC3Link(end.var_pos  [2]);
		AddC3Link(end.var_force[0]);
		AddC3Link(end.var_force[1]);
		AddC3Link(end.var_force[2]);
	}
}

CentroidEndPosCon::CentroidEndPosCon(Solver* solver, int _tag, string _name, CentroidKey* _obj, int _iend, int _dir, real_t _scale):
	CentroidCon(solver, 1, ConTag::CentroidEndPos, _name, _obj, _scale){
	iend = _iend;
	dir  = _dir ;
	
	AddSLink(obj[1]->ends[iend].var_pos[dir]);
	AddSLink(obj[0]->ends[iend].var_pos[dir]);
	AddSLink(obj[0]->ends[iend].var_vel[dir]);
}

CentroidEndRangeCon::CentroidEndRangeCon(Solver* solver, int _tag, string _name, CentroidKey* _obj, int _iend, vec3_t _dir, real_t _scale):
	Constraint(solver, 1, ID(_tag, _obj->node, _obj->tick, _name), _scale){
	obj  = _obj;
	iend = _iend;
	dir  = _dir;

	AddR3Link(obj->var_pos_t);
	AddR3Link(obj->var_pos_r);
	AddSLink (obj->ends[iend].var_pos[0]);
	AddSLink (obj->ends[iend].var_pos[1]);
	AddSLink (obj->ends[iend].var_pos[2]);
}

CentroidEndVelCon::CentroidEndVelCon(Solver* solver, int _tag, string _name, CentroidKey* _obj, int _iend, real_t _scale):
	Constraint(solver, 1, ID(_tag, _obj->node, _obj->tick, _name), _scale){
	obj  = _obj;
	iend = _iend;

	AddSLink(obj->ends[iend].var_vel[0]);
	AddSLink(obj->ends[iend].var_vel[1]);
	AddSLink(obj->ends[iend].var_pos[2]);
}

CentroidEndForceCon::CentroidEndForceCon(Solver* solver, int _tag, string _name, CentroidKey* _obj, int _iend, real_t _scale):
	Constraint(solver, 1, ID(_tag, _obj->node, _obj->tick, _name), _scale){
	obj  = _obj;
	iend = _iend;
	
	AddSLink(obj->ends[iend].var_force[0]);
	AddSLink(obj->ends[iend].var_force[1]);
	AddSLink(obj->ends[iend].var_force[2]);
}

///////////////////////////////////////////////////////////////////////////////////////////////////

void CentroidCon::Prepare(){
	hnorm = obj[0]->cen->hnorm;
}

void CentroidEndRangeCon::Prepare(){
	p       = obj->var_pos_t->val;
	q       = obj->var_pos_r->val;
	pend    = obj->ends[iend].GetPos();
	dp      = pend - p;
	dir_abs = q*dir;
}

void CentroidEndVelCon::Prepare(){
	eta  = obj->cen->param.eta;
	vt.x = obj->ends[iend].var_vel[0]->val;
	vt.y = obj->ends[iend].var_vel[1]->val;
	pz   = obj->ends[iend].var_pos[2]->val;
	vtnorm = vt.norm();
	if(vtnorm > eps)
		 vtn = vt/vtnorm;
	else vtn = vec2_t(1.0, 0.0);
}

void CentroidEndForceCon::Prepare(){
	mu   = obj->cen->param.mu;
	ft.x = obj->ends[iend].var_force[0]->val;
	ft.y = obj->ends[iend].var_force[1]->val;
	fz   = obj->ends[iend].var_force[2]->val;
	ftnorm = ft.norm();
	if(ftnorm > eps)
		 ftn = ft/ftnorm;
	else ftn = vec2_t(1.0, 0.0);
}

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

	int i = 0;
	((SLink*)links[i++])->SetCoef( 1.0);
	((SLink*)links[i++])->SetCoef(-1.0);
	for(CentroidKey::End& end : obj[0]->ends){
		((C3Link*)links[i++])->SetCoef(-hnorm * vec3_t(1.0, 0.0, 0.0));
		((C3Link*)links[i++])->SetCoef(-hnorm * vec3_t(0.0, 1.0, 0.0));
		((C3Link*)links[i++])->SetCoef(-hnorm * vec3_t(0.0, 0.0, 1.0));
	}
}

void CentroidVelConR::CalcCoef(){
	Prepare();

	int i = 0;
	((SLink*)links[i++])->SetCoef( 1.0);
	((SLink*)links[i++])->SetCoef(-1.0);

	vec3_t fsum;
	for(CentroidKey::End& end : obj[0]->ends){
		fsum += end.GetForce();
	}
	((X3Link*)links[i++])->SetCoef(-hnorm*fsum);

	for(int j = 0; j < obj[0]->ends.size(); j++){
		vec3_t r = obj[0]->ends[j].GetPos() - obj[0]->var_pos_t->val;
		vec3_t f = obj[0]->ends[j].GetForce();

		((C3Link*)links[i++])->SetCoef( hnorm*(f % vec3_t(1.0, 0.0, 0.0)));
		((C3Link*)links[i++])->SetCoef( hnorm*(f % vec3_t(0.0, 1.0, 0.0)));
		((C3Link*)links[i++])->SetCoef( hnorm*(f % vec3_t(0.0, 0.0, 1.0)));
		
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

	((R3Link*)links[0])->SetCoef(-dir_abs      );
	((R3Link*)links[1])->SetCoef( dir_abs % dp );
	((SLink* )links[2])->SetCoef( dir_abs[0]   );
	((SLink* )links[3])->SetCoef( dir_abs[1]   );
	((SLink* )links[4])->SetCoef( dir_abs[2]   );
}

void CentroidEndVelCon ::CalcCoef(){
	Prepare();

	((SLink*)links[0])->SetCoef(-vtn.x);
	((SLink*)links[1])->SetCoef(-vtn.y);
	((SLink*)links[2])->SetCoef( eta  );
}

void CentroidEndForceCon::CalcCoef(){
	Prepare();

	((SLink*)links[0])->SetCoef(-ftn.x);
	((SLink*)links[1])->SetCoef(-ftn.y);
	((SLink*)links[2])->SetCoef( mu   );
}

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
	
	for(CentroidKey::End& end : obj[0]->ends){
		y -= hnorm*end.GetForce();
	}

	y -= hnorm*vec3_t(0.0, 0.0, -1.0);
}

void CentroidVelConR::CalcDeviation(){
	y = obj[1]->var_vel_r->val - obj[0]->var_vel_r->val;

	for(int j = 0; j < obj[0]->ends.size(); j++){
		y -= hnorm*((obj[0]->ends[j].GetPos() - obj[0]->var_pos_t->val) % obj[0]->ends[j].GetForce());
	}
}

void CentroidEndPosCon::CalcDeviation(){
	y[0] = obj[1]->ends[iend].var_pos[dir]->val - (obj[0]->ends[iend].var_pos[dir]->val + hnorm*obj[0]->ends[iend].var_vel[dir]->val);
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

void CentroidEndVelCon::CalcDeviation(){
	if(vtnorm > eta*pz){
		y[0]   = eta*pz - vtnorm;
		active = true;
	}
	else{
		y[0]   = 0.0;
		active = false;
	}
}

void CentroidEndForceCon::CalcDeviation(){
	if(ftnorm > mu*fz){
		y[0]   = mu*fz - ftnorm;
		active = true;
	}
	else{
		y[0]   = 0.0;
		active = false;
	}
}

///////////////////////////////////////////////////////////////////////////////////////////////////

void CentroidPosConT::CalcLhs(){
	obj[1]->var_pos_t->val = obj[0]->var_pos_t->val + hnorm*obj[0]->var_vel_t->val;
}

void CentroidPosConR::CalcLhs(){
	obj[1]->var_pos_r->val = obj[0]->var_pos_r->val * quat_t(obj[0]->var_pos_r->val.Conjugated()*(hnorm*obj[0]->var_vel_r->val));
}

void CentroidVelConT::CalcLhs(){
	obj[1]->var_vel_t->val = obj[0]->var_vel_t->val;
	for(CentroidKey::End& end : obj[0]->ends){
		obj[1]->var_vel_t->val += hnorm*end.GetForce();
	}
	//obj[1]->var_vel_t->val += ((Centroid*)obj[0]->node)->gnorm;
	obj[1]->var_vel_t->val += hnorm*vec3_t(0.0, 0.0, -1.0);
}

void CentroidVelConR::CalcLhs(){
	obj[1]->var_vel_r->val = obj[0]->var_vel_r->val;
	//real_t Iinv = 1.0/((Centroid*)obj[0]->node)->Inorm;
	for(int j = 0; j < obj[0]->ends.size(); j++){
		obj[1]->var_vel_r->val += hnorm*((obj[0]->ends[j].GetPos() - obj[0]->var_pos_t->val) % obj[0]->ends[j].GetForce());
	}
}

void CentroidEndPosCon::CalcLhs(){
	obj[1]->ends[iend].var_pos[dir]->val = obj[0]->ends[iend].var_pos[dir]->val + hnorm*obj[0]->ends[iend].var_vel[dir]->val;
}

///////////////////////////////////////////////////////////////////////////////////////////////////

void CentroidEndRangeCon::Project(real_t& l, uint k) {
	if ( on_upper && l > 0.0  ) l = 0.0;
	if ( on_lower && l < 0.0  ) l = 0.0;
	if (!on_upper && !on_lower) l = 0.0;
}

void CentroidEndForceCon::Project(real_t& l, uint k) {
	if(l < 0.0) l = 0.0;
}

void CentroidEndVelCon::Project(real_t& l, uint k){
	if(l < 0.0) l = 0.0;
}

}
