#include <DiMP/Graph/Centroid.h>
#include <DiMP/Graph/Graph.h>
#include <DiMP/Render/Config.h>
#include <DiMP/Render/Canvas.h>

namespace DiMP {;

const real_t pi  = M_PI;
const real_t inf = numeric_limits<real_t>::max();

//-------------------------------------------------------------------------------------------------
// CentroidKey

vec3_t CentroidKey::Face::f(){
	return vec3_t(
		var_force[0]->val,
		var_force[1]->val,
		var_force[2]->val);
}

CentroidKey::CentroidKey() {
	
}

void CentroidKey::AddVar(Solver* solver) {
	Centroid* obj = (Centroid*)node;

	int nend  = (int)obj->param.ends .size();
	int nface = (int)obj->param.faces.size();
	ends.resize(nend);
	for(int i = 0; i < nend; i++){
		ends[i].faces.resize(nface);
	}

	// position and velocity
	var_pos_t = new V3Var(solver, ID(VarTag::CentroidTP, node, tick, name + "_tp"), node->graph->scale.pos_t);
	var_pos_r = new QVar (solver, ID(VarTag::CentroidRP, node, tick, name + "_rp"), node->graph->scale.pos_r);
	var_vel_t = new V3Var(solver, ID(VarTag::CentroidTV, node, tick, name + "_tv"), node->graph->scale.vel_t);
	var_vel_r = new V3Var(solver, ID(VarTag::CentroidRV, node, tick, name + "_rv"), node->graph->scale.vel_r);

	stringstream ss;
	for(int i = 0; i < nend; i++){
		ss.str("");
		ss << name << "_end" << i << "_pos";
		ends[i].var_pos = new V3Var(solver, ID(VarTag::CentroidEndPos, node, tick, ss.str()), node->graph->scale.pos_t);

		ss.str("");
		ss << name << "_end" << i << "_vel";
		ends[i].var_vel = new SVar(solver, ID(VarTag::CentroidEndVel, node, tick, ss.str()), node->graph->scale.vel_t);
	
		for(int j = 0; j < nface; j++){
			ss.str("");
			ss << name << "_end" << i << "_face" << j << "gap";
			ends[i].faces[j].var_gap = new SVar(solver, ID(VarTag::CentroidFaceGap, node, tick, ss.str()), node->graph->scale.pos_t);

			for(int k = 0; k < 3; k++){
				ss.str("");
				ss << name << "_end" << i << "_face" << j << "_force" << k;
				ends[i].faces[j].var_force[k] = new SVar(solver, ID(VarTag::CentroidFaceForce, node, tick, ss.str()), node->graph->scale.force_t);
			}
		}
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

	int nend  = (int)obj->param.ends .size();
	int nface = (int)obj->param.faces.size();

	stringstream ss;
	for(int i = 0; i < nend; i++){
		for(int j = 0; j < 3; j++){
			ss.str("");
			ss << name << "_end" << i << "_range" << j;
			ends[i].con_range[j] = new CentroidRangeCon(solver, ConTag::CentroidRange, ss.str(), this, i, j, node->graph->scale.pos_t);
		}

		if(next){
			ss.str("");
			ss << name << "_end" << i << "_vel";
			ends[i].con_vel = new CentroidVelCon(solver, ConTag::CentroidVel, ss.str(), this, i, node->graph->scale.vel_t);

			for(int j = 0; j < nface; j++){
				ss.str("");
				ss << name << "_end" << i << "_face" << j << "_gap";
				ends[i].faces[j].con_gap = new CentroidGapCon(solver, ConTag::CentroidGap, ss.str(), this, i, j, node->graph->scale.force_t);
			
				ss.str("");
				ss << name << "_end" << i << "_face" << j << "_fric";
				ends[i].faces[j].con_fric = new CentroidFrictionCon(solver, ConTag::CentroidFriction, ss.str(), this, i, j, node->graph->scale.force_t);
			}
		}
	}
}		

void CentroidKey::Prepare() {
	Centroid* obj = (Centroid*)node;

	// calculate contact activity
	for(End& end : ends){
		for(int i = 0; i < end.faces.size(); i++){
			obj->param.faces[i].CalcNearest(end.var_pos->val, end.faces[i].pc, end.faces[i].iedge, end.faces[i].ivtx);
		}
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
	gravity  = vec3_t(0.0, 0.0, -9.8);
	mass     = 1.0;
}

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

//-------------------------------------------------------------------------------------------------

Centroid::Waypoint::Waypoint() {
	k = 0;
	time = 0.0;
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

	real_t t = 0.0;
	for (uint k = 0; k < graph->ticks.size(); k++) {
		CentroidKey* key = (CentroidKey*)traj.GetKeypoint(graph->ticks[k]);
	}

	Curve3d          curve_t;
	QuatCurved       curve_r;
	vector<Curve3d>  curve_end;
	
	curve_t.SetType(Interpolate::Cubic);
	curve_r.SetType(Interpolate::Cubic);
	
	for (uint i = 0; i < waypoints.size(); i++) {
		Waypoint& wp = waypoints[i];
		CentroidKey* key = (CentroidKey*)traj.GetKeypoint(graph->ticks[wp.k]);
		
		curve_t.AddPoint(t);
		curve_t.SetPos(i, wp.pos_t);
		curve_t.SetVel(i, wp.vel_t);

		curve_r.AddPoint(t);
		curve_r.SetPos(i, wp.pos_r);
		curve_r.SetVel(i, wp.vel_r);
	}

	for (uint k = 0; k < graph->ticks.size(); k++) {
		CentroidKey* key = (CentroidKey*)traj.GetKeypoint(graph->ticks[k]);
		real_t t = graph->ticks[k]->time;

		key->var_pos_t->val = curve_t.CalcPos(t);
		key->var_pos_r->val = curve_r.CalcPos(t);

		key->var_vel_t->val = curve_t.CalcVel(t);
		key->var_vel_r->val = curve_r.CalcVel(t);
	}

	// 経由点上の変数を固定
	for (uint i = 0; i < waypoints.size(); i++) {
		Waypoint& wp = waypoints[i];
		CentroidKey* key = (CentroidKey*)traj.GetKeypoint(graph->ticks[wp.k]);

		key->var_pos_t->locked = wp.fix_pos_t;
		key->var_pos_r->locked = wp.fix_pos_r;
		key->var_vel_t->locked = wp.fix_vel_t;
		key->var_vel_r->locked = wp.fix_vel_r;
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
		k0->tick->time, k0->var_pos_t->val, k0->var_vel_t->val,
		k1->tick->time, k1->var_pos_t->val, k1->var_vel_t->val,
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
		k0->tick->time, k0->var_pos_t->val, k0->var_vel_t->val,
		k1->tick->time, k1->var_pos_t->val, k1->var_vel_t->val,
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
		k0->tick->time, k0->var_pos_r->val, k0->var_vel_r->val,
		k1->tick->time, k1->var_pos_r->val, k1->var_vel_r->val,
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
		k0->tick->time, k0->var_pos_r->val, k0->var_vel_r->val,
		k1->tick->time, k1->var_pos_r->val, k1->var_vel_r->val,
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
		k0->tick->time, k0->ends[index].var_pos->val, vec3_t(),
		k1->tick->time, k1->ends[index].var_pos->val, vec3_t(),
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
		k0->tick->time, k0->ends[index].var_pos->val, vec3_t(),
		k1->tick->time, k1->ends[index].var_pos->val, vec3_t(),
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
		canvas->SetLineWidth(3.0f);
		canvas->SetLineColor("black");
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
			canvas->SetLineWidth(1.0f);
			canvas->BeginPath();
			canvas->MoveTo(trajectory[0].end_pos[i]);
			for (int k = 1; k < trajectory.size(); k++) {
				canvas->LineTo(trajectory[k].end_pos[i]);
			}
			canvas->EndPath();
			canvas->EndLayer();
		}
	}

	// face
	if(conf->Set(canvas, Render::Item::CentroidFace, this)){
		for(int i = 0; i < param.faces.size(); i++){
			canvas->BeginLayer("centroid_face", true);
			canvas->SetLineWidth(1.0f);
			canvas->BeginPath();
			canvas->MoveTo(param.faces[i].vertices[0]);
			for (uint j = 1; j <= trajectory.size(); j++) {
				canvas->LineTo(param.faces[i].vertices[j % param.faces[i].vertices.size()]);
			}
			canvas->EndPath();
			canvas->EndLayer();
		}
	}
}

void Centroid::CreateSnapshot(real_t t, Centroid::Snapshot& s){
	s.t = t;
	s.pos = ComPos(t);

	s.end_pos.resize(param.ends.size());
	for(int i = 0; i < param.ends.size(); i++)
		s.end_pos[i] = EndPos(t, i);
}

void Centroid::CreateSnapshot(real_t t){
	CreateSnapshot(t, snapshot);
}

void Centroid::DrawSnapshot(Render::Canvas* canvas, Render::Config* conf) {
	// lines connecting com and feet
	canvas->SetLineWidth(2.0f);
	canvas->BeginPath();
	for(int i = 0; i < param.ends.size(); i++){
		canvas->MoveTo(snapshot.pos);
		canvas->LineTo(snapshot.end_pos[i]);
	}
	canvas->EndPath();
}

///////////////////////////////////////////////////////////////////////////////////////////////////

CentroidCon::CentroidCon(Solver* solver, int _tag, string _name, CentroidKey* _obj, real_t _scale):
	Constraint(solver, 3, ID(_tag, _obj->node, _obj->tick, _name), _scale) {
	obj[0] = _obj;
	obj[1] = (CentroidKey*)_obj->next;
}

CentroidPosConT::CentroidPosConT(Solver* solver, int _tag, string _name, CentroidKey* _obj, real_t _scale):
	CentroidCon(solver, ConTag::CentroidPosT, _name, _obj, _scale) {

	AddSLink(obj[1]->var_pos_t);
	AddSLink(obj[0]->var_pos_t);
	AddSLink(obj[0]->var_vel_t);
}

CentroidPosConR::CentroidPosConR(Solver* solver, int _tag, string _name, CentroidKey* _obj, real_t _scale):
	CentroidCon(solver, ConTag::CentroidPosR, _name, _obj, _scale) {

	AddSLink(obj[1]->var_pos_r);
	AddSLink(obj[0]->var_pos_r);
	AddSLink(obj[0]->var_vel_r);
}

CentroidVelConT::CentroidVelConT(Solver* solver, int _tag, string _name, CentroidKey* _obj, real_t _scale):
	CentroidCon(solver, ConTag::CentroidVelT, _name, _obj, _scale) {

	AddSLink(obj[1]->var_vel_t);
	AddSLink(obj[0]->var_vel_t);
	for(CentroidKey::End& end : obj[0]->ends){
		for(CentroidKey::Face& face : end.faces){
			for(int k = 0; k < 3; k++){
				AddC3Link(face.var_force[k]);
			}
		}
	}
}

CentroidVelConR::CentroidVelConR(Solver* solver, int _tag, string _name, CentroidKey* _obj, real_t _scale):
	CentroidCon(solver, ConTag::CentroidVelR, _name, _obj, _scale) {

	AddSLink (obj[1]->var_vel_r);
	AddSLink (obj[0]->var_vel_r);
	AddX3Link(obj[0]->var_pos_t);
	for(CentroidKey::End& end : obj[0]->ends){
		AddX3Link(end.var_pos  );
		for(CentroidKey::Face& face : end.faces){
			for(int k = 0; k < 3; k++)
				AddC3Link(face.var_force[k]);
		}
	}
}

CentroidRangeCon::CentroidRangeCon(Solver* solver, int _tag, string _name, CentroidKey* _obj, int _iend, int _dir, real_t _scale):
	Constraint(solver, 1, ID(_tag, _obj->node, _obj->tick, _name), _scale){
	obj  = _obj;
	iend = _iend;
	dir  = _dir;

	AddR3Link(obj->var_pos_t);
	AddR3Link(obj->var_pos_r);
	AddR3Link(obj->ends[iend].var_pos);
}

CentroidVelCon::CentroidVelCon(Solver* solver, int _tag, string _name, CentroidKey* _obj, int _iend, real_t _scale):
	Constraint(solver, 1, ID(_tag, _obj->node, _obj->tick, _name), _scale){
	obj[0] = _obj;
	obj[1] = (CentroidKey*)_obj->next;
	iend  = _iend ;
	
	AddSLink (obj[0]->ends[iend].var_vel);
	AddR3Link(obj[0]->ends[iend].var_pos);
	AddR3Link(obj[1]->ends[iend].var_pos);
}

CentroidGapCon::CentroidGapCon(Solver* solver, int _tag, string _name, CentroidKey* _obj, int _iend, int _iface, real_t _scale):
	Constraint(solver, 1, ID(_tag, _obj->node, _obj->tick, _name), _scale){
	obj   = _obj;
	iend  = _iend ;
	iface = _iface;
	
	AddSLink (obj->ends[iend].faces[iface].var_gap);
	AddR3Link(obj->ends[iend].var_pos);
}

CentroidFrictionCon::CentroidFrictionCon(Solver* solver, int _tag, string _name, CentroidKey* _obj, int _iend, int _iface, real_t _scale):
	Constraint(solver, 1, ID(_tag, _obj->node, _obj->tick, _name), _scale){
	obj   = _obj  ;
	iend  = _iend ;
	iface = _iface;
	
	AddSLink(obj->ends[iend].faces[iface].var_force[0]);
	AddSLink(obj->ends[iend].faces[iface].var_force[1]);
	AddSLink(obj->ends[iend].faces[iface].var_force[2]);
}

CmplCon::CmplCon(Solver* solver, ID _id, SVar* _var, SVar* _var_slack, int _side, real_t _scale):
	Constraint(solver, 1, _id, _scale){
	var       = _var;
	var_slack = _var_slack;
	side      = _side;
}

///////////////////////////////////////////////////////////////////////////////////////////////////

void CentroidCon::Prepare(){

}

void CentroidRangeCon::Prepare(){
	p      = obj->var_pos_t->val;
	q      = obj->var_pos_r->val;
	pend   = obj->ends[iend].var_pos->val;
	dp     = pend - p;
	n[dir] = 1.0;
	nabs   = q*n;
}

void CentroidVelCon::Prepare(){
	p0 = obj[0]->ends[iend].var_pos->val;
	p1 = obj[1]->ends[iend].var_pos->val;
	dp = p1 - p0;
	
	real_t n = dp.norm();
	const real_t eps = 1.0e-10;
	if(n > eps)
		 dpn = dp/n;
	else dpn.clear();
}

void CentroidGapCon::Prepare(){
	Centroid* cen = (Centroid*)obj->node;
	CentroidKey::End&  end  = obj->ends[iend];
	CentroidKey::Face& face = end.faces[iface];

	p  = end.var_pos->val;
	pc = face.pc;
	dp = p - pc;

	real_t n = dp.norm();
	const real_t eps = 1.0e-10;
	if(n > eps)
		 dpn = dp/n;
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
}

void CentroidFrictionCon::Prepare(){
	Centroid* cen = (Centroid*)obj->node;

	mu  = cen->param.faces[iface].mu;
	f   = obj->ends[iend].faces[iface].f();
	ftn = vec2_t(f.x, f.y).norm();
}

///////////////////////////////////////////////////////////////////////////////////////////////////

void CentroidPosConT::CalcCoef(){
	int i = 0;
	((SLink*)links[i++])->SetCoef( 1.0);
	((SLink*)links[i++])->SetCoef(-1.0);
	((SLink*)links[i++])->SetCoef(-1.0);
}

void CentroidPosConR::CalcCoef(){
	int i = 0;
	((SLink*)links[i++])->SetCoef( 1.0);
	((SLink*)links[i++])->SetCoef(-1.0);
	((SLink*)links[i++])->SetCoef(-1.0);
}

void CentroidVelConT::CalcCoef(){
	int i = 0;
	((SLink*)links[i++])->SetCoef( 1.0);
	((SLink*)links[i++])->SetCoef(-1.0);
	for(CentroidKey::End& end : obj[0]->ends){
		for(CentroidKey::Face& face : end.faces){
			for(int k = 0; k < 3; k++){
				((C3Link*)links[i++])->SetCoef(-face.R.col(k));
			}
		}
	}
}

void CentroidVelConR::CalcCoef(){
	Centroid* cen = (Centroid*)obj[0]->node;

	int i = 0;
	((SLink*)links[i++])->SetCoef( 1.0);
	((SLink*)links[i++])->SetCoef(-1.0);

	real_t Iinv = 1.0/((Centroid*)obj[0]->node)->Inorm;
	vec3_t fsum;
	for(CentroidKey::End& end : obj[0]->ends){
		for(CentroidKey::Face& face : end.faces){
			fsum += face.R*face.f();
		}
	}
	((X3Link*)links[i++])->SetCoef(-Iinv*fsum);

	for(CentroidKey::End& end : obj[0]->ends){
		fsum.clear();
		for(CentroidKey::Face& face : end.faces){
			fsum += face.R*face.f();
		}
		((X3Link*)links[i++])->SetCoef(Iinv*fsum);

		vec3_t r = end.var_pos->val - obj[0]->var_pos_t->val;
		for(CentroidKey::Face& face : end.faces){
			for(int k = 0; k < 3; k++){
				((C3Link*)links[i++])->SetCoef( -Iinv*(r % face.R.col(k)) );
			}
		}
	}
}

void CentroidRangeCon::CalcCoef(){
	((R3Link*)links[0])->SetCoef(-nabs);
	((R3Link*)links[1])->SetCoef( nabs % dp );
	((R3Link*)links[2])->SetCoef( nabs);
}

void CentroidVelCon::CalcCoef(){
	int i = 0;

	((SLink* )links[i++])->SetCoef( 1.0);
	((R3Link*)links[i++])->SetCoef( dpn);
	((R3Link*)links[i++])->SetCoef(-dpn);
}

void CentroidGapCon::CalcCoef(){
	int i = 0;

	((SLink* )links[i++])->SetCoef( 1.0);
	((R3Link*)links[i++])->SetCoef(-dpn);
}

void CentroidFrictionCon::CalcCoef(){
	int i = 0;

	((SLink*)links[i++])->SetCoef(mu);

	const real_t eps = 1.0e-10;
	if(ftn < eps){
		((SLink*)links[i++])->SetCoef( 0.0);
		((SLink*)links[i++])->SetCoef( 0.0);
	}
	else{
		((SLink*)links[i++])->SetCoef(-f.x/ftn);
		((SLink*)links[i++])->SetCoef(-f.y/ftn);
	}
}

void CmplCon::CalcCoef(){

}

///////////////////////////////////////////////////////////////////////////////////////////////////

void CentroidPosConT::CalcDeviation(){
	y = obj[1]->var_pos_t->val - (obj[0]->var_pos_t->val + obj[0]->var_vel_t->val);
}

void CentroidPosConR::CalcDeviation(){
	quat_t q0     = obj[0]->var_pos_r->val;
	quat_t q1     = obj[1]->var_pos_r->val;
	quat_t q      = q0*quat_t::Rot(q0.Conjugated()*obj[0]->var_vel_r->val);
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
		for(CentroidKey::Face& face : end.faces){
			y -= face.R*face.f();
		}
	}

	y -= ((Centroid*)obj[0]->node)->gnorm;
}

void CentroidVelConR::CalcDeviation(){
	y = obj[1]->var_vel_r->val - obj[0]->var_vel_r->val;

	real_t Iinv = 1.0/((Centroid*)obj[0]->node)->Inorm;
	for(CentroidKey::End& end : obj[0]->ends){
		for(CentroidKey::Face& face : end.faces){
			y -= Iinv*((end.var_pos->val - obj[0]->var_pos_t->val) % face.f());
		}
	}
}

void CentroidRangeCon::CalcDeviation(){
	Centroid* cen = (Centroid*)obj->node;
	
	real_t r    = nabs*dp;
	real_t rmin = cen->param.ends[iend].rangeMin[dir];
	real_t rmax = cen->param.ends[iend].rangeMax[dir];
	if(r < rmin){
		y[0]   = r - rmin;
		active = true;
	}
	else if(r > rmax){
		y[0]   = r - rmax;
		active = true;
	}
	else{
		y[0]   = 0.0;
		active = false;
	}
}

void CentroidVelCon::CalcDeviation(){
	y[0] = obj[0]->ends[iend].var_vel->val - dp.norm();
}

void CentroidGapCon::CalcDeviation(){
	CentroidKey::End&  end  = obj->ends[iend];
	CentroidKey::Face& face = end.faces[iface];

	y[0] = face.var_gap->val - (end.var_pos->val - face.pc).norm();
}

void CentroidFrictionCon::CalcDeviation(){
	CentroidKey::Face& face = obj->ends[iend].faces[iface];
	vec3_t f  = face.f();
	
	y[0] = face.mu*f[2] - vec2_t(f[0], f[1]).norm();
	if(y[0] < 0.0){
		active = true;
	}
	else{
		y[0] = 0.0;
		active = false;
	}
}

void CmplCon::CalcDeviation(){

}

///////////////////////////////////////////////////////////////////////////////////////////////////

void CentroidPosConT::CalcLhs(){
	obj[1]->var_pos_t->val = obj[0]->var_pos_t->val + obj[0]->var_vel_t->val;
}

void CentroidPosConR::CalcLhs(){
	obj[1]->var_pos_r->val = obj[0]->var_pos_r->val * quat_t(obj[0]->var_pos_r->val.Conjugated()*obj[0]->var_vel_r->val);
}

void CentroidVelConT::CalcLhs(){
	obj[1]->var_vel_t->val = obj[0]->var_vel_t->val;
	for(CentroidKey::End& end : obj[0]->ends){
		for(CentroidKey::Face& face : end.faces){
			obj[1]->var_vel_t->val += face.R*face.f();
		}
	}
	obj[1]->var_vel_t->val += ((Centroid*)obj[0]->node)->gnorm;
}

void CentroidVelConR::CalcLhs(){
	obj[1]->var_vel_r->val = obj[0]->var_vel_r->val;
	real_t Iinv = 1.0/((Centroid*)obj[0]->node)->Inorm;
	for(CentroidKey::End& end : obj[0]->ends){
		for(CentroidKey::Face& face : end.faces){
			obj[1]->var_vel_r->val += Iinv*( (end.var_pos->val - obj[0]->var_pos_t->val) % (face.R*face.f()) );
		}
	}
}

}
