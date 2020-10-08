#include <DiMP/Graph/Centroid.h>
#include <DiMP/Graph/Graph.h>
#include <DiMP/Render/Config.h>
#include <DiMP/Render/Canvas.h>

namespace DiMP {;

const real_t pi = M_PI;

//-------------------------------------------------------------------------------------------------
// CentroidKey

CentroidKey::CentroidKey() {
	Centroid* obj = (Centroid*)node;

	int nend = (int)obj->param.ends .size();
	ends.resize(nend );
}

void CentroidKey::AddVar(Solver* solver) {
	Centroid* obj = (Centroid*)node;

	// position and velocity
	var_pos_t = new V3Var(solver, ID(VarTag::CentroidTP, node, tick, name + "_tp"), node->graph->scale.pos_t);
	var_pos_r = new QVar (solver, ID(VarTag::CentroidRP, node, tick, name + "_rp"), node->graph->scale.pos_r);
	var_vel_t = new V3Var(solver, ID(VarTag::CentroidTV, node, tick, name + "_tv"), node->graph->scale.vel_t);
	var_vel_r = new V3Var(solver, ID(VarTag::CentroidRV, node, tick, name + "_rv"), node->graph->scale.vel_r);

	stringstream ss;
	for(int i = 0; i < ends.size(); i++){
		ss.str("");
		ss << name << "_end" << i << "_tp";
		ends[i].var_pos_t = new V3Var(solver, ID(VarTag::CentroidEndTP, node, tick, ss.str()), node->graph->scale.pos_t);
		
		ss.str("");
		ss << name << "_end" << i << "_tf";
		ends[i].var_force_t = new V3Var(solver, ID(VarTag::CentroidEndTF, node, tick, ss.str()), node->graph->scale.force_t);
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
		for(int j = 0; j < 3; j++){
			ss.str("");
			ss << name << "_end" << i << "_range" << j;
			ends[i].con_range[j] = new CentroidEndRangeCon(solver, ConTag::CentroidEndRange, ss.str(), this, i, j, node->graph->scale.pos_t);
		}

		ss.str("");
		ss << name << "_end" << i << "_vel";
		ends[i].con_vel = new CentroidEndVelCon(solver, ConTag::CentroidEndVel, ss.str(), this, i, node->graph->scale.vel_t);

		ss.str("");
		ss << name << "_end" << i << "_force";
		ends[i].con_force = new CentroidEndForceCon(solver, ConTag::CentroidEndForce, ss.str(), this, i, node->graph->scale.force_t);
	}
}		

void CentroidKey::Prepare() {
	Centroid* obj = (Centroid*)node;

	// calculate contact activity
	for(End& end : ends){
		obj->CalcNearest(end.var_pos_t->val, end.pc, end.Rc);
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

void Centroid::Param::Face::CalcNearest(const vec3_t& p, real_t& _dmin, vec3_t& _pc, mat3_t& _Rc, int& _dim){
	bool inside = true;
	for(Edge& e : edges){
		if(e.normal*p > e.offset)
			inside = false;
	}

	// projected on face
	if(inside){
		vec3_t n = R.col(2);
		_pc  = p - (n*(p - origin))*n;
		_Rc  = R;
		_dim = 2;
		return;
	}

	for(Edge& e : edges){
		if( e.n *p > e.offset &&
			e.t*(p - e.v0]) >= 0.0 &&
			e.t*(p - e.v1]) <= 0.0 ){
			_pc = e.v[0] + ((p - e.v[0])*e.t)*e.t; 
			n = (p - _pc)
		}
	}

	if(onedge){
		_dim = 1;
		return;
	}

	for(vec3_t& v : vertices){

	}
}

//-------------------------------------------------------------------------------------------------

Centroid::Waypoint::Waypoint() {
	k = 0;
	time = 0.0;
}

//-------------------------------------------------------------------------------------------------

Centroid::TrajPoint::TrajPoint() {
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
			vec3_t& v0 = f.vertices[(i+0)%nvtx];
			vec3_t& v1 = f.vertices[(i+1)%nvtx];
			vec3_t  te = v1 - v0;
			vec3_t  ne = te % nf;
			real_t  oe = ne * (v0 - f.origin);
			if(oe < 0.0){
				ne = -ne;
				oe = -oe;
			}

			e.normal = ne;
			e.offset = oe;
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

void Centroid::CalcNearest(const vec3_t& p, vec3_t& _pc, mat3_t& _Rc){
	Face*  fmin = 0 ;
	real_t dmin;
	for(Param::Face& f : param.faces){
		real_t d = f.CalcNearest(p, );
		if(d < dmin){
			fmin = &f;
			dmin = d;
		}
	}
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
		k0->tick->time, k0->ends[index].var_pos_t->val, vec3_t(),
		k1->tick->time, k1->ends[index].var_pos_t->val, vec3_t(),
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
		k0->tick->time, k0->ends[index].var_pos_t->val, vec3_t(),
		k1->tick->time, k1->ends[index].var_pos_t->val, vec3_t(),
		type);
}

void Centroid::CalcTrajectory() {
	real_t tf = traj.back()->tick->time;
	real_t dt = 0.01;

	trajectory.clear();
	for (real_t t = 0.0; t <= tf; t += dt) {
		TrajPoint tp;
		tp.t = t;
		tp.pos = ComPos(t);

		trajectory.push_back(tp);
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
			for (uint i = 1; i < trajectory.size(); i++) {
				canvas->LineTo(trajectory[i].end_pos[i]);
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

void Centroid::DrawSnapshot(real_t time, Render::Canvas* canvas, Render::Config* conf) {
	canvas->SetLineWidth(2.0f);
	canvas->BeginPath();
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

	AddSLink(obj[1]->var_vel_r);
	AddSLink(obj[0]->var_vel_r);
	for(CentroidKey::End& end : obj[0]->ends){
		AddSLink(end.var_force_t);
	}
}

CentroidVelConR::CentroidVelConR(Solver* solver, int _tag, string _name, CentroidKey* _obj, real_t _scale):
	CentroidCon(solver, ConTag::CentroidVelR, _name, _obj, _scale) {

	AddSLink (obj[1]->var_vel_r);
	AddSLink (obj[0]->var_vel_r);
	AddX3Link(obj[0]->var_pos_t);
	for(CentroidKey::End& end : obj[0]->ends){
		AddX3Link(end.var_pos_t  );
		AddX3Link(end.var_force_t);
	}
}

CentroidEndRangeCon::CentroidEndRangeCon(Solver* solver, int _tag, string _name, CentroidKey* _obj, int _iend, int _dir, real_t _scale):
	Constraint(solver, 1, ID(_tag, _obj->node, _obj->tick, _name), _scale){
	obj  = _obj;
	iend = _iend;
	dir  = _dir;

	AddSLink(obj->var_pos_t);
	AddSLink(obj->var_pos_r);
	AddSLink(obj->ends[iend].var_pos_t);
}

CentroidEndVelCon::CentroidEndVelCon(Solver* solver, int _tag, string _name, CentroidKey* _obj, int _iend, real_t _scale):
	Constraint(solver, 1, ID(_tag, _obj->node, _obj->tick, _name), _scale){
	obj[0] = _obj;
	obj[1] = (CentroidKey*)_obj->next;
	iend = _iend;
	
	AddR3Link(obj[0]->ends[iend].var_pos_t);
	AddR3Link(obj[0]->ends[iend].var_pos_t);
}

CentroidEndForceCon::CentroidEndForceCon(Solver* solver, int _tag, string _name, CentroidKey* _obj, int _iend, real_t _scale):
	Constraint(solver, 1, ID(_tag, _obj->node, _obj->tick, _name), _scale){
	obj  = _obj;
	iend = _iend;
	
	AddR3Link(obj->ends[iend].var_force_t);
}

///////////////////////////////////////////////////////////////////////////////////////////////////

void CentroidCon::Prepare(){

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
		for(int j = 0; j < 3; j++){
			vec3_t ej;
			ej[j] = 1.0;
			((C3Link*)links[i++])->SetCoef(-ej);
		}
	}
}

void CentroidVelConR::CalcCoef(){
	int i = 0;
	((SLink*)links[i++])->SetCoef( 1.0);
	((SLink*)links[i++])->SetCoef(-1.0);

	real_t Iinv = 1.0/((Centroid*)obj[0]->node)->Inorm;
	vec3_t fsum;
	for(CentroidKey::End& end : obj[0]->ends){
		fsum += end.var_force_t->val;
	}
	((X3Link*)links[i++])->SetCoef(Iinv*fsum);

	for(CentroidKey::End& end : obj[0]->ends){
		((X3Link*)links[i++])->SetCoef(-Iinv*end.var_force_t->val);
		((X3Link*)links[i++])->SetCoef( Iinv*(end.var_pos_t->val - obj[0]->var_pos_t->val));
	}
}

void CentroidEndRangeCon::CalcCoef(){

}

void CentroidEndVelCon::CalcCoef(){

}

void CentroidEndForceCon::CalcCoef(){

}

///////////////////////////////////////////////////////////////////////////////////////////////////

void CentroidPosConT::CalcDeviation(){
	y = obj[1]->var_pos_t->val - (obj[0]->var_pos_t->val + obj[0]->var_vel_t->val);
}

void CentroidPosConR::CalcDeviation(){
	quat_t q0     = obj[0]->var_pos_r->val;
	quat_t q1     = obj[1]->var_pos_r->val;
	quat_t q      = q0*quat_t(q0.Conjugated()*obj[0]->var_vel_r->val);
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
		y -= end.var_force_t->val;
	}

	y -= ((Centroid*)obj[0]->node)->gnorm;
}

void CentroidVelConR::CalcDeviation(){
	y = obj[1]->var_vel_r->val - obj[0]->var_vel_r->val;

	real_t Iinv = 1.0/((Centroid*)obj[0]->node)->Inorm;
	for(CentroidKey::End& end : obj[0]->ends){
		y -= Iinv*((end.var_pos_t->val - obj[0]->var_pos_t->val) % end.var_force_t->val);
	}
}

void CentroidEndRangeCon::CalcDeviation(){
	Centroid* cen = (Centroid*)obj->node;

	quat_t q    = obj->var_pos_r->val;
	quat_t qinv = q.Conjugated();
	vec3_t p    = obj->var_pos_t->val;
	vec3_t pend = obj->ends[iend].var_pos_t->val;
	
	real_t r    = (qinv*(pend - p))[dir];
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

void CentroidEndVelCon::CalcDeviation(){
	Centroid* cen = (Centroid*)obj[0]->node;

	vec3_t p0 = obj[0]->ends[iend].var_pos_t->val;
	vec3_t p1 = obj[1]->ends[iend].var_pos_t->val;
	vec3_t pc = obj[0]->ends[iend].pc;
	mat3_t Rc = obj[0]->ends[iend].Rc;

	vec3_t dp = p1 - p0;
	vec2_t dp2;
	dp2.x = Rc.col(0)*dp;
	dp2.y = Rc.col(1)*dp;

	y[0] = cen->param.gamma*Rc.col(2)*(p0 - pc) - dp2.norm();
	if(y[0] < 0.0){
		active = true;
	}
	else{
		y[0] = 0.0;
		active = false;
	}
}

void CentroidEndForceCon::CalcDeviation(){
	Centroid* cen = (Centroid*)obj->node;

	mat3_t Rc = obj->ends[iend].Rc;
	vec3_t f  = obj->ends[iend].var_force_t->val;
	vec2_t f2;
	f2.x = Rc.col(0)*f;
	f2.y = Rc.col(1)*f;

	y[0] = cen->param.mu*Rc.col(2)*f - f2.norm();
	if(y[0] < 0.0){
		active = true;
	}
	else{
		y[0] = 0.0;
		active = false;
	}
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
		obj[1]->var_vel_t->val += end.var_force_t->val;
	}
	obj[1]->var_vel_t->val += ((Centroid*)obj[0]->node)->gnorm;
}

void CentroidVelConR::CalcLhs(){
	obj[1]->var_vel_r->val = obj[0]->var_vel_r->val;
	real_t Iinv = 1.0/((Centroid*)obj[0]->node)->Inorm;
	for(CentroidKey::End& end : obj[0]->ends){
		obj[1]->var_vel_r->val += Iinv*((end.var_pos_t->val - obj[0]->var_pos_t->val) % end.var_force_t->val);
	}
}

}
