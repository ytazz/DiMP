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

	ends .resize( obj->param.ends .size() );
	faces.resize( obj->param.faces.size() );

}

void CentroidKey::AddVar(Solver* solver) {
	Centroid* obj = (Centroid*)node;

	// com position and velocity
	var_com_pos_t = new V3Var(solver, ID(VarTag::CentroidComTP, node, tick, name + "_com_tp"), node->graph->scale.pos_t);
	var_com_pos_r = new QVar (solver, ID(VarTag::CentroidComRP, node, tick, name + "_com_rp"), node->graph->scale.pos_r);
	var_com_vel_t = new V3Var(solver, ID(VarTag::CentroidComTV, node, tick, name + "_com_tv"), node->graph->scale.vel_t);
	var_com_vel_r = new V3Var(solver, ID(VarTag::CentroidComRV, node, tick, name + "_com_rv"), node->graph->scale.vel_r);

	stringstream ss;
	int i = 0;
	for(End& end : ends){
		ss.str("");
		ss << name << "_end" << i << "_tp";
		end.var_end_pos_t = new V3Var(solver, ID(VarTag::CentroidEndTP, node, tick, ss.str()), node->graph->scale.pos_t);
		//ss.str("");
		//ss << name << "_end" << i << "_tv";
		//end.var_end_vel_t = new V3Var(solver, ID(VarTag::CentroidEndTV, node, tick, ss.str()), node->graph->scale.vel_t);
		i++;
	}
	i = 0;
	for(Face& face : faces){
		int nbasis = obj->param.faces[i].coneBasis.size();
		face.var_force.resize(nbasis);
		face.var_vel  .resize(nbasis);

		for(int j = 0; j < nbasis; j++){
			ss.str("");
			ss << name << "_face" << i << "_force" << j;
			face.var_force[j] = new SVar(solver, ID(VarTag::CentroidForce, node, tick, ss.str()), node->graph->scale.force_t);

			ss.str("");
			ss << name << "_face" << i << "_vel" << j;
			face.var_vel[j] = new SVar(solver, ID(VarTag::CentroidVel, node, tick, ss.str()), node->graph->scale.vel_t);
		}
		i++;
	}
}

void CentroidKey::AddCon(Solver* solver) {
	Centroid* obj = (Centroid*)node;
	CentroidKey* nextObj = (CentroidKey*)next;

}

void CentroidKey::Prepare() {
	Centroid* obj = (Centroid*)node;

}

void CentroidKey::Draw(Render::Canvas* canvas, Render::Config* conf) {
	Centroid* obj = (Centroid*)node;

	Vec3f pcom, pcop, pf[2], pt;

	canvas->SetPointSize(5.0f);
	canvas->SetLineWidth(1.0f);

	pcom = var_com_pos_t->val;
	canvas->Point(pcom);
}

//-------------------------------------------------------------------------------------------------
// Centroid

Centroid::Param::Param() {
	gravity  = vec3_t(0.0, 0.0, -9.8);
	mass     = 1.0;
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

	real_t t = 0.0;
	for (uint k = 0; k < graph->ticks.size(); k++) {
		CentroidKey* key = (CentroidKey*)traj.GetKeypoint(graph->ticks[k]);
	}

	Curve3d          curve_com_t;
	QuatCurved       curve_com_r;
	vector<Curve3d>  curve_end;
	
	curve_com_t.SetType(Interpolate::Cubic);
	curve_com_r.SetType(Interpolate::Cubic);
	
	for (uint i = 0; i < waypoints.size(); i++) {
		Waypoint& wp = waypoints[i];
		CentroidKey* key = (CentroidKey*)traj.GetKeypoint(graph->ticks[wp.k]);
		
		curve_com_t.AddPoint(t);
		curve_com_t.SetPos(i, wp.com_pos_t);
		curve_com_t.SetVel(i, wp.com_vel_t);

		curve_com_r.AddPoint(t);
		curve_com_r.SetPos(i, wp.com_pos_r);
		curve_com_r.SetVel(i, wp.com_vel_r);
	}

	for (uint k = 0; k < graph->ticks.size(); k++) {
		CentroidKey* key = (CentroidKey*)traj.GetKeypoint(graph->ticks[k]);
		real_t t = graph->ticks[k]->time;

		key->var_com_pos_t->val = curve_com_t.CalcPos(t);
		key->var_com_pos_r->val = curve_com_r.CalcPos(t);

		key->var_com_vel_t->val = curve_com_t.CalcVel(t);
		key->var_com_vel_r->val = curve_com_r.CalcVel(t);
	}

	// 経由点上の変数を固定
	for (uint i = 0; i < waypoints.size(); i++) {
		Waypoint& wp = waypoints[i];
		CentroidKey* key = (CentroidKey*)traj.GetKeypoint(graph->ticks[wp.k]);

		key->var_com_pos_t->locked = wp.fix_com_pos_t;
		key->var_com_pos_r->locked = wp.fix_com_pos_r;
		key->var_com_vel_t->locked = wp.fix_com_vel_t;
		key->var_com_vel_r->locked = wp.fix_com_vel_r;
	}
}

void Centroid::Prepare() {
	TrajectoryNode::Prepare();
	trajReady = false;
}

vec3_t Centroid::ComPos(real_t t, int type) {
	if(traj.empty())
		return vec3_t();

	KeyPair      kp = traj.GetSegment(t);
	CentroidKey* k0 = (CentroidKey*)kp.first;
	CentroidKey* k1 = (CentroidKey*)kp.second;

	return InterpolatePos(
		t,
		k0->tick->time, k0->var_com_pos_t->val, k0->var_com_vel_t->val,
		k1->tick->time, k1->var_com_pos_t->val, k1->var_com_vel_t->val,
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
		k0->tick->time, k0->var_com_pos_t->val, k0->var_com_vel_t->val,
		k1->tick->time, k1->var_com_pos_t->val, k1->var_com_vel_t->val,
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
		k0->tick->time, k0->var_com_pos_r->val, k0->var_com_vel_r->val,
		k1->tick->time, k1->var_com_pos_r->val, k1->var_com_vel_r->val,
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
		k0->tick->time, k0->var_com_pos_r->val, k0->var_com_vel_r->val,
		k1->tick->time, k1->var_com_pos_r->val, k1->var_com_vel_r->val,
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
		k0->tick->time, k0->ends[index].var_end_pos_t->val, k0->ends[index].end_vel_t,
		k1->tick->time, k1->ends[index].var_end_pos_t->val, k1->ends[index].end_vel_t,
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
		k0->tick->time, k0->ends[index].var_end_pos_t->val, k0->ends[index].end_vel_t,
		k1->tick->time, k1->ends[index].var_end_pos_t->val, k1->ends[index].end_vel_t,
		type);
}

void Centroid::CalcTrajectory() {
	real_t tf = traj.back()->tick->time;
	real_t dt = 0.01;

	trajectory.clear();
	for (real_t t = 0.0; t <= tf; t += dt) {
		TrajPoint tp;
		tp.t = t;
		tp.com_pos = ComPos(t);

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

}

void Centroid::DrawSnapshot(real_t time, Render::Canvas* canvas, Render::Config* conf) {
	canvas->SetLineWidth(2.0f);
	canvas->BeginPath();
	canvas->EndPath();
}

}
