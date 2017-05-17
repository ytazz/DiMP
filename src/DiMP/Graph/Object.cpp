#include <DiMP/Graph/Object.h>
#include <DiMP/Graph/Geometry.h>
#include <DiMP/Graph/Joint.h>
#include <DiMP/Graph/Graph.h>
#include <DiMP/Solver/Solver.h>
#include <DiMP/Solver/Range.h>
#include <DiMP/Render/Config.h>
#include <DiMP/Render/Canvas.h>

namespace DiMP{;

//-------------------------------------------------------------------------------------------------
// ObjectKey

void ObjectKey::AddVar(Solver* solver){
	Object* obj = (Object*)node;
	
	for(uint i = 0; i < obj->cons.size(); i++){
		Connector* con = obj->cons[i];

		for(uint j = 0; j < con->joints.size(); j++){
			Joint* jnt = con->joints[j];

			joints.push_back( make_pair((JointKey*)jnt->traj.GetKeypoint(tick), jnt->sock == con) );
		}
	}

	if(obj->tree)
		 tree = (TreeKey*)obj->tree->traj.GetKeypoint(tick);
	else tree = 0;

	/// add variables
	pos_t = new V3Var(solver, ID(VarTag::ObjectTP, node, tick, name + "_tp"), obj->graph->scale.pos_t);
	pos_r = new QVar (solver, ID(VarTag::ObjectRP, node, tick, name + "_rp"), obj->graph->scale.pos_r);
	vel_t = new V3Var(solver, ID(VarTag::ObjectTV, node, tick, name + "_tv"), obj->graph->scale.vel_t);
	vel_r = new V3Var(solver, ID(VarTag::ObjectRV, node, tick, name + "_rv"), obj->graph->scale.vel_r);
}

void ObjectKey::AddCon(Solver* solver){
	Object* obj = (Object*)node;
	if(next){
		ObjectKey* nextObj = (ObjectKey*)next;

		if(!obj->tree){
			// C^1 continuity constraints
			con_c1_t    = new C1ConV3(solver, ID(ConTag::ObjectC1T, node, tick, name + "_c1_t"),
				pos_t, vel_t, nextObj->pos_t, nextObj->vel_t, node->graph->scale.pos_t);
			con_c1_t->h = hnext;
			con_c1_r[0] = new ObjectConC1R(solver, name + "_c1_r0", this, 0, node->graph->scale.pos_r);
			con_c1_r[1] = new ObjectConC1R(solver, name + "_c1_r1", this, 1, node->graph->scale.pos_r);
			con_c1_r[2] = new ObjectConC1R(solver, name + "_c1_r2", this, 2, node->graph->scale.pos_r);

			// sum-of-force constraints
			con_force_t = new ForceConT(solver, name + "_force_t", this, node->graph->scale.force_t);
			con_force_r = new ForceConR(solver, name + "_force_r", this, node->graph->scale.force_r);
		}
	}
}

void ObjectKey::AddLinks(Constraint* con, bool t_or_r, bool p_or_v, bool s_or_r){
	if(tree){
		for(uint j = 0; j < tree->joints.size(); j++){
			Variable* var = (p_or_v ? tree->joints[j]->pos[0] : tree->joints[j]->vel[0]);
			if(s_or_r)
				 con->AddCLink(var);
			else con->AddSLink(var);
		}
	}
	else{
		Variable* var;
		if(t_or_r){
			if(p_or_v)
				 var = pos_t;
			else var = vel_t;
		}
		else{
			if(p_or_v)
				 var = pos_r;
			else var = vel_r;
		}
		if(s_or_r)
			 con->AddSLink(var);
		else con->AddRLink(var);
	}
}

void ObjectKey::CalcCoef(Constraint* con, bool t_or_r, real_t k, uint& i){
	if(tree){
		int idx = tree->GetIndex(this);
		for(uint j = 0; j < tree->joints.size(); j++){
			vec3_t J = (t_or_r ? tree->Jv[idx][j] : tree->Jw[idx][j]);
			((CLink*)con->links[i++])->SetCoef(k * J);
		}
	}
	else ((SLink*)con->links[i++])->SetCoef(k);
}

void ObjectKey::CalcCoef(Constraint* con, bool t_or_r, vec3_t k, uint& i){
	if(tree){
		int idx = tree->GetIndex(this);
		for(uint j = 0; j < tree->joints.size(); j++){
			vec3_t J = (t_or_r ? tree->Jv[idx][j] : tree->Jw[idx][j]);
			((SLink*)con->links[i++])->SetCoef(k * J);
		}
	}
	else ((RLink*)con->links[i++])->SetCoef(k);
}

void ObjectKey::Prepare(){
	Object* obj = (Object*)node;
	fext_t = obj->param.mass * obj->graph->param.gravity;
	fext_r.clear();
}

void ObjectKey::Draw(Render::Canvas* canvas, Render::Config* conf){
	Object* obj = (Object*)node;
	Vec3f p0, p1;
	
	// position
	if(conf->Set(canvas, Render::Item::ObjectPos, obj)){
		p0 = pos_t->val;
		canvas->Point(p0);
	}
	// velocity
	if(conf->Set(canvas, Render::Item::ObjectVel, obj)){
		float s = conf->Scale(Render::Item::ObjectVel, obj);
		p1 = p0 + s * vel_t->val;
		canvas->Line(p0, p1);
	}
	// ang-velocity
	if(conf->Set(canvas, Render::Item::ObjectAngvel, obj)){
		float s = conf->Scale(Render::Item::ObjectAngvel, obj);
		p1 = p0 + s * vel_r->val;
		canvas->Line(p0, p1);
	}
}

//-------------------------------------------------------------------------------------------------
// Object

Object::Object(Graph* g, const string& n):TrajectoryNode(g, n){
	graph->objects.Add(this);
	tree = 0;
	type = Type::Object;
}

Object::~Object(){
	graph->objects.Remove(this);
}

void Object::Init(){
	for(uint i = 0; i < graph->ticks.size(); i++){
		ObjectKey* key = (ObjectKey*)traj.GetKeypoint(graph->ticks[i]);
		
		// treeŽž‚É]‘®‚·‚é•Ï”‚Ílock‚·‚é
		if(tree){
			key->pos_t->Lock();
			key->pos_r->Lock();
			key->vel_t->Lock();
			key->vel_r->Lock();
		}

		// set initial values
		key->pos_t->val = param.iniPos;
		key->pos_r->val = param.iniOri;
		key->vel_t->val = param.iniVel;
		key->vel_r->val = param.iniAngvel;

		// lock position and velocity if this keypoint is the first one, or the object is non-dynamical
		if(!param.dynamical || !key->prev){
			key->pos_t->Lock();
			key->pos_r->Lock();
			key->vel_t->Lock();
			key->vel_r->Lock();
		}
		
		// for non-dynamical objects, sum-of-force constraint is disabled
		if(!tree && !param.dynamical && key->next){
			key->con_force_t->enabled = false;
			key->con_force_r->enabled = false;
		}
	}
}

void Object::Prepare(){
	TrajectoryNode::Prepare();
	CalcBSphere();
}

void Object::ForwardKinematics(){
	for(uint i = 0; i < cons.size(); i++){
		Connector* con = cons[i];

		for(uint j = 0; j < con->joints.size(); j++){
			Joint* jnt = con->joints[j];

			if(jnt->sock == con)
				jnt->ForwardKinematics();
		}
	}
}

void Object::CalcBSphere(){
	bsphere = 0.0;
	for(uint i = 0; i < cons.size(); i++){
		Connector* con = cons[i];

		for(uint j = 0; j < con->geos.size(); j++){
			Geometry* geo = con->geos[j];

			bsphere = std::max(bsphere, con->pose.Pos().norm() + geo->CalcBSphere());	
		}
	}
}

vec3_t Object::Pos(real_t time, int type){
	KeyPair    kp = traj.GetSegment(time);
	ObjectKey* k0 = (ObjectKey*)kp.first ;
	ObjectKey* k1 = (ObjectKey*)kp.second;
	return InterpolatePos(
		time,
		k0->tick->time, k0->pos_t->val, k0->vel_t->val,
		k1->tick->time, k1->pos_t->val, k1->vel_t->val,
		type);
}

vec3_t Object::Vel(real_t time, int type){
	KeyPair    kp = traj.GetSegment(time);
	ObjectKey* k0 = (ObjectKey*)kp.first ;
	ObjectKey* k1 = (ObjectKey*)kp.second;
	return InterpolateVel(
		time,
		k0->tick->time, k0->pos_t->val, k0->vel_t->val,
		k1->tick->time, k1->pos_t->val, k1->vel_t->val,
		type);
}

vec3_t Object::Acc(real_t time, int type){
	KeyPair    kp = traj.GetSegment(time);
	ObjectKey* k0 = (ObjectKey*)kp.first ;
	ObjectKey* k1 = (ObjectKey*)kp.second;
	return InterpolateAcc(
		time,
		k0->tick->time, k0->pos_t->val, k0->vel_t->val,
		k1->tick->time, k1->pos_t->val, k1->vel_t->val,
		type);
}

quat_t Object::Ori(real_t time, int type){
	KeyPair    kp = traj.GetSegment(time);
	ObjectKey* k0 = (ObjectKey*)kp.first ;
	ObjectKey* k1 = (ObjectKey*)kp.second;
	return InterpolateOri(
		time,
		k0->tick->time, k0->pos_r->val, k0->vel_r->val,
		k1->tick->time, k1->pos_r->val, k1->vel_r->val,
		type);
}

vec3_t Object::Angvel(real_t time, int type){
	KeyPair    kp = traj.GetSegment(time);
	ObjectKey* k0 = (ObjectKey*)kp.first ;
	ObjectKey* k1 = (ObjectKey*)kp.second;
	return InterpolateAngvel(
		time,
		k0->tick->time, k0->pos_r->val, k0->vel_r->val,
		k1->tick->time, k1->pos_r->val, k1->vel_r->val,
		type);
}

vec3_t Object::Angacc(real_t time, int type){
	KeyPair    kp = traj.GetSegment(time);
	ObjectKey* k0 = (ObjectKey*)kp.first ;
	ObjectKey* k1 = (ObjectKey*)kp.second;
	return InterpolateAngacc(
		time,
		k0->tick->time, k0->pos_r->val, k0->vel_r->val,
		k1->tick->time, k1->pos_r->val, k1->vel_r->val,
		type);
}

void Object::Draw(Render::Canvas* canvas, Render::Config* conf){
	if(conf->Set(canvas, Render::Item::ObjectTrajectory, this)){
		DrawTrajectory(canvas);
	}

	for(Trajectory::iterator it = traj.begin(); it != traj.end(); it++)
		(*it)->Draw(canvas, conf);
}

void Object::DrawTrajectory(Render::Canvas* canvas, uint ndiv){
	if(traj.empty())
		return;

	// draw spline as line strip
	real_t tf = traj.back()->tick->time;
	real_t dt = tf/(real_t)ndiv;
	real_t t0 = 0.0, t1 = dt;
	Vec3f p0 = Pos(t0, Interpolate::Quadratic), p1;
	for(uint i = 0; i < ndiv; i++){
		p1 = Pos(t1, Interpolate::Quadratic);
		canvas->Line(p0, p1);
		p0 = p1;
		t0 = t1;
		t1 += dt;
	}
}

void Object::DrawSnapshot(real_t time, Render::Canvas* canvas, Render::Config* conf){
	pose_t pose;
	pose.Pos() = Pos(time, Interpolate::Quadratic);
	pose.Ori() = Ori(time, Interpolate::SlerpDiff);
	DrawSnapshot(pose, canvas, conf);
}

void Object::DrawSnapshot(const pose_t& pose, Render::Canvas* canvas, Render::Config* conf){
	Affinef aff;
	
	/// draw geometries attached to connectors
	pose.ToAffine(aff);
	canvas->Push();
	canvas->Transform(aff);
	
	for(uint i = 0; i < cons.size(); i++){
		Connector* con = cons[i];

		con->pose.ToAffine(aff);
		
		canvas->Push();
		canvas->Transform(aff);
		
		for(uint j = 0; j < con->geos.size(); j++){
			con->geos[j]->Draw(canvas, conf);
		}
		canvas->Pop();
	}
	canvas->Pop();
}

//-------------------------------------------------------------------------------------------------

ObjectConC1R::ObjectConC1R(Solver* solver, const string& _name, ObjectKey* _obj, int _idx, real_t _scale):
	Constraint(solver, 3, ID(ConTag::ObjectC1R, _obj->node, _obj->tick, _name), _scale){
	obj[0] = _obj;
	obj[1] = (ObjectKey*)_obj->next;
	idx	   = _idx;
	
	AddXLink(obj[0]->pos_r);
	AddXLink(obj[0]->vel_r);
	AddXLink(obj[1]->pos_r);
	AddXLink(obj[1]->vel_r);
}
	
void ObjectConC1R::CalcCoef(){
	real_t h = obj[0]->hnext;
	vec3_t	base;
	base[idx] = 1.0;
	r[0] = obj[0]->pos_r->val * base;
	r[1] = obj[1]->pos_r->val * base;

	uint i = 0;
	((XLink*)links[i++])->SetCoef( r[0]);
	((XLink*)links[i++])->SetCoef((0.5*h)*r[0]);
	((XLink*)links[i++])->SetCoef(-r[1]);
	((XLink*)links[i++])->SetCoef((0.5*h)*r[1]);
}

void ObjectConC1R::CalcDeviation(){
	real_t h = obj[0]->hnext;
	vec3_t w[2];
	w[0] = obj[0]->vel_r->val;
	w[1] = obj[1]->vel_r->val;
	
	y = r[1] - r[0] + ((0.5*h)*r[0]) % w[0] + ((0.5*h)*r[1]) % w[1];
}

//-------------------------------------------------------------------------------------------------

ForceConT::ForceConT(Solver* solver, const string& _name, ObjectKey* _obj, real_t _scale):
	Constraint(solver, 3, ID(ConTag::ForceT, _obj->node, _obj->tick, _name), _scale){
	obj[0] = _obj;
	obj[1] = (ObjectKey*)_obj->next;

	JointKey* jnt;
	bool sock;
	
	AddSLink(obj[0]->vel_t);
	AddSLink(obj[1]->vel_t);

	for(uint i = 0; i < obj[0]->joints.size(); i++){
		jnt  = obj[0]->joints[i].first;
		sock = obj[0]->joints[i].second;
		AddSLink(jnt->force_t, sock ? 1.0 : -1.0);
	}
}

void ForceConT::CalcCoef(){
	real_t h = obj[0]->hnext;
	real_t m = ((Object*)obj[0]->node)->param.mass;

	((SLink*)links[0])->SetCoef(-m/h);
	((SLink*)links[1])->SetCoef( m/h);
}

void ForceConT::CalcDeviation(){
	Constraint::CalcDeviation();
	y -= obj[0]->fext_t;	///< subtract external force
}

//-------------------------------------------------------------------------------------------------

ForceConR::ForceConR(Solver* solver, const string& _name, ObjectKey* _obj, real_t _scale):
	Constraint(solver, 3, ID(ConTag::ForceR, _obj->node, _obj->tick, _name), _scale){
	obj[0] = _obj;
	obj[1] = (ObjectKey*)_obj->next;

	JointKey* jnt;
	bool sock;
	
	AddSLink(obj[0]->vel_r);
	AddSLink(obj[1]->vel_r);
	AddMLink(obj[0]->pos_r);

	for(uint i = 0; i < obj[0]->joints.size(); i++){
		jnt  = obj[0]->joints[i].first;
		sock = obj[0]->joints[i].second;
		AddXLink(jnt->force_t);
		AddSLink(jnt->force_r, sock ? 1.0 : -1.0);
	}
}

void ForceConR::CalcCoef(){
	real_t h = obj[0]->hnext;
	real_t I = ((Object*)obj[0]->node)->param.inertia;

	JointKey* jnt;
	bool sock;

	((SLink*)links[0])->SetCoef(-I/h);
	((SLink*)links[1])->SetCoef( I/h);

	MLink* mlink = (MLink*)links[2];
	mat3_t m;
	m.clear();
	for(uint i = 0; i < obj[0]->joints.size(); i++){
		jnt  = obj[0]->joints[i].first;
		sock = obj[0]->joints[i].second;
	
		if(sock)
			 m += mat3_t::Cross(jnt->r[0] + jnt->q[0] * jnt->rrel) * mat3_t::Cross(jnt->force_t->val);
		else m -= mat3_t::Cross(jnt->r[1])                         * mat3_t::Cross(jnt->force_t->val);
	}
	mlink->SetCoef(m);
	
	for(uint i = 0; i < obj[0]->joints.size(); i++){
		jnt  = obj[0]->joints[i].first;
		sock = obj[0]->joints[i].second;
	
		XLink* xlink = (XLink*)links[3+2*i];
		if(sock)
			 xlink->SetCoef( jnt->r[0] + jnt->q[0] * jnt->rrel);
		else xlink->SetCoef(-jnt->r[1]);
	}
}

void ForceConR::CalcDeviation(){
	real_t h = obj[0]->hnext;
	real_t I = ((Object*)obj[0]->node)->param.inertia;

	JointKey* jnt;
	bool sock;

	y = (I/h) * (obj[1]->vel_r->val - obj[0]->vel_r->val);
	
	for(uint i = 0; i < obj[0]->joints.size(); i++){
		jnt  = obj[0]->joints[i].first;
		sock = obj[0]->joints[i].second;
		if(sock)
			 y += (jnt->force_r->val + (jnt->r[0] + jnt->q[0] * jnt->rrel) % jnt->force_t->val);
		else y -= (jnt->force_r->val +  jnt->r[1]                          % jnt->force_t->val);
	}

	y -= obj[0]->fext_r;	//< subtract external moment
}

}
