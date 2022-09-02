#include <DiMP/Graph/Object.h>
#include <DiMP/Graph/Geometry.h>
#include <DiMP/Graph/Joint.h>
#include <DiMP/Graph/Graph.h>
#include <DiMP/Render/Config.h>
#include <DiMP/Render/Canvas.h>

namespace DiMP{;

//-------------------------------------------------------------------------------------------------
// ObjectKey

void ObjectKey::AddVar(Solver* solver){
	Object* obj = (Object*)node;
	
	for(Connector* con : obj->cons){
		for(Joint* jnt : con->joints){
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


	//octtree = new OcttreeNode();
	//octtree->depth =  0;
	//octtree->id    =  0;
	//octtree->bbmin =  obj->graph->param.bbmin;
	//octtree->bbmax =  obj->graph->param.bbmax;

}

void ObjectKey::AddLinks(Constraint* con, const ObjectKey::OptionS& opt){
	if(tree){
		int i = ((Object*)node)->treeIndex;
		for(JointKey* jnt : tree->joints){
			int j = ((Joint*)jnt->node)->treeIndex;

			// skip non-dependent joint
			if( !((Tree*)tree->node)->dependent[i][j] )
				continue;

			int ndof = (int)((Joint*)jnt->node)->dof;
			for(int n = 0; n < ndof; n++){
				if(opt.tp || opt.rp) con->AddSLink(jnt->pos[n]);
				if(opt.tv || opt.rv) con->AddSLink(jnt->vel[n]);
			}
		}
	}
	else{
		if(opt.tp) con->AddR3Link(pos_t);
		if(opt.rp) con->AddR3Link(pos_r);
		if(opt.tv) con->AddR3Link(vel_t);
		if(opt.rv) con->AddR3Link(vel_r);
	}
}

void ObjectKey::AddLinks(Constraint* con, const ObjectKey::OptionV3& opt){
	if(tree){
		int i = ((Object*)node)->treeIndex;
		for(JointKey* jnt : tree->joints){
			int j = ((Joint*)jnt->node)->treeIndex;

			// skip non-dependent joint
			if( !((Tree*)tree->node)->dependent[i][j] )
				continue;

			int ndof = (int)((Joint*)jnt->node)->dof;
			for(int n = 0; n < ndof; n++){
				if(opt.tp || opt.rp) con->AddC3Link(jnt->pos[n]);
				if(opt.tv || opt.rv) con->AddC3Link(jnt->vel[n]);
			}
		}
	}
	else{
		if(opt.tp) con->AddSLink(pos_t);
		if(opt.rp) con->AddSLink(pos_r);
		if(opt.tv) con->AddSLink(vel_t);
		if(opt.rv) con->AddSLink(vel_r);
	}
}

void ObjectKey::CalcCoef(Constraint* con, const ObjectKey::OptionS& opt, uint& idx){
	if(tree){
		int i = ((Object*)node)->treeIndex;
		for(JointKey* jnt : tree->joints){
			int j   = ((Joint*)jnt->node)->treeIndex;
			int j2  = ((Joint*)jnt->node)->treeDofIndex;
			int dof = ((Joint*)jnt->node)->dof;
			
			// skip non-dependent joint
			if( !((Tree*)tree->node)->dependent[i][j] )
				continue;

			for(int n = 0; n < dof; n++){
				real_t J;
				if(opt.tp || opt.rp){
					J = 0.0;
					if(opt.tp) J += opt.k_tp * tree->Jv[i][j2+n];
					if(opt.rp) J += opt.k_rp * tree->Jw[i][j2+n];
					((SLink*)con->links[idx++])->SetCoef(J);
				}
				if(opt.tv || opt.rv){
					J = 0.0;
					if(opt.tv) J += opt.k_tv * tree->Jv[idx][j2+n];
					if(opt.rv) J += opt.k_rv * tree->Jw[idx][j2+n];
					((SLink*)con->links[idx++])->SetCoef(J);
				}
			}
		}
	}
	else{
		if(opt.tp) ((R3Link*)con->links[idx++])->SetCoef(opt.k_tp);
		if(opt.rp) ((R3Link*)con->links[idx++])->SetCoef(opt.k_rp);
		if(opt.tv) ((R3Link*)con->links[idx++])->SetCoef(opt.k_tv);
		if(opt.rv) ((R3Link*)con->links[idx++])->SetCoef(opt.k_rv);
	}
}

void ObjectKey::CalcCoef(Constraint* con, const ObjectKey::OptionV3& opt, uint& idx){
	if(tree){
		int i = ((Object*)node)->treeIndex;
		for(JointKey* jnt : tree->joints){
			int j   = ((Joint*)jnt->node)->treeIndex;
			int j2  = ((Joint*)jnt->node)->treeDofIndex;
			int dof = ((Joint*)jnt->node)->dof;
			
			// skip non-dependent joint
			if( !((Tree*)tree->node)->dependent[i][j] )
				continue;

			for(int n = 0; n < dof; n++){
				vec3_t J;
				if(opt.tp || opt.rp){
					J.clear();
					if(opt.tp) J += opt.k_tp * tree->Jv[i][j2+n];
					if(opt.rp) J += opt.k_rp * tree->Jw[i][j2+n];
					((C3Link*)con->links[idx++])->SetCoef(J);
				}
				if(opt.tv || opt.rv){
					J.clear();
					if(opt.tv) J += opt.k_tv * tree->Jv[i][j2+n];
					if(opt.rv) J += opt.k_rv * tree->Jw[i][j2+n];
					((C3Link*)con->links[idx++])->SetCoef(J);
				}
			}
		}
	}
	else{
		if(opt.tp) ((SLink*)con->links[idx++])->SetCoef(opt.k_tp);
		if(opt.rp) ((SLink*)con->links[idx++])->SetCoef(opt.k_rp);
		if(opt.tv) ((SLink*)con->links[idx++])->SetCoef(opt.k_tv);
		if(opt.rv) ((SLink*)con->links[idx++])->SetCoef(opt.k_rv);
	}
}

void ObjectKey::PrepareGeometry(){
	Object* obj = (Object*)node;
	
	if(geoInfos.empty()){
		for(Connector* con : obj->cons){
			for(Geometry* geo : con->geos){
				geoInfos .push_back(GeometryInfo(tick, con, geo));
			}
		}
		edgeInfos[0].resize(2*geoInfos.size());
		edgeInfos[1].resize(2*geoInfos.size());
		edgeInfos[2].resize(2*geoInfos.size());
		dminMax = 0.0;  //< dminMax will be calculated by Graph::ExtractGeometryPairs
	}

	// absolute pose of geometries
	for(GeometryInfo& info : geoInfos){
		info.poseAbs = pose_t(pos_t->val, pos_r->val) * info.con->pose;
		
		// bsphere
		info.bsphereCenterAbs = info.poseAbs * info.geo->bsphereCenter;
		
		// bbox
		vec3_t dir, dir_local;
		for(int j = 0; j < 3; j++){
			dir.clear();
			dir[j]    = 1.0;
			dir_local = info.poseAbs.Ori().Conjugated() * dir;
			info.bbmin[j]  = info.poseAbs.Pos()[j] + dir_local * info.geo->CalcSupport(-dir_local);
			info.bbmax[j]  = info.poseAbs.Pos()[j] + dir_local * info.geo->CalcSupport( dir_local);
		}
	}

	// update octtree
	/*
	static vector<GeometryInfo*> tmp;
	tmp.resize(geoInfos.size());
	copy(geoInfos.begin(), geoInfos.end(), tmp.begin());
	octtree->Assign(tmp);
	*/

	// calc edges
	for(int dir = 0; dir < 3; dir++){
		for(int i = 0; i < geoInfos.size(); i++){
			edgeInfos[dir][2*i+0].geoInfo = &geoInfos[i];
			edgeInfos[dir][2*i+1].geoInfo = &geoInfos[i];
			edgeInfos[dir][2*i+0].side    = 0;
			edgeInfos[dir][2*i+1].side    = 1;
			edgeInfos[dir][2*i+0].val     = geoInfos[i].bbmin[dir] - dminMax;
			edgeInfos[dir][2*i+1].val     = geoInfos[i].bbmax[dir] + dminMax;
		}
	}
	//sort(edgeInfos.begin(), edgeInfos.end());

}

void ObjectKey::Prepare(){
	Object* obj = (Object*)node;

	fext_t = obj->param.mass * obj->graph->param.gravity;
	fext_r.clear();

	if( obj->param.dynamical && !obj->param.stationary )
		PrepareGeometry();
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

	// geometries
	Affinef aff;
	for(GeometryInfo& info : geoInfos){
		info.poseAbs.ToAffine(aff);
		canvas->Push();
		canvas->Transform(aff);
		info.geo->Draw(canvas, conf);
		canvas->Pop();
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
	TrajectoryNode::Init();

	CalcBSphere();

	for(Tick* tick : graph->ticks){
		ObjectKey* key = (ObjectKey*)traj.GetKeypoint(tick);
		
		// tree時に従属する変数はlockする
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

		// lock position and velocity if this keypoint is the first one, or the object is non-dynamical or stationary
		if( (!param.dynamical || param.stationary) || !key->prev){
			key->pos_t->Lock();
			key->pos_r->Lock();
			key->vel_t->Lock();
			key->vel_r->Lock();
		}
		
		// for non-dynamical objects, sum-of-force constraint is disabled
		if(!tree && (!param.dynamical || param.stationary) && key->next){
			key->con_force_t->enabled = false;
			key->con_force_r->enabled = false;
		}

		// for non-dynamical object, PrepareGeometry is called here
		// for stationary object, PrepareGeometry is called for first keypoint only
		if( !param.dynamical && ( !param.stationary || tick->idx == 0) )
			key->PrepareGeometry();
	}

}

void Object::Prepare(){
	TrajectoryNode::Prepare();
}

void Object::ForwardKinematics(){
	for(Connector* con : cons){
		for(Joint* jnt : con->joints){
			if(jnt->sock == con)
				jnt->ForwardKinematics();
		}
	}
}

void Object::ForwardKinematics(real_t t){
	for(Connector* con : cons){
		for(Joint* jnt : con->joints){
			if(jnt->sock == con)
				jnt->ForwardKinematics(t);
		}
	}
}

void Object::CalcBSphere(){
	bsphere = 0.0;
	for(Connector* con : cons){
		for(Geometry* geo : con->geos){
			geo->CalcBSphere();
			bsphere = std::max(bsphere, con->pose.Pos().norm() + geo->bsphereCenter.norm() + geo->bsphereRadius);
		}
	}
}

vec3_t Object::Pos(real_t time, int type){
	if(traj.empty())
		return vec3_t();

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
	if(traj.empty())
		return vec3_t();

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
	if(traj.empty())
		return vec3_t();

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
	if(traj.empty())
		return quat_t();

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
	if(traj.empty())
		return vec3_t();

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
	if(traj.empty())
		return vec3_t();

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

void Object::CreateSnapshot(real_t t){
	snapshot.pos = Pos(t, Interpolate::Quadratic);
	snapshot.ori = Ori(t, Interpolate::SlerpDiff);
}

void Object::DrawSnapshot(Render::Canvas* canvas, Render::Config* conf){
	DrawSnapshot(pose_t(snapshot.pos, snapshot.ori), canvas, conf);
}

void Object::DrawSnapshot(const pose_t& pose, Render::Canvas* canvas, Render::Config* conf){
	Affinef aff;
	
	/// draw geometries attached to connectors
	pose.ToAffine(aff);
	canvas->Push();
	canvas->Transform(aff);
	
	for(Connector* con : cons){
		con->pose.ToAffine(aff);
		
		canvas->Push();
		canvas->Transform(aff);
		
		for(Geometry* geo : con->geos){
			geo->Draw(canvas, conf);
		}
		canvas->Pop();
	}
	canvas->Pop();
}

//-------------------------------------------------------------------------------------------------

ObjectConC1R::ObjectConC1R(Solver* solver, const string& _name, ObjectKey* _obj, int _idx, real_t _scale):
	Constraint(solver, 3, ID(ConTag::ObjectC1R, _obj->node, _obj->tick, _name), Constraint::Type::Equality, _scale){
	obj[0] = _obj;
	obj[1] = (ObjectKey*)_obj->next;
	idx	   = _idx;
	
	AddX3Link(obj[0]->pos_r);
	AddX3Link(obj[0]->vel_r);
	AddX3Link(obj[1]->pos_r);
	AddX3Link(obj[1]->vel_r);
}
	
void ObjectConC1R::CalcCoef(){
	real_t h = obj[0]->hnext;
	vec3_t	base;
	base[idx] = 1.0;
	r[0] = obj[0]->pos_r->val * base;
	r[1] = obj[1]->pos_r->val * base;

	uint i = 0;
	((X3Link*)links[i++])->SetCoef( r[0]);
	((X3Link*)links[i++])->SetCoef((0.5*h)*r[0]);
	((X3Link*)links[i++])->SetCoef(-r[1]);
	((X3Link*)links[i++])->SetCoef((0.5*h)*r[1]);
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
	Constraint(solver, 3, ID(ConTag::ForceT, _obj->node, _obj->tick, _name), Constraint::Type::Equality, _scale){
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
	Constraint(solver, 3, ID(ConTag::ForceR, _obj->node, _obj->tick, _name), Constraint::Type::Equality, _scale){
	obj[0] = _obj;
	obj[1] = (ObjectKey*)_obj->next;

	JointKey* jnt;
	bool sock;
	
	AddSLink (obj[0]->vel_r);
	AddSLink (obj[1]->vel_r);
	AddM3Link(obj[0]->pos_r);

	for(uint i = 0; i < obj[0]->joints.size(); i++){
		jnt  = obj[0]->joints[i].first;
		sock = obj[0]->joints[i].second;
		AddX3Link(jnt->force_t);
		AddSLink (jnt->force_r, sock ? 1.0 : -1.0);
	}
}

void ForceConR::CalcCoef(){
	real_t h = obj[0]->hnext;
	real_t I = ((Object*)obj[0]->node)->param.inertia;

	JointKey* jnt;
	bool sock;

	((SLink*)links[0])->SetCoef(-I/h);
	((SLink*)links[1])->SetCoef( I/h);

	M3Link* mlink = (M3Link*)links[2];
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
	
		X3Link* xlink = (X3Link*)links[3+2*i];
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
