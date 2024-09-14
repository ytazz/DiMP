#include <DiMP/Graph/Joint.h>
#include <DiMP/Graph/Object.h>
#include <DiMP/Graph/Graph.h>
#include <DiMP/Render/Config.h>
#include <DiMP/Render/Canvas.h>

namespace DiMP{;

static const real_t pi = M_PI;

JointKey::JointKey(){
}

void JointKey::AddVar(Solver* solver){
	Joint* jnt = (Joint*)node;

	uint n = jnt->dof;
	Jv          .resize(n);
	Jw          .resize(n);
	pos         .resize(n);
	vel         .resize(n);
	acc         .resize(n);
	torque      .resize(n);
	con_c0p     .resize(n);
	con_c0v     .resize(n);
	con_c1p     .resize(n);
	con_force   .resize(n);
	con_range_p .resize(n);
	con_range_v .resize(n);
	con_range_f .resize(n);
    con_des_p   .resize(n);
    con_des_v   .resize(n);

	sockObj = (ObjectKey*)jnt->sock->obj->traj.GetKeypoint(tick);
	plugObj = (ObjectKey*)jnt->plug->obj->traj.GetKeypoint(tick);
	if(jnt->tree)
		 tree = (TreeKey*)jnt->tree->traj.GetKeypoint(tick);
	else tree = 0;

	if(!tree){
		force_t = new V3Var(solver, ID(VarTag::ForceT, node, tick, name + "_force_t"), jnt->graph->scale.force_t);
		force_r = new V3Var(solver, ID(VarTag::ForceR, node, tick, name + "_force_r"), jnt->graph->scale.force_r);
		solver->AddInputVar(force_t, tick->idx);
		solver->AddInputVar(force_r, tick->idx);
	}

	// register joint position and velocity variables
	for(uint i = 0; i < n; i++){
		real_t sp = (jnt->IsRotational(i) ? jnt->graph->scale.pos_r   : jnt->graph->scale.pos_t  );
		real_t sv = (jnt->IsRotational(i) ? jnt->graph->scale.vel_r   : jnt->graph->scale.vel_t  );
		real_t sa = (jnt->IsRotational(i) ? jnt->graph->scale.acc_r   : jnt->graph->scale.acc_t  );
		real_t sf = (jnt->IsRotational(i) ? jnt->graph->scale.force_r : jnt->graph->scale.force_t);

		pos   [i] = new SVar(solver, ID(VarTag::JointP, node, tick, name + "_pos"   ), sp);
		solver->AddStateVar(pos[i], tick->idx);

		vel   [i] = new SVar(solver, ID(VarTag::JointV, node, tick, name + "_vel"   ), sv);

		if(jnt->param.trajType == Joint::TrajectoryType::C0){
			solver->AddInputVar(vel[i], tick->idx);
		}
		if(jnt->param.trajType == Joint::TrajectoryType::C1){
			solver->AddStateVar(vel[i], tick->idx);

			acc   [i] = new SVar(solver, ID(VarTag::JointA, node, tick, name + "_acc"   ), sa);
			torque[i] = new SVar(solver, ID(VarTag::JointF, node, tick, name + "_torque"), sf);
			solver->AddInputVar(acc   [i], tick->idx);
			solver->AddInputVar(torque[i], tick->idx);
		}
	}
}

void JointKey::AddCon(Solver* solver){
	Joint* jnt = (Joint*)node;

	if(!tree){
		con_tp = new JointConTP(solver, name + "_tp", this, node->graph->scale.pos_t);
		con_tv = new JointConTV(solver, name + "_tv", this, node->graph->scale.vel_t);
		con_rp = new JointConRP(solver, name + "_rp", this, node->graph->scale.pos_r);
		con_rv = new JointConRV(solver, name + "_rv", this, node->graph->scale.vel_r);
		solver->AddCostCon(con_tp, tick->idx);
		solver->AddCostCon(con_tv, tick->idx);
		solver->AddCostCon(con_rp, tick->idx);
		solver->AddCostCon(con_rv, tick->idx);
	}

	for(int i = 0; i < jnt->dof; i++){
		real_t sp = (jnt->IsRotational(i) ? jnt->graph->scale.pos_r   : jnt->graph->scale.pos_t);
		real_t sv = (jnt->IsRotational(i) ? jnt->graph->scale.vel_r   : jnt->graph->scale.vel_t);
		real_t sf = (jnt->IsRotational(i) ? jnt->graph->scale.force_r : jnt->graph->scale.force_t);
		stringstream ss;
		ss << i;
		
		// range constraint
		con_range_p[i] = new RangeConS(solver, ID(ConTag::JointRangeP, node, tick, name + "_range_p" + ss.str()), pos[i]   , sp);
		con_range_v[i] = new RangeConS(solver, ID(ConTag::JointRangeV, node, tick, name + "_range_v" + ss.str()), vel[i]   , sv);
		solver->AddCostCon(con_range_p[i], tick->idx);
		solver->AddCostCon(con_range_v[i], tick->idx);

        // desired value
        con_des_p[i] = new FixConS(solver, ID(ConTag::JointDesP, node, tick, name + "_des_p" + ss.str()), pos[i], sp);
        con_des_v[i] = new FixConS(solver, ID(ConTag::JointDesV, node, tick, name + "_des_v" + ss.str()), vel[i], sv);
        solver->AddCostCon(con_des_p[i], tick->idx);
		solver->AddCostCon(con_des_v[i], tick->idx);
        // disabled by default
        con_des_p[i]->enabled = false;
        con_des_v[i]->enabled = false;
		
		if(next){
			JointKey* nextJnt = (JointKey*)next;
			
			// position change range constraint
			//con_range_dp[i] = new DiffConS(solver, ID(ConTag::JointRangeDP, node, tick, name + "_range_dp" + ss.str()), nextJnt->pos[i], pos[i], sp);
			//solver->AddCostCon(con_range_dp[i], tick->idx);

			// continuity constraint
			if(jnt->param.trajType == Joint::TrajectoryType::C0){
				con_c0p[i] = new C0ConS(solver, ID(ConTag::JointC0P, node, tick, name + "_c0p" + ss.str()), pos[i], vel[i], nextJnt->pos[i], sp);
				con_c0p[i]->h = hnext;
				solver->AddTransitionCon(con_c0p[i], tick->idx);
			}
			if(jnt->param.trajType == Joint::TrajectoryType::C1){
				con_c0v[i] = new C0ConS(solver, ID(ConTag::JointC0V, node, tick, name + "_c0v" + ss.str()), vel[i], acc[i], nextJnt->vel[i], sv);
				con_c0v[i]->h = hnext;
				con_c1p[i] = new C1ConS(solver, ID(ConTag::JointC1P, node, tick, name + "_c1p" + ss.str()), pos[i], vel[i], acc[i], nextJnt->pos[i], sp);
				con_c1p[i]->h = hnext;
				solver->AddTransitionCon(con_c0v[i], tick->idx);
				solver->AddTransitionCon(con_c1p[i], tick->idx);

				// force-torque constraint
				if(!tree){
					con_force[i] = new JointConF(solver, name + "_torque" + ss.str(), this, i, sf);
				}
				// torque range constraint
				con_range_f[i] = new RangeConS(solver, ID(ConTag::JointRangeF, node, tick, name + "_range_f" + ss.str()), torque[i], sf);

			}
		}
	}
}

void JointKey::Prepare(){
	ScheduledKey::Prepare();

	Joint* jnt = (Joint*)node;
	r[0] = sockObj->pos_r->val * jnt->sock->pose.Pos();
	r[1] = plugObj->pos_r->val * jnt->plug->pose.Pos();
	q[0] = sockObj->pos_r->val * jnt->sock->pose.Ori();
	q[1] = plugObj->pos_r->val * jnt->plug->pose.Ori();

	if(jnt->dof > 0){
		vector<real_t> vpos(jnt->dof);
		for(int i = 0; i < jnt->dof; i++)
			vpos[i] = pos[i]->val;
	
		jnt->CalcRelativePose(&vpos[0], rrel  , qrel  );
		jnt->CalcJacobian    (&vpos[0], &Jv[0], &Jw[0]);

		for(int i = 0; i < jnt->dof; i++){
			Jv[i] = q[0] * Jv[i];
			Jw[i] = q[0] * Jw[i];
		}
	}

	vrel.clear();
	wrel.clear();
	for(int i = 0; i < jnt->dof; i++){
		vrel += Jv[i] * vel[i]->val;
		wrel += Jw[i] * vel[i]->val;
	}

	// タイムスロットとキーポイントの位置関係に応じて拘束と変数を設定
	bool in, inprev, innext;
	in     = (relation != Outside);
	inprev = prev && (((ScheduledKey*)prev)->relation != Outside);
	innext = next && (((ScheduledKey*)next)->relation != Outside);
	
	// 変数
	if(!(in && innext) && next){
		if(!tree){
			force_t->val.clear();
			force_r->val.clear();
			force_t->Lock();
			force_r->Lock();
		}
		if(jnt->param.trajType == Joint::TrajectoryType::C1){
			for(int i = 0; i < jnt->dof; i++){
				torque[i]->val = 0.0;
				torque[i]->Lock();
			}
		}
	}
	
	// 拘束
	if(!tree){
		con_tp->active = (in && inprev);
		con_rp->active = (in && inprev);
		con_tv->active = (in && inprev);
		con_rv->active = (in && inprev);
	}

	for(int i = 0; i < jnt->dof; i++){
		if(next){
			if(jnt->param.trajType == Joint::TrajectoryType::C0){
				con_c0p    [i]->active = in && innext;
			}
			if(jnt->param.trajType == Joint::TrajectoryType::C1){
				con_c0v    [i]->active = in && innext;
				con_c1p    [i]->active = in && innext;
				con_range_f[i]->active = in && innext;
				if(!tree){
					con_force[i]->active = in && innext;
				}
			}
		}
		
		con_range_p[i]->active = in && inprev;
		con_range_v[i]->active = in && inprev;
		con_des_p  [i]->active = in && inprev;
		con_des_v  [i]->active = in && inprev;
	}
}

void JointKey::Finish(){
	Joint* jnt = (Joint*)node;

	// wrap values to [-pi, pi] for rotaional joints
    for(int i = 0; i < (int)jnt->dof; i++){
    	//if(jnt->IsRotational(i)){
    	//	while(pos[i]->val < -pi) pos[i]->val += 2*pi;
    	//	while(pos[i]->val >  pi) pos[i]->val -= 2*pi;
    	//}
    }
}

void JointKey::Draw(Render::Canvas* canvas, Render::Config* conf){
	Joint* jnt = (Joint*)node;
	Vec3f p0, p1;
	
	// 拘束力の作用点
	Vec3f pf = sockObj->pos_t->val + r[0] + q[0] * rrel;

	// joint force
	if(conf->Set(canvas, Render::Item::JointForce, jnt)){
		real_t s = conf->Scale(Render::Item::JointForce, jnt);
		p0 = pf;
		p1 = p0 + s * force_t->val;
		canvas->Line(p0, p1);
	}
	// joint moment
	if(conf->Set(canvas, Render::Item::JointMoment, jnt)){
		real_t s = conf->Scale(Render::Item::JointMoment, jnt);
		p0 = pf;
		p1 = p0 + s * force_r->val;
		canvas->Line(p0, p1);
	}
	
	if(conf->Set(canvas, Render::Item::Connector, jnt)){
		// object center to plug
		p0 = plugObj->pos_t->val;
		p1 = p0 + r[1];
		canvas->Line(p0, p1);
	
		// object center to socket
		p0 = sockObj->pos_t->val;
		p1 = p0 + r[0];
		canvas->Line(p0, p1);
	}
}

//-------------------------------------------------------------------------------------------------

Joint::Param::Param(){
}

void Joint::Param::SetDof(uint dof){
	ini_p  .resize(dof);
	ini_v  .resize(dof);
	rmin_p .resize(dof);
	rmax_p .resize(dof);
	//rmin_dp.resize(dof);
	//rmax_dp.resize(dof);
	rmin_v .resize(dof);
	rmax_v .resize(dof);
	rmin_f .resize(dof);
	rmax_f .resize(dof);
    des_p  .resize(dof);
    des_v  .resize(dof);

	real_t inf = numeric_limits<real_t>::max();
	for(uint i = 0; i < dof; i++){
		ini_p  [i] =  0.0;
		ini_v  [i] =  0.0;
		rmin_p [i] = -inf;
		rmax_p [i] =  inf;
		//rmin_dp[i] = -inf;
		//rmax_dp[i] =  inf;
		rmin_v [i] = -inf;
		rmax_v [i] =  inf;
		rmin_f [i] = -inf;
		rmax_f [i] =  inf;
        des_p  [i] =  0.0;
        des_v  [i] =  0.0;
	}

	trajType = TrajectoryType::C0;
}

Joint::Joint(Connector* _sock, Connector* _plug, TimeSlot* _time, const string& n):ScheduledNode(_sock->graph, _time, n){
	sock = _sock;
	plug = _plug;
	sock->joints.Add(this);
	plug->joints.Add(this);

	tree = 0;
	type = Type::Joint;

	graph->joints.Add(this);
}

Joint::~Joint(){
	graph->joints.Remove(this);
}

void Joint::SetDof(uint n){
	dof = n;
	//axis .resize(n);
	param.SetDof(n);
}

void Joint::Init(){
	for(uint k = 0; k < graph->ticks.size(); k++){
		JointKey* key = (JointKey*)traj.GetKeypoint(graph->ticks[k]);

		for(int i = 0; i < dof; i++){
			// set initial values to positions and velocities
			key->pos[i]->val = param.ini_p[i];
			key->vel[i]->val = param.ini_v[i];
		
			// lock position and velocity of initial time
			if(!key->prev){
				key->pos[i]->Lock();
				key->vel[i]->Lock();
			}

			// add movable range constraint
			key->con_range_p[i]->_min = param.rmin_p[i];
			key->con_range_p[i]->_max = param.rmax_p[i];
			key->con_range_v[i]->_min = param.rmin_v[i];
			key->con_range_v[i]->_max = param.rmax_v[i];
			if(key->next){
				//key->con_range_dp[i]->_min = param.rmin_dp[i];
				//key->con_range_dp[i]->_max = param.rmax_dp[i];
				if(param.trajType == TrajectoryType::C1){
					key->con_range_f [i]->_min = param.rmin_f [i];
					key->con_range_f [i]->_max = param.rmax_f [i];
				}
			}

            key->con_des_p[i]->desired = param.des_p[i];
			key->con_des_v[i]->desired = param.des_v[i];
			
		}
	}
}

real_t Joint::Pos(uint i, real_t t, int type){
	if(!graph->ready)
		return 0.0;

	KeyPair   kp = traj.GetSegment(t)  ;
	JointKey* k0 = (JointKey*)kp.first ;
	JointKey* k1 = (JointKey*)kp.second;

	// consider wrapping for rotational joints
	real_t p0 = k0->pos[i]->val;
	real_t p1 = k1->pos[i]->val;

	//if(IsRotational(i)){
	//	while(p1 < p0 - pi) p1 += 2*pi;
	//	while(p1 > p0 + pi) p1 -= 2*pi;
	//}

	real_t pt = InterpolatePos(t,
		k0->tick->time, p0, k0->vel[i]->val,
		k1->tick->time, p1, k1->vel[i]->val,
		type);

	//if(IsRotational(i)){
	//	while(pt < -pi) pt += 2*pi;
	//	while(pt >  pi) pt -= 2*pi;
	//}

	return pt;
}

real_t Joint::Vel(uint i, real_t t, int type){
	if(!graph->ready)
		return 0.0;

	KeyPair   kp = traj.GetSegment(t)  ;
	JointKey* k0 = (JointKey*)kp.first ;
	JointKey* k1 = (JointKey*)kp.second;
	return InterpolateVel(t,
		k0->tick->time, k0->pos[i]->val, k0->vel[i]->val,
		k1->tick->time, k1->pos[i]->val, k1->vel[i]->val,
		type);
}

real_t Joint::Acc(uint i, real_t t, int type){
	if(!graph->ready)
		return 0.0;

	KeyPair   kp = traj.GetSegment(t)  ;
	JointKey* k0 = (JointKey*)kp.first ;
	JointKey* k1 = (JointKey*)kp.second;
	return InterpolateAcc(t,
		k0->tick->time, k0->pos[i]->val, k0->vel[i]->val,
		k1->tick->time, k1->pos[i]->val, k1->vel[i]->val,
		type);
}

void Joint::ForwardKinematics(){
	pose_t prel, p0, p1, psock, pplug;
	vec3_t vrel, v0, v1;
	vec3_t wrel, w0, w1;

	vector<real_t> vpos;
	vector<real_t> vvel;

	for(uint k = 0; k < graph->ticks.size(); k++){
		JointKey* key = (JointKey*)traj.GetKeypoint(graph->ticks[k]);

		// socket object pose
		p0.Pos() = key->sockObj->pos_t->val;
		p0.Ori() = key->sockObj->pos_r->val;
		v0       = key->sockObj->vel_t->val;
		w0       = key->sockObj->vel_r->val;

		// joint relative pose
		if(dof > 0){
			vpos.resize(dof);
			vvel.resize(dof);
			for(int i = 0; i < dof; i++){
				vpos[i] = key->pos[i]->val;
				vvel[i] = key->vel[i]->val;
			}
			CalcRelativePose(&vpos[0], prel.Pos(), prel.Ori());
			CalcRelativeVel (&vvel[0], vrel, wrel);
		}
		else{
			prel = pose_t();
			vrel = vec3_t();
			wrel = vec3_t();
		}

		psock = p0 * sock->pose;
		pplug = prel * plug->pose.Inv();
		
		// plug object pose
		p1 = psock * pplug;

		v1 = v0 + w0 % (p1.Pos() - p0.Pos()) + psock.Ori() * (vrel + wrel % pplug.Pos());
		w1 = w0 + psock.Ori() * wrel;

		key->plugObj->pos_t->val = p1.Pos();
		key->plugObj->pos_r->val = p1.Ori();
		key->plugObj->vel_t->val = v1;
		key->plugObj->vel_r->val = w1;
	}
	plug->obj->ForwardKinematics();
}

void Joint::ForwardKinematics(real_t t){
	pose_t prel, p0, p1, psock, pplug;
	vec3_t vrel, v0, v1;
	vec3_t wrel, w0, w1;

	// socket object pose
	p0.Pos() = sock->obj->snapshot.pos;
	p0.Ori() = sock->obj->snapshot.ori;
	v0       = sock->obj->snapshot.vel;
	w0       = sock->obj->snapshot.angvel;

	if(dof > 0){
		CreateSnapshot(t);
		CalcRelativePose(&snapshot.pos[0], prel.Pos(), prel.Ori());
		CalcRelativeVel (&snapshot.vel[0], vrel, wrel);
	}

	psock = p0 * sock->pose;
	pplug = prel * plug->pose.Inv();
		
	// plug object pose
	p1 = psock * pplug;

	v1 = v0 + w0 % (p1.Pos() - p0.Pos()) + psock.Ori() * (vrel + wrel % pplug.Pos());
	w1 = w0 + psock.Ori() * wrel;

	plug->obj->snapshot.pos    = p1.Pos();
	plug->obj->snapshot.ori    = p1.Ori();
	plug->obj->snapshot.vel    = v1;
	plug->obj->snapshot.angvel = w1;

	plug->obj->ForwardKinematics(t);
}

void Joint::ResetJointPos(){
	pose_t p0, p1, prel;
	vector<real_t> vpos(dof);

	for(uint k = 0; k < graph->ticks.size(); k++){
		JointKey* key = (JointKey*)traj.GetKeypoint(graph->ticks[k]);

		p0.Pos() = key->sockObj->pos_t->val;
		p0.Ori() = key->sockObj->pos_r->val;
		p1.Pos() = key->plugObj->pos_t->val;
		p1.Ori() = key->plugObj->pos_r->val;

		prel = (p0 * sock->pose).Inv() * (p1 * plug->pose);

		CalcJointPos(&vpos[0], prel.Pos(), prel.Ori());

		for(int i = 0; i < dof; i++)
			key->pos[i]->val = vpos[i];
	}
}

void Joint::CalcDeviation(real_t t, vec3_t& pos_dev, vec3_t& ori_dev){
	vec3_t r[2], rrel;
	quat_t q[2], qrel;
	r[0] = sock->obj->Ori(t) * sock->pose.Pos();
	r[1] = plug->obj->Ori(t) * plug->pose.Pos();
	q[0] = sock->obj->Ori(t) * sock->pose.Ori();
	q[1] = plug->obj->Ori(t) * plug->pose.Ori();

	vector<real_t> vpos;
	vpos.resize(dof);
	for(int i = 0; i < dof; i++)
		vpos[i] = Pos(i, t);
	CalcRelativePose(&vpos[0], rrel, qrel);

	pos_dev = (sock->obj->Pos(t) + r[0] + q[0] * rrel) - (plug->obj->Pos(t) + r[1]);

	quat_t qerror = q[1].Conjugated() * (q[0] * qrel);
	vec3_t axis   = qerror.Axis ();
	real_t theta  = qerror.Theta();
	if(theta > pi)
		theta -= 2*pi;
	ori_dev = q[1] * (theta * axis);
}

void Joint::CreateSnapshot(real_t t){
	snapshot.pos.resize(dof);
	snapshot.vel.resize(dof);
	for(int i = 0; i < dof; i++){
		snapshot.pos[i] = Pos(i, t);
		snapshot.vel[i] = Vel(i, t);
	}
}

void Joint::DrawSnapshot(Render::Canvas* canvas, Render::Config* conf){
	pose_t pose = pose_t(sock->obj->snapshot.pos, sock->obj->snapshot.ori) * sock->pose;
	
	/// draw geometries attached to connectors
	Affinef aff;
	pose.ToAffine(aff);
	canvas->Push     ();
	canvas->Transform(aff);

	OnDraw((dof > 0 ? &snapshot.pos[0] : 0), canvas);

	canvas->Pop();
}


//-------------------------------------------------------------------------------------------------

Hinge::Hinge(Connector* _sock, Connector* _plug, TimeSlot* _time, const string& n): Joint(_sock, _plug, _time, n){
	SetDof(1);
}

void Hinge::CalcRelativePose(real_t* pos, vec3_t& p, quat_t& q){
	p.clear();
	q = quat_t::Rot(pos[0], 'z');
}
void Hinge::CalcRelativeVel(real_t* vel, vec3_t& v, vec3_t& w){
	v.clear();
	w = vec3_t(0.0, 0.0, vel[0]);
}
void Hinge::CalcJointPos(real_t* pos, vec3_t& p, quat_t& q){
	pos[0] = q.Theta();
}

void Hinge::CalcJacobian(real_t* pos, vec3_t* Jv, vec3_t* Jw){
	Jv[0].clear();
	Jw[0] = vec3_t(0.0, 0.0, 1.0);
}

//-------------------------------------------------------------------------------------------------

Slider::Slider(Connector* _sock, Connector* _plug, TimeSlot* _time, const string& n): Joint(_sock, _plug, _time, n){
	SetDof(1);
}

void Slider::CalcRelativePose(real_t* pos, vec3_t& p, quat_t& q){
	p = vec3_t(0.0, 0.0, pos[0]);
	q = quat_t();
}
void Slider::CalcRelativeVel(real_t* vel, vec3_t& v, vec3_t& w){
	v = vec3_t(0.0, 0.0, vel[0]);
	w.clear();
}
void Slider::CalcJointPos(real_t* pos, vec3_t& p, quat_t& q){
	pos[0] = p.z;
}

void Slider::CalcJacobian(real_t* pos, vec3_t* Jv, vec3_t* Jw){
	Jv[0] = vec3_t(0.0, 0.0, 1.0);
	Jw[0].clear();
}

//-------------------------------------------------------------------------------------------------

Balljoint::Balljoint(Connector* _sock, Connector* _plug, TimeSlot* _time, const string& n): Joint(_sock, _plug, _time, n){
	SetDof(3);
}

void Balljoint::CalcRelativePose(real_t* pos, vec3_t& p, quat_t& q){
	real_t yaw   = pos[0];
	real_t pitch = pos[1];
	real_t roll  = pos[2];
	
	p.clear();
	q.w = cos(pitch/2) * cos(roll/2);
	q.x = sin(pitch/2) * cos(yaw - roll/2);
	q.y = sin(pitch/2) * sin(yaw - roll/2);
	q.z = cos(pitch/2) * sin(roll/2);
}
void Balljoint::CalcRelativeVel(real_t* vel, vec3_t& v, vec3_t& w){
	// T.B.D.
}
void Balljoint::CalcJointPos(real_t* pos, vec3_t& p, quat_t& q){
	// T.B.D.
}

void Balljoint::CalcJacobian(real_t* pos, vec3_t* Jv, vec3_t* Jw){
	Jv[0].clear();
	Jv[1].clear();
	Jv[2].clear();

	real_t yaw   = pos[0];
	real_t pitch = pos[1];

	// 行列の場合と行・列のインデックス順が逆なので注意
	Jw[0][0] = -sin(yaw) * sin(pitch);
	Jw[0][1] =  cos(yaw) * sin(pitch);
	Jw[0][2] =  1.0 - cos(pitch);
	Jw[1][0] =  cos(yaw);
	Jw[1][1] =  sin(yaw);
	Jw[1][2] =  0.0;
	Jw[2][0] =  sin(yaw) * sin(pitch);
	Jw[2][1] = -cos(yaw) * sin(pitch);
	Jw[2][2] =  cos(pitch);
}

//-------------------------------------------------------------------------------------------------

Planejoint::Planejoint(Connector* _sock, Connector* _plug, TimeSlot* _time, const string& n): Joint(_sock, _plug, _time, n){
	SetDof(3);
}

void Planejoint::CalcRelativePose(real_t* pos, vec3_t& p, quat_t& q){
	p = vec3_t(pos[0], pos[1], 0.0);
	q = quat_t::Rot(pos[2], 'z');
}
void Planejoint::CalcRelativeVel(real_t* vel, vec3_t& v, vec3_t& w){
	v = vec3_t(vel[0], vel[1], 0.0);
	w = vec3_t(0.0, 0.0, vel[2]);
}
void Planejoint::CalcJointPos(real_t* pos, vec3_t& p, quat_t& q){
	pos[0] = p.x;
	pos[1] = p.y;
	pos[2] = q.Theta();
}

void Planejoint::CalcJacobian(real_t* pos, vec3_t* Jv, vec3_t* Jw){
	Jv[0] = vec3_t(1.0, 0.0, 0.0);
	Jv[1] = vec3_t(0.0, 1.0, 0.0);
	Jv[2].clear();
	Jw[0].clear();
	Jw[1].clear();
	Jw[2] = vec3_t(0.0, 0.0, 1.0);
}

//-------------------------------------------------------------------------------------------------

Fixjoint::Fixjoint(Connector* _sock, Connector* _plug, TimeSlot* _time, const string& n): Joint(_sock, _plug, _time, n){
	SetDof(0);
}

void Fixjoint::CalcRelativePose(real_t* pos, vec3_t& p, quat_t& q){
	p.clear();
	q = quat_t();
}
void Fixjoint::CalcRelativeVel(real_t* vel, vec3_t& v, vec3_t& w){
	v.clear();
	w.clear();
}

//-------------------------------------------------------------------------------------------------

Genericjoint::Genericjoint(Connector* _sock, Connector* _plug, GenericjointCallback* cb, TimeSlot* _time, const string& n): Joint(_sock, _plug, _time, n){
	SetCallback(cb);
}

void Genericjoint::SetCallback(GenericjointCallback* cb){
	callback = cb;
	SetDof(callback->GetDof());
}

bool Genericjoint::IsRotational(uint i){
	if(callback)
		return callback->IsRotational(i);
	return false;
}

void Genericjoint::CalcRelativePose(real_t* pos, vec3_t& p, quat_t& q){
	if(callback)
		callback->CalcRelativePose(pos, p, q);
}
void Genericjoint::CalcRelativeVel(real_t* vel, vec3_t& v, vec3_t& w){
	if(callback)
		callback->CalcRelativeVel(vel, v, w);
}
void Genericjoint::CalcJointPos(real_t* pos, vec3_t& p, quat_t& q){
	if(callback)
		callback->CalcJointPos(pos, p, q);
}

void Genericjoint::CalcJacobian(real_t* pos, vec3_t* Jv, vec3_t* Jw){
	if(callback)
		callback->CalcJacobian(pos, Jv, Jw);
}

void Genericjoint::OnDraw(real_t* pos, Render::Canvas* canvas){
	if(callback)
		callback->OnDraw(pos, canvas);
}

//-------------------------------------------------------------------------------------------------
// constructors

JointCon::JointCon(Solver* solver, uint n, ID id, JointKey* _jnt, real_t _scale):Constraint(solver, n, id, Constraint::Type::Equality, _scale){
	jnt = _jnt;
}

JointConF::JointConF(Solver* solver, const string& _name, JointKey* _jnt, uint i, real_t _scale):
	JointCon(solver, 1, ID(ConTag::JointF, _jnt->node, _jnt->tick, _name), _jnt, _scale){
	idx = i;
	AddSLink (jnt->torque[i]);
	AddR3Link(jnt->force_t);
	AddR3Link(jnt->force_t);
}

JointConTP::JointConTP(Solver* solver, const string& _name, JointKey* _jnt, real_t _scale):
	JointCon(solver, 3, ID(ConTag::JointTP, _jnt->node, _jnt->tick, _name), _jnt, _scale){

	AddSLink (jnt->sockObj->pos_t,  1.0);
	AddSLink (jnt->plugObj->pos_t, -1.0);
	AddX3Link(jnt->sockObj->pos_r);
	AddX3Link(jnt->plugObj->pos_r);
	
	uint dof = ((Joint*)jnt->node)->dof;
	for(uint i = 0; i < dof; i++)
		AddC3Link(jnt->pos[i]);
}
JointConTV::JointConTV(Solver* solver, const string& _name, JointKey* _jnt, real_t _scale):
	JointCon(solver, 3, ID(ConTag::JointTV, _jnt->node, _jnt->tick, _name), _jnt, _scale){
	AddM3Link(jnt->sockObj->pos_r);
	AddM3Link(jnt->plugObj->pos_r);
	AddSLink (jnt->sockObj->vel_t,  1.0);
	AddSLink (jnt->plugObj->vel_t, -1.0);
	AddX3Link(jnt->sockObj->vel_r);
	AddX3Link(jnt->plugObj->vel_r);
	
	uint dof = ((Joint*)jnt->node)->dof;
	for(uint i = 0; i < dof; i++){
		AddC3Link(jnt->pos[i]);
		AddC3Link(jnt->vel[i]);
	}

	//transformable = true;
}
JointConRP::JointConRP(Solver* solver, const string& _name, JointKey* _jnt, real_t _scale):
	JointCon(solver, 3, ID(ConTag::JointRP, _jnt->node, _jnt->tick, _name), _jnt, _scale){	
	AddSLink(jnt->sockObj->pos_r,  1.0);
	AddSLink(jnt->plugObj->pos_r, -1.0);
	
	uint dof = ((Joint*)jnt->node)->dof;
	for(uint i = 0; i < dof; i++)
		AddC3Link(jnt->pos[i]);

	//transformable = true;
}
JointConRV::JointConRV(Solver* solver, const string& _name, JointKey* _jnt, real_t _scale):
	JointCon(solver, 3, ID(ConTag::JointRV, _jnt->node, _jnt->tick, _name), _jnt, _scale){	
	AddX3Link(jnt->sockObj->pos_r);
	AddSLink (jnt->sockObj->vel_r,  1.0);
	AddSLink (jnt->plugObj->vel_r, -1.0);
	
	uint dof = ((Joint*)jnt->node)->dof;
	for(uint i = 0; i < dof; i++)
		AddC3Link(jnt->vel[i]);

	//transformable = true;
}

//-------------------------------------------------------------------------------------------------
// CalcCoef

void JointConF::CalcCoef(){
	((R3Link*)links[1])->SetCoef(-jnt->Jv[idx]);
	((R3Link*)links[2])->SetCoef(-jnt->Jw[idx]);
}

void JointConTP::CalcCoef(){
	((X3Link*)links[2])->SetCoef(-(jnt->r[0] + jnt->q[0] * jnt->rrel));
	((X3Link*)links[3])->SetCoef(  jnt->r[1]);
	
	uint dof = ((Joint*)jnt->node)->dof;
	for(uint i = 0; i < dof; i++)
		((C3Link*)links[4+i])->SetCoef(jnt->Jv[i]);
}
void JointConTV::CalcCoef(){
	((M3Link*)links[0])->SetCoef(- mat3_t::Cross(jnt->sockObj->vel_r->val) * mat3_t::Cross(jnt->r[0] + jnt->q[0] * jnt->rrel) - mat3_t::Cross(jnt->vrel));
	((M3Link*)links[1])->SetCoef(  mat3_t::Cross(jnt->plugObj->vel_r->val) * mat3_t::Cross(jnt->r[1]));
	((X3Link*)links[4])->SetCoef(-(jnt->r[0] + jnt->q[0] * jnt->rrel));
	((X3Link*)links[5])->SetCoef(  jnt->r[1]);
	
	uint dof = ((Joint*)jnt->node)->dof;
	for(uint i = 0; i < dof; i++){
		((C3Link*)links[6+2*i+0])->SetCoef(jnt->sockObj->vel_r->val % jnt->Jv[i]);
		((C3Link*)links[6+2*i+1])->SetCoef(jnt->Jv[i]);
	}
}
void JointConRP::CalcCoef(){
	uint dof = ((Joint*)jnt->node)->dof;
	for(uint i = 0; i < dof; i++)
		((C3Link*)links[2+i])->SetCoef(jnt->Jw[i]);
}
void JointConRV::CalcCoef(){
	((X3Link*)links[0])->SetCoef(- jnt->wrel);
	
	uint dof = ((Joint*)jnt->node)->dof;
	for(uint i = 0; i < dof; i++)
		((C3Link*)links[3+i])->SetCoef(jnt->Jw[i]);
}

//-------------------------------------------------------------------------------------------------
// CalcDeviation
void JointConTP::CalcDeviation(){
	y = (jnt->sockObj->pos_t->val + jnt->r[0] + jnt->q[0] * jnt->rrel)
	  - (jnt->plugObj->pos_t->val + jnt->r[1]);
}
void JointConTV::CalcDeviation(){
	y = (jnt->sockObj->vel_t->val + jnt->sockObj->vel_r->val % (jnt->r[0] + jnt->q[0] * jnt->rrel) + jnt->vrel)
	  - (jnt->plugObj->vel_t->val + jnt->plugObj->vel_r->val %  jnt->r[1]);
}
void JointConRP::CalcDeviation(){
	quat_t q = jnt->q[0] * jnt->qrel;
	quat_t qerror = jnt->q[1].Conjugated() * q;
	vec3_t axis   = qerror.Axis ();
	real_t theta  = qerror.Theta();
	if(theta > pi)
		theta -= 2*pi;
	y = jnt->q[1] * (theta * axis);
}
void JointConRV::CalcDeviation(){
	y = (jnt->sockObj->vel_r->val + jnt->wrel)
	  - (jnt->plugObj->vel_r->val);
}

}
