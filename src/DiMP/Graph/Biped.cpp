#include <DiMP/Graph/Biped.h>
#include <DiMP/Graph/Graph.h>
#include <DiMP/Render/Config.h>
#include <DiMP/Render/Canvas.h>

#include<stdio.h>

namespace DiMP {
	;

	const real_t pi = M_PI;

	//-------------------------------------------------------------------------------------------------
	// BipedLIPKey

	BipedLIPKey::BipedLIPKey() {
		
	}

	//変数を追加する関数
	void BipedLIPKey::AddVar(Solver* solver) {
		BipedLIP* obj = (BipedLIP*)node;

		var_torso_pos_t = new V2Var(solver, ID(VarTag::BipedTorsoTP, node, tick, name + "_torso_tp"), node->graph->scale.pos_t);
		var_torso_pos_r = new SVar (solver, ID(VarTag::BipedTorsoRP, node, tick, name + "_torso_rp"), node->graph->scale.pos_r);
		var_torso_vel_t = new V2Var(solver, ID(VarTag::BipedTorsoTV, node, tick, name + "_torso_tv"), node->graph->scale.vel_t);
		var_torso_vel_r = new SVar (solver, ID(VarTag::BipedTorsoRV, node, tick, name + "_torso_rv"), node->graph->scale.vel_r);
		
		var_foot_pos_t[0] = new V2Var(solver, ID(VarTag::BipedFootT, node, tick, name + "_foot_r_t"), node->graph->scale.pos_t);
		var_foot_pos_r[0] = new SVar (solver, ID(VarTag::BipedFootR, node, tick, name + "_foot_r_r"), node->graph->scale.pos_r);
		var_foot_pos_t[1] = new V2Var(solver, ID(VarTag::BipedFootT, node, tick, name + "_foot_l_t"), node->graph->scale.pos_t);
		var_foot_pos_r[1] = new SVar (solver, ID(VarTag::BipedFootR, node, tick, name + "_foot_l_r"), node->graph->scale.pos_r);

		var_com_pos = new V2Var(solver, ID(VarTag::BipedComP, node, tick, name + "_com_p"), node->graph->scale.pos_t);
		var_com_vel = new V2Var(solver, ID(VarTag::BipedComV, node, tick, name + "_com_v"), node->graph->scale.vel_t);
		
		var_cop_pos = new V2Var(solver, ID(VarTag::BipedCop, node, tick, name + "_cop"), node->graph->scale.pos_t);
		
		var_duration = new SVar(solver, ID(VarTag::BipedDuration, node, tick, name + "_duration"), node->graph->scale.time);
		var_time     = new SVar(solver, ID(VarTag::BipedTime    , node, tick, name + "_time"    ), node->graph->scale.time);
	}

	//拘束条件を追加する関数
	void BipedLIPKey::AddCon(Solver* solver) {
		BipedLIP* obj = (BipedLIP*)node;
		BipedLIPKey* nextObj = (BipedLIPKey*)next;

		if (next) {
			con_lip_p = new BipedLipConP(solver, name + "_lip_p", this, node->graph->scale.pos_t);
			con_lip_v = new BipedLipConV(solver, name + "_lip_v", this, node->graph->scale.vel_t);
		}

		con_foot_t[0] = new BipedFootConT(solver, name + "_foot_range_r_t", this, 0, node->graph->scale.pos_t);
		con_foot_r[0] = new BipedFootConR(solver, name + "_foot_range_r_r", this, 0, node->graph->scale.pos_r);
		con_foot_t[1] = new BipedFootConT(solver, name + "_foot_range_l_t", this, 1, node->graph->scale.pos_t);
		con_foot_r[1] = new BipedFootConR(solver, name + "_foot_range_l_r", this, 1, node->graph->scale.pos_r);
			
		int phase = obj->phase[tick->idx];
		int side;
		if(phase == BipedLIP::Phase::R || phase == BipedLIP::Phase::RL)
			 side = 0;
		else side = 1;
		con_cop = new BipedCopCon(solver, name + "_cop", this, side, node->graph->scale.pos_t);
			
		if(next){
			con_duration = new RangeConS   (solver, ID(ConTag::BipedDuration, node, tick, name + "_duration"), var_duration, node->graph->scale.time);
			con_time     = new BipedTimeCon(solver, name + "_time", this, node->graph->scale.time);
		}

		if(next){
			con_foot_match_t[0] = new MatchConV2(solver, ID(ConTag::BipedFootMatchT, node, tick, name + "_foot_match_r_t"), var_foot_pos_t[0], nextObj->var_foot_pos_t[0], node->graph->scale.pos_t);
			con_foot_match_r[0] = new MatchConS (solver, ID(ConTag::BipedFootMatchR, node, tick, name + "_foot_match_r_r"), var_foot_pos_r[0], nextObj->var_foot_pos_r[0], node->graph->scale.pos_r);
			con_foot_match_t[1] = new MatchConV2(solver, ID(ConTag::BipedFootMatchT, node, tick, name + "_foot_match_l_t"), var_foot_pos_t[1], nextObj->var_foot_pos_t[1], node->graph->scale.pos_t);
			con_foot_match_r[1] = new MatchConS (solver, ID(ConTag::BipedFootMatchR, node, tick, name + "_foot_match_l_r"), var_foot_pos_r[1], nextObj->var_foot_pos_r[1], node->graph->scale.pos_r);
		}
		
		con_com_p = new BipedComConP(solver, name + "_com_p", this, node->graph->scale.pos_t);
		con_com_v = new BipedComConV(solver, name + "_com_v", this, node->graph->scale.vel_t);
	}

	void BipedLIPKey::Prepare() {
		BipedLIP* obj = (BipedLIP*)node;

		// 歩行周期変数の値を時刻に反映
		tick->time = var_time->val;

		if(!prev){
			// 初期時刻を固定
			var_time->val    = 0.0;
			var_time->locked = true;
		}

		if(next){
			int phase = obj->phase[tick->idx];
			con_foot_match_t[0]->enabled = (phase != BipedLIP::Phase::L);
			con_foot_match_r[0]->enabled = (phase != BipedLIP::Phase::L);
			con_foot_match_t[1]->enabled = (phase != BipedLIP::Phase::R);
			con_foot_match_r[1]->enabled = (phase != BipedLIP::Phase::R);
		}
	}

	void BipedLIPKey::Draw(Render::Canvas* canvas, Render::Config* conf) {
		BipedLIP* obj = (BipedLIP*)node;

		Vec3f pcom, pcop, pf[2], pt;
		float thetaf[2];

		canvas->SetPointSize(5.0f);
		canvas->SetLineWidth(1.0f);

		// com
		pcom.x   = (float)var_com_pos->val.x;
		pcom.y   = (float)var_com_pos->val.y;
		pcom.z   = (float)((BipedLIP*)node)->param.heightCoM;
		canvas->Point(pcom);

		// feet
		Vec2f cmin = obj->param.copPosMin;
		Vec2f cmax = obj->param.copPosMax;

		for(uint i = 0; i < 2; i++){
			pf[i].x   = (float)var_foot_pos_t[i]->val.x;
			pf[i].y   = (float)var_foot_pos_t[i]->val.y;
			pf[i].z   = 0.0f;
			canvas->Point(pf[i]);

			// foot print
			canvas->Push();
			thetaf[i] = (float)var_foot_pos_r[i]->val;
			canvas->Translate(pf[i].x, pf[i].y, pf[i].z);
			canvas->Rotate(thetaf[i], Vec3f(0.0f, 0.0f, 1.0f));
			canvas->Line(Vec3f(cmax.x, cmax.y, 0.0f), Vec3f(cmin.x, cmax.y, 0.0f));
			canvas->Line(Vec3f(cmin.x, cmax.y, 0.0f), Vec3f(cmin.x, cmin.y, 0.0f));
			canvas->Line(Vec3f(cmin.x, cmin.y, 0.0f), Vec3f(cmax.x, cmin.y, 0.0f));
			canvas->Line(Vec3f(cmax.x, cmin.y, 0.0f), Vec3f(cmax.x, cmax.y, 0.0f));
			canvas->Pop();
		}

		// torso
		pt = obj->TorsoPos(pcom, pf[0], pf[1]);
		canvas->Point(pt);

		// lines connecting torso and feet
		canvas->Line(pt, pf[0]);
		canvas->Line(pt, pf[1]);

		// cop
		pcop.x = (float)var_cop_pos->val.x;
		pcop.y = (float)var_cop_pos->val.y;
		pcop.z = 0.0f;
		canvas->Point(pcop);
		
	}

	//-------------------------------------------------------------------------------------------------
	// BipedLIP

	BipedLIP::Param::Param() {
		gravity        = 9.8 ;  //重力加速度
		heightCoM      = 0.55;  //重心高さ
		torsoMass      = 5.0 ;  //胴体質量
		footMass       = 0.5 ;  //足質量

		swingProfile   = SwingProfile::Cycloid;  //遊脚軌道の種類
		swingHeight[0] = 0.04;                   ///< 0: 遊脚の最大高さ
		swingHeight[1] = 0.01;                   ///< 1: 接地前の最小高さ for wedge only
		durationMin[Phase::R ] = 0.1;
		durationMax[Phase::R ] = 0.5;
		durationMin[Phase::L ] = 0.1;
		durationMax[Phase::L ] = 0.5;
		durationMin[Phase::RL] = 0.1;
		durationMax[Phase::RL] = 0.5;
		durationMin[Phase::LR] = 0.1;
		durationMax[Phase::LR] = 0.5;

		//実現可能なCoMとCoPの相対距離
		footPosMin[0] = vec2_t(-0.3, -0.2 );
		footPosMax[0] = vec2_t( 0.3, -0.05);
		footPosMin[1] = vec2_t(-0.3,  0.05);
		footPosMax[1] = vec2_t( 0.3,  0.2 );
		footOriMin[0] = -Rad(30.0);
		footOriMax[0] =  Rad(30.0);
		footOriMin[1] = -Rad(30.0);
		footOriMax[1] =  Rad(30.0);

		copPosMin = vec2_t(-0.1, -0.05);
		copPosMax = vec2_t( 0.1,  0.05);

		angAccMax = 0.3;
		turnMax   = 0.5;
	}

	//-------------------------------------------------------------------------------------------------
	/// 経由点
	//BipedLIPクラスの構造体Waypointのメンバ関数Waypoint()
	BipedLIP::Waypoint::Waypoint() {
		k             = 0;
		time          = 0.0;
		torso_pos_t   = vec2_t();
		torso_pos_r   = 0.0;
		torso_vel_t   = vec2_t();
		torso_vel_r   = 0.0;
		foot_pos_t[0] = vec2_t();
		foot_pos_r[0] = 0.0;
		foot_pos_t[1] = vec2_t();
		foot_pos_r[1] = 0.0;

		fix_torso_pos_t   = false;
		fix_torso_pos_r   = false;
		fix_torso_vel_t   = false;
		fix_torso_vel_r   = false;
		fix_foot_pos_t[0] = false;
		fix_foot_pos_r[0] = false;
		fix_foot_pos_t[1] = false;
		fix_foot_pos_r[1] = false;
	}

	//-------------------------------------------------------------------------------------------------
	//BipedLIPクラスの構造体TrajPointのメンバ関数Trajpoint()
	BipedLIP::TrajPoint::TrajPoint() {
		t             = 0.0;
		torso_pos_t   = vec3_t();
		torso_pos_r   = 0.0;
		foot_pos_t[0] = vec3_t();
		foot_pos_r[0] = quat_t();
		foot_pos_t[1] = vec3_t();
		foot_pos_r[1] = quat_t();
	}

	//-------------------------------------------------------------------------------------------------
	
	BipedLIP::BipedLIP(Graph* g, string n) :TrajectoryNode(g, n) {
		type = Type::Object;
		graph->bipeds.Add(this);
	}

	BipedLIP::~BipedLIP() {
		graph->bipeds.Remove(this);
	}

	void BipedLIP::Init() {
		TrajectoryNode::Init();

		param.T = sqrt(param.heightCoM / param.gravity);//時定数T=√h/g

		real_t durationAve[Phase::Num];
		for(int i = 0; i < Phase::Num; i++)
			durationAve[i] = (param.durationMin[i] + param.durationMax[i]) / 2.0;

		real_t t = 0.0;
		for (uint k = 0; k < graph->ticks.size(); k++) {
			BipedLIPKey* key = (BipedLIPKey*)traj.GetKeypoint(graph->ticks[k]);

			key->var_time->val = t;

			// 周期の初期値は下限と上限の中間値
			if(key->next){
				key->var_duration->val  = durationAve[phase[k]];
				key->con_duration->_min = param.durationMin[phase[k]];
				key->con_duration->_max = param.durationMax[phase[k]];
			}

			/*
			//ステップ始点と終点の角加速度制限
			key->con_com_r[0]->_min = -param.angAccMax;
			key->con_com_r[0]->_max =  param.angAccMax;
			key->con_com_r[1]->_min = -param.angAccMax;
			key->con_com_r[1]->_max =  param.angAccMax;
			*/

			//実現可能なCoMとCoPの相対距離
			for (int i = 0; i < 2; i++) {
				key->con_foot_t[i]->_min = param.footPosMin[i];
				key->con_foot_t[i]->_max = param.footPosMax[i];
				key->con_foot_r[i]->_min = param.footOriMin[i];
				key->con_foot_r[i]->_max = param.footOriMax[i];
			}

			key->con_cop->_min = param.copPosMin;
			key->con_cop->_max = param.copPosMax;

			t += durationAve[phase[k]];
		}

		// 重心位置，重心速度，着地位置の初期値を経由点を結ぶスプライン曲線(滑らかな補間曲線)で与える
		Curve2d curve_torso_t, curve_foot_t[2];
		Curved  curve_torso_r, curve_foot_r[2];
		
		curve_torso_t  .SetType(Interpolate::Cubic);
		curve_torso_r  .SetType(Interpolate::Cubic);
		curve_foot_t[0].SetType(Interpolate::Cubic);
		curve_foot_r[0].SetType(Interpolate::Cubic);
		curve_foot_t[1].SetType(Interpolate::Cubic);
		curve_foot_r[1].SetType(Interpolate::Cubic);
		
		for(uint i = 0; i < waypoints.size(); i++) {
			Waypoint& wp = waypoints[i];
			BipedLIPKey* key = (BipedLIPKey*)traj.GetKeypoint(graph->ticks[wp.k]);
			real_t t = key->var_time->val;

			curve_torso_t.AddPoint(t);
			curve_torso_t.SetPos(i, wp.torso_pos_t);
			curve_torso_t.SetVel(i, wp.torso_vel_t);

			curve_torso_r.AddPoint(t);
			curve_torso_r.SetPos(i, wp.torso_pos_r);
			curve_torso_r.SetVel(i, wp.torso_vel_r);

			for(uint j = 0; j < 2; j++){
				curve_foot_t[j].AddPoint(t);
				curve_foot_t[j].SetPos(i, wp.foot_pos_t[j]);
				curve_foot_t[j].SetVel(i, vec2_t());

				curve_foot_r[j].AddPoint(t);
				curve_foot_r[j].SetPos(i, wp.foot_pos_r[j]);
				curve_foot_r[j].SetVel(i, 0.0);
			}
		}

		for (uint k = 0; k < graph->ticks.size(); k++) {
			BipedLIPKey* key = (BipedLIPKey*)traj.GetKeypoint(graph->ticks[k]);
			real_t t = key->var_time->val;

			key->var_torso_pos_t->val = curve_torso_t.CalcPos(t);
			key->var_torso_pos_r->val = curve_torso_r.CalcPos(t);
		
			key->var_torso_vel_t->val = curve_torso_t.CalcVel(t);
			key->var_torso_vel_r->val = curve_torso_r.CalcVel(t);

			for(uint j = 0; j < 2; j++){
				key->var_foot_pos_t[j]->val = curve_foot_t[j].CalcPos(t);
				key->var_foot_pos_r[j]->val = curve_foot_r[j].CalcPos(t);
			}
			
			real_t mt = param.torsoMass;
			real_t mf = param.footMass;
			key->var_com_pos->val = (mt * key->var_torso_pos_t->val + mf * (key->var_foot_pos_t[0]->val + key->var_foot_pos_t[1]->val)) / (mt + 2.0*mf);
			key->var_com_vel->val = (mt * key->var_torso_vel_t->val) / (mt + 2.0*mf);
				
			if(phase[k] == Phase::R || phase[k] == Phase::RL)
				 key->var_cop_pos->val = key->var_foot_pos_t[0]->val;
			else key->var_cop_pos->val = key->var_foot_pos_t[1]->val;
		}

		// 経由点上の変数を固定
		for (uint i = 0; i < waypoints.size(); i++) {
			Waypoint& wp = waypoints[i];
			BipedLIPKey* key = (BipedLIPKey*)traj.GetKeypoint(graph->ticks[wp.k]);

			key->var_torso_pos_t->locked = wp.fix_torso_pos_t;
			key->var_torso_pos_r->locked = wp.fix_torso_pos_r;
			key->var_torso_vel_t->locked = wp.fix_torso_vel_t;
			key->var_torso_vel_r->locked = wp.fix_torso_vel_r;
			
			for(uint j = 0; j < 2; j++){
				key->var_foot_pos_t[j]->locked = wp.fix_foot_pos_t[j];
				key->var_foot_pos_r[j]->locked = wp.fix_foot_pos_r[j];
			}
		}
	}

	void BipedLIP::Prepare() {
		TrajectoryNode::Prepare();
		trajReady = false;
	}

	int BipedLIP::Phase(real_t t) {
		BipedLIPKey* key = (BipedLIPKey*)traj.GetSegment(t).first;
		return phase[key->tick->idx];
	}

	//重心位置
	vec3_t BipedLIP::ComPos(real_t t) {
		BipedLIPKey* key0 = (BipedLIPKey*)traj.GetSegment(t).first ;
		BipedLIPKey* key1 = (BipedLIPKey*)traj.GetSegment(t).second;

		vec2_t pt;

		if(key1){
			real_t T   = param.T;
			real_t dt  = t - key0->var_time->val;
			real_t tau = key0->var_duration->val;
			vec2_t p0  = key0->var_com_pos->val;
			vec2_t v0  = key0->var_com_vel->val;
			vec2_t c0  = key0->var_cop_pos->val;
			vec2_t c1  = key1->var_cop_pos->val;
			
			pt = c0 + (c1 - c0)*(dt/tau) + (p0 - c0)*cosh(dt/T) + T*(v0 - (c1 - c0)/tau)*sinh(dt/T);
		}
		else{
			pt = key0->var_com_pos->val;
		}

		return vec3_t(pt.x, pt.y, param.heightCoM);
	}

	//重心速度
	vec3_t BipedLIP::ComVel(real_t t) {
		BipedLIPKey* key0 = (BipedLIPKey*)traj.GetSegment(t).first ;
		BipedLIPKey* key1 = (BipedLIPKey*)traj.GetSegment(t).second;

		vec2_t vt;

		if(key1){
			real_t T   = param.T;
			real_t dt  = t - key0->var_time->val;
			real_t tau = key0->var_duration->val;
			vec2_t p0  = key0->var_com_pos->val;
			vec2_t v0  = key0->var_com_vel->val;
			vec2_t c0  = key0->var_cop_pos->val;
			vec2_t c1  = key1->var_cop_pos->val;
			
			vt = (c1 - c0)/tau + (1/T)*(p0 - c0)*sinh(dt/T) + (v0 - (c1 - c0)/tau)*cosh(dt/T);
		}
		else{
			vt = key0->var_com_vel->val;
		}
		
		return vec3_t(vt.x, vt.y, 0.0);
	}

	//重心加速度
	vec3_t BipedLIP::ComAcc(real_t t) {
		BipedLIPKey* key0 = (BipedLIPKey*)traj.GetSegment(t).first;
		BipedLIPKey* key1 = (BipedLIPKey*)traj.GetSegment(t).second;

		vec2_t at;

		if(key1){
			real_t T   = param.T;
			real_t T2  = T*T;
			real_t dt  = t - key0->var_time->val;
			real_t tau = key0->var_duration->val;
			vec2_t p0  = key0->var_com_pos->val;
			vec2_t v0  = key0->var_com_vel->val;
			vec2_t c0  = key0->var_cop_pos->val;
			vec2_t c1  = key1->var_cop_pos->val;
					
			vec2_t at = (1/T2)*(p0 - c0)*cosh(dt/T) + (1/T)*(v0 - (c1 - c0)/tau)*sinh(dt/T);
		}
		else{
			at.clear();
		}
		
		return vec3_t(at.x, at.y, 0.0);
	}

	real_t BipedLIP::TorsoOri(real_t t) {
		BipedLIPKey* key0 = (BipedLIPKey*)traj.GetSegment(t).first;
		BipedLIPKey* key1 = (BipedLIPKey*)traj.GetSegment(t).second;

		real_t o;

		if (key1) {
			real_t t0   = key0->var_time->val;
			real_t t1   = key1->var_time->val;
			
			o = InterpolatePos(
				t,
				t0,
				key0->var_torso_pos_r->val,
				key0->var_torso_vel_r->val,
				t1,
				key1->var_torso_pos_r->val,
				key1->var_torso_vel_r->val,
				Interpolate::Cubic);
		}
		else {
			o = key0->var_torso_pos_r->val;
		}

		return o;
	}

	real_t BipedLIP::TorsoAngVel(real_t t) {
		BipedLIPKey* key0 = (BipedLIPKey*)traj.GetSegment(t).first;
		BipedLIPKey* key1 = (BipedLIPKey*)traj.GetSegment(t).second;

		real_t r;

		if (key1) {
			real_t t0 = key0->var_time->val;
			real_t t1 = key1->var_time->val;
			
			r = InterpolateVel(
				t,
				t0,
				key0->var_torso_pos_r->val,
				key0->var_torso_vel_r->val,
				t1,
				key1->var_torso_pos_r->val,
				key1->var_torso_vel_r->val,
				Interpolate::Cubic);
		}
		else {
			r = key0->var_torso_vel_r->val;
		}

		return r;
	}

	real_t BipedLIP::TorsoAngAcc(real_t t) {
		BipedLIPKey* key0 = (BipedLIPKey*)traj.GetSegment(t).first;
		BipedLIPKey* key1 = (BipedLIPKey*)traj.GetSegment(t).second;

		real_t a;

		if (key1) {
			real_t t0 = key0->var_time->val;
			real_t t1 = key1->var_time->val;
			
			a = InterpolateAcc(
				t,
				t0,
				key0->var_torso_pos_r->val,
				key0->var_torso_vel_r->val,
				t1,
				key1->var_torso_pos_r->val,
				key1->var_torso_vel_r->val,
				Interpolate::Cubic);
		}
		else {
			a = 0.0;
		}

		return a;
	}

	vec3_t BipedLIP::FootPos(real_t t, int side) {
		BipedLIPKey* key0 = (BipedLIPKey*)traj.GetSegment(t).first;
		BipedLIPKey* key1 = (BipedLIPKey*)traj.GetSegment(t).second;

		vec2_t pt;
		real_t z = 0.0;

		if(key1){
			real_t dt  = t - key0->var_time->val;
			real_t tau = key0->var_duration->val;
			vec2_t p0  = key0->var_foot_pos_t[side]->val;
			vec2_t p1  = key1->var_foot_pos_t[side]->val;
			
			pt = p0 + (p1 - p0)*(dt/tau);

			// 単脚支持期の遊脚
			int ph = phase[key0->tick->idx];
			if( (ph == Phase::R && side == 1) ||
				(ph == Phase::L && side == 0) ){

			}
		}
		else{
			pt = key0->var_foot_pos_t[side]->val;
		}

		return vec3_t(pt.x, pt.y, z);
	}

	/*
	vec3_t BipedLIP::SwgFootPos(real_t t) {
		KeyPair      kp  = traj.GetSegment(t);
		BipedLIPKey* cur  = (BipedLIPKey*)kp.first;
		BipedLIPKey* next = (BipedLIPKey*)kp.second;
		BipedLIPKey* prev = (BipedLIPKey*)cur->prev;

		real_t       t0 = cur ->tick->time;
		real_t       t1 = next->tick->time;
		real_t       h = t1 - t0;
		real_t       hhalf = h / 2.0;

		real_t		 ta = t0 + 0.05*h;//踵が上がりきる時刻
		real_t		 tb = t0 + 0.95*h;//踵が接地する時刻

		real_t _2pi = 2.0 * M_PI;//2π
		real_t tau = (t - t0) / (ta - t0);//phase1
		real_t tau2 = (t - ta) / (tb - ta);//phase2
		real_t tau3 = (t1 - t) / (t1 - tb);//phase3

		real_t		 theta = (M_PI) / 12;//接地・離地時の角度
		real_t		 l = 0.1;//足の中心から踵・つま先までの距離
		real_t		 pa = l*(1 - cos(theta));//t0<t<taのx方向の距離
		real_t		 pb = l*(1 - cos(theta));//tb<t<t1のx方向の距離


											 // 遊脚の始点と終点は前後の接地点
											 // 0歩目の始点は0歩目の支持足の反対側に設定
											 // N-1歩目の終点はN-1歩目の支持足の反対側に設定
		vec2_t p0, p1;
		// 1～N-1歩目
		if (prev) {
			p0 = prev->var_foot_pos_t->val;//遊脚の始点の位置
		}
		// 0歩目
		else {
			p0 = cur->swg_pos_t->val;
		}

		if (next) {
			p1 = next->sup_pos_t->val;//遊脚の終点の位置
		}
		else {
			p1 = prev->sup_pos_t->val;
		}

		real_t s;
		real_t z;
		if (h == 0.0) {//境目
			s = 0.0;
			z = 0.0;
		}
		else {
			if (param.swingProfile == SwingProfile::Wedge) {
				if (t < t0 + hhalf) {
					z = param.swingHeight[0];
				}
				else {
					real_t a = (t - (t0 + hhalf)) / hhalf;
					z = (1 - a)*param.swingHeight[0] + a*param.swingHeight[1];
				}

				s = (t - t0) / h;//周期における時刻tの割合
			}

			if (param.swingProfile == SwingProfile::Cycloid) {//踵・つま先の遊脚軌道
				if (t < ta) {
					z = l*sin(theta*tau);
				}
				else if (ta <= t && t <= tb) {
					s = (tau2 - sin(_2pi*tau2) / _2pi);
					z = l*sin(theta) + (param.swingHeight[0] / 2.0) * (1 - cos(_2pi*tau2));
				}
				else {
					z = l*sin(theta*tau3);
				}

			}
		}

		vec3_t p;
		if (t < ta) {
			p[0] = p0[0] + l*(1 - cos(theta*tau));
			p[1] = p0[1];
		}
		else if (ta <= t && t <= tb) {
			p[0] = (1 - s) * (p0[0] + pa) + s * (p1[0] - pb);
			p[1] = (1 - s) * p0[1] + s * p1[1];
		}
		else {
			p[0] = p1[0] - l*(1 - cos(theta*tau3));
			p[1] = p1[1];
		}
		p[2] = z;
		return p;
	}
	*/
	/*
	real_t BipedLIP::SwgFootOri(real_t t) {
		KeyPair      kp = traj.GetSegment(t);
		BipedLIPKey* cur = (BipedLIPKey*)kp.first;
		BipedLIPKey* next = (BipedLIPKey*)kp.second;
		BipedLIPKey* prev = (BipedLIPKey*)cur->prev;
		real_t       t0 = cur->tick->time;
		real_t       t1 = next->tick->time;
		real_t       h = t1 - t0;

		real_t  q0, q1;
		if (prev)
			 q0 = prev->sup_pos_r->val;
		else q0 = cur ->swg_pos_r->val;

		if (next)
			 q1 = next->sup_pos_r->val;
		else q1 = prev->sup_pos_r->val;

		real_t s;
		if (h == 0.0)
			s = 0.0;
		else s = (t - t0) / h;

		real_t q = (1 - s) * q0 + s * q1;

		return q;
	}
	*/

	quat_t BipedLIP::FootOri(real_t t, int side) {
		BipedLIPKey* key0 = (BipedLIPKey*)traj.GetSegment(t).first;
		BipedLIPKey* key1 = (BipedLIPKey*)traj.GetSegment(t).second;

		real_t ot;
		quat_t qt;

		if(key1){
			real_t dt  = t - key0->var_time->val;
			real_t tau = key0->var_duration->val;
			real_t o0  = key0->var_foot_pos_r[side]->val;
			real_t o1  = key1->var_foot_pos_r[side]->val;
			
			ot = o0 + (o1 - o0)*(dt/tau);
		}
		else{
			ot = key0->var_foot_pos_r[side]->val;
		}

		qt = quat_t::Rot(ot, 'z');

		return qt;
	}

	vec3_t BipedLIP::CopPos(real_t t) {
		BipedLIPKey* key0 = (BipedLIPKey*)traj.GetSegment(t).first;
		BipedLIPKey* key1 = (BipedLIPKey*)traj.GetSegment(t).second;

		vec2_t ct;

		if(key1){
			real_t dt  = t - key0->var_time->val;
			real_t tau = key0->var_duration->val;
			vec2_t c0  = key0->var_cop_pos->val;
			vec2_t c1  = key1->var_cop_pos->val;
			
			ct = c0 + (c1 - c0)*(dt/tau);
		}
		else{
			ct = key0->var_cop_pos->val;
		}

		return vec3_t(ct.x, ct.y, 0.0);
	}

	vec3_t BipedLIP::TorsoPos(const vec3_t& pcom, const vec3_t& psup, const vec3_t& pswg) {
		// コンパスモデルより胴体の位置を求める
		real_t mt = param.torsoMass;
		real_t mf = param.footMass;
		vec3_t pt = ((mt + 2.0*mf)*pcom - mf*(psup + pswg))/mt;
		return pt;
	}

	//------------------------------------------------------------------------------------------------

	void BipedLIP::CalcTrajectory() {
		real_t tf = traj.back()->tick->time;
		real_t dt = 0.01;

		trajectory.clear();
		for (real_t t = 0.0; t <= tf; t += dt) {
			TrajPoint tp;
			tp.t             = t;
			tp.com_pos       = ComPos  (t);
			tp.foot_pos_t[0] = FootPos (t, 0);
			tp.foot_pos_r[0] = FootOri (t, 0);
			tp.foot_pos_t[1] = FootPos (t, 1);
			tp.foot_pos_r[1] = FootOri (t, 1);
			tp.torso_pos_t   = TorsoPos(tp.com_pos, tp.foot_pos_t[0], tp.foot_pos_t[1]);
			tp.torso_pos_r   = TorsoOri(t);
			tp.cop_pos       = CopPos  (t);
			
			trajectory.push_back(tp);
		}

		trajReady = true;
	}

	//------------------------------------------------------------------------------------------------

	void BipedLIP::Draw(Render::Canvas* canvas, Render::Config* conf) {
		TrajectoryNode::Draw(canvas, conf);

		if (!trajReady)
			CalcTrajectory();

		if (trajectory.empty())
			return;

		// com
		if (conf->Set(canvas, Render::Item::BipedCom, this)) {
			canvas->BeginLayer("biped_com", true);
			canvas->SetLineWidth(3.0f);
			canvas->SetLineColor("black");
			canvas->BeginPath();
			canvas->MoveTo(trajectory[0].com_pos);
			for (uint i = 1; i < trajectory.size(); i++) {
				canvas->LineTo(trajectory[i].com_pos);
			}
			canvas->EndPath();
			canvas->EndLayer();
		}

		// cop
		if (conf->Set(canvas, Render::Item::BipedCop, this)) {
			canvas->BeginLayer("biped_cop", true);
			canvas->SetLineWidth(3.0f);
			canvas->SetLineColor("magenta");
			canvas->BeginPath();
			canvas->MoveTo(trajectory[0].cop_pos);
			for (uint i = 1; i < trajectory.size(); i++) {
				canvas->LineTo(trajectory[i].cop_pos);
			}
			canvas->EndPath();
			canvas->EndLayer();
		}

		// torso
		if (conf->Set(canvas, Render::Item::BipedTorso, this)) {
			canvas->BeginLayer("biped_torso", true);
			canvas->SetLineWidth(2.0f);
			canvas->SetLineColor("blue");
			canvas->BeginPath();
			canvas->MoveTo(trajectory[0].torso_pos_t);
			for (uint i = 1; i < trajectory.size(); i++) {
				canvas->LineTo(trajectory[i].torso_pos_t);
			}
			canvas->EndPath();
			canvas->EndLayer();
		}

		/*
		// swing foot
		if (conf->Set(canvas, Render::Item::BipedFoot, this)) {
			canvas->BeginLayer("biped_foot", true);
			canvas->SetLineWidth(1.0f);
			canvas->BeginPath();
			canvas->MoveTo(trajectory[0].foot_pos_t[0]);
			for (uint i = 1; i < trajectory.size(); i++) {
				canvas->LineTo(trajectory[i].foot_pos_t[0]);
			}
			canvas->MoveTo(trajectory[0].foot_pos_t[1]);
			for (uint i = 1; i < trajectory.size(); i++) {
				canvas->LineTo(trajectory[i].foot_pos_t[1]);
			}
			canvas->EndPath();
			canvas->EndLayer();
		}
		*/
		// double support snapshot
		/*		if (conf->Set(canvas, Render::Item::BipedDouble, this)) {
		canvas->BeginLayer("biped_double", true);
		canvas->SetLineWidth(1.0f);
		Vec3f p0, p1;
		for (uint i = 1; i < trajectory.size(); i++) {
		if (trajectory[i - 1].pos_sup != trajectory[i].pos_sup) {
		p0 = trajectory[i].pos_torso;
		p1 = trajectory[i].pos_sup;
		canvas->Line(p0, p1);

		p0 = trajectory[i].pos_torso;
		p1 = trajectory[i - 1].pos_sup;
		canvas->Line(p0, p1);
		}
		}
		canvas->EndLayer();
		}*/

	}

	void BipedLIP::DrawSnapshot(real_t time, Render::Canvas* canvas, Render::Config* conf) {
		/*
		canvas->SetLineWidth(2.0f);
		canvas->BeginPath();
		canvas->MoveTo(CoMPos(time));
		canvas->LineTo(SupFootPos(time));
		canvas->MoveTo(CoMPos(time));
		canvas->LineTo(SwgFootPos(time));
		canvas->EndPath();
		*/
	}

	/*
	void BipedLIP::Save() {
		FILE* file1 = fopen("plan_step_case1(t_dt).csv", "w");
		fprintf(file1, "time, pos_com_x, pos_com_y, vel_com_x, vel_com_y,acc_com_x,acc_com_y\n");

		real_t dt = 0.01;
		real_t tf = traj.back()->tick->time;

		vec3_t pf = CoMPos(tf);
		vec3_t vf = CoMVel(tf);
		vec3_t af = CoMAcc(tf);

		for (real_t t = 0.0; t < tf; t += dt) {

			vec3_t p = CoMPos(t);
			vec3_t v = CoMVel(t);
			vec3_t a = CoMAcc(t);

			fprintf(file1, "%3.3lf, %3.3lf, %3.3lf,%3.3lf,%3.3lf,%3.3lf, %3.3lf\n", t, p.x, p.y, v.x, v.y, a.x, a.y);
		}
		fprintf(file1, "%3.3lf, %3.3lf, %3.3lf,%3.3lf,%3.3lf,%3.3lf, %3.3lf\n", tf, pf.x, pf.y, vf.x, vf.y, af.x, af.y);

		fclose(file1);




		FILE* file2 = fopen("plan_step_case1(t_k).csv", "w");
		fprintf(file2, "step, time, period, pos_com_x, pos_com_y, vel_com_x, vel_com_y, pos_cop_x, pos_cop_y\n");

		real_t t = 0.0;
		for (uint k = 0; k < graph->ticks.size(); k++) {
			BipedLIPKey* key = (BipedLIPKey*)traj.GetKeypoint(graph->ticks[k]);

			if (k != (graph->ticks.size()) - 1) {
				fprintf(file2, "%d, %3.3lf, %3.3lf, %3.3lf, %3.3lf, %3.3lf, %3.3lf, %3.3lf, %3.3lf\n",
					k, t,
					key->period->val,
					key->com_pos_t_ssp[0]->val, key->com_pos_t_ssp[1]->val,
					key->com_vel_t_ssp[0]->val, key->com_vel_t_ssp[1]->val,
					key->cop_pos_t[0]->val, key->cop_pos_t[1]->val);
				t += key->period->val;
			}
			else {
				fprintf(file2, "%d, %3.3lf, 0, %3.3lf, %3.3lf, %3.3lf, %3.3lf, %3.3lf, %3.3lf\n",
					k, t,
					key->com_pos_t_ssp[0]->val, key->com_pos_t_ssp[1]->val,
					key->com_vel_t_ssp[0]->val, key->com_vel_t_ssp[1]->val,
					key->cop_pos_t[0]->val, key->cop_pos_t[1]->val);
			}
		}

		fclose(file2);
	}
	*/

	//-------------------------------------------------------------------------------------------------

	// Constructors
	BipedLipCon::BipedLipCon(Solver* solver, int _tag, string _name, BipedLIPKey* _obj, real_t _scale) :
		Constraint(solver, 2, ID(_tag, _obj->node, _obj->tick, _name), _scale) {
		obj[0] = _obj;
		obj[1] = (BipedLIPKey*)_obj->next;
	}

	BipedLipConP::BipedLipConP(Solver* solver, string _name, BipedLIPKey* _obj, real_t _scale) :
		BipedLipCon(solver, ConTag::BipedLipP, _name, _obj, _scale) {

		AddSLink (obj[1]->var_com_pos );
		AddSLink (obj[1]->var_cop_pos );
		AddSLink (obj[0]->var_com_pos );
		AddSLink (obj[0]->var_com_vel );
		AddSLink (obj[0]->var_cop_pos );
		AddC2Link(obj[0]->var_duration);
	}

	BipedLipConV::BipedLipConV(Solver* solver, string _name, BipedLIPKey* _obj, real_t _scale) :
		BipedLipCon(solver, ConTag::BipedLipV, _name, _obj, _scale) {

		AddSLink (obj[1]->var_com_vel );
		AddSLink (obj[1]->var_cop_pos );
		AddSLink (obj[0]->var_com_pos );
		AddSLink (obj[0]->var_com_vel );
		AddSLink (obj[0]->var_cop_pos );
		AddC2Link(obj[0]->var_duration);
	}

	/*
	CoMConR::CoMConR(Solver* solver, string _name, BipedLIPKey* _obj, uint _idx, real_t _scale) :
		Constraint(solver, 1, ID(ConTag::BipedCoMR, _obj->node, _obj->tick, _name), _scale) {
		obj[0] = _obj;
		obj[1] = (BipedLIPKey*)_obj->next;
		idx = _idx;

		AddSLink(obj[0]->com_pos_r);
		AddSLink(obj[0]->com_vel_r);
		AddSLink(obj[0]->period   );
		AddSLink(obj[1]->com_pos_r);
		AddSLink(obj[1]->com_vel_r);
	}
	*/

	BipedComConP::BipedComConP(Solver* solver, string _name, BipedLIPKey* _obj, real_t _scale) :
		Constraint(solver, 2, ID(ConTag::BipedComP, _obj->node, _obj->tick, _name), _scale) {

		obj  = _obj;
		
		AddSLink(obj->var_com_pos      );
		AddSLink(obj->var_torso_pos_t  );
		AddSLink(obj->var_foot_pos_t[0]);
		AddSLink(obj->var_foot_pos_t[1]);		
	}

	BipedComConV::BipedComConV(Solver* solver, string _name, BipedLIPKey* _obj, real_t _scale) :
		Constraint(solver, 2, ID(ConTag::BipedComV, _obj->node, _obj->tick, _name), _scale) {

		obj  = _obj;
		
		AddSLink(obj->var_com_vel      );
		AddSLink(obj->var_torso_vel_t  );
	}

	BipedFootConT::BipedFootConT(Solver* solver, string _name, BipedLIPKey* _obj, uint _side, real_t _scale) :
		Constraint(solver, 2, ID(ConTag::BipedFootRangeT, _obj->node, _obj->tick, _name), _scale) {

		obj  = _obj;
		side = _side;

		AddM2Link(obj->var_foot_pos_t[side]);
		AddM2Link(obj->var_torso_pos_t     );
		AddC2Link(obj->var_torso_pos_r     );
	}

	BipedFootConR::BipedFootConR(Solver* solver, string _name, BipedLIPKey* _obj, uint _side, real_t _scale) :
		Constraint(solver, 1, ID(ConTag::BipedFootRangeR, _obj->node, _obj->tick, _name), _scale) {

		obj  = _obj ;
		side = _side;

		AddSLink(obj->var_foot_pos_r[side]);
		AddSLink(obj->var_torso_pos_r     );
	}

	BipedCopCon::BipedCopCon(Solver* solver, string _name, BipedLIPKey* _obj, uint _side, real_t _scale) :
		Constraint(solver, 2, ID(ConTag::BipedCop, _obj->node, _obj->tick, _name), _scale) {

		obj  = _obj;
		side = _side;

		AddM2Link(obj->var_cop_pos);
		AddM2Link(obj->var_foot_pos_t[side]);
		AddC2Link(obj->var_foot_pos_r[side]);
	}
	
	BipedTimeCon::BipedTimeCon(Solver* solver, string _name, BipedLIPKey* _obj, real_t _scale) :
		Constraint(solver, 1, ID(ConTag::BipedTime, _obj->node, _obj->tick, _name), _scale) {

		obj[0] = _obj;
		obj[1] = (BipedLIPKey*)_obj->next;

		AddSLink(obj[1]->var_time);
		AddSLink(obj[0]->var_time);
		AddSLink(obj[0]->var_duration);
	}
	
	//-------------------------------------------------------------------------------------------------

	void BipedLipCon::Prepare() {
		BipedLIP::Param& param = ((BipedLIP*)obj[0]->node)->param;
		
		T    = param.T;
		T2   = T*T;
		tau  = obj[0]->var_duration->val;
		tau2 = tau*tau;
		C    = cosh(tau/T);
		S    = sinh(tau/T);
		p0   = obj[0]->var_com_pos->val;
		v0   = obj[0]->var_com_vel->val;
		c0   = obj[0]->var_cop_pos->val;
		p1   = obj[1]->var_com_pos->val;
		v1   = obj[1]->var_com_vel->val;
		c1   = obj[1]->var_cop_pos->val;
	}

	void BipedLipConP::CalcCoef() {//重心位置(左辺-右辺)を偏微分
		Prepare();

		((SLink* )links[0])->SetCoef( 1.0            );
		((SLink* )links[1])->SetCoef(-1.0 + (T/tau)*S);
		((SLink* )links[2])->SetCoef(-C              );
		((SLink* )links[3])->SetCoef(-T*S            );
		((SLink* )links[4])->SetCoef( C   - (T/tau)*S);
		((C2Link*)links[5])->SetCoef(-(S/T)*(p0 - c0) - C*(v0 - (c1 - c0)/tau) - (T*S)*(c1 - c0)/tau2);
	}

	void BipedLipConV::CalcCoef() {//重心速度(左辺-右辺)を偏微分
		Prepare();

		((SLink* )links[0])->SetCoef( 1.0               );
		((SLink* )links[1])->SetCoef((C - 1.0)/tau      );
		((SLink* )links[2])->SetCoef(-S/T               );
		((SLink* )links[3])->SetCoef(-C                 );
		((SLink* )links[4])->SetCoef((1.0 - C)/tau + S/T);
		((C2Link*)links[5])->SetCoef(-(C/T2)*(p0 - c0) - (S/T)*(v0 - (c1 - c0)/tau) + (1.0 - C)*(c1 - c0)/tau2);
	}

	/*
	void CoMConR::CalcCoef() {
		q0 = obj[0]->com_pos_r->val;
		w0 = obj[0]->com_vel_r->val;
		q1 = obj[1]->com_pos_r->val;
		w1 = obj[1]->com_vel_r->val;
		tau = obj[0]->period->val;
		tau2 = tau*tau;
		tau3 = tau*tau2;

		if (idx == 0) {
			((SLink*)links[0])->SetCoef(-6.0 / tau2);
			((SLink*)links[1])->SetCoef(-4.0 / tau);
			((SLink*)links[2])->SetCoef(-12.0*(q1 - q0) / tau3 + (4.0*w0 + 2.0*w1) / tau2);
			((SLink*)links[3])->SetCoef(6.0 / tau2);
			((SLink*)links[4])->SetCoef(-2.0 / tau);
		}
		else {
			((SLink*)links[0])->SetCoef(6.0 / tau2);
			((SLink*)links[1])->SetCoef(2.0 / tau);
			((SLink*)links[2])->SetCoef(12.0*(q1 - q0) / tau3 - (2.0*w0 + 4.0*w1) / tau2);
			((SLink*)links[3])->SetCoef(-6.0 / tau2);
			((SLink*)links[4])->SetCoef(4.0 / tau);
		}
	}
	*/

	void BipedComConP::CalcCoef(){
		BipedLIP::Param& param = ((BipedLIP*)obj->node)->param;
		
		real_t mt = param.torsoMass;
		real_t mf = param.footMass;

		((SLink*)links[0])->SetCoef(mt + 2.0*mf);
		((SLink*)links[1])->SetCoef(-mt);
		((SLink*)links[2])->SetCoef(-mf);
		((SLink*)links[3])->SetCoef(-mf);

	}

	void BipedComConV::CalcCoef(){
		BipedLIP::Param& param = ((BipedLIP*)obj->node)->param;
		
		real_t mt = param.torsoMass;
		real_t mf = param.footMass;

		((SLink*)links[0])->SetCoef(mt + 2.0*mf);
		((SLink*)links[1])->SetCoef(-mt);
	}

	void BipedFootConT::CalcCoef() {
		pf     = obj->var_foot_pos_t[side]->val;
		pt     = obj->var_torso_pos_t->val;
		thetat = obj->var_torso_pos_r->val;
		 R     = mat2_t::Rot(thetat);
		dR     = mat2_t::Rot(thetat + pi/2);

		((M2Link*)links[0])->SetCoef( R.trans());
		((M2Link*)links[1])->SetCoef(-R.trans());
		((C2Link*)links[2])->SetCoef(dR.trans()*(pf - pt));
	}

	void BipedFootConR::CalcCoef() {
		thetaf = obj->var_foot_pos_r[side]->val;
		thetat = obj->var_torso_pos_r    ->val;

		((SLink*)links[0])->SetCoef( 1.0);
		((SLink*)links[1])->SetCoef(-1.0);
	}

	void BipedCopCon::CalcCoef() {
		pc     = obj->var_cop_pos->val;
		pf     = obj->var_foot_pos_t[side]->val;
		thetaf = obj->var_foot_pos_r[side]->val;
		 R = mat2_t::Rot(thetaf);
		dR = mat2_t::Rot(thetaf + pi/2);

		((M2Link*)links[0])->SetCoef( R.trans());
		((M2Link*)links[1])->SetCoef(-R.trans());
		((C2Link*)links[2])->SetCoef(dR.trans()*(pc - pf));
	}

	void BipedTimeCon::CalcCoef(){
		((SLink*)links[0])->SetCoef( 1.0);
		((SLink*)links[1])->SetCoef(-1.0);
		((SLink*)links[2])->SetCoef(-1.0);
	}

	//-------------------------------------------------------------------------------------------------

	void BipedLipConP::CalcDeviation(){
		vec2_t y2 = p1 - c1 - C*(p0 - c0) - (T*S)*(v0 - (c1 - c0)/tau);
		y[0] = y2[0];
		y[1] = y2[1];
	}

	void BipedLipConV::CalcDeviation(){
		vec2_t y2 = v1 - (c1 - c0)/tau - (S/T)*(p0 - c0) - C*(v0 - (c1 - c0)/tau);
		y[0] = y2[0];
		y[1] = y2[1];
	}

	/*
	void CoMConR::CalcDeviation() {
		real_t s;
		if (idx == 0)
			 s =  6.0*(q1 - q0) / tau2 - (4.0*w0 + 2.0*w1) / tau;
		else s = -6.0*(q1 - q0) / tau2 + (2.0*w0 + 4.0*w1) / tau;
		on_lower = (s < _min);
		on_upper = (s > _max);
		active = on_lower | on_upper;
		if (on_lower) y[0] = (s - _min);
		if (on_upper) y[0] = (s - _max);
	}
	*/

	void BipedFootConT::CalcDeviation() {
		vec2_t s = R.trans() * (pf - pt);

		active = false;
		for(uint i = 0; i < 2; i++){
			on_lower[i] = (s[i] < _min[i]);
			on_upper[i] = (s[i] > _max[i]);
			if(on_lower[i]){
				y[i]   = (s[i] - _min[i]);
				active = true;
			}
			if(on_upper[i]){
				y[i]   = (s[i] - _max[i]);
				active = true;
			}
		}
	}

	void BipedFootConR::CalcDeviation() {
		real_t s = thetaf - thetat;
		on_lower = (s < _min);
		on_upper = (s > _max);
		active = on_lower | on_upper;
		if (on_lower) y[0] = (s - _min);
		if (on_upper) y[0] = (s - _max);
	}

	void BipedCopCon::CalcDeviation() {
		vec2_t s = R.trans() * (pc - pf);

		active = false;
		for(uint i = 0; i < 2; i++){
			on_lower[i] = (s[i] < _min[i]);
			on_upper[i] = (s[i] > _max[i]);
			if(on_lower[i]){
				y[i]   = (s[i] - _min[i]);
				active = true;
			}
			if(on_upper[i]){
				y[i]   = (s[i] - _max[i]);
				active = true;
			}
		}
	}

	//-------------------------------------------------------------------------------------------------
	
	/*
	void CoMConR::Project(real_t& l, uint k) {
		if (on_upper &&  l > 0.0) l = 0.0;
		if (on_lower &&  l < 0.0) l = 0.0;
		if (!on_upper && !on_lower) l = 0.0;
	}
	*/

	void BipedFootConT::Project(real_t& l, uint k) {
		if (on_upper[k] &&  l > 0.0) l = 0.0;
		if (on_lower[k] &&  l < 0.0) l = 0.0;
		if (!on_upper[k] && !on_lower[k]) l = 0.0;
	}

	void BipedFootConR::Project(real_t& l, uint k) {
		if (on_upper &&  l > 0.0) l = 0.0;
		if (on_lower &&  l < 0.0) l = 0.0;
		if (!on_upper && !on_lower) l = 0.0;
	}

	void BipedCopCon::Project(real_t& l, uint k) {
		if (on_upper[k] &&  l > 0.0) l = 0.0;
		if (on_lower[k] &&  l < 0.0) l = 0.0;
		if (!on_upper[k] && !on_lower[k]) l = 0.0;
	}

}
