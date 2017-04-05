/*
enuvo2に向けて
動障害物回避のBiped2を移植する

3/14 中間地点指定ver
*/

#include <DiMP2/Graph.h>

namespace DiMP2
{;

//-------------------------------------------------------------------------------------------------
// BipedLIPKey

BipedLIPKey::BipedLIPKey(){
	phase = BipedLIP::Phase::Right;
}

void BipedLIPKey::AddVar(Solver* solver){
	BipedLIP* obj = GetNode();

	pos_com[0] = new SVar(solver, ID(0, node, tick, name + "_pos0"), node->graph->scale.pos_t);
	pos_com[1] = new SVar(solver, ID(0, node, tick, name + "_pos1"), node->graph->scale.pos_t);
	vel_com[0] = new SVar(solver, ID(0, node, tick, name + "_vel0"), node->graph->scale.vel_t);
	vel_com[1] = new SVar(solver, ID(0, node, tick, name + "_vel1"), node->graph->scale.vel_t);
	// N歩目のcopは遊脚軌道の終点として必要
	pos_cop[0] = new SVar(solver, ID(0, node, tick, name + "_cop0"), node->graph->scale.pos_t);
	pos_cop[1] = new SVar(solver, ID(0, node, tick, name + "_cop1"), node->graph->scale.pos_t);

	// 0歩目の遊脚始点
	if(!prev){
		pos_swg[0] = new SVar(solver, ID(0, node, tick, name + "_swg0"), node->graph->scale.pos_t);
		pos_swg[1] = new SVar(solver, ID(0, node, tick, name + "_swg1"), node->graph->scale.pos_t);
	}

	if(next){
		period       = new SVar(solver, ID(0, node, tick, name + "_T"   ), node->graph->scale.time );
		con_move_cop = new ProhMoveConS(solver, ID(0, node, tick, name + "_move_cop"), pos_cop[0], pos_cop[1], period, node->graph->scale.pos_t);
	}
}

void BipedLIPKey::AddCon(Solver* solver){
	BipedLIPKey* nextObj = (BipedLIPKey*)next;
	
	if(next){
		con_lip_pos[0]     = new LIPConP  (solver, name + "_lip_p0", this, 0, node->graph->scale.pos_t);
		con_lip_pos[1]     = new LIPConP  (solver, name + "_lip_p1", this, 1, node->graph->scale.pos_t);
		con_lip_vel[0]     = new LIPConV  (solver, name + "_lip_v0", this, 0, node->graph->scale.vel_t);
		con_lip_vel[1]     = new LIPConV  (solver, name + "_lip_v1", this, 1, node->graph->scale.vel_t);
		con_range_period   = new RangeConS(solver, ID(0, node, tick, name + "_range_period"), period, node->graph->scale.time);
		con_diff_cop[0][0] = new DiffConS (solver, ID(0, node, tick, name + "_range_cop0x" ), pos_com[0], pos_cop[0], node->graph->scale.pos_t);
		con_diff_cop[0][1] = new DiffConS (solver, ID(0, node, tick, name + "_range_cop0y" ), pos_com[1], pos_cop[1], node->graph->scale.pos_t);
		con_diff_cop[1][0] = new DiffConS (solver, ID(0, node, tick, name + "_range_cop1x" ), nextObj->pos_com[0], pos_cop[0], node->graph->scale.pos_t);
		con_diff_cop[1][1] = new DiffConS (solver, ID(0, node, tick, name + "_range_cop1y" ), nextObj->pos_com[1], pos_cop[1], node->graph->scale.pos_t);
	}
}

void BipedLIPKey::Prepare(){
	// 歩行周期変数の値を時刻に反映
	if(!prev){
		tick->time = 0.0;
	}
	else{
		BipedLIPKey* prevObj = (BipedLIPKey*)prev;
		tick->time = prevObj->tick->time + prevObj->period->val;
	}
}

void BipedLIPKey::Draw(DrawCanvas* canvas, DrawConfig* conf){
	// 脚交換時の重心位置
	canvas->SetPointSize(5.0f);
	canvas->Point(Vec3f((float)pos_com[0]->val, (float)pos_com[1]->val, (float)GetNode()->param.heightCoM));
	
	// 脚交換時の足先位置
	canvas->SetPointSize(5.0f);
	canvas->Point(Vec3f((float)pos_cop[0]->val, (float)pos_cop[1]->val, 0.0f));

	// 脚交換
	canvas->SetLineWidth(1.0f);
	canvas->Line(Vec3f((float)pos_cop[0]->val, (float)pos_cop[1]->val, 0.0f) , Vec3f((float)pos_com[0]->val, (float)pos_com[1]->val, (float)GetNode()->param.heightCoM));

	if(next){
	BipedLIPKey* nextObj = (BipedLIPKey*)next;
	canvas->Line(Vec3f((float)pos_cop[0]->val, (float)pos_cop[1]->val, 0.0f) , Vec3f((float)nextObj->pos_com[0]->val, (float)nextObj->pos_com[1]->val, (float)GetNode()->param.heightCoM));
	}

	//if(next){
	//	canvas->SetPointSize(13.0f);
	//	canvas->Point(Vec3f((float)pos_cop[0]->val, (float)pos_cop[1]->val, 0.0f));
	//}
}

//-------------------------------------------------------------------------------------------------
// BipedLIP

BipedLIP::Param::Param(){
	gravity        = 9.8;
	heightCoM      = 0.5;
	torsoMassRatio = 0.5;
	legMassRatio   = 0.5;
	swingProfile   = SwingProfile::Wedge;
	swingHeight[0] = 0.05;
	swingHeight[1] = 0.01;
	stepPeriodMin  = 0.1;
	stepPeriodMax  = 1.0;
	supportMin[0]  = vec2_t(-0.3, -0.2 );
	supportMax[0]  = vec2_t( 0.3, -0.05);
	supportMin[1]  = vec2_t(-0.3,  0.05);
	supportMax[1]  = vec2_t( 0.3,  0.2 );
}

BipedLIP::Waypoint::Waypoint(){
	k           = 0;
	fix_pos_com = false;
	fix_vel_com = false;
	fix_pos_cop = false;
	fix_pos_swg = false;
}

void BipedLIP::AddMoveArea(vec2_t proh_move, real_t proh_r,vec2_t vel_move){
	MoveArea ma;
	ma.proh_move = proh_move;
	ma.r		 = proh_r;
	ma.vel_move  = vel_move;
	moveareas.push_back(ma);
	int movesize = moveareas.size();
}

BipedLIP::TrajPoint::TrajPoint(){
	t = 0.0f;
}

BipedLIP::BipedLIP(Graph* g, string n):TrajectoryNode(g, n){
}

void BipedLIP::Init(){
	TrajectoryNode::Init();

	param.T = sqrt(param.heightCoM/param.gravity);

	real_t periodAve = (param.stepPeriodMin + param.stepPeriodMax) / 2.0;

	// 描写初期時刻
	tm = 0.0 ;
	start = 0;
	endPhase = false;

	for(uint k = 0; k < graph->ticks.size(); k++){
		BipedLIPKey* key = GetKeypoint(graph->ticks[k]);
		
		key->phase = phase[k];
		
		if (key->next){
			// 周期の初期値は下限と上限の中間値
			key->period->val            = periodAve;
			key->con_range_period->_min = param.stepPeriodMin;
			key->con_range_period->_max = param.stepPeriodMax;
			
			for(int i = 0; i < 2; i++){
				key->con_diff_cop[i][0]->_min = param.supportMin[key->phase].x;
				key->con_diff_cop[i][0]->_max = param.supportMax[key->phase].x;
				key->con_diff_cop[i][1]->_min = param.supportMin[key->phase].y;
				key->con_diff_cop[i][1]->_max = param.supportMax[key->phase].y;
			}

			// 動障害物
			key->con_move_cop->_movesize = moveareas.size();
			for (uint i = 0; i < moveareas.size() ; i++){
				MoveArea& ma = moveareas[i];
				key->con_move_cop->_ConIni[i][0] = ma.proh_move[0];
				key->con_move_cop->_ConIni[i][1] = ma.proh_move[1];
				key->con_move_cop->_r[i]			= ma.r;
				key->con_move_cop->_vx[i]			= ma.vel_move[0];
				key->con_move_cop->_vy[i]			= ma.vel_move[1];
			}
		}
	}

	// 重心位置，重心速度，着地位置の初期値を経由点を結ぶスプライン曲線で与える
	//Curve2d curve;
	//curve.SetType(Interpolate::Cubic);
	//for(uint i = 0; i < waypoints.size(); i++){
	//	Waypoint& wp = waypoints[i];
	//	curve.AddPoint(wp.k * periodAve);
	//	curve.SetPos(i, wp.pos_com);
	//	curve.SetVel(i, wp.vel_com);
	//}
	//for(uint k = 0; k < graph->ticks.size(); k++){
	//	BipedLIPKey* key = GetKeypoint(graph->ticks[k]);
	//	vec2_t p = curve.CalcPos(k * periodAve);
	//	vec2_t v = curve.CalcVel(k * periodAve);
	//	key->pos_com[0]->val = p[0];
	//	key->pos_com[1]->val = p[1];
	//	key->vel_com[0]->val = v[0];
	//	key->vel_com[1]->val = v[1];
	//	if(key->next){
	//		p = curve.CalcPos((k+0.5) * periodAve);
	//		key->pos_cop[0]->val = p[0];
	//		key->pos_cop[1]->val = p[1];
	//	}
	//}

	// 初期軌道の設計
	Waypoint* wp = &waypoints[0];
		for(int k = 0; k < (int)graph->ticks.size(); k++){
			BipedLIPKey* key = GetKeypoint(graph->ticks[k]);
			if(wp[1].k < k)
				wp++;
			//real_t s = (real_t)(k - wp[0].k)/(real_t)(wp[1].k - wp[0].k);
			//key->pos_com[0]->val = (1-s)*wp[0].pos_cop[0] + s*wp[1].pos_cop[0];
			//key->pos_cop[0]->val = (1-s)*wp[0].pos_cop[0] + s*wp[1].pos_cop[0];
			
			key->pos_com[0]->val = wp[1].pos_cop[0] * (real_t)(k)/graph->ticks.size();
			key->pos_cop[0]->val = wp[1].pos_cop[0] * (real_t)(k)/graph->ticks.size();


			// とりあえず横方向だけ手打ち
			real_t sideWalk; // 横方向移動距離
			int StartSide,MidSide;
			
			sideWalk = 0.3;
			StartSide = 0;
			MidSide = 12;

			if ( k > StartSide && k <= MidSide )
			{
				real_t i = k - StartSide;
				key->pos_com[1]->val = sideWalk * (i/ (MidSide - StartSide));
				key->pos_cop[1]->val = sideWalk * (i/ (MidSide - StartSide));
			}

			
			if ( k > MidSide )
			{
				real_t i = k - MidSide;
				key->pos_com[1]->val = 0.2; //sideWalk - (sideWalk * (i/ ((int)graph->ticks.size() - MidSide)));
				key->pos_cop[1]->val = 0.2; //sideWalk - (sideWalk * (i/ ((int)graph->ticks.size() - MidSide)));
			}

	}

	// 経由点上の変数を固定
	for(uint i = 0; i < waypoints.size(); i++){
		Waypoint& wp = waypoints[i];
		BipedLIPKey* key = GetKeypoint(graph->GetTick(wp.k));
		if(wp.fix_pos_com){
			key->pos_com[0]->val    = wp.pos_com[0];
			key->pos_com[1]->val    = wp.pos_com[1];
			key->pos_com[0]->locked = true;
			key->pos_com[1]->locked = true;
		}
		if(wp.fix_vel_com){
			key->vel_com[0]->val    = wp.vel_com[0];
			key->vel_com[1]->val    = wp.vel_com[1];
			key->vel_com[0]->locked = true;
			key->vel_com[1]->locked = true;
		}
		if(wp.fix_pos_cop){
			key->pos_cop[0]->val    = wp.pos_cop[0];
			key->pos_cop[1]->val    = wp.pos_cop[1];
			key->pos_cop[0]->locked = true;
			key->pos_cop[1]->locked = true;
		}
		if(wp.fix_pos_swg && !key->prev){
			key->pos_swg[0]->val    = wp.pos_swg[0];
			key->pos_swg[1]->val    = wp.pos_swg[1];
			key->pos_swg[0]->locked = true;
			key->pos_swg[1]->locked = true;
		}
	}
	
}

void BipedLIP::Prepare(){
	TrajectoryNode::Prepare();
	trajReady = false;
}

int BipedLIP::Phase(real_t t){
	BipedLIPKey* key = GetSegment(t).first;
	return key->phase;
}


vec3_t BipedLIP::PosCoM(real_t t){
	BipedLIPKey* key = GetSegment(t).first;

	vec3_t p;
	if(key->next){
		real_t T    = param.T;
		real_t t0   = key->tick->time;			
		real_t px   = key->pos_com[0]->val;		
		real_t py   = key->pos_com[1]->val;
		real_t vx   = key->vel_com[0]->val;		
		real_t vy   = key->vel_com[1]->val;
		real_t copx = key->pos_cop[0]->val;	
		real_t copy = key->pos_cop[1]->val;

		p.x = copx + (px - copx) * cosh((t-t0)/T)  + (vx*T) * sinh((t-t0)/T)  ;
		p.y = copy + (py - copy) * cosh((t-t0)/T)  + (vy*T) * sinh((t-t0)/T)  ;
		p.z = param.heightCoM;
	}
	else{
		p.x = key->pos_com[0]->val;
		p.y = key->pos_com[1]->val;
		p.z = param.heightCoM;
	}

	return p;
}

vec3_t BipedLIP::VelCoM(real_t t){
	BipedLIPKey* key = GetSegment(t).first;

	vec3_t v;
	if(key->next){
		real_t T    = param.T;
		real_t t0   = key->tick->time;
		real_t px   = key->pos_com[0]->val;
		real_t py   = key->pos_com[1]->val;
		real_t vx   = key->vel_com[0]->val;
		real_t vy   = key->vel_com[1]->val;
		real_t copx = key->pos_cop[0]->val;
		real_t copy = key->pos_cop[1]->val;

		v.x = ((px-copx)/T) * sinh((t-t0)/T) + (vx) * cosh((t-t0)/T);
		v.y = ((py-copy)/T) * sinh((t-t0)/T) + (vy) * cosh((t-t0)/T);
		v.z = 0.0;
	}
	else{
		v.x = key->vel_com[0]->val;
		v.y = key->vel_com[1]->val;
		v.z = 0.0;
	}

	return v;
}

vec3_t BipedLIP::AccCoM(real_t t){
	BipedLIPKey* key = GetSegment(t).first;

	vec3_t a;
	if(key->next){
		real_t T    = param.T;
		real_t copx = key->pos_cop[0]->val;
		real_t copy = key->pos_cop[1]->val;
		vec3_t p    = PosCoM(t);
		a.x = (p.x - copx)/(T*T);
		a.y = (p.y - copy)/(T*T);
		a.z = 0.0;
	}
	else{
		a.clear();
	}

	return a;
}

vec3_t BipedLIP::PosCoP(real_t t){
	BipedLIPKey* key = GetSegment(t).first;
	
	return vec3_t(key->pos_cop[0]->val, key->pos_cop[1]->val, 0.0);
}

vec3_t BipedLIP::PosTorso(const vec3_t& pcom, const vec3_t& psup, const vec3_t& pswg){
	// コンパスモデルより胴体の位置を求める
	real_t a = param.torsoMassRatio;
	real_t b = param.legMassRatio;
	//vec3_t p = (pcom - ((1-a)*(1-b)/2)*(psup+pswg)) / (a + (1-a)*b);
	vec3_t p = ((a+2)*pcom - (1-b)*(psup+pswg)) / (a + 2*b);
	return p;
}

vec3_t BipedLIP::PosSupportFoot(real_t t){
	return PosCoP(t);
}

real_t BipedLIP::period(real_t t){
	BipedLIPKey* key = GetSegment(t).first;
	
	return real_t (key->period->val);
}

vec3_t BipedLIP::PosSwingFoot(real_t t){
	BipedLIPKeyPair keys = GetSegment(t);
	BipedLIPKey*    cur  = keys.first;
	BipedLIPKey*    next = keys.second;
	BipedLIPKey*    prev = (BipedLIPKey*)cur->prev;
	real_t t0    = cur ->tick->time;
	real_t t1    = next->tick->time;
	real_t h     = t1 - t0;
	real_t hhalf = h/2.0;

	// 遊脚の始点と終点は前後の接地点
	// 0歩目の始点は0歩目の支持足の反対側に設定
	// N-1歩目の終点はN-1歩目の支持足の反対側に設定
	vec2_t p0, p1;
	// 1～N-1歩目
	if(prev){
		p0[0] = prev->pos_cop[0]->val;
		p0[1] = prev->pos_cop[1]->val;
	}
	// 0歩目
	else{
		p0[0] = cur->pos_swg[0]->val;
		p0[1] = cur->pos_swg[1]->val;
	}

	if(next){
		p1[0] = next->pos_cop[0]->val;
		p1[1] = next->pos_cop[1]->val;
	}
	else{
		p1[0] = prev->pos_cop[0]->val;
		p1[1] = prev->pos_cop[1]->val;
	}

	real_t s;
	real_t z;
	if(h == 0.0){
		s = 0.0;
		z = 0.0;
	}
	else{
		if(param.swingProfile == SwingProfile::Wedge){
			if(t < t0 + hhalf){
				z = param.swingHeight[0];
			}
			else{
				real_t a = (t - (t0+hhalf)) / hhalf;
				z = (1-a)*param.swingHeight[0] + a*param.swingHeight[1];
			}

			s = (t - t0)/h;
		}
		if(param.swingProfile == SwingProfile::Cycloid){
			real_t _2pi = 2.0 * M_PI;
			real_t tau = (t - t0)/h;
		
			s = (tau - sin(_2pi*tau)/_2pi);
			z = (param.swingHeight[0]/2.0) * (1 - cos(_2pi*tau));
		}
	}

	vec3_t p;
	p[0] = (1-s) * p0[0] + s * p1[0];
	p[1] = (1-s) * p0[1] + s * p1[1];
	p[2] = z;

	return p;
}

//------------------------------------------------------------------------------------------------

void BipedLIP::CalcTrajectory(){
	real_t tf = traj.back()->tick->time;
	real_t dt = 0.01;
	
	trajectory.clear();
	for(real_t t = 0.0; t < tf; t += dt){
		TrajPoint tp;
		tp.t         = t;
		tp.pos_com   = PosCoM(t);
		tp.pos_cop   = PosCoP(t);
		tp.pos_swing = PosSwingFoot(t);
		tp.pos_torso = PosTorso(tp.pos_com, tp.pos_cop, tp.pos_swing);

		trajectory.push_back(tp);
	}

	trajReady = true;
}

//------------------------------------------------------------------------------------------------

void BipedLIP::Draw(DrawCanvas* canvas, DrawConfig* conf){
	TrajectoryNode::Draw(canvas, conf);


	if(!trajReady)
		CalcTrajectory();

	if(trajectory.empty())
		return;

	// com
	if(conf->Set(canvas, DrawItem::BipedCoM, this)){
		canvas->BeginLayer("biped_com", true);
		canvas->SetLineWidth(4.0f);
		canvas->BeginPath();
		canvas->MoveTo(trajectory[0].pos_com);
		for(uint i = 1; i < trajectory.size(); i++){
			canvas->LineTo(trajectory[i].pos_com);
		}
		canvas->EndPath();
		canvas->EndLayer();
	}

	//// torso
	//if(conf->Set(canvas, DrawItem::BipedTorso, this)){
	//	canvas->BeginLayer("biped_torso", true);
	//	canvas->SetLineWidth(3.0f);
	//	canvas->BeginPath();
	//	canvas->MoveTo(trajectory[0].pos_torso);
	//	for(uint i = 1; i < trajectory.size(); i++){
	//		canvas->LineTo(trajectory[i].pos_torso);
	//	}
	//	canvas->EndPath();
	//	canvas->EndLayer();
	//}
	// swing foot
	if(conf->Set(canvas, DrawItem::BipedSwing, this)){
		canvas->BeginLayer("biped_swing", true);
		canvas->SetLineWidth(2.0f);
		canvas->BeginPath();
		canvas->MoveTo(trajectory[0].pos_swing);
		for(uint i = 1; i < trajectory.size(); i++){
			if(trajectory[i-1].pos_cop == trajectory[i].pos_cop){
				canvas->LineTo(trajectory[i].pos_swing);
			}
			else{
				canvas->EndPath();
				canvas->BeginPath();
				canvas->MoveTo(trajectory[i].pos_swing);
			}
		}
		canvas->EndPath();
		canvas->EndLayer();
	}
	//// double support snapshot
	//if(conf->Set(canvas, DrawItem::BipedDouble, this)){
	//	canvas->BeginLayer("biped_double", true);
	//	canvas->SetLineWidth(1.0f);
	//	for(uint i = 1; i < trajectory.size(); i++){
	//		if(trajectory[i-1].pos_cop != trajectory[i].pos_cop){
	//			canvas->Line(trajectory[i].pos_torso, trajectory[i  ].pos_cop);
	//			canvas->Line(trajectory[i].pos_torso, trajectory[i-1].pos_cop);
	//		}
	//	}
	//	canvas->EndLayer();
	//}


	// 動く様子の描写を行う
	if(move_trajectory){

		BipedLIPKey* startkey = GetSegment(0).first;


		// 描写開始時の時間
		if (start == 0)
			start = clock();


		tm = (float) (clock() - start) / (CLOCKS_PER_SEC);

		BipedLIPKey* key = GetSegment(tm).first;
		
		// 重心位置
		canvas->SetPointSize(12.0f);
		canvas->Point(Vec3f(PosCoM(tm)));
		
		// 接地位置
		canvas->SetPointSize(10.0f);
		canvas->Point(Vec3f(PosCoP(tm)));
		
		// 重心と接地位置を結ぶ 
		canvas->SetLineWidth(6.0f);
		canvas->Line(Vec3f(PosCoP(tm)) , Vec3f(PosCoM(tm)));
		
		// 遊脚位置
		canvas->SetPointSize(10.0f);
		canvas->Point(Vec3f(PosSwingFoot(tm)));
		
		
		//// 遊脚位置
		//if (!endPhase){
		//	canvas->SetPointSize(10.0f);
		//	canvas->Point(Vec3f(PosSwingFoot(tm)));
		//}
		//
		//else{
		//	canvas->SetPointSize(10.0f);
		//	canvas->Point(Vec3f(PosSwingFoot(tm).x,PosSwingFoot(tm).y,0.06f));
		//}

		// 重心と遊脚を結ぶ 
		canvas->SetLineWidth(6.0f);
		canvas->Line(Vec3f(PosSwingFoot(tm)), Vec3f(PosCoM(tm)));
		
		
		// 動障害物の描写
		canvas->SetLineWidth(2.0f);
		for (uint i = 0; i < moveareas.size() ; i++){
			MoveArea& ma = moveareas[i];
			canvas->Circle(Vec2f(ma.proh_move[0] + tm * ma.vel_move[0] ,ma.proh_move[1] + tm * ma.vel_move[1]), ma.r);
		}		
	}	// if


}

void BipedLIP::DrawSnapshot(real_t time, DrawCanvas* canvas, DrawConfig* conf){
}

void BipedLIP::Save(){
	FILE* file = fopen("plan_stepOA.csv", "w");
	fprintf(file, "step, period,, pos_com_x, pos_com_y, ,vel_com_x, vel_com_y,, pos_cop_x, pos_cop_y\n");

	real_t t = 0.0;
	for(uint k = 0; k < graph->ticks.size(); k++){
		BipedLIPKey* key = (BipedLIPKey*)GetKeypoint(graph->ticks[k]);

		if (k < graph->ticks.size() - 1){
			fprintf(file, "%d, %3.3lf,, %3.3lf, %3.3lf,, %3.3lf, %3.3lf,, %3.3lf, %3.3lf\n",
				k, 
				key->period->val, 
				key->pos_com[0]->val, key->pos_com[1]->val,
				key->vel_com[0]->val, key->vel_com[1]->val,
				key->pos_cop[0]->val, key->pos_cop[1]->val);
		}

		else{
			fprintf(file, "%d, %3.3lf,, %3.3lf, %3.3lf,, %3.3lf, %3.3lf,, %3.3lf, %3.3lf\n",
				k, 0.0,
				key->pos_com[0]->val, key->pos_com[1]->val,
				key->vel_com[0]->val, key->vel_com[1]->val,
				key->pos_cop[0]->val, key->pos_cop[1]->val);
		}
	}

	fclose(file);
}

//-------------------------------------------------------------------------------------------------
// Constructors

LIPCon::LIPCon(Solver* solver, string _name, BipedLIPKey* _obj, uint _idx, real_t _scale):
	Constraint(solver, 1, ID(0, _obj->node, _obj->tick, _name), _scale){
	obj[0] = _obj;
	obj[1] = (BipedLIPKey*)_obj->next;
	idx    = _idx;
}

LIPConP::LIPConP(Solver* solver, string _name, BipedLIPKey* _obj, uint _idx, real_t _scale):
	LIPCon(solver, _name, _obj, _idx, _scale){

	AddSLink(obj[0]->pos_com[idx]);
	AddSLink(obj[0]->vel_com[idx]);
	AddSLink(obj[0]->pos_cop[idx]);
	AddSLink(obj[0]->period      );
	AddSLink(obj[1]->pos_com[idx]);
}

LIPConV::LIPConV(Solver* solver, string _name, BipedLIPKey* _obj, uint _idx, real_t _scale):
	LIPCon(solver, _name, _obj, _idx, _scale){
	AddSLink(obj[0]->pos_com[idx]);
	AddSLink(obj[0]->vel_com[idx]);
	AddSLink(obj[0]->pos_cop[idx]);
	AddSLink(obj[0]->period      );
	AddSLink(obj[1]->vel_com[idx]);
}

//-------------------------------------------------------------------------------------------------

void LIPCon::Prepare(){
	BipedLIP::Param& param = obj[0]->GetNode()->param;
	T  = param.T;
	t  = obj[0]->period->val/T;
	ch = cosh(t);
	sh = sinh(t);
	p0 = obj[0]->pos_com[idx]->val;
	v0 = obj[0]->vel_com[idx]->val;
	c  = obj[0]->pos_cop[idx]->val;
	p1 = obj[1]->pos_com[idx]->val;
	v1 = obj[1]->vel_com[idx]->val;
}

void LIPConP::CalcCoef(){
	Prepare();
	((SLink*)links[0])->SetCoef(-ch);
	((SLink*)links[1])->SetCoef(-T * sh);
	((SLink*)links[2])->SetCoef(ch - 1.0);
	((SLink*)links[3])->SetCoef(- ((p0-c)/T)*sh - v0*ch);
	((SLink*)links[4])->SetCoef(1.0);
}

void LIPConV::CalcCoef(){
	Prepare();
	((SLink*)links[0])->SetCoef(-(1/T)*sh);
	((SLink*)links[1])->SetCoef(-ch);
	((SLink*)links[2])->SetCoef( (1/T)*sh);
	((SLink*)links[3])->SetCoef(- ((p0-c)/(T*T))*ch - (v0/T)*sh);
	((SLink*)links[4])->SetCoef(1.0);
}

//-------------------------------------------------------------------------------------------------

void LIPConP::CalcDeviation(){
	y[0] = p1 - c - (p0-c)*ch - (v0*T)*sh;		
}

void LIPConV::CalcDeviation(){
	y[0] = v1 - ((p0-c)/T)*sh - v0*ch;			
}

// 
//----------------------------------------------------------------------------------------------------------------------
// 動障害物に対する評価 
//----------------------------------------------------------------------------------------------------------------------

ProhMoveConS::ProhMoveConS(Solver* solver, ID id, SVar* var0, SVar* var1, SVar* var2, real_t _scale):Constraint(solver, 1, id, _scale){
	AddSLink(var0, 0.0);
	AddSLink(var1, 1.0);
	AddSLink(var2, 0.0); // period
//	real_t inf = numeric_limits<real_t>::max();
}


void ProhMoveConS::CalcCoef(){
	//((SLink*)links[0])->SetCoef(2.0*y[0]);
}
void ProhMoveConS::CalcDeviation(){
	real_t pos_x  = ((SVar*)links[0]->var)->val;
	real_t pos_y  = ((SVar*)links[1]->var)->val;
	real_t period = ((SVar*)links[2]->var)->val;	// 1ステップの時間
	t = tick->time ;								// ステップ開始時の時刻
	active = false ;	// 初期化
	vec2_t cur,next;	// cur：ステップ始まり時の障害物位置，next：ステップ終了時の障害物位

	for ( int i = 0; i < _movesize ; i++){

		cur[0]  = _ConIni[i][0] + t * _vx[i];
		cur[1]  = _ConIni[i][1] + t * _vy[i];
		next[0] = _ConIni[i][0] + (t + period) * _vx[i];
		next[1] = _ConIni[i][1] + (t + period) * _vy[i];

		// 移動距離
		real_t move = sqrt((cur.x-next.x)*(cur.x-next.x) + (cur.y-next.y)*(cur.y-next.y));
		// 移動の中間
		vec2_t mid = vec2_t((cur.x + next.x)/2,(cur.y + next.y)/2);
		// 新たな障害物半径
		real_t r_n = _r[i] + move / 2 ;

		// 移動障害物の中間位置と接地点との距離
		real_t dist = sqrt((mid.x - pos_x)*(mid.x - pos_x) + (mid.y - pos_y)*(mid.y - pos_y));
		if ( dist < r_n){
			
			active = true;
			if(_ConIni[i][1] > 0)	//_ConIni[i][1]
				y[0] = abs (dist - r_n) ;
			else
				y[0] =  - abs (dist - r_n) ;

		}
		
	}	// for

}	// calc diviation

// 修正量(l)の修正
void ProhMoveConS::Project(real_t& l, uint k){	// active のときだけ働く
	if (y[0] > 0 && l > 0)
		l = 0;
	if (y[0] < 0 && l < 0)
		l = 0;
}

}
