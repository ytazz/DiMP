// Bipedのコピー
// 接地不可領域の指定を行う
#include <DiMP2/Graph.h>
#include <DiMP2/App.h>


namespace DiMP2
{;

//-------------------------------------------------------------------------------------------------
// BipedLIPKey

BipedLIPKey::BipedLIPKey(){
	phase = BipedLIP::Left;
}

// variables
void BipedLIPKey::AddVar(Solver* solver){
	BipedLIP* obj = GetNode();

	pos_t  [0] = new SVar(solver, ID(0, node, tick, name + "_pos0"), node->graph->scale.pos_t);	// position of com
	pos_t  [1] = new SVar(solver, ID(0, node, tick, name + "_pos1"), node->graph->scale.pos_t);
	vel_t  [0] = new SVar(solver, ID(0, node, tick, name + "_vel0"), node->graph->scale.vel_t);	// velocity of com
	vel_t  [1] = new SVar(solver, ID(0, node, tick, name + "_vel1"), node->graph->scale.vel_t);
	pos_cop[0] = new SVar(solver, ID(0, node, tick, name + "_cop0"), node->graph->scale.pos_t);	// cop
	pos_cop[1] = new SVar(solver, ID(0, node, tick, name + "_cop1"), node->graph->scale.pos_t);
	period     = new SVar(solver, ID(0, node, tick, name + "_T"   ), node->graph->scale.time );	// step period
}


//constraints
void BipedLIPKey::AddCon(Solver* solver){
	BipedLIPKey* nextObj = (BipedLIPKey*)next;
	BipedLIPKey* prevObj = (BipedLIPKey*)prev;


	if(next){
		con_lip_pos[0]     = new LIPConP  (solver, name + "_lip_p0", this, 0, node->graph->scale.pos_t);
		con_lip_pos[1]     = new LIPConP  (solver, name + "_lip_p1", this, 1, node->graph->scale.pos_t);
		con_lip_vel[0]     = new LIPConV  (solver, name + "_lip_v0", this, 0, node->graph->scale.vel_t);
		con_lip_vel[1]     = new LIPConV  (solver, name + "_lip_v1", this, 1, node->graph->scale.vel_t);
		con_range_period   = new RangeConS(solver, ID(0, node, tick, name + "_range_period"), period, node->graph->scale.time);
		con_diff_cop[0][0] = new DiffConS (solver, ID(0, node, tick, name + "_range_cop0x" ), pos_t[0], pos_cop[0], node->graph->scale.pos_t);
		con_diff_cop[0][1] = new DiffConS (solver, ID(0, node, tick, name + "_range_cop0y" ), pos_t[1], pos_cop[1], node->graph->scale.pos_t);
		con_diff_cop[1][0] = new DiffConS (solver, ID(0, node, tick, name + "_range_cop1x" ), nextObj->pos_t[0], pos_cop[0], node->graph->scale.pos_t);
		con_diff_cop[1][1] = new DiffConS (solver, ID(0, node, tick, name + "_range_cop1y" ), nextObj->pos_t[1], pos_cop[1], node->graph->scale.pos_t);
		//con_acc[0][0]      = new MatchConS(solver, ID(0, node, tick, name + "_acc0x"       ), pos_t[0], pos_cop[0], node->graph->scale.pos_t);
		//con_acc[0][1]      = new MatchConS(solver, ID(0, node, tick, name + "_acc0y"       ), pos_t[1], pos_cop[1], node->graph->scale.pos_t);
		//con_acc[1][0]      = new MatchConS(solver, ID(0, node, tick, name + "_acc1x"       ), nextObj->pos_t[0], pos_cop[0], node->graph->scale.pos_t);
		//con_acc[1][1]      = new MatchConS(solver, ID(0, node, tick, name + "_acc1y"       ), nextObj->pos_t[1], pos_cop[1], node->graph->scale.pos_t);
		//con_acc[0][0]->SetPriority(1);
		//con_acc[0][1]->SetPriority(1);
		//con_acc[1][0]->SetPriority(1);
		//con_acc[1][1]->SetPriority(1);
		con_proh_cop		= new RevRangeConS(solver, ID(0, node, tick, name + "_step_test"), pos_cop[0], pos_cop[1], node->graph->scale.pos_t);
		con_move_cop		= new RevMoveConS(solver, ID(0, node, tick, name + "_step_test"), pos_cop[0], pos_cop[1], period, node->graph->scale.pos_t);
	}
	
	// 遊脚軌道の中間評価
		if (next && prev){
			con_proh_mid= new MidRangeConS(solver, ID(0, node, tick, name + "_step_test"),prevObj->pos_cop[0], prevObj->pos_cop[1], nextObj->pos_cop[0], nextObj->pos_cop[1], node->graph->scale.pos_t);
		}

	con_fix_pos[0] = new FixConS(solver, ID(0, node, tick, name + "_fix_pos0"), pos_t[0], node->graph->scale.pos_t);
	con_fix_pos[1] = new FixConS(solver, ID(0, node, tick, name + "_fix_pos1"), pos_t[1], node->graph->scale.pos_t);
	con_fix_vel[0] = new FixConS(solver, ID(0, node, tick, name + "_fix_vel0"), vel_t[0], node->graph->scale.vel_t);
	con_fix_vel[1] = new FixConS(solver, ID(0, node, tick, name + "_fix_vel1"), vel_t[1], node->graph->scale.vel_t);
	con_fix_pos[0]->enabled = false;
	con_fix_pos[1]->enabled = false;
	con_fix_vel[0]->enabled = false;
	con_fix_vel[1]->enabled = false;
}

void BipedLIPKey::Prepare(){
	if(!prev){
		tick->time = 0.0;
	}	
	else{
		BipedLIPKey* prevObj = (BipedLIPKey*)prev;
		tick->time = prevObj->tick->time + prevObj->period->val;
	}
}

void BipedLIPKey::Draw(DrawCanvas* canvas, DrawConfig* conf){
	
	canvas->SetPointSize(3.0f);
	canvas->Point(Vec3f((float)pos_cop[0]->val, (float)pos_cop[1]->val, 0.0f));

	canvas->SetPointSize(3.0f);
	canvas->Point(Vec3f((float)pos_t[0]->val, (float)pos_t[1]->val, (float)GetNode()->param.heightCoM));

	canvas->SetLineWidth(1.0f);
	canvas->Line(Vec3f((float)pos_cop[0]->val, (float)pos_cop[1]->val, 0.0f) , Vec3f((float)pos_t[0]->val, (float)pos_t[1]->val, (float)GetNode()->param.heightCoM));

	if(next){
	BipedLIPKey* nextObj = (BipedLIPKey*)next;
	canvas->Line(Vec3f((float)pos_cop[0]->val, (float)pos_cop[1]->val, 0.0f) , Vec3f((float)nextObj->pos_t[0]->val, (float)nextObj->pos_t[1]->val, (float)GetNode()->param.heightCoM));
	}
}



BipedLIP::Param::Param(){
	gravity        = 9.8;
	massTorso      = 1.0;
	massLeg        = 1.0;
	heightCoM      = 0.5;
	legCoMRatio    = 0.5;
	swingVelMax    = 1.0;
	swingHeight[0] = 0.05;
	swingHeight[1] = 0.01;
	stepPeriodMin  = 0.1;
	stepPeriodMax  = 1.0;
	//initialPos     = vec2_t();
	//initialVel     = vec2_t();
	//terminalPos    = vec2_t();
	//terminalVel    = vec2_t();
	supportMin[0]  = vec2_t(-0.3, -0.2 );  //< 初期(-0.3,-0.2 ) <歩幅 of Left Phase	
	supportMax[0]  = vec2_t( 0.3, -0.05);  //< 初期( 0.3,-0.05)
	supportMin[1]  = vec2_t(-0.3,  0.05);  //< 初期(-0.3, 0.05) <歩幅 of Right Phase
	supportMax[1]  = vec2_t( 0.3,  0.2 );  //< 初期( 0.3, 0.2 )
}

BipedLIP::BipedLIP(Graph* g, string n):TrajectoryNode(g, n){
}


// memo 初期に拘束の値を代入するのみ
void BipedLIP::Init(){
	TrajectoryNode::Init();

	// 描写初期時刻
	tm = 0.0 ;
	start = 0;

	param.T = sqrt(param.heightCoM/param.gravity);

	for(uint k = 0; k < graph->ticks.size(); k++){
		BipedLIPKey* key = GetKeypoint(graph->ticks[k]);
		
		std::map<int, int>::iterator it = param.phase.find(k);
		
		if(it == param.phase.end())
			 key->phase = BipedLIP::Left;
		else key->phase = it->second;

		// 初期位置のfix
		if(!key->prev){
			key->pos_t[0]->Lock();
			key->pos_t[1]->Lock();
			key->vel_t[0]->Lock();
			key->vel_t[1]->Lock();
		}
		
		if (key->next){
			key->period->val = (param.stepPeriodMin + param.stepPeriodMax)/2;
			key->con_range_period->_min = param.stepPeriodMin;
			key->con_range_period->_max = param.stepPeriodMax;
					
			bool lr = (key->phase == Left ? 0 : 1);			
			
			for(int i = 0; i < 2; i++){
				key->con_diff_cop[i][0]->_min = param.supportMin[lr].x;	
				key->con_diff_cop[i][0]->_max = param.supportMax[lr].x;
				key->con_diff_cop[i][1]->_min = param.supportMin[lr].y;	
				key->con_diff_cop[i][1]->_max = param.supportMax[lr].y;
		}

			// (add) prohibited area
			key->con_proh_cop->_prohsize = prohareas.size();
			for (uint i = 0; i < prohareas.size(); i++){
				ProhArea& pa = prohareas[i];
				key->con_proh_cop->_ConCenter[i][0]  = pa.proh_cop[0];
				key->con_proh_cop->_ConCenter[i][1]  = pa.proh_cop[1];
				key->con_proh_cop->_r[i]			 = pa.r;
			}

			// (add) prohibited moving area
			key->con_move_cop->_movesize = moveareas.size();
			for (uint i = 0; i < moveareas.size() ; i++){
				MoveArea& ma = moveareas[i];
				key->con_move_cop->_ConIni[i][0] = ma.proh_move[0];
				key->con_move_cop->_ConIni[i][1] = ma.proh_move[1];
				key->con_move_cop->_r[i]			= ma.r;
				key->con_move_cop->_vx[i]			= ma.vel_move[0];
				key->con_move_cop->_vy[i]			= ma.vel_move[1];
			}

		for(uint i = 0; i < waypoints.size(); i++){
			Waypoint& wp = waypoints[i];
			BipedLIPKey* key = GetKeypoint(graph->ticks[wp.k]);
			key->con_fix_pos[0]->desired = wp.pos[0];
			key->con_fix_pos[1]->desired = wp.pos[1];
			key->con_fix_vel[0]->desired = wp.vel[0];
			key->con_fix_vel[1]->desired = wp.vel[1];
			key->con_fix_pos[0]->enabled = true;
			key->con_fix_pos[1]->enabled = true;
			key->con_fix_vel[0]->enabled = true;
			key->con_fix_vel[1]->enabled = true;
		
			}

		// 重心位置，重心速度，着地位置の初期値を終端位置への線形補間で与える

		Waypoint* wp = &waypoints[0];
		for(int k = 0; k < (int)graph->ticks.size(); k++){
			BipedLIPKey* key = GetKeypoint(graph->ticks[k]);
			if(wp[1].k < k)
				wp++;
		
			real_t s = (real_t)(k - wp[0].k)/(real_t)(wp[1].k - wp[0].k);
			key->pos_t  [0]->val = (1-s)*wp[0].pos[0] + s*wp[1].pos[0];
			key->pos_t  [1]->val = (1-s)*wp[0].pos[1] + s*wp[1].pos[1];
			key->vel_t  [0]->val = (1-s)*wp[0].vel[0] + s*wp[1].vel[0];
			key->vel_t  [1]->val = (1-s)*wp[0].vel[1] + s*wp[1].vel[1];
			key->pos_cop[0]->val = (1-s)*wp[0].pos[0] + s*wp[1].pos[0];
			key->pos_cop[1]->val = (1-s)*wp[0].pos[1] + s*wp[1].pos[1];
			}	
		}
		
		
		// 遊脚軌道評価
		if (key->prev && key->next){
			key->con_proh_mid->_prohsize = prohareas.size();
			for (uint i = 0; i < prohareas.size(); i++){
				ProhArea& pa = prohareas[i];
				key->con_proh_mid->_Cen[i][0]  = pa.proh_cop[0]; //test_syougai;
				key->con_proh_mid->_Cen[i][1]  = pa.proh_cop[1];
				key->con_proh_mid->_r[i]	   = pa.r;
			}
		}
	}
}

void BipedLIP::Prepare(){
	TrajectoryNode::Prepare();
	trajReady = false;
}

void BipedLIP::SetPhase(int step, int phase){
	param.phase[step] = phase;
}

void BipedLIP::AddWaypoint(int step, vec2_t pos, vec2_t vel){
	Waypoint wp;
	wp.k   = step;
	wp.pos = pos;
	wp.vel = vel;
	waypoints.push_back(wp);
}

// add(prohibited area)
void BipedLIP::AddProhArea(vec2_t proh_cop, real_t proh_r){
	ProhArea pa;
	pa.proh_cop = proh_cop;
	pa.r		= proh_r;
	prohareas.push_back(pa);
	int prosize = prohareas.size();
}

// add(moving area)
void BipedLIP::AddMoveArea(vec2_t proh_move, real_t proh_r,vec2_t vel_move){
	MoveArea ma;
	ma.proh_move = proh_move;
	ma.r		 = proh_r;
	ma.vel_move  = vel_move;
	moveareas.push_back(ma);
	int movesize = moveareas.size();
}

//-----------------------------------------------------------------------------------------

vec3_t BipedLIP::PosCoM(real_t t){
	BipedLIPKey* key = GetSegment(t).first;

	real_t T    = param.T;
	real_t t0   = key->tick->time;
	real_t px   = key->pos_t[0]->val;		
	real_t py   = key->pos_t[1]->val;
	real_t vx   = key->vel_t[0]->val;		
	real_t vy   = key->vel_t[1]->val;
	real_t copx = key->pos_cop[0]->val;	
	real_t copy = key->pos_cop[1]->val;

	vec3_t p;
	p.x = copx + (px - copx) * cosh((t-t0)/T)  + (vx*T) * sinh((t-t0)/T)  ;
	p.y = copy + (py - copy) * cosh((t-t0)/T)  + (vy*T) * sinh((t-t0)/T)  ;
	p.z = param.heightCoM;

	return p;		
}

//------------------------------------------------------------------------------------

vec3_t BipedLIP::VelCoM(real_t t){
	BipedLIPKey* key = GetSegment(t).first;

	real_t T    = param.T;
	real_t t0   = key->tick->time;
	real_t px   = key->pos_t[0]->val;
	real_t py   = key->pos_t[1]->val;
	real_t vx   = key->vel_t[0]->val;
	real_t vy   = key->vel_t[1]->val;
	real_t copx = key->pos_cop[0]->val;
	real_t copy = key->pos_cop[1]->val;

	vec3_t v;
	v.x = ((px-copx)/T) * sinh((t-t0)/T) + (vx) * cosh((t-t0)/T);
	v.y = ((py-copy)/T) * sinh((t-t0)/T) + (vy) * cosh((t-t0)/T);
	v.z = 0.0;

	return v;
}

//-------------------------------------------------------------------------------------------

vec3_t BipedLIP::AccCoM(real_t t){
	BipedLIPKey* key = GetSegment(t).first;

	real_t T    = param.T;
	real_t copx = key->pos_cop[0]->val;
	real_t copy = key->pos_cop[1]->val;
	vec3_t p    = PosCoM(t);
	vec3_t a;
	a.x = (p.x - copx)/(T*T);
	a.y = (p.y - copy)/(T*T);
	a.z = 0.0;

	return a;
}

// memo　接地点を呼び出す（返す）

vec3_t BipedLIP::PosCoP(real_t t){
	BipedLIPKey* key = GetSegment(t).first;
	if(!key->next)
		key = (BipedLIPKey*)key->prev;

	return vec3_t(key->pos_cop[0]->val, key->pos_cop[1]->val, 0.0);

}


vec3_t BipedLIP::PosTorso(const vec3_t& pcom, const vec3_t& psup, const vec3_t& pswg){
	real_t mt   = param.massTorso;
	real_t ml   = param.massLeg;
	real_t beta = param.legCoMRatio;
	vec3_t p = ( (mt+2*ml)*pcom - (1-beta)*ml*(psup+pswg) ) / (mt+2*beta*ml);
	return p;
}


vec3_t BipedLIP::PosSupportFoot(real_t t){
	return PosCoP(t);
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

	// 前後の接地点
	vec2_t p0, p1;
	p0[0] = (prev ? prev : cur)->pos_cop[0]->val;
	p0[1] = (prev ? prev : cur)->pos_cop[1]->val;
	p1[0] = (next->next ? next : cur)->pos_cop[0]->val;
	p1[1] = (next->next ? next : cur)->pos_cop[1]->val;


	// 遊脚のスタートは支持位置の対称線上にする
	if ( t0 == 0){
		p0[0] = 0.0;
		p0[1] = - p0[1];
	}


	// 最後の遊脚接地点は支持脚と重心位置との対称線上
	if (traj.back()->tick->time <= t1){
		vec2_t com;
		com[0] = next->pos_t[0]->val;
		com[1] = next->pos_t[1]->val;
	
		// 延長線上への更新
		p1[0] = com[0] ;
		p1[1] = 2 * com[1] - p1[1];
	}

	// cout <<"t0:" << t0 << ",  p0:(" << p0[0] << "," << p0[1]  << "),  p1:(" << p1[0] << "," << p1[1]<<") \n" ;

	// 高さの遷移
	real_t z;
	if(t < t0 + hhalf){
		z = param.swingHeight[0];
	}
	else{
		real_t s = (t - (t0+hhalf)) / hhalf;
		z = (1-s)*param.swingHeight[0] + s*param.swingHeight[1];
	}

	real_t s = (t - t0) / h;
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
		tp.t      = t;
		tp.pcom   = PosCoM(t);
		tp.pcop   = PosCoP(t);
		tp.pswing = PosSwingFoot(t);
		tp.ptorso = PosTorso(tp.pcom, tp.pcop, tp.pswing);

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
		canvas->SetLineWidth(3.0f);
		canvas->BeginPath();
		canvas->MoveTo(trajectory[0].pcom);
		for(uint i = 1; i < trajectory.size(); i++){
			canvas->LineTo(trajectory[i].pcom);
		}
		canvas->EndPath();
		canvas->EndLayer();
	}
	//// torso
	//	if(conf->Set(canvas, DrawItem::BipedTorso, this)){
	//		canvas->BeginLayer("biped_torso", true);
	//		canvas->SetLineWidth(3.0f);
	//		canvas->BeginPath();
	//		canvas->MoveTo(trajectory[0].ptorso);
	//		for(uint i = 1; i < trajectory.size(); i++){
	//			canvas->LineTo(trajectory[i].ptorso);
	//		}
	//		canvas->EndPath();
	//		canvas->EndLayer();
	//	}
	// swing foot
	//if(conf->Set(canvas, DrawItem::BipedSwing, this)){
	//	canvas->BeginLayer("biped_swing", true);
	//	canvas->SetLineWidth(3.0f);
	//	canvas->BeginPath();
	//	canvas->MoveTo(trajectory[0].pswing);
	//	for(uint i = 1; i < trajectory.size(); i++){
	//		if(trajectory[i-1].pcop == trajectory[i].pcop){
	//			canvas->LineTo(trajectory[i].pswing);
	//		}
	//		else{
	//			canvas->EndPath();
	//			canvas->BeginPath();
	//			canvas->MoveTo(trajectory[i].pswing);
	//		}
	//	}
	//	canvas->EndPath();
	//	canvas->EndLayer();
	//}
	//// double support snapshot
	//if(conf->Set(canvas, DrawItem::BipedDouble, this)){
	//	canvas->BeginLayer("biped_double", true);
	//	canvas->SetLineWidth(1.0f);
	//	for(uint i = 1; i < trajectory.size(); i++){
	//		if(trajectory[i-1].pcop != trajectory[i].pcop){
	//			canvas->Line(trajectory[i].ptorso, trajectory[i  ].pcop);
	//			canvas->Line(trajectory[i].ptorso, trajectory[i-1].pcop);
	//		}
	//	}
	//	canvas->EndLayer();
	//}
	
	// draw prohibited area
	for (uint i = 0; i < prohareas.size(); i++){
		ProhArea& pa = prohareas[i];
		
		canvas->SetLineWidth(2.0f);
		canvas->Circle(pa.proh_cop, pa.r );
		
	}

	// 動く様子の描写を行う
	if(move_trajectory){
		if (tm > traj.back()->tick->time - 0.02)
			move_trajectory = false;

		// 描写開始時の時間
		if (start == 0)
			start = clock();

		tm = (float) (clock() - start) / (CLOCKS_PER_SEC * 2);

		BipedLIPKey* key = GetSegment(tm).first;
		// 重心位置
		canvas->SetPointSize(10.0f);
		canvas->Point(Vec3f(PosCoM(tm)));
		
		// 接地位置
		canvas->SetPointSize(10.0f);
		canvas->Point(Vec3f(key->pos_cop[0]->val, key->pos_cop[1]->val, 0.0f));

		// 重心と接地位置を結ぶ 
		canvas->SetLineWidth(5.0f);
		canvas->Line(Vec3f(key->pos_cop[0]->val, key->pos_cop[1]->val, 0.0f) , Vec3f(PosCoM(tm)));


		// 遊脚位置
		canvas->SetPointSize(5.0f);
		canvas->Point(Vec3f(PosSwingFoot(tm)));

		// 重心と遊脚を結ぶ 
		canvas->SetLineWidth(3.0f);
		canvas->Line(Vec3f(PosSwingFoot(tm)), Vec3f(PosCoM(tm)));


		// 動障害物の描写
		canvas->SetLineWidth(2.0f);
		for (uint i = 0; i < moveareas.size() ; i++){
			MoveArea& ma = moveareas[i];
			canvas->Circle(Vec2f(ma.proh_move[0] + tm * ma.vel_move[0],ma.proh_move[1] + tm * ma.vel_move[1]), ma.r-0.01f);
		}		
	}	// if
}



void BipedLIP::DrawSnapshot(real_t time, DrawCanvas* canvas, DrawConfig* conf){
}

void BipedLIP::Save(){
	FILE* file = fopen("plan_step.csv", "w");
	fprintf(file, "step, time, period, pos_com_x, pos_com_y, vel_com_x, vel_com_y, pos_cop_x, pos_cop_y\n");

	real_t t = 0.0;
	for(uint k = 0; k < graph->ticks.size(); k++){
		BipedLIPKey* key = (BipedLIPKey*)GetKeypoint(graph->ticks[k]);

		fprintf(file, "%d, %3.3lf, %3.3lf, %3.3lf, %3.3lf, %3.3lf, %3.3lf, %3.3lf, %3.3lf\n",
			k, t,
			key->period    ->val,
			key->pos_t  [0]->val, key->pos_t  [1]->val,
			key->vel_t  [0]->val, key->vel_t  [1]->val,
			key->pos_cop[0]->val, key->pos_cop[1]->val);

		t += key->period->val;
	}

	fclose(file);
}


//-------------------------------------------------------------------------------------------------
// Constructors

// memo objの追加（接地点の前後）
LIPCon::LIPCon(Solver* solver, string _name, BipedLIPKey* _obj, uint _idx, real_t _scale):
	Constraint(solver, 1, ID(0, _obj->node, _obj->tick, _name), _scale){
	obj[0] = _obj;
	obj[1] = (BipedLIPKey*)_obj->next;
	idx    = _idx;
}

	// 前接地点の状態と次の接地点位置を定義
LIPConP::LIPConP(Solver* solver, string _name, BipedLIPKey* _obj, uint _idx, real_t _scale):
	LIPCon(solver, _name, _obj, _idx, _scale){

	AddSLink(obj[0]->pos_t  [idx]);
	AddSLink(obj[0]->vel_t  [idx]);
	AddSLink(obj[0]->pos_cop[idx]);
	AddSLink(obj[0]->period      );
	AddSLink(obj[1]->pos_t  [idx]);
}

	// 速度について
LIPConV::LIPConV(Solver* solver, string _name, BipedLIPKey* _obj, uint _idx, real_t _scale):
	LIPCon(solver, _name, _obj, _idx, _scale){
	AddSLink(obj[0]->pos_t  [idx]);
	AddSLink(obj[0]->vel_t  [idx]);
	AddSLink(obj[0]->pos_cop[idx]);
	AddSLink(obj[0]->period	     );
	AddSLink(obj[1]->vel_t  [idx]);
}



//-------------------------------------------------------------------------------------------------

	// 変数の用意
void LIPCon::Prepare(){
	BipedLIP::Param& param = obj[0]->GetNode()->param;
	T  = param.T;					// 時定数
	t  = obj[0]->period->val/T;		// tau(single support period)/t
	ch = cosh(t);
	sh = sinh(t);
	p0 = obj[0]->pos_t  [idx]->val;
	v0 = obj[0]->vel_t  [idx]->val;
	c  = obj[0]->pos_cop[idx]->val;
	p1 = obj[1]->pos_t  [idx]->val;
	v1 = obj[1]->vel_t  [idx]->val;
}


// 倒立振子を表現（それぞれの要素に掛けあわされている係数）
void LIPConP::CalcCoef(){
	Prepare();
	((SLink*)links[0])->SetCoef(-ch);									// AddSLink(obj[0]->pos_t  [idx]);
	((SLink*)links[1])->SetCoef(-T * sh);								// AddSLink(obj[0]->vel_t  [idx]);
	((SLink*)links[2])->SetCoef(ch - 1.0);								// AddSLink(obj[0]->pos_cop[idx]);
	((SLink*)links[3])->SetCoef(- ((p0-c)/T)*sh - v0*ch);				// AddSLink(obj[0]->period      );
	((SLink*)links[4])->SetCoef(1.0);									// AddSLink(obj[1]->pos_t  [idx]);
}

void LIPConV::CalcCoef(){
	Prepare();
	((SLink*)links[0])->SetCoef(-(1/T)*sh);								// AddSLink(obj[0]->pos_t  [idx]);
	((SLink*)links[1])->SetCoef(-ch);									// AddSLink(obj[0]->vel_t  [idx]);
	((SLink*)links[2])->SetCoef( (1/T)*sh);								// AddSLink(obj[0]->pos_cop[idx]);
	((SLink*)links[3])->SetCoef(- ((p0-c)/(T*T))*ch - (v0/T)*sh);		// AddSLink(obj[0]->period	    );
	((SLink*)links[4])->SetCoef(1.0);									// AddSLink(obj[1]->vel_t  [idx]);
}

//-------------------------------------------------------------------------------------------------

// memo p1を次の目標位置に近づける
//		（p1と次の接地位置となる右式との差を小さくする働き）

void LIPConP::CalcDeviation(){
	y[0] = p1 - ( c + (p0-c)*ch + (v0*T)*sh );		
}

void LIPConV::CalcDeviation(){
	y[0] = v1 - ( ((p0-c)/T)*sh + v0*ch );			
}



// (add) about prohbited area
//----------------------------------------------------------------------------------------------------------------------
// add 11/6 制約内にあるときに誤差が増大する関数とする 
//----------------------------------------------------------------------------------------------------------------------

RevRangeConS::RevRangeConS(Solver* solver, ID id, SVar* var0, SVar* var1,real_t _scale):Constraint(solver, 1, id, _scale){
	AddSLink(var0, 0.0);
	AddSLink(var1, 1.0);
//	real_t inf = numeric_limits<real_t>::max();
}


void RevRangeConS::CalcCoef(){
	//((SLink*)links[0])->SetCoef(2.0*y[0]);
}
void RevRangeConS::CalcDeviation(){
	real_t pos_x = ((SVar*)links[0]->var)->val;
	real_t pos_y = ((SVar*)links[1]->var)->val;
	active = false ;	// 初期化

	for ( int i = 0; i < _prohsize ; i++){
		real_t dist = (pos_x - _ConCenter[i][0]) * (pos_x - _ConCenter[i][0]) + (pos_y - _ConCenter[i][1]) * (pos_y - _ConCenter[i][1]);

		//cout << "pos_x :" << pos_x << ",  _ConCenter.x : " << _ConCenter.x << "\n" ;
		//cout << "pos_y :" << pos_y << ",  _ConCenter.y : " << _ConCenter.y << ",  dist : " << dist << "\n";


		if ( dist < (_r[i] * _r[i]) ){
			active = true;
			if (_ConCenter[i][1] > 0)
				y[0] = abs (dist - (_r[i] * _r[i])) ;
			else
				y[0] = - abs (dist - (_r[i] * _r[i])) ;
		}
	}
}

// 修正量(l)の修正
void RevRangeConS::Project(real_t& l, uint k){	// active のときだけ働く
	if (y[0] > 0 && l > 0)
		l = 0;
	if (y[0] < 0 && l < 0)
		l = 0;
}




// (add) about prohbited area about swing leg
//----------------------------------------------------------------------------------------------------------------------
// add 11/22 遊脚軌道に対する接地不可制約 
//----------------------------------------------------------------------------------------------------------------------

MidRangeConS::MidRangeConS(Solver* solver, ID id, SVar* var0, SVar* var1, SVar* var2, SVar* var3, real_t _scale):Constraint(solver, 1, id, _scale){
	AddSLink(var0, 0.0);
	AddSLink(var1, 5.0);
	AddSLink(var2, 0.0);
	AddSLink(var3, 5.0);

//	real_t inf = numeric_limits<real_t>::max();
}


void MidRangeConS::CalcCoef(){
	//((SLink*)links[0])->SetCoef(2.0*y[0]);
}
void MidRangeConS::CalcDeviation(){
	real_t a1 = ((SVar*)links[0]->var)->val;
	real_t b1 = ((SVar*)links[1]->var)->val;
	real_t a2 = ((SVar*)links[2]->var)->val;
	real_t b2 = ((SVar*)links[3]->var)->val;
	active = false ;	// 初期化

	
	for ( int i = 0; i < _prohsize ; i++){
		
		// どちらかの座標が円横を通るときに判定
		if ((_Cen[i][0] - _r[i]< a1 &&  a1 < _Cen[i][0] + _r[i] )||(_Cen[i][0] - _r[i] < a2 &&  a2 < _Cen[i][0] + _r[i])){
			real_t dist = abs((b2-b1)*_Cen[i][0] + (a1-a2)*_Cen[i][1] + a2*b1 - a1*b2) / sqrt((b2-b1)*(b2-b1) + (a1-a2)*(a1-a2));	
			if ( dist < _r[i]){
				active = true;
				if (_Cen[i][1] > 0)
					y[0] = abs (dist - _r[i]) ;
				else
					y[0] = - abs (dist - _r[i]) ;
			}
		}
	}
}

// 修正量(l)の修正
void MidRangeConS::Project(real_t& l, uint k){	// active のときだけ働く
	if (y[0] > 0 && l > 0)
		l = 0;
	if (y[0] < 0 && l < 0)
		l = 0;
}



// 
//----------------------------------------------------------------------------------------------------------------------
// add 12/2 動障害物に対する評価 
//----------------------------------------------------------------------------------------------------------------------

RevMoveConS::RevMoveConS(Solver* solver, ID id, SVar* var0, SVar* var1, SVar* var2, real_t _scale):Constraint(solver, 1, id, _scale){
	AddSLink(var0, 0.0);
	AddSLink(var1, 1.0);
	AddSLink(var2, 0.0); // period
//	real_t inf = numeric_limits<real_t>::max();
}


void RevMoveConS::CalcCoef(){
	//((SLink*)links[0])->SetCoef(2.0*y[0]);
}
void RevMoveConS::CalcDeviation(){
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
			if(_ConIni[i][1] > 0)
				y[0] = abs (dist - r_n) ;
			else
				y[0] =  - abs (dist - r_n) ;
		}
		
	}	// for

}	// calc diviation

// 修正量(l)の修正
void RevMoveConS::Project(real_t& l, uint k){	// active のときだけ働く
	if (y[0] > 0 && l > 0)
		l = 0;
	if (y[0] < 0 && l < 0)
		l = 0;
}

}


