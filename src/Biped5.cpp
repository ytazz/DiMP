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

	pos_com[0] = new SVar(solver, ID(0, node, tick, name + "_pos0"  ), node->graph->scale.pos_t);
	pos_com[1] = new SVar(solver, ID(0, node, tick, name + "_pos1"  ), node->graph->scale.pos_t);
	vel_com[0] = new SVar(solver, ID(0, node, tick, name + "_vel0"  ), node->graph->scale.vel_t);
	vel_com[1] = new SVar(solver, ID(0, node, tick, name + "_vel1"  ), node->graph->scale.vel_t);
	// N歩目のcopは遊脚軌道の終点として必要
	pos_cop[0] = new SVar(solver, ID(0, node, tick, name + "_cop0"  ), node->graph->scale.pos_t);
	pos_cop[1] = new SVar(solver, ID(0, node, tick, name + "_cop1"  ), node->graph->scale.pos_t);
	ang_com	   = new SVar(solver, ID(0, node, tick, name + "_ang "  ), node->graph->scale.pos_r);
	angvel_com = new SVar(solver, ID(0, node, tick, name + "_angvel"), node->graph->scale.vel_r);

	// 0歩目の遊脚始点
	if(!prev){
		pos_swg[0] = new SVar(solver, ID(0, node, tick, name + "_swg0"), node->graph->scale.pos_t);
		pos_swg[1] = new SVar(solver, ID(0, node, tick, name + "_swg1"), node->graph->scale.pos_t);
	}

	if(next){
		period     = new SVar(solver, ID(0, node, tick, name + "_T"   ), node->graph->scale.time );
	}
}

void BipedLIPKey::AddCon(Solver* solver){
	BipedLIPKey* nextObj = (BipedLIPKey*)next;
	BipedLIPKey* prevObj = (BipedLIPKey*)prev;
	
	if(next){
		con_lip_pos[0]       = new LIPConP  (solver, name + "_lip_p0", this, 0, node->graph->scale.pos_t);
		con_lip_pos[1]       = new LIPConP  (solver, name + "_lip_p1", this, 1, node->graph->scale.pos_t);
		con_lip_vel[0]       = new LIPConV  (solver, name + "_lip_v0", this, 0, node->graph->scale.vel_t);
		con_lip_vel[1]       = new LIPConV  (solver, name + "_lip_v1", this, 1, node->graph->scale.vel_t);
		con_range_period     = new RangeConS(solver, ID(0, node, tick, name + "_range_period"), period, node->graph->scale.time);
		con_diff_cop[0][0]   = new RangeConCOP (solver, ID(0, node, tick, name + "_range_cop0x" ), pos_com[0], pos_com[1], pos_cop[0], pos_cop[1], ang_com, 0, node->graph->scale.pos_t);
		con_diff_cop[0][1]   = new RangeConCOP (solver, ID(0, node, tick, name + "_range_cop0x" ), pos_com[0], pos_com[1], pos_cop[0], pos_cop[1], ang_com, 1, node->graph->scale.pos_t);
		con_diff_cop[1][0]   = new RangeConCOP (solver, ID(0, node, tick, name + "_range_cop0x" ), nextObj->pos_com[0], nextObj->pos_com[1], pos_cop[0], pos_cop[1], nextObj->ang_com, 0, node->graph->scale.pos_t);
		con_diff_cop[1][1]   = new RangeConCOP (solver, ID(0, node, tick, name + "_range_cop0x" ), nextObj->pos_com[0], nextObj->pos_com[1], pos_cop[0], pos_cop[1], nextObj->ang_com, 1, node->graph->scale.pos_t);
		con_range_angaccL    = new RangeConAccL(solver, ID(0, node, tick, name + "_range_angacc"), ang_com, angvel_com, period, nextObj->ang_com, nextObj->angvel_com, node->graph->scale.acc_r); //追加
		con_range_angaccR    = new RangeConAccR(solver, ID(0, node, tick, name + "_range_angacc"), ang_com, angvel_com, period, nextObj->ang_com, nextObj->angvel_com, node->graph->scale.acc_r); //追加
		con_range_angle      = new DiffConAng(solver, ID(0, node, tick, name + "_range_angle"), ang_com, nextObj->ang_com, node->graph->scale.pos_r);
	}

	if(!next){
		con_range_angaccLf = new RangeConAccL(solver, ID(0, node, tick, name + "_range_angacc"), prevObj->ang_com, prevObj->angvel_com, prevObj->period, ang_com, angvel_com, node->graph->scale.acc_r); //追加
		con_range_angaccRf = new RangeConAccR(solver, ID(0, node, tick, name + "_range_angacc"), prevObj->ang_com, prevObj->angvel_com, prevObj->period, ang_com, angvel_com, node->graph->scale.acc_r); //追加
		
		//最終フェーズ左脚用拘束
		//n-1歩目とn歩目の支持脚の接地点の中点がn歩目の重心位置となるようにする拘束
		con_fix_pos_copL[0] = new FixConPCOP(solver, name + "_pos_cop0", this, 0, 0, node->graph->scale.pos_t);
		con_fix_pos_copL[1] = new FixConPCOP(solver, name + "_pos_cop1", this, 1, 0, node->graph->scale.pos_t);
		//n-1歩目の支持脚の接地位置とn歩目の重心位置を結ぶ線分がn歩目の胴体角度方向と直交するための条件
		con_pos_to_angL [0] = new FixConPtoA(solver, name + "_ang_com" , this, 0, 0, node->graph->scale.pos_r);
		con_pos_to_angL [1] = new FixConPtoA(solver, name + "_ang_com" , this, 1, 0, node->graph->scale.pos_r);


		//最終フェーズ右脚用拘束
		//n-1歩目とn歩目の支持脚の接地点の中点がn歩目の重心位置となるようにする拘束
		con_fix_pos_copR[0] = new FixConPCOP(solver, name + "_pos_cop0", this, 0, 1, node->graph->scale.pos_t);
		con_fix_pos_copR[1] = new FixConPCOP(solver, name + "_pos_cop1", this, 1, 1, node->graph->scale.pos_t);
		//n-1歩目の支持脚の接地位置とn歩目の重心位置を結ぶ線分がn歩目の胴体角度方向と直交するための条件
		con_pos_to_angR [0] = new FixConPtoA(solver, name + "_ang_com" , this, 0, 1, node->graph->scale.pos_r);
		con_pos_to_angR [1] = new FixConPtoA(solver, name + "_ang_com" , this, 1, 1, node->graph->scale.pos_r);

		con_fix_pos_copL[0]->enabled = false;
		con_fix_pos_copL[1]->enabled = false;
		con_pos_to_angL [0]->enabled = false;
		con_pos_to_angL [1]->enabled = false;
		con_fix_pos_copR[0]->enabled = false;
		con_fix_pos_copR[1]->enabled = false;
		con_pos_to_angR [0]->enabled = false;
		con_pos_to_angR [1]->enabled = false;
	}
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
	canvas->SetPointSize(8.0f);
	canvas->Point(Vec3f((float)pos_com[0]->val, (float)pos_com[1]->val, (float)GetNode()->param.heightCoM));

	if(next){
		canvas->SetPointSize(13.0f);
		canvas->Point(Vec3f((float)pos_cop[0]->val, (float)pos_cop[1]->val, 0.0f));
	}
}

BipedLIP::Param::Param(){
	gravity        = 9.8;
	massTorso      = 1.0;
	massLeg        = 1.0;
	heightCoM      = 0.5;
	legCoMRatio    = 0.5;
	swingHeight[0] = 0.05;
	swingHeight[1] = 0.01;
	stepPeriodMin  = 0.1;
	stepPeriodMax  = 1.0;
	supportMin[0]  = vec2_t(-0.3, -0.2 );  //< 初期(-0.3,-0.2 ) <歩幅 of Left Phase	
	supportMax[0]  = vec2_t( 0.3, -0.05);  //< 初期( 0.3,-0.05)
	supportMin[1]  = vec2_t(-0.3,  0.05);  //< 初期(-0.3, 0.05) <歩幅 of Right Phase
	supportMax[1]  = vec2_t( 0.3,  0.2 );  //< 初期( 0.3, 0.2 )
	AngAccMin	   =-0.3;
	AngAccMax      = 0.3;
    //tmargin        = 0.0;
	AngleMax	   = 0.52;
}

BipedLIP::Waypoint::Waypoint(){
	k           = 0;
	fix_pos_com    = false;
	fix_vel_com    = false;
	fix_pos_cop    = false;
	fix_pos_swg    = false;
	fix_ang_com    = false;
	fix_angvel_com = false;
}

BipedLIP::TrajPoint::TrajPoint(){
	t = 0.0f;
}

BipedLIP::BipedLIP(Graph* g, string n):TrajectoryNode(g, n){
}

void BipedLIP::Init(){
	TrajectoryNode::Init();

	// 描写初期時刻
	tm = 0.0 ;
	start = 0;

	//遊脚の振り上げのタイミングをずらす割合
	//param.tmargin = 0.5;

	param.T = sqrt(param.heightCoM/param.gravity);

	real_t periodAve = (param.stepPeriodMin + param.stepPeriodMax) / 2.0;
	 
	for(uint k = 0; k < graph->ticks.size(); k++){
		BipedLIPKey* key = GetKeypoint(graph->ticks[k]);
		
		key->phase = phase[k];

		if (key->next){
			// 周期の初期値は下限と上限の中間値
			key->period->val = periodAve;
			key->con_range_period->_min = param.stepPeriodMin;
			key->con_range_period->_max = param.stepPeriodMax;

			//ステップ開始時の角加速度制限
			key->con_range_angaccL->_min = param.AngAccMin;
			key->con_range_angaccL->_max = param.AngAccMax;

			//ステップ終了時の角加速度制限
			key->con_range_angaccR->_min = param.AngAccMin;
			key->con_range_angaccR->_max = param.AngAccMax;

			//1ステップ中の胴体角度の変化量の範囲制約
			key->con_range_angle->_max = param.AngleMax;
			
			bool lr = (key->phase == Phase::Left ? 0 : 1);			
						
			for(int i = 0; i < 2; i++){
				key->con_diff_cop[i][0]->_min = param.supportMin[lr].x;
				key->con_diff_cop[i][0]->_max = param.supportMax[lr].x;
				key->con_diff_cop[i][1]->_min = param.supportMin[lr].y;
				key->con_diff_cop[i][1]->_max = param.supportMax[lr].y;
			}
		}

		//最終フェーズが右脚か左足かによって最終歩に関する拘束条件を切り替え
		if(!key->next){

			key->con_range_angaccLf->_min = param.AngAccMin;
			key->con_range_angaccLf->_max = param.AngAccMax;
			key->con_range_angaccRf->_min = param.AngAccMin;
			key->con_range_angaccRf->_max = param.AngAccMax;

			lastphase = phase[graph->ticks.size() - 1];

			if(graph->ticks.size() - 1 >= 3)
				//最終フェーズが左脚
				if(lastphase == 1){
					key->con_fix_pos_copL[0]->enabled = true;
					key->con_fix_pos_copL[1]->enabled = true;
					key->con_pos_to_angL [0]->enabled = true;
					key->con_pos_to_angL [1]->enabled = true;
					key->con_fix_pos_copR[0]->enabled = false;
					key->con_fix_pos_copR[1]->enabled = false;
					key->con_pos_to_angR [0]->enabled = false;
					key->con_pos_to_angR [1]->enabled = false;
				}

				//最終フェーズが右脚
				if(lastphase == 0){
					key->con_fix_pos_copL[0]->enabled = false;
					key->con_fix_pos_copL[1]->enabled = false;
					key->con_pos_to_angL [0]->enabled = false;
					key->con_pos_to_angL [1]->enabled = false;
					key->con_fix_pos_copR[0]->enabled = true;
					key->con_fix_pos_copR[1]->enabled = true;
					key->con_pos_to_angR [0]->enabled = true;
					key->con_pos_to_angR [1]->enabled = true;
				}
			if(graph->ticks.size() - 1  < 3){
				key->con_fix_pos_copL[0]->enabled = false;
				key->con_fix_pos_copL[1]->enabled = false;
				key->con_pos_to_angL [0]->enabled = false;
				key->con_pos_to_angL [1]->enabled = false;
				key->con_fix_pos_copR[0]->enabled = false;
				key->con_fix_pos_copR[1]->enabled = false;
				key->con_pos_to_angR [0]->enabled = false;
				key->con_pos_to_angR [1]->enabled = false;
			}
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
		if(wp.fix_pos_cop && !key->prev){
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
		if(wp.fix_ang_com){
			key->ang_com->val		= wp.ang_com;
			key->ang_com->locked    = true;
		}
		if(wp.fix_angvel_com){
			key->angvel_com->val    = wp.angvel_com;
			key->angvel_com->locked = true;
		}
	}

	// 重心位置，重心速度，着地位置の初期値を経由点を結ぶスプライン曲線で与える
		//重心位置，重心速度，着地位置の補間
		Curve2d curve;
		curve.SetType(Interpolate::Cubic);
		for(uint i = 0; i < waypoints.size(); i++){
			Waypoint& wp = waypoints[i];
			curve.AddPoint(wp.k * periodAve);
			if(!FallAvoidance){
				curve.SetPos(i, wp.pos_com);
				curve.SetVel(i, wp.vel_com);
			}
			if(FallAvoidance){
				vec2_t pos_com_ref;
				if(i == 0){
					pos_com_ref.x = 0.0;
					pos_com_ref.y = 0.0;
				}
				if(i != 0){
					Waypoint& wp0 = waypoints[0];
					pos_com_ref.x = wp0.vel_com.x*periodAve*graph->ticks.size()*(1.0/2.0);
					pos_com_ref.y = wp0.vel_com.y*periodAve*graph->ticks.size()*(1.0/2.0);
				}
				curve.SetPos(i, pos_com_ref);
				curve.SetVel(i, wp.vel_com);
			}
		}
		for(uint k = 0; k < graph->ticks.size(); k++){
			BipedLIPKey* key = GetKeypoint(graph->ticks[k]);
			if(!FallAvoidance){
				vec2_t p = curve.CalcPos(k * periodAve);
				vec2_t v = curve.CalcVel(k * periodAve);
				key->pos_com[0]->val = p[0];
				key->pos_com[1]->val = p[1];
				key->vel_com[0]->val = v[0];
				key->vel_com[1]->val = v[1];
				if(key->next && key->prev){
					p = curve.CalcPos((k+0.5) * periodAve);
					key->pos_cop[0]->val = p[0];
					key->pos_cop[1]->val = p[1];
				}
			}
			if(FallAvoidance){
				vec2_t p = curve.CalcPos(k * periodAve);
				vec2_t v = curve.CalcVel(k * periodAve);
				key->pos_com[0]->val = p[0];
				key->pos_com[1]->val = p[1];
				key->vel_com[0]->val = v[0];
				key->vel_com[1]->val = v[1];
				if(key->next && key->prev){
					p = curve.CalcPos((k+0.5) * periodAve);
					key->pos_cop[0]->val = p[0];
					key->pos_cop[1]->val = p[1];
				}
			}
		}

		//胴体角度，胴体角速度の補間
		Curve2d quat;
		quat.SetType(Interpolate::Cubic);
		for(uint i = 0; i < waypoints.size(); i++){
			Waypoint& wp = waypoints[i];
			quat.AddPoint(wp.k * periodAve);
			if(!FallAvoidance){
				quat.SetPos(i, Vec2d(graph->ticks.size()*periodAve,wp.ang_com));
				quat.SetVel(i, Vec2d(graph->ticks.size()*periodAve,wp.angvel_com));
			}
			if(FallAvoidance){
				real_t ang_com_ref;
				if(i == 0){
					ang_com_ref = 0.0;
				}
				if(i != 0){
					Waypoint& wp0 = waypoints[i-1];
					ang_com_ref = wp0.angvel_com*periodAve*graph->ticks.size()*(1.0/2.0);
				}
				quat.SetPos(i, Vec2d(graph->ticks.size()*periodAve,ang_com_ref  ));
				quat.SetVel(i, Vec2d(graph->ticks.size()*periodAve,wp.angvel_com));
			}
		}
		for(uint k = 0; k < graph->ticks.size(); k++){
			BipedLIPKey* key = GetKeypoint(graph->ticks[k]);
			vec2_t p = quat.CalcPos(k * periodAve);
			vec2_t v = quat.CalcVel(k * periodAve);
			key->ang_com   ->val = p[1];
			key->angvel_com->val = v[1];
		}
	}
 
void BipedLIP::Prepare(){
	TrajectoryNode::Prepare();
	trajReady = false;
}

int BipedLIP::Phase(real_t t){
	int walkingphase=0;
	if( t < traj.back()->tick->time - 0.001f){
		BipedLIPKey* key = GetSegment(t).first;
		walkingphase = key->phase;
	}
	if( t >= traj.back()->tick->time - 0.001f){
		if((phase[0] == 0 && (graph->ticks.size()-1) % 2 == 0) || (phase[0] == 1 && (graph->ticks.size()-1) % 2 == 1))
			walkingphase = 0;
		else
			walkingphase = 1;
	}

	return walkingphase;
}

real_t BipedLIP::Angle0(real_t t){
	BipedLIPKey* key = GetSegment(t).first;
	BipedLIPKey* prev = (BipedLIPKey*) key->prev;
	return prev->ang_com->val;
}

real_t BipedLIP::Angle1(real_t t){
	BipedLIPKey* key = GetSegment(t).first;
	return key->ang_com->val;
}

real_t BipedLIP::Angle2(real_t t){
	BipedLIPKey* key = GetSegment(t).first;
	BipedLIPKey* next= (BipedLIPKey*) key->next;
	return next->ang_com->val;
}

real_t BipedLIP::Period(real_t t){
	BipedLIPKey* key = GetSegment(t).first;
	BipedLIPKey* next = (BipedLIPKey*)key->next;
	real_t tau;
	if(next)
		tau = next->tick->time - key->tick->time;
	else
		tau = traj.back()->tick->time - key->tick->time;

	return tau;

}

//-----------------------------------------------------------------------------------------

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

//------------------------------------------------------------------------------------

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

//-------------------------------------------------------------------------------------------

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
	//real_t t_m	 = /*param.tmargin;*/0.0;
	//real_t h_m   = h * t_m;
	//real_t h_r   = h - h_m; 
	//real_t hhalf = h_m/2.0;
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
		p1[0] = cur->pos_cop[0]->val;
		p1[1] = cur->pos_cop[1]->val;
	}

	real_t s;
	real_t z;
	if(h == 0.0){
		s = 0.0;
		z = 0.0;
	}
	//if(t - t0 > h_m){
	//	s = 0.0;
	//	z = 0.0;
	//}
	//else{
	//		real_t _2pi = 2.0 * M_PI;
	//		real_t tau = (t - (t0 + h_m))/h_r;
	//	
	//		s = (tau - sin(_2pi*tau)/_2pi);
	//		z = (param.swingHeight[0]/2.0) * (1 - cos(_2pi*tau));
	//	}
	//
	//vec3_t p;
	//p[0] = (1-s) * p0[0] + s * p1[0];
	//p[1] = (1-s) * p0[1] + s * p1[1];
	//p[2] = z;

	else{
		real_t _2pi = 2.0 * M_PI;
		real_t tau = (t - t0)/h;
		
		s = (tau - sin(_2pi*tau)/_2pi);
		z = (param.swingHeight[0]/2.0) * (1 - cos(_2pi*tau));
		
	}
	
	vec3_t p;
	p[0] = (1-s) * p0[0] + s * p1[0];
	p[1] = (1-s) * p0[1] + s * p1[1];
	p[2] = z;

	//cout << p << '\n';
	
	return p;
}

vec3_t BipedLIP::AngCoM(real_t t){
	BipedLIPKey* key = GetSegment(t).first;
	BipedLIPKey* next = (BipedLIPKey*)key->next;
	BipedLIPKey* prev = (BipedLIPKey*)key->prev;

	vec3_t ang;
	if(next){
	real_t pr0 = key ->ang_com->val;
	real_t pr1 = next->ang_com->val;
	real_t vr0 = key ->angvel_com->val;
	real_t vr1 = next->angvel_com->val;
	real_t t0  = key ->tick ->time; 
	real_t tau = (next->tick->time) - (key ->tick ->time);
	ang.x=0.0;
	ang.y=0.0;
	ang.z=(-(2.0/(tau*tau*tau))*(pr1-pr0)+(1.0/(tau*tau))*(vr1+vr0))*pow(t-t0,3.0)+((3.0/(tau*tau))*(pr1-pr0)-(1.0/tau)*(vr1+2.0*vr0))*pow(t-t0,2.0)+vr0*(t-t0)+pr0;
	}

	if(!next){
	real_t pr0 = prev ->ang_com->val;
	real_t pr1 = key->ang_com->val;
	real_t vr0 = prev ->angvel_com->val;
	real_t vr1 = key->angvel_com->val;
	real_t t0  = prev ->tick ->time; 
	real_t tau = (key->tick->time) - (prev ->tick ->time);
	ang.x=0.0;
	ang.y=0.0;
	ang.z=(-(2.0/(tau*tau*tau))*(pr1-pr0)+(1.0/(tau*tau))*(vr1+vr0))*pow(t-t0,3.0)+((3.0/(tau*tau))*(pr1-pr0)-(1.0/tau)*(vr1+2.0*vr0))*pow(t-t0,2.0)+vr0*(t-t0)+pr0;
	
	}
	return ang;
}

vec3_t BipedLIP::AngVelCoM(real_t t){
	BipedLIPKey* key = GetSegment(t).first;
	BipedLIPKey* next = (BipedLIPKey*)key->next;
	BipedLIPKey* prev = (BipedLIPKey*)key->prev;
	
	vec3_t angvel;
	if(next){
		real_t pr0 = key ->ang_com->val;
		real_t pr1 = next->ang_com->val;
		real_t vr0 = key ->angvel_com->val;
		real_t vr1 = next->angvel_com->val;
		real_t t0  = key ->tick ->time;
		real_t tau = (next->tick->time) - (key ->tick ->time);
		angvel.x=0.0;
		angvel.y=0.0;
		angvel.z=3*(-(2.0/(tau*tau*tau))*(pr1-pr0)+(1.0/(tau*tau))*(vr1+vr0))*pow(t-t0,2.0)+2.0*((3.0/(tau*tau))*(pr1-pr0)-(1.0/tau)*(vr1+2.0*vr0))*(t-t0)+vr0;
	}
	if(!next){
		real_t pr0 = prev->ang_com->val;
		real_t pr1 = key ->ang_com->val;
		real_t vr0 = prev->angvel_com->val;
		real_t vr1 = key ->angvel_com->val;
		real_t t0  = prev->tick ->time;
		real_t tau = (key->tick->time) - (prev ->tick ->time);
		angvel.x=0.0;
		angvel.y=0.0;
		angvel.z=3*(-(2.0/(tau*tau*tau))*(pr1-pr0)+(1.0/(tau*tau))*(vr1+vr0))*pow(t-t0,2.0)+2.0*((3.0/(tau*tau))*(pr1-pr0)-(1.0/tau)*(vr1+2.0*vr0))*(t-t0)+vr0;
	}
	return angvel;
}

vec3_t BipedLIP::AngAccCoM(real_t t){
	BipedLIPKey* key = GetSegment(t).first;
	BipedLIPKey* next = (BipedLIPKey*)key->next;
	BipedLIPKey* prev = (BipedLIPKey*)key->prev;

	vec3_t angacc;
	if(next){
		real_t pr0 = key->ang_com->val;
		real_t pr1 = next->ang_com->val;
		real_t vr0 = key->angvel_com->val;
		real_t vr1 = next->angvel_com->val;
		real_t t0  = key->tick->time; 
		real_t tau = (next->tick->time) - (key->tick->time);

		angacc.x = 0.0;
		angacc.y = 0.0;
		angacc.z = 6.0*(-(2.0/(tau*tau*tau))*(pr1-pr0)+((1.0/pow(tau, 2.0))*(vr1+vr0)))*(t-t0)+2*((3.0/pow(tau, 2.0))*(pr1-pr0)-((1.0/tau)*(vr1+2*vr0)));
	}

	if(!next){
		real_t pr0 = prev->ang_com->val;
		real_t pr1 = key->ang_com->val;
		real_t vr0 = prev->angvel_com->val;
		real_t vr1 = key->angvel_com->val;
		real_t t0  = prev->tick->time; 
		real_t tau = (key->tick->time) - (prev->tick->time);

		angacc.x = 0.0;
		angacc.y = 0.0;
		angacc.z = 6.0*(-(2.0/(tau*tau*tau))*(pr1-pr0)+((1.0/pow(tau, 2.0))*(vr1+vr0)))*(t-t0)+2*((3.0/pow(tau, 2.0))*(pr1-pr0)-((1.0/tau)*(vr1+2*vr0)));
	}

		return angacc;
}
//------------------------------------------------------------------------------------------------

void BipedLIP::CalcTrajectory(){
	real_t tf = traj.back()->tick->time;
	real_t dt = 0.01;
	
	trajectory.clear();
	for(real_t t = 0.0; t < tf; t += dt){
		TrajPoint tp;
		tp.t      = t;
		tp.pos_com   = PosCoM(t);
		tp.pos_cop   = PosCoP(t);
		tp.pos_swing = PosSwingFoot(t);
		tp.pos_torso = PosTorso(tp.pos_com, tp.pos_cop, tp.pos_swing);
		tp.ang_com   = AngCoM(t);

		trajectory.push_back(tp);
	}

	trajReady = true;
}

//------------------------------------------------------------------------------------------------

//角加速度拘束条件(ステップ開始時)
RangeConAccL::RangeConAccL(Solver* solver, ID id, SVar* var0, SVar* var1, SVar* var2, SVar* var3, SVar* var4, real_t _scale):Constraint(solver, 1, id, _scale){
	AddSLink(var0);
	AddSLink(var1);
	AddSLink(var2);
	AddSLink(var3);
	AddSLink(var4);
	real_t inf = numeric_limits<real_t>::max();
	_min = -inf;
	_max =  inf;
	on_lower = false;
	on_upper = false;
}



void RangeConAccL::Prepare(){
	pr0 = ((SVar*)links[0]->var)->val;
	vr0 = ((SVar*)links[1]->var)->val;
	tm  = ((SVar*)links[2]->var)->val;
	pr1 = ((SVar*)links[3]->var)->val;
	vr1 = ((SVar*)links[4]->var)->val;

	//cout << vr0 << '\n';
}

void RangeConAccL::CalcCoef(){
	Prepare();
	((SLink*)links[0])->SetCoef(-(6.0)/(tm*tm));
	((SLink*)links[1])->SetCoef(-(4.0)/tm);
	((SLink*)links[2])->SetCoef(-(12.0*pr1)/(tm*tm*tm)+(12.0*pr0)/(tm*tm*tm)+(2.0*vr1)/(tm*tm)+(4.0*vr0)/(tm*tm));
	((SLink*)links[3])->SetCoef((6.0)/(tm*tm));
	((SLink*)links[4])->SetCoef(-(2.0)/tm);
}

void RangeConAccL::CalcDeviation(){
	real_t s =-(6.0*pr0)/(tm*tm)-(4.0*vr0)/tm+(6.0*pr1)/(tm*tm)-(2.0*vr1)/tm;
	on_lower = (s < _min);
	on_upper = (s > _max);
	active = on_lower | on_upper;
	if(on_lower){
		y[0] = s - _min;
	}
	if(on_upper){
		y[0] = s - _max;
	}
}

void RangeConAccL::Project(real_t& l, uint k){
	// 下限と上限が同じ場合は射影操作しない
	if(_min == _max)
		return;
	if(on_upper && l > 0.0)
		l = 0.0;
	if(on_lower && l < 0.0)
		l = 0.0;
	if(!on_upper && !on_lower)
		l = 0.0;
}

//角加速度拘束条件(ステップ終了時)

RangeConAccR::RangeConAccR(Solver* solver, ID id, SVar* var0, SVar* var1, SVar* var2, SVar* var3, SVar* var4, real_t _scale):Constraint(solver, 1, id, _scale){
	AddSLink(var0);
	AddSLink(var1);
	AddSLink(var2);
	AddSLink(var3);
	AddSLink(var4);
	real_t inf = numeric_limits<real_t>::max();
	_min = -inf;
	_max =  inf;
	on_lower = false;
	on_upper = false;
}

void RangeConAccR::Prepare(){
	pr0 = ((SVar*)links[0]->var)->val;
	vr0 = ((SVar*)links[1]->var)->val;
	tm  = ((SVar*)links[2]->var)->val;
	pr1 = ((SVar*)links[3]->var)->val;
	vr1 = ((SVar*)links[4]->var)->val;
}

void RangeConAccR::CalcCoef(){
	Prepare();
	((SLink*)links[0])->SetCoef((6.0)/(tm*tm));
	((SLink*)links[1])->SetCoef(2.0/tm);
	((SLink*)links[2])->SetCoef((12.0*pr1)/(tm*tm*tm)+(-12.0*pr0)/(tm*tm*tm)-(2.0*vr1)/(tm*tm)-(4.0*vr0)/(tm*tm));
	((SLink*)links[3])->SetCoef(-(6.0)/(tm*tm));
	((SLink*)links[4])->SetCoef(4.0/tm);
}

void RangeConAccR::CalcDeviation(){
	real_t  s =  (6.0*pr0)/(tm*tm)+(2.0*vr0)/tm-(6.0*pr1)/(tm*tm)+(4.0*vr1)/tm;
	on_lower = (s < _min);
	on_upper = (s > _max);
	active = on_lower | on_upper;
	if(on_lower){
		y[0] = s - _min;
	}
	if(on_upper){
		y[0] = s - _max;
	}
}

void RangeConAccR::Project(real_t& l, uint k){
	// 下限と上限が同じ場合は射影操作しない
	if(_min == _max)
		return;
	if(on_upper && l > 0.0)
		l = 0.0;
	if(on_lower && l < 0.0)
		l = 0.0;
	if(!on_upper && !on_lower)
		l = 0.0;
}

//------------------------------------------------------------------------------------------------

//接地可能範囲
RangeConCOP::RangeConCOP(Solver* solver, ID id, SVar* var0, SVar* var1, SVar* var2, SVar* var3, SVar* var4, uint _idx, real_t _scale):
	Constraint(solver, 1, id, _scale){
	AddSLink(var0);
	AddSLink(var1);
	AddSLink(var2);
	AddSLink(var3);
	AddSLink(var4);
	real_t inf = numeric_limits<real_t>::max();
	_min = -inf;
	_max =  inf;
	on_lower = false;
	on_upper = false;
	xaxis    = false;
	yaxis    = false;
	if(_idx == 0){
		xaxis = true;
		yaxis = false;
	}
	if(_idx == 1){
		xaxis = false;
		yaxis = true;
	}
}

void RangeConCOP::Prepare(){
	com_x   = ((SVar*)links[0]->var)->val;
	com_y   = ((SVar*)links[1]->var)->val;
	cop_x   = ((SVar*)links[2]->var)->val;
	cop_y   = ((SVar*)links[3]->var)->val;
	com_ang = ((SVar*)links[4]->var)->val;
}

void RangeConCOP::CalcCoef(){
	Prepare();
	if(xaxis){
		((SLink*)links[0])->SetCoef(-cos(-com_ang));
		((SLink*)links[1])->SetCoef( sin(-com_ang));
		((SLink*)links[2])->SetCoef( cos(-com_ang));
		((SLink*)links[3])->SetCoef(-sin(-com_ang));
		((SLink*)links[4])->SetCoef(-sin(-com_ang)*(cop_x-com_x)-cos(-com_ang)*(cop_y-com_y));
	}
	if(yaxis){
		((SLink*)links[0])->SetCoef(-sin(-com_ang));
		((SLink*)links[1])->SetCoef(-cos(-com_ang));
		((SLink*)links[2])->SetCoef( sin(-com_ang));
		((SLink*)links[3])->SetCoef( cos(-com_ang));
		((SLink*)links[4])->SetCoef(cos(-com_ang)*(cop_x-com_x)-sin(-com_ang)*(cop_y-com_y));
	}
}

void RangeConCOP::CalcDeviation(){
	real_t diff;
	if(xaxis)
		diff = cos( -com_ang) * (cop_x - com_x) - sin( -com_ang) * (cop_y - com_y );
	if(yaxis)
		diff = sin( -com_ang) * (cop_x - com_x) + cos( -com_ang) * (cop_y - com_y ) ;
	on_lower = (diff < _min);
	on_upper = (diff > _max);
	if(on_lower){
		active = true;
		y[0] = (diff - _min) ;
	}
	if(on_upper){
		active = true;
		y[0] = (diff - _max) ;
	}
}


void RangeConCOP::Project(real_t& l, uint k){
	if(on_upper && l > 0.0)
		l = 0.0;
	if(on_lower && l < 0.0)
		l = 0.0;
	if(!on_upper && !on_lower)
		l = 0.0;
}


//------------------------------------------------------------------------------------------------

//最終歩で初期姿勢に戻すための制約
FixCon::FixCon(Solver* solver, string _name, BipedLIPKey* _obj, uint _idx, uint _lastphase, real_t _scale):
	Constraint(solver, 1, ID(0, _obj->node, _obj->tick, _name), _scale){
	obj[0] = (BipedLIPKey*)_obj->prev;
	obj[1] = _obj;
	idx    = _idx;
	phase  = _lastphase;
}

FixConPCOP::FixConPCOP(Solver* solver, string _name, BipedLIPKey* _obj, uint _idx, uint _lastphase, real_t _scale):
	FixCon(solver, _name, _obj, _idx, _lastphase, _scale){

	AddSLink(obj[0]->pos_cop [idx]);
	AddSLink(obj[1]->pos_cop [idx]);
	AddSLink(obj[1]->pos_com [idx]);
}

FixConPtoA::FixConPtoA(Solver* solver, string _name, BipedLIPKey* _obj, uint _idx, uint _lastphase, real_t _scale):
	FixCon(solver, _name, _obj, _idx, _lastphase, _scale){

	AddSLink(obj[0]->pos_cop [idx]);
	AddSLink(obj[1]->ang_com	  );
	AddSLink(obj[1]->pos_com [idx]);
}

void FixCon::Prepare(){
	BipedLIP::Param& param = obj[0]->GetNode()->param;
	 c0  = obj[0]->pos_cop[idx]->val;
	 c1  = obj[1]->pos_cop[idx]->val;
	 p1  = obj[1]->pos_com[idx]->val;
	 ang1= obj[1]->ang_com	   ->val;
}

void FixConPCOP::CalcCoef(){
	Prepare();
	((SLink*)links[0])->SetCoef(1.0);
	((SLink*)links[1])->SetCoef(1.0);
	((SLink*)links[2])->SetCoef(-2.0);
}

void FixConPtoA::CalcCoef(){
	Prepare();
	if(phase){
		((SLink*)links[0])->SetCoef(-1.0);
		if(idx == 0)
		((SLink*)links[1])->SetCoef(0.1*cos(ang1));
		if(idx == 1)
		((SLink*)links[1])->SetCoef(0.1*sin(ang1));
		((SLink*)links[2])->SetCoef(1.0);
	}
	else{
		((SLink*)links[0])->SetCoef(-1.0);
		if(idx == 0)
		((SLink*)links[1])->SetCoef(-0.1*cos(ang1));
		if(idx == 1)
		((SLink*)links[1])->SetCoef(-0.1*sin(ang1));
		((SLink*)links[2])->SetCoef(1.0);
	}
}

void FixConPCOP::CalcDeviation(){
	y[0] = c1 - (2.0*p1 - c0);		
}

void FixConPtoA::CalcDeviation(){
	if(phase){
		if(idx == 0)
		y[0] = p1 - (c0-0.1*sin(ang1));
		if(idx == 1)
		y[0] = p1 - (c0+0.1*cos(ang1));
	}
	else{
		if(idx == 0)
		y[0] = p1 - (c0+0.1*sin(ang1));
		if(idx == 1)
		y[0] = p1 - (c0-0.1*cos(ang1));
	}
}

//------------------------------------------------------------------------------------------------
//1ステップ間における角度変化量に関する制約
DiffConAng::DiffConAng(Solver* solver, ID id, SVar* var0, SVar* var1, real_t _scale):Constraint(solver, 1, id, _scale){
	AddSLink(var0, -1.0);
	AddSLink(var1,  1.0);
	real_t inf = numeric_limits<real_t>::max();
	//_min = -inf;
	_max =  inf;
	on_lower = false;
	on_upper = false;
}

void DiffConAng::CalcDeviation(){
	real_t diff = abs(((SVar*)links[1]->var)->val - ((SVar*)links[1]->var)->val);
	on_upper = (diff < _max);
	active = on_lower;
	if(on_upper)
		y[0] = (diff - _max) ;
}

void DiffConAng::Project(real_t& l, uint k){
	if(on_lower && l > 0.0)
		l = 0.0;
	if(on_lower && l < 0.0)
		l = 0.0;
	if(!on_upper && !on_lower)
		l = 0.0;
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
		canvas->SetLineWidth(1.0f);
		canvas->BeginPath();
		canvas->MoveTo(trajectory[0].pos_com);
		for(uint i = 1; i < trajectory.size(); i++){
			canvas->LineTo(trajectory[i].pos_com);
		}
		canvas->EndPath();
		canvas->EndLayer();
	}
	// torso
	if(conf->Set(canvas, DrawItem::BipedTorso, this)){
		canvas->BeginLayer("biped_torso", true);
		canvas->SetLineWidth(3.0f);
		canvas->BeginPath();
		canvas->MoveTo(trajectory[0].pos_torso);
		for(uint i = 1; i < trajectory.size(); i++){
			//canvas->LineTo(trajectory[i].pos_torso);
		}
		canvas->EndPath();
		canvas->EndLayer();
	}
	// swing foot
	if(conf->Set(canvas, DrawItem::BipedSwing, this)){
		canvas->BeginLayer("biped_swing", true);
		canvas->SetLineWidth(3.0f);
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
	// double support snapshot
	if(conf->Set(canvas, DrawItem::BipedDouble, this)){
		canvas->BeginLayer("biped_double", true);
		canvas->SetLineWidth(1.0f);
		Waypoint& wp0 = waypoints[0];
		for(uint i = 1; i < trajectory.size(); i++){
			//BipedLIPKey* key = GetKeypoint(graph->ticks[i]);
			//if(wp[1].k < i)wp++;
			if(trajectory[i-1].pos_cop != trajectory[i].pos_cop){
				real_t px  = trajectory[i].pos_com.x+0.03*cos(trajectory[i].ang_com.z);
				real_t py  = trajectory[i].pos_com.y+0.03*sin(trajectory[i].ang_com.z);
				real_t pz  = trajectory[i].pos_com.z;
				real_t p1x = trajectory[i].pos_com.x+0.8*cos( M_PI/4.0)*(px-trajectory[i].pos_com.x)-0.8*sin( M_PI/4.0)*(py-trajectory[i].pos_com.y);
				real_t p1y = trajectory[i].pos_com.y+0.8*sin( M_PI/4.0)*(px-trajectory[i].pos_com.x)+0.8*cos( M_PI/4.0)*(py-trajectory[i].pos_com.y);
				real_t p1z = trajectory[i].pos_com.z;		   										   
				real_t p2x = trajectory[i].pos_com.x+0.8*cos(-M_PI/4.0)*(px-trajectory[i].pos_com.x)-0.8*sin(-M_PI/4.0)*(py-trajectory[i].pos_com.y);
				real_t p2y = trajectory[i].pos_com.y+0.8*sin(-M_PI/4.0)*(px-trajectory[i].pos_com.x)+0.8*cos(-M_PI/4.0)*(py-trajectory[i].pos_com.y);
				real_t p2z = trajectory[i].pos_com.z;
 				canvas->Line(trajectory[i].pos_torso, trajectory[i  ].pos_cop);
				canvas->Line(trajectory[i].pos_torso, trajectory[i-1].pos_cop);
				//canvas->Line(Vec3f(trajectory[i].pos_com.x,trajectory[i].pos_com.y,0.0), Vec3f(px,py,0.0));
				//canvas->Line(Vec3f(p1x,p1y,0.0), Vec3f(px,py,0.0));
				//canvas->Line(Vec3f(p2x,p2y,0.0), Vec3f(px,py,0.0));
			}
			if(i == 1){
				real_t px  = wp0.pos_com.x+0.03*cos(wp0.ang_com);
				real_t py  = wp0.pos_com.y+0.03*sin(wp0.ang_com);
				real_t p1x = wp0.pos_com.x+0.8*cos( M_PI/4.0)*(px-wp0.pos_com.x)-0.8*sin( M_PI/4.0)*(py-wp0.pos_com.y);
				real_t p1y = wp0.pos_com.y+0.8*sin( M_PI/4.0)*(px-wp0.pos_com.x)+0.8*cos( M_PI/4.0)*(py-wp0.pos_com.y);	   										   
				real_t p2x = wp0.pos_com.x+0.8*cos(-M_PI/4.0)*(px-wp0.pos_com.x)-0.8*sin(-M_PI/4.0)*(py-wp0.pos_com.y);
				real_t p2y = wp0.pos_com.y+0.8*sin(-M_PI/4.0)*(px-wp0.pos_com.x)+0.8*cos(-M_PI/4.0)*(py-wp0.pos_com.y);
				//canvas->Line(Vec3f(wp0.pos_com.x,wp0.pos_com.y,0.0), Vec3f(px,py,0.0));
			    //canvas->Line(Vec3f(p1x,p1y,0.0), Vec3f(px,py,0.0));
			    //canvas->Line(Vec3f(p2x,p2y,0.0), Vec3f(px,py,0.0));
			}
		}

		real_t tf = graph->ticks.size()-1;
		BipedLIPKey* key = (BipedLIPKey*)GetKeypoint(graph->ticks[tf]);
		real_t px  = key->pos_com[0]->val+0.03*cos(key->ang_com->val);
		real_t py  = key->pos_com[1]->val+0.03*sin(key->ang_com->val);
		real_t p1x = key->pos_com[0]->val+0.8*cos( M_PI/4.0)*(px-key->pos_com[0]->val)-0.8*sin( M_PI/4.0)*(py-key->pos_com[1]->val);
		real_t p1y = key->pos_com[1]->val+0.8*sin( M_PI/4.0)*(px-key->pos_com[0]->val)+0.8*cos( M_PI/4.0)*(py-key->pos_com[1]->val);	   										   
		real_t p2x = key->pos_com[0]->val+0.8*cos(-M_PI/4.0)*(px-key->pos_com[0]->val)-0.8*sin(-M_PI/4.0)*(py-key->pos_com[1]->val);
		real_t p2y = key->pos_com[1]->val+0.8*sin(-M_PI/4.0)*(px-key->pos_com[0]->val)+0.8*cos(-M_PI/4.0)*(py-key->pos_com[1]->val);
		//canvas->Line(Vec3f(key->pos_com[0]->val,key->pos_com[1]->val,0.0), Vec3f(px,py,0.0));
		//canvas->Line(Vec3f(p1x,p1y,0.0), Vec3f(px,py,0.0));
		//canvas->Line(Vec3f(p2x,p2y,0.0), Vec3f(px,py,0.0));

		canvas->EndLayer();
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
		real_t px  = PosCoM(tm).x+0.03*cos(AngCoM(tm).z);
		real_t py  = PosCoM(tm).y+0.03*sin(AngCoM(tm).z);
		real_t p1x = PosCoM(tm).x+0.8*cos( M_PI/4.0)*(px-PosCoM(tm).x)-0.8*sin( M_PI/4.0)*(py-PosCoM(tm).y);
		real_t p1y = PosCoM(tm).y+0.8*sin( M_PI/4.0)*(px-PosCoM(tm).x)+0.8*cos( M_PI/4.0)*(py-PosCoM(tm).y);	   										   
		real_t p2x = PosCoM(tm).x+0.8*cos(-M_PI/4.0)*(px-PosCoM(tm).x)-0.8*sin(-M_PI/4.0)*(py-PosCoM(tm).y);
		real_t p2y = PosCoM(tm).y+0.8*sin(-M_PI/4.0)*(px-PosCoM(tm).x)+0.8*cos(-M_PI/4.0)*(py-PosCoM(tm).y);
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

		//胴体向き
		canvas->Line(Vec3f(PosCoM(tm)), Vec3f(px,py,PosCoM(tm).z));
		canvas->Line(Vec3f(p1x,p1y,PosCoM(tm).z), Vec3f(px,py,PosCoM(tm).z));
		canvas->Line(Vec3f(p2x,p2y,PosCoM(tm).z), Vec3f(px,py,PosCoM(tm).z));
    }

}

void BipedLIP::DrawSnapshot(real_t time, DrawCanvas* canvas, DrawConfig* conf){
}

void BipedLIP::Save(){
	real_t tf1 = traj.back()->tick->time;
	real_t dt1 = 0.01;

	FILE* file = fopen("plan_step.csv", "w");
	fprintf(file, "step, time, period, pos_com_x, pos_com_y, vel_com_x, vel_com_y, pos_cop_x, pos_cop_y, ang_com, angvel_com\n");

	real_t t = 0.0;
	if(!MotionData){
		for(uint k = 0; k < graph->ticks.size()-1; k++){
			BipedLIPKey* key = (BipedLIPKey*)GetKeypoint(graph->ticks[k]);

			fprintf(file, "%d, %3.3lf, %3.3lf, %3.3lf, %3.3lf, %3.3lf, %3.3lf, %3.3lf, %3.3lf, %3.31f, %3.31f\n",
				k, t,
				key->period    ->val,
				key->pos_com  [0]->val, key->pos_com  [1]->val,
				key->vel_com  [0]->val, key->vel_com  [1]->val,
				key->pos_cop[0]->val, key->pos_cop[1]->val,
				key->ang_com->val, key->angvel_com->val);

			t += key->period->val;
		}

		fclose(file);
	}
	else{
		real_t tf = traj.back()->tick->time;
		real_t dt = 0.02f;
		FILE* file = fopen("plan_step2.csv", "w");
		fprintf(file, "phase, time, pos_com_x, pos_com_y, pos_com_z, vel_com_x, vel_com_y, vel_com_z, pos_sup_x, pos_sup_y, pos_sup_z, pos_swg_x, pos_swg_y, pos_swg_z, ang_com, angvel_com, angacc_com\n");
		for(real_t t = 0; t < tf; t += dt){
			fprintf(file, "%d, %3.3lf, %3.3lf, %3.3lf, %3.3lf, %3.3lf, %3.3lf, %3.3lf, %3.3lf, %3.3lf, %3.3lf, %3.3lf, %3.3lf, %3.3lf, %3.3lf, %3.3lf, %3.3lf\n",
				Phase(t),t,
				PosCoM(t).x, PosCoM(t).y ,PosCoM(t).z, 
				VelCoM(t).x, VelCoM(t).y ,VelCoM(t).z,
				PosSupportFoot(t).x, PosSupportFoot(t).y, PosSupportFoot(t).z,
				PosSwingFoot(t).x,  PosSwingFoot(t).y, PosSwingFoot(t).z,
				AngCoM(t).z, AngVelCoM(t).z, AngAccCoM(t).z);
		}
		fprintf(file, "%d, %3.3lf, %3.3lf, %3.3lf, %3.3lf, %3.3lf, %3.3lf, %3.3lf, %3.3lf, %3.3lf, %3.3lf, %3.3lf, %3.3lf, %3.3lf, %3.3lf, %3.3lf, %3.3lf\n",
				Phase(tf),tf,
				PosCoM(tf).x, PosCoM(tf).y ,PosCoM(tf).z, 
				VelCoM(tf).x, VelCoM(tf).y ,VelCoM(tf).z,
				PosSupportFoot(tf).x, PosSupportFoot(tf).y, PosSupportFoot(tf).z,
				PosSwingFoot(tf).x,  PosSwingFoot(tf).y, PosSwingFoot(tf).z,
				AngCoM(tf).z, AngVelCoM(tf).z, AngAccCoM(tf));
	fclose(file);
	}
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

	AddSLink(obj[0]->pos_com  [idx]);
	AddSLink(obj[0]->vel_com  [idx]);
	AddSLink(obj[0]->pos_cop[idx]);
	AddSLink(obj[0]->period      );
	AddSLink(obj[1]->pos_com  [idx]);
}

LIPConV::LIPConV(Solver* solver, string _name, BipedLIPKey* _obj, uint _idx, real_t _scale):
	LIPCon(solver, _name, _obj, _idx, _scale){
	AddSLink(obj[0]->pos_com  [idx]);
	AddSLink(obj[0]->vel_com  [idx]);
	AddSLink(obj[0]->pos_cop[idx]);
	AddSLink(obj[0]->period      );
	AddSLink(obj[1]->vel_com  [idx]);
}

//-------------------------------------------------------------------------------------------------

void LIPCon::Prepare(){
	BipedLIP::Param& param = obj[0]->GetNode()->param;
	T  = param.T;
	t  = obj[0]->period->val/T;
	ch = cosh(t);
	sh = sinh(t);
	p0 = obj[0]->pos_com  [idx]->val;
	v0 = obj[0]->vel_com  [idx]->val;
	c  = obj[0]->pos_cop[idx]->val;
	p1 = obj[1]->pos_com  [idx]->val;
	v1 = obj[1]->vel_com  [idx]->val;
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

}
