#include <DiMP2/Graph.h>
#include <DiMP2/Mpc.h>


namespace DiMP2
{;
//-------------------------------------------------------------------------------------------------
// BipedLIPKey

BipedLIPKey::BipedLIPKey()
{
	phase = BipedLIP::Left;

	//オンラインのとき
	phase = MPC::cntrl.rl;
}

// memo：ここで重心位置・接地位置の計算を行わせている 歩数分の回数呼び出す
void BipedLIPKey::AddVar(Solver* solver)
{
	BipedLIP* obj = GetNode();

	pos_t[0] = new SVar(solver, ID(0, node, tick, name + "_pos0"), node->graph->scale.pos_t); //0:x,1:y
	pos_t[1] = new SVar(solver, ID(0, node, tick, name + "_pos1"), node->graph->scale.pos_t);
	vel_t[0] = new SVar(solver, ID(0, node, tick, name + "_vel0"), node->graph->scale.vel_t);
	vel_t[1] = new SVar(solver, ID(0, node, tick, name + "_vel1"), node->graph->scale.vel_t);

	//addition acc_t
	acc_t[0] = new SVar(solver, ID(0, node, tick, name + "_acc0"), node->graph->scale.acc_t);
	acc_t[1] = new SVar(solver, ID(0, node, tick, name + "_acc1"), node->graph->scale.acc_t);


	pos_cop[0] = new SVar(solver, ID(0, node, tick, name + "_cop0"), node->graph->scale.pos_t);
	pos_cop[1] = new SVar(solver, ID(0, node, tick, name + "_cop1"), node->graph->scale.pos_t);
	
	period  = new SVar (solver, ID(VarTag::ObjectTP, node, tick, name + "_T"), node->graph->scale.time );

}


void BipedLIPKey::AddCon(Solver* solver)
{
	BipedLIPKey* nextObj = (BipedLIPKey*)next;
	
	if(next)
	{

		//LIPモデル
		con_lip_pos[0] = new LIPConP(solver, name + "_lip_p0", this, 0, node->graph->scale.pos_t);
		con_lip_pos[1] = new LIPConP(solver, name + "_lip_p1", this, 1, node->graph->scale.pos_t);
		con_lip_vel[0] = new LIPConV(solver, name + "_lip_v0", this, 0, node->graph->scale.vel_t);
		con_lip_vel[1] = new LIPConV(solver, name + "_lip_v1", this, 1, node->graph->scale.vel_t);
		

		//1phaseの時間
		con_range_period = new RangeConS(solver, ID(0, node, tick, name + "_range_period"), period, node->graph->scale.time);
		
		con_diff_cop[0][0] = new DiffConS(solver, ID(0, node, tick, name + "_range_cop0x"), pos_t[0], pos_cop[0], node->graph->scale.pos_t);
		con_diff_cop[0][1] = new DiffConS(solver, ID(0, node, tick, name + "_range_cop0y"), pos_t[1], pos_cop[1], node->graph->scale.pos_t);
		con_diff_cop[1][0] = new DiffConS(solver, ID(0, node, tick, name + "_range_cop1x"), nextObj->pos_t[0], pos_cop[0], node->graph->scale.pos_t);
		con_diff_cop[1][1] = new DiffConS(solver, ID(0, node, tick, name + "_range_cop1y"), nextObj->pos_t[1], pos_cop[1], node->graph->scale.pos_t);

	}


	//終端状態
	if(!next)
	{
		if ((bool)GetNode()->param.use_terminal_pos == true){
			con_fix_pos[0] = new FixConS(solver, ID(0, node, tick, name + "_fix_pos0"), pos_t[0], node->graph->scale.pos_t);
			con_fix_pos[1] = new FixConS(solver, ID(0, node, tick, name + "_fix_pos1"), pos_t[1], node->graph->scale.pos_t);
		}

		if ((bool)GetNode()->param.use_terminal_vel == true){
		con_fix_vel[0] = new FixConS(solver, ID(0, node, tick, name + "_fix_vel0"), vel_t[0], node->graph->scale.vel_t);
		con_fix_vel[1] = new FixConS(solver, ID(0, node, tick, name + "_fix_vel1"), vel_t[1], node->graph->scale.vel_t);
		}
		
	}

}

//memo : 様々なパラメータ 最初に一回呼び出して終了
BipedLIP::Param::Param()
{
//	gravity         = 9.8;
//	height          = 0.7;
//	swing_vel_max   = 1.0;
//	step_period_min = 0.0;
//	step_period_max = 1.0;
	terminal_pos    = vec2_t();
	terminal_vel    = vec2_t();
	
	support_min[0]  = vec2_t( -0.08 , -0.12 );  ///  <歩幅 of Left Phase	(進行方向,横方向）
	support_max[0]  = vec2_t(  0.08 , -0.08 );  /// 
	
	support_min[1]  = vec2_t( -0.08 ,  0.08 );  ///  <歩幅 of Right Phase
	support_max[1]  = vec2_t(  0.08 ,  0.12 );  /// 

	// 共有メモリを開く
	MPC::cntrl.enuvoState  = MPC::cntrl.smState .TryOpen<ENuvo2::State>("ENUVO2_STATE_SHARED_MEMORY");

}


void BipedLIPKey::Prepare()
{	
	if(!prev){
		 tick->time = 0.0;
	}else{
		BipedLIPKey* prevObj = (BipedLIPKey*)prev;
		tick->time = prevObj->tick->time + prevObj->period->val;
	}
}





//両脚支持時の重心や接地点の位置の描写
void BipedLIPKey::Draw(GRRenderIf* render, DrawConfig* conf)
{
	render->SetPointSize(8.0f,0);
	Vec3f m,p,v;											//(Vec3f:float型三次元ベクトル)

	// 重心位置
	m.x = (float)pos_t[0]->val;
	m.y = (float)pos_t[1]->val;
	m.z = (float)GetNode()->param.height;
	render->DrawPoint(m);

	// cout << m << "\n" ;

	//p 点を描写する
	
	render->SetPointSize(8.0f,1);
	// COP位置(接地位置)
	p.x = (float)pos_cop[0]->val	;
	p.y = (float)pos_cop[1]->val	;
	p.z = 0.0f	;

	// cout << p << "\n" ;

	if(p.x!= 0.0 || p.y != 0.0) {
	render->DrawPoint(p);

	render->SetLineWidth(3.0f)		;		// 初期のzを表示
	render->DrawLine(vec3_t(0.0,0.0,0.0),vec3_t(0.0,0.0,(float)GetNode()->param.height));
	}
	
}



// BipedLIPKey ここまで
//-------------------------------------------------------------------------------------------------
// BipedLIP




BipedLIP::BipedLIP(Graph* g, string n):TrajectoryNode(g, n){
	}



//------------------------------------------------------------------------------------------------------//


// 各歩数における拘束条件与える（一回計算して終了）
void BipedLIP::Init()				
{
	if(param.online == true){
		float l1=80.0e-3, l2=70.0e-3, l3=274.0e-3, l4=274.0e-3, l5=70.0e-3;
		float HipYawR_c,HipYawL_c,HipRollR_c,HipRollL_c,HipPitchR_c,HipPitchL_c,KneePitchR_c,KneePitchL_c,AnklePitchR_c,AnklePitchL_c;
		float HipYawR_s,HipYawL_s,HipRollR_s,HipRollL_s,HipPitchR_s,HipPitchL_s,KneePitchR_s,KneePitchL_s,AnklePitchR_s,AnklePitchL_s;

		// 左右計画入れ替え
		if(MPC::calc.startcalc){
			if(MPC::calc.footcount == 1){
				MPC::calc.total_step = param.total_step;
				MPC::cntrl.rl = MPC::cntrl.accdirection;
			}else{
				MPC::cntrl.rl = !MPC::cntrl.rl;
			}
		}

		if(MPC::cntrl.rl == Left){
			for(DiMP2::uint i = 0; i < MPC::calc.total_step; i++){
				if(i % 2 == 0)
					SetPhase(i, BipedLIP::Left );
				else
					SetPhase(i, BipedLIP::Right );
			}
		}else{
			for(DiMP2::uint i = 0; i < MPC::calc.total_step; i++){
				if(i % 2 == 0)
					SetPhase(i, BipedLIP::Right );
				else
					SetPhase(i, BipedLIP::Left );
			}
		}
		BipedLIPKey::BipedLIPKey();



		//状態取得
		HipYawR_c		=	cos(MPC::cntrl.enuvoState->angleRef[MPC::cntrl.jointid.HipYawR]);
		HipYawL_c		=	cos(MPC::cntrl.enuvoState->angleRef[MPC::cntrl.jointid.HipYawL]);
		HipRollR_c		=	cos( - MPC::cntrl.enuvoState->angleRef[MPC::cntrl.jointid.HipRollR]);
		HipRollL_c   	=	cos( - MPC::cntrl.enuvoState->angleRef[MPC::cntrl.jointid.HipRollL]);
		HipPitchR_c		=	cos( - MPC::cntrl.enuvoState->angleRef[MPC::cntrl.jointid.HipPitchR]);
		HipPitchL_c 	=	cos( - MPC::cntrl.enuvoState->angleRef[MPC::cntrl.jointid.HipPitchL]);
		KneePitchR_c	=	cos( - MPC::cntrl.enuvoState->angleRef[MPC::cntrl.jointid.KneePitchR] + MPC::cntrl.enuvoState->angleRef[MPC::cntrl.jointid.HipPitchR]);
		KneePitchL_c 	=	cos( - MPC::cntrl.enuvoState->angleRef[MPC::cntrl.jointid.KneePitchL] + MPC::cntrl.enuvoState->angleRef[MPC::cntrl.jointid.HipPitchL]);
		AnklePitchR_c	=	cos( - MPC::cntrl.enuvoState->angleRef[MPC::cntrl.jointid.AnklePitchR] + MPC::cntrl.enuvoState->angleRef[MPC::cntrl.jointid.KneePitchR]);
		AnklePitchL_c	=	cos( - MPC::cntrl.enuvoState->angleRef[MPC::cntrl.jointid.AnklePitchL] + MPC::cntrl.enuvoState->angleRef[MPC::cntrl.jointid.KneePitchL]);
	
		HipYawR_s		=	sin(MPC::cntrl.enuvoState->angleRef[MPC::cntrl.jointid.HipYawR]);
		HipYawL_s		=	sin(MPC::cntrl.enuvoState->angleRef[MPC::cntrl.jointid.HipYawL]);
		HipRollR_s		=	sin( - MPC::cntrl.enuvoState->angleRef[MPC::cntrl.jointid.HipRollR]);
		HipRollL_s   	=	sin( - MPC::cntrl.enuvoState->angleRef[MPC::cntrl.jointid.HipRollL]);
		HipPitchR_s		=	sin( - MPC::cntrl.enuvoState->angleRef[MPC::cntrl.jointid.HipPitchR]);
		HipPitchL_s 	=	sin( - MPC::cntrl.enuvoState->angleRef[MPC::cntrl.jointid.HipPitchL]);
		KneePitchR_s	=	sin( - MPC::cntrl.enuvoState->angleRef[MPC::cntrl.jointid.KneePitchR] + MPC::cntrl.enuvoState->angleRef[MPC::cntrl.jointid.HipPitchR]);
		KneePitchL_s 	=	sin( - MPC::cntrl.enuvoState->angleRef[MPC::cntrl.jointid.KneePitchL] + MPC::cntrl.enuvoState->angleRef[MPC::cntrl.jointid.HipPitchL]);
		AnklePitchR_s	=	sin( - MPC::cntrl.enuvoState->angleRef[MPC::cntrl.jointid.AnklePitchR] + MPC::cntrl.enuvoState->angleRef[MPC::cntrl.jointid.KneePitchR]);
		AnklePitchL_s	=	sin( - MPC::cntrl.enuvoState->angleRef[MPC::cntrl.jointid.AnklePitchL] + MPC::cntrl.enuvoState->angleRef[MPC::cntrl.jointid.KneePitchL]);
		
		//順運動学
		param.lfoot_place.x = -HipYawL_c*(AnklePitchL_s*0.0*(HipPitchL_c*KneePitchL_c-HipPitchL_c*KneePitchL_s)+(AnklePitchL_c*0.0+l4)*(HipPitchL_c*KneePitchL_s+HipPitchL_s*KneePitchL_c)+HipPitchL_s*l3) - 0.025;
		param.lfoot_place.y = HipRollL_c*HipYawL_s*(AnklePitchL_s*l5*(HipPitchL_c*KneePitchL_c-HipPitchL_s*KneePitchL_s)+(AnklePitchL_c*l5+l4)*(HipPitchL_c*KneePitchL_s+HipPitchL_s*KneePitchL_c)+HipPitchL_s*l3)
							  -HipRollL_s*(AnklePitchL_s*l5*(HipPitchL_s*KneePitchL_c+HipPitchL_c*KneePitchL_s)+(AnklePitchL_c*l5+l4)*(HipPitchL_s*KneePitchL_s-HipPitchL_c*KneePitchL_c)-HipPitchL_c*l3-l2)+l1;
		param.rfoot_place.x = -HipYawR_c*(AnklePitchR_s*0.0*(HipPitchR_c*KneePitchR_c-HipPitchR_c*KneePitchR_s)+(AnklePitchR_c*0.0+l4)*(HipPitchR_c*KneePitchR_s+HipPitchR_s*KneePitchR_c)+HipPitchR_s*l3) - 0.025;
		param.rfoot_place.y = HipRollR_c*HipYawR_s*(AnklePitchR_s*l5*(HipPitchR_c*KneePitchR_s-HipPitchR_s*KneePitchR_s)+(AnklePitchR_c*l5+l4)*(HipPitchR_c*KneePitchR_s+HipPitchR_s*KneePitchR_c)+HipPitchR_s*l3)
							  -HipRollR_s*(AnklePitchR_s*l5*(HipPitchR_s*KneePitchR_c+HipPitchR_c*KneePitchR_s)+(AnklePitchR_c*l5+l4)*(HipPitchR_s*KneePitchR_s-HipPitchR_c*KneePitchR_c)-HipPitchR_c*l3-l2)-l1;
		
		/*斜め接地を考慮する場合
		if(param.lfoot_place.y > 0.095)			param.lfoot_place.y -= 0.0225;
		else if(param.lfoot_place.y < 0.075)	param.lfoot_place.y += 0.0225;
		if(param.rfoot_place.y < -0.095)		param.lfoot_place.y += 0.0225;
		else if(param.rfoot_place.y > -0.075)	param.lfoot_place.y -= 0.0225;
		*/

		//重心位置補正
		float deltax = (param.lfoot_place.y + param.rfoot_place.y) / (param.alpha + 2.0);
		float deltay = (param.lfoot_place.y + param.rfoot_place.y) / (param.alpha + 2.0);
		param.lfoot_place.x = param.lfoot_place.x - deltax;
		param.lfoot_place.y = param.lfoot_place.y - deltay; 					  
		param.rfoot_place.x = param.rfoot_place.x - deltax;
		param.rfoot_place.y = param.rfoot_place.y - deltay;




		//初期足先角度
		if((- MPC::cntrl.enuvoState->rate.x) > 0)	MPC::cntrl.initialangle[0].x =   atan((double)0.5445 / (double)(MPC::cntrl.enuvoCtrl->ikFoot[0].z + 0.08 + 0.045));
		else										MPC::cntrl.initialangle[0].x =   atan((double)0.5445 / (double)(MPC::cntrl.enuvoCtrl->ikFoot[0].z + 0.08 - 0.065));
		MPC::cntrl.initialangle[0].z = - atan((double)0.5445 / (double)(MPC::cntrl.enuvoCtrl->ikFoot[0].x - 0.025 - 0.1));
		MPC::cntrl.initialangle[0].y = - atan((double)0.5445 / (double)(MPC::cntrl.enuvoCtrl->ikFoot[0].x - 0.025 + 0.1)); //.zの負方向
		if((- MPC::cntrl.enuvoState->rate.x) > 0)	MPC::cntrl.initialangle[1].x =   atan((double)0.5445 / (double)(MPC::cntrl.enuvoCtrl->ikFoot[1].z - 0.08 + 0.065));
		else										MPC::cntrl.initialangle[1].x =   atan((double)0.5445 / (double)(MPC::cntrl.enuvoCtrl->ikFoot[1].z - 0.08 - 0.045));					
		MPC::cntrl.initialangle[1].z = - atan((double)0.5445 / (double)(MPC::cntrl.enuvoCtrl->ikFoot[1].x - 0.025 - 0.1));
		MPC::cntrl.initialangle[1].y = - atan((double)0.5445 / (double)(MPC::cntrl.enuvoCtrl->ikFoot[1].x - 0.025 + 0.1)); //.zの負方向
		//初期足長さ
		MPC::cntrl.initiallength[0].x = sqrt((double)0.5445 * 0.5445 + (double)(MPC::cntrl.enuvoCtrl->ikFoot[0].x - 0.025 + 0.1) * (MPC::cntrl.enuvoCtrl->ikFoot[0].x - 0.025 + 0.1));
		MPC::cntrl.initiallength[0].y = sqrt((double)0.5445 * 0.5445 + (double)(MPC::cntrl.enuvoCtrl->ikFoot[0].x - 0.025 - 0.1) * (MPC::cntrl.enuvoCtrl->ikFoot[0].x - 0.025 - 0.1)); //.xの負方向
		if((- MPC::cntrl.enuvoState->rate.x) > 0)	MPC::cntrl.initiallength[0].z = sqrt((double)0.5445 * 0.5445 + (double)(MPC::cntrl.enuvoCtrl->ikFoot[0].z + 0.08 + 0.045) * (MPC::cntrl.enuvoCtrl->ikFoot[0].z + 0.08 + 0.045));
		else										MPC::cntrl.initiallength[0].z = sqrt((double)0.5445 * 0.5445 + (double)(MPC::cntrl.enuvoCtrl->ikFoot[0].z + 0.08 - 0.065) * (MPC::cntrl.enuvoCtrl->ikFoot[0].z + 0.08 - 0.065));
		MPC::cntrl.initiallength[1].x = sqrt((double)0.5445 * 0.5445 + (double)(MPC::cntrl.enuvoCtrl->ikFoot[1].x - 0.025 + 0.1) * (MPC::cntrl.enuvoCtrl->ikFoot[1].x - 0.025 + 0.1));
		MPC::cntrl.initiallength[1].y = sqrt((double)0.5445 * 0.5445 + (double)(MPC::cntrl.enuvoCtrl->ikFoot[1].x - 0.025 - 0.1) * (MPC::cntrl.enuvoCtrl->ikFoot[1].x - 0.025 - 0.1)); //.xの負方向
		if((- MPC::cntrl.enuvoState->rate.x) > 0)	MPC::cntrl.initiallength[1].z = sqrt((double)0.5445 * 0.5445 + (double)(MPC::cntrl.enuvoCtrl->ikFoot[1].z - 0.08 + 0.065) * (MPC::cntrl.enuvoCtrl->ikFoot[1].z - 0.08 + 0.065));
		else										MPC::cntrl.initiallength[1].z = sqrt((double)0.5445 * 0.5445 + (double)(MPC::cntrl.enuvoCtrl->ikFoot[1].z - 0.08 - 0.045) * (MPC::cntrl.enuvoCtrl->ikFoot[1].z - 0.08 - 0.045));

		//重心並進速度を角速度から導出
		if(MPC::cntrl.rl == Left){
			MPC::cntrl.ratetovel.z = MPC::cntrl.initiallength[0].z * (- MPC::cntrl.enuvoState->rate.x) * sin((double)MPC::cntrl.initialangle[0].x); 
			if(MPC::cntrl.enuvoState->rate.z > 0){
				MPC::cntrl.ratetovel.x = - MPC::cntrl.initiallength[0].y * MPC::cntrl.enuvoState->rate.z * sin((double)MPC::cntrl.initialangle[0].z);
			}else{
				MPC::cntrl.ratetovel.x =   MPC::cntrl.initiallength[0].x * MPC::cntrl.enuvoState->rate.z * sin((double)MPC::cntrl.initialangle[0].y);
			}
		}else{
			MPC::cntrl.ratetovel.z = - MPC::cntrl.initiallength[1].z * (- MPC::cntrl.enuvoState->rate.x) * sin((double)MPC::cntrl.initialangle[1].x); 
			if(MPC::cntrl.enuvoState->rate.z > 0){
				MPC::cntrl.ratetovel.x = - MPC::cntrl.initiallength[1].y * MPC::cntrl.enuvoState->rate.z * sin((double)MPC::cntrl.initialangle[1].z);
			}else{
				MPC::cntrl.ratetovel.x =   MPC::cntrl.initiallength[1].x * MPC::cntrl.enuvoState->rate.z * sin((double)MPC::cntrl.initialangle[1].y);
			}
		}
		param.initial_velocity = DiMP2::vec2_t(MPC::cntrl.ratetovel.x, - MPC::cntrl.ratetovel.z);

		//log用
		MPC::calc.rfoot_place_log[MPC::calc.stepcount-1] = param.rfoot_place;
		MPC::calc.lfoot_place_log[MPC::calc.stepcount-1] = param.lfoot_place;
		MPC::calc.deltax_log[MPC::calc.stepcount-1] = deltax;
		MPC::calc.deltay_log[MPC::calc.stepcount-1] = deltay;
		MPC::calc.rate_log[MPC::calc.stepcount-1] = MPC::cntrl.enuvoState->rate;
		MPC::calc.initial_velocity_log[MPC::calc.stepcount-1] = param.initial_velocity;
		MPC::calc.rl_log[MPC::calc.stepcount-1] = MPC::cntrl.rl;
	}

	TrajectoryNode::Init();

	//最初の一歩の初速は0にする
	if(MPC::calc.footcount == 1){
		 param.initial_velocity.x = 0.0;
		 param.initial_velocity.y = 0.0;
	}
	
	
	// 初期速度の与え方について
	else{
		param.initial_velocity.x = MPC::calc.last_velocity.x ;
		param.initial_velocity.y = MPC::calc.last_velocity.y ;
	//	param.initial_velocity.x = 0.0 ; 
	//	param.initial_velocity.y = 0.0 ;
	}


		cout << "footcount:"  <<MPC::calc.footcount << "\n" ;
//	cout << "lfoot_place.x" <<param.lfoot_place.x << ",lfoot_place.y" << param.lfoot_place.y << "\n" ;
//	cout << "rfoot_place.x" <<param.rfoot_place.x << ",lfoot_place.y" << param.rfoot_place.y << "\n" ;
	cout << "velocity.x:" << param.initial_velocity.x << ", velocity.y:" << param.initial_velocity.y << "\n";


	for(uint k = 0; k < graph->ticks.size(); k++)
	{

		BipedLIPKey* key = GetKeypoint(graph->ticks[k]);  //keyでtickにアクセス
		
		std::map<int, int>::iterator it = param.phase.find(k);
		
		if(it == param.phase.end())
			 key->phase = BipedLIP::Left;
		else key->phase = it->second;

		if(!key->prev)
		{
			key->pos_t[0]->Lock();
			key->pos_t[1]->Lock();
			
			//初期状態を与える
			if(param.use_beginning == true)
			{
				key->vel_t[0]->val =  param.initial_velocity.x;
				key->vel_t[1]->val =  param.initial_velocity.y;

				if(MPC::cntrl.rl == Left){
					key->pos_cop[0]->val = param.rfoot_place.x;	
					key->pos_cop[1]->val = param.rfoot_place.y;
				}else{
					key->pos_cop[0]->val = param.lfoot_place.x;	
					key->pos_cop[1]->val = param.lfoot_place.y;
				}

				key->pos_cop[0]->Lock();
				key->pos_cop[1]->Lock();

			}
			
			key->vel_t[0]->Lock();
			key->vel_t[1]->Lock();

		}


		if (key->next)
		{
			//初期軌道(11/27)
			if (param.use_initial_trajectory){
			key->pos_t[0]->Set(k,param.terminal_pos.x * k * 1.5 / param.total_step);
			key->pos_t[1]->Set(k,param.terminal_pos.y * k * 1.5 / param.total_step);

			// key->pos_cop[0]->Set(k,param.terminal_pos.x * k * 1 / param.total_step);
			// key->pos_cop[1]->Set(k,param.terminal_pos.y * k * 1 / param.total_step);
			}
			// 周期の初期値は下限と上限の中間値
			key->period->val = (param.step_period_min + param.step_period_max) / 2.0  ;

////////////////////////////////// 1Phaseの時間の最小と最大のパラメータ//////////////////////////////

			key->con_range_period->_min = param.step_period_min	;
			key->con_range_period->_max = param.step_period_max	;


///////////////////////////////////// 接地位置に関する拘束 ///////////////////////////////////////

			bool lr = (key->phase == Left ? 0 : 1);	


			// 歩行用拘束

			for(int i = 0; i < 2; i++)
			{
				key->con_diff_cop[i][0]->_min = param.support_min[lr].x;
				key->con_diff_cop[i][0]->_max = param.support_max[lr].x;
				key->con_diff_cop[i][1]->_min = param.support_min[lr].y;
				key->con_diff_cop[i][1]->_max = param.support_max[lr].y;

			}

	


			//	転倒回避用拘束
			//if(k == MPC::calc.total_step - 3){
			//	if(abs(MPC::calc.firstrate.x) > 0.2 && abs(MPC::calc.firstrate.z) > 0.2){
			//		key->con_diff_cop[0][0]->_min = param.support_min[lr].x;
			//		key->con_diff_cop[0][0]->_max = param.support_max[lr].x;
			//		key->con_diff_cop[0][1]->_min = param.support_min[lr].y;
			//		key->con_diff_cop[0][1]->_max = param.support_max[lr].y;

			//		if(MPC::calc.firstrate.z < 0.0){
			//			key->con_diff_cop[1][0]->_min = param.support_min[lr].x;
			//			key->con_diff_cop[1][0]->_max = param.support_max[lr].x;
			//		}else{
			//			key->con_diff_cop[1][0]->_min = param.support_min[lr].x;
			//			key->con_diff_cop[1][0]->_max = param.support_max[lr].x;
			//		}
			//		if(lr == 0){
			//			key->con_diff_cop[1][1]->_min = param.support_min[0].y;
			//			key->con_diff_cop[1][1]->_max = -0.14;
			//		}else{
			//			key->con_diff_cop[1][1]->_min = 0.14;
			//			key->con_diff_cop[1][1]->_max = param.support_max[1].y;
			//		}
			//	}else if(abs(MPC::calc.firstrate.x) > 0.2){
			//		key->con_diff_cop[0][0]->_min = param.support_min[lr].x;
			//		key->con_diff_cop[0][0]->_max = param.support_max[lr].x;
			//		key->con_diff_cop[0][1]->_min = param.support_min[lr].y;
			//		key->con_diff_cop[0][1]->_max = param.support_max[lr].y;

			//		key->con_diff_cop[1][0]->_min = param.support_min[lr].x;
			//		key->con_diff_cop[1][0]->_max = param.support_max[lr].x;
			//		if(lr == 0){
			//			key->con_diff_cop[1][1]->_min = param.support_min[0].y;
			//			key->con_diff_cop[1][1]->_max = -0.145;
			//		}else{
			//			key->con_diff_cop[1][1]->_min = 0.145;
			//			key->con_diff_cop[1][1]->_max = param.support_max[1].y;
			//		}
			//	}else if(abs(MPC::calc.firstrate.z) > 0.2){
			//		key->con_diff_cop[0][0]->_min = param.support_min[lr].x;
			//		key->con_diff_cop[0][0]->_max = param.support_max[lr].x;
			//		key->con_diff_cop[0][1]->_min = param.support_min[lr].y;
			//		key->con_diff_cop[0][1]->_max = param.support_max[lr].y;

			//		if(MPC::calc.firstrate.z < 0.0){
			//			key->con_diff_cop[1][0]->_min = param.support_min[lr].x;
			//			key->con_diff_cop[1][0]->_max = param.support_max[lr].x;
			//		}else{
			//			key->con_diff_cop[1][0]->_min = param.support_min[lr].x;
			//			key->con_diff_cop[1][0]->_max = param.support_max[lr].x;
			//		}
			//		if(lr == 0){
			//			key->con_diff_cop[1][1]->_min = param.support_min[0].y;
			//			key->con_diff_cop[1][1]->_max = -0.10;
			//		}else{
			//			key->con_diff_cop[1][1]->_min = 0.10;
			//			key->con_diff_cop[1][1]->_max = param.support_max[1].y;
			//		}
			//	}else{
			//		for(int i = 0; i < 2; i++)
			//		{
			//			key->con_diff_cop[i][0]->_min = param.support_min[lr].x;
			//			key->con_diff_cop[i][0]->_max = param.support_max[lr].x;
			//			if(lr == 0){
			//				key->con_diff_cop[i][1]->_min = param.support_min[0].y;
			//				key->con_diff_cop[i][1]->_max = -0.10;
			//			}else{
			//				key->con_diff_cop[i][1]->_min = 0.10;
			//				key->con_diff_cop[i][1]->_max = param.support_max[1].y;
			//			}
			//		}
			//	}
			//}else{
			//	for(int i = 0; i < 2; i++)
			//	{
			//		key->con_diff_cop[i][0]->_min = -0.01;
			//		key->con_diff_cop[i][0]->_max = 0.01;
			//		if(lr == 0){
			//			key->con_diff_cop[i][1]->_min = param.support_min[0].y;
			//			key->con_diff_cop[i][1]->_max = -0.10;
			//		}else{
			//			key->con_diff_cop[i][1]->_min = 0.10;
			//			key->con_diff_cop[i][1]->_max = param.support_max[1].y;
			//		}
			//	}
			//}	

		}

/////////////////////////////////////////終端拘束////////////////////////////////////////////////
		if (key->next == 0)
		{
			//終端 初期軌道(11/27)
			if(param.use_initial_trajectory){
			key->pos_t[0]->Set(k,param.terminal_pos.x * 1.5);
			key->pos_t[0]->Set(k,param.terminal_pos.x * 1.5);

			// key->pos_cop[0]->Set(k,param.terminal_pos.x * k * 1 / param.total_step);
			// key->pos_cop[1]->Set(k,param.terminal_pos.y * k * 1 / param.total_step);
			}

			if (param.use_terminal_pos == true)	{
				key->con_fix_pos[0]->desired = param.terminal_pos[0];
				key->con_fix_pos[1]->desired = param.terminal_pos[1];
			}

			if (param.use_terminal_vel == true){
			key->con_fix_vel[0]->desired = param.terminal_vel[0];
			key->con_fix_vel[1]->desired = param.terminal_vel[1];
			}

		}

	}

}




void BipedLIP::Prepare()
{
	TrajectoryNode::Prepare();
}



void BipedLIP::SetPhase(int step, int phase)
{
	param.phase[step] = phase;
}



//-----------------------------------------------------------------------------------------
//位置
vec3_t BipedLIP::Pos(real_t t){
	BipedLIPKey* key = GetSegment(t).first;

	real_t T   = sqrt(param.height/param.gravity);
	real_t t0  = key->tick->time;			//支持期が切り替わった瞬間の時間（支持脚が変わるまで固定）
	real_t px   = key->pos_t[0]->val;		//支持期が切り替わった瞬間の位置（支持脚が変わるまで固定）
	real_t py   = key->pos_t[1]->val;
	real_t vx   = key->vel_t[0]->val;		//支持期が切り替わった瞬間の速度（支持脚が変わるまで固定）
	real_t vy   = key->vel_t[1]->val;
	real_t copx = key->pos_cop[0]->val;		//接地位置　（支持脚が変わるまで固定）
	real_t copy = key->pos_cop[1]->val;
	vec3_t p;

	p.x = copx + (px - copx) * cosh((t-t0)/T)  + (vx*T) * sinh((t-t0)/T)  ;
	p.y = copy + (py - copy) * cosh((t-t0)/T)  + (vy*T) * sinh((t-t0)/T)  ;
	p.z = param.height;

	return p;			//tを受け取り，p(位置)を返す
}



//------------------------------------------------------------------------------------
//速度
vec3_t BipedLIP::Vel(real_t t){
	BipedLIPKey* key = GetSegment(t).first;

	real_t T   = sqrt(param.height/param.gravity);
	real_t t0  = key->tick->time;
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
//加速度

vec3_t BipedLIP::Acc(real_t t){
	BipedLIPKey* key = GetSegment(t).first;

	real_t T    = sqrt(param.height/param.gravity);
	real_t copx = key->pos_cop[0]->val;
	real_t copy = key->pos_cop[1]->val;
	vec3_t p    = Pos(t);
	vec3_t a;

	a.x = (p.x - copx)/(T*T);
	a.y = (p.y - copy)/(T*T);
	a.z = 0.0;

	return a;
}



//付け加え，重心位置
vec3_t BipedLIP::PosCOM(real_t t)
{
	BipedLIPKey* key = GetSegment(t).first;

	return vec3_t(key->pos_t[0]->val, key->pos_t[1]->val, param.height);	//高さを変えるときは変更する必要あり
}

//接地位置
vec3_t BipedLIP::PosCOP(real_t t)
{
	BipedLIPKey* key = GetSegment(t).first;

	return vec3_t(key->pos_cop[0]->val, key->pos_cop[1]->val,0.0);
}



void BipedLIP::Draw(GRRenderIf* render, DrawConfig* conf)
{
	TrajectoryNode::Draw(render, conf);


	//----------------------------- 初期状態を与えるクラス -------------------------------




	////////////////////////座標軸の表示//////////////////////////////////
	render->SetLineWidth(0.1f);			
	render->DrawLine(vec3_t(0.0,0.0,0.0),vec3_t(1.0,0.0,0.0));	//進行方向
	render->DrawLine(vec3_t(0.0,-0.2,0.0),vec3_t(0.0,0.4,0.0));	//横方向(軸は右→左）
	render->DrawLine(vec3_t(0.0,0.0,0.0),vec3_t(0.0,0.0,0.8));	//高さ方向
	/////////////////////////////////////////////////////////////////////


	//初期足先位置の描写
	render->SetPointSize(12.0f);
	render->DrawPoint(vec3_t(param.lfoot_place.x,param.lfoot_place.y,0));
	render->DrawPoint(vec3_t(param.rfoot_place.x,-param.lfoot_place.y,0));




	if(conf->Set(render, DrawItem::ObjectTrajectory, this))
	{

		//////////////////// 使用する変数を定義
		real_t tf =  traj.back() -> tick -> time;	//終端時間
		real_t dt = param.calc_period;		//計算上周期
		real_t cdt = param.control_period;	//制御周期
		real_t t0 = 0.0; 
		real_t t1 = dt;
		Vec3f p0 , p1, p_cop, a ;
		
		
		// \\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\    足先位置・重心位置の呼び出し　\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\ 
		
		//-----------------------------------		 各配列の定義			---------------------------------------------

		//配列の用意
		Vec3d *cop_position;				// 接地位置
		cop_position = (Vec3d *)malloc(sizeof(Vec3d) * MPC::calc.total_step + 2);

		Vec3d *mass_position;				// 支持脚交換時の重心位置
		mass_position = (Vec3d *)malloc(sizeof(Vec3d) * MPC::calc.total_step + 1);

		Vec3d *mass_trajectory;				// 重心軌道
		real_t total_period = (int)(param.step_period_max / param.control_period) * MPC::calc.total_step ;	// 制御周期に従う重心位置数
		mass_trajectory = (Vec3d *)malloc(sizeof(Vec3d) * total_period);

		real_t *change_time;				// 支持脚交換時の時間
		change_time = (real_t *)malloc(sizeof(real_t) * total_period + 1);

		real_t *control_time;				// 時刻
		control_time = (real_t *)malloc(sizeof(real_t) * total_period); 


		//-------------------------- ここで接地位置・交換時の重心位置を格納 ------------------------------
		int mass_number = 0;	//重心軌道 インデックス
		int i = 1 ;			
		int phase_check;		//phaseが1なら右脚遊脚
		BipedLIPKey* start_get_phase = GetSegment(0).first;
		phase_check = start_get_phase->phase;	//最初のphase


		while ( t0 < tf ){
			BipedLIPKey* get_phase = GetSegment(t0).first;

			cop_position[0] = - PosCOP(0);	//最初の遊脚となる位置
			cop_position[1] = PosCOP(0);	//最初の支持脚位置
			mass_position[0] = PosCOM(0);	//最初の重心位置
			change_time[0] = 0.0;			//最初の時刻


			// phaseをチェックし、切り替わったところの足先位置、重心位置
			if ( phase_check != get_phase->phase ){

				cop_position[i+1].x = get_phase->pos_cop[0]->val;	//支持脚交換時の足先位置
				cop_position[i+1].y = get_phase->pos_cop[1]->val;
				cop_position[i+1].z = 0.0;							//高さに変更がない場合
				
				mass_position[i].x = get_phase->pos_t[0]->val;		//支持脚交換時の重心位置
				mass_position[i].y = get_phase->pos_t[1]->val;
				mass_position[i].z = param.height;					//高さに変更がない場合

			change_time[i] = t0;


			//一歩歩いた際の最終速度を格納
			if (i == 1){
				MPC::calc.last_velocity.x = get_phase->vel_t[0]->val;
				MPC::calc.last_velocity.y = get_phase->vel_t[1]->val;

				MPC::calc.last_cop.x	  = get_phase->pos_t[0]->val;
				MPC::calc.last_cop.y	  = get_phase->pos_t[1]->val;
				// cout << "一歩目周期:" << t0 << "\n" ;
			}

			cout << get_phase->pos_t[0]->val << "--";

			phase_check = get_phase->phase ;
			i++	;


			}
			t0 = t0 + cdt;
		}
		
		change_time[i] = t0;				//終端時刻を設定
		mass_position[i] = PosCOM(tf) ;	//終端の重心位置を作成

		//終端の接地位置(最後に脚をおろす場所、対称点)
		for (int ter=0 ; ter<2 ; ter++){
			cop_position[i+1][ter] = 2 * mass_position[i][ter] - cop_position[i][ter] ;
		}
		cop_position[i+1].z = 0.0 ;


		// ------------------------------ 初期化 ----------------------------------------
		t0 = 0.0;	 
		p0 = Pos(t0);

		// -------------------------- 各時刻における重心軌道の算出 ---------------------------
		while(true)
		{
			p_cop = PosCOP(t0);						//
			p1 = Pos(t1);							//


			render->SetLineWidth(0.5f);			
			render->DrawLine(p0, p_cop);			// 重心から接地点までを線で結ぶ

			render->SetPointSize(5.0f);
			render->DrawPoint(p1);					// 重心の位置を描写する


			// ---------------------------- 重心軌道の格納  -------------------------

			//重心位置の保存（ロボットの制御周期に合わせる)
			//dtが変化する(?)ので、適当な閾値（要検討）
			if(fmod(t0,cdt) < dt/4 || fmod(t0,cdt) > dt * (cdt/dt-0.5)){

				mass_trajectory[mass_number].x = p0.x ;
				mass_trajectory[mass_number].y = p0.y ;
				mass_trajectory[mass_number].z = param.height ;	//高さ一定の場合
				control_time[mass_number] = t0 ;
										
				mass_number ++ ;
			}
			
			p0 = p1;	
			t0 = t1;								
			
			if(t1 == tf)
				break;				//終端時間で終了

			t1 = std::min( tf, t1 + dt);		//小さいほうの値を返す

		}


		// \\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\　足先軌道の生成（胴体中心からの足先距離導出）　\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\

		real_t phase_number;		// そのphaseにおける制御点
		real_t all_number;			// 制御開始からの制御点
		real_t number_of_step;		// phase中の重心位置の数
		real_t time;				// 時刻

		real_t mass_pos;			// 支持点から重心位置までの距離

		Vec3d right_foot_pos;		//phase開始時の目標右足距離
		Vec3d left_foot_pos;		//phase開始時の目標左足距離 
			
		Vec3d right_foot_terminal;	//phase終了時の目標右足距離
		Vec3d left_foot_terminal;	//phase終了時の目標左脚距離
			
		Vec3d out_right_foot;		//目標右足距離
		Vec3d out_left_foot;		//目標左脚距離

		Vec3d delta_swing_leg;		//1歩で遊脚の動かすべき距離
		Vec3d period_swing_leg;		//制御周期で遊脚が動かすべき距離
		Vec3d target_swing_leg;		//遊脚目標位置

		BipedLIPKey* start = GetSegment(0).first;	
		int phase = start->phase ;					//最初のphaseを格納 (1なら右足遊脚、ここを切り替えればモーションの左右も入れ替わる)

 


		for (int l=0 ; l<MPC::calc.total_step - 1  ; l++){	
			
			//----- phase開始時の時刻はchange_time[l]、phase終了時の時刻はchange_time[l+1]  
			time = change_time[l];
			number_of_step = (change_time[l+1] - change_time[l] ) / cdt ;
			phase_number = 0.0 ;

			//---- 左脚遊脚のとき
			if (phase == 0){
				
				// 脚交換時の足先距離
				// 引数は左脚先、右足先、重心座標（それぞれDiMP座標）となっている
				Footcalc(cop_position[l],cop_position[l+1],mass_position[l], &left_foot_pos, &right_foot_pos) ;
				Footcalc(cop_position[l+2],cop_position[l+1],mass_position[l+1], &left_foot_terminal, &right_foot_terminal) ;

				// 遊脚軌道描写のための準備
				delta_swing_leg = cop_position[l+2] - cop_position[l] ;
				period_swing_leg = delta_swing_leg / number_of_step ;
				target_swing_leg = cop_position[l];

				while (time < change_time[l+1]){
					
					all_number = time / cdt;


					//^^^^ 高さ (高さの指定は初期姿勢からの変化量とする）
					out_right_foot.z = 0.0;

					if(phase_number == 0.0)	//phaseはじめは高さ0
						out_left_foot.z = param.max_swing_height;
					
					else{
						if (phase_number < number_of_step / 2 )
						out_left_foot.z = param.max_swing_height;

						else{
						out_left_foot.z = param.max_swing_height - (param.max_swing_height * 2 / (number_of_step - 1)) * (phase_number - (number_of_step - 1) / 2) ; 
						}
					}


					
					//^^^^ 前後
					//-- 遊脚は次の目標足先位置に向かって一意に動かす
					out_left_foot.x = left_foot_pos.x + (left_foot_terminal.x - left_foot_pos.x) * phase_number / number_of_step ;
					
					
					// 支持脚は脚交換時以外は計画された重心を保つように動かす
					if(phase_number == 0.0)
					out_right_foot.x = right_foot_pos.x ;

					// 重心と支持脚の前後関係で場合分け
					else{
						
						if(cop_position[l+1].x > mass_trajectory[(int)all_number].x){
						mass_pos = cop_position[l+1].x - mass_trajectory[(int)all_number].x ;
						out_right_foot.x =SupPos( abs(out_left_foot.x), mass_pos );

						}
						
						else{
						mass_pos = mass_trajectory[(int)all_number].x -  cop_position[l+1].x;
						out_right_foot.x = - SupPos( abs(out_left_foot.x),mass_pos);
						}

					}



					//^^^^ 左右
					//-- 遊脚は次の目標足先位置に向かって一意に動かす
					out_left_foot.y = left_foot_pos.y + (left_foot_terminal.y - left_foot_pos.y) * phase_number / number_of_step ;

					
					// 支持脚は脚交換時以外は計画された重心を保つように動かす
					if(phase_number == 0.0)
					out_right_foot.y = right_foot_pos.y ;

					else{
						mass_pos = mass_trajectory[(int)all_number].y - cop_position[l+1].y;
						out_right_foot.y =SupPos(mass_pos , out_left_foot.y) ;

					}
					
					// ---------------- 遊脚軌道の描写
					if (l != 0){
					target_swing_leg = target_swing_leg + period_swing_leg ;
					target_swing_leg.y = target_swing_leg.y + (mass_trajectory[(int)all_number + 1].y - mass_trajectory[(int)all_number].y);
					target_swing_leg.z = out_left_foot.z ;
					render->SetPointSize(5.0f);
					render->DrawPoint(target_swing_leg);

					render->SetLineWidth(0.5f);			
					render->DrawLine(mass_trajectory[(int)all_number], target_swing_leg);
					}

					//出力してインクリメント
					if(MPC::cntrl.m == 0 && MPC::cntrl.rl == phase){
						//out_left_foot.z -= 0.004; 
						Out_motion(out_right_foot,out_left_foot,time,phase);
					}else if(MPC::cntrl.m == 0){
						//out_left_foot.z += 0.005;
						//out_right_foot.z -= 0.01;
						Out_motion(out_right_foot,out_left_foot,time,phase);
					}else{
						Out_motion(out_right_foot,out_left_foot,time,phase);
					}
					time = time + cdt;
					phase_number ++;
				}



				//制御終了時だけは別途出力
				if(l == MPC::calc.total_step - 2){
				Out_motion(right_foot_terminal,left_foot_terminal,time,phase);
				}


			}

			//---- 右脚遊脚のとき
			else{		

				Footcalc(cop_position[l+1],cop_position[l],mass_position[l], &left_foot_pos, &right_foot_pos) ;
				Footcalc(cop_position[l+1],cop_position[l+2],mass_position[l+1], &left_foot_terminal, &right_foot_terminal) ;

				// 遊脚軌道描写のための準備
				delta_swing_leg = cop_position[l+2] - cop_position[l] ;
				period_swing_leg = delta_swing_leg / number_of_step ;
				target_swing_leg = cop_position[l];
				

				while (time < change_time[l+1]){
					all_number = time / cdt;
					
					//^^^^ 高さ
					out_left_foot.z = 0.0;
					
					if(phase_number == 0.0)	//phaseはじめは高さ0
						out_right_foot.z = param.max_swing_height;

					else{
						if (phase_number < number_of_step / 2 )
						out_right_foot.z = param.max_swing_height ;


					else{
						if (phase_number < number_of_step / 2 )
						out_right_foot.z = param.max_swing_height ;

						else{
						out_right_foot.z = param.max_swing_height - (param.max_swing_height * 2 / (number_of_step - 1)) * (phase_number - (number_of_step - 1) / 2) ; 
						}
					}
					}


					//^^^^ 前後
					out_right_foot.x = right_foot_pos.x + (right_foot_terminal.x - right_foot_pos.x) * phase_number / number_of_step ;
					
					// 支持脚は脚交換時以外は計画された重心を保つように動かす
					if(phase_number == 0.0)
					out_left_foot.x = left_foot_pos.x ;
					
					// 重心と支持脚の前後関係で場合分け
					else{
						
						if(cop_position[l+1].x > mass_trajectory[(int)all_number].x){
						mass_pos = cop_position[l+1].x - mass_trajectory[(int)all_number].x ;
						out_left_foot.x =SupPos( abs(out_right_foot.x), mass_pos );

						}
						
						else{
						mass_pos = mass_trajectory[(int)all_number].x -  cop_position[l+1].x;
						out_left_foot.x = - SupPos( abs(out_right_foot.x),mass_pos);
						}

					}


					//^^^^ 左右 
					//-- 遊脚は次の目標足先位置に向かって一意に動かす
					out_right_foot.y = right_foot_pos.y + (right_foot_terminal.y - right_foot_pos.y) * phase_number / number_of_step ;

					// 支持脚は脚交換時以外は計画された重心を保つように動かす
					if(phase_number == 0.0)
					out_left_foot.y = left_foot_pos.y ;

					else{
						mass_pos = cop_position[l+1].y - mass_trajectory[(int)all_number].y;
						out_left_foot.y =SupPos(mass_pos , out_right_foot.y);

					}


					// -------------------- 遊脚軌道の描写 
					
					target_swing_leg = target_swing_leg + period_swing_leg ;
					target_swing_leg.y = target_swing_leg.y + (mass_trajectory[(int)all_number + 1].y - mass_trajectory[(int)all_number].y);

					target_swing_leg.z = out_right_foot.z ;
					render->SetPointSize(5.0f);
					render->DrawPoint(target_swing_leg);

					render->SetLineWidth(0.5f);			
					render->DrawLine(mass_trajectory[(int)all_number], target_swing_leg);




					//出力してインクリメント
					if(MPC::cntrl.m == 0 && MPC::cntrl.rl == phase){
						//out_right_foot.z -= 0.004;
						Out_motion(out_right_foot,out_left_foot,time,phase);
					}else if(MPC::cntrl.m == 0){
						//out_right_foot.z += 0.005;
						//out_left_foot.z -= 0.01;
						Out_motion(out_right_foot,out_left_foot,time,phase);
					}else{
						Out_motion(out_right_foot,out_left_foot,time,phase);
					}
					time = time + cdt;
					phase_number ++;
				}
				

				//制御終了時は別途出力
				if(l == MPC::calc.total_step - 2)
				{
				Out_motion(right_foot_terminal,left_foot_terminal,time,phase);
				}
			}

			
			phase = (phase == 0 ? 1 : 0);		//phaseの切り替え
		}
	
	}
}
	


//（両足支持期）左接地位置、右接地位置、重心位置を受け取り、それを基にして胴体中心からの足先距離を返す

void BipedLIP::Footcalc(Vec3d left_cop,Vec3d right_cop,Vec3d com, Vec3d *left_pos, Vec3d *right_pos)
{


	Vec3d left_calc_pos,right_calc_pos ;	//計算結果をここに格納	
	real_t beta2 = 1 - 1 / param.beta;


	//----------進行方向----------
	real_t delta_front,center_front,delta_com_front;
	real_t delta_foot_front ;		//中心からの足先移動量
	
	delta_front = abs(left_cop.x - right_cop.x) ;	//両足先間距離
	center_front = (left_cop.x + right_cop.x ) / 2;	//両足先中心の座標
	delta_com_front = com.x - center_front;			//両足先中心からのずれ

	delta_foot_front = (2 + param.alpha) * delta_com_front / (2 * beta2 + param.alpha) ;  //中心からの各足先位置変化量
	
	//--- 脚の前後で変わる
	//左脚が前のとき
	if(left_cop.x > right_cop.x){
		left_calc_pos.x  = delta_front / 2.0 - delta_foot_front;
		right_calc_pos.x = - (delta_front / 2.0 + delta_foot_front ) ;
	}

	//右足が前のとき
	else{
		left_calc_pos.x  = - (delta_front / 2.0 + delta_foot_front);
		right_calc_pos.x = delta_front / 2.0 - delta_foot_front;
	}



	//---------- 横方向 ----------
	real_t delta_side,center_side,delta_com_side;
	real_t delta_foot_side;		//中心からの足先移動量


	delta_side = left_cop.y - right_cop.y;			//両足先間距離
	center_side = (left_cop.y + right_cop.y) / 2;	//両足先中心の座標
	delta_com_side = com.y - center_side;			//両足先中心からのずれ 

	delta_foot_side = (2 + param.alpha) * delta_com_side / (2 * beta2 + param.alpha) ;  //中心からの各足先位置変化量

	left_calc_pos.y  = delta_side / 2.0 - delta_foot_side ;	
	right_calc_pos.y = delta_side / 2.0 + delta_foot_side ;	


	//----------高さ方向(ここでは変更なし)-----------
	left_calc_pos.z = 0.0 ;
	right_calc_pos.z = 0.0;



	//---最後にまとめて返す
	*left_pos = left_calc_pos;
	*right_pos = right_calc_pos;

}



//（片足支持期）胴体中心からの遊脚の長さ、支持点からの重心長さを引数とし、胴体中心からの支持脚長さを返す
real_t BipedLIP::SupPos(real_t swing_pos, real_t com_pos){
	real_t sup_pos;	// 支持脚長さ

	sup_pos = ((2 + param.alpha) * com_pos - swing_pos / param.beta) / ( 2 + param.alpha - (1 / param.beta)) ;

	return sup_pos;
}



// enuvo2上では x:front y:height z:side
// enuvo2において0が右脚，1が左脚
//\\\\\\\\\\\\\\\\\\\\\\\\\\\ 足先目標位置を出力するクラス \\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\

// phaseが1なら右足遊脚
void BipedLIP::Out_motion(Vec3d p_r,Vec3d p_l,real_t t,int phase){
	if(!MPC::calc.startcalc){	//以降実行しない
		return;
	}
	
	//足先長さを変換
	pos_right.x = p_r.x + 0.022 ;	//進行方向
	pos_left.x = p_l.x + 0.022 ;

	
	pos_right.y = p_r.y - 80e-3;	//横方向
	pos_left.y = - (p_l.y - 80e-3) ;
	/*斜め接地を考慮する場合
	if(p_r.y > 0.095)		pos_right.y = p_r.y - 80e-3 + 0.0225;	//横方向
	else if(p_r.y < 0.075)	pos_right.y = p_r.y - 80e-3 - 0.0225;
	else					pos_right.y = p_r.y - 80e-3;
	if(p_l.y > 0.095)		pos_left.y = - (p_l.y - 80e-3 + 0.0225);
	else if(p_l.y < 0.075)	pos_left.y = - (p_l.y - 80e-3 - 0.0225);
	else					pos_left.y = - (p_l.y - 80e-3);
	*/

	pos_right.z = p_r.z ;			//高さ
	pos_left.z = p_l.z ;

	
	
	if( t == 0){
		if (param.out_csv){
			ofstream sam( "../../../enuvo/enuvo2/bin/motion/Biped2_start.csv", ios::out );	//ファイル新規作成
			sam << "time, foot0x, foot0y, foot0z, foot1x, foot1y, foot1z\n" ;
			sam << 0 << ","  << 0 << "," <<  0 << "," << 0 << "," ;
			sam << 0 << "," << 0 << "," << 0 << "\n"  ;
			sam << 100 << ","  << pos_right.x << "," << 0 << "," <<  pos_right.y << "," ;
			sam << pos_left.x << "," << 0 << "," << pos_left.y << "\n" << endl ;
		
			ofstream ofs( "../../../enuvo/enuvo2/bin/motion/Biped3.csv", ios::out );	//ファイル新規作成
			ofs << "time, foot0x, foot0y, foot0z, foot1x, foot1y, foot1z\n";

		
		}
		
		if (param.out_enuvo){
			if(MPC::calc.firstcalc){
				MPC::cntrl.timing_changephase = new int[MPC::calc.total_step - 2];
				MPC::cntrl.calcphase = new bool[MPC::calc.total_step - 2];
			}
			MPC::cntrl.prephase = phase;
			int n = param.step_period_max * MPC::calc.total_step / param.control_period;	//要素数
			MPC::cntrl.foot0 = new Vec3f[n];
			MPC::cntrl.foot1 = new Vec3f[n];
			MPC::calc.foot0_log[MPC::calc.stepcount-1] = new Vec3f[n];
			MPC::calc.foot1_log[MPC::calc.stepcount-1] = new Vec3f[n];
			MPC::calc.vel_log[MPC::calc.stepcount-1] = new vec2_t[n];
		}
	}	



	if (param.out_csv){
		ofstream ofs( "../../../enuvo/enuvo2/bin/motion/Biped3.csv", ios::app );	//ファイル更新
		ofs << t * 1000 << ","  << pos_right.x << "," <<  pos_right.z << "," <<  pos_right.y << "," ;
		ofs << pos_left.x << "," << pos_left.z << "," << pos_left.y << "\n" ;
	}


	if (param.out_enuvo){
		if(t != 0){
			MPC::cntrl.foot0[MPC::cntrl.j].x = pos_right.x;
			MPC::cntrl.foot0[MPC::cntrl.j].y = pos_right.z;					
			MPC::cntrl.foot0[MPC::cntrl.j].z = pos_right.y;
			MPC::cntrl.foot1[MPC::cntrl.j].x = pos_left.x;
			MPC::cntrl.foot1[MPC::cntrl.j].y = pos_left.z;
			MPC::cntrl.foot1[MPC::cntrl.j].z = pos_left.y;
			MPC::cntrl.j++;
		}
		//log用
		BipedLIPKey* testkey = GetSegment(t).first;
		MPC::calc.vel_log[MPC::calc.stepcount-1][MPC::cntrl.j].x = Vel(t).x;
		MPC::calc.vel_log[MPC::calc.stepcount-1][MPC::cntrl.j].y = Vel(t).y;
		MPC::calc.foot0_log[MPC::calc.stepcount-1][MPC::cntrl.j].x = pos_right.x;
		MPC::calc.foot0_log[MPC::calc.stepcount-1][MPC::cntrl.j].y = pos_right.z;					
		MPC::calc.foot0_log[MPC::calc.stepcount-1][MPC::cntrl.j].z = pos_right.y;
		MPC::calc.foot1_log[MPC::calc.stepcount-1][MPC::cntrl.j].x = pos_left.x;
		MPC::calc.foot1_log[MPC::calc.stepcount-1][MPC::cntrl.j].y = pos_left.z;
		MPC::calc.foot1_log[MPC::calc.stepcount-1][MPC::cntrl.j].z = pos_left.y;
		MPC::calc.j_log[MPC::calc.stepcount-1] = MPC::cntrl.j+1;

		//phaseが切り替わったときのタイミングを格納
		if(MPC::cntrl.prephase != phase){
			if(MPC::cntrl.m < (MPC::calc.total_step - 2)){
				MPC::cntrl.prephase = phase;
				MPC::cntrl.timing_changephase[MPC::cntrl.m] = MPC::cntrl.j - 1;
				MPC::cntrl.calcphase[MPC::cntrl.m] = phase;
				MPC::cntrl.m ++;
			}
		}

	}


	//最終歩で終了する場合
	if ( t >= traj.back() -> tick -> time){

		if(param.out_enuvo){
			//テスト用
			//BipedLIPKey* testkey = GetSegment(t).first;
			//MPC::calc.testvel_x   = Vel(t).x;
			//MPC::calc.testvel_y   = Vel(t).y;
			
			if(MPC::calc.firstcalc)
				MPC::timer.SetCallback(&MPC::myCallback);

			MPC::cntrl.enuvoCtrl->ikFoot[0].x = MPC::cntrl.foot0[0].x;
			MPC::cntrl.enuvoCtrl->ikFoot[0].y = MPC::cntrl.foot0[0].y;
			MPC::cntrl.enuvoCtrl->ikFoot[0].z = MPC::cntrl.foot0[0].z;
			MPC::cntrl.enuvoCtrl->ikFoot[1].x = MPC::cntrl.foot1[0].x;
			MPC::cntrl.enuvoCtrl->ikFoot[1].y = MPC::cntrl.foot1[0].y;
			MPC::cntrl.enuvoCtrl->ikFoot[1].z = MPC::cntrl.foot1[0].z;

			MPC::cntrl.evCtrlUpdate.Reset();

			// タイマ始動
			MPC::timer.Start(MPC::cntrl.timerPeriod);
			//タイマ終了判定
			MPC::Timerend();
		}

		MPC::calc.startcalc = false;
		MPC::calc.firstcalc = false;
	}
}





void BipedLIP::DrawSnapshot(real_t time, GRRenderIf* render, DrawConfig* conf){
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

	AddSLink(obj[0]->pos_t  [idx]);
	AddSLink(obj[0]->vel_t  [idx]);
	AddSLink(obj[0]->pos_cop[idx]);
	AddSLink(obj[0]->period);
	AddSLink(obj[1]->pos_t  [idx]);

}

LIPConV::LIPConV(Solver* solver, string _name, BipedLIPKey* _obj, uint _idx, real_t _scale):
	LIPCon(solver, _name, _obj, _idx, _scale){
	AddSLink(obj[0]->pos_t  [idx]);
	AddSLink(obj[0]->vel_t  [idx]);
	AddSLink(obj[0]->pos_cop[idx]);
	AddSLink(obj[0]->period);
	AddSLink(obj[1]->vel_t  [idx]);

}


//-------------------------------------------------------------------------------------------------
// CalcCoef(係数）


void LIPCon::Prepare()
{
	BipedLIP::Param& param = obj[0]->GetNode()->param;
	T  = sqrt(param.height/param.gravity);
	t  = obj[0]->period->val/T;
	ch = cosh(t);
	sh = sinh(t);
	p0 = obj[0]->pos_t  [idx]->val;
	v0 = obj[0]->vel_t  [idx]->val;
	c  = obj[0]->pos_cop[idx]->val;
	p1 = obj[1]->pos_t  [idx]->val;
	v1 = obj[1]->vel_t  [idx]->val;
}


void LIPConP::CalcCoef()
{
	Prepare();
	((SLink*)links[0])->SetCoef(-ch);
	((SLink*)links[1])->SetCoef(-T * sh);
	((SLink*)links[2])->SetCoef(ch - 1.0);
	((SLink*)links[3])->SetCoef(- ((p0-c)/T)*sh - v0*ch);
	((SLink*)links[4])->SetCoef(1.0);
}


void LIPConV::CalcCoef()
{
	Prepare();
	((SLink*)links[0])->SetCoef(-(1/T)*sh);
	((SLink*)links[1])->SetCoef(-ch);
	((SLink*)links[2])->SetCoef( (1/T)*sh);
	((SLink*)links[3])->SetCoef(- ((p0-c)/(T*T))*ch - (v0/T)*sh);
	((SLink*)links[4])->SetCoef(1.0);
}



//-------------------------------------------------------------------------------------------------
// CalcDeviation

void LIPConP::CalcDeviation(){
	y[0] =p1 - c - (p0-c)*ch - (v0*T)*sh;	//拘束誤差，これを最小化させる

}

void LIPConV::CalcDeviation(){
	y[0] = v1 - ((p0-c)/T)*sh - v0*ch;
	}
}
