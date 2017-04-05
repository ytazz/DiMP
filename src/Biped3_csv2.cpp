#include <DiMP2/Graph.h>
#include <DiMP2/Mpc.h>


namespace DiMP2
{;
//-------------------------------------------------------------------------------------------------
// BipedLIPKey

BipedLIPKey::BipedLIPKey()
{
	phase = BipedLIP::Left;
}



// memo：ここで重心位置・接地位置の計算を行わせている 最初に一回呼び出して終了
void BipedLIPKey::AddVar(Solver* solver)
{
	BipedLIP* obj = GetNode();

	pos_t[0] = new SVar(solver, ID(0, node, tick, name + "_pos0"), node->graph->scale.pos_t); //0:x,1:y
	pos_t[1] = new SVar(solver, ID(0, node, tick, name + "_pos1"), node->graph->scale.pos_t);
	vel_t[0] = new SVar(solver, ID(12, node, tick, name + "_vel0"), node->graph->scale.vel_t);
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

		
		//
		con_diff_cop[0][0] = new DiffConS(solver, ID(0, node, tick, name + "_range_cop0x"), pos_t[0], pos_cop[0], node->graph->scale.pos_t);
		con_diff_cop[0][1] = new DiffConS(solver, ID(0, node, tick, name + "_range_cop0y"), pos_t[1], pos_cop[1], node->graph->scale.pos_t);
		con_diff_cop[1][0] = new DiffConS(solver, ID(0, node, tick, name + "_range_cop1x"), nextObj->pos_t[0], pos_cop[0], node->graph->scale.pos_t);
		con_diff_cop[1][1] = new DiffConS(solver, ID(0, node, tick, name + "_range_cop1y"), nextObj->pos_t[1], pos_cop[1], node->graph->scale.pos_t);
		

	}

	//終端位置
	if(!next)
	{
//		con_fix_pos[0] = new FixConS(solver, ID(0, node, tick, name + "_fix_pos0"), pos_t[0], node->graph->scale.pos_t);
//		con_fix_pos[1] = new FixConS(solver, ID(0, node, tick, name + "_fix_pos1"), pos_t[1], node->graph->scale.pos_t);
		con_fix_vel[0] = new FixConS(solver, ID(0, node, tick, name + "_fix_vel0"), vel_t[0], node->graph->scale.vel_t);
		con_fix_vel[1] = new FixConS(solver, ID(0, node, tick, name + "_fix_vel1"), vel_t[1], node->graph->scale.vel_t);
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
	

	support_min[0]  = vec2_t( -0.03, -0.2 );  /// 初期(-0.3,-0.2) <歩幅 of Left Phase	(進行方向,横方向）
	support_max[0]  = vec2_t( 0.03, -0.05);		/// 初期( 0.3,-0.05)
	
	support_min[1]  = vec2_t( -0.03,  0.05);  /// 初期(-0.3,0.05) <歩幅 of Right Phase
	support_max[1]  = vec2_t( 0.03,  0.2 );	/// 初期(0.3,0.2)


	

}
;


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
	render->DrawPoint(m);					//p点を描写する
	
	render->SetPointSize(8.0f,1);
	// COP位置(接地位置)
	p.x = (float)pos_cop[0]->val;
	p.y = (float)pos_cop[1]->val;
	p.z = 0.0f ;

	
	if(p.x!= 0.0 || p.y != 0.0) {
	render->DrawPoint(p);


	render->SetLineWidth(3.0f);		//初期のzを表示
	render->DrawLine(vec3_t(0.0,0.0,0.0),vec3_t(0.0,0.0,(float)GetNode()->param.height)) ;
	}
	
}



// BipedLIPKey ここまで
//-------------------------------------------------------------------------------------------------
// BipedLIP




BipedLIP::BipedLIP(Graph* g, string n):TrajectoryNode(g, n){
	}



//------------------------------------------------------------------------------------------------------//
//////////////////////////(倒立振子モデルパラメータ)/////////////////////////////////


// 各歩数における拘束条件与える（一回計算して終了）

void BipedLIP::Init()				
{
	TrajectoryNode::Init();

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

				key->pos_cop[0]->val = param.rfoot_place.x;
				key->pos_cop[1]->val = param.rfoot_place.y;

				key->pos_cop[0]->Lock();
				key->pos_cop[1]->Lock();

			}
			
			
			key->vel_t[0]->Lock();
			key->vel_t[1]->Lock();

		}

		
		//（重心が終端位置以外ではkey->nextは1以外ではないか）

		if (key->next)
		{
			// 周期の初期値は下限と上限の中間値
			key->period->val = (param.step_period_min + param.step_period_max) / 2.0  ;



			//
////////////////////////////////// 1Phaseの時間の最小と最大のパラメータ//////////////////////////////

			key->con_range_period->_min = param.step_period_min;
			key->con_range_period->_max = param.step_period_max;


/////////////////////////////////////右左の拘束の切り替え///////////////////////////////////////
			
			bool lr = (key->phase == Left ? 0 : 1);			//PhaseがLeftならlr=0,Rightならlr=1
						
			key->con_diff_cop[lr][0]->_min = param.support_min[lr].x;	
			key->con_diff_cop[lr][0]->_max = param.support_max[lr].x;	
			key->con_diff_cop[lr][1]->_min = param.support_min[lr].y;	
			key->con_diff_cop[lr][1]->_max = param.support_max[lr].y;	

		}


/////////////////////////////////////////終端拘束////////////////////////////////////////////////
		if (key->next == 0)
		{
						
//			key->con_fix_pos[0]->desired = param.terminal_pos[0];
//			key->con_fix_pos[1]->desired = param.terminal_pos[1];
			key->con_fix_vel[0]->desired = param.terminal_vel[0];
			key->con_fix_vel[1]->desired = param.terminal_vel[1];

		
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

	return vec3_t(key->pos_t[0]->val, key->pos_t[1]->val,param.height);	//高さを変えるときは変更する必要あり
}

//接地位置
vec3_t BipedLIP::PosCOP(real_t t)
{
	BipedLIPKey* key = GetSegment(t).first;

	return vec3_t(key->pos_cop[0]->val, key->pos_cop[1]->val,0.0);
}




//------------------------------------------------------------------------------------------------
//重心軌道の計算，描写


void BipedLIP::Draw(GRRenderIf* render, DrawConfig* conf)
{
	TrajectoryNode::Draw(render, conf);


	//----------------------------- 初期状態を与えるクラス -------------------------------
	//if (param.use_beginning == true)
	//	BipedLIP::beginning();

	//////////////////////////座標軸の表示//////////////////////////////////
	//render->SetLineWidth(0.1f);			
	//render->DrawLine(vec3_t(0.0,0.0,0.0),vec3_t(1.0,0.0,0.0));	//進行方向
	//render->DrawLine(vec3_t(0.0,-0.2,0.0),vec3_t(0.0,0.4,0.0));	//横方向(軸は右→左）
	//render->DrawLine(vec3_t(0.0,0.0,0.0),vec3_t(0.0,0.0,0.8));	//高さ方向
	///////////////////////////////////////////////////////////////////////

	
	//csvファイルへ出力する足先目標位置の初期化
	pos_right = vec3_t(0.0,0.0,0.0);
	pos_left = vec3_t(0.0,0.0,0.0);


	//初期足先位置の描写
	render->SetPointSize(12.0f);
	render->DrawPoint(vec3_t(0,param.lfoot_place.y,0));
	render->DrawPoint(vec3_t(0,-param.lfoot_place.y,0));



	if(conf->Set(render, DrawItem::ObjectTrajectory, this))
	{

		//////////////////// 使用する変数を定義
		real_t tf =  traj.back() -> tick -> time;	//終端時間
		real_t dt = param.calc_period;							//周期(初期:0.05)
		real_t t0 = 0.0; 
		real_t t1 = dt;
		Vec3f p0 , p1, p_cop, a ;
		
		// Write_csvに渡す値を用意
		Vec3f delta_swing_leg ;		//遊脚位置差

		Vec3f delta_support_leg ;		//支持脚位置差
		
		
		// \\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\    足先位置・重心位置の呼び出し　\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\ 
		
		//-----------------------------------		 各配列の定義			---------------------------------------------

		//配列の用意
		Vec3f *cop_position;				// 接地位置
		cop_position = (Vec3f *)malloc(sizeof(Vec3f) * param.total_step);

		Vec3f *mass_position;				// 支持脚交換時の重心位置
		mass_position = (Vec3f *)malloc(sizeof(Vec3f) * param.total_step);

		Vec3f *mass_trajectory;				// 重心軌道
		real_t total_period = (int)(param.step_period_max / dt) * param.total_step ;	// 制御周期に従う重心位置数
		mass_trajectory = (Vec3f *)malloc(sizeof(Vec3f) * total_period);

		real_t *change_time;				// 支持脚交換時の時間
		change_time = (real_t *)malloc(sizeof(Vec3f) * total_period);


		//-------------------------- ここで接地位置・交換時の重心位置を格納 ------------------------------
		int mass_number = 0;	//重心軌道 インデックス
		int i = 0 ;			
		int phase_check;		//phaseが1なら右脚遊脚
		BipedLIPKey* start_get_phase = GetSegment(t0).first;
		phase_check = start_get_phase->phase;


		while ( t0 < tf ){
			BipedLIPKey* get_phase = GetSegment(t0).first;

			cop_position[0] = PosCOP(0);
			mass_position[0] = PosCOM(0);
			change_time[0] = 0.0;

			if ( phase_check != get_phase->phase ){

				i ++;
				cop_position[i] = PosCOP(t0);	
				mass_position[i] = PosCOM(t0);
				change_time[i] = t0;
				phase_check = get_phase->phase ;

			}
			
		t0 = t0 + dt;
		}

		change_time[i+1] = tf;				//終端時刻を定義しておく
	



		// ------------------------------ リセット ----------------------------------------
		t0 = 0.0;	 
		p0 = Pos(t0);


		// -------------------------- 各時刻における重心軌道の導出 ---------------------------
		while(true)
		{
			p_cop = PosCOP(t0);						//
			p1 = Pos(t1);							//


			render->SetLineWidth(1.0f);			
			render->DrawLine(p0, p_cop);			// 重心から接地点までを線で結ぶ

			render->SetPointSize(5.0f);
			render->DrawPoint(p1);					// 重心の位置を描写する
			

			// ---------------------------- 重心軌道の格納  -------------------------

			//ここで重心位置を保存する
			mass_trajectory[mass_number].x = p0.x ;
			mass_trajectory[mass_number].y = p0.y ;
			mass_trajectory[mass_number].z = 0.0 ;

			mass_number ++ ;



			p0 = p1;	
			t0 = t1;								

			
			if(t1 == tf)
				break;				//終端時間で終了

		t1 = std::min(tf, t1 + dt);		//小さいほうの値を返す


		
		}


		// \\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\　ループで重心軌道が求められるのでそれを利用　\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\

		real_t phase_time = 0.0;			// phase start time
		real_t phase_number;		// phaseのスタートが保存した重心位置の何番目に当たるか
		real_t number_of_step;		// phase中の重心位置の数

		Vec3f delta_cop_position;		
		Vec3f delta_mass_position;

		Vec3f d_cop_position;			//遊脚軌道


		for (int l=0 ; l<param.total_step - 1 ; l++){
	
		phase_time = change_time[l];
		phase_number = change_time[l] / dt ;	
		number_of_step = (change_time[l+1] - change_time[l]) / dt;	

		delta_mass_position = mass_position[l+1] - mass_position[l];

		//-------------------------------------- 初期の重心位置の決定 ---------------------------------
		if(l==0){

			BipedLIPKey* start = GetSegment(phase_time).first;	//	1なら右脚遊脚

			//最初が右脚遊脚のとき
			if(start->phase == 1){
				d_cop_position = vec3_t(0,-param.lfoot_place.y,0);
			}

			else{
				d_cop_position = vec3_t(0,param.lfoot_place.y,0);
			}

		}

		if(l==1){
			BipedLIPKey* next = GetSegment(phase_time).first;	//	1なら右脚遊脚
			if(next->phase == 1){
				d_cop_position = vec3_t(0,-param.lfoot_place.y,0);
			}

			else{
				d_cop_position = vec3_t(0,param.lfoot_place.y,0);
			}
		}





		//---------------------------------------	初期の挙動	---------------------------------------
		if(l==0){

			BipedLIPKey* start = GetSegment(phase_time).first;	//	1なら右脚遊脚


			while (phase_time < change_time[1]){

				Vec3f delta_mass_trajectory = mass_trajectory[(int)phase_number+1] - mass_trajectory[(int)phase_number] ;

				// ---------------進行方向導出---------------
				d_cop_position.x = d_cop_position.x + delta_mass_trajectory.x * 2;

				
				// csvファイル用進行方向差
				delta_support_leg.x = delta_mass_trajectory.x;		// 支持脚は重心軌道に等しい
				delta_swing_leg.x = - delta_mass_trajectory.x;		// 遊脚は重心軌道のマイナスに等しい
				

				// ----------------横方向導出---------------------
				d_cop_position.y = d_cop_position.y + (cop_position[1].y - param.lfoot_place.y) / number_of_step; 

				//csvファイル用横方向差
				// 注意：「重心が正方向に動く→重心からは負に動く」ことになる
				delta_support_leg.y = (-1) * delta_mass_trajectory.y;							//（支持脚軌道）重心軌道を参照する
				//delta_swing_leg.y = (cop_position[1].y - param.lfoot_place.y) / number_of_step;	//（遊脚軌道）設置位置差に対して徐々に動かしていく
				delta_swing_leg.y = 	 delta_mass_trajectory.y ;								//（遊脚軌道）重心軌道を参照する


				// -----------------高さ方向導出------------------
				if(phase_number  < number_of_step / 2 )	//入れ替わり時間の半分で切り替える
				{
					d_cop_position.z = d_cop_position.z + param.max_swing_height / (number_of_step / 2) ;
					delta_swing_leg.z =  param.max_swing_height / (number_of_step / 2);
				}
				
				else
				{
					d_cop_position.z = d_cop_position.z - param.max_swing_height / (number_of_step / 2) ;
					delta_swing_leg.z =  - param.max_swing_height / (number_of_step / 2);
				}
				
				

				//描写
				render->SetPointSize(5.0f);
				render->DrawPoint(d_cop_position);
				render->SetLineWidth(0.5f);			
				render->DrawLine(d_cop_position, Pos(phase_time));		//重心から接地点までを線で結ぶ



				//csvへ出力
				if (start->phase ==1){
					BipedLIP::Write_csv(delta_swing_leg,delta_support_leg,phase_time,dt);
				}
				else
				{
					BipedLIP::Write_csv(delta_support_leg,delta_swing_leg,phase_time,dt);
				}
			
				phase_number++	;
				phase_time = phase_time + dt;

			}
		}




		// --------------------------------------   終端の軌道   ---------------------------------------- 
		else if(l == param.total_step-2 )
		{
			BipedLIPKey* finish= GetSegment(phase_time).first;	//	1なら右脚遊脚
			d_cop_position = cop_position[l-1];		//初期の遊脚位置は前の支持脚位置に等しい
			int tra_number = 0  ;	// 区間の何番目の重心軌道か


			while (phase_time < change_time[l+1]){

				Vec3f delta_mass_trajectory = mass_trajectory[(int)phase_number] - mass_trajectory[(int)phase_number-1] ;

				// ---------------進行方向導出---------------
				d_cop_position.x = d_cop_position.x + delta_mass_trajectory.x * 2;

				
				// csvファイル用進行方向差
				delta_support_leg.x = delta_mass_trajectory.x;		// 支持脚は重心軌道に等しい
				delta_swing_leg.x = - delta_mass_trajectory.x;		// 遊脚は重心軌道のマイナスに等しい
				

				// ----------------横方向導出---------------------
				d_cop_position.y = d_cop_position.y + delta_mass_trajectory.y;		//ひとまず横方向も重心軌道に従わせる 

				//csvファイル用横方向差
				// 注意：「重心が正方向に動く→重心からは負に動く」ことになる
				delta_support_leg.y = (-1) * delta_mass_trajectory.y;		
				delta_swing_leg.y = 	 delta_mass_trajectory.y ;

				// -----------------高さ方向導出------------------
				if(tra_number < number_of_step / 2 )	//入れ替わり時間の半分で切り替える
				{
					d_cop_position.z = d_cop_position.z + param.max_swing_height / (number_of_step / 2) ;
					delta_swing_leg.z =  param.max_swing_height / (number_of_step / 2);
				}

				else
				{
					d_cop_position.z = d_cop_position.z - param.max_swing_height / (number_of_step / 2) ;
					delta_swing_leg.z =  - param.max_swing_height / (number_of_step / 2);
				}
				
				
				//描写
				render->SetPointSize(5.0f);
				render->DrawPoint(d_cop_position);
				render->SetLineWidth(0.5f);			
				render->DrawLine(d_cop_position, Pos(phase_time));		//重心から接地点までを線で結ぶ
				


				//csvへ出力
				if (finish->phase ==1){
					BipedLIP::Write_csv(delta_swing_leg,delta_support_leg,phase_time,dt);
				}
				else
				{
					BipedLIP::Write_csv(delta_support_leg,delta_swing_leg,phase_time,dt);
				}
			
				phase_number++	;
				phase_time = phase_time + dt;
				tra_number ++ ;



				}
			}





		// --------------------------------------   中間の挙動  ---------------------------------------

		else{
			delta_cop_position = cop_position[l+1] - cop_position[l-1];		//前の接地位置との差を取る
			d_cop_position = cop_position[l-1];		//初期の遊脚位置は前の支持脚位置に等しい
			int tra_number = 0  ;	// 区間の何番目の重心軌道か

			while ( phase_time < change_time[l+1] )
				{
			
					Vec3f delta_mass_trajectory = mass_trajectory[(int)phase_number] - mass_trajectory[(int)phase_number - 1] ;


					//--------------------- 進行方向導出  ----------------------------
					// グロバール下では遊脚の進行方向は重心軌道の2倍の距離を進む
					d_cop_position.x = d_cop_position.x + delta_mass_trajectory.x * 2 ;

					// csv用
					delta_support_leg.x = delta_mass_trajectory.x;		// 支持脚は重心軌道に等しい
					delta_swing_leg.x = - delta_mass_trajectory.x;		// 遊脚は重心軌道のマイナスに等しい
			
					
					//---------------------- 横方向導出 -----------------------------
					// グローバル化では横方向も重心軌道の2倍の距離を進む
					// 一方ローカル化では真っ直ぐ次の接地位置に向かって動かす
					d_cop_position.y = d_cop_position.y +  delta_mass_trajectory.y * 2;

					// 横方向差を求める
					delta_support_leg.y = (-1) * delta_mass_trajectory.y;			//支持脚軌道は重心軌道の負の値に等しい
					//delta_swing_leg.y = delta_cop_position.y / number_of_step;	//（遊脚軌道）接地位置差に対して徐々に動かす
					delta_swing_leg.y = 	 delta_mass_trajectory.y ;				//（誘客起動）重心軌道を参照


					
					//---------------------- 高さ方向導出 ---------------------------
					
					if( tra_number < number_of_step / 2 )	//入れ替わり時間の半分で切り替える
					{
						d_cop_position.z = d_cop_position.z + param.max_swing_height / (number_of_step / 2) ;
						delta_swing_leg.z = param.max_swing_height / (number_of_step / 2);
					}

					else
					{
						d_cop_position.z = d_cop_position.z - param.max_swing_height / (number_of_step / 2) ;
						delta_swing_leg.z =  - param.max_swing_height / (number_of_step / 2);
					}
					

					render->SetPointSize(5.0f);
					render->DrawPoint(d_cop_position);

					render->SetLineWidth(0.5f);			
					render->DrawLine(d_cop_position, Pos(phase_time));			//重心から接地点までを線で結ぶ



					BipedLIPKey* key = GetSegment(phase_time).first;	//	1なら右脚遊脚

					// enuvo2上では x:front y:height z:side
					// enuvo2において0が右脚，1が左脚
					if (key->phase ==1){
						BipedLIP::Write_csv(delta_swing_leg,delta_support_leg,phase_time,dt);
					}

					else
					{
						BipedLIP::Write_csv(delta_support_leg,delta_swing_leg,phase_time,dt);
					}
			

					// \\\\\\\\\\\\\\\\\\\\\\\\\\\\\\  ここまで　\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\

					phase_number++	;
					phase_time = phase_time + dt;
					tra_number ++;
				}
		
			}
		}
	
	}
}

// enuvo2上では x:front y:height z:side
// enuvo2において0が右脚，1が左脚
//\\\\\\\\\\\\\\\\\\\\\\\\\\\ 足先目標位置を出力するクラス \\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\

int cnt = 0;	//cnt=2のとき出力

void BipedLIP::Write_csv(Vec3f p_r,Vec3f p_l,real_t t,real_t dt){

	if(MPC::startmpc == false || MPC::endcalc == true){	//以降実行しない
		return;
	}
	
	if (param.out_csv == true){
		if ( t == 0 && cnt == 0 ){
			ofstream ofs( "../Biped3/data/delta_pos.csv", ios::out );	//ファイル新規作成
			ofs << "time, foot0x, foot0y, foot0z, foot1x, foot1y, foot1z\n";
		}
	}
			
	if(t == 0){	//差がない(小さい)とき
		cnt = 2;
	}

	if( t == 0){
		for (int i = 0 ; i<3 ; i++){	//初期化
			pos_right[i]  = 0.0;
			pos_left[i] = 0.0;
		}

	counter = 0 ;
	out_check = 0;

	/*
	// ---------------------------- ここで初期位置に対する補正を行う --------------------------------

	BipedLIPKey* start_cor = GetSegment(0).first;
			
	if(start_cor->phase == 1)	//左脚に補正を加える
	{
			for(real_t cor_time = 0 ; cor_time < param.start_move_time ; cor_time = cor_time + dt)
			{
			
			}
		}
	
		//(今回はこっちが使われている)
		else	//右脚に補正を加える
		{
			for(real_t cor_time = 0; cor_time < param.start_move_time ; cor_time = cor_time + dt)
			{
			
			}
			
		}
	*/

	}



	pos_right.x = pos_right.x + p_r.x ;	//進行方向
	pos_left.x = pos_left.x + p_l.x ;

	pos_right.y = pos_right.y + p_r.y;	//横方向
	pos_left.y = pos_left.y + p_l.y;

	pos_right.z = pos_right.z + p_r.z ;	//高さ
	pos_left.z = pos_left.z + p_l.z ;

	

	// -------------------------- 支持脚の高さを0にする ---------------------

	BipedLIPKey* reset_height = GetSegment(t).first;
	
	if(reset_height->phase == 1){
		pos_left.z = 0.0;
	}else{
		pos_right.z = 0.0; 
	}

	//周期に応じてcsvに出力するデータを決める
	int output_count = param.control_period / param.calc_period ;

	//横方向はDiMPとenuvoで座標が逆なので-をかける
	//時間には初期位置へ足先を動かす時間を加える

	if (counter % output_count == 0 && cnt == 2){ 
		if (param.out_csv == true){
				ofstream ofs( "../Biped3/data/delta_pos.csv", ios::app );	//ファイル更新
				ofs << (counter * param.control_period / output_count  + param.start_move_time) * 1000 << ","  << pos_right.x << "," <<  pos_right.z << "," <<  (-1) * pos_right.y << "," ;
				ofs << pos_left.x << "," << pos_left.z << "," << (-1) *  pos_left.y << "\n" ;
		}	
		out_check ++ ;
	}


	//終端状態
	//計算上最後の状態にする
	if ( t+dt > traj.back() -> tick -> time && cnt == 2){
		if (param.out_csv == true){
			ofstream ofs( "../Biped3/data/delta_pos.csv", ios::app );	//ファイル更新
			ofs << out_check * param.control_period * 1000<< ","  << pos_right.x << "," <<  0 << "," <<  (-1) * pos_right.y << "," ;
			ofs << pos_left.x << "," << 0 << "," << (-1) *  pos_left.y << "\n" ;
		}	
		MPC::endcalc = true;
		cout<< "complete" <<endl;
	}

	counter ++;

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
