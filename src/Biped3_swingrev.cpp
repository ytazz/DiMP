#include <DiMP2/Graph.h>
#include <DiMP2/Mpc.h>


/*
ここでの方針

○  初期軌道  ○
①軌道計画を初期位置を与えずに行う
②順キネの位置から軌道計画で求められた初期接地位置へと動かすモーションデータを作る
（このとき最初の遊脚は点対称位置に配置する）
③この状態から制御を開始


(○  終端軌道  ○
①軌道計画から求められる最後の支持脚と最後の重心位置の対称点に終了時の支持点を用意
②そこに向かって最後の遊脚を動かすようにモーション作成 )

*/


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
		if ((bool)GetNode()->param.use_terminal_pos == true){
		con_fix_pos[0] = new FixConS(solver, ID(0, node, tick, name + "_fix_pos0"), pos_t[0], node->graph->scale.pos_t);
		con_fix_pos[1] = new FixConS(solver, ID(0, node, tick, name + "_fix_pos1"), pos_t[1], node->graph->scale.pos_t);
		}

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
	

	support_min[0]  = vec2_t( -0.03, -0.13 );  /// 初期(-0.3,-0.2) <歩幅 of Left Phase	(進行方向,横方向）
	support_max[0]  = vec2_t( 0.03, -0.08);		/// 初期( 0.3,-0.05)
	
	support_min[1]  = vec2_t( -0.03,  0.08);  /// 初期(-0.3,0.05) <歩幅 of Right Phase
	support_max[1]  = vec2_t( 0.03,  0.13 );	/// 初期(0.3,0.2)


	

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
	Vec3f m,p,v;											
	// 重心位置
	m.x = (float)pos_t[0]->val;
	m.y = (float)pos_t[1]->val;
	m.z = (float)GetNode()->param.height;
	render->DrawPoint(m);

	cout << "com:" << m << "\n";
	
	render->SetPointSize(8.0f,1);
	// COP位置(接地位置)
	p.x = (float)pos_cop[0]->val;
	p.y = (float)pos_cop[1]->val;
	p.z = 0.0f ;
	
	cout << "cop:" << p << "\n";


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
			if (param.use_terminal_pos == true){
			key->con_fix_pos[0]->desired = param.terminal_pos[0];
			key->con_fix_pos[1]->desired = param.terminal_pos[1];
			}

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
//重心軌道の計算，描写　（Biped3からの書き換えは以下より)


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



void BipedLIP::Draw(GRRenderIf* render, DrawConfig* conf)
{
	TrajectoryNode::Draw(render, conf);


	//----------------------------- 初期状態を与えるクラス -------------------------------




	//////////////////////////座標軸の表示//////////////////////////////////
	//render->SetLineWidth(0.1f);			
	//render->DrawLine(vec3_t(0.0,0.0,0.0),vec3_t(1.0,0.0,0.0));	//進行方向
	//render->DrawLine(vec3_t(0.0,-0.2,0.0),vec3_t(0.0,0.4,0.0));	//横方向(軸は右→左）
	//render->DrawLine(vec3_t(0.0,0.0,0.0),vec3_t(0.0,0.0,0.8));	//高さ方向
	///////////////////////////////////////////////////////////////////////


	//初期足先位置の描写
	//render->SetPointSize(12.0f);
	//render->DrawPoint(vec3_t(0,param.lfoot_place.y,0));
	//render->DrawPoint(vec3_t(0,-param.lfoot_place.y,0));




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
		cop_position = (Vec3d *)malloc(sizeof(Vec3d) * param.total_step + 2);

		Vec3d *mass_position;				// 支持脚交換時の重心位置
		mass_position = (Vec3d *)malloc(sizeof(Vec3d) * param.total_step + 1);

		Vec3d *mass_trajectory;				// 重心軌道
		real_t total_period = (int)(param.step_period_max / param.control_period) * param.total_step ;	// 制御周期に従う重心位置数
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


		//phase切り替え時間チェック
		cout <<"\n"<< "tf : " << tf << "\n" ;
		for(int ch = 0 ; ch < param.total_step ; ch++)
		cout << change_time[ch] << "," ;
		cout << "\n" ;


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


		BipedLIPKey* start = GetSegment(0).first;	
		int phase = start->phase ;					//最初のphaseを格納 (1なら右足遊脚、ここを切り替えればモーションの左右も入れ替わる)



		for (int l=0 ; l<param.total_step - 1  ; l++){	
			
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

				while (time < change_time[l+1]){
					
					all_number = time / cdt;

					//^^^^ 高さ (高さの指定は初期姿勢からの変化量とする）
					out_right_foot.z = 0.0;

					if(phase_number == 0.0)	//phaseはじめは高さ0
					out_left_foot.z = 0.0;
					
					else{
						if (phase_number < number_of_step / 2 )
						out_left_foot.z = param.max_swing_height ;

						else{
						out_left_foot.z = param.max_swing_height - (param.max_swing_height * 2 / number_of_step) * (phase_number - number_of_step / 2) ; 
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
					out_right_foot.y = right_foot_pos.y;

					else{
						mass_pos = mass_trajectory[(int)all_number].y - cop_position[l+1].y;
						out_right_foot.y =SupPos(mass_pos , out_left_foot.y);

					}


					//出力してインクリメント
					Out_motion(out_right_foot,out_left_foot,time,phase);
					time = time + cdt;
					phase_number ++;
				}

				cout << "\n" ;
				//制御終了時だけは別途出力
				if(l == param.total_step - 2)
				Out_motion(right_foot_terminal,left_foot_terminal,time,phase);
				


			}

			//---- 右脚遊脚のとき
			else{		

				Footcalc(cop_position[l+1],cop_position[l],mass_position[l], &left_foot_pos, &right_foot_pos) ;
				Footcalc(cop_position[l+1],cop_position[l+2],mass_position[l+1], &left_foot_terminal, &right_foot_terminal) ;
				

				while (time < change_time[l+1]){
					all_number = time / cdt;
					
					//^^^^ 高さ
					out_left_foot.z = 0.0;
					
					if(phase_number == 0.0)	//phaseはじめは高さ0
					out_right_foot.z = 0.0;
					
					else{
						if (phase_number < number_of_step / 2 )
						out_right_foot.z = param.max_swing_height ;

						else{
						out_right_foot.z = param.max_swing_height - (param.max_swing_height * 2 / number_of_step) * (phase_number - number_of_step / 2) ; 
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
					out_left_foot.y = left_foot_pos.y;

					else{
						mass_pos = cop_position[l+1].y - mass_trajectory[(int)all_number].y;
						out_left_foot.y =SupPos(mass_pos , out_right_foot.y);

					}


					//出力してインクリメント
					Out_motion(out_right_foot,out_left_foot,time,phase);
					time = time + cdt;
					phase_number ++;
				}
				

				//制御終了時は別途出力
				if(l == param.total_step - 2)
				Out_motion(right_foot_terminal,left_foot_terminal,time,phase);

			}
			
			phase = (phase == 0 ? 1 : 0);		//phaseの切り替え
		}

	
	}
}
	


// enuvo2上では x:front y:height z:side
// enuvo2において0が右脚，1が左脚
//\\\\\\\\\\\\\\\\\\\\\\\\\\\ 足先目標位置を出力するクラス \\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\


// 引数は右足先距離、左足先距離、時刻
// 引数にphaseを追加する
void BipedLIP::Out_motion(Vec3d p_r,Vec3d p_l,real_t t,int phase){
	if(!MPC::calc.startcalc){	//以降実行しない
		return;
	}
	

	if(t == 0){
		if (param.out_csv){
			ofstream ofs( "../../../enuvo/enuvo2/bin/motion/delta_pos.csv", ios::out );	//ファイル新規作成
			ofs << "time, foot0x, foot0y, foot0z, foot1x, foot1y, foot1z\n";
		}
		
		if(MPC::calc.firstcalc){
			if (param.out_enuvo){
				// 共有メモリを開く
				MPC::cntrl.enuvoCtrl  = MPC::cntrl.smCtrl .TryOpen<ENuvo2::Control>("ENUVO2_CONTROL_SHARED_MEMORY");
			}
		}
		
		int n = param.step_period_max * param.total_step / param.control_period;	//要素数
		MPC::cntrl.foot0 = new Vec3f[n];
		MPC::cntrl.foot1 = new Vec3f[n];

		counter = 0 ;
		out_check = 0;

	}

	//足先長さを各ローカル座標へと変換
	pos_right.x = p_r.x + 0.03 ;	//進行方向
	pos_left.x = p_l.x + 0.03 ;

	pos_right.y = p_r.y - 80e-3;	//横方向
	pos_left.y = - (p_l.y - 80e-3) ;

	pos_right.z = p_r.z ;//高さ
	pos_left.z = p_l.z ;


 
	if (param.out_csv){
		ofstream ofs( "../../../enuvo/enuvo2/bin/motion/delta_pos.csv", ios::app );	//ファイル更新
		ofs << t * 1000 << ","  << pos_right.x << "," <<  pos_right.z << "," <<  pos_right.y << "," ;
		ofs << pos_left.x << "," << pos_left.z << "," << pos_left.y <<"," <<phase<<"\n" ;
	}

	if (param.out_enuvo){
		MPC::cntrl.foot0[MPC::cntrl.j].x = pos_right.x;
		MPC::cntrl.foot0[MPC::cntrl.j].y = pos_right.z;					
		MPC::cntrl.foot0[MPC::cntrl.j].z = pos_right.y;
		MPC::cntrl.foot1[MPC::cntrl.j].x = pos_left.x;
		MPC::cntrl.foot1[MPC::cntrl.j].y = pos_left.z;
		MPC::cntrl.foot1[MPC::cntrl.j].z = pos_left.y;
		MPC::cntrl.j++;
	}



	/*

	//終了時
	if ( t+param.calc_period > traj.back() -> tick -> time){
		if (param.out_csv){

		}

		if (param.out_enuvo){
			
			if(MPC::calc.firstcalc){
				MPC::timer.SetCallback(&MPC::myCallback);
				MPC::timer2.SetCallback(&MPC::myCallback2);
			}
			// タイマ始動
			MPC::timer.Start(MPC::cntrl.timerPeriod);
			// タイマ2始動
			MPC::timer2.Start(MPC::cntrl.timerPeriod2);

		}

//		cout<< "complete!" <<endl;
		MPC::calc.startcalc = false;
		MPC::calc.firstcalc = false;
		MPC::calc.firstout = true;
	}
	
	*/
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
