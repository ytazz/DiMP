#include "../SampleApp2.h"
using namespace DiMP2;

/** 2足ロボットのバランス制御

 */

class MyApp : public SampleApp, public DiMP2::DrawConfig
{
public:
	enum
	{
		MENU_MAIN = MENU_USER,  //(MENU_USER = 1)
	};
	enum
	{
		ID_X,
		ID_Y,
		ID_Z,
		ID_INC,
		ID_DEC,
	};


	DBipedLIP*			biped;
	

	DiMP2::vec3_t  targetPos;
	DiMP2::uint    curIdx;		//(x,y,zを入力することで0，1，2と変化する)


public:
	MyApp(){
		appName = "Biped";
		zAxisUp = true;

		AddAction(MENU_MAIN, ID_X, "switch to x")->AddHotKey('x');
		AddAction(MENU_MAIN, ID_Y, "switch to y")->AddHotKey('y');
		AddAction(MENU_MAIN, ID_Z, "switch to z")->AddHotKey('z');
		AddAction(MENU_MAIN, ID_INC, "incrase")->AddHotKey('n');
		AddAction(MENU_MAIN, ID_DEC, "decrease")->AddHotKey('m');

		curIdx = 0;
	}
	virtual ~MyApp(){}


	virtual void BuildScene(){								//パラメータ
		biped = graph->CreateBipedLIP("biped");


		//モーションデータ作成時に使用
		biped->param.alpha			= 1.647;								// 足先質量 × alpha = 胴体質量
		biped->param.beta			= 2.333;								// 胴体中心から足先重心までの長さ × beta = 脚長
		biped->param.real_height	= 0.806 ;						// 腰高さ

		biped->param.height    = (biped->param.real_height * (2 + biped->param.alpha - 2 / biped->param.beta) ) / ( 2 + biped->param.alpha) ;	//重心高さ


		//順運動学
		biped->param.lfoot_place.x		= -0.025  + 0.025 ;
		biped->param.lfoot_place.y		=  0.08   + 0.015;
		biped->param.rfoot_place.x		= -0.025  + 0.025 ;
		biped->param.rfoot_place.y		= -(0.08  + 0.015);
		biped->param.gravity         = 9.8;

//		biped->param.height          = 0.5445;	
//		biped->param.swing_vel_max   = 1.0;						///足の最大振り速度

		biped->param.step_period_min = 0.05 ;						///< minimum step period(1周期の時間)
		biped->param.step_period_max = 0.5 ;						///

		biped->param.use_terminal_pos = 1;											// 終端位置を用いる
		biped->param.use_initial_trajectory = 1;									// 初期軌道を与える
		biped->param.terminal_pos    = DiMP2::vec3_t(0.3, 0.0, biped->param.height);	//終端位置

		biped->param.use_terminal_vel = 0;								//終端速度を用いる
		biped->param.terminal_vel    = DiMP2::vec3_t(0.0, 0.0, 0.0);					//終端速度

		biped->param.initial_velocity = DiMP2::vec2_t(0.0, 0.0); // 初速度
																   
		biped->param.max_swing_height = 0.04;					// 遊脚高さの最大値
																// 横歩きだとあまり上げないほうがいい?

																   
		biped->param.calc_period = 0.01 ;						// 計算の分解能を上げると精度がよくなる
		biped->param.control_period = 0.01 ;					// 実際の制御周期
																   
		biped->param.total_step		 = MPC::calc.total_step;						// ステップ数
																   
		biped->param.out_csv   = 1;							// csvファイルを出力する
		biped->param.use_beginning = 1;						// 初期状態を用いる
		biped->param.out_enuvo = 0;							// enuvoの共有メモリに出力する
		biped->param.online    = 0;							// オンライン制御する






		//高さが変わる際の傾き
		biped->param.k_c			 = (biped->param.terminal_pos.z - biped->param.height) / biped->param.terminal_pos.x;

	
		// 歩数設定．tickの時刻は使われない
		for(DiMP2::uint i = 0; i < biped->param.total_step; i++)  //iが歩数となる
			graph->AddTick(0.0);

		for(DiMP2::uint i = 0; i < biped->param.total_step; i++){
			if(i % 2 == 0)
				biped->SetPhase(i, BipedLIP::Left );  //初期Left
			else
				biped->SetPhase(i, BipedLIP::Right );
		}
		
		graph->scale.Set(1.0, 1.0, 1.0);

		graph->Init();

		graph->SetCorrectionRate(ID(), 0.5);
		graph->solver.SetNumIteration(20, 0);
//		targetPos = vec3_t(0.0, 0.6, -0.2);
	}

	virtual void OnAction(int menu, int id)
	{
		if(menu == MENU_MAIN)
		{
			
			if(id == ID_X){
				curIdx = 0;
			}
			if(id == ID_Y){
				curIdx = 1;
			}
			if(id == ID_Z){
				curIdx = 2;
			}
			if(id == ID_INC){
			}
			if(id == ID_DEC){
			}
		}
		SampleApp::OnAction(menu, id);
	}

	
	virtual bool Set(GRRenderIf* render, int attr, DiMP2::Node* node)
	{
		return true;
	}

	
	virtual float Scale(int attr, DiMP2::Node* node)
	{
		return 0.1f;
	}


} app;


DiMP2::Graph graph;


int main(int argc, char* argv[])
{
	app.graph = &graph;
	app.Init(argc, argv);
	app.StartMainLoop();
}
