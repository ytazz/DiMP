#include "../SampleApp2csv.h"
using namespace DiMP2;

/** 2足ロボットのバランス制御

ここでは平坦面上で様々な歩行をさせることを目的とする

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
	

	vec3_t  targetPos;
	uint    curIdx;		//(x,y,zを入力することで0，1，2と変化する)


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
		float l1=81.965e-3, l2=55.049e-3, l3=230e-3, l4=230e-3, l5=102e-3, theta1=0.0, theta2=0.0, theta3=0.0, theta4=0.0, theta5=0.0,c1,c2,c3,c4,c5,s1,s2,s3,s4,s5; 

		c1	=	cos(theta1);
		c2	=	cos(theta2);
		c3	=	cos(theta3);
		c4	=	cos(theta4);
		c5	=	cos(theta5);
		s1	=	sin(theta1);
		s2	=	sin(theta2);
		s3	=	sin(theta3);
		s4	=	sin(theta4);
		s5	=	sin(theta5);

		biped = graph->CreateBipedLIP("biped");

		//順運動学
		biped->param.lfoot_place.x			= -c2*(s5*l5*(c3*c4-s3*s4)+(c5*l5+l4)*(c3*s4+s3*c4)+s3*l3);
		biped->param.lfoot_place.y			= c1*s2*(s5*l5*(c3*c4-s3*s4)+(c5*l5+l4)*(c3*s4+s3*c4)+s3*l3)-s1*(s5*l5*(s3*c4+c3*s4)+(c5*l5+l4)*(s3*s4-c3*c4)-c3*l3-l2)+l1;
		biped->param.rfoot_place.x			= - biped->param.lfoot_place.x;
		biped->param.rfoot_place.y			= - biped->param.lfoot_place.y;
		
		biped->param.gravity         = 9.8;
		biped->param.height          = 0.65;				
//		biped->param.swing_vel_max   = 1.0;						///足の最大振り速度（現在使用不可能）
		biped->param.step_period_min = 0.1;						///< minimum step period(1周期の時間)
		biped->param.step_period_max = 1.0;						///
		biped->param.terminal_vel    = vec3_t(0.0, 0.0, 0.0);	//終端速度
		biped->param.terminal_pos    = vec3_t(0.2, 0.0, biped->param.height);	//終端位置

		biped->param.initial_velocity = vec2_t(0.0, 0.0);		//初速度

		biped->param.start_move_time = 2.0;						//初期位置まで足先位置を動かす時間

		biped->param.max_swing_height = 0.15;					//遊脚高さの最大値

		biped->param.calc_period = 0.02 ;						//計算の分解能を上げると精度がよくなる
		biped->param.control_period = 0.10 ;					//実際の制御周期

		biped->param.total_step		 = 3;						//ステップ数

		biped->param.out_csv = true		;					//csvファイルを出力する
		biped->param.use_beginning = true ;						//初期状態を用いる
		biped->param.use_terminal_pos = false ;					//終端位置を与えるか



		//高さが変わる際の傾き
		biped->param.k_c			 = (biped->param.terminal_pos.z - biped->param.height) / biped->param.terminal_pos.x;

		


		// 歩数設定．tickの時刻は使われない
		for(uint i = 0; i < biped->param.total_step ; i++)  //iが歩数となる
			graph->AddTick(0.0);
		biped->SetPhase(0, BipedLIP::Left );  //初期Left
		biped->SetPhase(1, BipedLIP::Right);
		biped->SetPhase(2, BipedLIP::Left );
		biped->SetPhase(3, BipedLIP::Right);
		biped->SetPhase(4, BipedLIP::Left );
		biped->SetPhase(5, BipedLIP::Right);
		biped->SetPhase(6, BipedLIP::Left );
		biped->SetPhase(7, BipedLIP::Right);
		biped->SetPhase(8, BipedLIP::Left );
		biped->SetPhase(9, BipedLIP::Right);
		biped->SetPhase(10, BipedLIP::Left );
		biped->SetPhase(11, BipedLIP::Right);
		biped->SetPhase(12, BipedLIP::Left );
		biped->SetPhase(13, BipedLIP::Right);
		biped->SetPhase(14, BipedLIP::Left );
		biped->SetPhase(15, BipedLIP::Right);
		biped->SetPhase(16, BipedLIP::Left );
		biped->SetPhase(17, BipedLIP::Right);
		biped->SetPhase(18, BipedLIP::Left );
		biped->SetPhase(19, BipedLIP::Right);
		biped->SetPhase(20, BipedLIP::Left );
		biped->SetPhase(21, BipedLIP::Right);
		biped->SetPhase(22, BipedLIP::Left );
		biped->SetPhase(23, BipedLIP::Right);


	
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
