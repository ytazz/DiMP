#include <DiMP2/Graph.h>
#include <DiMP2/App.h>
using namespace DiMP2;

/** 2足ロボットのバランス制御
 */

class MyApp : public App, public DiMP2::DrawConfig{
public:
	enum{
		MENU_MAIN = MENU_USER, 
	};
	enum{
		ID_X,
		ID_Y,
		ID_Z,
		ID_INC,
		ID_DEC,
		ID_SAVE,
		ID_MOVE
	};

	DiMP2::BipedLIP*	biped;
	DiMP2::BipedLIPKey*	key;

	vec3_t  targetPos;
	uint    curIdx;		

public:
	MyApp(){
		appName = "Biped";
		zAxisUp = true;

		AddAction(MENU_MAIN, ID_X   , "switch to x")->AddHotKey('x');
		AddAction(MENU_MAIN, ID_Y   , "switch to y")->AddHotKey('y');
		AddAction(MENU_MAIN, ID_Z   , "switch to z")->AddHotKey('z');
		AddAction(MENU_MAIN, ID_INC , "incrase"    )->AddHotKey('n');
		AddAction(MENU_MAIN, ID_DEC , "decrease"   )->AddHotKey('m');
		AddAction(MENU_MAIN, ID_SAVE, "save"       )->AddHotKey('s');
		AddAction(MENU_MAIN, ID_MOVE, "description")->AddHotKey('d');

		curIdx = 0;
	}
	virtual ~MyApp(){}

	virtual void BuildScene(){
		biped = graph->CreateBipedLIP("biped");
		biped->param.gravity       = 9.8;
		biped->param.heightCoM     = 0.52;
		biped->param.massTorso	   = 1.0;
		biped->param.massLeg	   = 1.0;
		biped->param.legCoMRatio   = 0.5;				///(初期：0.7)
		//biped->param.swingVelMax   = 1.0;				///足の最大振り速度(初期：1.0）
		biped->param.stepPeriodMin = 0.0;				///< minimum step period(1周期の時間)(初期:0.2)
		biped->param.stepPeriodMax = 0.5;				///(初期:1.0)
		biped->param.swingHeight[0] = 0.1;
		biped->param.supportMin[0] = vec2_t(-0.10,  0.08);
		biped->param.supportMax[0] = vec2_t( 0.10,  0.15);
		biped->param.supportMin[1] = vec2_t(-0.10, -0.15);
		biped->param.supportMax[1] = vec2_t( 0.10, -0.08);
		biped->param.AngAccMin	 =-M_PI;
		biped->param.AngAccMax   = M_PI;
		//biped->param.AngAccMin	 =-1.0;
		//biped->param.AngAccMax   = 1.0;
		//biped->param.tmargin     = 0.0;
		//biped->param.AngleMin	=-0.01; 
		biped->move_trajectory     = false;
		biped->FallAvoidance       = true;				///<転倒回避モード(true)，通常歩行モード(false)切替用(add)
		biped->MotionData		   = true;				

		// 歩数設定．tickの時刻は使われない
		const uint N = 4;
		for(uint i = 0; i <= N; i++) 
			graph->AddTick(0.0);

		//if(!biped->FallAvoidance){
		//	biped->AddWaypoint(0 , vec2_t()        , vec2_t() , real_t() , real_t());
		//	//biped->AddWaypoint(2*8, vec2_t(2*0.3, 0.2), vec2_t(0.2,0.0) , real_t(2*3.14) , real_t());
		//	//biped->AddWaypoint(4*8, vec2_t(4*0.3,-0.2), vec2_t(0.2,0.0) , real_t(-2*3.14) , real_t());
		//	//biped->AddWaypoint(19, vec2_t(0.0, 0.6), vec2_t(-0.2,0.0), real_t(3.14) , real_t());
		//	//biped->AddWaypoint(N-1, vec2_t(0.0, 0.6), vec2_t(-0.2,0.0), real_t(3.14) , real_t());
		//	biped->AddWaypoint(N-1, vec2_t(0.3,0.0), vec2_t(0.0,0.0), real_t() , real_t());
		//}
		//
		//if(biped->FallAvoidance){
		//	biped->AddWaypoint(0 , vec2_t()        , vec2_t() , real_t(3.14) , real_t(3.14));
		//	//biped->AddWaypoint(13, vec2_t(0.3, 0.0), vec2_t() , real_t(0.75) , real_t());
		//	//biped->AddWaypoint(26, vec2_t(0.7, 0.2), vec2_t() , real_t() , real_t());
		//	biped->AddWaypoint(N-1, vec2_t(0.8, 0.5), vec2_t(0.0,0.0), real_t(0.5) , real_t());
		//}

		biped->phase.resize(N+1);
		for(uint i = 0; i <= N; i++)biped->phase[i] = (i % 2 == 0) ? BipedLIP::Phase::Left : BipedLIP::Phase::Right;
		//for(uint i = 0;  <= N; i++)biped->phase[i] = (i % 2 == 0) ? BipedLIP::Phase::Right: BipedLIP::Phase::Left;

		biped->waypoints.resize(2);
		//biped->waypoints.resize(4);
		if(!biped->FallAvoidance){
			//biped->waypoints.resize(4);
			biped->waypoints[0].k = 0;
			biped->waypoints[0].pos_com    = vec2_t(0.0, 0.05);
			biped->waypoints[0].vel_com    = vec2_t(0.0,  0.0);
			biped->waypoints[0].pos_cop    = vec2_t(0.0,  0.1);
			biped->waypoints[0].pos_swg    = vec2_t(0.0, -0.1);
			biped->waypoints[0].ang_com    = real_t(0.0);
			biped->waypoints[0].angvel_com = real_t(0.0);
			biped->waypoints[0].fix_pos_com    = true ;
			biped->waypoints[0].fix_vel_com    = true ;
			biped->waypoints[0].fix_pos_cop    = true ;
			biped->waypoints[0].fix_pos_swg    = true ;
			biped->waypoints[0].fix_ang_com    = true ;
			biped->waypoints[0].fix_angvel_com = true ;
			biped->waypoints[1].k = 8;
			biped->waypoints[1].pos_com    = vec2_t(0.4, 0.0);
			biped->waypoints[1].vel_com    = vec2_t(0.0, 0.0);
			biped->waypoints[1].ang_com    = real_t(0.0);
			biped->waypoints[1].angvel_com = real_t(0.0);
			biped->waypoints[1].fix_pos_com    = true;
			biped->waypoints[1].fix_vel_com    = true;
			biped->waypoints[1].fix_pos_cop    = false;
			biped->waypoints[1].fix_ang_com    = true;
			biped->waypoints[1].fix_angvel_com = true;
			biped->waypoints[2].k = 20;
			biped->waypoints[2].pos_com    = vec2_t(0.4, 0.5);
			biped->waypoints[2].vel_com    = vec2_t(0.0, 0.0);
			biped->waypoints[2].ang_com    = real_t(0.0);
			biped->waypoints[2].angvel_com = real_t(0.0);
			biped->waypoints[2].fix_pos_com    = true;
			biped->waypoints[2].fix_vel_com    = true;
			biped->waypoints[2].fix_pos_cop    = false;
			biped->waypoints[2].fix_ang_com    = true;
			biped->waypoints[2].fix_angvel_com = true;
			biped->waypoints[3].k = N;
			biped->waypoints[3].pos_com    = vec2_t(0.8, 0.5);
			biped->waypoints[3].vel_com    = vec2_t(0.0, 0.0);
			biped->waypoints[3].ang_com    = real_t(0.0);
			biped->waypoints[3].angvel_com = real_t(0.0);
			biped->waypoints[3].fix_pos_com    = true;
			biped->waypoints[3].fix_vel_com    = true;
			biped->waypoints[3].fix_pos_cop    = false;
			biped->waypoints[3].fix_ang_com    = true;
			biped->waypoints[3].fix_angvel_com = true;
		}

		if(biped->FallAvoidance){
			biped->waypoints[0].k = 0;
			biped->waypoints[0].pos_com    = vec2_t(0.0, 0.0);
			biped->waypoints[0].vel_com    = vec2_t(0.3, -0.2);
			biped->waypoints[0].pos_cop    = vec2_t(0.0, 0.1);
			biped->waypoints[0].pos_swg    = vec2_t(0.0, -0.1);
			biped->waypoints[0].ang_com    = real_t(0.0);
			biped->waypoints[0].angvel_com = real_t(-1.5);
			biped->waypoints[0].fix_pos_com    = true ;
			biped->waypoints[0].fix_vel_com    = true ;
			biped->waypoints[0].fix_pos_cop    = true ;
			biped->waypoints[0].fix_pos_swg    = true ;
			biped->waypoints[0].fix_ang_com    = true ;
			biped->waypoints[0].fix_angvel_com = true ;
			biped->waypoints[1].k = N;
			biped->waypoints[1].vel_com    = vec2_t(0.0, 0.0);
			biped->waypoints[1].angvel_com = real_t(0.0);
			biped->waypoints[1].fix_pos_com    = false;
			biped->waypoints[1].fix_vel_com    = true;
			biped->waypoints[1].fix_pos_cop    = false;
			biped->waypoints[1].fix_ang_com    = true;
			biped->waypoints[1].fix_angvel_com = true;
		}

		//if(biped->waypoints[0].angvel_com >= 0.0){
		//	for(uint i = 0; i < N; i++){
		//		biped->SetPhase(i, (i % 2 == 0) ? BipedLIP::Phase::Left : BipedLIP::Phase::Right);
		//		//cout << biped->phase[i] << '\n';
		//	}
		//}
		//if(biped->waypoints[0].angvel_com < 0.0){
		//	for(uint i = 0; i < N; i++)
		//		biped->SetPhase(i, (i % 2 == 0) ? BipedLIP::Phase::Right : BipedLIP::Phase::Left);
		//}
		
		graph->scale.Set(1.0, 1.0, 1.0);
		graph->Init();

		graph->SetCorrectionRate(ID(), 0.5);
		graph->solver.SetNumIteration(20, 0);
		
		targetPos = vec3_t(0.0, 0.6, -0.2);
	}

	virtual void OnAction(int menu, int id){
		if(menu == MENU_MAIN){
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
			if(id == ID_SAVE){
				biped->Save();
			}
			if(id == ID_MOVE){
				if(biped->move_trajectory == false)
					biped->move_trajectory = true;
				else
					biped->move_trajectory = false;
			}
		}
		App::OnAction(menu, id);
	}
	
	virtual bool Set(GRRenderIf* render, int attr, DiMP2::Node* node){
		return true;
	}
	
	virtual float Scale(int attr, DiMP2::Node* node){
		return 0.1f;
	}
} app;

DiMP2::Graph graph;

int main(int argc, char* argv[]){
	app.graph = &graph;
	app.Init(argc, argv);
	app.StartMainLoop();
}
