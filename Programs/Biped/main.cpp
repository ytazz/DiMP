#include <DiMP2/DiMP.h>

/** 2足ロボットのバランス制御
 */

class MyApp : public DiMP2::App, public DiMP2::DrawConfig{
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
	};

	DiMP2::BipedLIP*	biped;

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

		curIdx = 0;
	}
	virtual ~MyApp(){}

	virtual void BuildScene(){
		biped = new DiMP2::BipedLIP(graph, "biped");
		biped->param.gravity          = 9.8;
		biped->param.heightCoM        = 0.55;
		biped->param.stepPeriodMin    = 0.1;	
		biped->param.stepPeriodMax    = 0.5;
		biped->param.supportPosMin[0] = vec2_t(-0.10, -0.15);
		biped->param.supportPosMax[0] = vec2_t( 0.10, -0.05);
		biped->param.supportPosMin[1] = vec2_t(-0.10,  0.05);
		biped->param.supportPosMax[1] = vec2_t( 0.10,  0.15);
		biped->param.supportOriMin[0] = Rad(-15.0);
		biped->param.supportOriMax[0] = Rad( 15.0);
		biped->param.supportOriMin[1] = Rad(-15.0);
		biped->param.supportOriMax[1] = Rad( 15.0);

		// 歩数設定．tickの時刻は使われない
		const uint N = 10;
		for(uint i = 0; i <= N; i++) 
			new DiMP2::Tick(graph, 0.0, "");
		biped->phase.resize(N+1);
		for(uint i = 0; i < N; i++)
			biped->phase[i] = (i % 2 == 0) ? DiMP2::BipedLIP::Phase::Right : DiMP2::BipedLIP::Phase::Left;
	
		vec2_t goalPos(0.3, 0.3);
		real_t goalOri = Rad(90.0);
		real_t spacing = 0.1;
		biped->waypoints.resize(3);
		biped->waypoints[0].k = 0;
		biped->waypoints[0].com_pos_t     = vec2_t(0.0,  0.0);
		biped->waypoints[0].com_vel_t     = vec2_t(0.0,  0.0);
		biped->waypoints[0].cop_pos_t     = vec2_t(0.0, -spacing);
		biped->waypoints[0].swg_pos_t     = vec2_t(0.0,  spacing);
		biped->waypoints[0].fix_com_pos_t = true;
		biped->waypoints[0].fix_com_vel_t = true;
		biped->waypoints[0].fix_cop_pos_t = true;
		biped->waypoints[0].fix_swg_pos_t = true;
		biped->waypoints[1].k = N-1;
		biped->waypoints[1].cop_pos_t     = goalPos + mat2_t::Rot(goalOri) * vec2_t(0.0,  spacing);
		biped->waypoints[1].fix_cop_pos_t = true;
		biped->waypoints[2].k = N;
		biped->waypoints[2].com_pos_t     = goalPos;
		biped->waypoints[2].com_pos_r     = goalOri;
		biped->waypoints[2].com_vel_t     = vec2_t(0.0, 0.0);
		biped->waypoints[2].cop_pos_t     = goalPos + mat2_t::Rot(goalOri) * vec2_t(0.0, -spacing);
		biped->waypoints[2].fix_com_pos_t = true;
		biped->waypoints[2].fix_com_pos_r = true;
		biped->waypoints[2].fix_com_vel_r = true;
		biped->waypoints[2].fix_cop_pos_t = true;
		
		graph->scale.Set(1.0, 1.0, 1.0);
		graph->Init();

		graph->SetCorrectionRate(DiMP2::ID(), 0.5);
		graph->solver->SetNumIteration(20, 0);
		
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
