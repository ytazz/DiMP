#include <DiMP2/Graph.h>
#include <DiMP2/App.h>
using namespace DiMP2;

/** 
enuvo��ObstacleAvoidance�Ɠ��l�̊��D
���ʂ�`�ʂ�����D
�Čv��̃f�����������悤����
*/


class MyApp : public App, public DiMP2::DrawConfig{
public:

	enum{
		MENU_MAIN = MENU_USER, 
	};
	enum{
		ID_SAVE,
		ID_MOVE
	};

	DiMP2::BipedLIP*  biped;
	DiMP2::BipedLIPKey*	key;

	int		walkingPhase;
	int		numWalkingStep;
	float	walkDist;
	float	walkSide;


public:
	MyApp(){
		appName = "Biped";
		zAxisUp = true;

		AddAction(MENU_MAIN, ID_SAVE, "save"       )->AddHotKey('s');
		AddAction(MENU_MAIN, ID_MOVE, "description")->AddHotKey('d');
	}
	virtual ~MyApp(){}

	virtual void BuildScene(){
		
		// Obstacle Avoidance �̏���������
		walkDist		= 0.5;
		walkSide		= 0.2;

		biped = graph->CreateBipedLIP("biped");
		biped->param.gravity        = 9.8;
		biped->param.heightCoM      = 0.52;
		biped->param.torsoMassRatio = 1.0;
		biped->param.legMassRatio   = 0.5;
		biped->param.stepPeriodMin  = 0.1;
		biped->param.stepPeriodMax  = 1.0;
		biped->param.swingHeight[0] = 0.06;
		biped->param.swingHeight[1] = 0.01;
		biped->param.supportMin[0]  = vec2_t(-0.10, -0.14);
		biped->param.supportMax[0]  = vec2_t( 0.10, -0.07);
		biped->param.supportMin[1]  = vec2_t(-0.10, +0.07);
		biped->param.supportMax[1]  = vec2_t( 0.10, +0.14);

		float velObsX,velObsY;
		float iniObsX,iniObsY;
		float time;

		// �ύX����Ƃ��낱������

		walkingPhase	= DiMP2::BipedLIP::Phase::Right;
		numWalkingStep	= 20;


		biped->waypoints.resize(5);
		// ���[����
		biped->waypoints[0].k = 0;
		biped->waypoints[0].fix_pos_com = true;
		biped->waypoints[0].fix_vel_com = true;
		biped->waypoints[0].fix_pos_cop = true;
		biped->waypoints[0].fix_pos_swg = true;

		// �I�[����
		// N-1���ڂ̐ڒn�ʒu
		biped->waypoints[1].k = numWalkingStep - 1;
		biped->waypoints[1].fix_pos_cop = true;
		// N���ځiN-1���ڏI�[�j�̏d�S�ʒu�E���x
		biped->waypoints[2].k = numWalkingStep;
		biped->waypoints[2].fix_pos_com = true;
		biped->waypoints[2].fix_vel_com = true;
		biped->waypoints[2].fix_pos_cop = true;

		// ���Ԑ���
		biped->waypoints[3].k = 6;
		biped->waypoints[3].fix_pos_com = true;
		biped->waypoints[3].fix_vel_com = false;

		// ���Ԑ���
		biped->waypoints[4].k = 14;
		biped->waypoints[4].fix_pos_com = true;
		biped->waypoints[4].fix_vel_com = false;

		biped->phase.resize(numWalkingStep+1);

		for(uint i = 0; i <= numWalkingStep; i++) 
			graph->AddTick(0.0);
	
		for(uint i = 0; i <= numWalkingStep; i++)
			biped->phase[i] = ((i + walkingPhase) % 2 == 0) ? DiMP2::BipedLIP::Phase::Right : DiMP2::BipedLIP::Phase::Left;

			

		// ��������				
		 biped->waypoints[0].pos_com = vec2_t(0.0, (walkingPhase == 0 ? -1.0 :  1.0) * 0.035);
		 biped->waypoints[0].vel_com = vec2_t(0.0,  0.0);
		 biped->waypoints[0].pos_cop = vec2_t(0.0, (walkingPhase == 0 ? -1.0 :  1.0) * 0.1);
		 biped->waypoints[0].pos_swg = vec2_t(0.0, (walkingPhase == 0 ?  1.0 : -1.0) * 0.1);

		
		// �I�[����
		biped->waypoints[1].pos_cop = vec2_t(walkDist, ((walkingPhase + numWalkingStep-1)%2 == 0 ? -1.0 : 1.0) * 0.1 + walkSide);
		biped->waypoints[2].pos_com = vec2_t(walkDist,  walkSide);
		biped->waypoints[2].vel_com = vec2_t(0.0,  0.0);
		biped->waypoints[2].pos_cop = vec2_t(walkDist, ((walkingPhase + numWalkingStep)%2 == 0 ? -1.0 : 1.0) * 0.1 + walkSide);

		// ���Ԑ���
		biped->waypoints[3].pos_com = vec2_t(0.10,0.0);
		biped->waypoints[3].vel_com = vec2_t(0.0,0.0);
		
		// ���Ԑ���
		biped->waypoints[4].pos_com = vec2_t(0.35,0.2);
		biped->waypoints[4].vel_com = vec2_t(0.0,0.0);
		

		graph->scale.Set(1.0, 1.0, 1.0);
		graph->Init();

		graph->SetCorrectionRate(ID(), 0.5);
		graph->solver.SetNumIteration(20, 0);

		biped->move_trajectory = false;
		
	}
	
	virtual void OnAction(int menu, int id){
		if(menu == MENU_MAIN){

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
