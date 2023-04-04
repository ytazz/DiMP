#include <DiMP/DiMP.h>

class MyApp : public DiMP::App, public DiMP::Render::Config {
public:
	enum {
		MENU_MAIN = MENU_USER,
	};

	DiMP::BipedCapt*	biped;

	string  saveFilename;

public:
	MyApp() {
		appName = "BipedCapt";
		zAxisUp = true;
	}
	virtual ~MyApp() {}

	virtual void BuildScene() {
		biped = new DiMP::BipedCapt(graph, "biped");
		biped->param.gravity      = 9.8;
		biped->param.comHeight    = 0.75;//1.05;
		biped->param.landPosMin[0] = vec3_t(-0.45, -0.40, 0.0);
		biped->param.landPosMax[0] = vec3_t( 0.45, -0.10, 0.0);
		biped->param.landPosMin[1] = vec3_t(-0.45,  0.10, 0.0);
		biped->param.landPosMax[1] = vec3_t( 0.45,  0.40, 0.0);
		biped->param.landOriMin = Rad(-15.0);
		biped->param.landOriMax = Rad( 15.0);
		biped->param.copMin = vec3_t(-0.10, -0.05, -0.0);
		biped->param.copMax = vec3_t( 0.10,  0.05,  0.0);
		biped->param.vmax   = 2.0;
		biped->param.wmax   = 2.0;
		biped->param.tau_const = 0.05;
        
		int N = 5;
		for(int k = 0; k <= N; k++)
			new DiMP::Tick(graph, 0.0, "");

		biped->param.steps.resize(N+1);
		biped->param.steps[0] = DiMP::BipedCapt::Step(vec3_t(0.0, -0.1, 0.0), 0.0, vec3_t(0.0, -0.1, 0.0), vec3_t(0.0, -0.1, 0.0), 0.5, 0);
		biped->param.steps[1] = DiMP::BipedCapt::Step(vec3_t(0.1,  0.1, 0.0), 0.0, vec3_t(0.1,  0.1, 0.0), vec3_t(0.1,  0.1, 0.0), 0.5, 1);
		biped->param.steps[2] = DiMP::BipedCapt::Step(vec3_t(0.2, -0.1, 0.0), 0.0, vec3_t(0.2, -0.1, 0.0), vec3_t(0.2, -0.1, 0.0), 0.5, 0);
		biped->param.steps[3] = DiMP::BipedCapt::Step(vec3_t(0.3,  0.1, 0.0), 0.0, vec3_t(0.3,  0.1, 0.0), vec3_t(0.3,  0.1, 0.0), 0.5, 1);
		biped->param.steps[4] = DiMP::BipedCapt::Step(vec3_t(0.4, -0.1, 0.0), 0.0, vec3_t(0.4, -0.1, 0.0), vec3_t(0.4, -0.1, 0.0), 0.5, 0);
		biped->param.steps[5] = DiMP::BipedCapt::Step(vec3_t(0.5,  0.1, 0.0), 0.0, vec3_t(0.5,  0.1, 0.0), vec3_t(0.5,  0.1, 0.0), 0.5, 1);

		biped->param.steps[0].icp = vec3_t(0.0, -0.15, 0.0);
		
		graph->scale.Set(1.0, 1.0, 1.0);
		graph->Init();

		graph->solver->SetCorrection(ID(), 0.5);
		graph->solver->param.numIter[0] = 20;
		graph->solver->param.cutoffStepSize = 1.0;
		graph->solver->param.minStepSize = 1.0;
		graph->solver->param.maxStepSize = 1.0;
		//graph->solver->param.methodMajor = Solver::Method::Major::GaussNewton;
		graph->solver->param.methodMajor = Solver::Method::Major::DDP;
		graph->solver->param.methodMinor = Solver::Method::Minor::Direct;
		graph->solver->param.verbose = true;
		
		//graph->solver->Enable(ID(DiMP::ConTag::BipedCaptDuration), false);
		//graph->solver->Enable(ID(DiMP::ConTag::BipedLipVel        ), false);
		//graph->solver->Enable(ID(DiMP::ConTag::BipedFootPosT      ), false);
		//graph->solver->Enable(ID(DiMP::ConTag::BipedFootPosR      ), false);
		//graph->solver->Enable(ID(DiMP::ConTag::BipedFootCop       ), false);
		//graph->solver->Enable(ID(DiMP::ConTag::BipedFootPosRangeT ), false);
		//graph->solver->Enable(ID(DiMP::ConTag::BipedFootPosRangeR ), false);
		//graph->solver->Enable(ID(DiMP::ConTag::BipedFootCopRange  ), false);
		//graph->solver->Enable(ID(DiMP::ConTag::BipedFootVelZeroT  ), false);
		//graph->solver->Enable(ID(DiMP::ConTag::BipedFootVelZeroR  ), false);
		//graph->solver->Enable(ID(DiMP::ConTag::BipedComPos        ), false);
		//graph->solver->Enable(ID(DiMP::ConTag::BipedComVel        ), false);
		//graph->solver->Enable(ID(DiMP::ConTag::BipedDurationRange ), false);
		//graph->solver->Enable(ID(DiMP::ConTag::BipedTime          ), false);

	}

	virtual void OnAction(int menu, int id) {
		if (menu == MENU_MAIN) {
		}
		App::OnAction(menu, id);
	}

	virtual bool Set(GRRenderIf* render, int attr, DiMP::Node* node) {
		return true;
	}

	virtual float Scale(int attr, DiMP::Node* node) {
		return 0.1f;
	}

	void Save(){

	}

} app;

DiMP::Graph graph;

int main(int argc, char* argv[]) {
	app.graph = &graph;
	app.Init(argc, argv);
	app.StartMainLoop();
}
