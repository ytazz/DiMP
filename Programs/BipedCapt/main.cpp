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
		biped->param.landOriMin[0] = Rad(-15.0);
		biped->param.landOriMax[0] = Rad( 15.0);
		biped->param.landOriMin[1] = Rad(-15.0);
		biped->param.landOriMax[1] = Rad( 15.0);
		biped->param.copMin[0] = vec3_t(-0.10, -0.05, -0.0);
		biped->param.copMax[0] = vec3_t( 0.10,  0.05,  0.0);
		biped->param.copMin[1] = vec3_t(-0.10, -0.05, -0.0);
		biped->param.copMax[1] = vec3_t( 0.10,  0.05,  0.0);
		biped->param.copMinDsp[0] = vec3_t(-0.10, -0.05, -0.0);
		biped->param.copMaxDsp[0] = vec3_t( 0.10,  0.25,  0.0);
		biped->param.copMinDsp[1] = vec3_t(-0.10, -0.25, -0.0);
		biped->param.copMaxDsp[1] = vec3_t( 0.10,  0.05,  0.0);
		biped->param.vmax   = 20.0;
		biped->param.wmax   = 20.0;
		biped->param.tau_const = 0.05;
        
		int N = 7;
		for(int k = 0; k <= N; k++)
			new DiMP::Tick(graph, 0.0, "");

		biped->param.steps.resize(N+1);
		biped->param.steps[0] = DiMP::BipedCapt::Step(vec3_t(0.0, -0.1, 0.0), 0.0, vec3_t(0.0,  0.1, 0.0), 0.0, vec3_t(0.0, -0.1, 0.0), vec3_t(0.0, -0.1, 0.0), 0.5, 0, true);
		biped->param.steps[1] = DiMP::BipedCapt::Step(vec3_t(0.0,  0.1, 0.0), 0.0, vec3_t(0.0, -0.1, 0.0), 0.0, vec3_t(0.0,  0.1, 0.0), vec3_t(0.0,  0.1, 0.0), 0.5, 1, true);
		biped->param.steps[2] = DiMP::BipedCapt::Step(vec3_t(0.0, -0.1, 0.0), 0.0, vec3_t(0.0,  0.1, 0.0), 0.0, vec3_t(0.0, -0.1, 0.0), vec3_t(0.0, -0.1, 0.0), 0.5, 0, true);
		biped->param.steps[3] = DiMP::BipedCapt::Step(vec3_t(0.1,  0.1, 0.0), 0.0, vec3_t(0.0, -0.1, 0.0), 0.0, vec3_t(0.1,  0.1, 0.0), vec3_t(0.1,  0.1, 0.0), 0.5, 1, true);
		biped->param.steps[4] = DiMP::BipedCapt::Step(vec3_t(0.2, -0.1, 0.0), 0.0, vec3_t(0.1,  0.1, 0.0), 0.0, vec3_t(0.2, -0.1, 0.0), vec3_t(0.2, -0.1, 0.0), 0.5, 0, true);
		biped->param.steps[5] = DiMP::BipedCapt::Step(vec3_t(0.3,  0.1, 0.0), 0.0, vec3_t(0.2, -0.1, 0.0), 0.0, vec3_t(0.3,  0.1, 0.0), vec3_t(0.3,  0.1, 0.0), 0.5, 1, true);
		biped->param.steps[6] = DiMP::BipedCapt::Step(vec3_t(0.4, -0.1, 0.0), 0.0, vec3_t(0.3,  0.1, 0.0), 0.0, vec3_t(0.4, -0.1, 0.0), vec3_t(0.4, -0.1, 0.0), 0.5, 0, true);
		biped->param.steps[7] = DiMP::BipedCapt::Step(vec3_t(0.5,  0.1, 0.0), 0.0, vec3_t(0.4, -0.1, 0.0), 0.0, vec3_t(0.5,  0.1, 0.0), vec3_t(0.5,  0.1, 0.0), 0.5, 1, true);

		biped->param.steps[0].icp = vec3_t(0.15, 0.15, 0.0);
		
		graph->scale.Set(1.0, 1.0, 1.0);
		graph->Init();

		// gauss-newton: set positive correction rate
		// ddp: set correction rate as zero
		graph->solver->SetCorrection(ID(), 0.5);
		graph->solver->param.numIter[0] = 20;
		graph->solver->param.regularization = 1.0e-1;
		graph->solver->param.cutoffStepSize = 0.1;
		graph->solver->param.minStepSize = 1.0;
		graph->solver->param.maxStepSize = 1.0;
		//graph->solver->param.methodMajor = Solver::Method::Major::GaussNewton;
		graph->solver->param.methodMajor = Solver::Method::Major::DDP;
		graph->solver->param.methodMinor = Solver::Method::Minor::Direct;
		graph->solver->param.verbose = true;
		
		//graph->solver->Enable(ID(DiMP::ConTag::BipedCaptSupT      ), false);
		//graph->solver->Enable(ID(DiMP::ConTag::BipedCaptSupR      ), false);
		//graph->solver->Enable(ID(DiMP::ConTag::BipedCaptSwgT      ), false);
		//graph->solver->Enable(ID(DiMP::ConTag::BipedCaptSwgR      ), false);
		//graph->solver->Enable(ID(DiMP::ConTag::BipedCaptIcp       ), false);
		//graph->solver->Enable(ID(DiMP::ConTag::BipedCaptCop       ), false);
		//graph->solver->Enable(ID(DiMP::ConTag::BipedCaptDuration  ), false);
		//graph->solver->Enable(ID(DiMP::ConTag::BipedCaptLandRangeT), false);
		//graph->solver->Enable(ID(DiMP::ConTag::BipedCaptLandRangeR), false);
		//graph->solver->Enable(ID(DiMP::ConTag::BipedCaptCopRange  ), false);
		//graph->solver->Enable(ID(DiMP::ConTag::BipedCaptIcpRange  ), false);

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
