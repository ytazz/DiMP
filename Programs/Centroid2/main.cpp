#include <DiMP/DiMP.h>
#include <DiMP/Graph/Solver.h>

#include <sbrollpitchyaw.h>

#include "mpc.h"

/**
 centroidal trajectory planning with unscheduled contact
*/

const real_t inf = std::numeric_limits<real_t>::max();
const vec2_t one2(1.0, 1.0);
const vec3_t one(1.0, 1.0, 1.0);

class MyApp : public DiMP::App, public DiMP::Render::Config{
public:
	enum {
		MENU_MAIN = MENU_USER,
	};
	enum {
		ID_X,
		ID_Y,
		ID_Z,
		ID_INC,
		ID_DEC,
		ID_SAVE,
	};

	Mpc  mpc;

public:
	MyApp() {
		appName = "Centroid2";
		zAxisUp = true;

		AddAction(MENU_MAIN, ID_X, "switch to x")->AddHotKey('x');
		AddAction(MENU_MAIN, ID_Y, "switch to y")->AddHotKey('y');
		AddAction(MENU_MAIN, ID_Z, "switch to z")->AddHotKey('z');
		AddAction(MENU_MAIN, ID_INC, "incrase")->AddHotKey('n');
		AddAction(MENU_MAIN, ID_DEC, "decrease")->AddHotKey('m');
		AddAction(MENU_MAIN, ID_SAVE, "save")->AddHotKey('s');
	}
	virtual ~MyApp() {}

	virtual void BuildScene() {
		mpc.graph = graph;
		mpc.cen   = new DiMP::Centroid(graph, "centroid");
        mpc.Init();
    }

    virtual void OnStep(){
		App::OnStep();
		//graph->solver->param.stateRegularization = 0.0;

		mpc.UpdateInput();
		mpc.UpdateState();

		if(mpc.count > 0 && mpc.count % mpc.updateCycle == 0 && mpc.data_cur.duration > 0.15){
		//if(mpc.planTrigger){
			// store computed result to front buffer
			mpc.UpdateGain();

			// calc input here to set as initial condition of next mpc
			mpc.UpdateInput();

			// reset time and state and start next optimization
			//mpc.cen->Shift();
			mpc.cen->Setup();
			mpc.cen->Reset(true, false, false);
			graph->solver->InitDDP();

			mpc.planTrigger = false;
			//mpc->updateCycle = 10;
			//graph->solver->param.stateRegularization = 1000.0;
		}

		mpc.count++;

		/*
		fprintf(fileCost,
			"%d, %f, %f, %d, %d, %d, %d, "
			"%d, %d, %d, %d, "
			"%d, %d, %d, %d, "
			"\n",
			graph->solver->status.iterCount,
			graph->solver->status.stepSize,
			graph->solver->status.obj,
			graph->solver->status.timePre,
			graph->solver->status.timeDir,
			graph->solver->status.timeStep,
			graph->solver->status.timeMod,
			graph->solver->status.timeTrans,
			graph->solver->status.timeCost,
			graph->solver->status.timeCostGrad,
			graph->solver->status.timeBack,
			graph->TPrepare,
			graph->TPrepareStep,
			graph->TStep,
			graph->TFinish);
		fflush(fileCost);
		*/
		//graph->solver->param.complRelaxation = std::max(0.1, 0.99*graph->solver->param.complRelaxation);
    }

	virtual void OnAction(int menu, int id) {
		if (menu == MENU_MAIN) {
			if(id == ID_SAVE){
				Save();
			}
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
        mpc.SavePlan();
        mpc.SaveTraj();
    }
} app;

DiMP::Graph graph;

int main(int argc, char* argv[]) {
	app.graph = &graph;
	app.Init(argc, argv);
	app.StartMainLoop();
}
