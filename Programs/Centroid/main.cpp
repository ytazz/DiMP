#include <DiMP/DiMP.h>

/**
 centroidal trajectory planning with unscheduled contact
*/

class MyApp : public DiMP::App, public DiMP::Render::Config {
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

	DiMP::Centroid*	centroid;

	vec3_t  targetPos;
	uint    curIdx;

public:
	MyApp() {
		appName = "Centroid";
		zAxisUp = true;

		AddAction(MENU_MAIN, ID_X, "switch to x")->AddHotKey('x');
		AddAction(MENU_MAIN, ID_Y, "switch to y")->AddHotKey('y');
		AddAction(MENU_MAIN, ID_Z, "switch to z")->AddHotKey('z');
		AddAction(MENU_MAIN, ID_INC, "incrase")->AddHotKey('n');
		AddAction(MENU_MAIN, ID_DEC, "decrease")->AddHotKey('m');
		AddAction(MENU_MAIN, ID_SAVE, "save")->AddHotKey('s');

		curIdx = 0;
	}
	virtual ~MyApp() {}

	virtual void BuildScene() {
		centroid = new DiMP::Centroid(graph, "centroid");

		vec3_t startPos(0.0, 0.0, 1.0);
		quat_t startOri = quat_t();
		vec3_t goalPos (1.0, 0.0, 1.0);
		quat_t goalOri  = quat_t();

		centroid->param.g    = 9.8;
		centroid->param.m    = 1.0;
		centroid->param.I    = 1.0;
		centroid->param.h    = 0.1;
		centroid->param.mu   = 0.5;
		centroid->param.dmax = 1.0;
		centroid->param.fmax = 2.0;
		centroid->param.p    = 0.5;

		//centroid->param.l = 1.0;

		centroid->param.ends.resize(1);
		centroid->param.ends[0].rangeMin = vec3_t(-0.25, -0.1, -1.1);
		centroid->param.ends[0].rangeMax = vec3_t( 0.25,  0.1, -0.9);
		//centroid->param.ends[1].rangeMin = vec3_t(-0.25,  0.0, -1.1);
		//centroid->param.ends[1].rangeMax = vec3_t( 0.25,  0.3, -0.9);
		centroid->param.contacts.resize(2);
		centroid->param.contacts[0].iend = 0;
		centroid->param.contacts[0].pos  = vec3_t(startPos.x, startPos.y, 0.0);
		centroid->param.contacts[1].iend = 0;
		centroid->param.contacts[1].pos  = vec3_t(goalPos.x, goalPos.y, 0.0);

		/*
		centroid->param.faces.resize(1);
		DiMP::Centroid::Param::Face& f = centroid->param.faces[0];
		const real_t d = 3.0;
		f.vertices.resize(4);
		f.vertices[0] = vec3_t( 10.0,  10.0, 0.0);
		f.vertices[1] = vec3_t(-10.0,  10.0, 0.0);
		f.vertices[2] = vec3_t(-10.0, -10.0, 0.0);
		f.vertices[3] = vec3_t( 10.0, -10.0, 0.0);
		f.mu  = 1.0;
		f.eta = 1.0;
		*/
		const int N = 30;
		const real_t dt = 0.1;
		for(int k = 0; k <= N; k++)
			new DiMP::Tick(graph, k*dt, "");

		centroid->waypoints.push_back(DiMP::Centroid::Waypoint(0, 0.0, startPos, startOri, vec3_t(), vec3_t(), true, true, true, true));
		centroid->waypoints.back().ends.push_back(DiMP::Centroid::Waypoint::End(vec3_t(startPos.x, startPos.y, 0.0), vec3_t(), false, false));
		//centroid->waypoints.back().ends.push_back(DiMP::Centroid::Waypoint::End(vec3_t(startPos.x, startPos.y + 0.10, 0.0), vec3_t(), true, false));
		centroid->waypoints.push_back(DiMP::Centroid::Waypoint(N, centroid->param.h*N, goalPos, goalOri, vec3_t(), vec3_t(), true, true, true, true));
		centroid->waypoints.back().ends.push_back(DiMP::Centroid::Waypoint::End(vec3_t(goalPos.x, goalPos.y, 0.0), vec3_t(), false, false));
		//centroid->waypoints.back().ends.push_back(DiMP::Centroid::Waypoint::End(vec3_t(goalPos.x, goalPos.y + 0.10, 0.0), vec3_t(), true, false));

		graph->scale.Set(1.0, 1.0, 1.0);
		graph->Init();

		//graph->solver->Enable(ID(DiMP::ConTag::CentroidPosT    ), false);
		//graph->solver->Enable(ID(DiMP::ConTag::CentroidPosR    ), false);
		//graph->solver->Enable(ID(DiMP::ConTag::CentroidVelT    ), false);
		//graph->solver->Enable(ID(DiMP::ConTag::CentroidVelR        ), false);
		//graph->solver->Enable(ID(DiMP::ConTag::CentroidEndRange    ), false);
		//graph->solver->Enable(ID(DiMP::ConTag::CentroidEndPos      ), false);
		//graph->solver->Enable(ID(DiMP::ConTag::CentroidEndCmpl     ), false);
		//graph->solver->Enable(ID(DiMP::ConTag::CentroidContactGap  ), false);
		//graph->solver->Enable(ID(DiMP::ConTag::CentroidContactForce), false);
		//graph->solver->Enable(ID(DiMP::ConTag::CentroidContactCmpl ), false);
		
		graph->solver->SetCorrection(ID(), 0.5);
		graph->solver->param.numIter[0] = 20;
		graph->solver->param.cutoffStepSize = 0.0001;
		graph->solver->param.minStepSize = 0.0001;
		graph->solver->param.maxStepSize = 1.0;
		graph->solver->param.methodMajor = Solver::Method::Major::GaussNewton;
		graph->solver->param.methodMinor = Solver::Method::Minor::Direct;
		graph->solver->param.verbose = true;

		targetPos = vec3_t(0.0, 0.6, -0.2);
	}

	virtual void OnAction(int menu, int id) {
		if (menu == MENU_MAIN) {
			if (id == ID_X) {
				curIdx = 0;
			}
			if (id == ID_Y) {
				curIdx = 1;
			}
			if (id == ID_Z) {
				curIdx = 2;
			}
			if (id == ID_INC) {
			}
			if (id == ID_DEC) {
			}
			if (id == ID_SAVE) {
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
} app;

DiMP::Graph graph;

int main(int argc, char* argv[]) {
	app.graph = &graph;
	app.Init(argc, argv);
	app.StartMainLoop();
}
