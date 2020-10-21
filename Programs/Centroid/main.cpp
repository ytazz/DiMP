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

		centroid->param.gravity = vec3_t(0.0, 0.0, -9.8);
		centroid->param.mass    = 1.0;

		centroid->param.ends.resize(2);
		centroid->param.ends[0].rangeMin = vec3_t(-1.0, -1.0, -1.0);
		centroid->param.ends[0].rangeMax = vec3_t( 1.0,  1.0,  1.0);
		centroid->param.ends[1].rangeMin = vec3_t(-1.0, -1.0, -1.0);
		centroid->param.ends[1].rangeMax = vec3_t( 1.0,  1.0,  1.0);

		centroid->param.faces.resize(1);
		DiMP::Centroid::Param::Face& f = centroid->param.faces[0];
		f.vertices.resize(4);
		f.vertices[0] = vec3_t( 1.0,  1.0, 0.0);
		f.vertices[1] = vec3_t(-1.0,  1.0, 0.0);
		f.vertices[2] = vec3_t(-1.0, -1.0, 0.0);
		f.vertices[3] = vec3_t( 1.0, -1.0, 0.0);
		
		const int N = 10;
		for(int i = 0; i <= N; i++)
			new DiMP::Tick(graph, 0.0, "");

		vec3_t goalPos(2.0, 0.0, 0.0);
		quat_t goalOri = quat_t();

		centroid->waypoints.resize(2);
		centroid->waypoints[0].k = 0;
		centroid->waypoints[0].pos_t = vec3_t(0.0, 0.0, 0.0);
		centroid->waypoints[0].pos_r = quat_t();
		centroid->waypoints[0].vel_t = vec3_t(0.0, 0.0, 0.0);
		centroid->waypoints[0].vel_r = vec3_t(0.0, 0.0, 0.0);
		centroid->waypoints[0].fix_pos_t = true;
		centroid->waypoints[0].fix_pos_r = true;
		centroid->waypoints[0].fix_vel_t = true;
		centroid->waypoints[0].fix_vel_r = true;

		centroid->waypoints[1].k = N;
		centroid->waypoints[1].pos_t = goalPos;
		centroid->waypoints[1].pos_r = goalOri;
		centroid->waypoints[1].vel_t = vec3_t(0.0, 0.0, 0.0);
		centroid->waypoints[1].vel_r = vec3_t(0.0, 0.0, 0.0);
		centroid->waypoints[1].fix_pos_t = true;
		centroid->waypoints[1].fix_pos_r = true;
		centroid->waypoints[1].fix_vel_t = true;
		centroid->waypoints[1].fix_vel_r = true;

		graph->scale.Set(1.0, 1.0, 1.0);
		graph->Init();

		graph->solver->SetCorrection(ID(), 0.5);
		graph->solver->param.numIter[0] = 20;
		graph->solver->param.minStepSize = 1.0;
		graph->solver->param.maxStepSize = 1.0;
		graph->solver->param.methodMajor = Solver::Method::Major::Prioritized;
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
