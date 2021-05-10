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
    FILE*  fileDuration;

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
	}
	virtual ~MyApp() {}

	virtual void BuildScene() {
		centroid = new DiMP::Centroid(graph, "centroid");

		vec3_t startPos(0.0, 0.0, 1.0);
		quat_t startOri = quat_t();
		vec3_t goalPos (3.0, 0.0, 1.0);
		quat_t goalOri  = quat_t::Rot(Rad(30.0), 'z');
		real_t goalTime = 5.0;
		real_t spacing  = 0.2;

		centroid->param.g = 9.8;
		centroid->param.m = 1.0;
		centroid->param.I = 1.0;

        centroid->param.bodyRangeMin = vec3_t(-0.1, -0.1, -0.25);
        centroid->param.bodyRangeMax = vec3_t( 0.1,  0.1,  0.25);
		
		const int nend = 2;
		centroid->param.ends.resize(nend);
		for(int i = 0; i < nend; i++){
			if(i == 0){
				centroid->param.ends[i].basePos     = vec3_t( 0.0, -spacing/2.0,  0.0);
				centroid->param.ends[i].posRangeMin = vec3_t(-0.5,  0.0        , -1.1);
				centroid->param.ends[i].posRangeMax = vec3_t( 0.5,  0.0        , -0.9);
			}
			else{
				centroid->param.ends[i].basePos     = vec3_t( 0.0,  spacing/2.0,  0.0);
				centroid->param.ends[i].posRangeMin = vec3_t(-0.5,  0.0        , -1.1);
				centroid->param.ends[i].posRangeMax = vec3_t( 0.5,  0.0        , -0.9);
			}
			centroid->param.ends[i].velRangeMin    = vec3_t(-1.5, -100.0, -100.0);
			centroid->param.ends[i].velRangeMax    = vec3_t( 1.5,  100.0,  100.0);

			centroid->param.ends[i].momentRangeMin = vec3_t(-0.0, -0.0, -1.0);
			centroid->param.ends[i].momentRangeMax = vec3_t( 0.0,  0.0,  1.0);

            centroid->param.ends[i].copRangeMin = vec2_t(-0.1, -0.05);
			centroid->param.ends[i].copRangeMax = vec2_t( 0.1,  0.05);
		}

        // flat ground
        centroid->faces.push_back(DiMP::Centroid::Face(vec2_t(-10.0, -10.0), vec2_t( 10.0, 10.0), vec3_t(0.0, 0.0, 0.0), quat_t()));
		
        /* 
        // uneven terrain
        centroid->faces.push_back(DiMP::Centroid::Face(vec2_t(-10.0, -10.0), vec2_t( 1.0, 10.0), vec3_t(0.0, 0.0, 0.0), quat_t::Rot(Rad( 10.0), 'x')));
		centroid->faces.push_back(DiMP::Centroid::Face(vec2_t(  2.5, -10.0), vec2_t( 3.0, 10.0), vec3_t(0.0, 0.0, 0.3), quat_t::Rot(Rad(-10.0), 'x')));
		centroid->faces.push_back(DiMP::Centroid::Face(vec2_t(  4.5, -10.0), vec2_t(10.0, 10.0), vec3_t(0.0, 0.0, 0.3), quat_t::Rot(Rad(-10.0), 'x')));
        */

		const int N = 25;
		const real_t dt = goalTime/((real_t)N);
		for(int k = 0; k <= N; k++)
			new DiMP::Tick(graph, k*dt, "");
		
		centroid->waypoints.push_back(DiMP::Centroid::Waypoint(0, 0.0, startPos, startOri, vec3_t(), vec3_t(), true, true, true, true));
		centroid->waypoints.back().ends.push_back(DiMP::Centroid::Waypoint::End(vec3_t(startPos.x, startPos.y - spacing/2.0, 0.0), vec3_t(), true, true));
		centroid->waypoints.back().ends.push_back(DiMP::Centroid::Waypoint::End(vec3_t(startPos.x, startPos.y + spacing/2.0, 0.0), vec3_t(), true, true));

        //centroid->waypoints.push_back(DiMP::Centroid::Waypoint(0, 0.0, startPos, startOri, vec3_t(), vec3_t(), true, true, true, true));
		//centroid->waypoints.back().ends.push_back(DiMP::Centroid::Waypoint::End(vec3_t(startPos.x, startPos.y - spacing/2.0, 0.0), vec3_t(), true, true));
		//centroid->waypoints.back().ends.push_back(DiMP::Centroid::Waypoint::End(vec3_t(startPos.x, startPos.y + spacing/2.0, 0.0), vec3_t(), true, true));

		centroid->waypoints.push_back(DiMP::Centroid::Waypoint(N, dt*N, goalPos, goalOri, vec3_t(), vec3_t(), false, false, false, false));
		centroid->waypoints.back().ends.push_back(DiMP::Centroid::Waypoint::End(vec3_t(goalPos.x, goalPos.y - spacing/2.0, 0.0), vec3_t(), false, false));
		centroid->waypoints.back().ends.push_back(DiMP::Centroid::Waypoint::End(vec3_t(goalPos.x, goalPos.y + spacing/2.0, 0.0), vec3_t(), false, false));
		
		graph->scale.Set(1.0, 1.0, 1.0);
		graph->Init();

		//graph->solver->Enable(ID(DiMP::ConTag::CentroidPosT    ), false);
		//graph->solver->Enable(ID(DiMP::ConTag::CentroidPosR     ), false);
		//graph->solver->Enable(ID(DiMP::ConTag::CentroidVelT     ), false);
		//graph->solver->Enable(ID(DiMP::ConTag::CentroidVelR     ), false);
		//graph->solver->Enable(ID(DiMP::ConTag::CentroidTime     ), false);
		//graph->solver->Enable(ID(DiMP::ConTag::CentroidEndRange ), false);
		//graph->solver->Enable(ID(DiMP::ConTag::CentroidEndPos   ), false);
		//graph->solver->Enable(ID(DiMP::ConTag::CentroidEndStiff ), false);
		//graph->solver->Enable(ID(DiMP::ConTag::CentroidEndMoment), false);
		//graph->solver->Enable(ID(DiMP::ConTag::CentroidEndContact), false);
		//graph->solver->Enable(ID(DiMP::ConTag::CentroidEndVel  ), false);
		//graph->solver->Enable(ID(DiMP::ConTag::CentroidEndForce), false);
		//graph->solver->Enable(ID(DiMP::ConTag::CentroidEndCmpl), false);

		graph->solver->SetCorrection(ID(), 0.5);
		graph->solver->param.numIter[0]     = 20;
		graph->solver->param.cutoffStepSize = 0.5;
		graph->solver->param.minStepSize    = 0.5;
		graph->solver->param.maxStepSize    = 1.0;
        //graph->solver->param.methodMajor    = Solver::Method::Major::GaussNewton;
        graph->solver->param.methodMajor    = Solver::Method::Major::DDP;
		graph->solver->param.methodMinor    = Solver::Method::Minor::Direct;
		graph->solver->param.verbose        = true;

        fileDuration = fopen("duration.csv", "w");
	}

    virtual void OnStep(){
        App::OnStep();

        real_t t = 0.0;
        for (uint k = 0; k < graph->ticks.size(); k++) {
			DiMP::CentroidKey* key = (DiMP::CentroidKey*)centroid->traj.GetKeypoint(graph->ticks[k]);
            fprintf(fileDuration, "%f, ", t);
            if(key->next){
                t += key->var_duration->val;
			}
        }
        fprintf(fileDuration, "\n");
    }

	virtual void OnAction(int menu, int id) {
		if (menu == MENU_MAIN) {
			if(id == ID_SAVE){
				const char* filename = "log.csv";
				FILE* file = fopen(filename, "w");
				for (uint k = 0; k < graph->ticks.size(); k++) {
					DiMP::CentroidKey* key = (DiMP::CentroidKey*)centroid->traj.GetKeypoint(graph->ticks[k]);
					fprintf(file, "%f, %f, ", key->var_duration->val, key->var_time->val);
					fprintf(file, "%f, %f, %f, ", key->var_pos_t->val.x, key->var_pos_t->val.y, key->var_pos_t->val.z);
					fprintf(file, "%f, %f, %f, ", key->var_vel_t->val.x, key->var_vel_t->val.y, key->var_vel_t->val.z);
					
					for(int i = 0; i < key->ends.size(); i++){
						fprintf(file, "%d, ", key->ends[i].contact);
					}
					fprintf(file, "\n");
				}
				fclose(file);
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
