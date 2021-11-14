#include <DiMP/DiMP.h>
#include <DiMP/Graph/Solver.h>

/**
 centroidal trajectory planning with unscheduled contact
*/

const real_t inf = std::numeric_limits<real_t>::max();

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

		vec3_t startPos(0.0, 0.0, 1.0);
		quat_t startOri = quat_t();
		vec3_t goalPos (8.0, 0.0, 1.0);
		quat_t goalOri  = quat_t::Rot(Rad(0.0), 'z');
        real_t duration = 0.3;
		real_t spacing  = 0.2;
        int nend = 2;
        int N    = 14;
        real_t goalTime = duration * N;
		
        centroid = new DiMP::Centroid(graph, "centroid");

		centroid->param.g = 9.8;
		centroid->param.m = 1.0;
		centroid->param.I = 1.0;

        centroid->param.complWeightMin   = 0.1;
        centroid->param.complWeightMax   = 10.0;
        centroid->param.complWeightRate  = 2.0;
        //centroid->param.complWeightDecay = 0.5;

        // create geometry
        centroid->point = new DiMP::Point(graph);
        centroid->hull  = new DiMP::Hull (graph);

        centroid->param.bodyRangeMin = vec3_t(-0.1, -0.1, -0.25);
        centroid->param.bodyRangeMax = vec3_t( 0.1,  0.1,  0.25);
		
		centroid->ends.resize(nend);
		for(int iend = 0; iend < nend; iend++){
            centroid->ends[iend].point = new DiMP::Point(graph);
			
            if(iend == 0){
				centroid->ends[iend].basePos     = vec3_t( 0.0, -spacing/2.0,  0.0);
				centroid->ends[iend].posRangeMin = vec3_t(-0.5, -0.0        , -1.1);
				centroid->ends[iend].posRangeMax = vec3_t( 0.5,  0.0        , -0.9);
			}
			else{
				centroid->ends[iend].basePos     = vec3_t( 0.0,  spacing/2.0,  0.0);
				centroid->ends[iend].posRangeMin = vec3_t(-0.5, -0.0        , -1.1);
				centroid->ends[iend].posRangeMax = vec3_t( 0.5,  0.0        , -0.9);
			}
			centroid->ends[iend].velRangeMin    = vec3_t(-1.5, -100.0, -100.0);
			centroid->ends[iend].velRangeMax    = vec3_t( 1.5,  100.0,  100.0);

			centroid->ends[iend].momentRangeMin = vec3_t(-0.0, -0.0, -1.0);
			centroid->ends[iend].momentRangeMax = vec3_t( 0.0,  0.0,  1.0);

            centroid->ends[iend].copRangeMin = vec2_t(-0.1, -0.05);
			centroid->ends[iend].copRangeMax = vec2_t( 0.1,  0.05);

            centroid->ends[iend].stiffnessMax = 1.0;

            centroid->ends[iend].numSwitchMax = N/2;
		}

        // flat ground
        //centroid->faces.push_back(DiMP::Centroid::Face(vec2_t(-1.0, -5.0), vec2_t( 10.0, 5.0), vec3_t(0.0, 0.0, 0.0), quat_t::Rot(Rad(0.0), 'y')));
		
        // step
        DiMP::Centroid::Face face;
        face.hull = new DiMP::Hull(graph);
        face.hull->vertices.push_back(vec3_t(-1.0, -5.0,  0.0));
        face.hull->vertices.push_back(vec3_t(-1.0,  5.0,  0.0));
        face.hull->vertices.push_back(vec3_t( 8.0, -5.0,  0.0));
        face.hull->vertices.push_back(vec3_t( 8.0,  5.0,  0.0));
        face.hull->CalcBSphere();
        face.normal = vec3_t(0.0, 0.0, 1.0);
        centroid->faces.push_back(face);
        
        //hull = new DiMP::Hull(graph);
        //hull->vertices.push_back(vec3_t( 3.6, -5.0, 0.5));
        //hull->vertices.push_back(vec3_t( 3.6,  5.0, 0.5));
        //hull->vertices.push_back(vec3_t( 8.0, -5.0, 0.5));
        //hull->vertices.push_back(vec3_t( 8.0,  5.0, 0.5));
        //centroid->hulls.push_back(hull);
        //centroid->faces.push_back(DiMP::Centroid::Face(vec2_t(-1.0, -5.0), vec2_t( 3.5, 5.0), vec3_t(0.0, 0.0, 0.0), quat_t()));
		//centroid->faces.push_back(DiMP::Centroid::Face(vec2_t( 3.6, -5.0), vec2_t( 8.0, 5.0), vec3_t(0.0, 0.0, 0.5), quat_t()));

        // uneven terrain
        //centroid->faces.push_back(DiMP::Centroid::Face(vec2_t(-10.0, -10.0), vec2_t( 1.0, 10.0), vec3_t(0.0, 0.0, 0.0), quat_t::Rot(Rad( 10.0), 'x')));
		//centroid->faces.push_back(DiMP::Centroid::Face(vec2_t(  2.5, -10.0), vec2_t( 3.0, 10.0), vec3_t(0.0, 0.0, 0.3), quat_t::Rot(Rad(-10.0), 'x')));
		//centroid->faces.push_back(DiMP::Centroid::Face(vec2_t(  4.5, -10.0), vec2_t(10.0, 10.0), vec3_t(0.0, 0.0, 0.3), quat_t::Rot(Rad(-10.0), 'x')));
        
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

        graph->solver->callback = centroid;

		graph->solver->SetCorrection(ID(), 0.5);
		graph->solver->param.numIter[0]     = 20;
        graph->solver->param.regularization = 1.0e-5;
		graph->solver->param.cutoffStepSize = 0.01;
		graph->solver->param.minStepSize    = 0.01;
		graph->solver->param.maxStepSize    = 1.0;
        //graph->solver->param.methodMajor    = Solver::Method::Major::GaussNewton;
        graph->solver->param.methodMajor    = DiMP::CustomSolver::CustomMethod::SearchDDP;
		graph->solver->param.methodMinor    = Solver::Method::Minor::Direct;
		graph->solver->param.verbose        = false;
    
        fileDuration = fopen("duration.csv", "w");
	}

    virtual void OnStep(){
       App::OnStep();

        //real_t t = 0.0;
        //for (uint k = 0; k < graph->ticks.size(); k++) {
		//	DiMP::CentroidKey* key = (DiMP::CentroidKey*)centroid->traj.GetKeypoint(graph->ticks[k]);
        //    fprintf(fileDuration, "%f, ", t);
        //    if(key->next){
        //        t += key->var_duration->val;
		//	}
        //}
        //fprintf(fileDuration, "\n");
    }

	virtual void OnAction(int menu, int id) {
		if (menu == MENU_MAIN) {
			if(id == ID_SAVE){
                /*
				const char* filename = "schedule.csv";
				FILE* file = fopen(filename, "w");
				for (uint k = 0; k < graph->ticks.size(); k++) {
					DiMP::CentroidKey* key = (DiMP::CentroidKey*)centroid->traj.GetKeypoint(graph->ticks[k]);
                    for(int i = 0; i < key->ends.size(); i++){
                        if(key->ends[i].iface != -1){
                            fprintf(file, "%d, %f, %f\n", i, key->var_time->val, key->var_duration->val);
                        }
                    }
					//fprintf(file, "%f, %f, %f, ", key->var_pos_t->val.x, key->var_pos_t->val.y, key->var_pos_t->val.z);
					//fprintf(file, "%f, %f, %f, ", key->var_vel_t->val.x, key->var_vel_t->val.y, key->var_vel_t->val.z);
					
					//for(int i = 0; i < key->ends.size(); i++){
					//	fprintf(file, "%d, ", key->ends[i].contact);
					//}
					//for(int i = 0; i < key->ends.size(); i++){
					//	fprintf(file, "%f, ", key->ends[i].con_effort->y.norm());
					//}
					//fprintf(file, "\n");
				}
				fclose(file);
                */
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
