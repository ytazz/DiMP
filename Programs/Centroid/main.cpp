#include <DiMP/DiMP.h>
#include <DiMP/Graph/Solver.h>

/**
 centroidal trajectory planning with unscheduled contact
*/

const real_t inf = std::numeric_limits<real_t>::max();

class MyCentroidCallback : public DiMP::CentroidCallback{
public:
    virtual void EnableContact(DiMP::CentroidEndContactCon* con){
        if( ((con->iend == 0 || con->iend == 1) && (con->iface == 0 || con->iface == 1))
            ||
            ((con->iend == 2) && (con->iface == 2)) ||
            ((con->iend == 3) && (con->iface == 3))
            )
        {
            con->enabled = true;
        }
        else{
            con->enabled = false;
        }
    }
};

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

	DiMP::Centroid*	   centroid;
    MyCentroidCallback callback;
    FILE*              fileDuration;
    
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
        const real_t h = 0.7;
		vec3_t startPos(0.0, 0.0, h);
		quat_t startOri = quat_t();
		vec3_t goalPos (3.5, 0.0, h + 0.0);
		quat_t goalOri  = quat_t::Rot(Rad(0.0), 'z');
        const real_t duration = 0.3;
		const real_t legSpacing  = 0.2;
        const real_t armSpacing  = 0.25;
        const int nend = 4;
        const int N    = 12;
        const real_t goalTime = duration * N;

        vec3_t endBasePos  [nend];
        vec3_t endPosOrigin[nend];
        vec3_t endPosMin   [nend];
        vec3_t endPosMax   [nend];
        int    endIni      [nend];
        int    endTerm     [nend];
        //int    endSwitch   [nend];
        endBasePos  [0] = vec3_t( 0.0, -legSpacing/2.0,  0.0);
        endBasePos  [1] = vec3_t( 0.0,  legSpacing/2.0,  0.0);
        endBasePos  [2] = vec3_t( 0.0, -armSpacing/2.0,  0.5);
        endBasePos  [3] = vec3_t( 0.0,  armSpacing/2.0,  0.5);
        endPosOrigin[0] = vec3_t( 0.0,  0.0, -0.7);
        endPosOrigin[1] = vec3_t( 0.0,  0.0, -0.7);
        endPosOrigin[2] = vec3_t( 0.0, -0.1, -0.7);
        endPosOrigin[3] = vec3_t( 0.0,  0.1, -0.7);
        endPosMin   [0] = vec3_t(-0.3, -0.1, -0.8);
        endPosMin   [1] = vec3_t(-0.3, -0.1, -0.8);
        endPosMin   [2] = vec3_t(-0.6, -0.5, -0.7);
        endPosMin   [3] = vec3_t(-0.6,  0.0, -0.7);
        endPosMax   [0] = vec3_t( 0.3,  0.1, -0.2);
        endPosMax   [1] = vec3_t( 0.3,  0.1, -0.2);
        endPosMax   [2] = vec3_t( 0.6,  0.0,  0.0);
        endPosMax   [3] = vec3_t( 0.6,  0.5,  0.0);
        endIni      [0] = 0;
        endIni      [1] = 0;
        endIni      [2] = -1;
        endIni      [3] = -1;
        endTerm     [0] = 0;
        endTerm     [1] = 0;
        endTerm     [2] = -1;
        endTerm     [3] = -1;
        //endSwitch   [0] = 4;
        //endSwitch   [1] = 4;
        //endSwitch   [2] = 2;
        //endSwitch   [3] = 2;
		
        centroid = new DiMP::Centroid(graph, "centroid");
        //centroid->contactSequence.resize(nend);
        //centroid->contactSequence[0](0)(-1)(0)(-1)(0)(-1)(0);
        //centroid->contactSequence[1](0)(-1)(0)(-1)(0)(-1)(0);
        //centroid->contactSequence[2](-1)(0)(-1);
        //centroid->contactSequence[3](-1)(0)(-1);
        centroid->callback = &callback;

		centroid->param.g = 9.8;
		centroid->param.m = 44.0;
		centroid->param.I = centroid->param.m * h*h;

        centroid->param.complWeightMin   = 0.1;
        centroid->param.complWeightMax   = 100.0;
        centroid->param.complWeightRate  = 100.0;
        
        // create geometry
        centroid->point = new DiMP::Point(graph);
        centroid->hull  = new DiMP::Hull (graph);

        centroid->param.bodyRangeMin = vec3_t(-0.1, -0.1,  0.0);
        centroid->param.bodyRangeMax = vec3_t( 0.1,  0.1,  0.5);
		
		centroid->ends.resize(nend);
		for(int iend = 0; iend < nend; iend++){
            centroid->ends[iend].point = new DiMP::Point(graph);
			
            centroid->ends[iend].basePos     = endBasePos[iend];
			centroid->ends[iend].posRangeMin = endPosMin [iend];
			centroid->ends[iend].posRangeMax = endPosMax [iend];
			
            centroid->ends[iend].velRangeMin    = vec3_t(-1.5, -1.0, -1.0);
			centroid->ends[iend].velRangeMax    = vec3_t( 1.5,  1.0,  1.0);

			centroid->ends[iend].momentRangeMin = vec3_t(-0.0, -0.0, -1.0);
			centroid->ends[iend].momentRangeMax = vec3_t( 0.0,  0.0,  1.0);

            centroid->ends[iend].copRangeMin = vec2_t(-0.1, -0.05);
			centroid->ends[iend].copRangeMax = vec2_t( 0.1,  0.05);

            centroid->ends[iend].stiffnessMax = 100.0;

            //centroid->ends[iend].numSwitchMax    = endSwitch[iend];
            centroid->ends[iend].contactInitial  = endIni [iend];
            centroid->ends[iend].contactTerminal = endTerm[iend];
		}
        
        DiMP::Centroid::Face face;
        // flat ground
        face.hull = new DiMP::Hull(graph);
        face.hull->vertices.push_back(vec3_t(-1.0, -5.0,  0.0));
        face.hull->vertices.push_back(vec3_t(-1.0,  5.0,  0.0));
        face.hull->vertices.push_back(vec3_t( 5.0,  5.0,  0.0));
        face.hull->vertices.push_back(vec3_t( 5.0, -5.0,  0.0));
        face.normal = vec3_t(0.0, 0.0, 1.0);
        face.numSwitchMax = 100;
        centroid->faces.push_back(face);
        // gap
        /*
        face.hull = new DiMP::Hull(graph);
        face.hull->vertices.push_back(vec3_t(-1.0, -5.0,  0.0));
        face.hull->vertices.push_back(vec3_t(-1.0,  5.0,  0.0));
        face.hull->vertices.push_back(vec3_t( 0.5,  5.0,  0.0));
        face.hull->vertices.push_back(vec3_t( 0.5, -5.0,  0.0));
        face.normal = vec3_t(0.0, 0.0, 1.0);
        face.numSwitchMax = 4;
        centroid->faces.push_back(face);
        
        face.hull = new DiMP::Hull(graph);
        face.hull->vertices.push_back(vec3_t( 2.3, -5.0,  0.0));
        face.hull->vertices.push_back(vec3_t( 2.3,  5.0,  0.0));
        face.hull->vertices.push_back(vec3_t( 5.0,  5.0,  0.0));
        face.hull->vertices.push_back(vec3_t( 5.0, -5.0,  0.0));
        face.normal = vec3_t(0.0, 0.0, 1.0);
        face.numSwitchMax = 4;
        centroid->faces.push_back(face);
        
        face.hull = new DiMP::Hull(graph);
        face.hull->vertices.push_back(vec3_t( 0.2, -0.5,  0.6));
        face.hull->vertices.push_back(vec3_t( 0.2, -0.15, 0.6));
        face.hull->vertices.push_back(vec3_t( 2.3, -0.15, 0.6));
        face.hull->vertices.push_back(vec3_t( 2.3, -0.5,  0.6));
        face.normal = vec3_t(0.0, 0.0, 1.0);
        face.numSwitchMax = 2;
        centroid->faces.push_back(face);

        face.hull = new DiMP::Hull(graph);
        face.hull->vertices.push_back(vec3_t( 0.2,  0.15,  0.6));
        face.hull->vertices.push_back(vec3_t( 0.2,  0.5,  0.6));
        face.hull->vertices.push_back(vec3_t( 2.3,  0.5,  0.6));
        face.hull->vertices.push_back(vec3_t( 2.3,  0.15,  0.6));
        face.normal = vec3_t(0.0, 0.0, 1.0);
        face.numSwitchMax = 2;
        centroid->faces.push_back(face);
        */
        /*
        // step
        face.hull = new DiMP::Hull(graph);
        face.hull->vertices.push_back(vec3_t(-1.0,  -0.5,  0.0));
        face.hull->vertices.push_back(vec3_t(-1.0,   0.5,  0.0));
        face.hull->vertices.push_back(vec3_t( 0.15,  0.5,  0.0));
        face.hull->vertices.push_back(vec3_t( 0.15, -0.5,  0.0));
        face.normal = vec3_t(0.0, 0.0, 1.0);
        centroid->faces.push_back(face);
        
        face.hull = new DiMP::Hull(graph);
        face.hull->vertices.push_back(vec3_t( 0.4 , -0.5,  0.10));
        face.hull->vertices.push_back(vec3_t( 0.4 ,  0.5,  0.10));
        face.hull->vertices.push_back(vec3_t( 0.65,  0.5,  0.10));
        face.hull->vertices.push_back(vec3_t( 0.65, -0.5,  0.10));
        face.normal = vec3_t(0.0, 0.0, 1.0);
        centroid->faces.push_back(face);

        face.hull = new DiMP::Hull(graph);
        face.hull->vertices.push_back(vec3_t( 0.9,  -0.5,  0.20));
        face.hull->vertices.push_back(vec3_t( 0.9,   0.5,  0.20));
        face.hull->vertices.push_back(vec3_t( 1.15,  0.5,  0.20));
        face.hull->vertices.push_back(vec3_t( 1.15, -0.5,  0.20));
        face.normal = vec3_t(0.0, 0.0, 1.0);
        centroid->faces.push_back(face);

        face.hull = new DiMP::Hull(graph);
        face.hull->vertices.push_back(vec3_t( 1.4, -0.5,  0.30));
        face.hull->vertices.push_back(vec3_t( 1.4,  0.5,  0.30));
        face.hull->vertices.push_back(vec3_t( 2.0,  0.5,  0.30));
        face.hull->vertices.push_back(vec3_t( 2.0, -0.5,  0.30));
        face.normal = vec3_t(0.0, 0.0, 1.0);
        centroid->faces.push_back(face);
        */
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
		
		centroid->waypoints.push_back(DiMP::Centroid::Waypoint(0, 0.0, startPos, startOri, vec3_t(), vec3_t()));
        for(int iend = 0; iend < nend; iend++){
		    centroid->waypoints.back().ends.push_back(DiMP::Centroid::Waypoint::End(startPos + endBasePos[iend] + endPosOrigin[iend], vec3_t()));
        }

		centroid->waypoints.push_back(DiMP::Centroid::Waypoint(N, dt*N, goalPos, goalOri, vec3_t(), vec3_t()));
        for(int iend = 0; iend < nend; iend++){
		    centroid->waypoints.back().ends.push_back(DiMP::Centroid::Waypoint::End(goalPos + endBasePos[iend] + endPosOrigin[iend], vec3_t()));
        }
		
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
        graph->solver->param.regularization = 1.0e-1;
		graph->solver->param.cutoffStepSize = 0.1;
		graph->solver->param.minStepSize    = 0.5;
		graph->solver->param.maxStepSize    = 0.5;
        //graph->solver->param.methodMajor    = Solver::Method::Major::DDP;
        graph->solver->param.methodMajor    = DiMP::CustomSolver::CustomMethod::SearchDDP;
		graph->solver->param.methodMinor    = Solver::Method::Minor::Direct;
		graph->solver->param.verbose        = false;
    
        fileDuration = fopen("duration.csv", "w");
	}

    virtual void OnStep(){
       App::OnStep();

       graph->solver->param.complRelaxation = std::max(0.00001, 0.5*graph->solver->param.complRelaxation);
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
