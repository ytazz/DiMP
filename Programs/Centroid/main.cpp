#include <DiMP/DiMP.h>

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

    int  nend;
    int  N;
    int  nneighbor;

    struct Mode : vector<int>{
    };
    struct ModeSequence : vector<Mode>{
    };

    struct Neighbor{
        UTRef<DiMP::Graph   >  graph;
        UTRef<DiMP::Centroid>  centroid;
        ModeSequence           I;
    };

    vector<Neighbor>  neighbors;
    Mode              I0;
    ModeSequence      I;
    int               numSubIter;
    int               subIterCount;
    
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
        nend         = 2;
        N            = 15;
        nneighbor    = 1 + nend*(N-2);  // k = 1, 2, ..., N-1
        numSubIter   = 5;
        subIterCount = 0;

        neighbors.resize(nneighbor);
        I0.resize(nend);
        fill(I0.begin(), I0.end(), 0);

        I.resize(N);
        fill(I.begin(), I.end(), I0);
        
        neighbors[0].graph = graph;
        for(int i = 1; i < nneighbor; i++)
            neighbors[i].graph = new DiMP::Graph();

        for(int i = 0; i < nneighbor; i++)
		    neighbors[i].centroid = new DiMP::Centroid(neighbors[i].graph, "centroid");

		vec3_t startPos(0.0, 0.0, 1.0);
		quat_t startOri = quat_t();
		vec3_t goalPos (5.0, 0.0, 1.0);
		quat_t goalOri  = quat_t::Rot(Rad(0.0), 'z');
		real_t goalTime = 5.0;
		real_t spacing  = 0.2;

        for(int i = 0; i < nneighbor; i++){
            DiMP::Graph*    g   = neighbors[i].graph   ;
            DiMP::Centroid* cen = neighbors[i].centroid;

		    cen->param.g = 9.8;
		    cen->param.m = 1.0;
		    cen->param.I = 1.0;

            // create geometry
            cen->point = new DiMP::Point(g);
            cen->hull  = new DiMP::Hull (g);

            cen->param.bodyRangeMin = vec3_t(-0.1, -0.1, -0.25);
            cen->param.bodyRangeMax = vec3_t( 0.1,  0.1,  0.25);
		
		    cen->ends.resize(nend);
		    for(int iend = 0; iend < nend; iend++){
                cen->ends[iend].point = new DiMP::Point(g);
			
                if(iend == 0){
				    cen->ends[iend].basePos     = vec3_t( 0.0, -spacing/2.0,  0.0);
				    cen->ends[iend].posRangeMin = vec3_t(-0.5, -0.0        , -1.1);
				    cen->ends[iend].posRangeMax = vec3_t( 0.5,  0.0        , -0.9);
			    }
			    else{
				    cen->ends[iend].basePos     = vec3_t( 0.0,  spacing/2.0,  0.0);
				    cen->ends[iend].posRangeMin = vec3_t(-0.5, -0.0        , -1.1);
				    cen->ends[iend].posRangeMax = vec3_t( 0.5,  0.0        , -0.9);
			    }
			    cen->ends[iend].velRangeMin    = vec3_t(-1.5, -100.0, -100.0);
			    cen->ends[iend].velRangeMax    = vec3_t( 1.5,  100.0,  100.0);

			    cen->ends[iend].momentRangeMin = vec3_t(-0.0, -0.0, -1.0);
			    cen->ends[iend].momentRangeMax = vec3_t( 0.0,  0.0,  1.0);

                cen->ends[iend].copRangeMin = vec2_t(-0.1, -0.05);
			    cen->ends[iend].copRangeMax = vec2_t( 0.1,  0.05);

                cen->ends[iend].stiffnessMax = 1.0;
		    }

            // flat ground
            //centroid->faces.push_back(DiMP::Centroid::Face(vec2_t(-1.0, -5.0), vec2_t( 10.0, 5.0), vec3_t(0.0, 0.0, 0.0), quat_t::Rot(Rad(0.0), 'y')));
		
            // step
            DiMP::Centroid::Face face;
            face.hull = new DiMP::Hull(g);
            face.hull->vertices.push_back(vec3_t(-1.0, -5.0,  0.0));
            face.hull->vertices.push_back(vec3_t(-1.0,  5.0,  0.0));
            face.hull->vertices.push_back(vec3_t( 8.0, -5.0,  0.0));
            face.hull->vertices.push_back(vec3_t( 8.0,  5.0,  0.0));
            face.hull->CalcBSphere();
            face.normal = vec3_t(0.0, 0.0, 1.0);
            cen->faces.push_back(face);
        
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
			    new DiMP::Tick(g, k*dt, "");
		
		    cen->waypoints.push_back(DiMP::Centroid::Waypoint(0, 0.0, startPos, startOri, vec3_t(), vec3_t(), true, true, true, true));
		    cen->waypoints.back().ends.push_back(DiMP::Centroid::Waypoint::End(vec3_t(startPos.x, startPos.y - spacing/2.0, 0.0), vec3_t(), true, true));
		    cen->waypoints.back().ends.push_back(DiMP::Centroid::Waypoint::End(vec3_t(startPos.x, startPos.y + spacing/2.0, 0.0), vec3_t(), true, true));

            //centroid->waypoints.push_back(DiMP::Centroid::Waypoint(0, 0.0, startPos, startOri, vec3_t(), vec3_t(), true, true, true, true));
		    //centroid->waypoints.back().ends.push_back(DiMP::Centroid::Waypoint::End(vec3_t(startPos.x, startPos.y - spacing/2.0, 0.0), vec3_t(), true, true));
		    //centroid->waypoints.back().ends.push_back(DiMP::Centroid::Waypoint::End(vec3_t(startPos.x, startPos.y + spacing/2.0, 0.0), vec3_t(), true, true));

		    cen->waypoints.push_back(DiMP::Centroid::Waypoint(N, dt*N, goalPos, goalOri, vec3_t(), vec3_t(), false, false, false, false));
		    cen->waypoints.back().ends.push_back(DiMP::Centroid::Waypoint::End(vec3_t(goalPos.x, goalPos.y - spacing/2.0, 0.0), vec3_t(), false, false));
		    cen->waypoints.back().ends.push_back(DiMP::Centroid::Waypoint::End(vec3_t(goalPos.x, goalPos.y + spacing/2.0, 0.0), vec3_t(), false, false));
		
		    g->scale.Set(1.0, 1.0, 1.0);
		    g->Init();

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

		    g->solver->SetCorrection(ID(), 0.5);
		    g->solver->param.numIter[0]     = 20;
            g->solver->param.regularization = 1.0;
		    g->solver->param.cutoffStepSize = 0.1;
		    g->solver->param.minStepSize    = 1.0;
		    g->solver->param.maxStepSize    = 1.0;
            //graph->solver->param.methodMajor    = Solver::Method::Major::GaussNewton;
            g->solver->param.methodMajor    = Solver::Method::Major::DDP;
		    g->solver->param.methodMinor    = Solver::Method::Minor::Direct;
		    g->solver->param.verbose        = false;
        }

        fileDuration = fopen("duration.csv", "w");
	}

    virtual void OnStep(){
        // calc neighboring mode sequences
        for(int i = 0; i < nneighbor; i++){
            neighbors[i].I = I;
        }
        int i = 1;
        for(int iend = 0; iend < nend; iend++){
            for(int k = 1; k < N-1; k++){
                neighbors[i].I[k][iend] = (neighbors[i].I[k][iend] == -1 ? 0 : -1);
                i++;
            }
        }

        // set mode sequences
        for(int i = 0; i < nneighbor; i++){
            Neighbor& nei = neighbors[i];
            for(int k = 0; k < N; k++){
                DiMP::CentroidKey* key = (DiMP::CentroidKey*)nei.centroid->traj.GetKeypoint(nei.graph->ticks[k]);
                for(int iend = 0; iend < nend; iend++)
                    key->ends[iend].iface = nei.I[k][iend];
            }
        }

        // compute some steps of optimization
        for(int n = 0; n < numSubIter; n++){
            for(int i = 0; i < nneighbor; i++){
                neighbors[i].graph->Step();
                DSTR << "i: " << i << " V: " << neighbors[i].graph->solver->V[0] << endl; 
            }
        }

        real_t Vmin = inf;
        int    imin;
        for(int i = 0; i < nneighbor; i++){
            real_t V = neighbors[i].graph->solver->V[0];
            if(V < Vmin){
                Vmin = V;
                imin = i;
            }
        }

        DSTR << "imin: " << imin << endl;

        if(imin == 0){
            DSTR << "done" << endl;
        }

        I = neighbors[imin].I;
        for(Mode& Ik : I){
            DSTR << " " << Ik[0] << " " << Ik[1] << endl;
        }

        // copy the best trajectory to neighbors
        for(int i = 0; i < nneighbor; i++){
            if(i == imin)
                continue;

            neighbors[i].centroid->CopyVariables(neighbors[imin].centroid);
        }

        //App::OnStep();

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
