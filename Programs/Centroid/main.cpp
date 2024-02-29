#include <DiMP/DiMP.h>
#include <DiMP/Graph/Solver.h>

#include <sbrollpitchyaw.h>

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

    struct Scene{
        enum{
            Flat,
            Gap,
            GapWithRail,
            GapWithWall,
            Steps,
            Stairs,
        };
    };
    struct Task{
        enum{
            Travel,
            LongJump,
            Backflip,
        };
    };
    struct Gait{
        enum{
            WalkWithDoubleSupport,
            WalkWithoutDoubleSupport,
            Run,
            TrotWithQuadSupport,
            TrotWithoutQuadSupport,
            Pace,
            Gallop,
        };
    };
    struct Robot{
        enum{
            Biped,
            Humanoid,
            Quadruped,
        };
    };
    struct EndConfig{
        vec3_t basePos  ;
        vec3_t posOrigin;
        vec3_t posMin   ;
        vec3_t posMax   ;
        real_t stiffMax ;
        vec2_t cmpOffset;
    };

	DiMP::Centroid*	   centroid;
    FILE*              fileDuration;
    FILE*              fileCost;
    vector<EndConfig>  endConf;
        
    real_t comHeight;
	vec3_t startPos;
	vec3_t startOri;
	vec3_t goalPos;
	vec3_t goalOri;
    int    N;
    real_t dt;
    real_t goalTime;
    int    robotSelect;
    int    sceneSelect;
    int    taskSelect;
    int    gaitSelect;

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

        robotSelect = Robot::Biped;
        //robotSelect = Robot::Humanoid;
        //robotSelect = Robot::Quadruped;
        sceneSelect = Scene::Flat;
        //sceneSelect = Scene::Gap;
        //sceneSelect = Scene::Stairs;
        //sceneSelect = Scene::Steps;
        taskSelect = Task::Travel;
        //taskSelect = Task::LongJump;
        //taskSelect = Task::Backflip;
        //taskSelect = Task::Turn;
        gaitSelect = Gait::WalkWithDoubleSupport;
        //gaitSelect = Gait::Run;
        //gaitSelect = Gait::TrotWithQuadSupport;
        //gaitSelect = Gait::TrotWithoutQuadSupport;
        //gaitSelect = Gait::Pace;
        //gaitSelect = Gait::Gallop;
	}
	virtual ~MyApp() {}

	virtual void BuildScene() {
        if(robotSelect == Robot::Biped){
            comHeight = 0.7;
            endConf.resize(2);
            endConf[0].basePos   = vec3_t( 0.0, -0.15/2.0,  0.0);
            endConf[1].basePos   = vec3_t( 0.0,  0.15/2.0,  0.0);
            endConf[0].posOrigin = vec3_t( 0.0,  0.0, -0.7);
            endConf[1].posOrigin = vec3_t( 0.0,  0.0, -0.7);
            endConf[0].posMin    = vec3_t(-0.30, -0.0, -comHeight-0.0);
            endConf[1].posMin    = vec3_t(-0.30, -0.0, -comHeight-0.0);
            endConf[0].posMax    = vec3_t( 0.30,  0.0, -comHeight+0.0);
            endConf[1].posMax    = vec3_t( 0.30,  0.0, -comHeight+0.0);
            endConf[0].stiffMax  = 50.0;
            endConf[1].stiffMax  = 50.0;
            endConf[0].cmpOffset = -0.0*vec2_t(endConf[0].basePos.x, endConf[0].basePos.y);
            endConf[1].cmpOffset = -0.0*vec2_t(endConf[1].basePos.x, endConf[1].basePos.y);
        }
        if(robotSelect == Robot::Humanoid){
            comHeight = 0.7;
            endConf.resize(4);
            endConf[0].basePos   = vec3_t( 0.0, -0.20/2.0,  0.0);
            endConf[1].basePos   = vec3_t( 0.0,  0.20/2.0,  0.0);
            endConf[2].basePos   = vec3_t( 0.0, -0.25/2.0,  0.5);
            endConf[3].basePos   = vec3_t( 0.0,  0.25/2.0,  0.5);
            endConf[0].posOrigin = vec3_t( 0.0,  0.0, -0.7);
            endConf[1].posOrigin = vec3_t( 0.0,  0.0, -0.7);
            endConf[2].posOrigin = vec3_t( 0.0, -0.1, -0.5);
            endConf[3].posOrigin = vec3_t( 0.0,  0.1, -0.5);
            endConf[0].posMin    = vec3_t(-0.3, -0.05, -0.75);
            endConf[1].posMin    = vec3_t(-0.3, -0.05, -0.75);
            endConf[2].posMin    = vec3_t(-0.2, -0.3, -0.6);
            endConf[3].posMin    = vec3_t(-0.2,  0.0, -0.6);
            endConf[0].posMax    = vec3_t( 0.3,  0.05, -0.55);
            endConf[1].posMax    = vec3_t( 0.3,  0.05, -0.55);
            endConf[2].posMax    = vec3_t( 0.2,  0.0,  0.5);
            endConf[3].posMax    = vec3_t( 0.2,  0.3,  0.5);
            endConf[0].stiffMax  = 50.0;
            endConf[1].stiffMax  = 50.0;
            endConf[2].stiffMax  = 50.0;
            endConf[3].stiffMax  = 50.0;
        }
        if(robotSelect == Robot::Quadruped){
            comHeight = 0.5;
            endConf.resize(4);
            endConf[0].basePos   = vec3_t( 0.7/2.0, -0.4/2.0,  0.0);
            endConf[1].basePos   = vec3_t( 0.7/2.0,  0.4/2.0,  0.0);
            endConf[2].basePos   = vec3_t(-0.7/2.0, -0.4/2.0,  0.0);
            endConf[3].basePos   = vec3_t(-0.7/2.0,  0.4/2.0,  0.0);
            endConf[0].posOrigin = vec3_t( 0.0,  0.0, -0.5);
            endConf[1].posOrigin = vec3_t( 0.0,  0.0, -0.5);
            endConf[2].posOrigin = vec3_t( 0.0,  0.0, -0.5);
            endConf[3].posOrigin = vec3_t( 0.0,  0.0, -0.5);
            endConf[0].posMin    = vec3_t(-0.30, -0.15, -0.6);
            endConf[1].posMin    = vec3_t(-0.30, -0.15, -0.6);
            endConf[2].posMin    = vec3_t(-0.30, -0.15, -0.6);
            endConf[3].posMin    = vec3_t(-0.30, -0.15, -0.6);
            endConf[0].posMax    = vec3_t( 0.30,  0.15, -0.4);
            endConf[1].posMax    = vec3_t( 0.30,  0.15, -0.4);
            endConf[2].posMax    = vec3_t( 0.30,  0.15, -0.4);
            endConf[3].posMax    = vec3_t( 0.30,  0.15, -0.4);
            endConf[0].stiffMax  = 50.0;
            endConf[1].stiffMax  = 50.0;
            endConf[2].stiffMax  = 50.0;
            endConf[3].stiffMax  = 50.0;
		    endConf[0].cmpOffset = vec2_t(-0.0*endConf[0].basePos.x, -0.0*endConf[0].basePos.y);
            endConf[1].cmpOffset = vec2_t(-0.0*endConf[1].basePos.x, -0.0*endConf[1].basePos.y);
            endConf[2].cmpOffset = vec2_t(-0.0*endConf[2].basePos.x, -0.0*endConf[2].basePos.y);
            endConf[3].cmpOffset = vec2_t(-0.0*endConf[3].basePos.x, -0.0*endConf[3].basePos.y);
        }
        centroid = new DiMP::Centroid(graph, "centroid");
        
		centroid->param.g = 9.8;
		centroid->param.m = 44.0;
        centroid->param.I[0][0] = 2.0;
        centroid->param.I[1][1] = 2.0;
        centroid->param.I[2][2] = 2.0;
        centroid->param.swingHeight = 0.15;
        centroid->param.mu = 1.0;

        centroid->param.durationMin = 0.10;
        centroid->param.durationMax = 0.55;

        centroid->param.contactMargin = 0.0;

        centroid->param.enableRotation   = true;
        centroid->param.enableQuaternion = true;
        centroid->param.lockRpy[0]       = false;
        centroid->param.lockRpy[1]       = false;
        centroid->param.lockRpy[2]       = false;
        centroid->param.rotationResolution = 10;
        
        // create geometry
        centroid->point = new DiMP::Point(graph);
        centroid->hull  = new DiMP::Hull (graph);

        centroid->param.bodyRangeMin = vec3_t(-0.05, -0.05,  0.0);
        centroid->param.bodyRangeMax = vec3_t( 0.05,  0.05,  0.5);
		
		DiMP::Centroid::Face face;
        // flat ground
        if( sceneSelect == Scene::Flat ){
            real_t r = 0.0;
            face.hull = new DiMP::Hull(graph);
            face.hull->radius = r - centroid->param.contactMargin;
            face.hull->vertices.push_back(vec3_t(-1.0, -5.0,  -r));
            face.hull->vertices.push_back(vec3_t(-1.0,  5.0,  -r));
            face.hull->vertices.push_back(vec3_t(15.0,  5.0,  -r));
            face.hull->vertices.push_back(vec3_t(15.0, -5.0,  -r));
            face.normal = vec3_t(0.0, 0.0, 1.0);
            face.numSwitchMax = 100;
            centroid->faces.push_back(face);
        }
        // gap
        if(sceneSelect == Scene::Gap){
            real_t r = 0.0;
            face.hull = new DiMP::Hull(graph);
            face.hull->radius = r - centroid->param.contactMargin;
            face.hull->vertices.push_back(vec3_t(-1.0, -5.0,  -r));
            face.hull->vertices.push_back(vec3_t(-1.0,  5.0,  -r));
            face.hull->vertices.push_back(vec3_t( 1.0,  5.0,  -r));
            face.hull->vertices.push_back(vec3_t( 1.0, -5.0,  -r));
            face.normal = vec3_t(0.0, 0.0, 1.0);
            face.numSwitchMax = 100;
            centroid->faces.push_back(face);
        
            face.hull = new DiMP::Hull(graph);
            face.hull->radius = r - centroid->param.contactMargin;
            face.hull->vertices.push_back(vec3_t( 1.8, -5.0,  -r));
            face.hull->vertices.push_back(vec3_t( 1.8,  5.0,  -r));
            face.hull->vertices.push_back(vec3_t(15.0,  5.0,  -r));
            face.hull->vertices.push_back(vec3_t(15.0, -5.0,  -r));
            face.normal = vec3_t(0.0, 0.0, 1.0);
            face.numSwitchMax = 100;
            centroid->faces.push_back(face);
        }
        // gap with rail
        if(sceneSelect == Scene::GapWithRail){
            face.hull = new DiMP::Hull(graph);
            face.hull->vertices.push_back(vec3_t(-1.0, -5.0,  0.0));
            face.hull->vertices.push_back(vec3_t(-1.0,  5.0,  0.0));
            face.hull->vertices.push_back(vec3_t( 0.5,  5.0,  0.0));
            face.hull->vertices.push_back(vec3_t( 0.5, -5.0,  0.0));
            face.normal = vec3_t(0.0, 0.0, 1.0);
            face.numSwitchMax = 4;
            centroid->faces.push_back(face);
        
            face.hull = new DiMP::Hull(graph);
            face.hull->vertices.push_back(vec3_t( 1.3, -5.0,  0.0));
            face.hull->vertices.push_back(vec3_t( 1.3,  5.0,  0.0));
            face.hull->vertices.push_back(vec3_t( 5.0,  5.0,  0.0));
            face.hull->vertices.push_back(vec3_t( 5.0, -5.0,  0.0));
            face.normal = vec3_t(0.0, 0.0, 1.0);
            face.numSwitchMax = 4;
            centroid->faces.push_back(face);
        
            face.hull = new DiMP::Hull(graph);
            face.hull->vertices.push_back(vec3_t( 0.85, -0.5,  0.5));
            face.hull->vertices.push_back(vec3_t( 0.85, -0.25, 0.5));
            face.hull->vertices.push_back(vec3_t( 0.95, -0.25, 0.5));
            face.hull->vertices.push_back(vec3_t( 0.95, -0.5,  0.5));
            face.normal = vec3_t(0.0, 0.0, 1.0);
            face.numSwitchMax = 2;
            centroid->faces.push_back(face);

            face.hull = new DiMP::Hull(graph);
            face.hull->vertices.push_back(vec3_t( 0.85,  0.25,  0.5));
            face.hull->vertices.push_back(vec3_t( 0.85,  0.5,   0.5));
            face.hull->vertices.push_back(vec3_t( 0.95,  0.5,   0.5));
            face.hull->vertices.push_back(vec3_t( 0.95,  0.25,  0.5));
            face.normal = vec3_t(0.0, 0.0, 1.0);
            face.numSwitchMax = 2;
            centroid->faces.push_back(face);
        }
        // gap with wall
        if(sceneSelect == Scene::GapWithWall){
            face.hull = new DiMP::Hull(graph);
            face.hull->vertices.push_back(vec3_t(-1.0, -5.0,  0.0));
            face.hull->vertices.push_back(vec3_t(-1.0,  5.0,  0.0));
            face.hull->vertices.push_back(vec3_t( 0.5,  5.0,  0.0));
            face.hull->vertices.push_back(vec3_t( 0.5, -5.0,  0.0));
            face.normal = vec3_t(0.0, 0.0, 1.0);
            face.numSwitchMax = 4;
            centroid->faces.push_back(face);
        
            face.hull = new DiMP::Hull(graph);
            face.hull->vertices.push_back(vec3_t( 1.5, -5.0,  0.0));
            face.hull->vertices.push_back(vec3_t( 1.5,  5.0,  0.0));
            face.hull->vertices.push_back(vec3_t( 5.0,  5.0,  0.0));
            face.hull->vertices.push_back(vec3_t( 5.0, -5.0,  0.0));
            face.normal = vec3_t(0.0, 0.0, 1.0);
            face.numSwitchMax = 4;
            centroid->faces.push_back(face);
        
            face.hull = new DiMP::Hull(graph);
            face.hull->vertices.push_back(vec3_t( 0.5, -0.35, 0.0));
            face.hull->vertices.push_back(vec3_t( 0.5, -0.35, 2.0));
            face.hull->vertices.push_back(vec3_t( 1.5, -0.35, 2.0));
            face.hull->vertices.push_back(vec3_t( 1.5, -0.35, 0.0));
            face.normal = vec3_t(0.0, 1.0, 0.0);
            face.numSwitchMax = 2;
            centroid->faces.push_back(face);

            face.hull = new DiMP::Hull(graph);
            face.hull->vertices.push_back(vec3_t( 0.5,  0.35, 0.0));
            face.hull->vertices.push_back(vec3_t( 0.5,  0.35, 2.0));
            face.hull->vertices.push_back(vec3_t( 1.5,  0.35, 2.0));
            face.hull->vertices.push_back(vec3_t( 1.5,  0.35, 0.0));
            face.normal = vec3_t(0.0, -1.0, 0.0);
            face.numSwitchMax = 2;
            centroid->faces.push_back(face);
        }
        if(sceneSelect == Scene::Stairs){
            real_t r = 0.01;
            face.hull = new DiMP::Hull(graph);
            face.hull->radius = r;
            face.hull->vertices.push_back(vec3_t(-1.0,  -0.5,  -r));
            face.hull->vertices.push_back(vec3_t(-1.0,   0.5,  -r));
            face.hull->vertices.push_back(vec3_t( 0.15,  0.5,  -r));
            face.hull->vertices.push_back(vec3_t( 0.15, -0.5,  -r));
            face.normal = vec3_t(0.0, 0.0, 1.0);
            centroid->faces.push_back(face);
        
            face.hull = new DiMP::Hull(graph);
            face.hull->radius = r;
            face.hull->vertices.push_back(vec3_t( 0.5-0.1, -0.5,  0.10 - r));
            face.hull->vertices.push_back(vec3_t( 0.5-0.1,  0.5,  0.10 - r));
            face.hull->vertices.push_back(vec3_t( 0.5+0.1,  0.5,  0.10 - r));
            face.hull->vertices.push_back(vec3_t( 0.5+0.1, -0.5,  0.10 - r));
            face.normal = vec3_t(0.0, 0.0, 1.0);
            centroid->faces.push_back(face);

            face.hull = new DiMP::Hull(graph);
            face.hull->radius = r;
            face.hull->vertices.push_back(vec3_t( 0.8-0.1, -0.5,  0.20 - r));
            face.hull->vertices.push_back(vec3_t( 0.8-0.1,  0.5,  0.20 - r));
            face.hull->vertices.push_back(vec3_t( 0.8+0.1,  0.5,  0.20 - r));
            face.hull->vertices.push_back(vec3_t( 0.8+0.1, -0.5,  0.20 - r));
            face.normal = vec3_t(0.0, 0.0, 1.0);
            centroid->faces.push_back(face);

            face.hull = new DiMP::Hull(graph);
            face.hull->radius = r;
            face.hull->vertices.push_back(vec3_t( 1.1-0.1, -0.5,  0.30 - r));
            face.hull->vertices.push_back(vec3_t( 1.1-0.1,  0.5,  0.30 - r));
            face.hull->vertices.push_back(vec3_t( 1.1+0.1,  0.5,  0.30 - r));
            face.hull->vertices.push_back(vec3_t( 1.1+0.1, -0.5,  0.30 - r));
            face.normal = vec3_t(0.0, 0.0, 1.0);
            centroid->faces.push_back(face);

            face.hull = new DiMP::Hull(graph);
            face.hull->radius = r;
            face.hull->vertices.push_back(vec3_t( 1.4, -0.5,  0.40 - r));
            face.hull->vertices.push_back(vec3_t( 1.4,  0.5,  0.40 - r));
            face.hull->vertices.push_back(vec3_t( 2.0,  0.5,  0.40 - r));
            face.hull->vertices.push_back(vec3_t( 2.0, -0.5,  0.40 - r));
            face.normal = vec3_t(0.0, 0.0, 1.0);
            centroid->faces.push_back(face);
        }
		if(sceneSelect == Scene::Steps){
            real_t r = 0.01;
            face.hull = new DiMP::Hull(graph);
            face.hull->radius = r;
            face.hull->vertices.push_back(vec3_t( 0.0, -0.5, 0.3 - r));
            face.hull->vertices.push_back(vec3_t( 0.0,  0.5, 0.3 - r));
            face.hull->vertices.push_back(vec3_t( 0.5,  0.5, 0.3 - r));
            face.hull->vertices.push_back(vec3_t( 0.5, -0.5, 0.3 - r));
            face.normal = vec3_t(0.0, 0.0, 1.0);
            centroid->faces.push_back(face);
        
            face.hull = new DiMP::Hull(graph);
            face.hull->radius = r;
            face.hull->vertices.push_back(vec3_t(-0.5, -0.5, 0.0 - r));
            face.hull->vertices.push_back(vec3_t(-0.5,  0.5, 0.0 - r));
            face.hull->vertices.push_back(vec3_t( 0.0,  0.5, 0.0 - r));
            face.hull->vertices.push_back(vec3_t( 0.0, -0.5, 0.0 - r));
            face.normal = vec3_t(0.0, 0.0, 1.0);
            centroid->faces.push_back(face);
        }
		
        const int infi = numeric_limits<int>::max();
        int nend = endConf.size();
		
        if(taskSelect == Task::Travel){
            N    = 24;
            goalTime = 0.3 * N;
            dt = goalTime/((real_t)N);
		    
            if(sceneSelect == Scene::Flat){
                startPos = vec3_t(0.0, 0.0, comHeight);
		        startOri = vec3_t();
                if(robotSelect == Robot::Biped){
		            goalPos  = vec3_t(2.0, 0.0, comHeight);
		            goalOri  = vec3_t(0.0, 0.0, Rad(180.0));
                }
                if(robotSelect == Robot::Humanoid){
		            goalPos  = vec3_t(3.0, 3.0, comHeight);
		            goalOri  = vec3_t(0.0, 0.0, Rad(90.0));
                }
                if(robotSelect == Robot::Quadruped){
		            goalPos  = vec3_t(3.0, 0.0, comHeight);
		            goalOri  = vec3_t();
                }

                if(robotSelect == Robot::Biped){
                    if(gaitSelect == Gait::WalkWithDoubleSupport)
                        centroid->param.contactPattern = 
                            "0 0 " 
                            "0 - " 
                            "0 0 " 
                            "- 0 " 
                            "0 0 " 
                            "0 - " 
                            "0 0 " 
                            "- 0 " 
                            "0 0 " 
                            "0 - " 
                            "0 0 " 
                            "- 0 " 
                            "0 0 " 
                            "0 - " 
                            "0 0 " 
                            "- 0 " 
                            "0 0 " 
                            "0 - " 
                            "0 0 " 
                            "- 0 " 
                            "0 0 " 
                            "0 - " 
                            "0 0 " 
                            "- 0 " 
                            "0 0 ";
                    if(gaitSelect == Gait::WalkWithoutDoubleSupport)
                        centroid->param.contactPattern = 
                            "0 0 "
                            "0 - "
                            "- 0 "
                            "0 - "
                            "- 0 "
                            "0 - "
                            "- 0 "
                            "0 - "
                            "- 0 "
                            "0 - "
                            "- 0 "
                            "0 - "
                            "- 0 "
                            "0 - "
                            "- 0 "
                            "0 - "
                            "- 0 "
                            "0 - "
                            "- 0 "
                            "0 - "
                            "- 0 "
                            "0 - "
                            "- 0 "
                            "0 - "
                            "0 0";
                    if(gaitSelect == Gait::Run)
                        centroid->param.contactPattern =
                            "0 0 "
                            "0 - "
                            "- - "
                            "- 0 "
                            "- - "
                            "0 - "
                            "- - "
                            "- 0 "
                            "- - "
                            "0 - "
                            "- - "
                            "- 0 "
                            "- - "
                            "0 - "
                            "- - "
                            "- 0 "
                            "- - "
                            "0 - "
                            "- - "
                            "- 0 "
                            "- - "
                            "0 - "
                            "- - "
                            "- 0 "
                            "0 0 ";
                }
                if(robotSelect == Robot::Quadruped){
                    if(gaitSelect == Gait::TrotWithQuadSupport)
                        centroid->param.contactPattern = 
                            "0 0 0 0 "
                            "0 - - 0 "
                            "0 0 0 0 "
                            "- 0 0 - "
                            "0 0 0 0 "
                            "0 - - 0 "
                            "0 0 0 0 "
                            "- 0 0 - "
                            "0 0 0 0 "
                            "0 - - 0 "
                            "0 0 0 0 "
                            "- 0 0 - "
                            "0 0 0 0 "
                            "0 - - 0 "
                            "0 0 0 0 "
                            "- 0 0 - "
                            "0 0 0 0 "
                            "0 - - 0 "
                            "0 0 0 0 "
                            "- 0 0 - "
                            "0 0 0 0 "
                            "0 - - 0 "
                            "0 0 0 0 "
                            "- 0 0 - "
                            "0 0 0 0";
                    if(gaitSelect == Gait::TrotWithoutQuadSupport)
                        centroid->param.contactPattern = 
                            "0 0 0 0 "
                            "0 - - 0 "
                            "- 0 0 - "
                            "0 - - 0 "
                            "- 0 0 - "
                            "0 - - 0 "
                            "- 0 0 - "
                            "0 - - 0 "
                            "- 0 0 - "
                            "0 - - 0 "
                            "- 0 0 - "
                            "0 - - 0 "
                            "- 0 0 - "
                            "0 - - 0 "
                            "- 0 0 - "
                            "0 - - 0 "
                            "- 0 0 - "
                            "0 - - 0 "
                            "- 0 0 - "
                            "0 - - 0 "
                            "- 0 0 - "
                            "0 - - 0 "
                            "- 0 0 - "
                            "0 0 0 0 "
                            "0 0 0 0";
                    if(gaitSelect == Gait::Pace)
                        centroid->param.contactPattern = 
                            "0 0 0 0 "
                            "0 - 0 - "
                            "0 0 0 0 "
                            "- 0 - 0 "
                            "0 0 0 0 "
                            "0 - 0 - "
                            "0 0 0 0 "
                            "- 0 - 0 "
                            "0 0 0 0 "
                            "0 - 0 - "
                            "0 0 0 0 "
                            "- 0 - 0 "
                            "0 0 0 0 "
                            "0 - 0 - "
                            "0 0 0 0 "
                            "- 0 - 0 "
                            "0 0 0 0 "
                            "0 - 0 - "
                            "0 0 0 0 "
                            "- 0 - 0 "
                            "0 0 0 0 "
                            "0 - 0 - "
                            "0 0 0 0 "
                            "- 0 - 0 "
                            "0 0 0 0";
                    if(gaitSelect == Gait::Gallop)
                        centroid->param.contactPattern = 
                            "0 0 0 0 "
                            "- - 0 0 "
                            "- - - 0 "
                            "- - - - "
                            "0 - - - "
                            "- 0 - - "
                            "- - 0 - "
                            "- - - 0 "
                            "- - - - "
                            "0 - - - "
                            "- 0 - - "
                            "- - 0 - "
                            "- - - 0 "
                            "- - - - "
                            "0 - - - "
                            "- 0 - - "
                            "- - 0 - "
                            "- - - 0 "
                            "- - - - "
                            "0 - - - "
                            "- 0 - - "
                            "- - 0 - "
                            "- - - 0 "
                            "0 - - 0 "
                            "0 0 0 0";
                }
            }
            if( sceneSelect == Scene::Gap ||
                sceneSelect == Scene::GapWithRail ||
                sceneSelect == Scene::GapWithWall){

                startPos = vec3_t(0.0, 0.0, comHeight);
		        startOri = vec3_t();
                if(robotSelect == Robot::Biped){
		            goalPos  = vec3_t(3.0, 0.0, comHeight);
		            goalOri  = vec3_t();
                }
                if(robotSelect == Robot::Humanoid){
		            goalPos  = vec3_t(3.0, 0.0, comHeight);
		            goalOri  = vec3_t();
                }
                if(robotSelect == Robot::Quadruped){
		            goalPos  = vec3_t(1.5, 0.0, comHeight);
		            goalOri  = vec3_t();
                }
            }
            if(sceneSelect == Scene::Stairs){
                startPos = vec3_t(0.0, 0.0, comHeight);
		        startOri = vec3_t();
		        goalPos  = vec3_t(1.6, 0.0, comHeight + 0.4);
                goalOri  = vec3_t();
            }

            centroid->waypoints.resize(N+1);
            {
                DiMP::Centroid::Waypoint& wp = centroid->waypoints[0];
                wp.value  = DiMP::Centroid::Waypoint::Value (0.0, dt, startPos, startOri, vec3_t(), vec3_t());
                wp.weight = DiMP::Centroid::Waypoint::Weight(100.0, 0.1, 100.0*one, 100.0*one, 100.0*one, 100.0*one);
                wp.ends.resize(nend);
                for(int iend = 0; iend < nend; iend++){
		            wp.ends[iend].value  = DiMP::Centroid::Waypoint::End::Value (startPos + endConf[iend].basePos + endConf[iend].posOrigin, vec3_t(), vec3_t(), vec3_t(), infi);
                    wp.ends[iend].weight = DiMP::Centroid::Waypoint::End::Weight(100.0*one, 100.0*one, 100.0*one, 100.0*one, 1.0, 10.0*one2, 10*one);
                }
            }
            {
                DiMP::Centroid::Waypoint& wp = centroid->waypoints[1];
                wp.weight.time  = 0.1;
                wp.weight.pos_t = vec3_t(1.0, 1.0, 10.0);
                wp.weight.pos_r =  10*one;
                wp.weight.vel_t = 10*one;
                wp.weight.vel_r = 10*one;
                wp.ends.resize(nend);
                for(int iend = 0; iend < nend; iend++){
                    wp.ends[iend].weight.pos_t  = vec3_t(0.1, 10.0, 10.0);
                    wp.ends[iend].weight.pos_r  = vec3_t(10.0, 10.0, 0.1);
                    wp.ends[iend].weight.vel_t  = 0.1*one;
                    wp.ends[iend].weight.vel_r  = 0.1*one;
                }
            }
            {
                DiMP::Centroid::Waypoint& wp = centroid->waypoints[N-1];
                wp.ends.resize(nend);
                wp.weight.time  = 0.1;
                wp.weight.pos_t = vec3_t(1.0, 1.0, 10.0);
                wp.weight.pos_r =  10*one;
                wp.weight.vel_t = 10*one;
                wp.weight.vel_r = 10*one;
                for(int iend = 0; iend < nend; iend++){
                    wp.ends[iend].weight.pos_t  = vec3_t(0.1, 10.0, 10.0);
                    wp.ends[iend].weight.pos_r  = vec3_t(10.0, 10.0, 0.1);
                    wp.ends[iend].weight.vel_t  = 0.1*one;
                    wp.ends[iend].weight.vel_r  = 0.1*one;
                }
            }
            {
                DiMP::Centroid::Waypoint& wp = centroid->waypoints[N];
                wp.value  = DiMP::Centroid::Waypoint::Value (dt*N, dt, goalPos, goalOri, vec3_t(), vec3_t());
                wp.weight = DiMP::Centroid::Waypoint::Weight(0.1, 0.1, 100.0*one, 100.0*one, 100.0*one, 100.0*one);
                wp.ends.resize(nend);
                for(int iend = 0; iend < nend; iend++){
		            wp.ends[iend].value  = DiMP::Centroid::Waypoint::End::Value (goalPos + FromRollPitchYaw(goalOri)*(endConf[iend].basePos + endConf[iend].posOrigin), goalOri, vec3_t(), vec3_t(), infi);
                    wp.ends[iend].weight = DiMP::Centroid::Waypoint::End::Weight(100.0*one, 100.0*one, 100.0*one, 100.0*one, 1.0, 10.0*one2, 10.0*one);
                }
            }
        }
        if(taskSelect == Task::LongJump){
            N    = 5;
            goalTime = 0.3 * N;
            dt = goalTime/((real_t)N);
		    if(sceneSelect == Scene::Flat){
                startPos = vec3_t(0.0, 0.0, comHeight);
		        startOri = vec3_t();
                if(robotSelect == Robot::Biped){
		            goalPos  = vec3_t(1.0, 0.0, comHeight);
		            goalOri  = vec3_t(0.0, 0.0, Rad(0.0));
            
                    centroid->param.contactPattern = 
                        "0 0 "
                        "0 0 "
                        "- - "
                        "0 0 "
                        "0 0 "
                        "0 0 ";
                }
            }
            centroid->waypoints.resize(N+1);
            {
                DiMP::Centroid::Waypoint& wp = centroid->waypoints[0];
                wp.value  = DiMP::Centroid::Waypoint::Value (0.0, dt, startPos, startOri, vec3_t(), vec3_t());
                wp.weight = DiMP::Centroid::Waypoint::Weight(100.0, 0.1, 100.0*one, 100.0*one, 100.0*one, 100.0*one);
                wp.ends.resize(nend);
                for(int iend = 0; iend < nend; iend++){
		            wp.ends[iend].value  = DiMP::Centroid::Waypoint::End::Value (startPos + endConf[iend].basePos + endConf[iend].posOrigin, vec3_t(), vec3_t(), vec3_t(), infi);
                    wp.ends[iend].weight = DiMP::Centroid::Waypoint::End::Weight(100.0*one, 100.0*one, 100.0*one, 100.0*one, 1.0, 1.0*one2, 1.0*one);
                }
            }
            {
                DiMP::Centroid::Waypoint& wp = centroid->waypoints[1];
                wp.weight.time  = 0.1;
                wp.weight.pos_t =  1*one;
                wp.weight.pos_r =  1*one;
                wp.weight.vel_t = 10*one;
                wp.weight.vel_r = 10*one;
                wp.ends.resize(nend);
                for(int iend = 0; iend < nend; iend++){
                    wp.ends[iend].weight.pos_t  = 0.1*one;
                    wp.ends[iend].weight.vel_t  = 0.1*one;
                }
            }
            {
                DiMP::Centroid::Waypoint& wp = centroid->waypoints[N-1];
                wp.ends.resize(nend);
                wp.weight.time  = 0.1;
                wp.weight.pos_t =  1*one;
                wp.weight.pos_r =  1*one;
                wp.weight.vel_t = 10*one;
                wp.weight.vel_r = 10*one;
                for(int iend = 0; iend < nend; iend++){
                    wp.ends[iend].weight.pos_t  = 0.1*one;
                    wp.ends[iend].weight.vel_t  = 0.1*one;
                }
            }
            {
                DiMP::Centroid::Waypoint& wp = centroid->waypoints[N];
                wp.value  = DiMP::Centroid::Waypoint::Value (dt*N, dt, goalPos, goalOri, vec3_t(), vec3_t());
                wp.weight = DiMP::Centroid::Waypoint::Weight(100.0, 0.1, 100.0*one, 100.0*one, 100.0*one, 100.0*one);
                wp.ends.resize(nend);
                for(int iend = 0; iend < nend; iend++){
		            wp.ends[iend].value  = DiMP::Centroid::Waypoint::End::Value (goalPos + endConf[iend].basePos + endConf[iend].posOrigin, vec3_t(), vec3_t(), vec3_t(), infi);
                    wp.ends[iend].weight = DiMP::Centroid::Waypoint::End::Weight(100.0*one, 100.0*one, 100.0*one, 100.0*one, 1.0, 1.0*one2, 1.0*one);
                }
            }
        }
        if(taskSelect == Task::Backflip){
            N    = 6;
            goalTime = 0.3 * N;
            dt = goalTime/((real_t)N);

		    if(sceneSelect == Scene::Steps){
                startPos = vec3_t( 0.25, 0.0, comHeight + 0.3);
                startOri = vec3_t();
                goalPos  = vec3_t(-0.25, 0.0, comHeight);
                goalOri  = vec3_t(0.0, Rad(-360.0), 0.0);
                if(robotSelect == Robot::Biped){
                    centroid->param.contactPattern = 
                        "0 0 "
                        "0 0 "
                        "0 0 "
                        "- - "
                        "1 1 "
                        "1 1 "
                        "1 1 ";
                }
            }
            
            vec3_t jumpPos(0.5, 0.0, comHeight + 0.2);
            vec3_t jumpOri(0.0, Rad(180.0), 0.0);
            centroid->waypoints.resize(N+1);
            {
                DiMP::Centroid::Waypoint& wp = centroid->waypoints[0];
                wp.value  = DiMP::Centroid::Waypoint::Value (0.0, dt, startPos, startOri, vec3_t(), vec3_t());
                wp.weight = DiMP::Centroid::Waypoint::Weight(100.0, 0.01, 100.0*one, 100.0*one, 100.0*one, 100.0*one);
                wp.ends.resize(nend);
                for(int iend = 0; iend < nend; iend++){
		            wp.ends[iend].value  = DiMP::Centroid::Waypoint::End::Value (startPos + endConf[iend].basePos + endConf[iend].posOrigin, vec3_t(), vec3_t(), vec3_t(), infi);
                    wp.ends[iend].weight = DiMP::Centroid::Waypoint::End::Weight(100.0*one, 100.0*one, 100.0*one, 100.0*one, 1.0, 1.0*one2, 1.0*one);
                }
            }
            {
                DiMP::Centroid::Waypoint& wp = centroid->waypoints[1];
                wp.value  = DiMP::Centroid::Waypoint::Value (inf, inf, startPos, startOri, vec3_t(inf, inf, inf), vec3_t());
                wp.weight = DiMP::Centroid::Waypoint::Weight(0.01, 0.01, 1.0*one, vec3_t(100.0, 0.1, 100.0), 1.0*one, vec3_t(100.0, 1.0, 100.0));
                wp.ends.resize(nend);
                for(int iend = 0; iend < nend; iend++){
		            wp.ends[iend].value  = DiMP::Centroid::Waypoint::End::Value (startPos + endConf[iend].basePos + endConf[iend].posOrigin, vec3_t(), vec3_t(), vec3_t(), infi);
                    wp.ends[iend].weight = DiMP::Centroid::Waypoint::End::Weight(1*one,1*one,1*one,1*one, 0.1, 0.1*one2, 1*one);
                }
            }
            {
                DiMP::Centroid::Waypoint& wp = centroid->waypoints[2];
                wp.value  = DiMP::Centroid::Waypoint::Value (inf, inf, startPos, startOri, vec3_t(inf, inf, inf), vec3_t());
                wp.weight = DiMP::Centroid::Waypoint::Weight(inf, inf, 1.0*one, vec3_t(100.0, 0.1, 100.0), 1.0*one, vec3_t(100.0, 1.0, 100.0));
                wp.ends.resize(nend);
                for(int iend = 0; iend < nend; iend++){
		            wp.ends[iend].value  = DiMP::Centroid::Waypoint::End::Value (startPos + endConf[iend].basePos + endConf[iend].posOrigin, vec3_t(), vec3_t(), vec3_t(), infi);
                    wp.ends[iend].weight = DiMP::Centroid::Waypoint::End::Weight(1*one,1*one,1*one,1*one, 0.1, 0.1*one2, 1*one);
                }
            }
            {
                DiMP::Centroid::Waypoint& wp = centroid->waypoints[3];
                wp.value  = DiMP::Centroid::Waypoint::Value (inf, inf, startPos, startOri - vec3_t(0.0, Rad(45.0), 0.0), vec3_t(0.0, 0.0, 0.0), vec3_t(0.0, -10.0, 0.0));
                wp.weight = DiMP::Centroid::Waypoint::Weight(inf, inf, 1.0*one, vec3_t(100.0, 1.0, 100.0), 1.0*one, vec3_t(100.0, 1.0, 100.0));
                wp.ends.resize(nend);
                for(int iend = 0; iend < nend; iend++){
		            wp.ends[iend].value  = DiMP::Centroid::Waypoint::End::Value (startPos + endConf[iend].basePos + endConf[iend].posOrigin, vec3_t(), vec3_t(), vec3_t(), infi);
                    wp.ends[iend].weight = DiMP::Centroid::Waypoint::End::Weight(1*one,1*one,1*one,1*one, 0.1, 0.1*one2, 1*one);
                }
            }
            {
                DiMP::Centroid::Waypoint& wp = centroid->waypoints[4];
                wp.value  = DiMP::Centroid::Waypoint::Value (inf, inf, goalPos, goalOri + vec3_t(0.0, Rad(45.0), 0.0), vec3_t(0.0, 0.0, 0.0), vec3_t(0.0, -10.0, 0.0));
                wp.weight = DiMP::Centroid::Waypoint::Weight(inf, inf, 1.0*one, vec3_t(100.0, 1.0, 100.0), 1.0*one, vec3_t(100.0, 1.0, 100.0));
                wp.ends.resize(nend);
                for(int iend = 0; iend < nend; iend++){
		            wp.ends[iend].value  = DiMP::Centroid::Waypoint::End::Value (goalPos + endConf[iend].basePos + endConf[iend].posOrigin, vec3_t(), vec3_t(), vec3_t(), infi);
                    wp.ends[iend].weight = DiMP::Centroid::Waypoint::End::Weight(1*one,1*one,1*one,1*one, 0.1, 0.1*one2, 1*one);
                }
            }
            {
                DiMP::Centroid::Waypoint& wp = centroid->waypoints[5];
                wp.ends.resize(nend);
                wp.value  = DiMP::Centroid::Waypoint::Value (inf, inf, goalPos, goalOri, vec3_t(inf, inf, inf), vec3_t());
                wp.weight = DiMP::Centroid::Waypoint::Weight(inf, inf, 1.0*one, vec3_t(100.0, 0.1, 100.0), 1.0*one, vec3_t(100.0, 1.0, 100.0));
                for(int iend = 0; iend < nend; iend++){
		            wp.ends[iend].value  = DiMP::Centroid::Waypoint::End::Value (goalPos + endConf[iend].basePos + endConf[iend].posOrigin, vec3_t(), vec3_t(), vec3_t(), infi);
                    wp.ends[iend].weight = DiMP::Centroid::Waypoint::End::Weight(1*one,1*one,1*one,1*one, 0.1, 0.1*one2, 1*one);
                }
            }
            {
                DiMP::Centroid::Waypoint& wp = centroid->waypoints[6];
                wp.value  = DiMP::Centroid::Waypoint::Value (dt*N, dt, goalPos, goalOri, vec3_t(), vec3_t());
                wp.weight = DiMP::Centroid::Waypoint::Weight(0.01, 0.01, 100.0*one, 100.0*one, 100.0*one, 100.0*one);
                wp.ends.resize(nend);
                for(int iend = 0; iend < nend; iend++){
		            wp.ends[iend].value  = DiMP::Centroid::Waypoint::End::Value (goalPos + endConf[iend].basePos + endConf[iend].posOrigin, vec3_t(), vec3_t(), vec3_t(), infi);
                    wp.ends[iend].weight = DiMP::Centroid::Waypoint::End::Weight(100.0*one, 100.0*one, 100.0*one, 100.0*one, 1.0, 1.0*one2, 1.0*one);
                }
            }
        }
        /*
        if(taskSelect == Task::Turn){
            N    = 5;
            goalTime = 0.3 * N;
            dt = goalTime/((real_t)N);
		    if(sceneSelect == Scene::Flat){
                startPos = vec3_t(0.0, 0.0, comHeight);
		        startOri = vec3_t();
                if(robotSelect == Robot::Biped){
		            goalPos  = vec3_t(1.0, 0.0, comHeight);
		            goalOri  = vec3_t(0.0, 0.0, Rad(0.0));
            
                    centroid->param.contactPattern = 
                        "0 0 "
                        "0 - "
                        "0 0 "
                        "- 0 "
                        "0 0 "
                        "0 0 ";
                }
            }
            centroid->waypoints.resize(N+1);
            {
                DiMP::Centroid::Waypoint& wp = centroid->waypoints[0];
                wp.value  = DiMP::Centroid::Waypoint::Value (0.0, dt, startPos, startOri, vec3_t(), vec3_t());
                wp.weight = DiMP::Centroid::Waypoint::Weight(100.0, 0.1, 100.0*one, 100.0*one, 100.0*one, 100.0*one);
                wp.ends.resize(nend);
                for(int iend = 0; iend < nend; iend++){
		            wp.ends[iend].value  = DiMP::Centroid::Waypoint::End::Value (startPos + endConf[iend].basePos + endConf[iend].posOrigin, vec3_t(), vec3_t(), vec3_t(), infi);
                    wp.ends[iend].weight = DiMP::Centroid::Waypoint::End::Weight(100.0*one, 100.0*one, 100.0*one, 100.0*one, 1.0, 1.0*one2, 1.0*one);
                }
            }
            {
                DiMP::Centroid::Waypoint& wp = centroid->waypoints[1];
                wp.weight.time  = 0.1;
                wp.weight.pos_t =  1*one;
                wp.weight.pos_r =  1*one;
                wp.weight.vel_t = 10*one;
                wp.weight.vel_r = 10*one;
                wp.ends.resize(nend);
                for(int iend = 0; iend < nend; iend++){
                    wp.ends[iend].weight.pos_t  = 0.1*one;
                    wp.ends[iend].weight.vel_t  = 0.1*one;
                }
            }
            {
                DiMP::Centroid::Waypoint& wp = centroid->waypoints[N-1];
                wp.ends.resize(nend);
                wp.weight.time  = 0.1;
                wp.weight.pos_t =  1*one;
                wp.weight.pos_r =  1*one;
                wp.weight.vel_t = 10*one;
                wp.weight.vel_r = 10*one;
                for(int iend = 0; iend < nend; iend++){
                    wp.ends[iend].weight.pos_t  = 0.1*one;
                    wp.ends[iend].weight.vel_t  = 0.1*one;
                }
            }
            {
                DiMP::Centroid::Waypoint& wp = centroid->waypoints[N];
                wp.value  = DiMP::Centroid::Waypoint::Value (dt*N, dt, goalPos, goalOri, vec3_t(), vec3_t());
                wp.weight = DiMP::Centroid::Waypoint::Weight(100.0, 0.1, 100.0*one, 100.0*one, 100.0*one, 100.0*one);
                wp.ends.resize(nend);
                for(int iend = 0; iend < nend; iend++){
		            wp.ends[iend].value  = DiMP::Centroid::Waypoint::End::Value (goalPos + endConf[iend].basePos + endConf[iend].posOrigin, vec3_t(), vec3_t(), vec3_t(), infi);
                    wp.ends[iend].weight = DiMP::Centroid::Waypoint::End::Weight(100.0*one, 100.0*one, 100.0*one, 100.0*one, 1.0, 1.0*one2, 1.0*one);
                }
            }
        }
        */
        centroid->ends.resize(nend);
		for(int iend = 0; iend < nend; iend++){
            centroid->ends[iend].point = new DiMP::Point(graph);
			
            centroid->ends[iend].basePos = endConf[iend].basePos;
            centroid->ends[iend].posMin  = endConf[iend].posMin;
            centroid->ends[iend].posMax  = endConf[iend].posMax; 
			centroid->ends[iend].copMin  = vec2_t(-0.05, -0.01);
			centroid->ends[iend].copMax  = vec2_t( 0.05,  0.01);

            centroid->ends[iend].stiffnessMax = endConf[iend].stiffMax;
            centroid->ends[iend].cmpOffset  = endConf[iend].cmpOffset;
            centroid->ends[iend].lockOri    = false;
            centroid->ends[iend].lockCmp    = false;
            centroid->ends[iend].lockMoment = true;
		}
                
		for(int k = 0; k <= N; k++)
			new DiMP::Tick(graph, k*dt, "");
		
        centroid->SetScaling();
		graph->scale.Set(1.0, 1.0, 1.0);
		graph->Init();
        
		//graph->solver->Enable(ID(DiMP::ConTag::CentroidPosT      ), false);
		//graph->solver->Enable(ID(DiMP::ConTag::CentroidPosR      ), false);
		//graph->solver->Enable(ID(DiMP::ConTag::CentroidVelT      ), false);
		//graph->solver->Enable(ID(DiMP::ConTag::CentroidVelR      ), false);
		//graph->solver->Enable(ID(DiMP::ConTag::CentroidTime      ), false);
        //graph->solver->Enable(ID(DiMP::ConTag::CentroidEndPos    ), false);
		//graph->solver->Enable(ID(DiMP::ConTag::CentroidEndVel    ), false);
		//graph->solver->Enable(ID(DiMP::ConTag::CentroidEndStiff  ), false);
		//graph->solver->Enable(ID(DiMP::ConTag::CentroidEndPosRange  ), false);
		//graph->solver->Enable(ID(DiMP::ConTag::CentroidEndContact), false);
        graph->solver->Enable(ID(DiMP::ConTag::CentroidEndFriction), false);
		graph->solver->Enable(ID(DiMP::ConTag::CentroidEndMomentRange), false);
        
        graph->solver->SetCorrection(ID(), 0.1);
		graph->solver->param.regularization = 10;
		graph->solver->param.hastyStepSize  = false;
		graph->solver->param.cutoffStepSize = 0.1;
		graph->solver->param.minStepSize    = 0.1;
		graph->solver->param.maxStepSize    = 1.0;
        graph->solver->param.methodMajor    = Solver::Method::Major::DDP;
        //graph->solver->param.methodMajor    = DiMP::CustomSolver::CustomMethod::SearchDDP;
		graph->solver->param.methodMinor    = Solver::Method::Minor::Direct;
        graph->solver->param.useHessian     = false;
		graph->solver->param.verbose        = true;
        graph->solver->param.parallelize    = false;
    
        fileDuration = fopen("duration.csv", "w");
        fileCost     = fopen("cost.csv", "w");

        fprintf(fileCost,
            "iter, step, obj, Tpre, Tdir, Tstep, Tmod, Trans, Tcost, Tcostgrad, Tback, Tpre, Tprestep, Tstep, Tfinish, wcompl\n"
        );
	}

    virtual void OnStep(){
       App::OnStep();

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
        SavePlan();
        SaveTraj();
    }

    void SavePlan(){
        static int idx = 0;
        char filename[256];
        sprintf(filename, "plan_centroid.csv");
        FILE* file = fopen(filename, "w");
        idx++;

        fprintf(file, 
            "k, "
            "time, duration, "
            "cen_pos_t_x, cen_pos_t_y, cen_pos_t_z, "
            "cen_vel_t_x, cen_vel_t_y, cen_vel_t_z, "
            "cen_pos_r_x, cen_pos_r_y, cen_pos_r_z, "
            "cen_vel_r_x, cen_vel_r_y, cen_vel_r_z, "
        );
        for(int i = 0; i < centroid->ends.size(); i++){
            fprintf(file,
                "end%d_pos_t_x, end%d_pos_t_y, end%d_pos_t_z, "
                "end%d_vel_t_x, end%d_vel_t_y, end%d_vel_t_z, "
                "end%d_pos_r_x, end%d_pos_r_y, end%d_pos_r_z, "
                "end%d_vel_r_x, end%d_vel_r_y, end%d_vel_r_z, "
                "end%d_stiff, "
                "end%d_cmp_x, end%d_cmp_y, end%d_cmp_z, "
                "end%d_mom_x, end%d_mom_y, end%d_mom_z, ",
                i, i, i,
                i, i, i,
                i, i, i,
                i, i, i,
                i,
                i, i, i,
                i, i, i
                );
        }
        for(int i = 0; i < centroid->ends.size(); i++){
            fprintf(file,
                "end%d_iface, ",
                i
                );
        }
        fprintf(file, "\n");

        for(int k = 0; k < graph->ticks.size(); k++){
            auto key = (DiMP::CentroidKey*)centroid->traj.GetKeypoint(graph->ticks[k]);
        
            fprintf(file,
                "%d, "
                "%f, %f, "
                "%f, %f, %f, "
                "%f, %f, %f, ",
                k, 
                key->var_time->val, (key->next ? key->var_duration->val : 0.0),
                key->var_pos_t->val.x, key->var_pos_t->val.y, key->var_pos_t->val.z, 
                key->var_vel_t->val.x, key->var_vel_t->val.y, key->var_vel_t->val.z
            );
            if(centroid->param.enableRotation){
                fprintf(file,
                    "%f, %f, %f, "
                    "%f, %f, %f, ",
                    key->var_pos_r->val.x, key->var_pos_r->val.y, key->var_pos_r->val.z, 
                    key->var_vel_r->val.x, key->var_vel_r->val.y, key->var_vel_r->val.z
                );
            }
            else{
                fprintf(file,
                    "%f, %f, %f, "
                    "%f, %f, %f, ",
                    0.0, 0.0, 0.0,
                    0.0, 0.0, 0.0
                );
            }
            for(int i = 0; i < key->ends.size(); i++){
                DiMP::CentroidKey::End& end = key->ends[i];
                fprintf(file,
                    "%f, %f, %f, "
                    "%f, %f, %f, ",
                    end.var_pos_t->val.x, end.var_pos_t->val.y, end.var_pos_t->val.z, 
                    end.var_vel_t->val.x, end.var_vel_t->val.y, end.var_vel_t->val.z
                );
                if(centroid->param.enableRotation){
                    fprintf(file,
                        "%f, %f, %f, "
                        "%f, %f, %f, ",
                        end.var_pos_r->val.x, end.var_pos_r->val.y, end.var_pos_r->val.z, 
                        end.var_vel_r->val.x, end.var_vel_r->val.y, end.var_vel_r->val.z
                    );
                }
                else{
                    fprintf(file,
                        "%f, %f, %f, "
                        "%f, %f, %f, ",
                        0.0, 0.0, 0.0,
                        0.0, 0.0, 0.0
                    );
                }
                fprintf(file,
                    "%f, ",
                    end.var_stiff->val);

                if(centroid->param.enableRotation){
                    fprintf(file,
                        "%f, %f, %f, "
                        "%f, %f, %f, ",
                        end.var_cmp[0]->val, end.var_cmp[1]->val, 0.0, 
                        end.var_moment->val.x, end.var_moment->val.y, end.var_moment->val.z
                    );
                }
                else{
                    fprintf(file,
                        "%f, %f, %f, "
                        "%f, %f, %f, ",
                        0.0, 0.0, 0.0,
                        0.0, 0.0, 0.0
                    );
                }
            }
            for(int i = 0; i < key->ends.size(); i++){
                DiMP::CentroidKey::End& end = key->ends[i];
                fprintf(file,
                    "%d, ",
                    end.iface
                );    
            }
            fprintf(file, "\n");
        }

        fclose(file);
    }

    void SaveTraj(){
        char filename[256];
        sprintf(filename, "traj_centroid.csv");
        FILE* file = fopen(filename, "w");
    
        fprintf(file, 
            "k, "
            "cen_pos_t_x, cen_pos_t_y, cen_pos_t_z, "
            "cen_pos_r_w, cen_pos_r_x, cen_pos_r_y, cen_pos_r_z, "
            "cen_vel_t_x, cen_vel_t_y, cen_vel_t_z, "
            "cen_vel_r_x, cen_vel_r_y, cen_vel_r_z, "
        );
        for(int i = 0; i < centroid->ends.size(); i++){
            fprintf(file,
                "end%d_pos_t_x, end%d_pos_t_y, end%d_pos_t_z, "
                "end%d_pos_r_w, end%d_pos_r_x, end%d_pos_r_y, end%d_pos_r_z, "
                "end%d_vel_t_x, end%d_vel_t_y, end%d_vel_t_z, "
                "end%d_vel_r_x, end%d_vel_r_y, end%d_vel_r_z, "
                "end%d_force_t_x, end%d_force_t_y, end%d_force_t_z, "
                "end%d_force_r_x, end%d_force_r_y, end%d_force_r_z, ",
                i, i, i,
                i, i, i, i,
                i, i, i,
                i, i, i,
                i, i, i,
                i, i, i
                );
        }
        // additional info
        fprintf(file, "cen_mom_x, cen_mom_y, cen_mom_z, ");
        fprintf(file, "\n");


        auto key = (DiMP::CentroidKey*)centroid->traj.GetKeypoint(graph->ticks.back());
        real_t tf = key->var_time->val;
        const real_t dt = 0.01;
        vec3_t pc, vc, ac, wf;
        quat_t qf;
        vec3_t pe, ve, we, me;
        vec2_t re;
        real_t le;
        bool   ce;
        quat_t qe;

        for(real_t t = 0.0; t <= tf; t += dt){
            centroid->ComState(t, pc, vc, ac);
            centroid->TorsoState(t, qf, wf, Interpolate::SlerpDiff);

            fprintf(file,
                "%f, "
                "%f, %f, %f, "
                "%f, %f, %f, %f, "
                "%f, %f, %f, "
                "%f, %f, %f, ",
                t, 
                pc.x, pc.y, pc.z, 
                qf.w, qf.x, qf.y, qf.z,
                vc.x, vc.y, vc.z, 
                wf.x, wf.y, wf.z
            );
            vec3_t mom;
            for(int i = 0; i < key->ends.size(); i++){
                centroid->EndState(t, i, pe, qe, ve, we);
                centroid->EndForce(t, i, le, re, me, ce);

                vec3_t f   = (centroid->param.m*le*le)*(pc - pe - vec3_t(re.x, re.y, 0.0));
                vec3_t eta = (centroid->param.m*le*le)*me;

                mom += (pe - pc) % f + eta;
            
                fprintf(file,
                    "%f, %f, %f, "
                    "%f, %f, %f, %f, "
                    "%f, %f, %f, "
                    "%f, %f, %f, "
                    "%f, %f, %f, "
                    "%f, %f, %f, ",
                    pe.x, pe.y, pe.z, 
                    qe.w, qe.x, qe.y, qe.z,
                    ve.x, ve.y, ve.z, 
                    we.x, we.y, we.z,
                    f.x, f.y, f.z,
                    eta.x, eta.y, eta.z
                );    
            }
            fprintf(file, "%f, %f, %f, ", mom.x, mom.y, mom.z);
            fprintf(file, "\n");
        }

        fclose(file);
    }

} app;

DiMP::Graph graph;

int main(int argc, char* argv[]) {
	app.graph = &graph;
	app.Init(argc, argv);
	app.StartMainLoop();
}
