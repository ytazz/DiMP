#include <DiMP/DiMP.h>

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

	DiMP::BipedLIP*	biped;

	vec3_t  targetPos;
	uint    curIdx;
	string  saveFilename;

public:
	MyApp() {
		appName = "Biped";
		zAxisUp = true;

		AddAction(MENU_MAIN, ID_X, "switch to x")->AddHotKey('x');
		AddAction(MENU_MAIN, ID_Y, "switch to y")->AddHotKey('y');
		AddAction(MENU_MAIN, ID_Z, "switch to z")->AddHotKey('z');
		AddAction(MENU_MAIN, ID_INC, "incrase")->AddHotKey('n');
		AddAction(MENU_MAIN, ID_DEC, "decrease")->AddHotKey('m');
		AddAction(MENU_MAIN, ID_SAVE, "save")->AddHotKey('s');

		saveFilename = "save.csv";

		curIdx = 0;
	}
	virtual ~MyApp() {}

	virtual void BuildScene() {
		const real_t mt = 3.578+28.054+2.0*14.023+13.877;
		const real_t mf = 13.877/2;
		const real_t Ts = 0.5;
		const real_t Td = 0.2;
		const int    nstep_idle   = 2;  //< number of steps to step in place
		const int    nstep_acc    = 2;  //< number of steps to accelerate
		const int    nstep_cruise = 3;  //< number of steps to walk in constant speed
		const int    nstep_dec    = 1;  //< number of steps to decelerate
		const int    nstep        = nstep_idle + nstep_acc + nstep_cruise + nstep_dec;
		const int    nphase       = 2 * nstep + 3;  //< 2 phases per step + (one D at the beginning) + (two D at the end)
		const real_t vmax         = 0.8*(1.0 + (2.0*mf)/mt);    //< walking distance
		const real_t Tacc         = nstep_acc   *(Ts + Td);
		const real_t Tcruise      = nstep_cruise*(Ts + Td);
		const real_t Tdec         = nstep_dec   *(Ts + Td);
		const real_t acc          = vmax/Tacc;
		const real_t dec          = vmax/Tdec;
		const real_t dist_acc     = 0.5*acc*Tacc*Tacc;
		const real_t dist_cruise  = vmax*Tcruise;
		const real_t dist_dec     = vmax*Tdec - 0.5*dec*Tdec*Tdec;
		const real_t stepHeight   = 0.005;

		biped = new DiMP::BipedLIP(graph, "biped");
		biped->param.gravity      = 9.8;
		biped->param.comHeight    = 1.05;
		biped->param.torsoMass    = mt;
		biped->param.footMass     = mf;
		biped->param.durationMin[DiMP::BipedLIP::Phase::R ] = 0.65;
		biped->param.durationMax[DiMP::BipedLIP::Phase::R ] = 0.65;
		biped->param.durationMin[DiMP::BipedLIP::Phase::L ] = 0.65;
		biped->param.durationMax[DiMP::BipedLIP::Phase::L ] = 0.65;
		biped->param.durationMin[DiMP::BipedLIP::Phase::RL] = 0.05;
		biped->param.durationMax[DiMP::BipedLIP::Phase::RL] = 0.05;
		biped->param.durationMin[DiMP::BipedLIP::Phase::LR] = 0.05;
		biped->param.durationMax[DiMP::BipedLIP::Phase::LR] = 0.05;
		biped->param.durationMin[DiMP::BipedLIP::Phase::D ] = 0.30;
		biped->param.durationMax[DiMP::BipedLIP::Phase::D ] = 0.30;
		biped->param.footPosMin[0] = vec3_t(-0.45, -0.20, -1.5);
		biped->param.footPosMax[0] = vec3_t( 0.45, -0.09, -0.5);
		biped->param.footPosMin[1] = vec3_t(-0.45,  0.09, -1.5);
		biped->param.footPosMax[1] = vec3_t( 0.45,  0.20, -0.5);
		biped->param.footOriMin[0] = Rad(-0.0);
		biped->param.footOriMax[0] = Rad( 0.0);
		biped->param.footOriMin[1] = Rad(-0.0);
		biped->param.footOriMax[1] = Rad( 0.0);
		biped->param.swingHeight[0] = 0.050;
		biped->param.swingHeight[1] = 0.050;
		//biped->param.swingProfile = DiMP::BipedLIP::SwingProfile::Wedge;
		//biped->param.swingProfile = DiMP::BipedLIP::SwingProfile::Cycloid;
		biped->param.swingProfile       = DiMP::BipedLIP::SwingProfile::HeelToe;
		//biped->param.swingInterpolation = DiMP::BipedLIP::SwingInterpolation::Cubic;
		biped->param.swingInterpolation = DiMP::BipedLIP::SwingInterpolation::Quintic;
		biped->param.comHeightProfile   = DiMP::BipedLIP::ComHeightProfile::Constant;
		//biped->param.comHeightProfile = DiMP::BipedLIP::ComHeightProfile::Compass;
		biped->param.copMin        = vec3_t(-0.030,  0.00, -1.0);
		biped->param.copMax        = vec3_t( 0.110,  0.01,  1.0);
		biped->param.accMin        = vec3_t(-10.0, -1.0, -1.0);
		biped->param.accMax        = vec3_t( 10.0,  1.0,  1.0);
		biped->param.momMin        = vec3_t(-0.0, -0.0, -1.0);
		biped->param.momMax        = vec3_t( 0.0,  0.0,  1.0);
		//biped->param.footCurveType = DiMP::BipedLIP::FootCurveType::Arc;
		//biped->param.ankleToToe    = 0.100;
		//biped->param.ankleToHeel   = 0.070;
		//biped->param.toeCurvature  = 10.0;
		//biped->param.heelCurvature = 10.0;
		biped->param.footCurveType     = DiMP::BipedLIP::FootCurveType::Clothoid;
		biped->param.ankleToToe        = 0.070;
		biped->param.ankleToHeel       = 0.040;
		biped->param.toeCurvatureRate  = 156.0;
		biped->param.heelCurvatureRate = 156.0;
		
		/*
		 D -> RL -> L -> LR -> R ... -> RL -> D
		 */
		for (uint i = 0; i < nphase; i++)
			new DiMP::Tick(graph, 0.0, "");

		biped->phase.resize(nphase);
		biped->elevation.resize(nphase);

		real_t zg = 0.0;
		biped->phase[0] = DiMP::BipedLIP::Phase::D;
		biped->elevation[0] = zg;
		for (uint i = 1; i < nphase-2; i++) {

			biped->elevation[i] = zg;

			switch((i-1)%4){
			case 0:	{ biped->phase[i] = DiMP::BipedLIP::Phase::RL;                   break; }
			case 1:	{ biped->phase[i] = DiMP::BipedLIP::Phase::L ; zg += stepHeight; break; }
			case 2:	{ biped->phase[i] = DiMP::BipedLIP::Phase::LR;                   break; }
			case 3:	{ biped->phase[i] = DiMP::BipedLIP::Phase::R ; zg += stepHeight; break; }
			}
		}
		biped->phase[nphase-2] = DiMP::BipedLIP::Phase::D;
		biped->phase[nphase-1] = DiMP::BipedLIP::Phase::D;
		biped->elevation[nphase-2] = zg;
		biped->elevation[nphase-1] = zg;

		real_t spacing = 0.18/2;
		//vec2_t goalPos(3.0, 0.0);
		//real_t goalOri  = Rad(0.0);

		/* 4 waypoints
		 0-1: start, acceleration
		 1-2: constant speed
		 2-3: deceleration, stop
		 */
		biped->waypoints.resize(5);
		biped->waypoints[0].k                 = 0;
		biped->waypoints[0].com_pos           = vec3_t(0.0, 0.0, biped->param.comHeight);
		biped->waypoints[0].com_vel           = vec3_t(0.0, 0.0, 0.0);
		biped->waypoints[0].torso_pos_r       = 0.0;
		biped->waypoints[0].foot_pos_t[0]     = vec3_t(0.0, -spacing, 0.0);
		biped->waypoints[0].foot_pos_r[0]     = 0.0;
		biped->waypoints[0].foot_pos_t[1]     = vec3_t(0.0,  spacing, 0.0);
		biped->waypoints[0].foot_pos_r[1]     = 0.0;
		biped->waypoints[0].cop_pos           = vec3_t(0.0, 0.0, 0.0);
		biped->waypoints[0].fix_com_pos       = true;
		biped->waypoints[0].fix_com_vel       = true;
		biped->waypoints[0].fix_torso_pos_r   = true;
		biped->waypoints[0].fix_foot_pos_t[0] = true;
		biped->waypoints[0].fix_foot_pos_r[0] = true;
		biped->waypoints[0].fix_foot_pos_t[1] = true;
		biped->waypoints[0].fix_foot_pos_r[1] = true;
		biped->waypoints[0].fix_cop_pos       = true;
		biped->waypoints[0].fix_cmp_pos       = true;
		biped->waypoints[0].fix_mom           = true;

		biped->waypoints[1].k                 = 1 + 2*nstep_idle;
		biped->waypoints[1].com_pos           = vec3_t(0.0, 0.0, biped->param.comHeight + nstep_idle*stepHeight);
		biped->waypoints[1].com_vel           = vec3_t(0.0, 0.0, 0.0);
		biped->waypoints[1].torso_pos_r       = 0.0;
		biped->waypoints[1].foot_pos_t[0]     = vec3_t(0.0, -spacing, nstep_idle*stepHeight);
		biped->waypoints[1].foot_pos_r[0]     = 0.0;
		biped->waypoints[1].foot_pos_t[1]     = vec3_t(0.0,  spacing, nstep_idle*stepHeight);
		biped->waypoints[1].foot_pos_r[1]     = 0.0;
		biped->waypoints[1].cop_pos           = vec3_t(0.0,  0.0, nstep_idle*stepHeight);

		biped->waypoints[2].k                 = 1 + 2*(nstep_idle + nstep_acc);
		biped->waypoints[2].com_pos           = vec3_t(dist_acc, 0.0, biped->param.comHeight + (nstep_idle + nstep_acc)*stepHeight);
		biped->waypoints[2].com_vel           = vec3_t(vmax, 0.0, 0.0);
		biped->waypoints[2].torso_pos_r       = 0.0;
		biped->waypoints[2].foot_pos_t[0]     = vec3_t(dist_acc, -spacing, (nstep_idle + nstep_acc)*stepHeight);
		biped->waypoints[2].foot_pos_r[0]     = 0.0;
		biped->waypoints[2].foot_pos_t[1]     = vec3_t(dist_acc,  spacing, (nstep_idle + nstep_acc)*stepHeight);
		biped->waypoints[2].foot_pos_r[1]     = 0.0;
		biped->waypoints[2].cop_pos           = vec3_t(dist_acc,  0.0, (nstep_idle + nstep_acc)*stepHeight);
		
		biped->waypoints[3].k                 = 1 + 2*(nstep_idle + nstep_acc + nstep_cruise);
		biped->waypoints[3].com_pos           = vec3_t(dist_acc + dist_cruise, 0.0, biped->param.comHeight + (nstep_idle + nstep_acc + nstep_cruise)*stepHeight);
		biped->waypoints[3].com_vel           = vec3_t(vmax, 0.0, 0.0);
		biped->waypoints[3].torso_pos_r       = 0.0;
		biped->waypoints[3].foot_pos_t[0]     = vec3_t(dist_acc + dist_cruise, -spacing, (nstep_idle + nstep_acc + nstep_cruise)*stepHeight);
		biped->waypoints[3].foot_pos_r[0]     = 0.0;
		biped->waypoints[3].foot_pos_t[1]     = vec3_t(dist_acc + dist_cruise,  spacing, (nstep_idle + nstep_acc + nstep_cruise)*stepHeight);
		biped->waypoints[3].foot_pos_r[1]     = 0.0;
		biped->waypoints[3].cop_pos           = vec3_t(dist_acc + dist_cruise,  0.0, (nstep_idle + nstep_acc + nstep_cruise)*stepHeight);
		
		biped->waypoints[4].k                 = nphase - 1;
		biped->waypoints[4].com_pos           = vec3_t(dist_acc + dist_cruise + dist_dec, 0.0, biped->param.comHeight + nstep*stepHeight);
		biped->waypoints[4].com_vel           = vec3_t(0.0, 0.0, 0.0);
		biped->waypoints[4].torso_pos_r       = 0.0;
		biped->waypoints[4].foot_pos_t[0]     = vec3_t(dist_acc + dist_cruise + dist_dec, -spacing, nstep*stepHeight);
		biped->waypoints[4].foot_pos_r[0]     = 0.0;
		biped->waypoints[4].foot_pos_t[1]     = vec3_t(dist_acc + dist_cruise + dist_dec,  spacing, nstep*stepHeight);
		biped->waypoints[4].foot_pos_r[1]     = 0.0;
		biped->waypoints[4].cop_pos           = vec3_t(dist_acc + dist_cruise + dist_dec, 0.0, nstep*stepHeight);
		biped->waypoints[4].fix_com_pos       = true;
		biped->waypoints[4].fix_com_vel       = true;
		biped->waypoints[4].fix_torso_pos_r   = true;
		biped->waypoints[4].fix_foot_pos_t[0] = true;
		biped->waypoints[4].fix_foot_pos_r[0] = true;
		biped->waypoints[4].fix_foot_pos_t[1] = true;
		biped->waypoints[4].fix_foot_pos_r[1] = true;
		biped->waypoints[4].fix_cop_pos       = true;
		biped->waypoints[4].fix_cmp_pos       = true;
		biped->waypoints[4].fix_mom           = true;

		graph->scale.Set(1.0, 1.0, 1.0);
		graph->Init();

		graph->solver->SetConstraintWeight(ID(DiMP::ConTag::BipedAccRange), 1.0);
		graph->solver->SetConstraintWeight(ID(DiMP::ConTag::BipedMomRange), 1.0);
		//graph->solver->Enable(ID(DiMP::ConTag::BipedAccRange), false);
		
		graph->solver->SetCorrection(ID(), 0.5);
		graph->solver->param.numIter[0] = 20;
		graph->solver->param.cutoffStepSize = 0.1;
		graph->solver->param.minStepSize = 0.1;
		graph->solver->param.maxStepSize = 1.0;
		//graph->solver->param.methodMajor = Solver::Method::Major::Prioritized;
		graph->solver->param.methodMajor = Solver::Method::Major::GaussNewton;
		//graph->solver->param.methodMajor = Solver::Method::Major::DDP;
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
				//biped->Save(saveFilename.c_str());
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
