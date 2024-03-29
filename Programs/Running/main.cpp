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

	DiMP::BipedRunning* biped;

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
		const real_t mt = 3.578 + 28.054 + 2.0 * 14.023 + 13.877;
		const real_t mf = 13.877 / 2;
		const real_t Ts = 0.5;
		const real_t Td = 0.2;
		const int    nstep_idle = 2;  //< number of steps to step in place
		const int    nstep_acc = 6;  //< number of steps to accelerate
		const int    nstep_cruise = 8;  //< number of steps to walk in constant speed
		const int    nstep_dec = 6;  //< number of steps to decelerate
		const int    nstep = nstep_idle + nstep_acc + nstep_cruise + nstep_dec;
		const int    nphase = 2 * nstep + 3;  //< 2 phases per step + (one D at the beginning) + (two D at the end)
		const real_t vmax = 0.8 * (1.0 + (2.0 * mf) / mt);    //< walking distance
		const real_t Tacc = nstep_acc * (Ts + Td);
		const real_t Tcruise = nstep_cruise * (Ts + Td);
		const real_t Tdec = nstep_dec * (Ts + Td);
		const real_t acc = vmax / Tacc;
		const real_t dec = vmax / Tdec;
		const real_t dist_acc = 0.5 * acc * Tacc * Tacc;
		const real_t dist_cruise = vmax * Tcruise;
		const real_t dist_dec = vmax * Tdec - 0.5 * dec * Tdec * Tdec;

		biped = new DiMP::BipedRunning(graph, "biped");
		biped->param.gravity = vec3_t(0.0, 0.0, 9.8);
		biped->param.comHeight = 0.95;
		//biped->param.gaitType = DiMP::BipedRunning::GaitType::Run;
		biped->param.T[0] = 0.3656;
		biped->param.T[1] = 0.2432;
		biped->param.T[2] = 0.2350; // h=0.90 :0.2590; // calculated from specific duration and CoM height settings by external program
		biped->param.T[3] = sqrt(biped->param.comHeight / biped->param.gravity.z);
		biped->param.torsoMass = mt;
		biped->param.footMass = mf;
		biped->param.durationMin[DiMP::BipedRunning::Phase::R] = 0.30;
		biped->param.durationMax[DiMP::BipedRunning::Phase::R] = 0.30;
		biped->param.durationMin[DiMP::BipedRunning::Phase::L] = 0.30;
		biped->param.durationMax[DiMP::BipedRunning::Phase::L] = 0.30;
		biped->param.durationMin[DiMP::BipedRunning::Phase::RL] = 0.20;
		biped->param.durationMax[DiMP::BipedRunning::Phase::RL] = 0.20;
		biped->param.durationMin[DiMP::BipedRunning::Phase::LR] = 0.20;
		biped->param.durationMax[DiMP::BipedRunning::Phase::LR] = 0.20;
		biped->param.durationMin[DiMP::BipedRunning::Phase::R2] = 0.30;
		biped->param.durationMax[DiMP::BipedRunning::Phase::R2] = 0.30;
		biped->param.durationMin[DiMP::BipedRunning::Phase::L2] = 0.30;
		biped->param.durationMax[DiMP::BipedRunning::Phase::L2] = 0.30;
		biped->param.durationMin[DiMP::BipedRunning::Phase::RLF] = 0.20;
		biped->param.durationMax[DiMP::BipedRunning::Phase::RLF] = 0.20;
		biped->param.durationMin[DiMP::BipedRunning::Phase::LRF] = 0.20;
		biped->param.durationMax[DiMP::BipedRunning::Phase::LRF] = 0.20;
		biped->param.durationMin[DiMP::BipedRunning::Phase::D] = 0.20;
		biped->param.durationMax[DiMP::BipedRunning::Phase::D] = 0.20;
		biped->param.footPosMin[0] = vec3_t(-0.45, -0.20, -1.5);
		biped->param.footPosMax[0] = vec3_t(0.45, -0.09, -0.5);
		biped->param.footPosMin[1] = vec3_t(-0.45, 0.09, -1.5);
		biped->param.footPosMax[1] = vec3_t(0.45, 0.20, -0.5);
		biped->param.footOriMin[0] = Rad(-0.0);
		biped->param.footOriMax[0] = Rad(0.0);
		biped->param.footOriMin[1] = Rad(-0.0);
		biped->param.footOriMax[1] = Rad(0.0);
		biped->param.footCopMin[0] = vec3_t(-0.100, -0.05, 0.00);
		biped->param.footCopMax[0] = vec3_t(0.150, 0.05, 0.00);
		biped->param.footCopMin[1] = vec3_t(-0.100, -0.05, 0.00);
		biped->param.footCopMax[1] = vec3_t(0.150, 0.05, 0.00);
		biped->param.cmpMin        = vec3_t(-0.00, 0.0, 0.0);
		biped->param.cmpMax        = vec3_t(0.00, 0.0, 0.0);
		biped->param.momMin        = vec3_t(-0.0, 0.0, 0.0);
		biped->param.momMax        = vec3_t(0.0, 0.0, 0.0);
		biped->param.swingHeight = 0.10;
		//biped->param.swingProfile       = DiMP::BipedRunning::SwingProfile::Cycloid;
		//biped->param.swingProfile       = DiMP::BipedRunning::SwingProfile::Experiment;
		biped->param.swingProfile       = DiMP::BipedRunning::SwingProfile::HeelToe;
		//biped->param.swingInterpolation = DiMP::BipedRunning::SwingInterpolation::Cubic;
		biped->param.swingInterpolation = DiMP::BipedRunning::SwingInterpolation::Quintic;
		biped->param.footCurveType = DiMP::BipedRunning::FootCurveType::Arc;
		biped->param.ankleToToe = 0.100;
		biped->param.ankleToHeel = 0.070;
		biped->param.toeCurvature = 10.0;
		biped->param.heelCurvature = 10.0;
		
		for (uint i = 0; i < nphase; i++)
			new DiMP::Tick(graph, 0.0, "");

		biped->gaittype.resize(nphase);
		// set gait type on each phase
		for (uint i = 0; i < nphase; i++)
		{
			if      (i <= 13) biped->gaittype[i] = DiMP::BipedRunning::GaitType::Walk;
			else if (i <= 28) biped->gaittype[i] = DiMP::BipedRunning::GaitType::Run;
			else              biped->gaittype[i] = DiMP::BipedRunning::GaitType::Walk;
		}

		/*
		 D -> R -> RL -> L -> LR ... -> RL -> D
		 */
		biped->phase.resize(nphase);
		biped->phase[0] = DiMP::BipedRunning::Phase::D;
		for (uint i = 1; i < nphase - 2; i++) {
			switch ((i - 1) % 4) {
			case 0: { biped->phase[i] = (biped->gaittype[i] == DiMP::BipedRunning::GaitType::Walk ? DiMP::BipedRunning::Phase::RL : DiMP::BipedRunning::Phase::RLF); break; }
			case 1: { biped->phase[i] = (biped->gaittype[i] == DiMP::BipedRunning::GaitType::Walk ? DiMP::BipedRunning::Phase::L  : DiMP::BipedRunning::Phase::L2 ); break; }
			case 2: { biped->phase[i] = (biped->gaittype[i] == DiMP::BipedRunning::GaitType::Walk ? DiMP::BipedRunning::Phase::LR : DiMP::BipedRunning::Phase::LRF); break; }
			case 3: { biped->phase[i] = (biped->gaittype[i] == DiMP::BipedRunning::GaitType::Walk ? DiMP::BipedRunning::Phase::R  : DiMP::BipedRunning::Phase::R2 ); break; }
			}
		}
		biped->phase[nphase - 2] = DiMP::BipedRunning::Phase::D;
		biped->phase[nphase - 1] = DiMP::BipedRunning::Phase::D;


		real_t spacing = 0.18 / 2;
		//vec2_t goalPos(3.0, 0.0);
		//real_t goalOri  = Rad(0.0);

		/* 4 waypoints
		 0-1: start, acceleration
		 1-2: constant speed
		 2-3: deceleration, stop
		 */
		biped->waypoints.resize(5);
		biped->waypoints[0].k = 0;
		biped->waypoints[0].com_pos = vec3_t(0.0, 0.0, biped->param.comHeight);
		biped->waypoints[0].com_vel = vec3_t(0.0, 0.0, 0.0);
		biped->waypoints[0].cmp_pos = vec3_t(0.0, 0.0, 0.0);
		biped->waypoints[0].mom     = vec3_t(0.0, 0.0, 0.0);
		biped->waypoints[0].torso_pos_r = 0.0;
		biped->waypoints[0].foot_pos_t[0] = vec3_t(0.0, -spacing, 0.0);
		biped->waypoints[0].foot_pos_r[0] = 0.0;
		biped->waypoints[0].foot_pos_t[1] = vec3_t(0.0, spacing, 0.0);
		biped->waypoints[0].foot_pos_r[1] = 0.0;
		biped->waypoints[0].fix_com_pos = true;
		biped->waypoints[0].fix_com_vel = true;
		biped->waypoints[0].fix_cmp_pos = true;
		biped->waypoints[0].fix_mom     = true;
		biped->waypoints[0].fix_torso_pos_r = true;
		biped->waypoints[0].fix_foot_pos_t[0] = true;
		biped->waypoints[0].fix_foot_pos_r[0] = true;
		biped->waypoints[0].fix_foot_pos_t[1] = true;
		biped->waypoints[0].fix_foot_pos_r[1] = true;
		biped->waypoints[0].fix_foot_cop[0] = false;
		biped->waypoints[0].fix_foot_cop[1] = false;

		biped->waypoints[1].k = 1 + 2 * nstep_idle;
		biped->waypoints[1].com_pos = vec3_t(0.0, 0.0, biped->param.comHeight);
		biped->waypoints[1].com_vel = vec3_t(0.0, 0.0, 0.0);
		biped->waypoints[1].torso_pos_r = 0.0;
		biped->waypoints[1].foot_pos_t[0] = vec3_t(0.0, -spacing, 0);
		biped->waypoints[1].foot_pos_r[0] = 0.0;
		biped->waypoints[1].foot_pos_t[1] = vec3_t(0.0, spacing, 0);
		biped->waypoints[1].foot_pos_r[1] = 0.0;

		biped->waypoints[2].k = 1 + 2 * (nstep_idle + nstep_acc);
		biped->waypoints[2].com_pos = vec3_t(dist_acc, 0.0, biped->param.comHeight);
		biped->waypoints[2].com_vel = vec3_t(vmax, 0.0, 0.0);
		biped->waypoints[2].torso_pos_r = 0.0;
		biped->waypoints[2].foot_pos_t[0] = vec3_t(dist_acc, -spacing, 0.0);
		biped->waypoints[2].foot_pos_r[0] = 0.0;
		biped->waypoints[2].foot_pos_t[1] = vec3_t(dist_acc, spacing, 0.0);
		biped->waypoints[2].foot_pos_r[1] = 0.0;
		//biped->waypoints[2].fix_com_pos = true;
		//biped->waypoints[2].fix_cop_pos = true;

		biped->waypoints[3].k = 1 + 2 * (nstep_idle + nstep_acc + nstep_cruise);
		biped->waypoints[3].com_pos = vec3_t(dist_acc + dist_cruise, 0.0, biped->param.comHeight);
		biped->waypoints[3].com_vel = vec3_t(vmax, 0.0, 0.0);
		biped->waypoints[3].torso_pos_r = 0.0;
		biped->waypoints[3].foot_pos_t[0] = vec3_t(dist_acc + dist_cruise, -spacing, 0.0);
		biped->waypoints[3].foot_pos_r[0] = 0.0;
		biped->waypoints[3].foot_pos_t[1] = vec3_t(dist_acc + dist_cruise, spacing, 0.0);
		biped->waypoints[3].foot_pos_r[1] = 0.0;

		biped->waypoints[4].k = nphase - 1;
		biped->waypoints[4].com_pos = vec3_t(dist_acc + dist_cruise + dist_dec, 0.0, biped->param.comHeight);
		biped->waypoints[4].com_vel = vec3_t(0.0, 0.0, 0.0);
		biped->waypoints[4].torso_pos_r = 0.0;
		biped->waypoints[4].foot_pos_t[0] = vec3_t(dist_acc + dist_cruise + dist_dec, -spacing, 0.0);
		biped->waypoints[4].foot_pos_r[0] = 0.0;
		biped->waypoints[4].foot_pos_t[1] = vec3_t(dist_acc + dist_cruise + dist_dec, spacing, 0.0);
		biped->waypoints[4].foot_pos_r[1] = 0.0;
		biped->waypoints[4].fix_com_pos = true;
		biped->waypoints[4].fix_com_vel = true;
		biped->waypoints[4].fix_cmp_pos = true;
		biped->waypoints[4].fix_mom     = true;
		biped->waypoints[4].fix_torso_pos_r = true;
		biped->waypoints[4].fix_foot_pos_t[0] = true;
		biped->waypoints[4].fix_foot_pos_r[0] = true;
		biped->waypoints[4].fix_foot_pos_t[1] = true;
		biped->waypoints[4].fix_foot_pos_r[1] = true;
		biped->waypoints[4].fix_foot_cop[0] = false;
		biped->waypoints[4].fix_foot_cop[1] = false;


		graph->scale.Set(1.0, 1.0, 1.0);
		graph->Init();

		graph->solver->SetCorrection(ID(), 0.5);
		graph->solver->param.numIter[0] = 20;
		graph->solver->param.cutoffStepSize = 0.01;
		graph->solver->param.minStepSize = 0.01;
		graph->solver->param.maxStepSize = 1.0;
		//graph->solver->param.methodMajor = Solver::Method::Major::Prioritized;
		//graph->solver->param.methodMajor = Solver::Method::Major::GaussNewton;
		graph->solver->param.methodMajor = Solver::Method::Major::DDP;
		graph->solver->param.methodMinor = Solver::Method::Minor::Direct;
		graph->solver->param.verbose = true;
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
				biped->Save(saveFilename.c_str());
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
