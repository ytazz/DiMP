﻿#include <DiMP/DiMP.h>

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
		biped = new DiMP::BipedLIP(graph, "biped");
		biped->param.gravity = 9.8;
		biped->param.heightCoM = 1.05;
		biped->param.heightlow = 0.52;
		biped->param.heighthigh = 0.54;
		biped->param.heightmiddle = (biped->param.heightlow + biped->param.heighthigh)/2;
		biped->param.torsoMass = 3.578+28.054+2.0*14.023+13.877;
		biped->param.footMass = 13.877/2;
		biped->param.durationMin[DiMP::BipedLIP::Phase::R ] = 0.55;
		biped->param.durationMax[DiMP::BipedLIP::Phase::R ] = 0.55;
		biped->param.durationMin[DiMP::BipedLIP::Phase::L ] = 0.55;
		biped->param.durationMax[DiMP::BipedLIP::Phase::L ] = 0.55;
		biped->param.durationMin[DiMP::BipedLIP::Phase::RL] = 0.15;
		biped->param.durationMax[DiMP::BipedLIP::Phase::RL] = 0.15;
		biped->param.durationMin[DiMP::BipedLIP::Phase::LR] = 0.15;
		biped->param.durationMax[DiMP::BipedLIP::Phase::LR] = 0.15;
		biped->param.durationMin[DiMP::BipedLIP::Phase::D ] = 0.15;
		biped->param.durationMax[DiMP::BipedLIP::Phase::D ] = 0.15;
		biped->param.footPosMin[0] = vec2_t(-0.45, -0.20);
		biped->param.footPosMax[0] = vec2_t( 0.45, -0.115);
		biped->param.footPosMin[1] = vec2_t(-0.45,  0.115);
		biped->param.footPosMax[1] = vec2_t( 0.45,  0.20);
		biped->param.footOriMin[0] = Rad(-15.0);
		biped->param.footOriMax[0] = Rad(15.0);
		biped->param.footOriMin[1] = Rad(-15.0);
		biped->param.footOriMax[1] = Rad(15.0);
		biped->param.swingHeight[0] = 0.050;
		biped->param.swingHeight[1] = 0.050;
		//biped->param.swingProfile = DiMP::BipedLIP::SwingProfile::Wedge;
		//biped->param.swingProfile = DiMP::BipedLIP::SwingProfile::Cycloid;
		biped->param.swingProfile = DiMP::BipedLIP::SwingProfile::HeelToe;
		biped->param.copPosMin    = vec2_t(-0.040, -0.025 );
		biped->param.copPosMax    = vec2_t( 0.110,  0.025 );
		biped->param.ankleToToe   = 0.100;
		biped->param.ankleToHeel  = 0.070;
		biped->param.toeRadius    = 0.100;
		biped->param.heelRadius   = 0.100;
		biped->param.accWeight    = 1.0;
		biped->param.momWeight    = 0.0;

		/*
		 D -> RL -> L -> LR -> R ... -> RL -> D

		 */
		const uint nstep = 10;
		const uint nphase = 2 * nstep + 3;

		for (uint i = 0; i < nphase; i++)
			new DiMP::Tick(graph, 0.0, "");

		biped->phase.resize(nphase);
		biped->phase[0] = DiMP::BipedLIP::Phase::D;
		for (uint i = 1; i < nphase-2; i++) {
			switch((i-1)%4){
			case 0:	biped->phase[i] = DiMP::BipedLIP::Phase::RL; break;
			case 1:	biped->phase[i] = DiMP::BipedLIP::Phase::L ; break;
			case 2:	biped->phase[i] = DiMP::BipedLIP::Phase::LR; break;
			case 3:	biped->phase[i] = DiMP::BipedLIP::Phase::R ; break;
			}
		}
		biped->phase[nphase-2] = DiMP::BipedLIP::Phase::D;
		biped->phase[nphase-1] = DiMP::BipedLIP::Phase::D;

		real_t spacing = 0.23/2;
		vec2_t goalPos(2.0, 0.0);
		real_t goalOri = Rad(0);

		biped->waypoints.resize(2);
		
		biped->waypoints[0].k                 = 0;
		biped->waypoints[0].torso_pos_t       = vec2_t(0.0, 0.0);
		biped->waypoints[0].torso_pos_r       = 0.0;
		biped->waypoints[0].torso_vel_t       = vec2_t(0.0, 0.0);
		biped->waypoints[0].torso_vel_r       = 0.0;
		biped->waypoints[0].foot_pos_t[0]     = vec2_t(0.0, -spacing);
		biped->waypoints[0].foot_pos_r[0]     = 0.0;
		biped->waypoints[0].foot_pos_t[1]     = vec2_t(0.0, spacing);
		biped->waypoints[0].foot_pos_r[1]     = 0.0;
		biped->waypoints[0].cop_pos           = vec2_t(0.0, 0.0);
		biped->waypoints[0].fix_torso_pos_t   = true;
		biped->waypoints[0].fix_torso_pos_r   = true;
		biped->waypoints[0].fix_torso_vel_t   = true;
		biped->waypoints[0].fix_torso_vel_r   = true;
		biped->waypoints[0].fix_foot_pos_t[0] = true;
		biped->waypoints[0].fix_foot_pos_r[0] = true;
		biped->waypoints[0].fix_foot_pos_t[1] = true;
		biped->waypoints[0].fix_foot_pos_r[1] = true;
		biped->waypoints[0].fix_cop_pos       = true;
		biped->waypoints[0].fix_cmp_pos       = true;
		biped->waypoints[0].fix_mom           = true;

		biped->waypoints[1].k                 = nphase - 1;
		biped->waypoints[1].torso_pos_t       = goalPos;
		biped->waypoints[1].torso_pos_r       = goalOri;
		biped->waypoints[1].torso_vel_t       = vec2_t(0.0, 0.0);
		biped->waypoints[1].torso_vel_r       = 0.0;
		biped->waypoints[1].foot_pos_t[0]     = goalPos + mat2_t::Rot(goalOri) * vec2_t(0.0, -spacing);
		biped->waypoints[1].foot_pos_r[0]     = goalOri;
		biped->waypoints[1].foot_pos_t[1]     = goalPos + mat2_t::Rot(goalOri) * vec2_t(0.0, spacing);
		biped->waypoints[1].foot_pos_r[1]     = goalOri;
		biped->waypoints[1].cop_pos           = goalPos;
		biped->waypoints[1].fix_torso_pos_t   = true;
		biped->waypoints[1].fix_torso_pos_r   = true;
		biped->waypoints[1].fix_torso_vel_t   = true;
		biped->waypoints[1].fix_torso_vel_r   = true;
		biped->waypoints[1].fix_foot_pos_t[0] = true;
		biped->waypoints[1].fix_foot_pos_r[0] = true;
		biped->waypoints[1].fix_foot_pos_t[1] = true;
		biped->waypoints[1].fix_foot_pos_r[1] = true;
		biped->waypoints[1].fix_cop_pos       = true;
		biped->waypoints[1].fix_cmp_pos       = true;
		biped->waypoints[1].fix_mom           = true;

		graph->scale.Set(1.0, 1.0, 1.0);
		graph->Init();

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

		//graph->solver->Enable(ID(DiMP::ConTag::BipedLipP      ), false);
		//graph->solver->Enable(ID(DiMP::ConTag::BipedLipV      ), false);
		//graph->solver->Enable(ID(DiMP::ConTag::BipedFootRangeT), false);
		//graph->solver->Enable(ID(DiMP::ConTag::BipedFootRangeR), false);
		//graph->solver->Enable(ID(DiMP::ConTag::BipedFootMatchT), false);
		//graph->solver->Enable(ID(DiMP::ConTag::BipedFootMatchR), false);
		//graph->solver->Enable(ID(DiMP::ConTag::BipedComP      ), false);
		//graph->solver->Enable(ID(DiMP::ConTag::BipedComV      ), false);
		//graph->solver->Enable(ID(DiMP::ConTag::BipedCop       ), false);
		//graph->solver->Enable(ID(DiMP::ConTag::BipedDuration  ), false);
		//graph->solver->Enable(ID(DiMP::ConTag::BipedTime      ), false);

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
