#include <DiMP/DiMP.h>

/** 2足ロボットのバランス制御
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

	DiMP::BipedLIP*	biped;

	vec3_t  targetPos;
	uint    curIdx;

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

		curIdx = 0;
	}
	virtual ~MyApp() {}

	virtual void BuildScene() {
		biped = new DiMP::BipedLIP(graph, "biped");
		biped->param.gravity = 9.8;
		biped->param.heightCoM = 0.4823;
		biped->param.torsoMass = 4.432*0.8;
		biped->param.footMass = (4.432 - biped->param.torsoMass) / 2;
		//biped->param.thetaHeel = 15.9160 * M_PI / 180;
		//biped->param.thetaToe = biped->param.thetaHeel;
		biped->param.durationMin[DiMP::BipedLIP::Phase::R] = 0.1;
		biped->param.durationMax[DiMP::BipedLIP::Phase::R] = 0.7;
		biped->param.durationMin[DiMP::BipedLIP::Phase::L] = 0.1;
		biped->param.durationMax[DiMP::BipedLIP::Phase::L] = 0.7;
		biped->param.durationMin[DiMP::BipedLIP::Phase::RL] = 0.1;
		biped->param.durationMax[DiMP::BipedLIP::Phase::RL] = 0.3;
		biped->param.durationMin[DiMP::BipedLIP::Phase::LR] = 0.1;
		biped->param.durationMax[DiMP::BipedLIP::Phase::LR] = 0.3;
		biped->param.footPosMin[0] = vec2_t(-0.20, -0.14);
		biped->param.footPosMax[0] = vec2_t(0.20, -0.07);
		biped->param.footPosMin[1] = vec2_t(-0.20, 0.07);
		biped->param.footPosMax[1] = vec2_t(0.20, 0.14);
		biped->param.footOriMin[0] = Rad(-15.0);
		biped->param.footOriMax[0] = Rad(15.0);
		biped->param.footOriMin[1] = Rad(-15.0);
		biped->param.footOriMax[1] = Rad(15.0);

		// 歩数設定．tickの時刻は使われない
		const uint nstep = 10;
		//const uint nstep = 30;
		const uint nphase = 2 * nstep + 1;

		for (uint i = 0; i < nphase; i++)
			new DiMP::Tick(graph, 0.0, "");

		biped->phase.resize(nphase);
		for (uint i = 0; i < nphase; i++) {
			switch (i % 4) {
			case 0: biped->phase[i] = DiMP::BipedLIP::Phase::LR; break;
			case 1: biped->phase[i] = DiMP::BipedLIP::Phase::R; break;
			case 2: biped->phase[i] = DiMP::BipedLIP::Phase::RL; break;
			case 3: biped->phase[i] = DiMP::BipedLIP::Phase::L; break;
			}
		}

		real_t spacing = 0.1; //足の開き幅

							  //real_t step_length = 0.1; //定常歩行時の歩幅
							  //vec2_t steady_start_r(5 * step_length*0.6, -spacing); //定常歩行開始時の右足接地位置
							  //vec2_t steady_start_l(5 * step_length*0.6 - step_length, spacing); //定常歩行開始時の左足接地位置
							  //real_t steady_start_time = 5 * step_length*0.6;

							  //vec2_t goalPos(5*step_length*0.6*2 + 20* step_length, 0.0); //目標位置
		vec2_t goalPos(2.0, 0.0); //目標位置
		real_t goalOri = Rad(0); //目標姿勢

		biped->waypoints.resize(2);
		//biped->waypoints.resize(3);

		biped->waypoints[0].k = 0;
		biped->waypoints[0].torso_pos_t = vec2_t(0.0, 0.0);
		biped->waypoints[0].torso_pos_r = 0.0;
		biped->waypoints[0].torso_vel_t = vec2_t(0.0, 0.0);
		biped->waypoints[0].torso_vel_r = 0.0;
		biped->waypoints[0].foot_pos_t[0] = vec2_t(0.0, -spacing);
		biped->waypoints[0].foot_pos_r[0] = 0.0;
		biped->waypoints[0].foot_pos_t[1] = vec2_t(0.0, spacing);
		biped->waypoints[0].foot_pos_r[1] = 0.0;
		biped->waypoints[0].fix_torso_pos_t = true;
		biped->waypoints[0].fix_torso_pos_r = true;
		biped->waypoints[0].fix_torso_vel_t = true;
		biped->waypoints[0].fix_torso_vel_r = true;
		biped->waypoints[0].fix_foot_pos_t[0] = true;
		biped->waypoints[0].fix_foot_pos_r[0] = true;
		biped->waypoints[0].fix_foot_pos_t[1] = true;
		biped->waypoints[0].fix_foot_pos_r[1] = true;

		biped->waypoints[1].k = nphase - 1;
		biped->waypoints[1].torso_pos_t = goalPos;
		biped->waypoints[1].torso_pos_r = goalOri;
		biped->waypoints[1].torso_vel_t = vec2_t(0.0, 0.0);
		biped->waypoints[1].torso_vel_r = 0.0;
		biped->waypoints[1].foot_pos_t[0] = goalPos + mat2_t::Rot(goalOri) * vec2_t(0.0, -spacing);
		biped->waypoints[1].foot_pos_r[0] = goalOri;
		biped->waypoints[1].foot_pos_t[1] = goalPos + mat2_t::Rot(goalOri) * vec2_t(0.0, spacing);
		biped->waypoints[1].foot_pos_r[1] = goalOri;
		biped->waypoints[1].fix_torso_pos_t = true;
		biped->waypoints[1].fix_torso_pos_r = true;
		biped->waypoints[1].fix_torso_vel_t = true;
		biped->waypoints[1].fix_torso_vel_r = true;
		biped->waypoints[1].fix_foot_pos_t[0] = true;
		biped->waypoints[1].fix_foot_pos_r[0] = true;
		biped->waypoints[1].fix_foot_pos_t[1] = true;
		biped->waypoints[1].fix_foot_pos_r[1] = true;



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
				biped->Save();
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
