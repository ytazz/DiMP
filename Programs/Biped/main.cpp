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
		const real_t Ts = 0.05;
		const real_t Td = 0.40;
		const int    nstep_idle   = 2;  //< number of steps to step in place
		const int    nstep_acc    = 5;  //< number of steps to accelerate
		const int    nstep_cruise = 10;  //< number of steps to walk in constant speed
		const int    nstep_dec    = 5;  //< number of steps to decelerate
		const int    nstep        = nstep_idle + nstep_acc + nstep_cruise + nstep_dec;
		const int    nphase       = 2 * nstep + 4;  //< 2 phases per step + (two D at the beginning) + (two D at the end)
		const real_t vmax         = 0.9*(1.0 + (2.0*mf)/mt);    //< walking distance
		const real_t Tacc         = nstep_acc   *(Ts + Td);
		const real_t Tcruise      = nstep_cruise*(Ts + Td);
		const real_t Tdec         = nstep_dec   *(Ts + Td);
		const real_t acc          = vmax/Tacc;
		const real_t dec          = vmax/Tdec;
		const real_t dist_acc     = 0.6*acc*Tacc*Tacc;
		const real_t dist_cruise  = vmax*Tcruise;
		const real_t dist_dec     = vmax*Tdec - 0.5*dec*Tdec*Tdec;
		
		biped = new DiMP::BipedLIP(graph, "biped");
		biped->param.gravity      = 9.8;
		biped->param.comHeight    = 0.65;//1.05;
		biped->param.torsoMass    = mt;
		biped->param.footMass     = mf;
		biped->param.durationMin[DiMP::BipedLIP::Phase::R ] = 0.50;
		biped->param.durationMax[DiMP::BipedLIP::Phase::R ] = 0.50;
		biped->param.durationMin[DiMP::BipedLIP::Phase::L ] = 0.50;
		biped->param.durationMax[DiMP::BipedLIP::Phase::L ] = 0.50;
		biped->param.durationMin[DiMP::BipedLIP::Phase::RL] = 0.10;
		biped->param.durationMax[DiMP::BipedLIP::Phase::RL] = 0.10;
		biped->param.durationMin[DiMP::BipedLIP::Phase::LR] = 0.10;
		biped->param.durationMax[DiMP::BipedLIP::Phase::LR] = 0.10;
		biped->param.durationMin[DiMP::BipedLIP::Phase::D ] = 1.0;
		biped->param.durationMax[DiMP::BipedLIP::Phase::D ] = 1.0;
		biped->param.durationMin[DiMP::BipedLIP::Phase::F ] = 0.1;
		biped->param.durationMax[DiMP::BipedLIP::Phase::F ] = 0.1;
		biped->param.footPosMin[0] = vec3_t(-0.45, -0.20, -1.5);
		biped->param.footPosMax[0] = vec3_t( 0.45, -0.09, -0.5);
		biped->param.footPosMin[1] = vec3_t(-0.45,  0.09, -1.5);
		biped->param.footPosMax[1] = vec3_t( 0.45,  0.20, -0.5);
		biped->param.footOriMin[0] = Rad(-0.0);
		biped->param.footOriMax[0] = Rad( 0.0);
		biped->param.footOriMin[1] = Rad(-0.0);
		biped->param.footOriMax[1] = Rad( 0.0);
		biped->param.footCopMin[0] = vec3_t(-0.07, -0.02, -0.0);
		biped->param.footCopMax[0] = vec3_t( 0.10,  0.02,  0.0);
		biped->param.footCopMin[1] = vec3_t(-0.07, -0.02, -0.0);
		biped->param.footCopMax[1] = vec3_t( 0.10,  0.02,  0.0);
		biped->param.swingHeight   = 0.100;
		biped->param.swingProfile = DiMP::BipedLIP::SwingProfile::Cycloid;
		//biped->param.swingProfile       = DiMP::BipedLIP::SwingProfile::HeelToe;
		//biped->param.swingInterpolation = DiMP::BipedLIP::SwingInterpolation::Cubic;
		biped->param.swingInterpolation = DiMP::BipedLIP::SwingInterpolation::Quintic;
		//biped->param.footCurveType = DiMP::BipedLIP::FootCurveType::Arc;
		//biped->param.ankleToToe    = 0.100;
		//biped->param.ankleToHeel   = 0.070;
		//biped->param.toeCurvature  = 10.0;
		//biped->param.heelCurvature = 10.0;
		biped->param.footCurveType     = DiMP::BipedLIP::FootCurveType::Clothoid;
		biped->param.ankleToToe        = 0.040;
		biped->param.ankleToHeel       = 0.040;
		biped->param.toeCurvatureRate  = 156.0;
		biped->param.heelCurvatureRate = 156.0;
        
		/*
		 D -> RL -> L -> LR -> R ... -> RL -> D
		 */
		for (uint i = 0; i < nphase; i++)
			new DiMP::Tick(graph, 0.0, "");

		biped->phase.resize(nphase);

		int idx = 0;
		biped->phase[idx++] = DiMP::BipedLIP::Phase::D;
		biped->phase[idx++] = DiMP::BipedLIP::Phase::D;
		int side = 0;
		for(int i = 0; i < nstep; i++){
			if(i < nstep_acc){
				biped->phase[idx++] = (side == 0 ? DiMP::BipedLIP::Phase::LR : DiMP::BipedLIP::Phase::RL);
				biped->phase[idx++] = (side == 0 ? DiMP::BipedLIP::Phase::R  : DiMP::BipedLIP::Phase::L );
			}
			else if(i < nstep_acc + nstep_cruise){
				//biped->phase[idx++] = (side == 0 ? DiMP::BipedLIP::Phase::LR  : DiMP::BipedLIP::Phase::RL );
				biped->phase[idx++] = (side == 0 ? DiMP::BipedLIP::Phase::F  : DiMP::BipedLIP::Phase::F );
				biped->phase[idx++] = (side == 0 ? DiMP::BipedLIP::Phase::R  : DiMP::BipedLIP::Phase::L );
			}
			else{
				biped->phase[idx++] = (side == 0 ? DiMP::BipedLIP::Phase::LR : DiMP::BipedLIP::Phase::RL);
				biped->phase[idx++] = (side == 0 ? DiMP::BipedLIP::Phase::R  : DiMP::BipedLIP::Phase::L );
			}
			side = !side;
		}
		biped->phase[idx++] = DiMP::BipedLIP::Phase::D;
		biped->phase[idx++] = DiMP::BipedLIP::Phase::D;
		
		real_t spacing = 0.14/2;
		//vec2_t goalPos(3.0, 0.0);
		//real_t goalOri  = Rad(0.0);

		/* 4 waypoints
		 0-1: start, acceleration
		 1-2: constant speed
		 2-3: deceleration, stop
		 */
		vec3_t one(1.0, 1.0, 1.0);
		biped->waypoints.resize(5);
		biped->waypoints[0].k                 = 0;
		biped->waypoints[0].com_pos           = vec3_t(0.0, 0.0, biped->param.comHeight);
		biped->waypoints[0].com_vel           = vec3_t(0.0, 0.0, 0.0);
		biped->waypoints[0].torso_pos_r       = 0.0;
		biped->waypoints[0].foot_pos_t[0]     = vec3_t(0.0, -spacing, 0.0);
		biped->waypoints[0].foot_pos_r[0]     = 0.0;
		biped->waypoints[0].foot_pos_t[1]     = vec3_t(0.0,  spacing, 0.0);
		biped->waypoints[0].foot_pos_r[1]     = 0.0;
		biped->waypoints[0].weight_com_pos       = 1.0*one;
		biped->waypoints[0].weight_com_vel       = 1.0*one;
		biped->waypoints[0].weight_torso_pos_r   = 1.0;
		biped->waypoints[0].weight_foot_pos_t[0] = 1.0*one;
		biped->waypoints[0].weight_foot_pos_r[0] = 1.0;
		biped->waypoints[0].weight_foot_cop  [0] = 0.0*one;
		biped->waypoints[0].weight_foot_pos_t[1] = 1.0*one;
		biped->waypoints[0].weight_foot_pos_r[1] = 1.0;
		biped->waypoints[0].weight_foot_cop  [1] = 0.0*one;
		
		biped->waypoints[1].k                 = 1 + 2*nstep_idle;
		biped->waypoints[1].com_pos           = vec3_t(0.0, 0.0, biped->param.comHeight);
		biped->waypoints[1].com_vel           = vec3_t(0.0, 0.0, 0.0);
		biped->waypoints[1].torso_pos_r       = 0.0;
		biped->waypoints[1].foot_pos_t[0]     = vec3_t(0.0, -spacing, 0.0);
		biped->waypoints[1].foot_pos_r[0]     = 0.0;
		biped->waypoints[1].foot_pos_t[1]     = vec3_t(0.0,  spacing, 0.0);
		biped->waypoints[1].foot_pos_r[1]     = 0.0;

		biped->waypoints[2].k                 = 1 + 2*(nstep_idle + nstep_acc);
		biped->waypoints[2].com_pos           = vec3_t(dist_acc, 0.0, biped->param.comHeight);
		biped->waypoints[2].com_vel           = vec3_t(vmax, 0.0, 0.0);
		biped->waypoints[2].torso_pos_r       = 0.0;
		biped->waypoints[2].foot_pos_t[0]     = vec3_t(dist_acc, -spacing, 0.0);
		biped->waypoints[2].foot_pos_r[0]     = 0.0;
		biped->waypoints[2].foot_pos_t[1]     = vec3_t(dist_acc,  spacing, 0.0);
		biped->waypoints[2].foot_pos_r[1]     = 0.0;
		
		biped->waypoints[3].k                 = 1 + 2*(nstep_idle + nstep_acc + nstep_cruise);
		biped->waypoints[3].com_pos           = vec3_t(dist_acc + dist_cruise, 0.0, biped->param.comHeight);
		biped->waypoints[3].com_vel           = vec3_t(vmax, 0.0, 0.0);
		biped->waypoints[3].torso_pos_r       = 0.0;
		biped->waypoints[3].foot_pos_t[0]     = vec3_t(dist_acc + dist_cruise, -spacing, 0.0);
		biped->waypoints[3].foot_pos_r[0]     = 0.0;
		biped->waypoints[3].foot_pos_t[1]     = vec3_t(dist_acc + dist_cruise,  spacing, 0.0);
		biped->waypoints[3].foot_pos_r[1]     = 0.0;
		
		biped->waypoints[4].k                 = nphase - 1;
		biped->waypoints[4].com_pos           = vec3_t(dist_acc + dist_cruise + dist_dec, 0.0, biped->param.comHeight);
		biped->waypoints[4].com_vel           = vec3_t(0.0, 0.0, 0.0);
		biped->waypoints[4].torso_pos_r       = 0.0;
		biped->waypoints[4].foot_pos_t[0]     = vec3_t(dist_acc + dist_cruise + dist_dec, -spacing, 0.0);
		biped->waypoints[4].foot_pos_r[0]     = 0.0;
		biped->waypoints[4].foot_pos_t[1]     = vec3_t(dist_acc + dist_cruise + dist_dec,  spacing, 0.0);
		biped->waypoints[4].foot_pos_r[1]     = 0.0;
		biped->waypoints[4].weight_com_pos       = 1.0*one;
		biped->waypoints[4].weight_com_vel       = 1.0*one;
		biped->waypoints[4].weight_torso_pos_r   = 1.0;
		biped->waypoints[4].weight_foot_pos_t[0] = 1.0*one;
		biped->waypoints[4].weight_foot_pos_r[0] = 1.0;
		biped->waypoints[4].weight_foot_cop  [0] = 0.0*one;
		biped->waypoints[4].weight_foot_pos_t[1] = 1.0*one;
		biped->waypoints[4].weight_foot_pos_r[1] = 1.0;
		biped->waypoints[4].weight_foot_cop  [1] = 0.0*one;
		
		graph->scale.Set(1.0, 1.0, 1.0);
		graph->Init();

		graph->solver->SetCorrection(ID(), 0.1);
		graph->solver->param.regularization = 0.01;
		graph->solver->param.hastyStepSize  = false;
		graph->solver->param.numIter[0] = 20;
		graph->solver->param.cutoffStepSize = 0.1;
		graph->solver->param.minStepSize = 1.0;
		graph->solver->param.maxStepSize = 1.0;
		//graph->solver->param.methodMajor = Solver::Method::Major::Prioritized;
		//graph->solver->param.methodMajor = Solver::Method::Major::GaussNewton;
		graph->solver->param.methodMajor = Solver::Method::Major::DDP;
		graph->solver->param.methodMinor = Solver::Method::Minor::Direct;
		//graph->solver->param.useHessian     = false;
		graph->solver->param.verbose        = true;
        graph->solver->param.parallelize    = false;
		graph->solver->param.fixInitialState = true;

		targetPos = vec3_t(0.0, 0.6, -0.2);
		
		//graph->solver->Enable(ID(DiMP::ConTag::BipedLipPos        ), false);
		//graph->solver->Enable(ID(DiMP::ConTag::BipedLipVel        ), false);
		//graph->solver->Enable(ID(DiMP::ConTag::BipedTime          ), false);
		//graph->solver->Enable(ID(DiMP::ConTag::BipedFootPosT      ), false);
		//graph->solver->Enable(ID(DiMP::ConTag::BipedFootPosR      ), false);
		//graph->solver->Enable(ID(DiMP::ConTag::BipedFootCop       ), false);
		//graph->solver->Enable(ID(DiMP::ConTag::BipedFootPosRangeT ), false);
		//graph->solver->Enable(ID(DiMP::ConTag::BipedFootPosRangeR ), false);
		//graph->solver->Enable(ID(DiMP::ConTag::BipedFootCopRange  ), false);
		//graph->solver->Enable(ID(DiMP::ConTag::BipedFootVelZeroT  ), false);
		//graph->solver->Enable(ID(DiMP::ConTag::BipedFootVelZeroR  ), false);
		//graph->solver->Enable(ID(DiMP::ConTag::BipedComPos        ), false);
		//graph->solver->Enable(ID(DiMP::ConTag::BipedComVel        ), false);
		//graph->solver->Enable(ID(DiMP::ConTag::BipedDurationRange ), false);

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
		FILE* file = fopen("log.csv", "w");

		real_t tf = graph->ticks.back()->time;
		real_t dt = 0.001;

		vec3_t com_pos, com_vel, com_acc;
		pose_t foot_pose[2];
		vec3_t foot_vel[2], foot_angvel[2], foot_acc[2], foot_angacc[2];
		vec3_t foot_force[2], foot_moment[2];
		vec3_t foot_cop_pos[2], foot_cop_vel[2];
		real_t foot_cop_weight[2];
		int    foot_contact[2];
		real_t T;

		for(real_t t = 0.0; t <= tf; t += dt){
			biped->ComState(t, com_pos, com_vel, com_acc);
			biped->FootPose(t, 0, foot_pose[0], foot_vel[0], foot_angvel[0], foot_acc[0], foot_angacc[0], foot_contact[0]);
			biped->FootPose(t, 1, foot_pose[1], foot_vel[1], foot_angvel[1], foot_acc[1], foot_angacc[1], foot_contact[1]);
			biped->FootCopState(t, 0, foot_cop_pos[0], foot_cop_vel[0], foot_cop_weight[0]);
			biped->FootCopState(t, 1, foot_cop_pos[1], foot_cop_vel[1], foot_cop_weight[1]);
			T = biped->TValue(t);

			// convert CoP to wrench
			const real_t total_mass = 43.0;
			//real_t T = biped->param.T;

			for(int i = 0; i < 2; i++){
				foot_force [i] = ((total_mass*foot_cop_weight[i])/(T*T))*(com_pos - foot_cop_pos[i]);
				foot_moment[i] = (foot_cop_pos[i] - foot_pose[i].Pos()) % foot_force[i];
			}

			fprintf(file, 
				"%f, "
				"%f, %f, %f, "
				"%f, %f, %f, "
				"%f, %f, %f, %f, %f, %f, %f, "
				"%f, %f, %f, %f, %f, %f, "
				"%f, %f, %f, %f, %f, %f, "
				"%d, "
				"%f, %f, %f, %f, %f, %f, %f, "
				"%f, %f, %f, %f, %f, %f, "
				"%f, %f, %f, %f, %f, %f, "
				"%d\n",
				t,
				com_pos.x, com_pos.y, com_pos.z,
				com_vel.x, com_vel.y, com_vel.z, 
				foot_pose [0].Pos().x, foot_pose[0].Pos().y, foot_pose[0].Pos().z, foot_pose[0].Ori().w, foot_pose[0].x, foot_pose[0].y, foot_pose[0].z,
				foot_vel  [0].x, foot_vel  [0].y, foot_vel  [0].z, foot_angvel[0].x, foot_angvel[0].y, foot_angvel[0].z,
				foot_force[0].x, foot_force[0].y, foot_force[0].z, foot_moment[0].x, foot_moment[0].y, foot_moment[0].z,
				foot_contact[0],
				foot_pose [1].Pos().x, foot_pose[1].Pos().y, foot_pose[1].Pos().z, foot_pose[1].Ori().w, foot_pose[1].x, foot_pose[1].y, foot_pose[1].z,
				foot_vel  [1].x, foot_vel  [1].y, foot_vel  [1].z, foot_angvel[1].x, foot_angvel[1].y, foot_angvel[1].z,
				foot_force[1].x, foot_force[1].y, foot_force[1].z, foot_moment[1].x, foot_moment[1].y, foot_moment[1].z,
				foot_contact[1]
			);
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
