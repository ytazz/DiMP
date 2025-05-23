﻿#include <DiMP/DiMP.h>

#include "mpc.h"

using namespace DiMP;

class MyApp : public DiMP::App, public DiMP::Render::Config{
public:
	enum {
		MENU_MAIN = MENU_USER,
	};

	DiMP::Wholebody*	wb;
	MyIK*               myik;
	Mpc*                mpc;

public:
	MyApp() {
		appName = "Wholebody";
		zAxisUp = true;

	}
	virtual ~MyApp() {}

	virtual void BuildScene() {

		mpc  = new Mpc ();
		mpc->updateCycle = 10;
		mpc->N           = 5;
		mpc->dt          = 0.010;
		
		for(int k = 0; k <= mpc->N; k++){
			new DiMP::Tick(graph, k*mpc->dt, "");
		}

		graph->scale.Set(1.0, 1.0, 1.0);
		wb   = new DiMP::Wholebody(graph, "wb");
		myik = new MyIK();
		
		wb->links .resize(MyIK::Link ::Num);
		wb->joints.resize(MyIK::Link ::Num - 1);
		wb->ends  .resize(MyIK::End  ::Num);
		
		vec3_t e0;
		vec3_t ex(1.0, 0.0, 0.0);
		vec3_t ey(0.0, 1.0, 0.0);
		vec3_t ez(0.0, 0.0, 1.0);
		wb->links[ 0] = Wholebody::Link(10.0, vec3_t(0.0001, 0.0001, 0.0001), e0, -1, -1, -1, e0                  , e0);
		wb->links[ 1] = Wholebody::Link( 5.0, vec3_t(0.0001, 0.0001, 0.0001), e0, -1,  0,  0, myik->torsoLength*ez, ez);
		wb->links[ 2] = Wholebody::Link( 5.0, vec3_t(0.0001, 0.0001, 0.0001), e0,  0,  1,  1, e0                  , ey);
		wb->links[ 3] = Wholebody::Link( 2.0, vec3_t(0.0001, 0.0001, 0.0001), e0, -1,  2,  2, e0                  , ez);
		wb->links[ 4] = Wholebody::Link( 2.0, vec3_t(0.0001, 0.0001, 0.0001), e0, -1,  3,  3, e0                  , ey);
		
		wb->links[ 5] = Wholebody::Link( 0.5, vec3_t(0.0001, 0.0001, 0.0001), e0, -1,  2,  4, myik->armBase[0]        , ey);
		wb->links[ 6] = Wholebody::Link( 0.5, vec3_t(0.0001, 0.0001, 0.0001), e0, -1,  5,  5, e0                      , ex);
		wb->links[ 7] = Wholebody::Link( 1.0, vec3_t(0.0001, 0.0001, 0.0001), e0, -1,  6,  6, e0                      , ez);
		wb->links[ 8] = Wholebody::Link( 0.5, vec3_t(0.0001, 0.0001, 0.0001), e0, -1,  7,  7, -myik->upperArmLength*ez, ey);
		wb->links[ 9] = Wholebody::Link( 1.0, vec3_t(0.0001, 0.0001, 0.0001), e0, -1,  8,  8, -myik->lowerArmLength*ez, ez);
		wb->links[10] = Wholebody::Link( 0.5, vec3_t(0.0001, 0.0001, 0.0001), e0, -1,  9,  9, e0                      , ey);
		wb->links[11] = Wholebody::Link( 0.5, vec3_t(0.0001, 0.0001, 0.0001), e0,  1, 10, 10, e0                      , ex);
		
		wb->links[12] = Wholebody::Link( 0.5, vec3_t(0.0001, 0.0001, 0.0001), e0, -1,  2, 11, myik->armBase[1]        , ey);
		wb->links[13] = Wholebody::Link( 0.5, vec3_t(0.0001, 0.0001, 0.0001), e0, -1, 12, 12, e0                      , ex);
		wb->links[14] = Wholebody::Link( 1.0, vec3_t(0.0001, 0.0001, 0.0001), e0, -1, 13, 13, e0                      , ez);
		wb->links[15] = Wholebody::Link( 0.5, vec3_t(0.0001, 0.0001, 0.0001), e0, -1, 14, 14, -myik->upperArmLength*ez, ey);
		wb->links[16] = Wholebody::Link( 1.0, vec3_t(0.0001, 0.0001, 0.0001), e0, -1, 15, 15, -myik->lowerArmLength*ez, ez);
		wb->links[17] = Wholebody::Link( 0.5, vec3_t(0.0001, 0.0001, 0.0001), e0, -1, 16, 16, e0                      , ey);
		wb->links[18] = Wholebody::Link( 0.5, vec3_t(0.0001, 0.0001, 0.0001), e0,  2, 17, 17, e0                      , ex);

		wb->links[19] = Wholebody::Link( 0.5, vec3_t(0.0001, 0.0001, 0.0001), e0, -1,  0, 18, myik->legBase[0]        , ez);
		wb->links[20] = Wholebody::Link( 0.5, vec3_t(0.0001, 0.0001, 0.0001), e0, -1, 19, 19, e0                      , ex);
		wb->links[21] = Wholebody::Link( 1.5, vec3_t(0.0001, 0.0001, 0.0001), e0, -1, 20, 20, e0                      , ey);
		wb->links[22] = Wholebody::Link( 1.5, vec3_t(0.0001, 0.0001, 0.0001), e0, -1, 21, 21, -myik->upperLegLength*ez, ey);
		wb->links[23] = Wholebody::Link( 0.5, vec3_t(0.0001, 0.0001, 0.0001), e0, -1, 22, 22, -myik->lowerLegLength*ez, ey);
		wb->links[24] = Wholebody::Link( 0.5, vec3_t(0.0001, 0.0001, 0.0001), e0,  3, 23, 23, e0                      , ex);

		wb->links[25] = Wholebody::Link( 0.5, vec3_t(0.0001, 0.0001, 0.0001), e0, -1,  0, 24, myik->legBase[1]        , ez);
		wb->links[26] = Wholebody::Link( 0.5, vec3_t(0.0001, 0.0001, 0.0001), e0, -1, 25, 25, e0                      , ex);
		wb->links[27] = Wholebody::Link( 1.5, vec3_t(0.0001, 0.0001, 0.0001), e0, -1, 26, 26, e0                      , ey);
		wb->links[28] = Wholebody::Link( 1.5, vec3_t(0.0001, 0.0001, 0.0001), e0, -1, 27, 27, -myik->upperLegLength*ez, ey);
		wb->links[29] = Wholebody::Link( 0.5, vec3_t(0.0001, 0.0001, 0.0001), e0, -1, 28, 28, -myik->lowerLegLength*ez, ey);
		wb->links[30] = Wholebody::Link( 0.5, vec3_t(0.0001, 0.0001, 0.0001), e0,  4, 29, 29, e0                      , ex);

		wb->SetScaling();

		wb->ends[0] = Wholebody::End(MyIK::Link::ChestP, vec3_t());
		wb->ends[1] = Wholebody::End(MyIK::Link::HandRR, myik->wristToHand[0]);
		wb->ends[2] = Wholebody::End(MyIK::Link::HandLR, myik->wristToHand[1]);
		wb->ends[3] = Wholebody::End(MyIK::Link::FootRR, myik->ankleToFoot[0]);
		wb->ends[4] = Wholebody::End(MyIK::Link::FootLR, myik->ankleToFoot[1]);

		mpc->wb   = wb;
		wb->callback = mpc;
		mpc->Init();		
		
		graph->Init();

		//graph->solver->Enable(ID(DiMP::ConTag::WholebodyPosT         ), false);
		//graph->solver->Enable(ID(DiMP::ConTag::WholebodyPosR         ), false);
		//graph->solver->Enable(ID(DiMP::ConTag::WholebodyVelT         ), false);
		//graph->solver->Enable(ID(DiMP::ConTag::WholebodyVelR         ), false);
		//graph->solver->Enable(ID(DiMP::ConTag::WholebodyAccT         ), false);
		//graph->solver->Enable(ID(DiMP::ConTag::WholebodyAccR         ), false);
		//graph->solver->Enable(ID(DiMP::ConTag::WholebodyForceT       ), false);
		//graph->solver->Enable(ID(DiMP::ConTag::WholebodyForceR       ), false);
		//graph->solver->Enable(ID(DiMP::ConTag::WholebodyLimit        ), false);
		//graph->solver->Enable(ID(DiMP::ConTag::WholebodyContactPosT  ), false);
		//graph->solver->Enable(ID(DiMP::ConTag::WholebodyContactPosR  ), false);
		//graph->solver->Enable(ID(DiMP::ConTag::WholebodyContactVelT  ), false);
		//graph->solver->Enable(ID(DiMP::ConTag::WholebodyContactVelR  ), false);
		//graph->solver->Enable(ID(DiMP::ConTag::WholebodyNormalForce  ), false);
		//graph->solver->Enable(ID(DiMP::ConTag::WholebodyFrictionForce), false);
		//graph->solver->Enable(ID(DiMP::ConTag::WholebodyMoment       ), false);
		//graph->solver->Enable(ID(DiMP::ConTag::WholebodyComPos       ), false);
		//graph->solver->Enable(ID(DiMP::ConTag::WholebodyComVel       ), false);
		//graph->solver->Enable(ID(DiMP::ConTag::WholebodyMomentum     ), false);
		
		//graph->solver->SetCorrection(ID(), 1.0 - pow(0.9, mpc->dt/0.1));
		graph->solver->SetCorrection(ID(), 0.5);
		graph->solver->param.numIter[0] = 20;
		graph->solver->param.cutoffStepSize = 0.01;
		graph->solver->param.regularization = 1.0e-5;
		graph->solver->param.minStepSize = 1.0;
		graph->solver->param.maxStepSize = 1.0;
		graph->solver->param.hastyStepSize = true;
		//graph->solver->param.methodMajor = Solver::Method::Major::GaussNewton;
		graph->solver->param.methodMajor = Solver::Method::Major::DDP;
		//graph->solver->param.methodMajor = Solver::Method::Major::DDPContinuous;
		graph->solver->param.methodMinor = Solver::Method::Minor::Direct;
		graph->solver->param.verbose = true;

	}

	virtual void OnStep(){
		mpc->UpdateInput();
		mpc->UpdateState();

		if(mpc->count > 0 && mpc->count % mpc->updateCycle == 0){
			// store computed result to front buffer
			mpc->UpdateGain();

			// calc input here to set as initial condition of next mpc
			mpc->UpdateInput();

			// reset time and state and start next optimization
			mpc->timeMpc = mpc->time;
			wb->Shift(mpc->dt);
			wb->Setup();
			//wb->Reset();
			graph->solver->InitDDP();

			//mpc->updateCycle = 10;
		}

		mpc->Countup();

		App::OnStep();
	}

	virtual void OnAction(int menu, int id) {
		if (menu == MENU_MAIN) {
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
