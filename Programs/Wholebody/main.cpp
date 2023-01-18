#include <DiMP/DiMP.h>

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
		mpc->N           = 10;
		mpc->dt          = 0.025;
		
		for(int k = 0; k <= mpc->N; k++){
			new DiMP::Tick(graph, k*mpc->dt, "");
		}

		graph->scale.Set(1.0, 1.0, 1.0);
		wb = new DiMP::Wholebody(graph, "wb");
		
		wb->links .resize(MyIK::Link ::Num);
		wb->joints.resize(MyIK::Link ::Num - 1);
		wb->ends  .resize(MyIK::End  ::Num);
		wb->chains.resize(MyIK::Chain::Num);
		wb->limits.resize(MyIK::Limit::Num);

		vec3_t ex(1.0, 0.0, 0.0);
		vec3_t ey(0.0, 1.0, 0.0);
		vec3_t ez(0.0, 0.0, 1.0);
		wb->links[ 0] = Wholebody::Link(10.0, -1, -1);
		wb->links[ 1] = Wholebody::Link( 5.0,  0,  0, ez);
		wb->links[ 2] = Wholebody::Link( 5.0,  1,  1, ey);
		wb->links[ 3] = Wholebody::Link( 2.0,  2,  2);
		wb->links[ 4] = Wholebody::Link( 2.0,  3,  3);
		
		wb->links[ 5] = Wholebody::Link( 0.5,  2,  4, ey);
		wb->links[ 6] = Wholebody::Link( 0.5,  5,  5, ex);
		wb->links[ 7] = Wholebody::Link( 1.0,  6,  6, ez);
		wb->links[ 8] = Wholebody::Link( 0.5,  7,  7, ey);
		wb->links[ 9] = Wholebody::Link( 1.0,  8,  8, ez);
		wb->links[10] = Wholebody::Link( 0.5,  9,  9, ey);
		wb->links[11] = Wholebody::Link( 0.5, 10, 10, ex);
		
		wb->links[12] = Wholebody::Link( 0.5,  2, 11, ey);
		wb->links[13] = Wholebody::Link( 0.5, 12, 12, ex);
		wb->links[14] = Wholebody::Link( 1.0, 13, 13, ez);
		wb->links[15] = Wholebody::Link( 0.5, 14, 14, ey);
		wb->links[16] = Wholebody::Link( 1.0, 15, 15, ez);
		wb->links[17] = Wholebody::Link( 0.5, 16, 16, ey);
		wb->links[18] = Wholebody::Link( 0.5, 17, 17, ex);

		wb->links[19] = Wholebody::Link( 0.5,  0, 18, ez);
		wb->links[20] = Wholebody::Link( 0.5, 19, 19, ex);
		wb->links[21] = Wholebody::Link( 1.5, 20, 20, ey);
		wb->links[22] = Wholebody::Link( 1.5, 21, 21, ey);
		wb->links[23] = Wholebody::Link( 0.5, 22, 22, ey);
		wb->links[24] = Wholebody::Link( 0.5, 23, 23, ex);

		wb->links[25] = Wholebody::Link( 0.5,  0, 24, ez);
		wb->links[26] = Wholebody::Link( 0.5, 25, 25, ex);
		wb->links[27] = Wholebody::Link( 1.5, 26, 26, ey);
		wb->links[28] = Wholebody::Link( 1.5, 27, 27, ey);
		wb->links[29] = Wholebody::Link( 0.5, 28, 28, ey);
		wb->links[30] = Wholebody::Link( 0.5, 29, 29, ex);

		wb->SetScaling();

		wb->ends[0].ilink = MyIK::Link::Hips;
		wb->ends[1].ilink = MyIK::Link::ChestP;
		wb->ends[2].ilink = MyIK::Link::HandRR;
		wb->ends[3].ilink = MyIK::Link::HandLR;
		wb->ends[4].ilink = MyIK::Link::FootRR;
		wb->ends[5].ilink = MyIK::Link::FootLR;

		wb->limits[ 0] = Wholebody::Limit(MyIK::Chain::Torso, Constraint::Type::Equality         , wb->spt);
		wb->limits[ 1] = Wholebody::Limit(MyIK::Chain::Torso, Constraint::Type::Equality         , wb->spt);
		wb->limits[ 2] = Wholebody::Limit(MyIK::Chain::Torso, Constraint::Type::Equality         , wb->spt);
		wb->limits[ 3] = Wholebody::Limit(MyIK::Chain::Torso, Constraint::Type::Equality         , wb->spt);
		wb->limits[ 4] = Wholebody::Limit(MyIK::Chain::ArmR , Constraint::Type::InequalityPenalty, 1.0    );
		wb->limits[ 5] = Wholebody::Limit(MyIK::Chain::ArmR , Constraint::Type::InequalityPenalty, 1.0    );
		wb->limits[ 6] = Wholebody::Limit(MyIK::Chain::ArmL , Constraint::Type::InequalityPenalty, 1.0    );
		wb->limits[ 7] = Wholebody::Limit(MyIK::Chain::ArmL , Constraint::Type::InequalityPenalty, 1.0    );
		wb->limits[ 8] = Wholebody::Limit(MyIK::Chain::LegR , Constraint::Type::InequalityPenalty, 1.0    );
		wb->limits[ 9] = Wholebody::Limit(MyIK::Chain::LegR , Constraint::Type::InequalityPenalty, 1.0    );
		wb->limits[10] = Wholebody::Limit(MyIK::Chain::LegL , Constraint::Type::InequalityPenalty, 1.0    );
		wb->limits[11] = Wholebody::Limit(MyIK::Chain::LegL , Constraint::Type::InequalityPenalty, 1.0    );
		
		wb->chains[0] = Wholebody::Chain(MyIK::End::Hips  , MyIK::End::ChestP, {0, 1, 2, 3, 4}             , {0, 1, 2, 3});
		wb->chains[1] = Wholebody::Chain(MyIK::End::ChestP, MyIK::End::HandR , {5, 6, 7, 8, 9, 10, 11}     , {4, 5}      );
		wb->chains[2] = Wholebody::Chain(MyIK::End::ChestP, MyIK::End::HandL , {12, 13, 14, 15, 16, 17, 18}, {6, 7}      );
		wb->chains[3] = Wholebody::Chain(MyIK::End::Hips  , MyIK::End::FootR , {19, 20, 21, 22, 23, 24}    , {8, 9}      );
		wb->chains[4] = Wholebody::Chain(MyIK::End::Hips  , MyIK::End::FootL , {25, 26, 27, 28, 29, 30}    , {10, 11}    );
		
		myik = new MyIK();
		mpc->wb   = wb;
		mpc->myik = myik;
		mpc->Init();
		
		wb->callback = mpc;

		graph->Init();

		//graph->solver->Enable(ID(DiMP::ConTag::WholebodyPosT         ), false);
		//graph->solver->Enable(ID(DiMP::ConTag::WholebodyPosR         ), false);
		//graph->solver->Enable(ID(DiMP::ConTag::WholebodyCompl        ), false);
		//graph->solver->Enable(ID(DiMP::ConTag::WholebodyComPos       ), false);
		//graph->solver->Enable(ID(DiMP::ConTag::WholebodyComVel       ), false);
		//graph->solver->Enable(ID(DiMP::ConTag::WholebodyMomentum     ), false);
		//graph->solver->Enable(ID(DiMP::ConTag::WholebodyTotalForce   ), false);
		//graph->solver->Enable(ID(DiMP::ConTag::WholebodyTotalMoment  ), false);		
		//graph->solver->Enable(ID(DiMP::ConTag::WholebodyLimit        ), false);
		//graph->solver->Enable(ID(DiMP::ConTag::WholebodyNormalForce  ), false);
		//graph->solver->Enable(ID(DiMP::ConTag::WholebodyFrictionForce), false);
		//graph->solver->Enable(ID(DiMP::ConTag::WholebodyMoment       ), false);
		//graph->solver->Enable(ID(DiMP::ConTag::WholebodyComPosMatch  ), false);
		//graph->solver->Enable(ID(DiMP::ConTag::WholebodyComVelMatch  ), false);
		//graph->solver->Enable(ID(DiMP::ConTag::WholebodyMomentumMatch), false);
		
		graph->solver->SetCorrection(ID(), 0.1);
		graph->solver->param.numIter[0] = 20;
		graph->solver->param.cutoffStepSize = 0.01;
		graph->solver->param.regularization = 1.0e-1;
		graph->solver->param.minStepSize = 0.01;
		graph->solver->param.maxStepSize = 1.0;
		graph->solver->param.hastyStepSize = true;
		//graph->solver->param.methodMajor = Solver::Method::Major::GaussNewton;
		graph->solver->param.methodMajor = Solver::Method::Major::DDP;
		graph->solver->param.methodMinor = Solver::Method::Minor::Direct;
		graph->solver->param.verbose = true;

	}

	virtual void OnStep(){
		mpc->UpdateInput();
		mpc->UpdateState();

		if(mpc->count > 0 && mpc->count % mpc->updateCycle == 0){
			// store computed result to front buffer
			mpc->UpdateGain();

			// reset time and state and start next optimization
			mpc->timeMpc = mpc->time;
			wb->Setup();
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
