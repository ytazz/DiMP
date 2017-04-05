#define _CRT_SECURE_NO_WARNINGS

#include "../SampleApp.h"
#include "HumanoidCatching.h"

#include <boost/foreach.hpp>
#define foreach BOOST_FOREACH
using namespace boost;

class MyDrawConfig : public DiMP2::DrawConfig{
public:
	MyDrawConfig(){
		materials[DrawConfig::Trajectory].lineColor = Vec4f(0.0, 0.0, 0.0, 1.0);
		materials[DrawConfig::Trajectory].lineWidth = 2;
		materials[DrawConfig::JointPos].lineColor = Vec4f(0.0, 0.0, 0.0, 1.0);
		materials[DrawConfig::JointForce].lineColor = Vec4f(0.5, 0.5, 0.5, 1.0);
		materials[DrawConfig::JointMoment].lineColor = Vec4f(0.5, 0.5, 0.5, 1.0);

		scaleForce  = 0.01;
		scaleMoment = 0.1;
	}
};

class MyApp : public SampleApp{
public:
	enum{
		ID_INIT_STATE,
		ID_CHARGE_FORCE,
		ID_LIFT_OFF,
		ID_TASK_START,
		ID_PRINT_DATA,
	};

	Humanoid		robot;
	MyDrawConfig	myConf;

	double			error_tc;
	FILE*			fp_timing;
	FILE			*fp_task;

	MyApp(){
		graph = &robot;
		drawConfig = &myConf;

		appName = "HumanoidCatcher";

		AddAction(MENU_USER, ID_INIT_STATE, "initialize state")
			->AddHotKey('0');
		AddAction(MENU_USER, ID_CHARGE_FORCE, "charge force")
			->AddHotKey('1');
		AddAction(MENU_USER, ID_LIFT_OFF, "lift off")
			->AddHotKey('2');
		AddAction(MENU_USER, ID_TASK_START, "task start")
			->AddHotKey('3');
		AddAction(MENU_USER, ID_PRINT_DATA, "print data")
			->AddHotKey('P');
	}
	~MyApp(){}

	void PrintData(){
		running = false;
		FILE *fp_traj;
		char *fname2 = "e_humanoid_task.csv";
		fp_traj = fopen(fname2, "w" );
		foreach(DiMP2::Tick* tick, robot.ticks){
			fprintf(fp_traj, "%f,", tick->time);
		}
		fprintf(fp_traj, "\n");

		for(uint i = 0 ; i < robot.targetInfos.size(); i++){
			foreach(Humanoid::TargetInfo info, robot.targetInfos){
				foreach(DiMP2::Tick* tick, robot.ticks){
					double task_e =(info.hand->GetKeypoint(tick)->pos_t->val - info.target->GetKeypoint(tick)->pos_t->val).norm();
					//				vec3_t task_e = 
					//					robot.targetInfos[i].hand->GetKeypoint(tick)->pos_t->val - robot.targetInfos[i].target->GetKeypoint(tick)->pos_t->val;
					fprintf(fp_traj, "%f,", task_e);
				}
				fprintf(fp_traj, "\n");
			}
			fprintf(fp_traj, "\n");

			fclose(fp_traj);
			fclose(fp_task);
			fclose(fp_timing);
		}
	}
		virtual void BuildScene(){
			/// ロボットの構築
			robot.Build();
			robot.SetInitialState();
			robot.ChargeForce();
			robot.LiftOff();
			robot.TaskStart();

			//ファイルオープン
			cout << "timing.csv open" << endl;
			char *fname = "timing.csv";
			fp_timing = fopen( fname, "w" );

			cout << "task_errors.csv open" << endl;
			fname = "task_errors.csv";
			fp_task = fopen(fname,"w");
		}

		virtual void OnAction(int menu, int id){
			if(menu == MENU_USER){
				if(id == ID_INIT_STATE)
					robot.SetInitialState();
				if(id == ID_CHARGE_FORCE)
					robot.ChargeForce();
				if(id == ID_LIFT_OFF)
					robot.LiftOff();
				if(id == ID_TASK_START)
					robot.TaskStart();
				if(id == ID_PRINT_DATA)
					PrintData();
			}
			else SampleApp::OnAction(menu, id);
		}

		virtual void OnStep(){
			SampleApp::OnStep();
			// タイミング変数とタスク拘束の誤差をファイルへ出力
			foreach(Task* task, robot.tasks){
				MatchingTask* mtask = (MatchingTask*)task;
				if(task == robot.tasks.back()){
					fprintf(fp_timing, "%f,%f,\n", mtask->time->time_s->val, mtask->time->time_e->val);
					fprintf(fp_task, "%f,\n", sqrt(mtask->CalcTrajError()));			
				}
				else{
					fprintf(fp_timing, "%f,%f,,", mtask->time->time_s->val, mtask->time->time_e->val);
					fprintf(fp_task, "%f,,", sqrt(mtask->CalcTrajError()));		
				}
			}
		}
	};

	////////////////////////////////////////////////////////////////////////////////////////////////////////////

	MyApp app;

	int main(int argc, char *argv[]) {
		app.Init(argc, argv);
		app.StartMainLoop();
		return 0;
	}
