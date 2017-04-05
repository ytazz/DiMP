#include <Springhead.h>
#include <Framework/SprFWApp.h>

#include "BallCathing.h"
#include <string>

using namespace Spr;

class MyApp : public FWApp{
public:
	FWSceneIf*			scene;
	BallCatching2			robot;
	MyDrawConfig		conf;

	bool			running;	///< running the planner
	bool			logging;	///< taking error log
	bool			playing;	///< playing the trajectory
	double			playTime;

	UTTimerIf*		timerDraw;	///< timer id for rendering
	UTTimerIf*		timerPlan;	///< timer id for planning

	bool			renderMode;

	double			vars[nVariableTypes];
	double			errors[nConstraintTypes];	///< 拘束種別の誤差値
	double			error_tc;
	ofstream		errorlog;					///< 拘束誤差のログ
	ostringstream	oss;

	FILE *fp_timing;		/// タイミング変数の更新履歴
	FILE *fp_task;			/// タスク拘束誤差の更新履歴
	FILE *fp_joint;			/// 関節速度の更新履歴
	bool TaskON;

	MyApp(){}
	~MyApp(){}

	void CreateSVG(){
		FILE* file = fopen("view.svg", "w");
		DrawContextSVG draw(file);

		GRRenderIf* render = GetCurrentWin()->GetRender();
		GRCameraDesc camera = render->GetCamera();
		draw.camSize = camera.size;
		draw.vpSize  = render->GetViewportSize();

		draw.SetProjMatrix(Affinef::ProjectionD3D(Vec3f(camera.center.x, camera.center.y, camera.front), camera.size, camera.front, camera.back));
		draw.Transform(GetCurrentWin()->GetTrackball()->GetAffine().inv());

		draw.Header();
		robot.Draw(&draw, &conf);
		draw.Footer();

		fclose(file);
	}

	void PrintData(){
		running = false;
		FILE *fp_traj;
		char *fname2 = "e_humanoid_task.csv";
		fp_traj = fopen(fname2, "w" );
		for(uint i = 0 ; i < robot.events.size(); i++){
			if(i != robot.events.size() - 1 )
				fprintf(fp_traj, "%f,",robot.events[i]->time);
			else
				fprintf(fp_traj, "%f\n",robot.events[i]->time);
		}
		for(uint i = 0 ; i < robot.balls.size(); i++){
			vector<Vec3d> task_e;
			task_e.resize(robot.events.size());
			for(uint j = 0 ; j < robot.events.size(); j++){
				task_e[j] = robot.arm.hand->Pos(robot.position_matchings[i]->time_s->val) - robot.balls[i]->Pos(robot.position_matchings[i]->time_s->val);
				if(j != robot.events.size()-1)
					fprintf(fp_traj, "%f,",task_e[j] * task_e[j]);
				else
					fprintf(fp_traj, "%f\n",task_e[j] * task_e[j]);
			}
			fprintf(fp_traj, "\n");
		}

		fprintf(fp_traj, "\n");

		fclose(fp_traj);
		fclose(fp_timing);
		fclose(fp_joint);
	}

	virtual void BuildObject(){
		// Springheadシーンの構築
		scene = GetSdk()->GetScene(0);
		scene->GetPHScene()->SetTimeStep(0.05);		// 積分幅50ms
		scene->GetPHScene()->SetNumIteration(15);		// 拘束力計算の反復回数
		//scene->GetPHScene()->SetGravity(0.0 * Vec3d(0.0, -9.8, 0.0));	// 重力加速度有効
		scene->GetPHScene()->SetGravity(0.0 * Vec3d(0.0, -0.0, 0.0));	// 重力加速度無効

		/// ロボットの構築
		robot.Build();
		robot.SetInitialState();
		robot.ChargeForce();
		robot.LiftOff();
		robot.TaskStart();

		/// Springheadを1ステップ実行し関節誤差をなくす
		GetSdk()->Step();

		// デバッグ表示モード
		GetSdk()->SetDebugMode(false);

		renderMode = false;
		running = false;
		logging = false;
		playing = false;

		//ファイルオープン
		cout << "timing.csv open" << endl;
		char *fname = "timing.csv";
		fp_timing = fopen( fname, "w" );
		
		cout << "joint.csv open" << endl;
		char* fname3 = "joint.csv";
		fp_joint = fopen( fname3, "w" );

		TaskON = false;

	}

	virtual void Init(int argc, char* argv[]){
		CreateSdk();
		GetSdk()->CreateScene();

		SetGRAdaptee(TypeGLUT);
		GRInit(argc, argv);

		FWWinDesc windowDesc;
		windowDesc.title = "BallCatching";
		CreateWin(windowDesc);
		InitWindow();

		BuildObject();

		EnableIdleFunc(false);

		timerDraw = CreateTimer(UTTimerIf::FRAMEWORK);
		timerDraw->SetInterval(100);

		timerPlan = CreateTimer(UTTimerIf::THREAD);
		timerPlan->SetInterval(10);

	}

	// タイマコールバック関数．タイマ周期で呼ばれる
	virtual void TimerFunc(int id) {
		static uint runCount = 0;
			
		if(id == timerPlan->GetID() && running) {

			//タイミング変数の変化量が0.001以下になったら計算終了
			/*if(robot.position_matchings[0]->time_c->d < 0.01) {
				playing = 0;
			}*/

			// 動作計画を1ステップ実行
			robot.Step();
			cout << runCount++ << endl;

			// タイミング変数をファイルへ出力
			// タイミング変数、拘束誤差

			//for( int i = 0 ; i < robot.balls.size() ; i++){
			//	if(i != robot.balls.size()-1)
			//		fprintf (fp_timing, "%f,%f,,",robot.position_matchings[i]->time_c->val,robot.position_matchings[i]->catkey->con_timing->e);
			//	else
			//		fprintf (fp_timing, "%f,%f\n",robot.position_matchings[i]->time_c->val,robot.position_matchings[i]->catkey->con_timing->e);
			//		cout << "i = " << i << ", time_c = " << robot.position_matchings[i]->time_c->val << endl;
			//}
			
			// 関節速度をファイルへ出力
			/*for(int i = 0 ; i < robot.balls.size() ; i++){
				if(i != robot.balls.size()-1)
					fprintf (fp_joint, "%f,",VariableNames[5]);
				else
					fprintf (fp_joint, "%f\n",VariableNames[5]);
			}*/

			// 拘束誤差ログ
			for(uint i = 0; i < nConstraintTypes; i++){
				errors[i] = robot.CalcError(ID(i), false, true);	///< max
				if(logging){
					errorlog << errors[i];
					errorlog << ", ";
				}
			}

			// 変数値
			for(uint i = 0; i < nVariableTypes; i++){
				vars[i] = robot.CalcVariable(ID(i), false);	///< max
			}
		}

		if(id == timerDraw->GetID()){
			// 再生時刻を進める
			if(playing){
				playTime += 0.05;
				if(playTime > robot.events.back()->time)
					playTime = 0.0;
			}

			// 再描画要求
			PostRedisplay();
		}
	}

	// 描画関数．描画要求が来たときに呼ばれる
	virtual void Display() {
		GRRenderIf *render = GetCurrentWin()->GetRender();

		//render->SetClearColor(Vec4f(0.8f, 0.8f, 0.8f, 1.0f));
		render->SetClearColor(Vec4f(1.0f, 1.0f, 1.0f, 1.0f));
		render->ClearBuffer();
		render->BeginScene();

		GRCameraDesc camera = render->GetCamera();
		camera.front = 0.3;
		render->SetCamera(camera);
		render->SetViewMatrix(GetCurrentWin()->GetTrackball()->GetAffine().inv());


		render->SetLighting(false);
		render->SetDepthTest(false);

		//
		DrawContextGL gl;
		gl.camFront = camera.front;
		gl.camSize = camera.size;
		gl.vpSize = render->GetViewportSize();
		Affinef aff;
		render->GetProjectionMatrix(aff);
		gl.SetProjMatrix(aff);
		render->GetViewMatrix(aff);
		gl.SetViewMatrix(aff);

		robot.Draw(&gl, &conf);
		if(playing){
			conf.snapshot = true;
			gl.mat->lineWidth = 4.0;
			gl.mat->lineColor = Vec4f(0.0, 0.0, 0.0, 1.0);
			for(uint i = 0; i < robot.links.size(); i++)
				robot.links[i]->DrawSnapshot(playTime, &gl, &conf);
			for(uint i = 0; i < robot.balls.size(); i++)
				robot.balls[i]->DrawSnapshot(playTime, &gl, &conf);
			conf.snapshot = false;
		}

		// 文字情報の表示
		gl.EnterOverlay();
		gl.mat->textColor = Vec4f(0.0f, 0.0f, 0.0f, 1.0f);
		Vec2f posLabel(30.0f, 30.0f);		///< ラベル表示位置
		Vec2f posValue(200.0f, 30.0f);		///< 数値表示位置
		float lineSpace = 20.0f;

		// 拘束誤差
		for(uint i = 0; i < nConstraintTypes; i++){
			gl.Text(ConstraintNames[i], posLabel);
			oss.str("");
			oss << errors[i];
			gl.Text(oss.str().c_str(), posValue);
			posLabel.y += lineSpace;
			posValue.y += lineSpace;
		}
		posLabel.y += lineSpace;
		posValue.y += lineSpace;

		// 変数値
		for(uint i = 0; i < nVariableTypes; i++){
			gl.Text(VariableNames[i], posLabel);
			oss.str("");
			oss << vars[i];
			gl.Text(oss.str().c_str(), posValue);
			posLabel.y += lineSpace;
			posValue.y += lineSpace;
		}

		gl.LeaveOverlay();

		render->EndScene();
		render->SwapBuffers();

	}

	virtual void Keyboard(int key, int x, int y) {
		if(key == ' ') {
			running = !running;
		}
		else if(key == '0') {
			robot.SetInitialState();
		}
		else if(key == '1') {
			robot.ChargeForce();
		}
		else if(key == '2') {
			robot.LiftOff();
		}
		else if(key == '3') {
			robot.TaskStart();
			TaskON = true;
		}
		else if(key == 'e') {
			if(logging){
				cout << "end logging" << endl;
				errorlog.close();
				logging = false;
			}
			else{
				cout << "start logging" << endl;
				errorlog.open("error.csv");
				logging = true;
			}
		}
		else if(key == 'p') {
			if(playing){
				cout << "end playing" << endl;
				playing = false;
			}
			else{
				cout << "start playing" << endl;
				playing = true;
				playTime = 0.0;
			}
		}
		else if(key == 's'){
			cout << "generate svg image" << endl;
			CreateSVG();
		}
		else if(key == 'd'){
			cout << "PrintData" << endl;
			PrintData();
		}
		FWApp::Keyboard(key, x, y);
	}

};

////////////////////////////////////////////////////////////////////////////////////////////////////////////

MyApp app;

int main(int argc, char *argv[]) {
	app.Init(argc, argv);
	app.StartMainLoop();
	return 0;
}
