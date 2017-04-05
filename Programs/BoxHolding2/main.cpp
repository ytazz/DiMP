#include <Springhead.h>
#include <Framework/SprFWApp.h>

#include "BoxPositionMatching.h"
#include <string>

using namespace Spr;

#define ARRAY_SIZE(x) (sizeof(x)/sizeof(x[0]))


class MyApp : public FWApp{
public:
	FWSceneIf*		scene;
	BoxHolder		robot;
	MyDrawConfig	conf;

	bool			running;	///< running the planner
	bool			logging;	///< taking error log
	bool			playing;	///< playing the trajectory
	double			playTime;

	UTTimerIf*		timerDraw;	///< timer id for rendering
	UTTimerIf*		timerPlan;	///< timer id for planning


	bool			renderMode;
	char			activeAxis;
	Vec3d			targetPos;

	double			vars[nVariableTypes];
	double			errors[nConstraintTypes];	///< 拘束種別の誤差値
	double			error_tc;
	ofstream		errorlog;					///< 拘束誤差のログ
	ostringstream	oss;

	FILE *fp;
	bool TaskON;

	MyApp(){}
	~MyApp(){}

	void CreateSVG(){
FILE* file = fopen("view.svg", "w");
		DrawContextSVG draw(file);

		GRRenderIf* render = GetSdk()->GetRender();
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

	void Printdate(){
		running = false;
		FILE *fp2;
		char *fname2 = "e_interval_task.csv";
		fp2 = fopen(fname2, "w" );
		for(int i = 0 ; i < robot.events.size(); i++){
			if(i != robot.events.size() - 1 )
				fprintf(fp2, "%f,",robot.events[i]->time);
			else
				fprintf(fp2, "%f\n",robot.events[i]->time);
		}
		for(int i = 0 ; i < robot.boxes.size(); i++){
			vector<Vec3d> task_e;
			task_e.resize(robot.events.size());
			for(int j = 0 ; j < robot.events.size(); j++){
				task_e[j] = robot.links.back()->
					GetKeypoint(robot.events[j])->pos_t->val - robot.boxes[i]->GetKeypoint(robot.events[j])->pos_t->val;
				if(j != robot.events.size()-1)
					fprintf(fp2, "%f,",task_e[j] * task_e[j]);
				else
					fprintf(fp2, "%f\n",task_e[j] * task_e[j]);
			}
			fprintf(fp2, "\n");
		}
		fclose(fp2);

	}

	virtual void BuildObject(){
		// Springheadシーンの構築
		scene = GetSdk()->GetScene();
		scene->GetPHScene()->SetTimeStep(0.05);		// 積分幅50ms
		scene->GetPHScene()->SetNumIteration(15);		// 拘束力計算の反復回数
		//scene->GetPHScene()->SetGravity(0.0 * Vec3d(0.0, -9.8, 0.0));	// 重力加速度有効
		scene->GetPHScene()->SetGravity(0.0 * Vec3d(0.0, -0.0, 0.0));	// 重力加速度無効

		/// ロボットの構築
		//robot.Build(GetSdk()->GetPHSdk(), scene->GetPHScene());
		//planner.robot = &robot;
		robot.Build();
		robot.SetInitialState();
		robot.ChargeForce();
		robot.LiftOff();
		robot.TaskStart();

		// ソルバを選択
		//robot.solver.solverType = STEEPEST_DESCENT;
		//robot.solver.solverType = GAUSS_SEIDEL;

		/// Springheadを1ステップ実行し関節誤差をなくす
		GetSdk()->Step();

		activeAxis = 'x';				// 目標物体を動かす方向

		// デバッグ表示モード
		GetSdk()->SetDebugMode(false);

		renderMode = false;
		running = false;
		logging = false;
		playing = false;

		//ファイルオープン
		//cout << "timing.csv open" << endl;
		//char *fname = "timing.csv";
		//fp = fopen( fname, "w" );
		//TaskON = false;

		//for(int i = 0 ; i < robot.ball.size() ; i++){
		//	if(i != robot.ball.size()-1)
		//		fprintf (fp, "%f,,",robot.catching[i]->time_c->val);
		//	else
		//		fprintf (fp, "%f\n",robot.catching[i]->time_c->val);
		//	cout << "i = " << i << ", time_c = " << robot.catching[i]->time_c->val << endl;
		//}

	}

	virtual void Init(int argc, char* argv[]){
		CreateSdk();
		GetSdk()->CreateScene();

		SetGRAdaptee(TypeGLUT);
		GRInit(argc, argv);

		FWWinDesc windowDesc;
		windowDesc.title = "BoxHolder";
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

			// 動作計画を1ステップ実行
			robot.Step();
			cout << runCount++ << endl;

			// タイミング変数をファイルへ出力
			//for(int i = 0 ; i < robot.ball.size() ; i++){
			//	if(i != robot.ball.size()-1)
			//		fprintf (fp, "%f,,",robot.catching[i]->time_c->val);
			//	else
			//		fprintf (fp, "%f\n",robot.catching[i]->time_c->val);
			//	cout << "i = " << i << ", time_c = " << robot.catching[i]->time_c->val << endl;
			//}

			/*double max_e = 0.0;
			double e0, e1;
			if(robot.ball.size() > 1){
			for(int i = 0 ; i < robot.ball.size() - 1; i++){
			e0 = robot.catching[i  ]->catkey->con_timing->e.square();
			e1 = robot.catching[i+1]->catkey->con_timing->e.square();
			max_e = std::max(e0, e1);
			}
			}
			else max_e = robot.catching[0]->catkey->con_timing->e.square();

			cout << sqrt(max_e) << endl;*/

			/*
			if(  fabs(max_e - error_tc) < 0.001 && max_e < 0.4){
			//if(  fabs(max_e - error_tc) < 0.0001 && max_e < 0.3){
			running = false;
			FILE *fp2;
			char *fname2 = "e_task.csv";
			fp2 = fopen(fname2, "w" );
			for(int i = 0 ; i < robot.events.size(); i++){
			if(i != robot.events.size() - 1 )
			fprintf(fp2, "%f,",robot.events[i]->time);
			else
			fprintf(fp2, "%f\n",robot.events[i]->time);
			}
			for(int i = 0 ; i < robot.ball.size(); i++){
			vector<Vec3d> task_e;
			task_e.resize(robot.events.size());
			for(int j = 0 ; j < robot.events.size(); j++){
			task_e[j] = robot.links.back()->GetKeypoint(robot.events[j])->pos_t->val - robot.ball[i]->GetKeypoint(robot.events[j])->pos_t->val;
			if(j != robot.events.size()-1)
			fprintf(fp2, "%f,",task_e[j] * task_e[j]);
			else
			fprintf(fp2, "%f\n",task_e[j] * task_e[j]);
			}
			fprintf(fp2, "\n");
			}
			fclose(fp2);
			fclose(fp);
			}
			error_tc = max_e;
			*/

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
		GRRenderIf *render = GetSdk()->GetRender();

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
			for(uint i = 0; i < robot.links.size(); i++)
				robot.links[i]->DrawSnapshot(playTime, &gl, &conf);
			for(uint i = 0; i < robot.boxes.size(); i++)
				robot.boxes[i]->DrawSnapshot(playTime, &gl, &conf);
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
		glutSwapBuffers();

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
		else if(key == 'x') {
			cout << "switch to x-axis" << endl;
			activeAxis = 'x';
		}
		else if(key == 'y') {
			cout << "switch to y-axis" << endl;
			activeAxis = 'y';
		}
		else if(key == 'z') {
			cout << "switch to z-axis" << endl;
			activeAxis = 'z';
		}
		else if(key == 's'){
			cout << "generate svg image" << endl;
			CreateSVG();
		}
		else if(key == 'd'){
			cout << "Printdate" << endl;
			Printdate();
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
