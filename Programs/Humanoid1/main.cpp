#include <fstream>
#include "../src/Draw.h"
#include "../src/SceneBuilder.h"
#include "Humanoid.h"

#include <Springhead.h>
#include <Framework/SprFWApp.h>

//#include <unistd.h>
//#include <sys/stat.h>
//#include <sys/types.h>

//#include "../src/Directory.h"

using namespace Spr;

#define ARRAY_SIZE(x) (sizeof(x)/sizeof(x[0]))

class MyApp : public FWApp{
public:
	SceneBuilder	m_builder;
	Humanoid		m_robot;

	bool m_bRunning;
	bool m_renderMode;
	char m_activeAxis;
	Vec3d m_targetPos;

	MyApp() {

	}

	virtual void BuildObject(){
		/// load scenegraph from file and build Springhead scene
		/// file is editted and output using Blender.
		m_builder.Load(GetSdk(), "misz/johanshin2.x");
		///m_builder.Load(GetSdk(), "misz/johanshinbackup.x");

		PHSceneIf* scene = GetSdk()->GetScene()->GetPHScene();
		scene->SetGravity(1.0 * Vec3d(0.0, -0.0, 0.0));
		scene->SetContactMode(PHSceneDesc::MODE_NONE);

		//
		// link the planner with the Springhead scene
		m_robot.m_nPlanLength =10;
		m_robot.m_dt = 0.3;
		m_robot.Link(GetSdk()->GetPHSdk(), scene);

		scene->SetTimeStep(0.01);
		scene->SetNumIteration(10);

		///// here we set gravity.
		//
		GetSdk()->Step();

		m_robot.Initialize();
		m_robot.m_body.m_node->SetDynamical(true);
		//m_robot.m_chest.Enable(true);
		//m_robot.m_body.m_fix->Enable(true);
		//m_robot.m_arm[0].Enable(true);
		m_robot.UpdateState();
		m_robot.InitPlan();

		m_activeAxis = 'x';

		GetSdk()->SetDebugMode(false);

		m_renderMode = false;
		m_bRunning = false;
		m_robot.EnablePlan(true);
	}

	virtual void Init(int argc, char* argv[]){
		SetGRAdaptee(TypeGLUT);
		GRInit(argc, argv);
		CreateSdk();
		GetSdk()->Clear();
		GetSdk()->CreateScene(PHSceneDesc(), GRSceneDesc());
		GetSdk()->GetScene()->GetPHScene()->SetTimeStep(0.01);

		FWWinDesc windowDesc;
		windowDesc.title = "Humanoid";
		CreateWin(windowDesc);
		InitWindow();

		BuildObject();

		int timerId = CreateTimer(FWTimer::GLUT);
	}

	virtual	void Printf_Results(){
		FILE *fp;
		char *fname = "result.csv";
		fp = fopen( fname, "a" );
		if( fp == NULL ){
			cout << "file is not found" << endl;
			return ;
		}
		else{
			fprintf( fp, "%s" ,"-----------------------------------------------------------------------------------------------------\n" );

			fprintf( fp, "%s%d\n" ,  "roop=" , m_robot.m_target[0].m_trgNodes->m_udtc);
			for (int i=0 ; i<2 ; i++){
				fprintf( fp, "%s%f,%f,%f%s,%s%f,%f,%f%s,%s%f,%s%f\n" , 
					"initial p0=(" ,m_robot.m_target[i].m_trgPos0, ")" ,"initial v0=(" ,m_robot.m_target[i].m_trgVel0, ")" ,"initial t_c=" ,m_robot.m_target[i].t_0, "end t_c=", m_robot.m_target[i].m_trgNodes->t_c );
			}

			fprintf( fp, "\n" );
			fclose( fp );
			cout << "result is plinted" << endl;
		}
	}

//////////////////
	virtual void TimerFunc(int id) {
		if(m_bRunning) {
			
			m_robot.Step();
			//planning
			int planning_target = 0;//更新中のターゲットノードの数
			if(m_robot.m_bPlan == true){
				for (int i = 0; i < 2;i++){
					m_robot.m_target[i].m_trgNodes->PrintCVS(i);
					planning_target += m_robot.m_target[i].m_trgNodes->m_targetupdate;
				}
				if(planning_target == 0){
					m_bRunning = false;
					m_robot.m_bPlan = false;
					Printf_Results();
				}

			}
			// do simulation
			if(m_robot.m_bExecute == true){
				GetSdk()->Step();
				m_bRunning = !m_bRunning;
			}
		}
		PostRedisplay();
				
	}

	virtual void Display() {
		GRDebugRenderIf *render =
			DCAST(GRDebugRenderIf, GetSdk()->GetRender());

		render-> SetClearColor(Vec4f(1.0,1.0,1.0,1.0));//シミュレーションの背景色
		render->ClearBuffer();
		render->BeginScene();

		glEnable(GL_LIGHTING);
		glEnable(GL_BLEND);
		glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
		glEnable(GL_DEPTH_TEST);

		GRLightDesc ld;
		ld.diffuse = Vec4f(1, 1, 1, 1) * 0.3f;
		ld.specular = Vec4f(1, 1, 1, 1) * 0.2f;
		ld.ambient = Vec4f(1, 1, 1, 1) * 0.1f;

		ld.position = Vec4f(1, 1, 1, 0);
		render->PushLight(ld);
		ld.position = Vec4f(-1, 1, -1, 0);
		render->PushLight(ld);

		render->SetRenderMode(true , false);//フレーム表示か剛体表示か
		render->EnableRenderForce(false);
		render->EnableRenderAxis(false);
		render->EnableRenderContact(false);

		GRCameraDesc camera = render->GetCamera();
		camera.front = 0.3;
		render->SetCamera(camera);

		render->SetViewMatrix(cameraInfo.view.inv());
		GetSdk()->Draw();
		//GetSdk()->GetScene()->Draw(render);
		//render->DrawScene(GetSdk()->GetScene()->GetPHScene());
		m_robot.Draw(render);

		render->PopLight();
		render->PopLight();
		render->EndScene();
		glutSwapBuffers();

	}

	/*char m_cwd[256];
	void PushDir(){
	getcwd(m_cwd, 256);
	}

	void PopDir(){
	chdir(m_cwd);
	} */

	virtual void Keyboard(int key, int x, int y) {
		if(key == ' ') {
			m_bRunning = !m_bRunning;
			cout << (m_bRunning ? "run" : "stop") << endl;
		}
		else if(key == 'f') {
			m_robot.m_bOneSided = !m_robot.m_bOneSided;
			if(m_robot.m_bOneSided)
				cout << "one-sided mode" << endl;
			else
				cout << "normal mode" << endl;
		}
		else if(key == 'p') {
			m_robot.m_bPlan = !m_robot.m_bPlan;
			m_robot.EnablePlan(m_robot.m_bPlan);
			cout << (m_robot.
				m_bPlan ? "start" : "stop") << " planning" << endl;
		}
		else if(key == 'e') {
			m_robot.m_bExecute = !m_robot.m_bExecute;
			m_robot.EnableExecute(m_robot.m_bExecute);
			cout << (m_robot.
				m_bExecute ? "start" : "stop") << " executing the plan"
				<< endl;		
			for (int i = 0 ; i < 2 ; i++){
				m_robot.m_target[i].sld_trg->SetVelocity(m_robot.m_target[i].m_trgVel0);
			}
		}
		else if(key == 't') {
			m_robot.SwitchReplay(!m_robot.m_bReplay);
			cout << (m_robot.
				m_bReplay ? "start" : "stop") << " replay" << endl;
		}
		else if(key == 'i') {
			cout << "init plan" << endl;
			m_robot.InitPlan();
		}
		else if(key == 'x') {
			cout << "switch to x-axis" << endl;
			m_activeAxis = 'x';
		}
		else if(key == 'y') {
			cout << "switch to y-axis" << endl;
			m_activeAxis = 'y';
		}
		else if(key == 'z') {
			cout << "switch to z-axis" << endl;
			m_activeAxis = 'z';
		}
		else if(key == 'n') {
			cout << "increase value" << endl;
			m_targetPos[m_activeAxis - 'x'] += 0.05;
		}
		else if(key == 'm') {
			cout << "decrease value" << endl;
			m_targetPos[m_activeAxis - 'x'] -= 0.05;
		}
		else if(key == 'w') {
		
		}
		else if(key == 'r') {
		
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
