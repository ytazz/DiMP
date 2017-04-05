#include <Springhead.h>
#include <Framework/SprFWApp.h>

#include "Humanoid2.h"

using namespace Spr;

class MyApp : public FWApp{
public:
	FWSceneIf*		scene;
	Humanoid		robot;
	Planner			planner;

	bool			running;
	bool			renderMode;
	char			activeAxis;
	Vec3d			targetPos;

	ofstream		errorlog;

	MyApp(){}
	~MyApp(){}

	void ChangeTarget() {
		cout << "current pos: " << targetPos << endl;
		robot.SetTarget(targetPos);
	}

	void CreateJacobianBitmap(){
		Solver* solver = &planner.graph.solver;
		uint nc = solver->cons.size();

		vector<double> mat(nc*nc);

		solver->CreateCoefficientMatrix(false, &mat[0]);
		double largest = 0.0;
		for(uint i = 0; i < mat.size(); i++)
			largest = max(largest, mat[i]);

		Bitmap bitmap;
		bitmap.CreateBitmap(nc, nc, 24);
		Scan scan;
		scan.SetBitmap(&bitmap);

		for(uint x = 0; x < nc; x++)for(uint y = 0; y < nc; y++){
			scan.Seek(Scan::SK_TOPLEFT, x, y);
			int val = (int)(255.0 * (1.0 - mat[nc * y + x] / largest));
			val = min(max(0, val), 255);
			scan.SetColor(RGBTriple(val, val, val));
		}

		Load::SaveBMP("mat.bmp", &bitmap);
	}

	void CreateSVG(){
		FILE* file = fopen("view.svg", "w");
		DrawContextSVG draw(file);

		GRRenderIf* render = GetSdk()->GetRender();
		GRCameraDesc camera = render->GetCamera();
		Vec2f sz = render->GetViewportSize();
		
		draw.width  = sz.x;
		draw.height	= sz.y;
		draw.camsize = camera.size;

		draw.affProj = Affinef::ProjectionD3D(Vec3f(camera.center.x, camera.center.y, camera.front), 
			camera.size, camera.front, camera.back);
		draw.affView = GetCameraInfo().view.inv();
		
		draw.Header();
		planner.graph.Draw(&draw, &planner.conf);
		draw.Footer();

		fclose(file);
	}

	virtual void BuildObject(){
		// Springhead�V�[���̍\�z
		scene = GetSdk()->GetScene();
		scene->GetPHScene()->SetTimeStep(0.05);		// �ϕ���50ms
		scene->GetPHScene()->SetNumIteration(15);		// �S���͌v�Z�̔�����
		scene->GetPHScene()->SetGravity(0.0 * Vec3d(0.0, -9.8, 0.0));	// �d�͉����x
		
		/// ���{�b�g�̍\�z
		robot.Build(GetSdk()->GetPHSdk(), scene->GetPHScene());
		planner.robot = &robot;
		planner.Build();

		/// Springhead��1�X�e�b�v���s���֐ߌ덷���Ȃ���
		GetSdk()->Step();

		/// ������Ԃ����[�h
		planner.LoadState();

		activeAxis = 'x';				// �ڕW���̂𓮂�������

		// �f�o�b�O�\�����[�h
		GetSdk()->SetDebugMode();

		renderMode = false;
		running = false;
	}

	virtual void Init(int argc, char* argv[]){
		SetGRAdaptee(TypeGLUT);
		GRInit(argc, argv);
		CreateSdk();
		GetSdk()->CreateScene();

		FWWinDesc windowDesc;
		windowDesc.title = "RobotArm";
		CreateWin(windowDesc);
		InitWindow();

		BuildObject();

		errorlog.open("error.csv");

		EnableIdleFunc(false);

		int timerId = CreateTimer(FWTimer::GLUT);
	}

	// �^�C�}�R�[���o�b�N�֐��D�^�C�}�����ŌĂ΂��
	virtual void TimerFunc(int id) {
		if(running) {
			// �V�~�����[�V������1�X�e�b�v���s
			//GetSdk()->Step();

			// ����v���1�X�e�b�v���s
			//planner.LoadState();
			planner.Step();

			for(int i = 0 ; i < robot.balls.size() ; i++){
				if(i != robot.balls.size()-1)
					fprintf (fp, "%f,%f,,",robot.catching[i]->time_c->val,robot.catching[i]->catkey->con_timing->e);
				else
					fprintf (fp, "%f,%f\n",robot.catching[i]->time_c->val,robot.catching[i]->catkey->con_timing->e);
				cout << "i = " << i << ", time_c = " << robot.catching[i]->time_c->val << endl;
			}

			// �S���덷���O
			for(uint i = 0; i < nConstraintTypes; i++){
				errorlog << planner.graph.GetError(i, NULL, -1, false);		///< max
				if(i != nConstraintTypes - 1)
					errorlog << ", ";
			}
			errorlog << endl;

			//running = false;
		}
		// �ĕ`��v��
		PostRedisplay();
	}

	// �`��֐��D�`��v���������Ƃ��ɌĂ΂��
	virtual void Display() {
		GRDebugRenderIf *render = DCAST(GRDebugRenderIf, GetSdk()->GetRender());

		render->SetClearColor(Vec4f(1.0f, 1.0f, 1.0f, 1.0f));
		render->ClearBuffer();
		render->BeginScene();

		// �`��ݒ�
		glEnable(GL_LIGHTING);
		glEnable(GL_BLEND);
		glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
		glEnable(GL_DEPTH_TEST);

		// ����
		GRLightDesc ld;
		ld.diffuse = Vec4f(1, 1, 1, 1) * 0.3f;
		ld.specular = Vec4f(1, 1, 1, 1) * 0.2f;
		ld.ambient = Vec4f(1, 1, 1, 1) * 0.1f;

		ld.position = Vec4f(1, 1, 1, 0);
		render->PushLight(ld);
		ld.position = Vec4f(-1, 1, -1, 0);
		render->PushLight(ld);

		// Springhead�̕`�惂�[�h�ݒ�
		render->SetRenderMode(false, true);
		//render->EnableRenderAxis();
		//render->EnableRenderContact();

		// �J����
		GRCameraDesc camera = render->GetCamera();
		camera.front = 0.3;
		render->SetCamera(camera);

		render->SetViewMatrix(GetCameraInfo().view.inv());
		
		// draw springhead scene
		//GetSdk()->Draw();

		// Springhead�V�[���̕`��
		//render->DrawScene(GetSdk()->GetScene()->GetPHScene());

		DrawContextGL gl;
		// ����v��̕`��
		render->SetLighting(false);
		render->SetDepthTest(false);
		planner.Draw(&gl);
		render->SetLighting(true);
		render->SetDepthTest(true);
		
		render->PopLight();
		render->PopLight();
		render->EndScene();
		glutSwapBuffers();

	}

	virtual void Keyboard(int key, int x, int y) {
		if(key == ' ') {
			running = !running;
			//cout << (running ? "run" : "stop") << endl;
		}
		else if(key == 'p') {
			/*robot.EnablePlan(!robot.bPlan);
			cout << (robot.
					 bPlan ? "start" : "stop") << " planning" << endl;*/
		}
		else if(key == 'e') {
			/*robot.EnableExecute(!robot.bExecute);
			cout << (robot.bExecute ? "start" : "stop") << " executing the plan" << endl;*/
		}
		else if(key == 'i') {
			/*cout << "init plan" << endl;
			robot.InitPlan();*/
		}
		else if(key == '0') {
			cout << "reset position" << endl;
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
		else if(key == 'n') {
			cout << "increase value" << endl;
			targetPos[activeAxis - 'x'] += 0.5;
			ChangeTarget();
		}
		else if(key == 'm') {
			cout << "decrease value" << endl;
			targetPos[activeAxis - 'x'] -= 0.5;
			ChangeTarget();
		}
		else if(key == 'j'){
			cout << "generate jacobian bitmap" << endl;
			CreateJacobianBitmap();
		}
		else if(key == 's'){
			cout << "generate svg image" << endl;
			CreateSVG();
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
