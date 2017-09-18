#include "RobotArm.h"
#include "mymodule.h"

int main(int argc, char* argv[]){
	MyModule mod;
	if(!mod.Init(argc, argv))
		return -1;

	mod.MainLoop();
	return 0;
}

/*
#include <DiMP/DiMP.h>

#include <sbscenelocal.h>

class MyApp : public DiMP::App, public DiMP::Render::Config{
public:
	SceneLocal		scene;
	TypeDB			typedb;
	ModelContainer	models;
	Workspace		workspace;

	enum{
		ID_COLLISION,
		ID_JOINT,
		ID_HAND_AND_TARGET,
		ID_COMPATONCE,
		ID_IPOPT_WEIGHTED,
		ID_IPOPT_PHASED,
	};
	
public:
	MyApp(){
		workspace.graph = graph;
		conf            = this;
		appName		    = "RobotArm";

		AddAction(MENU_USER, ID_COLLISION      , "collision check" )->AddHotKey('c');
		AddAction(MENU_USER, ID_JOINT          , "joint error"     )->AddHotKey('j');
		AddAction(MENU_USER, ID_HAND_AND_TARGET, "hand and target" )->AddHotKey('w');
		AddAction(MENU_USER, ID_COMPATONCE     , "compute at once" )->AddHotKey('t');
		AddAction(MENU_USER, ID_IPOPT_WEIGHTED , "compute by ipopt (weighted)")->AddHotKey('W');
		AddAction(MENU_USER, ID_IPOPT_PHASED   , "compute by ipopt (phased)"  )->AddHotKey('P');
	}
	virtual ~MyApp(){}

	void CompAtOnce(real_t th){
		do{
			OnStep();
			DSTR << "dz: " << deltaNorm << endl;
		}
		while(deltaNorm > th);
		DSTR << "iter: " << iterCount << "  time: " << compTime << "  delta: " << deltaNorm << endl;
	}

	virtual void BuildScene(){
		// 
		typedb   .Register();
		scene    .Create();
		scene    .CreateObject(NamespaceProp::id, "world", &typedb);
		models   .Set(&scene);
		workspace.Set(&scene, &typedb, &models);
		workspace.Build(GetSdk());
	}

	void DrawSnapshot(GRRenderIf* render, real_t time){
		// �Đ������̏�Ԃ�DiMP����SprGR��sync
		workspace.adaptorDiMP .SetSyncTime(time);
		workspace.adaptorDiMP .SyncProperty(false, AttrCategory::State);
		workspace.adaptorSprGR.SyncProperty(true , AttrCategory::State);
		// �`��
		workspace.adaptorSprGR.Draw(render);
	}

	virtual void OnAction(int menu, int id){
		Action* act = GetAction(menu, id);
		if(menu == MENU_USER){
			// �ڍׂȏՓ˔���
			if(id == ID_COLLISION){
				workspace.CheckCollision();
			}
			if(id == ID_JOINT){
				workspace.CheckJointError();
			}
			if(id == ID_HAND_AND_TARGET){
				workspace.WriteHandAndTarget();
			}
			if(id == ID_COMPATONCE){
				CompAtOnce(0.05);
			}
			if(id == ID_IPOPT_WEIGHTED){
				//CompIpopt(DiMP2::IpoptAdaptor::Weighted);
			}
			if(id == ID_IPOPT_PHASED){
				//CompIpopt(DiMP2::IpoptAdaptor::Phased);
			}
		}
		App::OnAction(menu, id);
	}

	virtual void OnDraw(GRRenderIf* render){
		// �Đ����[�h�̏ꍇ�͍Đ������̃X�i�b�v�V���b�g��`��
		if(GetAction(MENU_ALWAYS, ID_PLAY)->GetBool()){
			DrawSnapshot(render, playTime);
		}
		// ����ȊO�͏��������Ɗe�^�X�N�̊J�n�E�I�������̃X�i�b�v�V���b�g��`��
		else{
			DrawSnapshot(render, 0.0);
			for(uint i = 0; i < workspace.task.size(); i++){
				DiMP::Task* task = workspace.task[i];
				DrawSnapshot(render, task->time->time_s->val);
				DrawSnapshot(render, task->time->time_e->val);
			}
		}
		
		// ���C�e�B���O��OFF�ɂ��ċO����`��
		render->SetLighting(false);
		graph->Draw(canvasGL, this);
		render->SetLighting(true);
	}

	virtual bool Set(GRRenderIf* render, int attr, DiMP::Node* node){
		if(attr == DiMP::Render::Item::ObjectPos){
			for(int i = 0; i < (int)workspace.robot.size(); i++){
				RobotArm* r = workspace.robot[i];
				if(node == r->hand){
					render->SetPointSize(10.0f);
					return true;
				}
			}
		}
		if(attr == DiMP::Render::Item::ObjectTrajectory){
			for(uint i = 0; i < workspace.robot.size(); i++){
				RobotArm* r = workspace.robot[i];
				if(node == r->hand){
					DiMP::Render::Config::Set(canvasGL, attr, node);
					render->SetLineWidth(4);
					return true;
				}
			}
			for(uint i = 0; i < workspace.target.size(); i++){
				DiMP::Object* t = workspace.target[i];
				if(node == t){
					DiMP::Render::Config::Set(canvasGL, attr, node);
					render->SetLineWidth(4);
					return true;
				}
			}
		}
		return false;
	}

	virtual float Scale(int attr, DiMP::Node* node){
		return 0.1f;
	}


} app;
*/

/**
 brief		���C���֐�
 param		<in/--> argc�@�@�R�}���h���C�����͂̌�
 param		<in/--> argv�@�@�R�}���h���C������
 return		0 (����I��)
 */
/*
int main(int argc, char* argv[]){
	app.Init(argc, argv);
	app.StartMainLoop();
}
*/
