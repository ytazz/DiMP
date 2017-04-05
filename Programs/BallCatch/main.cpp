#include <Springhead.h>
#include <Framework/SprFWApp.h>
#include "../DiMP1/Graph.h"
#include <time.h>

#define ARRAY_SIZE(x) (sizeof(x)/sizeof(x[0]))
#define N_b 3// N_b��class:BallCatch�ɂ��ă{�[���̌�
#define R_A 1

int n_b;//���ܖڕW�ɂ��Ă���{�[��
int r_a;//����v����s���A�[��

using namespace DiMP; 

class BallCatch : public Graph{
public:
	// springhead objects
	PHSolidIf*		base;
	PHSolidIf*		links[3];
	PHSolidIf*		target[N_b];
	PHHingeJointIf* joints[3];

	// graph nodes
	ObjectNode*		nodeBase;
	ObjectNode*		nodeLinks[3];
	HingeNode*		nodeJoints[3];
	TargetNode*		nodeTarget[N_b];

	double t_i[N_b];//t_c�����l

	// set target position
	void SetTarget(const Vec3d& pos){
		cout << "n_b=" <<(unsigned int)n_b << endl;
		target[n_b]->SetFramePosition(pos);
	}
	virtual double VelLimit(JointNode * jnt) {
		return 1.8;// rad/sec
	}

	virtual double TorqueLimit(JointNode * jnt) {
		//return FLT_MAX;
		return 15;

	}

	virtual void BallCatch::RandamT0(){
		for (int i=0 ; i<N_b ; i++ ){
			t_i[i] = rand() % m_nPlanLength * m_dt;
			if(t_i[i] == 0.0)
				t_i[i] += 1.0e-2;
			if(t_i[i] == (m_nPlanLength-1) * m_dt)
				t_i[i] -= 1.0e-2;
		}
	}


	//virtual bool  PosMask(ObjectNode* obj, size_t t, int k = -1){
	//	//if(obj == nodeLinks[2])
	//	//	return true;
	//	return false;
	//}	
	//virtual Vec3d DesiredPos(ObjectNode* obj, size_t t){
	//	//if(obj == nodeLinks[2])
	//	//	return target[n_b]->GetFramePosition();
	//	return Vec3d();
	//}

	virtual bool Link(PHSdkIf* phSdk, PHSceneIf* phScene){
		Graph::Link(phSdk, phScene);

		////////////////////////////////////////////////////////////////
		// springhead��̃A�[���̍\�z�@�@�@�@

		// box�`����쐬
		CDBoxDesc bd,endbd,basebd1,basebd2;
		bd.boxsize = 0.1*Vec3d(0.5, 6.0, 0.5);
		endbd.boxsize = 0.1*Vec3d(0.5, 1.5, 0.5);
		basebd1.boxsize = 0.1*Vec3d(2.0, 0.5, 2.0);
		basebd2.boxsize = 0.1*Vec3d(1.0, 3.0, 1.0);
		CDShapeIf* box = phSdk->CreateShape(bd);
		CDShapeIf* endbox = phSdk->CreateShape(endbd);
		CDShapeIf* basebox1 = phSdk->CreateShape(basebd1);
		CDShapeIf* basebox2 = phSdk->CreateShape(basebd2);
		CDRoundConeDesc RClink;//�֐߂̃J�v�Z��
        RClink.radius[0] = 0.1*0.27;
		RClink.radius[1] = 0.1*0.27;
		RClink.length = 0.1 * 0.49;
		CDShapeIf* linkring;
		CDSphereDesc BBDesc;//base�֐߂̃{�[��
		BBDesc.radius = 0.1 * 0.5;
		CDShapeIf* baseball;
		// �G���h�G�t�F�N�^�`����쐬
		CDRoundConeDesc RC1,RC2;
		RC1.radius[0] = 0.1*0.1;
		RC1.radius[1] = 0.1*0.1;
		RC1.length = 0.1 * 2.5;
        RC2.radius[0] = 0.1*0.3;
		RC2.radius[1] = 0.1*0.3;
		RC2.length = 0.1 * 1.2;
		
		CDShapeIf* top1[2];
		CDShapeIf* top2;

		///////////////// �ڕW�ʒu����������
		// ball�`����쐬
		CDSphereDesc descSphere;
		descSphere.radius = 0.12;
		CDShapeIf* ball[N_b];
		double ballPos[N_b][3];
		Vec3d ballv0[N_b];

		////////////////settig2
		ballv0[0]=Vec3d(-0.2,0.5,0.1);//ball0�̏����x
		ballPos[0][0]=0.8;//ball0�̏����ʒux
		ballPos[0][1]=0.6;//ball0�̏����ʒuy
		ballPos[0][2]=0.1;//ball0�̏����ʒuz

		ballv0[1]=Vec3d(0.1,-0.1,-0.4);//ball1�̏����x
		ballPos[1][0]=-0.6;//ball1�̏����ʒux
		ballPos[1][1]=1.1;//ball1�̏����ʒuy
		ballPos[1][2]=2.0;//ball1�̏����ʒuz

		ballv0[2]=Vec3d(-0.6,0.1,0.0);//ball1�̏����x
		ballPos[2][0]=-0.5;//ball1�̏����ʒux
		ballPos[2][1]=1.35;//ball1�̏����ʒuy
		ballPos[2][2]=0.1;//ball1�̏����ʒuz

		//�^�[�Q�b�g�̌`��ƈʒu�C����
		//////////////////////////////////////
		PHSolidDesc solidDesctrg;
		solidDesctrg.mass = 0.0000001;
		for (int i=0 ; i<N_b ; i++){ 
			ball[i]= phSdk->CreateShape(descSphere);//ball�̌`��𐶐�
			target[i] = phScene->CreateSolid(solidDesctrg);
			target[i]->SetDynamical(true);
			target[i]->AddShape(ball[n_b]);
			target[i]->SetFramePosition(*(Vec3d*)ballPos[i]);
			target[i]->SetVelocity(ballv0[i]);
		}

		// �A�[���̃x�[�X���̍���
		PHSolidDesc solidDesc;
		solidDesc.mass = 0.01;
		baseball =phSdk->CreateShape(BBDesc);
		solidDesc.inertia = 0.001 * Matrix3d::Unit();
		base = phScene->CreateSolid(solidDesc);
		base->AddShape(basebox1);
		base->AddShape(basebox2);
		base->AddShape(baseball);
		base->SetDynamical(false);		// �O�͂̉e�����󂯂Ȃ��悤�ɂ���
		Posed basepose =base->GetShapePose(1);
		Posed baseballpose =base->GetShapePose(2);
		basepose.py = 0.1*1.5;
		baseballpose.py += 0.1* 3.0;
		base->SetShapePose(1,basepose);
		base->SetShapePose(2,baseballpose);
		// �A�[���̃����N���̍���
		linkring =  phSdk->CreateShape(RClink);
		Posed lpose;
		for(int i = 0; i < 2; i++){
			links[i] = phScene->CreateSolid(solidDesc);
			links[i]->AddShape(box);
			links[i]->AddShape(linkring);
			lpose =links[i]->GetShapePose(1);
			lpose.py += 0.1 * 3.0;
		    links[i]->SetShapePose(1,lpose);
		}

		//�ȉ��C�G���h�G�t�F�N�^
		for (int i = 0 ; i < 2 ; i++ )
			top1[i] = phSdk->CreateShape(RC1);
		top2 = phSdk->CreateShape(RC2);
		links[2] = phScene->CreateSolid(solidDesc);
		links[2]->AddShape(endbox);
		for (int i = 0 ; i < 2 ; i++ )
			links[2]->AddShape(top1[i]);
		links[2]->AddShape(top2);

		links[0]->SetFramePosition(0.1*Vec3d(0.0,6.0,0.0));
		links[1]->SetFramePosition(0.1*Vec3d(0.0,12.0,0.0));
		links[2]->SetFramePosition(0.1*Vec3d(0.0,18.0,0.0));
		links[2]->SetDynamical(true);
		
		// �G���h�G�t�F�N�^�̍��̂̒���
		Posed toppose[4];//0:���@2:�́@3:�n��
		for (int i = 0 ; i < 3  ; i++ )
			toppose[i] =links[2]->GetShapePose(i);
		//���
		//��
		toppose[0].py -= 0.1 * 2.25;
		links[2]->SetShapePose(0,toppose[0]);
		//��
		toppose[1].pz += 0.1 * 0.5 ;
		toppose[2].pz -= 0.1 * 0.5 ;
		for (int i = 1 ; i < 3 ; i++){
			toppose[i].py -= 0.1 * 0.2;
			toppose[i].Ori() = Quaterniond::Rot(Rad(-90), 'x');
		links[2]->SetShapePose(i,toppose[i]);
		}
		//�킽��
		toppose[3].py -= 0.1 * 1.5;
		links[2]->SetShapePose(3,toppose[3]);
		// �A�[���̊֐�
		const double K = 895000, D = 5000;//1000,100
		PHHingeJointDesc descHinge;

		descHinge.poseSocket.Pos() = 0.1*Vec3d(0.0,  3.0, 0.0);	// �x�[�X�ɋ߂������N�Ɋ֐߂����t����ʒu
		descHinge.posePlug.Pos()   = 0.1*Vec3d(0.0, -3.0, 0.0);	// ��[�ɋ߂������N�Ɋ֐߂����t����ʒu
		for(int i = 1; i < 3; i++){
			joints[i] = (PHHingeJointIf*)phScene->CreateJoint(links[i-1], links[i], descHinge);
			joints[i]->SetSpring(K);	// �ʒu����p�̃o�l�ƃ_���s���O
			joints[i]->SetDamper(D);
		}
		descHinge.posePlug.Pos() = 0.1*Vec3d(0.0, -3.0, 0.0);	// ��[�Ɋ֐߂����t����ʒu

		descHinge.poseSocket.Ori() = Quaterniond::Rot(Rad(90), 'x');
		descHinge.posePlug.Ori() = Quaterniond::Rot(Rad(90), 'x');
		joints[0] = (PHHingeJointIf*)phScene->CreateJoint(base, links[0], descHinge);
		joints[0]->SetSpring(K);	// �ʒu����p�̃o�l�ƃ_���s���O
		joints[0]->SetDamper(D);

		//descHinge.poseSocket.Pos() = 0.1*Vec3d(0.0,  3.0, 0.0);	// ��[�ɋ߂������N�Ɋ֐߂����t����ʒu
		//descHinge.posePlug.Pos()   = 0.1*Vec3d(0.0, -3.0, 0.0);	// �x�[�X�ɋ߂������N�Ɋ֐߂����t����ʒu
		//for(int i = 1; i < 3; i++){
		//	joints[i] = (PHHingeJointIf*)phScene->CreateJoint(links[i-1], links[i], descHinge);
		//	joints[i]->SetSpring(K);	// �ʒu����p�̃o�l�ƃ_���s���O
		//	joints[i]->SetDamper(D);
		//}

		// �ڐG�͌v�Z���I�t (�\�z������ɌĂԕK�v������)
		phScene->SetContactMode(PHSceneDesc::MODE_NONE);
		srand((unsigned) time(NULL));

		//��ŏ����l����͂���
		//for (int i = 0 ; i < N_b ; i++){
		//	cout << "t_i[" << i << "]" << endl;
		//	cin >> t_i[i];
		//	if(t_i[i] == 0.0)
		//		t_i[i] += 1.0e-2;
		//	if(t_i[i] >= (m_nPlanLength-1) * m_dt)
		//		t_i[i] = (m_nPlanLength-1) * m_dt - 1.0e-2;	
		//}
		RandamT0();

		nodeBase = CreateObjectNode(base);
		for(int i = 0; i < 3; i++)
			nodeLinks[i] = CreateObjectNode(links[i]);

		nodeJoints[0] = CreateHingeNode(joints[0], nodeBase, nodeLinks[0]);
		for(int i = 1; i < 3; i++)
			nodeJoints[i] = (HingeNode*)CreateHingeNode(joints[i], nodeLinks[i-1], nodeLinks[i]);
		for (int i = 0 ; i < N_b ; i++){
			nodeTarget[i] = CreateTargetNode(target[i],nodeLinks[2]);
			nodeTarget[i]->t_c=t_i[i];//�K���ȏ����l
			cout<< "initial t_c=" << nodeTarget[i]->t_c <<endl;
			nodeTarget[i]->UpdateT0();
		}
		return true;
	}
	//�G������̕`��
	virtual void TauchBall(PHSdkIf* phSdk, PHSceneIf* phScene,int i){
		CDSphereDesc descSphere;
		descSphere.radius = 0.04;
		CDShapeIf* ball[N_b];
		Vec3d endp = nodeTarget[i]->m_v0*nodeTarget[i]->t_c + nodeTarget[i]->p0;
		ball[i]= phSdk->CreateShape(descSphere);//ball�̌`��𐶐�
		target[i] = phScene->CreateSolid();
		target[i]->SetDynamical(true);
		target[i]->ClearShape();
		target[i]->AddShape(ball[i]);
		target[i]->SetFramePosition(endp);
		target[i]->SetVelocity(Vec3d(0.0, 0.0, 0.0));
	}
};

using namespace Spr;

class MyApp : public FWApp{
public:
	//SceneBuilder	m_builder;
	FWSceneIf*		m_scene;
	BallCatch		m_robot[R_A];
	bool m_bRunning;
	bool m_renderMode;
	int UpdateT0;
	char m_activeAxis[N_b];
	clock_t start,end;//���ԑ�����
	//Vec3d m_targetPos[N_b];//�ڕW�ʒu->


	MyApp() {
	}

	void ResultCVS(double timer , int UpdateT0){
		if (UpdateT0 != 0)
			return;
		else{
			FILE *fp;
			char *fname = "result.csv";
			fp = fopen( fname, "a" );
			if( fp == NULL ){
				cout << "�ʂ�ʂ�ʂ��" << endl;
				return ;
			}
			else{
				fprintf( fp, "%s" ,"-----------------------------------------------------------------------------------------------------\n" );
				for (int i_r=0 ; i_r<R_A ; i_r++){
					fprintf( fp, "%s%f,%s%d,%s%d\n" , "timer(sec)=", timer ,"r_a=" , i_r, "roop=" , m_robot[0].nodeTarget[0]->m_udtc);
					for (int i_t=0 ; i_t<N_b ; i_t++){
						fprintf( fp, "%s%d,%s%f,%f,%f%s,%s%f,%f,%f%s,%s%f,%s%f\n" , "n_b=", i_t , "initial p0=(" ,m_robot[i_r].nodeTarget[i_t]->p0, ")" ,"initial v0=(" ,m_robot[i_r].nodeTarget[i_t]->m_v0, ")" ,"initial t_c=" ,m_robot[i_r].t_i[i_t], "end t_c=", m_robot[i_r].nodeTarget[i_t]->t_c );
					}
				}
				fprintf( fp, "\n" );
				fclose( fp );//�t�@�C�������̂��厖
				cout << "���̎����A�T�[�r�X�T�[�r�X��" << endl;
			}
		}
	}

	void ChangeTarget() {
		cout << "current pos: "<< "r_a=" << r_a << "n_b=" << n_b << "," << m_robot[r_a].nodeTarget[n_b]->p0 << endl;
		m_robot[r_a].SetTarget(m_robot[r_a].nodeTarget[n_b]->p0);
	}

	virtual void BuildObject(){
		// Springhead�V�[���̍\�z
		m_scene = GetSdk()->GetScene();
		m_scene->GetPHScene()->SetTimeStep(0.01);		// �ϕ���50ms
		m_scene->GetPHScene()->SetNumIteration(30);		// �S���͌v�Z�̔�����
		m_scene->GetPHScene()->SetGravity(0.0 * Vec3d(0.0, 0.0, 0.0));	// �d�͉����x

		/// ���{�b�g�̍\�z
		for (int i=0 ; i<R_A ; i++){
			m_robot[i].Link(GetSdk()->GetPHSdk(), m_scene->GetPHScene());
			m_robot[i].m_nPlanLength = 12;		// ����v��̗\���X�e�b�v��
			m_robot[i].m_dt = 0.3;	// ���Ԃ̗��U����
			m_robot[i].Initialize();
			m_robot[i].UpdateState();
			m_robot[i].InitPlan();
		}

		for (int i=0; i<N_b; i++)
			m_activeAxis[i] = 'x';				// �ڕW���̂𓮂�������

		// �f�o�b�O�\�����[�h
		GetSdk()->SetDebugMode();

		m_renderMode = false;
		m_bRunning = false;
		UpdateT0 = 1;
	}

	virtual void Init(int argc, char* argv[]){
		SetGRAdaptee(TypeGLUT);
		GRInit(argc, argv);
		CreateSdk();
		GetSdk()->CreateScene();

		FWWinDesc windowDesc;
		windowDesc.title = "BallCatch";
		CreateWin(windowDesc);
		InitWindow();

		BuildObject();
		GetSdk()->Step();
		int timerId = CreateTimer(FWTimer::GLUT);
		SetInterval(timerId ,100);//�V�~�����[�V�������������I
	}

	// �^�C�}�R�[���o�b�N�֐��D�^�C�}�����ŌĂ΂��
	virtual void TimerFunc(int id) {
		if(UpdateT0 == 0){
			m_robot[r_a].EnablePlan(false);
			m_bRunning = !m_bRunning;
			end = clock();
			double timer = (double)(end-start)/CLOCKS_PER_SEC;
			cout << "�v�Z����" << timer << "sec" << endl;
			for (int i_r=0 ; i_r<R_A ; i_r++){
				for (int i_t=0 ; i_t<N_b ; i_t++){
					m_robot[i_r].nodeTarget[i_t]->PrintCVS(i_t);

				}

			}
			ResultCVS(timer,UpdateT0);
			UpdateT0 = 10000;
			GetSdk()->Step();
			/* ����v���1�X�e�b�v���s*/
			for (int i_r=0 ; i_r<R_A ; i_r++){
				for (int i_t=0 ; i_t<N_b ; i_t++){
					m_robot[i_r].Step();
					m_robot[i_r].nodeTarget[i_t]->PrintCVS(i_t);
					//cout<< "n_b="<< " , " <<i_t <<"t_c="<<m_robot[i_r].nodeTarget[i_t]->t_c<< endl;
					m_robot[i_r].target[i_t]->SetCenterPosition (m_robot[i_r].nodeTarget[i_t]->t_c * m_robot[i_r].nodeTarget[i_t]->m_v0 + m_robot[i_r].nodeTarget[i_t]->p0);
				}
			}
			cout << "�p�^�[���I�ԈႢ����܂���A�g�k�ł��I" << endl;
		}
		else{
			if(m_bRunning) {
				// �V�~�����[�V������1�X�e�b�v���s
				GetSdk()->Step();
				/* ����v���1�X�e�b�v���s*/
				for (int i_r=0 ; i_r<R_A ; i_r++){
					for (int i_t=0 ; i_t<N_b ; i_t++){
						m_robot[i_r].Step();
						m_robot[i_r].nodeTarget[i_t]->PrintCVS(i_t);
						if (!m_robot[i_r].m_bExecute)
							m_robot[i_r].target[i_t]->SetVelocity(Vec3d(0.0,0.0,0.0));
						if (m_robot[i_r].m_bExecute){
							double endtc = m_robot[i_r].nodeTarget[i_t]->t_c;
							double ExTime = m_robot[i_r].m_time - m_robot[i_r].m_executeAbsoluteTime;
							//m_robot[i_r].target[i_t]->SetCenterPosition (ExTime * m_robot[i_r].nodeTarget[i_t]->m_v0 + m_robot[i_r].nodeTarget[i_t]->p0);
							//if (endtc <= ExTime ){
							//	m_robot[i_r].target[i_t]->ClearShape();
							//	m_robot[i_r].TauchBall(GetSdk()->GetPHSdk(), m_scene->GetPHScene(),i_t);
							//}
						}
					}
					//�ȉ��C��鏇�Ԃ����߂邽�߂̂�D�f�t�H���g�ł̓R�����g�A�E�g  012....�̏��Ɏ��/////////////////////
					//for (int i_t=0 ; i_t<N_b -1 ; i_t++){
					//	if(m_robot[i_r].nodeTarget[i_t + 1]->t_c <= m_robot[i_r].nodeTarget[i_t]->t_c){
					//		m_robot[i_r].nodeTarget[i_t + 1]->m_tcupdate = false;
					//		m_robot[i_r].nodeTarget[i_t + 1]->t_c = m_robot[i_r].nodeTarget[i_t]->t_c ;
					//	}
					//	else
					//		m_robot[i_r].nodeTarget[i_t + 1]->m_tcupdate = true;
					//}
					////////////////////////////////////////////////////////////////////////////////////////////////////////
				}
				if (UpdateT0 != 10000)
					UpdateT0 = 0;
				for (int i_r=0 ; i_r<R_A ; i_r++){
					for (int i_t=0 ; i_t<N_b ; i_t++){
						UpdateT0 += (int)m_robot[i_r].nodeTarget[i_t]->m_targetupdate;//�S�̂̃v�����j���O
					}
				}
				if (UpdateT0 != 10000)
				cout << "UpdateT0=" << UpdateT0 << endl; 
			}
		}
		// �ĕ`��v��
		PostRedisplay();

	}

	// �`��֐��D�`��v���������Ƃ��ɌĂ΂��
	virtual void Display() {
		GRDebugRenderIf *render = DCAST(GRDebugRenderIf, GetSdk()->GetRender());
		render-> SetClearColor(Vec4f(0.0,0.0,0.0,0.1));//�V�~�����[�V�����̔w�i�F
		render->ClearBuffer();
		render->BeginScene();

		// �`��ݒ�
		glEnable(GL_LIGHTING);
		glEnable(GL_BLEND);
		glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
		glEnable(GL_DEPTH_TEST);

		// ����
		GRLightDesc ld;
		ld.diffuse = Vec4f(1, 1, 1, 1) * 0.9f;
		ld.specular = Vec4f(1, 1, 1, 1) * 0.2f;
		ld.ambient = Vec4f(1, 1, 1, 1) * 0.1f;

		ld.position = Vec4f(1, 1, 1, 0);
		render->PushLight(ld);
		ld.position = Vec4f(-1, 1, -1, 0);
		render->PushLight(ld);

		// Springhead�̕`�惂�[�h�ݒ�
		render->SetRenderMode(true, true);
		//render->EnableRenderAxis();
		//render->EnableRenderContact();

		// �J����
		GRCameraDesc camera = render->GetCamera();
		camera.front = 0.3;
		render->SetCamera(camera);

		//render->SetViewMatrix(GetCameraInfo().view.inv());
		//GetSdk()->GetScene()->Draw(render);

		// Springhead�V�[���̕`��
		render->DrawScene(GetSdk()->GetScene()->GetPHScene());

		// ����v��̕`��
		for (int i=0 ; i<R_A ; i++){
			m_robot[i].Draw(render);
		}
		render->PopLight();
		render->PopLight();
		render->EndScene();

		glutSwapBuffers();

	}

	virtual void Keyboard(int key, int x, int y) {
		if(key == ' ') {
			m_bRunning = !m_bRunning;
			start = clock();
			cout << (m_bRunning ? "run" : "stop") << endl;
		}
		else if(key == 'p') {
			m_robot[r_a].EnablePlan(!m_robot[r_a].m_bPlan);
			cout << (m_robot[r_a].m_bPlan ? "start" : "stop") << " planning" << endl;
		}
		else if(key == 'e') {
			m_robot[r_a].EnableExecute(!m_robot[r_a].m_bExecute);
			for (int i_t=0 ; i_t < N_b ; i_t++){
				m_robot[r_a].target[i_t]->SetCenterPosition (m_robot[r_a].nodeTarget[i_t]->p0);
			}
			cout << (m_robot[r_a].m_bExecute ? "start" : "stop") << " executing the plan" << endl;
		}
		else if(key == 'i') {
			cout << "init plan" << endl;
			m_robot[r_a].InitPlan();
		}
		else if(key == 'r') {
			BuildObject();
			/*m_robot[r_a].Reset();
			for (int i_t=0 ; i_t < N_b ; i_t++)
				m_robot[r_a].target[i_t]->SetCenterPosition (m_robot[r_a].nodeTarget[i_t]->p0);
			cout << "reset position" << endl;*/
		}
		else if(key == '-') {
			if(n_b>0){
				cout << "n_b--" << endl;
				n_b -= 1;
				cout << "n_b=" << n_b << endl;
			}
			else{
				cout << "non!" << endl;
			}
		}
		else if(key == '+') {
			if(n_b<N_b - 1){
				cout << "n_b++" << endl;
				n_b += 1;
				cout << "n_b=" << n_b << endl;
			}
			else{
				cout << "non!" << endl;
			}
		}
		else if(key == ';') {
			if(r_a<R_A - 1){
				cout << "r_a++" << endl;
				r_a += 1;
				cout << "r_a=" << r_a << endl;
			}
			else{
				cout << "non!" << endl;
			}
		}
		else if(key == '^') {
			if(r_a>0){
				cout << "r_a--" << endl;
				r_a -= 1;
				cout << "r_a=" << r_a << endl;
			}
			else{
				cout << "non!" << endl;
			}
		}
		else if(key == 'x') {
			cout << "switch to x-axis" << endl;
			m_activeAxis[n_b] = 'x';
		}
		else if(key == 'y') {
			cout << "switch to y-axis" << endl;
			m_activeAxis[n_b] = 'y';
		}

		else if(key == 'z') {
			cout << "switch to z-axis" << endl;
			m_activeAxis[n_b] = 'z';
		}

		else if(key == 'm') {
			cout << "increase value" << endl;
			m_robot[r_a].nodeTarget[n_b]->p0[m_activeAxis[n_b] - 'x'] += 0.5;
			ChangeTarget();
		}
		else if(key == 'n') {
			cout << "decrease value" << endl;
			m_robot[r_a].nodeTarget[n_b]->p0[m_activeAxis[n_b] - 'x'] -= 0.5;
			ChangeTarget();
		}
		else if(key == 'q') {
			cout << "n_b=" <<(unsigned int)n_b << endl;
		}
		else if(key == 'u') {
			for (int i_r=0 ; i_r<R_A ; i_r++){
				for (int i_t=0 ; i_t<N_b ; i_t++){
					m_robot[i_r].nodeTarget[i_t]->m_tcupdateOn = !m_robot[i_r].nodeTarget[i_t]->m_tcupdateOn;
				}
			}
			cout << (m_robot[0].nodeTarget[0]->m_tcupdateOn ? "Tc_Update" : "Tc_NotUpdate") << endl; //�֋X��[0][0]�ŕ\��
		}
		else if(key == 'w') {
			cout << "r_a=" <<(unsigned int)r_a << endl;
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


/*

////////////////settig0
		ballv0[0]=Vec3d(0.0,0.0,0.0);//ball0�̏����x
		ballPos[0][0]=0.4;//ball0�̏����ʒux
		ballPos[0][1]=1.6;//ball0�̏����ʒuy
		ballPos[0][2]=-0.25;//ball0�̏����ʒuz

		ballv0[1]=Vec3d(0.0,-0.0,-0.0);//ball1�̏����x
		ballPos[1][0]=0.4;//ball1�̏����ʒux
		ballPos[1][1]=1.2;//ball1�̏����ʒuy
		ballPos[1][2]=-0.45;//ball1�̏����ʒuz

		ballv0[2]=Vec3d(0.0,0.0,0.0);//ball1�̏����x
		ballPos[2][0]=0.3;//ball1�̏����ʒux
		ballPos[2][1]=0.7;//ball1�̏����ʒuy
		ballPos[2][2]=0.5;//ball1�̏����ʒuz

////////////////settig1
		ballv0[0]=Vec3d(-0.2,0.5,0.1);//ball0�̏����x
		ballPos[0][0]=0.8;//ball0�̏����ʒux
		ballPos[0][1]=0.6;//ball0�̏����ʒuy
		ballPos[0][2]=0.1;//ball0�̏����ʒuz

		ballv0[1]=Vec3d(0.1,-0.1,-0.4);//ball1�̏����x
		ballPos[1][0]=0.6;//ball1�̏����ʒux
		ballPos[1][1]=1.0;//ball1�̏����ʒuy
		ballPos[1][2]=-0.45;//ball1�̏����ʒuz

		ballv0[2]=Vec3d(0.0,0.0,0.0);//ball1�̏����x
		ballPos[2][0]=0.3;//ball1�̏����ʒux
		ballPos[2][1]=0.7;//ball1�̏����ʒuy
		ballPos[2][2]=0.5;//ball1�̏����ʒuz

		////////////////settig1
		ballv0[0]=Vec3d(-0.2,0.5,0.1);//ball0�̏����x
		ballPos[0][0]=0.8;//ball0�̏����ʒux
		ballPos[0][1]=0.6;//ball0�̏����ʒuy
		ballPos[0][2]=0.1;//ball0�̏����ʒuz

		ballv0[1]=Vec3d(0.1,-0.1,-0.4);//ball1�̏����x
		ballPos[1][0]=-0.6;//ball1�̏����ʒux
		ballPos[1][1]=1.0;//ball1�̏����ʒuy
		ballPos[1][2]=1.2;//ball1�̏����ʒuz

		ballv0[2]=Vec3d(-0.6,0.1,0.0);//ball1�̏����x
		ballPos[2][0]=-0.5;//ball1�̏����ʒux
		ballPos[2][1]=1.35;//ball1�̏����ʒuy
		ballPos[2][2]=0.1;//ball1�̏����ʒuz

		////////////////settig2
		ballv0[0]=Vec3d(-0.2,0.5,0.1);//ball0�̏����x
		ballPos[0][0]=0.8;//ball0�̏����ʒux
		ballPos[0][1]=0.6;//ball0�̏����ʒuy
		ballPos[0][2]=0.1;//ball0�̏����ʒuz

		ballv0[1]=Vec3d(0.1,-0.1,-0.4);//ball1�̏����x
		ballPos[1][0]=-0.6;//ball1�̏����ʒux
		ballPos[1][1]=1.1;//ball1�̏����ʒuy
		ballPos[1][2]=2.0;//ball1�̏����ʒuz

		ballv0[2]=Vec3d(-0.6,0.1,0.0);//ball1�̏����x
		ballPos[2][0]=-0.5;//ball1�̏����ʒux
		ballPos[2][1]=1.35;//ball1�̏����ʒuy
		ballPos[2][2]=0.1;//ball1�̏����ʒuz

		*/