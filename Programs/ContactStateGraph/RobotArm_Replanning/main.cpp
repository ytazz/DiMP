#include "../SampleApp.h"
#include "Workspace.h"
#include "RobotArm.h"

#include <fstream>
#include <iostream>
#include <time.h>
using namespace std;

#include "TP_response.h"
#include "FileMapping.h"


class MyApp : public SampleApp, public DiMP2::DrawConfig{
public:
    SceneLocal		scene;
    TypeDB			typedb;
    Workspace		workspace;
    
public:
    MyApp(){
        graph		= &workspace.graph;		
        appName		= "RobotArm";
    }
    virtual ~MyApp(){}

    virtual void BuildScene(){
        // 
        typedb.Register();
        scene.Create(&typedb);
        workspace.SetScene(&scene, &typedb);
        workspace.Build(GetSdk());
    }

    void DrawSnapshot(GRRenderIf* render, real_t time){
        // �Đ������̏�Ԃ�DiMP����SprGR��sync
        workspace.adaptorDiMP .SetSyncTime(time);
        workspace.adaptorDiMP .SyncProperty(false);
        workspace.adaptorSprGR.SyncProperty(true);
        // �`��
        workspace.adaptorSprGR.Draw(render);
    }

    virtual void OnDraw(GRRenderIf* render){
        // �Đ����[�h�̏ꍇ�͍Đ������̃X�i�b�v�V���b�g��`��
        if(GetAction(MENU_ALWAYS, ID_PLAY)->GetBool()){
            DrawSnapshot(render, playTime);
        }
        // ����ȊO�͏��������Ɗe�^�X�N�̊J�n�E�I�������̃X�i�b�v�V���b�g��`��
        else{
            DrawSnapshot(render, 0.0);
            for each (DTask* task in workspace.task){
                DrawSnapshot(render, task->time->time_s->val);
                DrawSnapshot(render, task->time->time_e->val);
            }
        }
        
        // ���C�e�B���O��OFF�ɂ��ċO����`��
        render->SetLighting(false);
        workspace.graph.Draw(render, this);
        render->SetLighting(true);
    }

    virtual bool Set(GRRenderIf* render, int attr, DiMP2::Node* node){
        if(attr == DiMP2::DrawItem::ObjectTrajectory){
            for(int i = 0; i < (int)workspace.robot.size(); i++){
                RobotArm* r = workspace.robot[i];
                if(node == r->hand){
                    DiMP2::DrawConfig::Set(render, attr, node);
                    render->SetLineWidth(4);
                    return true;
                }
            }
            for(int i = 0; i < workspace.target.size(); i++){
                DObj* t = workspace.target[i];
                if(node == t){
                    DiMP2::DrawConfig::Set(render, attr, node);
                    render->SetLineWidth(4);
                    return true;
                }
            }
        }
        return false;
    }
    virtual float Scale(int attr, DiMP2::Node* node){
        return 0.1f;
    }


} app;

/**
 brief		���C���֐�
 param		<in/--> argc�@�@�R�}���h���C�����͂̌�
 param		<in/--> argv�@�@�R�}���h���C������
 return		0 (����I��)
 */
//int main(int argc, char* argv[])
//{
//	app.Init(argc, argv);
//
//	for( unsigned i = 0;  i < 100;  i++ ) {
//		cout << i << endl;
//		app.graph->Step();
//		app.Display();
//	}
//
//	app.StartMainLoop();
//
//}

//
//int main(int argc, char* argv[]){
//	app.Init(argc, argv);
//
//	ofstream fout( "constraints.csv" );
//	fout << "iteration," << "ObjectC1T," << "ObjectC1R," << "JointC1," << "JointC2," << "JointF," << "JointRangeP," << "JointRangeV,"		\
//		    << "JointRangeF," << "JointTP," << "JointRP," << "JointTV," << "JointRV," << "ForceT," << "ForceR," << "Friction,"		\
//			<< "ContactPF," << "ContactVF," << "TimeStartRange," << "TimeEndRange," << "TimeDurationRange," << "MatchingTP," << "MatchingTV,"		\
//			<< "MatchingRP," << "MatchingRV," << "AoidP," << "AvoidV," << endl;
//
//	const int nMaxIteration = 1500;
//	int i = 1;
//	DiMP2::real_t	conErr_MatchingTP	= 0;
//	const DiMP2::real_t	Threshold_ConErr = 0.05;
//
//	cout << "planning...";
//	clock_t start = clock();
//	do {
////		cout << i << endl;
//		app.graph->Step();
//		
//		//fout << i << ','
//		//		<< app.graph->CalcError(DiMP2::ConTag::ObjectC1T,  false) << ','		\
//		//	   << app.graph->CalcError(DiMP2::ConTag::ObjectC1R,  false) << ','		\
//		//	   << app.graph->CalcError(DiMP2::ConTag::JointC1,        false) << ','		\
//		//	   << app.graph->CalcError(DiMP2::ConTag::JointC2,        false) << ','		\
//		//	   << app.graph->CalcError(DiMP2::ConTag::JointF,           false) << ','		\
//		//	   << app.graph->CalcError(DiMP2::ConTag::JointRangeP,false) << ','		\
//		//	   << app.graph->CalcError(DiMP2::ConTag::JointRangeV,false) << ','		\
//		//	   << app.graph->CalcError(DiMP2::ConTag::JointRangeF,false) << ','		\
//		//	   << app.graph->CalcError(DiMP2::ConTag::JointTP,         false) << ','		\
//		//	   << app.graph->CalcError(DiMP2::ConTag::JointRP,         false) << ','		\
//		//	   << app.graph->CalcError(DiMP2::ConTag::JointTV,         false) << ','		\
//		//	   << app.graph->CalcError(DiMP2::ConTag::JointRV,         false) << ','		\
//		//	   << app.graph->CalcError(DiMP2::ConTag::ForceT,          false) << ','		\
//		//	   << app.graph->CalcError(DiMP2::ConTag::ForceR,          false) << ','		\
//		//	   << app.graph->CalcError(DiMP2::ConTag::Friction,         false) << ','		\
//		//	   << app.graph->CalcError(DiMP2::ConTag::ContactPF,    false) << ','		\
//		//	   << app.graph->CalcError(DiMP2::ConTag::ContactVF,    false) << ','		\
//		//	   << app.graph->CalcError(DiMP2::ConTag::TimeStartRange,false) << ','		\
//		//	   << app.graph->CalcError(DiMP2::ConTag::TimeEndRange,false) << ','		\
//		//	   << app.graph->CalcError(DiMP2::ConTag::TimeDurationRange,false) << ','		\
//		//	   << app.graph->CalcError(DiMP2::ConTag::MatchingTP, false) << ','		\
//		//	   << app.graph->CalcError(DiMP2::ConTag::MatchingTV, false) << ','		\
//		//	   << app.graph->CalcError(DiMP2::ConTag::MatchingRP, false) << ','	\
//		//	   << app.graph->CalcError(DiMP2::ConTag::MatchingRV, false) << ','	\
//		//	   << app.graph->CalcError(DiMP2::ConTag::AvoidP,          false) << ','	\
//		//	   << app.graph->CalcError(DiMP2::ConTag::AvoidV,          false) << ','	\
//		//	   << endl;
//		
//		conErr_MatchingTP	= app.graph->CalcError( DiMP2::ConTag::MatchingTP, true );
//
//		i += 1;
//	} while( i <= nMaxIteration && conErr_MatchingTP > Threshold_ConErr );
//	fout.close();
//	clock_t end = clock();
//	cout << "finish!" << endl;
//
//	cout << i << " cycle, " << end-start << "[msec]" << endl;
//	app.StartMainLoop();
//
//	if( conErr_MatchingTP <= Threshold_ConErr ) {
//		cout << "Success" << endl;
//		//ofstream foutt( "trajectory.csv" );
//
//		//for( size_t n = 0; n < app.graph->NTicks(); n++ ) {
//		//	DiMP2::Tick*		tick	= app.graph->GetTick(n);
//		//	DiMP2::real_t	t		= tick->GetTime();
//		//	
//		//	for( real_t k = 0;    k < app.graph->joints.size();    k++ ) {
//
//		//	}
//		//}
//		return 0;
//	} else {
//		cout << "Fail" << endl;
//		cout << conErr_MatchingTP << endl;
//		return 1;
//	}
////	app.StartMainLoop();
//}




int main(int argc, char* argv[]){
    app.Init(argc, argv);
//    app.StartMainLoop();

    const int nMaxIteration = 1500;
    int i = 1;
    DiMP2::real_t	conErr_MatchingTP	= 0;
    const DiMP2::real_t	Threshold_ConErr = 0.05;

    clock_t start = clock();
    do {
        cout << i << endl;
        app.graph->Step();
        conErr_MatchingTP	= app.graph->CalcError( DiMP2::ConTag::MatchingTP, true );
        i += 1;
    } while( i <= nMaxIteration && conErr_MatchingTP > Threshold_ConErr );
    clock_t end = clock();

    cout << i << " cycle, " << end-start << "[msec]" << endl;
//	app.StartMainLoop();


    TP_response                               res;
    FileMapping<TP_response>    fm;
    res.response = true;
    res.cycle = i;
    res.conError = conErr_MatchingTP;
    res.calcTime = end - start;
    if( conErr_MatchingTP <= Threshold_ConErr ) {   // ����
        cout << "Success" << endl;
        res.ret = TP_response::RES::SUCCESS; 
    } else {                // ���s
        cout << "Fail" <<  conErr_MatchingTP << endl;
        res.ret = TP_response::RES::FAIL;
    }
//	app.StartMainLoop();
//    return res.ret;



// ���Q��߂̋O���v��
    app.Init( argc, argv );
//    app.StartMainLoop();


    return 0;
}
