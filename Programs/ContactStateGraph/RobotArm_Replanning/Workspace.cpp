#include "Workspace.h"
#include "RobotArm.h"

#include <boost/lexical_cast.hpp>
using namespace boost;

#include "FileMapping.h"
#include "TP_setting.h"
#include <sstream>

//------------------------------------------------------------------------------------------------
// Workspace

Workspace::Workspace(){
	sceneSelect = Reaching2D;

	numTimeSteps = 5;
	samplePeriod = 0.5;
}

void Workspace::Build(FWSdkIf* sdk){
	XML xml;
	xml.Load("data/setting.xml");
	string str;
	xml.Get(str, ".scene");

	if(str == "reaching2d")
		sceneSelect = Reaching2D;
	if(str == "reaching3d")
		sceneSelect = Reaching3D;
	if(str == "toss3d")
		sceneSelect = Toss3D;

	// xml���p�[�X
	switch(sceneSelect){
	case Reaching2D: Parse("data/scene_reaching2d.xml"); break;
	case Reaching3D: Parse("data/scene_reaching3d.xml"); break;
	case Toss3D    : Parse("data/scene_toss3d.xml"    ); break;
	}
	
	// �e�A�_�v�^�Ń~���[�����O
	adaptorDiMP.Set(&graph);
	adaptorSprGR.Set(sdk->GetGRSdk(), sdk->GetScene()->GetGRScene(), sdk->GetFISdk());
	adaptorSprPH.Set(sdk->GetPHSdk(), sdk->GetScene()->GetPHScene(), sdk->GetGRSdk(), sdk->GetScene()->GetGRScene(), sdk->GetFISdk());

	Adaptor* adaptors[] = {&adaptorDiMP, &adaptorSprGR, &adaptorSprPH};
	for each(Adaptor* a in adaptors){
		a->SetScene(scene, typedb);
		a->Mirror();
		a->SyncProperty(true);
	}

	// ���{�b�g�𒊏o
	for(int i = 0; ; i++){
		UTRef<RobotArm>	r = new RobotArm();
		if(r->Build(i, &adaptorDiMP))
			robot.push_back(r);
		else break;
	}

	for(int i = 0; ; i++) try{
		DiMP2::Object* obj = adaptorDiMP.GetObject("workspace/body_target" + lexical_cast<string>(i));
		target.push_back(obj);
	} catch(...){ break; }

	if(sceneSelect == Reaching2D){
		// ������
		graph.AddTicks(0.0, 0.5, 5.0);

		// �^�C���X���b�g
		timeslot.push_back(graph.CreateTimeSlot("timeslot0"));
		timeslot.push_back(graph.CreateTimeSlot("timeslot1"));
		
		// �^�X�N
		task.push_back(graph.CreateMatchingTask("task0", robot[0]->hand, target[0], timeslot[0]));
		task.push_back(graph.CreateMatchingTask("task1", robot[0]->hand, target[1], timeslot[1]));
		
		// T, L, M
		graph.SetScaling(1.0, 1.0, 1.0);

		// �����ϐ�������
		graph.Init();
	
		// �^�X�N���s�����ƃ^�[�Q�b�g�̋O�����Œ�
		// ���J�n�E�I��������tick�ɋ߂��Ɛ��l�I�ɂ܂����o�O������悤�Ȃ̂Ŏb��I�ɉ�����Ēl��ݒ�
		timeslot[0]->Set(2.45, 2.55);
//		timeslot[0]->Lock();
		timeslot[1]->Set(4.45, 4.55);
//		timeslot[1]->Lock();
		target[0]->LockTrajectory();
		target[1]->LockTrajectory();
		
		// ���̂Ɗ֐߂�C^1�S���𓯎���ON�ɂ���ƐU���I�ɂȂ�
		graph.Enable(DiMP2::ID(DiMP2::ConTag::ObjectC1T), false);
		graph.Enable(DiMP2::ID(DiMP2::ConTag::ObjectC1R), false);

		// �D��x:
		// - ��{�S��		0
		// - ���[�`���O0		1
		// - ���[�`���O1		2
		graph.SetPriority(DiMP2::ID(DiMP2::ConTag::MatchingTP, task[0]), 1);
		graph.SetPriority(DiMP2::ID(DiMP2::ConTag::MatchingTV, task[0]), 1);
		graph.SetPriority(DiMP2::ID(DiMP2::ConTag::MatchingTP, task[1]), 2);
		graph.SetPriority(DiMP2::ID(DiMP2::ConTag::MatchingTV, task[1]), 2);
		
		// �ȒP�Ȍn�Ȃ̂Ŕ����񐔂����Ȃ߂�
		graph.solver.SetNumIteration(10);
		graph.SetCorrectionRate(DiMP2::ID(), .8);
		graph.SetCorrectionRate(DiMP2::ID(DiMP2::ConTag::MatchingTP, task[0]), .4);
		graph.SetCorrectionRate(DiMP2::ID(DiMP2::ConTag::MatchingTP, task[1]), .8);
	}
	if(sceneSelect == Reaching3D){
		//FileMapping<TP_setting> fm;
		//TP_setting tp;
		//fm.readTo( &tp );
		//// SHMEM����ݒ�󂯎��悤�ɕύX
		//// ������
		//graph.AddTicks(tp.ts, tp.dt, tp.te);

		//// �^�C���X���b�g
		//timeslot.resize(tp.nTSS);
		//for( unsigned i = 0; i < tp.nTSS; i++ ) {
		//	std::stringstream ss;
		//	ss << "timeslot" << i;
		//	timeslot[i] = graph.CreateTimeSlot( ss.str(), tp.tss[i].ts, tp.tss[i].te );
		//}
		//
		//// ���[�`���O�^�X�N
		//task.resize(tp.nTSS);
		//for( unsigned i = 0; i < tp.nTSS; i++ ) {
		//	std::stringstream ss;
		//	ss << "task" << i;
		//	task[i] = graph.CreateMatchingTask( ss.str(), robot[0]->hand, target[i], timeslot[i] );
		//}
		
		// ������
		graph.AddTicks(0.0, 0.25, 10.0);

		// �^�C���X���b�g
		timeslot.resize(5);
		timeslot[0] = graph.CreateTimeSlot("timeslot0", 2, 3);
		timeslot[1] = graph.CreateTimeSlot("timeslot1", 6,10);
		timeslot[2] = graph.CreateTimeSlot("timeslot2", 0,0);
		timeslot[3] = graph.CreateTimeSlot("timeslot3", 0,0 );
		timeslot[4] = graph.CreateTimeSlot("timeslot4", 0,0);
		
		// ���[�`���O�^�X�N
		task.resize(5);
		task[0] = graph.CreateMatchingTask("task0", robot[0]->hand, target[0], timeslot[0]);
		task[1] = graph.CreateMatchingTask("task1", robot[0]->hand, target[1], timeslot[1]);
		task[2] = graph.CreateMatchingTask("task2", robot[0]->hand, target[2], timeslot[2]);
		task[3] = graph.CreateMatchingTask("task2", robot[0]->hand, target[3], timeslot[3]);
		task[4] = graph.CreateMatchingTask("task2", robot[0]->hand, target[4], timeslot[4]);

		// �����
		//DiMP2::Object* obstacle = adaptorDiMP.GetObject("workspace/body_obstacle");
		//graph.CreateAvoidTask("avoid", robot[0]->link[0], obstacle);
		//graph.CreateAvoidTask("avoid", robot[0]->link[1], obstacle);
		//graph.CreateAvoidTask("avoid", robot[0]->link[2], obstacle);
		//graph.CreateAvoidTask("avoid", robot[0]->link[3], obstacle);
		//graph.CreateAvoidTask("avoid", robot[0]->link[4], obstacle);
		//graph.CreateAvoidTask("avoid", robot[0]->link[5], obstacle);
		//graph.CreateAvoidTask("avoid", robot[0]->link[6], obstacle);
		//graph.CreateAvoidTask("avoid", robot[0]->hand   , obstacle);

		// ��D��x�Ŋ֐ߑ��x��0�ɂ���
		for each(DJnt* jnt in robot[0]->joint){
			jnt->SetVelRange(0, 0.0, 0.0);
		}

		// �����ϐ�������
		graph.SetScaling(.5, .5, 1.0);
		graph.Init();
	
//		timeslot[0]->Set(3,5);//145, 155);
		timeslot[0]->Lock(false);
//		timeslot[1]->Set(6,7);//2.95, 3.05);
		timeslot[1]->Lock(false);
//		timeslot[2]->Set(9,9.5);//4.70, 4.80);
		timeslot[2]->Lock(false);
//		timeslot[3]->Set(9,9.5);//4.70, 4.80);
		timeslot[3]->Lock(false);
//		timeslot[4]->Set(9,9.5);//4.70, 4.80);
		timeslot[4]->Lock(false);
		target[0]->LockTrajectory();
		target[1]->LockTrajectory();
		target[2]->LockTrajectory();
		target[3]->LockTrajectory();
		target[4]->LockTrajectory();
		// SHMEM���炤���Ƃ��ώ��ɂ���
		//for( unsigned i = 0; i < tp.nTSS; i++ ) {
		//	target[i]->LockTrajectory();
		//}

		
		// ���̂Ɗ֐߂�C^1�S���𓯎���ON�ɂ���ƐU���I�ɂȂ�
		graph.Enable(DiMP2::ID(DiMP2::ConTag::ObjectC1T  ), false);
		graph.Enable(DiMP2::ID(DiMP2::ConTag::ObjectC1R  ), false);
		//graph.Enable(DiMP2::ID(DiMP2::ConTag::JointC1    ), false);
		//graph.Enable(DiMP2::ID(DiMP2::ConTag::JointRangeV), false);
		graph.Enable(DiMP2::ID(DiMP2::ConTag::ForceT     ), false);
		graph.Enable(DiMP2::ID(DiMP2::ConTag::ForceR     ), false);
		graph.Enable(DiMP2::ID(DiMP2::ConTag::MatchingRP ), false);
		graph.Enable(DiMP2::ID(DiMP2::ConTag::MatchingRV ), false);
		
		// �D��x
		graph.SetPriority(DiMP2::ID(DiMP2::ConTag::AvoidP), 1);
		graph.SetPriority(DiMP2::ID(DiMP2::ConTag::AvoidV), 1);
		graph.SetPriority(DiMP2::ID(DiMP2::ConTag::MatchingTP), 2);
		graph.SetPriority(DiMP2::ID(DiMP2::ConTag::MatchingTV), 2);
		graph.SetPriority(DiMP2::ID(DiMP2::ConTag::JointRangeV), 3);

		graph.solver.SetNumIteration(20, 0);
		graph.solver.SetNumIteration(5, 1);
		graph.solver.SetNumIteration(5, 2);
		graph.solver.SetNumIteration(5, 3);
		graph.SetCorrectionRate(DiMP2::ID(), .5);
		graph.SetCorrectionRate(DiMP2::ID(DiMP2::ConTag::JointRangeV), 0.25);
	}
	if(sceneSelect == Toss3D){
		// ������
		graph.AddTicks(0.0, 0.25, 5.0);

		// �^�C���X���b�g
		timeslot.resize(2);
		timeslot[0] = graph.CreateTimeSlot("timeslot0");
		timeslot[1] = graph.CreateTimeSlot("timeslot1");
		//timeslot[2] = graph.CreateTimeSlot("timeslot2");
		timeslot[0]->Set(4.0, 5.0);
		timeslot[1]->Set(0.0, 3.0);
		timeslot[0]->Lock();
		timeslot[1]->Lock();
		
		// ���Ԍ���Ńn���h�ƃ^�[�Q�b�g���Ȃ�
		graph.CreateFixjoint("fix1", robot[0]->hand->CreateConnector(), target[0]->CreateConnector(), timeslot[0]);
		graph.CreateFixjoint("fix2", robot[1]->hand->CreateConnector(), target[0]->CreateConnector(), timeslot[1]);
		//target[0]->SetInitialVelocity(vec3_t(1.0, 0.0, 0.0));

		/*foreach(RobotArm* r, robot){
			r->joint[0]->SetInitialPos(0, Rad(90.0));
			r->joint[1]->SetInitialPos(0, Rad(60.0));
			r->joint[2]->SetInitialPos(0, Rad( 0.0));
			r->joint[3]->SetInitialPos(0, Rad(90.0));
			r->joint[4]->SetInitialPos(0, Rad( 0.0));
			r->joint[5]->SetInitialPos(0, Rad( 0.0));
		}*/

		// ���[�`���O�^�X�N
		//task.resize(1);
		//task[0] = graph.CreateMatchingTask("task0", target[0], target[1], timeslot[1]);
		
		// ��D��x�Ŋ֐ߑ��x��0�ɂ���
		/*for each(DJnt* jnt in robot[0]->joint){
			jnt->SetVelRange(0, 0.0, 0.0);
		}
		for each(DJnt* jnt in robot[1]->joint){
			jnt->SetVelRange(0, 0.0, 0.0);
		}*/

		// �����ϐ�������
		graph.SetScaling(.5, .5, 1.0);
		graph.Init();
	
		// ���̂Ɗ֐߂�C^1�S���𓯎���ON�ɂ���ƐU���I�ɂȂ�
		graph.Enable(DiMP2::ID(DiMP2::ConTag::ObjectC1T), true);
		graph.Enable(DiMP2::ID(DiMP2::ConTag::ObjectC1R), false);
		//graph.Enable(DiMP2::ID(DiMP2::ConTag::ObjectC1T, target[0]), true);
		//graph.Enable(DiMP2::ID(DiMP2::ConTag::ObjectC1T, robot[0]->hand), true);
		graph.Enable(DiMP2::ID(DiMP2::ConTag::JointC1), false);
		graph.Enable(DiMP2::ID(DiMP2::ConTag::JointRangeV), false);
		//graph.Enable(DiMP2::ID(DiMP2::ConTag::ForceT), false);
		//graph.Enable(DiMP2::ID(DiMP2::ConTag::ForceR), false);
		
		// �D��x
		//graph.SetPriority(DiMP2::ID(DiMP2::ConTag::MatchingTP), 1);
		//graph.SetPriority(DiMP2::ID(DiMP2::ConTag::MatchingTV), 1);
		//graph.SetPriority(DiMP2::ID(DiMP2::ConTag::MatchingRP), 1);
		//graph.SetPriority(DiMP2::ID(DiMP2::ConTag::MatchingRV), 1);
		graph.SetPriority(DiMP2::ID(DiMP2::ConTag::JointRangeV), 1);

		graph.solver.SetNumIteration(20, 0);
		graph.solver.SetNumIteration(5, 1);
		graph.SetCorrectionRate(DiMP2::ID(), .5);
	}

	// �\���o�I�v�V�����̐ݒ�
	graph.solver.SetAlgorithm(DiMP2::Algorithm::Pareto);
	graph.solver.SetIterationOrder(DiMP2::Iteration::Sequential);

	for each(RobotArm* r in robot){
		// ���@�\�w�v�Z�Ń����N�ʒu������
		r->link[0]->ForwardKinematics();
	}

	// ���O�L����
	graph.solver.EnableLogging(DiMP2::Logging::MajorLoop, true);
}
