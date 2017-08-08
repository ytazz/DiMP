#pragma once

#include <sbbuilder.h>
#include <DiMP/sbdimp.h>
#include <SprGraphics/sbsprgraphics.h>
#include <SprPhysics/sbsprphysics.h>
//#include <DiMP2/Ipopt.h>

using namespace Scenebuilder;

class RobotArm;

/*
 * Workspace
 */

class Workspace : public Builder{
public:
	// �\�z���郍�{�b�g�A�[���̑I��
	enum{
		Reaching2D,		//< ���ʏ�̃��{�b�g�A�[���̃��[�`���O
		Reaching3D,		//< 3D��̃��{�b�g�A�[���̃��[�`���O�{��Q�����
		Toss3D,			//< 3D��̕����̃��{�b�g�A�[���̕��̔���
		Welding,
	};
	
	int sceneSelect;
	
	DiMP2::Graph*			graph;
	AdaptorDiMP				adaptorDiMP;
	AdaptorSprGR			adaptorSprGR;
	AdaptorSprPH			adaptorSprPH;
	
	uint			numTimeSteps;		///< number of time steps
	double			samplePeriod;		///< time resolution

	typedef vector< UTRef<RobotArm> >	Robots;
	Robots                       robot;			///< robotic arms;
	vector<DiMP2::Object*>       target;		///< target objects
	vector<DiMP2::Object*>       obstacle;		///< obstacles
	vector<DiMP2::TimeSlot*>     timeslot;		///< time slots
	vector<DiMP2::MatchTask*>    task;			///< tasks

public:
	void Build(FWSdkIf* sdk);

	/// �]���p
	void CheckCollision    ();
	void CheckJointError   ();
	void WriteHandAndTarget();

	Workspace();

};
