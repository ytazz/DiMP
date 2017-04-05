#pragma once

#include <Scenebuilder.h>
#include <DiMP/sbdimp.h>
#include <SprGraphics/sbsprgraphics.h>
#include <SprPhysics/sbsprphysics.h>

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
	};
	
	int sceneSelect;
	
	DGraph			graph;
	AdaptorDiMP		adaptorDiMP;
	AdaptorSprGR	adaptorSprGR;
	AdaptorSprPH	adaptorSprPH;
	
	uint			numTimeSteps;		///< number of time steps
	double			samplePeriod;		///< time resolution

	typedef vector< UTRef<RobotArm> >	Robots;
	Robots				robot;			///< robotic arms;
	vector<DObj*>		target;			///< target objects
	vector<DTime*>		timeslot;		///< time slots
	vector<DTask*>		task;			///< tasks

public:
	void Build(FWSdkIf* sdk);
	void Clear();
	void Step();

	Workspace();

};
