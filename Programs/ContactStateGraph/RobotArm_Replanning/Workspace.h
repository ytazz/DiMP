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
	// 構築するロボットアームの選択
	enum{
		Reaching2D,		//< 平面上のロボットアームのリーチング
		Reaching3D,		//< 3D上のロボットアームのリーチング＋障害物回避
		Toss3D,			//< 3D上の複数のロボットアームの物体搬送
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
