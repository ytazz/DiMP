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
	// 構築するロボットアームの選択
	enum{
		Reaching2D,		//< 平面上のロボットアームのリーチング
		Reaching3D,		//< 3D上のロボットアームのリーチング＋障害物回避
		Toss3D,			//< 3D上の複数のロボットアームの物体搬送
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

	/// 評価用
	void CheckCollision    ();
	void CheckJointError   ();
	void WriteHandAndTarget();

	Workspace();

};
