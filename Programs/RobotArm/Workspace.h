#pragma once

#include <sbbuilder.h>
#include <sbscenelocal.h>
#include <DiMP/sbdimp.h>
#include <SprGraphics/sbsprgraphics.h>
#include <SprPhysics/sbsprphysics.h>
//#include <DiMP2/Ipopt.h>

using namespace Scenebuilder;

class RobotArm;

/*
 * Workspace
 */

class Workspace : public UTRefCount{
public:
    string          sceneFilename;

    SceneLocal      scene;
	TypeDB          typedb;
	ModelContainer  models;
	Builder         builder;

	AdaptorDiMP		adaptorDiMP;
	AdaptorSprGR	adaptorSprGR;

    DiMP::Graph*                graph;
    vector<DiMP::Object*>       target;		    ///< target objects
	vector<DiMP::Object*>       obstacle;		///< obstacles
	vector<DiMP::TimeSlot*>     timeSlot;		///< time slots
	vector<DiMP::MatchTask*>    matchTask;		///< match tasks
	vector<DiMP::AvoidTask*>    avoidTask;      ///< avoid tasks

    typedef vector< UTRef<RobotArm> >	Robots;
	Robots          robot;			///< robotic arms;

public:
    bool Build();
    void DrawSnapshot(GRRenderIf* render, real_t time);

	 Workspace();
    ~Workspace();
};
