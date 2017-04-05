#ifndef BOXPOSITION_MATCHING_H
#define BOXPOSITION_MATCHING_H

#include "../DiMP2/Graph.h"
#include "../DiMP2/SprDiMP.h"
#include "../src/SceneBuilder2.h"
#include "../src/SprHandler.h"
#include "../src/DiMPHandler.h"
#include "../src/Draw.h"

using namespace DiMP2;

class MyDrawConfig : public DrawConfig{
public:
	bool					snapshot;
	DrawContext::Material	matSnapshot;

	virtual bool Set(DrawContext* draw, ID id);

	MyDrawConfig();
};

/** Serial-link manipulator base class
 */
class Manipulator : public DiMP2::Graph{
public:
	// joint configuration
	struct JointInfo{
		double	iniPos;
		double	iniVel;
		Posed	socket;
		Posed	plug;
		double	axisLength;		///< specifies the length of cylinder geometry
	};
	struct LinkInfo{
		double	mass;
		double	inertia;
		double	length;
		double	radius;

		Vec3d	inipos;
	};
	struct ObstableInfo{
		Vec3d	pos;
		double	radius;
	};

	struct BoxInfo{
		double	mass;
		double	inertia;
		vec3_t	boxSize;
		vec3_t	iniPos;
		vec3_t	iniVel;
		double	iniPositionMatchingStartTime;
		//double	iniPositionMatchingCompleteTime;
		double	iniPositionMatchingEndTime;
		BoxInfo(){}
		BoxInfo(double m,double i, vec3_t sz, vec3_t p,vec3_t v, double t_s, double t_e): mass(m), inertia(i), boxSize(sz), iniPos(p),
			iniVel(v), iniPositionMatchingStartTime(t_s), iniPositionMatchingEndTime(t_e){}
	};

//	Vec3d		target;

	vector<JointInfo>		jointInfos;
	vector<LinkInfo>		linkInfos;
	vector<ObstableInfo>	obstInfos;
	vector<BoxInfo>			boxInfos;
	// graph nodes
	DiMP2::Event*			eventReach;			///< target reaching event
	DiMP2::Graph*			graph;				///< scene graph
	
	//RobotArm
	DiMP2::Object*			base;
	vector<DiMP2::Object*>	links;
	vector<DiMP2::Hinge*>	joints;
	vector<DiMP2::Object*>	obstacles;

	//Target
	vector<DiMP2::Object*>		boxes;

	//PositionMatching
	vector<DiMP2::PositionMatching*>		position_matchings;

public:
	/// move to initial posture
	void SetInitialState();

	/// move force variables to compensate gravity
	void ChargeForce();

	/// enable all constraints
	void LiftOff();

	/// task start
	void TaskStart();

	virtual void Build();

	void	SetTarget(uint idx, bool on, vec3_t pos = vec3_t(), uint priority = 1);
};

class BoxHolder : public Manipulator{
public:
	virtual void Build();
	virtual void Draw(DrawContext* draw, DrawConfig* conf);
};


#endif
