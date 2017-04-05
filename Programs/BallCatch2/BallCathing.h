#ifndef BALLCATCHING2_H
#define BALLCATCHING2_H

#include "../DiMP2/Graph.h"

using namespace DiMP2;

class MyDrawConfig : public DrawConfig{
public:
	bool					snapshot;
	DrawContext::Material	matSnapshot;

	virtual bool Set(DrawContext* draw, ID id);

	MyDrawConfig();
};

class BallCatching2 : public DiMP2::Graph{
public:
	
	///Body
	struct Body{
		DiMP2::Object*	base;
		/*DiMP2::Object*	waist;
		DiMP2::Object*	chest;

		DiMP2::Hinge*	yaw;
		DiMP2::Hinge*	pitch;*/
	};

	///Arm
	struct Arm{
		DiMP2::Object*	hand;
		DiMP2::Object*	lower;
		DiMP2::Object*	upper;

		DiMP2::Hinge*	shoulder;
		DiMP2::Hinge*	twist;
		DiMP2::Hinge*	elbow;
	};

	Body		body;
	Arm			arm;

	vector<DiMP2::Object*>	links;
	vector<DiMP2::Hinge*>	joints;
	
	///position_matchings
	vector<DiMP2::Object*>		balls;
	vector<DiMP2::PositionMatching*>		position_matchings;
	vector<DiMP2::OR_Composition*>	or_compositions;
	
	///Ball
	struct BallInfo{
		Vec3d	iniPos;
		Vec3d	iniVel;
		double	radius;
		double	iniCatchTime;

		BallInfo(){}
		BallInfo(Vec3d p, Vec3d v, double r, double t): iniPos(p), iniVel(v), radius(r), iniCatchTime(t){}
	};

	vector<BallInfo>		ballInfos;
	





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
	virtual void Draw(DrawContext* draw, DrawConfig* conf);

};

#endif
