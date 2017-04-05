#ifndef HUMANOIDCATHING_H
#define HUMANOIDCATHING_H

#include <DiMP2/Graph.h>

using namespace DiMP2;

class Humanoid : public DiMP2::Graph{
public:
	/// ñ⁄ïWï®ëÃÇÃèÓïÒ
	struct TargetInfo{
		vec3_t	iniPos;
		vec3_t	iniVel;
		bool	ball_or_box;		///< ball or box
		bool	left_or_right;		///< catch by left or right hand
		real_t	ballRadius;
		vec3_t	boxSize;
		
		real_t	iniStartTime;
		real_t	iniEndTime;

		DiMP2::Object*			hand;		///< hand that catches the target
		DiMP2::Object*			target;		///< target object
		DiMP2::TimeSlot*		timeSlot;	///< catching time slot
		DiMP2::MatchingTask*	task;		///< catching/holding task

		TargetInfo(){
			ball_or_box = true;
			left_or_right = true;
			ballRadius = 1.0;
			boxSize = vec3_t(1.0, 1.0, 1.0);
			iniStartTime = 0.0;
			iniEndTime = 1.0;
		}
	};

	struct Body{
		DiMP2::Object*	base;
		DiMP2::Object*	waist;
		DiMP2::Object*	chest;

		DiMP2::Hinge*	yaw;
		DiMP2::Hinge*	pitch;
	};

	struct Arm{
		DiMP2::Object*	hand;
		DiMP2::Object*	lower;
		DiMP2::Object*	upper;

		DiMP2::Hinge*	shoulder;
		DiMP2::Hinge*	twist;
		DiMP2::Hinge*	elbow;
	};

	vector<DiMP2::Object*>	Targets;

	Body		body;
	Arm			arm[2];

	vector<DiMP2::Object*>		links;
	vector<DiMP2::Hinge*>		joints;

	vector<DiMP2::TimeSlot*>	timeslots;

	vector<TargetInfo>			targetInfos;
	
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
