#ifndef ROBOTARM_H
#define ROBOTARM_H

#include "../DiMP2/Graph.h"
#include "../DiMP2/SprDiMP.h"

using namespace DiMP2;

class MyDrawConfig : public DrawConfig{
public:
	virtual void Set(DrawContext* draw, uint tag, Node* node = NULL, uint idx = 0);
	MyDrawConfig();
};

class Humanoid{
public:
	MyDrawConfig	conf;
	Graph			graph;

	struct Arm{
		bool	side;	/// left or right
		DiMP2::Object*		upperArm;
		DiMP2::Object*		lowerArm;
		DiMP2::BallJoint*	shoulder;
		DiMP2::Hinge*		elbow;

		void Build(Graph* g);
	};
	struct Leg{
		bool	side;
		DiMP2::Object*		thigh;
		DiMP2::Object*		shank;
		DiMP2::Object*		foot;

		DiMP2::BallJoint*	hip;
		DiMP2::Hinge*		knee;
		DiMP2::BallJoint*	ankle;

		void Build(Graph* g);
	};
	struct Body{
		DiMP2::Object*		head;
		DiMP2::BallJoint*	neck;

		DiMP2::Object*		chest;
		DiMP2::Object*		pelvis;
		DiMP2::BallJoint*	waist;

		Arm		arm[2];
		Leg		leg[2];

		void Build(Graph* g);
	};

	// graph nodes
	DiMP2::Event*			eventReach;		///< target reaching event
	DiMP2::Cost*			costReach;

public:
	void Build();
	void LoadState();
	void Step();
	void Draw(DrawContext* draw);
	//virtual void Build(Spr::FWSceneIf* scene, Graph* graph);
	//virtual void LoadState();
	//virtual void Step();
};

#endif
