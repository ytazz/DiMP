#pragma once

#include <DiMP/sbdimp.h>

using namespace Scenebuilder;

/*
 * RobotArm
 */

class RobotArm : public UTRefCount{
public:
	vector<DiMP::Object*>	link;			///< link objects
	DiMP::Object*			hand;			///< hand object
	vector<DiMP::Joint*>	joint;			///< joints
	
public:
	bool Build(int idx, DiMP::Graph* graph);
	void Clear();
	void Step();

	 RobotArm();
	~RobotArm();
};
