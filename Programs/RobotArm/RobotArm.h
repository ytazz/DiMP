#pragma once

#include <DiMP/sbdimp.h>

using namespace Scenebuilder;

/*
 * RobotArm
 */

class RobotArm : public UTRefCount{
public:
	vector<DiMP2::Object*>	link;			///< link objects
	DiMP2::Object*			hand;			///< hand object
	vector<DiMP2::Joint*>	joint;			///< joints
	
public:
	bool Build(int idx, AdaptorDiMP* adaptor);
	void Clear();
	void Step();

	RobotArm();
};
