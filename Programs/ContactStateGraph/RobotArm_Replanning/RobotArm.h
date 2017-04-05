#pragma once

#include <DiMP/sbdimp.h>

using namespace Scenebuilder;

/*
 * RobotArm
 */

class RobotArm : public UTRefCount{
public:
	vector<DObj*>	link;			///< link objects
	DObj*			hand;			///< hand object
	vector<DJnt*>	joint;			///< joints
	
public:
	bool Build(int idx, AdaptorDiMP* adaptor);
	void Clear();
	void Step();

	RobotArm();
};
