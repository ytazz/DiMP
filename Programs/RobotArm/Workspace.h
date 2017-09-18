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

	 Workspace();
    ~Workspace();
};
