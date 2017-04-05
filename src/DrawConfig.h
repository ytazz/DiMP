#pragma once

#include <DiMP2/Types.h>

namespace DiMP2{;

class Node;
class DrawCanvas;

/// things to be drawn
struct DrawItem{
	enum{
		GlobalAxis,         ///< global coordinate axis
		ObjectTrajectory,	///< object trajectory
		ObjectPos,			///< object pos
		ObjectVel,			///< object velocity
		ObjectAngvel,		///< object angular velocity
		Connector,			///< socket and plug placement
		JointForce,			///< force
		JointMoment,		///< moment
		Geometry,			///< geometry
		BipedCoM,           ///< biped CoM trajectory
		BipedCoP,           ///< biped CoP
		BipedSwing,         ///< biped swing foot trajectory
		BipedDouble,        ///< biped double support
		BipedTorso,         ///< biped torso trajectory
	};
};

/**
	Configuration for Visualization
 */
class DrawConfig : public UTRefCount{
public:
	/** override this function to customize material settings
		@param render	rendering context
		@param id		identifier of an object
		@return			return true if the object specified by id is to be drawn, return false otherwise.
	 */
	virtual bool Set(DrawCanvas* canvas, int attr, Node* node);

	virtual float Scale(int attr, Node* node);

	DrawConfig();
};

}
