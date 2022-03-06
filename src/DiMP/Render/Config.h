#pragma once

#include <DiMP/Types.h>

namespace DiMP{;

class Node;

namespace Render{;

class Canvas;

/// things to be drawn
struct Item{
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
		BipedCom,           ///< biped CoM trajectory
		BipedCop,           ///< biped CoP
		BipedCmp,
		BipedFoot,          ///< biped swing foot trajectory
		BipedFootCop,
		BipedTorso,         ///< biped torso trajectory
		CentroidPos,
		CentroidEnd,
		CentroidFace,
		CentroidForce,
        CentroidTorso,
		Avoid,
	};
};

/**
	Configuration for Visualization
 */
class Config : public UTRefCount{
public:
	/** override this function to customize material settings
		@param render	rendering context
		@param id		identifier of an object
		@return			return true if the object specified by id is to be drawn, return false otherwise.
	 */
	virtual bool Set(Canvas* canvas, int attr, Node* node);

	virtual float Scale(int attr, Node* node);

	Config();
};

}
}
