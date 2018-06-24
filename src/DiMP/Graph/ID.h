#pragma once

#include <DiMP/Types.h>

namespace DiMP{;

class Node;
class Tick;

/// variable identifiers
struct VarTag{
	enum{
		Any = 0,
		ObjectTP,		///< object position: translational
		ObjectRP,		///< object position: rotational
		ObjectTV,		///< object velocity: translational
		ObjectRV,		///< object velocity: rotational
		JointP,			///< joint position
		JointV,			///< joint velocity
		JointF,			///< joint torque
		ForceT,			///< joint force: translational
		ForceR,			///< joint force: rotational
		TimeStart,		///< task starting time
		TimeEnd,		///< task ending time
		BipedTorsoTP,
		BipedTorsoRP,
		BipedTorsoTV,
		BipedTorsoRV,
		BipedFootT,
		BipedFootR,
		BipedComP,
		BipedComV,
		BipedCop,
		BipedDuration,
		BipedTime,
		NumTypes,
	};
};

/// constraint identifiers
struct ConTag{
	enum{
		Any = 0,
		ObjectC1T,			///< C^1 continuity of object position
		ObjectC1R,			///< C^1 continuity of object orientation
		JointC1,			///< C^1 continuity of joint position
		JointC2,			///< C^2 continuity of joint position
		JointF,				///< mapping between joint torque and joint force/moment
		JointRangeP,		///< joint position range
		JointRangeDP,		///< joint position change range
		JointRangeV,		///< joint velocity range
		JointRangeF,		///< joint force range
		JointTP,			///< kinematic constraint : position
		JointRP,			///< kinematic constraint : orientation
		JointTV,			///< kinematic constraint : velocity
		JointRV,			///< kinematic constraint : ang.velocity
		ForceT,				///< sum of forces
		ForceR,				///< sum of moments
		ContactP,			///< contact: position
		ContactV,			///< contact: velocity
		ContactFN,			///< contact: normal force
		ContactFT,			///< contact: tangential force
		//ContactPF,			///< contact: complementarity between position and force
		//ContactVF,			///< contact: complementarity between velocity and force
		TimeStartRange,		///< task starting time range
		TimeEndRange,		///< task ending time range
		TimeDurationRange,	///< task duration range
		MatchTP,			///< match task constraint
		MatchTV,
		MatchRP,
		MatchRV,
		AvoidP,
		AvoidV,
		BipedLipP,
		BipedLipV,
		BipedFootRangeT,
		BipedFootRangeR,
		BipedFootMatchT,
		BipedFootMatchR,
		BipedComP,
		BipedComV,
		BipedCop,
		BipedDuration,
		BipedTime,
		NumTypes,
	};
};

struct Type{
	enum{
		Object  ,
		Joint   ,
		Tree    ,
		Task    ,
	};
};

extern const char* VarNames[VarTag::NumTypes];
extern const char* ConNames[ConTag::NumTypes];

}
