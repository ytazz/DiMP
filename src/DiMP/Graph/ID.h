#pragma once

#include <DiMP/Types.h>

namespace DiMP{;

class Node;
class Tick;

/// variable identifiers
struct VarTag{
	enum{
		Any = -1,
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
		Any = -1,
		ObjectC1T,			///< C^1 continuity of object position
		ObjectC1R,			///< C^1 continuity of object orientation
		JointC1,			///< C^1 continuity of joint position
		JointC2,			///< C^2 continuity of joint position
		JointF,				///< mapping between joint torque and joint force/moment
		JointRangeP,		///< joint position range
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
		Eom,				///< equation of motion in joint space
		BipedLipP,
		BipedLipV,
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

/// identifier for variable or constraint
//class ID{
//public:
//	int			tag;		///< variable type
//	Node*		node;		///< reference to owner node, if owned by a node
//	Tick*		tick;		///< refernece to time instant
//	string		name;
//
//	/** calculates match score against given id		
//	 */
//	int Match(ID* id){
//		// tagが未指定のマスクは必ずマッチする
//		if(tag == -1)
//			return 1;
//		// tag不一致
//		if(tag != id->tag)
//			return 0;
//		// tag一致, node未指定
//		if(!node)
//			return 2;
//		// node不一致
//		if(node != id->node)
//			return 0;
//		// tag, node一致, tick未指定
//		if(!tick)
//			return 3;
//		// tick不一致
//		if(tick != id->tick)
//			return 0;
//		// 全一致
//		return 4;
//	}
//
//	ID():tag(-1), node(0), tick(0){}
//	ID(int _tag, Node* _node = 0, Tick* _tick = 0, string _name = ""){
//		tag  = _tag;
//		node = _node;
//		tick = _tick;
//		name = _name;
//	}
//};

}
