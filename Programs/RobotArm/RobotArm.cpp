#include "RobotArm.h"

#include <boost/lexical_cast.hpp>
using namespace boost;

//------------------------------------------------------------------------------------------------
// RobotArm

RobotArm::RobotArm(){

}

RobotArm::~RobotArm(){

}

bool RobotArm::Build(int idx, AdaptorDiMP* adaptor){
	string strIdx = lexical_cast<string>(idx);

	for(int i = 0; ; i++) try{
		DiMP::Object* obj = adaptor->GetObject("workspace/robot" + strIdx + "/body_link" + lexical_cast<string>(i));
		link.push_back(obj);
	} catch(...){ break; }

	if(link.empty())
		return false;

	hand = link.back();

	for(int i = 0; ; i++) try{
		DiMP::Joint* jnt = adaptor->GetJoint("workspace/robot" + strIdx + "/joint" + lexical_cast<string>(i));
		joint.push_back(jnt);
	} catch(...){ break; }

	return true;
}
