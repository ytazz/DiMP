#include "RobotArm.h"

//------------------------------------------------------------------------------------------------
// RobotArm

RobotArm::RobotArm(){
	hand = 0;
}

RobotArm::~RobotArm(){

}

bool RobotArm::Build(int idx, AdaptorDiMP* adaptor){
	stringstream ss;

	for(int i = 0; ; i++) try{
		ss.str("");
		ss << "workspace/robot" << idx << "/body_link" << i;
		DiMP::Object* obj = adaptor->GetObject(ss.str());
		link.push_back(obj);
	} catch(Exception&){ break; }

	if(link.empty())
		return false;

	// link.backとするとreleaseビルドで例外　原因不明
	hand = link[link.size()-1];

	for(int i = 0; ; i++) try{
		ss.str("");
		ss << "workspace/robot" << idx << "/joint" << i;
		DiMP::Joint* jnt = adaptor->GetJoint(ss.str());
		joint.push_back(jnt);
	} catch(Exception&){ break; }

	return true;
}
