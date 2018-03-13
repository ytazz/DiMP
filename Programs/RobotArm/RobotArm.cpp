#include "RobotArm.h"

//------------------------------------------------------------------------------------------------
// RobotArm

RobotArm::RobotArm(){
	hand = 0;
}

RobotArm::~RobotArm(){

}

bool RobotArm::Build(int idx, DiMP::Graph* graph){
	stringstream ss;

	for(int i = 0; ; i++){
		ss.str("");
		ss << "workspace/robot" << idx << "/body_link" << i;
		DiMP::Object* obj = graph->objects.Find(ss.str());
		if(!obj)
			break;
		link.push_back(obj);
	}

	if(link.empty())
		return false;

	// link.backとするとreleaseビルドで例外　原因不明
	hand = link[link.size()-1];

	for(int i = 0; ; i++){
		ss.str("");
		ss << "workspace/robot" << idx << "/joint" << i;
		DiMP::Joint* jnt = graph->joints.Find(ss.str());
		if(!jnt)
			break;
		joint.push_back(jnt);
	}

	return true;
}
