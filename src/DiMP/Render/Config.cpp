#include <DiMP/Graph/Graph.h>
#include <DiMP/Solver/Solver.h>
#include <DiMP/Render/Config.h>
#include <DiMP/Render/Canvas.h>

namespace DiMP{;
namespace Render{;

Config::Config(){
}

bool Config::Set(Canvas* canvas, int attr, Node* node){
	string c;
	float  lw = 1.0f;
	float  ps = 1.0f;
	
	if(attr == Item::ObjectTrajectory){
		c  = "black";
		lw = 2.0f;
	}
	else if(attr == Item::ObjectPos){
		c  = "black";
		ps = 3.0f;
	}
	else if(attr == Item::ObjectVel   ) c = "blue";
	else if(attr == Item::ObjectAngvel) c = "cyan";
	else if(attr == Item::Connector   ) c = "gray";
	else if(attr == Item::JointForce  ) c = "red";
	else if(attr == Item::JointMoment ) c = "magenta";
	else if(attr == Item::Geometry    ) c = "green";
	
	canvas->SetLineColor(c, 0);
	canvas->SetLineColor(c, 1);
	canvas->SetFillColor(c);
	canvas->SetPointSize(ps);
	canvas->SetLineWidth(lw);
	
	return true;
}

float Config::Scale(int attr, Node* node){
	return 1.0f;
}

}
}
