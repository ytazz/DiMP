#include <DiMP2/Graph.h>
#include <DiMP2/Solver.h>
#include <DiMP2/DrawConfig.h>
#include <DiMP2/DrawCanvas.h>

namespace DiMP2{;

DrawConfig::DrawConfig(){
}

bool DrawConfig::Set(DrawCanvas* canvas, int attr, Node* node){
	string c;
	float  lw = 1.0f;
	float  ps = 1.0f;
	
	if(attr == DrawItem::ObjectTrajectory){
		c  = "black";
		lw = 2.0f;
	}
	else if(attr == DrawItem::ObjectPos){
		c  = "black";
		ps = 3.0f;
	}
	else if(attr == DrawItem::ObjectVel   ) c = "blue";
	else if(attr == DrawItem::ObjectAngvel) c = "cyan";
	else if(attr == DrawItem::Connector   ) c = "gray";
	else if(attr == DrawItem::JointForce  ) c = "red";
	else if(attr == DrawItem::JointMoment ) c = "magenta";
	else if(attr == DrawItem::Geometry    ) c = "green";
	
	canvas->SetLineColor(c, 0);
	canvas->SetLineColor(c, 1);
	canvas->SetFillColor(c);
	canvas->SetPointSize(ps);
	canvas->SetLineWidth(lw);
	
	return true;
}

float DrawConfig::Scale(int attr, Node* node){
	return 1.0f;
}

}