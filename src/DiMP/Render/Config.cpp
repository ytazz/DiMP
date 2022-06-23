#include <DiMP/Graph/Graph.h>
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
	
	     if(attr == Item::ObjectTrajectory){ c = "black"  ; lw = 2.0f; }
	else if(attr == Item::ObjectPos       ){ c = "black"  ; ps = 3.0f; }
	else if(attr == Item::ObjectVel       ){ c = "blue"   ;            }
	else if(attr == Item::ObjectAngvel    ){ c = "cyan"   ;            }
	else if(attr == Item::Connector       ){ c = "gray"   ;            }
	else if(attr == Item::JointForce      ){ c = "red"    ;            }
	else if(attr == Item::JointMoment     ){ c = "magenta";            }
	else if(attr == Item::Geometry        ){ c = "green"  ;            }
	else if(attr == Item::Avoid           ){ c = "red"    ;            }
	else if(attr == Item::BipedCom        ){ c = "black"  ; lw = 3.0f; }
	else if(attr == Item::BipedCop        ){ c = "magenta"; lw = 3.0f; }
	else if(attr == Item::BipedCmp        ){ c = "green"  ; lw = 1.0f; }
	else if(attr == Item::BipedTorso      ){ c = "blue"   ; lw = 2.0f; }
	else if(attr == Item::BipedFoot       ){ c = "black"  ; lw = 1.0f; }
	else if(attr == Item::BipedFootCop    ){ c = "magenta"; lw = 1.0f; }
	else if(attr == Item::CentroidPos     ){ c = "black"  ; lw = 3.0f; }
	else if(attr == Item::CentroidEnd     ){ c = "black"  ; lw = 0.5f; }
	else if(attr == Item::CentroidEndTraj ){ c = "black"  ; lw = 1.5f; }
	else if(attr == Item::CentroidFace    ){ c = "green"  ; lw = 1.0f; }
	else if(attr == Item::CentroidForce   ){ c = "magenta"; lw = 1.0f; }
    else if(attr == Item::CentroidTorso   ){ c = "black"  ; lw = 1.0f; }

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
