#include <DiMP2/Connector.h>
#include <DiMP2/Graph.h>
#include <DiMP2/Object.h>
#include <DiMP2/Solver.h>
#include <DiMP2/DrawConfig.h>

namespace DiMP2{;

//-------------------------------------------------------------------------------------------------
// Connector

Connector::Connector(Object* o, const vec3_t& p, const quat_t& q, const string& n):Node(o->graph, n), obj(o){
	graph->cons.Add(this);
	obj  ->cons.Add(this);
	pose.Pos() = p;
	pose.Ori() = q;
}

Connector::~Connector(){
	obj  ->cons.Remove(this);
	graph->cons.Remove(this);
}

void Connector::Attach(Geometry* geo){
	geos.push_back(geo);
}

}
