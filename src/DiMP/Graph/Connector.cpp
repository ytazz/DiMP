#include <DiMP/Graph/Connector.h>
#include <DiMP/Graph/Graph.h>
#include <DiMP/Graph/Object.h>
#include <DiMP/Solver/Solver.h>
#include <DiMP/Render/Config.h>

namespace DiMP{;

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
