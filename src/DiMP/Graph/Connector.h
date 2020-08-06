#pragma once

#include <DiMP/Graph/Node.h>

namespace DiMP{;

class Object  ;
class Joint   ;
class Geometry;

/**
	connector
 */

class Connector : public Node{
public:
	Object*                 obj;
	ArrayBase<Joint   *>    joints;
	ArrayBase<Geometry*>    geos;

	pose_t		pose;

public:
	void Attach(Geometry* geo);

	Connector(Object*, const vec3_t& p = vec3_t(), const quat_t& q = quat_t(), const string& n = "");
	virtual ~Connector();
};	

typedef NodeArray<Connector>  Connectors;

}
