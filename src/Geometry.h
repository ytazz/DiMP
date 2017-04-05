#pragma once

#include <DiMP2/Node.h>

namespace DiMP2{;

class Object;
class Connector;

/** geometry base class
 **/
class Geometry : public Node{
public:
	virtual real_t CalcBSphere() = 0;

	Geometry(Graph* g, const string& n);
	virtual ~Geometry();
};

/** sphere
 **/
class Sphere : public Geometry{
public:
	real_t	radius;

	virtual void   Draw       (DrawCanvas* canvas, DrawConfig* conf);
	virtual real_t CalcBSphere();

	Sphere(Graph* g, real_t r, const string& n = "");
};

/** box
 */
class Box : public Geometry{
public:
	vec3_t size;

	virtual void   Draw       (DrawCanvas* canvas, DrawConfig* conf);
	virtual real_t CalcBSphere();

	Box(Graph* g, const vec3_t& sz, const string& n = "");
};

/** cylinder
 */
class Cylinder : public Geometry{
public:
	real_t radius;
	real_t length;

	virtual void   Draw       (DrawCanvas* canvas, DrawConfig* conf);
	virtual real_t CalcBSphere();

	Cylinder(Graph* g, real_t r, real_t l, const string& n = "");
};

/** infinite x-y plane
 */
class Plane : public Geometry{
public:
	virtual real_t CalcBSphere(){ return 0.0; } //< Žb’è

	Plane(Graph* g, const string& n = ""):Geometry(g, n){}
};

}
