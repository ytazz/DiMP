#pragma once

#include <DiMP/Graph/Node.h>

namespace DiMP{;

class Object;
class Connector;

/** geometry base class
 **/
class Geometry : public Node{
public:
	vec3_t bsphereCenter;
	real_t bsphereRadius;

public:
	virtual void CalcBSphere() = 0;

	Geometry(Graph* g, const string& n);
	virtual ~Geometry();
};

class ConvexGeometry : public Geometry{
public:
	virtual void CalcSupport(const vec3_t& dir, vec3_t& sup) = 0;

	ConvexGeometry(Graph* g, const string& n);
	virtual ~ConvexGeometry();
};

/** sphere
 **/
class Sphere : public ConvexGeometry{
public:
	real_t	radius;

	virtual void Draw       (Render::Canvas* canvas, Render::Config* conf);
	virtual void CalcBSphere();
	virtual void CalcSupport(const vec3_t& dir, vec3_t& sup);

	Sphere(Graph* g, real_t r, const string& n = "");
};

/** box
 */
class Box : public ConvexGeometry{
public:
	vec3_t size;

	virtual void Draw       (Render::Canvas* canvas, Render::Config* conf);
	virtual void CalcBSphere();
	virtual void CalcSupport(const vec3_t& dir, vec3_t& sup);

	Box(Graph* g, const vec3_t& sz, const string& n = "");
};

/** cylinder
 */
class Cylinder : public ConvexGeometry{
public:
	real_t radius;
	real_t length;

	virtual void Draw       (Render::Canvas* canvas, Render::Config* conf);
	virtual void CalcBSphere();
	virtual void CalcSupport(const vec3_t& dir, vec3_t& sup);

	Cylinder(Graph* g, real_t r, real_t l, const string& n = "");
};

/** infinite x-y plane
 */
class Plane : public ConvexGeometry{
public:
	virtual void CalcBSphere();
	virtual void CalcSupport(const vec3_t& dir, vec3_t& sup);

	Plane(Graph* g, const string& n = "");
};

class Triangle : public ConvexGeometry{
public:
	vec3_t vertices[3];
	vec3_t normal;

	virtual void CalcBSphere();
	virtual void CalcSupport(const vec3_t& dir, vec3_t& sup);

	Triangle(Graph* g, const string& n = "");
};

/** mesh
 */
class ConvexMesh : public ConvexGeometry{
public:
	vector<Triangle> tris;

public:
	virtual void CalcBSphere();
	virtual void CalcSupport(const vec3_t& dir, vec3_t& sup);

	ConvexMesh(Graph* g, const string& n = "");
};

void CalcNearest(ConvexGeometry   * g0, ConvexGeometry   * g1, vec3_t& p0, vec3_t& p1);

}
