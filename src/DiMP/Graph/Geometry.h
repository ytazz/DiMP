#pragma once

#include <DiMP/Graph/Node.h>

namespace DiMP{;

class Object;
class Connector;

class Geometry : public Node{
public:
	vec3_t bsphereCenter;
	real_t bsphereRadius;

public:
	virtual void   CalcBSphere() = 0;
	virtual vec3_t CalcSupport(const vec3_t& dir) = 0;

	Geometry(Graph* g, const string& n);
	virtual ~Geometry();
};

/** sphere
 **/
class Sphere : public Geometry{
public:
	real_t	radius;

	virtual void   Draw       (Render::Canvas* canvas, Render::Config* conf);
	virtual void   CalcBSphere();
	virtual vec3_t CalcSupport(const vec3_t& dir);

	Sphere(Graph* g, real_t r, const string& n = "");
};

/** box
 */
class Box : public Geometry{
public:
	vec3_t size;

	virtual void   Draw       (Render::Canvas* canvas, Render::Config* conf);
	virtual void   CalcBSphere();
	virtual vec3_t CalcSupport(const vec3_t& dir);

	Box(Graph* g, const vec3_t& sz, const string& n = "");
};

/** cylinder
 */
class Cylinder : public Geometry{
public:
	real_t radius;
	real_t length;

	virtual void   Draw       (Render::Canvas* canvas, Render::Config* conf);
	virtual void   CalcBSphere();
	virtual vec3_t CalcSupport(const vec3_t& dir);

	Cylinder(Graph* g, real_t r, real_t l, const string& n = "");
};

/** capsule
 */
class Capsule : public Geometry{
public:
	real_t radius;
	real_t length;

	virtual void   Draw       (Render::Canvas* canvas, Render::Config* conf);
	virtual void   CalcBSphere();
	virtual vec3_t CalcSupport(const vec3_t& dir);

	Capsule(Graph* g, real_t r, real_t l, const string& n = "");
};

/** infinite x-y plane
 */
class Plane : public Geometry{
public:
	virtual void   CalcBSphere();
	virtual vec3_t CalcSupport(const vec3_t& dir);

	Plane(Graph* g, const string& n = "");
};

struct TriangleBase{
	vec3_t vertices[3];
	vec3_t normal;

	void  Draw(Render::Canvas* canvas);
};

class Triangle : public Geometry, public TriangleBase{
public:
	virtual void   Draw       (Render::Canvas* canvas, Render::Config* conf);
	virtual void   CalcBSphere();
	virtual vec3_t CalcSupport(const vec3_t& dir);

	Triangle(Graph* g, const string& n = "");
};

/** mesh
 */
class Mesh : public Geometry{
public:
	int ntheta;
	int nphi;

	vector<TriangleBase>     tris;
	vector< pair<int, int> > supportMap;

public:
	void CreateSupportMap();

	virtual void   Draw       (Render::Canvas* canvas, Render::Config* conf);
	virtual void   CalcBSphere();
	virtual vec3_t CalcSupport(const vec3_t& dir);

	Mesh(Graph* g, const string& n = "");
};

void CalcNearest(Geometry* g0, Geometry* g1, const pose_t& pose0, const pose_t& pose1, vec3_t& sup0, vec3_t& sup1, real_t& dist);

}
