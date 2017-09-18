#include <DiMP/Graph/Geometry.h>
#include <DiMP/Graph/Graph.h>
#include <DiMP/Render/Config.h>
#include <DiMP/Render/Canvas.h>

namespace DiMP{;

///////////////////////////////////////////////////////////////////////////////////////////////////

Geometry::Geometry(Graph* g, const string& n):Node(g, n){
	graph->geos.Add(this);
}

Geometry::~Geometry(){
	graph->geos.Remove(this);
}

///////////////////////////////////////////////////////////////////////////////////////////////////

Sphere::Sphere(Graph* g, real_t r, const string& n):Geometry(g, n){
	radius = r;
}

real_t Sphere::CalcBSphere(){
	return radius;
}

void Sphere::Draw(Render::Canvas* canvas, Render::Config* conf){
	if(conf->Set(canvas, Render::Item::Geometry, this))
		canvas->Sphere(Vec3f(), (float)radius);
}

///////////////////////////////////////////////////////////////////////////////////////////////////

Box::Box(Graph* g, const vec3_t& sz, const string& n):Geometry(g, n){
	size = sz;
}

real_t Box::CalcBSphere(){
	return 0.5 * size.norm();
}

void Box::Draw(Render::Canvas* canvas, Render::Config* conf){
	if(conf->Set(canvas, Render::Item::Geometry, this)){
		canvas->Box(-0.5*size, 0.5*size);
	}
}

///////////////////////////////////////////////////////////////////////////////////////////////////

Cylinder::Cylinder(Graph* g, real_t r, real_t l, const string& n):Geometry(g, n){
	radius = r;
	length = l;
}

real_t Cylinder::CalcBSphere(){
	return sqrt(radius * radius + 0.25 * length * length);
}

void Cylinder::Draw(Render::Canvas* canvas, Render::Config* conf){
	if(conf->Set(canvas, Render::Item::Geometry, this))
		canvas->Cylinder((float)radius, (float)length);
}

}
