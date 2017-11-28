#include <DiMP/Graph/Geometry.h>
#include <DiMP/Graph/Graph.h>
#include <DiMP/Render/Config.h>
#include <DiMP/Render/Canvas.h>

namespace DiMP{;

const real_t inf = numeric_limits<real_t>::max();

///////////////////////////////////////////////////////////////////////////////////////////////////

Geometry::Geometry(Graph* g, const string& n):Node(g, n){
	graph->geos.Add(this);

	bsphereCenter = vec3_t();
	bsphereRadius = 0.0;
}

Geometry::~Geometry(){
	graph->geos.Remove(this);
}

///////////////////////////////////////////////////////////////////////////////////////////////////

ConvexGeometry::ConvexGeometry(Graph* g, const string& n):Geometry(g, n){

}

ConvexGeometry::~ConvexGeometry(){

}

///////////////////////////////////////////////////////////////////////////////////////////////////

Sphere::Sphere(Graph* g, real_t r, const string& n):ConvexGeometry(g, n){
	radius = r;
}

void Sphere::CalcBSphere(){
	bsphereCenter = vec3_t();
	bsphereRadius = radius;
}

void Sphere::CalcSupport(const vec3_t& dir, vec3_t& sup){
	sup = radius * dir.unit();
}

void Sphere::Draw(Render::Canvas* canvas, Render::Config* conf){
	if(conf->Set(canvas, Render::Item::Geometry, this))
		canvas->Sphere(Vec3f(), (float)radius);
}

///////////////////////////////////////////////////////////////////////////////////////////////////

Box::Box(Graph* g, const vec3_t& sz, const string& n):ConvexGeometry(g, n){
	size = sz;
}

void Box::CalcBSphere(){
	bsphereCenter = vec3_t();
	bsphereRadius = 0.5 * size.norm();
}

void Box::CalcSupport(const vec3_t& dir, vec3_t& sup){
	for(int i = 0; i < 3; i++){
		sup[i] = (dir[i] >= 0.0 ? 1.0 : -1.0) * size[i]/2.0;
	}
}

void Box::Draw(Render::Canvas* canvas, Render::Config* conf){
	if(conf->Set(canvas, Render::Item::Geometry, this)){
		canvas->Box(-0.5*size, 0.5*size);
	}
}

///////////////////////////////////////////////////////////////////////////////////////////////////

Cylinder::Cylinder(Graph* g, real_t r, real_t l, const string& n):ConvexGeometry(g, n){
	radius = r;
	length = l;
}

void Cylinder::CalcBSphere(){
	bsphereCenter = vec3_t();
	bsphereRadius = sqrt(radius * radius + 0.25 * length * length);
}

void Cylinder::CalcSupport(const vec3_t& d, vec3_t& p){
	// T.B.D.
}

void Cylinder::Draw(Render::Canvas* canvas, Render::Config* conf){
	if(conf->Set(canvas, Render::Item::Geometry, this))
		canvas->Cylinder((float)radius, (float)length);
}

///////////////////////////////////////////////////////////////////////////////////////////////////

Plane::Plane(Graph* g, const string& n):ConvexGeometry(g, n){

}

void Plane::CalcBSphere(){
	bsphereCenter.clear();
	bsphereRadius = inf;
}

void Plane::CalcSupport(const vec3_t& dir, vec3_t& sup){
	// T.B.D.
}

///////////////////////////////////////////////////////////////////////////////////////////////////

Triangle::Triangle(Graph* g, const string& n):ConvexGeometry(g, n){

}

void Triangle::CalcBSphere(){
	bsphereCenter = (1.0/3.0) * (vertices[0] + vertices[1] + vertices[2]);
	bsphereRadius = -inf;
	for(int i = 0; i < 3; i++)
		bsphereRadius = std::max(bsphereRadius, (vertices[i] - bsphereCenter).norm());
}

void Triangle::CalcSupport(const vec3_t& dir, vec3_t& sup){
	real_t dmax = -inf;
	for(int i = 0; i < 3; i++){
		real_t d = dir * vertices[i];
		if(d > dmax){
			sup  = vertices[i];
			dmax = d;
		}
	}
}

///////////////////////////////////////////////////////////////////////////////////////////////////

ConvexMesh::ConvexMesh(Graph* g, const string& n):ConvexGeometry(g, n){

}

void ConvexMesh::CalcBSphere(){
	bsphereCenter = vec3_t();
	bsphereRadius = 0.0;

	for(int i = 0; i < (int)tris.size(); i++){
		tris[i].CalcBSphere();
		bsphereCenter += tris[i].bsphereCenter;
	}
	bsphereCenter *= (1.0 / (real_t)tris.size());
	
	for(int i = 0; i < (int)tris.size(); i++){
		bsphereRadius = std::max(bsphereRadius, (bsphereCenter - tris[i].bsphereCenter).norm() + tris[i].bsphereRadius);
	}
}

void ConvexMesh::CalcSupport(const vec3_t& dir, vec3_t& sup){
	vec3_t _sup;
	real_t dmax = -inf;
	real_t _d;
	for(int i = 0; i < (int)tris.size(); i++){
		tris[i].CalcSupport(dir, _sup);
		_d = dir * _sup;
		if(_d > dmax){
			dmax = _d;
			sup  = _sup;
		}
	}
}

///////////////////////////////////////////////////////////////////////////////////////////////////

class Simplex{
public:
	vector<vec3_t> vtx;
	real_t  depth;

public:
	void Add(const vec3_t& p){
		vtx.push_back(p);
	}

	void CalcNearestFromEdge(int* iedge, const vec3_t& nedge, vec3_t& p){
		// p is already projected to this face
		// project p to this edge
		p += ((nedge*(vtx[iedge[0]] - p)) / (nedge*nedge)) * nedge;
	
		// p is on edge
		vec3_t d0 = p - vtx[iedge[0]];
		vec3_t d1 = p - vtx[iedge[1]];
		if( d0 * d1 <= 0.0 )
			return;

		if( d0.square() <= d1.square() ){
			p = vtx[iedge[0]];
			return;
		}

		p = vtx[iedge[1]];
	}

	// nearest point to origin on a face
	void CalcNearestFromFace(int* iface, const vec3_t& nface, vec3_t& p){
		// projection of origin to this face
		p += ((nface*(vtx[iface[0]] - p)) / (nface*nface)) * nface;

		int iedge[2];
		int j = 0;
		for(int ic = 0; ic < 3; ic++){
			for(int i = 0; i < 3; i++)
				if(i != ic) iedge[j++] = iface[i];

			// edge normal
			vec3_t nedge = nface % (vtx[iedge[1]] - vtx[iedge[0]]);
			if( nedge * (vtx[iface[ic]] - vtx[iedge[0]]) > 0.0 )
				nedge = -nedge;

			// if projected origin is outside of this edge
			if( nedge * (p - vtx[iedge[0]]) > 0.0 ){
				CalcNearestFromEdge(iedge, nedge, p);
				return;
			}
		}

		// projected origin is the nearest point
	}

	// nearest point to origin in simplex
	bool CalcNearestFromTetrahedron(vec3_t& p){
		depth = inf;

		int    iface[3];
		int    j = 0;
		vec3_t nface;
		for(int ic = 0; ic < 4; ic++){
			for(int i = 0; i < 4; i++)
				if(i != ic) iface[j++] = i;

			// initialize p as origin
			p.clear();
		
			// calc face normal
			nface = (vtx[iface[1]] - vtx[iface[0]]) % (vtx[iface[2]] - vtx[iface[0]]);
			if( nface * (vtx[ic] - vtx[iface[0]]) > 0.0 )
				nface = -nface;
			
			// if origin is outside of this face
			real_t dface = nface * vtx[iface[0]];
			depth = std::min(depth, dface);
			if(dface < 0.0){
				CalcNearestFromFace(iface, nface, p);
				// remove vertex not contained in this face
				vtx.erase(vtx.begin() + ic);
				return false;
			}
		}

		// origin is inside simplex
		return true;
	}

	bool CalcNearest(vec3_t& p){
		if(vtx.size() == 1){
			p = vtx[0];
			return false;
		}
		if(vtx.size() == 2){
			int iedge[] = {0, 1};
			vec3_t nface = (vtx[iedge[1]] - p) % (vtx[iedge[0]] - p);
			vec3_t nedge = nface % (vtx[iedge[1]] - vtx[iedge[0]]);
			CalcNearestFromEdge(iedge, nedge, p);
			return false;
		}
		if(vtx.size() == 3){
			int iface[] = {0, 1, 2};
			vec3_t nface = (vtx[iface[2]] - vtx[iface[0]]) % (vtx[iface[1]] - vtx[iface[0]]);
			CalcNearestFromFace(iface, nface, p);
			return false;
		}
		if(vtx.size() == 4){
			return CalcNearestFromTetrahedron(p);
		}

		// you don't come here
		return false;
	}
};

void CalcNearest(ConvexGeometry* g0, ConvexGeometry* g1, vec3_t& p0, vec3_t& p1, real_t& dist){
	const real_t eps = 1.0e-10;

	// simplex vertices
	Simplex simplex;

	// initial direction
	vec3_t dir;
	dir = (g1->bsphereCenter - g0->bsphereCenter);
	if(dir.norm() < eps)
		dir.x += eps;

	vec3_t sup[2];
	while(true){
		// calc support points
		g0->CalcSupport( dir, sup[0]);
		g1->CalcSupport(-dir, sup[1]);

		// see if we got nearest
		if(dir * (sup[0] - sup[1]) <= -dir.square() - eps){
			p0   = sup[0];
			p1   = sup[1];
			dist = dir.norm();
			break;
		}

		// add support point to simplex
		simplex.Add(sup[0] - sup[1]);

		// calculate nearest point of simplex to origin
		if(simplex.CalcNearest(dir)){
			// if simplex contains origin
			p0   = sup[0];
			p1   = sup[1];
			dist = -simplex.depth;
			break;
		}
	}
}

}
