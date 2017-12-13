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

Sphere::Sphere(Graph* g, real_t r, const string& n):Geometry(g, n){
	radius = r;
}

void Sphere::CalcBSphere(){
	bsphereCenter = vec3_t();
	bsphereRadius = radius;
}

vec3_t Sphere::CalcSupport(const vec3_t& dir){
	const real_t eps = 1.0e-10;
	real_t dnorm = dir.norm();
	if(dnorm < eps)
		return radius * vec3_t(1.0, 0.0, 0.0);

	return (radius/dnorm) * dir;
}

void Sphere::Draw(Render::Canvas* canvas, Render::Config* conf){
	if(conf->Set(canvas, Render::Item::Geometry, this))
		canvas->Sphere(Vec3f(), (float)radius);
}

///////////////////////////////////////////////////////////////////////////////////////////////////

Box::Box(Graph* g, const vec3_t& sz, const string& n):Geometry(g, n){
	size = sz;
}

void Box::CalcBSphere(){
	bsphereCenter = vec3_t();
	bsphereRadius = 0.5 * size.norm();
}

vec3_t Box::CalcSupport(const vec3_t& dir){
	vec3_t sup;
	for(int i = 0; i < 3; i++){
		sup[i] = (dir[i] >= 0.0 ? 1.0 : -1.0) * size[i]/2.0;
	}
	return sup;
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

void Cylinder::CalcBSphere(){
	bsphereCenter = vec3_t();
	bsphereRadius = sqrt(radius * radius + 0.25 * length * length);
}

vec3_t Cylinder::CalcSupport(const vec3_t& d){
	// T.B.D.
	return vec3_t();
}

void Cylinder::Draw(Render::Canvas* canvas, Render::Config* conf){
	if(conf->Set(canvas, Render::Item::Geometry, this))
		canvas->Cylinder((float)radius, (float)length);
}

///////////////////////////////////////////////////////////////////////////////////////////////////

Plane::Plane(Graph* g, const string& n):Geometry(g, n){

}

void Plane::CalcBSphere(){
	bsphereCenter.clear();
	bsphereRadius = inf;
}

vec3_t Plane::CalcSupport(const vec3_t& dir){
	// T.B.D.
	return vec3_t();
}

///////////////////////////////////////////////////////////////////////////////////////////////////

Triangle::Triangle(Graph* g, const string& n):Geometry(g, n){

}

void Triangle::CalcBSphere(){
	bsphereCenter = (1.0/3.0) * (vertices[0] + vertices[1] + vertices[2]);
	bsphereRadius = -inf;
	for(int i = 0; i < 3; i++)
		bsphereRadius = std::max(bsphereRadius, (vertices[i] - bsphereCenter).norm());
}

vec3_t Triangle::CalcSupport(const vec3_t& dir){
	vec3_t sup;
	real_t dmax = -inf;
	for(int i = 0; i < 3; i++){
		real_t d = dir * vertices[i];
		if(d > dmax){
			sup  = vertices[i];
			dmax = d;
		}
	}
	return sup;
}

///////////////////////////////////////////////////////////////////////////////////////////////////

Mesh::Mesh(Graph* g, const string& n):Geometry(g, n){

}

void Mesh::CalcBSphere(){
	bsphereCenter = vec3_t();
	bsphereRadius = 0.0;

	for(int i = 0; i < (int)tris.size(); i++){
		for(int j = 0; j < 3; j++)
			bsphereCenter += tris[i].vertices[j];
	}
	bsphereCenter *= (1.0 / (real_t)(tris.size() * 3));
	
	for(int i = 0; i < (int)tris.size(); i++){
		for(int j = 0; j < 3; j++)
			bsphereRadius = std::max(bsphereRadius, (bsphereCenter - tris[i].vertices[j]).norm());
	}
}

vec3_t Mesh::CalcSupport(const vec3_t& dir){
	vec3_t sup;
	real_t dmax = -inf;
	real_t d;
	for(int i = 0; i < (int)tris.size(); i++){
		for(int j = 0; j < 3; j++){
			d = dir * tris[i].vertices[j];
			if(d > dmax){
				dmax = d;
				sup  = tris[i].vertices[j];
			}
		}
	}
	return sup;
}

///////////////////////////////////////////////////////////////////////////////////////////////////
/* 
def func(Sd, o)
  oをSdを含む部分空間に射影 -> p
  if pがSdの内部
    if pがoに十分近い
      return pからSd-1,iへの最短距離の最小値 (交差する）
    else
      return pとoの距離
  else
    for each Sd-1,i
      call func(Sd-1, o)
    end
    return 上の最小値
end
*/

class Simplex{
public:
	struct S{
		int idx[4];
		int dim;

		S Sub(int ic){
			S sc;
			int j = 0;
			for(int i = 0; i < dim+1; i++){
				if(i != ic)
					sc.idx[j++] = idx[i];
			}
			sc.dim = dim-1;
			return sc;
		}
	};

	vector<vec3_t> vtx;
	vector<vec3_t> vtx_new;
	//vector<int>    iproj, iproj_nearest;
	//vector<int>    icmpl;
	//real_t         depth;
	//vec3_t         nearest;    //< nearest point to origin
	//vec3_t         p_proj;     //< projected origin
	//real_t         dmin;       //< minimum distance to origin
	//vec3_t         nface;      //< face normal
	//vec3_t         nedge;      //< edge normal
	//vec3_t         tedge;      //< edge tangent
	
public:
	void Add(const vec3_t& p){
		vtx.push_back(p);
	}

	//
	// S(idx, dim) : dim-dimantional simplex defined by  vtx[idx[0]],..,vtx[idx[dim-1]]
	// P(S) : subspace containing S
	//
	// project origin to P(S)
	vec3_t Project(S s){
		if(s.dim == 0){
			return vtx[s.idx[0]];
		}
		if(s.dim == 1){
			// (v0 + k(v1-v0) - p)*(v1-v0) = 0
			// k = (p-v0)*(v1-v0) / (v1-v0)*(v1-v0)
			vec3_t& v0 = vtx[s.idx[0]];
			vec3_t& v1 = vtx[s.idx[1]];
			vec3_t  d  = v1 - v0;
			real_t  k  = -(v0*d)/(d*d);
			return v0 + k*d;
		}
		if(s.dim == 2){
			// (o + s*n - v0)*n = 0
			// s = ((v0-o)*n) / (n*n)
			vec3_t& v0 = vtx[s.idx[0]];
			vec3_t& v1 = vtx[s.idx[1]];
			vec3_t& v2 = vtx[s.idx[2]];
			vec3_t  n  = (v1 - v0)%(v2 - v0);
			real_t  k  = ((n*v0)/(n*n));
			return k*n;
		}
		// dim == 3
		return vec3_t();
	}

	// p is a point in P(S)
	// decide if p is inside S
	bool IsInside(S s, const vec3_t& p){
		if(s.dim == 0){
			return true;
		}
		if(s.dim == 1){
			vec3_t& v0 = vtx[s.idx[0]];
			vec3_t& v1 = vtx[s.idx[1]];
			return (v1 - p)*(v0 - p) <= 0.0;
		}
		if(s.dim == 2){
			vec3_t& v0 = vtx[s.idx[0]];
			vec3_t& v1 = vtx[s.idx[1]];
			vec3_t& v2 = vtx[s.idx[2]];
			vec3_t  n  = (v1 - v0)%(v2 - v0);
			vec3_t  t0 = n % (v2 - v1);
			vec3_t  t1 = n % (v0 - v2);
			vec3_t  t2 = n % (v1 - v0);
			return (t0*(p-v1))*(t0*(v0-v1)) >= 0.0 &&
				   (t1*(p-v2))*(t1*(v1-v2)) >= 0.0 &&
				   (t2*(p-v0))*(t2*(v2-v0)) >= 0.0;
		}
		// dim == 3
		vec3_t& v0 = vtx[s.idx[0]];
		vec3_t& v1 = vtx[s.idx[1]];
		vec3_t& v2 = vtx[s.idx[2]];
		vec3_t& v3 = vtx[s.idx[3]];
		vec3_t  n0 = (v2 - v1)%(v3 - v1);
		vec3_t  n1 = (v3 - v2)%(v0 - v2);
		vec3_t  n2 = (v0 - v3)%(v1 - v3);
		vec3_t  n3 = (v1 - v0)%(v2 - v0);
		return (n0*(p-v1))*(n0*(v0-v1)) >= 0.0 &&
			   (n1*(p-v2))*(n1*(v1-v2)) >= 0.0 &&
			   (n2*(p-v3))*(n2*(v2-v3)) >= 0.0 &&
			   (n3*(p-v0))*(n3*(v3-v0)) >= 0.0;
	}

	// calculate nearest point on s to origin
	// smin : minimum subsimplex of s that containt the nearest point
	// pmin : nearest point
	// dmin : if origin is outside s => distance between pmin and o
	//        if origin is inside  s => negative of minimum distance between o and the boundary of S
	//
	void CalcNearest(S s, S& smin, vec3_t& pmin, real_t& dmin){
		const real_t eps = 1.0e-10;
		real_t dmin2;
		vec3_t p, p2, pmin2;
		S smin2;
			
		// project o to the subspace of s
		p = Project(s);

		if(IsInside(s, p)){
			// this simplex is the minimum simplex
			smin = s;

			real_t d = p.norm();
			// 
			if(d < eps){
				// calculate minimum distance to subsimplices
				dmin = inf;
				for(int ic = 0; ic < s.dim+1; ic++)
					dmin = std::min(dmin, Project(s.Sub(ic)).norm());
				pmin =  p;
				dmin = -dmin;
			}
			else{
				pmin = p;
				dmin = d;
			}
		}
		else{
			// call recursively for all subsimplices
			dmin = inf;
			for(int ic = 0; ic < s.dim+1; ic++){
				CalcNearest(s.Sub(ic), smin2, pmin2, dmin2);
				if(dmin2 < dmin){
					dmin = dmin2;
					pmin = pmin2;
					smin = smin2;
				}
			}
		}
	}

	void CalcNearest(vec3_t& pmin, real_t& dmin){
		S s, smin;

		s.dim = (int)vtx.size()-1;
		for(int i = 0; i < (int)vtx.size(); i++)
			s.idx[i] = i;

		CalcNearest(s, smin, pmin, dmin);

		// update to minimum simplex 
		vtx_new.clear();
		for(int i = 0; i < smin.dim+1; i++)
			vtx_new.push_back(vtx[smin.idx[i]]);
		vtx.swap(vtx_new);
	}

	//void Update(const vec3_t& p){
	//	real_t d = p.norm();
	//	if(d < dmin){
	//		nearest       = p;
	//		dmin          = d;
	//		iproj_nearest = iproj;
	//	}
	//}
	//
	//void CalcNearestFromVertex(){
	//	Update(vtx[iproj[0]]);
	//}
	//
	//void CalcNearestFromEdge(vec3_t p){
	//	// project p to a line v0-v1
	//	// (v0 + s(v1-v0) - p)*(v1-v0) = 0
	//	// s = (p-v0)*(v1-v0) / (v1-v0)*(v1-v0)
	//
	//	vec3_t& v0 = vtx[iproj[0]];
	//	vec3_t& v1 = vtx[iproj[1]];
	//	vec3_t  d  = v1 - v0;
	//	real_t s = ((p - v0)*d)/(d*d);
	//
	//	if(s < 0.0){
	//		icmpl.push_back(iproj[1]);
	//		iproj.erase(iproj.begin() + 1);
	//		Update(v0);
	//		iproj.insert(iproj.begin() + 1, icmpl.back());
	//		icmpl.pop_back();
	//	}
	//	else if(s > 1.0){
	//		icmpl.push_back(iproj[0]);
	//		iproj.erase(iproj.begin() + 0);
	//		Update(v1);
	//		iproj.insert(iproj.begin() + 0, icmpl.back());
	//		icmpl.pop_back();
	//	}
	//	else{
	//		Update(v0 + s*d);
	//	}
	//
	//	/*
	//	p += ((nedge*(vtx[iproj[0]] - p)) / (nedge*nedge)) * nedge;
	//
	//	bool inside = true;
	//	for(int ic = 0; ic < 2; ic++){
	//		icmpl.push_back(iproj[ic]);
	//		iproj.erase(iproj.begin() + ic);
	//
	//		// edge tangent
	//		tedge = nface % nedge;
	//		if( tedge * (vtx[icmpl.back()] - vtx[iproj[0]]) > 0.0 )
	//			tedge = -tedge;
	//
	//		// if projected origin is outside of this vertex
	//		if( tedge * (p - vtx[iproj[0]]) > 0.0 ){
	//			CalcNearestFromVertex();
	//			inside = false;
	//		}
	//
	//		iproj.insert(iproj.begin() + ic, icmpl.back());
	//		icmpl.pop_back();
	//	}
	//
	//	if(inside)
	//		Update(p);
	//	*/
	//}
	//
	//// nearest point to origin on a face
	//void CalcNearestFromFace(vec3_t p){
	//	// projection of origin to this face
	//	p += ((nface*(vtx[iproj[0]] - p)) / (nface*nface)) * nface;
	//
	//	bool inside = true;
	//	for(int ic = 0; ic < 3; ic++){
	//		icmpl.push_back(iproj[ic]);
	//		iproj.erase(iproj.begin() + ic);
	//
	//		// edge normal
	//		nedge = nface % (vtx[iproj[1]] - vtx[iproj[0]]);
	//		if( nedge * (vtx[icmpl.back()] - vtx[iproj[0]]) > 0.0 )
	//			nedge = -nedge;
	//
	//		// if projected origin is outside of this edge
	//		if( nedge * (p - vtx[iproj[0]]) > 0.0 ){
	//			CalcNearestFromEdge(p);
	//			inside = false;
	//		}
	//
	//		iproj.insert(iproj.begin() + ic, icmpl.back());
	//		icmpl.pop_back();
	//	}
	//
	//	if(inside)
	//		Update(p);
	//}
	//
	//// nearest point to origin in simplex
	//void CalcNearestFromTetrahedron(vec3_t p){
	//	depth = -inf;
	//	
	//	bool inside = true;
	//	for(int ic = 0; ic < 4; ic++){
	//		icmpl.push_back(iproj[ic]);
	//		iproj.erase(iproj.begin() + ic);
	//		
	//		// calc face normal
	//		nface = (vtx[iproj[1]] - vtx[iproj[0]]) % (vtx[iproj[2]] - vtx[iproj[0]]);
	//		if( nface * (vtx[icmpl.back()] - vtx[iproj[0]]) > 0.0 )
	//			nface = -nface;
	//		
	//		// if origin is outside of this face
	//		real_t dface = nface * (p - vtx[iproj[0]]);
	//		if( dface > 0.0 ){
	//			CalcNearestFromFace(p);
	//			inside = false;
	//		}
	//
	//		depth = std::max(depth, dface/nface.norm());
	//		
	//		iproj.insert(iproj.begin() + ic, icmpl.back());
	//		icmpl.pop_back();
	//	}
	//}
	//
	//// calculate nearest point to origin
	//bool CalcNearest(){
	//	vec3_t p;
	//	dmin = inf;
	//	
	//	if(vtx.size() == 1){
	//		icmpl.resize(0);
	//		iproj.resize(1);
	//		iproj[0] = 0;
	//		CalcNearestFromVertex();
	//	}
	//	if(vtx.size() == 2){
	//		icmpl.resize(0);
	//		iproj.resize(2);
	//		iproj[0] = 0;
	//		iproj[1] = 1;
	//		nface = vtx[iproj[1]] % vtx[iproj[0]];
	//		nedge = nface % (vtx[iproj[1]] - vtx[iproj[0]]);
	//		CalcNearestFromEdge(p);
	//	}
	//	if(vtx.size() == 3){
	//		icmpl.resize(0);
	//		iproj.resize(3);
	//		iproj[0] = 0;
	//		iproj[1] = 1;
	//		iproj[2] = 2;
	//		nface = (vtx[iproj[2]] - vtx[iproj[0]]) % (vtx[iproj[1]] - vtx[iproj[0]]);
	//		CalcNearestFromFace(p);
	//	}
	//	if(vtx.size() == 4){
	//		icmpl.resize(0);
	//		iproj.resize(4);
	//		iproj[0] = 0;
	//		iproj[1] = 1;
	//		iproj[2] = 2;
	//		iproj[3] = 3;
	//		CalcNearestFromTetrahedron(p);
	//	}
	//
	//	if(vtx.size() == 4 && iproj.size() == 4)
	//		return true;
	//
	//	// remove vertices that do not span the convex hull containing the projected origin
	//	vtx_new.clear();
	//	for(int i = 0; i < iproj_nearest.size(); i++)
	//		vtx_new.push_back(vtx[iproj_nearest[i]]);
	//	vtx.swap(vtx_new);
	//
	//	return false;
	//}
};

void CalcNearest(Geometry* g0, Geometry* g1, const pose_t& pose0, const pose_t& pose1, vec3_t& sup0, vec3_t& sup1, real_t& dist){
	const real_t eps = 1.0e-10;

	// simplex vertices
	Simplex simplex;

	// initial direction
	vec3_t dir;
	dir = (pose1 * g1->bsphereCenter - pose0 * g0->bsphereCenter);
	if(dir.norm() < eps)
		dir.x += eps;

	vec3_t dir0, dir1;
	while(true){
		// calc support points
		dir0 = pose0.Ori().Conjugated() * dir;
		dir1 = pose1.Ori().Conjugated() * dir;
		sup0 = pose0 * g0->CalcSupport( dir0);
		sup1 = pose1 * g1->CalcSupport(-dir1);
		
		// see if we got nearest
		real_t lhs = dir * (sup0 - sup1);
		real_t rhs = -dir.square();
		if(lhs <= rhs + eps){
			dist = dir.norm();
			break;
		}

		// add support point to simplex
		simplex.Add(sup0 - sup1);

		// calculate nearest point of simplex to origin
		vec3_t pmin;
		real_t dmin = 0.0;
		simplex.CalcNearest(pmin, dmin);
		if(dmin < 0.0){
			// if simplex contains origin
			dist = dmin;
			break;
		}
		dir = -pmin;
	}
}

}
