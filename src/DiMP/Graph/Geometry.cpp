#include <DiMP/Graph/Geometry.h>
#include <DiMP/Graph/Graph.h>
#include <DiMP/Render/Config.h>
#include <DiMP/Render/Canvas.h>

#ifdef _WIN32
# include <mkl_lapacke.h>
#else
# include <lapacke.h>
#endif

namespace DiMP{;

const real_t inf = numeric_limits<real_t>::max();
const real_t pi  = M_PI;

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

vec3_t Cylinder::CalcSupport(const vec3_t& dir){
	const real_t eps = 1.0e-10;
	real_t dnorm = sqrt(dir.x*dir.x + dir.y*dir.y);
	if(dnorm < eps){
		return vec3_t(0.0, 0.0, (dir.z == 0.0 ? 0.0 : (dir.z > 0.0 ? +1.0 : -1.0)*(0.5*length)));
	}

	return vec3_t((radius/dnorm) * dir.x, (radius/dnorm) * dir.y, (dir.z == 0.0 ? 0.0 : (dir.z > 0.0 ? +1.0 : -1.0)*(0.5*length)) );
}

void Cylinder::Draw(Render::Canvas* canvas, Render::Config* conf){
	if(conf->Set(canvas, Render::Item::Geometry, this))
		canvas->Cylinder((float)radius, (float)length);
}

///////////////////////////////////////////////////////////////////////////////////////////////////

Capsule::Capsule(Graph* g, real_t r, real_t l, const string& n):Geometry(g, n){
	radius = r;
	length = l;
}

void Capsule::CalcBSphere(){
	bsphereCenter = vec3_t();
	bsphereRadius = sqrt(radius + 0.5 * length);
}

vec3_t Capsule::CalcSupport(const vec3_t& dir){
	const real_t eps = 1.0e-10;
	real_t dnorm = dir.norm();
	if(dnorm < eps)
		return vec3_t(radius, 0.0, 0.0);

	return (radius/dnorm) * dir + vec3_t(0.0, 0.0, (dir.z > 0.0 ? +1.0 : -1.0)*(0.5*length));
}

void Capsule::Draw(Render::Canvas* canvas, Render::Config* conf){
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

void TriangleBase::Draw(Render::Canvas* canvas){
	Vec3f p0, p1;
	for(int i = 0; i < 3; i++){
		p0 = vertices[(i+0)%3];
		p1 = vertices[(i+1)%3];
		canvas->Line(p0, p1);
	}
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

void Triangle::Draw(Render::Canvas* canvas, Render::Config* conf){
	//if(conf->Set(canvas, Render::Item::Geometry, this)){
	//	TriangleBase::Draw(canvas);
	//}
}

///////////////////////////////////////////////////////////////////////////////////////////////////

Mesh::Mesh(Graph* g, const string& n):Geometry(g, n){
	ntheta = 50;
	nphi   = 50;
}

void Mesh::CreateSupportMap(){
	real_t theta, phi, cosphi;
	vec3_t dir;
	real_t d, dmax;
	int    itri_max, iver_max;

	supportMap.resize(ntheta * nphi);
	for(int itheta = 0; itheta < ntheta; itheta++){
		for(int iphi = 0; iphi < nphi; iphi++){
			theta = 2.0*pi*((real_t)itheta / (real_t)ntheta) - pi;
			phi   = 2.0*pi*((real_t)iphi   / (real_t)nphi  ) - pi;
			cosphi = cos(phi);
			dir   = vec3_t(cosphi * cos(theta), cosphi * sin(theta), sin(phi));

			dmax = -inf;
			for(int itri = 0; itri < (int)tris.size(); itri++){
				for(int iver = 0; iver < 3; iver++){
					d = dir * tris[itri].vertices[iver];
					if(d > dmax){
						dmax = d;
						itri_max = itri;
						iver_max = iver;
					}
				}
			}
			supportMap[nphi * itheta + iphi] = make_pair(itri_max, iver_max);
		}
	}
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
	if(supportMap.empty())
		CreateSupportMap();

	real_t theta = atan2(dir.y, dir.x);
	real_t phi   = atan2(dir.z, sqrt(dir.x*dir.x + dir.y*dir.y));
	int itheta = std::min(std::max( 0, (int)(ntheta * (theta + pi)/(2*pi)) ), ntheta-1);
	int iphi   = std::min(std::max( 0, (int)(nphi   * (phi   + pi)/(2*pi)) ), nphi  -1);
	pair<int,int> sup = supportMap[nphi * itheta + iphi];
	return tris[sup.first].vertices[sup.second];
}

void Mesh::Draw(Render::Canvas* canvas, Render::Config* conf){
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
	// element of minkowski difference space
	typedef PTM::TVector<9, double> vec9_t;
	typedef PTM::TMatrixCol<3, 2, double> mat32_t;

	struct VP : vec9_t{
		vec3_t& p(){ return *((vec3_t*)this + 0); }
		vec3_t& q(){ return *((vec3_t*)this + 1); }
		vec3_t& d(){ return *((vec3_t*)this + 2); }
		
		const vec3_t& p()const{return *((const vec3_t*)this + 0);}
		const vec3_t& q()const{return *((const vec3_t*)this + 1);}
		const vec3_t& d()const{return *((const vec3_t*)this + 2);}

		VP& operator=(const vec9_t& v){
			(vec9_t&)*this = v;
			return *this;
		}

		VP(){}
		VP(const vec9_t& v){
			*this = v;
		}
		VP(const vec3_t& _p, const vec3_t& _q){
			p() = _p;
			q() = _q;
			d() = _p - _q;
		}
	};
	
	// simplex in minkowski difference space
	struct S{
		int idx[4]; //< vertex indices
		int dim;    //< dimension of subspace i.e. number of vertices - 1

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

	vector<VP> vtx;
	vector<VP> vtx_new;
	
public:
	void Add(const vec3_t& p, const vec3_t& q){
		vtx.push_back(VP(p, q));
	}

	//
	// S(idx, dim) : dim-dimantional simplex defined by  vtx[idx[0]],..,vtx[idx[dim-1]]
	// P(S) : subspace containing S
	//
	// project origin to P(S)
	void Project(const S& s, VP& p){
		if(s.dim == 0){
			p = vtx[s.idx[0]];
			return;
		}
		if(s.dim == 1){
			// (v0 + k(v1-v0) - p)*(v1-v0) = 0
			// k = (p-v0)*(v1-v0) / (v1-v0)*(v1-v0)
			VP&     v0 = vtx[s.idx[0]];
			VP&     v1 = vtx[s.idx[1]];
			VP      d  = v1 - v0;
			real_t  k  = -(v0.d()*d.d())/(d.d().square());
			p = v0 + k * d;
			return;
		}
		if(s.dim == 2){
			//// (o + s*n - v0)*n = 0
			//// s = ((v0-o)*n) / (n*n)
			// d1 = v1 - v0
			// d2 = v2 - v0
			// (v0 + k1*d1 + k2*d2)*d1 = 0
			// (v0 + k1*d1 + k2*d2)*d2 = 0
			// [d1*d1 d1*d2][k1] = -[v0*d1]
			// [d1*d2 d2*d2][k2]   -[v0*d2]
			VP& v0 = vtx[s.idx[0]];
			VP& v1 = vtx[s.idx[1]];
			VP& v2 = vtx[s.idx[2]];
			VP  d1 = v1 - v0;
			VP  d2 = v2 - v0;
			mat3_t A;
			vec3_t b;
			A.col(0) = d1.d();
			A.col(1) = d2.d();
			b        = v0.d();
			//vec2_t k = -(A.trans()*A).inv() * A.trans() * b;
			LAPACKE_dgels(LAPACK_COL_MAJOR, 'N', 3, 2, 1, &A[0][0], 3, &b[0], 3);
			vec3_t k = -b;
			p = v0 + k[0]*d1 + k[1]*d2;
			return;
			//vec3_t  n  = (v1 - v0)%(v2 - v0);
			//real_t  k  = ((n*v0)/(n*n));
			//return k*n;
		}
		// dim == 3
		{
			// d1 = v1 - v0
			// d2 = v2 - v0
			// d3 = v3 - v0
			// v0 + k1*d1 + k2*d2 + k3*d3 = 0
			// [d1 d2 d3][k] = -v0
			VP& v0 = vtx[s.idx[0]];
			VP& v1 = vtx[s.idx[1]];
			VP& v2 = vtx[s.idx[2]];
			VP& v3 = vtx[s.idx[3]];
			VP  d1 = v1 - v0;
			VP  d2 = v2 - v0;
			VP  d3 = v3 - v0;
			mat3_t A;
			vec3_t b;
			A.col(0) = d1.d();
			A.col(1) = d2.d();
			A.col(2) = d3.d();
			b        = v0.d();
			//vec3_t k = -(A.trans()*A).inv() * A.trans() * b;
			LAPACKE_dgels(LAPACK_COL_MAJOR, 'N', 3, 3, 1, &A[0][0], 3, &b[0], 3);
			vec3_t k = -b;
			p = v0 + k[0]*d1 + k[1]*d2 + k[2]*d3;
		}
		//return vec3_t();
	}

	// p is a point in P(S)
	// decide if p is inside S
	bool IsInside(S s, const VP& p){
		if(s.dim == 0){
			return true;
		}
		if(s.dim == 1){
			VP& v0 = vtx[s.idx[0]];
			VP& v1 = vtx[s.idx[1]];
			return (v1.d() - p.d())*(v0.d() - p.d()) <= 0.0;
		}
		if(s.dim == 2){
			VP& v0 = vtx[s.idx[0]];
			VP& v1 = vtx[s.idx[1]];
			VP& v2 = vtx[s.idx[2]];
			vec3_t  n  = (v1.d() - v0.d())%(v2.d() - v0.d());
			vec3_t  t0 = n % (v2.d() - v1.d());
			vec3_t  t1 = n % (v0.d() - v2.d());
			vec3_t  t2 = n % (v1.d() - v0.d());
			return (t0*(p.d() - v1.d()))*(t0*(v0.d() - v1.d())) >= 0.0 &&
				   (t1*(p.d() - v2.d()))*(t1*(v1.d() - v2.d())) >= 0.0 &&
				   (t2*(p.d() - v0.d()))*(t2*(v2.d() - v0.d())) >= 0.0;
		}
		// dim == 3
		{
			VP& v0    = vtx[s.idx[0]];
			VP& v1    = vtx[s.idx[1]];
			VP& v2    = vtx[s.idx[2]];
			VP& v3    = vtx[s.idx[3]];
			vec3_t n0 = (v2.d() - v1.d())%(v3.d() - v1.d());
			vec3_t n1 = (v3.d() - v2.d())%(v0.d() - v2.d());
			vec3_t n2 = (v0.d() - v3.d())%(v1.d() - v3.d());
			vec3_t n3 = (v1.d() - v0.d())%(v2.d() - v0.d());
			return (n0*(p.d() - v1.d()))*(n0*(v0.d() - v1.d())) >= 0.0 &&
				   (n1*(p.d() - v2.d()))*(n1*(v1.d() - v2.d())) >= 0.0 &&
				   (n2*(p.d() - v3.d()))*(n2*(v2.d() - v3.d())) >= 0.0 &&
				   (n3*(p.d() - v0.d()))*(n3*(v3.d() - v0.d())) >= 0.0;
		}
	}

	// calculate nearest point on s to origin
	// smin : minimum subsimplex of s that containt the nearest point
	// pmin : nearest point
	// dmin : if origin is outside s => distance between pmin and o
	//        if origin is inside  s => negative of minimum distance between o and the boundary of S
	//
	void CalcNearest(S s, S& smin, VP& pmin, real_t& dmin){
		const real_t eps = 1.0e-10;
		real_t d2, dmin2;
		VP p, p2, pmin2;
		S smin2;
			
		// project origin to the subspace of s
		Project(s, p);

		if(IsInside(s, p)){
			// this simplex is the minimum simplex
			smin = s;

			real_t d = p.d().norm();
			
			// projected point is close enough to origin
			if(d < eps){
				// calculate minimum distance to subsimplices
				dmin2 = inf;
				for(int ic = 0; ic < s.dim+1; ic++){
					Project(s.Sub(ic), p2);
					d2 = p2.d().norm();
					if(d2 < dmin2){
						pmin2 = p2;
						dmin2 = d2;
					}
				}
				pmin =  p2;
				dmin = -dmin2;
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

	void CalcNearest(VP& pmin, real_t& dmin){
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
};

void CalcNearest(Geometry* g0, Geometry* g1, const pose_t& pose0, const pose_t& pose1, vec3_t& sup0, vec3_t& sup1, real_t& dist){
	const real_t eps = 1.0e-10;

	// simplex vertices
	Simplex simplex;
	Simplex::VP pmin;
		
	// initial direction
	if(dist == inf)
		 pmin = Simplex::VP(pose0 * g0->bsphereCenter, pose1 * g1->bsphereCenter);
	else pmin = Simplex::VP(sup0, sup1);

	if(pmin.d().norm() < eps)
		pmin.d().x += eps;

	vec3_t dir, dir0, dir1;
	int niter = 0;
	while(true){
		// calc support points
		dir  = -1.0 * pmin.d();
		dir0 = pose0.Ori().Conjugated() * dir;
		dir1 = pose1.Ori().Conjugated() * dir;
		sup0 = pose0 * g0->CalcSupport( dir0);
		sup1 = pose1 * g1->CalcSupport(-dir1);
		
		// see if we got nearest
		real_t lhs = dir * (sup0 - sup1);
		real_t rhs = -dir.square();
		if(lhs <= rhs + eps){
			sup0 = pmin.p();
			sup1 = pmin.q();
			dist = pmin.d().norm();
			break;
		}

		// add support point to simplex
		simplex.Add(sup0, sup1);

		// calculate nearest point of simplex to origin
		real_t      dmin = 0.0;
		simplex.CalcNearest(pmin, dmin);
		if(dmin < 0.0){
			// if simplex contains origin
			sup0 = pmin.p();
			sup1 = pmin.q();
			dist = dmin;
			break;
		}
		dir = -pmin.d();
		niter++;
	}
	//DSTR << "niter: " << niter << endl;
}

}
