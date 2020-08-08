#pragma once

#include <DiMP/Graph/Node.h>

namespace DiMP{;

class Connector;
class Geometry;

class GeometryInfo{
public:
	Tick*         tick;
	Connector*    con;
	Geometry*     geo;
	int           geoIndex[2];   ///< index to GPTable  used for broad phase collision detection
	pose_t        poseAbs;
	vec3_t        bsphereCenterAbs;
	vec3_t        bbmin;
	vec3_t        bbmax;

	GeometryInfo(Tick* _tick, Connector* _con, Geometry* _geo){
		tick        = _tick;
		con         = _con;
		geo         = _geo;
		geoIndex[0] = -1;
		geoIndex[1] = -1;
	}
};

class EdgeInfo{
public:
	GeometryInfo*  geoInfo;
	real_t         val;
	bool           side;       //< 0:min  1:max

	EdgeInfo(){}
	EdgeInfo(GeometryInfo* _geoInfo, bool _side){
		geoInfo = _geoInfo;
		side    = _side;
	}
};

inline bool operator<(const EdgeInfo& e0, const EdgeInfo& e1){
	return
		(e0.val <  e1.val) || 
		(e0.val == e1.val && e0.geoInfo < e1.geoInfo) ||
		(e0.val == e1.val && e0.geoInfo == e1.geoInfo && e0.side < e1.side);  //< make sure open edge comes before close edge when values are the same
}

class GeometryPair : public UTRefCount{
public:
	GeometryInfo* info0;
	GeometryInfo* info1;
	real_t        dmin;         //< distance lower bound 
	real_t        dmax;         //< distance upper bound
	real_t        dist;         //< distance between objects: dist > 0 if apart, dist < 0 if intersect
	vec3_t        sup0;         //< nearest point
	vec3_t        sup1;
	vec3_t        normal;       //< contact normal
	bool          cullOcttree;  //< culled by octtree
	bool          cullSphere;   //< culled by bsphere
	bool          cullBox;      //< culled by bbox
	bool          cullGjk;      //< culled by gjk

	GeometryPair();
};

inline bool operator<(const GeometryPair& gp0, const GeometryPair& gp1){
	return (gp0.info0 <  gp1.info0) ||
		   (gp0.info0 == gp1.info0 && gp0.info1 < gp1.info1);
}

typedef vector<GeometryInfo> GeometryInfos;
typedef vector<EdgeInfo>     EdgeInfos;
typedef vector<GeometryPair> GeometryPairs;

//void ExtractGeometryPairs(
//	const GeometryInfos& geoInfos0, const EdgeInfos& edgeInfos0,
//	const GeometryInfos& geoInfos1, const EdgeInfos& edgeInfos1,
//	GeometryPairs& geoPairs);

/*
class OcttreeNode : public UTRefCount{
public:
	static int maxDepth;

	vec3_t  bbmin;
	vec3_t  bbmid;
	vec3_t  bbmax;
	int     depth;
	int     id;

	vector<GeometryInfo*>  geoInfos;
	UTRef<OcttreeNode>     children[2][2][2];

public:
	void          Init   (int depth);
	void          Assign (vector<GeometryInfo*>&  _geoInfos);
	OcttreeNode*  GetNode(const vec3_t& _min, const vec3_t& _max);
	bool          IsAncestor(const OcttreeNode* n);

};
*/

}

