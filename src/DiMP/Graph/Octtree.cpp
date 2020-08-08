#include <DiMP/Graph/Octtree.h>
#include <DiMP/Graph/Object.h>

#include <set>
using namespace std;

namespace DiMP{;

const real_t inf = numeric_limits<real_t>::max();

///////////////////////////////////////////////////////////////////////////////////////////////////

GeometryPair::GeometryPair(){
	info0 = 0;
	info1 = 0;
	dmin  = inf; 
	dmax  = inf;
	dist  = inf;
}

///////////////////////////////////////////////////////////////////////////////////////////////////

//int OcttreeNode::maxDepth = 8;
/*
void OcttreeNode::Init(int depth){
	bbmid = (bbmin + bbmax)/2.0;
	if(depth == 0)
		return;

	for(int ix = 0; ix < 2; ix++)for(int iy = 0; iy < 2; iy++)for(int iz = 0; iz < 2; iz++){
		OcttreeNode* c = new OcttreeNode();
		c->bbmin[0] = (ix == 0 ? bbmin[0] : bbmid[0]);
		c->bbmin[1] = (iy == 0 ? bbmin[1] : bbmid[1]);
		c->bbmin[2] = (iz == 0 ? bbmin[2] : bbmid[2]);
		c->bbmax[0] = (ix == 0 ? bbmid[0] : bbmax[0]);
		c->bbmax[1] = (iy == 0 ? bbmid[1] : bbmax[1]);
		c->bbmax[2] = (iz == 0 ? bbmid[2] : bbmax[2]);
		c->depth = depth + 1;
		c->id    = (ix << (3*depth+0)) | ((iy << (3*depth+1)) | (iz << (3*depth+2))) | id;
		children[ix][iy][iz] = c;
		c->Init(depth - 1);
	}
}

void OcttreeNode::Assign(vector<GeometryInfo*>& reg){
	vector<GeometryInfo*> tmp[2][2][2];

	geoInfos.clear();
	for(GeometryInfo* info : reg){
		int side[3];
		for(int i = 0; i < 3; i++){
			// 0 -> negative side, 1 -> positive side
			side[i] = (info->bbmax[i] <= bbmid[i] ? 0 : (bbmid[i] <= info->bbmin[i] ? 1 : -1));
		}

		// crosses the border -> assign to this node
		if(side[0] == -1 || side[1] == -1 || side[2] == -1){
			geoInfos.push_back(info);
			continue;
		}
		else{
			tmp[side[0]][side[1]][side[2]].push_back(info);
		}
	}

	for(int ix = 0; ix < 2; ix++)for(int iy = 0; iy < 2; iy++)for(int iz = 0; iz < 2; iz++){
		if( !tmp[ix][iy][iz].empty() ){
			if( depth == OcttreeNode::maxDepth ){
				geoInfos.insert(geoInfos.end(), tmp[ix][iy][iz].begin(), tmp[ix][iy][iz].end());
			}
			else{
				if( !children[ix][iy][iz] ){
					OcttreeNode* c = new OcttreeNode();
					c->bbmin[0] = (ix == 0 ? bbmin[0] : bbmid[0]);
					c->bbmin[1] = (iy == 0 ? bbmin[1] : bbmid[1]);
					c->bbmin[2] = (iz == 0 ? bbmin[2] : bbmid[2]);
					c->bbmax[0] = (ix == 0 ? bbmid[0] : bbmax[0]);
					c->bbmax[1] = (iy == 0 ? bbmid[1] : bbmax[1]);
					c->bbmax[2] = (iz == 0 ? bbmid[2] : bbmax[2]);
					c->depth = depth + 1;
					c->id    = (ix << (3*depth+0)) | ((iy << (3*depth+1)) | (iz << (3*depth+2))) | id;
					children[ix][iy][iz] = c;
				}
				children[ix][iy][iz]->Assign(tmp[ix][iy][iz]);
			}
		}
	}
}*/
/*
OcttreeNode* OcttreeNode::GetNode(const vec3_t& _min, const vec3_t& _max){
	int side[3];
	for(int i = 0; i < 3; i++){
		// crosses the border -> return this node
		if(_min[i] < bbmid[i] && bbmid[i] < _max[i])
			return this;

		// 0 -> negative side, 1 -> positive side
		side[i] = (_max[i] <= bbmid[i] ? 0 : (bbmid[i] <= _min[i] ? 1 : -1));
	}

	return children[side[0]][side[1]][side[2]]->GetNode(_min, _max);

}

bool OcttreeNode::IsAncestor(const OcttreeNode* n){
	int d = std::min(depth, n->depth);
	if(d == 0)
		return true;

	int mask = (0x1 << (3*d)) - 1;  //< e.g. d == 2  -> mask = 0b111111

	return (id&mask) == (n->id&mask);
}
*/
void ExtractGeometryPairs(
	const GeometryInfos& geoInfos0, const EdgeInfos& edgeInfos0,
	const GeometryInfos& geoInfos1, const EdgeInfos& edgeInfos1,
	GeometryPairs& geoPairs)
{
	if(geoInfos0.empty() || geoInfos1.empty())
		return;

	Object* obj0 = geoInfos0[0].con->obj;
	Object* obj1 = geoInfos1[0].con->obj;

	static vector<EdgeInfo> tmp;

	tmp.resize(edgeInfos0.size() + edgeInfos1.size());
	std::merge(
		edgeInfos0.begin(), edgeInfos0.end(),
		edgeInfos1.begin(), edgeInfos1.end(),
		tmp.begin());

	set<GeometryInfo*> queue0;
	set<GeometryInfo*> queue1;
	
	geoPairs.clear();

	for(vector<EdgeInfo>::iterator it = tmp.begin(); it != tmp.end(); it++){
		EdgeInfo& e = *it;
		//DSTR << e.val << " " << e.side << endl;

		GeometryPair gp;
		if(e.side == 0){
			if(e.geoInfo->con->obj == obj0){
				for(GeometryInfo* g1 : queue1){
					gp.info0 = e.geoInfo;
					gp.info1 = g1;
					geoPairs.push_back(gp);
				}
				queue0.insert(e.geoInfo);
			}
			else{
				for(GeometryInfo* g0 : queue0){
					gp.info0 = g0;
					gp.info1 = e.geoInfo;
					geoPairs.push_back(gp);
				}
				queue1.insert(e.geoInfo);
			}
		}
		else{
			if(e.geoInfo->con->obj == obj0){
				queue0.erase(e.geoInfo);
			}
			else{
				queue1.erase(e.geoInfo);
			}
		}
	}

}

///////////////////////////////////////////////////////////////////////////////////////////////////

}
