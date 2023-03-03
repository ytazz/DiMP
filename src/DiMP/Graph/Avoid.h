#pragma once

#include <DiMP/Graph/Task.h>
#include <DiMP/Graph/Octtree.h>
//#include <DiMP/Solver/Constraint.h>

/** Collision avoidance
	- 2つの剛体間の距離が一定値以下にならないようにする
 */

namespace DiMP{;

class Connector;
class Geometry;
class GeometryInfo;
class GeometryPair;
class AvoidTask;
class AvoidConP;
class AvoidConV;

class AvoidKey : public TaskKey{
public:
	GeometryPairs geoPairs;

	vector<AvoidConP*>  con_p;
	vector<AvoidConV*>  con_v;

	vector<GeometryPair*>  gpactive;

public:
	virtual void AddCon(Solver* solver);
	virtual void Prepare();
	virtual void Draw   (Render::Canvas* canvas, Render::Config* conf);	

	AvoidKey();
};

class AvoidTask : public Task{
public:
	struct Param{
		bool	avoid_p;
		bool	avoid_v;
		real_t	dmin;		  ///< 最小接近距離
		int     nactive_max;  ///< maximum number of simultaneously active collision avoidance constraints
	
		Param();
	} param;
	
public:
	virtual Keypoint*	CreateKeypoint(){ return new AvoidKey(); }
	virtual void Prepare();
	
	 AvoidTask(Object* _obj0, Object* _obj1, TimeSlot* _time, const string& n);
	~AvoidTask();
};

class AvoidCon : public Constraint{
public:
	AvoidKey*  key;
	GeometryPair* gp;

	AvoidCon(Solver* solver, ID id, AvoidKey* _key, GeometryPair* _gp, real_t _scale);
	virtual ~AvoidCon(){}
};

class AvoidConP : public AvoidCon{
public:
	virtual void CalcCoef();
	virtual void CalcDeviation();
	virtual void Project(real_t& l, uint k);

	AvoidConP(Solver* solver, const string& _name, AvoidKey* _key, GeometryPair* _gp, real_t _scale);
	virtual ~AvoidConP(){}
};

class AvoidConV : public AvoidCon{
public:
	virtual void CalcCoef();
	virtual void CalcDeviation();
	virtual void Project(real_t& l, uint k);

	AvoidConV(Solver* solver, const string& _name, AvoidKey* _key, GeometryPair* _gp, real_t _scale);
	virtual ~AvoidConV(){}
};

}
