#pragma

#include <DiMP/Graph/Task.h>
//#include <DiMP/Solver/Constraint.h>

/** Collision avoidance
	- 2‚Â‚Ì„‘ÌŠÔ‚Ì‹——£‚ªˆê’è’lˆÈ‰º‚É‚È‚ç‚È‚¢‚æ‚¤‚É‚·‚é
 */

namespace DiMP{;

class Connector;
class Geometry;
class GeometryInfo;
class AvoidTask;
class AvoidConP;
class AvoidConV;

class AvoidKey : public TaskKey{
public:
	struct GeometryPair{
		GeometryInfo* info0;
		GeometryInfo* info1;
		real_t        dmin;   //< distance lower bound 
		real_t        dmax;   //< distance upper bound
		real_t        dist;   //< distance between objects: dist > 0 if apart, dist < 0 if intersect
		vec3_t        sup0;   //< nearest point
		vec3_t        sup1;
		vec3_t        normal; //< contact normal
		
		AvoidConP*	  con_p;
		AvoidConV*	  con_v;

		GeometryPair();
	};
	typedef vector<GeometryPair> GeometryPairs;
	GeometryPairs geoPairs;

public:
	virtual void AddCon(Solver* solver);
	virtual void Prepare();
	virtual void Draw   (Render::Canvas* canvas, Render::Config* conf);	

	AvoidKey();
};

class AvoidTask : public Task{
public:
	real_t	dmin;		///< Å¬Ú‹ß‹——£

public:
	virtual Keypoint*	CreateKeypoint(){ return new AvoidKey(); }
	virtual void Prepare();
	
	AvoidTask(Object* _obj0, Object* _obj1, TimeSlot* _time, const string& n);

};

class AvoidCon : public Constraint{
public:
	AvoidKey*  key;
	AvoidKey::GeometryPair* gp;

	AvoidCon(Solver* solver, ID id, AvoidKey* _key, AvoidKey::GeometryPair* _gp, real_t _scale);
	virtual ~AvoidCon(){}
};

class AvoidConP : public AvoidCon{
public:
	virtual void CalcCoef();
	virtual void CalcDeviation();
	virtual void Project(real_t& l, uint k);

	AvoidConP(Solver* solver, const string& _name, AvoidKey* _key, AvoidKey::GeometryPair* _gp, real_t _scale);
	virtual ~AvoidConP(){}
};

class AvoidConV : public AvoidCon{
public:
	virtual void CalcCoef();
	virtual void CalcDeviation();
	virtual void Project(real_t& l, uint k);

	AvoidConV(Solver* solver, const string& _name, AvoidKey* _key, AvoidKey::GeometryPair* _gp, real_t _scale);
	virtual ~AvoidConV(){}
};

}
