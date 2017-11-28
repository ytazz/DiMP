#pragma

#include <DiMP/Graph/Task.h>
//#include <DiMP/Solver/Constraint.h>

/** Collision avoidance
	- 2‚Â‚Ì„‘ÌŠÔ‚Ì‹——£‚ªˆê’è’lˆÈ‰º‚É‚È‚ç‚È‚¢‚æ‚¤‚É‚·‚é
 */

namespace DiMP{;

class AvoidTask;
class AvoidConP;
class AvoidConV;

class AvoidKey : public TaskKey{
public:
	ObjectKey*  obj[2];

	AvoidConP*	con_p;
	AvoidConV*	con_v;

	vec3_t prox0, prox1;
	vec3_t normal;
	real_t depth;

public:
	virtual void AddCon(Solver* solver);
	virtual void Prepare();

	AvoidKey();
};

class AvoidTask : public Task{
public:
	real_t	dmin;		///< Å¬Ú‹ß‹——£

public:
	virtual Keypoint*	CreateKeypoint(){ return new AvoidKey(); }
	virtual void Prepare();
	
	AvoidTask(Connector* _con0, Connector* _con1, TimeSlot* _time, const string& n);

};

class AvoidCon : public Constraint{
public:
	AvoidKey*	key;

	AvoidCon(Solver* solver, ID id, AvoidKey* _key, real_t _scale);
	virtual ~AvoidCon(){}
};

class AvoidConP : public AvoidCon{
public:
	virtual void CalcCoef();
	virtual void CalcDeviation();
	virtual void Project(real_t& l, uint k);

	AvoidConP(Solver* solver, const string& _name, AvoidKey* _key, real_t _scale);
	virtual ~AvoidConP(){}
};

class AvoidConV : public AvoidCon{
public:
	virtual void CalcCoef();
	virtual void CalcDeviation();
	virtual void Project(real_t& l, uint k);

	AvoidConV(Solver* solver, const string& _name, AvoidKey* _key, real_t _scale);
	virtual ~AvoidConV(){}
};

}
