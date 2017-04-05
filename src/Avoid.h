#pragma

#include <DiMP2/Task.h>
#include <DiMP2/Constraint.h>

/** Collision avoidance
	- 2つの剛体間の距離が一定値以下にならないようにする
 */

namespace DiMP2{;

class AvoidTask;
class AvoidConP;
class AvoidConV;

class AvoidKey : public TaskKey{
public:	
	AvoidConP*	con_p;
	AvoidConV*	con_v;

public:
	virtual void AddCon(Solver* solver);
	virtual void Prepare();

	AvoidKey();
};

class AvoidTask : public Task{
public:
	real_t	dmin;		///< 最小接近距離

public:
	virtual Keypoint*	CreateKeypoint(){ return new AvoidKey(); }
	virtual void Prepare();
	
	AvoidTask(Object* _obj0, Object* _obj1, TimeSlot* _time, const string& n);

};

class AvoidCon : public Constraint{
public:
	AvoidKey*	key;

	virtual void CalcCoef();

	AvoidCon(Solver* solver, ID id, AvoidKey* _key, real_t _scale);
	virtual ~AvoidCon(){}
};

class AvoidConP : public AvoidCon{
public:
	virtual void CalcDeviation();
	virtual void Project(real_t& l, uint k);

	AvoidConP(Solver* solver, const string& _name, AvoidKey* _key, real_t _scale);
	virtual ~AvoidConP(){}
};

class AvoidConV : public AvoidCon{
public:
	virtual void CalcDeviation();
	virtual void Project(real_t& l, uint k);

	AvoidConV(Solver* solver, const string& _name, AvoidKey* _key, real_t _scale);
	virtual ~AvoidConV(){}
};

}
