#pragma once

#include <DiMP2/Constraint.h>

namespace DiMP2{;

using namespace std;
using namespace Spr;

/** fixation constraint for scalar
 */
class FixConS : public Constraint{
public:
	real_t	desired;
	virtual void CalcDeviation();
	FixConS(Solver* solver, ID id, SVar* var, real_t _scale);
};
class MatchConS : public Constraint{
public:
	MatchConS(Solver* solver, ID id, SVar* var0, SVar* var1, real_t _scale);
};

/** fixation constraint for vector3
 */
class FixConV3 : public Constraint{
public:
	vec3_t	desired;
	virtual void CalcDeviation();
	FixConV3(Solver* solver, ID id, V3Var* var, real_t _scale);
};
class MatchConV3 : public Constraint{
public:
	MatchConV3(Solver* solver, ID id, V3Var* var0, V3Var* var1, real_t _scale);
};

/** fixation constraint for quaternion
 */
class FixConQ : public Constraint{
public:
	quat_t	desired;
	virtual void CalcDeviation();
	FixConQ(Solver* solver, ID id, QVar* var, real_t _scale);
};
class MatchConQ : public Constraint{
public:
	virtual void CalcDeviation();
	MatchConQ(Solver* solver, ID id, QVar* var0, QVar* var1, real_t _scale);
};


/**	range constraint for scalar variables
	l <= x <= u
 */
class RangeConS : public Constraint{
public:
	real_t	_min, _max;
	bool	on_lower, on_upper;

	virtual void CalcCoef();
	virtual void CalcDeviation();
	virtual void Project(real_t& l, uint k);

	RangeConS(Solver* solver, ID id, SVar* var, real_t _scale);
};
class RangeConV3 : public Constraint{
public:
	vec3_t	_min, _max;
	bool	on_lower[3], on_upper[3];

	virtual void CalcDeviation();
	virtual void Project(real_t& l, uint k);

	RangeConV3(Solver* solver, ID id, V3Var* var, real_t _scale);
};

/**	range constraint for difference of scalar variables
	l <= (x1 - x0) <= u
 */
class DiffConS : public Constraint{
public:
	real_t	_min, _max;
	bool	on_lower, on_upper;

	virtual void CalcDeviation();
	virtual void Project(real_t& l, uint k);

	DiffConS(Solver* solver, ID id, SVar* var0, SVar* var1, real_t _scale);
};

/** general linear inequality constraint of scalar variables
	A0 * x0 + A1 * x1 <= B
 */
/*struct LinearConS : Constraint{
	
};*/

/** fix vector on plane
	n^T (x - o) = 0
	n: normal of plane
	o: origin of plane
 */
class FixConPlane : public Constraint{
public:
	vec3_t	normal;
	vec3_t  origin;
	
	virtual void CalcCoef();
	virtual void CalcDeviation();
	FixConPlane(Solver* solver, ID id, V3Var* var, real_t _scale);
};

class RangeConPlane : public Constraint{
public:
	vec3_t  normal;
	vec3_t  origin;
	real_t  _min, _max;
	bool	on_lower, on_upper;

	virtual void CalcCoef();
	virtual void CalcDeviation();
	virtual void Project(real_t& l, uint k);
	RangeConPlane(Solver* solver, ID id, V3Var* var, real_t _scale);
};

/** complementarity constraint of scalar variables
	x0 * x1 = 0
 */
class ComplConS : public Constraint{
public:
	virtual void CalcCoef();
	virtual void CalcDeviation();

	ComplConS(Solver* solver, ID id, SVar* var0, SVar* var1, real_t _scale);
};

/** range constraint on euclidean distance
	l <= ||x - y|| <= u
 */
class DistanceConV3 : public Constraint{
public:
	vec3_t	diff;
	real_t	diff_norm;
	real_t  _min, _max;
	bool	on_lower, on_upper;

	virtual void CalcCoef();
	virtual void CalcDeviation();
	virtual void Project(real_t& l, uint k);
	DistanceConV3(Solver* solver, ID id, V3Var* var0, V3Var* var1, real_t _scale);
};

/**
	C1 continuity constraint
 */
class C1ConS : public Constraint{
public:
	real_t h;

	virtual void CalcCoef();
	C1ConS(Solver* solver, ID id, SVar* p0, SVar* v0, SVar* p1, SVar* v1, real_t _scale);
};
class C1ConV3 : public Constraint{
public:
	real_t h;

	virtual void CalcCoef();
	C1ConV3(Solver* solver, ID id, V3Var* p0, V3Var* v0, V3Var* p1, V3Var* v1, real_t _scale);
};

}
