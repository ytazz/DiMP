#pragma once

#include <DiMP2/ID.h>
#include <DiMP2/Variable.h>
#include <DiMP2/Constraint.h>

namespace DiMP2{;

class Variable;
class Constraint;

/**
	Link: Jacobian matrix from variable to constraint error
 */
class Link : public UTRefCount{
public:
	Variable*		var;
	Constraint*		con;
	bool			active;

	void Connect(Variable* v, Constraint* c);

	/// add norm of rows of Jacobian
	virtual void AddRowSqr(vec3_t& v) = 0;

	/// x -> c(x)
	virtual void AddError() = 0;

	virtual void   Forward (uint k, real_t d) = 0;
	virtual void   Backward(uint k, real_t d) = 0;
	virtual vec3_t Backward(vec3_t v) = 0;

	Link();
};

typedef std::vector< UTRef<Link> >	LinkRefs;

/** link with scalar coefficient
	y = c(x) = k * x
	k is scalar
	x is scalar or vector3
	y is scalar or vector3
 */
class SLink : public Link{
public:
	real_t coef, coefsqr;
	
	void SetCoef(real_t k);

	virtual void   AddRowSqr(vec3_t& v);
	virtual void   AddError ();
	virtual void   Forward  (uint k, real_t d);
	virtual void   Backward (uint k, real_t d);
	virtual vec3_t Backward (vec3_t v);

	SLink(real_t k);
};

class V3Link : public Link{
public:
	vec3_t	coef, coefsqr;

	void SetCoef(vec3_t k);

	V3Link(vec3_t k = vec3_t(1.0,1.0,1.0));
};

/** link with cross product matrix
	y = c(x) = k % x
	k is vector3
	x is vector3
	y is vector3
 */
class XLink : public V3Link{
public:
	virtual void   AddRowSqr(vec3_t& v);
	virtual void   AddError ();
	virtual void   Forward  (uint k, real_t d);
	virtual void   Backward (uint k, real_t d);
	virtual vec3_t Backward (vec3_t v);

	XLink(){}
};

/** link between vec3 constraint and scalar variable
	y = c(x) = k * x
	k is vector3
	x is scalar
	y is vector3

	* C stands for column
 */
class CLink : public V3Link{
public:
	virtual void   AddRowSqr(vec3_t& v);
	virtual void   AddError ();
	virtual void   Forward  (uint k, real_t d);
	virtual void   Backward (uint k, real_t d);
	virtual vec3_t Backward (vec3_t v);

	CLink(){}
};

/** link between scalar constraint and vec3 variable
	y = c(x) = k * x	(inner product)

	R stands for row
 */
class RLink : public V3Link{
public:
	virtual void   AddRowSqr(vec3_t& v);
	virtual void   AddError ();
	virtual void   Forward  (uint k, real_t d);
	virtual void   Backward (uint k, real_t d);
	virtual vec3_t Backward (vec3_t v);

	RLink(){}
};

/** general linear map
	y = A x
 */
class MLink : public Link{
public:
	mat3_t	coef;
	mat3_t	coefsqr;

public:
	void SetCoef(const mat3_t& m);

	virtual void   AddRowSqr(vec3_t& v);
	virtual void   AddError ();
	virtual void   Forward  (uint k, real_t d);
	virtual void   Backward (uint k, real_t d);
	virtual vec3_t Backward (vec3_t v);

	MLink(){}
};

}
