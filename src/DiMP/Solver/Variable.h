#pragma once

#include <DiMP/Graph/ID.h>

#include <vector>
using namespace std;

namespace DiMP{;

class	Solver;
class	Keypoint;
class	Link;

typedef std::vector< Link* >		  Links;

/**
	çSë©Ç≥ÇÍÇÈïœêî
 */
class Variable : public ID, public UTRefCount{
public:
	/// variable types
	enum{
		Scalar = 1,
		Vec3   = 3,
		Quat   = 4,
	};

	typedef void (Variable::*UpdateFunc)(uint k, real_t d);

	Links	links;			///< links to constraints
	bool	locked;			///< locked or not
	uint	type;			///< variable type
	uint	nelem;			///< number of elements

	real_t	scale, scale2, scale_inv, scale_inv2;	///< scaling factor, its inverse and squared inverse

	vec3_t	d;				///< delta
	vec3_t	dd;				///< delta-delta
	vec3_t	de;				///< J^T * Constraint::de
	vec3_t	J, Jinv;		///< square sum of Jacobian column and its inverse
	
public:
	void Lock(bool on = true);
	void SetScale(real_t sc);
	
	/** propagate change of variable
		@param k			element index
		@param dd_noscale	change of change, unscaled
		@param caller		Link that called this function
	 */
	void UpdateVar(uint k, real_t _dd);
	
	virtual void	Reset() = 0;
	virtual void    ResetState();
	virtual real_t	Get(uint k) = 0;
	virtual void    Set(uint k, real_t v) = 0;
	virtual real_t	Norm() = 0;
	virtual void	Modify(real_t alpha) = 0;

	Variable(uint _type, Solver* solver, ID _id, real_t _scale);
};

template<class T>
class VariableImpl : public Variable{
public:
	T val;
	T val_tmp;

	virtual void ResetState(){
		Variable::ResetState();
		val_tmp = val;
	}

	VariableImpl(uint _type, Solver* solver, ID _id, real_t _scale):Variable(_type, solver, _id, _scale){}
};

/**
	scalar variable
 */
class SVar : public VariableImpl<real_t>{
public:
	virtual void Reset(){
		val = val_tmp = 0.0;
	}
	virtual real_t Get(uint k){
		return val;
	}
	virtual void Set(uint k, real_t v){
		val = v;
	}
	virtual real_t Norm(){
		return abs(val);
	}
	virtual void Modify(real_t alpha){
		val = val_tmp + alpha * d[0];
	}

	SVar(Solver* solver, ID _id, real_t _scale):VariableImpl(Variable::Scalar, solver, _id, _scale){
		Reset();
	}
};

/**
	3D vector variable
 */
class V3Var : public VariableImpl<vec3_t>{
public:
	virtual void Reset(){
		val    .clear();
		val_tmp.clear();
	}
	virtual real_t Get(uint k){
		return val[k];
	}
	virtual void Set(uint k, real_t v){
		val[k] = v;
	}
	virtual real_t Norm(){
		return val.norm();
	}
	virtual void Modify(real_t alpha){
		val = val_tmp + alpha * d;
	}

	V3Var(Solver* solver, ID _id, real_t _scale):VariableImpl(Variable::Vec3, solver, _id, _scale){}
};

/**
	quaternion variable
 */
class QVar : public VariableImpl<quat_t>{
public:
	virtual void Reset(){
		val     = quat_t();
		val_tmp = quat_t();
	}
	virtual real_t Get(uint k){
		return val[k];
	}
	virtual void Set(uint k, real_t v){
		val[k] = v;
	}
	virtual real_t Norm(){
		return ((vec4_t&)val).norm();
	}
	virtual void Modify(real_t alpha){
		val = quat_t::Rot(alpha * d) * val_tmp;
		//val = val_tmp * quat_t::Rot(alpha * d);
		val.unitize();
	}

	QVar(Solver* solver, ID _id, real_t _scale):VariableImpl(Variable::Quat, solver, _id, _scale){}
};

}
