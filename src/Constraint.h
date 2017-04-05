#pragma once

#include <DiMP2/ID.h>
#include <DiMP2/Variable.h>

#include <map>

namespace DiMP2{;

using namespace std;
using namespace Spr;

class Solver;
class SLink;
class XLink;
class CLink;
class RLink;
class MLink;

/**
	constraint base class
 */
class Constraint : public ID, public UTRefCount{
public:
	Solver*		solver;
	Links		links;			///< links to constrained variables
	uint		nelem;
	uint		level;			///< priority level
	bool		enabled;		///< enabled constraint (controlled by user)
	bool		active;			///< active constraint (task-related constraints, range constraints)
	real_t		scale, scale2, scale_inv, scale2_inv;		///< scaling coefficient
		
	/** error correction rate
		誤差修正率．
		(誤差修正量) = (誤差) x (correction rate)
		通常は[0,1)の値を設定する．理論上は(-1,1)であれば誤差は0へ収束する
		実用上，corrRateを2などに設定すると少ない反復で早い収束が得られることもある
		*/
	real_t	corrRate;

	/** maximum error correction
		誤差修正量の上限．
		誤差修正量はこの値を超えないようにクリッピングされる．
		優先度レベルが複数ある場合に定常偏差を低減するにはこの値を小さ目に設定する．
		ただし小さくするほど収束スピードは低下する．
	 */
	real_t		corrMax;

	vec3_t		e;			///< error value
	vec3_t		de;			///< change of error
	vec3_t		ded;		///< desired change of error

	vec3_t		y;			///< constraint error
	vec3_t		dy;			///< change of constraint error
	vec3_t		dyd;		///< desired change of constraint error
	
	vec3_t		l0, l1;		///< multiplier
	vec3_t		dl0, dl1;	///< change of multiplier
	vec3_t		J, Jinv;	///< square sum of Jacobian row and its inverse

	/// テスト用
	ofstream	file;

public:
	SLink*		AddSLink(Variable* var, real_t coef = 1.0);
	XLink*		AddXLink(Variable* var);
	CLink*		AddCLink(Variable* var);
	RLink*		AddRLink(Variable* var);
	MLink*		AddMLink(Variable* var);

	void SetPriority(uint newlv);

	/// reset internal variables
	void ResetState();

	/// preparation
	void CalcCorrection();

	/// G-S related routines
	//void UpdateMultiplierCorr(uint k);
	//void UpdateMultiplierProj(uint k);
	void UpdateMultiplier(uint k);
	void UpdateError(uint k, real_t ddy);

public:	
	/// calc constraint coefficient
	virtual void CalcCoef(){}

	/// calc constraint error
	virtual void CalcError();

public:
	/// virtual functions to be overridden by derived classes ///

	/** calculation of constraint error
		default implementation calculates linear combination of constrained variables
		must be overridden if the constraint is nonlinear or it has bias terms
	 */
	virtual void CalcDeviation();

public:
	/// G-S related virtual functions
	
	/** do projection on multiplier
	 */
	virtual void Project(real_t& l, uint k){}
	
	Constraint(Solver* solver, uint n, ID id, real_t _scale);
};

}
