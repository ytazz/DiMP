#ifndef OPERATOR_H
#define OPERATOR_H

#include <DiMP2/ID.h>
#include <vector>

namespace DiMP2{;

/** 演算子
	- 演算子は親子関係によりつながる非循環グラフを成す．
	- 最下流にはConstraintがつながる．
	
	- 論理演算と拘束誤差の代数演算は1対1に対応する
	　これによって論理条件も誤差関数の最小化の枠組みに入れることができる．
 */

class Solver;

/** 演算子の基本クラス．各種演算子とConstraintが派生する

 */
class Operator : public UTRefCount{
public:
	struct Child{
		real_t			coef;	///< partial derivative
		UTRef<Operator>	op;

		Child(real_t c, Operator* o):coef(c), op(o){}
	};
	typedef	std::vector<Child>		Children;	///< parent operators
	typedef std::vector<Operator*>	Parents;	///< child operators
	
	Solver*		solver;
	Parents		parents;	///< parent operators
	Children	children;	///< child operators

	bool		enabled;	///< enabled or not
	uint		level;		///< priority level

	real_t	coef_prod;		///< total product of coefficients from root operator down to this operator
	//real_t	coef_sum;	///< sum of squares of coefficients
	real_t	e;				///< error value
	real_t	de;				///< change of error
	real_t	ded;			///< desired change of error
	
public:
	void	Clear();
	void	AddChildOp(Operator* op);

	/** 優先度を決定
		- 子演算子の優先度からボトムアップに優先度を決定
		- 子演算子の優先度が食い違う場合は未定義となる
	 */
	void	CalcPriority();

	/// 誤差と微係数を再帰的に計算
	void	CalcErrorAndCoef();
	/// ルート演算子から末端の拘束までの係数の積を計算
	void	CalcCoefProd(real_t prod);

	/// 再帰的にCalcCoefを呼ぶ
	//void	CalcCoefRecurs(real_t prod);

	/// deかdedをボトムアップに計算
	virtual void Forward(real_t Operator::*var);
	
	virtual void CalcCoef(){}
	virtual void CalcError() = 0;
	//virtual void CalcCorrection();
	
	Operator();
};

/// unary operator 単項演算子
class UnaryOperator : public Operator{
public:
	UnaryOperator(Operator* o){
		AddChildOp(o);
	}
};

/// binary operator 2項演算子
class BinaryOperator : public Operator{
public:
	BinaryOperator(Operator* o0, Operator* o1){
		AddChildOp(o0);
		AddChildOp(o1);
	}
};

/// N-ary operator N項演算子
class NaryOperator : public Operator{
public:

};

/** NOT (op0)
	- e = 1/(1 + a*e0)
	- de = -a/(1 + a*e0)^2 * de0
 */
class OperatorNOT: public UnaryOperator{
public:
	static const real_t ratio;

	real_t	den, den2;		///< denominator part and its square

	virtual void CalcCoef();
	virtual void CalcError();

	OperatorNOT(Operator* o):UnaryOperator(o){}
};

/** (op0) OR (op1)
	- e = e0 * e1
	- de = e1 * de0 + e0 * de1
 */
class OperatorOR : public BinaryOperator{
public:
	virtual void CalcCoef();
	virtual void CalcError();

	OperatorOR(Operator* o0, Operator* o1):BinaryOperator(o0, o1){}
};

/** (op0) -> (op1)
	- (NOT (op0)) | (op1) と等価
	- e = e1 / (1 + a*e0)
	- de = -(a*e1) / (1 + a*e0)^2 * de0 + 1/(1 + a*e0) * de1
 */
class OperatorIF : public BinaryOperator{
public:
	real_t	den, den2;		///< denominator part and its square

	virtual void CalcCoef();
	virtual void CalcError();

	OperatorIF(Operator* o0, Operator* o1):BinaryOperator(o0, o1){}
};

/** (op0) AND (op1) AND ... AND (opN)
	- e = e0 + e1 + ... + eN
	- de = de0 + de1 + ... + deN
 */
class OperatorAND : public NaryOperator{
public:
	virtual void CalcCoef();
	virtual void CalcError();
};

}

#endif
