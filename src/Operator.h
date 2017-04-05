#ifndef OPERATOR_H
#define OPERATOR_H

#include <DiMP2/ID.h>
#include <vector>

namespace DiMP2{;

/** ���Z�q
	- ���Z�q�͐e�q�֌W�ɂ��Ȃ����z�O���t�𐬂��D
	- �ŉ����ɂ�Constraint���Ȃ���D
	
	- �_�����Z�ƍS���덷�̑㐔���Z��1��1�ɑΉ�����
	�@����ɂ���Ę_���������덷�֐��̍ŏ����̘g�g�݂ɓ���邱�Ƃ��ł���D
 */

class Solver;

/** ���Z�q�̊�{�N���X�D�e�퉉�Z�q��Constraint���h������

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

	/** �D��x������
		- �q���Z�q�̗D��x����{�g���A�b�v�ɗD��x������
		- �q���Z�q�̗D��x���H���Ⴄ�ꍇ�͖���`�ƂȂ�
	 */
	void	CalcPriority();

	/// �덷�Ɣ��W�����ċA�I�Ɍv�Z
	void	CalcErrorAndCoef();
	/// ���[�g���Z�q���疖�[�̍S���܂ł̌W���̐ς��v�Z
	void	CalcCoefProd(real_t prod);

	/// �ċA�I��CalcCoef���Ă�
	//void	CalcCoefRecurs(real_t prod);

	/// de��ded���{�g���A�b�v�Ɍv�Z
	virtual void Forward(real_t Operator::*var);
	
	virtual void CalcCoef(){}
	virtual void CalcError() = 0;
	//virtual void CalcCorrection();
	
	Operator();
};

/// unary operator �P�����Z�q
class UnaryOperator : public Operator{
public:
	UnaryOperator(Operator* o){
		AddChildOp(o);
	}
};

/// binary operator 2�����Z�q
class BinaryOperator : public Operator{
public:
	BinaryOperator(Operator* o0, Operator* o1){
		AddChildOp(o0);
		AddChildOp(o1);
	}
};

/// N-ary operator N�����Z�q
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
	- (NOT (op0)) | (op1) �Ɠ���
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
