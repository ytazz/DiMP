#include <DiMP2/Operator.h>
#include <DiMP2/Constraint.h>

#include <boost/foreach.hpp>
#define foreach BOOST_FOREACH
using namespace boost;

namespace DiMP2{;

//-------------------------------------------------------------------------------------------------

Operator::Operator(){
	enabled		= true;
	level		= 0;

	coef_prod	= 0.0;
	e			= 0.0;
	de			= 0.0;
	ded			= 0.0;
}

void Operator::Clear(){
	parents.clear();
	children.clear();
}

void Operator::AddChildOp(Operator* op){
	children.push_back(Child(1.0, op));
	op->parents.push_back(this);
}

void Operator::CalcErrorAndCoef(){
	foreach(const Child& child, children)
		child.op->CalcErrorAndCoef();
	CalcCoef();
	CalcError();
}

void Operator::CalcCoefProd(real_t prod){
	coef_prod = prod;
	foreach(const Child& child, children)
		child.op->CalcCoefProd(prod * child.coef);
}

void Operator::Forward(real_t (Operator::*var)){
	this->*var = 0.0;
	foreach(const Child& child, children){
		child.op->Forward(var);
		this->*var += child.coef * child.op->*var;
	}
}

/*	この演算子の誤差減少量をdedとするには
	i番目の子演算子の誤差減少量は
		ded_i = (c_i / csum) * ded
	とすればよい．
	ただしc_iは係数，csumは全子演算子の係数の二乗和
 */
/*
void Operator::CalcCorrection(){
	// 係数の二乗和を求める
	coef_sum = 0.0;
	foreach(const Child& child, children)
		coef_sum += child.coef * child.coef;

	// 子演算子に目標減少量を割当て
	foreach(const Child& child, children){
		child.op->ded = (child.coef / coef_sum) * ded;
		child.op->CalcCorrection();
	}
}
*/

void Operator::CalcPriority(){
	// 子演算子のCalcPriorityを再帰的に呼び，すべての子の優先度が等しければそれを
	// この演算子の優先度とする．
	bool found = false;
	uint L;
	for(Children::iterator it = children.begin(); it != children.end(); it++){
		it->op->CalcPriority();
		if(!it->op->enabled)
			continue;

		// 異なる優先度が混在する場合は無効化
		if(!found){
			L = it->op->level;
			found = true;
		}
		if(found && it->op->level != L){
			enabled = false;
			return;
		}
	}
	// 有効な子演算子が無かった場合は自分も無効
	if(!found){
		enabled = false;
		return;
	}
	level = L;
}

//-------------------------------------------------------------------------------------------------

const real_t OperatorNOT::ratio = 1.0;

void OperatorNOT::CalcCoef(){
	children[0].coef = -ratio * den2;
}

void OperatorNOT::CalcError(){
	if(children[0].op->enabled){
		den = 1.0 / (1.0 + ratio * children[0].op->e);
		den2 = den*den;
		e = den;
	}
	else{
		e = 0.0;
		enabled = false;
	}
}

/*
void OperatorNOT::CalcCorrection(real_t dir){
	// not con の係数
	real_t coef0 = -2*a/ enot_den * enot_den;
	real_t normalize = e0/(enot_den * enot_den * enot_den);
	const real_t eps = 1.0e-10;
	if(normalize < eps)
		return;

	// 誤差修正量の重みつけ
	con->dir *= coef0/normalize;
}
*/

//-------------------------------------------------------------------------------------------------

void OperatorOR::CalcCoef(){
	children[0].coef = children[1].op->e;
	children[1].coef = children[0].op->e;
}

/*
void OperatorOR::CalcCorrection(real_t dir){
	// 各拘束の二乗ノルムとその和
	real_t e0 = con0->e.square();
	real_t e1 = con1->e.square();
	real_t esum = e0 + e1;

	const real_t eps = 1.0e-10;
	if(esum < eps)
		return;

	// 誤差修正量の重みつけ
	//  誤差の小さい方により大きな重み
	con0->dir *= e1/esum;
	con1->dir *= e0/esum;
}
*/

void OperatorOR::CalcError(){
	if(children[0].op->enabled && children[1].op->enabled)
		e = children[0].op->e * children[1].op->e;
	else{
		e = 0;
		enabled = false;
	}
}

//-------------------------------------------------------------------------------------------------

void OperatorIF::CalcCoef(){
	children[0].coef = -OperatorNOT::ratio * children[1].op->e * den2;
	children[1].coef = den;
}

void OperatorIF::CalcError(){
	if(children[0].op->enabled && children[1].op->enabled){
		den = 1.0 / (1.0 + OperatorNOT::ratio * children[0].op->e);
		den2 = den*den;
		e = children[1].op->e * den;
	}
	else{
		e = 0.0;
		enabled = false;
	}
}

/*
=======
// 条件のほうのnotをとる必要があるかないかは再考の必要あり．
>>>>>>> .r504
void OperatorIF::CalcCorrection(){
	// 各拘束の二乗ノルム
	real_t e0 = con0->e.square();
	real_t e1 = con1->e.square();

	// e0が十分小さい時，if->then は and と同義
	if(e0<0.05)
		return;

	//  not con　の誤差関数の定義:　enot = e1 / ( a*e0 + 1)

	// not con1 or con2 のための係数計算
	real_t a =  1.0;
	real_t note0_den =  a*e0 + 1;
	real_t e_norm = a*a*e0*e1 + 1;

	if( e_norm > 1.0e6)
		e_norm = 1.0e6;
//	real_t gain0 = - (a*e1)/(e_norm*note0_den);
	real_t gain1 = 1.0/e_norm;

	real_t coef0 = - (a*e1)/(e_norm*note0_den);
	real_t coef1 = 1.0/e_norm;

	// 誤差修正量の重みつけ
	// con0->dir *= coef0;	いったんコメントアウト
	con1->dir *= coef1;
}
*/

//-------------------------------------------------------------------------------------------------

void OperatorAND::CalcCoef(){
	foreach(Child& child, children)
		child.coef = 1.0;
}

void OperatorAND::CalcError(){
	e = 0.0;
	foreach(const Child& child, children){
		if(child.op->enabled)
			e += child.op->e;
	}

	/*
	// not con の係数
	real_t enot_den2 =  enot_den * enot_den;
	// not con の係数
	real_t coef0 = -a/ enot_den2;
	real_t normalize = (4*a*a*e0) / (enot_den2*enot_den2) ;
	const real_t eps = 1.0e-10;
	if(normalize < eps)
		return;

	// 誤差修正量の重みつけ
	con->dir *= coef0/normalize;
	*/
}

}
