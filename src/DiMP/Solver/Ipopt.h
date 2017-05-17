#pragma once

#define HAVE_CONFIG_H
#include <IpTNLP.hpp>
#include <IpIpoptApplication.hpp>

using namespace Ipopt;

#include <DiMP/Types.h>

#include <vector>
#include <fstream>
using namespace std;

namespace DiMP{;

class Graph;
class Solver;
class Variable;

/** IPOPTソルバを使うためのアダプタ
	- IPOPT3.11使用 (2013/12)
	- Debugビルドではうまく動作しない．Release限定

	- quaternionは4次元ベクトルとして扱いつつノルムを1とする等式拘束を課しているが，
	  うまく収束せずエラー終了するケースが多い
	- IPOPTでうまく解を出すにはTreeを使って関節座標で表現することが必要
 **/

class IpoptAdaptor : public TNLP{
public:
	typedef PTM::TMatrixCol<3,4,real_t>	mat34_t;

	enum{
		Weighted,		///< 重み付誤差最小化
		Phased,			///< 高優先度から段階的に最小化
	};

	Graph*			graph;		///< グラフ
	Solver*			solver;		///< ソルバ
	int				problem;	///< 問題の種類
	uint			phase;		///< Phasedで現在最小化している優先度レベル
	vector<real_t>	error_min;	///< Phasedで既に最小化した拘束誤差
	vector<real_t>	error;		///< 優先度レベル別拘束誤差の現在値

	SmartPtr<IpoptApplication>	app;
	
	uint			nvar;
	uint			ncon;
	vector<uint>	quatIndex;		///< quaternion変数へのインデックス

	ofstream		file;

	void    SetVariable (const Number* x);
	mat34_t QuatJacobian(quat_t q);
	real_t  CalcCost    (uint l);
	void    CalcGradient(Number* grad, Variable* var, uint level);

public:
	virtual bool get_nlp_info      (Index& n, Index& m, Index& nnz_jac_g, Index& nnz_h_lag, IndexStyleEnum& index_style);
	virtual bool get_bounds_info   (Index n, Number* x_l, Number* x_u, Index m, Number* g_l, Number* g_u);
	virtual bool get_starting_point(Index n, bool init_x, Number* x, bool init_z, Number* z_L, Number* z_U,
		                            Index m, bool init_lambda, Number* lambda);
	virtual bool eval_f            (Index n, const Number* x, bool new_x, Number& obj_value);
	virtual bool eval_grad_f       (Index n, const Number* x, bool new_x, Number* grad_f);
	virtual bool eval_g            (Index n, const Number* x, bool new_x, Index m, Number* g);
	virtual bool eval_jac_g        (Index n, const Number* x, bool new_x, Index m,
		                            Index nele_jac, Index* iRow, Index* jCol, Number* values);
	virtual bool eval_h            (Index n, const Number* x, bool new_x, Number obj_factor,
		                            Index m, const Number* lambda, bool new_lambda,
									Index nele_hess, Index* iRow, Index* jCol, Number* values);
	virtual void finalize_solution (SolverReturn status, Index n, const Number* x, const Number* z_L, const Number* z_U,
		                            Index m, const Number* g, const Number* lambda, Number obj_value,
									const IpoptData* ip_data, IpoptCalculatedQuantities* ip_cq);
	virtual bool intermediate_callback(AlgorithmMode mode, Index iter, Number obj_value,
									Number inf_pr, Number inf_du, Number mu, Number d_norm,
									Number regularization_size, Number alpha_du, Number alpha_pr,
									Index ls_trials, const IpoptData* ip_data, IpoptCalculatedQuantities* ip_cq);

public:
	void	SetProblem(int p);
	void	Init      (Graph* g);
	int		Solve     ();

	IpoptAdaptor();

};

}
