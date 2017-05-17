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

/** IPOPT�\���o���g�����߂̃A�_�v�^
	- IPOPT3.11�g�p (2013/12)
	- Debug�r���h�ł͂��܂����삵�Ȃ��DRelease����

	- quaternion��4�����x�N�g���Ƃ��Ĉ����m������1�Ƃ��铙���S�����ۂ��Ă��邪�C
	  ���܂����������G���[�I������P�[�X������
	- IPOPT�ł��܂������o���ɂ�Tree���g���Ċ֐ߍ��W�ŕ\�����邱�Ƃ��K�v
 **/

class IpoptAdaptor : public TNLP{
public:
	typedef PTM::TMatrixCol<3,4,real_t>	mat34_t;

	enum{
		Weighted,		///< �d�ݕt�덷�ŏ���
		Phased,			///< ���D��x����i�K�I�ɍŏ���
	};

	Graph*			graph;		///< �O���t
	Solver*			solver;		///< �\���o
	int				problem;	///< ���̎��
	uint			phase;		///< Phased�Ō��ݍŏ������Ă���D��x���x��
	vector<real_t>	error_min;	///< Phased�Ŋ��ɍŏ��������S���덷
	vector<real_t>	error;		///< �D��x���x���ʍS���덷�̌��ݒl

	SmartPtr<IpoptApplication>	app;
	
	uint			nvar;
	uint			ncon;
	vector<uint>	quatIndex;		///< quaternion�ϐ��ւ̃C���f�b�N�X

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
