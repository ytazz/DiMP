#include <DiMP/Solver/Ipopt.h>
#include <DiMP/Graph/Graph.h>

namespace DiMP{;

///////////////////////////////////////////////////////////////////////////////////////////////////

IpoptAdaptor::IpoptAdaptor(){
	graph   = 0;
	solver  = 0;
	problem = Weighted;
}

void IpoptAdaptor::SetProblem(int p){
	problem = p;
}

void IpoptAdaptor::Init(Graph* g){
	graph  = g;
	solver = &graph->solver;
	solver->Init();

	// quaternion�̈ʒu
	quatIndex.clear();
	for(uint i = 0; i < solver->vars.size(); i++){
		Variable* var = solver->vars[i];
		if(var->locked)
			continue;

		if(var->nelem == 4)
			quatIndex.push_back(nvar);
	}

	// �S���덷���L������z��
	error.resize(solver->maxLevel+1);
	fill(error.begin(), error.end(), 0.0);
	error_min.resize(solver->maxLevel+1);
	fill(error_min.begin(), error_min.end(), 0.0);
	
	app = IpoptApplicationFactory();
	// �ł��؂�덷
	app->Options()->SetNumericValue("tol"            , 1.0e-1);
	/*app->Options()->SetNumericValue("dual_inf_tol"   , 1.0e+20);
	app->Options()->SetNumericValue("constr_viol_tol", 1.0e+20);
	app->Options()->SetNumericValue("compl_inf_tol"  , 1.0e+20);
	app->Options()->SetNumericValue("acceptable_tol"            , 1.0e+20);
	app->Options()->SetNumericValue("acceptable_dual_inf_tol"   , 1.0e+20);
	app->Options()->SetNumericValue("acceptable_constr_viol_tol", 0.1);
	app->Options()->SetNumericValue("acceptable_compl_inf_tol"  , 1.0e+20);
	app->Options()->SetNumericValue("acceptable_obj_change_tol" , 1.0e-7);
	app->Options()->SetIntegerValue("acceptable_iter"           , 5);
	*/

	//app->Options()->SetNumericValue("soft_resto_pderror_reduction_factor", 0.0);
	//app->Options()->SetNumericValue("required_infeasibility_reduction", 0.0);

	// �ő唽����
	app->Options()->SetIntegerValue("max_iter", 1000);
	
	//app->Options()->SetNumericValue("mu_target", 0.1);
	
	// �w�V�A���͓����ŋߎ��v�Z
	app->Options()->SetStringValue ("hessian_approximation", "limited-memory");
	
	// watchdog: �������X�e�b�v�����Â����Ƃ��Ɋ��������p������͗l
	// ���I�œK���𓾂�\�����オ�邪�C�]���l�̃v���b�g�ɃX�p�C�N���̂�̂Ō����ڂ͂��܂�ǂ��Ȃ�
	// �ȉ���0�ɂ����watchdog������
	app->Options()->SetIntegerValue("watchdog_shortened_iter_trigger", 0);
	// �ȉ���yes�ɂ����line search������
	//app->Options()->SetStringValue ("accept_every_trial_step", "yes");
	
	// ���b�Z�[�W���x��: �K�v�Ȃ��Ƃ̂�
	app->Options()->SetIntegerValue("print_level", 4);
	app->Initialize();

	file.open("ipopt_log.csv", ios_base::out);
	if(file.is_open()){
		file << "iter" << ", " << "obj_value";
		uint L = solver->maxLevel;
		for(uint l = 0; l <= L; l++){
			file << ", " << "lv" << l;
		}
		for(uint l = 0; l <= L; l++){
			file << ", " << "lv" << l << "sat";
		}
		if(problem == Phased)
			file << ", " << "phase";
		file << endl;
	}
}

int IpoptAdaptor::Solve(){
	ApplicationReturnStatus stat;
		
	if(problem == Phased)
		phase = 0;

	while(true){
		cout << "phase: " << phase << endl;
		try{
			stat = app->OptimizeTNLP(this);
		}
		catch(...){
			cout << "something wrong" << endl;
		}
		
		if(problem == Weighted)
			break;
		if(problem == Phased){
			if(++phase > solver->maxLevel){
				cout << "phase exceeded " << solver->maxLevel << endl;
				break;
			}
		}
	}
	return stat == Solve_Succeeded;
}

void IpoptAdaptor::SetVariable(const Number* x){
	for(uint i = 0; i < solver->vars.size(); i++){
		Variable* var = solver->vars[i];
		if(var->locked)
			continue;
		for(uint k = 0; k < var->nelem; k++){
			var->Set(k, *x++);
		}
		if(var->nelem == 4)
			((QVar*)var)->val.unitize();
	}
	
	for(uint i = 0; i < graph->trees.size(); i++)
		graph->trees[i]->root->ForwardKinematics();
	graph->Prepare();

	for(uint i = 0; i < solver->cons.size(); i++){
		Constraint* con = solver->cons[i];
		con->CalcCoef ();
		con->CalcError();
	}

}

// quaternion�̎��Ԕ�������p���x��Ԃ��s��
IpoptAdaptor::mat34_t IpoptAdaptor::QuatJacobian(quat_t q){
	mat34_t M;
	M[0][0] = -q[1]; M[0][1] =  q[0]; M[0][2] = -q[3]; M[0][3] =  q[2];
	M[1][0] = -q[2]; M[1][1] =  q[3]; M[1][2] =  q[0]; M[1][3] = -q[1];
	M[2][0] = -q[3]; M[2][1] = -q[2]; M[2][2] =  q[1]; M[2][3] =  q[0];
	return 2.0 * M;
}

real_t IpoptAdaptor::CalcCost(uint level){
	real_t cost = 0.0;

	for(uint i = 0; i < solver->cons.size(); i++){
		Constraint* con = solver->cons[i];
		if(!con->enabled)
			continue;
		if(!con->active)
			continue;
		if(con->level != level)
			continue;

		cost += con->y.square();
	}
	return cost;
}

void IpoptAdaptor::CalcGradient(Number* grad, Variable* var, uint level){
	for(uint k = 0; k < var->nelem; k++)
		grad[k] = 0.0;

	for(uint l = 0; l < var->links.size(); l++){
		Link* link = var->links[l];

		Constraint* con = link->con;
		if(!con->enabled || !con->active)
			continue;

		// Phased�̏ꍇ�͌��݂̗D��x���x���̂�
		if(problem == Phased && con->level != level)
			continue;
		
		real_t w = (problem == Weighted ? solver->weights[con->level] : 1.0);
		vec3_t g = link->Backward(con->y);	///< ���R�r�A���]�u * �S���΍�

		// quaternion�̏ꍇ�͉�]�x�N�g�����獷���ւ̕ϊ��s���������
		if(var->nelem == 4){
			vec4_t q = QuatJacobian(((QVar*)var)->val).trans() * g;
			for(uint k = 0; k < var->nelem; k++)
				grad[k] += 2.0 * w * q[k];
		}
		else{
			for(uint k = 0; k < var->nelem; k++)
				grad[k] += 2.0 * w * g[k];
		}
	}

}

///////////////////////////////////////////////////////////////////////////////////////////////////

bool IpoptAdaptor::get_nlp_info(Index& n, Index& m, Index& nnz_jac_g, Index& nnz_h_lag, IndexStyleEnum& index_style){
	Solver* solver = &graph->solver;

	// �ϐ��̐�
	nvar = 0;
	for(uint i = 0; i < solver->vars.size(); i++){
		Variable* var = solver->vars[i];
		if(var->locked)
			continue;

		nvar += var->nelem;
	}
	n = nvar;
	cout << "nvar: " << nvar << endl;

	// �S���̐�
	//  �P��quaternion�S��
	ncon = quatIndex.size();
	//  Phased�Ȃ��ʗD��x�̌덷�S��
	if(problem == Phased)
		ncon += phase;
	m = ncon;
	cout << "ncon: " << ncon << endl;

	// ���R�r�A���̔�0�v�f��
	//  quaternion�S��
	nnz_jac_g = 4 * quatIndex.size();
	if(problem == Phased){
		// ��ʗD��x�̍S���덷�ɂ��Ă͑a�����l�����Ȃ��i�S�ϐ��ɂ��Ĕ�0�W���Ƃ��Ĉ����j
		nnz_jac_g += (phase * nvar);
	}
	cout << "nnz_jac_g: " << nnz_jac_g << endl;

	// �w�V�A���̔�0�v�f��
	nnz_h_lag = 0;

	// 0-based or 1-based
	index_style = C_STYLE;

	return true;
}

bool IpoptAdaptor::get_bounds_info(Index n, Number* x_l, Number* x_u, Index m, Number* g_l, Number* g_u){
	Number inf = 1.0e10;
	uint nquat = quatIndex.size();

	// �ϐ��̏㉺��
	for(Index j = 0; j < n; j++){
		x_l[j] = -1.0;
		x_u[j] =  1.0;
	}
	// quaternion��[-1, 1]
	for(uint i = 0; i < nquat; i++){
		for(uint k = 0; k < 4; k++){
			x_l[quatIndex[i] + k] = -1.0;
			x_u[quatIndex[i] + k] =  1.0;
		}
	}
	
	// quaternion�̃m�����S��: �㉺����1.0�Ƃ��ē�������
	for(Index i = 0; i < nquat; i++){
		g_l[i] = 1.0-0.01;
		g_u[i] = 1.0+0.01;
	}
	// ��ʗD��x�̍S���덷
	if(problem == Phased){
		for(Index i = 0; i < phase; i++){
			g_l[nquat + i] = -inf;
			g_u[nquat + i] = error_min[i] + 0.01;
		}
	}
	
	return true;
}

bool IpoptAdaptor::get_starting_point(Index n, bool init_x, Number* x, bool init_z, Number* z_L, Number* z_U,
    Index m, bool init_lambda, Number* lambda)
{
	// �ϐ��̏����l
	for(uint i = 0; i < solver->vars.size(); i++){
		Variable* var = solver->vars[i];
		if(var->locked)
			continue;
		
		for(uint k = 0; k < var->nelem; k++){
			*x++ = var->Get(k);
		}
	}
	return true;
}

bool IpoptAdaptor::eval_f(Index n, const Number* x, bool new_x, Number& obj_value){
	// �ϐ��l���Z�b�g
	SetVariable(x);

	uint L = solver->maxLevel;
	for(uint l = 0; l <= L; l++){
		error[l] = CalcCost(l);
	}

	// �]���֐��̒l
	if(problem == Weighted){
		obj_value = 0.0;
		for(uint l = 0; l <= L; l++){
			obj_value += solver->weights[l] * error[l];
		}
	}
	if(problem == Phased){
		obj_value = error[phase];
	}

	return true;
}

bool IpoptAdaptor::eval_grad_f(Index n, const Number* x, bool new_x, Number* grad_f){
	static vector<Number> xtmp;
	Number ftmp[2];

	const Number h   = 1.0e-10;
	const Number div = 1.0 / (2.0 * h);

	xtmp.resize(n);
	copy(x, x+n, &xtmp[0]);
	for(uint i = 0; i < n; i++){
		xtmp[i] -= h;
		eval_f(n, &xtmp[0], true, ftmp[0]);
		xtmp[i] += 2.0 * h;
		eval_f(n, &xtmp[0], true, ftmp[1]);
		xtmp[i] -= h;
		grad_f[i] = (ftmp[1] - ftmp[0]) * div;
	}

	/*
	// �ϐ��l���Z�b�g
	SetVariable(x);

	for(Index i = 0; i < n; i++)
		grad_f[i] = 0.0;

	// �]���֐��̌��z
	uint idx = 0;
	for(uint i = 0; i < solver->vars.size(); i++){
		Variable* var = solver->vars[i];
		if(var->locked)
			continue;

		CalcGradient(grad_f + idx, var, phase);
		idx += var->nelem;
	}
	*/
	return true;
}

bool IpoptAdaptor::eval_g(Index n, const Number* x, bool new_x, Index m, Number* g){
	SetVariable(x);

	// quaternion�̃m�����S��
	uint nquat = quatIndex.size();
	for(Index i = 0; i < nquat; i++){
		g[i] = 0.0;
		for(int j = 0; j < 4; j++){
			Number v = x[quatIndex[i] + j];
			g[i] += v*v;
		}
	}

	// ��ʗD��x�̌덷�S��
	if(problem == Phased){
		for(Index i = 0; i < phase; i++)
			g[nquat + i] = CalcCost(i);
	}

	return true;
}

bool IpoptAdaptor::eval_jac_g(Index n, const Number* x, bool new_x, Index m, Index nele_jac, Index* iRow, Index* jCol, Number* values){
	uint nquat = quatIndex.size();
	static vector<Number>	gtmp[2];
	static vector<Number>   xtmp;

	static vector<Index> iRowStore;
	static vector<Index> jColStore;

	// ���R�r�A���̍\��
	if(!values){
		iRowStore.resize(nele_jac);
		jColStore.resize(nele_jac);
		
		uint idx = 0;
		for(uint i = 0; i < nquat; i++){
			for(uint k = 0; k < 4; k++){
				iRowStore[idx] = iRow[idx] = i;
				jColStore[idx] = jCol[idx] = quatIndex[i] + k;
				idx++;
			}
		}
		if(problem == Phased){
			for(uint i = 0; i < phase; i++){
				for(uint k = 0; k < nvar; k++){
					iRowStore[idx] = iRow[idx] = nquat + i;
					jColStore[idx] = jCol[idx] = k;
					idx++;
				}
			}
		}
	}
	// ���R�r�A���̒l
	else{
		// ���l�����Ń��R�r�A�����v�Z
		gtmp[0].resize(m);
		gtmp[1].resize(m);
		xtmp.resize(n);
		const Number h   = 1.0e-10;
		const Number div = 1.0 / (2.0 * h);

		copy(x, x+n, &xtmp[0]);

		for(uint idx = 0; idx < nele_jac; idx++){
			uint i = iRowStore[idx];
			uint j = jColStore[idx];
			xtmp[j] -= h;
			eval_g(n, &xtmp[0], true, m, &gtmp[0][0]);
			xtmp[j] += 2.0 * h;
			eval_g(n, &xtmp[0], true, m, &gtmp[1][0]);
			xtmp[j] -= h;
			values[idx] = (gtmp[1][i] - gtmp[0][i]) * div;
		}

		/*
		uint idx = 0;
		for(uint i = 0; i < nquat; i++){
			for(uint k = 0; k < 4; k++){
				values[idx] = 2.0 * x[quatIndex[i] + k];
				idx++;
			}
		}
		if(problem == Phased){
			for(uint i = 0; i < phase; i++){
				for(uint j = 0; j < solver->vars.size(); j++){
					Variable* var = solver->vars[j];
					if(var->locked)
						continue;

					CalcGradient(values + idx, var, i);
					idx += var->nelem;
				}
			}
		}
		*/
	}

	return true;
}

bool IpoptAdaptor::eval_h(Index n, const Number* x, bool new_x, Number obj_factor, Index m, const Number* lambda, bool new_lambda,
	Index nele_hess, Index* iRow, Index* jCol, Number* values)
{
	return false;
}

/*
	status		�v�Z����
	x			�ϐ��̒l
	z_L			�����S���̏搔
	z_U			����S���̏搔
	g			�S���̒l
	lambda		�S���̏搔
	obj_value	�]���֐��̒l
	ip_data		�t�����
	ip_cq;
 */
void IpoptAdaptor::finalize_solution(SolverReturn status, Index n, const Number* x, const Number* z_L, const Number* z_U,
	Index m, const Number* g, const Number* lambda, Number obj_value, const IpoptData* ip_data, IpoptCalculatedQuantities* ip_cq)
{
	// �ŏ������ꂽ�S���덷���L��
	if(problem == Phased){
		error_min[phase] = obj_value;
		cout << "phase " << phase << " obj_value: " << obj_value << endl;
	}
}

/*
	mode
	iter				iteration count
	obj_value			objective value
	inf_pr				constraint violation
	inf_du				dual infeasibility
	mu					barrier parameter
	d_norm				norm of primal step
	regularization_size
	alpha_du			step size for dual variables
	alpha_pr			step size for primal variables
	ls_trials
	ip_data
	ip_cq
 */
bool IpoptAdaptor::intermediate_callback(AlgorithmMode mode, Index iter, Number obj_value,
	Number inf_pr, Number inf_du, Number mu, Number d_norm, Number regularization_size, Number alpha_du, Number alpha_pr,
	Index ls_trials, const IpoptData* ip_data, IpoptCalculatedQuantities* ip_cq)
{
	if(file.is_open()){
		file << iter << ", " << obj_value;
		uint L = solver->maxLevel;
		for(uint l = 0; l <= L; l++){
			file << ", " << sqrt(error[l]);
		}
		for(uint l = 0; l <= L; l++){
			file << ", " << min(5.0, sqrt(error[l]));
		}
		if(problem == Phased)
			file << ", " << phase;
		file << endl;
	}
	return true;
}


}
