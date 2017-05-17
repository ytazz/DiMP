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

	// quaternionの位置
	quatIndex.clear();
	for(uint i = 0; i < solver->vars.size(); i++){
		Variable* var = solver->vars[i];
		if(var->locked)
			continue;

		if(var->nelem == 4)
			quatIndex.push_back(nvar);
	}

	// 拘束誤差を記憶する配列
	error.resize(solver->maxLevel+1);
	fill(error.begin(), error.end(), 0.0);
	error_min.resize(solver->maxLevel+1);
	fill(error_min.begin(), error_min.end(), 0.0);
	
	app = IpoptApplicationFactory();
	// 打ち切り誤差
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

	// 最大反復回数
	app->Options()->SetIntegerValue("max_iter", 1000);
	
	//app->Options()->SetNumericValue("mu_target", 0.1);
	
	// ヘシアンは内部で近似計算
	app->Options()->SetStringValue ("hessian_approximation", "limited-memory");
	
	// watchdog: 小さいステップ幅がつづいたときに喝を入れる作用がある模様
	// 大域的最適解を得る可能性が上がるが，評価値のプロットにスパイクがのるので見た目はあまり良くない
	// 以下を0にするとwatchdog無効化
	app->Options()->SetIntegerValue("watchdog_shortened_iter_trigger", 0);
	// 以下をyesにするとline search無効化
	//app->Options()->SetStringValue ("accept_every_trial_step", "yes");
	
	// メッセージレベル: 必要なことのみ
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

// quaternionの時間微分から角速度を返す行列
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

		// Phasedの場合は現在の優先度レベルのみ
		if(problem == Phased && con->level != level)
			continue;
		
		real_t w = (problem == Weighted ? solver->weights[con->level] : 1.0);
		vec3_t g = link->Backward(con->y);	///< ヤコビアン転置 * 拘束偏差

		// quaternionの場合は回転ベクトルから差分への変換行列をかける
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

	// 変数の数
	nvar = 0;
	for(uint i = 0; i < solver->vars.size(); i++){
		Variable* var = solver->vars[i];
		if(var->locked)
			continue;

		nvar += var->nelem;
	}
	n = nvar;
	cout << "nvar: " << nvar << endl;

	// 拘束の数
	//  単位quaternion拘束
	ncon = quatIndex.size();
	//  Phasedなら上位優先度の誤差拘束
	if(problem == Phased)
		ncon += phase;
	m = ncon;
	cout << "ncon: " << ncon << endl;

	// ヤコビアンの非0要素数
	//  quaternion拘束
	nnz_jac_g = 4 * quatIndex.size();
	if(problem == Phased){
		// 上位優先度の拘束誤差については疎性を考慮しない（全変数について非0係数として扱う）
		nnz_jac_g += (phase * nvar);
	}
	cout << "nnz_jac_g: " << nnz_jac_g << endl;

	// ヘシアンの非0要素数
	nnz_h_lag = 0;

	// 0-based or 1-based
	index_style = C_STYLE;

	return true;
}

bool IpoptAdaptor::get_bounds_info(Index n, Number* x_l, Number* x_u, Index m, Number* g_l, Number* g_u){
	Number inf = 1.0e10;
	uint nquat = quatIndex.size();

	// 変数の上下限
	for(Index j = 0; j < n; j++){
		x_l[j] = -1.0;
		x_u[j] =  1.0;
	}
	// quaternionは[-1, 1]
	for(uint i = 0; i < nquat; i++){
		for(uint k = 0; k < 4; k++){
			x_l[quatIndex[i] + k] = -1.0;
			x_u[quatIndex[i] + k] =  1.0;
		}
	}
	
	// quaternionのノルム拘束: 上下限を1.0として等式制約
	for(Index i = 0; i < nquat; i++){
		g_l[i] = 1.0-0.01;
		g_u[i] = 1.0+0.01;
	}
	// 上位優先度の拘束誤差
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
	// 変数の初期値
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
	// 変数値をセット
	SetVariable(x);

	uint L = solver->maxLevel;
	for(uint l = 0; l <= L; l++){
		error[l] = CalcCost(l);
	}

	// 評価関数の値
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
	// 変数値をセット
	SetVariable(x);

	for(Index i = 0; i < n; i++)
		grad_f[i] = 0.0;

	// 評価関数の勾配
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

	// quaternionのノルム拘束
	uint nquat = quatIndex.size();
	for(Index i = 0; i < nquat; i++){
		g[i] = 0.0;
		for(int j = 0; j < 4; j++){
			Number v = x[quatIndex[i] + j];
			g[i] += v*v;
		}
	}

	// 上位優先度の誤差拘束
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

	// ヤコビアンの構造
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
	// ヤコビアンの値
	else{
		// 数値微分でヤコビアンを計算
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
	status		計算結果
	x			変数の値
	z_L			下限拘束の乗数
	z_U			上限拘束の乗数
	g			拘束の値
	lambda		拘束の乗数
	obj_value	評価関数の値
	ip_data		付加情報
	ip_cq;
 */
void IpoptAdaptor::finalize_solution(SolverReturn status, Index n, const Number* x, const Number* z_L, const Number* z_U,
	Index m, const Number* g, const Number* lambda, Number obj_value, const IpoptData* ip_data, IpoptCalculatedQuantities* ip_cq)
{
	// 最小化された拘束誤差を記憶
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
