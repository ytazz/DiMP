#include <DiMP/Solver/Solver.h>
#include <DiMP/Graph/Graph.h>
#include <DiMP/Graph/Object.h>
#include <DiMP/Render/Config.h>

#include <Foundation/UTPreciseTimer.h>

#include <omp.h>

namespace DiMP{;

//-------------------------------------------------------------------------------------------------

Solver::Solver(){
	maxIterDefault = 50;
	
	algo		= Algorithm::Pareto;
	order		= Iteration::Sequential;

	SetTolerance(1.0e-5, 1.0e-2);

	stepSizeMin = 1.0e-1;
	stepSizeMax = 1.0e+1;

	doLog[Logging::MajorLoop] = false;
	doLog[Logging::MinorLoop] = false;

	ready = false;
}

void Solver::SetNumIteration(uint n, uint l){
	if(l >= maxIter.size())
		maxIter.resize(l+1);
	maxIter[l] = n;
}

void Solver::SetAlgorithm(int t){
	if(0 <= t && t < Algorithm::numTypes)
		algo = t;
}

void Solver::SetIterationOrder(int o){
	order = o;
}

void Solver::SetTolerance(real_t eps_abs, real_t eps_rel){
	epsAbs = eps_abs;
	epsRel = eps_rel;
	epsAbs2 = epsAbs * epsAbs;
	epsRel2 = epsRel * epsRel;
}

void Solver::EnableLogging(int mode, bool on){
	if(0 <= mode && mode < Logging::numModes){
		doLog[mode] = on;
		ready = false;
	}
}

void Solver::AddVar(Variable* var){
	#pragma omp critical
	{
		vars.push_back(var);
	}
}

void Solver::AddCon(Constraint* con){
	#pragma omp critical
	{
		con->solver = this;
		cons.push_back(con);
	}
}

void Solver::AddLink(Link* link){
	#pragma omp critical
	{
		links.push_back(link);
	}
}

void Solver::LogLabel(int mode){
	if(mode == Logging::MajorLoop){
		file[mode] << "e";
		for(int i = 0; i < ConTag::NumTypes; i++)
			file[mode] << ", " << ConNames[i];
		for(int i = 0; i <= maxLevel; i++)
			file[mode] << ", lv" << i;
	}
	else{

	}
	file[mode] << endl;
}

void Solver::LogValue(int mode){
	if(mode == Logging::MajorLoop){
		file[mode] << e;
		for(int i = 0; i < ConTag::NumTypes; i++)
			file[mode] << ", " << e_type[i];
		for(int i = 0; i <= maxLevel; i++)
			file[mode] << ", " << e_level[i];
	}
	else{

	}
	file[mode] << endl;
}

void Solver::Init(){
	// 優先度の最大値を求める
	maxLevel = 0;
	for(uint i = 0; i < cons.size(); i++)
		maxLevel = std::max(maxLevel, cons[i]->level);
	e_level.resize(maxLevel+1);
	e_type .resize(ConTag::NumTypes);

	// レベル数とmaxIterのサイズが合わせてデフォルト値をセット
	uint sz = maxIter.size();
	uint sznew = maxLevel+1;
	if(sz < sznew){
		maxIter.resize(sznew);
		for(uint i = sz; i < sznew; i++) 
			maxIter[i] = maxIterDefault;
	}

	// ログ有効時
	int i = Logging::MajorLoop;
	if(doLog[i]){
		if(!file[i].is_open()){
			file[i].open("log_major.csv");
			LogLabel(i);
		}
		LogValue(i);
	}
	else{
		if(file[i].is_open())
			file[i].close();
	}

	ready = true;
}

real_t Solver::CalcCost(real_t alpha, int level){
	real_t cost = 0.0;
	for(uint i = 0; i < vars.size(); i++)
		vars[i]->Modify(alpha);

	for(uint i = 0; i < graph->trees.size(); i++)
		graph->trees[i]->root->ForwardKinematics();
	graph->Prepare();
	
	for(uint i = 0; i < cons.size(); i++){
		Constraint* con = cons[i];
		if(!con->enabled)
			continue;
		
		// 範囲制約などで新たにactive化するかも知れない
		con->CalcCoef ();
		con->CalcError();
		
		if(!con->active)
			continue;
		if(level != -1 && con->level > level)
			continue;

		if(algo == Algorithm::Steepest){
			cost += weights[con->level] * con->y.square();
		}
		if(algo == Algorithm::Pareto){
			// Pareto改善性を維持しながら総コストを最小化する幅
			cost += con->y.square();
		}
	}
	return cost;
}

real_t Solver::CalcStepSize(int level){
	// amin < amaxじゃない場合は直線探索無効
	if(stepSizeMin >= stepSizeMax)
		return stepSizeMin;

	real_t amax = stepSizeMax;
	real_t amin = stepSizeMin;
	const real_t cutoff = 0.001;
	real_t a[3], aprobe;
	real_t c[3], cprobe;
	a[0] = 0.0;
	c[0] = CalcCost(a[0], level);
	a[1] = a[2] = 1.0;
	c[1] = c[2] = CalcCost(1.0, level);
	if(c[2] < c[0]){
		do{
			a[2] *= 2.0;
			c[2] = CalcCost(a[2], level);
		}while(c[2] < c[0] && a[2] < amax);
		if(a[2] >= amax)
			return amax;
	}
	else{
		do{
			a[1] *= 0.5;
			c[1] = CalcCost(a[1], level);
		}while(c[1] > c[0] && a[1] > amin);
		if(a[1] <= amin)
			return amin;
	}

	real_t adiff = a[2] - a[0];

	while(true){
		if(a[2] - a[0] < cutoff * adiff)
			break;

		if(a[1] - a[0] > a[2] - a[1]){
			aprobe = 0.5 * (a[0] + a[1]);
			cprobe = CalcCost(aprobe, level);
			if(cprobe < c[1]){
				a[2] = a[1];
				a[1] = aprobe;
				c[2] = c[1];
				c[1] = cprobe;
			}
			else{
				a[0] = aprobe;
				c[0] = cprobe;
			}
		}
		else{
			aprobe = 0.5 * (a[1] + a[2]);
			cprobe = CalcCost(aprobe, level);
			if(cprobe < c[1]){
				a[0] = a[1];
				a[1] = aprobe;
				c[0] = c[1];
				c[1] = cprobe;
			}
			else{
				a[2] = aprobe;
				c[2] = cprobe;
			}
		}
	}
	return 0.5 * (a[0] + a[2]);
}

void Solver::Step(){
	UTPreciseTimer ptimer;
	ptimer.CountUS();

	if(!ready)
		Init();

	// 優先度別に有効な拘束をまとめる
	cons_level.resize(maxLevel+1);
	for(int l = 0; l <= maxLevel; l++)
		cons_level[l].clear();
	
	// 変数の変化量（とその変化量）をリセット
	#pragma omp parallel for
	for(int i = 0; i < (int)vars.size(); i++)
		vars[i]->ResetState();
	
	// 拘束誤差と微分係数を計算．誤差は種別，優先度別にも集計
	e = 0.0;
	fill(e_type .begin(), e_type .end(), 0.0);
	fill(e_level.begin(), e_level.end(), 0.0);
	#pragma omp parallel for
	for(int i = 0; i < (int)cons.size(); i++){
		Constraint* con = cons[i];
		if(!con->enabled)
			continue;

		con->CalcCoef();
		con->CalcError();

		if(con->active){
			// 二乗和でとる場合
			e_type [con->tag  ] += con->y.square();
			e_level[con->level] += con->y.square();

			cons_level[con->level].push_back(con);
		}
	}

	// ノルムをとる場合
	for(int i = 0; i < ConTag::NumTypes; i++)
		e_type[i] = sqrt(e_type[i]);
	for(int i = 0; i < maxLevel+1; i++)
		e_level[i] = sqrt(e_level[i]);
	
	// ヤコビ行列，目標誤差修正量
	#pragma omp parallel for
	for(int i = 0; i < (int)cons.size(); i++)
		cons[i]->CalcCorrection();

	// 選択されている手法を用いて変数の変化量を計算
	switch(algo){
	//case Algorithm::Steepest:		StepSteepest();		break;
	case Algorithm::Pareto:			StepPareto();		break;
	}

	// 直線探索
	real_t alpha = 1.0;
	if(algo == Algorithm::Steepest){
		alpha = CalcStepSize(-1);
	}
	if(algo == Algorithm::Pareto){
		for(int L = maxLevel; L >= 0; L--){
			alpha = CalcStepSize(L);
			if(L > 0){
				// ステップ幅がほぼ0だとNG
				if(alpha == stepSizeMin)
					continue;

				// 上位優先度の誤差増大量が大きいとNG
				real_t c0 = CalcCost(0.0  , L-1);
				real_t c1 = CalcCost(alpha, L-1);
				const real_t rate   = 1.1;
				const real_t offset = 0.1;
				if(c1 > rate * c0 + offset)
					continue;
			}
			break;
		}
	}
	
	// modify variables
	#pragma omp parallel for
	for(uint i = 0; i < vars.size(); i++)
		vars[i]->Modify(alpha);

	// ログ有効時
	int i = Logging::MajorLoop;
 	if(doLog[i]){
		LogValue(i);
	}

}

void Solver::Clear(){
	vars  .clear();
	cons  .clear();
	links .clear();
}

void Solver::Reset(){
	for(uint i = 0; i < vars.size(); i++)
		vars[i]->Reset();
}

}
