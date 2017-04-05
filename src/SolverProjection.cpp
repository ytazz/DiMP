#include <DiMP2/Solver.h>

#include <boost/foreach.hpp>
#define foreach BOOST_FOREACH
using namespace boost;

namespace DiMP2{;

/*void Solver::IterateProjection(uint L, bool corr_or_proj){
	real_t diff_sum;
	real_t dy_sum, dyd_sum;

	uint n;
	for(n = 1; n <= maxIter[L]; n++){
		foreach(Constraint* con, cons_arranged){
			if(corr_or_proj && con->level == L){
				for(uint k = 0; k < con->nelem; k++)
					con->UpdateMultiplierCorr(k);
			}
			if(!corr_or_proj && con->level < L){
				for(uint k = 0; k < con->nelem; k++)
					con->UpdateMultiplierProj(k);
			}
		}
		
		diff_sum = 0.0;
		dy_sum   = 0.0;
		dyd_sum  = 0.0;
		
		foreach(Constraint* con, cons_arranged){
			diff_sum += (con->dy - con->dyd).square();
			dy_sum   += con->dy.square();
			dyd_sum  += con->dyd.square();
		}
		
		// 拘束誤差の変化量と目標変化量の偏差が十分小さくなったら終了
		//  絶対誤差
		if(diff_sum < epsAbs2)
			break;
		//  相対誤差
		if(diff_sum < epsRel2 * (dy_sum + dyd_sum))
			break;
	}
	//DSTR << "# of itr: " << n << endl;
}
*/

void Solver::StepProjection(){
	foreach(Variable* var, vars)
		var->dtmp.resize(maxLevel+1);

	// 各優先度レベルについて：
	for(int L = maxLevel; L >= 0; L--){
		/*	各レベルごとに乗数lと誤差変化量deは個別に計算するので毎回リセットする
			一方で変数変化量はリセットしない．これにより全レベルの修正量の和が求まる
		 */
		foreach(Constraint* con, cons_arranged)
			con->ResetState();
		foreach(Variable* var, vars)
			var->ResetState();	

		// この優先度レベルの拘束誤差を減らすためのループ
		//IterateProjection(L, true);
		for(uint n = 1; n <= maxIter[L]; n++){
			foreach(Constraint* con, cons_arranged){
				if(con->level == L){
					for(uint k = 0; k < con->nelem; k++)
						con->UpdateMultiplierCorr(k);
				}
			}
		}

		// 低優先度レベルの変化量を射影するためのループ
		for(uint n = 1; n <= maxIter[L]; n++){
			foreach(Constraint* con, cons_arranged){
				if(con->level < L){
					for(uint k = 0; k < con->nelem; k++)
						con->UpdateMultiplierProj(k);
				}
			}
		}

		// 直線探索
		real_t step = 3.0;
		const real_t dy_tol = 1.0e-1;
		const real_t dy_eps = 1.0e-10;

		foreach(Constraint* con, cons_arranged){
			if(con->level > L)
				continue;

			for(uint k = 0; k < con->nelem; k++){
				real_t dy_abs  = std::abs(con->dy[k]);
				real_t dyd_abs = std::abs(con->dyd[k]);
				if(dy_abs < dy_eps)
					continue;
				real_t rate = con->dyd[k] / con->dy[k];
				if(rate < 0 || dyd_abs < dy_tol)
					rate = dy_tol / dy_abs;
				step = std::min(step, rate);
			}
		}

		//DSTR << "phase: " << L << " step: " << step << endl;

		foreach(Variable* var, vars)
			var->dtmp[L] = step * var->d;
	}

	foreach(Variable* var, vars){
		var->d.clear();
		for(uint L = 0; L <= maxLevel; L++)
			var->d += var->dtmp[L];
	}

}

}
