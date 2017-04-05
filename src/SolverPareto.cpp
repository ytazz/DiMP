#include <DiMP2/Solver.h>

#include <algorithm>

#include <omp.h>

#include <Foundation/UTPreciseTimer.h>

namespace DiMP2{;

void Solver::StepPareto(){
	UTPreciseTimer ptimer;
	
	#pragma omp parallel for
	for(int i = 0; i < (int)cons.size(); i++)
		cons[i]->ResetState();

	for(int L = maxLevel; L >= 0; L--){
		ptimer.CountUS();
	
		uint N = maxIter[L];
		for(uint n = 1; n <= N; n++){
			for(uint l = 0; l <= L; l++){
				for(uint i = 0; i < cons_level[l].size(); i++){
					Constraint* con = cons_level[l][i];
					for(uint k = 0; k < con->nelem; k++)
						con->UpdateMultiplier(k);
				}
			}
			
			/*
			// ガウス-ザイデルの並列計算
			// 同一phase内（干渉しない拘束）は並列実行する
			for(int phase = 0; phase < (int)cons_arranged.size(); phase++){
				int ncon = (int)cons_arranged[phase].size();
				#pragma omp parallel for if(ncon > 10)
				for(int i = 0; i < ncon; i++){
					Constraint* con = cons_arranged[phase][i];
					if(!con->enabled)
						continue;
					if(!con->active)
						continue;
					if((int)con->level > l)
						continue;
					for(uint k = 0; k < con->nelem; k++)
						con->UpdateMultiplierCorr(k);
				}
			}
			*/
		}

		//DSTR << "level " << L << ": " << ptimer.CountUS() << endl;
	}

}

}
