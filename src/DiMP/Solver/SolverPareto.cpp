#include <DiMP/Solver/Solver.h>

#include <algorithm>

#include <omp.h>

#include <Foundation/UTPreciseTimer.h>

namespace DiMP{;

void Solver::StepPareto(){
	UTPreciseTimer ptimer;
	
	#pragma omp parallel for
	for(int i = 0; i < (int)cons.size(); i++)
		cons[i]->ResetState();

	for(int L = maxLevel; L >= 0; L--){
		ptimer.CountUS();
	
		int N = maxIter[L];
		for(int n = 1; n <= N; n++){
			for(int l = 0; l <= L; l++){
				for(int i = 0; i < (int)cons_level[l].size(); i++){
					Constraint* con = cons_level[l][i];
					for(int k = 0; k < con->nelem; k++)
						con->UpdateMultiplier(k);
				}
			}
			
			/*
			// �K�E�X-�U�C�f���̕���v�Z
			// ����phase���i�����Ȃ��S���j�͕�����s����
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
