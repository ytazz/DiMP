#include <DiMP2/Solver.h>

#include <boost/foreach.hpp>
#define foreach BOOST_FOREACH
using namespace boost;

namespace DiMP2{;

void Solver::StepDynamic(){
	foreach(Constraint* con, cons)
		con->ResetState();

	uint n;
	vector< pair<Constraint*, uint> >	cons_over;		//< �ߏ�S�������m���ꂽ�S��
	uint Lmin, Lmax;			//< �ߏ�S�����ōŏ�/�ő�̗D��x���x��

	for(n = 1; n <= maxIter[0]; n++){
		foreach(Constraint* con, cons_arranged){
			for(uint k = 0; k < con->nelem; k++){
				con->dlprev[k] = con->dl[k];
				
				// �ߏ�S���̌��m�ɂ�薳�������ꂽ�S���̓X�L�b�v
				if(con->over_off[k])
					continue;

				con->UpdateMultiplierCorr(k);
			}
		}
	
		foreach(Constraint* con, cons_arranged){
			for(uint k = 0; k < con->nelem; k++){
				real_t ddl = std::abs(con->dlprev[k] - con->dl[k]);
				real_t dl = std::abs(con->dl[k]);
				//real_t over = num / den;
				//if(over < 0.01)
				//	con->over[k] = true;

				//if(hoge && !con->over[k] && dl > 0.001 && ddl < 0.5 * dl){
				//	//DSTR << "over detect " << con->name << endl;
				//	con->over[k] = true;
				//}
			}
		}

		// �ߏ�S�������m���ꂽ�S�������߂�
		cons_over.clear();
		Lmin = numeric_limits<int>::max(), Lmax = 0;

		foreach(Constraint* con, cons_arranged){
			for(uint k = 0; k < con->nelem; k++){
				if(con->over[k]){
					cons_over.push_back(make_pair(con, k));
					Lmin = std::min(Lmin, con->level);
					Lmax = std::max(Lmax, con->level);
				}
			}
		}

		// �ߏ�S���̒��ŗD��x���x���ő�̂��̂𖳌����i���ׂē����x���̏ꍇ�͖��������Ȃ��j
		//bool deactivated = false;
		for(uint i = 0; i < cons_over.size(); i++){
			Constraint* con = cons_over[i].first;
			uint k = cons_over[i].second;
			
			if(Lmin < Lmax && con->level == Lmax){
				con->over_off[k] = true;
				//DSTR << con->name << "[" << k << "] deactivated at " << n << "/" << maxIter << endl;
				//DSTR << con->dlprev[k] << " " << con->dl[k] << " " << endl;
				//DSTR << con->name << "[" << k << "] deactivated" << endl;
			}
		}
		
		foreach(Constraint* con, cons_arranged){
			for(uint k = 0; k < con->nelem; k++)
				con->over[k] = false;
		}
	}

	// �ꎞ�����������S�������ɖ߂�
	foreach(Constraint* con, cons_arranged){
		for(uint k = 0; k < con->nelem; k++)
			con->over_off[k] = false;
	}

	DSTR << endl;
}

}
