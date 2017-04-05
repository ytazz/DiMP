#include <DiMP2/Solver.h>

namespace DiMP2{;

void Solver::StepSteepest(){
	for(uint i = 0; i < vars.size(); i++)
		vars[i]->ResetState();

	for(uint i = 0; i < cons.size(); i++){
		Constraint* con = cons[i];
		if(!con->enabled)
			continue;
		if(!con->active)
			continue;
		for(Links::iterator it = con->links.begin(); it != con->links.end(); it++){
			for(uint k = 0; k < con->nelem; k++)
				(*it)->Backward(k, -weights[con->level] * con->y[k]);
		}
	}
}

}
