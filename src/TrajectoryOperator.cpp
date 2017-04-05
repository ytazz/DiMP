#include <DiMP2/TrajectoryOperator.h>

namespace DiMP2{;
TrajectoryOperator::TrajectoryOperator(Graph* g, TrajectoryNode* _trj0, TrajectoryNode*	_trj1 = NULL){
	graph = g;
	traj[0] = _trj0;
	if(_trj1){
		traj[1] = _trj1;
		for(int i = 0; i<2 ; i++)
			traj[i]->parent = this;
	}
	else
		traj[0]->parent = this;
}
//real_t	TrajectoryOperator::CalcErrorFunc(size_t trj_0or1){ return 0; }

TrajectoryOR::TrajectoryOR(Graph* g, TrajectoryNode* _trj0, TrajectoryNode*	_trj1):TrajectoryOperator(g, _trj0, _trj1){
	o_mode = OR;
}


TrajectoryIfThen::TrajectoryIfThen(Graph* g, TrajectoryNode* _trj0, TrajectoryNode*	_trj1):TrajectoryOperator(g, _trj0, _trj1){
	o_mode = IFTHEN;
}

TrajectoryNOT::TrajectoryNOT(Graph* g, TrajectoryNode* _trj0):TrajectoryOperator(g, _trj0){
	o_mode = NOT;
}


}
