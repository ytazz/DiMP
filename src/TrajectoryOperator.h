#ifndef TRJ_OP_H
#define TRJ_OP_H

#include <DiMP2/Node.h>

namespace DiMP2{;
class Graph;
/** tfj operator
	- Trajectory trj0 と trj1 の論理演算を定義する基本クラス
 **/

enum TrjOperations{
	OR,
	IFTHEN,
	NOT,
	nTrjOperations,
};

class TrajectoryOperator{
public:
	TrajectoryNode*	traj[2];
	TrjOperations o_mode;
	Graph* graph;
public:
	TrajectoryOperator(Graph* g, TrajectoryNode*	_trj0, TrajectoryNode*	_trj1);
	virtual void CalcCorrection(){}
};

class TrajectoryOR : public TrajectoryOperator{
public:

public:
	TrajectoryOR(Graph* g, TrajectoryNode* _trj0, TrajectoryNode*	_trj1);
};

class TrajectoryIfThen : public TrajectoryOperator{
public:

public:
	TrajectoryIfThen(Graph* g, TrajectoryNode* _trj0, TrajectoryNode*	_trj1);
};

class TrajectoryNOT:public TrajectoryOperator{
public:

public:
	TrajectoryNOT(Graph* g, TrajectoryNode* _trj0);
};


}
#endif