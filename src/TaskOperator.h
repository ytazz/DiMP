#ifndef TASK_OP_H
#define TASK_OP_H

#include <DiMP2/TrajectoryOperator.h>
#include <DiMP2/Matching.h>

namespace DiMP2{;
class MatchingTask;
/** matching operator
	-�}�b�`���O�^�X�N task0 �� task1 �̘_�����Z���`�����{�N���X
 **/

class MatchingOR : public TrajectoryOR{
public:
	MatchingTask*	match[2];
public:
	MatchingOR(Graph* g, MatchingTask* _match0, MatchingTask*	_match1);
	void CalcCorrection();
};

// if task0 then task1
class MatchingIfThen : public TrajectoryIfThen{
public:
	MatchingTask*	match[2];
public:
	MatchingIfThen(Graph* g, MatchingTask* _match0, MatchingTask*	_match1);
	void CalcCorrection();
};

class MatchingNOT : public TrajectoryNOT{
public:
	MatchingTask*	match;
public:
	MatchingNOT(Graph* g, MatchingTask* _match);
	void CalcCorrection();
};


}

#endif