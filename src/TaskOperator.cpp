#include <DiMP2/TaskOperator.h>
#include <boost/bind.hpp>
#include <boost/foreach.hpp>
#include <DiMP2/Graph.h>
#define foreach BOOST_FOREACH
using namespace boost;

namespace DiMP2{;
MatchingOR::MatchingOR(Graph* g, MatchingTask* _match0, MatchingTask* _match1)
	:TrajectoryOR(g, (TrajectoryNode*)_match0, (TrajectoryNode*)_match1){
		match[0] = _match0;
		match[1] = _match1;
}

void MatchingOR::CalcCorrection(){
	const real_t eps = 1.0e-8;
	real_t gain0 = match[1]->CalcTrajError();
	real_t gain1 = match[0]->CalcTrajError();
	real_t normalize = gain0 + gain1;
	if( normalize < eps )
		return ;
	else{
		match[0]->traj_gain = gain0/normalize;
		match[1]->traj_gain = gain1/normalize;
		for(size_t i =0; i<2 ; i++){
			foreach(Keypoint* key, match[i]->traj){
				MatchingKey* mkey = (MatchingKey*)key;
				for(size_t j = 0; j<3 ; j++){
					if(mkey->con_pos_t[j])
						mkey->con_pos_t[j]->trajgain = match[i]->traj_gain;
					if(mkey->con_pos_r[j])
						mkey->con_pos_r[j]->trajgain = match[i]->traj_gain;
					if(mkey->con_vel_t[j])
						mkey->con_vel_t[j]->trajgain = match[i]->traj_gain;
					if(mkey->con_vel_r[j])
						mkey->con_vel_r[j]->trajgain = match[i]->traj_gain;
				}
			}
		}

	}
}

MatchingIfThen::MatchingIfThen(Graph* g, MatchingTask* _match0, MatchingTask*	_match1)
	:TrajectoryIfThen(g, (TrajectoryNode*)_match0, (TrajectoryNode*)_match1){
		match[0] = _match0;
		match[1] = _match1;
}

void MatchingIfThen::CalcCorrection(){
	real_t e0 =  match[0]->CalcTrajError();
	real_t e1 =  match[1]->CalcTrajError();	
	// e0 が十分小さい時，if task0 then task1 は task0 and task1と同義
	cout << e0 << endl;
	cout << e1 << endl;
	if(e0 <  3.0* 1.0e-2 ){
		match[0]->traj_gain = 1.0;
		match[1]->traj_gain = 1.0;	
	}
	else{
	const real_t eps = 1.0e-8;
	//  not task0 の誤差関数の定義:　enot = e1 / ( a*e0 + 1)
	real_t a = 10.0;
	real_t note0_den =  a*e0 + 1;
	real_t note0_den2 = note0_den*note0_den;

	real_t gain0 = (-a*e1)/note0_den2;
	real_t gain1 = 1/note0_den;
	real_t normalize = e0 + e1;
	if(normalize< eps)
		return;
	match[0]->traj_gain = gain0/normalize;	
	match[1]->traj_gain = gain1/normalize;
	}
		for(size_t i =0; i<2 ; i++){
			foreach(Keypoint* key, match[i]->traj){
				MatchingKey* mkey = (MatchingKey*)key;
				for(size_t j = 0; j<3 ; j++){
					if(mkey->con_pos_t[j])
						mkey->con_pos_t[j]->trajgain = match[i]->traj_gain;
					if(mkey->con_pos_r[j])
						mkey->con_pos_r[j]->trajgain = match[i]->traj_gain;
					if(mkey->con_vel_t[j])
						mkey->con_vel_t[j]->trajgain = match[i]->traj_gain;
					if(mkey->con_vel_r[j])
						mkey->con_vel_r[j]->trajgain = match[i]->traj_gain;
				}
			}
		}
}

MatchingNOT::MatchingNOT(Graph* g, MatchingTask* _match0)
	:TrajectoryNOT(g, (TrajectoryNode*)_match0){
		match = _match0;
}

void MatchingNOT::CalcCorrection(){
	// 各拘束の二乗ノルム
	real_t e0 =  match->CalcTrajError();
	const real_t eps = 1.0e-8;
	//  not con　の誤差関数の定義:　enot = 1 / ( a*e0 + 1)
	real_t a = 10.0;
	real_t enot_den =  a*e0 + 1;
	if(enot_den < eps)
		return;
	else{
		match->traj_gain = -a / enot_den;
		foreach(Keypoint* key, match->traj){
			MatchingKey* mkey = (MatchingKey*)key;
			for(size_t j = 0; j<3 ; j++){
				if(mkey->con_pos_t[j])
					mkey->con_pos_t[j]->trajgain = match->traj_gain;
				if(mkey->con_pos_r[j])
					mkey->con_pos_r[j]->trajgain = match->traj_gain;
				if(mkey->con_vel_t[j])
					mkey->con_vel_t[j]->trajgain = match->traj_gain;
				if(mkey->con_vel_r[j])
					mkey->con_vel_r[j]->trajgain = match->traj_gain;
			}
		}
	}

}

}
