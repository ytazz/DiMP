#pragma once

#include <DiMP/DiMP.h>

class Mpc : public DiMP::CentroidCallback{
public:
	struct Phase{
		enum{
			R, RL, L, LR, Num,
		};
	};
	struct EndConfig{
        vec3_t basePos  ;
        vec3_t posOrigin;
        vec3_t posMin   ;
        vec3_t posMax   ;
        real_t stiffMax ;
        vec2_t cmpOffset;
    };

	DiMP::Graph*       graph;
	DiMP::Centroid*	   cen;
    //FILE*              fileDuration;
    //FILE*              fileCost;
    vector<EndConfig>  endConf;
        
    real_t comHeight;
	vec3_t startPos;
	vec3_t startOri;
	vec3_t desVel;
	real_t desDuration;
    int    N;
    
	DiMP::CentroidData  data_cur;  //< current state
	DiMP::CentroidData  data_ref;  //< reference state and input computed by most recent MPC optimization
	DiMP::CentroidData  data_ref_des;

	int     nx, nu;
	Vector  dx, du;
	Vector  u, uref;
	Matrix  Quuinv_Qux;

	bool   inputReady;
	bool   delayMode;
	int    updateCycle;
	int    count;
	real_t dt;
	real_t time;     //< real time
	int    phase;
	bool   planTrigger;

public:
	void Init       ();
	void UpdateState();
	void UpdateInput();
	void UpdateGain ();
	void Countup    ();
	void SavePlan   ();
	void SaveTraj   ();

	virtual void GetInitialState(DiMP::CentroidData& d);
	virtual void GetDesiredState(int k, real_t t, DiMP::CentroidData& d);

	Mpc();
};
