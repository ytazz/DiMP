﻿#pragma once

#include <DiMP/DiMP.h>

#include "myik.h"

class Mpc : public DiMP::WholebodyCallback{
public:
	DiMP::Wholebody*     wb;
	DiMP::WholebodyData  data_cur;  //< current state
	DiMP::WholebodyData  data_ref;  //< reference state and input computed by most recent MPC optimization
	DiMP::WholebodyData  data_ref_des;
	MyIK*                myik;

	Vector  dx, du;
	Vector  u, uref;
	Matrix  Quuinv_Qux;

	bool   ready;
	int    N;
	int    updateCycle;
	int    count;
	real_t dt;
	real_t time;     //< real time
	real_t timeMpc;  //< initial time of MPC prediction horizon

	vec3_t handOffset[2];
	
	CsvReader csv;

public:
	void Init       ();
	void UpdateState();
	void UpdateInput();
	void UpdateGain ();
	void Countup    ();

	virtual void GetInitialState(DiMP::WholebodyData& d);
	virtual void GetDesiredState(int k, real_t t, DiMP::WholebodyData& d);

	Mpc();
};
