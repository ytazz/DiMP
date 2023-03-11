#pragma once

#include <DiMP/DiMP.h>

#include "myik.h"

class Mpc : public DiMP::WholebodyCallback{
public:
	DiMP::Wholebody*     wb;
	DiMP::WholebodyData  data_cur;  //< current state
	vector<DiMP::WholebodyData>  data_tmp;
	DiMP::WholebodyData  data_ref;  //< reference state and input computed by most recent MPC optimization
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

	virtual void CalcIK(int ichain, const vec3_t& pe_local, const quat_t& qe_local, vvec_t& joint, vvec_t& error, vmat_t& Jq, vmat_t& Je, bool calc_jacobian);
	virtual void Setup (int k, real_t t, DiMP::WholebodyData& data);

	Mpc();
};
