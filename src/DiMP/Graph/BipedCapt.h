#pragma once

#include <DiMP/Graph/Node.h>

namespace DiMP {;

/*
 * simple DCM dynamics
 * 
 */

struct BipedCaptIcpCon;
struct BipedCaptSupConT;
struct BipedCaptSupConR;
struct BipedCaptSwgConT;
struct BipedCaptSwgConR;
struct BipedCaptDurationCon;
struct BipedCaptLandRangeConT;
struct BipedCaptLandRangeConR;
struct BipedCaptCopRangeCon;
struct BipedCaptIcpRangeCon;

class BipedCaptKey : public Keypoint {
public:
	int         side;          ///< 0: right support  1: left support
	V3Var*      var_sup_t;
	SVar*       var_sup_r;
	V3Var*      var_swg_t;
	SVar*       var_swg_r;
	V3Var*		var_icp;
	
	V3Var*      var_land_t;
	SVar*       var_land_r;
	V3Var*      var_cop;
	SVar*		var_duration;

	BipedCaptIcpCon*   con_icp;    ///< icp transition
	BipedCaptSupConT*  con_sup_t;  ///< support foot transition
	BipedCaptSupConR*  con_sup_r;  ///< support foot transition
	BipedCaptSwgConT*  con_swg_t;  ///< swing foot transition
	BipedCaptSwgConR*  con_swg_r;  ///< swing foot transition

	BipedCaptDurationCon*    con_duration;
	BipedCaptLandRangeConT*  con_land_range_t[4];
	BipedCaptLandRangeConR*  con_land_range_r[2];
	BipedCaptCopRangeCon*    con_cop_range[4];
	BipedCaptIcpRangeCon*    con_icp_range[4];
	
public:
	virtual void AddVar(Solver* solver);
	virtual void AddCon(Solver* solver);
	virtual void Prepare();
	virtual void Finish ();
	virtual void Draw(Render::Canvas* canvas, Render::Config* conf);

	BipedCaptKey();
};

class BipedCapt : public TrajectoryNode {
public:
	struct Step{
		vec3_t  sup_t;
		real_t  sup_r;
		vec3_t  swg_t;
		real_t  swg_r;
		vec3_t  icp;
		vec3_t  cop;
		real_t  duration;
		int     side;

		Step(){}
		Step(vec3_t _sup_t, real_t _sup_r, vec3_t _swg_t, real_t _swg_r, vec3_t _icp, vec3_t _cop, real_t _duration, int _side);
	};

	struct Param {
		real_t	gravity;				///< gravity (positive)
		real_t  T;                      ///< time constant of LIP
		real_t  comHeight;
		vec3_t	landPosMin[2];          ///< admissible range of foot relative to torso
		vec3_t  landPosMax[2];
		real_t  landOriMin;
		real_t  landOriMax;
		vec3_t  copMin;           ///< admissible range of CoP relative to foot
		vec3_t  copMax;
		real_t  vmax;
		real_t  wmax;
		real_t  tau_const;

		vector<Step>  steps;
	
		Param();
	};

	struct Snapshot{
		real_t  t;
		vec3_t  icp;
		vec3_t  cop;
		vec3_t  sup_t;
		real_t  sup_r;
		vec3_t  swg_t;
		real_t  swg_r;
			
		Snapshot();
	};

	Param	            param;
	Snapshot            snapshot;
	vector<Snapshot>    trajectory;
	bool                trajReady;

	virtual Keypoint*	CreateKeypoint() { return new BipedCaptKey(); }
	virtual void		Init();
	virtual void		Prepare();
	virtual void        Finish ();
	virtual void        CreateSnapshot(real_t time);
	virtual void        DrawSnapshot  (Render::Canvas* canvas, Render::Config* conf);
	virtual void        Draw          (Render::Canvas* canvas, Render::Config* conf);
		
	void CreateSnapshot(real_t t, Snapshot& s);
	void CalcTrajectory();

public:
	BipedCapt(Graph* g, string n);
	virtual ~BipedCapt();
};

struct BipedCaptIcpCon : Constraint {
	BipedCaptKey* obj[2];

	real_t  T, tau, alpha;
	vec3_t  cop, icp, icp_rhs;

	void Prepare();

	virtual void CalcCoef();
	virtual void CalcDeviation();
	virtual void CalcLhs();

	BipedCaptIcpCon(Solver* solver, string _name, BipedCaptKey* _obj, real_t _scale);
};

struct BipedCaptSupConT : Constraint {
	BipedCaptKey* obj[2];

	void Prepare();

	virtual void CalcCoef();
	virtual void CalcDeviation();
	virtual void CalcLhs();

	BipedCaptSupConT(Solver* solver, string _name, BipedCaptKey* _obj, real_t _scale);
};

struct BipedCaptSupConR : Constraint {
	BipedCaptKey* obj[2];

	void Prepare();

	virtual void CalcCoef();
	virtual void CalcDeviation();
	virtual void CalcLhs();

	BipedCaptSupConR(Solver* solver, string _name, BipedCaptKey* _obj, real_t _scale);
};

struct BipedCaptSwgConT : Constraint {
	BipedCaptKey* obj[2];

	void Prepare();

	virtual void CalcCoef();
	virtual void CalcDeviation();
	virtual void CalcLhs();

	BipedCaptSwgConT(Solver* solver, string _name, BipedCaptKey* _obj, real_t _scale);
};

struct BipedCaptSwgConR : Constraint {
	BipedCaptKey* obj[2];

	void Prepare();

	virtual void CalcCoef();
	virtual void CalcDeviation();
	virtual void CalcLhs();

	BipedCaptSwgConR(Solver* solver, string _name, BipedCaptKey* _obj, real_t _scale);
};

struct BipedCaptDurationCon : Constraint {
	BipedCaptKey* obj;

	real_t tau, tau_const;
	real_t vmax, vmax_inv;
	real_t wmax, wmax_inv;
	vec3_t dp, dpn;
	real_t dpnorm;
	real_t dr, drabs, drn;

	void Prepare();

	virtual void CalcCoef();
	virtual void CalcDeviation();
	
	BipedCaptDurationCon(Solver* solver, string _name, BipedCaptKey* _obj, real_t _scale);
};

struct BipedCaptLandRangeConT : Constraint {
	BipedCaptKey* obj;

	vec3_t dir, dir_abs, r, ez;
	real_t lim, theta;
	mat3_t R;

	void Prepare();

	virtual void CalcCoef();
	virtual void CalcDeviation();
	
	BipedCaptLandRangeConT(Solver* solver, string _name, BipedCaptKey* _obj, vec3_t _dir, real_t _lim, real_t _scale);
};

struct BipedCaptLandRangeConR : Constraint {
	BipedCaptKey* obj;

	real_t dir, theta_land, theta_sup, r;
	real_t lim;

	void Prepare();

	virtual void CalcCoef();
	virtual void CalcDeviation();
	
	BipedCaptLandRangeConR(Solver* solver, string _name, BipedCaptKey* _obj, real_t _dir, real_t _lim, real_t _scale);
};

struct BipedCaptCopRangeCon : Constraint {
	BipedCaptKey* obj;

	vec3_t dir, dir_abs, r, ez;
	real_t lim, theta;
	mat3_t R;

	void Prepare();

	virtual void CalcCoef();
	virtual void CalcDeviation();
	
	BipedCaptCopRangeCon(Solver* solver, string _name, BipedCaptKey* _obj, vec3_t _dir, real_t _lim, real_t _scale);
};

struct BipedCaptIcpRangeCon : Constraint {
	BipedCaptKey* obj;

	vec3_t dir, dir_abs, r, ez;
	real_t lim, theta;
	mat3_t R;

	void Prepare();

	virtual void CalcCoef();
	virtual void CalcDeviation();
	
	BipedCaptIcpRangeCon(Solver* solver, string _name, BipedCaptKey* _obj, vec3_t _dir, real_t _lim, real_t _scale);
};

}
