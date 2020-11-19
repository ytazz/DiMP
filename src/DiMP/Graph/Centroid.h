#pragma once

#include <DiMP/Graph/Node.h>

namespace DiMP {;

struct CentroidPosConT;
struct CentroidPosConR;
struct CentroidVelConT;
struct CentroidVelConR;
struct CentroidEndPosCon;
struct CentroidEndRangeCon;
struct CentroidEndCmplCon;
struct CentroidContactGapCon;
struct CentroidContactForceCon;
struct CentroidContactCmplCon;

/**
	centroidal dynamics model
*/
class CentroidKey : public Keypoint {
public:
	V3Var*  var_pos_t;    ///< position        
	QVar*   var_pos_r;    ///< orientation     
	V3Var*  var_vel_t;    ///< velocity        
	V3Var*  var_vel_r;    ///< angular velocity

	CentroidPosConT*  con_pos_t;
	CentroidPosConR*  con_pos_r;
	CentroidVelConT*  con_vel_t;
	CentroidVelConR*  con_vel_r;

	struct End{
		V3Var*                 var_pos;
		V3Var*                 var_vel;
		
		CentroidEndPosCon*     con_pos;
		CentroidEndRangeCon*   con_range[3];
		CentroidEndCmplCon*    con_cmpl;
	};

	struct Contact{
		SVar*                     var_active[2];
		SVar*                     var_force[3];    //< contact force
		
		RangeConS*                con_active[2];
		CentroidContactGapCon*    con_gap;
		CentroidContactForceCon*  con_force[3];
		CentroidContactCmplCon*   con_cmpl;
		
		vec3_t  f;
	};

	vector<End>          ends;
	vector<Contact>      contacts;

public:
	virtual void AddVar(Solver* solver);
	virtual void AddCon(Solver* solver);
	virtual void Prepare();
	virtual void Draw(Render::Canvas* canvas, Render::Config* conf);

	CentroidKey();
};

class Centroid : public TrajectoryNode {
public:
	struct Param {
		struct End{
			vec3_t  rangeMin;
			vec3_t  rangeMax;
		};
		struct Contact{
			int     iend;     //< end-effector index
			vec3_t  pos;
		};

		real_t	m;  //< mass
		real_t  I;  //< inertia
		real_t	g;  //< gravity
		real_t  h;  //< time resolution
		real_t  mu;
		real_t  fmax;
		real_t  dmax;
		real_t  p;
		
		vector<End>       ends;
		vector<Contact>   contacts;

		Param();
	};

	struct Waypoint {
		struct End{
			vec3_t  pos;
			vec3_t  vel;
			bool    fix_pos;
			bool    fix_vel;

			End(vec3_t _pos, vec3_t _vel, bool _fix_pos, bool _fix_vel);
		};

		int     k;
		real_t  time;

		vec3_t  pos_t;
		quat_t  pos_r;
		vec3_t  vel_t;
		vec3_t  vel_r;
		bool    fix_pos_t;
		bool    fix_pos_r;
		bool    fix_vel_t;
		bool    fix_vel_r;

		vector<End>  ends;

		Waypoint();
		Waypoint(int _k,
			real_t _time, vec3_t _pos_t, quat_t _pos_r, vec3_t _vel_t, vec3_t _vel_r, 
			bool _fix_pos_t, bool _fix_pos_r, bool _fix_vel_t, bool _fix_vel_r);
	};

	struct Contact{
		V3Var*  var_pos;
	};

	struct Snapshot{
		struct End{
			vec3_t pos;
		};
		struct Contact{
			vec3_t pos;
			vec3_t force;
		};

		real_t       t;
		vec3_t       pos;
		vector<End>      ends;
		vector<Contact>  contacts;
		
		Snapshot();
	};

	Param	            param;
	vector<Waypoint>    waypoints;
	vector<Contact>     contacts;

	Snapshot            snapshot;
	vector<Snapshot>    trajectory;
	bool                trajReady;

	real_t              L;  //< length scaler
	real_t              T;  //< time scaler
	real_t              hnorm;

	virtual Keypoint*	CreateKeypoint() { return new CentroidKey(); }
	virtual void        AddVar ();
	virtual void        AddCon ();
	virtual void		Init   ();
	virtual void		Prepare();
	virtual void        Finish ();
	virtual void        CreateSnapshot(real_t time);
	virtual void        DrawSnapshot  (Render::Canvas* canvas, Render::Config* conf);
	virtual void        Draw          (Render::Canvas* canvas, Render::Config* conf);

	vec3_t ComPos      (real_t t, int type = Interpolate::Quadratic);
	vec3_t ComVel      (real_t t, int type = Interpolate::Quadratic);
	quat_t ComOri      (real_t t, int type = Interpolate::SlerpDiff);
	vec3_t ComAngVel   (real_t t, int type = Interpolate::Quadratic);
	vec3_t EndPos      (real_t t, int index, int type = Interpolate::Quadratic);
	vec3_t EndVel      (real_t t, int index, int type = Interpolate::Quadratic);
	vec3_t ContactForce(real_t t, int index, int type = Interpolate::LinearDiff);
	vec3_t ContactPos  (int index);
	
	void CreateSnapshot(real_t t, Snapshot& s);
	void CalcTrajectory();
	
public:
	         Centroid(Graph* g, string n);
	virtual ~Centroid();
};

struct CentroidCon : Constraint {
	CentroidKey*  obj[2];
	real_t        hnorm;
	
	void Prepare();

	CentroidCon(Solver* solver, int _dim, int _tag, string _name, CentroidKey* _obj, real_t _scale);
};

struct CentroidPosConT : CentroidCon{
	virtual void  CalcCoef();
	virtual void  CalcDeviation();
	virtual void  CalcLhs();
		
	CentroidPosConT(Solver* solver, int _tag, string _name, CentroidKey* _obj, real_t _scale);
};
struct CentroidPosConR : CentroidCon{
	virtual void  CalcCoef();
	virtual void  CalcDeviation();
	virtual void  CalcLhs();
		
	CentroidPosConR(Solver* solver, int _tag, string _name, CentroidKey* _obj, real_t _scale);
};
struct CentroidVelConT : CentroidCon{
	virtual void  CalcCoef();
	virtual void  CalcDeviation();
	virtual void  CalcLhs();
		
	CentroidVelConT(Solver* solver, int _tag, string _name, CentroidKey* _obj, real_t _scale);
};
struct CentroidVelConR : CentroidCon{
	virtual void  CalcCoef();
	virtual void  CalcDeviation();
	virtual void  CalcLhs();
		
	CentroidVelConR(Solver* solver, int _tag, string _name, CentroidKey* _obj, real_t _scale);
};

struct CentroidEndPosCon : CentroidCon{
	int          iend;
	
	virtual void  CalcCoef();
	virtual void  CalcDeviation();
	virtual void  CalcLhs();

	CentroidEndPosCon(Solver* solver, int _tag, string _name, CentroidKey* _obj, int _iend, real_t _scale);
};

struct CentroidEndRangeCon : Constraint{
	CentroidKey* obj;
	int          iend;
	vec3_t       dir;
	vec3_t       dir_abs;
	vec3_t       p;
	quat_t       q;
	vec3_t       pend;
	vec3_t       dp;
	real_t       _min, _max;
	bool	     on_lower, on_upper;

	void Prepare();

	virtual void  CalcCoef();
	virtual void  CalcDeviation();
	virtual void  Project(real_t& l, uint k);

	CentroidEndRangeCon(Solver* solver, int _tag, string _name, CentroidKey* _obj, int _iend, vec3_t _dir, real_t _scale);
};

struct CentroidEndCmplCon : Constraint{
	CentroidKey*    obj;
	int             iend;
	vector<real_t>  a;
	real_t          p;
	
	void Prepare();

	virtual void  CalcCoef();
	virtual void  CalcDeviation();
	
	CentroidEndCmplCon(Solver* solver, int _tag, string _name, CentroidKey* _obj, int _iend, real_t _scale);
};

struct CentroidContactGapCon : Constraint{
	CentroidKey*  obj;
	int           icon;
	int           iend;
	real_t        dmax;
	real_t        ac;
	vec3_t        pe;      //< end position
	vec3_t        pc;      //< contact point
	vec3_t        dp;      //< point difference
	vec3_t        dpn;     //< normalized dp
	real_t        dpnorm;  //< norm of dp
	
	void Prepare();

	virtual void  CalcCoef();
	virtual void  CalcDeviation();
	virtual void  Project(real_t& l, uint k);

	CentroidContactGapCon(Solver* solver, int _tag, string _name, CentroidKey* _obj, int _icon, real_t _scale);
};

struct CentroidContactForceCon : Constraint{
	CentroidKey*  obj;
	int           icon;
	int           dir;
	real_t        mu;
	real_t        fmax;
	real_t        a;
	vec3_t        f;
	bool          on_lower;
	bool          on_upper;
	
	void Prepare();

	virtual void  CalcCoef();
	virtual void  CalcDeviation();
	virtual void  Project(real_t& l, uint k);

	CentroidContactForceCon(Solver* solver, int _tag, string _name, CentroidKey* _obj, int _icon, int _dir, real_t _scale);
};

struct CentroidContactCmplCon : Constraint{
	CentroidKey*  obj;
	int           icon;
	real_t        a;
	real_t        ac;
	real_t        p;
	
	void Prepare();

	virtual void  CalcCoef();
	virtual void  CalcDeviation();
	
	CentroidContactCmplCon(Solver* solver, int _tag, string _name, CentroidKey* _obj, int _icon, real_t _scale);
};

}
