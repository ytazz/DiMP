﻿#pragma once

#include <DiMP/Graph/Node.h>

namespace DiMP {;

struct CentroidPosConT;
struct CentroidPosConR;
struct CentroidVelConT;
struct CentroidVelConR;
struct CentroidEndRangeCon;
struct CentroidEndVelCon;
struct CentroidPosCmplCon;
struct CentroidVelCmplCon;
struct CentroidFrictionCon;

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

	struct Face{
		V3Var*                 var_force;
		SVar*                  var_pos_cmpl;
		SVar*                  var_vel_cmpl;

		CentroidFrictionCon*   con_fric;
		CentroidPosCmplCon*    con_pos_cmpl[2];
		CentroidVelCmplCon*    con_vel_cmpl[2];
		
		int     iedge; //< index to nearest edge
		int     ivtx;  //< index to nearest vertex
		vec3_t  pc;    //< nearest point to end
		mat3_t  R;
		real_t  mu;
	};
	struct End{
		V3Var*                 var_pos;
		V3Var*                 var_vel;
		
		CentroidEndRangeCon*   con_range[3];
		CentroidEndVelCon*     con_vel;

		vector<Face>  faces;
	};
	
	vector<End>      ends;

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
		struct Edge{
			vec3_t  n;
			vec3_t  v[2];
		};
		struct Face{
			vec3_t  origin;
			mat3_t  R;
			real_t  mu;
		
			vector<vec3_t>  vertices;
			vector<Edge>    edges;

			void CalcNearest(const vec3_t& p, vec3_t& pc, int& iedge, int& ivtx);
		};

		real_t	m;  //< mass
		real_t  I;  //< inertia
		vec3_t	g;  //< gravity
		real_t  h;  //< time resolution
		real_t  l;  //< nominal length
		
		vector<End>   ends;
		vector<Face>  faces;

		Param();
	};

	struct Waypoint {
		int     k;
		real_t  time;

		vec3_t  pos_t;
		vec3_t  vel_t;
		quat_t  pos_r;
		vec3_t  vel_r;
			
		bool    fix_time;
		bool    fix_pos_t;
		bool    fix_pos_r;
		bool    fix_vel_t;
		bool    fix_vel_r;
			
		Waypoint();
	};

	struct Snapshot{
		struct End{
			vec3_t pos;
			vec3_t force;
		};

		real_t       t;
		vec3_t       pos;
		vector<End>  ends;
		
		Snapshot();
	};

	Param	            param;
	vector<Waypoint>    waypoints;
	Snapshot            snapshot;
	vector<Snapshot>    trajectory;
	bool                trajReady;

	real_t              Inorm;  //< normalized inertia
	vec3_t              gnorm;  //< normalized gravity

	virtual Keypoint*	CreateKeypoint() { return new CentroidKey(); }
	virtual void		Init();
	virtual void		Prepare();
	virtual void        Finish ();
	virtual void        CreateSnapshot(real_t time);
	virtual void        DrawSnapshot  (Render::Canvas* canvas, Render::Config* conf);
	virtual void        Draw          (Render::Canvas* canvas, Render::Config* conf);

	vec3_t ComPos   (real_t t, int type = Interpolate::Quadratic);
	vec3_t ComVel   (real_t t, int type = Interpolate::Quadratic);
	quat_t ComOri   (real_t t, int type = Interpolate::SlerpDiff);
	vec3_t ComAngVel(real_t t, int type = Interpolate::Quadratic);
	vec3_t EndPos   (real_t t, int index, int type = Interpolate::Quadratic);
	vec3_t EndVel   (real_t t, int index, int type = Interpolate::Quadratic);
	vec3_t EndForce (real_t t, int index, int type = Interpolate::LinearDiff);

	void CreateSnapshot(real_t t, Snapshot& s);
	void CalcTrajectory();
	
public:
	         Centroid(Graph* g, string n);
	virtual ~Centroid();
};

struct CentroidCon : Constraint {
	CentroidKey* obj[2];
	
	void Prepare();

	CentroidCon(Solver* solver, int _tag, string _name, CentroidKey* _obj, real_t _scale);
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

struct CentroidEndVelCon : Constraint{
	CentroidKey* obj[2];
	int          iend;
	
	void Prepare();

	virtual void  CalcCoef();
	virtual void  CalcDeviation();

	CentroidEndVelCon(Solver* solver, int _tag, string _name, CentroidKey* _obj, int _iend, real_t _scale);
};

struct CentroidPosCmplCon : Constraint{
	CentroidKey* obj;
	bool         side;
	int          iend;
	int          iface;
	real_t       gamma;
	vec3_t       p;
	vec3_t       pc;
	vec3_t       dp;
	vec3_t       dpn;
	
	void Prepare();
	
	virtual void  CalcCoef();
	virtual void  CalcDeviation();

	CentroidPosCmplCon(Solver* solver, int _tag, string _name, CentroidKey* _obj, int _iend, int _iface, bool _side, real_t _scale);
};

struct CentroidVelCmplCon : Constraint{
	CentroidKey* obj;
	bool         side;
	int          iend;
	int          iface;
	real_t       gamma;
	vec3_t       p;
	vec3_t       pc;
	vec3_t       dp;
	vec3_t       dpn;
	
	void Prepare();
	
	virtual void  CalcCoef();
	virtual void  CalcDeviation();

	CentroidVelCmplCon(Solver* solver, int _tag, string _name, CentroidKey* _obj, int _iend, int _iface, bool _side, real_t _scale);
};

struct CentroidFrictionCon : Constraint{
	CentroidKey* obj;
	int          iend;
	int          iface;
	real_t       mu;
	vec3_t       f;
	real_t       ftn;

	void Prepare();

	virtual void  CalcCoef();
	virtual void  CalcDeviation();
	virtual void  Project(real_t& l, uint k);

	CentroidFrictionCon(Solver* solver, int _tag, string _name, CentroidKey* _obj, int _iend, int _nface, real_t _scale);
};

}
