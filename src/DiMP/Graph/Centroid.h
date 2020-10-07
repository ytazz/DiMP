#pragma once

#include <DiMP/Graph/Node.h>

namespace DiMP {;

struct CentroidPosConT;
struct CentroidPosConR;
struct CentroidVelConT;
struct CentroidVelConR;
struct CentroidEndRangeCon;
struct CentroidEndVelCon;
struct CentroidEndForceCon;

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
		V3Var*                 var_pos_t;
		V3Var*                 var_force_t;
		
		CentroidEndRangeCon*   con_range[3];
		CentroidEndVelCon*     con_vel;
		CentroidEndForceCon*   con_force;

		vec3_t  pc;
		mat3_t  Rc;
	};

	vector<End>  ends;

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
			vec3_t  normal;
			real_t  offset;
		};
		struct Face{
			vec3_t  origin;
			mat3_t  R;

			vector<vec3_t>  vertices;
			vector<Edge>    edges;
		};

		real_t	mass;
		vec3_t	gravity;
		real_t  mu;
		real_t  gamma;

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

	struct TrajPoint {
		real_t  t;
		vec3_t  pos;
		vector<vec3_t>  end_pos;
		
		TrajPoint();
	};

	Param	            param;
	vector<Waypoint>    waypoints;
	vector<TrajPoint>   trajectory;
	bool                trajReady;

	real_t              Inorm;  //< normalized inertia
	vec3_t              gnorm;  //< normalized gravity

	virtual Keypoint*	CreateKeypoint() { return new CentroidKey(); }
	virtual void		Init();
	virtual void		Prepare();

	void CalcNearest(const vec3_t& p, vec3_t& _pc, mat3_t& _Rc);

	vec3_t ComPos   (real_t t, int type = Interpolate::Quadratic);
	vec3_t ComVel   (real_t t, int type = Interpolate::Quadratic);
	quat_t ComOri   (real_t t, int type = Interpolate::SlerpDiff);
	vec3_t ComAngVel(real_t t, int type = Interpolate::Quadratic);
	vec3_t EndPos   (real_t t, int index, int type = Interpolate::LinearDiff);
	vec3_t EndVel   (real_t t, int index, int type = Interpolate::LinearDiff);

	void CalcTrajectory();
	void Draw(Render::Canvas* canvas, Render::Config* conf);
	void DrawSnapshot(real_t time, Render::Canvas* canvas, Render::Config* conf);
	void Save();
	void Print();

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
	int          dir;

	virtual void  CalcCoef();
	virtual void  CalcDeviation();

	CentroidEndRangeCon(Solver* solver, int _tag, string _name, CentroidKey* _obj, int _iend, int _dir, real_t _scale);
};

struct CentroidEndVelCon : Constraint{
	CentroidKey* obj[2];
	int          iend;

	virtual void  CalcCoef();
	virtual void  CalcDeviation();

	CentroidEndVelCon(Solver* solver, int _tag, string _name, CentroidKey* _obj, int _iend, real_t _scale);
};

struct CentroidEndForceCon : Constraint{
	CentroidKey* obj;
	int          iend;

	virtual void  CalcCoef();
	virtual void  CalcDeviation();

	CentroidEndForceCon(Solver* solver, int _tag, string _name, CentroidKey* _obj, int _iend, real_t _scale);
};

}
