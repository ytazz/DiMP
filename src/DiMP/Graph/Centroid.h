#pragma once

#include <DiMP/Graph/Node.h>

namespace DiMP {;

class  Centroid;
struct CentroidPosConT;
struct CentroidPosConR;
struct CentroidVelConT;
struct CentroidVelConR;
struct CentroidTimeCon;
//struct CentroidEndEffortCon;
struct CentroidEndPosCon;
struct CentroidEndPosRangeCon;
struct CentroidEndVelRangeCon;
struct CentroidEndContactCon;
//struct CentroidEndCmplCon;
//struct CentroidEndStiffCon;
//struct CentroidEndMomentCon;
//struct CentroidEndForceRangeCon;

/**
	centroidal dynamics model
*/
class CentroidKey : public Keypoint {
public:
	Centroid*     cen;
	CentroidKey*  endNext;

	V3Var*  var_pos_t;    ///< position        
	QVar*   var_pos_r;    ///< orientation     
	V3Var*  var_vel_t;    ///< velocity        
	V3Var*  var_vel_r;    ///< angular velocity
	SVar*   var_time;
	SVar*   var_duration;  //< duration of this contact phase
	
	CentroidPosConT*    con_pos_t;
	CentroidPosConR*    con_pos_r;
	CentroidVelConT*    con_vel_t;
	CentroidVelConR*    con_vel_r;
	CentroidTimeCon*    con_time;
	RangeConS*          con_duration_range;
    FixConV3*           con_des_pos_t;
    FixConQ*            con_des_pos_r;
	//MatchConV3*         con_vel_match;
    FixConV3*           con_vel_zero;

	int  iend;   //< end-effector index

	int    ncon;
	real_t C;
	real_t S;
	real_t lbar;
	vec3_t pbar;
	real_t tau;
	vec3_t p, p_rhs;
	quat_t q, q_rhs;
	vec3_t v, v_rhs;
	vec3_t w, w_rhs;
	real_t k_p_p   ;
	real_t k_p_v   ;
	vec3_t k_p_tau ;
	real_t k_p_pbar;
	vec3_t k_p_lbar;
	real_t k_v_p   ;
	real_t k_v_v   ;
	vec3_t k_v_tau ;
	real_t k_v_pbar;
	vec3_t k_v_lbar;
	
	struct End{
		CentroidKey*  key;
		
		bool   contact;       //< true if this end effector is in contact
		V3Var* var_pos;       //< end effector position
		V3Var* var_vel;
		SVar*  var_stiff;     //< contact stiffness
		V3Var* var_moment;

		//MatchConV3*               con_pos_match;
		//MatchConS*                con_stiff_match;
		//MatchConV3*               con_moment_match;
		CentroidEndPosCon*              con_pos;
		CentroidEndPosRangeCon*         con_pos_range[3];
		CentroidEndVelRangeCon*         con_vel_range[3];
		RangeConS*                      con_stiff_range;
		RangeConV3*                     con_moment_range[3];
		vector<CentroidEndContactCon*>  con_contact;
        FixConS*                        con_stiff_zero;
        //CentroidEndEffortCon*     con_effort;
        //CentroidEndCmplCon*       con_cmpl;

		real_t k_pbar_pe;
		vec3_t k_pbar_le;
		real_t k_lbar_le;
		
		//vec3_t GetPos    ();
		//vec3_t GetForce  ();
		//void   SetPos  (const vec3_t& p);
	};

	vector<End>  ends;
	
public:
	void Swap(CentroidKey* key);

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
			vec3_t  basePos;
			vec3_t  posRangeMin;
			vec3_t  posRangeMax;
			vec3_t  velRangeMin;
			vec3_t  velRangeMax;
			vec3_t  momentRangeMin;
			vec3_t  momentRangeMax;
            vec2_t  copRangeMin;
            vec2_t  copRangeMax;
		};

		real_t	m;  //< mass
		real_t  I;  //< inertia
		real_t	g;  //< gravity

        vec3_t  bodyRangeMin;
        vec3_t  bodyRangeMax;
		
		vector<End>   ends;
		
		Param();
	};
	
    struct Edge;
    struct Vertex{
        vec3_t  p;
        Edge*   edge[2];

        bool IsOutside(const vec3_t& _p);
    };
    struct Edge{
        vec3_t  t;
        vec3_t  n;
        Vertex* vtx[2];

        bool IsOutside(const vec3_t& _p);
    };
	struct Face{
		vec2_t  rangeMin;
		vec2_t  rangeMax;
		vec3_t  pos;
        quat_t  ori;

        Vertex  vtx [4];
        Edge    edge[4];
        vec3_t  c;
        vec3_t  n;

        void Init();
        bool IsInside   (const vec3_t& p);
        //void CalcNearest(const vec3_t& p, vec3_t& pf, vec3_t& nf);

		Face(const vec2_t _rmin, const vec2_t _rmax, const vec3_t& _pos, const quat_t& _ori);
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
	
	struct Snapshot{
		struct End{
			vec3_t  pos;
			vec3_t  force;
            bool    contact;
		};
	
		real_t       t;
		vec3_t       pos;
		quat_t       ori;
		vec3_t       vel;
		vector<End>  ends;
		
		Snapshot();
	};

	Param	            param;
	vector<Face>        faces;
	vector<Waypoint>    waypoints;
	
	Snapshot            snapshot;
	vector<Snapshot>    trajectory;
	bool                trajReady;

	real_t              L;  //< length scaler
	real_t              T;  //< time scaler
	real_t              V;  //< velocity scaler
	real_t              F;  //< force scaler
	real_t              M;
	
	virtual Keypoint*	CreateKeypoint() { return new CentroidKey(); }
	virtual void		Init   ();
	virtual void		Prepare();
	virtual void        Finish ();
	virtual void        CreateSnapshot(real_t time);
	virtual void        DrawSnapshot  (Render::Canvas* canvas, Render::Config* conf);
	virtual void        Draw          (Render::Canvas* canvas, Render::Config* conf);

	//Face*  FindFace  (const vec3_t& p, vec3_t& pf, vec3_t& nf);
	vec3_t ComPos    (real_t t, int type = Interpolate::Cubic);
	vec3_t ComVel    (real_t t, int type = Interpolate::Cubic);
	quat_t ComOri    (real_t t, int type = Interpolate::SlerpDiff);
	vec3_t ComAngVel (real_t t, int type = Interpolate::Cubic);
	vec3_t EndPos    (real_t t, int index, int type = Interpolate::LinearDiff);
	bool   EndContact(real_t t, int index);
	//vec3_t EndForce (real_t t, int index, int type = Interpolate::LinearDiff);
	
	void CreateSnapshot(real_t t, Snapshot& s);
	void CalcTrajectory();
	
public:
	         Centroid(Graph* g, string n);
	virtual ~Centroid();
};

struct CentroidCon : Constraint {
	CentroidKey*  obj[2];

	CentroidCon(Solver* solver, int _dim, int _tag, string _name, CentroidKey* _obj, real_t _scale);
};

struct CentroidPosConT : CentroidCon{
	virtual void  CalcCoef();
	virtual void  CalcDeviation();
	virtual void  CalcLhs();
		
	CentroidPosConT(Solver* solver, string _name, CentroidKey* _obj, real_t _scale);
};

struct CentroidPosConR : CentroidCon{
	virtual void  CalcCoef();
	virtual void  CalcDeviation();
	virtual void  CalcLhs();
		
	CentroidPosConR(Solver* solver, string _name, CentroidKey* _obj, real_t _scale);
};

struct CentroidVelConT : CentroidCon{
	virtual void  CalcCoef();
	virtual void  CalcDeviation();
	virtual void  CalcLhs();
		
	CentroidVelConT(Solver* solver, string _name, CentroidKey* _obj, real_t _scale);
};

struct CentroidVelConR : CentroidCon{
	virtual void  CalcCoef();
	virtual void  CalcDeviation();
	virtual void  CalcLhs();
		
	CentroidVelConR(Solver* solver, string _name, CentroidKey* _obj, real_t _scale);
};

struct CentroidTimeCon : CentroidCon{
	virtual void  CalcCoef();
	virtual void  CalcDeviation();
	virtual void  CalcLhs();
		
	CentroidTimeCon(Solver* solver, string _name, CentroidKey* _obj, real_t _scale);
};

struct CentroidEndPosCon : CentroidCon{
	int iend;

	virtual void  CalcCoef();
	virtual void  CalcDeviation();
	virtual void  CalcLhs();
		
	CentroidEndPosCon(Solver* solver, string _name, CentroidKey* _obj, int _iend, real_t _scale);
};

struct CentroidEndPosRangeCon : Constraint{
	CentroidKey* obj;
	int          iend;
	vec3_t       dir;
	vec3_t       dir_abs;
	vec3_t       p;
	quat_t       q;
	vec3_t       pbase;
	vec3_t       pend;
	real_t       _min, _max;
	bool	     on_lower, on_upper;

	void Prepare();

	virtual void  CalcCoef();
	virtual void  CalcDeviation();
	virtual void  Project(real_t& l, uint k);

	CentroidEndPosRangeCon(Solver* solver, string _name, CentroidKey* _obj, int _iend, vec3_t _dir, real_t _scale);
};

struct CentroidEndVelRangeCon : Constraint{
	CentroidKey* obj;
	int          iend;
	vec3_t       dir;
	vec3_t       dir_abs;
	quat_t       q;
	vec3_t       vend;
	real_t       _min, _max;
	bool	     on_lower, on_upper;

	void Prepare();

	virtual void  CalcCoef();
	virtual void  CalcDeviation();
	virtual void  Project(real_t& l, uint k);

	CentroidEndVelRangeCon(Solver* solver, string _name, CentroidKey* _obj, int _iend, vec3_t _dir, real_t _scale);
};

struct CentroidEndContactCon : Constraint{
	CentroidKey*     obj;
	int              iend;
    int              iface;
	//Centroid::Face*  face;
    vec3_t           pe;
    vec3_t           pf;
    vec3_t           nf;
    bool             inside;
	real_t           _min, _max;
	bool	         on_lower, on_upper;

	void Prepare();

	virtual void  CalcCoef();
	virtual void  CalcDeviation();
	virtual void  Project(real_t& l, uint k);

	CentroidEndContactCon(Solver* solver, string _name, CentroidKey* _obj, int _iend, int _iface, real_t _scale);
};

//struct CentroidEndCmplCon : Constraint{
//	CentroidKey*     obj;
//	int              iend;
//    real_t           le;
//    vec3_t           ve, ven;
//	real_t           ve_norm;
//
//	void Prepare();
//
//	virtual void  CalcCoef();
//	virtual void  CalcDeviation();
//	
//	CentroidEndCmplCon(Solver* solver, string _name, CentroidKey* _obj, int _iend, real_t _scale);
//};

//struct CentroidEndEffortCon : Constraint{
//	CentroidKey*     obj;
//	int              iend;
//    real_t           le, le2;
//
//    void Prepare();
//
//    virtual void  CalcCoef();
//	virtual void  CalcDeviation();
//		
//	CentroidEndEffortCon(Solver* solver, string _name, CentroidKey* _obj, int _iend, real_t _scale);
//};

}
