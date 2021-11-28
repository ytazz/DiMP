#pragma once

#include <DiMP/Graph/Node.h>
#include <DiMP/Graph/Geometry.h>
#include <DiMP/Graph/Solver.h>

namespace DiMP {;

class  Centroid;
struct CentroidPosConT;
struct CentroidPosConR;
struct CentroidVelConT;
struct CentroidVelConR;
struct CentroidTimeCon;
struct CentroidEndPosCon;
struct CentroidEndPosRangeCon;
struct CentroidEndVelRangeCon;
struct CentroidEndContactCon;

class CentroidCallback{
public:
    virtual void EnableContact(CentroidEndContactCon* con) = 0;
};

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
    FixConV3*           con_vel_zero;
    vector<Solver::SubCost*>  subcost_u;
    vector<Solver::SubCost*>  subcost_x;

	real_t C;
	real_t S;
	real_t lbar;
	vec3_t pbar;
    vec3_t pave;
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
		
		int    iface;         //< -1: no contact, otherwise index to contact face
		V3Var* var_pos;       //< end effector position
		V3Var* var_vel;
		SVar*  var_stiff;     //< contact stiffness
		V3Var* var_moment;

		CentroidEndPosCon*              con_pos;
		CentroidEndPosRangeCon*         con_pos_range[3];
		CentroidEndVelRangeCon*         con_vel_range[3];
		RangeConS*                      con_stiff_range;
		RangeConV3*                     con_moment_range[3];
        FixConS*                        con_stiff_zero;
        FixConV3*                       con_moment_zero;
		FixConV3*                       con_des_pos;
		FixConV3*                       con_vel_zero;
        vector<CentroidEndContactCon*>  con_contact;

        vector<Solver::SubCost*>  subcost_c;     ///< costs active when in contact
        vector<Solver::SubCost*>  subcost_nc;    ///< costs active when not in contact
        vector<Solver::SubCost*>  subcost_face;  ///< cost active when in contact with each face

		real_t k_pbar_pe;
		vec3_t k_pbar_le;
		real_t k_lbar_le;
	};

	vector<End>  ends;
	
public:
    virtual void AddVar(Solver* solver);
	virtual void AddCon(Solver* solver);
	virtual void Prepare();
	virtual void Draw(Render::Canvas* canvas, Render::Config* conf);

	CentroidKey();
};

class CentroidDDPNode : public CustomSolver::DDPNode{
public:
    vector<int>  contact;  //< for each iend, -1: not in contact, otherwise index of face in contact with
    vector<int>  count;    //< for each iend, number of contact switches so far

public:
    
             CentroidDDPNode();
    virtual ~CentroidDDPNode();
};

class Centroid : public TrajectoryNode, public CustomSolver::DDPCallback{
public:
	struct Param {
		real_t	m;  //< mass
		real_t  I;  //< inertia
		real_t	g;  //< gravity

        vec3_t  bodyRangeMin;
        vec3_t  bodyRangeMax;

        real_t  complWeightMin;
        real_t  complWeightMax;
        real_t  complWeightRate;
        //real_t  complWeightDecay;

        real_t  swingSlope;
        real_t  swingHeight;
		
		Param();
	};
	
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
        real_t  stiffnessMax;
        int     numSwitchMax;

        Point*  point;

        End();
	};

	struct Waypoint {
		struct End{
			vec3_t  pos;
			vec3_t  vel;
			
            End();
			End(vec3_t _pos, vec3_t _vel);
		};

		int     k;
		real_t  time;

		vec3_t  pos_t;
		quat_t  pos_r;
		vec3_t  vel_t;
		vec3_t  vel_r;
		
		vector<End>  ends;

		Waypoint();
		Waypoint(int _k,
			real_t _time, vec3_t _pos_t, quat_t _pos_r, vec3_t _vel_t, vec3_t _vel_r);
	};
	
	struct Snapshot{
		struct End{
			vec3_t  pos;
			vec3_t  vel;
            real_t  stiffness;
            vec3_t  force;
            vec3_t  moment;
            bool    contact;
		};
	
		real_t       t;
		vec3_t       pos;
		vec3_t       vel;
        vec3_t       acc;
        quat_t       ori;
        vec3_t       angvel;
		
		vector<End>  ends;
		
		Snapshot();
	};

    struct Face{
        vec3_t normal;
        Hull*  hull;
    };

    Param	            param;
    vector<End>         ends;
    vector<Face>        faces;
	vector<Waypoint>    waypoints;
    CentroidCallback*   callback;
	
	Snapshot            snapshot;
	vector<Snapshot>    trajectory;
	bool                trajReady;

	real_t              L;  //< length scaler
	real_t              T;  //< time scaler
	real_t              V;  //< velocity scaler
    real_t              A;  //< acceleration scaler
	real_t              F;  //< force scaler
	real_t              M;  //< moment scaler
    real_t              S;  //< stiffness scaler

    Point*              point;  //< geometries used for internal computation
    Hull*               hull;

    real_t              complWeight;
	
	virtual Keypoint*	CreateKeypoint() { return new CentroidKey(); }
	virtual void		Init   ();
	virtual void		Prepare();
	virtual void        Finish ();
	virtual void        CreateSnapshot(real_t time);
	virtual void        DrawSnapshot  (Render::Canvas* canvas, Render::Config* conf);
	virtual void        Draw          (Render::Canvas* canvas, Render::Config* conf);

    // DDPCallback
    virtual int                     NumBranches (CustomSolver::DDPNode* _parent);
    virtual CustomSolver::DDPNode*  CreateNode  (CustomSolver::DDPNode* _parent, CustomSolver* _solver, int idx, int nx, int nu);
    virtual void                    FinishNode  (CustomSolver::DDPNode* _node);
    virtual real_t                  CalcNodeCost(CustomSolver::DDPNode* _node);

    //Face*  FindFace  (const vec3_t& p, vec3_t& pf, vec3_t& nf);
	void ComState   (real_t t, vec3_t& pos, vec3_t& vel, vec3_t& acom);
	void TorsoState (real_t t, quat_t& ori, vec3_t& angvel, int type = Interpolate::SlerpDiff);
	void EndState   (real_t t, int index, vec3_t& pos, vec3_t& vel);
    void EndForce   (real_t t, int index, real_t& stiff, vec3_t& moment, bool& contact);
    
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
	vec3_t       v;
	vec3_t       vend;
	real_t       _min, _max;
	bool	     on_lower, on_upper;

	void Prepare();

	virtual void  CalcCoef();
	virtual void  CalcDeviation();
	virtual void  Project(real_t& l, uint k);

	CentroidEndVelRangeCon(Solver* solver, string _name, CentroidKey* _obj, int _iend, vec3_t _dir, real_t _scale);
};
/*
struct CentroidEndVelZeroCon : Constraint{
	CentroidKey* obj;
	int          iend;
	vec3_t       dir;
	vec3_t       dir_abs;
	quat_t       q;
    vec3_t       v;
	vec3_t       vend;
	
	void Prepare();

	virtual void  CalcCoef();
	virtual void  CalcDeviation();
	
	CentroidEndVelZeroCon(Solver* solver, string _name, CentroidKey* _obj, int _iend, vec3_t _dir, real_t _scale);
};
*/
struct CentroidEndContactCon : Constraint{
	CentroidKey*     obj;
	int              iend;
    int              iface;
    Point*           point;
    Centroid::Face*  face;
	vec3_t           pe;
    vec3_t           pf;
    vec3_t           nf;
    
	void Prepare();

	virtual void  CalcCoef();
	virtual void  CalcDeviation();

	CentroidEndContactCon(Solver* solver, string _name, CentroidKey* _obj, int _iend, int _iface, real_t _scale);
};

}
