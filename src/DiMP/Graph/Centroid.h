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
struct CentroidAccConT;
struct CentroidTimeCon;
struct CentroidJointPosCon;
struct CentroidEndPosCon;
struct CentroidJointPosRangeCon;
//struct CentroidJointVelRangeCon;
//struct CentroidEndVelZeroCon;
struct CentroidEndKinematicsCon;
struct CentroidEndContactCon;

class CentroidDDPState;

class CentroidCallback{
public:
    //virtual void EnableContact (CentroidEndContactCon* con) = 0;
	virtual bool IsValidState     (CentroidDDPState* st){ return true; }
	virtual bool IsValidTransition(CentroidDDPState* st0, CentroidDDPState* st1){ return true; }
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
	CentroidAccConT*    con_acc_t;
	CentroidTimeCon*    con_time;
	RangeConS*          con_duration_range;
    FixConV3*           con_des_pos_t;
    FixConQ*            con_des_pos_r;
    FixConV3*           con_des_vel_t;
	FixConS*            con_des_time;

	//Solver::SubState*   subst_pos_t;
	//Solver::SubState*   subst_pos_r;
	//Solver::SubState*   subst_vel_t;
	//Solver::SubState*   subst_vel_r;

	real_t C;
	real_t S;
	real_t lbar;
	vec3_t pbar;
    vec3_t pave;
	real_t tau;
	vec3_t p, p_rhs;
	quat_t q, q_rhs;
	vec3_t v, v_rhs;
	vec3_t    a_rhs;
	vec3_t w, w_rhs;
	mat3_t R;
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
	//real_t k_a_p   ;
	//real_t k_a_pbar;
	//vec3_t k_a_lbar;
	//vec3_t k_pbar_q;
	
	struct End{
		CentroidKey*  key;
		
		int    iface;         //< -1: no contact, otherwise index to contact face
		V3Var* var_pos;       //< end effector position (in global coordinate)
		V3Var* var_vel;       //< end effector velocity (in global coordinate)
		SVar*  var_joint_pos[3]; //< end effector position (in local coordinate)
		SVar*  var_joint_vel[3]; //< end effector velocity (in local coordinate)
		//QVar*  var_ori;
		//V3Var* var_angvel;
		//V3Var* var_cop;
		SVar*  var_stiff;     //< contact stiffness
		V3Var* var_moment;

		CentroidEndPosCon*              con_pos;
		CentroidJointPosCon*            con_joint[3];
		//vector<RangeConS*>              con_joint_pos_range;
		CentroidJointPosRangeCon*       con_joint_pos_range[3][2];
		//CentroidEndVelRangeCon*         con_vel_range[3][2];
		//CentroidEndCopRangeCon*         con_cop_range[3][2];
		RangeConS*                      con_stiff_range;
		RangeConV3*                     con_moment_range[3];
        FixConS*                        con_stiff_zero;
        FixConV3*                       con_moment_zero;
		FixConV3*                       con_des_pos;
		FixConV3*                       con_des_vel;
		FixConV3*                       con_vel_zero;
		FixConS*                        con_joint_vel_zero[3];
		CentroidEndKinematicsCon*       con_kin;
		//CentroidEndVelZeroCon*          con_vel_zero;
        vector<CentroidEndContactCon*>  con_contact;

		Solver::SubInput*  subin_stiff;
		Solver::SubInput*  subin_moment;
		Solver::SubInput*  subin_vel;
		Solver::SubState*  subst_pos;

        real_t k_pbar_pe;
		vec3_t k_pbar_le;
		real_t k_lbar_le;
	};

	vector<End>    ends;
	
public:
    virtual void AddVar(Solver* solver);
	virtual void AddCon(Solver* solver);
	virtual void Prepare();
	virtual void Finish ();
	virtual void Draw(Render::Canvas* canvas, Render::Config* conf);

	CentroidKey();
};

class CentroidDDPState : public DDPState{
public:
	Centroid*    cen;
	vector<int>  contact;  //< contact state of each end effector
	int          end_sw;   //< end effector which is changing contact state. -1 if none
	int          face_sw;  //< face which the end effector indicated by end_sw is making contact to. -1 if lifting off
	
	virtual bool IsIdentical(const DDPState* st);
	virtual bool IsTerminal ();
	virtual void CalcCost   ();
	virtual void Finish     ();
	virtual void Print      ();

             CentroidDDPState(Centroid* _cen, CustomSolver* _solver);
    virtual ~CentroidDDPState();
};

class Centroid : public TrajectoryNode, public DDPCallback{
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

		real_t  durationMin;
		real_t  durationMax;
        
        real_t  swingSlope;
        real_t  swingHeight;
		
		Param();
	};
	
   	struct End{
		vec3_t  basePos;
		vec3_t  jointPosMin;
		vec3_t  jointPosMax;
		vec3_t  jointVelMin;
		vec3_t  jointVelMax;
		vec3_t  momentRangeMin;
		vec3_t  momentRangeMax;
        vec2_t  copRangeMin;
        vec2_t  copRangeMax;
        real_t  stiffnessMax;
        int     contactInitial;
        int     contactTerminal;
		
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
        int    numSwitchMax;

        Face();
    };

    Param	            param;
    vector<End>         ends;
    vector<Face>        faces;
	vector<Waypoint>    waypoints;
    CentroidCallback*   callback;
	vector< vector<int> >  validContacts;

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
	virtual DDPState*  CreateInitialState();
	virtual void       CreateNextStates  (DDPState* _state, vector<DDPState*>& _next);
	virtual void       OnThreadUpdate    (DDPThread* _thread);
	
    //Face*  FindFace  (const vec3_t& p, vec3_t& pf, vec3_t& nf);
	void ComState       (real_t t, vec3_t& pos, vec3_t& vel, vec3_t& acom);
	void TorsoState     (real_t t, quat_t& ori, vec3_t& angvel, int type = Interpolate::SlerpDiff);
	void EndState       (real_t t, int index, vec3_t& pos, vec3_t& vel);
    void EndForce       (real_t t, int index, real_t& stiff, vec3_t& moment, bool& contact);
	void EndSwitchTiming(real_t t, int index, real_t& tprev, real_t& tnext);
    
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

struct CentroidAccConT : CentroidCon{
	virtual void  CalcCoef();
	virtual void  CalcDeviation();
		
	CentroidAccConT(Solver* solver, string _name, CentroidKey* _obj, real_t _scale);
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

struct CentroidJointPosCon : CentroidCon{
	int    iend;
	int    idx;
	real_t q0, q1;
	real_t qd0;
	real_t tau;
	//real_t qmin;
	//real_t qmax;
	//real_t sig_min;
	//real_t sig_max;
	//real_t sig;
	//bool   on_lower;
	//bool   on_upper;

	void Prepare();

	virtual void  CalcCoef();
	virtual void  CalcDeviation();
	virtual void  CalcLhs();
		
	CentroidJointPosCon(Solver* solver, string _name, CentroidKey* _obj, int _iend, int _idx, real_t _scale);
};

struct CentroidEndPosCon : CentroidCon{
	int    iend;
	vec3_t pe0, pe1;
	vec3_t ve0;
	real_t tau;

	void Prepare();

	virtual void  CalcCoef();
	virtual void  CalcDeviation();
	virtual void  CalcLhs();
		
	CentroidEndPosCon(Solver* solver, string _name, CentroidKey* _obj, int _iend, real_t _scale);
};

struct CentroidJointPosRangeCon : Constraint{
	CentroidKey* obj;
	int          iend;
	int          idx;
	real_t       dir;
	real_t       q;
	real_t       bound;
	
	void Prepare();

	virtual void  CalcCoef();
	virtual void  CalcDeviation();

	CentroidJointPosRangeCon(Solver* solver, string _name, CentroidKey* _obj, int _iend, int _idx, real_t _dir, real_t _scale);
};

/*
struct CentroidEndVelRangeCon : Constraint{
	CentroidKey* obj;
	int          iend;
	vec3_t       dir;
	vec3_t       dir_abs;
	quat_t       q;
	vec3_t       v;
	vec3_t       vend;
    real_t       bound;
	
	void Prepare();

	virtual void  CalcCoef();
	virtual void  CalcDeviation();

	CentroidEndVelRangeCon(Solver* solver, string _name, CentroidKey* _obj, int _iend, vec3_t _dir, real_t _scale);
};
*/

struct CentroidEndKinematicsCon : Constraint{
	CentroidKey*    obj;
	int             iend;
	vec3_t          p;
	quat_t          q;
	vector<real_t>  qj;
	vector<vec3_t>  dir, dir_abs;
	vec3_t          pe_local, pe;
	vec3_t          pe_base;
	
	void Prepare();

	virtual void  CalcCoef();
	virtual void  CalcDeviation();
	
	CentroidEndKinematicsCon(Solver* solver, string _name, CentroidKey* _obj, int _iend, real_t _scale);
};

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
