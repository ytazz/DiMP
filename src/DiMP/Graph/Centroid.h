#pragma once

#include <DiMP/Graph/Node.h>
#include <DiMP/Graph/Geometry.h>
#include <DiMP/Graph/Solver.h>

namespace DiMP {;

class  Centroid;
struct CentroidPosConT;
struct CentroidPosConR;
struct CentroidPosConRPY;
struct CentroidVelConT;
struct CentroidVelConR;
struct CentroidTimeCon;
struct CentroidEndPosConT;
struct CentroidEndPosConR;
struct CentroidDurationRangeCon;
struct CentroidEndPosRangeCon;
struct CentroidEndContactCon;
struct CentroidEndFrictionCon;
struct CentroidEndMomentCon;

/**
	centroidal dynamics model
*/
class CentroidKey : public Keypoint {
public:
	Centroid*     cen;
	CentroidKey*  endNext;

	V3Var*  var_pos_t;    ///< position        
	QVar*   var_pos_r;    ///< orientation
	V3Var*  var_pos_rpy;
	V3Var*  var_vel_t;    ///< velocity        
	V3Var*  var_vel_r;    ///< angular velocity
	SVar*   var_time;
	SVar*   var_duration;  //< duration of this contact phase
	
	CentroidPosConT*           con_pos_t;
	CentroidPosConR*           con_pos_r;
	CentroidPosConRPY*         con_pos_rpy;
	CentroidVelConT*           con_vel_t;
	CentroidVelConR*           con_vel_r;
	CentroidTimeCon*           con_time ;
	CentroidDurationRangeCon*  con_duration_range[2];
    FixConV3*                  con_des_pos_t;
    FixConQ*                   con_des_pos_r;
    FixConV3*                   con_des_pos_rpy;
    FixConV3*                  con_des_vel_t;
	FixConV3*                  con_des_vel_r;
	FixConS*                   con_des_time;
	FixConS*                   con_des_duration;

	struct End{
		CentroidKey*  key;
		
		int    iface;         //< -1: no contact, otherwise index to contact face
		V3Var* var_pos_t;     //< end effector position (in global coordinate)
		QVar*  var_pos_r;
		V3Var* var_vel_t;     //< end effector velocity (in global coordinate)
		V3Var* var_vel_r;
		SVar*  var_stiff;     //< contact stiffness
		SVar*  var_cmp[2];
		V3Var* var_moment;

		CentroidEndPosConT*             con_pos_t;
		CentroidEndPosConR*             con_pos_r;
		CentroidEndPosRangeCon*         con_pos_range[3][2];
		RangeConS*                      con_stiff_range;
		FixConV3*                       con_des_pos_t;
		FixConQ *                       con_des_pos_r;
		FixConV3*                       con_des_vel_t;
		FixConV3*                       con_des_vel_r;
		FixConV3*                       con_des_acc_t;
		FixConV3*                       con_des_acc_r;
		FixConS*                        con_des_stiff;
		FixConS*                        con_des_cmp[2];
		FixConV3*                       con_des_moment;

		CentroidEndFrictionCon*         con_friction;
		CentroidEndMomentCon*           con_moment[3][2];
		
		vector<CentroidEndContactCon*>  con_contact;
	};

	vec3_t msum;
	real_t lbar, l2sum;
	vec3_t pbar, rbar, etabar;
	mat3_t pbar_cross, rbar_cross;
	vector<real_t> li, li2;
	vector<vec3_t> pi, ri, etai;
	vector<mat3_t> pi_cross, ri_cross;

	real_t  tau, dtau;
	vector<real_t> t, C, S;
	vector<real_t> C_tau, S_tau, C_lbar, S_lbar;
	mat3_t  Iinv_m;
	vec3_t  g, psum, rsum;

	struct Coef{
		vec3_t p, p_lhs, p_rhs;
		vec3_t v, v_lhs;
		quat_t q, q_lhs;
		vec3_t rpy, rpy_lhs, rpy_rhs;
		vec3_t w, w_lhs;

		vector<vec3_t> v_rhs, w_rhs;
		vector<quat_t> q_rhs;

		vector<mat3_t> R_omega;
		
		vector<real_t> lbar_li;
		vec3_t         pbar_lbar;
		vector<vec3_t> pbar_li;
		vector<real_t> pbar_pi, pbar_vi;
		vec3_t         rbar_lbar;
		vector<vec3_t> rbar_li;
		vector<real_t> rbar_ri;		
		vec3_t         etabar_lbar;
		mat3_t         etabar_pbar, etabar_rbar;
		vector<vec3_t> etabar_li;
		vector<mat3_t> etabar_pi, etabar_vi, etabar_ri;
		vector<real_t> etabar_etai;
		
		vec3_t p_C, p_S;
		real_t p_p, p_v, p_pbar, p_rbar;
		vec3_t p_lbar, p_tau;
		vector<vec3_t> p_li;              //< nend
		vector<real_t> p_pi, p_vi, p_ri;  //< nend

		vec3_t v_C, v_S;
		vector<real_t> v_p, v_v, v_pbar, v_rbar;     //< ndiv array
		vector<vec3_t> v_lbar, v_tau;                //< ndiv array
		vector< vector<vec3_t> >  v_li;              //< nend x ndiv array
		vector< vector<real_t> >  v_pi, v_vi, v_ri;  //< nend x ndiv array

		real_t w_w;
		mat3_t w_v1;
		vector<mat3_t> w_p, w_v;                             //< ndiv
		vector<vec3_t> w_tau;                                //< ndiv
		vector<mat3_t> w_rbar, w_etabar;                     //< ndiv
		vector< vector<mat3_t> >  w_pi, w_vi, w_ri, w_etai;  //< nend x ndiv
		vector< vector<vec3_t> >  w_li;                      //< nend x ndiv

		mat3_t q_q, q_w;
		mat3_t q_p, q_v;
		vec3_t q_tau;
		vector<mat3_t> q_w1;                       //< ndiv
		vector<mat3_t> q_pi, q_vi, q_ri, q_etai;   //< nend
		vector<vec3_t> q_li;                       //< nend
	};

	Coef  coef;
	
	vector<End>    ends;
	
public:
    virtual void AddVar(Solver* solver);
	virtual void AddCon(Solver* solver);
	virtual void Prepare();
	virtual void PrepareStep();
	virtual void Finish ();
	virtual void Draw(Render::Canvas* canvas, Render::Config* conf);

	CentroidKey();
};

class Centroid : public TrajectoryNode{
public:
	struct Param {
		real_t	m;  //< mass
		mat3_t  I;  //< inertia
		mat3_t  Iinv;
		real_t	g;  //< gravity
		real_t  mu;

        vec3_t  bodyRangeMin;
        vec3_t  bodyRangeMax;


		real_t  durationMin;
		real_t  durationMax;
        
        real_t  swingSlope;
        real_t  swingHeight;

		bool  enableRotation;    ///< enable rotational dynamics
		bool  enableQuaternion;
		bool  lockRpy[3];
		int   rotationResolution;

		real_t  contactMargin;
		real_t  contactSwitchCost;
		real_t  contactFaceSwitchCost;
		vector<real_t>  numOfContactsCost;

		string  contactPattern;
		
		Param();
	};
	
	struct Scale{
		real_t  l;
		real_t  t, tinv;   //< time scaling
		real_t  pt, pt2;  //< position scaling
		real_t  pr;
		real_t  vt;  //< velocity scaling
		real_t  vr;  //< angular velocity scaling
		real_t  at;
		real_t  ar;
		real_t  ft;  //< force scaling
		real_t  fr;  //< moment scaling
		real_t  L;   //< momentum scaling
	};

   	struct End{
		vec3_t  basePos;
		vec3_t  posMin;
		vec3_t  posMax;
		vec2_t  copMin;
        vec2_t  copMax;
        real_t  stiffnessMax;
        bool    lockOri;
		bool    lockCmp;
		bool    lockMoment;
		vec2_t  cmpOffset;
		
        Point*  point;

        End();
	};

	struct Waypoint {
		struct End{
			struct Value{
				vec3_t  pos_t;  //< specified in local coordinate
				vec3_t  pos_r;
				vec3_t  vel_t;
				vec3_t  vel_r;
				int     iface;

				Value();
				Value(vec3_t _p, vec3_t _q, vec3_t _v, vec3_t _w, int _iface);
			};
			struct Weight{
				vec3_t  pos_t ;
				vec3_t  pos_r ;
				vec3_t  vel_t ;
				vec3_t  vel_r ;
				real_t  stiff ;
				vec2_t  cmp   ;
				vec3_t  moment;

				Weight();
				Weight(vec3_t _p, vec3_t _q, vec3_t _v, vec3_t _w, real_t _l, vec2_t _r, vec3_t _eta);
			};

			Value   value;
			Weight  weight;
			
            End();
		};

		struct Value{
			real_t  time;
			real_t  duration;
			vec3_t  pos_t;
			vec3_t  pos_r;
			vec3_t  vel_t;
			vec3_t  vel_r;

			Value();
			Value(real_t _t, real_t _tau, vec3_t _p, vec3_t _q, vec3_t _v, vec3_t _w);
		};
		struct Weight{
			real_t  time;
			real_t  duration;
			vec3_t  pos_t;
			vec3_t  pos_r;
			vec3_t  vel_t;
			vec3_t  vel_r;

			Weight();
			Weight(real_t _t, real_t _tau, vec3_t _p, vec3_t _q, vec3_t _v, vec3_t _w);
		};

		Value   value;
		Weight  weight;
		
		vector<End>  ends;

		Waypoint();
	};
	
	struct Snapshot{
		struct End{
			vec3_t  pos;
			quat_t  ori;
			vec3_t  vel;
            vec3_t  angvel;
            real_t  stiffness;
			vec2_t  cmp;
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
	Scale               scale;
    vector<End>         ends;
    vector<Face>        faces;
	vector<Waypoint>    waypoints;
    
	Snapshot            snapshot;
	vector<Snapshot>    trajectory;
	bool                trajReady;

    Point*              point;  //< geometries used for internal computation
    Hull*               hull;

	virtual Keypoint*	CreateKeypoint() { return new CentroidKey(); }
	virtual void		Init   ();
	virtual void		Prepare();
	virtual void		PrepareStep();
	virtual void        Finish ();
	virtual void        CreateSnapshot(real_t time);
	virtual void        DrawSnapshot  (Render::Canvas* canvas, Render::Config* conf);
	virtual void        Draw          (Render::Canvas* canvas, Render::Config* conf);

    void SetScaling     ();
	void ComState       (real_t t, vec3_t& pos, vec3_t& vel, vec3_t& acom);
	void TorsoState     (real_t t, quat_t& ori, vec3_t& angvel, int type = Interpolate::SlerpDiff);
	void EndState       (real_t t, int index, vec3_t& pos, quat_t& ori, vec3_t& vel, vec3_t& angvel);
    void EndForce       (real_t t, int index, real_t& stiff, vec2_t& cmp, vec3_t& moment, bool& contact);
	void EndSwitchTiming(real_t t, int index, real_t& tprev, real_t& tnext);
	void CreateSnapshot (real_t t, Snapshot& s);
	void CalcTrajectory ();
	
public:
	         Centroid(Graph* g, string n);
	virtual ~Centroid();
};

struct CentroidCon : Constraint {
	CentroidKey*  obj[2];
	
	void Prepare();

	CentroidCon(Solver* solver, int _dim, int _tag, string _name, CentroidKey* _obj, real_t _scale);
};

struct CentroidComCon : CentroidCon{
	void Prepare();

	CentroidComCon(Solver* solver, int _dim, int _tag, string _name, CentroidKey* _obj, real_t _scale);
};

struct CentroidPosConT : CentroidComCon{
	void Prepare();
	
	virtual void  CalcCoef();
	virtual void  CalcDeviation();
		
	CentroidPosConT(Solver* solver, string _name, CentroidKey* _obj, real_t _scale);
};

struct CentroidVelConT : CentroidComCon{
	void Prepare();

	virtual void  CalcCoef();
	virtual void  CalcDeviation();
		
	CentroidVelConT(Solver* solver, string _name, CentroidKey* _obj, real_t _scale);
};

struct CentroidPosConR : CentroidCon{
	void Prepare();

	virtual void  CalcCoef();
	virtual void  CalcDeviation();
		
	CentroidPosConR(Solver* solver, string _name, CentroidKey* _obj, real_t _scale);
};

struct CentroidPosConRPY : CentroidCon{
	void Prepare();

	virtual void  CalcCoef();
	virtual void  CalcDeviation();
		
	CentroidPosConRPY(Solver* solver, string _name, CentroidKey* _obj, real_t _scale);
};

struct CentroidVelConR : CentroidCon{
	void Prepare();

	virtual void  CalcCoef();
	virtual void  CalcDeviation();
		
	CentroidVelConR(Solver* solver, string _name, CentroidKey* _obj, real_t _scale);
};

struct CentroidTimeCon : CentroidCon{
	real_t t, t_lhs, t_rhs;

	void Prepare();

	virtual void  CalcCoef();
	virtual void  CalcDeviation();
		
	CentroidTimeCon(Solver* solver, string _name, CentroidKey* _obj, real_t _scale);
};

struct CentroidEndPosConT : CentroidCon{
	int    iend;
	vec3_t pe, ve, ae, pe_lhs, pe_rhs;
	
	void Prepare();

	virtual void  CalcCoef();
	virtual void  CalcDeviation();
		
	CentroidEndPosConT(Solver* solver, string _name, CentroidKey* _obj, int _iend, real_t _scale);
};

struct CentroidEndPosConR : CentroidCon{
	int    iend;
	quat_t qe, qe_lhs, qe_rhs, q_omega;
	vec3_t we, ue, omega;
	mat3_t R_omega, A_omega;
	
	void Prepare();

	virtual void  CalcCoef();
	virtual void  CalcDeviation();
		
	CentroidEndPosConR(Solver* solver, string _name, CentroidKey* _obj, int _iend, real_t _scale);
};

struct CentroidDurationRangeCon : Constraint{
	CentroidKey* obj;
	real_t       dir;
	real_t       bound;
	
	virtual void  CalcCoef();
	virtual void  CalcDeviation();

	CentroidDurationRangeCon(Solver* solver, string _name, CentroidKey* _obj, real_t _dir, real_t _scale);
};

struct CentroidEndPosRangeCon : Constraint{
	CentroidKey* obj;
	int          iend;
	int          idx;
	real_t       dir;
	vec3_t       p, pe, pbase, eta, eta_abs;
	quat_t       q;
	real_t       bound;
	
	void Prepare();

	virtual void  CalcCoef();
	virtual void  CalcDeviation();

	CentroidEndPosRangeCon(Solver* solver, string _name, CentroidKey* _obj, int _iend, int _idx, real_t _dir, real_t _scale);
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

struct CentroidEndFrictionCon : Constraint{
	CentroidKey*  obj;
	int           iend;
    vec3_t        f, d;
	vec2_t        ft, ftn;
	real_t        ftnorm, mu;
    
	void Prepare();

	virtual void  CalcCoef();
	virtual void  CalcDeviation();

	CentroidEndFrictionCon(Solver* solver, string _name, CentroidKey* _obj, int _iend, real_t _scale);
};

struct CentroidEndMomentCon : Constraint{
	CentroidKey*  obj;
	int           iend;
    int           idx;
	real_t        dir;
	vec3_t        bound;
	vec3_t        f, eta;
	mat3_t        Rc;
    
	void Prepare();

	virtual void  CalcCoef();
	virtual void  CalcDeviation();

	CentroidEndMomentCon(Solver* solver, string _name, CentroidKey* _obj, int _iend, int _idx, int _dir, real_t _scale);
};

}
