#pragma once

#include <DiMP/Graph/Node.h>
#include <DiMP/Graph/Geometry.h>
#include <DiMP/Graph/Solver.h>

#include <array>

namespace DiMP {;

class  Wholebody;
struct WholebodyPosConT;
struct WholebodyPosConR;
struct WholebodyVelConT;
struct WholebodyVelConR;
struct WholebodyJointPosCon;
struct WholebodyJointVelCon;
struct WholebodyCentroidPosConT;
struct WholebodyCentroidVelConT;
struct WholebodyCentroidPosConR;
struct WholebodyCentroidVelConR;
struct WholebodyDesPosConT;
struct WholebodyDesPosConR;
struct WholebodyDesVelConT;
struct WholebodyDesVelConR;
struct WholebodyLimitCon;
//struct WholebodyLdCon;
struct WholebodyLCon;
struct WholebodyContactPosConT;
struct WholebodyContactPosConR;
struct WholebodyContactVelConT;
struct WholebodyContactVelConR;
struct WholebodyNormalForceCon;
struct WholebodyFrictionForceCon;
struct WholebodyMomentCon;

class WholebodyCallback;

typedef PTM::TMatrixCol<6,6,real_t>  mat6_t;
typedef PTM::TMatrixCol<6,3,real_t>  mat63_t;
typedef PTM::TMatrixCol<3,6,real_t>  mat36_t;

struct WholebodyData{
	struct Link{
		vec3_t  pos_t;  // position (base link local)
		quat_t  pos_r;  // orientation
		vec3_t  vel_t;  // velocity
		vec3_t  vel_r;  // angular velocity
		vec3_t  acc_t;  // acceleration
		vec3_t  acc_r;  // angular acceleration
		vec3_t  force_t, force_t_par, force_t_child;
		vec3_t  force_r, force_r_par, force_r_child;
		mat3_t  I;
	};

	struct End{
		vec3_t  pos_t, pos_tc, pos_te;
		quat_t  pos_r, pos_rc;
		vec3_t  vel_t;
		vec3_t  vel_r;
		vec3_t  acc_t;
		vec3_t  acc_r;
		vec3_t  force_t, force_target_t;
		vec3_t  force_r, force_target_r;

		vec3_t  pos_t_weight, pos_r_weight;
		vec3_t  vel_t_weight, vel_r_weight;
		vec3_t  acc_t_weight, acc_r_weight;
		vec3_t  force_t_weight, force_r_weight;

		int     state;       ///< contact state
		real_t  mu;          ///< friction
		vec2_t  cop_min;
		vec2_t  cop_max;

		End();
	};

	struct Base{
		quat_t pos_r;
		vec3_t vel_r;
		vec3_t acc_r;
		vec3_t pos_r_weight;
		vec3_t vel_r_weight;
		vec3_t acc_r_weight;

		Base();
	};

	struct Centroid{
		vec3_t pos_t;   ///< com position (local), desired com position (global)
		vec3_t vel_t;   ///< com velocity (local), desired com velocity (global)
		vec3_t acc_t;
		quat_t pos_r;
		vec3_t vel_r;
		vec3_t acc_r;
		vec3_t pos_t_weight;
		vec3_t vel_t_weight;
		vec3_t pos_r_weight;
		vec3_t vel_r_weight;
		vec3_t L, Ld;                  ///< momentum (local) and its derivative
		vec3_t L_weight;
		//vec3_t Ld_weight;
		mat3_t I, Iinv;                ///< inertia matrix around com and its inverse

		Centroid();
	};

	Base          base;
	Centroid      centroid;
	vector<End>   ends;
	vector<Link>  links;
	
	vvec_t q, qd, qdd, tau;
	vvec_t q_weight, qd_weight, qdd_weight;
	vvec_t q_min, q_max;
	vvec_t e;
	vector<vmat_t>   Jq;
	vector<vmat_t>   Je;

	vmat_t                   Jik_e_v0;
	vector< vmat_t >         Jik_e_ve;              //< jacobian from end pose to error
	vmat_t                   Jik_q_v0;
	vector<vmat_t>           Jik_q_ve;
	vector<mat6_t>           Jfk_vi_v0;
	vector<vmat_t>           Jfk_vi_q;
	vector< mat6_t >         Jfkik_vi_v0;   //< jacobian from base velocity to link velocity
	vector< vector<mat6_t> > Jfkik_vi_ve;   //< jacobian from end pose to link pose
	
	void Init        (Wholebody* wb);
	void InitJacobian(Wholebody* wb);
};

/*
 *  whold-body kinematics and (semi)dynamics
 */
class WholebodyKey : public Keypoint {
public:
	Wholebody*  wb;
	
	struct End{
		V3Var*  var_pos_t;    ///< position (base link: global, otherwise: base link local)
		QVar*   var_pos_r;    ///< orientation
		V3Var*  var_vel_t;    ///< velocity
		V3Var*  var_vel_r;    ///< angular velocity
		V3Var*  var_acc_t;    ///< acceleration
		V3Var*  var_acc_r;    ///< angular acceleration
		V3Var*  var_force_t;  ///< force (contact frame)
		V3Var*  var_force_r;  ///< moment
		
		WholebodyPosConT*     con_pos_t;
		WholebodyPosConR*     con_pos_r;
		WholebodyVelConT*     con_vel_t;
		WholebodyVelConR*     con_vel_r;

		WholebodyDesPosConT*  con_des_pos_t  ;   ///< desired position (global)
		WholebodyDesPosConR*  con_des_pos_r  ;   ///< desired orientation
		WholebodyDesVelConT*  con_des_vel_t  ;   ///< desired velocity
		WholebodyDesVelConR*  con_des_vel_r  ;   ///< desired angular velocity
		FixConV3*             con_des_acc_t  ;   ///< desired acceleration (local)
		FixConV3*             con_des_acc_r  ;   ///< desired angular acceleration
		FixConV3*             con_des_force_t;   ///< desired force (contact frame)
		FixConV3*             con_des_force_r;   ///< desired moment
		
		WholebodyContactPosConT*    con_contact_pos_t;
		WholebodyContactPosConR*    con_contact_pos_r;
		WholebodyContactVelConT*    con_contact_vel_t;
		WholebodyContactVelConR*    con_contact_vel_r;
		WholebodyNormalForceCon*    con_force_normal;
		WholebodyFrictionForceCon*  con_force_friction[2][2];
		WholebodyMomentCon*         con_moment[2][2];
	};

	struct Base{
		QVar*   var_pos_r;
		V3Var*  var_vel_r;
		V3Var*  var_acc_r;
		
		WholebodyPosConR*     con_pos_r;
		WholebodyVelConR*     con_vel_r;

		FixConQ*  con_des_pos_r;
		FixConV3* con_des_vel_r;
		FixConV3* con_des_acc_r;
	};

	struct Centroid{		
		V3Var*  var_pos_t;
		V3Var*  var_vel_t;
		QVar*   var_pos_r;
		V3Var*  var_vel_r;

		WholebodyCentroidPosConT*  con_pos_t;
		WholebodyCentroidVelConT*  con_vel_t;
		WholebodyCentroidPosConR*  con_pos_r;
		WholebodyCentroidVelConR*  con_vel_r;

		FixConV3*  con_des_pos_t;
		FixConV3*  con_des_vel_t;
		FixConQ*   con_des_pos_r;
		FixConV3*  con_des_vel_r;

		//WholebodyLdCon*        con_Ld;
		WholebodyLCon*         con_L;
	};

	struct Joint{
		SVar*  var_q;
		SVar*  var_qd;
		SVar*  var_qdd;

		WholebodyJointPosCon*  con_q;
		WholebodyJointVelCon*  con_qd;

		FixConS*  con_des_q;
		FixConS*  con_des_qd;
		FixConS*  con_des_qdd;

		RangeConS* con_range_q;
	};

	vector<WholebodyLimitCon*>  con_limit;
	
	WholebodyData          data;
	WholebodyData          data_des;
	vector<WholebodyData>  data_tmp;

	Base           base;
	Centroid       centroid;
	vector<End>    ends;
	vector<Joint>  joints;

	// working variables
	quat_t          q0;
	mat3_t          R0;
	vector<vec3_t>  re, fe, me;
	vector<mat3_t>  rec;
	vec3_t          fsum, msum;
	mat3_t          J_L_qb, J_L_wb;
	mat3_t          J_Ld_qb, J_Ld_ub;
	vector<mat3_t>  J_L_pe, J_L_qe, J_L_ve, J_L_we;
	vmat_t          J_L_q, J_L_qd;
	vector<mat3_t>  J_Ld_pe, J_Ld_qe, J_Ld_ae, J_Ld_ue;
	vmat_t          J_Ld_q, J_Ld_qdd;
	
public:	
    virtual void AddVar(Solver* solver);
	virtual void AddCon(Solver* solver);
	virtual void Prepare     ();
	virtual void PrepareStep ();
	virtual void Finish      ();
	virtual void Draw(Render::Canvas* canvas, Render::Config* conf);

	WholebodyKey();
};

class Wholebody : public TrajectoryNode{
public:
	struct ContactState{
		enum{
			Free,
			Surface,
			Line,
			Point,
		};
	};

	struct Parametrization{
		enum{
			End,
			Joint,
		};
	};

	struct Param {
		real_t  totalMass;  ///< total mass of wholebody
		real_t  gravity;
		bool    analyticalJacobian;
		bool    useLd;
		int     comIkNumIter;
		real_t  comIkRatio;
		int     parametrization;
		
		Param();
	};

	struct Joint{

	};

	struct Link{
		real_t       mass;         ///< mass of link
		real_t       mass_ratio;
		mat3_t       inertia;
		int          iparent;      ///< parent link index
		vector<int>  ichildren;    ///< child link indices
		int          ijoint;       ///< joint index
		int          iend;
		vec3_t       trn;          ///< translation from parent
		vec3_t       axis;         ///< joint axis
	
		Link(real_t _mass = 0.0, vec3_t _inertia = vec3_t(), int _iend = -1, int _iparent = -1, int _ijoint = -1, vec3_t _trn = vec3_t(), vec3_t _axis = vec3_t());
	};

	struct End{
		int    ilink;    ///< link index
		vec3_t offset;

		End(int _ilink = 0.0, vec3_t _offset = vec3_t());
	};

	struct Limit{
		int     ichain;
		int     type;
		real_t  scale;

		Limit(){}
		Limit(int _ichain, int _type, real_t _scale);
	};

	struct Chain{
		vector<int>   ilink;
		vector<int>   ilimit;
		
		Chain(){}
		Chain(const vector<int>& _ilink, const vector<int>& _ilimit);
	};
	
   	struct Snapshot{
		struct Link{
			vec3_t  pos_t;
			quat_t  pos_r;
			vec3_t  vel_t;
			vec3_t  vel_r;
			vec3_t  force_t;
			vec3_t  force_r;
		};
	
		real_t  t;
		vec3_t  com_pos;
		vec3_t  com_vel;
		quat_t  base_pos_r;
		vec3_t  base_vel_r;
		vector<Link>  links;
		
		Snapshot();
	};

    Param	            param;
    vector<Link>        links;
	vector<Joint>       joints;
	vector<Limit>       limits;
	vector<End>         ends;
	vector<Chain>       chains;
	WholebodyCallback*  callback;
	real_t              sl;
	real_t              st;   //< time scaling
	real_t              spt;  //< position scaling
	real_t              spr;
	real_t              svt;  //< velocity scaling
	real_t              svr;  //< angular velocity scaling
	real_t              sat;
	real_t              sar;
	real_t              sft;  //< force scaling
	real_t              sfr;  //< moment scaling
	real_t              sL;   //< momentum scaling
	
	Snapshot            snapshot;
	vector<Snapshot>    trajectory;
	bool                trajReady;

	virtual Keypoint*	CreateKeypoint() { return new WholebodyKey(); }
	virtual void		Init   ();
	virtual void		Prepare();
	virtual void		PrepareStep();
	virtual void        Finish ();
	virtual void        CreateSnapshot(real_t time);
	virtual void        DrawSnapshot  (Render::Canvas* canvas, Render::Config* conf);
	virtual void        Draw          (Render::Canvas* canvas, Render::Config* conf);

	void SetScaling();
	void Reset();
	void Shift();
	void Setup();
	void CalcFK                (WholebodyData& d, int ichain, bool calc_end);
	void CalcIK                (WholebodyData& d, int ichain, bool calc_jacobian);
	void CalcPosition          (WholebodyData& d, bool fk_or_ik);
	void CalcJacobian          (WholebodyData& d, vector<WholebodyData>& dtmp);
	void CalcJacobianAnalytical(WholebodyData& d);
	void CalcJacobianNumerical (WholebodyData& d, vector<WholebodyData>& dtmp);
	void CalcJacobianNumerical2(WholebodyData& d, vector<WholebodyData>& dtmp);
	void SaveJacobian          (WholebodyData& d);
	void TransformJacobian     (WholebodyData& d);
	void CalcVelocity          (WholebodyData& d, bool fk_or_ik);
	void CalcAcceleration      (WholebodyData& d, bool fk_or_ik);
	void CalcComAcceleration   (WholebodyData& d);
	void CalcBaseAcceleration  (WholebodyData& d);
	void CalcMomentum          (WholebodyData& d);
	void CalcMomentumDerivative(WholebodyData& d);
	void CalcForce             (WholebodyData& d);
	
	void ComState    (real_t t, vec3_t& pos, vec3_t& vel   );
	void BaseState   (real_t t, quat_t& ori, vec3_t& angvel);
	void LinkPose    (real_t t, int i, vec3_t& pos  , quat_t& ori   );
    void LinkVelocity(real_t t, int i, vec3_t& vel  , vec3_t& angvel);
    void LinkForce   (real_t t, int i, vec3_t& force, vec3_t& moment);
    
	void CreateSnapshot(real_t t, Snapshot& s);
	void CalcTrajectory();
	
public:
	         Wholebody(Graph* g, string n);
	virtual ~Wholebody();
};

class WholebodyCallback{
public:
	virtual void   CalcIK(int ichain, const vec3_t& pe_local, const quat_t& qe_local, vvec_t& joint, vvec_t& error, vmat_t& Jq, vmat_t& Je, bool calc_jacobian) = 0;
	virtual void   GetInitialState(WholebodyData& d) = 0;
	virtual void   GetDesiredState(int k, real_t t, WholebodyData& d) = 0;
};

struct WholebodyCon : Constraint {
	WholebodyKey*  obj[2];

	WholebodyCon(Solver* solver, int _dim, int _tag, string _name, WholebodyKey* _obj, real_t _scale);
};

struct WholebodyPosConT : WholebodyCon{
	int iend;
	vec3_t p0, v0, a0, p1, p_rhs;
	real_t h, h2;
	
	void Prepare();

	virtual void  CalcCoef();
	virtual void  CalcDeviation();
		
	WholebodyPosConT(Solver* solver, string _name, WholebodyKey* _obj, int _iend, real_t _scale);
};

struct WholebodyPosConR : WholebodyCon{
	int iend;
	quat_t q0, q1, q_rhs;
	vec3_t w0, u0;
	real_t h, h2;
	
	void Prepare();

	virtual void  CalcCoef();
	virtual void  CalcDeviation();
		
	WholebodyPosConR(Solver* solver, string _name, WholebodyKey* _obj, int _iend, real_t _scale);
};

struct WholebodyVelConT : WholebodyCon{
	int iend;
	vec3_t v0, a0, v1, v_rhs;
	real_t h;
	
	void Prepare();

	virtual void  CalcCoef();
	virtual void  CalcDeviation();
		
	WholebodyVelConT(Solver* solver, string _name, WholebodyKey* _obj, int _iend, real_t _scale);
};

struct WholebodyVelConR : WholebodyCon{
	int iend;
	vec3_t w0, u0, w1, w_rhs;
	real_t h;
	
	void Prepare();

	virtual void  CalcCoef();
	virtual void  CalcDeviation();
		
	WholebodyVelConR(Solver* solver, string _name, WholebodyKey* _obj, int _iend, real_t _scale);
};

struct WholebodyJointPosCon : WholebodyCon{
	int    ijoint;
	real_t q0, qd0, qdd0, q1, q_rhs;
	real_t h, h2;
	
	void Prepare();

	virtual void  CalcCoef();
	virtual void  CalcDeviation();
		
	WholebodyJointPosCon(Solver* solver, string _name, WholebodyKey* _obj, int _ijoint, real_t _scale);
};

struct WholebodyJointVelCon : WholebodyCon{
	int    ijoint;
	real_t qd0, qdd0, qd1, qd_rhs;
	real_t h;
	
	void Prepare();

	virtual void  CalcCoef();
	virtual void  CalcDeviation();
		
	WholebodyJointVelCon(Solver* solver, string _name, WholebodyKey* _obj, int _ijoint, real_t _scale);
};

struct WholebodyCentroidPosConT : WholebodyCon{
	real_t h, h2, m;
	vec3_t g;
	vec3_t pc0, vc0, ac0, pc1, pc_rhs;
	
	void Prepare();

	virtual void  CalcCoef();
	virtual void  CalcDeviation();
		
	WholebodyCentroidPosConT(Solver* solver, string _name, WholebodyKey* _obj, real_t _scale);
};

struct WholebodyCentroidVelConT : WholebodyCon{
	real_t h, m;
	vec3_t g;
	vec3_t vc0, ac0, vc1, vc_rhs;
	
	void Prepare();

	virtual void  CalcCoef();
	virtual void  CalcDeviation();
		
	WholebodyCentroidVelConT(Solver* solver, string _name, WholebodyKey* _obj, real_t _scale);
};

struct WholebodyCentroidPosConR : WholebodyCon{
	quat_t q1, q_rhs;
	vec3_t w0, u0, Ld;
	mat3_t Iinv;
	real_t h, h2;
	
	void Prepare();

	virtual void  CalcCoef();
	virtual void  CalcDeviation();
		
	WholebodyCentroidPosConR(Solver* solver, string _name, WholebodyKey* _obj, real_t _scale);
};

struct WholebodyCentroidVelConR : WholebodyCon{
	vec3_t pc, w0, u0, w1, w_rhs, Ld;
	mat3_t Iinv;
	real_t h;
	
	void Prepare();

	virtual void  CalcCoef();
	virtual void  CalcDeviation();
		
	WholebodyCentroidVelConR(Solver* solver, string _name, WholebodyKey* _obj, real_t _scale);
};

struct WholebodyDesPosConT : Constraint{
	WholebodyKey*  obj;
	int    iend;
	vec3_t desired;
	vec3_t pc, pi;
	quat_t q0;
	mat3_t R0;
	
	void Prepare();

	virtual void  CalcCoef();
	virtual void  CalcDeviation();
		
	WholebodyDesPosConT(Solver* solver, string _name, WholebodyKey* _obj, int _iend, real_t _scale);
};

struct WholebodyDesPosConR : Constraint{
	WholebodyKey*  obj;
	int    iend;
	quat_t desired;
	quat_t q0, qi;
	mat3_t R0;

	void Prepare();

	virtual void  CalcCoef();
	virtual void  CalcDeviation();
		
	WholebodyDesPosConR(Solver* solver, string _name, WholebodyKey* _obj, int _iend, real_t _scale);
};

struct WholebodyDesVelConT : Constraint{
	WholebodyKey*  obj;
	int    iend;
	vec3_t desired;
	vec3_t vc, w0, pi, vi;
	quat_t q0;
	mat3_t R0;

	void Prepare();

	virtual void  CalcCoef();
	virtual void  CalcDeviation();
		
	WholebodyDesVelConT(Solver* solver, string _name, WholebodyKey* _obj, int _iend, real_t _scale);
};

struct WholebodyDesVelConR : Constraint{
	WholebodyKey*  obj;
	int    iend;
	vec3_t desired;
	vec3_t w0, wi;
	quat_t q0;
	mat3_t R0;
	
	void Prepare();

	virtual void  CalcCoef();
	virtual void  CalcDeviation();
		
	WholebodyDesVelConR(Solver* solver, string _name, WholebodyKey* _obj, int _iend, real_t _scale);
};

struct WholebodyLimitCon : Constraint{
	WholebodyKey*  obj;
	int  ierror;
	
	void Prepare();

	virtual void  CalcCoef();
	virtual void  CalcDeviation();
	
	WholebodyLimitCon(Solver* solver, string _name, WholebodyKey* _obj, int _ierror, int _type, real_t _scale);
};

/*struct WholebodyLdCon : Constraint{
	WholebodyKey*  obj;
	
	void Prepare();

	virtual void  CalcCoef();
	virtual void  CalcDeviation();
	
	WholebodyLdCon(Solver* solver, string _name, WholebodyKey* _obj, real_t _scale);
};*/

struct WholebodyLCon : Constraint{
	WholebodyKey*  obj;
	
	void Prepare();

	virtual void  CalcCoef();
	virtual void  CalcDeviation();
	
	WholebodyLCon(Solver* solver, string _name, WholebodyKey* _obj, real_t _scale);
};

struct WholebodyContactPosConT : Constraint{
	WholebodyKey*  obj;
	int    iend;
	vec3_t pc, pi, po, r;
	quat_t q0, qi, qo;
	mat3_t R0, Ro;

	void Prepare();

	virtual void  CalcCoef();
	virtual void  CalcDeviation();
	
	WholebodyContactPosConT(Solver* solver, string _name, WholebodyKey* _obj, int _iend, real_t _scale);
};

struct WholebodyContactPosConR : Constraint{
	WholebodyKey*  obj;
	int    iend;
	quat_t q0, qi, qo;
	mat3_t R0, Ro;
	
	void Prepare();

	virtual void  CalcCoef();
	virtual void  CalcDeviation();
	
	WholebodyContactPosConR(Solver* solver, string _name, WholebodyKey* _obj, int _iend, real_t _scale);
};

struct WholebodyContactVelConT : Constraint{
	WholebodyKey*  obj;
	int    iend;
	vec3_t vc, w0, pi, vi, wi, po, r;
	quat_t q0, qi, qo;
	mat3_t R0, Ro;
	
	void Prepare();

	virtual void  CalcCoef();
	virtual void  CalcDeviation();
	
	WholebodyContactVelConT(Solver* solver, string _name, WholebodyKey* _obj, int _iend, real_t _scale);
};

struct WholebodyContactVelConR : Constraint{
	WholebodyKey*  obj;
	int    iend;
	quat_t q0, qi, qo;
	mat3_t R0, Ro;
	vec3_t w0, wi;
	
	void Prepare();

	virtual void  CalcCoef();
	virtual void  CalcDeviation();
	
	WholebodyContactVelConR(Solver* solver, string _name, WholebodyKey* _obj, int _iend, real_t _scale);
};

struct WholebodyNormalForceCon : Constraint{
	WholebodyKey*  obj;
	int    iend;
	vec3_t n, f;
	real_t fn;

	void Prepare();

	virtual void  CalcCoef();
	virtual void  CalcDeviation();
	
	WholebodyNormalForceCon(Solver* solver, string _name, WholebodyKey* _obj, int _iend, real_t _scale);
};

struct WholebodyFrictionForceCon : Constraint{
	WholebodyKey*  obj;
	int    iend;
	int    dir;   //< x or y
	int    side;  //< upper or lower bound
	real_t fn, ft, mu;
	
	void Prepare();

	virtual void  CalcCoef();
	virtual void  CalcDeviation();
	
	WholebodyFrictionForceCon(Solver* solver, string _name, WholebodyKey* _obj, int _iend, int _dir, int _side, real_t _scale);
};

struct WholebodyMomentCon : Constraint{
	WholebodyKey*  obj;
	int    iend;
	int    dir;   //< x or y
	int    side;  //< upper or lower bound
	real_t fn;
	vec2_t m;
	vec2_t cmin, cmax;

	void Prepare();

	virtual void  CalcCoef();
	virtual void  CalcDeviation();
	
	WholebodyMomentCon(Solver* solver, string _name, WholebodyKey* _obj, int _iend, int _dir, int _side, real_t _scale);
};

}
