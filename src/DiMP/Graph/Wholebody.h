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
struct WholebodyComPosCon;
struct WholebodyComVelCon;
struct WholebodyBasePosConR;
struct WholebodyBaseVelConR;
struct WholebodyDesPosConT;
struct WholebodyDesPosConR;
struct WholebodyDesVelConT;
struct WholebodyDesVelConR;
struct WholebodyLimitCon;
struct WholebodyContactPosConT;
struct WholebodyContactPosConR;
struct WholebodyContactVelConT;
struct WholebodyContactVelConR;
struct WholebodyNormalForceCon;
struct WholebodyFrictionForceCon;
struct WholebodyMomentCon;

class WholebodyCallback;

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
	};

	struct End{
		vec3_t  pos_t, pos_t_des, pos_t_ini, pos_tc;
		quat_t  pos_r, pos_r_des, pos_r_ini, pos_rc;
		vec3_t  vel_t, vel_t_des, vel_t_ini;
		vec3_t  vel_r, vel_r_des, vel_r_ini;
		vec3_t  acc_t, acc_t_des, acc_t_ini;
		vec3_t  acc_r, acc_r_des, acc_r_ini;
		vec3_t  force_t, force_t_des, force_t_ini;
		vec3_t  force_r, force_r_des, force_r_ini;

		vec3_t pos_t_weight, pos_r_weight;
		vec3_t vel_t_weight, vel_r_weight;
		vec3_t acc_t_weight, acc_r_weight;
		vec3_t force_t_weight, force_r_weight;

		//vec3_t  center;  //< contact center in link's local coordinate
		//real_t  offset;  //< offset from contact plane to contact center

		//vec3_t  rlc, rcc;
		//mat3_t  Rc, rcc_cross_Rc;
	
		int     state;       ///< contact state
		real_t  mu;          ///< friction
		vec2_t  cop_min;
		vec2_t  cop_max;

		End();
	};

	vector<Link>  links;
	vector<End>   ends;

	vec3_t com_pos, com_pos_des, com_pos_ini;   ///< com position (local), desired com position (global)
	vec3_t com_vel, com_vel_des, com_vel_ini;   ///< com velocity (local), desired com velocity (global)
	vec3_t com_pos_weight;
	vec3_t com_vel_weight;
	vec3_t com_acc;
	quat_t base_pos_r, base_pos_r_des, base_pos_r_ini;
	vec3_t base_vel_r, base_vel_r_des, base_vel_r_ini;
	vec3_t base_pos_r_weight;
	vec3_t base_vel_r_weight;
	vec3_t base_acc_r;
	vec3_t L, Ld;                  ///< momentum (local) and its derivative
	mat3_t I, Iinv;                ///< inertia matrix around com and its inverse
	vvec_t q, qd, tau;
	vvec_t e;

	void Init(Wholebody* wb);
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
		
	V3Var*  var_com_pos;
	V3Var*  var_com_vel;
	QVar*   var_base_pos_r;
	V3Var*  var_base_vel_r;

	WholebodyComPosCon*    con_com_pos;
	WholebodyComVelCon*    con_com_vel;
	WholebodyBasePosConR*  con_base_pos_r;
	WholebodyBaseVelConR*  con_base_vel_r;
	FixConV3*  con_des_com_pos;
	FixConV3*  con_des_com_vel;
	FixConQ*   con_des_base_pos_r;
	FixConV3*  con_des_base_vel_r;
	
	vector<WholebodyLimitCon*>  con_limit;
	
	WholebodyData          data;
	WholebodyData          data_tmp[3];
	vector< vector<WholebodyData> >  data_tmp2;
	
	vector<vmat_t>  J_e_pe, J_e_qe;              //< jacobian from end pose to error
	vector< vector<mat3_t> > J_pi_pe, J_pi_qe;   //< jacobian from end pose to link position

	vector<End>    ends;
	
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

	struct Param {
		real_t  total_mass;  ///< total mass of wholebody
		real_t  gravity;
		
		Param();
	};

	struct Joint{

	};

	struct Link{
		real_t       mass;         ///< mass of link
		real_t       mass_ratio;
		int          iparent;      ///< parent link index
		vector<int>  ichildren;    ///< child link indices
		int          ijoint;       ///< joint index
		bool         is_end;
		vec3_t       trn;          ///< translation from parent
		vec3_t       axis;         ///< joint axis
	
		Link(real_t _mass = 0.0, bool _is_end = false, int _iparent = -1, int _ijoint = -1, vec3_t _trn = vec3_t(), vec3_t _axis = vec3_t());
	};

	struct End{
		int  ilink;    ///< link index

		End();
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
	virtual void        Finish ();
	virtual void        CreateSnapshot(real_t time);
	virtual void        DrawSnapshot  (Render::Canvas* canvas, Render::Config* conf);
	virtual void        Draw          (Render::Canvas* canvas, Render::Config* conf);

	void SetScaling();
	void Setup();
	void CalcPosition        (WholebodyData& d);
	void CalcVelocity        (const WholebodyData& d0, const WholebodyData& d1, WholebodyData& d, real_t epsinv);
	void CalcAcceleration    (const WholebodyData& d0, const WholebodyData& d1, WholebodyData& d, real_t epsinv);
	void CalcLinkPosition    (WholebodyData& d);
	//void CalcLinkVelocity    (WholebodyData& d, bool calc_end);
	void CalcComAcceleration (WholebodyData& d);
	void CalcBaseAcceleration(WholebodyData& d);
	void CalcMomentum        (WholebodyData& d);
	void CalcMomentumDerivative(WholebodyData& d);
	void CalcPVA             (WholebodyData* data_tmp, WholebodyData& data);
	void CalcForce           (WholebodyData& d);
	
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
	virtual void   CalcIK(WholebodyData& data) = 0;
	virtual void   Setup (int k, real_t t, WholebodyData& data) = 0;
};

struct WholebodyCon : Constraint {
	WholebodyKey*  obj[2];

	WholebodyCon(Solver* solver, int _dim, int _tag, string _name, WholebodyKey* _obj, real_t _scale);
};

struct WholebodyPosConT : WholebodyCon{
	int iend;
	vec3_t p0, p1, p_rhs;
	vec3_t v0;
	
	void Prepare();

	virtual void  CalcCoef();
	virtual void  CalcDeviation();
	virtual void  CalcLhs();
		
	WholebodyPosConT(Solver* solver, string _name, WholebodyKey* _obj, int _iend, real_t _scale);
};

struct WholebodyPosConR : WholebodyCon{
	int iend;
	quat_t q0, q1, q_rhs;
	vec3_t w0;
	
	void Prepare();

	virtual void  CalcCoef();
	virtual void  CalcDeviation();
	virtual void  CalcLhs();
		
	WholebodyPosConR(Solver* solver, string _name, WholebodyKey* _obj, int _iend, real_t _scale);
};

struct WholebodyVelConT : WholebodyCon{
	int iend;
	vec3_t v0, v1, v_rhs;
	vec3_t a0;
	
	void Prepare();

	virtual void  CalcCoef();
	virtual void  CalcDeviation();
	virtual void  CalcLhs();
		
	WholebodyVelConT(Solver* solver, string _name, WholebodyKey* _obj, int _iend, real_t _scale);
};

struct WholebodyVelConR : WholebodyCon{
	int iend;
	vec3_t w0, w1, w_rhs;
	vec3_t u0;
	
	void Prepare();

	virtual void  CalcCoef();
	virtual void  CalcDeviation();
	virtual void  CalcLhs();
		
	WholebodyVelConR(Solver* solver, string _name, WholebodyKey* _obj, int _iend, real_t _scale);
};

struct WholebodyComPosCon : WholebodyCon{
	vec3_t pc0, pc1, pc_rhs;
	vec3_t vc0;

	void Prepare();

	virtual void  CalcCoef();
	virtual void  CalcDeviation();
	virtual void  CalcLhs();
		
	WholebodyComPosCon(Solver* solver, string _name, WholebodyKey* _obj, real_t _scale);
};

struct WholebodyComVelCon : WholebodyCon{
	quat_t q0;
	mat3_t R0;
	vec3_t vc0, vc1, vc_rhs;
	vec3_t fsum;
	vector<vec3_t>  f;
	vector<mat3_t>  R;
	
	void Prepare();

	virtual void  CalcCoef();
	virtual void  CalcDeviation();
	virtual void  CalcLhs();
		
	WholebodyComVelCon(Solver* solver, string _name, WholebodyKey* _obj, real_t _scale);
};

struct WholebodyBasePosConR : WholebodyCon{
	quat_t q0, q1, q_rhs;
	vec3_t w0;
	
	void Prepare();

	virtual void  CalcCoef();
	virtual void  CalcDeviation();
	virtual void  CalcLhs();
		
	WholebodyBasePosConR(Solver* solver, string _name, WholebodyKey* _obj, real_t _scale);
};

struct WholebodyBaseVelConR : WholebodyCon{
	vec3_t pc;
	vec3_t w0, w1, w_rhs;
	quat_t q0;
	mat3_t R0;
	vec3_t msum;
	vec3_t Ld;
	mat3_t Iinv;
	vector<vec3_t>  r, f, m;
	vector<mat3_t>  R, rc;
	vector<mat3_t>  J_Ld_pe, J_Ld_qe, J_Ld_ae, J_Ld_ue;
	
	void Prepare();

	virtual void  CalcCoef();
	virtual void  CalcDeviation();
	virtual void  CalcLhs();
		
	WholebodyBaseVelConR(Solver* solver, string _name, WholebodyKey* _obj, real_t _scale);
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

struct WholebodyContactPosConT : Constraint{
	WholebodyKey*  obj;
	int    iend;
	vec3_t n, o;
	vec3_t pc, pi;
	quat_t q0, qi;

	void Prepare();

	virtual void  CalcCoef();
	virtual void  CalcDeviation();
	
	WholebodyContactPosConT(Solver* solver, string _name, WholebodyKey* _obj, int _iend, real_t _scale);
};

struct WholebodyContactPosConR : Constraint{
	WholebodyKey*  obj;
	int    iend;
	quat_t qc, q0, qi;
	mat3_t R0;
	
	void Prepare();

	virtual void  CalcCoef();
	virtual void  CalcDeviation();
	
	WholebodyContactPosConR(Solver* solver, string _name, WholebodyKey* _obj, int _iend, real_t _scale);
};

struct WholebodyContactVelConT : Constraint{
	WholebodyKey*  obj;
	int    iend;
	vec3_t n, vc, w0, pi, vi;
	quat_t q0, qi;
	mat3_t R0;
	
	void Prepare();

	virtual void  CalcCoef();
	virtual void  CalcDeviation();
	
	WholebodyContactVelConT(Solver* solver, string _name, WholebodyKey* _obj, int _iend, real_t _scale);
};

struct WholebodyContactVelConR : Constraint{
	WholebodyKey*  obj;
	int    iend;
	quat_t q0, qi;
	mat3_t R0;
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
