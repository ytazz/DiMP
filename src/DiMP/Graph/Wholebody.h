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
struct WholebodyTotalForceCon;
struct WholebodyTotalMomentCon;
struct WholebodyLimitCon;
struct WholebodyNormalForceCon;
struct WholebodyFrictionForceCon;
struct WholebodyMomentCon;
struct WholebodyComPosMatchCon;
struct WholebodyComVelMatchCon;
struct WholebodyMomentumMatchCon;

class WholebodyCallback;

struct WholebodyData{
	struct Link{
		vec3_t  pos_t;
		quat_t  pos_r;
		vec3_t  vel_t;
		vec3_t  vel_r;
		vec3_t  acc_t;
		vec3_t  acc_r;
		vec3_t  force_t, force_t_par;
		vec3_t  force_r, force_r_par;
		bool    force_ready;
	};

	struct End{
		vec3_t  pos_t_des, pos_t_ini, pos_tc;
		quat_t  pos_r_des, pos_r_ini, pos_rc;
		vec3_t  vel_t_des, vel_t_ini;
		vec3_t  vel_r_des, vel_r_ini;
		vec3_t  acc_tc  , acc_tc_des, acc_tc_ini;
		vec3_t  acc_rc  , acc_rc_des, acc_rc_ini;
		vec3_t  force_tc, force_tc_des, force_tc_ini;
		vec3_t  force_rc, force_rc_des, force_rc_ini;

		vec3_t  rlc, rcc;
		mat3_t  Rc, rlc_cross_Rc, rcc_cross_Rc;
	
		int             state;       ///< contact state
		array<bool, 6>  v_or_f;      ///< velocity/force complimentarity
		real_t          mu;          ///< friction
		vec2_t          cop_min;
		vec2_t          cop_max;
	};

	vector<Link>  links;
	vector<End>   ends;

	vec3_t com_pos, com_pos_des;
	vec3_t com_vel, com_vel_des;
	vec3_t mom, mom_des;
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
		V3Var*  var_pos_t;    ///< position        
		QVar*   var_pos_r;    ///< orientation
		V3Var*  var_vel_t;
		V3Var*  var_vel_r;
		SVar*   var_compl[6];
		
		FixConV3*  con_des_pos_t  ;
		FixConQ *  con_des_pos_r  ;
		FixConV3*  con_des_vel_t  ;
		FixConV3*  con_des_vel_r  ;
		FixConS *  con_des_compl[6];

		WholebodyPosConT*           con_pos_t;
		WholebodyPosConR*           con_pos_r;
		WholebodyVelConT*           con_vel_t;
		WholebodyVelConR*           con_vel_r;

		WholebodyNormalForceCon*    con_force_normal;
		WholebodyFrictionForceCon*  con_force_friction[2][2];
		WholebodyMomentCon*         con_moment[2][2];
	};

	V3Var*  var_com_pos;
	V3Var*  var_com_vel;
	V3Var*  var_mom;

	FixConV3*  con_des_com_pos;
	FixConV3*  con_des_com_vel;
	FixConV3*  con_des_mom;
		
	vector<WholebodyLimitCon*>  con_limit;

	WholebodyComPosCon*         con_com_pos;	
	WholebodyTotalForceCon*     con_total_force;
	WholebodyTotalMomentCon*    con_total_moment;

	WholebodyComPosMatchCon*    con_com_pos_match;	
	WholebodyComVelMatchCon*    con_com_vel_match;
	WholebodyMomentumMatchCon*  con_mom_match;
	
	WholebodyData  data, data_plus, data_minus, data_tmp;

	vector<mat3_t>  J_pcom_pe, J_pcom_qe, J_pcom_ve, J_pcom_we;
	vector<mat3_t>  J_vcom_pe, J_vcom_qe, J_vcom_ve, J_vcom_we;
	vector<mat3_t>  J_L_pe   , J_L_qe   , J_L_ve   , J_L_we   ;
	vector<vmat_t>  J_q_pe   , J_q_qe   , J_q_ve   , J_q_we   ;
	vector<vmat_t>  J_e_pe   , J_e_qe   , J_e_ve   , J_e_we   ;

	vector<End>    ends;
	
public:
	void CalcIK(WholebodyData& data, bool calc_acc, bool calc_force);
	
    virtual void AddVar(Solver* solver);
	virtual void AddCon(Solver* solver);
	virtual void Prepare    ();
	virtual void PrepareStep();
	virtual void Finish     ();
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
		vec3_t       axis;         ///< joint axis
	
		Link(real_t _mass = 0.0, int _iparent = -1, int _ijoint = -1, vec3_t _axis = vec3_t());
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
		int           ibase;
		int           iend;
		vector<int>   ilink;
		vector<int>   ilimit;
		
		Chain(){}
		Chain(int _ibase, int _iend, const vector<int>& _ilink, const vector<int>& _ilimit);
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
		vector<Link>  links;
		
		Snapshot();
	};

    Param	            param;
    vector<Link>        links;
	vector<Joint>       joints;
	vector<Limit>       limits;
	vector<int>         idOrder;
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

	void ComState (real_t t, vec3_t& pos, vec3_t& vel);
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
	virtual void   Setup (real_t t, WholebodyData& data) = 0;
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
	vec3_t pc0, vc0, pc1, pc_rhs;

	void Prepare();

	virtual void  CalcCoef();
	virtual void  CalcDeviation();
	virtual void  CalcLhs();
	
	WholebodyComPosCon(Solver* solver, string _name, WholebodyKey* _obj, real_t _scale);
};

struct WholebodyTotalForceCon : WholebodyCon{
	//real_t  m;
	vec3_t  fsum;
	vec3_t  vc_rhs;

	void Prepare();

	virtual void  CalcCoef();
	virtual void  CalcDeviation();
	virtual void  CalcLhs();
	
	WholebodyTotalForceCon(Solver* solver, string _name, WholebodyKey* _obj, real_t _scale);
};

struct WholebodyTotalMomentCon : WholebodyCon{
	vec3_t          msum;
	vec3_t          L_rhs;

	void Prepare();

	virtual void  CalcCoef();
	virtual void  CalcDeviation();
	virtual void  CalcLhs();
	
	WholebodyTotalMomentCon(Solver* solver, string _name, WholebodyKey* _obj, real_t _scale);
};

struct WholebodyLimitCon : Constraint{
	WholebodyKey*  obj;
	int  idx;
	
	void Prepare();

	virtual void  CalcCoef();
	virtual void  CalcDeviation();
	
	WholebodyLimitCon(Solver* solver, string _name, WholebodyKey* _obj, int _idx, int _type, real_t _scale);
};

struct WholebodyNormalForceCon : Constraint{
	WholebodyKey*  obj;
	int    iend;
	bool   on;
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
	bool   on;
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
	bool   on;
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

struct WholebodyComPosMatchCon : Constraint{
	WholebodyKey*  obj;
	
	void Prepare();

	virtual void  CalcCoef();
	virtual void  CalcDeviation();
	
	WholebodyComPosMatchCon(Solver* solver, string _name, WholebodyKey* _obj, real_t _scale);
};

struct WholebodyComVelMatchCon : Constraint{
	WholebodyKey*  obj;
	
	void Prepare();

	virtual void  CalcCoef();
	virtual void  CalcDeviation();
	
	WholebodyComVelMatchCon(Solver* solver, string _name, WholebodyKey* _obj, real_t _scale);
};

struct WholebodyMomentumMatchCon : Constraint{
	WholebodyKey*  obj;
	
	void Prepare();

	virtual void  CalcCoef();
	virtual void  CalcDeviation();
	
	WholebodyMomentumMatchCon(Solver* solver, string _name, WholebodyKey* _obj, real_t _scale);
};

}
