#pragma once

#include <DiMP2/Node.h>
#include <DiMP2/Range.h>

namespace DiMP2{;

class  BipedLIP;
struct CoMConTP;
struct CoMConTV;
struct CoMConR;
struct CoPConT;
struct CoPConR;

/**
	linear inverted pendulum model
 */
class BipedLIPKey : public Keypoint{
public:
	int			phase;			  ///< walking phase (right support or left support)

	SVar*		com_pos_t[2];     ///< position of CoM
	SVar*		com_vel_t[2];     ///< velocity of CoM
	SVar*       com_pos_r;        ///< orientation of CoM
	SVar*       com_vel_r;        ///< angular velocity of CoM
	SVar*		cop_pos_t[2];     ///< position of CoP
	SVar*       cop_pos_r;        ///< orienation of CoP (foot)
	SVar*       swg_pos_t[2];     ///< 0•à–Ú‚Ì—V‹rŽn“_@dS‹O“¹‚É‚Í‰e‹¿‚µ‚È‚¢‚ª—V‹r‹O“¹‚ÌŒˆ’è‚Ì‚½‚ß‚ÉŽg—p
	SVar*       swg_pos_r;        ///<
	SVar*		period;           ///< stepping period ˆê•à‚ ‚½‚è‚ÌŽžŠÔ

	CoMConTP *  con_com_tp[2]   ;   ///< LIP(Linear Inverted Pendulum) dynamics
	CoMConTV *  con_com_tv[2]   ;
	CoMConR  *  con_com_r [2]   ;   ///< range constraint on angular acceleration at beginning and end of step
	CoPConT  *  con_cop_t [2][2];   ///< range constraint on relative position between CoM and support point
	CoPConR  *  con_cop_r [2]   ;   ///< range constraint on turning angle of CoM
	RangeConS*  con_period      ;   ///< range constraint on step period	
	
public:
	virtual void AddVar (Solver* solver);
	virtual void AddCon (Solver* solver);
	virtual void Prepare();
	virtual void Draw   (DrawCanvas* canvas, DrawConfig* conf);

	BipedLIPKey();
};

class BipedLIP : public TrajectoryNode{
public:
	/// walking phase
	struct Phase{
		enum{
			Right,
			Left ,
		};
	};
	/// swing foot trajectory type
	struct SwingProfile{
		enum{
			Wedge,        ///< 
			Cycloid,      ///< 
		};
	};

	struct Param{
		real_t	gravity;				///< gravity (positive)
		real_t  heightCoM;
		real_t  torsoMassRatio;
		real_t  legMassRatio;
		int     swingProfile;
		real_t  swingHeight[2];         ///< 0: maximum swing height
		                                ///< 1: lowest height before touch down. for wedge only
		real_t	stepPeriodMin;		    ///< minimum step period
		real_t	stepPeriodMax;		    ///< maximum stpe period
		vec2_t	supportPosMin[2];       ///< admissible cop range ralative to com
		vec2_t  supportPosMax[2];      
		real_t  supportOriMin[2];
		real_t  supportOriMax[2];
		real_t  T;                      ///< time constant of LIP Žž’è”
		real_t  angAccMax;              ///< maximum admissible angular acceleration
		real_t  turnMax;                ///< maximum admissible turning angle in single step

		Param();
	};
	
	/// Œo—R“_
	struct Waypoint{
		int     k;
		vec2_t  com_pos_t;
		vec2_t  com_vel_t;
		real_t  com_pos_r;
		real_t  com_vel_r;
		vec2_t  cop_pos_t;
		real_t  cop_pos_r;
		vec2_t  swg_pos_t;
		real_t  swg_pos_r;
		bool    fix_com_pos_t;
		bool    fix_com_vel_t;
		bool	fix_com_pos_r;
		bool	fix_com_vel_r;
		bool    fix_cop_pos_t;
		bool    fix_cop_pos_r;
		bool    fix_swg_pos_t;
		bool    fix_swg_pos_r;
		
		Waypoint();
	};

	/// ‹O“¹
	struct TrajPoint{
		real_t  t;
		vec3_t  pos_com;
		real_t  ori_com;
		vec3_t  pos_torso;
		vec3_t  pos_sup;
		real_t  ori_sup;
		vec3_t  pos_swg;
		real_t  ori_swg;
		
		TrajPoint();
	};

	Param	            param;
	vector<int>	        phase;		///< walking phase at each step
	vector<Waypoint>    waypoints;
	vector<TrajPoint>   trajectory;
	bool                trajReady;

	virtual Keypoint*	CreateKeypoint(){ return new BipedLIPKey(); }
	virtual void		Init();
	virtual void		Prepare();

	int    Phase     (real_t t);
	vec3_t CoMPos    (real_t t);
	vec3_t CoMVel    (real_t t);
	vec3_t CoMAcc    (real_t t);
	real_t CoMOri    (real_t t);
	real_t CoMAngVel (real_t t);
	real_t CoMAngAcc (real_t t);
	vec3_t SupFootPos(real_t t);
	real_t SupFootOri(real_t t);
	vec3_t SwgFootPos(real_t t);
	real_t SwgFootOri(real_t t);
	vec3_t TorsoPos  (const vec3_t& pcom, const vec3_t& psup, const vec3_t& pswg);
	
	void CalcSimple    ();
	void CalcTrajectory();
	void Draw          (DrawCanvas* canvas, DrawConfig* conf);
	void DrawSnapshot  (real_t time       , DrawCanvas* canvas, DrawConfig* conf);
	void Save          ();

public:
	BipedLIP(Graph* g, const string& n);
	virtual ~BipedLIP();
};

struct CoMCon : Constraint{
	BipedLIPKey* obj[2];
	uint         dir;
	real_t	T, t, ch, sh, p0, v0, c, p1, v1;

	void Prepare();

	CoMCon(Solver* solver, int _tag, const string& _name, BipedLIPKey* _obj, uint _dir, real_t _scale);
};

/// CoM position constraint based on LIP model
struct CoMConTP : CoMCon{
	virtual void CalcCoef();
	virtual void CalcDeviation();

	CoMConTP(Solver* solver, const string& _name, BipedLIPKey* _obj, uint _dir, real_t _scale);
};

/// CoM velocity constraint based on LIP model
struct CoMConTV : CoMCon{
	virtual void CalcCoef();
	virtual void CalcDeviation();

	CoMConTV(Solver* solver, const string& _name, BipedLIPKey* _obj, uint _dir, real_t _scale);
};

/// angular acceleration limit
struct CoMConR : Constraint{
	BipedLIPKey* obj[2];
	uint         idx;
	real_t       _min, _max;
	bool	     on_lower, on_upper;
	real_t       q0, w0, q1, w1, tau, tau2, tau3;
	
	virtual void CalcCoef     ();
	virtual void CalcDeviation();
	virtual void Project      (real_t& l, uint k);

	CoMConR(Solver* solver, const string& _name, BipedLIPKey* _obj, uint _idx, real_t _scale);
};

/// range limit of CoP relative to CoM
struct CoPConT : Constraint{
	BipedLIPKey* obj[2];
	uint         idx;
	uint         dir;
	real_t       _min, _max;
	bool	     on_lower, on_upper;
	real_t       cx, cy, cr, px, py, _cos, _sin;

	virtual void CalcCoef     ();
	virtual void CalcDeviation();
	virtual void Project      (real_t& l, uint k);

	CoPConT(Solver* solver, const string& _name, BipedLIPKey* _obj, uint _idx, uint _dir, real_t _scale);
};
struct CoPConR : Constraint{
	BipedLIPKey* obj[2];
	uint         idx;
	real_t       _min, _max;
	bool	     on_lower, on_upper;
	real_t       cr, pr;

	virtual void CalcCoef     ();
	virtual void CalcDeviation();
	virtual void Project      (real_t& l, uint k);

	CoPConR(Solver* solver, const string& _name, BipedLIPKey* _obj, uint _idx, real_t _scale);
};

}
