#pragma once

#include <DiMP2/Node.h>
#include <DiMP2/Range.h>

namespace DiMP2{;

class  BipedLIP;
struct LIPConP;
struct LIPConV;
struct RangeConAccL;
struct RangeConAccR;
struct RangeConAccf;
struct FixConPCOP;
struct FixConPtoA;
struct RangeConCOP;
struct DiffConAng;

/**
	linear inverted pendulum model
 */
class BipedLIPKey : public Keypoint{
public:
	int			phase;			    ///< walking phase
								    
	SVar*		pos_com[2];		    ///< position of COM
	SVar*		vel_com[2];		    ///< velocity of COM
	SVar*		pos_cop[2];			///< position of COP
	SVar*       pos_swg[2];         ///< 0歩目の遊脚始点　重心軌道には影響しないが遊脚軌道の決定のために使用
	SVar*		ang_com;		    ///< angle of COM            (add)
	SVar*		angvel_com;		    ///< angular velocity of COM (add)
	SVar*		period;				///< stepping period 一歩あたりの時間

	LIPConP*	con_lip_pos[2];		///< LIP(Linear Inverted Pendulum) dynamics
	LIPConV*	con_lip_vel[2];
	
	FixConS*	con_fix_pos[2];		///< terminal constraint on position
	FixConS*	con_fix_vel[2];		///< terminal constraint on velocity
	FixConS*	con_fix_ang;		///< terminal constraint on angle			 (add)
	FixConS*	con_fix_angvel;		///< terminal constraint on angular velocity (add)

	FixConPCOP* con_fix_pos_copL[2];
	FixConPtoA* con_pos_to_angL [2];
	FixConPCOP* con_fix_pos_copR[2];
	FixConPtoA* con_pos_to_angR [2];

	RangeConS*	con_range_period;	///< range constraint on step period
	RangeConAccL*  con_range_angaccL; ///< range constraint on angular acceleration at begin and end of step
	RangeConAccR*  con_range_angaccR;
	RangeConAccL*  con_range_angaccLf;
	RangeConAccR*  con_range_angaccRf;
	RangeConCOP*    con_diff_cop[2][2];
								    ///< acceleration limit
								    ///< velocity limit
								    ///< range limit between COM and COP
								    ///< range limit between two consecutive COP positions
								    ///< range limit on stride
	DiffConAng* con_range_angle;     ///< range constraint on angle variation quantity of body during one step

public:
	BipedLIP* GetNode(){ return (BipedLIP*)node; }
	
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
			Left,
			Right,
		};
	};

	struct Param{
		real_t	gravity;				///< gravity (positive)
		real_t  massTorso;              ///< mass of torso
		real_t  massLeg;                ///< mass of leg
		real_t  heightCoM;
		real_t  legCoMRatio;
		real_t  swingHeight[2];
		real_t	stepPeriodMin;		    ///< minimum step period
		real_t	stepPeriodMax;		    ///< maximum stpe period
		vec2_t	supportMin[2];			///< admissible cop range
		vec2_t  supportMax[2];
		real_t  T;                      ///< 時定数
		real_t  AngAccMin;
		real_t  AngAccMax;
	    //real_t  tmargin;
		real_t  AngleMax;

		Param();
	};
	/// 経由点
	struct Waypoint{
		int     k;
		vec2_t  pos_com;
		vec2_t  vel_com;
		vec2_t  pos_cop;
		vec2_t  pos_swg;
		real_t  ang_com;
		real_t  angvel_com;
		bool    fix_pos_com;
		bool    fix_vel_com;
		bool    fix_pos_cop;
		bool    fix_pos_swg;
		bool	fix_ang_com;
		bool	fix_angvel_com;

		Waypoint();
	};
	/// 軌道
	struct TrajPoint{
		float  t;
		Vec3f  pos_com;
		Vec3f  pos_torso;
		Vec3f  pos_cop;
		Vec3f  pos_swing;
		Vec3f ang_com;

		TrajPoint();
	};

	Param	           param;
	bool			   FallAvoidance;
	vector<int>			   phase;
	bool			   lastphase;
	vector<Waypoint>   waypoints;
	vector<TrajPoint>  trajectory;
	bool               trajReady;

	BipedLIPKey*	GetKeypoint(Tick* tick){ return TrajectoryNode::GetKeypoint<BipedLIPKey>(tick); }
	BipedLIPKeyPair	GetSegment (real_t t)  { return TrajectoryNode::GetSegment <BipedLIPKey>(t   ); }
	
	virtual Keypoint*	CreateKeypoint(){ return new BipedLIPKey(); }
	virtual void		Init();
	virtual void		Prepare();

	int	   Phase		 (real_t t);
	real_t Angle0        (real_t t);//k-1ステップ前の胴体角度
	real_t Angle1        (real_t t);//kステップ目の胴体角度
	real_t Angle2        (real_t t);//k+1ステップ目の胴体角度
	real_t Period		 (real_t t);//1ステップに要する時間
	vec3_t PosCoM        (real_t t);
	vec3_t VelCoM        (real_t t);
	vec3_t AccCoM        (real_t t);
	vec3_t PosCoP        (real_t t);
	vec3_t PosSupportFoot(real_t t);
	vec3_t PosSwingFoot  (real_t t);
	vec3_t PosTorso      (const vec3_t& pcom, const vec3_t& psup, const vec3_t& pswg);
	vec3_t AngCoM		 (real_t t);//add
	vec3_t AngVelCoM	 (real_t t);//add
	vec3_t AngAccCoM	 (real_t t);//add


	void CalcTrajectory();
	void Draw          (DrawCanvas* canvas, DrawConfig* conf);
	void DrawSnapshot  (real_t time       , DrawCanvas* canvas, DrawConfig* conf);
	void Save          ();
	bool MotionData;

	//Vec3f pos_right, pos_left;
	
	bool move_trajectory;
	float tm;	// 描写時間
	clock_t start,test;

public:
	BipedLIP(Graph* g, string n);
	virtual ~BipedLIP(){}
};

struct LIPCon : Constraint{
	BipedLIPKey*	obj[2];
	uint			idx;
	real_t	T, t, ch, sh, p0, v0, c, p1, v1;

	void Prepare();

	LIPCon(Solver* solver, string _name, BipedLIPKey* _obj, uint _idx, real_t _scale);
};

struct LIPConP : LIPCon{
	virtual void CalcCoef();
	virtual void CalcDeviation();

	LIPConP(Solver* solver, string _name, BipedLIPKey* _obj, uint _idx, real_t _scale);
};

struct LIPConV : LIPCon{
	virtual void CalcCoef();
	virtual void CalcDeviation();

	LIPConV(Solver* solver, string _name, BipedLIPKey* _obj, uint _idx, real_t _scale);
};

//角加速度拘束条件(ステップ開始時および終了時)
struct RangeConAccL :Constraint{
	real_t	 tm,pr0,vr0,pr1,vr1, _min, _max;
	bool	on_lower, on_upper;

	virtual void Prepare();
	virtual void CalcCoef();
	virtual void CalcDeviation();
	virtual void Project(real_t& l, uint k);

	RangeConAccL(Solver* solver, ID id, SVar* var0, SVar* var1, SVar* var2, SVar* var3, SVar* var4, real_t _scale);
};

struct RangeConAccR : Constraint{
	real_t	 tm,pr0,vr0,pr1,vr1, _min, _max;
	bool	on_lower, on_upper;

	virtual void Prepare();
	virtual void CalcCoef();
	virtual void CalcDeviation();
	virtual void Project(real_t& l, uint k);

	RangeConAccR(Solver* solver, ID id, SVar* var0, SVar* var1, SVar* var2, SVar* var3, SVar* var4, real_t _scale);
};

////接地点探索範囲
struct RangeConCOP : Constraint{
	real_t	 cop_x, cop_y, com_x, com_y, com_ang, _min, _max;
	bool	on_lower, on_upper;
	bool	xaxis;
	bool	yaxis;

	virtual void Prepare();
	virtual void CalcCoef();
	virtual void CalcDeviation();
	virtual void Project(real_t& l, uint k);
	
	RangeConCOP(Solver* solver, ID id, SVar* var0, SVar* var1, SVar* var2, SVar* var3, SVar* var4, uint _idx, real_t _sacle);
};

struct FixCon : Constraint{
	BipedLIPKey*	obj[2];
	uint			idx;
	BipedLIP* biped;
	Graph* graph;
	real_t  c0, c1, p1, ang1;
	bool   phase;
	
	void Prepare();

	FixCon(Solver* solver, string _name, BipedLIPKey* _obj, uint _idx, uint _lastphase, real_t _scale);
};

struct FixConPCOP : FixCon{
	virtual void CalcCoef();
	virtual void CalcDeviation();

	FixConPCOP(Solver* solver, string _name, BipedLIPKey* _obj, uint _idx, uint _lastphase, real_t _scale);
};

struct FixConPtoA : FixCon{
	virtual void CalcCoef();
	virtual void CalcDeviation();

	FixConPtoA(Solver* solver, string _name, BipedLIPKey* _obj, uint _idx, uint _lastphase, real_t _scale);
};

struct DiffConAng : Constraint{
	real_t _min, _max;
	bool   on_lower, on_upper;

	virtual void CalcDeviation();
	virtual void Project(real_t& l, uint k);

	DiffConAng(Solver* solver, ID id, SVar* var0, SVar* var1, real_t sacale);
};
}
		

