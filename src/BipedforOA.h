#pragma once

#include <DiMP2/Node.h>
#include <DiMP2/Range.h>
#include <DiMP2/App.h>

namespace DiMP2{;

class  BipedLIP;
struct LIPConP;
struct LIPConV;
struct ProhMoveConS;

/**
	linear inverted pendulum model
 */
class BipedLIPKey : public Keypoint{
public:
	int			phase;			    ///< walking phase (right support or left support)

	SVar*		pos_com[2];		    ///< position of COM
	SVar*		vel_com[2];		    ///< velocity of COM
	SVar*		pos_cop[2];			///< position of COP
	SVar*       pos_swg[2];         ///< 0歩目の遊脚始点　重心軌道には影響しないが遊脚軌道の決定のために使用
	SVar*		period;				///< stepping period 一歩あたりの時間

	LIPConP*	con_lip_pos[2];		///< LIP(Linear Inverted Pendulum) dynamics
	LIPConV*	con_lip_vel[2];
	
	RangeConS*	con_range_period;	 ///< range constraint on step period	
	DiffConS*   con_diff_cop[2][2];
								     ///< acceleration limit
								     ///< velocity limit
								     ///< range limit between COM and COP
								     ///< range limit between two consecutive COP positions
								     ///< range limit on stride
	ProhMoveConS*	con_move_cop;	 ///< (add) 動障害物


public:
	BipedLIP* GetNode(){ return (BipedLIP*)node; }
	
	virtual void AddVar (Solver* solver);
	virtual void AddCon (Solver* solver);
	virtual void Prepare();
	virtual void Draw   (DrawCanvas* canvas, DrawConfig* conf);

	BipedLIPKey();
};

typedef pair<BipedLIPKey*, BipedLIPKey*>	BipedLIPKeyPair;

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
		real_t  torsoMassRatio;			///< 脚に対する胴体の質量比
		real_t  legMassRatio;			///< 脚の重心位置
		int     swingProfile;
		real_t  swingHeight[2];         ///< 0: constant height at first half period, 1: lowest height before touch down
		real_t	stepPeriodMin;		    ///< minimum step period
		real_t	stepPeriodMax;		    ///< maximum stpe period
		vec2_t	supportMin[2];			///< admissible cop range
		vec2_t  supportMax[2];
		real_t  T;                      ///< 時定数

		Param();
	};
	
	/// 経由点
	struct Waypoint{
		int     k;
		vec2_t  pos_com;
		vec2_t  vel_com;
		vec2_t  pos_cop;
		vec2_t  pos_swg;
		bool    fix_pos_com;
		bool    fix_vel_com;
		bool    fix_pos_cop;
		bool    fix_pos_swg;

		Waypoint();
	};

	/// 軌道
	struct TrajPoint{
		float  t;
		Vec3f  pos_com;
		Vec3f  pos_torso;
		Vec3f  pos_cop;
		Vec3f  pos_swing;

		TrajPoint();
	};


	/// 動障害物
	struct MoveArea{
		vec2_t proh_move;
		vec2_t vel_move;
		real_t r;
	};

	void AddMoveArea	(vec2_t proh_move, real_t proh_r,vec2_t vel_move);
	vector<MoveArea>	moveareas;


	Param	          param;
	vector<int>	      phase;		///< walking phase at each step
	vector<Waypoint>  waypoints;
	vector<TrajPoint> trajectory;
	bool              trajReady;

	BipedLIPKey*	GetKeypoint(Tick* tick){ return TrajectoryNode::GetKeypoint<BipedLIPKey>(tick); }
	BipedLIPKeyPair	GetSegment (real_t t)  { return TrajectoryNode::GetSegment <BipedLIPKey>(t   ); }
	
	virtual Keypoint*	CreateKeypoint(){ return new BipedLIPKey(); }
	virtual void		Init();
	virtual void		Prepare();

	int    Phase         (real_t t);
	real_t period		 (real_t t);
	vec3_t PosCoM        (real_t t);
	vec3_t VelCoM        (real_t t);
	vec3_t AccCoM        (real_t t);
	vec3_t PosCoP        (real_t t);
	vec3_t PosSupportFoot(real_t t);
	vec3_t PosSwingFoot  (real_t t);
	vec3_t PosTorso      (const vec3_t& pcom, const vec3_t& psup, const vec3_t& pswg);

	void CalcTrajectory();
	void Draw          (DrawCanvas* canvas, DrawConfig* conf);
	void DrawSnapshot  (real_t time       , DrawCanvas* canvas, DrawConfig* conf);
	void Save          ();

	bool move_trajectory;
	float tm;	// 描写時間
	clock_t start;

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

// 動障害物についての評価
struct ProhMoveConS : Constraint{

	int _movesize;

	// 配列サイズは適当
	vec2_t  _ConIni[10];	// 動制約のスタート地点
	real_t	_r[10];			// 中心からの範囲
	real_t  _vx[10];		// 障害物x軸方向速度
	real_t  _vy[10];		// 障害物y軸方向速度
	real_t  t;			// ステップ開始時の時刻

	virtual void CalcCoef();
	virtual void CalcDeviation();
	virtual void Project(real_t& l, uint k);

	ProhMoveConS(Solver* solver, ID id, SVar* var0, SVar* var1, SVar* var2, real_t _scale);

};

}
