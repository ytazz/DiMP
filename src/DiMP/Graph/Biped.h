#pragma once

#include <DiMP/Graph/Node.h>

namespace DiMP {;

	class  BipedLIP;

	struct BipedLipPosCon;
	struct BipedLipVelCon;
	struct BipedLipCopCon;
	struct BipedLipCmpCon;
	struct BipedLipMomCon;
	struct BipedComConP;
	struct BipedComConV;
	struct BipedFootRangeConT;
	struct BipedFootRangeConR;
	struct BipedCopRangeCon;
	struct BipedCmpRangeCon;
	struct BipedAccRangeCon;
	struct BipedMomRangeCon;
	struct BipedTimeCon;

	/**
	linear inverted pendulum model
	*/
	class BipedLIPKey : public Keypoint {
	public:
		V2Var*      var_torso_pos_t;    ///< position         of torso
		SVar*       var_torso_pos_r;    ///< orientation      of torso
		V2Var*      var_torso_vel_t;    ///< velocity         of torso

		V2Var*		var_com_pos;        ///< position of CoM
		V2Var*		var_com_vel;        ///< velocity of CoM
		V2Var*      var_mom;            ///< angular momentum around CoM; it is actually normalized momentum L/mg

		V2Var*		var_cop_pos;        ///< position of CoP
		V2Var*      var_cop_vel;        ///< velocity of CoP

		V2Var*      var_cmp_pos;        ///< CMP offset
		V2Var*      var_cmp_vel;        ///< CMP offset time derivative

		V2Var*      var_foot_pos_t[2];  ///< position    of foot, R/L
		SVar*       var_foot_pos_r[2];  ///< orientation of foot, R/L

		SVar*       var_time;           ///< time
		SVar*		var_duration;       ///< duration

		BipedLipPosCon*    con_lip_pos;     ///< LIP position constraint
		BipedLipVelCon*    con_lip_vel;     ///< LIP velocity constraint
		BipedLipCopCon*    con_lip_cop;     ///< LIP cop constraint
		BipedLipCmpCon*    con_lip_cmp;     ///< LIP cmp constraint
		BipedLipMomCon*    con_lip_mom;     ///< LIP angular momentum constraint

		BipedComConP*    con_com_pos;     ///< torso, feet, and com position constraint based on 3-mass model
		BipedComConV*    con_com_vel;     ///< torso, feet, and com velocity constraint based on 3-mass model

		BipedFootRangeConT*   con_foot_range_t[2][2];   ///< range constraint on foot position relative to torso, [r|l][x|y]
		BipedFootRangeConR*   con_foot_range_r[2];      ///< range constraint on foot orientation relative to torso, [r|l]

		BipedCopRangeCon*  con_cop_range[2];   ///< range constraint on CoP relative to support foot, [x|y]
		BipedCmpRangeCon*  con_cmp_range[2];   ///< range constraint on CMP
		BipedAccRangeCon*  con_acc_range[2];   ///< range constraint on CoM acceleration
		BipedMomRangeCon*  con_mom_range[2];

		BipedTimeCon*    con_time;			   ///< relates step duration and cumulative time
		RangeConS*       con_duration_range;   ///< range constraint on step period

		MatchConV2*      con_foot_match_t[2];
		MatchConS *      con_foot_match_r[2];

	public:
		virtual void AddVar(Solver* solver);
		virtual void AddCon(Solver* solver);
		virtual void Prepare();
		virtual void Finish ();
		virtual void Draw(Render::Canvas* canvas, Render::Config* conf);

		BipedLIPKey();
	};

	class BipedLIP : public TrajectoryNode {
	public:
		/// walking phase
		struct Phase {
			enum {
				R ,  //< right support
				L ,  //< left support
				RL,  //< double support: transition from R to L
				LR,  //< double support: trabsition from L to R
				D ,  //< double support for starting and stopping
				Num
			};
		};
		/// swing foot trajectory type
		struct SwingProfile {
			enum {
				Wedge,        ///< flat-landing with wedge-like swing profile
				Cycloid,      ///< flat-landing with cycloid swing profile
				HeelToe,      ///< heel-to-toe walking
			};
		};
		/// contact state
		struct ContactState{
			enum{
				Float,    //< in floating state
				Surface,  //< in surface contact
				Heel,     //< in line contact on heel
				Toe,      //< in line contact on toe
			};
		};

		struct Param {
			real_t	gravity;				///< gravity (positive)
			real_t  T;                      ///< time constant of LIP
			real_t  heightCoM;
			real_t  heightlow;
			real_t  heighthigh;
			real_t  heightmiddle;
			real_t  torsoMass;
			real_t  footMass;
			int     swingProfile;
			real_t  swingHeight[2];             ///< 0: maximum swing height
												///< 1: lowest height before touch down. for wedge only
			real_t	durationMin[Phase::Num];	///< minimum duration of each phase
			real_t	durationMax[Phase::Num];	///< maximum duration of each phase
			vec2_t	footPosMin[2];              ///< admissible range of foot relative to torso
			vec2_t  footPosMax[2];
			real_t  footOriMin[2];
			real_t  footOriMax[2];
			vec2_t  copMin;                  ///< admissible range of CoP relative to foot
			vec2_t  copMax;
			vec2_t  cmpMin;                  ///< admissible range of CoP relative to foot
			vec2_t  cmpMax;
			vec2_t  accMin;                  ///< admissible range of CoM acceleration
			vec2_t  accMax;
			vec2_t  momMin;                  ///< admissible range of angular momentum
			vec2_t  momMax;
			
			real_t  ankleToToe ;
			real_t  ankleToHeel;
			real_t  toeRadius  ;
			real_t  heelRadius ;

			Param();
		};

		struct Waypoint {
			int     k;
			real_t  time;
			vec2_t  torso_pos_t;
			vec2_t  torso_vel_t;
			real_t  torso_pos_r;
			vec2_t  foot_pos_t[2];
			real_t  foot_pos_r[2];
			vec2_t  cop_pos;

			bool    fix_time;
			bool    fix_torso_pos_t;
			bool    fix_torso_pos_r;
			bool    fix_torso_vel_t;
			bool    fix_foot_pos_t[2];
			bool    fix_foot_pos_r[2];
			bool    fix_cop_pos;
			bool    fix_cmp_pos;
			bool    fix_mom;

			Waypoint();
		};

		struct Snapshot{
			real_t  t;
			vec3_t  com_pos;
			vec3_t  torso_pos_t;
			real_t  torso_pos_r;
			vec3_t  foot_pos_t[2];
			quat_t  foot_pos_r[2];
			vec3_t  cop_pos;
			vec3_t  cmp_pos;

			Snapshot();
		};

		Param	            param;
		vector<int>	        phase;		///< walking phase at each step
		vector<Waypoint>    waypoints;
		Snapshot            snapshot;
		vector<Snapshot>    trajectory;
		bool                trajReady;

		virtual Keypoint*	CreateKeypoint() { return new BipedLIPKey(); }
		virtual void		Init();
		virtual void		Prepare();
		virtual void        Finish ();
		virtual void        CreateSnapshot(real_t time);
		virtual void        DrawSnapshot  (Render::Canvas* canvas, Render::Config* conf);
		virtual void        Draw          (Render::Canvas* canvas, Render::Config* conf);
		
		int    Phase      (real_t t);
		vec3_t ComPos     (real_t t);
		vec3_t ComVel     (real_t t);
		vec3_t ComAcc     (real_t t);
		real_t TorsoOri   (real_t t);
		real_t TorsoAngVel(real_t t);
		real_t TorsoAngAcc(real_t t);
		void   FootPose   (real_t t, int side, pose_t& pose, vec3_t& vel, vec3_t& angvel, vec3_t& acc, vec3_t& angacc, int& contact);
		vec3_t CopPos     (real_t t);
		vec3_t CmpPos     (real_t t);
		vec3_t Momentum   (real_t t);
		
		vec3_t TorsoPos(const vec3_t& pcom, const vec3_t& psup, const vec3_t& pswg);
		vec3_t TorsoVel(const vec3_t& vcom, const vec3_t& vsup, const vec3_t& vswg);
		vec3_t TorsoAcc(const vec3_t& acom, const vec3_t& asup, const vec3_t& aswg);

		void CreateSnapshot(real_t t, Snapshot& s);
		void CalcTrajectory();
		void Save (const char* filename);
		void Print();

	public:
		BipedLIP(Graph* g, string n);
		virtual ~BipedLIP();
	};

	struct BipedLipCon : Constraint {
		BipedLIPKey* obj[2];

		real_t T;
		mat2_t H;
		real_t tau;
		real_t C, S;
		vec2_t p0, p1;
		vec2_t v0, v1;
		vec2_t c0, c1;
		vec2_t cm0, cm1;
		vec2_t L0, L1;
		vec2_t cmv0;
		vec2_t cv0;
		
		void Prepare();

		BipedLipCon(Solver* solver, int _tag, string _name, BipedLIPKey* _obj, real_t _scale);
	};

	/// CoM position constraint based on LIP model
	struct BipedLipPosCon : BipedLipCon {
		virtual void  CalcCoef();
		virtual void  CalcDeviation();
		virtual void  CalcLhs();
		
		BipedLipPosCon(Solver* solver, string _name, BipedLIPKey* _obj, real_t _scale);
	};

	/// CoM velocity constraint based on LIP model
	struct BipedLipVelCon : BipedLipCon {
		virtual void CalcCoef();
		virtual void CalcDeviation();
		virtual void CalcLhs();
		
		BipedLipVelCon(Solver* solver, string _name, BipedLIPKey* _obj, real_t _scale);
	};

	/// CoP constraint based on LIP model
	struct BipedLipCopCon : BipedLipCon {
		virtual void CalcCoef();
		virtual void CalcDeviation();
		virtual void CalcLhs();
		
		BipedLipCopCon(Solver* solver, string _name, BipedLIPKey* _obj, real_t _scale);
	};

	/// CMP constraint based on LIP model
	struct BipedLipCmpCon : BipedLipCon {
		virtual void CalcCoef();
		virtual void CalcDeviation();
		virtual void CalcLhs();
		
		BipedLipCmpCon(Solver* solver, string _name, BipedLIPKey* _obj, real_t _scale);
	};

	struct BipedLipMomCon : BipedLipCon {
		virtual void  CalcCoef();
		virtual void  CalcDeviation();
		virtual void  CalcLhs();

		BipedLipMomCon(Solver* solver, string _name, BipedLIPKey* _obj, real_t _scale);
	};

	/// CoM position constraint (com position is constrained to weighted average of torso and feet position)
	struct BipedComConP : Constraint {
		BipedLIPKey* obj;

		virtual void CalcCoef();

		BipedComConP(Solver* solver, string _name, BipedLIPKey* _obj, real_t _scale);
	};

	/// CoM velocity constraint (com velocity is constrained to weighted average of torso and feet velocity)
	struct BipedComConV : Constraint {
		BipedLIPKey* obj;

		virtual void CalcCoef();

		BipedComConV(Solver* solver, string _name, BipedLIPKey* _obj, real_t _scale);
	};

	/// range limit of support foot relative to torso
	struct BipedFootRangeConT : Constraint {
		BipedLIPKey* obj;
		uint         side;
		vec2_t       dir, dir_abs;
		vec2_t       pf, pt;
		real_t       thetat;
		mat2_t       R, dR;
		real_t       _min, _max;
		bool         on_lower, on_upper;

		virtual void CalcCoef();
		virtual void CalcDeviation();
		virtual void Project(real_t& l, uint k);

		BipedFootRangeConT(Solver* solver, string _name, BipedLIPKey* _obj, uint _side, vec2_t _dir, real_t _scale);
	};

	struct BipedFootRangeConR : Constraint {
		BipedLIPKey* obj;
		uint         side;
		real_t       thetaf, thetat;
		real_t       _min, _max;
		bool	     on_lower, on_upper;

		virtual void CalcCoef();
		virtual void CalcDeviation();
		virtual void Project(real_t& l, uint k);

		BipedFootRangeConR(Solver* solver, string _name, BipedLIPKey* _obj, uint _side, real_t _scale);
	};

	/// CoP range constraint
	struct BipedCopRangeCon : Constraint {
		BipedLIPKey* obj;
		uint         side;
		vec2_t       dir, dir_abs;
		vec2_t       pc, pf;
		real_t       thetaf;
		mat2_t       R, dR;
		real_t       _min, _max;
		bool         on_lower, on_upper;

		virtual void CalcCoef();
		virtual void CalcDeviation();
		virtual void Project(real_t& l, uint k);

		BipedCopRangeCon(Solver* solver, string _name, BipedLIPKey* _obj, uint _side, vec2_t _dir, real_t _scale);
	};

	/// CMP range constraint
	struct BipedCmpRangeCon : Constraint {
		BipedLIPKey* obj;
		vec2_t       dir, dir_abs;
		vec2_t       pc;
		real_t       thetat;
		mat2_t       R, dR;
		real_t       _min, _max;
		bool         on_lower, on_upper;

		virtual void CalcCoef();
		virtual void CalcDeviation();
		virtual void Project(real_t& l, uint k);

		BipedCmpRangeCon(Solver* solver, string _name, BipedLIPKey* _obj, vec2_t _dir, real_t _scale);
	};

	/// CoM acceleration range constraint
	struct BipedAccRangeCon : Constraint {
		BipedLIPKey* obj;
		vec2_t       dir, dir_abs;
		real_t       T;
		vec2_t       a;
		real_t       thetat;
		mat2_t       R, dR;
		real_t       _min, _max;
		bool         on_lower, on_upper;

		virtual void CalcCoef();
		virtual void CalcDeviation();
		virtual void Project(real_t& l, uint k);

		BipedAccRangeCon(Solver* solver, string _name, BipedLIPKey* _obj, vec2_t _dir, real_t _scale);
	};

	/// angular momentum range constraint
	struct BipedMomRangeCon : Constraint {
		BipedLIPKey* obj;
		vec2_t       dir, dir_abs;
		vec2_t       m;
		real_t       thetat;
		mat2_t       R, dR;
		real_t       _min, _max;
		bool         on_lower, on_upper;

		virtual void CalcCoef();
		virtual void CalcDeviation();
		virtual void Project(real_t& l, uint k);

		BipedMomRangeCon(Solver* solver, string _name, BipedLIPKey* _obj, vec2_t _dir, real_t _scale);
	};

	/// time constraint
	struct BipedTimeCon : Constraint {
		BipedLIPKey* obj[2];

		virtual void CalcCoef();
		virtual void CalcLhs ();

		BipedTimeCon(Solver* solver, string _name, BipedLIPKey* _obj, real_t _scale);
	};

}
