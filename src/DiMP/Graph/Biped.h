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
	struct BipedFootHeightCon;
	struct BipedTimeCon;

	/**
	linear inverted pendulum model
	*/
	class BipedLIPKey : public Keypoint {
	public:
		V3Var*      var_torso_pos_t;    ///< position         of torso
		SVar*       var_torso_pos_r;    ///< orientation      of torso
		V3Var*      var_torso_vel_t;    ///< velocity         of torso

		V3Var*		var_com_pos;        ///< position of CoM
		V3Var*		var_com_vel;        ///< velocity of CoM
		V3Var*      var_mom;            ///< angular momentum around CoM; it is actually normalized momentum L/mg

		V3Var*		var_cop_pos;        ///< position of CoP
		V3Var*      var_cop_vel;        ///< velocity of CoP

		V3Var*      var_cmp_pos;        ///< CMP offset
		V3Var*      var_cmp_vel;        ///< CMP offset time derivative

		V3Var*      var_foot_pos_t[2];  ///< position    of foot, R/L
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

		BipedFootRangeConT*   con_foot_range_t[2][3];   ///< range constraint on foot position relative to torso, [r|l][x|y|z]
		BipedFootRangeConR*   con_foot_range_r[2];      ///< range constraint on foot orientation relative to torso, [r|l]

		BipedCopRangeCon*  con_cop_range[3];   ///< range constraint on CoP relative to support foot, [x|y|z]
		BipedAccRangeCon*  con_acc_range[3];   ///< range constraint on CoM acceleration, [x|y|z]
		BipedCmpRangeCon*  con_cmp_range[3];   ///< range constraint on CMP, [x|y|z]
		BipedMomRangeCon*  con_mom_range[3];   ///< range constraint on angular momentum, [x|y|z]

		BipedTimeCon*    con_time;			   ///< relates step duration and cumulative time
		RangeConS*       con_duration_range;   ///< range constraint on step period

		MatchConV3*      con_foot_match_t[2];
		MatchConS *      con_foot_match_r[2];

		BipedFootHeightCon*  con_foot_height[2];

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
        /// swing foot interpolation
        struct SwingInterpolation{
            enum{
                Cubic   = 3,
                Quintic = 5,
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
		struct FootCurveType{
			enum{
				Arc,
				Clothoid,
			};
		};
		struct ComHeightProfile{
			enum{
				Constant,
				Compass,
			};
		};

		struct Param {
			real_t	gravity;				///< gravity (positive)
			real_t  T;                      ///< time constant of LIP
			real_t  comHeight;
			real_t  torsoMass;
			real_t  footMass;
			int     swingProfile;
            int     swingInterpolation;
			real_t  swingHeight[2];             ///< 0: maximum swing height
												///< 1: lowest height before touch down. for wedge only
			int     comHeightProfile;
			real_t	durationMin[Phase::Num];	///< minimum duration of each phase
			real_t	durationMax[Phase::Num];	///< maximum duration of each phase
			vec3_t	footPosMin[2];              ///< admissible range of foot relative to torso
			vec3_t  footPosMax[2];
			real_t  footOriMin[2];
			real_t  footOriMax[2];
			vec3_t  copMin;                  ///< admissible range of CoP relative to foot
			vec3_t  copMax;
			vec3_t  cmpMin;                  ///< admissible range of CoP relative to foot
			vec3_t  cmpMax;
			vec3_t  accMin;                  ///< admissible range of CoM acceleration
			vec3_t  accMax;
			vec3_t  momMin;                  ///< admissible range of angular momentum
			vec3_t  momMax;
			
			int     footCurveType;
			real_t  ankleToToe ;
			real_t  ankleToHeel;
			real_t  toeCurvature ;
			real_t  heelCurvature;
			real_t  toeCurvatureRate ;
			real_t  heelCurvatureRate;

			real_t  minSpacing;   ///< minimum lateral spacing of feet with which swing foot does not collide with support foot
			real_t  swingMargin;  ///< 

			Param();
		};

		struct Waypoint {
			int     k;
			real_t  time;
			//vec3_t  torso_pos_t;
			//vec3_t  torso_vel_t;
			vec3_t  com_pos;
			vec3_t  com_vel;
			real_t  torso_pos_r;
			vec3_t  foot_pos_t[2];
			real_t  foot_pos_r[2];
			vec3_t  cop_pos;

			bool    fix_time;
			//bool    fix_torso_pos_t;
			//bool    fix_torso_vel_t;
			bool    fix_com_pos;
			bool    fix_com_vel;
			bool    fix_torso_pos_r;
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
			vec3_t  com_vel;
			vec3_t  com_acc;
			vec3_t  torso_pos_t;
			real_t  torso_pos_r;
			vec3_t  foot_pos_t[2];
			quat_t  foot_pos_r[2];
			vec3_t  cop_pos;
			vec3_t  cmp_pos;

			Snapshot();
		};

		Param	            param;
		vector<int>	        phase;		   ///< walking phase at each step
		vector<real_t>      elevation;     ///< ground z 
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
		
		int    Phase        (real_t t);
		//vec3_t ComPos       (real_t t);
		//vec3_t ComVel       (real_t t);
		//vec3_t ComAcc       (real_t t);
		void   ComState     (real_t t, vec3_t& pos, vec3_t& vel, vec3_t& acc);
		real_t TorsoOri     (real_t t);
		real_t TorsoAngVel  (real_t t);
		real_t TorsoAngAcc  (real_t t);
		void   FootRotation (real_t px0, real_t pz0, real_t cp, real_t cv, real_t ca, vec3_t& pos, vec3_t& angle, vec3_t& vel, vec3_t& angvel, vec3_t& acc, vec3_t& angacc, int& contact);
		void   FootPose     (real_t t, int side, pose_t& pose, vec3_t& vel, vec3_t& angvel, vec3_t& acc, vec3_t& angacc, int& contact);
		real_t TimeToLiftoff(real_t t, int side);
		real_t TimeToLanding(real_t t, int side);
		vec3_t CopPos       (real_t t);
		vec3_t CmpPos       (real_t t);
		vec3_t Momentum     (real_t t);
		
		vec3_t TorsoPos(const vec3_t& pcom, const vec3_t& psup, const vec3_t& pswg);
		vec3_t TorsoVel(const vec3_t& vcom, const vec3_t& vsup, const vec3_t& vswg);
		vec3_t TorsoAcc(const vec3_t& acom, const vec3_t& asup, const vec3_t& aswg);

		void CreateSnapshot(real_t t, Snapshot& s);
		void CalcTrajectory();
		//void Save (const char* filename);
		//void Print();

	public:
		BipedLIP(Graph* g, string n);
		virtual ~BipedLIP();
	};

	struct BipedLipCon : Constraint {
		BipedLIPKey* obj[2];

		real_t T;
		vec3_t ez;
		real_t tau;
		real_t C, S;
		vec3_t p0, p1;
		vec3_t v0, v1;
		vec3_t c0, c1;
		vec3_t cm0, cm1;
		vec3_t L0, L1;
		vec3_t cmv0;
		vec3_t cv0;
		
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

	struct BipedRangeCon : Constraint{
		BipedLIPKey* obj;
		vec3_t       dir, dir_abs;
		real_t       theta;
		mat3_t       R, dR;
		vec3_t       ez;
		vec3_t       r;
		real_t       _min, _max;
		bool         on_lower, on_upper;

		void Prepare();

		virtual void CalcDeviation();
		virtual void Project(real_t& l, uint k);

		BipedRangeCon(Solver* solver, int _tag, string _name, BipedLIPKey* _obj, vec3_t _dir, real_t _scale);
	};

	/// range limit of support foot relative to torso
	struct BipedFootRangeConT : BipedRangeCon {
		uint         side;
		
		virtual void CalcCoef();
		
		BipedFootRangeConT(Solver* solver, string _name, BipedLIPKey* _obj, uint _side, vec3_t _dir, real_t _scale);
	};

	struct BipedFootRangeConR : BipedRangeCon {
		uint         side;
		real_t       thetaf, thetat;

		virtual void CalcCoef();
		virtual void CalcDeviation();
		
		BipedFootRangeConR(Solver* solver, string _name, BipedLIPKey* _obj, uint _side, real_t _scale);
	};

	/// CoP range constraint
	struct BipedCopRangeCon : BipedRangeCon {
		uint         side;
		
		virtual void CalcCoef();
		
		BipedCopRangeCon(Solver* solver, string _name, BipedLIPKey* _obj, uint _side, vec3_t _dir, real_t _scale);
	};

	/// CMP range constraint
	struct BipedCmpRangeCon : BipedRangeCon {
		
		virtual void CalcCoef();
		
		BipedCmpRangeCon(Solver* solver, string _name, BipedLIPKey* _obj, vec3_t _dir, real_t _scale);
	};

	/// CoM acceleration range constraint
	struct BipedAccRangeCon : BipedRangeCon {
		
		virtual void CalcCoef();
		
		BipedAccRangeCon(Solver* solver, string _name, BipedLIPKey* _obj, vec3_t _dir, real_t _scale);
	};

	/// angular momentum range constraint
	struct BipedMomRangeCon : BipedRangeCon {
		
		virtual void CalcCoef();
		
		BipedMomRangeCon(Solver* solver, string _name, BipedLIPKey* _obj, vec3_t _dir, real_t _scale);
	};

	/// foot height constraint
	struct BipedFootHeightCon : Constraint{
		BipedLIPKey* obj;
		uint         side;

		virtual void CalcCoef     ();
		virtual void CalcDeviation();
		
		BipedFootHeightCon(Solver* solver, string _name, BipedLIPKey* _obj, uint _side, real_t _scale);
	};

	/// time constraint
	struct BipedTimeCon : Constraint {
		BipedLIPKey* obj[2];

		virtual void CalcCoef();
		virtual void CalcLhs ();

		BipedTimeCon(Solver* solver, string _name, BipedLIPKey* _obj, real_t _scale);
	};

}
