#pragma once

#include <DiMP/Graph/Node.h>
//#include <DiMP/Graph/Biped.h>

namespace DiMP {;

	class  BipedRunning;

	struct RunnerLipPosCon;
	struct RunnerLipVelCon;
	struct RunnerComConP;
	struct RunnerComConV;

	struct RunnerFootPosConT;
	struct RunnerFootPosConR;
	struct RunnerFootCopPosCon;
	struct RunnerFootCopVelCon;
	struct RunnerFootPosRangeConT;
	struct RunnerFootPosRangeConR;
	struct RunnerFootCopRangeCon;

	struct RunnerLipCmpCon;
	struct RunnerLipMomCon;
	struct RunnerCmpRangeCon;
	struct RunnerMomRangeCon;

	struct RunnerTimeCon;

	/**
	linear inverted pendulum model
	*/
	class BipedRunKey : public Keypoint {
	public:
		struct Foot {
			V3Var* var_pos_t  ;  ///< position
			SVar*  var_pos_r  ;  ///< orientation
			V3Var* var_vel_t  ;  ///< velocity
			SVar*  var_vel_r  ;  ///< angular velocity
			V3Var* var_cop_pos;
			V3Var* var_cop_vel;
			V3Var* var_cop_vel_diff;

			RunnerFootPosConT*   con_pos_t;
			RunnerFootPosConR*   con_pos_r;
			RunnerFootCopPosCon* con_cop_pos;
			RunnerFootCopVelCon* con_cop_vel;

			RunnerFootPosRangeConT* con_pos_range_t[3][2];   ///< range constraint on foot position relative to torso, [x|y|z]
			RunnerFootPosRangeConR* con_pos_range_r[2];      ///< range constraint on foot orientation relative to torso

			RunnerFootCopRangeCon* con_cop_range[3][2];   ///< range constraint on CoP relative to support foot, [x|y|z]


			FixConV3* con_vel_zero_t;
			FixConS*  con_vel_zero_r;
			FixConV3* con_cop_vel_diff_zero;

			real_t  weight;
		};

		V3Var* var_cmp_pos;        ///< position of CMP
		V3Var* var_cmp_vel;        ///< velocity of CMP
		V3Var* var_mom;            ///< angular momentum around CoM

		V3Var* var_torso_pos_t;    ///< position         of torso
		SVar*  var_torso_pos_r;    ///< orientation      of torso
		V3Var* var_torso_vel_t;    ///< velocity         of torso

		V3Var* var_com_pos;        ///< position of CoM
		V3Var* var_com_vel;        ///< velocity of CoM

		SVar* var_time;           ///< time
		SVar* var_duration;       ///< duration

		Foot        foot[2];

		RunnerLipPosCon* con_lip_pos;     ///< LIP position constraint
		RunnerLipVelCon* con_lip_vel;     ///< LIP velocity constraint
		RunnerLipCmpCon* con_lip_cmp;
		RunnerLipMomCon* con_lip_mom;

		RunnerComConP* con_com_pos;     ///< torso, feet, and com position constraint based on 3-mass model
		RunnerComConV* con_com_vel;     ///< torso, feet, and com velocity constraint based on 3-mass model

		RunnerTimeCon* con_time;	    ///< relates step duration and cumulative time
		RangeConS* con_duration_range;  ///< range constraint on step period
		
		RunnerCmpRangeCon* con_cmp_range[3][2];         ///< range constraint on CMP offset [x|y|z]
		RunnerMomRangeCon* con_mom_range[3][2];         ///< range constraint on centroidal mommentum [x|y|z]

		FixConV3* con_com_vel_zero;

		vec3_t  cop_pos;
		vec3_t  cop_vel;
		vec3_t  cop_acc;
	public:
		void AddVar(Solver* solver);
		void AddCon(Solver* solver);
		void Prepare();
		void Finish();
		void Draw(Render::Canvas* canvas, Render::Config* conf);

		BipedRunKey();
	};

	class BipedRunning : public TrajectoryNode {
	public:
		/// gait type
		struct GaitType {
			enum {
				Walk, //< walking gait
				Run,  //< running gait
			};
		};
		/// phase
		struct Phase {
			enum {
				R,   //< right support
				L,   //< left support
				RL,  //< double support: transition from R to L
				LR,  //< double support: trabsition from L to R
				R2,  //< right support for running
				L2,  //< left support for running
				RLF, //< flight (substitution of RL)
				LRF, //< filght (substitution of LR)
				D,   //< double support for starting and stopping
				Num
			};
		};
		/// swing foot trajectory type
		struct SwingProfile {
			enum {
				Cycloid,      //< flat-landing with cycloid swing profile
				HeelToe,      //< heel-to-toe
				Experiment,   //< experimental (may not work correctly)
			};
		};
		/// swing foot interpolation
		struct SwingInterpolation{
			enum {
				Cubic = 3,
			    Quintic = 5,
			};
		};
		/// contact state
		struct ContactState {
			enum {
				Float,    //< in floating state
				Surface,  //< in surface contact
				Heel,     //< in line contact on heel
				Toe,      //< in line contact on toe
			};
		};
		struct FootCurveType {
			enum {
				Arc,
				Clothoid,
			};
		};

		struct Param {
			vec3_t	gravity;				///< gravity (positive)
			real_t  T[4];                   ///< time constant of LIP ([start1, start2, cyclic run, walk])
			real_t  comHeight;
			real_t  torsoMass;
			real_t  footMass;
			//int     gaitType;
			int     swingProfile;
			int     swingInterpolation;
			real_t  swingHeight;                ///< 0: maximum swing height
			real_t	durationMin[Phase::Num];	///< minimum duration of each phase
			real_t	durationMax[Phase::Num];	///< maximum duration of each phase
			vec3_t	footPosMin[2];              ///< admissible range of foot relative to torso
			vec3_t  footPosMax[2];
			real_t  footOriMin[2];
			real_t  footOriMax[2];
			vec3_t  footCopMin[2];           ///< admissible range of CoP relative to foot
			vec3_t  footCopMax[2];

			vec3_t  cmpMin;                  ///< admissible range of CoP relative to foot
			vec3_t  cmpMax;
			vec3_t  momMin;
			vec3_t  momMax;
			
			int     footCurveType;
			real_t  ankleToToe;             ///< offset from foot center to the begining of toe|heel
			real_t  ankleToHeel;
			real_t  toeCurvature;           ///< toe|heel curvature (for arc)
			real_t  heelCurvature;
			real_t  toeCurvatureRate;       ///< toe|heel curvature rate (for clothoid)
			real_t  heelCurvatureRate;

			real_t  minSpacing;   ///< minimum lateral spacing of feet with which swing foot does not collide with support foot
			real_t  minDist;      ///< minimum distance between each foot with which swing foot does not collide with support foot
			real_t  swingMargin;  ///<

			Param();
		};

		struct Waypoint {
			int     k;
			real_t  time;
			vec3_t  com_pos;
			vec3_t  com_vel;
			real_t  torso_pos_r;
			vec3_t  foot_pos_t[2];
			real_t  foot_pos_r[2];
			vec3_t  foot_cop[2];
			vec3_t  foot_cop_min[2];
			vec3_t  foot_cop_max[2];
			vec3_t  cmp_pos;
			vec3_t  mom;

			bool    fix_com_pos;
			bool    fix_com_vel;
			bool    fix_torso_pos_r;
			bool    fix_foot_pos_t[2];
			bool    fix_foot_pos_r[2];
			bool    fix_foot_cop[2];
			bool    fix_cmp_pos;
			bool    fix_mom;
			bool    set_cop_range[2];

			Waypoint();
		};

		struct Snapshot {
			real_t  t;
			vec3_t  com_pos;
			vec3_t  com_vel;
			vec3_t  com_acc;
			vec3_t  cop_pos;
			vec3_t  cmp_pos;
			vec3_t  torso_pos_t;
			real_t  torso_pos_r;
			vec3_t  foot_pos_t[2];
			quat_t  foot_pos_r[2];
			vec3_t  foot_cop[2];

			Snapshot();
		};

		Param	            param;
		vector<int>	        phase;		   ///< phase at each step
		vector<int>	        gaittype;	   ///< gaitmode at each step
		vector<Waypoint>    waypoints;
		Snapshot            snapshot;
		vector<Snapshot>    trajectory;
		bool                trajReady;

		virtual Keypoint* CreateKeypoint() { return new BipedRunKey(); }
		virtual void		Init();
		virtual void		Prepare();
		virtual void        Finish();
		virtual void        CreateSnapshot(real_t time);
		virtual void        DrawSnapshot(Render::Canvas* canvas, Render::Config* conf);
		virtual void        Draw(Render::Canvas* canvas, Render::Config* conf);

		int    Phase(real_t t);
		bool   OnTransition(real_t t);

		void   ComState(real_t t, vec3_t& pos, vec3_t& vel, vec3_t& acc);
		void   CopState(real_t t, vec3_t& pos, vec3_t& vel, vec3_t& acc);
		void   TorsoState(real_t t, real_t& ori, real_t& angvel, real_t& angacc);
		void   FootRotation(real_t cp, real_t cv, real_t ca, vec3_t& pos, vec3_t& vel, vec3_t& acc, vec3_t& ori, vec3_t& angvel, vec3_t& angacc, int& contact);
		void   FootRotationInv(real_t ori, real_t angvel, real_t angacc, real_t& cp, real_t& cv, real_t& ca);
		void   FootPose(real_t t, int side, pose_t& pose, vec3_t& vel, vec3_t& angvel, vec3_t& acc, vec3_t& angacc, int& contact);
		void   FootCopState(real_t t, int side, vec3_t& pos, vec3_t& vel, real_t& weight);
		real_t TimeToLiftoff(real_t t, int side);
		real_t TimeToLanding(real_t t, int side);

		

		vec3_t Momentum(real_t t);
		vec3_t CmpPos(real_t t);
		vec3_t TorsoPos(const vec3_t& pcom, const vec3_t& psup, const vec3_t& pswg);
		vec3_t TorsoVel(const vec3_t& vcom, const vec3_t& vsup, const vec3_t& vswg);
		vec3_t TorsoAcc(const vec3_t& acom, const vec3_t& asup, const vec3_t& aswg);

		void CreateSnapshot(real_t t, Snapshot& s);
		void CalcTrajectory();
		void Save(const char* filename);
		//void Print();

	public:
		BipedRunning(Graph* g, string n);
		virtual ~BipedRunning();
	};

	// supplementaly function
	template <typename T>
	void   Interpolate(real_t t, T& p, T& v, T& a, real_t t0, const T& p0, const T& v0, const T& a0, real_t t1, const T& p1, const T& v1, const T& a1, int type);

	struct BipedRunCon : Constraint {
		BipedRunKey* obj[2];

		real_t T, T2;
		vec3_t g;
		vec3_t ez;
		real_t tau, tau2;
		real_t C, S;
		vec3_t p0, v0, p1, v1;
		vec3_t c0, cv0, ca0, c1;
		vec3_t cm0, cmv0, L0, cm1, L1;
		real_t k_p_p, k_p_v, k_p_c, k_p_cv, k_p_ca;
		real_t k_v_p, k_v_v, k_v_c, k_v_cv, k_v_ca;
		vec3_t k_p_tau, k_v_tau;
		real_t k_c_c[2], k_cv_c[2], k_cv_cv[2], k_ca_cv[2];
		vec3_t p_rhs;
		vec3_t v_rhs;

		int ph;
		int gtype;

		virtual void Prepare();

		BipedRunCon(Solver* solver, int _tag, string _name, BipedRunKey* _obj, real_t _scale);
	};

	/// CoM position constraint based on LIP model
	struct RunnerLipPosCon : BipedRunCon {
		virtual void  CalcCoef();
		virtual void  CalcDeviation();
		virtual void  CalcLhs();

		RunnerLipPosCon(Solver* solver, string _name, BipedRunKey* _obj, real_t _scale);
	};

	/// CoM velocity constraint based on LIP model
	struct RunnerLipVelCon : BipedRunCon {
		virtual void CalcCoef();
		virtual void CalcDeviation();
		virtual void CalcLhs();

		RunnerLipVelCon(Solver* solver, string _name, BipedRunKey* _obj, real_t _scale);
	};

	struct RunnerLipCmpCon : BipedRunCon {
		virtual void CalcCoef();
		virtual void CalcDeviation();
		virtual void CalcLhs();

		RunnerLipCmpCon(Solver* solver, string _string, BipedRunKey* _obj, real_t _scale);
	};

	struct RunnerLipMomCon : BipedRunCon {
		virtual void CalcCoef();
		virtual void CalcDeviation();
		virtual void CalcLhs();

		RunnerLipMomCon(Solver* solver, string _string, BipedRunKey* _obj, real_t _scale);
	};

	struct RunnerComConP : Constraint {
		BipedRunKey* obj;

		virtual void CalcCoef();

		RunnerComConP(Solver* solver, string _name, BipedRunKey* _obj, real_t _scale);
	};

	struct RunnerComConV : Constraint {
		BipedRunKey* obj;

		virtual void CalcCoef();

		RunnerComConV(Solver* solver, string _name, BipedRunKey* _obj, real_t _scale);
	};

	// foot position update
	struct RunnerFootPosConT : Constraint {
		BipedRunKey* obj[2];
		int side;
		vec3_t p0, v0, p1;
		real_t	tau;

		void Prepare();

		virtual void CalcCoef();
		virtual void CalcDeviation();
		virtual void CalcLhs();

		RunnerFootPosConT(Solver* solver, string _name, BipedRunKey* _obj, int _side, real_t _scale);
	};

	// foot orientation update
	struct RunnerFootPosConR : Constraint {
		BipedRunKey* obj[2];
		int           side;
		real_t        theta0, omega0, theta1;
		real_t        tau;

		void Prepare();

		virtual void CalcCoef();
		virtual void CalcDeviation();
		virtual void CalcLhs();

		RunnerFootPosConR(Solver* solver, string _name, BipedRunKey* _obj, int _side, real_t _scale);
	};

	// CoP position update constraint
	struct RunnerFootCopPosCon : Constraint {
		BipedRunKey* obj[2];
		int           side;
		vec3_t        c0, cv0, c1;
		real_t        tau;

		void Prepare();

		virtual void CalcCoef();
		virtual void CalcDeviation();
		virtual void CalcLhs();

		RunnerFootCopPosCon(Solver* solver, string _name, BipedRunKey* _obj, int _side, real_t _scale);
	};

	struct RunnerFootCopVelCon : Constraint {
		BipedRunKey* obj[2];
		int           side;
		vec3_t        cv0, cvd0, cv1;
		real_t        tau;

		void Prepare();

		virtual void CalcCoef();
		virtual void CalcDeviation();
		virtual void CalcLhs();

		RunnerFootCopVelCon(Solver* solver, string _name, BipedRunKey* _obj, int _side, real_t _scale);
	};

	/// range limit of support foot relative to torso
	struct RunnerFootPosRangeConT : Constraint {
		BipedRunKey* obj;
		int          side;
		real_t       bound;
		vec3_t       dir, dir_abs;
		real_t       theta;
		mat3_t       R;
		vec3_t       ez;
		vec3_t       r;

		void Prepare();

		virtual void CalcCoef();
		virtual void CalcDeviation();

		RunnerFootPosRangeConT(Solver* solver, string _name, BipedRunKey* _obj, int _side, vec3_t _dir, real_t _scale);
	};

	/// range limit of support foot relative to torso
	struct RunnerFootPosRangeConR : Constraint {
		BipedRunKey* obj;
		int          side;
		real_t       bound;
		real_t       thetaf, thetat;
		real_t       dir;
		real_t       r;

		void Prepare();

		virtual void CalcCoef();
		virtual void CalcDeviation();

		RunnerFootPosRangeConR(Solver* solver, string _name, BipedRunKey* _obj, int _side, real_t _dir, real_t _scale);
	};

	/// CoP range constraint
	struct RunnerFootCopRangeCon : Constraint {
		BipedRunKey* obj;
		int          side;
		real_t       bound;
		vec3_t       dir, dir_abs;
		real_t       theta;
		mat3_t       R;
		vec3_t       ez;
		vec3_t       r;

		void Prepare();

		virtual void CalcCoef();
		virtual void CalcDeviation();

		RunnerFootCopRangeCon(Solver* solver, string _name, BipedRunKey* _obj, int _side, vec3_t _dir, real_t _scale);
	};

	struct RunnerCmpRangeCon : Constraint {
		BipedRunKey* obj;
		int          side;
		real_t       bound;
		vec3_t       dir, dir_abs;
		real_t       theta;
		mat3_t       R;
		vec3_t       ez;
		vec3_t       r;

		void Prepare();

		virtual void CalcCoef();
		virtual void CalcDeviation();

		RunnerCmpRangeCon(Solver* solver, string _name, BipedRunKey* _obj, vec3_t _dir, real_t _scale);
	};

	struct RunnerMomRangeCon : Constraint {
		BipedRunKey* obj;
		int          side;
		real_t       bound;
		vec3_t       dir, dir_abs;
		real_t       theta;
		mat3_t       R;
		vec3_t       ez;
		vec3_t       r;

		void Prepare();

		virtual void CalcCoef();
		virtual void CalcDeviation();

		RunnerMomRangeCon(Solver* solver, string _name, BipedRunKey* _obj, vec3_t dir, real_t _scale);
	};

	/// time constraint
	struct RunnerTimeCon : Constraint {
		BipedRunKey* obj[2];

		virtual void CalcCoef();
		virtual void CalcLhs();

		RunnerTimeCon(Solver* solver, string _name, BipedRunKey* _obj, real_t _scale);
	};
}
