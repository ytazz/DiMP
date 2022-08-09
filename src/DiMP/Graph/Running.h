#pragma once

#include <DiMP/Graph/Node.h>
#include <DiMP/Graph/Biped.h>

namespace DiMP {;

	class  BipedRunning;

	struct RunnerLipPosCon;
	struct RunnerLipVelCon;
	struct RunnerComConP;
	struct RunnerComConV;

	struct RunnerFootPosConT;
	struct RunnerFootPosConR;
	struct RunnerFootCopPosCon;
	struct RunnerFootCoPVelCon;
	struct RunnerFootPosRangeConT;
	struct RunnerFootPosRangeConR;
	struct RunnerFootCopRangeCon;
	struct RunnerTimeCon;


	struct RunnerFootRangeConT;
	struct RunnerFootRangeConR;
	//struct BipedFootCopPosCon;
	struct RunnerCopRangeCon;
	struct RunnerCmpRangeCon;
	struct RunnerAccRangeCon;
	//struct RunnerMomRangeCon;
	struct RunnerFootHeightCon;
	struct RunnerTimeCon;

	/**
	linear inverted pendulum model
	*/
	class BipedRunKey : public BipedLIPKey {
	public:
		void AddVar(Solver* solver);
		void AddCon(Solver* solver);
		//void Draw(Render::Canvas* canvas, Render::Config* conf);
	};

	class BipedRunning : public BipedLIP {
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
				HeelToe,
				Experiment,   //< experimental (may not work correctly)
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
			
			//vec3_t  accMin;                  ///< admissible range of CoM acceleration
			//vec3_t  accMax;
			vec3_t  momMin;                  ///< admissible range of angular momentum
			vec3_t  momMax;

			Param();
		};

		Param	            param;
		vector<int>	        phase;		   ///< phase at each step
		vector<int>	        gaittype;	   ///< gaitmode at each step
		vector<Waypoint>    waypoints;
		Snapshot            snapshot;
		vector<Snapshot>    trajectory;
		bool                trajReady;

		virtual Keypoint* CreateKeypoint() { return new BipedLIPKey(); }
		virtual void		Init();
		virtual void		Prepare();
		virtual void        Finish();
		virtual void        CreateSnapshot(real_t time);
		virtual void        DrawSnapshot(Render::Canvas* canvas, Render::Config* conf);
		virtual void        Draw(Render::Canvas* canvas, Render::Config* conf);

		int    Phase(real_t t);
		//vec3_t ComPos       (real_t t);
		//vec3_t ComVel       (real_t t);
		//vec3_t ComAcc       (real_t t);
		bool   OnTransition(real_t t);
		void   ComState(real_t t, vec3_t& pos, vec3_t& vel, vec3_t& acc);
		real_t TorsoOri(real_t t);
		real_t TorsoAngVel(real_t t);
		real_t TorsoAngAcc(real_t t);
		void   FootPose(real_t t, int side, pose_t& pose, vec3_t& vel, vec3_t& angvel, vec3_t& acc, vec3_t& angacc, int& contact);
		real_t TimeToLiftoff(real_t t, int side);
		real_t TimeToLanding(real_t t, int side);
		vec3_t CopPos(real_t t);
		

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

	struct BipedRunCon : BipedLipCon {
		BipedRunKey* obj[2];
		int ph;
		int gtype;
		vec3_t g;

		void Prepare();
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

	struct RunnerFootPosConT : Constraint {
		BipedRunKey* obj[2];
		int side;
		vec3_t p0, v0, p1;
		real_t	tau;

		void Prepare();

		virtual void CalcCoeff();
		virtual void CalcDeviation();
		virtual void CalcLhs();

		RunnerFootPosConT(Solver* solver, string _name, BipedRunKey* _obj, int _side, real_t _scale);
	};

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

	/// time constraint
	struct RunnerTimeCon : Constraint {
		BipedRunKey* obj[2];

		virtual void CalcCoef();
		virtual void CalcLhs();

		RunnerTimeCon(Solver* solver, string _name, BipedRunKey* _obj, real_t _scale);
	};
}
