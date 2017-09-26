#pragma once

#include <DiMP/Graph/Node.h>
//#include <DiMP/Solver/Range.h>

namespace DiMP {
	;

	class  BipedLIP;

	struct BipedLipConP;
	struct BipedLipConV;

	//struct CoMConR;

	struct BipedComConP;
	struct BipedComConV;

	struct BipedFootConT;
	struct BipedFootConR;

	struct BipedCopCon;

	struct BipedTimeCon;

	/**
	linear inverted pendulum model
	*/
	class BipedLIPKey : public Keypoint {
	public:
		V2Var*      var_torso_pos_t;    ///< position         of torso
		SVar*       var_torso_pos_r;    ///< orientation      of torso
		V2Var*      var_torso_vel_t;    ///< velocity         of torso
		SVar*       var_torso_vel_r;    ///< angular velocity of torso

		V2Var*		var_com_pos;        ///< position of CoM
		V2Var*		var_com_vel;        ///< velocity of CoM

		V2Var*		var_cop_pos;        ///< position of CoP

		V2Var*      var_foot_pos_t[2];  ///< position    of foot, R/L
		SVar*       var_foot_pos_r[2];  ///< orientation of foot, R/L

		SVar*       var_time;           ///< time
		SVar*		var_duration;       ///< duration

		BipedLipConP*    con_lip_p;          ///< LIP position constraint
		BipedLipConV*    con_lip_v;          ///< LIP velocity constraint

		BipedComConP*    con_com_p;          ///< CoM position constraint
		BipedComConV*    con_com_v;          ///< CoM velocity constraint

		BipedFootConT*   con_foot_t[2];      ///< range constraint on support foot position relative to torso
		BipedFootConR*   con_foot_r[2];      ///< range constraint on support foot orientation relative to torso

		BipedCopCon*     con_cop;            ///< range constraint on CoP relative to support foot
		
		BipedTimeCon*    con_time;
		RangeConS*       con_duration;       ///< range constraint on step period

		MatchConV2*      con_foot_match_t[2];
		MatchConS *      con_foot_match_r[2];

		//CoMConR  *  con_com_r[2];      ///< range constraint on angular acceleration at beginning and end of step
		//CoPConR  *  con_cop_r[2];      ///< range constraint on turning angle of CoM
		
	public:
		virtual void AddVar(Solver* solver);
		virtual void AddCon(Solver* solver);
		virtual void Prepare();
		virtual void Draw(Render::Canvas* canvas, Render::Config* conf);

		BipedLIPKey();
	};

	class BipedLIP : public TrajectoryNode {
	public:
		/// walking phase
		struct Phase {
			enum {
				R,
				L,
				RL,
				LR,
				Num
			};
		};
		/// swing foot trajectory type
		struct SwingProfile {
			enum {
				Wedge,        ///< 
				Cycloid,      ///< 
			};
		};

		struct Param {
			real_t	gravity;				///< gravity (positive)
			real_t  heightCoM;
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
			vec2_t  copPosMin;                  ///< admissible range of CoP relative to foot
			vec2_t  copPosMax;
			real_t  T;                      ///< time constant of LIP Žž’è”
			real_t  angAccMax;              ///< maximum admissible angular acceleration
			real_t  turnMax;                ///< maximum admissible turning angle in single step

			Param();
		};

		/// Œo—R“_
		struct Waypoint {
			int     k;
			real_t  time;
			vec2_t  torso_pos_t;
			vec2_t  torso_vel_t;
			real_t  torso_pos_r;
			real_t  torso_vel_r;
			vec2_t  foot_pos_t[2];
			real_t  foot_pos_r[2];

			bool    fix_time;
			bool    fix_torso_pos_t;
			bool    fix_torso_pos_r;
			bool    fix_torso_vel_t;
			bool    fix_torso_vel_r;
			bool    fix_foot_pos_t[2];
			bool    fix_foot_pos_r[2];

			Waypoint();
		};

		/// ‹O“¹
		struct TrajPoint {
			real_t  t;
			vec3_t  com_pos;
			vec3_t  torso_pos_t;
			real_t  torso_pos_r;
			vec3_t  foot_pos_t[2];
			real_t  foot_pos_r[2];
			vec3_t  cop_pos;
			
			TrajPoint();
		};

		Param	            param;
		vector<int>	        phase;		///< walking phase at each step
		vector<Waypoint>    waypoints;
		vector<TrajPoint>   trajectory;
		bool                trajReady;

		virtual Keypoint*	CreateKeypoint() { return new BipedLIPKey(); }
		virtual void		Init();
		virtual void		Prepare();

		int    Phase      (real_t t);
		vec3_t ComPos     (real_t t);
		vec3_t ComVel     (real_t t);
		vec3_t ComAcc     (real_t t);
		real_t TorsoOri   (real_t t);
		real_t TorsoAngVel(real_t t);
		real_t TorsoAngAcc(real_t t);
		vec3_t FootPos    (real_t t, int side);
		real_t FootOri    (real_t t, int side);
		vec3_t CopPos     (real_t t);
		
		vec3_t TorsoPos(const vec3_t& pcom, const vec3_t& psup, const vec3_t& pswg);
		
		void CalcTrajectory();
		void Draw          (Render::Canvas* canvas, Render::Config* conf);
		void DrawSnapshot  (real_t time, Render::Canvas* canvas, Render::Config* conf);
		//void Save          ();

	public:
		BipedLIP(Graph* g, string n);
		virtual ~BipedLIP();
	};

	struct BipedLipCon : Constraint {
		BipedLIPKey* obj[2];
		
		real_t tau, tau2;
		real_t T, T2;
		vec2_t p0, p1;
		vec2_t v0, v1;
		vec2_t c0, c1;
		real_t C, S;
		
		void Prepare();

		BipedLipCon(Solver* solver, int _tag, string _name, BipedLIPKey* _obj, real_t _scale);
	};

	/// CoM position constraint based on LIP model
	struct BipedLipConP : BipedLipCon {
		virtual void CalcCoef     ();
		virtual void CalcDeviation();

		BipedLipConP(Solver* solver, string _name, BipedLIPKey* _obj, real_t _scale);
	};

	/// CoM velocity constraint based on LIP model
	struct BipedLipConV : BipedLipCon {
		virtual void CalcCoef     ();
		virtual void CalcDeviation();

		BipedLipConV(Solver* solver, string _name, BipedLIPKey* _obj, real_t _scale);
	};

	/// CoM position constraint
	struct BipedComConP : Constraint {
		BipedLIPKey* obj;
		
		virtual void CalcCoef();

		BipedComConP(Solver* solver, string _name, BipedLIPKey* _obj, real_t _scale);
	};

	/// CoM velocity constraint
	struct BipedComConV : Constraint {
		BipedLIPKey* obj;
		
		virtual void CalcCoef();

		BipedComConV(Solver* solver, string _name, BipedLIPKey* _obj, real_t _scale);
	};

	/// range limit of support foot relative to torso
	struct BipedFootConT : Constraint {
		BipedLIPKey* obj;
		uint         side;
		vec2_t       pf, pt;
		real_t       thetat;
		mat2_t       R, dR;
		vec2_t       _min, _max;
		bool         on_lower[2], on_upper[2];
		
		virtual void CalcCoef     ();
		virtual void CalcDeviation();
		virtual void Project      (real_t& l, uint k);

		BipedFootConT(Solver* solver, string _name, BipedLIPKey* _obj, uint _side, real_t _scale);
	};

	struct BipedFootConR : Constraint {
		BipedLIPKey* obj;
		uint         side;
		real_t       thetaf, thetat;
		real_t       _min, _max;
		bool	     on_lower, on_upper;
		
		virtual void CalcCoef     ();
		virtual void CalcDeviation();
		virtual void Project      (real_t& l, uint k);

		BipedFootConR(Solver* solver, string _name, BipedLIPKey* _obj, uint _side, real_t _scale);
	};

	/// CoP constraint
	struct BipedCopCon : Constraint {
		BipedLIPKey* obj;
		uint         side;
		vec2_t       pc, pf;
		real_t       thetaf;
		mat2_t       R, dR;
		vec2_t       _min, _max;
		bool         on_lower[2], on_upper[2];
		
		virtual void CalcCoef     ();
		virtual void CalcDeviation();
		virtual void Project      (real_t& l, uint k);

		BipedCopCon(Solver* solver, string _name, BipedLIPKey* _obj, uint _side, real_t _scale);
	};

	struct BipedTimeCon : Constraint{
		BipedLIPKey* obj[2];
		
		virtual void CalcCoef();
		
		BipedTimeCon(Solver* solver, string _name, BipedLIPKey* _obj, real_t _scale);
	};

	/// angular acceleration limit
	/*
	struct CoMConR : Constraint {
		BipedLIPKey* obj[2];
		uint         idx;
		real_t       _min, _max;
		bool	     on_lower, on_upper;
		real_t       q0, w0, q1, w1, tau, tau2, tau3;

		virtual void CalcCoef     ();
		virtual void CalcDeviation();
		virtual void Project      (real_t& l, uint k);

		CoMConR(Solver* solver, string _name, BipedLIPKey* _obj, uint _idx, real_t _scale);
	};
	*/

}
