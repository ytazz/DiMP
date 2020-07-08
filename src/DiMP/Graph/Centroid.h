#pragma once

#include <DiMP/Graph/Node.h>

namespace DiMP {;

	/**
		centroidal dynamics model
	*/
	class CentroidKey : public Keypoint {
	public:
		V3Var*      var_com_pos_t;    ///< position         of com
		QVar*       var_com_pos_r;    ///< orientation      of com
		V3Var*      var_com_vel_t;    ///< velocity         of com
		V3Var*      var_com_vel_r;    ///< angular velocity of com

		struct End{
			V3Var*  var_end_pos_t;
			vec3_t  end_vel_t;
			//V3Var*  var_end_vel_t;
		};

		struct Face{
			real_t         activity;
			vector<SVar*>  var_force;
			vector<SVar*>  var_vel;
		};

		vector<End >  ends;
		vector<Face>  faces;

	public:
		virtual void AddVar(Solver* solver);
		virtual void AddCon(Solver* solver);
		virtual void Prepare();
		virtual void Draw(Render::Canvas* canvas, Render::Config* conf);

		CentroidKey();
	};

	class Centroid : public TrajectoryNode {
	public:
		struct Param {
			struct End{
				vec3_t  rangeMin;
				vec3_t  rangeMax;
			};
			struct Face{
				vec3_t  pos;
				quat_t  ori;
				vec3_t  normal;
				vec2_t  rangeMin;
				vec2_t  rangeMax;

				vector<vec3_t>  coneBasis;
			};

			real_t	mass;
			vec3_t	gravity;

			vector<End>   ends;
			vector<Face>  faces;

			Param();
		};

		struct Waypoint {
			int     k;
			real_t  time;

			vec3_t  com_pos_t;
			vec3_t  com_vel_t;
			quat_t  com_pos_r;
			vec3_t  com_vel_r;
			
			bool    fix_time;
			bool    fix_com_pos_t;
			bool    fix_com_pos_r;
			bool    fix_com_vel_t;
			bool    fix_com_vel_r;
			
			Waypoint();
		};

		struct TrajPoint {
			real_t  t;
			vec3_t  com_pos;
		
			TrajPoint();
		};

		Param	            param;
		vector<Waypoint>    waypoints;
		vector<TrajPoint>   trajectory;
		bool                trajReady;

		virtual Keypoint*	CreateKeypoint() { return new CentroidKey(); }
		virtual void		Init();
		virtual void		Prepare();

		vec3_t ComPos   (real_t t, int type = Interpolate::Quadratic);
		vec3_t ComVel   (real_t t, int type = Interpolate::Quadratic);
		quat_t ComOri   (real_t t, int type = Interpolate::SlerpDiff);
		vec3_t ComAngVel(real_t t, int type = Interpolate::Quadratic);
		vec3_t EndPos   (real_t t, int index, int type = Interpolate::Quadratic);
		vec3_t EndVel   (real_t t, int index, int type = Interpolate::Quadratic);

		void CalcTrajectory();
		void Draw(Render::Canvas* canvas, Render::Config* conf);
		void DrawSnapshot(real_t time, Render::Canvas* canvas, Render::Config* conf);
		void Save();
		void Print();

	public:
		         Centroid(Graph* g, string n);
		virtual ~Centroid();
	};

	struct CentroidCon : Constraint {
		CentroidKey* obj[2];

		void Prepare();

		CentroidCon(Solver* solver, int _tag, string _name, CentroidKey* _obj, real_t _scale);
	};

}
