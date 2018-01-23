#pragma once

#include <DiMP/Graph/ID.h>
#include <DiMP/Graph/Node.h>
#include <DiMP/Graph/Tree.h>
#include <DiMP/Graph/Connector.h>
#include <DiMP/Render/Canvas.h>
#include <DiMP/Render/Config.h>

namespace DiMP{;

class Geometry  ;
class TimeSlot  ;
class Object    ;
class BipedLIP  ;
class Joint     ;
class Task      ;

typedef ArrayBase<Geometry *>  Geometries;
typedef ArrayBase<TimeSlot *>  TimeSlots ;
typedef ArrayBase<Object   *>  Objects   ;
typedef ArrayBase<Connector*>  Connectors;
typedef ArrayBase<BipedLIP *>  Bipeds    ;
typedef ArrayBase<Joint    *>  Joints    ;
typedef ArrayBase<Task     *>  Tasks     ;

/**
	graph structure expressing a Multi-body System
 */
class Graph : public UTRefCount{
public:
	/// physical parameters
	struct Param{
		vec3_t			gravity;		///< gravity

		Param(){
			gravity   = vec3_t(0.0, 0.0, 0.0);
		}
	};

	/// scaling factors
	struct Scale{
		real_t		time,    time_inv;
		real_t		size,    size_inv;
		real_t		mass,    mass_inv;
		real_t		inertia, inertia_inv;

		real_t		pos_t,  pos_t_inv;
		real_t		vel_t,  vel_t_inv;
		real_t		acc_t,  acc_t_inv;
	
		real_t		pos_r,  pos_r_inv;
		real_t		vel_r,  vel_r_inv;
		real_t		acc_r,  acc_r_inv;
	
		real_t		force_t,  force_t_inv;
		real_t		force_r,  force_r_inv;

		void Set(real_t T, real_t L, real_t M);

		Scale();
	};

	bool			ready;				///< ready flag

	/// nodes of different types
	NodeArray            nodes;
	TrajectoryNodeArray  trajNodes;

	Ticks			ticks;			///< ticks
	Objects			objects;		///< objects
	Connectors      cons;
	Bipeds			bipeds;         ///< bipeds
	Trees			trees;			///< trees
	Joints			joints;			///< joints
	Geometries		geos;			///< geometries
	TimeSlots		timeslots;		///< time slots
	Tasks			tasks;			///< tasks
	
	UTRef<Solver>	       solver;	///< internal solver
	UTRef<Render::Config>  conf;	///< default draw configuration

	Param		 param;
	Scale		 scale;
	
	void Prepare();
	void Finish ();

public:
	/// set scaling factor;
	void	SetScaling(real_t T, real_t L, real_t M);

	/// take snapshot
	void    CreateSnapshot(real_t t);

	/// does initialization
	virtual void Init();

	/// clears all objects
	virtual void Clear();

	/// resets all variables. objects are not deleted
	virtual void Reset();

	/** 
		computes one step of planning
	 **/
	virtual void Step();

	/// visualize the plan
	virtual void Draw(Render::Canvas* canvas, Render::Config* conf = 0);

	/// draw snapshot
	virtual void DrawSnapshot(real_t time, Render::Canvas* canvas, Render::Config* conf = 0);


	Graph();
	virtual ~Graph(){}
};

}
