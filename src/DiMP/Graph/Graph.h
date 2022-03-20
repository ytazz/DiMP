#pragma once

#include <DiMP/Graph/ID.h>
#include <DiMP/Graph/Node.h>
#include <DiMP/Graph/Tree.h>
#include <DiMP/Graph/Geometry.h>
#include <DiMP/Graph/Connector.h>
#include <DiMP/Graph/Octtree.h>
#include <DiMP/Graph/Solver.h>
#include <DiMP/Render/Canvas.h>
#include <DiMP/Render/Config.h>

#include <set>

namespace DiMP{;

class Geometry ;
class TimeSlot ;
class Object   ;
class BipedLIP ;
class BipedRunning;
class Centroid ;
class Joint    ;
class Task     ;
class AvoidTask;

typedef NodeArray<Geometry >  Geometries;
typedef NodeArray<TimeSlot >  TimeSlots ;
typedef NodeArray<Object   >  Objects   ;
typedef NodeArray<BipedLIP >  Bipeds    ;
typedef NodeArray<BipedRunning >  Runners    ;
typedef NodeArray<Centroid >  Centroids ;
typedef NodeArray<Joint    >  Joints    ;
typedef NodeArray<Task     >  Tasks     ;
typedef NodeArray<AvoidTask>  Avoids    ;

/**
	graph structure expressing a Multi-body System
 */
class Graph : public UTRefCount{
public:
	/// physical parameters
	struct Param{
		vec3_t	 gravity;  ///< gravity
		//vec3_t   bbmin;    ///< range of global bounding box
		//vec3_t   bbmax;
		//int      octtreeDepth;

		Param();
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
	NodeArray<Node>      nodes;
	TrajectoryNodeArray  trajNodes;

	Ticks			ticks;			///< ticks
	Objects			objects;		///< objects
	Connectors      cons;
	Bipeds			bipeds;         ///< bipeds
	Runners         runners;        ///< runners
	Centroids       centroids;      ///< centroids
	Trees			trees;			///< trees
	Joints			joints;			///< joints
	Geometries		geos;			///< geometries
	TimeSlots		timeslots;		///< time slots
	Tasks			tasks;			///< tasks
	Avoids          avoids;         ///< avoid tasks

	UTRef<CustomSolver>	   solver;	///< internal solver
	UTRef<Render::Config>  conf;	///< default draw configuration

	Param		 param;
	Scale		 scale;

	/// variables used for broad phase collision detection
	struct GPTableEntry{
		GeometryInfo* info[2];
		uint8_t       intersect;

		GPTableEntry(){
			info[0]   = 0;
			info[1]   = 0;
			intersect = 0;
		}
	};

	template<typename T>
	class Table : public vector<T>{
	public:
		int nrow;
		int ncol;

		void Resize(int _nrow, int _ncol, T _val = T()){
			resize(_nrow*_ncol, _val);
			nrow = _nrow;
			ncol = _ncol;
		}

		T& operator()(int r, int c){
			return vector<T>::at(ncol*r + c);
		}
	};

	class GPTable    : public Table<GPTableEntry>{};
	class AvoidTable : public Table<AvoidTask*>{};

	EdgeInfos                          edgeInfos[3];   ///< edge infos of all objects in x,y,z
	AvoidTable                         avoidTable;
	vector< std::set<GeometryInfo*> >  queue[3];
	GPTable                            gpTable;
	
	void ExtractGeometryPairs();
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
