#pragma once

#include <DiMP2/ID.h>
#include <DiMP2/Node.h>
#include <DiMP2/Tree.h>
#include <DiMP2/Connector.h>

namespace DiMP2{;

class Geometry  ;
class TimeSlot  ;
class Object    ;
class BipedLIP  ;
class Joint     ;
class Task      ;
class DrawConfig;
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
		real_t		time,		time_inv;
		real_t		size,		size_inv;
		real_t		mass,		mass_inv;
		real_t		inertia,	inertia_inv;

		real_t		pos_t,	pos_t_inv;
		real_t		vel_t,	vel_t_inv;
		real_t		acc_t,	acc_t_inv;
	
		real_t		pos_r,	pos_r_inv;
		real_t		vel_r,	vel_r_inv;
		real_t		acc_r,	acc_r_inv;
	
		real_t		force_t, force_t_inv;
		real_t		force_r, force_r_inv;

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
	
	UTRef<Solver>	   solver;				///< internal solver
	UTRef<DrawConfig>  drawConf;			///< default draw configuration

	Param		 param;
	Scale		 scale;
	
	void Prepare();

public:
	/// set scaling factor;
	void	SetScaling(real_t T, real_t L, real_t M);
	
	/** @brief	enable or disable constraints
		@param	mask	constraint id mask
		@param	enable	enable or disable
		@return	number of matches
	 */
	int Enable(ID mask, bool enable = true);

	/** @brief  lock or unlock variables
		@param	mask	variable id mask
		@param	lock	lock or unlock
		@return	number of matches
	 */
	int Lock(ID mask, bool lock = true);

	/** @brief	set priority level
		@param	mask	constraint id mask
		@param	level	priority level
		@return	number of matches

		level 0 has the largest priority
	 */
	int SetPriority(ID mask, uint level);

	/** @brief	set correction rate
	 */
	int SetCorrectionRate(ID mask, real_t rate, real_t lim = FLT_MAX);
	
	/** @brief calculate constraint error
		@param mask			constraint id mask
		@param sum_or_max	if true, sum of constraint errors is returned. otherwise the maximum is returned.
		@param abs_or_rel	if true, absolute error is calculated. otherwise relative error is calculated.

		id‚É‡’v‚·‚éS‘©‚É‚Â‚¢‚ÄCabs_or_rel‚É‚µ‚½‚ª‚Á‚Äâ‘ÎŒë·‚ ‚é‚¢‚Í‘Š‘ÎŒë·‚ðŒvŽZ‚µC
		sum_or_max‚É‚µ‚½‚ª‚Á‚Ä‚»‚ê‚ç‚Ì‘˜a‚ ‚é‚¢‚ÍÅ‘å’l‚ð•Ô‚·D

		‚½‚¾‚µâ‘ÎŒë·‚Æ‚ÍŒë·ƒxƒNƒgƒ‹‚Ìƒmƒ‹ƒ€C‘Š‘ÎŒë·‚Æ‚ÍŒë·ƒxƒNƒgƒ‹‚Ìƒmƒ‹ƒ€‚ðCS‘©‚³‚ê‚é•Ï”‚Ìƒmƒ‹ƒ€‚Ì˜a‚ÅŠ„‚Á‚½‚à‚ÌD
	 */
	real_t CalcError(ID mask, bool sum_or_max);

	/// calculate average or max of variables
	real_t CalcVariable(ID mask, bool ave_or_max);

	/// does initialization
	virtual void Init();

	/// clears all objects
	virtual void Clear();

	/// resets all variables. objects are not deleted
	virtual void Reset();

	/// resets all variables. and do extra initialization
	//virtual void Reset2();

	/** 
		computes one step of planning
	 **/
	virtual void Step();

	/// visualize the plan
	virtual void Draw(DrawCanvas* canvas, DrawConfig* conf = 0);

	/// draw snapshot
	virtual void DrawSnapshot(real_t time, DrawCanvas* canvas, DrawConfig* conf = 0);


	Graph();
	virtual ~Graph(){}
};

}
