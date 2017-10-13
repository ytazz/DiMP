#pragma once

#include <module/manager.h>

class SimulationManager : public Manager{
public:
	int			timerPeriod;		///< シミュレーションタイマ周期[ms]
	real_t		timeStep;			///< シミュレーションのステップ幅
		
	int			count;
	int         time;

	int         planTime;

	Event		evStartPlan;
	Event		evStopPlan;
	Event		evPlanning;
	
public:
	virtual void Func();

	void StepPlan    ();
	
	void Read    (XML& xml);
	bool Init    ();
	void Step    ();
	void OnEvent (SDL_Event* ev);
	
	SimulationManager();
};
