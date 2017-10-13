#pragma once

#include <module/manager.h>

class SimulationManager : public Manager{
public:
	int			timerPeriod;		///< �V�~�����[�V�����^�C�}����[ms]
	real_t		timeStep;			///< �V�~�����[�V�����̃X�e�b�v��
		
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
