#include <module/simulation.h>
#include <module/rendering.h>
#include <module/module.h>

#include <Foundation/UTQPTimer.h>

UTQPTimer ptimer;

SimulationManager::SimulationManager(){
	timerPeriod	= 100;
}

void SimulationManager::Read(XML& xml){
	XMLNode* renNode = xml.GetRootNode()->GetNode("simulation", 0, false);
	if(!renNode){
		Message::Error("simulation manager: xml node not found");
		return;
	}
	renNode->Get(timerPeriod, ".timer_period");
}

bool SimulationManager::Init(){
	Module* mod = Module::Get();

	evPlanning  .Create(true);

	evGroup.Add(&mod->evExit);
	evGroup.Add(&evStartPlan);
	evGroup.Add(&evStopPlan );
	
	return true;
}

void SimulationManager::StepPlan(){
	Module* mod = Module::Get();
	CriticalSection _cs(&mod->cs);

	mod->OnStep();
}

void SimulationManager::OnEvent(SDL_Event* ev){
	if(ev->type == SDL_USEREVENT){
		if(ev->user.code == (int)this){
		}
	}
}

void SimulationManager::Func(){
	Module* mod = Module::Get();
	//vector<Event*> ev;
	//ev.push_back(&mod->evExit);
	//ev.push_back(&evStartPlan);
	//ev.push_back(&evStopPlan );

	while(true){
		//int i = Event::Wait(&ev[0], (uint)ev.size(), (evPlanning.IsSet() ? 10 : 1000), false);
		int i = evGroup.Wait(evPlanning.IsSet() ? 10 : 1000);
		cout << "sim wait break " << i << endl;
		if(i == -1){
			if(evPlanning.IsSet()){
				cout << "step plan" << endl;
				StepPlan();
			}
			continue;
		}
		if(evGroup[i] == &evStartPlan) evPlanning.Set  ();
		if(evGroup[i] == &evStopPlan ) evPlanning.Reset();
		if(evGroup[i] == &mod->evExit)
			break;
	}
}
