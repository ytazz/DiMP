#include <DiMP/Graph/Task.h>
#include <DiMP/Graph/Object.h>
#include <DiMP/Graph/Graph.h>
#include <DiMP/Render/Config.h>

namespace DiMP{;

//-------------------------------------------------------------------------------------------------
// TaskKey

TaskKey::TaskKey(){
}

void TaskKey::AddVar(Solver* solver){
	// get object keypoints to constrain
	obj0 = (ObjectKey*)((Task*)node)->obj0->traj.GetKeypoint(tick);
	obj1 = (ObjectKey*)((Task*)node)->obj1->traj.GetKeypoint(tick);
}

void TaskKey::Draw(Render::Canvas* canvas){
	/*Vec3f p0_s, p0_e, p1_s, p1_e;

	p0_s = obj0->GetNode()->Pos( GetNode()->time->time_s->val, Interpolate::Quadratic);
	p0_e = obj0->GetNode()->Pos( GetNode()->time->time_e->val, Interpolate::Quadratic);
	p1_s = obj1->GetNode()->Pos( GetNode()->time->time_s->val, Interpolate::Quadratic);
	p1_e = obj1->GetNode()->Pos( GetNode()->time->time_e->val, Interpolate::Quadratic);

	if(obj_no == 0)
		render->draw->Line(p0_s, p0_e);
	if(obj_no == 1)
		draw->Line(p1_s, p1_e);
	if(obj_no == 1){
		draw->Line(p0_s, p0_e);
		draw->Line(p1_s, p1_e);
	}*/
}

//-------------------------------------------------------------------------------------------------
// Task

Task::Task(Object* _obj0, Object* _obj1, TimeSlot* _time, const string& n) : ScheduledNode(_obj0->graph, _time, n){
	obj0 = _obj0;
	obj1 = _obj1;

	graph->tasks.Add(this);

	type = Type::Task;
}

Task::~Task(){
	graph->tasks.Remove(this);
}

}
