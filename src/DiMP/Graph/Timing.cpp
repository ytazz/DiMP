#include <DiMP/Graph/Timing.h>
#include <DiMP/Graph/Graph.h>
#include <DiMP/Render/Config.h>

namespace DiMP{;

TimeSlot::Param::Param(){
	real_t inf = numeric_limits<real_t>::max();
	ts_ini =  0.0;
	te_ini =  0.0;
	ts_min = -inf;
	ts_max =  inf;
	te_min = -inf;
	te_max =  inf;
	T_min  =  0.0;
	T_max  =  inf;
	lock   =  true;
}

TimeSlot::TimeSlot(Graph* g, real_t ts, real_t te, bool lock, const string& n):Node(g, n){
	time_s      = 0;
	time_e      = 0;
	con_range_s = 0;
	con_range_e = 0;
	con_diff    = 0;
	param.ts_ini = ts;
	param.te_ini = te;
	param.lock   = lock;

	graph->timeslots.Add(this);
}

TimeSlot::~TimeSlot(){
	graph->timeslots.Remove(this);
}

void TimeSlot::AddVar(){
	time_s = new SVar(graph->solver, ID(VarTag::TimeStart, this, 0), graph->scale.time);
	time_e = new SVar(graph->solver, ID(VarTag::TimeEnd,   this, 0), graph->scale.time);

	time_s->val    = param.ts_ini;
	time_e->val    = param.te_ini;
	time_s->locked = param.lock;
	time_e->locked = param.lock;
}

void TimeSlot::AddCon(){
	con_range_s = new RangeConS(graph->solver, ID(ConTag::TimeStartRange   , this, 0), time_s, graph->scale.time);
	con_range_e = new RangeConS(graph->solver, ID(ConTag::TimeEndRange     , this, 0), time_e, graph->scale.time);
	con_diff    = new DiffConS (graph->solver, ID(ConTag::TimeDurationRange, this, 0), time_s, time_e, graph->scale.time);

	con_range_s->_min = param.ts_min;
	con_range_s->_max = param.ts_max;
	con_range_e->_min = param.te_min;
	con_range_e->_max = param.te_max;
	con_diff   ->_min = param.T_min ;
	con_diff   ->_max = param.T_max ;
}

}