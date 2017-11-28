#include <DiMP/Load/Loader.h>
#include <DiMP/Graph/Graph.h>
#include <DiMP/Graph/Timing.h>
#include <DiMP/Graph/Geometry.h>
#include <DiMP/Graph/Object.h>
#include <DiMP/Graph/Biped.h>
#include <DiMP/Graph/Joint.h>
#include <DiMP/Graph/Avoid.h>
#include <DiMP/Graph/Match.h>
#include <DiMP/Render/Config.h>

namespace DiMP{;

Loader::Loader(){

}

bool Loader::Load(XML& xml, Graph* graph){
	return Load(xml.GetRootNode(), graph);
}

bool Loader::Load(XMLNode* node, Graph* graph){
	//
	string n = node->name;
	if( n == "ticks"      ) LoadTicks   (node, graph);
	if( n == "geometry"   ) LoadGeometry(node, graph);
	if( n == "object"     ) LoadObject  (node, graph);
	if( n == "joint"      ) LoadJoint   (node, graph);
	if( n == "timeslot"   ) LoadTimeSlot(node, graph);
	if( n == "task"       ) LoadTask    (node, graph);
	if( n == "scaling"    ) LoadScaling (node, graph);
	if( n == "enable"     ) LoadEnable  (node, graph);
	if( n == "priority"   ) LoadPriority(node, graph);
	if( n == "param"      ) LoadParam   (node, graph);
	
	for(uint i = 0; ; i++) try{
		XMLNode* child = node->GetNode(i);
		Load(child, graph);
	}
	catch(Exception&){ break; }

	return true;
}

void Loader::LoadTicks(XMLNode* node, Graph* graph){
	real_t timeBegin = 0.0;
	real_t timeEnd   = 0.0;
	real_t timeStep  = 0.0;
	node->Get(timeBegin, ".time_begin");
	node->Get(timeEnd  , ".time_end"  );
	node->Get(timeStep , ".time_step" );
	if(timeBegin >= 0.0 && timeBegin < timeEnd && timeStep > 0.0){
		for(real_t t = timeBegin; t <= timeEnd; t += timeStep)
			new Tick(graph, t);
	}

	for(uint i = 0; ; i++) try{
		XMLNode* child = node->GetNode(i);
		if(child->name == "tick"){
			real_t t;
			child->Get(t, ".t");
			new Tick(graph, t);
		}
	}
	catch(Exception&){ break; }
}

void Loader::LoadGeometry(XMLNode* node, Graph* graph){
	string name;
	string type;
	real_t radius;
	real_t length;
	vec3_t size;
	
	node->Get(name  , ".name"  );
	node->Get(type  , ".type"  );
	node->Get(radius, ".radius");
	node->Get(length, ".length");
	node->Get(size  , ".size"  );
	
	if(type == "sphere"  ) new Sphere  (graph, radius, name);
	if(type == "box"     ) new Box     (graph, size, name);
	if(type == "cylinder") new Cylinder(graph, radius, length, name);
	if(type == "plane"   ) new Plane   (graph, name);
}

void Loader::LoadObject(XMLNode* node, Graph* graph){
	Object* obj = new Object(graph, node->GetName());
	node->Get(obj->param.mass     , ".mass"      );
	node->Get(obj->param.inertia  , ".inertia"   );
	node->Get(obj->param.dynamical, ".dynamical" );
	node->Get(obj->param.iniPos   , ".ini_pos"   );
	node->Get(obj->param.iniOri   , ".ini_ori"   );
	node->Get(obj->param.iniVel   , ".ini_vel"   );
	node->Get(obj->param.iniAngvel, ".ini_angvel");
}

void Loader::LoadJoint(XMLNode* node, Graph* graph){
	string name;
	string type;
	string sockName;
	string plugName;
	string tsName;

	node->Get(name    , ".name"    );
	node->Get(type    , ".type"    );
	node->Get(sockName, ".sock"    );
	node->Get(plugName, ".plug"    );
	node->Get(tsName  , ".timeslot");

	Connector* sock = graph->cons     .Find(sockName);
	Connector* plug = graph->cons     .Find(plugName);
	TimeSlot * ts   = graph->timeslots.Find(tsName  );

	if(sock && plug && ts){
		if(type == "hinge"     ) new Hinge     (sock, plug, ts, name);
		if(type == "slider"    ) new Slider    (sock, plug, ts, name);
		if(type == "balljoint" ) new Balljoint (sock, plug, ts, name);
		if(type == "planejoint") new Planejoint(sock, plug, ts, name);
		if(type == "fixjoint"  ) new Fixjoint  (sock, plug, ts, name);
	}

}

void Loader::LoadTimeSlot(XMLNode* node, Graph* graph){
	string name;
	real_t timeBegin = 0.0;
	real_t timeEnd   = 0.0;
	bool   locked    = false;

	node->Get(name     , ".name"      );
	node->Get(timeBegin, ".time_begin");
	node->Get(timeEnd  , ".time_end"  );
	node->Get(locked   , ".locked"    );

	TimeSlot* ts = new TimeSlot(graph, timeBegin, timeEnd, locked, name);
}

void Loader::LoadTask(XMLNode* node, Graph* graph){
	string name;
	string type;
	string con0Name;
	string con1Name;
	string tsName;

	node->Get(name    , ".name"    );
	node->Get(type    , ".type"    );
	node->Get(con0Name, ".con0"    );
	node->Get(con1Name, ".con1"    );
	node->Get(tsName  , ".timeslot");

	Connector* con0 = graph->cons     .Find(con0Name);
	Connector* con1 = graph->cons     .Find(con1Name);
	TimeSlot * ts   = graph->timeslots.Find(tsName  );

	if(con0 && con1 && ts){
		if(type == "avoid"){
			new AvoidTask(con0, con1, ts, name);
		}
		if(type == "match"){
			MatchTask* task = new MatchTask(con0, con1, ts, name);
			node->Get(task->param.match_tp, ".match_tp");
			node->Get(task->param.match_tv, ".match_tv");
			node->Get(task->param.match_rp, ".match_rp");
			node->Get(task->param.match_rv, ".match_rv");
		}
	}

}

void Loader::LoadScaling(XMLNode* node, Graph* graph){
	real_t T = 1.0;
	real_t L = 1.0;
	real_t M = 1.0;

	node->Get(T, ".T");
	node->Get(L, ".L");
	node->Get(M, ".M");

	graph->SetScaling(T, L, M);
}

int StrToTag(string idstr){
	int tag = -1;
	if(idstr.empty())
		return tag;

	for(uint i = 0; i < VarTag::NumTypes; i++){
		if(idstr == VarNames[i])
			tag = i;
	}
	for(uint i = 0; i < ConTag::NumTypes; i++){
		if(idstr == ConNames[i])
			tag = i;
	}
	return tag;
}

void Loader::LoadEnable(XMLNode* node, Graph* graph){
	string idstr;
	string targetName;
	bool   enable = true;

	node->Get(idstr     , ".id"    );
	node->Get(targetName, ".target");
	node->Get(enable    , ".enable");

	ID id;
	id.tag   = StrToTag(idstr);
	id.owner = graph->nodes.Find(targetName);

	graph->solver->Enable(id, enable);
}

void Loader::LoadPriority(XMLNode* node, Graph* graph){
	string idstr;
	string targetName;
	uint   level = 0;

	node->Get(idstr     , ".id"    );
	node->Get(targetName, ".target");
	node->Get(level     , ".level" );

	ID id;
	id.tag   = StrToTag(idstr);
	id.owner = graph->nodes.Find(targetName);

	graph->solver->SetPriority(id, level);
}

void Loader::LoadCorrection(XMLNode* node, Graph* graph){
	string idstr;
	string targetName;
	real_t rate  = 1.0;
	real_t limit = FLT_MAX;

	node->Get(idstr     , ".id"    );
	node->Get(targetName, ".target");
	node->Get(rate      , ".rate"  );
	node->Get(limit     , ".limit" );

	ID id;
	id.tag   = StrToTag(idstr);
	id.owner = graph->nodes.Find(targetName);

	graph->solver->SetCorrectionRate(id, rate, limit);
}

void Loader::LoadParam(XMLNode* node, Graph* graph){
	string name, value;
	node->Get(name , ".name" );
	node->Get(value, ".value");
	
	string str;
	if(name == "verbose"){
		node->Get(graph->solver->param.verbose, ".value");
	}
	if(name == "method_major"){
		node->Get(str, ".value");
		if(str == "steepest_descent") graph->solver->param.methodMajor = Solver::Method::Major::SteepestDescent;
		if(str == "gauss_newton1"   ) graph->solver->param.methodMajor = Solver::Method::Major::GaussNewton1;
		if(str == "gauss_newton2"   ) graph->solver->param.methodMajor = Solver::Method::Major::GaussNewton2;
		if(str == "prioritized"     ) graph->solver->param.methodMajor = Solver::Method::Major::Prioritized;
	}
	if(name == "method_minor"){
		node->Get(str, ".value");
		if(str == "direct"      ) graph->solver->param.methodMinor = Solver::Method::Minor::Direct;
		if(str == "jacobi"      ) graph->solver->param.methodMinor = Solver::Method::Minor::Jacobi;
		if(str == "gauss_seidel") graph->solver->param.methodMinor = Solver::Method::Minor::GaussSeidel;
	}
	if(name == "num_iter"){
		vvec_t v;
		node->Get(v, ".value");
		graph->solver->param.numIter.resize(v.size());
		for(int i = 0; i < (int)v.size(); i++)
			graph->solver->param.numIter[i] = (int)v[i];
	}
	if(name == "min_step_size"){
		node->Get(graph->solver->param.minStepSize, ".value");
	}
	if(name == "max_step_size"){
		node->Get(graph->solver->param.maxStepSize, ".value");
	}
	if(name == "cutoff_step_size"){
		node->Get(graph->solver->param.cutoffStepSize, ".value");
	}
}

}
