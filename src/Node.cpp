#include <DiMP2/Node.h>
#include <DiMP2/Graph.h>
#include <DiMP2/Solver.h>
#include <DiMP2/Timing.h>
#include <DiMP2/DrawConfig.h>

namespace DiMP2{;

//-------------------------------------------------------------------------------------------------

Tick::Tick(Graph* g, real_t t, const string& n):Node(g, n){
	time = t;
	graph->ticks.Add(this);
}

//-------------------------------------------------------------------------------------------------

void Ticks::AssignIndices(){
	for(uint i = 0; i < size(); i++){
		at(i)->idx = i;
	}
}

void Ticks::Add(Tick* tick){
	ArrayBase<Tick*>::Add(tick);
	sort(begin(), end(), Tick::CompareByTime());
	AssignIndices();
}

void Ticks::Remove(Tick* tick){
	ArrayBase<Tick*>::Remove(tick);
	AssignIndices();
}

//-------------------------------------------------------------------------------------------------

Node::Node(Graph* g, const string& n){
	graph   = g;
	type    = -1;
	name	= n;

	graph->nodes.Add(this);
	graph->ready = false;
}

Node::~Node(){
	graph->nodes.Remove(this);
	graph->ready = false;
}
	
//-------------------------------------------------------------------------------------------------

void Trajectory::Update(){
	for(uint i = 0; i < size(); i++){
		Keypoint* key = at(i);
		if(key->prev) key->hprev = key->tick->time - key->prev->tick->time;
		if(key->next) key->hnext = key->next->tick->time - key->tick->time;
	}
}

void Trajectory::Set(const Ticks& ticks, TrajectoryNode* node){
	clear();
	for(uint i = 0; i < ticks.size(); i++)
		push_back(node->CreateKeypoint());
		
	// name keypoints [node name]_i
	stringstream ss;
	for(uint i = 0; i < ticks.size(); i++){
		ss.str("");
		ss << node->name << '_' << i;
		at(i)->name = ss.str();
	}

	// then link them together
	for(uint i = 0; i < size(); i++){
		Keypoint* key = at(i);
		key->node = node;
		key->tick = ticks[i];
		key->prev = (i > 0		  ? (*this)[i-1] : NULL);
		key->next = (i < size()-1 ? (*this)[i+1] : NULL);
	}

	Update();
}

void Trajectory::AddVar(Solver* solver){
	for(uint i = 0; i < size(); i++)
		at(i)->AddVar(solver);
}
void Trajectory::AddCon(Solver* solver){
	for(uint i = 0; i < size(); i++)
		at(i)->AddCon(solver);
}
void Trajectory::Prepare(){
	Update();
	for(uint i = 0; i < size(); i++)
		at(i)->Prepare();
}
void Trajectory::Draw(DrawCanvas* canvas, DrawConfig* conf){
	for(uint i = 0; i < size(); i++)
		at(i)->Draw(canvas, conf  );
}

//-------------------------------------------------------------------------------------------------

TrajectoryNode::TrajectoryNode(Graph* g, const string& n):Node(g, n){
	graph->trajNodes.Add(this);
}

TrajectoryNode::~TrajectoryNode(){
	graph->trajNodes.Remove(this);
}

void TrajectoryNode::AddVar(){
	traj.AddVar(graph->solver);
}
void TrajectoryNode::AddCon(){
	traj.AddCon(graph->solver);
}
void TrajectoryNode::Prepare(){
	traj.Prepare();
}
void TrajectoryNode::Draw(DrawCanvas* canvas, DrawConfig* conf){
	traj.Draw(canvas, conf);
}
void TrajectoryNode::AddKeypoints(){
	traj.Set(graph->ticks, this);
}

//-------------------------------------------------------------------------------------------------

ScheduledKey::ScheduledKey(){
	relation = Outside;
}

void ScheduledKey::CalcRelation(){
	// タイムスロット未指定（全区間）ならInside判定
	if(!((ScheduledNode*)node)->time){
		relation = Inside;
		return;
	}

	int idx = tick->idx;
	int idx_s = ((ScheduledNode*)node)->key_s->tick->idx;
	int idx_e = ((ScheduledNode*)node)->key_e->tick->idx;

	if(idx == idx_s)
		 relation = Start;
	else if(idx == idx_e)
		 relation = End;
	else if(idx_s < idx && idx < idx_e)
		 relation = Inside;
	else relation = Outside;
}

//-------------------------------------------------------------------------------------------------

ScheduledNode::ScheduledNode(Graph* g, TimeSlot* _time, const string& n):TrajectoryNode(g, n){
	time = _time;
}

void ScheduledNode::Prepare(){
	if(time){
		// 開始時刻の手前の点
		KeyPair kp;
		kp    = traj.GetSegment(time->time_s->val);
		key_s = (ScheduledKey*)kp.first;
		// 終了時刻の次の点
		kp    = traj.GetSegment(time->time_e->val);
		key_e = (ScheduledKey*)kp.second;
	}
	else{
		key_s = (ScheduledKey*)traj.GetKeypoint(graph->ticks.front());
		key_e = (ScheduledKey*)traj.GetKeypoint(graph->ticks.back ());
	}

	for(uint i = 0; i < traj.size(); i++)
		((ScheduledKey*)traj.GetKeypoint(i))->CalcRelation();

	TrajectoryNode::Prepare();
}

}
