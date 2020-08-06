#include <DiMP/Graph/Node.h>
#include <DiMP/Graph/Graph.h>
#include <DiMP/Graph/Timing.h>
#include <DiMP/Render/Config.h>

#include <omp.h>

namespace DiMP{;

//-------------------------------------------------------------------------------------------------

Tick::Tick(Graph* g, real_t t, const string& n):Node(g, n){
	time = t;
	graph->ticks.Add(this);
}

//-------------------------------------------------------------------------------------------------

void Ticks::AssignIndices(){
	for(int i = 0; i < (int)size(); i++){
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
	for(Keypoint* key : *this){
		if(key->prev) key->hprev = key->tick->time - key->prev->tick->time;
		if(key->next) key->hnext = key->next->tick->time - key->tick->time;
	}
}

void Trajectory::Set(const Ticks& ticks, TrajectoryNode* node){
	clear();
	for(Tick* t : ticks)
		push_back(node->CreateKeypoint());
		
	// name keypoints [node name]_i
	stringstream ss;
	for(int i = 0; i < (int)ticks.size(); i++){
		ss.str("");
		ss << node->name << '_' << i;
		at(i)->name = ss.str();
	}

	// then link them together
	for(int i = 0; i < (int)size(); i++){
		Keypoint* key = at(i);
		key->node = node;
		key->tick = ticks[i];
		key->prev = (i > 0		  ? (*this)[i-1] : NULL);
		key->next = (i < size()-1 ? (*this)[i+1] : NULL);
	}

	Update();
}

void Trajectory::AddVar(Solver* solver){
	for(Keypoint* key : *this)
		key->AddVar(solver);
}
void Trajectory::AddCon(Solver* solver){
	for(Keypoint* key : *this)
		key->AddCon(solver);
}
void Trajectory::Init(){
	for(Keypoint* key : *this)
		key->Init();
}
void Trajectory::Prepare(){
	Update();
	for(Keypoint* key : *this)
		key->Prepare();
}
void Trajectory::Finish(){
	for(Keypoint* key : *this)
		key->Finish();
}
void Trajectory::Draw(Render::Canvas* canvas, Render::Config* conf){
	for(Keypoint* key : *this)
		key->Draw(canvas, conf);
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
void TrajectoryNode::Finish(){
	traj.Finish();
}
void TrajectoryNode::Draw(Render::Canvas* canvas, Render::Config* conf){
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
