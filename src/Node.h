#pragma once

#include <DiMP2/Types.h>

namespace DiMP2{;

class SVar;
class Solver;
class Graph;
class DrawCanvas;
class DrawConfig;

/**
	graph node base class
 */

class Node : public UTRefCount{
public:
	string		name;
	int         type;
	Graph*		graph;

	struct CompareByType{
		bool operator()(const Node* lhs, const Node* rhs){
			return lhs->type < rhs->type;
		}
	};
	
public:
	/// add variables
	virtual void AddVar(){}

	/// add constraints
	virtual void AddCon(){}

	/// do extra initialization
	virtual void Init(){}

	/// preparation before each iteration
	virtual void Prepare(){}

	/// draw
	virtual void Draw(DrawCanvas* canvas, DrawConfig* conf){}

	Node(Graph* g, const string& n);
	virtual ~Node();
};

template<class T>
class ArrayBase : public vector<T>{
public:
	void Add(T v){
		push_back(v);
	}
	void Remove(T v){
		erase(find(begin(), end(), v));
	}
	T Find(const string& name){
		if(name.empty())
			return 0;
		for(uint i = 0; i < size(); i++){
			if(at(i)->name == name)
				return at(i);
		}
		return 0;
	}
};

class NodeArray : public ArrayBase< UTRef<Node> >{
public:
	void AddVar (){ for(uint i = 0; i < size(); i++) at(i)->AddVar (); }
	void AddCon (){ for(uint i = 0; i < size(); i++) at(i)->AddCon (); }
	void Init   (){ for(uint i = 0; i < size(); i++) at(i)->Init   (); }
	void Prepare(){ for(uint i = 0; i < size(); i++) at(i)->Prepare(); }
	void Draw(DrawCanvas* canvas, DrawConfig* conf){ for(uint i = 0; i < size(); i++) at(i)->Draw(canvas, conf); }
};

/**
	tick : 軌道を表現するスプライン曲線の離散時刻
 */
class Tick : public Node{
public:
	struct CompareByTime{
		bool operator()(const Tick* lhs, const Tick* rhs){
			return lhs->time < rhs->time;
		}
	};

	real_t		time;		///< time instant
	uint		idx;		///< index in tick sequence

public:
	
	Tick(Graph* g, real_t t, const string& n = "");
};

class Ticks : public ArrayBase<Tick*>{
	void	AssignIndices();
public:
	void	Add   (Tick* tick);
	void	Remove(Tick* tick);
};

/**
	base class of trajectory keypoint
 */
class Keypoint : public UTRefCount{
public:
	string		name;
	Node*		node;			///< reference to owner node
	Tick*		tick;			///< time instant at which this keypoint is put
	real_t		hprev, hnext;	///< time interval between adjacent keypoints
	Keypoint	*prev, *next;	///< reference to adjacent keypoints	
	
public:
	/// add variables
	virtual void AddVar(Solver* solver){}
	/// add constraints
	virtual void AddCon(Solver* solver){}
	/// preparation before each iteration
	virtual void Prepare(){}
	/// draw
	virtual void Draw(DrawCanvas* canvas, DrawConfig* conf){}

	Keypoint(){
		tick  = 0;
		hprev = hnext = 0.0;
		prev  = next  = 0;
	}
};

typedef pair<Keypoint*, Keypoint*> KeyPair;

/**
	trajectory: a sequnce of keypoints
 */
class TrajectoryNode;
class Trajectory : public std::vector< UTRef<Keypoint> >{
public:
	/// update length of time between each pair of keypoints
	void Update();

	/// create keypoints
	void Set(const Ticks& ticks, TrajectoryNode* node);
	/// get keypoint
	Keypoint* GetKeypoint(int idx   ){ return at(idx);       }
	Keypoint* GetKeypoint(Tick* tick){ return at(tick->idx); }
	
	/// get segment (pair of keypoints) which includes specified time instant
	KeyPair GetSegment(real_t time){
		int idx;
		for(idx = -1; idx < (int)size()-1; idx++){
			if(time < (*this)[idx+1]->tick->time)
				break;
		}
		if(idx == -1)
			return make_pair(GetKeypoint(0), GetKeypoint(0));
		if(idx == size()-1)
			return make_pair(GetKeypoint(idx), GetKeypoint(idx));
		return make_pair(GetKeypoint(idx), GetKeypoint(idx+1));
	}

	void AddVar(Solver* solver);
	void AddCon(Solver* solver);
	void Prepare();
	void Draw(DrawCanvas* canvas, DrawConfig* conf);
};

/**
	node with trajectory
 **/
class TrajectoryNode : public Node{
public:
	Trajectory	traj;

public:
	virtual void AddVar ();
	virtual void AddCon ();
	virtual void Prepare();
	virtual void Draw(DrawCanvas* canvas, DrawConfig* conf);

	/// create keypoint
	virtual Keypoint* CreateKeypoint() = 0;
	/// draw snapshot at given time
	virtual void DrawSnapshot(real_t time, DrawCanvas* canvas, DrawConfig* conf){}

 	void AddKeypoints();

	TrajectoryNode(Graph* g, const string& n);
	virtual ~TrajectoryNode();
};

class TrajectoryNodeArray : public ArrayBase<TrajectoryNode*>{
public:
	void AddKeypoints(){
		for(uint i = 0; i < size(); i++)
			at(i)->AddKeypoints();
	}
	
	void DrawSnapshot(real_t time, DrawCanvas* canvas, DrawConfig* conf){
		for(uint i = 0; i < size(); i++)
			at(i)->DrawSnapshot(time, canvas, conf);
	}
};

class TimeSlot;

/**
	trajectory node with time slot
 **/
class ScheduledNode;
class ScheduledKey : public Keypoint{
public:
	/// タスク実行区間とこのキーポイントの時刻の関係
	enum{
		Start,		///< 始点の手前
		Inside,		///< 区間の内部
		End,		///< 終点の次
		Outside,	///< 区間の外部
	};
	int	 relation;

public:
	void	CalcRelation();
	
	ScheduledKey();
};

class ScheduledNode : public TrajectoryNode{
public:
	TimeSlot*	    time;
	ScheduledKey*	key_s;
	ScheduledKey*	key_e;

public:
	virtual void Prepare();

	ScheduledNode(Graph* g, TimeSlot* _time, const string& n);
};

}
