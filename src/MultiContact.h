#pragma once

#include <DiMP2/Object.h>

namespace DiMP2{;

class MCObject;
class ContactTask;
class ContactTaskKey;
class BalanceTask;
struct MCConTP;
struct MCConTV;
struct MCForceConT;
struct MCForceConR;

//-------------------------------------------------------------------------------------------------

class MCCallback{
public:
	MCObject*	node;	///< 逆参照用

	virtual uint NumJoints     () = 0;
	virtual uint NumEndpoints  () = 0;
	virtual bool IsRotational  (uint j) = 0;
	virtual void Calc          (const real_t* pos) = 0;
	virtual void GetEndpointPos(vec3_t& p , uint i) = 0;
	virtual void GetJacobian   (vec3_t& Jv, uint i, uint j) = 0;
	virtual void Draw          (GRRenderIf* render, DrawConfig* conf){}
};

/**
	object with multiple contacts
 */
class MCObjectKey : public ObjectBaseKey{
public:
	MCForceConT*	con_force_t;
	MCForceConR*	con_force_r;

	// 関節の情報
	struct JointInfo{
		SVar*			pos;
		SVar*			vel;
	};
	
	// 端点の情報
	struct EndInfo{
		vec3_t			prel;			///< contact position w.r.t. body
		vector<vec3_t>	J;				///< Jacobian from joint angles to end points

		V3Var*			pos;
		V3Var*			vel;
		C1ConV3*		con_c1;
		MCConTP*		con_tp;
		MCConTV*		con_tv;
	};

	vector<JointInfo  >		joint;
	vector<EndInfo>			end;
	vector<ContactTaskKey*>	contact;

public:
	MCObject* GetNode(){ return (MCObject*)node; }
	virtual void AddVar(Solver* solver);
	virtual void AddCon(Solver* solver);
	virtual void Prepare();
	virtual void Draw(GRRenderIf* render, DrawConfig* conf);
};

typedef pair<MCObjectKey*, MCObjectKey*>	MCObjectKeyPair;

//-------------------------------------------------------------------------------------------------

class MCObject : public ObjectBase{
public:
	Joint::Param	jparam;
	
	MCCallback*		callback;
	
	vector<ContactTask*>	contact;

	MCObjectKey*	GetKeypoint(Tick* tick){ return TrajectoryNode::GetKeypoint<MCObjectKey>(tick); }
	MCObjectKeyPair	GetSegment(real_t t)   { return TrajectoryNode::GetSegment <MCObjectKey>(t); }
	
	virtual Keypoint*	CreateKeypoint(){ return new MCObjectKey(); }
	virtual void		Init();
	virtual void		Prepare();

	void SetCallback       (MCCallback* cb);			///< コールバックを登録
	uint NumJoints         ();
	uint NumEndpoints      ();
	uint NumContacts       ();
	bool IsRotational      (uint i);
	void SetInitialJointPos(uint j, real_t pos);		///< 関節角度を設定
	void Draw              (GRRenderIf* render, DrawConfig* conf);
	void DrawSnapshot      (real_t time       , GRRenderIf* render, DrawConfig* conf);
	void DrawSnapshot      (const pose_t& pose, GRRenderIf* render, DrawConfig* conf);

public:
	MCObject(Graph* g, string n);
	virtual ~MCObject(){}
};

//-------------------------------------------------------------------------------------------------

class ContactTaskKey : public TaskBaseKey{
public:
	MCObjectKey*	obj;

	SVar*			force_n;
	SVar*			force_t[2];

	RangeConS*		con_force_n;		///< normal force must be positive
	FrictionCon*	con_force_t;		///< tagential force must be inside friction cone
	MatchConV3*		con_pos;
	FixConV3*		con_vel;

	//DistanceConV3*	con_pos_dist;
	//FixToPlaneCon*	con_pos;			///< contact position must be on contact plane

public:
	ContactTask* GetNode(){ return (ContactTask*)node; }
	virtual void AddVar (Solver* solver);
	virtual void AddCon (Solver* solver);
	virtual void Prepare();
	virtual void Draw   (GRRenderIf* render, DrawConfig* conf);

	ContactTaskKey();
};
typedef pair<ContactTaskKey*, ContactTaskKey*>	ContactTaskKeyPair;

class ContactTask : public TaskBase{
public:
	struct Param{
		real_t mu;
		vec3_t normal;
		vec3_t tangent[2];
		real_t dmin;
		real_t dmax;

		Param();
	};
	Param   param;

	V3Var*	        pos;
	FixConPlane*    con_pos_plane;

public:
	MCObject*		obj;
	uint			idx;
	
public:
	ContactTaskKey*	   GetKeypoint(Tick* tick){ return TrajectoryNode::GetKeypoint<ContactTaskKey>(tick); }
	ContactTaskKeyPair GetSegment (real_t t)  { return TrajectoryNode::GetSegment <ContactTaskKey>(t   ); }
	
	virtual Keypoint*  CreateKeypoint(){ return new ContactTaskKey(); }

	virtual void       Prepare();
	virtual void       AddVar();
	virtual void       AddCon();
	virtual void       Draw(GRRenderIf* render, DrawConfig* conf); 

public:
	ContactTask(Graph* g, string name, MCObject* _obj, uint _idx, TimeSlot* _time);
};

//-------------------------------------------------------------------------------------------------

class BalanceTaskKey : public TaskBaseKey{
public:
	MCObjectKey*	obj;

	FixConV3*   con_fix_pos_t;		///< desired posture
	FixConV3*	con_fix_vel_t;		///< desired velocity

public:
	BalanceTask* GetNode(){ return (BalanceTask*)node; }
	virtual void AddVar(Solver* solver);
	virtual void AddCon(Solver* solver);
	virtual void Prepare();

	BalanceTaskKey();
};
typedef pair<BalanceTaskKey*, BalanceTaskKey*>	BalanceTaskKeyPair;

class BalanceTask : public TaskBase{
public:
	MCObject*		obj;

	BalanceTaskKey*	   GetKeypoint(Tick* tick){ return TrajectoryNode::GetKeypoint<BalanceTaskKey>(tick); }
	BalanceTaskKeyPair GetSegment (real_t t)  { return TrajectoryNode::GetSegment <BalanceTaskKey>(t   ); }
	
	virtual Keypoint*	CreateKeypoint(){ return new BalanceTaskKey(); }
	
	BalanceTask(Graph* g, string name, MCObject* _obj, TimeSlot* _time);
};

//-------------------------------------------------------------------------------------------------

struct MCConTP : Constraint{
	MCObjectKey*	obj;
	uint idx;	///< contact point index

	virtual void CalcCoef();
	virtual void CalcDeviation();
	MCConTP(Solver* solver, string _name, MCObjectKey* _obj, uint i, real_t _scale);
};
struct MCConTV : Constraint{
	MCObjectKey*	obj;
	uint idx;	///< contact point index

	virtual void CalcCoef();
	MCConTV(Solver* solver, string _name, MCObjectKey* _obj, uint i, real_t _scale);
};
struct MCForceConT : Constraint{
	MCObjectKey*	obj[2];

	virtual void CalcCoef();
	virtual void CalcDeviation();
	MCForceConT(Solver* solver, string _name, MCObjectKey* _obj, real_t _scale);
};
struct MCForceConR : Constraint{
	MCObjectKey*	obj;

	virtual void CalcCoef();
	MCForceConR(Solver* solver, string _name, MCObjectKey* _obj, real_t _scale);
};


}
