#pragma once

#include <DiMP/Graph/Node.h>
//#include <DiMP/Solver/Constraint.h>

namespace DiMP{;

class Graph;
class Object;
class ObjectKey;
class Joint;
class JointKey;

class EomCon;

/**
	object (rigid body)
 */

class TreeKey : public Keypoint{
public:
	struct Adjacent{
		JointKey*	joint;
		bool		sock;
		int			iobj;

		Adjacent(){}
		Adjacent(JointKey* j, bool s, int i):joint(j), sock(s), iobj(i){}
	};

	vector<ObjectKey*>		objects;
	vector<JointKey *>		joints;
	vector<Adjacent>		adjacents;	///< ツリーに隣接するJoint
	
	vector< vector< vec3_t> >	Jv;		///< Jv[i][j] : joint-j to body-i velocity
	vector< vector< vec3_t> >   Jw;     ///< Jw[i][j] : joint-j to body-i ang.velocity
	vector< vector< real_t> >   M;		///< inertia matrix

	vector<EomCon*>			con_eom;

public:
	int GetIndex(ObjectKey* obj);
	
	virtual void AddVar(Solver* solver);
	virtual void AddCon(Solver* solver);
	virtual void Prepare();

};

class Tree : public TrajectoryNode{
public:
	struct Adjacent{
		Joint*	joint;
		bool	sock;
		int     iobj;

		Adjacent(){}
		Adjacent(Joint* j, bool s, int i):joint(j), sock(s), iobj(i){}
	};

	Object*					root;
	vector<Object*>			objects;	///< ツリーに属するObject
	vector<Joint*>			joints;		///< ツリーに属するJoint
	vector<Adjacent>		adjacents;	///< ツリーに隣接するJoint
	vector<int>             parObject;	///< 
	vector<int>             parJoint;	///< 
	vector< vector<bool> >	dependent;	///< object-iがjoint-jの下流にあるならtrue

	bool	IsDependent(int i, int j);
	
public:
	void Extract();
	
	virtual Keypoint*	CreateKeypoint(){ return new TreeKey(); }
	virtual void		Prepare();
	
public:

	Tree(Graph* g, string n);
	virtual ~Tree();
};

class Trees : public ArrayBase<Tree*>{
public:
	void Extract          ();
	void ForwardKinematics();
};

//-------------------------------------------------------------------------------------------------

class EomCon : public Constraint{
public:
	TreeKey*	tree[2];
	uint		idx;		///< 運動方程式の行
	
	virtual void CalcCoef();
	virtual void CalcDeviation();

	EomCon(Solver* solver, string _name, TreeKey* _tree, uint _idx, real_t _scale);
};


}
