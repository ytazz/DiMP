#pragma once

#include <DiMP/Graph/Node.h>
//#include <DiMP/Solver/Constraint.h>

namespace DiMP{;

class Graph;
class Object;
class ObjectKey;
class Joint;
class JointKey;

/**
	object (rigid body)
 */

class TreeKey : public Keypoint{
public:
	vector<ObjectKey*>		objects;
	vector<JointKey *>		joints;
	
	vector< vector< vec3_t> >	Jv;		///< Jv[i][j] : j-th dof to body-i velocity
	vector< vector< vec3_t> >   Jw;     ///< Jw[i][j] : j-th dof to body-i ang.velocity
	
public:
	virtual void AddVar(Solver* solver);
	virtual void AddCon(Solver* solver);
	virtual void Prepare();

};

class Tree : public TrajectoryNode{
public:
	Object*					root;
	vector<Object*>			objects;	///< ツリーに属するObject
	vector<Joint*>			joints;		///< ツリーに属するJoint
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

}
