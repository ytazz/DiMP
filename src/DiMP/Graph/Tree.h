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
	struct Adjacent{
		JointKey*	joint;
		bool		sock;
		int			iobj;

		Adjacent(){}
		Adjacent(JointKey* j, bool s, int i):joint(j), sock(s), iobj(i){}
	};

	vector<ObjectKey*>		objects;
	vector<JointKey *>		joints;
	vector<Adjacent>		adjacents;	///< �c���[�ɗאڂ���Joint
	
	vector< vector< vec3_t> >	Jv;		///< Jv[i][j] : joint-j to body-i velocity
	vector< vector< vec3_t> >   Jw;     ///< Jw[i][j] : joint-j to body-i ang.velocity
	vector< vector< real_t> >   M;		///< inertia matrix

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
	vector<Object*>			objects;	///< �c���[�ɑ�����Object
	vector<Joint*>			joints;		///< �c���[�ɑ�����Joint
	vector<Adjacent>		adjacents;	///< �c���[�ɗאڂ���Joint
	vector<int>             parObject;	///< 
	vector<int>             parJoint;	///< 
	vector< vector<bool> >	dependent;	///< object-i��joint-j�̉����ɂ���Ȃ�true

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
