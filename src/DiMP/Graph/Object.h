#pragma once

#include <DiMP/Graph/Node.h>
#include <DiMP/Graph/Connector.h>
//#include <DiMP/Solver/Constraint.h>
#include <DiMP/Render/Canvas.h>
#include <DiMP/Render/Config.h>

namespace DiMP{;

class  Graph;
class  Object;
class  Tree;
class  TreeKey;
class  Joint;
class  JointKey;
class  Geometry;

class  ForceConT;
class  ForceConR;
class  ObjectConC1R;

/**
	object (rigid body)
 */

class GeometryInfo{
public:
	Connector* con;
	Geometry*  geo;
	pose_t     poseAbs;
	vec3_t     bsphereCenterAbs;
	vec3_t     bbmin;
	vec3_t     bbmax;
};

class ObjectKey : public Keypoint{
public:
	struct OptionS{
		bool    tp, rp, tv, rv;
		vec3_t  k_tp, k_rp, k_tv, k_rv;
	};
	struct OptionV3{
		bool    tp, rp, tv, rv;
		real_t  k_tp, k_rp, k_tv, k_rv;
	};

	typedef vector< pair<JointKey*, bool> >	JointKeys;
	JointKeys	joints;		///< 接続しているJointのKeypoint, ソケット側true
	TreeKey*	tree;

	vector<GeometryInfo>  geoInfos;

	/// variables
	V3Var*		pos_t;				///< position
	V3Var*		vel_t;				///< velocity
	QVar*		pos_r;				///< orientation
	V3Var*		vel_r;				///< ang. velocity

	vec3_t		fext_t;				///< external force
	vec3_t		fext_r;				///< external moment

	C1ConV3*		con_c1_t;		///< C^1 constraint for position spline
	ObjectConC1R*	con_c1_r[3];	///< C^1 constraint for orientation spline (one for each axis)

	ForceConT*		con_force_t;	/// sum of force constraint
	ForceConR*		con_force_r;	/// sum of moment constraint

	/* ツリーに属しているかを考慮してリンクと係数を設定する
		t_or_r : translational or rotational
		p_or_v : position or velocity
		s_or_r : v3 constraint or scalar constraint
	 */
	void AddLinks(Constraint* con, const OptionS & opt);
	void AddLinks(Constraint* con, const OptionV3& opt);
	void CalcCoef(Constraint* con, const OptionS & opt, uint& i);
	void CalcCoef(Constraint* con, const OptionV3& opt, uint& i);

	void PrepareGeometry();

public:
	virtual void AddVar (Solver* solver);
	virtual void AddCon (Solver* solver);
	virtual void Prepare();
	virtual void Draw   (Render::Canvas* canvas, Render::Config* conf);	
};

class Object : public TrajectoryNode{
public:
	// 物理属性
	struct Param{
		real_t			mass;			///< mass
		real_t			inertia;		///< inertia (approximated as a scalar)
		bool			dynamical;		///<
		vec3_t			iniPos;
		quat_t			iniOri;
		vec3_t			iniVel;
		vec3_t			iniAngvel;

		Param(){
			mass      = 1.0;
			inertia   = 1.0;
			dynamical = true;
		}
	};

	struct Snapshot{
		real_t time;
		vec3_t pos;
		quat_t ori;
		vec3_t vel;
		vec3_t angvel;
	};
	
	Param		param;

	Snapshot	snapshot;

	Connectors	cons;

	real_t		bsphere;		///< bounding sphere radius

	Tree*		tree;			///< a tree this object belongs to
	int         treeIndex;      ///< index of this object in the tree

public:	
	void	CalcBSphere();
	/** @brief interpolation functions
		@param	t		time
		@param	spline	if true, cubic spline interpolation is used. otherwise, quadratic interpolation is used.
	 **/
	vec3_t		Pos   (real_t t, int type = Interpolate::Quadratic);
	vec3_t		Vel   (real_t t, int type = Interpolate::Quadratic);
	vec3_t		Acc   (real_t t, int type = Interpolate::Quadratic);
	quat_t		Ori   (real_t t, int type = Interpolate::SlerpDiff);
	vec3_t		Angvel(real_t t, int type = Interpolate::Quadratic);
	vec3_t		Angacc(real_t t, int type = Interpolate::Quadratic);

	/// draw position trajectory as a curve
	void DrawTrajectory(Render::Canvas* canvas, uint ndiv = 100);
	
	virtual Keypoint*	CreateKeypoint(){ return new ObjectKey(); }
	virtual void		Init();
	virtual void		Prepare();
	virtual void        Draw(Render::Canvas* canvas, Render::Config* conf);
	
	/// virtual functions of TrajectoryNode
	virtual void        CreateSnapshot(real_t t);
	virtual void		DrawSnapshot(Render::Canvas* canvas, Render::Config* conf);
	
public:
	/// compute forward kinematics
	void ForwardKinematics();
	void ForwardKinematics(real_t time);

	/// draw object in given pose
	void DrawSnapshot(const pose_t& pose, Render::Canvas* canvas, Render::Config* conf);
	
	Object(Graph* g, const string& n = "");
	virtual ~Object();
};

//-------------------------------------------------------------------------------------------------

class ObjectConC1R : public Constraint{
public:
	ObjectKey*	obj[2];
	vec3_t		r[2];
	int			idx;		///< which rotation axis to constrain, 0, 1, or 2
	
	virtual void CalcCoef();
	virtual void CalcDeviation();

	ObjectConC1R(Solver* solver, const string& _name, ObjectKey* _obj, int _idx, real_t _scale);
};

//-------------------------------------------------------------------------------------------------

class ForceConT : public Constraint{
public:
	ObjectKey*	obj[2];

	virtual void CalcCoef();
	virtual void CalcDeviation();

	ForceConT(Solver* solver, const string& _name, ObjectKey* obj, real_t _scale);
};

class ForceConR : public Constraint{
public:
	ObjectKey*	obj[2];
	
	virtual void CalcCoef();
	virtual void CalcDeviation();

	ForceConR(Solver* solver, const string& _name, ObjectKey* obj, real_t _scale);
};

}
