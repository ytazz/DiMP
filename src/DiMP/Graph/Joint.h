#pragma once

#include <DiMP/Graph/Node.h>
//#include <DiMP/Solver/Constraint.h>

namespace DiMP{;

class Graph;
class Object;
class ObjectKey;
class Connector;
class Tree;
class TreeKey;
class Joint;
class Hinge;
class Slider;
class Balljoint;
class Planejoint;
class Fixjoint;
class Genericjoint;
class TimeSlot;

class JointConF;

class JointConTP;
class JointConT1P;
class JointConT2P;
class JointConT3P;

class JointConTV;
class JointConT1V;
class JointConT2V;
class JointConT3V;

class JointConRP;
class JointConR1P;
class JointConR3P;

class JointConRV;
class JointConR1V;
class JointConR3V;

/**
	Joint
 */
class JointKey : public ScheduledKey{
public:
	ObjectKey*		sockObj;		///< connected rigid bodies
	ObjectKey*		plugObj;
	TreeKey*		tree;
	
	vec3_t			r[2];			///< absolute position of socket and plug
	quat_t			q[2];			///< absolute orientation of socket and plug
	vec3_t			rrel;			///< relative position from socket to plug
	quat_t			qrel;			///< relative orientation from socket to plug
	vec3_t          vrel;           ///< relative velocity from socket to plug (in global coord)
	vec3_t          wrel;           ///< relative ang.velocity from socket to plug (in global coord)
	
	V3Var*			force_t;		///< joint translational force in global coord.
	V3Var*			force_r;		///< joint rotational force in global coord.

	JointConTP*		con_tp;
	JointConTV*		con_tv;
	JointConRP*		con_rp;
	JointConRV*		con_rv;

	vector<vec3_t    > Jv         ;	///< joint axis direction in global coord.
	vector<vec3_t    > Jw         ; ///< 
	vector<SVar*     > pos        ;	///< joint position
	vector<SVar*     > vel        ;	///< joint velocity
	vector<SVar*     > torque     ;	///< joint torque
	vector<C1ConS*   > con_c1     ;	///< C1 constraint
	vector<JointConF*> con_force  ;	///< force-torque mapping
	vector<RangeConS*> con_range_p;	///< position range
	vector<RangeConS*> con_range_v;	///< velocity range
	vector<RangeConS*> con_range_f;	///< torque range

public:
	virtual void AddVar (Solver* solver);
	virtual void AddCon (Solver* solver);
	virtual void Prepare();
	virtual void Draw   (Render::Canvas* canvas, Render::Config* conf);

	JointKey();
};

class Joint : public ScheduledNode{
public:
	struct Snapshot{
		vector<real_t>	pos;
		vector<real_t>	vel;
	};

	struct Param{
		vector<real_t>	rmin_p;		///< joint angle lower bound
		vector<real_t>	rmax_p;		///< joint angle upper bound
		vector<real_t>	rmin_v;		///< joint velocity lower bound
		vector<real_t>	rmax_v;		///< joint velocity upper bound
		vector<real_t>	rmin_f;		///< joint torque lower bound
		vector<real_t>	rmax_f;		///< joint torque upper bound
		vector<real_t>	ini_p ;		///< initial angle position
		vector<real_t>	ini_v ;		///< initial angle velocity

		void SetDof(uint dof);
		Param();
	};
	
	Param			param;
	Snapshot	    snapshot;

	Connector*		sock;		///< socket connector
	Connector*		plug;		///< plug connector
	
	Tree*			tree;

	uint			dof;		///< degrees of freedom
	
public:
	void			SetDof(uint n);
	
	/// virtual function of Node
	virtual void	Init();

	///
	virtual void    CreateSnapshot(real_t t);

	/// virtual functions of TrajectoryNode
	virtual void	DrawSnapshot(Render::Canvas* canvas, Render::Config* conf);

	/// returns true if i-th DOF is rotational
	virtual bool	IsRotational(uint i) = 0;
	/// returns true if i-th DOF is translational
	bool			IsTranslational(uint i){ return !IsRotational(i); }

	virtual void	CalcRelativePose(real_t* pos, vec3_t& p , quat_t& q ){}
	virtual void    CalcRelativeVel (real_t* vel, vec3_t& v , vec3_t& w ){}
	virtual void	CalcJointPos    (real_t* pos, vec3_t& p , quat_t& q ){}
	virtual void    CalcJacobian    (real_t* pos, vec3_t* Jv, vec3_t* Jw){}

public:
	/// compute forward kinematics
	void	ForwardKinematics();
	void	ForwardKinematics(real_t t);

	/// calculate joint position from relative pose of objects
	void    ResetJointPos();

	/// interpolation functions
	real_t	Pos(uint i, real_t t, int type = Interpolate::Quadratic);
	real_t	Vel(uint i, real_t t, int type = Interpolate::Quadratic);
	real_t	Acc(uint i, real_t t, int type = Interpolate::Quadratic);

	/// Žžt‚Å‚ÌS‘©Œë·‚ðŒvŽZi•]‰¿—pj
	void CalcDeviation(real_t t, vec3_t& pos_dev, vec3_t& ori_dev);

	/// draw joint for given joint angle. to be overridden by derived classes
	virtual void OnDraw(real_t* pos, Render::Canvas* canvas){}
	
	Joint(Connector* sock, Connector* plug, TimeSlot* time, const string& n);
	virtual ~Joint();
};

/**
	Hinge
	- 1D rotation along z-axis
 */
class HingeKey : public JointKey{
public:
};

class Hinge : public Joint{
public:
	virtual bool IsRotational    (uint i){ return true; }
	virtual void CalcRelativePose(real_t* pos, vec3_t& p , quat_t& q );
	virtual void CalcRelativeVel (real_t* vel, vec3_t& v , vec3_t& w );
	virtual void CalcJointPos    (real_t* pos, vec3_t& p , quat_t& q );
	virtual void CalcJacobian    (real_t* pos, vec3_t* Jv, vec3_t* Jw);
	virtual Keypoint*	CreateKeypoint(){ return new HingeKey(); }

	Hinge(Connector* sock, Connector* plug, TimeSlot* time = 0, const string& n = "");
};

/**
	Slider
	- 1D translation along z-axis
 */
class SliderKey : public JointKey{
public:
};

class Slider : public Joint{
public:
	virtual bool      IsRotational    (uint i){ return false; }
	virtual void	  CalcRelativePose(real_t* pos, vec3_t& p , quat_t& q );
	virtual void      CalcRelativeVel (real_t* vel, vec3_t& v , vec3_t& w );
	virtual void	  CalcJointPos    (real_t* pos, vec3_t& p , quat_t& q );
	virtual void      CalcJacobian    (real_t* pos, vec3_t* Jv, vec3_t* Jw);
	virtual Keypoint* CreateKeypoint  (){ return new SliderKey(); }
	
	Slider(Connector* sock, Connector* plug, TimeSlot* time = 0, const string& n = "");
};

/**
	Spherical joint
	- 3D rotation
 */
class BalljointKey : public JointKey{
public:
};

class Balljoint : public Joint{
public:	
	virtual bool      IsRotational    (uint i){ return true; }
	virtual void      CalcRelativePose(real_t* pos, vec3_t& p , quat_t& q );
	virtual void      CalcRelativeVel (real_t* vel, vec3_t& v , vec3_t& w );
	virtual void      CalcJointPos    (real_t* pos, vec3_t& p , quat_t& q );
	virtual void      CalcJacobian    (real_t* pos, vec3_t* Jv, vec3_t* Jw);
	virtual Keypoint* CreateKeypoint  (){ return new BalljointKey(); }
	
	Balljoint(Connector* sock, Connector* plug, TimeSlot* time = 0, const string& n = "");
};

/**
	Planar joint
	- 2D translation in xy-plane
	- 1D rotation along z-axis
 */
class PlanejointKey : public JointKey{
public:
};

class Planejoint : public Joint{
public:
	virtual bool		IsRotational(uint i){
		if(i == 0 || i == 1)
			return false;
		return true;
	}
	virtual Keypoint* CreateKeypoint  (){ return new PlanejointKey(); }
	virtual void      CalcRelativePose(real_t* pos, vec3_t& p , quat_t& q );
	virtual void      CalcRelativeVel (real_t* vel, vec3_t& v , vec3_t& w );
	virtual void      CalcJointPos    (real_t* pos, vec3_t& p , quat_t& q );
	virtual void      CalcJacobian    (real_t* pos, vec3_t* Jv, vec3_t* Jw);

	Planejoint(Connector* sock, Connector* plug, TimeSlot* time = 0, const string& n = "");
};

/**
	Fix joint
	- simply fix socket and plug together
 */
class FixjointKey : public JointKey{
public:
};

class Fixjoint : public Joint{
public:
	virtual bool		IsRotational(uint i){ return false;	}
	virtual Keypoint*	CreateKeypoint(){ return new FixjointKey(); }
	virtual void		CalcRelativePose(real_t* pos, vec3_t& p, quat_t& q);
	virtual void		CalcRelativeVel (real_t* vel, vec3_t& v, vec3_t& w);

	Fixjoint(Connector* sock, Connector* plug, TimeSlot* time = 0, const string& n = "");
};

/** Generic joint
	- fully customizable joint
 */
class GenericjointCallback{
public:
	virtual uint GetDof          () = 0;
	virtual bool IsRotational    (uint i) = 0;
	virtual void CalcRelativePose(real_t* pos, vec3_t& p , quat_t& q ) = 0;
	virtual void CalcRelativeVel (real_t* vel, vec3_t& v , vec3_t& w ) = 0;
	virtual void CalcJointPos    (real_t* pos, vec3_t& p , quat_t& q ) = 0;
	virtual void CalcJacobian    (real_t* pos, vec3_t* Jv, vec3_t* Jw) = 0;
	virtual void OnDraw          (real_t* pos, Render::Canvas* canvas){}
};

class GenericjointKey : public JointKey{
public:
};

class Genericjoint : public Joint{
public:
	GenericjointCallback*	callback;

public:
	void SetCallback(GenericjointCallback* cb);
	
	virtual bool	  IsRotational    (uint i);
	virtual Keypoint* CreateKeypoint  (){ return new GenericjointKey(); }
	virtual void      CalcRelativePose(real_t* pos, vec3_t& p , quat_t& q );
	virtual void      CalcRelativeVel (real_t* vel, vec3_t& v , vec3_t& w );
	virtual void      CalcJointPos    (real_t* pos, vec3_t& p , quat_t& q );
	virtual void      CalcJacobian    (real_t* pos, vec3_t* Jv, vec3_t* Jw);
	virtual void      OnDraw          (real_t* pos, Render::Canvas* canvas);

	Genericjoint(Connector* sock, Connector* plug, GenericjointCallback* cb, TimeSlot* time = 0, const string& n = "");
};


//-------------------------------------------------------------------------------------------------

/**
	* naming convention:
		T : translational
		R : rotational
		n : # of joint DOFs, n = 0,1,2,3
		P : position level
		V : velocity level

	* In these constraints, lower-order variables are regarded as constants.
	  (positions in velocity constraints,
	   positions and velocities in accel constraints)
	  This simplifies the constraint Jacobians but not theoretically justified.
 */

/// base class of kinematic constraint
class JointCon : public Constraint{
public:
	JointKey* jnt;

	JointCon(Solver* solver, uint n, ID id, JointKey* _jnt, real_t _scale);
};

/**
	mapping between force/moment in 3D and torque in joint coord.
 */
class JointConF : public JointCon{
public:
	uint idx;

	virtual void CalcCoef();
	JointConF(Solver* solver, const string& _name, JointKey* _jnt, uint i, real_t _scale);
};

class JointConTP : public JointCon{
public:
	virtual void CalcCoef();
	virtual void CalcDeviation();
	JointConTP(Solver* solver, const string& _name, JointKey* _jnt, real_t _scale);
};
class JointConTV : public JointCon{
public:
	virtual void CalcCoef();
	virtual void CalcDeviation();
	JointConTV(Solver* solver, const string& _name, JointKey* _jnt, real_t _scale);
};
class JointConRP : public JointCon{
public:
	virtual void CalcCoef();
	virtual void CalcDeviation();
	JointConRP(Solver* solver, const string& _name, JointKey* _jnt, real_t _scale);
};
class JointConRV : public JointCon{
public:
	virtual void CalcCoef();
	virtual void CalcDeviation();
	JointConRV(Solver* solver, const string& _name, JointKey* _jnt, real_t _scale);
};

/// ŠÖßÀ•W•\Œ»‚Å‚Ì‰^“®•û’öŽ®S‘©
class JointConDense{

};

}
