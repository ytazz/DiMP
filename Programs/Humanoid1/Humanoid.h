#ifndef HUMANOID_H
#define HUMANOID_H

#include "../DiMP1/Graph.h"

enum ArmLabels {
	SHOULDER,
	SHOULDERROLL,
	FOREARM,
	ELBOW,
	WRIST, HAND = WRIST,
	nArmJoints
};

enum LegLabels {
	HIPYAW,
	HIPSIDE,
	HIPPITCH, THIGH = HIPPITCH,
	KNEE, SHANK = KNEE,
	ANKLEPITCH, ANKLE = ANKLEPITCH,
	ANKLEROLL, FOOT = ANKLEROLL,
	nLegJoints
};

using namespace DiMP;

/// Humanoid robot class
class Humanoid : public Graph {
public:
	/// Body class
	struct Body {
		ObjectNode *m_node;
		FixNode *m_fix;

		bool m_forceMask[3];	///< whether to specify desired force (for each element)
		bool m_momentMask[3];	///< whether to specify desired moment (for each element)
		Vec3d m_desiredForce;	///< desired force
		Vec3d m_desiredMoment;	///< desired moment
	};

	//Chest class
	struct Chest {
	ObjectNode* m_objNodes;
	HingeNode* m_jntNodes;
	bool m_forceMask[3];	///< whether to specify desired force (for each element)
	bool m_momentMask[3];	///< whether to specify desired moment (for each element)
	Vec3d m_desiredForce;	///< desired force
	Vec3d m_desiredMoment;	///< desired moment
	
	void Enable(bool on = true);///< [en/dis]able. If disabled, chest will be disregarded in planning
	};

	/// Limb class
	struct Limb{
		vector<ObjectNode*>	m_objNodes;	///< rigid bodies composing this limb
		vector<HingeNode*>	m_jntNodes;	///< joints composing this limb

		bool m_bTarget;			///< whether desired end-point position is specified
		Vec3d m_targetPos;		///< desired end-point position

		void Enable(bool on = true);	///< [en/dis]able. If disabled, this limb will be disregarded in planning
		void SetTarget(const Vec3d & target);	///< specify desired end-point position

		Limb(){
			m_bTarget = false;
		}
	};
	struct Arm:Limb {

	};
	struct Leg:Limb {

	};
	struct Floor {
		ObjectNode *m_node;
	};
	//Target class
	struct Target {
		TargetNode* m_trgNodes; ///< targets composing
		PHSolidIf*	sld_trg;///solid of target
		double t_0; /// intial t_c
		Vec3d m_trgPos0; /// initial target position
		Vec3d m_trgVel0; /// initial target velocity
	};

	Body m_body;
	Chest m_chest;
	Arm m_arm[2];
	Leg m_leg[2];
	Floor m_floor;
	Target m_target[2];

	bool m_bNeutralPose;		///< true => return-to-neutral-posture mode
	bool m_bBodyMoment;			///< true => generate-body-moment mode

	FILE *m_bodyForceLog;		///< file handle to output body force log
	FILE *m_errorLog;			///< file handle to output constraint residual log

	Vec3d m_com;				///< center of mass
	Vec3d m_comProj;			///< center of mass projected onto the floor
	Vec3d m_cop;				///< center of pressure

	/// Link springhead scene and build graph for planning
	void Link(ObjectNode * &objNode, UTString name, double density);
	void Link(HingeNode * &joint, ObjectNode * lhs, ObjectNode * rhs, UTString name, double spring, double damper, double lower, double upper);
	void Link(Target &target, ObjectNode* hand, PHSdkIf * sdk,PHSceneIf * scene);
	void Link(Target &target, ObjectNode* hand, UTString name);
	bool Link(PHSdkIf * sdk, PHSceneIf * scene);


	/// enable contact calculation between object o1 and o2
	void EnableContact(PHSolidIf * o1, PHSolidIf * o2);

	/// scale the mass of obj by rate k
	void ScaleMass(PHSolidIf * obj, double k);

	/// helper function for calculating center of mass of the robot
	void AccumulateCom(PHSolidIf * obj, double &mtotal, Vec3d & com);

	//create target
	//void CreateTarget();

	///
	/// virtual functions derived from Graph
	///
	virtual double LowerLimit(JointNode * jnt);
	virtual double UpperLimit(JointNode * jnt);
	virtual double VelLimit(JointNode * jnt);
	virtual double TorqueLimit(JointNode * jnt);

	virtual void Initialize();
	virtual void UpdateState();
	virtual void Execute();
	virtual void EnablePlan(bool on = true);
	virtual void EnableExecute(bool on = true);
	virtual void Step();
	virtual void Draw(GRDebugRenderIf * render);

	Humanoid();
	virtual ~Humanoid(){}
};

#endif
