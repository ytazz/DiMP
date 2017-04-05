#pragma warning(disable : 4996)

#include "Humanoid.h"
#include "../src/Draw.h"
#include <Physics/PHSpatial.h>
#include <Physics/PHSolid.h>
#include <Physics/PHTreeNode.h>

//obj->sld
const char *armObjectNames[2][5] = {
	{"sld_shoulderl", "sld_arml", "sld_elbowl", "sld_forearml", "sld_handl"},
	{"sld_shoulderr", "sld_armr", "sld_elbowr", "sld_forearmr", "sld_handr"}
};

const char *armJointNames[2][5] = {
	{"shoulderl", "shoulderrolll", "upperarml", "elbowl", "wristl"},
	{"shoulderr", "shoulderrollr", "upperarmr", "elbowr", "wristr"}
};

const char *legObjectNames[2][6] = {
	{"sld_hipyawl", "sld_hipsidel", "sld_thighl", "sld_shankl", "sld_anklel", "sld_footl"},
	{"sld_hipyawr", "sld_hipsider", "sld_thighr", "sld_shankr", "sld_ankler", "sld_footr"}
};

const char *legJointNames[2][6] = {
	{"hipyawl", "hipsidel", "hippitchl", "kneel", "anklepitchl",
	"anklerolll"},
	{"hipyawr", "hipsider", "hippitchr", "kneer", "anklepitchr", "anklerollr"}
};

const char *targetSolidName[2] = {
	{"sld_target0"},
	{"sld_target1"}
};

const double armJointLimits[2][5][2] = {
	//shoulder,     shoulderside, armtwist, elbow, wrist
	{{-150.0, 150.0}, {0.0, 80.0}, {-90.0, 90.0}, {-80.0, -0.0}, {-45.0, 45.0}},
	{{-150.0, 150.0}, {0.0, 80.0}, {-90.0, 90.0}, {-80.0, -0.0}, {-45.0, 45.0}}
};

const double armJointCenters[2][5] = {
	{0.0, 45.0, 0.0, 0.0, 0.0},
	{0.0, 45.0, 0.0, 0.0, 0.0}
};

const double legJointLimits[2][6][2] = {
	//hipyaw,       hipside     ,  hippitch,    knee,         anklepitch,    ankleroll
	{{-30.0, 30.0}, {-30.0, 30.0}, {-90.0, 90.0}, {-90.0, 0.0}, {-30.0, 30.0}, {-30.0, 30.0}},
	{{-30.0, 30.0}, {-30.0, 30.0}, {-90.0, 90.0}, {-90.0, 0.0}, {-30.0, 30.0}, {-30.0, 30.0}}
};

const double legJointCenters[2][6] = {
	{0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
	{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}
};

const double footContactPoints[2][4][3] = {
	{{0.1, 0.08, 0.0}, {0.1, -0.06, 0.0}, {-0.08, 0.08, 0.0}, {-0.08, -0.06, 0.0}},
	{{0.1, 0.06, 0.0}, {0.1, -0.08, 0.0}, {-0.08, 0.06, 0.0}, {-0.08, -0.08, 0.0}},
};

const double waistJointLimits[2]={
	-70.0,
	 70.0,
};



//////////////////////////////////////////////////////////////////////////////////////
void Humanoid::Chest::Enable(bool on) {
	if(m_objNodes)
		m_objNodes->Enable(on);
	if(m_jntNodes)
		m_jntNodes->Enable(on);
	cout << "Enable Chest" << endl; 
}

void Humanoid::Limb::Enable(bool on) {
	for(size_t i = 0; i < m_objNodes.size(); i++)
		if(m_objNodes[i])
			m_objNodes[i]->Enable(on);
	for(size_t i = 0; i < m_jntNodes.size(); i++)
		if(m_jntNodes[i])
			m_jntNodes[i]->Enable(on);
	cout << "Enable Arm" << endl; 
}

//////////////////////////////////////////////////////////////////////////////////////

Humanoid::Humanoid() {
}

void Humanoid::ScaleMass(PHSolidIf * obj, double k) {
	obj->SetMass(obj->GetMass() * k);
	obj->SetInertia(obj->GetInertia() * k);
}

void Humanoid::EnableContact(PHSolidIf * o1, PHSolidIf * o2) {
	m_scene->SetContactMode(o1, o2, PHSceneDesc::MODE_LCP);
}

void Humanoid::Link(ObjectNode * &objNode, UTString name, double density) {
	PHSolidIf *obj = DCAST(PHSolidIf, m_scene->FindObject(name));
	if(obj) {
		//ScaleMass(obj, density);
		obj->SetMass(0.01);//質量
		obj->SetInertia(0.01 * Matrix3d::Unit());//慣性モーメント
		cout << "mass: " << obj->GetMass() << endl;
		cout << "iner: " << obj->GetInertia() << endl;
		objNode = CreateObjectNode(obj);
		cout << "linked object " << name.c_str() << endl;
	}
	else {
		objNode = NULL;
		cout << "could not link object " << name.c_str() << endl;
	}
}

void Humanoid::Link(HingeNode * &jointNode, ObjectNode * lhs, ObjectNode * rhs, UTString name,
	double spring, double damper, double lower, double upper)
{
	PHHingeJointIf *joint = DCAST(PHHingeJointIf, m_scene->FindObject(name));
	if(joint) {
		joint->SetSpring(spring);
		joint->SetDamper(damper);
		//joint->SetTargetPosition(0.0);
		joint->SetRange(lower, upper);
		jointNode = (HingeNode *) CreateHingeNode(joint, lhs, rhs);

		cout << "linked joint " << name.c_str() << endl;
	}
	else {
		jointNode = NULL;
		cout << "could not link joint " << name.c_str() << endl;
	}
}

////////////////Link of Target
//void Humanoid::Link(Target &target, ObjectNode* hand, PHSdkIf * sdk, PHSceneIf * scene){
//	PHSolidDesc solidDesctrg;
//
//	/// shape of target 
//	solidDesctrg.mass = 0.1; //mass of target
//	CDSphereDesc descSphere; //shape of target
//	descSphere.radius = 0.04; //radius of target
//	CDShapeIf* trgshape; 
//	trgshape = sdk->CreateShape(descSphere);//create shape of target
//
//	target.sld_trg = scene->CreateSolid(solidDesctrg);
//	target.sld_trg->SetDynamical(false);
//	target.sld_trg->AddShape(trgshape);
//	target.sld_trg->SetFramePosition(target.m_trgPos0);
//	target.sld_trg->SetVelocity(target.m_trgVel0);
//
//	///Hand to target
//	target.m_trgNodes = CreateTargetNode(target.sld_trg,hand);
//	(target.m_trgNodes)->t_c = target.t_0;//適当な初期値
//	cout<< "initial t_c=" << target.t_0 <<endl;
//	cout << "trgNodes =" << target.m_trgNodes << endl;
//	target.m_trgNodes->UpdateT0();
//
//	//target is stopped in  planning 
//	target.sld_trg->SetVelocity(Vec3d(0,0,0));
//}

////////////////Link of Target2
void Humanoid::Link(Target &target, ObjectNode* hand, UTString name){
	target.sld_trg = DCAST(PHSolidIf, m_scene->FindObject(name));
	if(target.sld_trg) {
		//ScaleMass(obj, density);
		target.sld_trg->SetMass(0.01);//質量
		target.sld_trg->SetInertia(0.01 * Matrix3d::Unit());//慣性モーメント
		cout << "target_mass: " << target.sld_trg->GetMass() << endl;
		cout << "target_iner: " << target.sld_trg->GetInertia() << endl;
		target.sld_trg->SetDynamical(false);//物理法則のon/off
		target.sld_trg->SetFramePosition(target.m_trgPos0);//初期位置
		target.sld_trg->SetVelocity(target.m_trgVel0);//初期速度

		///Hand to target
		target.m_trgNodes = CreateTargetNode(target.sld_trg,hand);
		(target.m_trgNodes)->t_c = target.t_0;//適当な初期値
		cout<< "initial t_c=" << target.t_0 <<endl;
		cout << "trgNodes =" << target.m_trgNodes << endl;
		target.m_trgNodes->UpdateT0();
	}
	else {
		cout << "could not link" << name.c_str() << endl;
	}
}


//////////////


bool Humanoid::Link(PHSdkIf * sdk, PHSceneIf * scene) {
	cout << "linking Humanoid to scene" << endl;

	if(!Graph::Link(sdk, scene))
		return false;

	const double density = 1500.0;	// g/cm^3 = 1000kg/m^3
	//const double density = 1.0;
	//double D = 2.0, K = 100.0;	// Nm/rad = 0.16 kgcm/deg
	double D = 1000, K = 810;	// Nm/rad = 0.16 kgcm/deg

	Link(m_floor.m_node, "sld_floor", density);

	cout << "link body" << endl;
	Link(m_body.m_node, "sld_body", density);
	m_body.m_fix = CreateFixNode("fix_body", m_floor.m_node, m_body
		.m_node,
		m_floor.m_node->m_object->GetPose().Inv() * m_body.m_node->m_object->GetPose(), Posed());
	// link chest
	cout << "link chest" << endl;
	Link(m_chest.m_objNodes, "sld_chest", density);
	ObjectNode *parent = m_body.m_node;
	Link(m_chest.m_jntNodes, parent, m_chest.m_objNodes, "waist",
		K, D, Rad(waistJointLimits[0]) , Rad(waistJointLimits[1]));

	// link arms
	cout << "link arms" << endl;
	for(int i = 0; i < 2; i++) {
		m_arm[i].m_objNodes.resize(nArmJoints);
		m_arm[i].m_jntNodes.resize(nArmJoints);
		//ObjectNode *parent =  m_body.m_node;
		ObjectNode *parent = m_chest.m_objNodes;
		for(int j = 0; j < nArmJoints; j++) {
			Link(m_arm[i].m_objNodes[j], armObjectNames[i][j], density);
			if(!m_arm[i].m_objNodes[j])
				break;
			Link(m_arm[i].m_jntNodes[j], parent, m_arm[i].m_objNodes[j], armJointNames[i][j],
				K, D, Rad(armJointLimits[i][j][0]), Rad(armJointLimits[i][j][1]));
			if(!m_arm[i].m_jntNodes[j])
				break;

			parent = m_arm[i].m_objNodes[j];
		}
	}

	// link legs
	cout << "link legs" << endl;
	for(int i = 0; i < 2; i++) {
		m_leg[i].m_objNodes.resize(nLegJoints);
		m_leg[i].m_jntNodes.resize(nLegJoints);
		ObjectNode *parent = m_body.m_node;
		for(int j = 0; j < nLegJoints; j++) {
			Link(m_leg[i].m_objNodes[j], legObjectNames[i][j], density);
			Link(m_leg[i].m_jntNodes[j], parent, m_leg[i].m_objNodes[j], legJointNames[i][j],
				K, D, Rad(legJointLimits[i][j][0]), Rad(legJointLimits[i][j][1]));
			parent = m_leg[i].m_objNodes[j];
		}
	}

	//targetの設定
	//initial t_c
	m_target[0].t_0 = 1.0;
	m_target[1].t_0 = 1.5;

	//initial position
	//m_target[0].m_trgPos0 = Vec3d(0.2,0.92,0.3);
	m_target[0].m_trgPos0 = Vec3d(0.16,0.46,0.1);
	m_target[1].m_trgPos0 = Vec3d(0.3,0.15,-0.1);

	//initial Velcity
	m_target[0].m_trgVel0 = 0.3 * Vec3d(0.16,0.46,0.1);
	m_target[1].m_trgVel0 = Vec3d(0.1,0.5,-0.05);

	
	//link target
	cout << "ling target" << endl;
	for(int i = 0; i < 1; i++){
		Link(m_target[i], m_arm[0].m_objNodes[HAND] , targetSolidName[i]);
	}



	//for(int i = 0; i < 2; i++){
	//	Link(m_target[i], m_arm[0].m_objNodes[HAND] , sdk, scene);
	//}

	return true;
}

void Humanoid::Initialize() {
	Graph::Initialize();

	// body object is manually set "dynamical" to compute constraint forces
	m_body.m_node->m_bDynamical = true;
}

void Humanoid::AccumulateCom(PHSolidIf * obj, double &mtotal, Vec3d & com) {
	com += obj->GetMass() * obj->GetCenterPosition();
	mtotal += obj->GetMass();
}

double Humanoid::LowerLimit(JointNode * jnt) {
	if(jnt == m_chest.m_jntNodes)
		return Rad(waistJointLimits[0]);

	for(int i = 0; i < 2; i++)
		for(int j = 0; j < nArmJoints; j++)
			if(jnt == m_arm[i].m_jntNodes[j])
				return Rad(armJointLimits[i][j][0]);
	return -FLT_MAX;
}

double Humanoid::UpperLimit(JointNode * jnt) {
	if(jnt == m_chest.m_jntNodes)
		return Rad(waistJointLimits[1]);

	for(int i = 0; i < 2; i++)
		for(int j = 0; j < nArmJoints; j++)
			if(jnt == m_arm[i].m_jntNodes[j])
				return Rad(armJointLimits[i][j][1]);
	return FLT_MAX;
}

double Humanoid::VelLimit(JointNode * jnt) {
	//return 1.0;
	return FLT_MAX;
}

double Humanoid::TorqueLimit(JointNode * jnt) {
	return FLT_MAX;
}

void Humanoid::UpdateState() {
	Graph::UpdateState();
}

void Humanoid::Execute() {
	Graph::Execute();

	if(m_bReplay){
		size_t idx = m_replayTime;
		const int idMap[2][5] =
		{ {10, 11, 12, 13, 14}, {17, 18, 19, 20, 21} };
		const double offset[2][5] =
		{ {90.0, -10.0, 0.0, 0.0, 0.0}, {90.0, 10.0, 0.0, 0.0, 0.0} };
		const double dir[2][5] =
		{ {1.0, 1.0, -1.0, 1.0, 1.0}, {-1.0, -1.0, -1.0, -1.0, 1.0} };
		//const double test[2][5] = {{}, {}};

		for(size_t i = 0; i < 2; i++) {
			//hand = m_arm[i].m_objNodes[HAND];

			for(size_t j = 0; j < nArmJoints; j++) {
				if(idx >= m_arm[i].m_jntNodes[j]->m_history.size())
					continue;
				double pos = m_arm[i].m_jntNodes[j]->m_history[idx].p;
				pos = Rad(offset[i][j] + dir[i][j] * Deg(pos));
			}
		}
	}
}

void Humanoid::EnablePlan(bool on) {
	Graph::EnablePlan(on);
	if(m_bPlan) {
		m_errorLog = fopen("error.csv", "w");
	}
	else {
		fclose(m_errorLog);
	}
}

void Humanoid::EnableExecute(bool on) {
	Graph::EnableExecute(on);
	if(m_bExecute) {
		m_bodyForceLog = fopen("bodyforce.csv", "w");
	}
	else {
		fclose(m_bodyForceLog);
	}
}

void Humanoid::Step() {
	Graph::Step();
}

void Humanoid::Draw(GRDebugRenderIf * render) {
	Graph::Draw(render);
}
