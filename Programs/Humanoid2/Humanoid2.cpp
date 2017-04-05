#include "RobotArm.h"

void RobotArm::Build(PHSdkIf* phSdk, PHSceneIf* phScene){
	const double K = 100, D = 10000, L = 3.0;
	
	// box形状を作成
	CDBoxDesc bd;
	bd.boxsize = 0.1*Vec3d(0.6*L, 2*L, 0.6*L);
	CDShapeIf* box = phSdk->CreateShape(bd);

	// ball形状を作成
	CDSphereDesc descSphere;
	descSphere.radius = 0.1;
	CDShapeIf* ball = phSdk->CreateShape(descSphere);

	// アームのベース部の剛体
	PHSolidDesc solidDesc;
	solidDesc.mass = 1.0;
	solidDesc.inertia = 1.0 * Matrix3d::Unit();
	sldBase = phScene->CreateSolid(solidDesc);
	sldBase->AddShape(box);
	sldBase->SetDynamical(false);		// 外力の影響を受けないようにする
	
	// アームのリンク部の剛体
	for(int i = 0; i < 3; i++){
		sldLinks[i] = phScene->CreateSolid(solidDesc);
		sldLinks[i]->AddShape(box);
	}

	// アームの関節
	PHHingeJointDesc descHinge;
	descHinge.posePlug.Pos() = 0.1*Vec3d(0.0, -L, 0.0);	// ベースに関節を取り付ける位置
	hngJoints[0] = (PHHingeJointIf*)phScene->CreateJoint(sldBase, sldLinks[0], descHinge);
	hngJoints[0]->SetSpring(K);	// 位置制御用のバネとダンピング
	hngJoints[0]->SetDamper(D);

	descHinge.poseSocket.Pos() = 0.1*Vec3d(0.0,  L, 0.0);	// ベースに近いリンクに関節を取り付ける位置
	descHinge.posePlug.Pos()   = 0.1*Vec3d(0.0, -L, 0.0);	// 先端に近いリンクに関節を取り付ける位置
	for(int i = 1; i < 3; i++){
		hngJoints[i] = (PHHingeJointIf*)phScene->CreateJoint(sldLinks[i-1], sldLinks[i], descHinge);
		hngJoints[i]->SetSpring(K);	// 位置制御用のバネとダンピング
		hngJoints[i]->SetDamper(D);
	}

	PHTreeNodeIf* node;
	node = phScene->CreateRootNode(sldBase);
	node = phScene->CreateTreeNode(node, sldLinks[0]);
	node = phScene->CreateTreeNode(node, sldLinks[1]);
	node = phScene->CreateTreeNode(node, sldLinks[2]);

	// 目標位置を示す剛体
	sldTarget = phScene->CreateSolid();
	sldTarget->SetDynamical(false);
	sldTarget->AddShape(ball);

	// 接触力計算をオフ (構築した後に呼ぶ必要がある)
	phScene->SetContactMode(PHSceneDesc::MODE_NONE);

}

void RobotArm::SetTarget(const Vec3d& pos){
	sldTarget->SetFramePosition(pos);
}


//-------------------------------------------------------------------------------------------------

MyDrawConfig::MyDrawConfig(){
	// object trajectory: thick black line
	materials[MAT_OBJECT ].lineColor = Vec4d(0.0, 0.0, 0.0, 1.0);
	materials[MAT_OBJECT ].lineColorName = "black";
	materials[MAT_OBJECT ].lineWidth = 2;

	// joint: thin black line
	materials[MAT_JOINT  ].lineColor = Vec4d(0.0, 0.0, 0.0, 1.0);
	materials[MAT_JOINT  ].lineColorName = "darkblue";
	
	// forces: gray line
	materials[MAT_FORCE  ].lineColor = Vec4d(0.5, 0.5, 0.5, 1.0);
	materials[MAT_FORCE  ].lineColorName = "gray";

	materials[MAT_MOMENT ].lineColor = Vec4d(0.5, 0.5, 0.5, 1.0);
	materials[MAT_MOMENT ].lineColorName = "darkgray";

	// desired position: large dot
	materials[MAT_DESIRED].lineColor = Vec4d(0.0, 0.0, 0.0, 1.0);
	materials[MAT_DESIRED].lineColorName = "black";
	materials[MAT_DESIRED].pointSize = 4;
}

void MyDrawConfig::Set(DrawContext* draw, uint tag, Node* node, uint idx){
	DrawConfig::Set(draw, tag, node, idx);
}

void Planner::Build(){
	
	// create graph nodes
	base = graph.CreateObject();
	objPairs.push_back(make_pair(base, robot->sldBase));
	for(int i = 0; i < 3; i++){
		links[i] = graph.CreateObject();
		objPairs.push_back(make_pair(links[i], robot->sldLinks[i]));
	}

	Frame *fr0, *fr1;
	Posed pose;
	robot->hngJoints[0]->GetSocketPose(pose);
	fr0 = base    ->CreateFrame(pose);
	robot->hngJoints[0]->GetPlugPose(pose);
	fr1 = links[0]->CreateFrame(pose);
	joints[0] = graph.CreateHinge(fr0, fr1);
	jntPairs.push_back(make_pair(joints[0], robot->hngJoints[0]));

	for(int i = 1; i < 3; i++){
		robot->hngJoints[i]->GetSocketPose(pose);
		fr0 = links[i-1]->CreateFrame(pose);
		robot->hngJoints[i]->GetPlugPose(pose);
		fr1 = links[i  ]->CreateFrame(pose);
		joints[i] = graph.CreateHinge(fr0, fr1);
		jntPairs.push_back(make_pair(joints[i], robot->hngJoints[i]));
	}
	
	graph.CreateEvent(1.0);
	graph.CreateEvent(2.0);
	eventReach = graph.CreateEvent(3.0);

	costReach = graph.CreateCost(eventReach, OBJECT_POS_T, links[2]);
	costReach->SetWeight(0.5);

	// initialize graph
	graph.Init();

	// 
	//graph.SetGroup(-1, NULL, -1, 2);				//< put all variables to group 1
	//graph.SetGroup(-1, joints[2], -1, 1);	//< put end-effector variables to group 0
	//graph.SetGroup(-1, links[2], -1, 0);	//< put end-effector variables to group 0

	// move force/moment variables to group 1
	//graph.SetGroup(JOINT_FORCE_T, NULL, -1, 1);
	//graph.SetGroup(JOINT_FORCE_R, NULL, -1, 1);

	//graph.Enable(-1, NULL, -1, false);				//< disable all constraints
	//graph.Enable(OBJECT_SPLINE, NULL, -1, false);
	//graph.Enable(KINEMATIC_POS_T, NULL, -1, false);
	//graph.Enable(KINEMATIC_POS_R, NULL, -1, false);
	//graph.Enable(KINEMATIC_VEL_T, NULL, -1, false);
	//graph.Enable(KINEMATIC_VEL_R, NULL, -1, false);
	//graph.Enable(KINEMATIC_ACC_T, NULL, -1, false);
	//graph.Enable(KINEMATIC_ACC_R, NULL, -1, false);
	//graph.Enable(NETFORCE_T, NULL, -1, false);
	//graph.Enable(NETFORCE_R, NULL, -1, false);

	/*	変数のスケーリング
		各物理変数の大まかな値域を考えて決める
	 */
	graph.SetScale(OBJECT_POS_T,	NULL, -1, 1.0);		// 1[m]
	graph.SetScale(OBJECT_POS_R,	NULL, -1, 3.14);	// 3.14[rad]
	graph.SetScale(OBJECT_VEL_T,	NULL, -1, 1.0);		// 1[m/s]
	graph.SetScale(OBJECT_VEL_R,	NULL, -1, 3.14);	// 3.14[rad/s]
	graph.SetScale(OBJECT_ACC_T,	NULL, -1, 3*1.0);		// 1[m/s2]
	graph.SetScale(OBJECT_ACC_R,	NULL, -1, 3*3.14);	// 3.14[rad/s2]
	graph.SetScale(OBJECT_FORCE_T,	NULL, -1, 1.0);		// 10[N]
	graph.SetScale(OBJECT_FORCE_R,	NULL, -1, 1.0);		// 3.14[Nm]
	graph.SetScale(JOINT_FORCE_T,	NULL, -1, 1.0);		// 10[N]
	graph.SetScale(JOINT_FORCE_R,	NULL, -1, 1.0);		// 3.14[Nm]

	graph.SetScale(JOINT_POS, NULL, -1, 3.14);
	graph.SetScale(JOINT_VEL, NULL, -1, 3.14);

	//graph.Enable(KINEMATIC_POS_T, NULL, -1, true);	//< enable Kinematic constraints
	//Enable(KINEMATIC_POS_R, NULL, -1, true);	//< enable Kinematic constraints

	// グループとイネーブルを設定後に呼ぶ
	graph.solver.Init();
}

void Planner::LoadState(){
	for(ObjectPairs::iterator it = objPairs.begin(); it != objPairs.end(); it++){
		DiMP2::Object* obj = it->first;
		PHSolidIf* sld = it->second;

		obj->mass = sld->GetMass();
		obj->inertia = sld->GetInertia()[0][0];
		obj->dynamical = sld->IsDynamical();

		obj->SetInitialPose(sld->GetPose());
		obj->SetInitialVel(Vec3d(), Vec3d());
	}
	for(JointPairs::iterator it = jntPairs.begin(); it != jntPairs.end(); it++){
		it->first->SetInitialPos(it->second->GetPosition());
		it->first->SetInitialVel(it->second->GetVelocity());
	}
}
	
void Planner::Step(){
	// set desired position
	costReach->SetDesired(robot->sldTarget->GetFramePosition());

	graph.Step();
}

void Planner::Draw(DrawContext* draw){
	graph.Draw(draw, &conf);
}
