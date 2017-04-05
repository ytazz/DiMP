#include "HumanoidCatching.h"

#include <sstream>
using namespace std;

#include <boost/foreach.hpp>
#define foreach BOOST_FOREACH
using namespace boost;

//------------------------------------------------------------------------------------------------
// Humanoid

/// 各部寸法，質量
const real_t	baseR			=    0.4;		///< 腰半径
const real_t	baseL			=    0.35;		///< 腰長さ
const real_t	legUpperR		=    0.15;		///< 腿半径
const real_t	legUpperL		=    0.7;		///< 腿長さ
const real_t	legUpperX		=    0.23;		///< 腰からの腿中心X
const real_t	legUpperY		= -  0.56;		///< 腰からの腿中心Y
const real_t	legLowerR		=    0.15;		///< 脛半径
const real_t	legLowerL		=    0.84;		///< 脛長さ
const real_t	legLowerX		=    0.2;		///< 腰からの脛中心X
const real_t	legLowerY		= -  1.3;		///< 腰からの脛中心Y
const real_t	legCamber		=    2.0;		///< 腿の内向き角
const real_t	waistM			=    5.0;		///< ウエスト質量
const real_t	waistI			=    0.3;		///< ウエスト慣性モーメント
const real_t	waistR			=    0.26;		///< ウエスト半径
const real_t	waistL			=    0.21;		///< ウエスト長さ
const real_t	chestM			=    1.0;		///< 胴質量
const real_t	chestI			=    0.01;		///< 胴慣性モーメント
const real_t	chestR			=    0.45;		///< 胴半径
const real_t	chestL			=    0.7;		///< 胴長さ
const real_t	neckR			=    0.13;		///< 首半径
const real_t	neckL			=    0.21;		///< 首長さ
const real_t	neckY			=    0.42;		///< 胴からの首中心Y
const real_t	headR			=    0.27;		///< 頭半径
const real_t	headL			=    0.42;		///< 頭長さ
const real_t	headY			=    0.7;		///< 胴からの頭中心Y
const real_t	headZ			=    0.035;		///< 胴からの頭中心Z
const real_t	headPitch		=    5.0;		///< 頭前傾角
const real_t	baseToYawY		=    0.15;		///< 腰からヨー軸まで
const real_t	waistToYawY		= -  0.15;		///< ウエストからヨー軸まで
const real_t	chestToPitchY	= -  0.5;		///< 胴からピッチ軸まで
const real_t	yawMin			= - 10.0;		///< ヨー角最小値//-30
const real_t	yawMax			=   10.0;		///< ヨー角最大値//+30
const real_t	pitchMin		= -  5.0;		///< ピッチ角最小値
const real_t	pitchMax		=   15.0;		///< ピッチ角最大値+15
const real_t	armUpperR		=    0.15;		///< 上腕（肩-ツイスト間）半径
const real_t	armUpperL		=    0.15;		///< 上腕長さ
const real_t	armUpperM		=    1.0;		///< 上腕質量
const real_t	armUpperI		=    0.01;		///< 上腕慣性モーメント
const real_t	armLowerR		=    0.13;		///< 中腕（ツイストから肘まで）半径
const real_t	armLowerL		=    0.6;		///< 中腕長さ
const real_t	armLowerM		=    1.0;		///< 中腕質量
const real_t	armLowerI		=    0.01;		///< 中腕慣性モーメント
const real_t	wristR			=    0.13;		///< 下腕（肘から手先まで）半径
const real_t	wristL			=    0.8;		///< 下腕長さ
const real_t	wristX			=    wristL/2;	///< 下腕
const real_t	handM			=    1.0;		///< 手先質量
const real_t	handI			=    0.01;		///< 手先慣性モーメント
const real_t	handR			=    0.13;		///< 手先半径
const real_t	handL			=    0.13;		///< 手先長さ
const real_t	lowerToElbowX	=    armLowerL/2;	///< 中腕から肘まで
const real_t	handToElbowX	=    wristL;	///< 手先から肘まで
const real_t	upperToTwistX	=    0.05;		///< 上腕からツイストまで
const real_t	lowerToTwistX	=    armLowerL/2;	///< 中腕からツイストまで
const real_t	elbowMin		= - 80.0;		///< 肘角最小値
const real_t	elbowMax		=    0.0;		///< 肘角最大値
const real_t	twistMin		= - 90.0;		///< ツイスト角最小値
const real_t	twistMax		=   90.0;		///< ツイスト角最大値
const real_t	shoulderX		=    0.4;		///< 胴からの肩中心X
const real_t	shoulderY		=    0.2;		///< 胴からの肩中心Y
const real_t	shoulderZ		=    0.05;		///< 胴からの肩中心Z
const real_t	shoulderMin		= -150.0;		///< 肩角最小値
const real_t	shoulderMax		=  150.0;		///< 肩角最大値

void Humanoid::Build(){
	stringstream ss;
	TargetInfo* info;
	//task0
	targetInfos.push_back(TargetInfo());
	info = &targetInfos.back();
	info->left_or_right = true;
	info->ball_or_box	= false;
	info->boxSize		= vec3_t(0.25, 0.25, 0.25);
	info->iniPos		= vec3_t(1.0, 0.75, 1.0);
	info->iniVel		= vec3_t(-0.5, 0.0, 0.0);
	info->iniStartTime	= 2.0;
	info->iniEndTime	= 5.6;
	////task1
	targetInfos.push_back(TargetInfo());
	info = &targetInfos.back();
	info->left_or_right = false;
	info->ball_or_box	= false;
	info->boxSize		= vec3_t(0.25, 0.25, 0.25);
	info->iniPos		= vec3_t(1.0, 0.75, 1.0);
	info->iniVel		= vec3_t(-0.5, 0.0, 0.0);
	info->iniStartTime	= 2.0;
	info->iniEndTime	= 5.6;

	//task2(obstacle)
	targetInfos.push_back(TargetInfo());
	info = &targetInfos.back();
	info->left_or_right = false;
	info->ball_or_box	= true;
	info->ballRadius	= 0.4;
	info->iniPos		= vec3_t(-1.5, 0.75, 0.75);
	info->iniVel		= vec3_t(0.0, 0.0, 0.0);
	info->iniStartTime	= 0.0;
	info->iniEndTime	= 6.0;

	//task3(obstacle)
	targetInfos.push_back(TargetInfo());
	info = &targetInfos.back();
	info->left_or_right = false;
	info->ball_or_box	= true;
	info->ballRadius	= 0.3;
	info->iniPos		= vec3_t(-1.5, 0.75, 0.75);
	info->iniVel		= vec3_t(0.0, 0.0, 0.0);
	info->iniStartTime	= 0.0;
	info->iniEndTime	= 6.0;

	// body
	{
		/// base
		body.base = CreateObject("base");
		body.base->SetDynamical(false);
		AttachGeometry(
			CreateCylinder("cylinder_base", baseR, baseL),
			body.base->CreateConnector( pose_t(vec3_t(), quat_t::Rot(Rad(90.0), 'x')) )
			);

		//legs
		Geometry* geoLegUpper = CreateCylinder("cylinder_leg_upper", legUpperR, legUpperL);
		Geometry* geoLegLower = CreateCylinder("cylinder_leg_lower", legLowerR, legLowerL);
		AttachGeometry(
			geoLegUpper,
			body.base->CreateConnector( pose_t(vec3_t( legUpperX, legUpperY, 0.0), quat_t::Rot(Rad(90.0 + legCamber), 'x')) )
			);
		AttachGeometry(
			geoLegLower,
			body.base->CreateConnector( pose_t(vec3_t( legLowerX, legLowerY, 0.0), quat_t::Rot(Rad(90.0), 'x')))
			);
		AttachGeometry(
			geoLegUpper,
			body.base->CreateConnector( pose_t(vec3_t(-legUpperX, legUpperY, 0.0), quat_t::Rot(Rad(90.0 - legCamber), 'x')))
			);
		AttachGeometry(
			geoLegLower,
			body.base->CreateConnector( pose_t(vec3_t(-legLowerX, legLowerY, 0.0), quat_t::Rot(Rad(90.0), 'x')))
			);
		links.push_back(body.base);

		/// waist
		body.waist = CreateObject("waist");
		body.waist->mass    = waistM;
		body.waist->inertia = waistI;
		AttachGeometry(
			CreateCylinder("cylinder_waist", waistR, waistL),
			body.waist->CreateConnector( pose_t(vec3_t(), quat_t::Rot(Rad(90.0), 'x')))
			);
		links.push_back(body.waist);

		/// chest
		body.chest = CreateObject("chest");
		body.chest->mass    = chestM;
		body.chest->inertia = chestI;
		//chest
		AttachGeometry(
			CreateCylinder("cylinder_chest", chestR, chestL),
			body.chest->CreateConnector( pose_t(vec3_t(), quat_t::Rot(Rad(90.0), 'x')))
			);
		//neck
		AttachGeometry(
			CreateCylinder("cylinder_neck", neckR, neckL),
			body.chest->CreateConnector( pose_t(vec3_t(0.0, neckY, 0.0), quat_t::Rot(Rad(90.0), 'x')))
			);
		//head
		AttachGeometry(
			CreateCylinder("cylinder_head", headR, headL),
			body.chest->CreateConnector( pose_t(vec3_t(0.0, headY, headZ), quat_t::Rot(Rad(90.0 + headPitch), 'x')))
			);

		/* 球状の頭
		body.chest->AddGeometry(CreateSphere(0.6*R), pose_t(vec3_t(0.0, L, 0.0)));
		body.chest->AddGeometry(CreateSphere(0.6*R), pose_t(vec3_t(0.0, L, 0.0), quat_t::Rot(Rad( 45.0), 'y')));
		body.chest->AddGeometry(CreateSphere(0.6*R), pose_t(vec3_t(0.0, L, 0.0), quat_t::Rot(Rad(-45.0), 'y')));
		*/
		links.push_back(body.chest);

		//base-waist
		body.yaw = CreateHinge(
			"hinge_waist",
			body.base ->CreateConnector( pose_t(vec3_t(0.0, baseToYawY, 0.0), quat_t::Rot(Rad(90.0), 'x')) ),
			body.waist->CreateConnector( pose_t(vec3_t(0.0, waistToYawY, 0.0), quat_t::Rot(Rad(90.0), 'x')) )
			);
		body.yaw->SetInitialPos(0.0);
		body.yaw->SetInitialVel(0.0);
		body.yaw->SetRange(Rad(yawMin), Rad(yawMax));		///< ヨー可動範囲（左右に30度）
		joints.push_back(body.yaw);

		//waist-chest
		body.pitch = CreateHinge(
			"hinge_chest",
			body.waist->CreateConnector( pose_t(vec3_t(), quat_t::Rot(Rad(90.0), 'y')) ),
			body.chest->CreateConnector( pose_t(vec3_t(0.0, chestToPitchY, 0.0), quat_t::Rot(Rad(90.0), 'y')) )
			);
		body.pitch->SetInitialPos(0.0);
		body.pitch->SetInitialVel(0.0);
		body.pitch->SetRange(Rad(pitchMin), Rad(pitchMax));		///< ピッチ可動範囲（かがむ向きに大きい）
		joints.push_back(body.pitch);
	}

	// arms
	Geometry* geoWrist = CreateCylinder("cylinder_wrist", wristR, wristL);
	Geometry* geoHand  = CreateSphere("sphere_hand", handR);
	Geometry* geoArmLower = CreateCylinder("cylinder_arm_lower", armLowerR, armLowerL);
	Geometry* geoArmUpper = CreateCylinder("cylinder_arm_upper", armUpperR, armUpperL);

	for(int i = 0; i < 2; i++){
		// hand
		arm[i].hand = CreateObject(i == 0 ? "hand_left" : "hand_right");
		arm[i].hand->mass = handM;
		arm[i].hand->inertia = handI;
		AttachGeometry(
			geoWrist,
			arm[i].hand->CreateConnector( pose_t(vec3_t((i == 0 ? 1.0 : -1.0) * wristX, 0.0, 0.0), quat_t::Rot(Rad(90.0), 'y')) )
			);
		AttachGeometry(
			geoHand,
			arm[i].hand->CreateConnector( pose_t() )
			);

		// lower arm
		arm[i].lower = CreateObject(i == 0 ? "arm_lower_left" : "arm_lower_right");
		arm[i].lower->mass = armLowerM;
		arm[i].lower->inertia = armLowerI;
		AttachGeometry(
			geoArmLower,
			arm[i].lower->CreateConnector( pose_t(vec3_t(), quat_t::Rot(Rad(90.0), 'y')) )
			);

		// upper arm
		arm[i].upper = CreateObject(i == 0 ? "arm_upper_left" : "arm_upper_right");
		arm[i].upper->mass = armUpperM;
		arm[i].upper->inertia = armUpperI;
		AttachGeometry(
			geoArmUpper,
			arm[i].upper->CreateConnector( pose_t(vec3_t(), quat_t::Rot(Rad(90.0), 'y')) )
			);

		//hand-lowerarm
		arm[i].elbow = CreateHinge(i == 0 ? "elbow_left" : "elbow_right",
			arm[i].lower->CreateConnector( pose_t(vec3_t((i == 0 ? -1.0 : 1.0) * lowerToElbowX, 0.0, 0.0)) ),
			arm[i].hand ->CreateConnector( pose_t(vec3_t((i == 0 ? 1.0 : -1.0) * handToElbowX, 0.0 ,0.0)) )
			);
		arm[i].elbow->SetRange(Rad(elbowMin), Rad(elbowMax));

		//lowerarm-upperarm
		arm[i].twist = CreateHinge(i == 0 ? "twist_left" : "twist_right",
			arm[i].upper->CreateConnector( pose_t(vec3_t((i == 0 ? -1.0 : 1.0) * upperToTwistX, 0.0, 0.0)) ),
			arm[i].lower->CreateConnector( pose_t(vec3_t((i == 0 ? 1.0 : -1.0) * lowerToTwistX, 0.0, 0.0)) )
			);
		arm[i].twist->SetRange(Rad(twistMin), Rad(twistMax));

		//upperarm-chest
		arm[i].shoulder = CreateHinge(i == 0 ? "shoulder_left" : "shoulder_right",
			body.chest  ->CreateConnector( pose_t(vec3_t((i == 0 ? -1.0 : 1.0) * shoulderX, shoulderY, shoulderZ), quat_t::Rot(Rad(90.0), 'y')) ),
			arm[i].upper->CreateConnector( pose_t(vec3_t(), quat_t::Rot(Rad(90.0), 'y')) )
			);
		arm[i].shoulder->SetRange(Rad(shoulderMin), Rad(shoulderMax));

		links.push_back(arm[i].hand);
		links.push_back(arm[i].lower);
		links.push_back(arm[i].upper);
		joints.push_back(arm[i].elbow);
		joints.push_back(arm[i].twist);
		joints.push_back(arm[i].shoulder);
	}

	// target objects
	//ss.str("");
	//ss << "target" ;
	//Targets.push_back(CreateObject(ss.str(),true));
	//Targets.resize(1);

	for(uint i = 0; i < targetInfos.size(); i++){

		TargetInfo& info = targetInfos[i];
		//info.hand = arm[info.left_or_right].hand;
		//info.target = Targets[0];
		if(i != targetInfos.size() -1){
		info.hand = arm[info.left_or_right].hand;
		info.target = CreateObject(ss.str());
		}
		else{
		ss.str("");
		ss << "target" << i;
		info.hand = arm[info.left_or_right].hand;
		info.target = CreateObject(ss.str());
		}
		if(info.ball_or_box){
			ss.str("");
			ss << "sphere_target" << i;
			AttachGeometry(CreateSphere(ss.str(), info.ballRadius), info.target->CreateConnector(pose_t()) );
		}
		else{
			ss.str("");
			ss << "box_target" << i;
			AttachGeometry(CreateBox(ss.str(), info.boxSize), info.target->CreateConnector(pose_t()) );
		}

		// time slot
		ss.str("");
		ss << "timeslot" << i;
		timeslots.push_back(CreateTimeSlot(ss.str(),info.iniStartTime,info.iniEndTime));


		// catchings node
		ss.str("");
		ss << "matching" << i;
		info.task = CreateMatchingTask(ss.str(), info.hand, info.target, timeslots.back(), true);
	}

		// reaching node
		//ss.str("");
		//ss << "reaching" <<0;
		//targetInfos[0].task = CreateMatchingTask(ss.str(),  targetInfos[0].hand,  targetInfos[0].target,timeslots[0], true);

		//// catchings node
		//ss.str("");
		//ss << "holding" <<"left";
		//targetInfos[1].task = CreateMatchingTask(ss.str(),  targetInfos[1].hand,  targetInfos[1].target,timeslots[1], true);

		//// catchings node
		//ss.str("");
		//ss << "reachint" <<"right";
		//targetInfos[2].task = CreateMatchingTask(ss.str(),  targetInfos[0].hand,  targetInfos[0].target,timeslots[0], true);
		// goal
		//ss.str("");
		//ss << "goal" ;
		//targetInfos[2].hand = arm[ targetInfos[2].left_or_right].hand;
		//targetInfos[2].task = CreateMatchingTask(ss.str(),  Targets[0],  targetInfos[2].target,timeslots[2], true);

		//CreateMatchingNOT((MatchingTask*)targetInfos[0].task);
		CreateMatchingOR((MatchingTask*)targetInfos[1].task, (MatchingTask*)targetInfos[0].task);
		CreateMatchingNOT((MatchingTask*)targetInfos[2].task);
		CreateMatchingNOT((MatchingTask*)targetInfos[3].task);


		//CreateMatchingIfThen((MatchingTask*)targetInfos[0].task, (MatchingTask*)targetInfos[1].task);
		
	/*	for(uint i = 0; i < targetInfos.size(); i++){
			ss.str("");
			ss << "matching" <<i;
			TargetInfo& info = targetInfos[i];
			info.hand = arm[info.left_or_right].hand;
			info.task = CreateMatchingTask(ss.str(), info.hand, info.target,timeslots[0], true);
		}*/

	// ticks
	uint N = 16; //num of event
	const double h = 0.4;
	for(uint i =0 ; i < N ; i++)
		AddTick((double)i * h);

	//SetGravity(vec3_t(0.0, -9.8, 0.0));
	//SetGravity(vec3_t(0.0, -0.0, 0.0));

	// 係数のスケーリング
	SetScaling( 1.0, 1.0, 1.0);

	// initialize graph
	Init();
	
	//task0 and task1 are continuity
	//Task::MakeContinuity(targetInfos[0].task, targetInfos[1].task);
	//Task::MakeContinuity(targetInfos[1].task, targetInfos[2].task);
	//HoldingTask::SetReachHold(targetInfos[0].task, (HoldingTask*)targetInfos[1].task);

	//// take OR composition of tasks
	//MatchingTask::ComposeOR(
	//	targetInfos[0].task,
	//	targetInfos[1].task
	//	);
}

void Humanoid::Draw(DrawContext* draw, DrawConfig* conf){
	
	body.base->DrawSnapshot(ticks.front()->time, draw, conf);

	for(uint i = 0; i < links.size() ; i++){
		//links[i]->DrawSnapshot(ticks.front()->time, draw, conf);
		//links[i]->DrawSnapshot(ticks.back()->time, draw, conf);
	
		/*foreach(TimeSlot* ts, timeslots){
			links[i]->DrawSnapshot(ts->time_s->val, draw, conf);
			links[i]->DrawSnapshot(ts->time_e->val, draw, conf);
		}*/
	}

	// ハンドとボールと箱の軌道を描画
	foreach(TargetInfo& info, targetInfos){
		info.hand->DrawTrajectory(draw, conf);	
		info.target->DrawTrajectory(draw, conf);
	}
}

void Humanoid::SetInitialState(){
	// 順機構学で初期位置を計算
	body.base->ForwardKinematics();
	body.base->SetDynamical(false);

	// 目標物体の全時刻の位置と速度を初期値に固定
	foreach(Tick* tick, ticks){
		foreach(TargetInfo& info, targetInfos){
			ObjectKey* key = info.target->GetKeypoint(tick);
			key->vel_t->val = info.iniVel;
			key->pos_t->val = info.iniPos + key->vel_t->val * tick->time;
			key->pos_t->Lock();
			key->pos_r->Lock();
			key->vel_t->Lock();
			key->vel_r->Lock();
		}
	}

	// 関節を初期値に固定
	foreach(Tick* tick, ticks){
		foreach(Joint* jnt, joints){
			Joint1DKey* key = ((Joint1D*)jnt)->GetKeypoint(tick);
			key->pos->Lock();
			key->vel->Lock();
		}
	}

	// 箱の保持時間を初期値に固定
	foreach(TimeSlot* ts, timeslots){
		ts->time_s->Lock();
		ts->time_e->Lock();
	}

	// 幾何拘束以外の全拘束の無効化
	Enable(ID(), false);
	Enable(ID(KINEMATIC_POS_T), true);
	Enable(ID(KINEMATIC_POS_R), true);
	Enable(ID(KINEMATIC_VEL_T), true);
	Enable(ID(KINEMATIC_VEL_R), true);
}

void Humanoid::ChargeForce(){
	//// 力のつり合い拘束を有効化
	//Enable(ID(NETFORCE_T), true);
	//Enable(ID(NETFORCE_R), true);
	//Enable(ID(CONSTANT_VEL_T), true);
}

void Humanoid::LiftOff(){
	// 初期時刻以外の関節の固定を解除
	foreach(Tick* tick, ticks){
		foreach(Joint* jnt, joints){
			Joint1DKey* key = ((Joint1D*)jnt)->GetKeypoint(tick);
			key->pos->Lock(false);
			key->vel->Lock(false);
		}
	}
	
	// 関節速度0（全時刻）
	foreach(Joint* jnt, joints){
		SetDesired(ID(JOINT_VEL, jnt), 0.0);
		SetPriority(ID(DESIRED + JOINT_VEL, jnt), 3);
	}

	//連続性拘束を有効に
	Enable(ID(OBJECT_QUADRATIC_T), true);
	Enable(ID(OBJECT_QUADRATIC_R), true);
	Enable(ID(JOINT_POS + RANGE), true);
}

void Humanoid::TaskStart(){
	// モノを動かすためにターゲットの時間の拘束を解除
	foreach(Tick* tick, ticks){
		foreach(TargetInfo& info, targetInfos){
			ObjectKey* key = info.target->GetKeypoint(tick);
			key->pos_t->Lock(false);
			key->pos_r->Lock(false);
			key->vel_t->Lock(false);
			key->vel_r->Lock(false);
		}
	}

	// scheduling enable
	foreach(TimeSlot* ts, timeslots){
		ts->time_s->Lock(false);
		ts->time_e->Lock(false);
	}
	foreach(TargetInfo& info, targetInfos){	
		// position_matching制約有効化，優先度1
		Enable(ID(MATCHING, info.task), true);
		SetPriority(ID(MATCHING,  info.task), 1);
	}
	SetPriority(ID(MATCHING,  targetInfos[0].task), 1);
	SetPriority(ID(MATCHING,  targetInfos[1].task), 1);
	// holding task の制約有効化，優先度1
	//Enable(ID(HOLDING, (HoldingTask*)targetInfos[1].task), true);
	//SetPriority(ID(HOLDING,  (HoldingTask*)targetInfos[1].task), 1);
	// タイミング変数の最適化
	foreach(TargetInfo& info, targetInfos){	
		SetDesired(ID(TIME_START, info.task), 0.0);
		SetPriority(ID(DESIRED+TIME_START,  info.task), 2);
		SetDesired(ID(TIME_END, info.task), 2.0);
		SetPriority(ID(DESIRED+TIME_END,  info.task), 2);
	}	



	// 目標値を0とし，優先度を2に設定
		/*SetDesired(ID(TIME_S, position_matchings[i], 0), 1.0);
		SetPriority(ID(DESIRED + TIME_C, position_matchings[i], 0), balls.size() + boxes.size() + 3);*/
	//}
}

