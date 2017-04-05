#include "BallCathing.h"

//-------------------------------------------------------------------------------------------------
// MyDrawConfig

MyDrawConfig::MyDrawConfig(){
	// object trajectory: thick black line
	materials[MAT_OBJECT ].lineColor = Vec4f(0.0, 0.0, 0.0, 1.0);
	materials[MAT_OBJECT ].lineColorName = "black";
	materials[MAT_OBJECT ].lineWidth = 2;

	// joint: thin black line
	materials[MAT_JOINT  ].lineColor = Vec4f(0.0, 0.0, 0.0, 1.0);
	materials[MAT_JOINT  ].lineColorName = "darkblue";

	// forces: gray line
	materials[MAT_FORCE  ].lineColor = Vec4f(0.5, 0.5, 0.5, 1.0);
	materials[MAT_FORCE  ].lineColorName = "gray";

	materials[MAT_MOMENT ].lineColor = Vec4f(0.5, 0.5, 0.5, 1.0);
	materials[MAT_MOMENT ].lineColorName = "darkgray";

	// target posision: red large dot
	materials[MAT_TARGET ].lineColor = Vec4f(0.0, 0.0, 0.0, 1.0);
	materials[MAT_TARGET ].lineColorName = "red";
	materials[MAT_TARGET ].lineWidth = 2;

	// material for snapshot mode
	matSnapshot.lineColor = Vec4f(0.8, 0.0, 0.2, 1.0);
	matSnapshot.lineColorName = "magenta";
	matSnapshot.lineWidth = 2;

	scaleForce  = 0.01;
	scaleMoment = 0.1;
}

bool MyDrawConfig::Set(DrawContext* draw, ID id){
	if(snapshot){
		draw->mat = &matSnapshot;
		return true;
	}
	return DrawConfig::Set(draw, id);
}

//------------------------------------------------------------------------------------------------
// Humanoid


//double t0 = 3.0;

void BallCatching2::Build(){
	const double S = 0.5;
	double m, I, L, R;
	double V, t0;	//ボールの速度、タイミング変数初期値
	V = 0.30;
	t0 = 3.0;
	
		// body:base
		m = 1.0;
		I = 0.01;
		L = 0.8*S;
		R = 0.9*S;
		body.base = CreateObject();
		body.base->dynamical = false;
		body.base->AddGeometry(CreateCylinder(R, L), Posed(Vec3d(), Quaterniond::Rot(Rad(90.0), 'x')));
		links.push_back(body.base);

		// arm
		// hand
		m = 1.0;
		I = 0.01;
		L = 1.6*S;
		R = 0.15*S;
		arm.hand = CreateObject();
		arm.hand->mass = m;
		arm.hand->inertia = I;
		arm.hand->length = L;
		arm.hand->AddGeometry(CreateCylinder(R, L),
			Posed(Vec3d(0.0, -0.5*L, 0.0), Quaterniond::Rot(Rad(90.0), 'x')));
		arm.hand->AddGeometry(CreateSphere(R), Posed(Vec3d(0.0, 0.0, 0.0)));

		// lower arm
		L = 1.6*S;
		R = 0.25*S;
		arm.lower = CreateObject();
		arm.lower->mass = m;
		arm.lower->inertia = I;
		arm.lower->length = L;
		arm.lower->AddGeometry(CreateCylinder(R, L), Posed(Vec3d(), Quaterniond::Rot(Rad(90.0), 'x')));

		// upper arm
		L = 0.4 * S;
		R = 0.3 * S;
		arm.upper = CreateObject();
		arm.upper->mass = m;
		arm.upper->inertia = I;
		arm.upper->length = L;
		arm.upper->AddGeometry(CreateCylinder(R, L), Posed(Vec3d(), Quaterniond::Rot(Rad(90.0), 'x')));

		//elbow:hand-lowerarm
		arm.elbow = CreateHinge(arm.lower, arm.hand,
			Posed(Vec3d(0.0, 0.8*S, 0.0)),
			Posed(Vec3d(0.0, -1.6*S, 0.0)));
		arm.elbow->SetRange(Rad(0.0), Rad(150.0));

		//twist:lowerarm-upperarm
		arm.twist = CreateHinge(arm.upper, arm.lower,
			Posed(Vec3d(0.0, 0.2*S, 0.0)),
			Posed(Vec3d(0.0, -0.8*S, 0.0)));
		arm.twist->SetRange(Rad(0.0), Rad(150.0));

		//shoulder:upperarm-base
		arm.shoulder = CreateHinge(body.base, arm.upper,
			Posed(Vec3d(0.0, 0.6*S, 0.0), Quaterniond::Rot(Rad(90.0), 'x')),
			Posed(Vec3d(), Quaterniond::Rot(Rad(90.0), 'x')));
		arm.shoulder->SetRange(Rad(-180.0), Rad(180.0));

		links.push_back(arm.hand);
		links.push_back(arm.lower);
		links.push_back(arm.upper);
		joints.push_back(arm.elbow);
		joints.push_back(arm.twist);
		joints.push_back(arm.shoulder);

		//create balls
		//for(t0 = 0.0; t0 <= 5.0; t0 += 0.50) {
		//	//BallInfo(Vec3d p, Vec3d v, double r, double t): iniPos(p), iniVel(v), radius(r), iniCatchTime(t){}
		//	ballInfos.push_back(BallInfo(S * Vec3d( 3.5, 3.5, 3.5), V * Vec3d(-1.0, -0.5, -2.0), 0.3, t0));
		//}

		//BallInfo(Vec3d p, Vec3d v, double r, double t): iniPos(p), iniVel(v), radius(r), iniCatchTime(t){}
		ballInfos.push_back(BallInfo(S * Vec3d( 3.5, 3.5, 3.5), V * Vec3d(-1.0, -0.5, -2.0), 0.3, t0));
		//ballInfos.push_back(BallInfo(S * Vec3d( 3.5, 3.5, -3.5), V * Vec3d(-1.0, -0.5, -2.0), 0.3, t0));
		uint nBalls = ballInfos.size();
		balls.resize(nBalls);
		
		for(uint i = 0; i < nBalls; i++){
			balls[i] = CreateObject();
			balls[i]->AddGeometry(CreateSphere(ballInfos[i].radius), Posed());
		}

	// position_matchings nodes
	position_matchings.resize(nBalls);	
	for(uint i = 0; i < balls.size() ; i++){
		position_matchings[i] = CreatePositionMatching(arm.hand, balls[i],0.0);
	}

	// or_composition nodes
	//or_compositions.resize(1);
	//for(uint i=0; i < or_compositions.size(); i++){
	//	or_compositions[i] = CreateOR_Composition(position_matchings[0],position_matchings[1]);
	//}
	
		
	// events
	uint N = 16; //num of event
	//uint N = 2;
	const double h = 0.4;
	for(uint i =0 ; i < N ; i++)
		CreateEvent((double)i * h);

	//SetGravity(Vec3d(0.0, -9.8, 0.0));
	SetGravity(Vec3d(0.0, -0.0, 0.0));

	// initialize graph
	Init();

}

void BallCatching2::Draw(DrawContext* draw, DrawConfig* conf){
	// 初期時刻，終端時刻およびキャッチング時刻のスナップショットを描画
	conf->Set(draw, ID(OBJECT_POS_T));

	draw->mat->lineWidth = 1;
	draw->mat->lineColor = Vec4f(0.4, 0.4, 0.4, 1.0);
	body.base->DrawSnapshot(events.front()->time, draw, conf);

	for(uint i = 0; i < links.size() ; i++){
		links[i]->DrawSnapshot(events.front()->time, draw, conf);
		links[i]->DrawSnapshot(events.back()->time, draw, conf);
	}
	for(uint i = 0; i < links.size() ; i++){
		for(uint j = 0; j < position_matchings.size(); j++)
			links[i]->DrawSnapshot(position_matchings[j]->time_s->val, draw, conf);
	}
	for(uint i = 0; i < balls.size(); i++){
		balls[i]->DrawSnapshot(position_matchings[i]->time_s->val, draw, conf);
	}

	

	//body.waist->DrawSnapshot(events.front()->time, draw, conf);
	// ハンドとボールの軌道を描画
	draw->mat->lineWidth = 3;
	draw->mat->lineColor = Vec4f(0.0, 0.0, 0.7, 1.0);
	if(balls.size())
	arm.hand->DrawTrajectory(draw, conf);
	draw->mat->lineWidth = 2;
	draw->mat->lineColor = Vec4f(0.6, 0.1, 0.1, 1.0);
	for(uint i = 0; i < balls.size(); i++)
		balls[i]->DrawTrajectory(draw, conf);
}

void BallCatching2::SetInitialState(){
	// 順機構学で初期位置を計算
	body.base->ForwardKinematics();

	// ベースリンクの全時刻の位置・速度を固定
	for(uint t = 0; t < events.size(); t++){
		ObjectKeypoint* st = body.base->GetKeypoint(events[t]);
		st->pos_t->Lock();
		st->vel_t->Lock();
		st->pos_r->Lock();
		st->vel_r->Lock();
	}

	// ボールの全時刻の位置と速度を初期値に固定
	for(uint t = 0; t < events.size(); t++){
		for(uint i = 0; i < balls.size() ; i++){
			ObjectKeypoint* st = balls[i]->GetKeypoint(events[t]);;
			st->vel_t->val = ballInfos[i].iniVel;
			st->pos_t->val = ballInfos[i].iniPos + st->vel_t->val * st->ev->time;
			st->pos_t->Lock();
			st->pos_r->Lock();
			st->vel_t->Lock();
			st->vel_r->Lock();
		}
	}

	// 関節を初期値に固定
	for(uint t = 0; t < events.size(); t++){
		for(uint i = 0; i < joints.size()-1; i++){
			Joint1DKeypoint* st = joints[i]->GetKeypoint(events[t]);
			st->pos->Lock();
			st->vel->Lock();
		}
	}

	/*
	// 各リンク剛体の全時刻の位置・速度を固定
	for(uint t = 0; t < events.size(); t++){
	for(uint i = 0; i < links.size(); i++){
	ObjectKeypoint* st = links[i]->GetKeypoint(events[t]);
	//st->pos_t->val = linkInfos[i].inipos;
	st->pos_t->Lock();
	st->pos_r->Lock();
	st->vel_t->Lock();
	st->vel_r->Lock();
	}
	}
	*/
	// ボールのキャッチング時刻を初期値に固定
	for(uint i = 0; i < balls.size() ; i++){
		position_matchings[i]->time_s->val = ballInfos[i].iniCatchTime;
		position_matchings[i]->time_e->val = ballInfos[i].iniCatchTime;

		position_matchings[i]->time_s->Lock();
		position_matchings[i]->time_e->Lock();
	}
	

	// 幾何拘束以外の全拘束の無効化
	Enable(ID(), false);
	Enable(ID(KINEMATIC_POS_T), true);
	Enable(ID(KINEMATIC_POS_R), true);
	Enable(ID(KINEMATIC_VEL_T), true);
	Enable(ID(KINEMATIC_VEL_R), true);
}

void BallCatching2::ChargeForce(){
	//// 力のつり合い拘束を有効化
	Enable(ID(NETFORCE_T), false);
	Enable(ID(NETFORCE_R), false);
	SetPriority(ID(NETFORCE_T), balls.size()  + 1 );
	SetPriority(ID(NETFORCE_R), balls.size()  + 1 );
}

void BallCatching2::LiftOff(){
	// 初期時刻以外の関節の固定を解除
	for(uint t = 1; t < events.size(); t++){
		for(uint i = 0; i < joints.size(); i++){
			Joint1DKeypoint* st = joints[i]->GetKeypoint(events[t]);
			st->pos->Lock(false);
			st->vel->Lock(false);
		}
	}
	/*
	// 各リンク剛体の全時刻の位置・速度を固定を解除
	for(uint t = 1; t < events.size(); t++){
	for(uint i = 0; i < links.size(); i++){
	ObjectKeypoint* st = links[i]->GetKeypoint(events[t]);
	st->pos_t->Lock(false);
	st->pos_r->Lock(false);
	st->vel_t->Lock(false);
	st->vel_r->Lock(false);
	}
	}*/

	// 初期時刻以外のボール位置の固定を解除．
	for(uint t = 1; t < events.size(); t++){
		for(uint i = 0; i < balls.size(); i++){
			ObjectKeypoint* st = balls[i]->GetKeypoint(events[t]);
			st->pos_t->Lock(false);
		}
	}



	 //関節速度0（全時刻）, 優先度を2に設定
	/*for(uint i = 0; i < joints.size(); i++){
		SetDesired(ID(JOINT_VEL, joints[i]), 0.0);
		SetPriority(ID(DESIRED + JOINT_VEL, joints[i]), balls.size() + 2);
	}*/

	//連続性拘束を有効に
	Enable(ID(OBJECT_QUADRATIC_T), true);
	Enable(ID(OBJECT_QUADRATIC_R), true);
}

void BallCatching2::TaskStart(){
	for(uint i = 0; i < balls.size() ; i++){
		// このときはもうボールの位置は変えないよ！
		for(uint t = 0; t < events.size(); t++){
			ObjectKeypoint* st = balls[i]->GetKeypoint(events[t]);
			st->pos_t->Lock();
		}

		// ボールのキャッチング時刻の固定を解除
		//Lock(ID(TIME_S, position_matchings[i]), false);
		//Lock(ID(TIME_E, position_matchings[i]), false);
		// キャッチング制約有効化，優先度1
		SetPriority(ID(CATCHING, position_matchings[i]),  1);
		// 目標値を0とし，優先度を2に設定
		SetDesired(ID(TIME_S, position_matchings[i], 0), 0);
		SetPriority(ID(DESIRED + TIME_S, position_matchings[i], 0), balls.size() + 2);
	}

}

