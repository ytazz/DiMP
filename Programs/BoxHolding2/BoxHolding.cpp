#include "BoxPositionMatching.h"

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
// Manipulator

void Manipulator::Build(){

	// object nodes
	base = CreateObject();
	links.resize(linkInfos.size());

	Posed pose;

	for(uint i = 0; i < links.size(); i++){
		links[i] = CreateObject();
		links[i]->mass = linkInfos[i].mass;
		links[i]->inertia = linkInfos[i].inertia;

		// assign geometries
		// hand
		if(i == links.size()-1){
			pose.Pos().y = -0.25 * linkInfos[i].length;
			pose.Ori() = Quaterniond::Rot(Rad(90.0), 'x');
			links[i]->AddGeometry(CreateCylinder(linkInfos[i].radius, 0.5 * linkInfos[i].length), pose);
			links[i]->AddGeometry(CreateSphere(linkInfos[i].radius), Posed());
		}
		// other links
		else{
			pose.Ori() = Quaterniond::Rot(Rad(90.0), 'x');
			links[i]->AddGeometry(CreateCylinder(linkInfos[i].radius, linkInfos[i].length), pose);
			//links[i]->AddGeometry(CreateBox(linkInfos[i].boxSize), Posed());
		}

	}

	// hinge nodes
	joints.resize(jointInfos.size());
	for(uint i = 0; i < joints.size(); i++)
		joints[i] = CreateHinge((i == 0 ? base : links[i-1]), links[i], jointInfos[i].socket, jointInfos[i].plug);

	// create boxes
	uint nBoxes = boxInfos.size();
	boxes.resize(nBoxes);
	
	for(uint i = 0; i < nBoxes; i++){
		boxes[i] = CreateObject();
		boxes[i]->mass = boxInfos[i].mass;
		boxes[i]->inertia = boxInfos[i].inertia;
		boxes[i]->AddGeometry(CreateBox(boxInfos[i].boxSize), Posed());
	}

	// position_matching nodes
	position_matchings.resize(nBoxes);
	for(uint i = 0; i < boxes.size() ; i++){
		position_matchings[i] = CreatePositionMatching(links.back(), boxes[i],false);
		position_matchings[i]->task_interval = 
			boxInfos[i].iniPositionMatchingEndTime -  boxInfos[i].iniPositionMatchingStartTime;//これもコンストラクタでやらんとかんね
	}

	// events
	uint N = 10; //num of event
	const double h = 0.6;
	for(uint i =0 ; i < N ; i++){
		CreateEvent((double)i * h);
	}

	//SetGravity(Vec3d(0.0, -9.8, 0.0));
	SetGravity(Vec3d(0.0, -0.0, 0.0));

	// initialize graph
	Init();

}

void Manipulator::SetInitialState(){
	// ベースリンクの全時刻の位置・速度を固定
	for(uint t = 0; t < events.size(); t++){
		ObjectKeypoint* st = base->GetKeypoint(events[t]);
		st->pos_t->Lock();
		st->vel_t->Lock();
		st->pos_r->Lock();
		st->vel_r->Lock();
	}

	// 箱の全時刻の位置と速度を初期値に固定
	for(uint t = 0; t < events.size(); t++){
		for(uint i = 0; i < boxes.size() ; i++){
			ObjectKeypoint* st = boxes[i]->GetKeypoint(events[t]);
			st->vel_t->val = boxInfos[i].iniVel; 
			st->pos_t->val = boxInfos[i].iniPos + st->vel_t->val * st->ev->time;
			st->pos_t->Lock();
			st->pos_r->Lock();
			st->vel_t->Lock();
			st->vel_r->Lock();
		}
	}

	// 各リンク剛体の全時刻の位置・速度を固定を解除
	for(uint t = 0; t < events.size(); t++){
		for(uint i = 0; i < links.size(); i++){
			ObjectKeypoint* st = links[i]->GetKeypoint(events[t]);
			st->pos_t->val = linkInfos[i].inipos;
			st->pos_t->Lock();
			st->pos_r->Lock();
			st->vel_t->Lock();
			st->vel_r->Lock();
		}
	}

	// 箱の保持時間を初期値に固定
	for(uint i = 0; i < boxes.size() ; i++){
		position_matchings[i]->time_s->val = boxInfos[i].iniPositionMatchingStartTime;
		//position_matchings[i]->time_s->val = boxInfos[i].iniPositionMatchingCompleteTime;
		position_matchings[i]->time_e->val = boxInfos[i].iniPositionMatchingEndTime;
		
		position_matchings[i]->time_s->Lock();
		position_matchings[i]->time_c->Lock();
		position_matchings[i]->time_e->Lock();
	}

	// 幾何拘束以外の全拘束の無効化
	Enable(ID(), false);
	Enable(ID(KINEMATIC_POS_T), true);
	Enable(ID(KINEMATIC_POS_R), true);
	Enable(ID(KINEMATIC_VEL_T), true);
	Enable(ID(KINEMATIC_VEL_R), true);
}

void Manipulator::ChargeForce(){
	//// 力のつり合い拘束を有効化
	Enable(ID(NETFORCE_T), true);
	Enable(ID(NETFORCE_R), true);
	SetPriority(ID(NETFORCE_T), boxes.size() + 1 );
	SetPriority(ID(NETFORCE_R), boxes.size() + 1 );
}

void Manipulator::LiftOff(){

	// 初期時刻以外の関節の固定を解除
	for(uint t = 1; t < events.size(); t++){
		for(uint i = 0; i < joints.size(); i++){
			Joint1DKeypoint* st = joints[i]->GetKeypoint(events[t]);
			st->pos->Lock(false);
			st->vel->Lock(false);
		}
	}
	// 各リンク剛体の全時刻の位置・速度を固定を解除
	for(uint t = 1; t < events.size(); t++){
		for(uint i = 0; i < links.size(); i++){
			ObjectKeypoint* st = links[i]->GetKeypoint(events[t]);
			st->pos_t->Lock(false);
			st->pos_r->Lock(false);
			st->vel_t->Lock(false);
			st->vel_r->Lock(false);
		}
	}
	// 初期時刻以外の箱の位置の固定を解除．
	for(uint t = 1; t < events.size(); t++){
		for(uint i = 0; i < boxes.size(); i++){
			ObjectKeypoint* st = boxes[i]->GetKeypoint(events[t]);
			st->pos_t->Lock(false);
		}
	}
	//連続性拘束を有効に
	Enable(ID(OBJECT_QUADRATIC_T), true);
	Enable(ID(OBJECT_QUADRATIC_R), true);
		// 関節速度0（全時刻）
	for(uint i = 0; i < joints.size(); i++){
		SetDesired(ID(JOINT_VEL, joints[i]), 0.0);
		SetPriority(ID(DESIRED + JOINT_VEL, joints[i]), boxes.size() + 2);
	}
}

void Manipulator::TaskStart(){

		// 箱の位置の固定．
	for(uint t = 1; t < events.size(); t++){
		for(uint i = 0; i < boxes.size(); i++){
			ObjectKeypoint* st = boxes[i]->GetKeypoint(events[t]);
			st->pos_t->Lock();
		}
	}

	for(uint i = 0; i < boxes.size() ; i++){
		Enable(ID(POSITION_MATCHING, position_matchings[i]), false);
		// 箱の保持時刻の固定を解除
		Lock(ID(TIME_S, position_matchings[i]), false);
		//Lock(ID(TIME_E, position_matchings[i]), false);//これも場当たり的な処理で本来はpriod_modeでうまく定義するべき
		// position_matching制約有効化，優先度1
		Enable(ID(POSITION_MATCHING, position_matchings[i]), true);
		SetPriority(ID(POSITION_MATCHING, position_matchings[i]), 1 * i + 1);
		// 目標値を0とし，優先度を2に設定
		SetDesired(ID(TIME_S, position_matchings[i], 0), 1.3);
		SetPriority(ID(DESIRED + TIME_C, position_matchings[i], 0), boxes.size() + 2);
	}
	
}

void Manipulator::SetTarget(uint idx, bool on, vec3_t pos, uint priority){
	if(graph->events.size() <= idx)
		return;

	DiMP2::Event* ev = graph->events[idx];
	if(on){
		graph->SetDesired(ID(OBJECT_POS_T, links.back(), ev), pos);
		graph->SetPriority(ID(DESIRED + OBJECT_POS_T, links.back(), ev), priority);
		graph->Enable(ID(DESIRED + OBJECT_POS_T, links.back(), ev), true);
	}
	else{
		graph->Enable(ID(DESIRED + OBJECT_POS_T, links.back(), ev), false);
	}
}

//------------------------------------------------------------------------------------------------
// BoxHolder

void BoxHolder::Build(){
	const double L = 0.5;

	jointInfos.resize(4);
	linkInfos.resize(4);
	obstInfos.resize(1);

	jointInfos[0].socket.Pos() = Vec3d(0.0,  0.0, 0.0);
	jointInfos[0].plug  .Pos() = Vec3d(0.0, -L, 0.0);
	jointInfos[1].socket.Pos() = Vec3d(0.0,  L, 0.0);
	jointInfos[1].plug  .Pos() = Vec3d(0.0, -L, 0.0);
	jointInfos[2].socket.Pos() = Vec3d(0.0,  L, 0.0);
	jointInfos[2].plug  .Pos() = Vec3d(0.0, -L, 0.0);
	jointInfos[3].socket.Pos() = Vec3d(0.0,  L, 0.0);
	jointInfos[3].plug  .Pos() = Vec3d(0.0, -L, 0.0);

	linkInfos[0].mass    = 1.0;
	linkInfos[0].inertia = 1.0;
	linkInfos[0].length = 2.0 * L;
	linkInfos[0].radius = 0.5 * L;
	linkInfos[0].inipos  = Vec3d (0.0,L,0.0);
	linkInfos[1].mass    = 1.0;
	linkInfos[1].inertia = 1.0;
	linkInfos[1].length = 2.0 * L;
	linkInfos[1].radius = 0.3 * L;
	linkInfos[1].inipos  = Vec3d (0.0,3.0*L,0.0);
	linkInfos[2].mass    = 1.0;
	linkInfos[2].inertia = 1.0;
	linkInfos[2].length = 2.0 * L;
	linkInfos[2].radius = 0.3 * L;
	linkInfos[2].inipos  = Vec3d (0.0,5.0*L,0.0);
	linkInfos[3].mass    = 1.0;
	linkInfos[3].inertia = 1.0;
	linkInfos[3].length = 2.0 * L;
	linkInfos[3].radius = 0.3 * L;
	linkInfos[3].inipos  = Vec3d (0.0,7*L,0.0);

	jointInfos[0].iniPos = 0.0;
	jointInfos[0].iniVel = 0.0;
	jointInfos[1].iniPos = 0.0;
	jointInfos[1].iniVel = 0.0;
	jointInfos[2].iniPos = 0.0;
	jointInfos[2].iniVel = 0.0;
	jointInfos[3].iniPos = 0.0;
	jointInfos[3].iniVel = 0.0;

	obstInfos[0].pos = Vec3d(0.4, 0.3, 0.0);
	obstInfos[0].radius = 0.1;

	jointInfos[0].socket.Ori() = Quaterniond::Rot(Rad(90.0), 'x');
	jointInfos[0].plug  .Ori() = Quaterniond::Rot(Rad(90.0), 'x');
	jointInfos[1].socket.Ori() = Quaterniond::Rot(Rad(0.0), 'z');
	jointInfos[1].plug  .Ori() = Quaterniond::Rot(Rad(0.0), 'z');
	jointInfos[2].socket.Ori() = Quaterniond::Rot(Rad(0.0), 'z');
	jointInfos[2].plug  .Ori() = Quaterniond::Rot(Rad(0.0), 'z');
	jointInfos[3].socket.Ori() = Quaterniond::Rot(Rad(0.0), 'z');
	jointInfos[3].plug  .Ori() = Quaterniond::Rot(Rad(0.0), 'z');
	
	// case A - 1
	boxInfos.push_back(BoxInfo(1.0, 1.0, L * vec3_t(0.5,0.5,0.5), L * vec3_t(2.5,4.0,-1.5),L * vec3_t(0.0,0.0,0.75) ,1.9, 3.9));
	
	Manipulator::Build();
}

void BoxHolder::Draw(DrawContext* draw, DrawConfig* conf){
	conf->Set(draw, ID(OBJECT_POS_T));

	draw->mat->lineWidth = 1;
	base->DrawSnapshot(events.front()->time, draw, conf);
		
	for(uint i = 0; i < links.size(); i++){
		//for(uint k = 0; k < events.size(); k++){
		//	links[i]->DrawSnapshot(events[k]->time, draw, conf);
		//}
		//links[i]->DrawSnapshot(events.front()->time, draw, conf);
		//links[i]->DrawSnapshot(events.back()->time, draw, conf);
		for(uint j = 0; j < position_matchings.size(); j++){
			links[i]->DrawSnapshot(events[0]->time, draw, conf);
			links[i]->DrawSnapshot(position_matchings[j]->time_s->val, draw, conf);
			links[i]->DrawSnapshot(position_matchings[j]->time_e->val, draw, conf);
		}
	}
	for(uint i = 0; i < boxes.size(); i++){
		boxes[i]->DrawSnapshot(position_matchings[i]->time_s->val, draw, conf);
		boxes[i]->DrawSnapshot(position_matchings[i]->time_e->val, draw, conf);
		//boxes[i]->DrawSnapshot(events.front()->time, draw, conf);
	}
	
	// ハンドと箱の軌道を描画
	draw->mat->lineWidth = 3;
	links.back()->DrawTrajectory(draw, conf);
	for(uint i = 0; i < boxes.size(); i++)
		boxes[i]->DrawTrajectory(draw, conf);
}
