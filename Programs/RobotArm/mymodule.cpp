#include <module/rendering.h>
#include "mymodule.h"
#include "RobotArm.h"

#include <DiMP/Load/Loader.h>

#include <sbcsv.h>

///////////////////////////////////////////////////////////////////////////////////////////////////

MyModule::Config::Welding::Welding() {
	pointsFilename = "";
	startTime = 0.0;
	endTime = 0.0;
	numTicks = 0;
	mockupOffset = vec3_t();
	useTree = false;
}

///////////////////////////////////////////////////////////////////////////////////////////////////

MyModule::MyModule() {
	sceneSelect = Reaching2D;

}

MyModule::~MyModule() {

}

void MyModule::Read(XML& xml) {
	if (sceneSelect == Welding) {
		xml.Get(conf.welding.pointsFilename, ".points_filename");
		xml.Get(conf.welding.startTime, ".start_time");
		xml.Get(conf.welding.endTime, ".end_time");
		xml.Get(conf.welding.numTicks, ".num_ticks");
		xml.Get(conf.welding.mockupOffset, ".mockup_offset");
		xml.Get(conf.welding.useTree, ".use_tree");
	}
}

bool MyModule::Build() {
	XML xml;

	// シーン選択
	xml.Load("conf/robotarm/setting.xml");
	string str;
	xml.Get(str, ".scene");
	if (str == "reaching2d") sceneSelect = Reaching2D;
	if (str == "reaching3d") sceneSelect = Reaching3D;
	if (str == "toss3d") sceneSelect = Toss3D;
	if (str == "welding") sceneSelect = Welding;

	// シーン別設定ファイル
	switch (sceneSelect) {
	case Reaching2D: sceneFilename = "conf/robotarm/scene_reaching2d.xml"; break;
	case Reaching3D: sceneFilename = "conf/robotarm/scene_reaching3d.xml"; break;
	case Toss3D: sceneFilename = "conf/robotarm/scene_toss3d.xml"; break;
	case Welding: sceneFilename = "conf/robotarm/scene_welding.xml"; break;
	}
	switch (sceneSelect) {
	case Reaching2D: break;
	case Reaching3D: break;
	case Toss3D: break;
	case Welding: confFilename = "conf/robotarm/welding.xml"; break;
	}

	// コンフィグ
	if (!confFilename.empty()) {
		XML confXml;
		confXml.Load(confFilename);
		Read(confXml);
	}

	// Scenebuiler::Builderでパース
	typedb.Register();
	scene.Create();
	scene.CreateObject(NamespaceProp::id, "world", &typedb);
	models.Set(&scene);
	builder.Set(&scene, &typedb, &models);
	builder.Parse(sceneFilename);

	// 各アダプタでミラーリング
	adaptorDiMP.Set(graph);

	Adaptor* adaptors[] = { &adaptorDiMP, &adaptorSprGR };
	for each(Adaptor* a in adaptors) {
		a->Set(&scene, &typedb, &models);
		a->Mirror();
		a->SyncProperty(true);
	}

	// ロボットを抽出
	for (int i = 0; ; i++) {
		UTRef<RobotArm>	r = new RobotArm();
		if (r->Build(i, &adaptorDiMP))
			robot.push_back(r);
		else break;
	}

	// 目標物体を抽出
	stringstream ss;
	for (int i = 0; ; i++) try {
		ss.str("");
		ss << "workspace/body_target" << i;
		DiMP::Object* obj = adaptorDiMP.GetObject(ss.str());
		target.push_back(obj);
	}
	catch (...) { break; }

	// 障害物を抽出
	for (int i = 0; ; i++) try {
		ss.str("");
		ss << "workspace/body_obstacle" << i;
		DiMP::Object* obj = adaptorDiMP.GetObject(ss.str());
		obstacle.push_back(obj);
	}
	catch (...) { break; }

	adaptorSprGR.ShowSolid(true);
	adaptorSprGR.ShowWireframe(false);

	// DiMP2依存設定項目をLoaderで処理
	DiMP::Loader loader;
	xml.Load(sceneFilename);
	loader.Load(xml, graph);

	// Init前の処理
	if (sceneSelect == Welding) {
		// tree生成
		if (conf.welding.useTree) {
			DiMP::Tree* tree = new DiMP::Tree(graph, "tree");
			tree->root = robot[0]->link[0];
		}

		// 溶接点列を読み込み
		CsvReader csv;
		csv.Read(conf.welding.pointsFilename, ",");
		int nrow = csv.NumRow();
		weldingPoints.resize(nrow);
		for (int i = 0; i < nrow; i++) {
			// mm -> m
			// オフセット加算
			weldingPoints[i] = conf.welding.mockupOffset + 0.001 * vec3_t(csv.Get<real_t>(i, 0), csv.Get<real_t>(i, 1), csv.Get<real_t>(i, 2));
		}

		// tick生成
		if (conf.welding.startTime != 0.0)
			new DiMP::Tick(graph, 0.0);
		for (int i = 0; i <= conf.welding.numTicks; i++) {
			real_t t = conf.welding.startTime + ((conf.welding.endTime - conf.welding.startTime) / (real_t)conf.welding.numTicks) * (real_t)i;
			new DiMP::Tick(graph, t);
		}

		//初期
		
		// マッチングタスク生成(肘e1をモックアップ円筒前に移動)
		// マッチングタスク生成(手w2をモックアップ円筒前に移動)
		DiMP::TimeSlot* timeSlot0 = new DiMP::TimeSlot(graph, 0.1, 0.5, true, "ts_welding");
		DiMP::MatchTask* match0a = new DiMP::MatchTask(robot[0]->link[4], target[1], timeSlot0, "match_shoki_e1_welding");
		DiMP::MatchTask* match0b = new DiMP::MatchTask(robot[0]->link[8], target[2], timeSlot0, "match_shoki_y1welding");
		DiMP::MatchTask* match0c = new DiMP::MatchTask(robot[0]->link.back(), target[3], timeSlot0, "match_shoki_tesaki_welding");
		
		//溶接時
		/*
		// タイムスロット
		DiMP::TimeSlot* timeSlot = new DiMP::TimeSlot(graph, conf.welding.startTime, conf.welding.endTime, true, "ts_welding");
		// マッチングタスク生成
		new DiMP::MatchTask(robot[0]->link.back(), target[0], timeSlot, "match_welding");
		*/

		
		// マッチングタスク生成（円筒付近でe1を固定）
		// マッチングタスク生成（円筒付近でw2を固定）
		DiMP::TimeSlot* timeSlot1 = new DiMP::TimeSlot(graph, 1.0, 4.1, true, "ts_welding");
		DiMP::MatchTask* match1a = new DiMP::MatchTask(robot[0]->link[4], target[4], timeSlot1, "match_entou_e1_welding");
		DiMP::MatchTask* match1b = new DiMP::MatchTask(robot[0]->link[8], target[5], timeSlot1, "match_entou_y1_welding");
		match1b->param.match_tp = true;
		match1b->param.match_rp = true;
		
		
		// マッチングタスク生成（下カーブで肘e1を固定）
		// マッチングタスク生成（下カーブで手首w2を固定）
		DiMP::TimeSlot* timeSlot2 = new DiMP::TimeSlot(graph, 4.11, 7.1, true, "ts_welding");
		DiMP::MatchTask* match2a = new DiMP::MatchTask(robot[0]->link[4], target[6], timeSlot2, "match_undercurve_e1_welding");
		DiMP::MatchTask* match2b = new DiMP::MatchTask(robot[0]->link[8], target[7], timeSlot2, "match_undercurve_y1_welding");
		match2b->param.match_tp = true;
		match2b->param.match_rp = true;
		
		
		// マッチングタスク生成（上カーブで肘e1を固定）
		// マッチングタスク生成（上カーブで手首w2を固定）
		DiMP::TimeSlot* timeSlot3 = new DiMP::TimeSlot(graph, 7.11, 10.0, true, "ts_welding");
		DiMP::MatchTask* match3a = new DiMP::MatchTask(robot[0]->link[4], target[8], timeSlot3, "match_overcurve_e1_welding");
		DiMP::MatchTask* match3b = new DiMP::MatchTask(robot[0]->link[8], target[9], timeSlot3, "match_overcurve_y1_welding");
		match3b->param.match_tp = true;
		match3b->param.match_rp = true;
		


		// 回避タスク生成
		stringstream ss;
		for (int i = 0; i < (int)robot[0]->link.size(); i++) {
			//if(i == 11){
			//	ss.str("");
			//	ss << "avoid" << i;
			//	new DiMP::AvoidTask(robot[0]->link[i], obstacle[0], timeSlot, ss.str());
			//}
		}
	}

	// 初期化
	graph->Init();

	// Init後の処理
	if (sceneSelect == Reaching2D) {
	}
	if (sceneSelect == Reaching3D) {
		// 低優先度で関節速度を0にする
		for (uint i = 0; i < robot[0]->joint.size(); i++) {
			robot[0]->joint[i]->param.rmin_v[0] = 0.0;
			robot[0]->joint[i]->param.rmax_v[0] = 0.0;
		}
	}
	if (sceneSelect == Toss3D) {
		// 低優先度で関節速度を0にする
		for (uint i = 0; i < robot[0]->joint.size(); i++) {
			robot[0]->joint[i]->param.rmin_v[0] = 0.0;
			robot[0]->joint[i]->param.rmax_v[0] = 0.0;
		}
		for (uint i = 0; i < robot[1]->joint.size(); i++) {
			robot[1]->joint[i]->param.rmin_v[0] = 0.0;
			robot[1]->joint[i]->param.rmax_v[0] = 0.0;
		}
	}
	if (sceneSelect == Welding) {
		// 目標点列の軌跡を溶接点列に合わせて固定
		int npoints = (int)weldingPoints.size();
		for (int k = 0; k < (int)graph->ticks.size(); k++) {
			DiMP::ObjectKey* key = (DiMP::ObjectKey*)target[0]->traj.GetKeypoint(graph->ticks[k]);
			real_t t = graph->ticks[k]->time;

			int idx = ((t - conf.welding.startTime) / (conf.welding.endTime - conf.welding.startTime)) * npoints;
			idx = std::min(std::max(0, idx), npoints - 1);

			key->pos_t->val = weldingPoints[idx];
			key->pos_r->val = quat_t();
			key->vel_t->val = vec3_t();
			key->vel_r->val = vec3_t();

			key->pos_t->locked = true;
			key->pos_r->locked = true;
			key->vel_t->locked = true;
			key->vel_r->locked = true;
		}

		// 初期軌道を設定
		for (int k = 0; k < (int)graph->ticks.size(); k++) {
			for (int j = 0; j < (int)robot[0]->joint.size(); j++) {
				// 関節jの時刻kのキーポイント
				DiMP::JointKey* key = (DiMP::JointKey*)robot[0]->joint[j]->traj.GetKeypoint(graph->ticks[k]);

				// 関節角を0.5[rad]にする場合
				if (j == 1)
					key->pos[0]->val = Rad(90.0);
				else key->pos[0]->val = Rad(0.0);
			}
		}
	}

	for (uint i = 0; i < robot.size(); i++) {
		RobotArm* r = robot[i];
		// 順運動学計算でリンク位置を決定
		r->link[0]->ForwardKinematics();
	}

	return true;
}

bool MyModule::OnRequest() {
	return Module::OnRequest();
}

void MyModule::OnStep() {
	Module::OnStep();
}

void MyModule::DrawSnapshot(real_t time) {
	//adaptorDiMP .SetSyncTime(time);
	//adaptorDiMP .graph->CreateSnapshot(time);
	robot[0]->link[0]->ForwardKinematics(time);
	adaptorDiMP.SyncProperty(false, AttrCategory::State);
	adaptorSprGR.SyncProperty(true, AttrCategory::State);

	adaptorSprGR.Draw(renManager->render);
}

void MyModule::OnDraw(DiMP::Render::Canvas* canvas) {
	//// 再生モードの場合は再生時刻のスナップショットを描画
	//if(GetAction(MENU_ALWAYS, ID_PLAY)->GetBool()){
	//	DrawSnapshot(render, playTime);
	//}
	//// それ以外は初期時刻と各タスクの開始・終了時刻のスナップショットを描画
	//else{
	//	DrawSnapshot(render, 0.0);
	//	for(uint i = 0; i < workspace.task.size(); i++){
	//		DiMP::Task* task = workspace.task[i];
	//		DrawSnapshot(render, task->time->time_s->val);
	//		DrawSnapshot(render, task->time->time_e->val);
	//	}
	//}

	// 光源設定
	GRLightDesc ld;
	ld.diffuse = Vec4f(0.6f, 0.6f, 0.6f, 1.0f);
	ld.specular = Vec4f(0.0f, 0.0f, 0.0f, 1.0f);
	ld.ambient = Vec4f(0.1f, 0.1f, 0.1f, 1.0f);
	ld.position = Vec4f(20.0f, 50.0f, 20.0f, 1.0f);
	renManager->render->PushLight(ld);

	renManager->render->SetLighting(true);
	if (isPlaying) {
		DrawSnapshot(playTime);
	}
	else {
		DrawSnapshot(0.0);
		for (uint i = 0; i < graph->tasks.size(); i++) {
			DiMP::Task* task = graph->tasks[i];
			DrawSnapshot(task->time->time_s->val);
			DrawSnapshot(task->time->time_e->val);
		}
	}
	renManager->render->PopLight();

	renManager->render->SetLighting(false);

	if (sceneSelect == Welding) {
		// 溶接点列
		canvas->SetPointSize(3.0f);
		canvas->SetPointColor("magenta");
		Vec3f p;
		for (uint i = 0; i < (uint)weldingPoints.size(); i++) {
			p = weldingPoints[i];
			canvas->Point(p);
		}

	}

	Module::OnDraw(canvas);
}
