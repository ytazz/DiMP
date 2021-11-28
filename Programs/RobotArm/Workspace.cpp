#include "Workspace.h"
#include "RobotArm.h"
#include <DiMP/Load/Loader.h>

#include <boost/lexical_cast.hpp>
using namespace boost;

//------------------------------------------------------------------------------------------------
// Workspace

Workspace::Workspace(){
	
}

Workspace::~Workspace(){
	
}

bool Workspace::Build(){
    // Scenebuiler::Builderでパース
	typedb.Register();
	scene.Create();
	scene.CreateObject(NamespaceProp::id, "world", &typedb);
	models.Set(&scene);
	builder.Set(&scene, &typedb, &models);
	builder.Parse(sceneFilename);

	// 各アダプタでミラーリング
	graph = new DiMP::Graph();
	adaptorDiMP.Set(graph);

	Adaptor* adaptors[] = { &adaptorDiMP, &adaptorSprGR };
	for(Adaptor* a : adaptors) {
		a->Set(&scene, &typedb, &models);
		a->Mirror();
		a->SyncProperty(true);
	}

	// DiMP依存設定項目をLoaderで処理
	DiMP::Loader loader;
    XML xml;
	xml.Load(sceneFilename);
	loader.Load(xml, graph);

	// ロボットを抽出
	for (int i = 0; ; i++) {
		UTRef<RobotArm>	r = new RobotArm();
		if (r->Build(i, graph))
			robot.push_back(r);
		else break;
	}

	// 目標物体を抽出
	stringstream ss;
	for (int i = 0; ; i++){
		ss.str("");
		ss << "workspace/body_target" << i;
		DiMP::Object* obj = graph->objects.Find(ss.str());
		if(!obj)
			break;
		target.push_back(obj);
	}

	// 障害物を抽出
	for (int i = 0; ; i++){
		ss.str("");
		ss << "workspace/body_obstacle" << i;
		DiMP::Object* obj = graph->objects.Find(ss.str());
		if(!obj)
			break;
		obstacle.push_back(obj);
	}

	for (int i = 0; ; i++){
		ss.str("");
		ss << "timeslot" << i;
		DiMP::TimeSlot* ts = graph->timeslots.Find(ss.str());
		if(!ts)
			break;
		timeSlot.push_back(ts);
	}
	for (int i = 0; ; i++){
		ss.str("");
		ss << "match" << i;
		DiMP::Task* task = graph->tasks.Find(ss.str());
		if(!task)
			break;
		matchTask.push_back((DiMP::MatchTask*)task);
	}
	for (int i = 0; ; i++){
		ss.str("");
		ss << "avoid" << i;
		DiMP::Task* task = graph->tasks.Find(ss.str());
		if(!task)
			break;
		avoidTask.push_back((DiMP::AvoidTask*)task);
	}

	adaptorSprGR.ShowSolid     (true);
	adaptorSprGR.ShowWireframe (true);
	adaptorSprGR.ShowPointcloud(true);

    return true;
}

void Workspace::DrawSnapshot(GRRenderIf* render, real_t time){
	graph->CreateSnapshot(time);
	robot[0]->link[0]->ForwardKinematics(time);
	adaptorDiMP.SyncProperty(false, AttrCategory::State);
	adaptorSprGR.SyncProperty(true, AttrCategory::State);

	adaptorSprGR.Draw(render);
}
