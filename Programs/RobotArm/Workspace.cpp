#include "Workspace.h"
#include "RobotArm.h"
#include <DiMP/Load/Loader.h>

#include <boost/lexical_cast.hpp>
using namespace boost;

//------------------------------------------------------------------------------------------------
// Workspace

Workspace::Workspace(){
	sceneSelect = Reaching2D;

	numTimeSteps = 5;
	samplePeriod = 0.5;
}

Workspace::~Workspace(){

}

void Workspace::Build(FWSdkIf* sdk){
	XML xml;

	// シーン選択
	xml.Load("conf/robotarm/setting.xml");
	string str;
	xml.Get(str, ".scene");
	if(str == "reaching2d") sceneSelect = Reaching2D;
	if(str == "reaching3d") sceneSelect = Reaching3D;
	if(str == "toss3d"    ) sceneSelect = Toss3D;

	// シーン別設定ファイル
	string filename;
	switch(sceneSelect){
	case Reaching2D: filename = "conf/robotarm/scene_reaching2d.xml"; break;
	case Reaching3D: filename = "conf/robotarm/scene_reaching3d.xml"; break;
	case Toss3D    : filename = "conf/robotarm/scene_toss3d.xml"    ; break;
	}

	// Scenebuiler::Builderでパース
	Parse(filename);
	
	// 各アダプタでミラーリング
	adaptorDiMP.Set(graph);
	adaptorSprPH.Set(sdk->GetPHSdk(), sdk->GetScene()->GetPHScene());

	Adaptor* adaptors[] = {&adaptorDiMP, &adaptorSprGR, &adaptorSprPH};
	for each(Adaptor* a in adaptors){
		a->Set(scene, typedb, models);
		a->Mirror();
		a->SyncProperty(true);
	}

	adaptorSprGR.ShowSolid    (true );
	adaptorSprGR.ShowWireframe(false);

	// DiMP2依存設定項目をLoaderで処理
	DiMP::Loader loader;
	xml.Load(filename);
	loader.Load(xml, graph);

	// ロボットを抽出
	for(int i = 0; ; i++){
		UTRef<RobotArm>	r = new RobotArm();
		if(r->Build(i, &adaptorDiMP))
			robot.push_back(r);
		else break;
	}

	// 目標物体を抽出
	for(int i = 0; ; i++) try{
		DiMP::Object* obj = adaptorDiMP.GetObject("workspace/body_target" + lexical_cast<string>(i));
		target.push_back(obj);
	} catch(...){ break; }

	// 障害物を抽出
	for(int i = 0; ; i++) try{
		DiMP::Object* obj = adaptorDiMP.GetObject("workspace/body_obstacle" + lexical_cast<string>(i));
		obstacle.push_back(obj);
	} catch(...){ break; }

	if(sceneSelect == Reaching2D){
		graph->Init();
	}
	if(sceneSelect == Reaching3D){	
		graph->Init();	

		// 低優先度で関節速度を0にする
		for(uint i = 0; i < robot[0]->joint.size(); i++){
			robot[0]->joint[i]->param.rmin_v[0] = 0.0;
			robot[0]->joint[i]->param.rmax_v[0] = 0.0;
		}
	}
	if(sceneSelect == Toss3D){
		graph->Init();

		// 低優先度で関節速度を0にする
		for(uint i = 0; i < robot[0]->joint.size(); i++){
			robot[0]->joint[i]->param.rmin_v[0] = 0.0;
			robot[0]->joint[i]->param.rmax_v[0] = 0.0;
		}
		for(uint i = 0; i < robot[1]->joint.size(); i++){
			robot[1]->joint[i]->param.rmin_v[0] = 0.0;
			robot[1]->joint[i]->param.rmax_v[0] = 0.0;
		}
	}

	// ソルバオプションの設定
	graph->solver->SetAlgorithm(DiMP::Algorithm::Pareto);
	//graph.solver.SetAlgorithm(DiMP2::Algorithm::Steepest);
	graph->solver->weights.resize(4);
	graph->solver->weights[0] = 10.0;
	graph->solver->weights[1] = 1.0;
	graph->solver->weights[2] = 5.0;
	graph->solver->weights[3] = 0.5;
	graph->solver->SetIterationOrder(DiMP::Iteration::Sequential);

	for(uint i = 0; i < robot.size(); i++){
		RobotArm* r = robot[i];
		// 順機構学計算でリンク位置を決定
		r->link[0]->ForwardKinematics();
	}

	// ログ有効化
	graph->solver->EnableLogging(DiMP::Logging::MajorLoop, true);
}

void Workspace::CheckCollision(){
	// ロボットハンドと障害物の距離をCSVに出力
	ofstream ofs("collision.csv");

	real_t T = graph->ticks.back()->time - graph->ticks.front()->time;
	real_t h = 0.01;
	for(real_t t = 0.0; t <= T; t += h){
		vec3_t pos_hand = robot[0]->hand->Pos(t);
		vec3_t pos_obst = obstacle[0]->Pos(t);
		real_t d = (pos_hand - pos_obst).norm();

		ofs << t << ", " << d << endl;
	}

	ofs.close();
}

void Workspace::CheckJointError(){
	// ロボットハンドと障害物の距離をCSVに出力
	ofstream ofs("joint.csv");

	real_t T = graph->ticks.back()->time - graph->ticks.front()->time;
	real_t h = 0.01;
	uint njoint = robot[0]->joint.size();
	vec3_t pos_dev;
	vec3_t ori_dev;
	for(real_t t = 0.0; t <= T; t += h){
		real_t d = 0.0;
		for(uint j = 0; j < njoint; j++){
			robot[0]->joint[j]->CalcDeviation(t, pos_dev, ori_dev);
			d += pos_dev.square();// + ori_dev.square();
		}
		d = sqrt(d);
		ofs << t << ", " << d << endl;
	}

	ofs.close();
}

void Workspace::WriteHandAndTarget(){
	ofstream ofs("hand.csv");

	real_t T = graph->ticks.back()->time - graph->ticks.front()->time;
	real_t h = 0.01;
	
	for(real_t t = 0.0; t <= T; t += h){
		vec3_t phand[2];
		vec3_t ptarget;
		phand[0] = robot[0]->hand->Pos(t);
		phand[1] = robot[1]->hand->Pos(t);
		ptarget  = target[0]->Pos(t);
		ofs << t << ", "
			<< -phand[0].z << ", " << phand[0].y << ", "
			<< -phand[1].z << ", " << phand[1].y << ", "
			<< -ptarget .z << ", " << ptarget .y << ", "
			<< (phand[0] - ptarget).norm() << ", "
			<< (phand[1] - ptarget).norm() << endl;
	}

	ofs.close();

}
