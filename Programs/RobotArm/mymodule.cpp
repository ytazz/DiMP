#include <module/rendering.h>
#include <module/request.h>
#include "mymodule.h"
#include "Workspace.h"
#include "RobotArm.h"

#include <DiMP/Load/Loader.h>

#include <sbcsv.h>

///////////////////////////////////////////////////////////////////////////////////////////////////

MyModule::Config::Welding::Segment::Segment(){
	startIndex  = -1;
	endIndex    = -1;
}

MyModule::Config::Welding::Welding() {
	pointsFilename = "";
	mockupPos = vec3_t();
	mockupOri = quat_t();
	useTree   = false;
}

///////////////////////////////////////////////////////////////////////////////////////////////////

MyModule::MyModule() {
	sceneSelect = Reaching2D;

	reqManager->Add("enable" )->AddArg("mode", ArgType::String);
    reqManager->Add("disable")->AddArg("mode", ArgType::String);
	reqManager->Add("report");
}

MyModule::~MyModule() {

}

void MyModule::Read(XML& xml) {
	if (sceneSelect == Welding) {
		xml.Get(conf.welding.pointsFilename, ".points_filename"   );
		xml.Get(conf.welding.mockupPos, ".mockup_pos");
		xml.Get(conf.welding.mockupOri, ".mockup_ori");
		xml.Get(conf.welding.useTree  , ".use_tree"  );

		for(int i = 0; ; i++) try{
			XMLNode* node = xml.GetRootNode()->GetNode("segment", i);
			Config::Welding::Segment seg;
			node->Get(seg.startIndex            , ".start_index"  );
			node->Get(seg.endIndex              , ".end_index"    );
			node->Get(seg.timeslotName          , ".timeslot"     );
			node->Get(seg.startPosture          , ".start_posture");
			node->Get(seg.endPosture            , ".end_posture"  );
			
			conf.welding.segments.push_back(seg);
		}
		catch(Exception&){ break; }
	}
}

bool MyModule::Build() {
	XML xml;

	// シーン選択
	xml.Load("conf/robotarm/setting.xml");
    XMLNode* rootNode = xml.GetRootNode();
    rootNode->Get(sceneName, ".name");
	if (sceneName == "reaching2d"         ) sceneSelect = Reaching2D;
	if (sceneName == "reaching3d"         ) sceneSelect = Reaching3D;
	if (sceneName == "reaching3d_twostage") sceneSelect = Reaching3DTwoStage;
	if (sceneName == "toss3d"             ) sceneSelect = Toss3D;
	if (sceneName == "welding"            ) sceneSelect = Welding;

	// コンフィグ
    rootNode->Get(confFilename, ".conf_filename");
	if (!confFilename.empty()) {
		XML confXml;
		confXml.Load(confFilename);
		Read(confXml);
	}

    // 
    for(int i = 0; ; i++)try{
        XMLNode* wsNode = rootNode->GetNode("workspace", i);
        UTRef<Workspace> ws = new Workspace();
        wsNode->Get(ws->sceneFilename, ".filename");
        
        if(ws->Build()){
            workspace.push_back(ws);
        }
    }
    catch(Exception&){ break; }

    graph = workspace[0]->graph;

    /*
	// シーン別設定ファイル
	switch (sceneSelect) {
	case Reaching2D: sceneFilename = "conf/robotarm/scene_reaching2d.xml"; break;
	case Reaching3D: sceneFilename = "conf/robotarm/scene_reaching3d.xml"; break;
	case Toss3D: sceneFilename     = "conf/robotarm/scene_toss3d.xml"    ; break;
	case Welding: sceneFilename    = "conf/robotarm/scene_welding.xml"   ; break;
	}
    */
    /*
	switch (sceneSelect) {
	case Reaching2D: break;
	case Reaching3D: break;
	case Toss3D    : break;
	case Welding   : confFilename = "conf/robotarm/welding.xml"; break;
	}
    */

	// Init前の処理
	if (sceneSelect == Welding) {
		// tree生成
		if (conf.welding.useTree) {
			DiMP::Tree* tree = new DiMP::Tree(graph, "tree");
			tree->root = workspace[0]->robot[0]->link[0];
		}

		for(Config::Welding::Segment& seg : conf.welding.segments){
			seg.timeslot = graph->timeslots.Find(seg.timeslotName);
		}

		//// 低優先度で関節速度を0にする
		for (DiMP::Joint* jnt : workspace[0]->robot[0]->joint) {
			jnt->param.rmin_v[0] = 0.0;
			jnt->param.rmax_v[0] = 0.0;
		}

		// 溶接点列を読み込み
		CsvReader csv;
		csv.Read(conf.welding.pointsFilename, ",");
		int nrow = csv.NumRow();
		weldingPoints.resize(nrow);
		for (int i = 0; i < nrow; i++) {
			// mm -> m
			vec3_t p = 0.001 * vec3_t(csv.Get<real_t>(i, 0), csv.Get<real_t>(i, 1), csv.Get<real_t>(i, 2));
			// 座標変換
			weldingPoints[i] = conf.welding.mockupPos + conf.welding.mockupOri * p;
		}
	}
	if(sceneSelect == Reaching2D || sceneSelect == Reaching3D){
		DiMP::Tree* tree = new DiMP::Tree(graph, "tree");
		tree->root = workspace[0]->robot[0]->link[0];

		for (DiMP::Joint* jnt : workspace[0]->robot[0]->joint) {
			jnt->param.rmin_v[0] = 0.0;
			jnt->param.rmax_v[0] = 0.0;
		}
	}

	// 初期化
    for(Workspace* w : workspace)
	    w->graph->Init();

	// Init後の処理
	if (sceneSelect == Reaching2D) {
	}
	if (sceneSelect == Reaching3D) {
	}
	if (sceneSelect == Toss3D) {
		// 低優先度で関節速度を0にする
		for (uint i = 0; i < workspace[0]->robot[0]->joint.size(); i++) {
			workspace[0]->robot[0]->joint[i]->param.rmin_v[0] = 0.0;
			workspace[0]->robot[0]->joint[i]->param.rmax_v[0] = 0.0;
		}
		for (uint i = 0; i < workspace[0]->robot[1]->joint.size(); i++) {
			workspace[0]->robot[1]->joint[i]->param.rmin_v[0] = 0.0;
			workspace[0]->robot[1]->joint[i]->param.rmax_v[0] = 0.0;
		}
	}
	if (sceneSelect == Welding) {
		// 目標点列の軌跡を溶接点列に合わせて固定
		int npoints = (int)weldingPoints.size();
		for (DiMP::Tick* tick : graph->ticks) {
			DiMP::ObjectKey* key = (DiMP::ObjectKey*)workspace[0]->target[0]->traj.GetKeypoint(tick);
			real_t t = tick->time;

			for(Config::Welding::Segment& seg : conf.welding.segments){
				int    is = seg.startIndex;
				int    ie = seg.endIndex;
				real_t ts = seg.timeslot->param.ts_ini;
				real_t te = seg.timeslot->param.te_ini;
				if( is != -1 && ie != -1 && ts <= t && t <= te ){
					int idx = std::min(std::max(is, (int)( is + ((t - ts) / (te - ts)) * (ie - is)) ), ie-1);
					
					key->pos_t->val = weldingPoints[idx];
					key->pos_r->val = quat_t();
					key->vel_t->val = vec3_t();
					key->vel_r->val = vec3_t();

					key->pos_t->locked = true;
					key->pos_r->locked = true;
					key->vel_t->locked = true;
					key->vel_r->locked = true;
				}
			}
		}

		// 初期軌道を設定
		for(DiMP::Tick* tick : workspace[0]->graph->ticks){
			real_t t = tick->time;

			for(int i = 0; i < (int)workspace[0]->robot[0]->joint.size(); i++){
				DiMP::Joint*    jnt = workspace[0]->robot[0]->joint[i];
				DiMP::JointKey* key = (DiMP::JointKey*)jnt->traj.GetKeypoint(tick);
				if(jnt->dof == 0)
					continue;

				for(Config::Welding::Segment& seg : conf.welding.segments){
					real_t ts = seg.timeslot->param.ts_ini;
					real_t te = seg.timeslot->param.te_ini;
					if(ts <= t && t <= te){
						key->pos[0]->val = seg.startPosture[i] + (seg.endPosture[i] - seg.startPosture[i])*(t - ts)/(te - ts);
						key->vel[0]->val = (seg.endPosture[i] - seg.startPosture[i])/(te - ts);
					}
				}
			}
		}

		// 初期設定として仮想ターゲットへのマッチングを有効とし，溶接用マッチングと干渉回避を無効とする
        EnableConstraints("match", true );
        EnableConstraints("avoid", true ); 
	}

    for(Workspace* w : workspace){
        for(RobotArm* r : w->robot){
		    // 順運動学計算でリンク位置を決定
		    r->link[0]->ForwardKinematics();
        }
	}

	return true;
}

void MyModule::EnableConstraints(string mode, bool enable){
    if(sceneSelect == Welding){
        if(mode == "match"){
			for(DiMP::MatchTask* task : workspace[0]->matchTask){
				task->param.match_tp = enable;
			}
        }
        if(mode == "avoid"){
			for(DiMP::AvoidTask* task : workspace[0]->avoidTask){
				task->param.avoid_p = enable;
			}
        }
    }
}

void MyModule::Report(){
	for(DiMP::MatchTask* task : workspace[0]->matchTask){
		for(DiMP::Tick* tick : workspace[0]->graph->ticks){
			DiMP::MatchTaskKey* key = (DiMP::MatchTaskKey*)task->traj.GetKeypoint(tick);
			DSTR << tick->idx;
			if(key->con_tp[0]) DSTR << " " << key->con_tp[0]->y.norm();
			if(key->con_tp[1]) DSTR << " " << key->con_tp[1]->y.norm();
			if(key->con_tp[2]) DSTR << " " << key->con_tp[2]->y.norm();
			DSTR << endl;
		}
	}
}

bool MyModule::OnRequest() {
	string name           = reqManager->name;
    vector<ArgData>& args = reqManager->args;

    bool ret = false;

    if(name == "enable"){
        EnableConstraints(args[0].str, true);
        ret = true;
    }
    if(name == "disable"){
        EnableConstraints(args[0].str, false);
        ret = true;
    }
	if(name == "report"){
		Report();
	}

    return ret | Module::OnRequest(); 
}

void MyModule::OnStep() {
	Module::OnStep();
}

void MyModule::DrawSnapshot(real_t time) {
	//adaptorDiMP .SetSyncTime(time);
    for(Workspace* w : workspace){
        w->DrawSnapshot(renManager->render, time);
    }
}

void MyModule::OnDraw(DiMP::Render::Canvas* canvas) {
	// 光源設定
	GRLightDesc ld;
	ld.diffuse  = Vec4f(0.2f, 0.2f, 0.2f, 1.0f);
	ld.specular = Vec4f(0.0f, 0.0f, 0.0f, 1.0f);
	ld.ambient  = Vec4f(0.01f, 0.01f, 0.01f, 1.0f);
	ld.position = Vec4f( 10.0f,  10.0f, 10.0f, 1.0f);
	renManager->render->PushLight(ld);
	ld.position = Vec4f( 10.0f, -10.0f, 10.0f, 1.0f);
	renManager->render->PushLight(ld);
	ld.position = Vec4f(-10.0f,  10.0f, 10.0f, 1.0f);
	renManager->render->PushLight(ld);
	ld.position = Vec4f(-10.0f, -10.0f, 10.0f, 1.0f);
	renManager->render->PushLight(ld);
	
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	glEnable(GL_BLEND);
	
	renManager->render->SetLighting (true);
	renManager->render->SetDepthTest(true);
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
	renManager->render->PopLight();
	renManager->render->PopLight();
	renManager->render->PopLight();

	renManager->render->SetDepthTest(true );
	renManager->render->SetLighting (false);

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

bool MyModule::Set(DiMP::Render::Canvas* canvas, int attr, DiMP::Node* node){
	if(attr == DiMP::Render::Item::ObjectPos){
		canvas->SetPointColor("black");
		canvas->SetPointSize(2.0f);
		return true;
	}
	if(attr == DiMP::Render::Item::ObjectTrajectory){
		canvas->SetLineColor("black");
		canvas->SetLineWidth(1);
		return true;
	}
	if(attr == DiMP::Render::Item::Avoid){
		canvas->SetLineColor("red");
		canvas->SetPointSize(3);
		canvas->SetLineWidth(1);
		return true;
	}
	if(attr == DiMP::Render::Item::Geometry){
		canvas->SetLineColor("blue", 0, 0.1f);
		canvas->SetLineWidth(1);
		return false;
	}
	return false;
}
