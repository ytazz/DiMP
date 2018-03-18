#include <module/rendering.h>
#include <module/request.h>
#include "mymodule.h"
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
	mockupOffset   = vec3_t();
	useTree        = false;
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
		xml.Get(conf.welding.mockupOffset  , ".mockup_offset"     );
		xml.Get(conf.welding.useTree       , ".use_tree"          );

		for(int i = 0; ; i++) try{
			XMLNode* node = xml.GetRootNode()->GetNode("segment", i);
			Config::Welding::Segment seg;
			node->Get(seg.startIndex            , ".start_index"  );
			node->Get(seg.endIndex              , ".end_index"    );
			node->Get(seg.timeslotName          , ".timeslot"     );
			node->Get(seg.posture               , ".posture"      );
			
			conf.welding.segments.push_back(seg);
		}
		catch(Exception&){ break; }
	}
}

bool MyModule::Build() {
	XML xml;

	// �V�[���I��
	xml.Load("conf/robotarm/setting.xml");
	string str;
	xml.Get(str, ".scene");
	if (str == "reaching2d") sceneSelect = Reaching2D;
	if (str == "reaching3d") sceneSelect = Reaching3D;
	if (str == "toss3d"    ) sceneSelect = Toss3D;
	if (str == "welding"   ) sceneSelect = Welding;

	// �V�[���ʐݒ�t�@�C��
	switch (sceneSelect) {
	case Reaching2D: sceneFilename = "conf/robotarm/scene_reaching2d.xml"; break;
	case Reaching3D: sceneFilename = "conf/robotarm/scene_reaching3d.xml"; break;
	case Toss3D: sceneFilename     = "conf/robotarm/scene_toss3d.xml"    ; break;
	case Welding: sceneFilename    = "conf/robotarm/scene_welding.xml"   ; break;
	}
	switch (sceneSelect) {
	case Reaching2D: break;
	case Reaching3D: break;
	case Toss3D    : break;
	case Welding   : confFilename = "conf/robotarm/welding.xml"; break;
	}

	// �R���t�B�O
	if (!confFilename.empty()) {
		XML confXml;
		confXml.Load(confFilename);
		Read(confXml);
	}

	// Scenebuiler::Builder�Ńp�[�X
	typedb.Register();
	scene.Create();
	scene.CreateObject(NamespaceProp::id, "world", &typedb);
	models.Set(&scene);
	builder.Set(&scene, &typedb, &models);
	builder.Parse(sceneFilename);

	// �e�A�_�v�^�Ń~���[�����O
	adaptorDiMP.Set(graph);

	Adaptor* adaptors[] = { &adaptorDiMP, &adaptorSprGR };
	for each(Adaptor* a in adaptors) {
		a->Set(&scene, &typedb, &models);
		a->Mirror();
		a->SyncProperty(true);
	}

	// DiMP�ˑ��ݒ荀�ڂ�Loader�ŏ���
	DiMP::Loader loader;
	xml.Load(sceneFilename);
	loader.Load(xml, graph);

	// ���{�b�g�𒊏o
	for (int i = 0; ; i++) {
		UTRef<RobotArm>	r = new RobotArm();
		if (r->Build(i, graph))
			robot.push_back(r);
		else break;
	}

	// �ڕW���̂𒊏o
	stringstream ss;
	for (int i = 0; ; i++){
		ss.str("");
		ss << "workspace/body_target" << i;
		DiMP::Object* obj = graph->objects.Find(ss.str());
		if(!obj)
			break;
		target.push_back(obj);
	}

	// ��Q���𒊏o
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

	adaptorSprGR.ShowSolid(true);
	adaptorSprGR.ShowWireframe(false);

	// Init�O�̏���
	if (sceneSelect == Welding) {
		// tree����
		if (conf.welding.useTree) {
			DiMP::Tree* tree = new DiMP::Tree(graph, "tree");
			tree->root = robot[0]->link[0];
		}

		for(Config::Welding::Segment& seg : conf.welding.segments){
			seg.timeslot = graph->timeslots.Find(seg.timeslotName);
		}

		//// ��D��x�Ŋ֐ߑ��x��0�ɂ���
		for (DiMP::Joint* jnt : robot[0]->joint) {
			jnt->param.rmin_dp[0] = 0.0;
			jnt->param.rmax_dp[0] = 0.0;
		}

		// �n�ړ_���ǂݍ���
		CsvReader csv;
		csv.Read(conf.welding.pointsFilename, ",");
		int nrow = csv.NumRow();
		weldingPoints.resize(nrow);
		for (int i = 0; i < nrow; i++) {
			// mm -> m
			// �I�t�Z�b�g���Z
			weldingPoints[i] = conf.welding.mockupOffset + 0.001 * vec3_t(csv.Get<real_t>(i, 0), csv.Get<real_t>(i, 1), csv.Get<real_t>(i, 2));
		}
	}

	// ������
	graph->Init();

	// Init��̏���
	if (sceneSelect == Reaching2D) {
	}
	if (sceneSelect == Reaching3D) {
		// ��D��x�Ŋ֐ߑ��x��0�ɂ���
		for (uint i = 0; i < robot[0]->joint.size(); i++) {
			robot[0]->joint[i]->param.rmin_v[0] = 0.0;
			robot[0]->joint[i]->param.rmax_v[0] = 0.0;
		}
	}
	if (sceneSelect == Toss3D) {
		// ��D��x�Ŋ֐ߑ��x��0�ɂ���
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
		// �ڕW�_��̋O�Ղ�n�ړ_��ɍ��킹�ČŒ�
		int npoints = (int)weldingPoints.size();
		for (DiMP::Tick* tick : graph->ticks) {
			DiMP::ObjectKey* key = (DiMP::ObjectKey*)target[0]->traj.GetKeypoint(tick);
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

		// �����O����ݒ�
		for(DiMP::Tick* tick : graph->ticks){
			real_t t = tick->time;

			for(Config::Welding::Segment& seg : conf.welding.segments){
				if(seg.timeslot->param.ts_ini <= t && t <= seg.timeslot->param.te_ini){				
					for(int i = 0; i < (int)robot[0]->joint.size(); i++){
						DiMP::Joint* jnt = robot[0]->joint[i];
						// �֐�j�̎���k�̃L�[�|�C���g
						DiMP::JointKey* key = (DiMP::JointKey*)jnt->traj.GetKeypoint(tick);
						if(jnt->dof > 0)
							key->pos[0]->val = seg.posture[i];
					}
				}
			}
		}

		// �����ݒ�Ƃ��ĉ��z�^�[�Q�b�g�ւ̃}�b�`���O��L���Ƃ��C�n�ڗp�}�b�`���O�Ɗ�����𖳌��Ƃ���
        EnableConstraints("match", true );
        EnableConstraints("avoid", true ); 
	}

	for (uint i = 0; i < robot.size(); i++) {
		RobotArm* r = robot[i];
		// ���^���w�v�Z�Ń����N�ʒu������
		r->link[0]->ForwardKinematics();
	}

	return true;
}

void MyModule::EnableConstraints(string mode, bool enable){
    if(sceneSelect == Welding){
        if(mode == "match"){
			for(DiMP::MatchTask* task : matchTask){
				task->param.match_tp = enable;
			}
        }
        if(mode == "avoid"){
			for(DiMP::AvoidTask* task : avoidTask){
				task->param.avoid_p = enable;
			}
        }
    }
}

void MyModule::Report(){
	for(DiMP::MatchTask* task : matchTask){
		for(DiMP::Tick* tick : graph->ticks){
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
	graph->CreateSnapshot(time);
	robot[0]->link[0]->ForwardKinematics(time);
	adaptorDiMP.SyncProperty(false, AttrCategory::State);
	adaptorSprGR.SyncProperty(true, AttrCategory::State);

	adaptorSprGR.Draw(renManager->render);
}

void MyModule::OnDraw(DiMP::Render::Canvas* canvas) {
	//// �Đ����[�h�̏ꍇ�͍Đ������̃X�i�b�v�V���b�g��`��
	//if(GetAction(MENU_ALWAYS, ID_PLAY)->GetBool()){
	//	DrawSnapshot(render, playTime);
	//}
	//// ����ȊO�͏��������Ɗe�^�X�N�̊J�n�E�I�������̃X�i�b�v�V���b�g��`��
	//else{
	//	DrawSnapshot(render, 0.0);
	//	for(uint i = 0; i < workspace.task.size(); i++){
	//		DiMP::Task* task = workspace.task[i];
	//		DrawSnapshot(render, task->time->time_s->val);
	//		DrawSnapshot(render, task->time->time_e->val);
	//	}
	//}

	// �����ݒ�
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
		// �n�ړ_��
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
		canvas->SetLineWidth(1);
		return true;
	}
	return false;
}
