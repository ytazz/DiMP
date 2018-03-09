#include <module/rendering.h>
#include <module/request.h>
#include "mymodule.h"
#include "RobotArm.h"

#include <DiMP/Load/Loader.h>

#include <sbcsv.h>

///////////////////////////////////////////////////////////////////////////////////////////////////

MyModule::Config::Welding::Welding() {
	pointsFilename = "";
	weldingStartTime = 0.0;
	weldingEndTime   = 0.0;
	lowerStartTime   = 0.0;
	lowerEndTime     = 0.0;
	lowerStartIndex  = 0;
	lowerEndIndex    = 0;
	lowerDiv         = 0;
	upperStartTime   = 0.0;
	upperEndTime     = 0.0;
	upperStartIndex  = 0;
	upperEndIndex    = 0;
	upperDiv         = 0;
	mockupOffset = vec3_t();
	useTree = false;
}

///////////////////////////////////////////////////////////////////////////////////////////////////

MyModule::MyModule() {
	sceneSelect = Reaching2D;

	reqManager->Add("enable" )->AddArg("mode", ArgType::String);
    reqManager->Add("disable")->AddArg("mode", ArgType::String); 
}

MyModule::~MyModule() {

}

void MyModule::Read(XML& xml) {
	if (sceneSelect == Welding) {
		xml.Get(conf.welding.pointsFilename  , ".points_filename"   );
		xml.Get(conf.welding.weldingStartTime, ".welding_start_time");
		xml.Get(conf.welding.weldingEndTime  , ".welding_end_time"  );
		xml.Get(conf.welding.lowerStartTime  , ".lower_start_time"  );
		xml.Get(conf.welding.lowerEndTime    , ".lower_end_time"    );
		xml.Get(conf.welding.lowerStartIndex , ".lower_start_index" );
		xml.Get(conf.welding.lowerEndIndex   , ".lower_end_index"   );
		xml.Get(conf.welding.lowerDiv        , ".lower_div"         );
		xml.Get(conf.welding.upperStartTime  , ".upper_start_time"  );
		xml.Get(conf.welding.upperEndTime    , ".upper_end_time"    );
		xml.Get(conf.welding.upperStartIndex , ".upper_start_index" );
		xml.Get(conf.welding.upperEndIndex   , ".upper_end_index"   );
		xml.Get(conf.welding.upperDiv        , ".upper_div"         );
		xml.Get(conf.welding.mockupOffset    , ".mockup_offset"     );
		xml.Get(conf.welding.useTree         , ".use_tree"          );
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

	// ���{�b�g�𒊏o
	for (int i = 0; ; i++) {
		UTRef<RobotArm>	r = new RobotArm();
		if (r->Build(i, &adaptorDiMP))
			robot.push_back(r);
		else break;
	}

	// �ڕW���̂𒊏o
	stringstream ss;
	for (int i = 0; ; i++) try {
		ss.str("");
		ss << "workspace/body_target" << i;
		DiMP::Object* obj = adaptorDiMP.GetObject(ss.str());
		target.push_back(obj);
	}
	catch (...) { break; }

	// ��Q���𒊏o
	for (int i = 0; ; i++) try {
		ss.str("");
		ss << "workspace/body_obstacle" << i;
		DiMP::Object* obj = adaptorDiMP.GetObject(ss.str());
		obstacle.push_back(obj);
	}
	catch (...) { break; }

	adaptorSprGR.ShowSolid(true);
	adaptorSprGR.ShowWireframe(false);

	// DiMP2�ˑ��ݒ荀�ڂ�Loader�ŏ���
	DiMP::Loader loader;
	xml.Load(sceneFilename);
	loader.Load(xml, graph);

	// Init�O�̏���
	if (sceneSelect == Welding) {
		// tree����
		if (conf.welding.useTree) {
			DiMP::Tree* tree = new DiMP::Tree(graph, "tree");
			tree->root = robot[0]->link[0];
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

		// tick����
		new DiMP::Tick(graph, conf.welding.weldingStartTime);
		for (int i = 0; i <= conf.welding.lowerDiv; i++) {
			real_t t = conf.welding.lowerStartTime + ((conf.welding.lowerEndTime - conf.welding.lowerStartTime) / (real_t)conf.welding.lowerDiv) * (real_t)i;
			new DiMP::Tick(graph, t);
		}
		for (int i = 0; i <= conf.welding.upperDiv; i++) {
			real_t t = conf.welding.upperStartTime + ((conf.welding.upperEndTime - conf.welding.upperStartTime) / (real_t)conf.welding.upperDiv) * (real_t)i;
			new DiMP::Tick(graph, t);
		}
		new DiMP::Tick(graph, conf.welding.weldingEndTime);
		
		// �n�ڃ^�X�N�p�}�b�`���O�^�X�N
        timeSlot .push_back(new DiMP::TimeSlot(graph, conf.welding.lowerStartTime, conf.welding.lowerEndTime, true, "ts_welding_lower"));
        matchTask.push_back(new DiMP::MatchTask(robot[0]->link.back(), target[0], timeSlot[0], "match_welding_lower"));

		timeSlot .push_back(new DiMP::TimeSlot(graph, conf.welding.upperStartTime, conf.welding.upperEndTime, true, "ts_welding_upper"));
        matchTask.push_back(new DiMP::MatchTask(robot[0]->link.back(), target[0], timeSlot[1], "match_welding_upper"));
       
        // ������^�X�N �i�v�Z���d���̂Ŗ��[�����N�̂݁j
        //avoidTask.push_back(new DiMP::AvoidTask(robot[0]->link[8], obstacle[0], timeSlot[0], "avoid_welding")); 
		
		// �}�b�`���O�^�X�N����(�Ie1�����b�N�A�b�v�~���O�Ɉړ�)
		// �}�b�`���O�^�X�N����(��w2�����b�N�A�b�v�~���O�Ɉړ�)
		//timeSlot .push_back(new DiMP::TimeSlot(graph, 0.1, 1.1, true, "ts_welding"));
		//matchTask.push_back(new DiMP::MatchTask(robot[0]->link[4]    , target[1], timeSlot[1], "match_initial_e1_welding" ));
		//matchTask.push_back(new DiMP::MatchTask(robot[0]->link[8]    , target[2], timeSlot[1], "match_initial_y1_welding" ));
		//matchTask.push_back(new DiMP::MatchTask(robot[0]->link.back(), target[3], timeSlot[1], "match_initial_end_welding"));
		
		// �}�b�`���O�^�X�N�����i�~���t�߂�e1���Œ�j
		// �}�b�`���O�^�X�N�����i�~���t�߂�w2���Œ�j
		//timeSlot .push_back(new DiMP::TimeSlot(graph, 1.11, 4.1, true, "ts_welding"));
		//matchTask.push_back(new DiMP::MatchTask(robot[0]->link[4], target[4], timeSlot[2], "match_cylinder_e1_welding"));
		//matchTask.push_back(new DiMP::MatchTask(robot[0]->link[8], target[5], timeSlot[2], "match_cylinder_y1_welding"));
		
		// �}�b�`���O�^�X�N�����i���J�[�u�ŕIe1���Œ�j
		// �}�b�`���O�^�X�N�����i���J�[�u�Ŏ��w2���Œ�j
		//timeSlot .push_back(new DiMP::TimeSlot(graph, 4.11, 7.1, true, "ts_welding"));
		//matchTask.push_back(new DiMP::MatchTask(robot[0]->link[4], target[6], timeSlot[3], "match_undercurve_e1_welding"));
		//matchTask.push_back(new DiMP::MatchTask(robot[0]->link[8], target[7], timeSlot[3], "match_undercurve_y1_welding"));
		
		// �}�b�`���O�^�X�N�����i��J�[�u�ŕIe1���Œ�j
		// �}�b�`���O�^�X�N�����i��J�[�u�Ŏ��w2���Œ�j
		//timeSlot .push_back(new DiMP::TimeSlot(graph, 7.11, 10.0, true, "ts_welding"));
		//matchTask.push_back(new DiMP::MatchTask(robot[0]->link[4], target[8], timeSlot[4], "match_overcurve_e1_welding"));
		//matchTask.push_back(new DiMP::MatchTask(robot[0]->link[8], target[9], timeSlot[4], "match_overcurve_y1_welding"));
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
		for (int k = 0; k < (int)graph->ticks.size(); k++) {
			DiMP::ObjectKey* key = (DiMP::ObjectKey*)target[0]->traj.GetKeypoint(graph->ticks[k]);
			real_t t = graph->ticks[k]->time;

			int idx = -1;
			if(conf.welding.lowerStartTime <= t && t <= conf.welding.lowerEndTime){
				idx = conf.welding.lowerStartIndex + ((t - conf.welding.lowerStartTime) / (conf.welding.lowerEndTime - conf.welding.lowerStartTime)) * (conf.welding.lowerEndIndex - conf.welding.lowerStartIndex);
				idx = std::min(std::max(conf.welding.lowerStartIndex, idx), conf.welding.lowerEndIndex-1);
			}
			if(conf.welding.upperStartTime <= t && t <= conf.welding.upperEndTime){
				idx = conf.welding.upperStartIndex + ((t - conf.welding.upperStartTime) / (conf.welding.upperEndTime - conf.welding.upperStartTime)) * (conf.welding.upperEndIndex - conf.welding.upperStartIndex);
				idx = std::min(std::max(conf.welding.upperStartIndex, idx), conf.welding.upperEndIndex-1);
			}

			if(idx != -1){
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

		// �����O����ݒ�
		for (int k = 0; k < (int)graph->ticks.size(); k++) {
			for (int j = 0; j < (int)robot[0]->joint.size(); j++) {
				// �֐�j�̎���k�̃L�[�|�C���g
				DiMP::JointKey* key = (DiMP::JointKey*)robot[0]->joint[j]->traj.GetKeypoint(graph->ticks[k]);

				// �֐ߊp��0.5[rad]�ɂ���ꍇ
				if (j == 1)
					key->pos[0]->val = Rad(0.0);
				else key->pos[0]->val = Rad(0.0);
			}
		}

		// �����ݒ�Ƃ��ĉ��z�^�[�Q�b�g�ւ̃}�b�`���O��L���Ƃ��C�n�ڗp�}�b�`���O�Ɗ�����𖳌��Ƃ���
        EnableConstraints("target" , true );
        EnableConstraints("welding", false);
        EnableConstraints("avoid"  , false); 
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
        if(mode == "target"){
            for(DiMP::MatchTask* task : matchTask){
                if( task->name == "match_cylinder_y1_welding"   ||
					task->name == "match_undercurve_y1_welding" ||
					task->name == "match_overcurve_y1_welding"  ){
                    task->param.match_tp = enable;
                    task->param.match_tv = false;
                    task->param.match_rp = enable;
                    task->param.match_rv = false;
                }
                if( task->name == "match_initial_y1_welding"    ||
					task->name == "match_initial_end_welding"   ||
					task->name == "match_initial_e1_welding"    ||
					task->name == "match_cylinder_e1_welding"   ||
					task->name == "match_undercurve_e1_welding" ||
					task->name == "match_overcurve_e1_welding"  ){
                    task->param.match_tp = false;
                    task->param.match_tv = false;
                    task->param.match_rp = false;
                    task->param.match_rv = false;
                }
            }
        }
        if(mode == "welding"){
            for(DiMP::MatchTask* task : matchTask){
                if( task->name == "match_welding_lower" ||
					task->name == "match_welding_upper" ){
                    task->param.match_tp = enable;
                    task->param.match_tv = enable;
                    task->param.match_rp = false;
                    task->param.match_rv = false;
                }
            }
        }
        if(mode == "avoid"){
            for(DiMP::AvoidTask* task : avoidTask){
                if(task->name == "avoid_welding"){
                    task->param.avoid_p = enable;
                    task->param.avoid_v = enable;
                }
            }
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

    return ret | Module::OnRequest(); 
}

void MyModule::OnStep() {
	Module::OnStep();
}

void MyModule::DrawSnapshot(real_t time) {
	//adaptorDiMP .SetSyncTime(time);
	graph->CreateSnapshot(time);
	//robot[0]->link[0]->ForwardKinematics(time);
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
		return false;
	}
	return false;
}
