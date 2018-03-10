#include <DiMP/Graph/Graph.h>
#include <DiMP/Graph/Object.h>
#include <DiMP/Graph/Joint.h>
#include <DiMP/Render/Config.h>
#include <DiMP/Render/Canvas.h>

#include <Foundation/UTPreciseTimer.h>

#include <omp.h>

namespace DiMP{;

const char* VarNames[] = {
	"object_pos_t"     ,
	"object_pos_r"     ,
	"object_vel_t"     ,
	"object_vel_r"     ,
	"joint_pos"        ,
	"joint_vel"        ,
	"joint_torque"     ,
	"joint_force_t"    ,
	"joint_force_r"    ,
	"time_start"       ,
	"time_end"         ,
	"biped_torso_pos_t",
	"biped_torso_pos_r",
	"biped_torso_vel_t",
	"biped_torso_vel_r",
	"biped_foot_t"     ,
	"biped_foot_r"     ,
	"biped_com_pos"    ,
	"biped_com_vel"    ,
	"biped_cop"        ,
	"biped_duration"   ,
	"biped_time"       
};

const char* ConNames[] = {
	"object_c1t"         ,
	"object_c1r"         ,
	"joint_c1"           ,
	"joint_c2"           ,
	"joint_f"            ,
	"joint_range_p"      ,
	"joint_range_dp"     ,
	"joint_range_v"      ,
	"joint_range_f"      ,
	"joint_tp"           ,
	"joint_rp"           ,
	"joint_tv"           ,
	"joint_rv"           ,
	"force_t"            ,
	"force_r"            ,
	"contact_p"          ,
	"contact_v"          ,
	"contact_fn"         ,
	"contact_ft"         ,
	"time_start_range"   ,
	"time_end_range"     ,
	"time_duration_range",
	"match_tp"           ,
	"match_tv"           ,
	"match_rp"           ,
	"match_rv"           ,
	"avoid_p"            ,
	"avoid_v"            ,
	"biped_lip_p"        ,
	"biped_lip_v"        ,
	"biped_foot_range_t" ,
	"biped_foot_range_r" ,
	"biped_foot_match_t" ,
	"biped_foot_match_r" ,
	"biped_com_p"        ,
	"biped_com_v"        ,
	"biped_cop"          ,
	"biped_duration"     ,
	"biped_time"         
};

Graph::Scale::Scale(){
	Set(1.0, 1.0, 1.0);
}

void Graph::Scale::Set(real_t T, real_t L, real_t M){
	time = T;			time_inv = 1.0 / time;
	size = L;			size_inv = 1.0 / size;
	mass = M;			mass_inv = 1.0 / mass;
	inertia = M*L*L;	inertia_inv = 1.0 / inertia;
	
	pos_t = L;			pos_t_inv = 1.0 / pos_t;
	vel_t = L/T;		vel_t_inv = 1.0 / vel_t;
	acc_t = L/(T*T);	acc_t_inv = 1.0 / acc_t;

	real_t s = 1.0;
	pos_r = s;			pos_r_inv = 1.0 / pos_r;
	vel_r = s/T;		vel_r_inv = 1.0 / vel_r;
	acc_r = s/(T*T);	acc_r_inv = 1.0 / acc_r;
	
	force_t = (M*L)/(T*T);		force_t_inv = 1.0 / force_t;
	force_r = (M*L*L)/(T*T);	force_r_inv = 1.0 / force_r;
}

//////////////////////////////////////////////////////////////////////////////////////

Graph::Graph(){
	solver = new Solver();
	//solver->graph = this;

	conf = new Render::Config();
}

void Graph::SetScaling(real_t T, real_t L, real_t M){
	scale.Set(T, L, M);
	ready = false;
}

void Graph::Init(){
	//omp_set_num_threads(1);
	sort(nodes    .begin(), nodes    .end(), Node::CompareByType());
	sort(trajNodes.begin(), trajNodes.end(), Node::CompareByType());
	
	//solver.numthread = param.numthread;
	solver->Clear();

	// create keypoints of trajectory nodes
	trajNodes.AddKeypoints();

	// ツリー構造抽出
	trees.Extract();
	
	// register variables to solver
	nodes.AddVar();

	// register constraints to solver
	nodes.AddCon();

	// do extra initialization
	nodes.Init();

	solver->Init();

	ready = true;
}

void Graph::Clear(){
	nodes    .clear();
	trajNodes.clear();
	ticks    .clear();
	objects  .clear();
	bipeds   .clear();
	trees    .clear();
	joints   .clear();
	geos     .clear();
	timeslots.clear();
	tasks    .clear();
	solver->Clear();
	ready = false;
}

void Graph::Reset(){
	solver->Reset();
}

void Graph::Prepare(){
	nodes.Prepare();
}

void Graph::Finish(){
	nodes.Finish();

	// ツリーについて順キネ計算
	trees.ForwardKinematics();
}

void Graph::Step(){
	if(!ready)
		Init();

	static Spr::UTPreciseTimer timer;
	timer.CountUS();
	Prepare();
	uint TPrepare = timer.CountUS();
	//DSTR << "Prepare: " << TPrepare << endl;
	
	timer.CountUS();
	solver->Step();
	uint TStep = timer.CountUS();
	//DSTR << "Step: " << TStep << endl;

	timer.CountUS();
	Finish();
	uint TFinish = timer.CountUS();
}

void Graph::Draw(Render::Canvas* canvas, Render::Config* _conf){
	if(!_conf)
		_conf = conf;

	if(conf->Set(canvas, Render::Item::GlobalAxis, 0)){
		float l = conf->Scale(Render::Item::GlobalAxis, 0);
		canvas->Line(Vec3f(), Vec3f(l   , 0.0f, 0.0f));
		canvas->Line(Vec3f(), Vec3f(0.0f, l   , 0.0f));
		canvas->Line(Vec3f(), Vec3f(0.0f, 0.0f, l   ));
	}

	nodes.Draw(canvas, _conf);
	
	for(Tick* tick : ticks)
		DrawSnapshot(tick->time, canvas, conf);
}

void Graph::CreateSnapshot(real_t t){
	trajNodes.CreateSnapshot(t);
}

void Graph::DrawSnapshot(real_t time, Render::Canvas* canvas, Render::Config* _conf){
	if(!_conf)
		_conf = conf;

	trajNodes.CreateSnapshot(time);
	trajNodes.DrawSnapshot(canvas, _conf);
}

}
