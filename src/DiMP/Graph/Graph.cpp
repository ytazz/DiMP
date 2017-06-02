#include <DiMP/Graph/Graph.h>
#include <DiMP/Graph/Object.h>
#include <DiMP/Solver/Range.h>
#include <DiMP/Solver/Solver.h>
#include <DiMP/Render/Config.h>
#include <DiMP/Render/Canvas.h>

#include <Foundation/UTPreciseTimer.h>

#include <omp.h>

namespace DiMP{;

const char* VarNames[] = {
	"object_pos_t"   ,
	"object_pos_r"   ,
	"object_vel_t"   ,
	"object_vel_r"   ,
	"joint_pos"      ,
	"joint_vel"      ,
	"joint_torque"   ,
	"joint_force_t"  ,
	"joint_force_r"  ,
	"time_start"     ,
	"time_end"       ,
	"biped_com_pos_t",
	"biped_com_pos_r",
	"biped_com_vel_t",
	"biped_com_vel_r",
	"biped_cop_pos_t",
	"biped_cop_pos_r",
	"biped_period"   ,
};

const char* ConNames[] = {
	"object_c1t"         ,
	"object_c1r"         ,
	"joint_c1"           ,
	"joint_c2"           ,
	"joint_f"            ,
	"joint_range_p"      ,
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
	"eom"                ,
	"biped_com_tp"       ,
	"biped_com_tv"       ,
	"biped_com_r"        ,
	"biped_cop_t"        ,
	"biped_cop_r"        ,
	"biped_period"       ,
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
	solver->graph = this;

	conf = new Render::Config();
}

void Graph::SetScaling(real_t T, real_t L, real_t M){
	scale.Set(T, L, M);
	ready = false;
}

int Graph::SetPriority(ID mask, uint level){
	int match = 0;
	for(uint i = 0; i < solver->cons.size(); i++){
		Constraint* con = solver->cons[i];
		if(mask.Match(con)){
			con->SetPriority(level);
			match++;
		}
	}
	if(match)
		solver->ready = false;
	return match;
}

int Graph::Enable(ID mask, bool enable){
	int match = 0;
	for(uint i = 0; i < solver->cons.size(); i++){
		Constraint* con = solver->cons[i];
		if(mask.Match(con)){
			con->enabled = enable;
			match++;
		}
	}
	return match;
}

int Graph::Lock(ID mask, bool lock){
	int match = 0;
	for(uint i = 0; i < solver->vars.size(); i++){
		Variable* var = solver->vars[i];
		if(mask.Match(var)){
			var->Lock(lock);
			match++;
		}
	}
	return match;
}

int Graph::SetCorrectionRate(ID mask, real_t rate, real_t lim){
	int match = 0;
	for(uint i = 0; i < solver->cons.size(); i++){
		Constraint* con = solver->cons[i];
		if(mask.Match(con)){
			con->corrRate = rate;
			con->corrMax  = lim;
			match++;
		}
	}
	return match;
}

real_t Graph::CalcError(ID mask, bool sum_or_max){
	real_t E = 0.0;
	for(uint i = 0; i < solver->cons.size(); i++){
		Constraint* con = solver->cons[i];
		if(mask.Match(con) && con->enabled && con->active){
			for(uint k = 0; k < (uint)con->nelem; k++){
				if(sum_or_max)
					 E += con->e[k];
				else E = std::max(E, con->e[k]);
			}
		}
	}
	return E;
}

real_t Graph::CalcVariable(ID mask, bool ave_or_max){
	real_t v = 0.0;
	uint match = 0;
	for(uint i = 0; i < solver->vars.size(); i++){
		Variable* var = solver->vars[i];
		if(mask.Match(var)){
			if(ave_or_max)
				v += var->Norm();
			else v = std::max(v, var->Norm());
			match++;
		}
	}
	if(ave_or_max && match != 0)
		v /= (real_t)match;
	return v;
}

void Graph::Init(){
	//omp_set_num_threads(1);
	sort(nodes    .begin(), nodes    .end(), Node::CompareByType());
	sort(trajNodes.begin(), trajNodes.end(), Node::CompareByType());
	
	//solver.numthread = param.numthread;
	solver->Clear();

	// create keypoints of trajectory nodes
	trajNodes.AddKeypoints();

	// �c���[�\�����o
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

void Graph::Step(){
	if(!ready)
		Init();

	static Spr::UTPreciseTimer timer;
	timer.CountUS();
	Prepare();
	uint TPrepare = timer.CountUS();
	//cout << "Prepare: " << TPrepare << endl;
	
	timer.CountUS();
	solver->Step();
	uint TStep = timer.CountUS();
	//cout << "Step: " << TStep << endl;
	
	// �c���[�ɂ��ď��L�l�v�Z
	trees.ForwardKinematics();
	
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
	
	for(int i = 0; i < (int)ticks.size(); i++)
		DrawSnapshot(ticks[i]->time, canvas, conf);
}

void Graph::DrawSnapshot(real_t time, Render::Canvas* canvas, Render::Config* _conf){
	if(!_conf)
		_conf = conf;

	trajNodes.DrawSnapshot(time, canvas, _conf);
}

}