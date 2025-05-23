﻿#include <DiMP/Graph/Graph.h>
#include <DiMP/Graph/Object.h>
#include <DiMP/Graph/Joint.h>
#include <DiMP/Graph/Avoid.h>
#include <DiMP/Render/Config.h>
#include <DiMP/Render/Canvas.h>

#include <omp.h>

namespace DiMP{;

const char* VarNames[] = {
	"any"               ,
	"object_pos_t"      ,
	"object_pos_r"      ,
	"object_vel_t"      ,
	"object_vel_r"      ,
	"joint_pos"         ,
	"joint_vel"         ,
	"joint_acc"         ,
	"joint_torque"      ,
	"joint_force_t"     ,
	"joint_force_r"     ,
	"time_start"        ,
	"time_end"          ,
	"biped_torso_pos_t" ,
	"biped_torso_pos_r" ,
	"biped_torso_vel_t" ,
	"biped_torso_vel_r" ,
	"biped_com_pos"     ,
	"biped_com_vel"     ,
	"biped_duration"    ,
	"biped_time"        ,
	"biped_T"           ,
	"biped_foot_pos_t"  ,
	"biped_foot_pos_r"  ,
	"biped_foot_vel_t"  ,
	"biped_foot_vel_r"  ,
	"biped_foot_cop_pos",
	"biped_foot_cop_vel",
	"biped_capt_sup_t"   ,
	"biped_capt_sup_r"   ,
	"biped_capt_swg_t"   ,
	"biped_capt_swg_r"   ,
	"biped_capt_icp"     ,
	"biped_capt_land_t"  ,
	"biped_capt_land_r"  ,
	"biped_capt_cop"     ,
	"biped_capt_duration",	
	"centroid_pos_t"    ,
	"centroid_pos_r"    ,
	"centroid_vel_t"    ,
	"centroid_vel_r"    ,
	"centroid_momentum" ,
	"centroid_time"     ,
	"centroid_duration" ,
	"centroid_end_pos"  ,
	"centroid_end_vel"  ,
	"centroid_end_acc"  ,
	"centroid_end_stiff",
	"centroid_end_cmp",
	"centroid_end_force",
	"centroid_end_moment",
	"wholebody_pos_t",
	"wholebody_pos_r",
	"wholebody_vel_t",
	"wholebody_vel_r",
	"wholebody_acc_t",
	"wholebody_acc_r",
	"wholebody_momentum",
	"wholebody_joint_pos",
	"wholebody_joint_vel",
	"wholebody_joint_acc",
	"wholebody_joint_jerk",
	"wholebody_force_t",
	"wholebody_force_r"
};

const char* ConNames[] = {
	"any"                 ,
	"object_c1t"          ,
	"object_c1r"          ,
	"joint_c0p"           ,
	"joint_c0v"           ,
	"joint_c1p"           ,
	"joint_f"             ,
	"joint_range_p"       ,
	"joint_range_v"       ,
	"joint_range_f"       ,
	"joint_des_p"         ,
	"joint_des_v"         ,
	"joint_tp"            ,
	"joint_rp"            ,
	"joint_tv"            ,
	"joint_rv"            ,
	"force_t"             ,
	"force_r"             ,
	"contact_p"           ,
	"contact_v"           ,
	"contact_fn"          ,
	"contact_ft"          ,
	"time_start_range"    ,
	"time_end_range"      ,
	"time_duration_range" ,
	"match_tp"            ,
	"match_tv"            ,
	"match_rp"            ,
	"match_rv"            ,
	"avoid_p"             ,
	"avoid_v"             ,
	"biped_lip_pos"       ,
	"biped_lip_vel"       ,
	"biped_foot_pos_t"    ,
	"biped_foot_pos_r"    ,
	"biped_foot_cop"      ,
	"biped_foot_pos_range_t",
	"biped_foot_pos_range_r",
	"biped_foot_cop_range"  ,
	"biped_foot_vel_zero_t" ,
	"biped_foot_vel_zero_r" ,
	"biped_com_p"         ,
	"biped_com_v"         ,
	"biped_torso",
	"biped_duration_range",
	"biped_time"          ,
	"biped_capt_sup_t"       ,
	"biped_capt_sup_r"       ,
	"biped_capt_swg_t"       ,
	"biped_capt_swg_r"       ,
	"biped_capt_icp"         ,
	"biped_capt_cop"         ,
	"biped_capt_duration"    ,
	"biped_capt_land_range_t",
	"biped_capt_land_range_r",
	"biped_capt_cop_range"   ,
	"biped_capt_icp_range"   ,	
	"centroid_pos_t"           ,
	"centroid_pos_r"           ,
	"centroid_vel_t"           ,
	"centroid_vel_r"           ,
	"centroid_momentum"        ,
	"centroid_time"            ,
	"centroid_end_pos_t"       ,
	"centroid_end_pos_r"       ,
	"centroid_des_pos_t"       ,
	"centroid_des_pos_r"       ,
	"centroid_des_vel_t"       ,
	"centroid_des_vel_r"       ,
	"centroid_des_momentum"    ,
	"centroid_des_time"        ,
	"centroid_des_duration"    ,
	"centroid_des_end_pos_t"   ,
	"centroid_des_end_vel_t"   ,
	"centroid_des_end_pos_r"   ,
	"centroid_des_end_vel_r"   ,
	"centroid_des_end_stiff"   ,
	"centroid_des_end_cmp"     ,
	"centroid_des_end_force"   ,
	"centroid_des_end_moment"  ,
	"centroid_duration_range"  ,
	"centroid_end_pos_range"   ,
	"centroid_end_stiff_range" ,
	"centroid_end_contact"     ,
	"centroid_end_friction"    ,
	"centroid_end_moment_range",
	"wholebody_pos_t",
	"wholebody_pos_r",
	"wholebody_vel_t",
	"wholebody_vel_r",
	"wholebody_acc_t",
	"wholebody_acc_r",
	"wholebody_joint_pos",
	"wholebody_joint_vel",
	"wholebody_joint_acc",
	"wholebody_joint_jerk",
	"wholebody_force_t",
	"wholebody_force_r",
	"wholebody_limit",
	"wholebody_contact_pos_t",
	"wholebody_contact_pos_r",
	"wholebody_contact_vel_t",
	"wholebody_contact_vel_r",
	"wholebody_normal_force",
	"wholebody_friction_force",
	"wholebody_moment",
	"wholebody_momentum"
};

///////////////////////////////////////////////////////////////////////////////////////////////////

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

///////////////////////////////////////////////////////////////////////////////////////////////////

Graph::Param::Param(){
	gravity      = vec3_t( 0.0,  0.0,  0.0);
	supportMapResolution = 50;
	//bbmin        = vec3_t(-1.0, -1.0, -1.0);
	//bbmax        = vec3_t( 1.0,  1.0,  1.0);
	//octtreeDepth = 8;
}

//////////////////////////////////////////////////////////////////////////////////////

Graph::Graph(){
	solver = new CustomSolver();
	solver->graph = this;
	
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
	nodes      .clear();
	trajNodes  .clear();
	ticks      .clear();
	objects    .clear();
	cons       .clear();
	bipeds     .clear();
	bipedCapts .clear();
	runners    .clear();
	centroids  .clear();
	wholebodies.clear();
	trees      .clear();
	joints     .clear();
	geos       .clear();
	timeslots  .clear();
	tasks      .clear();
	solver->Clear();
	ready = false;
}

void Graph::Reset(){
	solver->Reset();
}

void Graph::Prepare(){
	// compute forward kinematics
	trees.ForwardKinematics();

	objects    .Prepare();
	cons       .Prepare();
	bipeds     .Prepare();
	bipedCapts .Prepare();
	runners    .Prepare();
	centroids  .Prepare();
	wholebodies.Prepare();
	trees      .Prepare();
	joints     .Prepare();
	geos       .Prepare();
	timeslots  .Prepare();
	tasks      .Prepare();

	//nodes.Prepare();
}

void Graph::PrepareStep(){
	objects    .PrepareStep();
	cons       .PrepareStep();
	bipeds     .PrepareStep();
	bipedCapts .PrepareStep();
	runners    .PrepareStep();
	centroids  .PrepareStep();
	wholebodies.PrepareStep();
	trees      .PrepareStep();
	joints     .PrepareStep();
	geos       .PrepareStep();
	timeslots  .PrepareStep();

  // call broad phase here
	ExtractGeometryPairs();

	tasks      .PrepareStep();
}

void Graph::ExtractGeometryPairs(){
	static Timer timer2;

	// create table
	int nobj = objects.size();
	if(avoidTable.empty()){
		// assign serial index to objects
		for(int i = 0; i < nobj; i++){
			objects[i]->objIndex = i;
		}

		int geoIndex[2] = {0, 0};

		avoidTable.Resize(nobj, nobj, (AvoidTask*)0);
		for(AvoidTask* avoid : avoids){
			// register avoid to table
			avoidTable(avoid->obj0->objIndex, avoid->obj1->objIndex) = avoid;

			// assign serial index to geoinfos
			for(Tick* tick : ticks){
				ObjectKey* key0 = (ObjectKey*)avoid->obj0->traj.GetKeypoint(tick);
				ObjectKey* key1 = (ObjectKey*)avoid->obj1->traj.GetKeypoint(tick);

				for(GeometryInfo& geo0 : key0->geoInfos){
					if(geo0.geoIndex[0] == -1)
						geo0.geoIndex[0] = geoIndex[0]++;
				}
				for(GeometryInfo& geo1 : key1->geoInfos){
					if(geo1.geoIndex[1] == -1)
						geo1.geoIndex[1] = geoIndex[1]++;
				}

			}
		}
		gpTable.Resize(geoIndex[0], geoIndex[1]);
	}

    // for each object, calculate maximum dmin of avoid tasks assigned to it
    for(Object* obj : objects){
		for(Tick* tick : ticks){
			ObjectKey* key = (ObjectKey*)obj->traj.GetKeypoint(tick);
			key->dminMax = 0.0;
        }
    }	
	for(AvoidTask* avoid : avoids){
		for(Tick* tick : ticks){
			ObjectKey* key0 = (ObjectKey*)avoid->obj0->traj.GetKeypoint(tick);
			ObjectKey* key1 = (ObjectKey*)avoid->obj1->traj.GetKeypoint(tick);
			
			key0->dminMax = std::max(key0->dminMax, avoid->param.dmin);
			key1->dminMax = std::max(key1->dminMax, avoid->param.dmin);
        }	    
	}

	timer2.CountUS();
	// for each dimension: merge edgeinfos of all objects and ticks, and sort it
	for(int dir = 0; dir < 3; dir++){
        // create edge list for the first time
        if(edgeInfosStationary[dir].empty()){
		    for(Object* obj : objects){
                // for stationary objects, insert edges for the first tick only
                if(obj->param.stationary){
					ObjectKey* key = (ObjectKey*)obj->traj.GetKeypoint(ticks[0]);
                    edgeInfosStationary[dir].insert(edgeInfosStationary[dir].end(), key->edgeInfos[dir].begin(), key->edgeInfos[dir].end());
					//for(int i = 0; i < key->edgeInfos[dir].size(); i++)
					//	printf("stationary edge: %d %d %f\n", dir, key->edgeInfos[dir][i].side, key->edgeInfos[dir][i].val);
                }
            }
            // sort it only once upon creation
		    sort(edgeInfosStationary[dir].begin(), edgeInfosStationary[dir].end());
        }
	}
	for(int dir = 0; dir < 3; dir++){
        // create edge list for the first time
		edgeInfosMoving[dir].clear();
		for(Object* obj : objects){
			// for moving objects, insert edges for all ticks
			if(!obj->param.stationary){
				for(Tick* tick : ticks){
					ObjectKey* key = (ObjectKey*)obj->traj.GetKeypoint(tick);
					edgeInfosMoving[dir].insert(edgeInfosMoving[dir].end(), key->edgeInfos[dir].begin(), key->edgeInfos[dir].end());
				}
			}
		}
        // sort it every time
		sort(edgeInfosMoving[dir].begin(), edgeInfosMoving[dir].end());
	}
    
    for(int dir = 0; dir < 3; dir++){
        edgeInfos[dir].resize(edgeInfosStationary[dir].size() + edgeInfosMoving[dir].size());
        std::merge(
            edgeInfosStationary[dir].begin(), edgeInfosStationary[dir].end(),
            edgeInfosMoving[dir].begin(), edgeInfosMoving[dir].end(),
            edgeInfos[dir].begin()
            );
    }

	// merge edge infos of moving and stationary objects into one
	//printf("edges of moving objects: %d\n", edgeInfosMoving[0].size());
	//printf("edges of stationary objects: %d\n", edgeInfosStationary[0].size());
	//printf("edges of all objects: %d\n", edgeInfos[0].size());

	int timeSort = timer2.CountUS();
	
	int szmax = 0;
	
	// sweep sorted edge info and mark candidate geopairs in the table
	timer2.CountUS();
	int numInt[3] = {0, 0, 0};
	// reset table
	for(GPTableEntry& gp : gpTable)
		gp.intersect = 0;

	for(int dir = 0; dir < 3; dir++){
		queue[dir].resize(nobj);
		for(int i = 0; i < nobj; i++)
			queue[dir][i].clear();

		for(vector<EdgeInfo>::iterator it = edgeInfos[dir].begin(); it != edgeInfos[dir].end(); it++){
			EdgeInfo& e = *it;
			//DSTR << e.val << " " << e.side << endl;

			GeometryInfo* geo0 = e.geoInfo;
			Object*       obj0 = geo0->con->obj;
			if(e.side == 0){	
				for(int i = 0; i < nobj; i++){
					Object* obj1 = objects[i];

					AvoidTask* avoid[2];
					avoid[0] = avoidTable(obj0->objIndex, obj1->objIndex);
					avoid[1] = avoidTable(obj1->objIndex, obj0->objIndex);
					if(!avoid[0] && !avoid[1])
						continue;

					for(GeometryInfo* geo1 : queue[dir][i]){
						// if both objects are non-stationary, geos should be associated with the same tick
						if( !obj0->param.stationary &&
							!obj1->param.stationary &&
							geo0->tick != geo1->tick )
							continue;

						if(avoid[0]){
							GPTableEntry& gp = gpTable(geo0->geoIndex[0], geo1->geoIndex[1]);
							gp.info[0] = geo0;
							gp.info[1] = geo1;
							gp.intersect |= (1 << dir);
							numInt[dir]++;
						}
						if(avoid[1]){
							GPTableEntry& gp = gpTable(geo1->geoIndex[0], geo0->geoIndex[1]);
							gp.info[0] = geo1;
							gp.info[1] = geo0;
							gp.intersect |= (1 << dir);
							numInt[dir]++;
						}
					}
				}

				queue[dir][obj0->objIndex].insert(e.geoInfo);
				szmax = std::max(szmax, (int)queue[dir][obj0->objIndex].size());
			}
			else{
				queue[dir][obj0->objIndex].erase(e.geoInfo);
			}
		}
	}
	int timeEnum = timer2.CountUS();

	timer2.CountUS();

	for(AvoidTask* avoid : avoids){
		for(Tick* tick : ticks){
			AvoidKey* key = (AvoidKey*)avoid->traj.GetKeypoint(tick);
			key->geoPairs.clear();
		}
	}

	int numIntAll = 0;
	for(GPTableEntry& gp : gpTable){
		if( gp.intersect != 0b111 )
			continue;

		GeometryInfo* geo0 = gp.info[0];
		GeometryInfo* geo1 = gp.info[1];
		Object*       obj0 = geo0->con->obj;
		Object*       obj1 = geo1->con->obj;
		Tick* tick = (!obj0->param.stationary ? geo0->tick : geo1->tick);
	
		AvoidTask* avoid = avoidTable(obj0->objIndex, obj1->objIndex);
		AvoidKey*  key   = (AvoidKey*)avoid->traj.GetKeypoint(tick);
		
		GeometryPair geoPair;
		geoPair.info0 = geo0;
		geoPair.info1 = geo1;
		key->geoPairs.push_back(geoPair);
		
		//printf("%s %s %d - %s %s %d\n",
		// geo0->con->obj->name.c_str(), geo0->geo->name.c_str(), geo0->tick->idx,
		// geo1->con->obj->name.c_str(), geo1->geo->name.c_str(), geo1->tick->idx);

		numIntAll++;
	}
	int timeInt = timer2.CountUS();

	if(solver->param.verbose){
		DSTR << " tsort: "    << timeSort
			 << " tenum: "    << timeEnum 
			 << " tint: "     << timeInt
			 << " queuemax: " << szmax
			 << " numint: " << numInt[0] << " " << numInt[1] << " " << numInt[2] << " " << numIntAll
			 << endl;
	}
}

void Graph::Finish(){
	nodes.Finish();

	// compute forward kinematics
	trees.ForwardKinematics();
}

void Graph::Step(){
	if(!ready)
		Init();

	timer.CountUS();
	Prepare();
	TPrepare = timer.CountUS();
	
	timer.CountUS();
	PrepareStep();
	TPrepareStep = timer.CountUS();
	
	timer.CountUS();
	solver->Step();
	TStep = timer.CountUS();

	timer.CountUS();
	Finish();
	TFinish = timer.CountUS();

	if(solver->param.verbose){
		DSTR << " tpre1: " << TPrepare;
		DSTR << " tpre2: " << TPrepareStep;
		DSTR << " tstp: "  << TStep;
		DSTR << " tfin: "  << TFinish << endl;
	}

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
		DrawSnapshot(tick->time, canvas, _conf);
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
