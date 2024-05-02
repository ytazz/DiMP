#include "mpc.h"

#include <sbtimer.h>
static Timer timer;

const real_t pi = 3.14159265358979;
const vec3_t one (1.0, 1.0, 1.0);
const vec2_t one2(1.0, 1.0);

Mpc::Mpc(){
}

void Mpc::Init(){
    comHeight = 0.7;
    endConf.resize(2);
    endConf[0].basePos   = vec3_t( 0.0, -0.15/2.0,  0.0);
    endConf[1].basePos   = vec3_t( 0.0,  0.15/2.0,  0.0);
    endConf[0].posOrigin = vec3_t( 0.0,  0.0, -0.7);
    endConf[1].posOrigin = vec3_t( 0.0,  0.0, -0.7);
    endConf[0].posMin    = vec3_t(-0.30, -0.05, -comHeight-0.1);
    endConf[1].posMin    = vec3_t(-0.30, -0.05, -comHeight-0.1);
    endConf[0].posMax    = vec3_t( 0.30,  0.05, -comHeight+0.1);
    endConf[1].posMax    = vec3_t( 0.30,  0.05, -comHeight+0.1);
    endConf[0].stiffMax  = 50.0;
    endConf[1].stiffMax  = 50.0;
    endConf[0].cmpOffset = -0.0*vec2_t(endConf[0].basePos.x, endConf[0].basePos.y);
    endConf[1].cmpOffset = -0.0*vec2_t(endConf[1].basePos.x, endConf[1].basePos.y);

    cen->callback = this;

	cen->param.g = 9.8;
	cen->param.m = 44.0;
    cen->param.I[0][0] = 20.0;
    cen->param.I[1][1] = 20.0;
    cen->param.I[2][2] = 20.0;
    cen->param.swingHeight = 0.15;
    cen->param.mu = 1.0;

    cen->param.durationMin = 0.10;
    cen->param.durationMax = 0.55;

    cen->param.contactMargin = 0.0;

    cen->param.enableRotation   = true;
    cen->param.rotationResolution = 10;
        
    // create geometry
    cen->point = new DiMP::Point(graph);
    cen->hull  = new DiMP::Hull (graph);

    cen->param.bodyRangeMin = vec3_t(-0.05, -0.05,  0.0);
    cen->param.bodyRangeMax = vec3_t( 0.05,  0.05,  0.5);
		
	DiMP::Centroid::Face face;
    // flat ground
    real_t r = 0.0;
    face.hull = new DiMP::Hull(graph);
    face.hull->radius = r - cen->param.contactMargin;
    face.hull->vertices.push_back(vec3_t(-1.0, -5.0,  -r));
    face.hull->vertices.push_back(vec3_t(-1.0,  5.0,  -r));
    face.hull->vertices.push_back(vec3_t(15.0,  5.0,  -r));
    face.hull->vertices.push_back(vec3_t(15.0, -5.0,  -r));
    face.normal = vec3_t(0.0, 0.0, 1.0);
    face.numSwitchMax = 100;
    cen->faces.push_back(face);
		
    const int infi = numeric_limits<int>::max();
    int nend = endConf.size();
		
    N = 12;
    startPos = vec3_t(0.0, 0.0, 0.0);
	startOri = vec3_t();
    desVel   = vec3_t(0.3, 0.0, 0.0);
    desDuration = 0.3;

    cen->ends.resize(nend);
	for(int iend = 0; iend < nend; iend++){
        cen->ends[iend].point = new DiMP::Point(graph);
			
        cen->ends[iend].basePos = endConf[iend].basePos;
        cen->ends[iend].posMin  = endConf[iend].posMin;
        cen->ends[iend].posMax  = endConf[iend].posMax; 
		cen->ends[iend].copMin  = vec2_t(-0.05, -0.01);
		cen->ends[iend].copMax  = vec2_t( 0.05,  0.01);

        cen->ends[iend].stiffnessMax = endConf[iend].stiffMax;
        cen->ends[iend].cmpOffset  = endConf[iend].cmpOffset;
        cen->ends[iend].lockOri    = false;
        cen->ends[iend].lockCmp    = false;
        cen->ends[iend].lockMoment = false;
	}
                
	for(int k = 0; k <= N; k++)
		new DiMP::Tick(graph, k*desDuration, "");
		
    cen->SetScaling();
	graph->scale.Set(1.0, 1.0, 1.0);
	graph->Init();

    data_cur.Init(cen);
    data_ref.Init(cen);
    data_ref_des.Init(cen);

    // initialize state using desired state
    phase = Phase::RL;
    planTrigger = false;
    delayMode   = false;
    inputReady  = false;
    data_cur.time     = 0.0;
    data_cur.duration = desDuration;
    GetDesiredState(0, time, data_cur);
    cen->CalcStiffness(data_cur);
    data_ref     = data_cur;
    data_ref_des = data_cur;

    cen->Setup();
    cen->Reset(true, true, true);
        
	//graph->solver->Enable(ID(DiMP::ConTag::CentroidPosT      ), false);
	//graph->solver->Enable(ID(DiMP::ConTag::CentroidPosR      ), false);
	//graph->solver->Enable(ID(DiMP::ConTag::CentroidVelT      ), false);
	//graph->solver->Enable(ID(DiMP::ConTag::CentroidVelR      ), false);
	//graph->solver->Enable(ID(DiMP::ConTag::CentroidTime      ), false);
    //graph->solver->Enable(ID(DiMP::ConTag::CentroidEndPos    ), false);
	//graph->solver->Enable(ID(DiMP::ConTag::CentroidEndVel    ), false);
	//graph->solver->Enable(ID(DiMP::ConTag::CentroidEndStiff  ), false);
	graph->solver->Enable(ID(DiMP::ConTag::CentroidEndPosRange  ), false);
	//graph->solver->Enable(ID(DiMP::ConTag::CentroidEndContact), false);
    graph->solver->Enable(ID(DiMP::ConTag::CentroidEndFriction), false);
	graph->solver->Enable(ID(DiMP::ConTag::CentroidEndMomentRange), false);
        
    graph->solver->SetCorrection(ID(), 0.5);
	graph->solver->param.regularization = 10;
	graph->solver->param.stateRegularization = 0;
	graph->solver->param.hastyStepSize  = false;
	graph->solver->param.cutoffStepSize = 0.1;
	graph->solver->param.minStepSize    = 0.1;
	graph->solver->param.maxStepSize    = 1.0;
    graph->solver->param.methodMajor    = Solver::Method::Major::DDP;
    //graph->solver->param.methodMajor    = Solver::Method::Major::GaussNewton;
    //graph->solver->param.methodMajor    = DiMP::CustomSolver::CustomMethod::SearchDDP;
	graph->solver->param.methodMinor    = Solver::Method::Minor::Direct;
    graph->solver->param.useHessian     = false;
	graph->solver->param.verbose        = true;
    graph->solver->param.parallelize    = false;
    graph->solver->param.fixInitialState = true;
    graph->solver->param.fixInitialInput = false;
    
    //fileDuration = fopen("duration.csv", "w");
    //fileCost     = fopen("cost.csv", "w");

    //fprintf(fileCost,
    //    "iter, step, obj, Tpre, Tdir, Tstep, Tmod, Trans, Tcost, Tcostgrad, Tback, Tpre, Tprestep, Tstep, Tfinish, wcompl\n"
    //);

    updateCycle = 5;
    count   = 0;
    dt      = 0.005;
    time    = 0.0;
	
    nx = 6 + (cen->param.enableRotation ? 6 : 0) + 1;
    nu = 1;
    for(int i = 0; i < nend; i++){
        nx += 3 + (cen->ends[i].lockOri ? 0 : 3);
        nu += 3 + (cen->ends[i].lockOri ? 0 : 3) + 1 + (cen->ends[i].lockCmp ? 0 : 2) + (cen->ends[i].lockMoment ? 0 : 3);
    }
    dx  .Allocate(nx);
    du  .Allocate(nu);
    u   .Allocate(nu);
    uref.Allocate(nu);
    Quuinv_Qux.Allocate(nu, nx);
    vec_clear(uref);
    mat_clear(Quuinv_Qux);
}

void Mpc::UpdateState(){
    int nend = (int)cen->ends.size();
    
    real_t dt2 = dt*dt;

    cen->CalcWrench          (data_cur);
    cen->CalcComAcceleration (data_cur);
    cen->CalcBaseAcceleration(data_cur);
    
    data_cur.pos_t += data_cur.vel_t*dt + data_cur.acc_t*(0.5*dt2);
    data_cur.vel_t += data_cur.acc_t*dt;
    data_cur.pos_r = quat_t::Rot(data_cur.vel_r*dt + data_cur.acc_r*(0.5*dt2))*data_cur.pos_r;
    data_cur.pos_r.unitize();
    data_cur.vel_r += data_cur.acc_r*dt;

    DSTR << data_cur.pos_t << " " << data_cur.vel_t << endl;

    for(int i = 0; i < nend; i++){
        DiMP::CentroidData::End& dend = data_cur.ends[i];

        dend.pos_t += dend.vel_t*dt;
        dend.pos_r = quat_t::Rot(dend.vel_r*dt)*dend.pos_r;
        dend.pos_r.unitize();
    }

    data_cur.time += dt;
    data_cur.duration -= dt;

    if(data_cur.duration <= 0.0){
        data_cur.duration = desDuration;
        phase = (phase + 1)%Phase::Num;
        for(int i = 0; i < nend; i++){
            data_cur.ends[i].iface = ((i == 0 && phase != Phase::L) || (i == 1 && phase != Phase::R)) ? 0 : -1;
        }
        planTrigger = true;
    }
}

inline vec3_t quat_diff(quat_t q0, quat_t q1){
    quat_t qdiff = q0.Conjugated()*q1;
    real_t theta = qdiff.Theta();
    if(theta > pi)
        theta -= 2*pi;
    vec3_t w = theta*qdiff.Axis();
    return (1.0/2.0)*(q0*w + q1*w);
}

inline void subvec_set(Vector& v, int& idx, vec3_t sv){
    v(idx++) = sv[0];
    v(idx++) = sv[1];
    v(idx++) = sv[2];
}

inline vec2_t subvec2_get(Vector& v, int& idx){
    vec2_t sv(v(idx+0), v(idx+1));
    idx += 2;
    return sv;
}
inline vec3_t subvec3_get(Vector& v, int& idx){
    vec3_t sv(v(idx+0), v(idx+1), v(idx+2));
    idx += 3;
    return sv;
}

void Mpc::UpdateInput(){
    if(!inputReady)
        return;

    int nend = (int)cen->ends.size();
    int idx;

    // calc dx
    idx = 0;
    vec_clear(dx);

    subvec_set(dx, idx, (data_cur.pos_t - data_ref.pos_t)/cen->scale.pt);
    if(cen->param.enableRotation)
        subvec_set(dx, idx, quat_diff(data_ref.pos_r, data_cur.pos_r)/cen->scale.pr);
    subvec_set(dx, idx, (data_cur.vel_t - data_ref.vel_t)/cen->scale.vt);
    if(cen->param.enableRotation)
        subvec_set(dx, idx, (data_cur.vel_r - data_ref.vel_r)/cen->scale.vr);
    dx(idx++) = (data_cur.time - data_ref.time)/cen->scale.t;

    for(int i = 0; i < nend; i++){
        DiMP::CentroidData::End&  dend     = data_cur.ends[i];
        DiMP::CentroidData::End&  dend_ref = data_ref.ends[i];
        
        subvec_set(dx, idx, (dend.pos_t - dend_ref.pos_t)/cen->scale.pt);
        if(!cen->ends[i].lockOri)
            subvec_set(dx, idx, quat_diff(dend_ref.pos_r, dend.pos_r)/cen->scale.pr);
    }

    // calc u
    mat_vec_mul(Quuinv_Qux, dx, du, -0.0, 0.0);

    // decode u
    idx = 0;

    // duration is not updated
    //data_cur.duration = data_ref.duration + cen->scale.t*du(idx++);
    idx++;

    for(int i = 0; i < nend; i++){
        DiMP::CentroidData::End&  dend     = data_cur.ends[i];
        DiMP::CentroidData::End&  dend_ref = data_ref.ends[i];

        dend.vel_t  = dend_ref.vel_t + cen->scale.vt*subvec3_get(du, idx);
        if(!cen->ends[i].lockOri)
            dend.vel_r  = dend_ref.vel_r + cen->scale.vr*subvec3_get(du, idx);
        
        dend.stiff  = dend_ref.stiff  + cen->scale.tinv*du(idx++);
        if(!cen->ends[i].lockCmp)
            dend.cmp    = dend_ref.cmp    + cen->scale.pt  *subvec2_get(du, idx);
        if(!cen->ends[i].lockMoment)
            dend.moment = dend_ref.moment + cen->scale.pt2 *subvec3_get(du, idx);
    }
}

void Mpc::UpdateGain(){
    // copy data
    auto key = (DiMP::CentroidKey*)cen->traj.GetKeypoint(cen->graph->ticks[(delayMode ? 1 : 0)]);
    data_ref_des = key->data_des;
    data_ref     = key->data    ;
    
    // store matrices
    mat_copy(cen->graph->solver->Quuinv_Qux[delayMode ? 1 : 0], Quuinv_Qux);

    //SaveGain();

    inputReady = true;
}

void Mpc::Countup(){
    count++;
    time += dt/updateCycle;
}

void Mpc::GetInitialState(DiMP::CentroidData& d){
    d.pos_t    = data_cur.pos_t;
    d.vel_t    = data_cur.vel_t;
    d.pos_r    = data_cur.pos_r;
    d.vel_r    = data_cur.vel_r;
    d.time     = data_cur.time;
    
    d.duration = data_cur.duration;
    d.duration_weight = 1000.0;  //< large weight for initial duration

    int nend = (int)d.ends.size();
    
    for(int i = 0; i < nend; i++){
        DiMP::CentroidData::End&  dend     = d.ends[i];
        DiMP::CentroidData::End&  dend_cur = data_cur.ends[i];
        
        dend.pos_t = dend_cur.pos_t;
        dend.pos_r = dend_cur.pos_r;
        dend.vel_t = dend_cur.vel_t;
        dend.vel_r = dend_cur.vel_r;

        dend.iface = dend_cur.iface;
    }
}

void Mpc::GetDesiredState(int k, real_t t, DiMP::CentroidData& d){
    int ph = (k == N ? Phase::LR : ((phase + k) % Phase::Num));

    // k=0 : t=t_cur
    // k=1 : t=t_cur + tau_cur
    // k=2 : t=t_cur + tau_cur + tau_des
    // k=3 : t=t_cur + tau_cur + 2*tau_des

    real_t tref = data_cur.time + data_cur.duration + (k-1)*desDuration;
    d.pos_t = startPos + desVel*tref + vec3_t(0.0, 0.0, comHeight);
    d.vel_t = desVel;
    d.pos_r = quat_t();
    d.vel_r = vec3_t();
    
    d.pos_t_weight = (k == N ? 10.0 : 1.0)*1.0*one;
    d.vel_t_weight = (k == N ? 10.0 : 1.0)*10.0*one;
    d.pos_r_weight = (k == N ? 10.0 : 1.0)*10.0*one;
    //d.vel_r_weight = (k == N ? 10.0 : 1.0)*10.0*one;
    d.L_weight     = (k == N ? 10.0 : 1.0)*10.0*one;

    d.time  = tref;
    d.time_weight  = (k == N ? 10.0 : 1.0);
    
    d.duration = desDuration;
    d.duration_weight = 1.0;

    int nend = (int)d.ends.size();
    for(int i = 0; i < nend; i++){
        DiMP::CentroidData::End& dend = d.ends[i];
        
        dend.pos_t = startPos + desVel*tref + cen->ends[i].basePos;
        dend.pos_r = quat_t();
        dend.vel_t = vec3_t();//desVel;
        dend.vel_r = vec3_t();

        dend.pos_t_weight = (k == N ? 10.0 : 1.0)*vec3_t(1.0, 10.0, 10.0);
        dend.pos_r_weight = (k == N ? 10.0 : 1.0)*one;
        dend.vel_t_weight = (k == N ? 10.0 : 1.0)*one;
        dend.vel_r_weight = (k == N ? 10.0 : 1.0)*one;

        dend.stiff_weight  = 1.0;
        dend.cmp_weight    = 10.0*one2;
        dend.moment_weight = 10.0*one;
    
        dend.iface = ((i == 0 && ph != Phase::L) || (i == 1 && ph != Phase::R)) ? 0 : -1;
    }
}	

void Mpc::SavePlan(){
    static int idx = 0;
    char filename[256];
    sprintf(filename, "plan_centroid.csv");
    FILE* file = fopen(filename, "w");
    idx++;

    fprintf(file, 
        "k, "
        "time, duration, "
        "cen_pos_t_x, cen_pos_t_y, cen_pos_t_z, "
        "cen_vel_t_x, cen_vel_t_y, cen_vel_t_z, "
        "cen_pos_r_x, cen_pos_r_y, cen_pos_r_z, "
        "cen_vel_r_x, cen_vel_r_y, cen_vel_r_z, "
    );
    for(int i = 0; i < cen->ends.size(); i++){
        fprintf(file,
            "end%d_pos_t_x, end%d_pos_t_y, end%d_pos_t_z, "
            "end%d_vel_t_x, end%d_vel_t_y, end%d_vel_t_z, "
            "end%d_pos_r_x, end%d_pos_r_y, end%d_pos_r_z, "
            "end%d_vel_r_x, end%d_vel_r_y, end%d_vel_r_z, "
            "end%d_stiff, "
            "end%d_cmp_x, end%d_cmp_y, end%d_cmp_z, "
            "end%d_mom_x, end%d_mom_y, end%d_mom_z, ",
            i, i, i,
            i, i, i,
            i, i, i,
            i, i, i,
            i,
            i, i, i,
            i, i, i
            );
    }
    for(int i = 0; i < cen->ends.size(); i++){
        fprintf(file,
            "end%d_iface, ",
            i
            );
    }
    fprintf(file, "\n");

    for(int k = 0; k < graph->ticks.size(); k++){
        auto key = (DiMP::CentroidKey*)cen->traj.GetKeypoint(graph->ticks[k]);
        
        fprintf(file,
            "%d, "
            "%f, %f, "
            "%f, %f, %f, "
            "%f, %f, %f, ",
            k, 
            key->var_time->val, (key->next ? key->var_duration->val : 0.0),
            key->var_pos_t->val.x, key->var_pos_t->val.y, key->var_pos_t->val.z, 
            key->var_vel_t->val.x, key->var_vel_t->val.y, key->var_vel_t->val.z
        );
        if(cen->param.enableRotation){
            fprintf(file,
                "%f, %f, %f, "
                "%f, %f, %f, ",
                key->var_pos_r->val.x, key->var_pos_r->val.y, key->var_pos_r->val.z, 
                //key->var_vel_r->val.x, key->var_vel_r->val.y, key->var_vel_r->val.z
                key->var_L->val.x, key->var_L->val.y, key->var_L->val.z
            );
        }
        else{
            fprintf(file,
                "%f, %f, %f, "
                "%f, %f, %f, ",
                0.0, 0.0, 0.0,
                0.0, 0.0, 0.0
            );
        }
        for(int i = 0; i < key->ends.size(); i++){
            DiMP::CentroidKey::End& end = key->ends[i];
            fprintf(file,
                "%f, %f, %f, "
                "%f, %f, %f, ",
                end.var_pos_t->val.x, end.var_pos_t->val.y, end.var_pos_t->val.z, 
                end.var_vel_t->val.x, end.var_vel_t->val.y, end.var_vel_t->val.z
            );
            if(cen->param.enableRotation){
                fprintf(file,
                    "%f, %f, %f, "
                    "%f, %f, %f, ",
                    end.var_pos_r->val.x, end.var_pos_r->val.y, end.var_pos_r->val.z, 
                    end.var_vel_r->val.x, end.var_vel_r->val.y, end.var_vel_r->val.z
                );
            }
            else{
                fprintf(file,
                    "%f, %f, %f, "
                    "%f, %f, %f, ",
                    0.0, 0.0, 0.0,
                    0.0, 0.0, 0.0
                );
            }
            fprintf(file,
                "%f, ",
                end.var_stiff->val);

            if(cen->param.enableRotation){
                fprintf(file,
                    "%f, %f, %f, "
                    "%f, %f, %f, ",
                    end.var_cmp[0]->val, end.var_cmp[1]->val, 0.0, 
                    end.var_moment->val.x, end.var_moment->val.y, end.var_moment->val.z
                );
            }
            else{
                fprintf(file,
                    "%f, %f, %f, "
                    "%f, %f, %f, ",
                    0.0, 0.0, 0.0,
                    0.0, 0.0, 0.0
                );
            }
        }
        for(int i = 0; i < key->ends.size(); i++){
            fprintf(file,
                "%d, ",
                key->data_des.ends[i].iface
            );    
        }
        fprintf(file, "\n");
    }

    fclose(file);
}

void Mpc::SaveTraj(){
    char filename[256];
    sprintf(filename, "traj_centroid.csv");
    FILE* file = fopen(filename, "w");
    
    fprintf(file, 
        "k, "
        "cen_pos_t_x, cen_pos_t_y, cen_pos_t_z, "
        "cen_pos_r_w, cen_pos_r_x, cen_pos_r_y, cen_pos_r_z, "
        "cen_vel_t_x, cen_vel_t_y, cen_vel_t_z, "
        "cen_vel_r_x, cen_vel_r_y, cen_vel_r_z, "
    );
    for(int i = 0; i < cen->ends.size(); i++){
        fprintf(file,
            "end%d_pos_t_x, end%d_pos_t_y, end%d_pos_t_z, "
            "end%d_pos_r_w, end%d_pos_r_x, end%d_pos_r_y, end%d_pos_r_z, "
            "end%d_vel_t_x, end%d_vel_t_y, end%d_vel_t_z, "
            "end%d_vel_r_x, end%d_vel_r_y, end%d_vel_r_z, "
            "end%d_force_t_x, end%d_force_t_y, end%d_force_t_z, "
            "end%d_force_r_x, end%d_force_r_y, end%d_force_r_z, ",
            i, i, i,
            i, i, i, i,
            i, i, i,
            i, i, i,
            i, i, i,
            i, i, i
            );
    }
    // additional info
    fprintf(file, "cen_mom_x, cen_mom_y, cen_mom_z, ");
    fprintf(file, "\n");


    auto key = (DiMP::CentroidKey*)cen->traj.GetKeypoint(graph->ticks.back());
    real_t tf = key->var_time->val;
    const real_t dt = 0.01;
    DiMP::CentroidData d;

    for(real_t t = 0.0; t <= tf; t += dt){
        cen->CalcState(t, d);

        fprintf(file,
            "%f, "
            "%f, %f, %f, "
            "%f, %f, %f, %f, "
            "%f, %f, %f, "
            "%f, %f, %f, ",
            t, 
            d.pos_t.x, d.pos_t.y, d.pos_t.z, 
            d.pos_r.w, d.pos_r.x, d.pos_r.y, d.pos_r.z,
            d.vel_t.x, d.vel_t.y, d.vel_t.z, 
            d.vel_r.x, d.vel_r.y, d.vel_r.z
        );
        vec3_t mom;
        for(int i = 0; i < key->ends.size(); i++){
            mom += (d.ends[i].pos_t - d.pos_t) % d.ends[i].force_t + d.ends[i].force_r;
            
            fprintf(file,
                "%f, %f, %f, "
                "%f, %f, %f, %f, "
                "%f, %f, %f, "
                "%f, %f, %f, "
                "%f, %f, %f, "
                "%f, %f, %f, ",
                d.ends[i].pos_t.x, d.ends[i].pos_t.y, d.ends[i].pos_t.z, 
                d.ends[i].pos_r.w, d.ends[i].pos_r.x, d.ends[i].pos_r.y, d.ends[i].pos_r.z,
                d.ends[i].vel_t.x, d.ends[i].vel_t.y, d.ends[i].vel_t.z, 
                d.ends[i].vel_r.x, d.ends[i].vel_r.y, d.ends[i].vel_r.z,
                d.ends[i].force_t.x, d.ends[i].force_t.y, d.ends[i].force_t.z,
                d.ends[i].force_r.x, d.ends[i].force_r.y, d.ends[i].force_r.z
            );    
        }
        fprintf(file, "%f, %f, %f, ", mom.x, mom.y, mom.z);
        fprintf(file, "\n");
    }

    fclose(file);
}
