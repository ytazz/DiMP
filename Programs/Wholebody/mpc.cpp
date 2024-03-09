#include "mpc.h"

#include <sbtimer.h>
static Timer timer;

const real_t pi = 3.14159265358979;
const vec3_t one(1.0, 1.0, 1.0);

Mpc::Mpc(){
    handOffset[0]  = vec3_t(0.0, -0.20, -0.0);
    handOffset[1]  = vec3_t(0.0,  0.20, -0.0);
}

void Mpc::Init(){
    int nend   = (int)wb->ends  .size();
    int njoint = (int)wb->joints.size();

    data_cur.Init(wb);
    data_cur.InitJacobian(wb);
    data_ref.Init(wb);
    data_ref_des.Init(wb);

    // load reference trajectory
    csv.Read("log.csv", ", ", true);
	
    count   = 0;
    time    = 0.0;
	timeMpc = 0.0;

    // initialize state using desired state
    GetDesiredState(0, time, data_cur);
    data_ref = data_cur;
    data_ref_des = data_cur;
    
    wb->CalcPosition    (data_cur);
    wb->CalcJacobian    (data_cur);
    wb->CalcVelocity    (data_cur);
    wb->CalcAcceleration(data_cur);
    wb->CalcMomentum    (data_cur);
    wb->CalcMomentumDerivative(data_cur);
    wb->CalcForce       (data_cur);

    int nx = 12 + 2*njoint;    //< centroid (pos_t pos_r vel_t vel_r)  njoint*(q, qd)
    int nu = njoint + 6*nend;  //< njoint*qdd + nend*(force_t, force_r)
    dx  .Allocate(nx);
    du  .Allocate(nu);
    u   .Allocate(nu);
    uref.Allocate(nu);
    Quuinv_Qux.Allocate(nu, nx);
    vec_clear(uref);
    mat_clear(Quuinv_Qux);

    ready = false;
}

void Mpc::UpdateState(){
    int nend   = (int)wb->ends  .size();
    int njoint = (int)wb->joints.size();
    
    real_t dt_ctrl = dt/updateCycle;
    real_t dt_ctrl2 = dt_ctrl*dt_ctrl;
    
    // calc force and acceleration of base link
    wb->CalcComAcceleration(data_cur);
    wb->CalcBaseAcceleration(data_cur);
    
    data_cur.centroid.pos_t += data_cur.centroid.vel_t*dt_ctrl + data_cur.centroid.acc_t*(0.5*dt_ctrl2);
    data_cur.centroid.vel_t += data_cur.centroid.acc_t*dt_ctrl;
    data_cur.centroid.pos_r = quat_t::Rot(data_cur.centroid.vel_r*dt_ctrl + data_cur.centroid.acc_r*(0.5*dt_ctrl2))*data_cur.centroid.pos_r;
    data_cur.centroid.pos_r.unitize();
    data_cur.centroid.vel_r += data_cur.centroid.acc_r*dt_ctrl;

    for(int i = 0; i < njoint; i++){
        data_cur.q [i] += data_cur.qd [i]*dt_ctrl + data_cur.qdd[i]*(0.5*dt_ctrl2);
        data_cur.qd[i] += data_cur.qdd[i]*dt_ctrl;
    }

    wb->CalcPosition    (data_cur);
    wb->CalcJacobian    (data_cur);
    wb->CalcVelocity    (data_cur);
    wb->CalcAcceleration(data_cur);
    wb->CalcMomentum    (data_cur);
    wb->CalcMomentumDerivative(data_cur);
    wb->CalcForce       (data_cur);

}

void Mpc::UpdateInput(){
    if(!ready)
        return;

    int nend   = (int)wb->ends  .size();
    int njoint = (int)wb->joints.size();
    int idx;

    // calc dx
    idx = 0;
    vec_clear(dx);

    dx(idx++) = (data_cur.centroid.pos_t.x - data_ref.centroid.pos_t.x)/wb->scale.pt;
    dx(idx++) = (data_cur.centroid.pos_t.y - data_ref.centroid.pos_t.y)/wb->scale.pt;
    dx(idx++) = (data_cur.centroid.pos_t.z - data_ref.centroid.pos_t.z)/wb->scale.pt;

    quat_t qdiff = data_cur.centroid.pos_r*data_ref.centroid.pos_r.Conjugated();
    real_t theta = qdiff.Theta();
    if(theta > pi)
        theta -= 2*pi;
    vec3_t w = theta*qdiff.Axis();
    dx(idx++) = w.x/wb->scale.pr;
    dx(idx++) = w.y/wb->scale.pr;
    dx(idx++) = w.z/wb->scale.pr;

    dx(idx++) = (data_cur.centroid.vel_t.x - data_ref.centroid.vel_t.x)/wb->scale.vt;
    dx(idx++) = (data_cur.centroid.vel_t.y - data_ref.centroid.vel_t.y)/wb->scale.vt;
    dx(idx++) = (data_cur.centroid.vel_t.z - data_ref.centroid.vel_t.z)/wb->scale.vt;

    dx(idx++) = (data_cur.centroid.vel_r.x - data_ref.centroid.vel_r.x)/wb->scale.vr;
    dx(idx++) = (data_cur.centroid.vel_r.y - data_ref.centroid.vel_r.y)/wb->scale.vr;
    dx(idx++) = (data_cur.centroid.vel_r.z - data_ref.centroid.vel_r.z)/wb->scale.vr;

    for(int i = 0; i < njoint; i++){
        dx(idx++) = (data_cur.q [i] - data_ref.q [i])/wb->scale.pr;
        dx(idx++) = (data_cur.qd[i] - data_ref.qd[i])/wb->scale.vr;
    }

    // calc u
    // du = -Quu^-1 * (Qu + Qux * dx)
    //  * Qu shall be zero after convergence
    // K = -Quu^-1 * Qux
    // u = uref - Quuinv_Qux*dx;
    du.Resize(uref.n);
    u .Resize(uref.n);
    mat_vec_mul(Quuinv_Qux, dx, du, -1.0, 0.0);

    idx = 0;

    for(int i = 0; i < njoint; i++){
        du(idx++) *= wb->scale.ar;
    }

    for(int i = 0; i < nend; i++){
        du(idx++) *= wb->scale.ft;
        du(idx++) *= wb->scale.ft;
        du(idx++) *= wb->scale.ft;

        du(idx++) *= wb->scale.fr;
        du(idx++) *= wb->scale.fr;
        du(idx++) *= wb->scale.fr;
    }

    vec_copy(uref, u);
    vec_add (du, u);

    // decode u
    idx = 0;
    
    for(int i = 0; i < njoint; i++){
        data_cur.qdd[i] = u(idx++);
    }

    for(int i = 0; i < nend; i++){
        DiMP::WholebodyData::End&  dend_ref_des = data_ref_des.ends[i];
        DiMP::WholebodyData::End&  dend_cur = data_cur.ends[i];
        
        dend_cur.force_t.x = u(idx++);
        dend_cur.force_t.y = u(idx++);
        dend_cur.force_t.z = u(idx++);
        dend_cur.force_r.x = u(idx++);
        dend_cur.force_r.y = u(idx++);
        dend_cur.force_r.z = u(idx++);
    }
}

void Mpc::UpdateGain(){
    // copy data
    DiMP::WholebodyKey* key1 = (DiMP::WholebodyKey*)wb->traj.GetKeypoint(wb->graph->ticks[1]);
    data_ref     = key1->data;
    data_ref_des = key1->data_des;

    // calc input dimension
    int nend   = (int)wb->ends  .size();
    int njoint = (int)wb->joints.size();
    int nu     = njoint + nend*6;
    
    // calc uref
    uref.Resize(nu);
    int idx = 0;

    for(int i = 0; i < njoint; i++){
        uref(idx++) = data_ref.qdd[i];
    }
    for(int i = 0; i < nend; i++){
        DiMP::WholebodyData::End&  dend_ref = data_ref.ends[i];

        uref(idx++) = dend_ref.force_t.x;
        uref(idx++) = dend_ref.force_t.y;
        uref(idx++) = dend_ref.force_t.z;
        uref(idx++) = dend_ref.force_r.x;
        uref(idx++) = dend_ref.force_r.y;
        uref(idx++) = dend_ref.force_r.z;
    }

    // store matrices
    Quuinv_Qux.Resize(nu, Quuinv_Qux.n);
    mat_copy(wb->graph->solver->Quuinv_Qux[1], Quuinv_Qux);

    ready = true;
}

void Mpc::Countup(){
    count++;
    time += dt/updateCycle;
}

void Mpc::GetInitialState(DiMP::WholebodyData& d){
    d.centroid.pos_t = data_cur.centroid.pos_t;
    d.centroid.vel_t = data_cur.centroid.vel_t;
    d.centroid.pos_r = data_cur.centroid.pos_r;
    d.centroid.vel_r = data_cur.centroid.vel_r;

    int nend   = (int)d.ends.size();
    int njoint = (int)d.q   .size();

    for(int i = 0; i < njoint; i++){
        d.q  [i] = data_cur.q  [i];
        d.qd [i] = data_cur.qd [i];
        d.qdd[i] = data_cur.qdd[i];
    }

    for(int i = 0; i < nend; i++){
        DiMP::WholebodyData::End&  dend     = d.ends[i];
        DiMP::WholebodyData::End&  dend_cur = data_cur.ends[i];
        DiMP::WholebodyData::End&  dend_ref = data_ref.ends[i];
        DiMP::WholebodyData::End&  dend_ref_des = data_ref_des.ends[i];

        dend.pos_t   = dend_cur.pos_t;
        dend.pos_r   = dend_cur.pos_r;
        dend.vel_t   = dend_cur.vel_t;
        dend.vel_r   = dend_cur.vel_r;

        if(dend_ref_des.state == DiMP::Wholebody::ContactState::Free){
            dend.force_t.clear();
            dend.force_r.clear();
        }
        else{
            dend.force_t = dend_cur.force_t;
            dend.force_r = dend_cur.force_r;
        }
    }
}

void Mpc::GetDesiredState(int k, real_t t, DiMP::WholebodyData& d){
    const real_t dt = 0.001;
    int r = (t + timeMpc)/dt;
    int c = 1;

    d.centroid.pos_t = vec3_t(csv.Get<real_t>(r, c+0), csv.Get<real_t>(r, c+1), csv.Get<real_t>(r, c+2)); c += 3;
    d.centroid.vel_t = vec3_t(csv.Get<real_t>(r, c+0), csv.Get<real_t>(r, c+1), csv.Get<real_t>(r, c+2)); c += 3;
    d.centroid.pos_r = quat_t();
    d.centroid.vel_r = vec3_t();
    
    d.centroid.pos_t_weight = (k == N ? 10.0 : 1.0)*one;
    d.centroid.vel_t_weight = (k == N ? 10.0 : 1.0)*one;
    d.centroid.pos_r_weight = (k == N ? 10.0 : 1.0)*one;
    d.centroid.vel_r_weight = (k == N ? 10.0 : 1.0)*one;
    
    int nend   = (int)d.ends.size();
    int njoint = (int)d.q.size();

    for(int i = 0; i < njoint; i++){
        d.q  [i] = 0.0;
        d.qd [i] = 0.0;
        d.qdd[i] = 0.0;
    }
    d.q[ 4] = 0.5; d.q[ 7] = -1.0;
    d.q[11] = 0.5; d.q[14] = -1.0;
    d.q[20] = -0.5; d.q[21] = 1.0; d.q[22] = -0.5;
    d.q[26] = -0.5; d.q[27] = 1.0; d.q[28] = -0.5;

    for(int i = 0;  i < njoint; i++){
        d.q_weight  [i] = 0.01;
        d.qd_weight [i] = 0.1;
        d.qdd_weight[i] = 10.0;
    }

    for(int i = 0; i < nend; i++){
        DiMP::WholebodyData::End&  dend     = d.ends[i];
        
        vec3_t pe;
        quat_t qe;
        vec3_t ve;
        vec3_t we;

        if(i == MyIK::End::ChestP){
            pe         = d.centroid.pos_t + vec3_t(0.0, 0.0, myik->torsoLength);
            qe         = quat_t();
            ve         = d.centroid.vel_t;
            we         = vec3_t();
            dend.state  = DiMP::Wholebody::ContactState::Free;
        }
        if(i == MyIK::End::HandR){
            pe         = d.centroid.pos_t + handOffset[0];
            qe         = quat_t();
            ve         = d.centroid.vel_t;
            we         = vec3_t();
            dend.state  = DiMP::Wholebody::ContactState::Free;
        }
        if(i == MyIK::End::HandL){
            pe         = d.centroid.pos_t + handOffset[1];
            qe         = quat_t();
            ve         = d.centroid.vel_t;
            we         = vec3_t();
            dend.state  = DiMP::Wholebody::ContactState::Free;
        }
        if(i == MyIK::End::FootR || i == MyIK::End::FootL){
            c = (i == MyIK::End::FootR ? 1+3+3 : 1+3+3+7+6+6+1);
            pe           = vec3_t(csv.Get<real_t>(r, c+0), csv.Get<real_t>(r, c+1), csv.Get<real_t>(r, c+2)); c += 3;
            qe           = quat_t(csv.Get<real_t>(r, c+0), csv.Get<real_t>(r, c+1), csv.Get<real_t>(r, c+2), csv.Get<real_t>(r, c+3)); c += 4;
            ve           = vec3_t(csv.Get<real_t>(r, c+0), csv.Get<real_t>(r, c+1), csv.Get<real_t>(r, c+2)); c += 3;
            we           = vec3_t(csv.Get<real_t>(r, c+0), csv.Get<real_t>(r, c+1), csv.Get<real_t>(r, c+2)); c += 3;
            dend.force_t = vec3_t(csv.Get<real_t>(r, c+0), csv.Get<real_t>(r, c+1), csv.Get<real_t>(r, c+2)); c += 3;
            dend.force_r = vec3_t(csv.Get<real_t>(r, c+0), csv.Get<real_t>(r, c+1), csv.Get<real_t>(r, c+2)); c += 3;

            int stat = csv.Get<int>(r, c++);

            if(stat == DiMP::BipedLIP::ContactState::Float){
                dend.state  = DiMP::Wholebody::ContactState::Free;
                dend.force_t.clear();
                dend.force_r.clear();
            }
            if(stat == DiMP::BipedLIP::ContactState::Surface){
                dend.state   = DiMP::Wholebody::ContactState::Surface;
                dend.mu      = 0.2;
                dend.cop_min = vec3_t(-0.1, -0.05, 0.0);
                dend.cop_max = vec3_t( 0.1,  0.05, 0.0);
                dend.pos_te  = vec3_t(0.0, 0.0, 0.0);
                dend.pos_tc  = pe;
                dend.pos_rc  = quat_t();
            }
            if(stat == DiMP::BipedLIP::ContactState::Heel){
                dend.state   = DiMP::Wholebody::ContactState::Line;
                dend.mu      = 0.2;
                dend.cop_min = vec3_t(-0.1, -0.05, 0.0);
                dend.cop_max = vec3_t( 0.1,  0.05, 0.0);
                dend.pos_te  = vec3_t(0.1, 0.0, 0.0);
                dend.pos_tc  = pe + qe*dend.pos_te;
                dend.pos_rc  = quat_t();
            }
            if(stat == DiMP::BipedLIP::ContactState::Toe ){
                dend.state   = DiMP::Wholebody::ContactState::Line;
                dend.mu      = 0.2;
                dend.cop_min = vec3_t(-0.1, -0.05, 0.0);
                dend.cop_max = vec3_t( 0.1,  0.05, 0.0);
                dend.pos_te  = vec3_t(-0.1, 0.0, 0.0);
                dend.pos_tc  = pe + qe*dend.pos_te;
                dend.pos_rc  = quat_t();
            }
        }

        // transform to local coordinate
        dend.pos_t = d.centroid.pos_r.Conjugated()*(pe - d.centroid.pos_t);
        dend.pos_r = d.centroid.pos_r.Conjugated()*qe;
        dend.vel_t = d.centroid.pos_r.Conjugated()*(ve - (d.centroid.vel_t + d.centroid.vel_r%(pe - d.centroid.pos_t)));
        dend.vel_r = d.centroid.pos_r.Conjugated()*(we -  d.centroid.vel_r);	

        dend.pos_t_weight   = one*(k == N ? 10.0 : 1.0)*(dend.state == DiMP::Wholebody::ContactState::Free ? 0.1 : 1.0);
        dend.pos_r_weight   = one*(k == N ? 10.0 : 1.0)*(dend.state == DiMP::Wholebody::ContactState::Free ? 0.1 : 1.0);
        dend.vel_t_weight   = one*(k == N ? 10.0 : 1.0)*(dend.state == DiMP::Wholebody::ContactState::Free ? 0.1 : 1.0);
        dend.vel_r_weight   = one*(k == N ? 10.0 : 1.0)*(dend.state == DiMP::Wholebody::ContactState::Free ? 0.1 : 1.0);
        dend.force_t_weight = one*(k == N ? 10.0 : 1.0);
        dend.force_r_weight = one*(k == N ? 10.0 : 1.0);
    }
}	
