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
    int nend = wb->ends.size();

    data_cur.Init(wb);
    data_cur.InitJacobian(wb);
    data_ref.Init(wb);
    data_ref_des.Init(wb);
    data_tmp.resize(3 + 6*nend);
	for(auto& d : data_tmp){
		d.Init(wb);
	}	

    // load reference trajectory
    csv.Read("log.csv", ", ", true);
	
    count   = 0;
    time    = 0.0;
	timeMpc = 0.0;

    // initialize state using desired state
    GetDesiredState(0, time, data_cur);

    wb->CalcPosition    (data_cur, false);
    wb->CalcJacobian    (data_cur, data_tmp);
    wb->CalcVelocity    (data_cur, false);
    wb->CalcAcceleration(data_cur);
    wb->CalcMomentum    (data_cur);
    wb->CalcMomentumDerivative(data_cur);
    wb->CalcForce       (data_cur);

    int nx = 12 + 6 + 18*nend; //< centroid (pos_t pos_r vel_t vel_r) base (pos_r vel_r)  nend*(pos_t pos_r vel_t vel_r force_t force_r)
    int nu = 3 + 12*nend;      //< base (acc_r)  nend*(acc_t acc_r forcerate_t forcerate_r)  maximum size: actual size is changed depending on contact state
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
    int nend = wb->ends.size();
    
    real_t dt_ctrl = dt/updateCycle;
    
    // calc force and acceleration of base link
    wb->CalcComAcceleration(data_cur);
    wb->CalcBaseAcceleration(data_cur);
    
    data_cur.centroid.pos_t += data_cur.centroid.vel_t*dt_ctrl;
    data_cur.centroid.vel_t += data_cur.centroid.acc_t*dt_ctrl;
    data_cur.centroid.pos_r = quat_t::Rot(data_cur.centroid.vel_r*dt_ctrl)*data_cur.centroid.pos_r;
    data_cur.centroid.pos_r.unitize();
    data_cur.centroid.vel_r += data_cur.centroid.acc_r*dt_ctrl;

    data_cur.base.pos_r = quat_t::Rot(data_cur.base.vel_r*dt_ctrl)*data_cur.base.pos_r;
    data_cur.base.pos_r.unitize();
    data_cur.base.vel_r += data_cur.base.acc_r*dt_ctrl;

    // update position and velocity
    for(int i = 0; i < nend; i++){
        DiMP::WholebodyData::End& dend_cur = data_cur.ends[i];

        dend_cur.pos_t += dend_cur.vel_t*dt_ctrl;
        dend_cur.pos_r = quat_t::Rot(dend_cur.vel_r*dt_ctrl)*dend_cur.pos_r;
        dend_cur.pos_r.unitize();

        dend_cur.vel_t += dend_cur.acc_t*dt_ctrl;
        dend_cur.vel_r += dend_cur.acc_r*dt_ctrl;

        dend_cur.force_t += dend_cur.forcerate_t*dt_ctrl;
        dend_cur.force_r += dend_cur.forcerate_r*dt_ctrl;
    }
    
    wb->CalcPosition    (data_cur, false);
    wb->CalcJacobian    (data_cur, data_tmp);
    wb->CalcVelocity    (data_cur, false);
    wb->CalcAcceleration(data_cur);
    wb->CalcMomentum    (data_cur);
    wb->CalcMomentumDerivative(data_cur);
    wb->CalcForce       (data_cur);

}

void Mpc::UpdateInput(){
    if(!ready)
        return;

    int nend = wb->ends.size();
    int idx;

    // calc dx
    idx = 0;
    vec_clear(dx);

    dx(idx++) = (data_cur.centroid.pos_t.x - data_ref.centroid.pos_t.x)/wb->spt;
    dx(idx++) = (data_cur.centroid.pos_t.y - data_ref.centroid.pos_t.y)/wb->spt;
    dx(idx++) = (data_cur.centroid.pos_t.z - data_ref.centroid.pos_t.z)/wb->spt;

    quat_t qdiff = data_cur.centroid.pos_r*data_ref.centroid.pos_r.Conjugated();
    real_t theta = qdiff.Theta();
    if(theta > pi)
        theta -= 2*pi;
    vec3_t w = theta*qdiff.Axis();
    dx(idx++) = w.x/wb->spr;
    dx(idx++) = w.y/wb->spr;
    dx(idx++) = w.z/wb->spr;

    dx(idx++) = (data_cur.centroid.vel_t.x - data_ref.centroid.vel_t.x)/wb->svt;
    dx(idx++) = (data_cur.centroid.vel_t.y - data_ref.centroid.vel_t.y)/wb->svt;
    dx(idx++) = (data_cur.centroid.vel_t.z - data_ref.centroid.vel_t.z)/wb->svt;

    dx(idx++) = (data_cur.centroid.vel_r.x - data_ref.centroid.vel_r.x)/wb->svr;
    dx(idx++) = (data_cur.centroid.vel_r.y - data_ref.centroid.vel_r.y)/wb->svr;
    dx(idx++) = (data_cur.centroid.vel_r.z - data_ref.centroid.vel_r.z)/wb->svr;

    qdiff = data_cur.base.pos_r*data_ref.base.pos_r.Conjugated();
    theta = qdiff.Theta();
    if(theta > pi)
        theta -= 2*pi;
    w = theta*qdiff.Axis();
    dx(idx++) = w.x/wb->spr;
    dx(idx++) = w.y/wb->spr;
    dx(idx++) = w.z/wb->spr;

    dx(idx++) = (data_cur.base.vel_r.x - data_ref.base.vel_r.x)/wb->svr;
    dx(idx++) = (data_cur.base.vel_r.y - data_ref.base.vel_r.y)/wb->svr;
    dx(idx++) = (data_cur.base.vel_r.z - data_ref.base.vel_r.z)/wb->svr;

    for(int i = 0; i < nend; i++){
        DiMP::WholebodyData::End& dend_ref = data_ref.ends[i];
        DiMP::WholebodyData::End& dend_cur = data_cur.ends[i];
        
        dx(idx++) = (dend_cur.pos_t.x - dend_ref.pos_t.x)/wb->spt;
        dx(idx++) = (dend_cur.pos_t.y - dend_ref.pos_t.y)/wb->spt;
        dx(idx++) = (dend_cur.pos_t.z - dend_ref.pos_t.z)/wb->spt;

        quat_t qdiff = dend_cur.pos_r*dend_ref.pos_r.Conjugated();
        real_t theta = qdiff.Theta();
        if(theta > pi)
            theta -= 2*pi;
        vec3_t w = theta*qdiff.Axis();
        dx(idx++) = w.x/wb->spr;
        dx(idx++) = w.y/wb->spr;
        dx(idx++) = w.z/wb->spr;

        dx(idx++) = (dend_cur.vel_t.x - dend_ref.vel_t.x)/wb->svt;
        dx(idx++) = (dend_cur.vel_t.y - dend_ref.vel_t.y)/wb->svt;
        dx(idx++) = (dend_cur.vel_t.z - dend_ref.vel_t.z)/wb->svt;

        dx(idx++) = (dend_cur.vel_r.x - dend_ref.vel_r.x)/wb->svr;
        dx(idx++) = (dend_cur.vel_r.y - dend_ref.vel_r.y)/wb->svr;
        dx(idx++) = (dend_cur.vel_r.z - dend_ref.vel_r.z)/wb->svr;

        dx(idx++) = (dend_cur.force_t.x - dend_ref.force_t.x)/wb->sft;
        dx(idx++) = (dend_cur.force_t.y - dend_ref.force_t.y)/wb->sft;
        dx(idx++) = (dend_cur.force_t.z - dend_ref.force_t.z)/wb->sft;

        dx(idx++) = (dend_cur.force_r.x - dend_ref.force_r.x)/wb->sfr;
        dx(idx++) = (dend_cur.force_r.y - dend_ref.force_r.y)/wb->sfr;
        dx(idx++) = (dend_cur.force_r.z - dend_ref.force_r.z)/wb->sfr;
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
    // base acc_r
    du(idx++) *= wb->sar;
    du(idx++) *= wb->sar;
    du(idx++) *= wb->sar;
    for(int i = 0; i < nend; i++){
        // end acc_t
        du(idx++) *= wb->sat;
        du(idx++) *= wb->sat;
        du(idx++) *= wb->sat;
        // end acc_r
        du(idx++) *= wb->sar;
        du(idx++) *= wb->sar;
        du(idx++) *= wb->sar;
        if(data_ref_des.ends[i].state != DiMP::Wholebody::ContactState::Free){
            // end forcerate_t
            du(idx++) *= wb->sfdt;
            du(idx++) *= wb->sfdt;
            du(idx++) *= wb->sfdt;
            // end forcerate_r
            du(idx++) *= wb->sfdr;
            du(idx++) *= wb->sfdr;
            du(idx++) *= wb->sfdr;
        }
    }

    vec_copy(uref, u);
    vec_add (du, u);

    // decode u
    idx = 0;
    data_cur.base.acc_r.x = u(idx++);
    data_cur.base.acc_r.y = u(idx++);
    data_cur.base.acc_r.z = u(idx++);

    for(int i = 0; i < nend; i++){
        //DiMP::WholebodyData::End&  dend_ref = data_ref.ends[i];
        DiMP::WholebodyData::End&  dend_ref_des = data_ref_des.ends[i];
        DiMP::WholebodyData::End&  dend_cur = data_cur.ends[i];
        
        dend_cur.acc_t.x = u(idx++);
        dend_cur.acc_t.y = u(idx++);
        dend_cur.acc_t.z = u(idx++);
        dend_cur.acc_r.x = u(idx++);
        dend_cur.acc_r.y = u(idx++);
        dend_cur.acc_r.z = u(idx++);
        if(data_ref_des.ends[i].state == DiMP::Wholebody::ContactState::Free){
            dend_cur.force_t.clear();
            dend_cur.force_r.clear();
        }
        else{
            dend_cur.forcerate_t.x = u(idx++);
            dend_cur.forcerate_t.y = u(idx++);
            dend_cur.forcerate_t.z = u(idx++);
            dend_cur.forcerate_r.x = u(idx++);
            dend_cur.forcerate_r.y = u(idx++);
            dend_cur.forcerate_r.z = u(idx++);

            // enforce contact force constraint
            //dend_cur.force_t.z = std::max(0.0, dend_cur.force_t.z);
            //dend_cur.force_t.x = std::min(std::max(-dend_ref_des.mu*dend_cur.force_t.z, dend_cur.force_t.x), dend_ref_des.mu*dend_cur.force_t.z);
            //dend_cur.force_t.y = std::min(std::max(-dend_ref_des.mu*dend_cur.force_t.z, dend_cur.force_t.y), dend_ref_des.mu*dend_cur.force_t.z);
            //dend_cur.force_r.x = std::min(std::max( dend_ref_des.cop_min.y*dend_cur.force_t.z, dend_cur.force_r.x),  dend_ref_des.cop_max.y*dend_cur.force_t.z);
            //dend_cur.force_r.y = std::min(std::max(-dend_ref_des.cop_max.x*dend_cur.force_t.z, dend_cur.force_r.y), -dend_ref_des.cop_min.x*dend_cur.force_t.z);
        }
    }

    //for(int j = 0; j < u.n; j++)
    //    DSTR << u(j) << " ";
    //DSTR << endl;

}

void Mpc::UpdateGain(){
    // copy data
    DiMP::WholebodyKey* key1 = (DiMP::WholebodyKey*)wb->traj.GetKeypoint(wb->graph->ticks[1]);
    data_ref     = key1->data;
    data_ref_des = key1->data_des;

    // calc input dimension
    int nu = 3;  //< base acc_r
    int nend = wb->ends.size();
    for(int i = 0; i < nend; i++){
        // end acc_t acc_r (+ force_t force_r if in contact)
        nu += (data_ref_des.ends[i].state == DiMP::Wholebody::ContactState::Free ?  6 : 12);
    }

    // calc uref
    uref.Resize(nu);
    int idx = 0;
    uref(idx++) = data_ref.base.acc_r.x;
    uref(idx++) = data_ref.base.acc_r.y;
    uref(idx++) = data_ref.base.acc_r.z;

    for(int i = 0; i < nend; i++){
        DiMP::WholebodyData::End&  dend_ref = data_ref.ends[i];

        uref(idx++) = dend_ref.acc_t.x;
        uref(idx++) = dend_ref.acc_t.y;
        uref(idx++) = dend_ref.acc_t.z;
        uref(idx++) = dend_ref.acc_r.x;
        uref(idx++) = dend_ref.acc_r.y;
        uref(idx++) = dend_ref.acc_r.z;
        if(data_ref_des.ends[i].state != DiMP::Wholebody::ContactState::Free){
            uref(idx++) = dend_ref.forcerate_t.x;
            uref(idx++) = dend_ref.forcerate_t.y;
            uref(idx++) = dend_ref.forcerate_t.z;
            uref(idx++) = dend_ref.forcerate_r.x;
            uref(idx++) = dend_ref.forcerate_r.y;
            uref(idx++) = dend_ref.forcerate_r.z;
        }
    }

    //for(int j = 0; j < uref.n; j++)
    //    DSTR << uref(j) << " ";
    //DSTR << endl;

    // store matrices
    Quuinv_Qux.Resize(nu, Quuinv_Qux.n);
    mat_copy(wb->graph->solver->Quuinv_Qux[1], Quuinv_Qux);

    ready = true;
}

void Mpc::Countup(){
    count++;
    time += dt/updateCycle;
}

void Mpc::CalcIK(int ichain, const vec3_t& pe_local, const quat_t& qe_local, vvec_t& joint, vvec_t& error, vmat_t& Jq, vmat_t& Je, bool calc_jacobian){
    if(ichain == MyIK::Chain::Torso) myik->CalcTorsoIK(pe_local, qe_local, joint, error, Jq, Je, calc_jacobian);
    if(ichain == MyIK::Chain::ArmR ) myik->CalcArmIK  (pe_local, qe_local, joint, error, Jq, Je, calc_jacobian, 0);
    if(ichain == MyIK::Chain::ArmL ) myik->CalcArmIK  (pe_local, qe_local, joint, error, Jq, Je, calc_jacobian, 1);
    if(ichain == MyIK::Chain::LegR ){
        myik->CalcLegIK2 (pe_local, qe_local, joint, error, Jq, Je, calc_jacobian, 0);
        /*
        vvec_t _joint[2];
        vvec_t _error[2];
        vmat_t _Jq[2];
        vmat_t _Je[2];
        //myik->CalcLegIK  (pe_local, qe_local, joint, error, Jq, Je, calc_jacobian, 0);
        myik->CalcLegIK  (pe_local, qe_local, _joint[0], _error[0], _Jq[0], _Je[0], true, 0);
        myik->CalcLegIK2 (pe_local, qe_local, _joint[1], _error[1], _Jq[1], _Je[1], true, 0);
        DSTR << _joint[0] << endl;
        DSTR << _joint[1] << endl;
        DSTR << _error[0] << endl;
        DSTR << _error[1] << endl;
        DSTR << _Jq[0] << endl;
        DSTR << _Jq[1] << endl;
        DSTR << _Je[0] << endl;
        DSTR << _Je[1] << endl;
        int hoge = 0;
        */
    }
    if(ichain == MyIK::Chain::LegL ) myik->CalcLegIK2  (pe_local, qe_local, joint, error, Jq, Je, calc_jacobian, 1);
}

void Mpc::GetInitialState(DiMP::WholebodyData& d){
    d.centroid.pos_t = data_cur.centroid.pos_t;
    d.centroid.vel_t = data_cur.centroid.vel_t;
    d.centroid.pos_r = data_cur.centroid.pos_r;
    d.centroid.vel_r = data_cur.centroid.vel_r;

    d.base.pos_r = data_cur.base.pos_r;
    d.base.vel_r = data_cur.base.vel_r;
    d.base.acc_r = data_cur.base.acc_r;

    d.centroid.pos_t_weight = (100.0)*one;
    d.centroid.vel_t_weight = (100.0)*one;
    d.centroid.pos_r_weight = (100.0)*one;
    d.centroid.vel_r_weight = (100.0)*one;

    d.base.pos_r_weight = (100.0)*one;
    d.base.vel_r_weight = (100.0)*one;
    d.base.acc_r_weight = (100.0)*one;

    for(int i = 0; i < MyIK::End::Num; i++){
        DiMP::WholebodyData::End&  dend     = d.ends[i];
        DiMP::WholebodyData::End&  dend_cur = data_cur.ends[i];

        dend.pos_t   = dend_cur.pos_t;
        dend.pos_r   = dend_cur.pos_r;
        dend.vel_t   = dend_cur.vel_t;
        dend.vel_r   = dend_cur.vel_r;
        dend.acc_t   = dend_cur.acc_t;
        dend.acc_r   = dend_cur.acc_r;
        dend.force_t = dend_cur.force_t;
        dend.force_r = dend_cur.force_r;
    
        dend.pos_t_weight   = 100.0*one;
        dend.pos_r_weight   = 100.0*one;
        dend.vel_t_weight   = 100.0*one;
        dend.vel_r_weight   = 100.0*one;
        dend.acc_t_weight   = 100.0*one;
        dend.acc_r_weight   = 100.0*one;
        dend.force_t_weight = 100.0*one;
        dend.force_r_weight = 100.0*one;
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
    
    d.base.pos_r = quat_t();
    d.base.vel_r = vec3_t();

    d.centroid.pos_t_weight = (k == N ? 100.0 : 1.0)*one;
    d.centroid.vel_t_weight = (k == N ? 100.0 : 1.0)*one;
    d.centroid.pos_r_weight = (k == N ? 100.0 : 1.0)*one;
    d.centroid.vel_r_weight = (k == N ? 100.0 : 1.0)*one;
    
    d.base.pos_r_weight = (k == N ? 100.0 : 1.0)*one;
    d.base.vel_r_weight = (k == N ? 100.0 : 1.0)*one;

    for(int i = 0; i < MyIK::End::Num; i++){
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
                dend.cop_min = vec2_t(-0.1, -0.05);
                dend.cop_max = vec2_t( 0.1,  0.05);
                dend.pos_te  = vec3_t(0.0, 0.0, 0.0);
                dend.pos_tc  = pe;
                dend.pos_rc  = quat_t();
            }
            if(stat == DiMP::BipedLIP::ContactState::Heel){
                dend.state   = DiMP::Wholebody::ContactState::Line;
                dend.mu      = 0.2;
                dend.cop_min = vec2_t(-0.1, -0.05);
                dend.cop_max = vec2_t( 0.1,  0.05);
                dend.pos_te  = vec3_t(0.1, 0.0, 0.0);
                dend.pos_tc  = pe + qe*dend.pos_te;
                dend.pos_rc  = quat_t();
            }
            if(stat == DiMP::BipedLIP::ContactState::Toe ){
                dend.state   = DiMP::Wholebody::ContactState::Line;
                dend.mu      = 0.2;
                dend.cop_min = vec2_t(-0.1, -0.05);
                dend.cop_max = vec2_t( 0.1,  0.05);
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

        dend.pos_t_weight   = one*(k == N ? 100.0 : 1.0)*(dend.state == DiMP::Wholebody::ContactState::Free ? 0.1 : 1.0);
        dend.pos_r_weight   = one*(k == N ? 100.0 : 1.0)*(dend.state == DiMP::Wholebody::ContactState::Free ? 0.1 : 1.0);
        dend.vel_t_weight   = one*(k == N ? 100.0 : 1.0)*(dend.state == DiMP::Wholebody::ContactState::Free ? 0.1 : 1.0);
        dend.vel_r_weight   = one*(k == N ? 100.0 : 1.0)*(dend.state == DiMP::Wholebody::ContactState::Free ? 0.1 : 1.0);
        dend.acc_t_weight   = one;
        dend.acc_r_weight   = one;
        dend.force_t_weight = one;
        dend.force_r_weight = one;
    }
}	
