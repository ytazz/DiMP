#include "mpc.h"

#include <sbtimer.h>
static Timer timer;

Mpc::Mpc(){
    handOffset[0]  = vec3_t(0.0, -0.20, -0.1);
    handOffset[1]  = vec3_t(0.0,  0.20, -0.1);
}

void Mpc::Init(){
    data_cur.Init(wb);
    data_ref.Init(wb);

    // load reference trajectory
    csv.Read("log.csv", ", ", true);
	
    count   = 0;
    time    = 0.0;
	timeMpc = 0.0;

    // initialize state using desired state
    Setup(time, data_cur);

    int nend = wb->ends.size();

    for(int i = 0; i < nend; i++){
        DiMP::WholebodyData::End&  dend = data_cur.ends[i];
        DiMP::WholebodyData::Link& dlnk = data_cur.links[wb->ends[i].ilink];

        dlnk.pos_t   = dend.pos_t_des;
        dlnk.pos_r   = dend.pos_r_des;
        dlnk.vel_t   = dend.vel_t_des;
        dlnk.vel_r   = dend.vel_r_des;
        //dlnk.force_t = dend.force_t_des;
        //dlnk.force_r = dend.force_r_des;
    }

    int nx = 12*nend + 9;
    int nu =  6*nend;
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
    
    real_t dt_ctrl  = dt/updateCycle;
    real_t dt_ctrl2 = dt_ctrl*dt_ctrl;

    for(int i = 0; i < nend; i++){
        DiMP::WholebodyData::Link& dlnk_cur = data_cur.links[wb->ends[i].ilink];

        dlnk_cur.pos_t += dlnk_cur.vel_t*dt_ctrl + dlnk_cur.acc_t*(0.5*dt_ctrl2);
        dlnk_cur.pos_r = quat_t::Rot(dlnk_cur.vel_r*dt_ctrl + dlnk_cur.acc_r*(0.5*dt_ctrl2))*dlnk_cur.pos_r;

        dlnk_cur.vel_t += dlnk_cur.acc_t*dt_ctrl;
        dlnk_cur.vel_r += dlnk_cur.acc_r*dt_ctrl;

        //DSTR << dlnk_cur.acc_t << endl;
    }

}

void Mpc::UpdateInput(){
    if(!ready)
        return;

    int nend = wb->ends.size();
    int idx;

    // calc dx
    idx = 0;
    vec_clear(dx);
    for(int i = 0; i < nend; i++){
        DiMP::WholebodyData::End&  dend_ref = data_ref.ends[i];
        DiMP::WholebodyData::Link& dlnk_cur = data_cur.links[wb->ends[i].ilink];
        DiMP::WholebodyData::Link& dlnk_ref = data_ref.links[wb->ends[i].ilink];

        dx(idx++) = (dlnk_cur.pos_t.x - dlnk_ref.pos_t.x)/wb->spt;
        dx(idx++) = (dlnk_cur.pos_t.y - dlnk_ref.pos_t.y)/wb->spt;
        dx(idx++) = (dlnk_cur.pos_t.z - dlnk_ref.pos_t.z)/wb->spt;

        quat_t qdiff = dlnk_cur.pos_r*dlnk_ref.pos_r.Conjugated();
        vec3_t w = qdiff.Theta()*qdiff.Axis();
        dx(idx++) = w.x/wb->spr;
        dx(idx++) = w.y/wb->spr;
        dx(idx++) = w.z/wb->spr;

        dx(idx++) = (dlnk_cur.vel_t.x - dlnk_ref.vel_t.x)/wb->svt;
        dx(idx++) = (dlnk_cur.vel_t.y - dlnk_ref.vel_t.y)/wb->svt;
        dx(idx++) = (dlnk_cur.vel_t.z - dlnk_ref.vel_t.z)/wb->svt;

        dx(idx++) = (dlnk_cur.vel_r.x - dlnk_ref.vel_r.x)/wb->svr;
        dx(idx++) = (dlnk_cur.vel_r.y - dlnk_ref.vel_r.y)/wb->svr;
        dx(idx++) = (dlnk_cur.vel_r.z - dlnk_ref.vel_r.z)/wb->svr;
    }

    //dx(idx++) = (data_cur.com_pos.x - data_ref.com_pos.x)/wb->spt;
    //dx(idx++) = (data_cur.com_pos.y - data_ref.com_pos.y)/wb->spt;
    //dx(idx++) = (data_cur.com_pos.z - data_ref.com_pos.z)/wb->spt;
    //
    //dx(idx++) = (data_cur.com_vel.x - data_ref.com_vel.x)/wb->svt;
    //dx(idx++) = (data_cur.com_vel.y - data_ref.com_vel.y)/wb->svt;
    //dx(idx++) = (data_cur.com_vel.z - data_ref.com_vel.z)/wb->svt;
    //
    //dx(idx++) = (data_cur.mom.x - data_ref.mom.x)/wb->sL;
    //dx(idx++) = (data_cur.mom.y - data_ref.mom.y)/wb->sL;
    //dx(idx++) = (data_cur.mom.z - data_ref.mom.z)/wb->sL;

    // calc u
    // du = -Quu^-1 * (Qu + Qux * dx)
    //  * Qu shall be zero after convergence
    // K = -Quu^-1 * Qux
    // u = uref - Quuinv_Qux*dx;
    mat_vec_mul(Quuinv_Qux, dx, du, -1.0, 0.0);

    idx = 0;
    for(int i = 0; i < nend; i++){
        DiMP::WholebodyData::End&  dend_ref = data_ref.ends[i];

        du(idx++) *= (dend_ref.v_or_f[0] ? wb->sat : wb->sft);
        du(idx++) *= (dend_ref.v_or_f[1] ? wb->sat : wb->sft);
        du(idx++) *= (dend_ref.v_or_f[2] ? wb->sat : wb->sft);
        du(idx++) *= (dend_ref.v_or_f[3] ? wb->sar : wb->sfr);
        du(idx++) *= (dend_ref.v_or_f[4] ? wb->sar : wb->sfr);
        du(idx++) *= (dend_ref.v_or_f[5] ? wb->sar : wb->sfr);
    }

    vec_copy(uref, u);
    vec_add (du, u);

    // decode u
    idx = 0;
    for(int i = 0; i < nend; i++){
        DiMP::WholebodyData::End&  dend_ref = data_ref.ends[i];
        DiMP::WholebodyData::End&  dend_cur = data_cur.ends[i];
        DiMP::WholebodyData::Link& dlnk_ref = data_ref.links[wb->ends[i].ilink];
        DiMP::WholebodyData::Link& dlnk_cur = data_cur.links[wb->ends[i].ilink];
        
        dend_cur.acc_tc  .clear();
        dend_cur.acc_rc  .clear();
        dend_cur.force_tc.clear();
        dend_cur.force_rc.clear();

        (dend_ref.v_or_f[0] ? dend_cur.acc_tc.x : dend_cur.force_tc.x) = u(idx++);
        (dend_ref.v_or_f[1] ? dend_cur.acc_tc.y : dend_cur.force_tc.y) = u(idx++);
        (dend_ref.v_or_f[2] ? dend_cur.acc_tc.z : dend_cur.force_tc.z) = u(idx++);
        (dend_ref.v_or_f[3] ? dend_cur.acc_rc.x : dend_cur.force_rc.x) = u(idx++);
        (dend_ref.v_or_f[4] ? dend_cur.acc_rc.y : dend_cur.force_rc.y) = u(idx++);
        (dend_ref.v_or_f[5] ? dend_cur.acc_rc.z : dend_cur.force_rc.z) = u(idx++);

        dlnk_cur.acc_r = dend_ref.pos_rc*dend_cur.acc_rc;
		dlnk_cur.acc_t = dend_ref.pos_rc*dend_cur.acc_tc - dlnk_cur.acc_r % dend_ref.rlc - dlnk_cur.vel_r % (dlnk_cur.vel_r % dend_ref.rlc);

		dlnk_cur.force_t = dend_ref.pos_rc*dend_cur.force_tc;
		dlnk_cur.force_r = dend_ref.pos_rc*dend_cur.force_rc + dend_ref.rlc % dend_cur.force_tc;

    }
}

void Mpc::UpdateGain(){
    // copy data
    DiMP::WholebodyKey* key1 = (DiMP::WholebodyKey*)wb->traj.GetKeypoint(wb->graph->ticks[1]);
    data_ref = key1->data;

    // calc uref
    int nend = wb->ends.size();
    int idx = 0;
    for(int i = 0; i < nend; i++){
        DiMP::WholebodyData::End&  dend_ref = data_ref.ends[i];
        //DiMP::WholebodyData::Link& dlnk_ref = data_ref.links[wb->ends[i].ilink];

        uref(idx++) = (dend_ref.v_or_f[0] ? dend_ref.acc_tc.x : dend_ref.force_tc.x);
        uref(idx++) = (dend_ref.v_or_f[1] ? dend_ref.acc_tc.y : dend_ref.force_tc.y);
        uref(idx++) = (dend_ref.v_or_f[2] ? dend_ref.acc_tc.z : dend_ref.force_tc.z);
        uref(idx++) = (dend_ref.v_or_f[3] ? dend_ref.acc_rc.x : dend_ref.force_rc.x);
        uref(idx++) = (dend_ref.v_or_f[4] ? dend_ref.acc_rc.y : dend_ref.force_rc.y);
        uref(idx++) = (dend_ref.v_or_f[5] ? dend_ref.acc_rc.z : dend_ref.force_rc.z);
    }

    // store matrices
    mat_copy(wb->graph->solver->Quuinv_Qux[1], Quuinv_Qux);

    ready = true;
}

void Mpc::Countup(){
    count++;
    time += dt/updateCycle;
}

void Mpc::CalcIK(DiMP::WholebodyData& data){
    vec3_t pe_local;
    quat_t qe_local;
    vvec_t joint;
    vvec_t error;
    vector<vec3_t> pi_local;
    vector<quat_t> qi_local;

    timer.CountUS();
    for(int i = 0; i < wb->chains.size(); i++){
        DiMP::Wholebody::Chain& ch = wb->chains[i];

        int ibase = wb->ends[ch.ibase].ilink;
        int iend  = wb->ends[ch.iend ].ilink;

        vec3_t pb = data.links[ibase].pos_t;
        quat_t qb = data.links[ibase].pos_r;
        vec3_t pe = data.links[iend ].pos_t;
        quat_t qe = data.links[iend ].pos_r;

        pe_local = qb.Conjugated()*(pe - pb);
        qe_local = qb.Conjugated()*qe;

        if(i == MyIK::Chain::Torso){
            myik->CalcTorsoIK(pe_local, qe_local, joint, error);
            myik->CalcTorsoFK(joint, pi_local, qi_local);
        }
        if(i == MyIK::Chain::ArmR){
            myik->CalcArmIK(pe_local, qe_local, joint, error, 0);
            myik->CalcArmFK(joint, pi_local, qi_local, 0);
        }
        if(i == MyIK::Chain::ArmL){
            myik->CalcArmIK(pe_local, qe_local, joint, error, 1);
            myik->CalcArmFK(joint, pi_local, qi_local, 1);
        }
        if(i == MyIK::Chain::LegR){
            myik->CalcLegIK(pe_local, qe_local, joint, error, 0);
            myik->CalcLegFK(joint, pi_local, qi_local, 0);
        }
        if(i == MyIK::Chain::LegL){
            myik->CalcLegIK(pe_local, qe_local, joint, error, 1);
            myik->CalcLegFK(joint, pi_local, qi_local, 1);
        }

        for(int j = 0; j < ch.ilink.size(); j++){
            int i = ch.ilink[j];
            if(!wb->links[i].is_end){
                data.links[i].pos_t = pb + qb*pi_local[j];
                data.links[i].pos_r = qb*qi_local[j];
            }
            if(wb->links[i].ijoint != -1){
                data.q[wb->links[i].ijoint] = joint[j];
            }
        }

        for(int j = 0; j < ch.ilimit.size(); j++){
            data.e[ch.ilimit[j]] = error[j];
        }
    }

    //int tik = timer.CountUS();
    //DSTR << "tik: " << tik << endl;
}

void Mpc::Setup(real_t t, DiMP::WholebodyData& _data){
    const real_t dt = 0.001;
    int r = (t + timeMpc + 2)/dt;
    int c = 1;
    _data.com_pos_des.x = csv.Get<real_t>(r, c++);
    _data.com_pos_des.y = csv.Get<real_t>(r, c++);
    _data.com_pos_des.z = csv.Get<real_t>(r, c++);
    
    c = 1+3;
    _data.com_vel_des.x = csv.Get<real_t>(r, c++);
    _data.com_vel_des.y = csv.Get<real_t>(r, c++);
    _data.com_vel_des.z = csv.Get<real_t>(r, c++);

    _data.mom_des = vec3_t();

    for(int i = 0; i < MyIK::End::Num; i++){
        DiMP::WholebodyData::End&  dend     = _data.ends[i];
        DiMP::WholebodyData::End&  dend_cur = data_cur.ends [i];
        DiMP::WholebodyData::Link& dlnk_cur = data_cur.links[wb->ends[i].ilink];
        
        // set current state as initial state of MPC
        dend.pos_t_ini = dlnk_cur.pos_t;
        dend.pos_r_ini = dlnk_cur.pos_r;
        dend.vel_t_ini = dlnk_cur.vel_t;
        dend.vel_r_ini = dlnk_cur.vel_r;

        // set current input as initial input
        dend.acc_tc_ini   = dend_cur.acc_tc;
        dend.acc_rc_ini   = dend_cur.acc_rc;
        dend.force_tc_ini = dend_cur.force_tc;
        dend.force_rc_ini = dend_cur.force_rc;

        vec3_t atd, ard, ftd, frd;

        if(i == MyIK::End::Hips){
            dend.pos_t_des = _data.com_pos_des;
            dend.pos_r_des = quat_t();
            dend.vel_t_des = _data.com_vel_des;
            dend.vel_r_des = vec3_t();
            dend.state     = DiMP::Wholebody::ContactState::Free;
            dend.pos_tc    = dend.pos_t_des;
        }
        if(i == MyIK::End::ChestP){
            dend.pos_t_des = _data.com_pos_des + vec3_t(0.0, 0.0, myik->torsoLength);
            dend.pos_r_des = quat_t();
            dend.vel_t_des = _data.com_vel_des;
            dend.vel_r_des = vec3_t();
            dend.state     = DiMP::Wholebody::ContactState::Free;
            dend.pos_tc    = dend.pos_t_des;
        }
        if(i == MyIK::End::HandR){
            dend.pos_t_des = _data.com_pos_des + handOffset[0];
            dend.pos_r_des = quat_t();
            dend.vel_t_des = _data.com_vel_des;
            dend.vel_r_des = vec3_t();
            dend.state     = DiMP::Wholebody::ContactState::Free;
            dend.pos_tc    = dend.pos_t_des;
        }
        if(i == MyIK::End::HandL){
            dend.pos_t_des = _data.com_pos_des + handOffset[1];
            dend.pos_r_des = quat_t();
            dend.vel_t_des = _data.com_vel_des;
            dend.vel_r_des = vec3_t();
            dend.state     = DiMP::Wholebody::ContactState::Free;
            dend.pos_tc    = dend.pos_t_des;
        }
        if(i == MyIK::End::FootR || i == MyIK::End::FootL){
            c = (i == MyIK::End::FootR ? 1+3+3 : 1+3+3+7+6+6+1);
            dend.pos_t_des.x   = csv.Get<real_t>(r, c++);
            dend.pos_t_des.y   = csv.Get<real_t>(r, c++);
            dend.pos_t_des.z   = csv.Get<real_t>(r, c++);
            dend.pos_r_des.w   = csv.Get<real_t>(r, c++);
            dend.pos_r_des.x   = csv.Get<real_t>(r, c++);
            dend.pos_r_des.y   = csv.Get<real_t>(r, c++);
            dend.pos_r_des.z   = csv.Get<real_t>(r, c++);
            dend.vel_t_des.x   = csv.Get<real_t>(r, c++);
            dend.vel_t_des.y   = csv.Get<real_t>(r, c++);
            dend.vel_t_des.z   = csv.Get<real_t>(r, c++);
            dend.vel_r_des.x   = csv.Get<real_t>(r, c++);
            dend.vel_r_des.y   = csv.Get<real_t>(r, c++);
            dend.vel_r_des.z   = csv.Get<real_t>(r, c++);
            ftd.x = csv.Get<real_t>(r, c++);
            ftd.y = csv.Get<real_t>(r, c++);
            ftd.z = csv.Get<real_t>(r, c++);
            frd.x = csv.Get<real_t>(r, c++);
            frd.y = csv.Get<real_t>(r, c++);
            frd.z = csv.Get<real_t>(r, c++);

            int stat = csv.Get<int>(r, c++);

            if(stat == DiMP::BipedLIP::ContactState::Float){
                dend.state  = DiMP::Wholebody::ContactState::Free;
                dend.pos_tc   = dend.pos_t_des;
            }
            if(stat == DiMP::BipedLIP::ContactState::Surface){
                dend.state    = DiMP::Wholebody::ContactState::Surface;
                dend.pos_tc   = dend.pos_t_des;
		        dend.pos_tc.z = 0.0;
                dend.pos_rc   = quat_t();
                dend.cop_min  = vec2_t(-0.1, -0.1);
                dend.cop_max  = vec2_t( 0.1,  0.1);
            }
            if( stat == DiMP::BipedLIP::ContactState::Heel ||
                stat == DiMP::BipedLIP::ContactState::Toe ){
                dend.state   = DiMP::Wholebody::ContactState::Line;
                dend.mu      = 0.2;
                dend.cop_min = vec2_t(-0.1, -0.1);
                dend.cop_max = vec2_t( 0.1,  0.1);

                vec3_t c = (stat == DiMP::BipedLIP::ContactState::Heel ? vec3_t(-0.07, 0.0, 0.1) : vec3_t( 0.1, 0.0, 0.1));
                dend.pos_tc = dend.pos_t_des + dend.pos_r_des*c;
		        dend.pos_tc.z = 0.0;
            }
        }

        // ul = qc*uc
        // al = qc*ac - qc*uc % rlc - qc*wc % (qc*vc - vl)
		//    = qc*ac - ul % rlc - wl % (wl % rlc)

        // uc = qc^-1* ul
        // ac = qc^-1*(al + ul % rlc + wl % (wl % rlc))

        vec3_t rlc = dend.pos_tc - dend.pos_t_des;
        vec3_t wl  = dend.vel_r_des;
        dend.acc_tc_des   = dend.pos_rc.Conjugated()*(atd + ard % rlc + wl % (wl % rlc));
	    dend.acc_rc_des   = dend.pos_rc.Conjugated()* ard;
        dend.force_tc_des = dend.pos_rc.Conjugated()* ftd;
	    dend.force_rc_des = dend.pos_rc.Conjugated()*(frd - rlc % ftd);
    }
}	
