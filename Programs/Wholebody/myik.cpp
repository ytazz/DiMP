#include "myik.h"

#include <sbrollpitchyaw.h>

const real_t pi = 3.1415926535;
const real_t eps = 0.01;

using namespace PTM;

inline mat3_t JacobianRPY(vec3_t angle){
    mat3_t Jrpy;
    Jrpy[0][0] =  cos(angle[2])*cos(angle[1]); Jrpy[0][1] = -sin(angle[2]); Jrpy[0][2] = 0.0;
    Jrpy[1][0] =  sin(angle[2])*cos(angle[1]); Jrpy[1][1] =  cos(angle[2]); Jrpy[1][2] = 0.0;
    Jrpy[2][0] = -sin(angle[1]);               Jrpy[2][1] =  0.0;           Jrpy[2][2] = 1.0;
    return Jrpy;
}

inline mat3_t vvmat(vec3_t c, vec3_t r){
    mat3_t m;
    for(int i = 0; i < 3; i++)for(int j = 0; j < 3; j++)
        m[i][j] = c[i]*r[j];

    return m;
}

MyIK::MyIK(){
    armBase[0]     = vec3_t(0.0, -0.1 ,  0.1);
	armBase[1]     = vec3_t(0.0,  0.1 ,  0.1);
	legBase[0]     = vec3_t(0.0, -0.1 , -0.1);
	legBase[1]     = vec3_t(0.0,  0.1 , -0.1);
	wristToHand[0]     = vec3_t(0.0,  0.0 ,  0.0);
	wristToHand[1]     = vec3_t(0.0,  0.0 ,  0.0);
	ankleToFoot[0]     = vec3_t(0.0,  0.0 , -0.0);
	ankleToFoot[1]     = vec3_t(0.0,  0.0 , -0.0);
    elbowYaw[0]    = 0.0;
	elbowYaw[1]    = 0.0;
	torsoLength    = 0.2;
	upperArmLength = 0.2;
	lowerArmLength = 0.2;
	upperLegLength = 0.35;
	lowerLegLength = 0.35;
}

void MyIK::CalcTorsoIK(const vec3_t& pos, const quat_t& ori, vvec_t& joint, vvec_t& error, vmat_t& Jq, vmat_t& Je, bool calc_jacobian){
    // hip, chest_p, chest_y
    joint.resize(4);
    error.resize(4);

    vec3_t theta = ToRollPitchYaw(ori);
    joint[0] = theta.z;
    joint[1] = theta.y;
    joint[2] = 0.0;
    joint[3] = 0.0;

    //vec3_t offset(0.0, 0.0, torsoLength);
    vec3_t offset(0.0, 0.0, 0.0);
    error[0] = pos.x - offset.x;
    error[1] = pos.y - offset.y;
    error[2] = pos.z - offset.z;
    error[3] = theta.x;

    Jq.resize(4, 6, 0.0);
    Je.resize(4, 6, 0.0);

    Jq[0][5] = 1.0;
    Jq[1][4] = 1.0;
    Je[0][0] = 1.0;
    Je[1][1] = 1.0;
    Je[2][2] = 1.0;
    Je[3][3] = 1.0;
}

void MyIK::CalcArmIK(const vec3_t& pos, const quat_t& ori, vvec_t& joint, vvec_t& error, vmat_t& Jq, vmat_t& Je, bool calc_jacobian, int side){
    real_t l1 = upperArmLength;
    real_t l2 = lowerArmLength;

    // shoulder, elbow, wrist
    joint.resize(7);
    error.resize(2);

    // elbow pitch from trigonometrics
    real_t d = pos.norm();
    real_t c = (l1*l1 + l2*l2 - d*d)/(2*l1*l2);
    real_t beta;
    
    //  singularity: too close
    if(c > 1.0){
        beta     = 0.0;
        error[0] = 1.0 - c;
        error[1] = 0.0;
    }
    //  singularity: too far
    else if(c < -1.0){
        beta     = pi;
        error[0] = 0.0;
        error[1] = c + 1.0;
    }
    //  nonsingular
    else{
        beta = acos(c);
        error[0] = 0.0;
        error[1] = 0.0;
    }

    joint[3] = -(pi - beta);

    // shoulder yaw is given
    joint[2] = elbowYaw[side];

    // wrist pos expressed in shoulder pitch-roll local
    vec3_t pos_local = quat_t::Rot(joint[2], 'z')*vec3_t(-l2*sin(joint[3]), 0.0, -l1 - l2*cos(joint[3]));

    // shoulder roll
    real_t c2 = pos.y/sqrt(pos.y*pos.y + pos.z*pos.z);
    real_t gamma;
    if(c2 > 1.0){
        gamma = 0.0;
    }
    else if(c2 < -1.0){
        gamma = pi;
    }
    else{
        gamma = acos(c2);
    }

    real_t alpha = atan2(pos_local.z, pos_local.y);
    joint[1] = -alpha - gamma;
    
    // wrist pos expressed in shoulder pitch local
    vec3_t pos_local2 = quat_t::Rot(joint[1], 'x')*pos_local;

    // shoulder pitch
    joint[0] = atan2(pos.x, pos.z)
             - atan2(pos_local2.x, pos_local2.z);
    if(joint[0] >  pi) joint[0] -= 2.0*pi;
    if(joint[0] < -pi) joint[0] += 2.0*pi;

    // desired hand orientation
    quat_t qy    = quat_t::Rot(joint[3], 'y');
    quat_t qzy   = quat_t::Rot(joint[2], 'z')*qy;
    quat_t qxzy  = quat_t::Rot(joint[1], 'x')*qzy;
    quat_t qyxzy = quat_t::Rot(joint[0], 'y')*qxzy;
    quat_t qhand = qyxzy.Conjugated()*ori;

    // convert it to roll-pitch-yaw
    vec3_t angle_hand = ToRollPitchYaw(qhand);

    // then wrist angles are determined
    joint[4] = angle_hand.z;
    joint[5] = angle_hand.y;
    joint[6] = angle_hand.x;

    if(calc_jacobian){
        const vec3_t ex(1.0, 0.0, 0.0);
        const vec3_t ey(0.0, 1.0, 0.0);
        const vec3_t ez(0.0, 0.0, 1.0);
    
        vec3_t J_d_p        = pos/pos.norm();
        real_t J_c_d        = -d/(l1*l2);
        vec3_t J_c_p        = J_c_d*J_d_p;
        real_t J_beta_c     = (-1.0 + eps < c && c < 1.0 - eps) ? -1.0/sqrt(1.0 - c*c) : 0.0;
        real_t J_beta_d     = J_beta_c*J_c_d;
        vec3_t J_beta_p     = J_beta_d*J_d_p;
    
        Jq.row(2).clear();
        Jq.row(3).v_range(0,3) = J_beta_p;
        Jq.row(3).v_range(3,3).clear();

        vec3_t tmp          = quat_t::Rot(joint[2], 'z')*vec3_t(-l2*cos(joint[3]), 0.0, l2*sin(joint[3]));
        mat3_t J_pd_p       = vvmat(tmp, Jq.row(3).v_range(0,3));
        //mat3_t J_pd_q       = vvmat(tmp, Jq.row(3).v_range(3,3));

        real_t tmp2         = (pos_local.y*pos_local.y + pos_local.z*pos_local.z);
        real_t tmp3         = pow(tmp2, 1.5);
        vec3_t J_c2_p       = (tmp2 > eps) ? vec3_t(0.0,  1.0/sqrt(tmp2), 0.0) : vec3_t();
        vec3_t J_c2_pd      = (tmp3 > eps) ? vec3_t(0.0, -(pos.y*pos_local.y)/tmp3, -(pos.y*pos_local.z)/tmp3) : vec3_t();
        real_t J_gamma_c2   = (-1.0 + eps < c2 && c2 < 1.0 - eps) ? -1.0/sqrt(1.0 - c2*c2) : 0.0;
        vec3_t J_gamma_p    = J_gamma_c2*J_c2_p;
        vec3_t J_gamma_pd   = J_gamma_c2*J_c2_pd;
        vec3_t J_alpha_pd   = (tmp2 > eps) ? vec3_t(0.0, -pos_local.z/tmp2, pos_local.y/tmp2) : vec3_t();

        Jq.row(1).v_range(0,3) = -J_pd_p.trans()*(J_alpha_pd + J_gamma_pd) - J_gamma_p;
        Jq.row(1).v_range(3,3).clear();// = J_pd_q.trans()*(-J_alpha_pd - J_gamma_pd);

        mat3_t J_pdd_p     = (mat3_t::Rot(joint[1], 'x') + vvmat(ex%pos_local2, -J_alpha_pd))*J_pd_p + vvmat(ex%pos_local2, -J_gamma_p);
        //mat3_t J_pdd_p      = J_pdd_pd*J_pd_p;
        //mat3_t J_pdd_q      = J_pdd_pd*J_pd_q;

        real_t tmp4 = (pos.x*pos.x + pos.z*pos.z);
        real_t tmp5 = (pos_local2.x*pos_local2.x + pos_local2.z*pos_local2.z);
        vec3_t tmp6 = (tmp4 > eps) ? vec3_t(pos       .z/tmp4, 0.0, -pos       .x/tmp4) : vec3_t();
        vec3_t tmp7 = (tmp5 > eps) ? vec3_t(pos_local2.z/tmp5, 0.0, -pos_local2.x/tmp5) : vec3_t();
    
        Jq.row(0).v_range(0,3) = tmp6 - J_pdd_p.trans()*tmp7;
        Jq.row(0).v_range(3,3).clear();// =      - J_pdd_q.trans()*tmp7;

        mat3_t Jrpy = JacobianRPY(angle_hand);
        mat3_t Jrpy_inv = Jrpy.inv();
        mat3_t J_qd_p;
        mat3_t J_qd_q;

        J_qd_p = - vvmat(qyxzy.Conjugated()*ey, Jq.row(0).v_range(0,3))
                 - vvmat(qxzy .Conjugated()*ex, Jq.row(1).v_range(0,3))
                 - vvmat(qzy  .Conjugated()*ez, Jq.row(2).v_range(0,3))
                 - vvmat(qy   .Conjugated()*ey, Jq.row(3).v_range(0,3));

        mat3_t Ryxzy;
        qyxzy.ToMatrix(Ryxzy);
        J_qd_q = Ryxzy.trans()
                 - vvmat(qyxzy.Conjugated()*ey, Jq.row(0).v_range(3,3))
                 - vvmat(qxzy .Conjugated()*ex, Jq.row(1).v_range(3,3))
                 - vvmat(qzy  .Conjugated()*ez, Jq.row(2).v_range(3,3))
                 - vvmat(qy   .Conjugated()*ey, Jq.row(3).v_range(3,3));

        Jq.row(4).v_range(0,3) = J_qd_p.trans()*Jrpy_inv.row(2);
        Jq.row(4).v_range(3,3) = J_qd_q.trans()*Jrpy_inv.row(2);

        Jq.row(5).v_range(0,3) = J_qd_p.trans()*Jrpy_inv.row(1);
        Jq.row(5).v_range(3,3) = J_qd_q.trans()*Jrpy_inv.row(1);

        Jq.row(6).v_range(0,3) = J_qd_p.trans()*Jrpy_inv.row(0);
        Jq.row(6).v_range(3,3) = J_qd_q.trans()*Jrpy_inv.row(0);

        // consider offset
        //mat6_t X;
        //X.sub_matrix(TSubMatrixDim<0,0,3,3>()) = mat3_t();
        //X.sub_matrix(TSubMatrixDim<0,3,3,3>()) = mat3_t::Cross(ori*wristToHand[side]);
        //X.sub_matrix(TSubMatrixDim<3,0,3,3>()).clear();
        //X.sub_matrix(TSubMatrixDim<3,3,3,3>()) = mat3_t();
        //Jq = Jq*X;

        if(c > 1.0){
            Je.row(0).v_range(0,3) = -J_c_p;
            Je.row(0).v_range(3,3).clear();
            Je.row(1).clear();
        }
        else if(c < -1.0){
            Je.row(0).clear();
            Je.row(1).v_range(0,3) =  J_c_p;
            Je.row(1).v_range(3,3).clear();
        }
        else{
            Je.row(0).clear();
            Je.row(1).clear();
        }
    }
}

void MyIK::CalcLegIK2(const vec3_t& pos, const quat_t& ori, vvec_t& joint, vvec_t& error, vmat_t& Jq, vmat_t& Je, bool calc_jacobian, int side){
    real_t l1 = upperLegLength;
    real_t l2 = lowerLegLength;

    // hip, knee, ankle
    joint.resize(6);
    error.resize(2);
    Jq.resize(6, 6, 0.0);
    Je.resize(2, 6, 0.0);

    // hip pitch and knee pitch from trigonometrics
    real_t d = pos.norm();
    real_t c = (l1*l1 + l2*l2 - d*d)/(2*l1*l2);
    real_t beta;
    
    //  singularity: too close
    if(c > 1.0){
        beta     = 0.0;
        error[0] = 1.0 - c;
        error[1] = 0.0;
    }
    //  singularity: too far
    else if(c < -1.0){
        beta     = pi;
        error[0] = 0.0;
        error[1] = c + 1.0;
    }
    //  nonsingular
    else{
        beta  = acos(c);
        error[0] = 0.0;
        error[1] = 0.0;
    }

    joint[3] = pi - beta;

    quat_t qinv =   ori.Conjugated();
    vec3_t phat = -(qinv*pos);
    quat_t qhat =   qinv;

    // ankle pitch
    vec3_t phatd = vec3_t(-l1*sin(joint[3]), 0.0, l1*cos(joint[3]) + l2);
    real_t c2 = phat.x/sqrt(phatd.x*phatd.x + phatd.z*phatd.z);
    real_t gamma;
    if(c2 > 1.0){
        gamma = 0.0;
    }
    else if(c2 < -1.0){
        gamma = pi;
    }
    else{
        gamma = acos(c2);
    }

    real_t alpha = atan2(phatd.z, phatd.x);
    joint[4] = -alpha + gamma;
    
    // hip pos expressed in ankle pitch local
    vec3_t phatdd = quat_t::Rot(-joint[4], 'y')*phatd;

    // ankle roll
    joint[5] = -atan2(phat.z, phat.y)
              + atan2(phatdd.z, phatdd.y);
    if(joint[5] >  pi) joint[5] -= 2.0*pi;
    if(joint[5] < -pi) joint[5] += 2.0*pi;

    // desired hip rotation
    quat_t qyy    =     quat_t::Rot(joint[3] + joint[4], 'y');
    quat_t qyyx   = qyy*quat_t::Rot(joint[5], 'x');
    quat_t qhip   = ori*qyyx.Conjugated();
    quat_t qzquad = quat_t::Rot(pi/2.0, 'z');

    // convert it to roll-pitch-yaw
    vec3_t angle_hip = ToRollPitchYaw(qzquad*qhip*qzquad.Conjugated());

    // then wrist angles are determined
    joint[0] =  angle_hip.z;
    joint[1] =  angle_hip.y;
    joint[2] = -angle_hip.x;

    if(calc_jacobian){
        const vec3_t ex(1.0, 0.0, 0.0);
        const vec3_t ey(0.0, 1.0, 0.0);
        const vec3_t ez(0.0, 0.0, 1.0);

        mat3_t Rinv;
        qinv.ToMatrix(Rinv);
        mat3_t Rzquad;
        qzquad.ToMatrix(Rzquad);

        mat3_t J_phat_p = -Rinv;
        mat3_t J_phat_q = -Rinv*mat3_t::Cross(pos);
        mat3_t J_qhat_q = -Rinv;
    
        vec3_t J_d_p        = pos/pos.norm();
        real_t J_c_d        = -d/(l1*l2);
        vec3_t J_c_p        = J_c_d*J_d_p;
        real_t J_beta_c     = (-1.0 + eps < c && c < 1.0 - eps) ? -1.0/sqrt(1.0 - c*c) : 0.0;
        real_t J_beta_d     = J_beta_c*J_c_d;
        vec3_t J_beta_p     = J_beta_d*J_d_p;
    
        Jq.row(3).v_range(0,3) = -J_beta_p;
        Jq.row(3).v_range(3,3).clear();

        vec3_t tmp       = vec3_t(-l1*cos(joint[3]), 0.0, -l1*sin(joint[3]));
        mat3_t J_phatd_p = vvmat(tmp, Jq.row(3).v_range(0,3));

        real_t tmp2          = (phatd.x*phatd.x + phatd.z*phatd.z);
        real_t tmp3          = pow(tmp2, 1.5);
        vec3_t J_c2_phat     = (tmp2 > eps) ? vec3_t( 1.0/sqrt(tmp2)       , 0.0,  0.0)                   : vec3_t();
        vec3_t J_c2_phatd    = (tmp3 > eps) ? vec3_t(-(phat.x*phatd.x)/tmp3, 0.0, -(phat.x*phatd.z)/tmp3) : vec3_t();
        real_t J_gamma_c2    = (-1.0 + eps < c2 && c2 < 1.0 - eps) ? -1.0/sqrt(1.0 - c2*c2) : 0.0;
        vec3_t J_gamma_phat  = J_gamma_c2*J_c2_phat;
        vec3_t J_gamma_phatd = J_gamma_c2*J_c2_phatd;
        vec3_t J_alpha_phatd = (tmp2 > eps) ? vec3_t(-phatd.z/tmp2, 0.0, phatd.x/tmp2) : vec3_t();

        Jq.row(4).v_range(0,3) = J_phat_p.trans()*J_gamma_phat + J_phatd_p.trans()*(-J_alpha_phatd + J_gamma_phatd);
        Jq.row(4).v_range(3,3) = J_phat_q.trans()*J_gamma_phat;

        mat3_t J_phatdd_p = vvmat(phatdd%ey, Jq.row(4).v_range(0,3)) + mat3_t::Rot(-joint[4], 'y')*J_phatd_p;
        mat3_t J_phatdd_q = vvmat(phatdd%ey, Jq.row(4).v_range(3,3));

        real_t tmp4 = (phat.y*phat.y + phat.z*phat.z);
        real_t tmp5 = (phatdd.y*phatdd.y + phatdd.z*phatdd.z);
        vec3_t tmp6 = (tmp4 > eps) ? vec3_t(0.0, -phat  .z/tmp4, phat  .y/tmp4) : vec3_t();
        vec3_t tmp7 = (tmp5 > eps) ? vec3_t(0.0, -phatdd.z/tmp5, phatdd.y/tmp5) : vec3_t();
    
        Jq.row(5).v_range(0,3) = -J_phat_p.trans()*tmp6 + J_phatdd_p.trans()*tmp7;
        Jq.row(5).v_range(3,3) = -J_phat_q.trans()*tmp6 + J_phatdd_q.trans()*tmp7;

        mat3_t Jrpy = JacobianRPY(angle_hip);
        mat3_t Jrpy_inv = Jrpy.inv();
        mat3_t J_qd_p;
        mat3_t J_qd_q;

        J_qd_p = - vvmat(qhip*ey    , Jq.row(3).v_range(0,3) + Jq.row(4).v_range(0,3))
                 - vvmat(qhip*qyy*ex, Jq.row(5).v_range(0,3));

        J_qd_q = mat3_t()
                 - vvmat(qhip*ey    , Jq.row(3).v_range(3,3) + Jq.row(4).v_range(3,3))
                 - vvmat(qhip*qyy*ex, Jq.row(5).v_range(3,3));

        J_qd_p = Rzquad*J_qd_p;
        J_qd_q = Rzquad*J_qd_q;

        Jq.row(0).v_range(0,3) =  J_qd_p.trans()*Jrpy_inv.row(2);
        Jq.row(0).v_range(3,3) =  J_qd_q.trans()*Jrpy_inv.row(2);

        Jq.row(1).v_range(0,3) =  J_qd_p.trans()*Jrpy_inv.row(1);
        Jq.row(1).v_range(3,3) =  J_qd_q.trans()*Jrpy_inv.row(1);

        Jq.row(2).v_range(0,3) = -J_qd_p.trans()*Jrpy_inv.row(0);
        Jq.row(2).v_range(3,3) = -J_qd_q.trans()*Jrpy_inv.row(0);

        if(c > 1.0){
            Je.row(0).v_range(0,3) = -J_c_p;
            Je.row(0).v_range(3,3).clear();
            Je.row(1).clear();
        }
        else if(c < -1.0){
            Je.row(0).clear();
            Je.row(1).v_range(0,3) =  J_c_p;
            Je.row(1).v_range(3,3).clear();
        }
        else{
            Je.row(0).clear();
            Je.row(1).clear();
        }
    }
}

void MyIK::CalcLegIK(const vec3_t& pos, const quat_t& ori, vvec_t& joint, vvec_t& error, vmat_t& Jq, vmat_t& Je, bool calc_jacobian, int side){
    // from shoulder to wrist
    //vec3_t pos_base = pos - legBase[side] - ori*ankleToFoot[side];

    real_t l1 = upperLegLength;
    real_t l2 = lowerLegLength;

    // hip, knee, ankle
    joint.resize(6);
    error.resize(2);
    Jq.resize(6, 6, 0.0);
    Je.resize(2, 6, 0.0);

    vec3_t angle = ToRollPitchYaw(ori);

    // hip yaw is directly determined from foot yaw
    joint[0] = angle.z;

    // ankle pos expressed in hip-yaw local
    vec3_t pos_local = quat_t::Rot(-joint[0], 'z')*pos;

    // hip roll
    joint[1] = atan2(pos_local.y, -pos_local.z);

    // ankle pos expressed in hip yaw and hip roll local
    vec3_t pos_local2 = quat_t::Rot(-joint[1], 'x')*pos_local;

    real_t alpha = -atan2(pos_local2.x, -pos_local2.z);
    
    // hip pitch and knee pitch from trigonometrics
    real_t d = pos.norm();
    real_t c = (l1*l1 + l2*l2 - d*d)/(2*l1*l2);
    real_t s;
    real_t beta;
    real_t gamma;

    //  singularity: too close
    if(c > 1.0){
        beta     = 0.0;
        gamma    = 0.0;
        error[0] = 1.0 - c;
        error[1] = 0.0;
    }
    //  singularity: too far
    else if(c < -1.0){
        beta     = pi;
        gamma    = 0.0;
        error[0] = 0.0;
        error[1] = c + 1.0;
    }
    //  nonsingular
    else{
        beta  = acos(c);
        joint[3] = pi - beta;
        
        s = (l2/d)*sin(joint[3]);
        gamma = asin(s);
        
        error[0] = 0.0;
        error[1] = 0.0;
    }

    joint[3] = pi - beta;
    joint[2] = alpha - gamma;

    quat_t qyy   = quat_t::Rot(joint[2]+joint[3], 'y');
    quat_t qxyy  = quat_t::Rot(joint[1], 'x')*qyy;
    quat_t qzxyy = quat_t::Rot(joint[0], 'z')*qxyy;
    quat_t qrel = qzxyy.Conjugated()*ori;
    vec3_t angle_rel = ToRollPitchYaw(qrel);

    joint[4] = angle_rel.y;
    joint[5] = angle_rel.x;

    if(calc_jacobian){
        const vec3_t ex(1.0, 0.0, 0.0);
        const vec3_t ey(0.0, 1.0, 0.0);
        const vec3_t ez(0.0, 0.0, 1.0);
        
        mat3_t J_pd_p       = mat3_t::Rot(joint[0], 'z').trans();
        vec3_t J_pd_theta0  = pos % ez;
        mat3_t J_pd_q       = vvmat(J_pd_theta0, ez);

        real_t tmp          = (pos_local.y*pos_local.y + pos_local.z*pos_local.z);
        vec3_t J_theta1_pd  = (tmp > eps ? vec3_t(0.0, -pos_local.z/tmp, -pos_local.y/tmp) : vec3_t());

        vec3_t J_pdd_theta1 = pos_local % ex;
        mat3_t J_pdd_pd     = mat3_t::Rot(joint[1], 'x').trans() + vvmat(J_pdd_theta1, J_theta1_pd);
        mat3_t J_pdd_p      = J_pdd_pd*J_pd_p;
        mat3_t J_pdd_q      = J_pdd_pd*J_pd_q;

        real_t tmp2         = pos_local2.x*pos_local2.x + pos_local2.z*pos_local2.z;
        real_t tmp3         = sqrt(tmp2);
        vec3_t J_d_pdd      = (tmp3 > eps) ? vec3_t(pos_local2.x/tmp3, 0.0, pos_local2.z/tmp3) : vec3_t();
        real_t J_c_d        = -d/(l1*l2);
        vec3_t J_c_pdd      = J_c_d*J_d_pdd;
        real_t J_s_d        = (-1.0 + eps < c && c < 1.0 - eps) ? (cos(beta)/(l1*sin(beta)) - (l2*sin(beta))/(d*d)) : 0.0;
        vec3_t J_alpha_pdd  = (tmp2 > eps) ? vec3_t(pos_local2.z/tmp2, 0.0, pos_local2.x/tmp2) : vec3_t();
        real_t J_beta_c     = (-1.0 + eps < c && c < 1.0 - eps) ? -1.0/sqrt(1.0 - c*c) : 0.0;
        //c = std::min(std::max(-1.0 + eps, c), 1.0 - eps);
        //real_t J_beta_c     = -1.0/sqrt(1.0 - c*c);
        real_t J_beta_d     = J_beta_c*J_c_d;
        vec3_t J_beta_pdd   = J_beta_d*J_d_pdd;
        real_t J_gamma_s    = (-1.0 + eps < s && s < 1.0 - eps) ?  1.0/sqrt(1.0 - s*s) : 0.0;
        //s = std::min(std::max(-1.0 + eps, s), 1.0 - eps);
        //real_t J_gamma_s    = 1.0/sqrt(1.0 - s*s);
        vec3_t J_gamma_pdd  = J_gamma_s*J_s_d*J_d_pdd;

        Jq.row(0).v_range(0,3).clear();
        Jq.row(0).v_range(3,3) = ez;

        Jq.row(1).v_range(0,3) = J_pd_p.trans()*J_theta1_pd;
        Jq.row(1).v_range(3,3) = J_pd_q.trans()*J_theta1_pd;

        Jq.row(2).v_range(0,3) = J_pdd_p.trans()*(J_alpha_pdd - J_gamma_pdd);
        Jq.row(2).v_range(3,3) = J_pdd_q.trans()*(J_alpha_pdd - J_gamma_pdd);

        Jq.row(3).v_range(0,3) = J_pdd_p.trans()*(-J_beta_pdd);
        Jq.row(3).v_range(3,3) = J_pdd_q.trans()*(-J_beta_pdd);

        mat3_t Jrpy = JacobianRPY(angle_rel);
        mat3_t Jrpy_inv = Jrpy.inv();
        mat3_t J_qd_p;
        mat3_t J_qd_q;

        J_qd_p = //- vvmat(qzxyy.Conjugated()*ez, Jq.row(0).v_range(0,3))  //< this term is zero
                 - vvmat(qxyy.Conjugated()*ex, Jq.row(1).v_range(0,3))
                 - vvmat(qyy .Conjugated()*ey, Jq.row(2).v_range(0,3) + Jq.row(3).v_range(0,3));
        mat3_t Rzxyy;
        qzxyy.ToMatrix(Rzxyy);
        J_qd_q = Rzxyy.trans()
                 - vvmat(qzxyy.Conjugated()*ez, Jq.row(0).v_range(3,3))
                 - vvmat(qxyy .Conjugated()*ex, Jq.row(1).v_range(3,3))
                 - vvmat(qyy  .Conjugated()*ey, Jq.row(2).v_range(3,3) + Jq.row(3).v_range(3,3));

        Jq.row(4).v_range(0,3) = J_qd_p.trans()*Jrpy_inv.row(1);
        Jq.row(4).v_range(3,3) = J_qd_q.trans()*Jrpy_inv.row(1);

        Jq.row(5).v_range(0,3) = J_qd_p.trans()*Jrpy_inv.row(0);
        Jq.row(5).v_range(3,3) = J_qd_q.trans()*Jrpy_inv.row(0);

        // consider offset
        //mat6_t X;
        //X.sub_matrix(TSubMatrixDim<0,0,3,3>()) = mat3_t();
        //X.sub_matrix(TSubMatrixDim<0,3,3,3>()) = mat3_t::Cross(ori*ankleToFoot[side]);
        //X.sub_matrix(TSubMatrixDim<3,0,3,3>()).clear();
        //X.sub_matrix(TSubMatrixDim<3,3,3,3>()) = mat3_t();
        //Jq = Jq*X;

        if(c > 1.0){
            Je.row(0).v_range(0,3) = -J_pdd_p.trans()*J_c_pdd;
            Je.row(0).v_range(3,3) = -J_pdd_q.trans()*J_c_pdd;
            Je.row(1).clear();
        }
        else if(c < -1.0){
            Je.row(0).clear();
            Je.row(1).v_range(0,3) =  J_pdd_p.trans()*J_c_pdd;
            Je.row(1).v_range(3,3) =  J_pdd_q.trans()*J_c_pdd;
        }
        else{
            Je.row(0).clear();
            Je.row(1).clear();
        }
    }
}
