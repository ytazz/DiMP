#include "myik.h"

#include <sbrollpitchyaw.h>

const real_t pi = 3.1415926535;

MyIK::MyIK(){
    armBase[0]     = vec3_t(0.0, -0.1 ,  0.1);
	armBase[1]     = vec3_t(0.0,  0.1 ,  0.1);
	legBase[0]     = vec3_t(0.0, -0.1 , -0.1);
	legBase[1]     = vec3_t(0.0,  0.1 , -0.1);
	elbowYaw[0]    = 0.0;
	elbowYaw[1]    = 0.0;
	torsoLength    = 0.2;
	upperArmLength = 0.2;
	lowerArmLength = 0.2;
	upperLegLength = 0.35;
	lowerLegLength = 0.35;
}

void MyIK::CalcTorsoIK(const vec3_t& pos, const quat_t& ori, vvec_t& joint, vvec_t& error){
    // hip, chest_p, chest_y
    joint.resize(5);
    error.resize(4);

    vec3_t theta = ToRollPitchYaw(ori);
    joint[0] = 0.0;
    joint[1] = theta.z;
    joint[2] = theta.y;
    joint[3] = 0.0;
    joint[4] = 0.0;

    vec3_t offset(0.0, 0.0, torsoLength);
    error[0] = pos.x - offset.x;
    error[1] = pos.y - offset.y;
    error[2] = pos.z - offset.z;
    error[3] = theta.x;

}

void MyIK::CalcArmIK(const vec3_t& pos, const quat_t& ori, vvec_t& joint, vvec_t& error, int side){
    vec3_t pos_base = pos - armBase[side];

    real_t l1 = upperArmLength;
    real_t l2 = lowerArmLength;

    // shoulder, elbow, wrist
    joint.resize(7);
    error.resize(2);

    // elbow pitch from trigonometrics
    real_t tmp = (l1*l1 + l2*l2 - pos_base.square())/(2*l1*l2);
    
    //  singularity: too close
    if(tmp > 1.0){
        joint[3] = pi;
        error[0] = 1.0 - tmp;
        error[1] = 0.0;
    }
    //  singularity: too far
    else if(tmp < -1.0){
        joint[3] = 0.0;
        error[0] = 0.0;
        error[1] = tmp + 1.0;
    }
    //  nonsingular
    else{
        joint[3] = -(pi - acos(tmp));
        error[0] = 0.0;
        error[1] = 0.0;
    }

    // shoulder yaw is given
    joint[2] = elbowYaw[side];

    // wrist pos expressed in shoulder pitch-roll local
    vec3_t pos_local = quat_t::Rot(joint[2], 'z')*vec3_t(-l2*sin(joint[3]), 0.0, -l1 - l2*cos(joint[3]));

    // shoulder roll
    tmp = pos_base.y/sqrt(pos_local.y*pos_local.y + pos_local.z*pos_local.z);
    real_t delta;
    if(tmp > 1.0){
        delta = 0.0;
    }
    else if(tmp < -1.0){
        delta = pi;
    }
    else{
        delta =  acos(tmp);
    }

    real_t alpha = atan2(pos_local.z, pos_local.y);
    joint[1] = -alpha - delta;
    
    // wrist pos expressed in shoulder pitch local
    vec3_t pos_local2 = quat_t::Rot(joint[1], 'x')*pos_local;

    // shoulder pitch
    joint[0] = atan2(pos_base.x, pos_base.z)
             - atan2(pos_local2.x, pos_local2.z);
    if(joint[0] >  pi) joint[0] -= 2.0*pi;
    if(joint[0] < -pi) joint[0] += 2.0*pi;

    // desired hand orientation
    quat_t qhand = ( quat_t::Rot(joint[0], 'y')
                    *quat_t::Rot(joint[1], 'x')
                    *quat_t::Rot(joint[2], 'z')
                    *quat_t::Rot(joint[3], 'y') ).Conjugated()*ori;

    // convert it to roll-pitch-yaw
    vec3_t angle_hand = ToRollPitchYaw(qhand);

    // then wrist angles are determined
    joint[4] = angle_hand.z;
    joint[5] = angle_hand.y;
    joint[6] = angle_hand.x;

}

void MyIK::CalcLegIK(const vec3_t& pos, const quat_t& ori, vvec_t& joint, vvec_t& error, int side){
    vec3_t pos_base = pos - legBase[side];

    real_t l1 = upperLegLength;
    real_t l2 = lowerLegLength;

    // hip, knee, ankle
    joint.resize(6);
    error.resize(2);

    vec3_t angle = ToRollPitchYaw(ori);

    // hip yaw is directly determined from foot yaw
    joint[0] = angle.z;

    // ankle pos expressed in hip-yaw local
    vec3_t pos_local = quat_t::Rot(-joint[0], 'z')*pos_base;

    // hip roll
    joint[1] = atan2(pos_local.y, -pos_local.z);

    // ankle pos expressed in hip yaw and hip roll local
    vec3_t pos_local2 = quat_t::Rot(-joint[1], 'x')*pos_local;

    real_t alpha = -atan2(pos_local2.x, -pos_local2.z);
    
    // hip pitch and knee pitch from trigonometrics
    real_t d   = pos_base.norm();
    real_t tmp = (l1*l1 + l2*l2 - d*d)/(2*l1*l2);

    //  singularity: too close
    if(tmp > 1.0){
        joint[3] = pi;
        joint[2] = alpha;
        error[0] = 1.0 - tmp;
        error[1] = 0.0;
    }
    //  singularity: too far
    else if(tmp < -1.0){
        joint[3] = 0.0;
        joint[2] = alpha;
        error[0] = 0.0;
        error[1] = tmp + 1.0;
    }
    //  nonsingular
    else{
        joint[3] = pi - acos(tmp);
        joint[2] = alpha - asin((l2/d)*sin(joint[3]));
        error[0] = 0.0;
        error[1] = 0.0;
    }

    quat_t qzxyyy = quat_t::Rot(joint[0],          'z')
                   *quat_t::Rot(joint[1],          'x')
                   *quat_t::Rot(joint[2]+joint[3], 'y');
    quat_t qrel = qzxyyy.Conjugated()*ori;
    vec3_t angle_rel = ToRollPitchYaw(qrel);

    joint[4] = angle_rel.y;
    joint[5] = angle_rel.x;

}

void MyIK::CalcTorsoFK(const vvec_t& joint, vector<vec3_t>& pos, vector<quat_t>& ori){
    vec3_t trn[5];
    quat_t rot[5];
    trn[0] = vec3_t();
    trn[1] = vec3_t(0.0, 0.0, torsoLength);
    trn[2] = vec3_t();
    trn[3] = vec3_t();
    trn[4] = vec3_t();
    rot[0] = quat_t();
    rot[1] = quat_t::Rot(joint[1], 'z');
    rot[2] = quat_t::Rot(joint[2], 'y');
    rot[3] = quat_t();
    rot[4] = quat_t();

    pos.resize(5);
    ori.resize(5);
    
    vec3_t pbase;
    quat_t qbase;
    for(int i = 0; i < 5; i++){
        pos[i] = (i == 0 ? pbase : pos[i-1]) + (i == 0 ? qbase : ori[i-1])*trn[i];
        ori[i] = (i == 0 ? qbase : ori[i-1])*rot[i];
    }
}

void MyIK::CalcArmFK(const vvec_t& joint, vector<vec3_t>& pos, vector<quat_t>& ori, int side){
    vec3_t trn[7];
    quat_t rot[7];
    trn[0] = vec3_t();
    trn[1] = vec3_t();
    trn[2] = vec3_t();
    trn[3] = vec3_t(0.0, 0.0, -upperArmLength);
    trn[4] = vec3_t(0.0, 0.0, -lowerArmLength);
    trn[5] = vec3_t();
    trn[6] = vec3_t();
    rot[0] = quat_t::Rot(joint[0], 'y');
    rot[1] = quat_t::Rot(joint[1], 'x');
    rot[2] = quat_t::Rot(joint[2], 'z');
    rot[3] = quat_t::Rot(joint[3], 'y');
    rot[4] = quat_t::Rot(joint[4], 'z');
    rot[5] = quat_t::Rot(joint[5], 'y');
    rot[6] = quat_t::Rot(joint[6], 'x');

    pos.resize(7);
    ori.resize(7);

    vec3_t pbase = armBase[side];
    quat_t qbase;
    for(int i = 0; i < 7; i++){
        pos [i] = (i == 0 ? pbase : pos[i-1]) + (i == 0 ? qbase : ori[i-1])*trn[i];
        ori [i] = (i == 0 ? qbase : ori[i-1])*rot[i];
    }
}

void MyIK::CalcLegFK(const vvec_t& joint, vector<vec3_t>& pos, vector<quat_t>& ori, int side){
    vec3_t trn[6];
    quat_t rot[6];
    trn[0] = vec3_t();
    trn[1] = vec3_t();
    trn[2] = vec3_t();
    trn[3] = vec3_t(0.0, 0.0, -upperLegLength);
    trn[4] = vec3_t(0.0, 0.0, -lowerLegLength);
    trn[5] = vec3_t();
    rot[0] = quat_t::Rot(joint[0], 'z');
    rot[1] = quat_t::Rot(joint[1], 'x');
    rot[2] = quat_t::Rot(joint[2], 'y');
    rot[3] = quat_t::Rot(joint[3], 'y');
    rot[4] = quat_t::Rot(joint[4], 'y');
    rot[5] = quat_t::Rot(joint[5], 'x');

    pos.resize(6);
    ori.resize(6);

    vec3_t pbase = legBase[side];
    quat_t qbase;
    for(int i = 0; i < 6; i++){
        pos [i] = (i == 0 ? pbase : pos[i-1]) + (i == 0 ? qbase : ori[i-1])*trn[i];
        ori [i] = (i == 0 ? qbase : ori[i-1])*rot[i];
    }
}
