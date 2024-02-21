#include "myik.h"

#include <sbrollpitchyaw.h>

const real_t pi = 3.1415926535;
const real_t eps = 0.01;

using namespace PTM;

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
