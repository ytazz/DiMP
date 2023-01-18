#pragma once

#include <DiMP/DiMP.h>

#include <sbcsv.h>

class MyIK {
public:
	struct Link{
		enum{
			Hips,
			ChestY,
			ChestP,
			HeadY,
			HeadP,
			UpperArmRP,
			UpperArmRR,
			UpperArmRY,
			LowerArmRP,
			LowerArmRY,
			HandRP,
			HandRR,
			UpperArmLP,
			UpperArmLR,
			UpperArmLY,
			LowerArmLP,
			LowerArmLY,
			HandLP,
			HandLR,
			UpperLegRY,
			UpperLegRR,
			UpperLegRP,
			LowerLegRP,
			FootRP,
			FootRR,
			UpperLegLY,
			UpperLegLR,
			UpperLegLP,
			LowerLegLP,
			FootLP,
			FootLR,		
			Num
		};
	};
	struct End{
		enum{
			Hips,
			ChestP,
			HandR,
			HandL,
			FootR,
			FootL,
			Num
		};
	};
	struct Chain{
		enum{
			Torso,
			ArmR,
			ArmL,
			LegR,
			LegL,
			Num
		};
	};
	struct Limit{
		enum{
			Num = 12
		};
	};

	DiMP::Wholebody*  wb;

	vec3_t armBase[2];
	vec3_t legBase[2];
	real_t elbowYaw[2];
	real_t torsoLength;
	real_t upperArmLength;
	real_t lowerArmLength;
	real_t upperLegLength;
	real_t lowerLegLength;

public:
	void CalcTorsoIK(const vec3_t& pos, const quat_t& ori, vvec_t& joint, vvec_t& error);
	void CalcArmIK  (const vec3_t& pos, const quat_t& ori, vvec_t& joint, vvec_t& error, int side);
	void CalcLegIK  (const vec3_t& pos, const quat_t& ori, vvec_t& joint, vvec_t& error, int side);
	
	void CalcTorsoFK(const vvec_t& joint, vector<vec3_t>& pos, vector<quat_t>& ori);
	void CalcArmFK  (const vvec_t& joint, vector<vec3_t>& pos, vector<quat_t>& ori, int side);
	void CalcLegFK  (const vvec_t& joint, vector<vec3_t>& pos, vector<quat_t>& ori, int side);

	MyIK();
};
