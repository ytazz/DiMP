#pragma once

#include <DiMP/Graph/Joint.h>

namespace DiMP{;

class  Contact;
class  FrictionCon;
class  ComplConS;

/**
	Contact
	- ���ʂƓ_�̐ڐG�S��

 */
class ContactKey : public JointKey{
public:
	FrictionCon*	con_f;
	int             mode;

public:
	virtual void AddVar(Solver* solver);
	virtual void AddCon(Solver* solver);
	virtual void Prepare();
};

class Contact : public Joint{
public:
	/// �ڐG���[�h
	struct Mode{
		enum{
			Float,		///< ��ڐG
			Stick,		///< �ڐG�C���ׂ�Ȃ�
			Slip,		///< �ڐG�C���ׂ肠��
		};
	};
	struct ModeSetting{
		real_t ts, te;
		int mode;
		
		ModeSetting(real_t _ts, real_t _te, int _mode):ts(_ts), te(_te), mode(_mode){}
	};
	struct Param{
		real_t mu;		///< �Î~���C�W��
		real_t margin;	///< ��ڐG���̃}�[�W��

		vector<ModeSetting>	setting;

		Param();
	} con_param;

	virtual bool		IsRotational(uint i){ return false; }
	virtual Keypoint*	CreateKeypoint(){ return new ContactKey(); }

	virtual void	CalcRelativePose(real_t* pos, vec3_t& p, quat_t& q);
	virtual void	CalcJointPos    (real_t* pos, vec3_t& p, quat_t& q);
	virtual void    CalcJacobian    (real_t* pos, vec3_t* Jv, vec3_t* Jw);
	
	Contact(Connector* sock, Connector* plug, TimeSlot* time, const string& n);
};

/*
	���C��
 */
class FrictionCon : public Constraint{
public:
	SVar*	fn;			///< normal force
	SVar*	ft[2];		///< tangential force
	
	real_t mu;
	real_t ft_norm;

	virtual void CalcCoef();
	virtual void CalcDeviation();
	virtual void Project(real_t& l, uint k);

	FrictionCon(Solver* solver, ID id, SVar* _fn, SVar* _ft0, SVar* ft1, real_t _mu, real_t _scale);
};

}
