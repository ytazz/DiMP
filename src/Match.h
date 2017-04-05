#pragma once

#include <DiMP2/Task.h>
#include <DiMP2/Constraint.h>

namespace DiMP2{;

class MatchTask;
class MatchConTP;
class MatchConTV;
class MatchConRP;
class MatchConRV;

class MatchTaskKey : public TaskKey{
public:	
	/** �ʒu/���x�̕��i/��]�����ɑ΂���S��
		- ���ꂼ��ɂ���Start, Inside, End��3������
		- Start�͂��̃L�[�|�C���g�Ƃ��̎��̊ԂɃ^�C�~���O�n�_���܂܂��Ƃ�
		- End�͂��̃L�[�|�C���g�Ƃ��̎��̊ԂɃ^�C�~���O�I�_���܂܂��Ƃ�
		- Inside�͂��̃L�[�|�C���g���^�C�~���O��ԂɊ܂܂��Ƃ�
		�ɗL���ƂȂ�
	 **/
	MatchConTP*	con_tp[3];
	MatchConTV*	con_tv[3];
	MatchConRP*  con_rp[3];
	MatchConRV*  con_rv[3];

public:
	void	SumError(real_t& sum);

	virtual void AddCon(Solver* solver);
	virtual void Prepare();

	MatchTaskKey();
};

/** �|�W�V�����}�b�`���O�^�X�N
	- 2�̍��̂̈ʒu����v������
 **/
class MatchTask : public Task{
public:
	struct Param{
		bool	spline;		///< if true, cubic spline. otherwise, quadratic curve.
		bool	match_tp;
		bool	match_rp;
		bool	match_tv;
		bool	match_rv;

		Param();
	} param;
	
public:
	
	virtual Keypoint* CreateKeypoint(){ return new MatchTaskKey(); }
	virtual void      Draw          (DrawCanvas* canvas);
	
	MatchTask(Object* _obj0, Object* _obj1, TimeSlot* _time, const string& n = "");
};

class MatchCon : public Constraint{
public:
	MatchTaskKey*	key;
	int				mode;

public:
	void AddLinks(bool pos_or_vel);

	MatchCon(Solver* solver, ID id, MatchTaskKey* _key, int _mode, real_t _scale);
	virtual ~MatchCon(){}
};

class MatchConT : public MatchCon{
public:
	void AddLinks(bool pos_or_vel);
	MatchConT(Solver* solver, ID id, MatchTaskKey* _key, int _mode, real_t _scale);
};

class MatchConR : public MatchCon{
public:
	void AddLinks(bool pos_or_vel);
	MatchConR(Solver* solver, ID id, MatchTaskKey* _key, int _mode, real_t _scale);
};

class MatchConTP : public MatchConT{
public:
	virtual void CalcCoef();
	virtual void CalcDeviation();

	MatchConTP(Solver* solver, const string& _name, MatchTaskKey* _key, int _mode, real_t _scale);
	virtual ~MatchConTP(){}
};

class MatchConTV : public MatchConT{
public:
	virtual void CalcCoef();
	virtual void CalcDeviation();

	MatchConTV(Solver* solver, const string& _name, MatchTaskKey* _key, int _mode, real_t _scale);
	virtual ~MatchConTV(){}
};

class MatchConRP : public MatchConR{
public:
	virtual void CalcCoef();
	virtual void CalcDeviation();

	MatchConRP(Solver* solver, const string& _name, MatchTaskKey* _key, int _mode, real_t _scale);
	virtual ~MatchConRP(){}
};

class MatchConRV : public MatchConR{
public:
	virtual void CalcCoef();
	virtual void CalcDeviation();

	MatchConRV(Solver* solver, const string& _name, MatchTaskKey* _key, int _mode, real_t _scale);
	virtual ~MatchConRV(){}
};

}
