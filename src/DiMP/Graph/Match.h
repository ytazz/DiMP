#pragma once

#include <DiMP/Graph/Task.h>
//#include <DiMP/Solver/Constraint.h>

namespace DiMP{;

class MatchTask;
class MatchConTP;
class MatchConTV;
class MatchConRP;
class MatchConRV;

class MatchTaskKey : public TaskKey{
public:	
	/** 位置/速度の並進/回転成分に対する拘束
		- それぞれについてStart, Inside, Endの3つがある
		- Startはこのキーポイントとその次の間にタイミング始点が含まれるとき
		- Endはこのキーポイントとその次の間にタイミング終点が含まれるとき
		- Insideはこのキーポイントがタイミング区間に含まれるとき
		に有効となる
	 **/
	MatchConTP*	 con_tp[3];
	MatchConTV*	 con_tv[3];
	MatchConRP*  con_rp[3];
	MatchConRV*  con_rv[3];

public:
	void	SumError(real_t& sum);

	virtual void AddCon(Solver* solver);
	virtual void Prepare();

	MatchTaskKey();
};

/** ポジションマッチングタスク
	- 2つの剛体の位置を一致させる
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
	virtual void      Draw          (Render::Canvas* canvas);
	
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
