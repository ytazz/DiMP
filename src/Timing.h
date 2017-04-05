#pragma once

#include <DiMP2/Node.h>

namespace DiMP2{;

class Graph;
class Object;

class RangeConS;
class DiffConS;

class TimeSlot : public Node{
public:
	struct Param{
		real_t	ts_ini;
		real_t	te_ini;
		real_t	ts_min, ts_max;
		real_t	te_min, te_max;
		real_t	T_min, T_max;
		bool	lock;

		Param();
	} param;

	SVar*		time_s;				///< starting time variable
	SVar*		time_e;				///< ending time variable
	RangeConS*	con_range_s;
	RangeConS*	con_range_e;
	DiffConS*	con_diff;

public:
	virtual void AddVar();
	virtual void AddCon();

	TimeSlot(Graph* g, real_t ts, real_t te, bool lock = true, const string& n = "");
	virtual ~TimeSlot();
};

}
