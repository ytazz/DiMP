#pragma once

#include <DiMP/Types.h>

namespace DiMP{;

class CustomSolver : public Solver{
public:
	virtual void    CalcDirection();
};

}
