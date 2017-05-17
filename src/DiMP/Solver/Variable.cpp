#include <DiMP/Solver/Variable.h>
#include <DiMP/Solver/Constraint.h>
#include <DiMP/Solver/Link.h>
#include <DiMP/Solver/Solver.h>
#include <DiMP/Solver/Range.h>

namespace DiMP{;

Variable::Variable(uint _type, Solver* solver, ID id, real_t _scale):ID(id){
	solver->AddVar(this);
	SetScale(_scale);

	type = _type;
	switch(type){
	case Scalar:	nelem = 1; break;
	case Vec3:		nelem = 3; break;
	case Quat:
	default:		nelem = 4; break;
	}
	
	locked    = false;
}

void Variable::Lock(bool on){
	locked = on;
}

void Variable::SetScale(real_t sc){
	scale      = sc;
	scale2     = sc*sc;
	scale_inv  = (real_t)1.0/sc;
	scale_inv2 = scale_inv*scale_inv;
}

void Variable::ResetState(){
	d .clear();
	dd.clear();
	de.clear();
}

void Variable::UpdateVar(uint k, real_t dd_noscale){
	// no update if locked
	if(locked)
		return;

	// note that scaling ratio is multiplied twice
	d[k] += (dd[k] = scale2 * dd_noscale);
	
	// error update: 
	for(uint l = 0; l < links.size(); l++)
		links[l]->Forward(k, dd[k]);
}

}
