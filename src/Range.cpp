#include <DiMP2/Range.h>
#include <DiMP2/Link.h>


namespace DiMP2{;

//-------------------------------------------------------------------------------------------------

FixConS::FixConS(Solver* solver, ID id, SVar* var, real_t _scale):Constraint(solver, 1, id, _scale){
	desired = 0.0;
	AddSLink(var, 1.0);
}

void FixConS::CalcDeviation(){
	y[0] = ((SVar*)links[0]->var)->val - desired;
}

//-------------------------------------------------------------------------------------------------

MatchConS::MatchConS(Solver* solver, ID id, SVar* var0, SVar* var1, real_t _scale):Constraint(solver, 1, id, _scale){
	AddSLink(var0, -1.0);
	AddSLink(var1,  1.0);
}

//-------------------------------------------------------------------------------------------------

FixConV3::FixConV3(Solver* solver, ID id, V3Var* var, real_t _scale):Constraint(solver, 3, id, _scale){
	AddSLink(var, 1.0);
}

void FixConV3::CalcDeviation(){
	y = ((V3Var*)links[0]->var)->val - desired;
}

//-------------------------------------------------------------------------------------------------

MatchConV3::MatchConV3(Solver* solver, ID id, V3Var* var0, V3Var* var1, real_t _scale):Constraint(solver, 3, id, _scale){
	AddSLink(var0, -1.0);
	AddSLink(var1,  1.0);
}

//-------------------------------------------------------------------------------------------------

FixConQ::FixConQ(Solver* solver, ID id, QVar* var, real_t _scale):Constraint(solver, 3, id, _scale){
	AddSLink(var, 1.0);
}

void FixConQ::CalcDeviation(){
	quat_t q0 = desired;
	quat_t q1 = ((QVar*)links[0]->var)->val;
	quat_t qerror = q0.Conjugated() * q1;
	y = q0 * (qerror.Theta() * qerror.Axis());
}

//-------------------------------------------------------------------------------------------------

MatchConQ::MatchConQ(Solver* solver, ID id, QVar* var0, QVar* var1, real_t _scale):Constraint(solver, 3, id, _scale){
	AddSLink(var0, -1.0);
	AddSLink(var1,  1.0);
}

void MatchConQ::CalcDeviation(){
	quat_t q0 = ((QVar*)links[0]->var)->val;
	quat_t q1 = ((QVar*)links[1]->var)->val;
	quat_t qerror = q0.Conjugated() * q1;
	y = q0 * (qerror.Theta() * qerror.Axis());
}

//-------------------------------------------------------------------------------------------------

RangeConS::RangeConS(Solver* solver, ID id, SVar* var, real_t _scale):Constraint(solver, 1, id, _scale){
	AddSLink(var, 1.0);
	real_t inf = numeric_limits<real_t>::max();
	_min = -inf;
	_max =  inf;
	on_lower = false;
	on_upper = false;
}
void RangeConS::CalcCoef(){
}
void RangeConS::CalcDeviation(){
	real_t s = ((SVar*)links[0]->var)->val;
	on_lower = (s < _min);
	on_upper = (s > _max);
	active = on_lower | on_upper;
	if(on_lower)
		y[0] = s - _min;
	if(on_upper)
		y[0] = s - _max;
}

void RangeConS::Project(real_t& l, uint k){
	// ‰ºŒÀ‚ÆãŒÀ‚ª“¯‚¶ê‡‚ÍŽË‰e‘€ì‚µ‚È‚¢
	if(_min == _max)
		return;
	if(on_upper && l > 0.0)
		l = 0.0;
	if(on_lower && l < 0.0)
		l = 0.0;
	if(!on_upper && !on_lower)
		l = 0.0;
}

//-------------------------------------------------------------------------------------------------

RangeConV3::RangeConV3(Solver* solver, ID id, V3Var* var, real_t _scale):Constraint(solver, 3, id, _scale){
	AddSLink(var, 1.0);
	real_t inf = numeric_limits<real_t>::max();
	for(int i = 0; i < 3; i++){
		_min    [i] = -inf;
		_max    [i] =  inf;
		on_lower[i] = false;
		on_upper[i] = false;
	}
}

void RangeConV3::CalcDeviation(){
	active = false;
	for(int i = 0; i < 3; i++){
		real_t s = ((V3Var*)links[0]->var)->val[i];
		on_lower[i] = (s < _min[i]);
		on_upper[i] = (s > _max[i]);
		active |= on_lower[i] | on_upper[i];
		if(on_lower[i])
			y[i] = s - _min[i];
		if(on_upper[i])
			y[i] = s - _max[i];
	}
}

void RangeConV3::Project(real_t& l, uint k){
	if(on_upper[k] && l > 0.0)
		l = 0.0;
	if(on_lower[k] && l < 0.0)
		l = 0.0;
	if(!on_upper[k] && !on_lower[k])
		l = 0.0;
}
	
//-------------------------------------------------------------------------------------------------

DiffConS::DiffConS(Solver* solver, ID id, SVar* var0, SVar* var1, real_t _scale):Constraint(solver, 1, id, _scale){
	AddSLink(var0, -1.0);
	AddSLink(var1,  1.0);
	real_t inf = numeric_limits<real_t>::max();
	_min = -inf;
	_max =  inf;
	on_lower = false;
	on_upper = false;
}

void DiffConS::CalcDeviation(){
	real_t diff = ((SVar*)links[1]->var)->val - ((SVar*)links[0]->var)->val;
	on_lower = (diff < _min);
	on_upper = (diff > _max);
	active = on_lower | on_upper;
	if(on_lower)
		y[0] = (diff - _min) ;
	if(on_upper)			   
		y[0] = (diff - _max) ;
}

void DiffConS::Project(real_t& l, uint k){
	if(on_upper && l > 0.0)
		l = 0.0;
	if(on_lower && l < 0.0)
		l = 0.0;
	if(!on_upper && !on_lower)
		l = 0.0;
}

//-------------------------------------------------------------------------------------------------

FixConPlane::FixConPlane(Solver* solver, ID id, V3Var* var, real_t _scale):
	Constraint(solver, 1, id, _scale){
	AddRLink(var);
}

void FixConPlane::CalcCoef(){
	((RLink*)links[0])->SetCoef(normal);
}

void FixConPlane::CalcDeviation(){
	y[0] = normal * ( ((V3Var*)links[0]->var)->val - origin);
}

//-------------------------------------------------------------------------------------------------

RangeConPlane::RangeConPlane(Solver* solver, ID id, V3Var* var, real_t _scale):
	Constraint(solver, 1, id, _scale){
	AddRLink(var);
	normal = vec3_t(0.0, 1.0, 0.0);
	origin = vec3_t();
	real_t inf = numeric_limits<real_t>::max();
	_min = -inf;
	_max =  inf;
	on_lower = false;
	on_upper = false;
}

void RangeConPlane::CalcCoef(){
	((RLink*)links[0])->SetCoef(normal);
}

void RangeConPlane::CalcDeviation(){
	real_t s = normal * ( ((V3Var*)links[0]->var)->val - origin);
	on_lower = (s < _min);
	on_upper = (s > _max);
	active = on_lower | on_upper;
	if(on_lower)
		y[0] = s - _min;
	if(on_upper)
		y[0] = s - _max;
}

void RangeConPlane::Project(real_t& l, uint k){
	if(on_upper && l > 0.0)
		l = 0.0;
	if(on_lower && l < 0.0)
		l = 0.0;
	if(!on_upper && !on_lower)
		l = 0.0;
}

//-------------------------------------------------------------------------------------------------

ComplConS::ComplConS(Solver* solver, ID id, SVar* var0, SVar* var1, real_t _scale):Constraint(solver, 1, id, _scale){
	AddSLink(var0);
	AddSLink(var1);
}

void ComplConS::CalcCoef(){
	((SLink*)links[0])->SetCoef(((SVar*)links[1]->var)->val);
	((SLink*)links[1])->SetCoef(((SVar*)links[0]->var)->val);
}

void ComplConS::CalcDeviation(){
	y[0] = ((SVar*)links[0]->var)->val * ((SVar*)links[1]->var)->val;
}

//-------------------------------------------------------------------------------------------------

DistanceConV3::DistanceConV3(Solver* solver, ID id, V3Var* var0, V3Var* var1, real_t _scale):Constraint(solver, 1, id, _scale){
	AddRLink(var0);
	AddRLink(var1);
}

void DistanceConV3::CalcCoef(){
	diff = ((V3Var*)links[0]->var)->val - ((V3Var*)links[1]->var)->val;
	diff_norm = diff.norm();
	const real_t eps = 1.0e-10;
	if(diff_norm < eps){
		((RLink*)links[0])->SetCoef( diff);
		((RLink*)links[1])->SetCoef(-diff);
	}
	else{
		((RLink*)links[0])->SetCoef( diff/diff_norm);
		((RLink*)links[1])->SetCoef(-diff/diff_norm);
	}
}

void DistanceConV3::CalcDeviation(){
	on_lower = (diff_norm < _min);
	on_upper = (diff_norm > _max);
	active = on_lower | on_upper;
	if(on_lower)
		y[0] = diff_norm - _min;
	if(on_upper)
		y[0] = diff_norm - _max;
}

void DistanceConV3::Project(real_t& l, uint k){
	if(on_upper && l > 0.0)
		l = 0.0;
	if(on_lower && l < 0.0)
		l = 0.0;
	if(!on_upper && !on_lower)
		l = 0.0;
}

//-------------------------------------------------------------------------------------------------

C1ConS::C1ConS(Solver* solver, ID id, SVar* p0, SVar* v0, SVar* p1, SVar* v1, real_t _scale):
	Constraint(solver, 1, id, _scale){
	AddSLink(p0);
	AddSLink(v0);
	AddSLink(p1);
	AddSLink(v1);
}
void C1ConS::CalcCoef(){
	((SLink*)links[0])->SetCoef(-1.0);
	((SLink*)links[1])->SetCoef(-0.5 * h);
	((SLink*)links[2])->SetCoef( 1.0);
	((SLink*)links[3])->SetCoef(-0.5 * h);
}

//-------------------------------------------------------------------------------------------------

C1ConV3::C1ConV3(Solver* solver, ID id, V3Var* p0, V3Var* v0, V3Var* p1, V3Var* v1, real_t _scale):
	Constraint(solver, 3, id, _scale){
	AddSLink(p0);
	AddSLink(v0);
	AddSLink(p1);
	AddSLink(v1);
}
void C1ConV3::CalcCoef(){
	uint i = 0;
	((SLink*)links[i++])->SetCoef(-1.0);
	((SLink*)links[i++])->SetCoef(-0.5*h);
	((SLink*)links[i++])->SetCoef( 1.0);
	((SLink*)links[i++])->SetCoef(-0.5*h);
}

}
