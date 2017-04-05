#include <DiMP2/Link.h>
#include <DiMP2/Variable.h>
#include <DiMP2/Constraint.h>

namespace DiMP2{;

Link::Link(){
	active = true;
}

void Link::Connect(Variable* v, Constraint* c){
	var = v;
	con = c;
	var->links.push_back(this);
	con->links.push_back(this);
}

//-------------------------------------------------------------------------------------------------

SLink::SLink(real_t k):Link(){
	SetCoef(k);
}

void SLink::SetCoef(real_t k){
	coef    = k;
	coefsqr = k*k;
}

void SLink::AddRowSqr(vec3_t& v){
	for(uint k = 0; k < con->nelem; k++)
		v[k] += coefsqr * var->scale2;
}

void SLink::AddError(){
	if(con->nelem == 1)
		 con->y[0] += coef * ((SVar *)var)->val;
	else con->y    += coef * ((V3Var*)var)->val;
}
	
void SLink::Forward(uint k, real_t d){
	con->UpdateError(k, coef * d);
}
void SLink::Backward(uint k, real_t d){
	var->UpdateVar(k, coef * d);
}
vec3_t SLink::Backward(vec3_t v){
	return coef * v;
}

//-------------------------------------------------------------------------------------------------

V3Link::V3Link(vec3_t k){
	SetCoef(k);
}

void V3Link::SetCoef(vec3_t k){
	coef = k;
	for(uint i = 0; i < 3; i++)
		coefsqr[i] = k[i]*k[i];
}

//-------------------------------------------------------------------------------------------------

void XLink::AddRowSqr(vec3_t& v){
	v[0] += (coefsqr[2] + coefsqr[1]) * var->scale2;
	v[1] += (coefsqr[2] + coefsqr[0]) * var->scale2;
	v[2] += (coefsqr[1] + coefsqr[0]) * var->scale2;
}

void XLink::AddError(){
	con->y += coef % ((V3Var*)var)->val;
}

void XLink::Forward(uint k, real_t d){
	int k1 = (k+1)%3;
	int k2 = (k+2)%3;
	con->UpdateError(k1,  coef[k2] * d);
	con->UpdateError(k2, -coef[k1] * d);
}
void XLink::Backward(uint k, real_t d){
	int k1 = (k+1)%3;
	int k2 = (k+2)%3;
	var->UpdateVar(k1, -coef[k2] * d);
	var->UpdateVar(k2,  coef[k1] * d);
}
vec3_t XLink::Backward(vec3_t v){
	return -coef % v;
}

//-------------------------------------------------------------------------------------------------

void CLink::AddRowSqr(vec3_t& v){
	v[0] += coefsqr[0] * var->scale2;
	v[1] += coefsqr[1] * var->scale2;
	v[2] += coefsqr[2] * var->scale2;
}

void CLink::AddError(){
	con->y += coef * ((SVar*)var)->val;
}

void CLink::Forward(uint k, real_t d){
	con->UpdateError(0, coef[0] * d);
	con->UpdateError(1, coef[1] * d);
	con->UpdateError(2, coef[2] * d);
}
void CLink::Backward(uint k, real_t d){
	var->UpdateVar(0, coef[k] * d);
}
vec3_t CLink::Backward(vec3_t v){
	return vec3_t(coef * v, 0.0, 0.0);
}


//-------------------------------------------------------------------------------------------------

void RLink::AddRowSqr(vec3_t& v){
	v[0] += coefsqr[0] * var->scale2;
	v[0] += coefsqr[1] * var->scale2;
	v[0] += coefsqr[2] * var->scale2;
}

void RLink::AddError(){
	con->y[0] += coef * ((V3Var*)var)->val;
}

void RLink::Forward(uint k, real_t d){
	con->UpdateError(0, coef[k] * d);
}

void RLink::Backward(uint k, real_t d){
	var->UpdateVar(0, coef[0] * d);
	var->UpdateVar(1, coef[1] * d);
	var->UpdateVar(2, coef[2] * d);
}
vec3_t RLink::Backward(vec3_t v){
	return coef * v[0];
}

//-------------------------------------------------------------------------------------------------

void MLink::SetCoef(const mat3_t& m){
	coef = m;
	for(uint i = 0; i < 3; i++)for(uint j = 0; j < 3; j++)
		coefsqr[i][j] = coef[i][j]*coef[i][j];
}

void MLink::AddRowSqr(vec3_t& v){
	v[0] += (coefsqr[0][0] + coefsqr[0][1] + coefsqr[0][2]) * var->scale2;
	v[1] += (coefsqr[1][0] + coefsqr[1][1] + coefsqr[1][2]) * var->scale2;
	v[2] += (coefsqr[2][0] + coefsqr[2][1] + coefsqr[2][2]) * var->scale2;
}

void MLink::AddError(){
	con->y += coef * ((V3Var*)var)->val;
}

void MLink::Forward(uint k, real_t d){
	con->UpdateError(0, coef[0][k] * d);
	con->UpdateError(1, coef[1][k] * d);
	con->UpdateError(2, coef[2][k] * d);
}

void MLink::Backward(uint k, real_t d){
	var->UpdateVar(0, coef[k][0] * d);
	var->UpdateVar(1, coef[k][1] * d);
	var->UpdateVar(2, coef[k][2] * d);
}

vec3_t MLink::Backward(vec3_t v){
	return coef.trans() * v;
}

}
