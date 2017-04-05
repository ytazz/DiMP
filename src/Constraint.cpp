#include <DiMP2/Solver.h>
#include <DiMP2/Link.h>

namespace DiMP2{;

Constraint::Constraint(Solver* solver, uint n, ID id, real_t _scale):ID(id){
	nelem	   = n;
	level	   = 0;
	enabled    = true;
	active     = true;
	scale      = _scale;
	scale2     = scale * scale;
	scale_inv  = 1.0 / scale;
	scale2_inv = scale_inv * scale_inv;
	corrRate   = 0.5;
	corrMax    = FLT_MAX;
	
	solver->AddCon(this);
}

SLink* Constraint::AddSLink(Variable* var, real_t coef){
	SLink* link = new SLink(coef);
	link->Connect(var, this);
	solver->AddLink(link);
	return link;
}

XLink* Constraint::AddXLink(Variable* var){
	XLink* link = new XLink();
	link->Connect(var, this);
	solver->AddLink(link);
	return link;
}

CLink* Constraint::AddCLink(Variable* var){
	CLink* link = new CLink();
	link->Connect(var, this);
	solver->AddLink(link);
	return link;
}

RLink* Constraint::AddRLink(Variable* var){
	RLink* link = new RLink();
	link->Connect(var, this);
	solver->AddLink(link);
	return link;
}

MLink* Constraint::AddMLink(Variable* var){
	MLink* link = new MLink();
	link->Connect(var, this);
	solver->AddLink(link);
	return link;
}

void Constraint::SetPriority(uint newlv){
	level = newlv;
}

void Constraint::CalcError(){
	if(enabled){
		CalcDeviation();
		for(uint k = 0; k < nelem; k++)
			e[k] = 0.5 * y[k] * y[k];
	}
	else{
		y.clear();
		e.clear();
	}
}

void Constraint::CalcDeviation(){
	y.clear();
	for(uint l = 0; l < links.size(); l++)
		links[l]->AddError();
}

void Constraint::ResetState(){
	dy .clear();
	l0 .clear();
	l1 .clear();
	dl0.clear();
	dl1.clear();
}

void Constraint::CalcCorrection(){
	// ƒKƒEƒXƒUƒCƒfƒ‹‚Å—p‚¢‚éJ*J^T‚Ì‘ÎŠp¬•ª = J‚ÌŠes‚Ì“ñæ˜a‚ğŒvZ
	const real_t eps = (real_t)1.0e-10;
	
	J.clear();
	for(uint l = 0; l < links.size(); l++){
		Link* lnk = links[l];
		if(!lnk->var->locked)
			lnk->AddRowSqr(J);
	}
	
	// ‘ÎŠp¬•ª‚Ì‹t”
	for(uint k = 0; k < nelem; k++){
		Jinv[k] = (J[k] > eps ? (real_t)1.0/J[k] : (real_t)0.0);
	}

	// S‘©•Î·y‚ÆS‘©Œë·e‚ÌC³—Ê‚ğİ’è
	// ‚½‚¾‚µC³•‚ÍãŒÀ‚ğ’´‚¦‚È‚¢‚æ‚¤‚É‚·‚é
	dyd = -corrRate * y;
	const real_t dyd_lim = corrMax * scale;
	real_t dyd_max = 0.0;
	for(uint k = 0; k < nelem; k++)
		dyd_max = std::max(dyd_max, std::abs(dyd[k]));
	
	if(dyd_max > dyd_lim)
		dyd *= (dyd_lim / dyd_max);

}

void Constraint::UpdateMultiplier(uint k){
	real_t l0new, l1new;

	// update multiplier
	dl0[k] = -1.0 * Jinv[k] * (dy[k] - dyd[k]);
	l0new  = l0[k] + dl0[k];
	Project(l0new, k);
	dl0[k] = l0new - l0[k];
	l0[k]  = l0new;
	
	// update variable
	for(uint l = 0; l < links.size(); l++)
		links[l]->Backward(k, dl0[k]);
}

void Constraint::UpdateError(uint k, real_t ddy){
	dy[k] += ddy;
}

}
