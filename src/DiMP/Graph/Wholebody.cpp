#include <DiMP/Graph/Wholebody.h>
#include <DiMP/Graph/Graph.h>
#include <DiMP/Render/Config.h>
#include <DiMP/Render/Canvas.h>

#include <omp.h>

#include <sbtimer.h>
static Timer timer;
static Timer timer2;

using namespace PTM;

namespace DiMP {;

const real_t _pi     = M_PI;
const real_t inf     = numeric_limits<real_t>::max();
const real_t damping = 0.1;
const vec3_t one(1.0, 1.0, 1.0);
const real_t eps     = 1.0e-10;

inline mat3_t vvtrmat(vec3_t c, vec3_t r){
	mat3_t m;
	for(int i = 0; i < 3; i++) for(int j = 0; j < 3; j++)
		m[i][j] = c[i]*r[j];
	return m;
}

inline mat3_t RotJacobian(vec3_t omega){
	real_t theta = omega.norm();
	if(theta < eps)
		return mat3_t();

	vec3_t eta = omega/theta;

	mat3_t A = (sin(theta)/theta)*mat3_t() + (1.0 - (sin(theta)/theta))*vvtrmat(eta, eta) + ((cos(theta) - 1.0)/theta)*mat3_t::Cross(eta);
	
	return A;
}

//-------------------------------------------------------------------------------------------------
// IKData

WholebodyData::End::End(){
	pos_t_weight       = one;
	pos_r_weight       = one;
	vel_t_weight       = one;
	vel_r_weight       = one;
	acc_t_weight       = one;
	acc_r_weight       = one;
	force_t_weight     = one;
	force_r_weight     = one;
	
	state  = Wholebody::ContactState::Free;
	mu     = 0.5;
}

WholebodyData::Centroid::Centroid(){
	pos_t_weight = one;
	pos_r_weight = one;
	vel_t_weight = one;
	vel_r_weight = one;
	acc_t_weight = one;
	acc_r_weight = one;
	L_weight     = one;
}

void WholebodyData::Init(Wholebody* wb){
	int nlink  = (int)wb->links .size();
	int nend   = (int)wb->ends  .size();
	int nchain = (int)wb->chains.size();
	int njoint = (int)wb->joints.size();

	links.resize(nlink );
	ends .resize(nend  );
	q    .resize(njoint, 0.0);
	qd   .resize(njoint, 0.0);
	qdd  .resize(njoint, 0.0);
	tau  .resize(njoint, 0.0);

	q_weight  .resize(njoint, 1.0);
	qd_weight .resize(njoint, 1.0);
	qdd_weight.resize(njoint, 1.0);

	q_min.resize(njoint, -inf);
	q_max.resize(njoint,  inf);
}

void WholebodyData::InitJacobian(Wholebody* wb){
	int nlink  = (int)wb->links .size();
	int njoint = (int)wb->joints.size();
		
	Jcom.resize(3, njoint, 0.0);
	Jfk.resize(nlink);
	for(int i = 0; i < nlink; i++){
		Jfk[i].resize(6, njoint, 0.0);
	}
}

void WholebodyData::CopyVars(WholebodyData& d){
	d.centroid = centroid;
	d.ends     = ends;
	d.links    = links;
	d.q        = q;
	d.qd       = qd;
	d.qdd      = qdd;
	d.tau      = tau;
}

//-------------------------------------------------------------------------------------------------
// WholebodyKey

WholebodyKey::WholebodyKey() {
	
}

void WholebodyKey::AddVar(Solver* solver) {
	wb = (Wholebody*)node;

	int nend   = (int)wb->ends  .size();
	int nlink  = (int)wb->links .size();
	int njoint = (int)wb->joints.size();
	
	ends  .resize(nend);
	joints.resize(njoint);

	stringstream ss, ss2;

	// states
	centroid.var_pos_t = new V3Var(solver, ID(VarTag::WholebodyPosT, node, tick, name + "_centroid_pos_t"), wb->scale.pt);
	centroid.var_pos_r = new QVar (solver, ID(VarTag::WholebodyPosR, node, tick, name + "_centroid_pos_r"), wb->scale.pr);
	centroid.var_vel_t = new V3Var(solver, ID(VarTag::WholebodyVelT, node, tick, name + "_centroid_vel_t"), wb->scale.vt);
	centroid.var_vel_r = new V3Var(solver, ID(VarTag::WholebodyVelR, node, tick, name + "_centroid_vel_r"), wb->scale.vr);
	
	for(int i = 0; i < njoint; i++){
		ss.str("");
		ss << name << "_joint" << i;

		joints[i].var_q  = new SVar(solver, ID(VarTag::WholebodyJointPos, node, tick, ss.str() + "_q" ), wb->scale.pr);
		joints[i].var_qd = new SVar(solver, ID(VarTag::WholebodyJointVel, node, tick, ss.str() + "_qd"), wb->scale.vr);
	}
	
	// inputs
	centroid.var_acc_t = new V3Var(solver, ID(VarTag::WholebodyAccT, node, tick, name + "_centroid_acc_t"), wb->scale.at);
	centroid.var_acc_r = new V3Var(solver, ID(VarTag::WholebodyAccR, node, tick, name + "_centroid_acc_r"), wb->scale.ar);

	for(int i = 0; i < njoint; i++){
		ss.str("");
		ss << name << "_joint" << i;
		joints[i].var_qdd = new SVar(solver, ID(VarTag::WholebodyJointAcc, node, tick, ss.str() + "_qdd" ), wb->scale.ar);
	}		
	for(int i = 0; i < nend; i++){
		ss.str("");
		ss << name << "_end" << i;
		ends[i].var_force_t = new V3Var(solver, ID(VarTag::WholebodyForceT, node, tick, ss.str() + "_force_t"), wb->scale.ft);
		ends[i].var_force_r = new V3Var(solver, ID(VarTag::WholebodyForceR, node, tick, ss.str() + "_force_r"), wb->scale.fr);
	}

	solver->AddStateVar(centroid.var_pos_t, tick->idx);
	solver->AddStateVar(centroid.var_pos_r, tick->idx);
	solver->AddStateVar(centroid.var_vel_t, tick->idx);
	solver->AddStateVar(centroid.var_vel_r, tick->idx);
	for(int i = 0; i < njoint; i++){
		solver->AddStateVar(joints[i].var_q , tick->idx);
		solver->AddStateVar(joints[i].var_qd, tick->idx);
	}

	solver->AddInputVar(centroid.var_acc_t, tick->idx);
	solver->AddInputVar(centroid.var_acc_r, tick->idx);
	for(int i = 0; i < njoint; i++){
		solver->AddInputVar(joints[i].var_qdd , tick->idx);	
	}
	for(int i = 0; i < nend; i++){
		solver->AddInputVar(ends[i].var_force_t, tick->idx);			
		solver->AddInputVar(ends[i].var_force_r, tick->idx);
	}

	for(int i = 0; i < nend; i++){
		ends[i].var_force_t->locked = !(wb->ends[i].enableForce);
		ends[i].var_force_r->locked = !(wb->ends[i].enableMoment);
	}

	data.Init(wb);
	data.InitJacobian(wb);
	data_des.Init(wb);
}

void WholebodyKey::AddCon(Solver* solver) {
	WholebodyKey* nextObj = (WholebodyKey*)next;

    int nend   = (int)wb->ends  .size();
	int njoint = (int)wb->joints.size();
	int nchain = (int)wb->chains.size();
	int k = tick->idx;
    
	stringstream ss;

	if(next){
		centroid.con_pos_t = new WholebodyCentroidPosConT(solver, name + "_centroid_pos_t", this, wb->scale.pt);
		centroid.con_pos_r = new WholebodyCentroidPosConR(solver, name + "_centroid_pos_r", this, wb->scale.pr);
		centroid.con_vel_t = new WholebodyCentroidVelConT(solver, name + "_centroid_vel_t", this, wb->scale.vt);
		centroid.con_vel_r = new WholebodyCentroidVelConR(solver, name + "_centroid_vel_r", this, wb->scale.vr);
	}

	centroid.con_des_pos_t = new FixConV3(solver, ID(ConTag::WholebodyPosT, node, tick, name + "_des_centroid_pos_t"), centroid.var_pos_t, wb->scale.pt);
	centroid.con_des_pos_r = new FixConQ (solver, ID(ConTag::WholebodyPosR, node, tick, name + "_des_centroid_pos_r"), centroid.var_pos_r, wb->scale.pr);
	centroid.con_des_vel_t = new FixConV3(solver, ID(ConTag::WholebodyVelT, node, tick, name + "_des_centroid_vel_t"), centroid.var_vel_t, wb->scale.vt);
	centroid.con_des_vel_r = new FixConV3(solver, ID(ConTag::WholebodyVelR, node, tick, name + "_des_centroid_vel_r"), centroid.var_vel_r, wb->scale.vr);
	
	if(next){
		centroid.con_des_acc_t = new FixConV3(solver, ID(ConTag::WholebodyAccT, node, tick, name + "_des_centroid_acc_t"), centroid.var_acc_t, wb->scale.at);
		centroid.con_des_acc_r = new FixConV3(solver, ID(ConTag::WholebodyAccR, node, tick, name + "_des_centroid_acc_r"), centroid.var_acc_r, wb->scale.ar);

		centroid.con_L = new WholebodyLCon(solver, name + "_L", this, wb->scale.L);
	}

	for(int i = 0; i < njoint; i++){
		ss.str("");
		ss << name << "_joint" << i;

		if(next){
			joints[i].con_q  = new WholebodyJointPosCon(solver, ss.str() + "_q" , this, i, wb->scale.pr);
			joints[i].con_qd = new WholebodyJointVelCon(solver, ss.str() + "_qd", this, i, wb->scale.vr);        
		}
			
		joints[i].con_des_q  = new FixConS(solver, ID(ConTag::WholebodyJointPos, node, tick, ss.str() + "_des_q" ), joints[i].var_q , wb->scale.pr);
		joints[i].con_des_qd = new FixConS(solver, ID(ConTag::WholebodyJointVel, node, tick, ss.str() + "_des_qd"), joints[i].var_qd, wb->scale.vr);
		if(next){
			joints[i].con_des_qdd = new FixConS(solver, ID(ConTag::WholebodyJointAcc, node, tick, ss.str() + "_des_qdd"), joints[i].var_qdd, wb->scale.ar);
		}

		joints[i].con_range_q = new RangeConS(solver, ID(ConTag::WholebodyJointPos, node, tick, ss.str() + "_range_q"), joints[i].var_q, wb->scale.pr);
	}
	
	for(int i = 0; i < nend; i++){
		ss.str("");
		ss << name << "_end" << i;

		ends[i].con_des_pos_t   = new WholebodyDesPosConT(solver, ss.str() + "_des_pos_t", this, i, wb->scale.pt);	
		ends[i].con_des_pos_r   = new WholebodyDesPosConR(solver, ss.str() + "_des_pos_r", this, i, wb->scale.pr);
		ends[i].con_des_vel_t   = new WholebodyDesVelConT(solver, ss.str() + "_des_vel_t", this, i, wb->scale.vt);
		ends[i].con_des_vel_r   = new WholebodyDesVelConR(solver, ss.str() + "_des_vel_r", this, i, wb->scale.vr);
		
		if(next){
			ends[i].con_force_normal = new WholebodyNormalForceCon(solver, ss.str() + "_force_normal", this, i, wb->scale.ft);

			ends[i].con_force_friction[0][0] = new WholebodyFrictionForceCon(solver, ss.str() + "_force_friction", this, i, 0, 0, wb->scale.ft);
			ends[i].con_force_friction[0][1] = new WholebodyFrictionForceCon(solver, ss.str() + "_force_friction", this, i, 0, 1, wb->scale.ft);
			ends[i].con_force_friction[1][0] = new WholebodyFrictionForceCon(solver, ss.str() + "_force_friction", this, i, 1, 0, wb->scale.ft);
			ends[i].con_force_friction[1][1] = new WholebodyFrictionForceCon(solver, ss.str() + "_force_friction", this, i, 1, 1, wb->scale.ft);

			ends[i].con_moment[0][0] = new WholebodyMomentCon(solver, ss.str() + "_moment", this, i, 0, 0, wb->scale.fr);
			ends[i].con_moment[0][1] = new WholebodyMomentCon(solver, ss.str() + "_moment", this, i, 0, 1, wb->scale.fr);
			ends[i].con_moment[1][0] = new WholebodyMomentCon(solver, ss.str() + "_moment", this, i, 1, 0, wb->scale.fr);
			ends[i].con_moment[1][1] = new WholebodyMomentCon(solver, ss.str() + "_moment", this, i, 1, 1, wb->scale.fr);
			ends[i].con_moment[2][0] = new WholebodyMomentCon(solver, ss.str() + "_moment", this, i, 2, 0, wb->scale.fr);
			ends[i].con_moment[2][1] = new WholebodyMomentCon(solver, ss.str() + "_moment", this, i, 2, 1, wb->scale.fr);
		}

		ends[i].con_contact_pos_t = new WholebodyContactPosConT(solver, ss.str() + "_contact_pos_t", this, i, wb->scale.pt);
		ends[i].con_contact_pos_r = new WholebodyContactPosConR(solver, ss.str() + "_contact_pos_r", this, i, wb->scale.pr);
		ends[i].con_contact_vel_t = new WholebodyContactVelConT(solver, ss.str() + "_contact_vel_t", this, i, wb->scale.vt);
		ends[i].con_contact_vel_r = new WholebodyContactVelConR(solver, ss.str() + "_contact_vel_r", this, i, wb->scale.vr);
		
		if(next){
			ends[i].con_des_force_t = new FixConV3(solver, ID(ConTag::WholebodyForceT, node, tick, ss.str() + "_des_force_t"), ends[i].var_force_t, wb->scale.ft);
			ends[i].con_des_force_r = new FixConV3(solver, ID(ConTag::WholebodyForceR, node, tick, ss.str() + "_des_force_r"), ends[i].var_force_r, wb->scale.fr);		
		}
	}

	if(next){
		solver->AddTransitionCon(centroid.con_pos_t, tick->idx);
		solver->AddTransitionCon(centroid.con_pos_r, tick->idx);
		solver->AddTransitionCon(centroid.con_vel_t, tick->idx);
		solver->AddTransitionCon(centroid.con_vel_r, tick->idx);
	}
	solver->AddCostCon(centroid.con_des_pos_t, tick->idx);
	solver->AddCostCon(centroid.con_des_pos_r, tick->idx);
	solver->AddCostCon(centroid.con_des_vel_t, tick->idx);
	solver->AddCostCon(centroid.con_des_vel_r, tick->idx);

	if(next){
		solver->AddCostCon(centroid.con_des_acc_t, tick->idx);
		solver->AddCostCon(centroid.con_des_acc_r, tick->idx);

		solver->AddCostCon(centroid.con_L, tick->idx);
	}
	for(int i = 0; i < njoint; i++){
		if(next){
			solver->AddTransitionCon(joints[i].con_q , tick->idx);
			solver->AddTransitionCon(joints[i].con_qd, tick->idx);
		}
		solver->AddCostCon(joints[i].con_des_q , tick->idx);
		solver->AddCostCon(joints[i].con_des_qd, tick->idx);
		if(next){
			solver->AddCostCon(joints[i].con_des_qdd, tick->idx);
		}
		solver->AddCostCon(joints[i].con_range_q, tick->idx);
	}
	for(int i = 0; i < nend; i++){
		solver->AddCostCon(ends[i].con_des_pos_t  , tick->idx);
		solver->AddCostCon(ends[i].con_des_pos_r  , tick->idx);
		solver->AddCostCon(ends[i].con_des_vel_t  , tick->idx);
		solver->AddCostCon(ends[i].con_des_vel_r  , tick->idx);

		if(next){
			solver->AddCostCon(ends[i].con_force_normal, tick->idx);
			solver->AddCostCon(ends[i].con_force_friction[0][0], tick->idx);
			solver->AddCostCon(ends[i].con_force_friction[0][1], tick->idx);
			solver->AddCostCon(ends[i].con_force_friction[1][0], tick->idx);
			solver->AddCostCon(ends[i].con_force_friction[1][1], tick->idx);
			solver->AddCostCon(ends[i].con_moment[0][0], tick->idx);
			solver->AddCostCon(ends[i].con_moment[0][1], tick->idx);
			solver->AddCostCon(ends[i].con_moment[1][0], tick->idx);
			solver->AddCostCon(ends[i].con_moment[1][1], tick->idx);
			solver->AddCostCon(ends[i].con_moment[2][0], tick->idx);
			solver->AddCostCon(ends[i].con_moment[2][1], tick->idx);
		}
		solver->AddCostCon(ends[i].con_contact_pos_t, tick->idx);
		solver->AddCostCon(ends[i].con_contact_pos_r, tick->idx);
		solver->AddCostCon(ends[i].con_contact_vel_t, tick->idx);
		solver->AddCostCon(ends[i].con_contact_vel_r, tick->idx);
		if(next){
			solver->AddCostCon(ends[i].con_des_force_t, tick->idx);
			solver->AddCostCon(ends[i].con_des_force_r, tick->idx);
		}
	}

	for(int i = 0; i < nend; i++){
		ends[i].con_des_pos_t->enabled = (wb->ends[i].enableTranslation);
		ends[i].con_des_pos_r->enabled = (wb->ends[i].enableRotation   );
		ends[i].con_des_vel_t->enabled = (wb->ends[i].enableTranslation);
		ends[i].con_des_vel_r->enabled = (wb->ends[i].enableRotation   );

		if(next){
			ends[i].con_force_normal->enabled = (wb->ends[i].enableForce);
			ends[i].con_force_friction[0][0]->enabled = (wb->ends[i].enableForce);
			ends[i].con_force_friction[0][1]->enabled = (wb->ends[i].enableForce);
			ends[i].con_force_friction[1][0]->enabled = (wb->ends[i].enableForce);
			ends[i].con_force_friction[1][1]->enabled = (wb->ends[i].enableForce);
			ends[i].con_moment[0][0]->enabled = (wb->ends[i].enableForce && wb->ends[i].enableMoment);
			ends[i].con_moment[0][1]->enabled = (wb->ends[i].enableForce && wb->ends[i].enableMoment);
			ends[i].con_moment[1][0]->enabled = (wb->ends[i].enableForce && wb->ends[i].enableMoment);
			ends[i].con_moment[1][1]->enabled = (wb->ends[i].enableForce && wb->ends[i].enableMoment);
			ends[i].con_moment[2][0]->enabled = (wb->ends[i].enableForce && wb->ends[i].enableMoment);
			ends[i].con_moment[2][1]->enabled = (wb->ends[i].enableForce && wb->ends[i].enableMoment);
			ends[i].con_des_force_t->enabled = (wb->ends[i].enableForce );
			ends[i].con_des_force_r->enabled = (wb->ends[i].enableMoment);
		}
	}
}

void WholebodyKey::Prepare() {
	int nlink  = (int)wb->links .size();
	int nend   = (int)wb->ends  .size();
	int njoint = (int)wb->joints.size();

	// copy variables to data
	data.centroid.pos_t = centroid.var_pos_t->val;
	data.centroid.pos_r = centroid.var_pos_r->val;
	data.centroid.vel_t = centroid.var_vel_t->val;
	data.centroid.vel_r = centroid.var_vel_r->val;

	for(int i = 0; i < njoint; i++){
		data.q [i] = joints[i].var_q ->val;
		data.qd[i] = joints[i].var_qd->val;

		if(next){
			data.qdd[i] = joints[i].var_qdd->val;
		}
	}
	
	for(int i = 0; i < nend; i++){
		End& end = ends[i];
		WholebodyData::End & dend = data.ends [i];
		
		if(next){
			dend.force_t = end.var_force_t->val;
			dend.force_r = end.var_force_r->val;
		}
	}

	wb->CalcPosition          (data);
	wb->CalcJacobian          (data);
	wb->CalcVelocity          (data);
	wb->CalcAcceleration      (data);
	wb->CalcMomentum          (data);
	wb->CalcMomentumDerivative(data);
	//wb->CalcForce             (data);

	// working variables
	if(next){
		q0 = centroid.var_pos_r->val;
		q0.ToMatrix(R0);

		fe .resize(nend);
		me .resize(nend);
		re .resize(nend);
		rec.resize(nend);
		
		J_L_q   .resize(3, njoint, 0.0);
		J_L_qd  .resize(3, njoint, 0.0);
		J_Ld_q  .resize(3, njoint, 0.0);
		J_Ld_qdd.resize(3, njoint, 0.0);
		
		fsum.clear();
		msum.clear();
		for(int i = 0; i < nend; i++){
			WholebodyData::End& dend = data.ends[i];
			
			re [i] = data.centroid.pos_r*dend.pos_t;
			rec[i] = mat3_t::Cross(re[i]);
			
			fe[i] = dend.force_t;
			me[i] = dend.force_r;
			
			fsum += fe[i];
			msum += me[i] + re[i] % fe[i];
		}

		J_L_q   .clear();
		J_L_qd  .clear();
		J_Ld_q  .clear();
		J_Ld_qdd.clear();
		for(int i = 0; i < njoint; i++){
			for(int j = 0; j < nlink; j++){
				WholebodyData::Link& dlnk = data.links[j];

				real_t mj = wb->links[j].mass;
				mat3_t mj_ajc = mj*mat3_t::Cross(dlnk.acc_t);
				mat3_t mj_vjc = mj*mat3_t::Cross(dlnk.vel_t);
				mat3_t mj_pjc = mj*mat3_t::Cross(dlnk.pos_t);
				mat3_t Ij     = dlnk.I;
				vec3_t Jv = data.Jfk[j].col(i).sub_vector(TSubVectorDim<0,3>());
				vec3_t Jw = data.Jfk[j].col(i).sub_vector(TSubVectorDim<3,3>());
				J_L_q   .col(i) -= mj_vjc*Jv;
				J_L_qd  .col(i) += mj_pjc*Jv + Ij*Jw;
				J_Ld_q  .col(i) -= mj_ajc*Jv;
				J_Ld_qdd.col(i) += mj_pjc*Jv + Ij*Jw;
			}
		}
	}
}

void WholebodyKey::PrepareStep(){
}

void WholebodyKey::Finish(){
}

void WholebodyKey::Draw(Render::Canvas* canvas, Render::Config* conf) {
}

//-------------------------------------------------------------------------------------------------
// Wholebody

Wholebody::Param::Param() {
	gravity            = 9.8;
}

//-------------------------------------------------------------------------------------------------

Wholebody::Link::Link(real_t _mass, vec3_t _inertia, int _iend, int _iparent, int _ijoint, vec3_t _trn, vec3_t _axis){
	mass    = _mass;
	iend    = _iend;
	iparent = _iparent;
	ijoint  = _ijoint;
	trn     = _trn;
	axis    = _axis;

	inertia.clear();
	inertia[0][0] = _inertia[0];
	inertia[1][1] = _inertia[1];
	inertia[2][2] = _inertia[2];
}

//-------------------------------------------------------------------------------------------------

Wholebody::Joint::Joint(real_t Ir){
	rotor_inertia = Ir;
}

//-------------------------------------------------------------------------------------------------

Wholebody::End::End(int _ilink, vec3_t _offset, bool _enable_trn, bool _enable_rot, bool _enable_force, bool _enable_moment){
	ilink             = _ilink;
	offset            = _offset;
	enableTranslation = _enable_trn;
	enableRotation    = _enable_rot;
	enableForce       = _enable_force;
	enableMoment      = _enable_moment;
}

//-------------------------------------------------------------------------------------------------

Wholebody::Chain::Chain(const vector<int>& _ilink){
	ilink  = _ilink;
}

//-------------------------------------------------------------------------------------------------

Wholebody::Snapshot::Snapshot() {

}

//-------------------------------------------------------------------------------------------------

Wholebody::Wholebody(Graph* g, string n) :TrajectoryNode(g, n) {
	type = Type::Object;
	graph->wholebodies.Add(this);
}

Wholebody::~Wholebody() {
	graph->wholebodies.Remove(this);
}

void Wholebody::SetScaling(){
	// calc scaling constants
	param.totalMass = 0.0;
	for(int i = 0; i < links.size(); i++){
		param.totalMass += links[i].mass;
	}
	for(int i = 0; i < links.size(); i++){
		links[i].mass_ratio = links[i].mass/param.totalMass;
	}

	//scale.l    = 1.0;  //< unit length
	scale.l    = sqrt(param.nominalInertia[0]/(0.4*param.totalMass));
	//scale.t    = sqrt(scale.l/param.g);
	scale.t    = graph->ticks[1]->time - graph->ticks[0]->time;
	scale.tinv = 1.0/scale.t;
	scale.at   = param.gravity;
	scale.vt   = scale.at*scale.t;
	scale.ft   = param.totalMass*param.gravity;
	scale.pt   = scale.vt*scale.t;
	scale.pr   = scale.pt/scale.l;
	scale.vr   = scale.vt/scale.l;
	scale.ar   = scale.at/scale.l;
	scale.fr   = scale.ft*scale.l;
	scale.L    = scale.fr*scale.t;
	/*
	scale.l    = 1.0;  //< unit length
	scale.t    = 1.0;
	scale.tinv = 1.0;
	scale.at   = 1.0;
	scale.vt   = 1.0;
	scale.ft   = 1.0;
	scale.pt   = 1.0;
	scale.pr   = 1.0;
	scale.vr   = 1.0;
	scale.ar   = 1.0;
	scale.fr   = 1.0;
	scale.L    = 1.0;
	*/
}

void Wholebody::Reset(){
	int nend   = (int)ends  .size();
	int njoint = (int)joints.size();
	int N = (int)graph->ticks.size()-1;

	for (int k = 0; k <= N; k++) {
		WholebodyKey* key = (WholebodyKey*)traj.GetKeypoint(graph->ticks[k]);
		WholebodyData& d = key->data_des;

		key->centroid.var_pos_t->val = d.centroid.pos_t;
		key->centroid.var_pos_r->val = d.centroid.pos_r;
		key->centroid.var_vel_t->val = d.centroid.vel_t;
		key->centroid.var_vel_r->val = d.centroid.vel_r;
		key->centroid.var_acc_t->val.clear();
		key->centroid.var_acc_r->val.clear();

		for(int i = 0; i < njoint; i++){
			WholebodyKey::Joint& jnt = key->joints[i];

			jnt.var_q ->val = d.q [i];
			jnt.var_qd->val = d.qd[i];
			if(key->next){
				jnt.var_qdd->val = d.qdd[i];
			}
		}

		for(int i = 0; i < nend; i++){
			WholebodyKey ::End&  end  = key->ends[i];
			WholebodyData::End&  dend = d.ends [i];
			
			if(key->next){
				end.var_force_t->val = dend.force_t;
				end.var_force_r->val = dend.force_r;
			}
		}
	}

	trajReady = false;
}

void Wholebody::Shift(){
	int nend   = (int)ends  .size();
	int njoint = (int)joints.size();
	int N = (int)graph->ticks.size()-1;

	for (int k = 0; k < N; k++) {
		WholebodyKey* key0 = (WholebodyKey*)traj.GetKeypoint(graph->ticks[k+0]);
		WholebodyKey* key1 = (WholebodyKey*)traj.GetKeypoint(graph->ticks[k+1]);
		
		key0->centroid.var_pos_t->val = key1->centroid.var_pos_t->val;
		key0->centroid.var_pos_r->val = key1->centroid.var_pos_r->val;
		key0->centroid.var_vel_t->val = key1->centroid.var_vel_t->val;
		key0->centroid.var_vel_r->val = key1->centroid.var_vel_r->val;
		if(key1->next){
			key0->centroid.var_acc_t->val = key1->centroid.var_acc_t->val;
			key0->centroid.var_acc_r->val = key1->centroid.var_acc_r->val;
		}

		for(int i = 0; i < njoint; i++){
			WholebodyKey::Joint& jnt0 = key0->joints[i];
			WholebodyKey::Joint& jnt1 = key1->joints[i];

			jnt0.var_q ->val = jnt1.var_q ->val;
			jnt0.var_qd->val = jnt1.var_qd->val;
				
			if(key1->next){
				jnt0.var_qdd->val = jnt1.var_qdd->val;
			}
		}
		
		for(int i = 0; i < nend; i++){
			WholebodyKey ::End&  end0 = key0->ends[i];
			WholebodyKey ::End&  end1 = key1->ends[i];
			
			if(key1->next){
				end0.var_force_t->val = end1.var_force_t->val;
				end0.var_force_r->val = end1.var_force_r->val;
			}
		}
	}
}

void Wholebody::Setup(){
	int nend   = (int)ends  .size();
	int njoint = (int)joints.size();
	int N = (int)graph->ticks.size()-1;

	for (int k = 0; k <= N; k++) {
		WholebodyKey* key = (WholebodyKey*)traj.GetKeypoint(graph->ticks[k]);
		WholebodyData& d = key->data_des;

		real_t t = graph->ticks[k]->time;

		// need to get contact state as desired state
		callback->GetDesiredState(k, t, d);

		if(k == 0){
			callback->GetInitialState(d);
		
			key->centroid.var_pos_t->val = d.centroid.pos_t;
			key->centroid.var_pos_r->val = d.centroid.pos_r;
			key->centroid.var_vel_t->val = d.centroid.vel_t;
			key->centroid.var_vel_r->val = d.centroid.vel_r;
			
			for(int i = 0; i < njoint; i++){
				WholebodyKey::Joint& jnt = key->joints[i];

				jnt.var_q  ->val = d.q  [i];
				jnt.var_qd ->val = d.qd [i];
				jnt.var_qdd->val = d.qdd[i];
			}

			for(int i = 0; i < nend; i++){
				WholebodyKey ::End&  end  = key->ends[i];
				WholebodyData::End&  dend = d.ends [i];

				end.var_force_t->val = dend.force_t;
				end.var_force_r->val = dend.force_r;
			}
		}

		key->centroid.con_des_pos_t->desired = d.centroid.pos_t;
		key->centroid.con_des_pos_r->desired = d.centroid.pos_r;
		key->centroid.con_des_vel_t->desired = d.centroid.vel_t;
		key->centroid.con_des_vel_r->desired = d.centroid.vel_r;
		key->centroid.con_des_pos_t->weight  = d.centroid.pos_t_weight;
		key->centroid.con_des_pos_r->weight  = d.centroid.pos_r_weight;
		key->centroid.con_des_vel_t->weight  = d.centroid.vel_t_weight;
		key->centroid.con_des_vel_r->weight  = d.centroid.vel_r_weight;

		if(key->next){
			key->centroid.con_des_acc_t->desired.clear();
			key->centroid.con_des_acc_r->desired.clear();
			key->centroid.con_des_acc_t->weight  = d.centroid.acc_t_weight;
			key->centroid.con_des_acc_r->weight  = d.centroid.acc_r_weight;

			key->centroid.con_L->desired = d.centroid.Labs;
			key->centroid.con_L->weight  = d.centroid.L_weight;
		}

		for(int i = 0; i < njoint; i++){
			WholebodyKey::Joint& jnt = key->joints[i];
			
			jnt.con_des_q ->desired = d.q [i];
			jnt.con_des_qd->desired = d.qd[i];
			jnt.con_des_q ->weight[0] = d.q_weight [i];
			jnt.con_des_qd->weight[0] = d.qd_weight[i];

			jnt.con_range_q->_min = d.q_min[i];
			jnt.con_range_q->_max = d.q_max[i];
			
			if(key->next){
				jnt.con_des_qdd->desired = d.qdd[i];
				jnt.con_des_qdd->weight[0] = d.qdd_weight[i];
			}
		}

		for(int i = 0; i < nend; i++){
			WholebodyKey ::End&  end  = key->ends[i];
			WholebodyData::End&  dend = d.ends [i];

			key->data.ends[i].state = d.ends[i].state;

			// transform to absolute coordinate
			end.con_des_pos_t->desired = d.centroid.pos_t + d.centroid.pos_r*dend.pos_t;
			end.con_des_vel_t->desired = d.centroid.vel_t + d.centroid.vel_r%(d.centroid.pos_r*dend.pos_t) + d.centroid.pos_r*dend.vel_t;
			end.con_des_pos_t->weight  = dend.pos_t_weight;
			end.con_des_vel_t->weight  = dend.vel_t_weight;
				
			end.con_des_pos_r->desired = d.centroid.pos_r*dend.pos_r;
			end.con_des_vel_r->desired = d.centroid.vel_r + d.centroid.pos_r*dend.vel_r;
			end.con_des_pos_r->weight  = dend.pos_r_weight;
			end.con_des_vel_r->weight  = dend.vel_r_weight;
				
			if(dend.state == ContactState::Free){
				end.con_contact_pos_t->weight = vec3_t(0.0, 0.0, 0.0);
				end.con_contact_vel_t->weight = vec3_t(0.0, 0.0, 0.0);				
				end.con_contact_pos_r->weight = vec3_t(0.0, 0.0, 0.0);
				end.con_contact_vel_r->weight = vec3_t(0.0, 0.0, 0.0);
			}
			if(dend.state == ContactState::Surface){
				end.con_contact_pos_t->weight = vec3_t(0.0, 0.0, 1.0);
				end.con_contact_vel_t->weight = vec3_t(0.0, 0.0, 1.0);
				end.con_contact_pos_r->weight = vec3_t(1.0, 1.0, 0.0);
				end.con_contact_vel_r->weight = vec3_t(1.0, 1.0, 1.0);
			}
			if(dend.state == ContactState::Line){
				end.con_contact_pos_t->weight = vec3_t(0.0, 0.0, 1.0);
				end.con_contact_vel_t->weight = vec3_t(0.0, 0.0, 1.0);
				end.con_contact_pos_r->weight = vec3_t(0.0, 0.0, 0.0);
				end.con_contact_vel_r->weight = vec3_t(0.0, 0.0, 1.0);
			}
			if(dend.state == ContactState::Point){
				end.con_contact_pos_t->weight = vec3_t(0.0, 0.0, 1.0);
				end.con_contact_vel_t->weight = vec3_t(1.0, 1.0, 1.0);
				end.con_contact_pos_r->weight = vec3_t(0.0, 0.0, 0.0);
				end.con_contact_vel_r->weight = vec3_t(0.0, 0.0, 0.0);
			}
	
			if(key->next){
				end.con_des_force_t->desired = dend.force_t;
				end.con_des_force_r->desired = dend.force_r;
				end.con_des_force_t->weight = dend.force_t_weight;
				end.con_des_force_r->weight = dend.force_r_weight;

				end.con_force_normal->active  = (dend.state != ContactState::Free);
				end.con_force_normal->weight[0] = 10.0;
				end.con_force_normal->barrier_margin = 0.00001;

				end.con_force_friction[0][0]->active = (dend.state != ContactState::Free);
				end.con_force_friction[0][1]->active = (dend.state != ContactState::Free);
				end.con_force_friction[1][0]->active = (dend.state != ContactState::Free);
				end.con_force_friction[1][1]->active = (dend.state != ContactState::Free);
				end.con_force_friction[0][0]->weight[0] = 10.0;
				end.con_force_friction[0][1]->weight[0] = 10.0;
				end.con_force_friction[1][0]->weight[0] = 10.0;
				end.con_force_friction[1][1]->weight[0] = 10.0;
				end.con_force_friction[0][0]->barrier_margin = 0.00001;
				end.con_force_friction[0][1]->barrier_margin = 0.00001;
				end.con_force_friction[1][0]->barrier_margin = 0.00001;
				end.con_force_friction[1][1]->barrier_margin = 0.00001;
				
				end.con_moment[0][0]->active = (dend.state != ContactState::Free);
				end.con_moment[0][1]->active = (dend.state != ContactState::Free);
				end.con_moment[1][0]->active = (dend.state != ContactState::Free);
				end.con_moment[1][1]->active = (dend.state != ContactState::Free);
				end.con_moment[2][0]->active = (dend.state != ContactState::Free);
				end.con_moment[2][1]->active = (dend.state != ContactState::Free);
				end.con_moment[0][0]->weight[0] = 10.0;
				end.con_moment[0][1]->weight[0] = 10.0;
				end.con_moment[1][0]->weight[0] = 10.0;
				end.con_moment[1][1]->weight[0] = 10.0;
				end.con_moment[2][0]->weight[0] = 10.0;
				end.con_moment[2][1]->weight[0] = 10.0;
				end.con_moment[0][0]->barrier_margin = 0.00001;
				end.con_moment[0][1]->barrier_margin = 0.00001;
				end.con_moment[1][0]->barrier_margin = 0.00001;
				end.con_moment[1][1]->barrier_margin = 0.00001;
				end.con_moment[2][0]->barrier_margin = 0.00001;
				end.con_moment[2][1]->barrier_margin = 0.00001;
			}
		}
	}

	trajReady = false;
}

void Wholebody::Init() {
	TrajectoryNode::Init();

	int nlink  = (int)links .size();
	int nend   = (int)ends  .size();
	int nchain = (int)chains.size();
	
	for(int i = 0; i < nlink; i++){
		if(links[i].iparent != -1)
			links[links[i].iparent].ichildren.push_back(i);
	}
	
	// call prepare here so that initial trajectory is visualized properly
	Setup  ();
	Reset  ();
    Prepare();

    trajReady = false;
}

void Wholebody::Prepare() {
	trajReady = false;

	traj.Update();
	
	#pragma omp parallel for if(graph->solver->param.parallelize)
	for(int k = 0; k < traj.size(); k++){
		traj[k]->Prepare();
	}
	//TrajectoryNode::Prepare();
}

void Wholebody::PrepareStep(){
	#pragma omp parallel for if(graph->solver->param.parallelize)
	for(int k = 0; k < traj.size(); k++){
		traj[k]->PrepareStep();
	}
	//TrajectoryNode::PrepareStep();
}

void Wholebody::Finish(){
	TrajectoryNode::Finish();
}

void Wholebody::CalcFK(WholebodyData& d, int ichain, bool calc_end){
	Chain& ch = chains[ichain];

	// calc fk
	for(int i : ch.ilink){
		int ip = links[i].iparent;
		WholebodyData::Link& dlnk  = d.links[i ];
		WholebodyData::Link& dlnkp = d.links[ip];

		dlnk.pos_t = dlnkp.pos_t + dlnkp.pos_r*links[i].trn;
		if(links[i].ijoint != -1)
			 dlnk.pos_r = dlnkp.pos_r*quat_t::Rot(d.q[links[i].ijoint], links[i].axis);
		else dlnk.pos_r = dlnkp.pos_r;
		dlnk.pos_r.unitize();
	}

	// calc end pose
	if(calc_end){
		WholebodyData::Link& dlnk = d.links[ch.ilink.back()];
		WholebodyData::End&  dend = d.ends[ichain];
		dend.pos_t = dlnk.pos_t + dlnk.pos_r*ends[ichain].offset;
		dend.pos_r = dlnk.pos_r;
	}
}

void Wholebody::CalcPosition(WholebodyData& d){
	timer2.CountUS();
	int nlink  = (int)links.size();
	int nend   = (int)ends.size();
	int nchain = (int)chains.size();

	d.links[0].pos_t = vec3_t();
	d.links[0].pos_r = quat_t();

	vec3_t pc;

	// one iteration is enough in the case of FK
	for(int ic = 0; ic < nchain; ic++)
		CalcFK(d, ic, false);
		
	pc.clear();
	for(int i = 0; i < nlink; i++){
		pc += links[i].mass_ratio*d.links[i].pos_t;
	}
		
	d.links[0].pos_t = -pc;
	
	for(int ic = 0; ic < nchain; ic++)
		CalcFK(d, ic, true);
	
	int T = timer2.CountUS();
}

void Wholebody::CalcComAcceleration (WholebodyData& d){
	vec3_t fsum;
	
	int nend = (int)ends.size();
    for(int i = 0; i < nend; i++){
        WholebodyData::End& dend = d.ends[i];
        fsum += dend.force_t;
    }
    d.centroid.acc_t = (1.0/param.totalMass)*fsum - vec3_t(0.0, 0.0, param.gravity);
}

void Wholebody::CalcBaseAcceleration(WholebodyData& d){
	vec3_t msum;
	
	int nend = (int)ends.size();
    for(int i = 0; i < nend; i++){
        WholebodyData::End& dend = d.ends[i];
        msum += dend.force_r + (d.centroid.pos_r*dend.pos_t) % dend.force_t;
    }
    d.centroid.acc_r = d.centroid.Iinv*(msum - d.centroid.pos_r*d.centroid.Ld);
}

inline mat6_t Xmat(vec3_t r){
	mat6_t X;
	X.sub_matrix(TSubMatrixDim<0,0,3,3>()) =  mat3_t();
	X.sub_matrix(TSubMatrixDim<0,3,3,3>()) = -mat3_t::Cross(r);
	X.sub_matrix(TSubMatrixDim<3,0,3,3>()).clear();
	X.sub_matrix(TSubMatrixDim<3,3,3,3>()) =  mat3_t();
	return X;
}

void Wholebody::CalcJacobian(WholebodyData& d){
	int nlink  = (int)links .size();
	int nend   = (int)ends  .size();
	int nchain = (int)chains.size();
	int njoint = (int)joints.size();
	
	// clear
	for(int i = 0; i < nlink; i++){
		d.Jfk[i].clear();
	}
	
	for(int ic = 0; ic < nchain; ic++){
		CalcFK(d, ic, false);
	}

	for(int ic = 0; ic < nchain; ic++){
		Chain& ch = chains[ic];
		int ir = links[ch.ilink[0]].iparent;
		int ie = ends[ic].ilink;
		int nq = (int)ch.ilink .size();

		vec3_t pr = d.links[ir].pos_t;

		for(int j = 0; j < ch.ilink.size(); j++){
			int i = ch.ilink[j];

			vec3_t pi = d.links[i].pos_t;
			quat_t qi = d.links[i].pos_r;

			if(ir != 0){
				mat6_t Xir = Xmat(pi - pr);
				d.Jfk[i] = Xir*d.Jfk[ir];
			}

			for(int j0 = 0; j0 <= j; j0++){
				int i0 = ch.ilink[j0];
				int iq = links[i0].ijoint;

				if(iq != -1){
					vec3_t pi0 = d.links[i0].pos_t;
					quat_t qi0 = d.links[i0].pos_r;

					vec3_t eta = qi0*links[i0].axis;
					vec3_t r   = pi - pi0;
				
					d.Jfk[i].col(iq).v_range(0,3) = eta % r;
					d.Jfk[i].col(iq).v_range(3,3) = eta;
				}
			}
		}
	}

	d.Jcom.clear();
	for(int i = 1; i < nlink; i++){
		d.Jcom += links[i].mass_ratio*d.Jfk[i].vsub_matrix(0,0,3,njoint);
	}
	for(int i = 0; i < nlink; i++){
		d.Jfk[i].vsub_matrix(0,0,3,njoint) -= d.Jcom;
	}
}

void Wholebody::CalcVelocity(WholebodyData& d){
	int nlink  = (int)links .size();
	int nend   = (int)ends  .size();
	int njoint = (int)joints.size();

	d.links[0].vel_t.clear();
	d.links[0].vel_r.clear();
	/*	
	for(Chain& ch : chains){
		for(int i : ch.ilink){
			int ip = links[i].iparent;
			WholebodyData::Link& dlnk  = d.links[i];
			WholebodyData::Link& dlnkp = d.links[ip];

			dlnk.vel_t = dlnkp.vel_t + dlnkp.vel_r % (dlnkp.pos_r*links[i].trn);
			dlnk.vel_r = dlnkp.vel_r + dlnkp.pos_r*(links[i].axis*d.qd[links[i].ijoint]);
		}
	}

	// calc com velocity
	vec3_t vc;
	for(int i = 0; i < nlink; i++){
		vc += links[i].mass_ratio*d.links[i].vel_t;
	}
		
	// subtract calculated com vel
	for(int i = 0; i < nlink; i++){
		d.links[i].vel_t -= vc;
	}
	*/
	vec6_t vi;
	for(int i = 0; i < nlink; i++){
		vi.clear();
		for(int j = 0; j < njoint; j++){
			vi += d.Jfk[i].col(j)*d.qd[j];
		}
		d.links[i].vel_t = vi.sub_vector(TSubVectorDim<0,3>());
		d.links[i].vel_r = vi.sub_vector(TSubVectorDim<3,3>());
	}

	// calc end vel
	for(int iend = 0; iend < nend; iend++){
		int i = ends[iend].ilink;
		d.ends[iend].vel_t = d.links[i].vel_t + d.links[i].vel_r%(d.links[i].pos_r*ends[iend].offset);
		d.ends[iend].vel_r = d.links[i].vel_r;
	}
}

void Wholebody::CalcAcceleration(WholebodyData& d){
	int nlink  = (int)links.size();
	int nend   = (int)ends .size();
	int njoint = (int)joints.size();

	d.links[0].acc_t.clear();
	d.links[0].acc_r.clear();
	
	for(Chain& ch : chains){
		for(int i : ch.ilink){
			int ip = links[i].iparent;
			WholebodyData::Link& dlnk  = d.links[i];
			WholebodyData::Link& dlnkp = d.links[ip];

			dlnk.acc_t = dlnkp.acc_t + dlnkp.acc_r % (dlnkp.pos_r*links[i].trn) + dlnkp.vel_r % (dlnkp.vel_r % (dlnkp.pos_r*links[i].trn));
			dlnk.acc_r = dlnkp.acc_r + dlnkp.pos_r*(links[i].axis*d.qdd[links[i].ijoint]) + dlnkp.vel_r % (dlnkp.pos_r*(links[i].axis*d.qd[links[i].ijoint]));
		}
	}

	// calc com velocity
	vec3_t ac;
	for(int i = 0; i < nlink; i++){
		ac += links[i].mass_ratio*d.links[i].acc_t;
	}
		
	// subtract calculated com vel
	for(int i = 0; i < nlink; i++){
		d.links[i].acc_t -= ac;
	}
	
	/*vec6_t ai;
	for(int i = 0; i < nlink; i++){
		ai.clear();
		for(int j = 0; j < njoint; j++){
			ai += d.Jfk[i].col(j)*d.qdd[j];
		}
		d.links[i].acc_t = ai.sub_vector(TSubVectorDim<0,3>());
		d.links[i].acc_r = ai.sub_vector(TSubVectorDim<3,3>());
	}*/

	// calc end acc
	for(int iend = 0; iend < nend; iend++){
		int i = ends[iend].ilink;
		d.ends[iend].acc_t = d.links[i].acc_t + d.links[i].acc_r % (d.links[i].pos_r*ends[iend].offset) + d.links[i].vel_r % (d.links[i].vel_r % (d.links[i].pos_r*ends[iend].offset));
		d.ends[iend].acc_r = d.links[i].acc_r;
	}
}

void Wholebody::CalcMomentum(WholebodyData& d){
	int nlink = (int)links.size();

	// calc inertial matrix
	d.centroid.I.clear();
	for(int j = 0; j < nlink; j++){
		vec3_t rj  = d.centroid.pos_r*d.links[j].pos_t;
		quat_t qj  = d.centroid.pos_r*d.links[j].pos_r;
		mat3_t Rj;
		qj.ToMatrix(Rj);
		mat3_t rjc = mat3_t::Cross(rj);
		mat3_t Ij  = Rj*links[j].inertia*Rj.trans();
		d.centroid.I += links[j].mass*(rjc*rjc.trans()) + Ij;
	}
	d.centroid.Iinv = d.centroid.I.inv();

	// calc momentum in local coordinate
	d.centroid.L.clear();
	for(int j = 0; j < nlink; j++){
		WholebodyData::Link& dlnk  = d.links[j];
		
		// local inertia
		mat3_t Rj_local;
		dlnk.pos_r.ToMatrix(Rj_local);
		dlnk.I = Rj_local*links[j].inertia*Rj_local.trans();

		d.centroid.L += dlnk.pos_t % (links[j].mass*dlnk.vel_t) + dlnk.I*dlnk.vel_r;
	}

	// momentum in global coordinate
	d.centroid.Labs = d.centroid.I*d.centroid.vel_r + d.centroid.pos_r*d.centroid.L;
}

void Wholebody::CalcMomentumDerivative(WholebodyData& d){
	int nlink = (int)links.size();

	// calc momentum derivative in local coordinate
	d.centroid.Ld.clear();
	for(int i = 0; i < nlink; i++){
		WholebodyData::Link& dlnk  = d.links[i];

		d.centroid.Ld += dlnk.pos_t % (links[i].mass*dlnk.acc_t) + dlnk.I*dlnk.acc_r;
	}
}
	
void Wholebody::CalcForce(WholebodyData & d){
	int nchain = (int)chains.size();
	int nend   = (int)ends.size();

	// transform and copy end forces to links
	for(int i = 0; i < nend; i++){
		WholebodyData::End&  dend = d.ends[i];
	    WholebodyData::Link& dlnk = d.links[ends[i].ilink];
		
		dlnk.force_t = dend.force_t;
		dlnk.force_r = dend.force_r + ends[i].offset % dend.force_t;
	}

	vec3_t ac = d.centroid.acc_t;
	quat_t q0 = d.centroid.pos_r;
	vec3_t w0 = d.centroid.vel_r;
	vec3_t u0 = d.centroid.acc_r;

	// trace chains in reverse order
	for(int ic = nchain-1; ic >= 0; ic--){
		int nlink = (int)chains[ic].ilink.size();
		for(int j = nlink-1; j >= 0; j--){
			int i = chains[ic].ilink[j];
			WholebodyData::Link& dlnk = d.links[i];
			
			dlnk.force_t_child.clear();
			dlnk.force_r_child.clear();

			for(int ic : links[i].ichildren){
				WholebodyData::Link& dlnkc = d.links[ic];
		
				vec3_t r = q0*(dlnkc.pos_t - dlnk.pos_t);
				dlnk.force_t_child -=  dlnkc.force_t_par;
				dlnk.force_r_child -= (dlnkc.force_r_par + r % dlnkc.force_t_par);
			}

			// pj = pc * q0 * pjhat;
			// vj = vc + w0 % (q0*pjhat) + q0*vjhat
			// aj = ac + u0 % (q0*pjhat) + w0 % (w0 % (q0*pjhat)) + 2*w0 % (q0*vjhat) + q0*ajhat;

			// qj = q0 * qjhat;
			// wj = w0 + q0*wjhat;
			// uj = u0 + q0*ujhat + w0 % (q0*wjhat);
			vec3_t acc_t_abs   = ac + u0 % (q0*dlnk.pos_t) + w0 % (w0 % (q0*dlnk.pos_t)) + 2.0*(w0 % (q0*dlnk.vel_t)) + q0*dlnk.acc_t;
			vec3_t vel_r_abs   = w0 + q0*dlnk.vel_r;
			vec3_t acc_r_abs   = u0 + w0 % (q0*dlnk.vel_r) + q0*dlnk.acc_r;
			
			quat_t qj = q0*dlnk.pos_r;
			vec3_t acc_r_local = qj.Conjugated()*acc_r_abs;
			vec3_t vel_r_local = qj.Conjugated()*vel_r_abs;
					
			dlnk.force_t_par = links[i].mass*(acc_t_abs + vec3_t(0.0, 0.0, param.gravity)) - dlnk.force_t - dlnk.force_t_child;
			dlnk.force_r_par = qj*(links[j].inertia*acc_r_local - vel_r_local % (links[j].inertia*vel_r_local)) - dlnk.force_r - dlnk.force_r_child;
				
			// calc joint torque
			int iq = links[i].ijoint;
			if(iq != -1){
				d.tau[iq] = (qj*links[i].axis)*dlnk.force_r_par + joints[iq].rotor_inertia*d.qdd[iq];
			}
		}
	}
}

void Wholebody::ComState (real_t t, vec3_t& pos, vec3_t& vel){
	if(traj.empty())
		return;

	KeyPair       kp = traj.GetSegment(t);
	WholebodyKey* k0 = (WholebodyKey*)kp.first;
	WholebodyKey* k1 = (WholebodyKey*)kp.second;

    if(k1 == k0->next){
        pos = InterpolatePos(
		    t,
		    k0->tick->time, k0->data.centroid.pos_t, k0->data.centroid.vel_t,
		    k1->tick->time, k1->data.centroid.pos_t, k1->data.centroid.vel_t,
		    Interpolate::LinearDiff);
        vel = InterpolateVel(
		    t,
		    k0->tick->time, k0->data.centroid.pos_t, k0->data.centroid.vel_t,
		    k1->tick->time, k1->data.centroid.pos_t, k1->data.centroid.vel_t,
		    Interpolate::LinearDiff);
    }
    else{
        pos = k0->data.centroid.pos_t;
		vel = k0->data.centroid.vel_t;
    }
}

void Wholebody::BaseState(real_t t, quat_t& ori, vec3_t& angvel) {
	if(traj.empty())
		return;

	KeyPair       kp = traj.GetSegment(t);
	WholebodyKey* k0 = (WholebodyKey*)kp.first;
	WholebodyKey* k1 = (WholebodyKey*)kp.second;

    if(k1 == k0->next){
        ori = InterpolateOri(
		    t,
		    k0->tick->time, k0->data.centroid.pos_r, k0->data.centroid.vel_r,
		    k1->tick->time, k1->data.centroid.pos_r, k1->data.centroid.vel_r,
		    Interpolate::SlerpDiff);
        angvel = k0->data.centroid.vel_r;
    }
    else{
        ori    = k0->data.centroid.pos_r;
        angvel = k0->data.centroid.vel_r;
    }
}

void Wholebody::LinkPose(real_t t, int i, vec3_t& pos, quat_t& ori) {
	if(traj.empty())
		return;

	KeyPair      kp = traj.GetSegment(t);
	WholebodyKey* k0 = (WholebodyKey*)kp.first;
	WholebodyKey* k1 = (WholebodyKey*)kp.second;

    if(k1 == k0->next){
        pos = InterpolatePos(
		    t,
		    k0->tick->time, k0->data.links[i].pos_t, k0->data.links[i].vel_t,
		    k1->tick->time, k1->data.links[i].pos_t, k1->data.links[i].vel_t,
		    Interpolate::LinearDiff);
        ori = InterpolateOri(
		    t,
		    k0->tick->time, k0->data.links[i].pos_r, k0->data.links[i].vel_r,
		    k1->tick->time, k1->data.links[i].pos_r, k1->data.links[i].vel_r,
		    Interpolate::SlerpDiff);
    }
    else{
        pos = k0->data.links[i].pos_t;
        ori = k0->data.links[i].pos_r;
    }
}

void Wholebody::LinkVelocity(real_t t, int i, vec3_t& vel, vec3_t& angvel) {
	if(traj.empty())
		return;

	KeyPair      kp = traj.GetSegment(t);
	WholebodyKey* k0 = (WholebodyKey*)kp.first;
	WholebodyKey* k1 = (WholebodyKey*)kp.second;

    vel    = k0->data.links[i].vel_t;
    angvel = k0->data.links[i].vel_r;
}

void Wholebody::LinkForce(real_t t, int i, vec3_t& force, vec3_t& moment){
	if(traj.empty())
		return;

	KeyPair      kp = traj.GetSegment(t);
	WholebodyKey* k0 = (WholebodyKey*)kp.first;
	WholebodyKey* k1 = (WholebodyKey*)kp.second;

    force  = k0->data.links[i].force_t;
    moment = k0->data.links[i].force_r;
}

void Wholebody::CalcTrajectory() {
	real_t tf = traj.back()->tick->time;
	real_t dt = 0.01;

	trajectory.clear();
	for (real_t t = 0.0; t <= tf; t += dt) {
		Snapshot s;
		CreateSnapshot(t, s);
		trajectory.push_back(s);
	}

	trajReady = true;
}

void Wholebody::Draw(Render::Canvas* canvas, Render::Config* conf) {
	TrajectoryNode::Draw(canvas, conf);

	if (!trajReady)
		CalcTrajectory();

	if (trajectory.empty())
		return;

	stringstream ss;

	// link
	if (conf->Set(canvas, Render::Item::WholebodyLinkTraj, this)) {
		for(int i = 0; i < links.size(); i++){
			ss.str("");
			ss << i;
			canvas->BeginLayer("wholebody_link" + ss.str(), true);
			canvas->SetPointSize(5.0);
			vec3_t pc = trajectory[0].pos_t;
			quat_t q0 = trajectory[0].pos_r;
			vec3_t pi = trajectory[0].links[i].pos_t;
			canvas->Point(Vec3f(pc + q0*pi));
			canvas->BeginPath();
			canvas->MoveTo(pc + q0*pi);
			for (int k = 1; k < trajectory.size(); k++) {
				pc = trajectory[k].pos_t;
				q0 = trajectory[k].pos_r;
				pi = trajectory[k].links[i].pos_t;
				canvas->LineTo(pc + q0*pi);
			}
			canvas->EndPath();
			canvas->EndLayer();
		}
	}
}

void Wholebody::CreateSnapshot(real_t t, Wholebody::Snapshot& s){
	s.t = t;
    
	ComState (t, s.pos_t, s.vel_t);
	BaseState(t, s.pos_r, s.vel_r);

	s.links.resize(links.size());
	for(int i = 0; i < links.size(); i++){
        LinkPose    (t, i, s.links[i].pos_t  , s.links[i].pos_r  );
        LinkVelocity(t, i, s.links[i].vel_t  , s.links[i].vel_r  );
        LinkForce   (t, i, s.links[i].force_t, s.links[i].force_r);
	}
}

void Wholebody::CreateSnapshot(real_t t){
	CreateSnapshot(t, snapshot);
}

void Wholebody::DrawSnapshot(Render::Canvas* canvas, Render::Config* conf) {
	if (conf->Set(canvas, Render::Item::WholebodyLink, this)) {
		vec3_t pc = snapshot.pos_t;
		quat_t q0 = snapshot.pos_r;

		for(int i = 1; i < links.size(); i++){
			canvas->BeginLayer("wholebody_link_snapshot", true);
			canvas->SetLineColor(i % 2 == 0 ? "cyan" : "blue");

			int ipar = links[i].iparent;
			
			// line connecting each link and its parent
			canvas->BeginPath();
			canvas->MoveTo(pc + q0*snapshot.links[ipar].pos_t);
			canvas->LineTo(pc + q0*snapshot.links[i   ].pos_t);
			canvas->EndPath();
			
            // line indicating force
			canvas->SetLineColor("magenta");
			canvas->BeginPath();
			canvas->MoveTo(pc + q0*snapshot.links[i].pos_t);
			canvas->LineTo(pc + q0*snapshot.links[i].pos_t + 0.001*snapshot.links[i].force_t);
			canvas->EndPath();
			canvas->EndLayer();
		}

		for(int i = 0; i < ends.size(); i++){
			int ilink = ends[i].ilink;
			canvas->SetPointSize(5.0);
			canvas->Point(Vec3f(pc + q0*snapshot.links[ilink].pos_t));
		}
	}	
}

///////////////////////////////////////////////////////////////////////////////////////////////////

WholebodyCon::WholebodyCon(Solver* solver, int _dim, int _tag, string _name, WholebodyKey* _obj, real_t _scale):
	Constraint(solver, _dim, ID(_tag, _obj->node, _obj->tick, _name), Constraint::Type::Equality, _scale) {
	obj[0] = _obj;
	obj[1] = (WholebodyKey*)_obj->next;
}

WholebodyJointPosCon::WholebodyJointPosCon(Solver* solver, string _name, WholebodyKey* _obj, int _ijoint, real_t _scale):
	WholebodyCon(solver, 1, ConTag::WholebodyJointPos, _name, _obj, _scale) {

	ijoint = _ijoint;

	AddSLink(obj[1]->joints[ijoint].var_q  );
	AddSLink(obj[0]->joints[ijoint].var_q  );
	AddSLink(obj[0]->joints[ijoint].var_qd );
	AddSLink(obj[0]->joints[ijoint].var_qdd);
}

WholebodyJointVelCon::WholebodyJointVelCon(Solver* solver, string _name, WholebodyKey* _obj, int _ijoint, real_t _scale):
	WholebodyCon(solver, 1, ConTag::WholebodyJointVel, _name, _obj, _scale) {

	ijoint = _ijoint;

	AddSLink(obj[1]->joints[ijoint].var_qd );
	AddSLink(obj[0]->joints[ijoint].var_qd );
	AddSLink(obj[0]->joints[ijoint].var_qdd);
}

WholebodyCentroidPosConT::WholebodyCentroidPosConT(Solver* solver, string _name, WholebodyKey* _obj, real_t _scale):
	WholebodyCon(solver, 3, ConTag::WholebodyPosT, _name, _obj, _scale) {

	AddSLink(obj[1]->centroid.var_pos_t);
	AddSLink(obj[0]->centroid.var_pos_t);
	AddSLink(obj[0]->centroid.var_vel_t);
	AddSLink(obj[0]->centroid.var_acc_t);
	int nend   = (int)obj[0]->ends  .size();
	for(int i = 0; i < nend; i++){
		AddSLink(obj[0]->ends[i].var_force_t);
	}
}

WholebodyCentroidVelConT::WholebodyCentroidVelConT(Solver* solver, string _name, WholebodyKey* _obj, real_t _scale):
	WholebodyCon(solver, 3, ConTag::WholebodyVelT, _name, _obj, _scale) {

	AddSLink(obj[1]->centroid.var_vel_t);
	AddSLink(obj[0]->centroid.var_vel_t);
	AddSLink(obj[0]->centroid.var_acc_t);
	int nend   = (int)obj[0]->ends  .size();
	for(int i = 0; i < nend; i++){
		AddSLink(obj[0]->ends[i].var_force_t);
	}
}

WholebodyCentroidPosConR::WholebodyCentroidPosConR(Solver* solver, string _name, WholebodyKey* _obj, real_t _scale):
	WholebodyCon(solver, 3, ConTag::WholebodyPosR, _name, _obj, _scale) {

	AddSLink (obj[1]->centroid.var_pos_r);
	AddM3Link(obj[0]->centroid.var_pos_r);
	AddM3Link(obj[0]->centroid.var_vel_r);
	AddM3Link(obj[0]->centroid.var_acc_r);

	int nend   = (int)obj[0]->ends  .size();
	int njoint = (int)obj[0]->joints.size();
	for(int i = 0; i < njoint; i++)
		AddC3Link(obj[0]->joints[i].var_q);
	for(int i = 0; i < njoint; i++)
		AddC3Link(obj[0]->joints[i].var_qdd);
	
	for(int i = 0; i < nend; i++){
		AddM3Link(obj[0]->ends[i].var_force_t);
		AddM3Link(obj[0]->ends[i].var_force_r);
	}
}

WholebodyCentroidVelConR::WholebodyCentroidVelConR(Solver* solver, string _name, WholebodyKey* _obj, real_t _scale):
	WholebodyCon(solver, 3, ConTag::WholebodyVelR, _name, _obj, _scale) {

	AddSLink(obj[1]->centroid.var_vel_r);
	AddSLink(obj[0]->centroid.var_vel_r);
	AddSLink(obj[0]->centroid.var_acc_r);

	int nend   = (int)obj[0]->ends  .size();
	int njoint = (int)obj[0]->joints.size();
	
	for(int i = 0; i < njoint; i++)
		AddC3Link(obj[0]->joints[i].var_q);
	for(int i = 0; i < njoint; i++)
		AddC3Link(obj[0]->joints[i].var_qdd);

	for(int i = 0; i < nend; i++){
		AddM3Link(obj[0]->ends[i].var_force_t);
		AddM3Link(obj[0]->ends[i].var_force_r);
	}
}

WholebodyDesPosConT::WholebodyDesPosConT(Solver* solver, string _name, WholebodyKey* _obj, int _iend, real_t _scale):
	Constraint(solver, 3, ID(ConTag::WholebodyPosT, _obj->node, _obj->tick, _name), Constraint::Type::Equality, _scale){
	obj = _obj;
	iend = _iend;

	AddSLink (obj->centroid.var_pos_t);
	AddX3Link(obj->centroid.var_pos_r);

	int njoint = (int)obj->joints.size();
	for(int i = 0; i < njoint; i++)
		AddC3Link(obj->joints[i].var_q);

}

WholebodyDesPosConR::WholebodyDesPosConR(Solver* solver, string _name, WholebodyKey* _obj, int _iend, real_t _scale):
	Constraint(solver, 3, ID(ConTag::WholebodyPosR, _obj->node, _obj->tick, _name), Constraint::Type::Equality, _scale){
	obj = _obj;
	iend = _iend;

	AddSLink (obj->centroid.var_pos_r);

	int njoint = (int)obj->joints.size();
	for(int i = 0; i < njoint; i++)
		AddC3Link(obj->joints[i].var_q);

}

WholebodyDesVelConT::WholebodyDesVelConT(Solver* solver, string _name, WholebodyKey* _obj, int _iend, real_t _scale):
	Constraint(solver, 3, ID(ConTag::WholebodyVelT, _obj->node, _obj->tick, _name), Constraint::Type::Equality, _scale){
	obj = _obj;
	iend = _iend;

	AddSLink (obj->centroid.var_vel_t);
	AddX3Link(obj->centroid.var_vel_r);

	int njoint = (int)obj->joints.size();
	for(int i = 0; i < njoint; i++)
		AddC3Link(obj->joints[i].var_qd);

}

WholebodyDesVelConR::WholebodyDesVelConR(Solver* solver, string _name, WholebodyKey* _obj, int _iend, real_t _scale):
	Constraint(solver, 3, ID(ConTag::WholebodyVelR, _obj->node, _obj->tick, _name), Constraint::Type::Equality, _scale){
	obj = _obj;
	iend = _iend;

	AddSLink (obj->centroid.var_vel_r);
	
	int njoint = (int)obj->joints.size();
	for(int i = 0; i < njoint; i++)
		AddC3Link(obj->joints[i].var_qd);

}

WholebodyLCon::WholebodyLCon(Solver* solver, string _name, WholebodyKey* _obj, real_t _scale):
	Constraint(solver, 3, ID(ConTag::WholebodyMomentum, _obj->node, _obj->tick, _name), Constraint::Type::Equality, _scale){
	obj = _obj;
	
	AddX3Link(obj->centroid.var_pos_r);
	AddM3Link(obj->centroid.var_vel_r);

	int njoint = (int)obj->joints.size();
	for(int i = 0; i < njoint; i++)
		AddC3Link(obj->joints[i].var_q);
	for(int i = 0; i < njoint; i++)
		AddC3Link(obj->joints[i].var_qd);

}

WholebodyContactPosConT::WholebodyContactPosConT(Solver* solver, string _name, WholebodyKey* _obj, int _iend, real_t _scale):
	Constraint(solver, 3, ID(ConTag::WholebodyContactPosT, _obj->node, _obj->tick, _name), Constraint::Type::Equality, _scale){
	obj  = _obj;
	iend = _iend;
    
	AddM3Link(obj->centroid.var_pos_t);
	AddM3Link(obj->centroid.var_pos_r);

	int njoint = (int)obj->joints.size();
	for(int i = 0; i < njoint; i++)
		AddC3Link(obj->joints[i].var_q);
}

WholebodyContactPosConR::WholebodyContactPosConR(Solver* solver, string _name, WholebodyKey* _obj, int _iend, real_t _scale):
	Constraint(solver, 3, ID(ConTag::WholebodyContactPosR, _obj->node, _obj->tick, _name), Constraint::Type::Equality, _scale){
	obj  = _obj;
	iend = _iend;
    
	AddM3Link(obj->centroid.var_pos_r);

	int njoint = (int)obj->joints.size();
	for(int i = 0; i < njoint; i++)
		AddC3Link(obj->joints[i].var_q);
}

WholebodyContactVelConT::WholebodyContactVelConT(Solver* solver, string _name, WholebodyKey* _obj, int _iend, real_t _scale):
	Constraint(solver, 3, ID(ConTag::WholebodyContactVelT, _obj->node, _obj->tick, _name), Constraint::Type::Equality, _scale){
	obj  = _obj;
	iend = _iend;
    
	AddM3Link(obj->centroid.var_vel_t);
	AddM3Link(obj->centroid.var_vel_r);

	int njoint = (int)obj->joints.size();
	for(int i = 0; i < njoint; i++)
		AddC3Link(obj->joints[i].var_qd);
}

WholebodyContactVelConR::WholebodyContactVelConR(Solver* solver, string _name, WholebodyKey* _obj, int _iend, /*int _dir, */real_t _scale):
	Constraint(solver, 3, ID(ConTag::WholebodyContactVelR, _obj->node, _obj->tick, _name), Constraint::Type::Equality, _scale){
	obj  = _obj;
	iend = _iend;
	
	AddM3Link(obj->centroid.var_vel_r);

	int njoint = (int)obj->joints.size();
	for(int i = 0; i < njoint; i++)
		AddC3Link(obj->joints[i].var_qd);
}

WholebodyNormalForceCon::WholebodyNormalForceCon(Solver* solver, string _name, WholebodyKey* _obj, int _iend, real_t _scale):
	Constraint(solver, 1, ID(ConTag::WholebodyNormalForce, _obj->node, _obj->tick, _name), Constraint::Type::InequalityBarrier, _scale){
	obj  = _obj;
	iend = _iend;
    
	AddR3Link(obj->ends[iend].var_force_t);
}

WholebodyFrictionForceCon::WholebodyFrictionForceCon(Solver* solver, string _name, WholebodyKey* _obj, int _iend, int _dir, int _side, real_t _scale):
	Constraint(solver, 1, ID(ConTag::WholebodyFrictionForce, _obj->node, _obj->tick, _name), Constraint::Type::InequalityPenalty, _scale){
	obj  = _obj;
	iend = _iend;
	dir  = _dir;
	side = _side;
    
	AddR3Link(obj->ends[iend].var_force_t);
}

WholebodyMomentCon::WholebodyMomentCon(Solver* solver, string _name, WholebodyKey* _obj, int _iend, int _dir, int _side, real_t _scale):
	Constraint(solver, 1, ID(ConTag::WholebodyMoment, _obj->node, _obj->tick, _name), Constraint::Type::InequalityPenalty, _scale){
	obj   = _obj;
	iend  = _iend;
	dir   = _dir;
	side  = _side;
    
	AddR3Link(obj->ends[iend].var_force_t);
	AddR3Link(obj->ends[iend].var_force_r);
}

///////////////////////////////////////////////////////////////////////////////////////////////////

void WholebodyJointPosCon::Prepare(){
	q0     = obj[0]->joints[ijoint].var_q  ->val;
	qd0    = obj[0]->joints[ijoint].var_qd ->val;
	qdd0   = obj[0]->joints[ijoint].var_qdd->val;
	q1     = obj[1]->joints[ijoint].var_q  ->val;
	h      = obj[0]->hnext;
	h2     = h*h;
	
	q_rhs = q0 + h*qd0 + (0.5*h2)*qdd0;
}

void WholebodyJointVelCon::Prepare(){
	qd0     = obj[0]->joints[ijoint].var_qd ->val;
	qdd0    = obj[0]->joints[ijoint].var_qdd->val;
	qd1     = obj[1]->joints[ijoint].var_qd ->val;
	h       = obj[0]->hnext;
	
	qd_rhs = qd0 + h*qdd0;
}

void WholebodyCentroidPosConT::Prepare(){
	pc0 = obj[0]->centroid.var_pos_t->val;
	vc0 = obj[0]->centroid.var_vel_t->val;
	pc1 = obj[1]->centroid.var_pos_t->val;
	h   = obj[0]->hnext;
	h2  = h*h;
	m   = obj[0]->wb->param.totalMass;
	g   = vec3_t(0.0, 0.0, -obj[0]->wb->param.gravity);
	ac0 = obj[0]->centroid.var_acc_t->val + (1.0/m)*(obj[0]->fsum) + g;

	pc_rhs = pc0 + h*vc0 + ((1.0/2.0)*h2)*ac0;
}

void WholebodyCentroidVelConT::Prepare(){
	vc0 = obj[0]->centroid.var_vel_t->val;
	vc1 = obj[1]->centroid.var_vel_t->val;
	h   = obj[0]->hnext;
	m   = obj[0]->wb->param.totalMass;
	g   = vec3_t(0.0, 0.0, -obj[0]->wb->param.gravity);
	ac0 = obj[0]->centroid.var_acc_t->val + (1.0/m)*(obj[0]->fsum) + g;

	vc_rhs = vc0 + h*ac0;
}

void WholebodyCentroidPosConR::Prepare(){
	// L = I*w + q*Lhat
	// L'= I'*w + I*w' + q*Lhat' + w % (q*Lhat) = msum
	// 
	// w' = Iinv*(msum - q*Lhat' - (I' - (q*Lhat)^x)*w)

	w0    = obj[0]->centroid.var_vel_r->val;
	q1    = obj[1]->centroid.var_pos_r->val;
	h     = obj[0]->hnext;
	h2    = h*h;
	//L     = obj[0]->data.centroid.L;
	Ld    = obj[0]->data.centroid.Ld;
	Iinv  = obj[0]->data.centroid.Iinv;
	u0    = obj[0]->centroid.var_acc_r->val + Iinv*(obj[0]->msum - obj[0]->q0*Ld/* - w0 % (obj[0]->q0*L)*/);
	omega = h*w0 + (0.5*h2)*u0;
	q_omega = quat_t::Rot(omega);
	q_omega.ToMatrix(R_omega);
	A_omega = RotJacobian(omega);

	q_rhs = q_omega*obj[0]->q0;
	//q_rhs = obj[0]->q0*quat_t::Rot(obj[0]->q0.Conjugated()*();
}

void WholebodyCentroidVelConR::Prepare(){
	pc   = obj[0]->centroid.var_pos_t->val;
	w0   = obj[0]->centroid.var_vel_r->val;
	w1   = obj[1]->centroid.var_vel_r->val;
	Ld   = obj[0]->data.centroid.Ld;
	Iinv = obj[0]->data.centroid.Iinv;
	h    = obj[0]->hnext;
	u0   = obj[0]->centroid.var_acc_r->val + Iinv*(obj[0]->msum - obj[0]->q0*Ld);
	
	w_rhs = w0 + h*u0;
}

void WholebodyDesPosConT::Prepare(){
	WholebodyKey::End& end = obj->ends[iend];
	WholebodyData::End& dend = obj->data.ends[iend];

	/*
	 pi_abs = pc + q0*pi
	 vi_abs = vc + w0 % (q0*pi) + q0*vi
	 qi_abs = q0*qi
	 wi_abs = w0 + q0*wi
	*/
	pc = obj->centroid.var_pos_t->val;
	q0 = obj->centroid.var_pos_r->val;
	q0.ToMatrix(R0);
	pi = dend.pos_t;
}

void WholebodyDesPosConR::Prepare(){
	WholebodyKey::End& end = obj->ends[iend];
	WholebodyData::End& dend = obj->data.ends[iend];

	q0 = obj->centroid.var_pos_r->val;
	q0.ToMatrix(R0);
	qi = dend.pos_r;
}

void WholebodyDesVelConT::Prepare(){
	WholebodyKey::End& end = obj->ends[iend];
	WholebodyData::End& dend = obj->data.ends[iend];

	vc = obj->centroid.var_vel_t->val;
	w0 = obj->centroid.var_vel_r->val;
	q0 = obj->centroid.var_pos_r->val;
	q0.ToMatrix(R0);
	pi = dend.pos_t;
	vi = dend.vel_t;
}

void WholebodyDesVelConR::Prepare(){
	WholebodyKey::End& end = obj->ends[iend];
	WholebodyData::End& dend = obj->data.ends[iend];

	w0 = obj->centroid.var_vel_r->val;
	q0 = obj->centroid.var_pos_r->val;
	q0.ToMatrix(R0);
	wi = dend.vel_r;
}

void WholebodyLCon::Prepare(){
	obj->data.centroid.pos_r.ToMatrix(Rf);
}

void WholebodyContactPosConT::Prepare(){
	WholebodyData::End&  dend  = obj->data.ends[iend];
	WholebodyData::End&  dend_des  = obj->data_des.ends[iend];
	
	po = dend_des.pos_tc;
	qo = dend_des.pos_rc;
	qo.ToMatrix(Ro);
	r  = dend_des.pos_te;
	pc = obj->centroid.var_pos_t->val;
	q0 = obj->centroid.var_pos_r->val;
	q0.ToMatrix(R0);
	pi = dend.pos_t;
	qi = dend.pos_r;
}

void WholebodyContactPosConR::Prepare(){
	WholebodyData::End& dend = obj->data.ends[iend];
	WholebodyData::End& dend_des = obj->data_des.ends[iend];
	
	qo = dend_des.pos_rc;
	qo.ToMatrix(Ro);
	q0 = obj->centroid.var_pos_r->val;
	q0.ToMatrix(R0);
	qi = dend.pos_r;
}

void WholebodyContactVelConT::Prepare(){
	WholebodyData::End& dend = obj->data.ends[iend];
	WholebodyData::End& dend_des = obj->data_des.ends[iend];
	
	qo = dend_des.pos_rc;
	qo.ToMatrix(Ro);
	r  = dend_des.pos_te;
	vc = obj->centroid.var_vel_t->val;
	q0 = obj->centroid.var_pos_r->val;
	q0.ToMatrix(R0);
	w0 = obj->centroid.var_vel_r->val;
	pi = dend.pos_t;
	qi = dend.pos_r;
	vi = dend.vel_t;
	wi = dend.vel_r;
}

void WholebodyContactVelConR::Prepare(){
	WholebodyData::End& dend = obj->data.ends[iend];
	WholebodyData::End& dend_des = obj->data_des.ends[iend];
	
	qo = dend_des.pos_rc;
	qo.ToMatrix(Ro);
	q0 = obj->centroid.var_pos_r->val;
	q0.ToMatrix(R0);
	w0 = obj->centroid.var_vel_r->val;
	qi = dend.pos_r;
	wi = dend.vel_r;
}

void WholebodyNormalForceCon::Prepare(){
	WholebodyKey ::End& end  = obj->ends[iend];
	WholebodyData::End& dend = obj->data.ends[iend];

	fn = end.var_force_t->val.z;
}

void WholebodyFrictionForceCon::Prepare(){
	WholebodyKey ::End& end  = obj->ends[iend];
	WholebodyData::End& dend = obj->data.ends[iend];
	WholebodyData::End& dend_des = obj->data_des.ends[iend];

	mu = dend_des.mu;
	vec3_t f = end.var_force_t->val;
	ft = (dir == 0 ? f.x : f.y);
	fn = f.z;
}

void WholebodyMomentCon::Prepare(){
	WholebodyKey ::End& end  = obj->ends[iend];
	WholebodyData::End& dend = obj->data.ends[iend];
	WholebodyData::End& dend_des = obj->data_des.ends[iend];

	fn   = end.var_force_t->val.z;
	m    = end.var_force_r->val;
	cmin = dend_des.cop_min;
	cmax = dend_des.cop_max;
}

///////////////////////////////////////////////////////////////////////////////////////////////////

void WholebodyJointPosCon::CalcCoef(){
	Prepare();

	int idx = 0;
	dynamic_cast<SLink*>(links[idx++])->SetCoef( 1.0);
	dynamic_cast<SLink*>(links[idx++])->SetCoef(-1.0);
	dynamic_cast<SLink*>(links[idx++])->SetCoef(-h);
	dynamic_cast<SLink*>(links[idx++])->SetCoef(-(0.5*h2));
}

void WholebodyJointVelCon::CalcCoef(){
	Prepare();

	int idx = 0;
	dynamic_cast<SLink*>(links[idx++])->SetCoef( 1.0);
	dynamic_cast<SLink*>(links[idx++])->SetCoef(-1.0);
	dynamic_cast<SLink*>(links[idx++])->SetCoef(-h);
}

void WholebodyCentroidPosConT::CalcCoef(){
	Prepare();

	int idx = 0;
	dynamic_cast<SLink*>(links[idx++])->SetCoef( 1.0);
	dynamic_cast<SLink*>(links[idx++])->SetCoef(-1.0);
	dynamic_cast<SLink*>(links[idx++])->SetCoef(-h);
	dynamic_cast<SLink*>(links[idx++])->SetCoef(-0.5*h2);

	int nend = (int)obj[0]->ends.size();
	for(int i = 0; i < nend; i++){
		dynamic_cast<SLink*>(links[idx++])->SetCoef(-(0.5*h2/m));
	}
}

void WholebodyCentroidVelConT::CalcCoef(){
	Prepare();

	int idx = 0;
	dynamic_cast<SLink*>(links[idx++])->SetCoef( 1.0);
	dynamic_cast<SLink*>(links[idx++])->SetCoef(-1.0);
	dynamic_cast<SLink*>(links[idx++])->SetCoef(-h);

	int nend = (int)obj[0]->ends.size();
	for(int i = 0; i < nend; i++){
		dynamic_cast<SLink*>(links[idx++])->SetCoef(-(h/m));
	}
}

void WholebodyCentroidPosConR::CalcCoef(){
	Prepare();

	mat3_t tmp = A_omega*((0.5*h2)*Iinv);
	mat3_t tmpR0 = tmp*obj[0]->R0;
	
	int idx = 0;
	dynamic_cast<SLink *>(links[idx++])->SetCoef( 1.0);
	dynamic_cast<M3Link*>(links[idx++])->SetCoef(-R_omega);
	dynamic_cast<M3Link*>(links[idx++])->SetCoef(-h*A_omega);
	dynamic_cast<M3Link*>(links[idx++])->SetCoef(-(0.5*h2)*A_omega);

	int nend = (int)obj[0]->ends.size();
	int njoint = (int)obj[0]->joints.size();
	for(int i = 0; i < njoint; i++){
		dynamic_cast<C3Link*>(links[idx++])->SetCoef(tmpR0*obj[0]->J_Ld_q  .col(i));
	}
	for(int i = 0; i < njoint; i++){
		dynamic_cast<C3Link*>(links[idx++])->SetCoef(tmpR0*obj[0]->J_Ld_qdd.col(i));
	}
	for(int i = 0; i < nend; i++){
		dynamic_cast<M3Link*>(links[idx++])->SetCoef(-tmp*obj[0]->rec[i]);
		dynamic_cast<M3Link*>(links[idx++])->SetCoef(-tmp);
	}
}

void WholebodyCentroidVelConR::CalcCoef(){
	Prepare();

	mat3_t tmp   = h*Iinv;
	mat3_t tmpR0 = tmp*obj[0]->R0;

	int idx = 0;
	dynamic_cast<SLink*>(links[idx++])->SetCoef( 1.0);
	dynamic_cast<SLink*>(links[idx++])->SetCoef(-1.0);
	dynamic_cast<SLink*>(links[idx++])->SetCoef(-h);

	int nend = (int)obj[0]->ends.size();
	int njoint = (int)obj[0]->joints.size();
	for(int i = 0; i < njoint; i++){
		dynamic_cast<C3Link*>(links[idx++])->SetCoef(tmpR0*obj[0]->J_Ld_q  .col(i));
	}
	for(int i = 0; i < njoint; i++){
		dynamic_cast<C3Link*>(links[idx++])->SetCoef(tmpR0*obj[0]->J_Ld_qdd.col(i));
	}
	for(int i = 0; i < nend; i++){
		dynamic_cast<M3Link*>(links[idx++])->SetCoef(-tmp*obj[0]->rec[i]);
		dynamic_cast<M3Link*>(links[idx++])->SetCoef(-tmp);
	}
}

void WholebodyDesPosConT::CalcCoef(){
	Prepare();

	int idx = 0;
	dynamic_cast<SLink *>(links[idx++])->SetCoef(1.0);
	dynamic_cast<X3Link*>(links[idx++])->SetCoef(-(q0*pi));

	int i = obj->wb->ends[iend].ilink;
	int njoint = (int)obj->joints.size();

	for(int j = 0; j < njoint; j++){
		vec3_t Jv = obj->data.Jfk[i].col(j).sub_vector(TSubVectorDim<0,3>());
		dynamic_cast<C3Link*>(links[idx++])->SetCoef(R0*Jv);
	}
}

void WholebodyDesPosConR::CalcCoef(){
	Prepare();

	int idx = 0;
	dynamic_cast<SLink *>(links[idx++])->SetCoef(1.0);

	int njoint = (int)obj->joints.size();
	int i = obj->wb->ends[iend].ilink;

	for(int j = 0; j < njoint; j++){
		vec3_t Jw = obj->data.Jfk[i].col(j).sub_vector(TSubVectorDim<3,3>());
		dynamic_cast<C3Link*>(links[idx++])->SetCoef(R0*Jw);
	}
}

void WholebodyDesVelConT::CalcCoef(){
	Prepare();

	int idx = 0;
	dynamic_cast<SLink *>(links[idx++])->SetCoef(1.0);
	dynamic_cast<X3Link*>(links[idx++])->SetCoef(-(q0*pi));
	
	int njoint = (int)obj->joints.size();
	int i = obj->wb->ends[iend].ilink;

	for(int j = 0; j < njoint; j++){
		vec3_t Jv = obj->data.Jfk[i].col(j).sub_vector(TSubVectorDim<0,3>());
		dynamic_cast<C3Link*>(links[idx++])->SetCoef(R0*Jv);
	}
}

void WholebodyDesVelConR::CalcCoef(){
	Prepare();

	int idx = 0;
	dynamic_cast<SLink *>(links[idx++])->SetCoef(1.0);

	int njoint = (int)obj->joints.size();
	int i = obj->wb->ends[iend].ilink;

	for(int j = 0; j < njoint; j++){
		vec3_t Jw = obj->data.Jfk[i].col(j).sub_vector(TSubVectorDim<3,3>());
		dynamic_cast<C3Link*>(links[idx++])->SetCoef(R0*Jw);
	}
}

void WholebodyLCon::CalcCoef(){
	Prepare();

	int nend = (int)obj->ends.size();

	int idx = 0;
	dynamic_cast<X3Link*>(links[idx++])->SetCoef(-Rf*obj->data.centroid.L);
	dynamic_cast<M3Link*>(links[idx++])->SetCoef(obj->data.centroid.I);

	int njoint = (int)obj->joints.size();
	for(int j = 0; j < njoint; j++)
		dynamic_cast<C3Link*>(links[idx++])->SetCoef(Rf*obj->J_L_q .col(j));
	for(int j = 0; j < njoint; j++)
		dynamic_cast<C3Link*>(links[idx++])->SetCoef(Rf*obj->J_L_qd.col(j));
}

void WholebodyContactPosConT::CalcCoef(){
	/*
	y  = qo^T*(pc + q0*(pi + qi*r) - po)
	dy = qo^T*(dpc + Omega0 % q0*(pi + qi*r) + q0*(dpi + Omegai % qi*r))
	   = qo^T*dpc + qo^T (q0*(pi + qi*r)^x)^T Omega0 + (qo^T*q0)*dpi + qo^T q0 ((qi*r)^x)^T Omegai
    */
	Prepare();

	int idx = 0;
	dynamic_cast<M3Link*>(links[idx++])->SetCoef( Ro.trans());
	dynamic_cast<M3Link*>(links[idx++])->SetCoef(-Ro.trans()*mat3_t::Cross(q0*(pi + qi*r)));

	int njoint = (int)obj->joints.size();
	int i = obj->wb->ends[iend].ilink;
	for(int j = 0; j < njoint; j++){
		vec3_t Jv = obj->data.Jfk[i].col(j).sub_vector(TSubVectorDim<0,3>());
		vec3_t Jw = obj->data.Jfk[i].col(j).sub_vector(TSubVectorDim<3,3>());
		dynamic_cast<C3Link*>(links[idx++])->SetCoef(Ro.trans()*R0*(Jv + Jw % (qi*r)));
	}
}

void WholebodyContactPosConR::CalcCoef(){
	Prepare();

	int idx = 0;
	dynamic_cast<M3Link*>(links[idx++])->SetCoef(Ro.trans());

	int njoint = (int)obj->joints.size();
	int i = obj->wb->ends[iend].ilink;
	for(int j = 0; j < njoint; j++){
		vec3_t Jw = obj->data.Jfk[i].col(j).sub_vector(TSubVectorDim<3,3>());
		dynamic_cast<C3Link*>(links[idx++])->SetCoef(Ro.trans()*R0*Jw);
	}
}

void WholebodyContactVelConT::CalcCoef(){
	/*
	y = qo^T*(vc + w0 % q0*(pi + qi*r) + q0*(vi + wi % qi*r))
	dy = qo^T*dvc + qo^T*(q0*(pi + qi*r))^x^T * dw0 + qo^T*q0*dvi + qo^T*q0*(qi*r)^x^T * dwi
	*/
	Prepare();

	int idx = 0;
	dynamic_cast<M3Link*>(links[idx++])->SetCoef( Ro.trans());
	dynamic_cast<M3Link*>(links[idx++])->SetCoef(-Ro.trans()*mat3_t::Cross(q0*(pi + qi*r)));

	int njoint = (int)obj->joints.size();
	int i = obj->wb->ends[iend].ilink;
	for(int j = 0; j < njoint; j++){
		vec3_t Jv = obj->data.Jfk[i].col(j).sub_vector(TSubVectorDim<0,3>());
		vec3_t Jw = obj->data.Jfk[i].col(j).sub_vector(TSubVectorDim<3,3>());
		dynamic_cast<C3Link*>(links[idx++])->SetCoef(Ro.trans()*R0*(Jv + Jw % (qi*r)));
	}
}

void WholebodyContactVelConR::CalcCoef(){
	Prepare();

	int idx = 0;
	dynamic_cast<M3Link*>(links[idx++])->SetCoef(Ro.trans());

	int njoint = (int)obj->joints.size();
	int i = obj->wb->ends[iend].ilink;
	for(int j = 0; j < njoint; j++){
		vec3_t Jw = obj->data.Jfk[i].col(j).sub_vector(TSubVectorDim<3,3>());
		dynamic_cast<C3Link*>(links[idx++])->SetCoef(Ro.trans()*R0*Jw);
	}
}

void WholebodyNormalForceCon::CalcCoef(){
	Prepare();

	dynamic_cast<R3Link*>(links[0])->SetCoef(vec3_t(0.0, 0.0, 1.0));
}

void WholebodyFrictionForceCon::CalcCoef(){
	Prepare();

	// -mu*fn <= ft <= mu*fn
	// -ft + mu*fn >= 0
	//  ft + mu*fn >= 0

	dynamic_cast<R3Link*>(links[0])->SetCoef(
		vec3_t(
			(dir == 0 ? (side == 0 ? -1.0 : 1.0) : 0.0),
			(dir == 1 ? (side == 0 ? -1.0 : 1.0) : 0.0),
			mu));
}

void WholebodyMomentCon::CalcCoef(){
	Prepare();

	// cop_min.x <= -my/fz <= cop_max.x
	// cop_min.y <=  mx/fz <= cop_max.y
	//
	// cop_min.x fz <= -my <= cop_max.x fz
	// cop_min.y fz <=  mx <= cop_max.y fz
	//
	// -my - (cop_min.x)*fz >= 0
	//  my + (cop_max.x)*fz >= 0
	//  mx - (cop_min.y)*fz >= 0
	// -mx + (cop_max.y)*fz >= 0
	//  mz - (cop_min.z)*fz >= 0
	// -mz + (cop_max.z)*fz >= 0

	if(dir == 0){
		dynamic_cast<R3Link*>(links[0])->SetCoef(vec3_t(0.0, 0.0, (side == 0 ? -cmin.x : cmax.x)));
		dynamic_cast<R3Link*>(links[1])->SetCoef(vec3_t(0.0, (side == 0 ? -1.0 : +1.0), 0.0));
	}
	if(dir == 1){
		dynamic_cast<R3Link*>(links[0])->SetCoef(vec3_t(0.0, 0.0, (side == 0 ? -cmin.y : cmax.y)));
		dynamic_cast<R3Link*>(links[1])->SetCoef(vec3_t((side == 0 ? +1.0 : -1.0), 0.0, 0.0));
	}
	if(dir == 2){
		dynamic_cast<R3Link*>(links[0])->SetCoef(vec3_t(0.0, 0.0, (side == 0 ? -cmin.z : cmax.z)));
		dynamic_cast<R3Link*>(links[1])->SetCoef(vec3_t(0.0, 0.0, (side == 0 ? +1.0 : -1.0)));
	}
}

///////////////////////////////////////////////////////////////////////////////////////////////////

void WholebodyJointPosCon::CalcDeviation(){
	y[0] = q1 - q_rhs;
}

void WholebodyJointVelCon::CalcDeviation(){
	y[0] = qd1 - qd_rhs;
}

void WholebodyCentroidPosConT::CalcDeviation(){
	y = pc1 - pc_rhs;
}

void WholebodyCentroidVelConT::CalcDeviation(){
	y = vc1 - vc_rhs;
}

void WholebodyCentroidPosConR::CalcDeviation(){
	quat_t qerror = q_rhs.Conjugated()*q1;
	vec3_t axis   = qerror.Axis ();
	real_t theta  = qerror.Theta();
	if(theta > _pi)
		theta -= 2*_pi;
	y = (1.0/2.0)*(q_rhs*(theta*axis) + q1*(theta*axis));
}

void WholebodyCentroidVelConR::CalcDeviation(){
	y = w1 - w_rhs;
}

void WholebodyDesPosConT::CalcDeviation(){
	y = (pc + q0*pi) - desired;
}

void WholebodyDesPosConR::CalcDeviation(){
	quat_t qerror = desired.Conjugated()*(q0*qi);
	vec3_t axis   = qerror.Axis ();
	real_t theta  = qerror.Theta();
	if(theta > _pi)
		theta -= 2*_pi;
	y = (1.0/2.0)*(desired*(theta*axis) + (q0*qi)*(theta*axis));
}

void WholebodyDesVelConT::CalcDeviation(){
	y = (vc + w0 % (q0*pi) + q0*vi) - desired;
}

void WholebodyDesVelConR::CalcDeviation(){
	y = (w0 + q0*wi) - desired;
}

void WholebodyLCon::CalcDeviation(){
	y = obj->data.centroid.Labs - desired;
}

void WholebodyContactPosConT::CalcDeviation(){
	y = qo.Conjugated()*(pc + q0*(pi + qi*r) - po);
}

void WholebodyContactPosConR::CalcDeviation(){
	quat_t qerror = qo.Conjugated()*(q0*qi);
	vec3_t axis   = qerror.Axis ();
	real_t theta  = qerror.Theta();
	if(theta > _pi)
		theta -= 2*_pi;
	y = (1.0/2.0)*(qo*(theta*axis) + (q0*qi)*(theta*axis));
}

void WholebodyContactVelConT::CalcDeviation(){
	y = qo.Conjugated()*(vc + w0 % (q0*(pi + qi*r)) + q0*vi);
}

void WholebodyContactVelConR::CalcDeviation(){
	y = qo.Conjugated()*(w0 + q0*wi);
}

void WholebodyNormalForceCon::CalcDeviation(){
	y[0] = fn;
	//DSTR << "fn: " << fn << endl;
	/*if(fn > 0.0){
		y[0] = 0.0;
		active = false;
	}
	else{
		y[0] = fn;
		active = true;
	}*/
}

void WholebodyFrictionForceCon::CalcDeviation(){
	y[0] = (side == 0 ? mu*fn - ft : ft + mu*fn);
	active = y[0] < 0.0;
	/*real_t e = (side == 0 ? mu*fn - ft : ft + mu*fn);
	if(e > 0.0){
		y[0] = 0.0;
		active = false;
	}
	else{
		y[0] = e;
		active = true;
	}*/		
}

void WholebodyMomentCon::CalcDeviation(){
	// -ty*m - (cop_min.x)*fn >= 0
	//  ty*m + (cop_max.x)*fn >= 0
	//  tx*m - (cop_min.y)*fn >= 0
	// -tx*m + (cop_max.y)*fn >= 0
	//  tz*m - (cop_min.z)*fn >= 0
	// -tz*m + (cop_max.z)*fn >= 0
	if(dir == 0){
		y[0] = (side == 0 ? (-m.y - cmin.x*fn) : ( m.y + cmax.x*fn));
	}
	if(dir == 1){
		y[0] = (side == 0 ? ( m.x - cmin.y*fn) : (-m.x + cmax.y*fn));
	}
	if(dir == 2){
		y[0] = (side == 0 ? ( m.z - cmin.z*fn) : ( -m.z + cmax.z*fn));
	}
	active = y[0] < 0.0;
	/*
	real_t e = (dir == 0 ? 
		(side == 0 ? (-m.y - cmin.x*fn) : ( m.y + cmax.x*fn)) :
		(side == 0 ? ( m.x - cmin.y*fn) : (-m.x + cmax.y*fn))
		);

	// always active in barrier mode
	//y[0] = std::max(1.0, e);
	if(e > 0.0){
		y[0] = 0.0;
		active = false;
	}
	else{
		y[0] = e;
		active = true;
	}
	*/
}

}
