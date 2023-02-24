#include <DiMP/Graph/Wholebody.h>
#include <DiMP/Graph/Graph.h>
#include <DiMP/Render/Config.h>
#include <DiMP/Render/Canvas.h>

#include <sbtimer.h>
static Timer timer;

namespace DiMP {;

const real_t pi      = M_PI;
const real_t inf     = numeric_limits<real_t>::max();
const real_t damping = 0.0;
const vec3_t one(1.0, 1.0, 1.0);

//-------------------------------------------------------------------------------------------------
// IKData

WholebodyData::End::End(){
	pos_t_weight   = one;
	pos_r_weight   = one;
	vel_t_weight   = one;
	vel_r_weight   = one;
	acc_t_weight   = one;
	acc_r_weight   = one;
	force_t_weight = one;
	force_r_weight = one;

	//offset = 0.0;
	state  = Wholebody::ContactState::Free;
	mu     = 0.5;
}

void WholebodyData::Init(Wholebody* wb){
	links.resize(wb->links .size());
	ends .resize(wb->ends  .size());
	q    .resize(wb->joints.size());
	qd   .resize(wb->joints.size());
	tau  .resize(wb->joints.size());
	e    .resize(wb->limits.size());

	com_pos_weight    = one;
	com_vel_weight    = one;
	base_pos_r_weight = one;
	base_vel_r_weight = one;
}

//-------------------------------------------------------------------------------------------------
// WholebodyKey

WholebodyKey::WholebodyKey() {
	
}

void WholebodyKey::AddVar(Solver* solver) {
	wb = (Wholebody*)node;

	int nend  = wb->ends .size();
	int nlink = wb->links.size();
	
	ends.resize(nend);
	stringstream ss, ss2;

	var_com_pos    = new V3Var(solver, ID(VarTag::WholebodyComPos, node, tick, name + "_com_pos"   ), wb->spt);
	var_com_vel    = new V3Var(solver, ID(VarTag::WholebodyComVel, node, tick, name + "_com_pos"   ), wb->svt);
	var_base_pos_r = new QVar (solver, ID(VarTag::WholebodyPosR  , node, tick, name + "_base_pos_r"), wb->spr);
	var_base_vel_r = new V3Var(solver, ID(VarTag::WholebodyVelR  , node, tick, name + "_base_vel_r"), wb->svr);
	var_com_pos   ->weight = damping*one;
	var_com_vel   ->weight = damping*one;
	var_base_pos_r->weight = damping*one;
	var_base_vel_r->weight = damping*one;
	solver->AddStateVar(var_com_pos   , tick->idx);
	solver->AddStateVar(var_com_vel   , tick->idx);
	solver->AddStateVar(var_base_pos_r, tick->idx);
	solver->AddStateVar(var_base_vel_r, tick->idx);

	for(int i = 0; i < nend; i++){
		ss.str("");
		ss << name << "_end" << i;
		
		ends[i].var_pos_t = new V3Var(solver, ID(VarTag::WholebodyPosT, node, tick, ss.str() + "_pos_t"), wb->spt);
		ends[i].var_pos_r = new QVar (solver, ID(VarTag::WholebodyPosR, node, tick, ss.str() + "_pos_r"), wb->spr);
		ends[i].var_vel_t = new V3Var(solver, ID(VarTag::WholebodyVelT, node, tick, ss.str() + "_vel_t"), wb->svt);
		ends[i].var_vel_r = new V3Var(solver, ID(VarTag::WholebodyVelR, node, tick, ss.str() + "_vel_r"), wb->svr);
		ends[i].var_pos_t->weight = damping*one;
		ends[i].var_pos_r->weight = damping*one;
		ends[i].var_vel_t->weight = damping*one;
		ends[i].var_vel_r->weight = damping*one;
		solver->AddStateVar(ends[i].var_pos_t, tick->idx);
		solver->AddStateVar(ends[i].var_pos_r, tick->idx);
		solver->AddStateVar(ends[i].var_vel_t, tick->idx);
		solver->AddStateVar(ends[i].var_vel_r, tick->idx);
		
		if(next){
			ends[i].var_acc_t   = new V3Var(solver, ID(VarTag::WholebodyAccT  , node, tick, ss.str() + "_acc_t"  ), wb->sat);
			ends[i].var_acc_r   = new V3Var(solver, ID(VarTag::WholebodyAccR  , node, tick, ss.str() + "_acc_r"  ), wb->sar);
			ends[i].var_force_t = new V3Var(solver, ID(VarTag::WholebodyForceT, node, tick, ss.str() + "_force_t"), wb->sft);
			ends[i].var_force_r = new V3Var(solver, ID(VarTag::WholebodyForceR, node, tick, ss.str() + "_force_r"), wb->sfr);
			ends[i].var_acc_t  ->weight = damping*one;
			ends[i].var_acc_r  ->weight = damping*one;
			ends[i].var_force_t->weight = damping*one;
			ends[i].var_force_r->weight = damping*one;
			solver->AddInputVar(ends[i].var_acc_t  , tick->idx);
			solver->AddInputVar(ends[i].var_acc_r  , tick->idx);
			solver->AddInputVar(ends[i].var_force_t, tick->idx);
			solver->AddInputVar(ends[i].var_force_r, tick->idx);
		}
	}

	data.Init(wb);
	for(int index = 0; index < 3; index++){
		data_tmp[index].Init(wb);
	}
	data_tmp2.resize(nend);
	for(int iend = 0; iend < nend; iend++){
		data_tmp2[iend].resize(6);
		for(int j = 0; j < 6; j++){
			data_tmp2[iend][j].Init(wb);
		}
	}
	
	J_e_pe .resize(nend);
	J_e_qe .resize(nend);
	J_pi_pe.resize(nend);
	J_pi_qe.resize(nend);
	
	int njoint = data.q.size();
	int nlimit = data.e.size();
	
	for(int i = 0; i < nend; i++){
		J_e_pe[i].resize(nlimit, 3);
		J_e_qe[i].resize(nlimit, 3);
		J_pi_pe[i].resize(nlink);
		J_pi_qe[i].resize(nlink);
	}
}

void WholebodyKey::AddCon(Solver* solver) {
	WholebodyKey* nextObj = (WholebodyKey*)next;

    int nend   = wb->ends  .size();
	int nchain = wb->chains.size();
	int nlimit = wb->limits.size();
    
	stringstream ss;

	if(next){
		con_com_pos    = new WholebodyComPosCon  (solver, name + "_com_pos"   , this, wb->spt);
		con_com_vel    = new WholebodyComVelCon  (solver, name + "_com_vel"   , this, wb->svt);
		con_base_pos_r = new WholebodyBasePosConR(solver, name + "_base_pos_r", this, wb->spr);
		con_base_vel_r = new WholebodyBaseVelConR(solver, name + "_base_vel_r", this, wb->svr);
		solver->AddTransitionCon(con_com_pos   , tick->idx);
		solver->AddTransitionCon(con_com_vel   , tick->idx);
		solver->AddTransitionCon(con_base_pos_r, tick->idx);
		solver->AddTransitionCon(con_base_vel_r, tick->idx);
	}
	if(prev){
		con_des_com_pos    = new FixConV3(solver, ID(ConTag::WholebodyComPos, node, tick, name + "_des_com_pos"   ), var_com_pos   , wb->spt);
		con_des_com_vel    = new FixConV3(solver, ID(ConTag::WholebodyComVel, node, tick, name + "_des_com_vel"   ), var_com_vel   , wb->svt);
		con_des_base_pos_r = new FixConQ (solver, ID(ConTag::WholebodyPosR  , node, tick, name + "_des_base_pos_r"), var_base_pos_r, wb->spr);
		con_des_base_vel_r = new FixConV3(solver, ID(ConTag::WholebodyVelR  , node, tick, name + "_des_base_vel_r"), var_base_vel_r, wb->svr);
		solver->AddCostCon(con_des_com_pos   , tick->idx);
		solver->AddCostCon(con_des_com_vel   , tick->idx);
		solver->AddCostCon(con_des_base_pos_r, tick->idx);
		solver->AddCostCon(con_des_base_vel_r, tick->idx);
	
		con_limit.resize(nlimit);
		for(int j = 0; j < nlimit; j++){
			ss.str("");
			ss << name << "_lim" << j;
			
			con_limit[j] = new WholebodyLimitCon(solver, ss.str(), this, j, wb->limits[j].type, wb->limits[j].scale);
			solver->AddCostCon(con_limit[j], tick->idx);
		}
	}
	for(int i = 0; i < nend; i++){
		ss.str("");
		ss << name << "_end" << i;

		if(next){
			ends[i].con_pos_t = new WholebodyPosConT(solver, ss.str() + "_pos_t", this, i, wb->spt);
			ends[i].con_pos_r = new WholebodyPosConR(solver, ss.str() + "_pos_r", this, i, wb->spr);
			ends[i].con_vel_t = new WholebodyVelConT(solver, ss.str() + "_vel_t", this, i, wb->svt);
			ends[i].con_vel_r = new WholebodyVelConR(solver, ss.str() + "_vel_r", this, i, wb->svr);        
			solver->AddTransitionCon(ends[i].con_pos_t, tick->idx);
			solver->AddTransitionCon(ends[i].con_pos_r, tick->idx);
			solver->AddTransitionCon(ends[i].con_vel_t, tick->idx);
			solver->AddTransitionCon(ends[i].con_vel_r, tick->idx);
		}
		if(prev){
			ends[i].con_des_pos_t = new WholebodyDesPosConT(solver, ss.str() + "_des_pos_t", this, i, wb->spt);
			ends[i].con_des_pos_r = new WholebodyDesPosConR(solver, ss.str() + "_des_pos_r", this, i, wb->spr);
			ends[i].con_des_vel_t = new WholebodyDesVelConT(solver, ss.str() + "_des_vel_t", this, i, wb->svt);
			ends[i].con_des_vel_r = new WholebodyDesVelConR(solver, ss.str() + "_des_vel_r", this, i, wb->svr);
			solver->AddCostCon(ends[i].con_des_pos_t, tick->idx);
			solver->AddCostCon(ends[i].con_des_pos_r, tick->idx);
			solver->AddCostCon(ends[i].con_des_vel_t, tick->idx);
			solver->AddCostCon(ends[i].con_des_vel_r, tick->idx);

			ends[i].con_contact_pos_t = new WholebodyContactPosConT(solver, ss.str() + "_contact_pos_t", this, i, wb->spt);
			ends[i].con_contact_pos_r = new WholebodyContactPosConR(solver, ss.str() + "_contact_pos_r", this, i, wb->spr);
			ends[i].con_contact_vel_t = new WholebodyContactVelConT(solver, ss.str() + "_contact_vel_t", this, i, wb->svt);
			ends[i].con_contact_vel_r = new WholebodyContactVelConR(solver, ss.str() + "_contact_vel_r", this, i, wb->svr);
			solver->AddCostCon(ends[i].con_contact_pos_t, tick->idx);
			solver->AddCostCon(ends[i].con_contact_pos_r, tick->idx);
			solver->AddCostCon(ends[i].con_contact_vel_t, tick->idx);
			solver->AddCostCon(ends[i].con_contact_vel_r, tick->idx);
		}

		if(next){
			ends[i].con_des_acc_t   = new FixConV3(solver, ID(ConTag::WholebodyAccT  , node, tick, ss.str() + "_des_acc_t"  ), ends[i].var_acc_t  , wb->sat);
			ends[i].con_des_acc_r   = new FixConV3(solver, ID(ConTag::WholebodyAccR  , node, tick, ss.str() + "_des_acc_r"  ), ends[i].var_acc_r  , wb->sar);
			ends[i].con_des_force_t = new FixConV3(solver, ID(ConTag::WholebodyForceT, node, tick, ss.str() + "_des_force_t"), ends[i].var_force_t, wb->sft);
			ends[i].con_des_force_r = new FixConV3(solver, ID(ConTag::WholebodyForceR, node, tick, ss.str() + "_des_force_r"), ends[i].var_force_r, wb->sfr);
			solver->AddCostCon(ends[i].con_des_acc_t  , tick->idx);
			solver->AddCostCon(ends[i].con_des_acc_r  , tick->idx);
			solver->AddCostCon(ends[i].con_des_force_t, tick->idx);
			solver->AddCostCon(ends[i].con_des_force_r, tick->idx);
		
			ends[i].con_force_normal         = new WholebodyNormalForceCon  (solver, ss.str() + "_force_normal"  , this, i, wb->sft);
		    ends[i].con_force_friction[0][0] = new WholebodyFrictionForceCon(solver, ss.str() + "_force_friction", this, i, 0, 0, wb->sft);
		    ends[i].con_force_friction[0][1] = new WholebodyFrictionForceCon(solver, ss.str() + "_force_friction", this, i, 0, 1, wb->sft);
		    ends[i].con_force_friction[1][0] = new WholebodyFrictionForceCon(solver, ss.str() + "_force_friction", this, i, 1, 0, wb->sft);
		    ends[i].con_force_friction[1][1] = new WholebodyFrictionForceCon(solver, ss.str() + "_force_friction", this, i, 1, 1, wb->sft);
			ends[i].con_moment[0][0]         = new WholebodyMomentCon       (solver, ss.str() + "_moment"        , this, i, 0, 0, wb->sfr);
		    ends[i].con_moment[0][1]         = new WholebodyMomentCon       (solver, ss.str() + "_moment"        , this, i, 0, 1, wb->sfr);
		    ends[i].con_moment[1][0]         = new WholebodyMomentCon       (solver, ss.str() + "_moment"        , this, i, 1, 0, wb->sfr);
		    ends[i].con_moment[1][1]         = new WholebodyMomentCon       (solver, ss.str() + "_moment"        , this, i, 1, 1, wb->sfr);
		    
			solver->AddCostCon(ends[i].con_force_normal        , tick->idx);
            solver->AddCostCon(ends[i].con_force_friction[0][0], tick->idx);
            solver->AddCostCon(ends[i].con_force_friction[0][1], tick->idx);
            solver->AddCostCon(ends[i].con_force_friction[1][0], tick->idx);
            solver->AddCostCon(ends[i].con_force_friction[1][1], tick->idx);
            solver->AddCostCon(ends[i].con_moment[0][0]        , tick->idx);
            solver->AddCostCon(ends[i].con_moment[0][1]        , tick->idx);
            solver->AddCostCon(ends[i].con_moment[1][0]        , tick->idx);
            solver->AddCostCon(ends[i].con_moment[1][1]        , tick->idx);
        }
	}
}

void WholebodyKey::Prepare() {
	int nlink = wb->links.size();
	int nend  = wb->ends .size();

	// copy variables to data
	data.com_pos    = var_com_pos   ->val;
	data.com_vel    = var_com_vel   ->val;
	data.base_pos_r = var_base_pos_r->val;
	data.base_vel_r = var_base_vel_r->val;

	for(int i = 0; i < nend; i++){
		End& end = ends[i];

		WholebodyData::End & dend = data.ends [i];
		
		// set pos and vel
		dend.pos_t = end.var_pos_t->val;
		dend.pos_r = end.var_pos_r->val;
		dend.vel_t = end.var_vel_t->val;
		dend.vel_r = end.var_vel_r->val;

		// update complimentarity
		if(next){
			// set compl variable to acc|force
			dend.acc_t   = end.var_acc_t  ->val;
			dend.acc_r   = end.var_acc_r  ->val;
			dend.force_t = end.var_force_t->val;
			dend.force_r = end.var_force_r->val;
		}
	}

	wb->CalcPVA(data_tmp, data);
	wb->CalcForce(data);
	//DSTR << data.e << endl;
}

void WholebodyKey::PrepareStep(){
	timer.CountUS();

	int nend  = wb->ends .size();
	int nlink = wb->links.size();

	const real_t eps    = 1.0e-2;
	const real_t epsinv = 1.0/eps;

	for(int iend = 0; iend < nend; iend++){
		for(int j = 0; j < 3; j++){
			WholebodyData& d = data_tmp2[iend][j+0];
			d = data;

			d.ends[iend].pos_t[j] += eps;
			wb->CalcPosition(d);

			for(int i = 0; i < nlink; i++){
				J_pi_pe[iend][i].col(j) = (d.links[i].pos_t - data.links[i].pos_t)*epsinv;
			}
			J_e_pe[iend].col(j) = (d.e - data.e)*epsinv;

		}
		for(int j = 0; j < 3; j++){
			WholebodyData& d = data_tmp2[iend][j+3];
			d = data;

			d.ends[iend].pos_r = quat_t::Rot(eps, 'x' + (j % 3))*d.ends[iend].pos_r;
			d.ends[iend].pos_r.unitize();
			wb->CalcPosition(d);

			for(int i = 0; i < nlink; i++){
				J_pi_qe[iend][i].col(j) = (d.links[i].pos_t - data.links[i].pos_t)*epsinv;
			}
			J_e_qe[iend].col(j) = (d.e - data.e)*epsinv;
		}
	}
						
	int T = timer.CountUS();
	//DSTR << "prepare step: " << T << endl;
}

void WholebodyKey::Finish(){

}

void WholebodyKey::Draw(Render::Canvas* canvas, Render::Config* conf) {

}

//-------------------------------------------------------------------------------------------------
// Wholebody

Wholebody::Param::Param() {
	gravity = 9.8;
}

//-------------------------------------------------------------------------------------------------

Wholebody::Link::Link(real_t _mass, bool _is_end, int _iparent, int _ijoint, vec3_t _trn, vec3_t _axis){
	mass    = _mass;
	is_end  = _is_end;
	iparent = _iparent;
	ijoint  = _ijoint;
	trn     = _trn;
	axis    = _axis;
}

//-------------------------------------------------------------------------------------------------

Wholebody::End::End(){
	ilink = 0;
}

//-------------------------------------------------------------------------------------------------

Wholebody::Limit::Limit(int _ichain, int _type, real_t _scale){
	ichain = _ichain;
	type   = _type;
	scale  = _scale;
}

//-------------------------------------------------------------------------------------------------

Wholebody::Chain::Chain(const vector<int>& _ilink, const vector<int>& _ilimit){
	ilink  = _ilink;
	ilimit = _ilimit;
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
	param.total_mass = 0.0;
	for(int i = 0; i < links.size(); i++){
		param.total_mass += links[i].mass;
	}
	for(int i = 0; i < links.size(); i++){
		links[i].mass_ratio = links[i].mass/param.total_mass;
	}

	sl  = 1.0;  //< unit length
	st  = graph->ticks[1]->time - graph->ticks[0]->time;
	sat = param.gravity;
	svt = sat*st;
	sft = param.total_mass*param.gravity;
	spt = svt*st;
	spr = spt/sl;
	svr = svt/sl;
	sar = sat/sl;
	sfr = sft*sl;
	sL  = sfr*st;
}

void Wholebody::Setup(){
	int nend = ends.size();

	for (int k = 0; k < graph->ticks.size(); k++) {
		WholebodyKey* key = (WholebodyKey*)traj.GetKeypoint(graph->ticks[k]);

		real_t t = graph->ticks[k]->time;

		callback->Setup(k, t, key->data);

		if(k == 0){
			key->var_com_pos   ->val = key->data.com_pos_ini;
			key->var_com_vel   ->val = key->data.com_vel_ini;
			key->var_base_pos_r->val = key->data.base_pos_r_ini;
			key->var_base_vel_r->val = key->data.base_vel_r_ini;
			key->var_com_pos   ->locked = true;
			key->var_com_vel   ->locked = true;
			key->var_base_pos_r->locked = true;
			key->var_base_vel_r->locked = true;
		}
		else{
			key->var_com_pos   ->val = key->con_des_com_pos   ->desired = key->data.com_pos_des;
			key->var_com_vel   ->val = key->con_des_com_vel   ->desired = key->data.com_vel_des;
			key->var_base_pos_r->val = key->con_des_base_pos_r->desired = key->data.base_pos_r_des;
			key->var_base_vel_r->val = key->con_des_base_vel_r->desired = key->data.base_vel_r_des;
			key->con_des_com_pos   ->weight = key->data.com_pos_weight;
			key->con_des_com_vel   ->weight = key->data.com_vel_weight;
			key->con_des_base_pos_r->weight = key->data.base_pos_r_weight;
			key->con_des_base_vel_r->weight = key->data.base_vel_r_weight;			
		}
		for(int i = 0; i < nend; i++){
			WholebodyKey ::End&  end  = key->ends[i];
			WholebodyData::End&  dend = key->data.ends [i];
			
			if(k == 0){
				end.var_pos_t->val = dend.pos_t_ini;
				end.var_pos_r->val = dend.pos_r_ini;
				end.var_vel_t->val = dend.vel_t_ini;
				end.var_vel_r->val = dend.vel_r_ini;
				end.var_pos_t->locked = true;
				end.var_pos_r->locked = true;
				end.var_vel_t->locked = true;
				end.var_vel_r->locked = true;

				end.var_acc_t  ->val = dend.acc_t_ini;
				end.var_acc_r  ->val = dend.acc_r_ini;
				end.var_force_t->val = dend.force_t_ini;
				end.var_force_r->val = dend.force_r_ini;
				end.var_acc_t  ->locked = true;
				end.var_acc_r  ->locked = true;
				end.var_force_t->locked = true;
				end.var_force_r->locked = true;
			}
			else{
				vec3_t pc_des = key->data.com_pos_des;
				vec3_t vc_des = key->data.com_vel_des;
				quat_t q0_des = key->data.base_pos_r_des;
				vec3_t w0_des = key->data.base_vel_r_des;
				end.var_pos_t->val = q0_des.Conjugated()*(dend.pos_t_des - pc_des);
				end.var_pos_r->val = q0_des.Conjugated()* dend.pos_r_des;
				end.var_vel_t->val = q0_des.Conjugated()*(dend.vel_t_des - (vc_des + w0_des % (dend.pos_t_des - pc_des)));
				end.var_vel_r->val = q0_des.Conjugated()*(dend.vel_r_des - w0_des);
				
				end.con_des_pos_t->desired = dend.pos_t_des;
				end.con_des_pos_r->desired = dend.pos_r_des;
				end.con_des_vel_t->desired = dend.vel_t_des;
				end.con_des_vel_r->desired = dend.vel_r_des;
				end.con_des_pos_t->weight  = dend.pos_t_weight;
				end.con_des_pos_r->weight  = dend.pos_r_weight;
				end.con_des_vel_t->weight  = dend.vel_t_weight;
				end.con_des_vel_r->weight  = dend.vel_r_weight;

				end.con_contact_pos_t->active = (dend.state != ContactState::Free);
				end.con_contact_pos_r->active = (dend.state != ContactState::Free);
				end.con_contact_pos_r->weight = vec3_t(1.0, 1.0, 0.0);  //< do not constrain in contact normal direction
				end.con_contact_vel_t->active = (dend.state != ContactState::Free);
				end.con_contact_vel_r->active = (dend.state != ContactState::Free);
				
				if(key->next){
					end.var_acc_t  ->val = dend.acc_t_des;
					end.var_acc_r  ->val = dend.acc_r_des;
					end.con_des_acc_t  ->desired = dend.acc_t_des;
					end.con_des_acc_r  ->desired = dend.acc_r_des;
					end.con_des_acc_t  ->weight = dend.acc_t_weight;
					end.con_des_acc_r  ->weight = dend.acc_r_weight;
					
					if(dend.state == ContactState::Free){
						end.var_force_t->val.clear();
						end.var_force_r->val.clear();
						end.var_force_t->locked = true;
						end.var_force_r->locked = true;
						end.con_des_force_t->active = false;
						end.con_des_force_r->active = false;
					}
					else{
						end.var_force_t->val = dend.force_t_des;
						end.var_force_r->val = dend.force_r_des;
						end.var_force_t->locked = false;
						end.var_force_r->locked = false;
						end.con_des_force_t->active = true;
						end.con_des_force_r->active = true;
						end.con_des_force_t->desired = dend.force_t_des;
						end.con_des_force_r->desired = dend.force_r_des;
						end.con_des_force_t->weight = dend.force_t_weight;
						end.con_des_force_r->weight = dend.force_r_weight;
					}

					end.con_force_normal        ->active = (dend.state != ContactState::Free);
					end.con_force_friction[0][0]->active = (dend.state != ContactState::Free);
					end.con_force_friction[0][1]->active = (dend.state != ContactState::Free);
					end.con_force_friction[1][0]->active = (dend.state != ContactState::Free);
					end.con_force_friction[1][1]->active = (dend.state != ContactState::Free);
					end.con_moment[0][0]        ->active = (dend.state != ContactState::Free);
					end.con_moment[0][1]        ->active = (dend.state != ContactState::Free);
					end.con_moment[1][0]        ->active = (dend.state != ContactState::Free);
					end.con_moment[1][1]        ->active = (dend.state != ContactState::Free);
				}
			}			
		}
	}

	trajReady = false;
}

void Wholebody::Init() {
	TrajectoryNode::Init();

	int nlink  = links .size();
	int nend   = ends  .size();
	int nchain = chains.size();
	
	for(int i = 0; i < nlink; i++){
		if(links[i].iparent != -1)
			links[links[i].iparent].ichildren.push_back(i);
	}
	
	// call prepare here so that initial trajectory is visualized properly
	Setup  ();
    Prepare();

    trajReady = false;
}

void Wholebody::Prepare() {
	trajReady = false;

	TrajectoryNode::Prepare();
}

void Wholebody::Finish(){
	TrajectoryNode::Finish();
}

void Wholebody::CalcPosition(WholebodyData& d){
	int nlink = links.size();
	int nend  = ends.size();

	// calc base link position by iteration
	const int niter = 15;
	const real_t gain = 1.0;

	// initialize base link pose (com local)
	d.links[0].pos_t = vec3_t();
	d.links[0].pos_r = quat_t();
	for(int n = 0; n <= niter; n++){
		// copy end pose to link
		for(int i = 0; i < nend; i++){
			d.links[ends[i].ilink].pos_t = d.ends[i].pos_t;
			d.links[ends[i].ilink].pos_r = d.ends[i].pos_r;
		}
		callback->CalcIK(d);
		CalcLinkPosition(d);

		if(n == niter)
			break;

		vec3_t pc;
		for(int i = 0; i < nlink; i++){
			pc += links[i].mass_ratio*d.links[i].pos_t;
		}
		
		d.links[0].pos_t -= gain*pc;
		//DSTR << pc << endl;
	}
}

void Wholebody::CalcLinkPosition(WholebodyData& d){
	for(Chain& ch : chains){
		// calc child link poses
		for(int i : ch.ilink){
			int ip = links[i].iparent;
			WholebodyData::Link& dlnk  = d.links[i ];
			WholebodyData::Link& dlnkp = d.links[ip];

			dlnk.pos_t = dlnkp.pos_t + dlnkp.pos_r*links[i].trn;
			dlnk.pos_r = dlnkp.pos_r*quat_t::Rot(d.q[links[i].ijoint], links[i].axis);
			dlnk.pos_r.unitize();
		}
	}
}

void Wholebody::CalcComAcceleration (WholebodyData& d){
	vec3_t fsum;
	
	int nend = ends.size();
    for(int i = 0; i < nend; i++){
        WholebodyData::End& dend = d.ends[i];
        fsum += d.base_pos_r*dend.pos_r*dend.force_t;
    }
    d.com_acc = (1.0/param.total_mass)*fsum - vec3_t(0.0, 0.0, param.gravity);
}

void Wholebody::CalcBaseAcceleration(WholebodyData& d){
	vec3_t msum;
	
	int nend = ends.size();
    for(int i = 0; i < nend; i++){
        WholebodyData::End& dend = d.ends[i];
        msum += d.base_pos_r*(dend.pos_r*dend.force_r + dend.pos_t % (dend.pos_r*dend.force_t));
    }
    d.base_acc_r = d.Iinv*(msum - d.base_pos_r*d.Ld);
}
	
void Wholebody::CalcVelocity(const WholebodyData& d0, const WholebodyData& d1, WholebodyData& d, real_t epsinv){
	int nlink = links.size();

	for(int i = 0; i < nlink; i++){
		const WholebodyData::Link& dlnk0 = d0.links[i];
		const WholebodyData::Link& dlnk1 = d1.links[i];
		      WholebodyData::Link& dlnk  = d .links[i];
		
		dlnk.vel_t = (dlnk1.pos_t - dlnk0.pos_t)*epsinv;
						
		quat_t qdiff = dlnk1.pos_r*dlnk0.pos_r.Conjugated();
		real_t theta = qdiff.Theta();
		if(theta > pi)
			theta -= 2*pi;
		dlnk.vel_r = qdiff.Axis()*(theta*epsinv);
	}

	// calc joint velocity
	d.qd = (d1.q - d0.q)*epsinv;

	CalcMomentum(d);
}

void Wholebody::CalcAcceleration(const WholebodyData& d0, const WholebodyData& d1, WholebodyData& d, real_t epsinv){
	int nlink = links.size();

	for(int i = 0; i < nlink; i++){
		const WholebodyData::Link& dlnk0 = d0.links[i];
		const WholebodyData::Link& dlnk1 = d1.links[i];
		      WholebodyData::Link& dlnk  = d .links[i];
		
		dlnk.acc_t = (dlnk1.vel_t - dlnk0.vel_t)*epsinv;
		dlnk.acc_r = (dlnk1.vel_r - dlnk0.vel_r)*epsinv;
	}

	CalcMomentumDerivative(d);
}

void Wholebody::CalcMomentum(WholebodyData& d){
	int nlink = links.size();

	// calc inertial matrix
	d.I.clear();
	for(int i = 0; i < nlink; i++){
		vec3_t r = d.base_pos_r*d.links[i].pos_t;
		mat3_t rc = mat3_t::Cross(r);
		d.I += links[i].mass*(rc*rc.trans());
	}
	d.Iinv = d.I.inv();

	// calc momentum in local coordinate
	d.L.clear();
	for(int i = 0; i < nlink; i++){
		WholebodyData::Link& dlnk  = d.links[i];

		d.L += dlnk.pos_t % (links[i].mass*dlnk.vel_t);
	}
}

void Wholebody::CalcMomentumDerivative(WholebodyData& d){
	int nlink = links.size();

	// calc momentum derivative in local coordinate
	d.Ld.clear();
	for(int i = 0; i < nlink; i++){
		WholebodyData::Link& dlnk  = d.links[i];

		d.Ld += dlnk.pos_t % (links[i].mass*dlnk.acc_t);
	}
}
	
void Wholebody::CalcPVA(WholebodyData* data_tmp, WholebodyData& data){
	int nend = ends.size();

	CalcPosition(data);

	// calc velocity and acceleration by finite difference
	const real_t eps    = 0.01;
	const real_t epsinv = 1.0/eps;
	const real_t eps2   = eps*eps;
	
	// index
	//  0 : p+vh
	//  1 : p+vh+(1/2)ah^2
	//  2 : p-vh+(1/2)ah^2
	for(int index = 0; index < 3; index++){
		WholebodyData& d = data_tmp[index];
		d = data;

		for(int i = 0; i < nend; i++){
			WholebodyData::End& dend = d.ends[i];

			if(index == 0){
				dend.pos_t = dend.pos_t + dend.vel_t*eps;
				dend.pos_r = quat_t::Rot( dend.vel_r*eps)*dend.pos_r;
			}
			if(index == 1){
				dend.pos_t = dend.pos_t + dend.vel_t*eps + dend.acc_t*(0.5*eps2);
				dend.pos_r = quat_t::Rot( dend.vel_r*eps + dend.acc_r*(0.5*eps2))*dend.pos_r;
			}
			if(index == 2){
				dend.pos_t = dend.pos_t - dend.vel_t*eps + dend.acc_t*(0.5*eps2);
				dend.pos_r = quat_t::Rot(-dend.vel_r*eps + dend.acc_r*(0.5*eps2))*dend.pos_r;
			}
			dend.pos_r.unitize();
		}

		CalcPosition(d);
	}

	CalcVelocity    (data       , data_tmp[0], data       , epsinv);
	CalcVelocity    (data       , data_tmp[1], data_tmp[1], epsinv);
	CalcVelocity    (data_tmp[2], data       , data_tmp[2], epsinv);
	CalcAcceleration(data_tmp[2], data_tmp[1], data       , epsinv);
}

void Wholebody::CalcForce(WholebodyData & d){
	int nchain = chains.size();
	int nend   = ends.size();

	// transform and copy end forces to links
	for(int i = 0; i < nend; i++){
		WholebodyData::End&  dend = d.ends[i];
	    WholebodyData::Link& dlnk = d.links[ends[i].ilink];
		
		dlnk.force_t = d.base_pos_r*dend.pos_r*dend.force_t;
		dlnk.force_r = d.base_pos_r*dend.pos_r*dend.force_r;
	}

	vec3_t ac = d.com_acc;
	quat_t q0 = d.base_pos_r;
	vec3_t w0 = d.base_vel_r;
	vec3_t u0 = d.base_acc_r;

	// trace chains in reverse order
	for(int ic = nchain-1; ic >= 0; ic--){
		int nlink = chains[ic].ilink.size();
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

			// pi = pc * q0 * pihat;
			// vi = vc + w0 % (q0*pihat) + q0*vihat
			// ai = ac + u0 % (q0*pihat) + w0 % (w0 % (q0*pihat)) + 2*w0 % (q0*vihat) + q0*aihat;
			vec3_t acc_t_abs = ac + u0 % (q0*dlnk.pos_t) + w0 % (w0 % (q0*dlnk.pos_t)) + 2.0*(w0 % (q0*dlnk.vel_t)) + q0*dlnk.acc_t;
					
			dlnk.force_t_par = links[i].mass*(acc_t_abs + vec3_t(0.0, 0.0, param.gravity)) - dlnk.force_t - dlnk.force_t_child;
			dlnk.force_r_par = -dlnk.force_r - dlnk.force_r_child;
				
			// calc joint torque
			d.tau[links[i].ijoint] = links[i].axis*dlnk.force_r_par;
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
		    k0->tick->time, k0->data.com_pos, k0->data.com_vel,
		    k1->tick->time, k1->data.com_pos, k1->data.com_vel,
		    Interpolate::LinearDiff);
        vel = InterpolateVel(
		    t,
		    k0->tick->time, k0->data.com_pos, k0->data.com_vel,
		    k1->tick->time, k1->data.com_pos, k1->data.com_vel,
		    Interpolate::LinearDiff);
    }
    else{
        pos = k0->data.com_pos;
		vel = k0->data.com_vel;
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
		    k0->tick->time, k0->data.base_pos_r, k0->data.base_vel_r,
		    k1->tick->time, k1->data.base_pos_r, k1->data.base_vel_r,
		    Interpolate::LinearDiff);
        angvel = k0->data.base_vel_r;
    }
    else{
        ori    = k0->data.base_pos_r;
        angvel = k0->data.base_vel_r;
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
		    Interpolate::LinearDiff);
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
			vec3_t pc = trajectory[0].com_pos;
			quat_t q0 = trajectory[0].base_pos_r;
			vec3_t pi = trajectory[0].links[i].pos_t;
			canvas->Point(Vec3f(pc + q0*pi));
			canvas->BeginPath();
			canvas->MoveTo(pc + q0*pi);
			for (int k = 1; k < trajectory.size(); k++) {
				pc = trajectory[k].com_pos;
				q0 = trajectory[k].base_pos_r;
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
    
	ComState (t, s.com_pos, s.com_vel);
	BaseState(t, s.base_pos_r, s.base_vel_r);

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
		vec3_t pc = snapshot.com_pos;
		quat_t q0 = snapshot.base_pos_r;

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

WholebodyPosConT::WholebodyPosConT(Solver* solver, string _name, WholebodyKey* _obj, int _iend, real_t _scale):
	WholebodyCon(solver, 3, ConTag::WholebodyPosT, _name, _obj, _scale) {

	iend = _iend;

	AddSLink(obj[1]->ends[iend].var_pos_t);
	AddSLink(obj[0]->ends[iend].var_pos_t);
	AddSLink(obj[0]->ends[iend].var_vel_t);
}

WholebodyPosConR::WholebodyPosConR(Solver* solver, string _name, WholebodyKey* _obj, int _iend, real_t _scale):
	WholebodyCon(solver, 3, ConTag::WholebodyPosR, _name, _obj, _scale) {

	iend = _iend;

	AddSLink(obj[1]->ends[iend].var_pos_r);
	AddSLink(obj[0]->ends[iend].var_pos_r);
	AddSLink(obj[0]->ends[iend].var_vel_r);
}

WholebodyVelConT::WholebodyVelConT(Solver* solver, string _name, WholebodyKey* _obj, int _iend, real_t _scale):
	WholebodyCon(solver, 3, ConTag::WholebodyVelT, _name, _obj, _scale) {

	iend = _iend;

	AddSLink(obj[1]->ends[iend].var_vel_t);
	AddSLink(obj[0]->ends[iend].var_vel_t);
	AddSLink(obj[0]->ends[iend].var_acc_t);
}

WholebodyVelConR::WholebodyVelConR(Solver* solver, string _name, WholebodyKey* _obj, int _iend, real_t _scale):
	WholebodyCon(solver, 3, ConTag::WholebodyVelR, _name, _obj, _scale) {

	iend = _iend;

	AddSLink(obj[1]->ends[iend].var_vel_r);
	AddSLink(obj[0]->ends[iend].var_vel_r);
	AddSLink(obj[0]->ends[iend].var_acc_r);
}

WholebodyComPosCon::WholebodyComPosCon(Solver* solver, string _name, WholebodyKey* _obj, real_t _scale):
	WholebodyCon(solver, 3, ConTag::WholebodyComPos, _name, _obj, _scale) {

	AddSLink(obj[1]->var_com_pos);
	AddSLink(obj[0]->var_com_pos);
	AddSLink(obj[0]->var_com_vel);
}

WholebodyComVelCon::WholebodyComVelCon(Solver* solver, string _name, WholebodyKey* _obj, real_t _scale):
	WholebodyCon(solver, 3, ConTag::WholebodyComVel, _name, _obj, _scale) {

	AddSLink(obj[1]->var_com_vel);
	AddSLink(obj[0]->var_com_vel);
	int nend = obj[0]->ends.size();
	for(int i = 0; i < nend; i++){
		AddM3Link(obj[0]->ends[i].var_force_t);
	}
}

WholebodyBasePosConR::WholebodyBasePosConR(Solver* solver, string _name, WholebodyKey* _obj, real_t _scale):
	WholebodyCon(solver, 3, ConTag::WholebodyPosR, _name, _obj, _scale) {

	AddSLink(obj[1]->var_base_pos_r);
	AddSLink(obj[0]->var_base_pos_r);
	AddSLink(obj[0]->var_base_vel_r);
}

WholebodyBaseVelConR::WholebodyBaseVelConR(Solver* solver, string _name, WholebodyKey* _obj, real_t _scale):
	WholebodyCon(solver, 3, ConTag::WholebodyVelR, _name, _obj, _scale) {

	AddSLink(obj[1]->var_base_vel_r);
	AddSLink(obj[0]->var_base_vel_r);
	int nend = obj[0]->ends.size();
	for(int i = 0; i < nend; i++){
		AddM3Link(obj[0]->ends[i].var_pos_t);
		AddM3Link(obj[0]->ends[i].var_pos_r);
		AddM3Link(obj[0]->ends[i].var_acc_t);
		AddM3Link(obj[0]->ends[i].var_acc_r);
		AddM3Link(obj[0]->ends[i].var_force_t);
		AddM3Link(obj[0]->ends[i].var_force_r);
	}

	J_Ld_pe.resize(nend);
	J_Ld_qe.resize(nend);
	J_Ld_ae.resize(nend);
	J_Ld_ue.resize(nend);
}

WholebodyDesPosConT::WholebodyDesPosConT(Solver* solver, string _name, WholebodyKey* _obj, int _iend, real_t _scale):
	Constraint(solver, 3, ID(ConTag::WholebodyPosT, _obj->node, _obj->tick, _name), Constraint::Type::Equality, _scale){
	obj = _obj;
	iend = _iend;

	AddSLink (obj->var_com_pos);
	AddX3Link(obj->var_base_pos_r);
	AddM3Link(obj->ends[iend].var_pos_t);
}

WholebodyDesPosConR::WholebodyDesPosConR(Solver* solver, string _name, WholebodyKey* _obj, int _iend, real_t _scale):
	Constraint(solver, 3, ID(ConTag::WholebodyPosR, _obj->node, _obj->tick, _name), Constraint::Type::Equality, _scale){
	obj = _obj;
	iend = _iend;

	AddSLink (obj->var_base_pos_r);
	AddM3Link(obj->ends[iend].var_pos_r);
}

WholebodyDesVelConT::WholebodyDesVelConT(Solver* solver, string _name, WholebodyKey* _obj, int _iend, real_t _scale):
	Constraint(solver, 3, ID(ConTag::WholebodyVelT, _obj->node, _obj->tick, _name), Constraint::Type::Equality, _scale){
	obj = _obj;
	iend = _iend;

	AddSLink (obj->var_com_vel);
	AddX3Link(obj->var_base_vel_r);
	AddM3Link(obj->ends[iend].var_vel_t);
}

WholebodyDesVelConR::WholebodyDesVelConR(Solver* solver, string _name, WholebodyKey* _obj, int _iend, real_t _scale):
	Constraint(solver, 3, ID(ConTag::WholebodyVelR, _obj->node, _obj->tick, _name), Constraint::Type::Equality, _scale){
	obj = _obj;
	iend = _iend;

	AddSLink (obj->var_base_vel_r);
	AddM3Link(obj->ends[iend].var_vel_r);
}

WholebodyLimitCon::WholebodyLimitCon(Solver* solver, string _name, WholebodyKey* _obj, int _ierror, int _type, real_t _scale):
	Constraint(solver, 1, ID(ConTag::WholebodyLimit, _obj->node, _obj->tick, _name), _type, _scale){
	obj    = _obj;
	ierror = _ierror;
    
	int nend = obj->ends.size();
	for(int i = 0; i < nend; i++){
		AddR3Link(obj->ends[i].var_pos_t);
		AddR3Link(obj->ends[i].var_pos_r);
	}
}

WholebodyContactPosConT::WholebodyContactPosConT(Solver* solver, string _name, WholebodyKey* _obj, int _iend, real_t _scale):
	Constraint(solver, 1, ID(ConTag::WholebodyContactPosT, _obj->node, _obj->tick, _name), Constraint::Type::Equality, _scale){
	obj  = _obj;
	iend = _iend;
    
	AddR3Link(obj->var_com_pos);
	AddR3Link(obj->var_base_pos_r);
	AddR3Link(obj->ends[iend].var_pos_t);
}

WholebodyContactPosConR::WholebodyContactPosConR(Solver* solver, string _name, WholebodyKey* _obj, int _iend, real_t _scale):
	Constraint(solver, 3, ID(ConTag::WholebodyContactPosR, _obj->node, _obj->tick, _name), Constraint::Type::Equality, _scale){
	obj  = _obj;
	iend = _iend;
    
	AddSLink (obj->var_base_pos_r);
	AddM3Link(obj->ends[iend].var_pos_r);
}

WholebodyContactVelConT::WholebodyContactVelConT(Solver* solver, string _name, WholebodyKey* _obj, int _iend, real_t _scale):
	Constraint(solver, 3, ID(ConTag::WholebodyContactVelT, _obj->node, _obj->tick, _name), Constraint::Type::Equality, _scale){
	obj  = _obj;
	iend = _iend;
    
	AddSLink (obj->var_com_vel);
	AddX3Link(obj->var_base_vel_r);
	AddM3Link(obj->ends[iend].var_vel_t);
}

WholebodyContactVelConR::WholebodyContactVelConR(Solver* solver, string _name, WholebodyKey* _obj, int _iend, /*int _dir, */real_t _scale):
	Constraint(solver, 3, ID(ConTag::WholebodyContactVelR, _obj->node, _obj->tick, _name), Constraint::Type::Equality, _scale){
	obj  = _obj;
	iend = _iend;
	
	AddSLink (obj->var_base_vel_r);
	AddM3Link(obj->ends[iend].var_vel_r);
}

WholebodyNormalForceCon::WholebodyNormalForceCon(Solver* solver, string _name, WholebodyKey* _obj, int _iend, real_t _scale):
	Constraint(solver, 1, ID(ConTag::WholebodyNormalForce, _obj->node, _obj->tick, _name), Constraint::Type::InequalityPenalty, _scale){
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

void WholebodyPosConT::Prepare(){
	WholebodyKey::End& end0 = obj[0]->ends[iend];
	WholebodyKey::End& end1 = obj[1]->ends[iend];

	p0 = end0.var_pos_t->val;
	v0 = end0.var_vel_t->val;
	p1 = end1.var_pos_t->val;
	
	p_rhs = p0 + v0*obj[0]->hnext;
}

void WholebodyPosConR::Prepare(){
	WholebodyKey::End& end0 = obj[0]->ends[iend];
	WholebodyKey::End& end1 = obj[1]->ends[iend];

	q0 = end0.var_pos_r->val;
	w0 = end0.var_vel_r->val;
	q1 = end1.var_pos_r->val;
	
	q_rhs = q0*quat_t::Rot(q0.Conjugated()*(w0*obj[0]->hnext));
}

void WholebodyVelConT::Prepare(){
	WholebodyKey::End& end0 = obj[0]->ends[iend];
	WholebodyKey::End& end1 = obj[1]->ends[iend];

	v0 = end0.var_vel_t->val;
	a0 = end0.var_acc_t->val;
	v1 = end1.var_vel_t->val;

	v_rhs = v0 + a0*obj[0]->hnext;
}

void WholebodyVelConR::Prepare(){
	WholebodyKey::End& end0 = obj[0]->ends[iend];
	WholebodyKey::End& end1 = obj[1]->ends[iend];

	w0 = end0.var_vel_r->val;
	u0 = end0.var_acc_r->val;
	w1 = end1.var_vel_r->val;

	w_rhs = w0 + u0*obj[0]->hnext;
}

void WholebodyComPosCon::Prepare(){
	pc0 = obj[0]->var_com_pos->val;
	vc0 = obj[0]->var_com_vel->val;
	pc1 = obj[1]->var_com_pos->val;
	
	pc_rhs = pc0 + vc0*obj[0]->hnext;
}

void WholebodyComVelCon::Prepare(){
	vc0 = obj[0]->var_com_vel->val;
	vc1 = obj[1]->var_com_vel->val;
	q0  = obj[0]->var_base_pos_r->val;
	q0.ToMatrix(R0);
	
	int nend  = obj[0]->ends.size();

	R .resize(nend);
	f .resize(nend);

	fsum.clear();
	for(int i = 0; i < nend; i++){
		WholebodyData::End& dend = obj[0]->data.ends[i];
		WholebodyKey::End& end = obj[0]->ends[i];

		end.var_pos_r->val.ToMatrix(R[i]);
		f[i] = end.var_force_t->val;
		
		fsum += R0*R[i]*f[i];
	}

	vec3_t g(0.0, 0.0, -obj[0]->wb->param.gravity);
	vc_rhs = vc0 + (obj[0]->hnext/obj[0]->wb->param.total_mass)*fsum + obj[0]->hnext*g;
}

void WholebodyBasePosConR::Prepare(){
	q0 = obj[0]->var_base_pos_r->val;
	w0 = obj[0]->var_base_vel_r->val;
	q1 = obj[1]->var_base_pos_r->val;
	
	q_rhs = q0*quat_t::Rot(q0.Conjugated()*(w0*obj[0]->hnext));
}

void WholebodyBaseVelConR::Prepare(){
	pc   = obj[0]->var_com_pos->val;
	w0   = obj[0]->var_base_vel_r->val;
	w1   = obj[1]->var_base_vel_r->val;
	q0   = obj[0]->var_base_pos_r->val;
	q0.ToMatrix(R0);
	Ld   = obj[0]->data.Ld;
	Iinv = obj[0]->data.Iinv;

	DSTR << Ld << endl;
	
	int nend  = obj[0]->ends.size();
	int nlink = obj[0]->wb->links.size();

	r .resize(nend);
	rc.resize(nend);
	R .resize(nend);
	f .resize(nend);
	m .resize(nend);
		
	msum.clear();
	for(int i = 0; i < nend; i++){
		WholebodyKey::End& end = obj[0]->ends[i];

		end.var_pos_r->val.ToMatrix(R[i]);
		r [i] = end.var_pos_t->val;
		rc[i] = mat3_t::Cross(r[i]);

		f[i] = end.var_force_t->val;
		m[i] = end.var_force_r->val;
		msum += R0*(R[i]*m[i] + r[i] % (R[i]*f[i]));

		J_Ld_pe[i].clear();
		J_Ld_qe[i].clear();
		J_Ld_ae[i].clear();
		J_Ld_ue[i].clear();
		for(int j = 0; j < nlink; j++){
			WholebodyData::Link& dlnk = obj[0]->data.links[j];

			real_t mj = obj[0]->wb->links[j].mass;
			mat3_t mj_ajc = mj*mat3_t::Cross(dlnk.acc_t);
			mat3_t mj_pjc = mj*mat3_t::Cross(dlnk.pos_t);
			J_Ld_pe[i] -= mj_ajc*obj[0]->J_pi_pe[i][j];
			J_Ld_qe[i] -= mj_ajc*obj[0]->J_pi_qe[i][j];
			J_Ld_ae[i] += mj_pjc*obj[0]->J_pi_pe[i][j];
			J_Ld_ue[i] += mj_pjc*obj[0]->J_pi_qe[i][j];
		}
	}

	w_rhs = w0 + (Iinv*(msum - q0*Ld))*obj[0]->hnext;
}

void WholebodyDesPosConT::Prepare(){
	WholebodyKey::End& end = obj->ends[iend];

	pc = obj->var_com_pos->val;
	q0 = obj->var_base_pos_r->val;
	q0.ToMatrix(R0);
	pi = end.var_pos_t->val;
}

void WholebodyDesPosConR::Prepare(){
	WholebodyKey::End& end = obj->ends[iend];

	q0 = obj->var_base_pos_r->val;
	q0.ToMatrix(R0);
	qi = end.var_pos_r->val;
}

void WholebodyDesVelConT::Prepare(){
	WholebodyKey::End& end = obj->ends[iend];

	vc = obj->var_com_vel->val;
	w0 = obj->var_base_vel_r->val;
	q0 = obj->var_base_pos_r->val;
	q0.ToMatrix(R0);
	vi = end.var_vel_t->val;
}

void WholebodyDesVelConR::Prepare(){
	WholebodyKey::End& end = obj->ends[iend];

	w0 = obj->var_base_vel_r->val;
	q0 = obj->var_base_pos_r->val;
	q0.ToMatrix(R0);
	wi = end.var_vel_r->val;
}

void WholebodyLimitCon::Prepare(){
	
}

void WholebodyContactPosConT::Prepare(){
	WholebodyData::End&  dend  = obj->data.ends[iend];
	WholebodyKey::End& end = obj->ends[iend];
	
	n  = dend.pos_rc*vec3_t(0.0, 0.0, 1.0);
	o  = dend.pos_tc;
	pc = obj->var_com_pos->val;
	q0 = obj->var_base_pos_r->val;
	pi = end.var_pos_t->val;
	qi = end.var_pos_r->val;
}

void WholebodyContactPosConR::Prepare(){
	WholebodyData::End& dend = obj->data.ends[iend];
	WholebodyKey::End& end = obj->ends[iend];
	
	qc = dend.pos_rc;
	q0 = obj->var_base_pos_r->val;
	qi = end.var_pos_r->val;
}

void WholebodyContactVelConT::Prepare(){
	WholebodyData::End& dend = obj->data.ends[iend];
	WholebodyKey::End& end = obj->ends[iend];
	
	n  = dend.pos_rc*vec3_t(0.0, 0.0, 1.0);
	vc = obj->var_com_vel->val;
	q0 = obj->var_base_pos_r->val;
	q0.ToMatrix(R0);
	w0 = obj->var_base_vel_r->val;
	pi = end.var_pos_t->val;
	vi = end.var_vel_t->val;
	qi = end.var_pos_r->val;
}

void WholebodyContactVelConR::Prepare(){
	WholebodyData::End& dend = obj->data.ends[iend];
	WholebodyKey::End& end = obj->ends[iend];
	
	q0 = obj->var_base_pos_r->val;
	q0.ToMatrix(R0);
	w0 = obj->var_base_vel_r->val;
	qi = end.var_pos_r->val;
	wi = end.var_vel_r->val;
}

void WholebodyNormalForceCon::Prepare(){
	WholebodyKey ::End& end  = obj->ends[iend];
	WholebodyData::End& dend = obj->data.ends[iend];

	fn = end.var_force_t->val.z;
}

void WholebodyFrictionForceCon::Prepare(){
	WholebodyKey ::End& end  = obj->ends[iend];
	WholebodyData::End& dend = obj->data.ends[iend];

	mu = dend.mu;
	vec3_t f = end.var_force_t->val;
	ft = (dir == 0 ? f.x : f.y);
	fn = f.z;
}

void WholebodyMomentCon::Prepare(){
	WholebodyKey ::End& end  = obj->ends[iend];
	WholebodyData::End& dend = obj->data.ends[iend];

	fn   = end.var_force_t->val.z;
	m.x  = end.var_force_r->val.x;
	m.y  = end.var_force_r->val.y;
	cmin = dend.cop_min;
	cmax = dend.cop_max;
}

///////////////////////////////////////////////////////////////////////////////////////////////////

void WholebodyPosConT::CalcCoef(){
	Prepare();

	int idx = 0;
	((SLink*)links[idx++])->SetCoef( 1.0);
	((SLink*)links[idx++])->SetCoef(-1.0);
	((SLink*)links[idx++])->SetCoef(-obj[0]->hnext);
}

void WholebodyPosConR::CalcCoef(){
	Prepare();

	int idx = 0;
	((SLink*)links[idx++])->SetCoef( 1.0);
	((SLink*)links[idx++])->SetCoef(-1.0);
	((SLink*)links[idx++])->SetCoef(-obj[0]->hnext);
}

void WholebodyVelConT::CalcCoef(){
	Prepare();

	int idx = 0;
	((SLink*)links[idx++])->SetCoef( 1.0);
	((SLink*)links[idx++])->SetCoef(-1.0);
	((SLink*)links[idx++])->SetCoef(-obj[0]->hnext);
}

void WholebodyVelConR::CalcCoef(){
	Prepare();

	int idx = 0;
	((SLink*)links[idx++])->SetCoef( 1.0);
	((SLink*)links[idx++])->SetCoef(-1.0);
	((SLink*)links[idx++])->SetCoef(-obj[0]->hnext);
}

void WholebodyComPosCon::CalcCoef(){
	Prepare();

	int idx = 0;
	((SLink*)links[idx++])->SetCoef( 1.0);
	((SLink*)links[idx++])->SetCoef(-1.0);
	((SLink*)links[idx++])->SetCoef(-obj[0]->hnext);
}

void WholebodyComVelCon::CalcCoef(){
	Prepare();

	int idx = 0;
	((SLink*)links[idx++])->SetCoef( 1.0);
	((SLink*)links[idx++])->SetCoef(-1.0);

	int nend = obj[0]->ends.size();
	for(int i = 0; i < nend; i++){
		WholebodyData::End& dend = obj[0]->data.ends[i];

		((M3Link*)links[idx++])->SetCoef(-(obj[0]->hnext/obj[0]->wb->param.total_mass)*(R0*R[i]));
	}
}

void WholebodyBasePosConR::CalcCoef(){
	Prepare();

	int idx = 0;
	((SLink*)links[idx++])->SetCoef( 1.0);
	((SLink*)links[idx++])->SetCoef(-1.0);
	((SLink*)links[idx++])->SetCoef(-obj[0]->hnext);
}

void WholebodyBaseVelConR::CalcCoef(){
	Prepare();

	int idx = 0;
	((SLink*)links[idx++])->SetCoef( 1.0);
	((SLink*)links[idx++])->SetCoef(-1.0);

	int nend = obj[0]->ends.size();
	for(int i = 0; i < nend; i++){
		((M3Link*)links[idx++])->SetCoef( obj[0]->hnext*(Iinv*R0*J_Ld_pe[i]));
		((M3Link*)links[idx++])->SetCoef( obj[0]->hnext*(Iinv*R0*J_Ld_qe[i]));
		((M3Link*)links[idx++])->SetCoef( obj[0]->hnext*(Iinv*R0*J_Ld_ae[i]));
		((M3Link*)links[idx++])->SetCoef( obj[0]->hnext*(Iinv*R0*J_Ld_ue[i]));
		((M3Link*)links[idx++])->SetCoef(-obj[0]->hnext*(R0*rc[i]*R[i]));
		((M3Link*)links[idx++])->SetCoef(-obj[0]->hnext*(R0*R [i]     ));
	}
}

void WholebodyDesPosConT::CalcCoef(){
	Prepare();

	((SLink *)links[0])->SetCoef(1.0);
	((X3Link*)links[1])->SetCoef(-(q0*pi));
	((M3Link*)links[2])->SetCoef(R0);
}

void WholebodyDesPosConR::CalcCoef(){
	Prepare();

	((SLink *)links[0])->SetCoef(1.0);
	((M3Link*)links[1])->SetCoef(R0);
}

void WholebodyDesVelConT::CalcCoef(){
	Prepare();

	((SLink *)links[0])->SetCoef(1.0);
	((X3Link*)links[1])->SetCoef(-(q0*pi));
	((M3Link*)links[2])->SetCoef(R0);
}

void WholebodyDesVelConR::CalcCoef(){
	Prepare();

	((SLink *)links[0])->SetCoef(1.0);
	((M3Link*)links[1])->SetCoef(R0);
}

void WholebodyLimitCon::CalcCoef(){
	Prepare();

	int nend = obj->ends.size();
	int idx = 0;
	for(int i = 0; i < nend; i++){
		((R3Link*)links[idx++])->SetCoef(vec3_t(obj->J_e_pe[i].row(ierror)));
		((R3Link*)links[idx++])->SetCoef(vec3_t(obj->J_e_qe[i].row(ierror)));
	}
	
}

void WholebodyContactPosConT::CalcCoef(){
	/*
	y  = n*(pc + q0*(pi + qi*c) - o) - r
	dy = n*(dpc + Omega0 % q0*(pi + qi*c) + q0*(dpi + Omegai % qi*c))
	   = n*dpc + n^T (q0*(pi + qi*c)^x)^T Omega0 + (q0^T n)*dpi + n^T q0 ((qi*c)^x)^T Omegai
	   = n*dpc + (q0*(pi + qi*c) % n)*Omega0 + (q0^T n)*dpi + (qi*c % q0^T n)*Omegai
    */
	Prepare();

	((R3Link*)links[0])->SetCoef(n);
	((R3Link*)links[1])->SetCoef((q0*pi) % n);
	((R3Link*)links[2])->SetCoef(q0.Conjugated()*n);
}

void WholebodyContactPosConR::CalcCoef(){
	Prepare();

	((SLink *)links[0])->SetCoef(1.0);
	((M3Link*)links[1])->SetCoef(R0);
}

void WholebodyContactVelConT::CalcCoef(){
	Prepare();

	((SLink *)links[0])->SetCoef(1.0);
	((X3Link*)links[1])->SetCoef(-(q0*pi));
	((M3Link*)links[2])->SetCoef( R0);
}

void WholebodyContactVelConR::CalcCoef(){
	Prepare();

	((SLink *)links[0])->SetCoef(1.0);
	((M3Link*)links[1])->SetCoef(R0);
}

void WholebodyNormalForceCon::CalcCoef(){
	Prepare();

	((R3Link*)links[0])->SetCoef(vec3_t(0.0, 0.0, 1.0));
}

void WholebodyFrictionForceCon::CalcCoef(){
	Prepare();

	// -mu*fn <= ft <= mu*fn
	// -ft + mu*fn >= 0
	//  ft + mu*fn >= 0

	((R3Link*)links[0])->SetCoef(
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

	((R3Link*)links[0])->SetCoef(
		vec3_t(
			0.0,
			0.0,
			(dir == 0 ? (side == 0 ? -cmin.x : cmax.x) : (side == 0 ? -cmin.y : cmax.y)) ));
	((R3Link*)links[1])->SetCoef(
		vec3_t(
			(dir == 0 ? 0.0 : (side == 0 ? +1.0 : -1.0)),
			(dir == 1 ? 0.0 : (side == 0 ? -1.0 : +1.0)),
			0.0));
}

///////////////////////////////////////////////////////////////////////////////////////////////////

void WholebodyPosConT::CalcDeviation(){
	y = p1 - p_rhs;
}

void WholebodyPosConR::CalcDeviation(){
	quat_t qerror = q_rhs.Conjugated()*q1;
	vec3_t axis   = qerror.Axis ();
	real_t theta  = qerror.Theta();
	if(theta > pi)
		theta -= 2*pi;
	y = q_rhs*(theta*axis);
}

void WholebodyVelConT::CalcDeviation(){
	y = v1 - v_rhs;
}

void WholebodyVelConR::CalcDeviation(){
	y = w1 - w_rhs;
}

void WholebodyComPosCon::CalcDeviation(){
	y = pc1 - pc_rhs;
}

void WholebodyComVelCon::CalcDeviation(){
	y = vc1 - vc_rhs;
}

void WholebodyBasePosConR::CalcDeviation(){
	quat_t qerror = q_rhs.Conjugated()*q1;
	vec3_t axis   = qerror.Axis ();
	real_t theta  = qerror.Theta();
	if(theta > pi)
		theta -= 2*pi;
	y = q_rhs*(theta*axis);
}

void WholebodyBaseVelConR::CalcDeviation(){
	y = w1 - w_rhs;
}

void WholebodyDesPosConT::CalcDeviation(){
	y = (pc + q0*pi) - desired;
}

void WholebodyDesPosConR::CalcDeviation(){
	quat_t qerror = desired.Conjugated()*(q0*qi);
	vec3_t axis   = qerror.Axis ();
	real_t theta  = qerror.Theta();
	if(theta > pi)
		theta -= 2*pi;
	y = desired*(theta*axis);
}

void WholebodyDesVelConT::CalcDeviation(){
	y = (vc + w0 % (q0*pi) + q0*vi) - desired;
}

void WholebodyDesVelConR::CalcDeviation(){
	y = (w0 + q0*wi) - desired;
}

void WholebodyLimitCon::CalcDeviation(){
	real_t e = obj->data.e[ierror];

	if(type == Constraint::Type::Equality){
		y[0]   = e;
		active = true;
	}
	if(type == Constraint::Type::InequalityPenalty){
		if(e > 0.0){
			y[0] = 0.0;
			active = false;
		}
		else{
			y[0] = e;
			active = true;
		}
	}
	//DSTR << "limit error: " << y[0] << endl;
}

void WholebodyContactPosConT::CalcDeviation(){
	y[0] = n*(pc + q0*pi - o);
}

void WholebodyContactPosConR::CalcDeviation(){
	quat_t qerror = qc.Conjugated()*(q0*qi);
	vec3_t axis   = qerror.Axis ();
	real_t theta  = qerror.Theta();
	if(theta > pi)
		theta -= 2*pi;
	y = qc*(theta*axis);
}

void WholebodyContactVelConT::CalcDeviation(){
	y = vc + w0 % (q0*pi) + q0*vi;
}

void WholebodyContactVelConR::CalcDeviation(){
	y = w0 + q0*wi;
}

void WholebodyNormalForceCon::CalcDeviation(){
	if(fn > 0.0){
		y[0] = 0.0;
		active = false;
	}
	else{
		y[0] = fn;
		active = true;
	}
}

void WholebodyFrictionForceCon::CalcDeviation(){
	real_t e = (side == 0 ? mu*fn - ft : ft + mu*fn);
	if(e > 0.0){
		y[0] = 0.0;
		active = false;
	}
	else{
		y[0] = e;
		active = true;
	}		
}

void WholebodyMomentCon::CalcDeviation(){
	// -ty*m - (cop_min.x)*fn >= 0
	//  ty*m + (cop_max.x)*fn >= 0
	//  tx*m - (cop_min.y)*fn >= 0
	// -tx*m + (cop_max.y)*fn >= 0
	real_t e = (dir == 0 ? 
		(side == 0 ? (-m.y - cmin.x*fn) : ( m.y + cmax.x*fn)) :
		(side == 0 ? ( m.x - cmin.y*fn) : (-m.x + cmax.y*fn))
		);

	if(e > 0.0){
		y[0] = 0.0;
		active = false;
	}
	else{
		y[0] = e;
		active = true;
	}
	
}

///////////////////////////////////////////////////////////////////////////////////////////////////

void WholebodyPosConT::CalcLhs(){
	obj[1]->ends[iend].var_pos_t->val = p_rhs;
}

void WholebodyPosConR::CalcLhs(){
	obj[1]->ends[iend].var_pos_r->val = q_rhs;
}

void WholebodyVelConT::CalcLhs(){
	obj[1]->ends[iend].var_vel_t->val = v_rhs;
}

void WholebodyVelConR::CalcLhs(){
	obj[1]->ends[iend].var_vel_r->val = w_rhs;
}

void WholebodyComPosCon::CalcLhs(){
	obj[1]->var_com_pos->val = pc_rhs;
}

void WholebodyComVelCon::CalcLhs(){
	obj[1]->var_com_vel->val = vc_rhs;
}

void WholebodyBasePosConR::CalcLhs(){
	obj[1]->var_base_pos_r->val = q_rhs;
}

void WholebodyBaseVelConR::CalcLhs(){
	obj[1]->var_base_vel_r->val = w_rhs;
}

///////////////////////////////////////////////////////////////////////////////////////////////////

}
