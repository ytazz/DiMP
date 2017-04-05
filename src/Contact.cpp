#include <DiMP2/Contact.h>
#include <DiMP2/Graph.h>
#include <DiMP2/Solver.h>
#include <DiMP2/Range.h>
#include <DiMP2/DrawConfig.h>

namespace DiMP2{;

void ContactKey::AddVar(Solver* solver){
	JointKey::AddVar(solver);

	if(next){
		force_r->val.clear();
		force_r->Lock();
	}
}

void ContactKey::AddCon(Solver* solver){
	JointKey::AddCon(solver);

	Graph::Scale& scale = node->graph->scale;
	
	if(mode == Contact::Mode::Stick){
		ContactKey* conNext = (ContactKey*)next;
		if(conNext){
			real_t spf = scale.pos_t * scale.force_t;
			real_t svf = scale.vel_t * scale.force_t;
	
			//con_f        = new FrictionCon(solver, name + "_f", this, scale.force_t);

			//con_pf[1]    = new ComplConS(solver, ID(ConTag::ContactPF, node, tick, name + "_pf1" ), torque[0], conNext->pos[0], spf);
		}
	}
}

void ContactKey::Prepare(){
	JointKey::Prepare();
	
	// âÒì]ÇÕçSë©ÇµÇ»Ç¢ÇÃÇ≈ñ≥å¯âª
	con_rp->enabled = false;
	con_rv->enabled = false;

	// ÉÇÅ[ÉhéÊìæ
	mode = Contact::Mode::Stick;
	for(uint i = 0; i < ((Contact*)node)->con_param.setting.size(); i++){
		Contact::ModeSetting& s = ((Contact*)node)->con_param.setting[i];
		if(s.ts <= tick->time && tick->time <= s.te){
			mode = s.mode;
			break;
		}
	}

	if(mode == Contact::Mode::Stick){
		if(next){
			force_t->Lock(false);
			con_range_f[0]->_min = 0.0;
		}
				
		pos[0]->val = 0.0;
		pos[0]->Lock();
		
		for(int i = 0; i < 3; i++){
			vel[i]->val = 0.0;
			vel[i]->Lock();
		}
	}
	if(mode == Contact::Mode::Float){
		if(next){
			force_t->val.clear();
			force_t->Lock();
		}

		pos[0]->Lock(false);
		con_range_p[0]->_min = ((Contact*)node)->con_param.margin;

		for(int i = 0; i < 3; i++)
			vel[i]->Lock(false);
	}
}

//-------------------------------------------------------------------------------------------------

Contact::Param::Param(){
	mu     = 1.0;
	margin = 0.0;
}

Contact::Contact(Connector* _sock, Connector* _plug, TimeSlot* _time, const string& n):Joint(_sock, _plug, _time, n){
	SetDof(3);
}

void Contact::CalcRelativePose(real_t* pos, vec3_t& p, quat_t& q){
	p = vec3_t(pos[0], pos[1], pos[2]);
	q = quat_t();
}

void Contact::CalcJointPos(real_t* pos, vec3_t& p, quat_t& q){
	pos[0] = p.x;
	pos[1] = p.y;
	pos[2] = p.z;
}

void Contact::CalcJacobian(real_t* pos, vec3_t* Jv, vec3_t* Jw){
	Jv[0] = vec3_t(1.0, 0.0, 0.0);
	Jv[1] = vec3_t(0.0, 1.0, 0.0);
	Jv[2] = vec3_t(0.0, 0.0, 1.0);
	Jw[0].clear();
	Jw[1].clear();
	Jw[2].clear();
}

//-------------------------------------------------------------------------------------------------

FrictionCon::FrictionCon(Solver* solver, ID id, SVar* _fn, SVar* _ft0, SVar* _ft1, real_t _mu, real_t _scale):
	Constraint(solver, 1, id, _scale){
	fn    = _fn;
	ft[0] = _ft0;
	ft[1] = _ft1;
	mu    = _mu;
	AddSLink(fn);
	AddSLink(ft[0]);
	AddSLink(ft[1]);
}

//-------------------------------------------------------------------------------------------------

void FrictionCon::CalcCoef(){
	((SLink*)links[0])->SetCoef(mu);
	vec2_t _ft;
	_ft[0] = ft[0]->val;
	_ft[1] = ft[1]->val;
	ft_norm = _ft.norm();
	const real_t eps = 1.0e-10;
	if(ft_norm < eps){
		((SLink*)links[1])->SetCoef(0.0);
		((SLink*)links[2])->SetCoef(0.0);
	}
	else{
		((SLink*)links[1])->SetCoef(-_ft[0] / ft_norm);
		((SLink*)links[2])->SetCoef(-_ft[1] / ft_norm);
	}
}

//-------------------------------------------------------------------------------------------------

void FrictionCon::CalcDeviation(){
	real_t s = mu * fn->val - ft_norm;
	if(s < 0.0){
		y[0] = s;
		active = true;
	}
	else active = false;
}

//-------------------------------------------------------------------------------------------------

void FrictionCon::Project(real_t& l, uint k){
	if(k == 0 && l < 0.0)
		l = 0.0;
}

}
