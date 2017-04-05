#include <DiMP2/Graph.h>
#include <DiMP2/Mpc.h>


/*
�����ł̕��j

��  �����O��  ��
�@�O���v��������ʒu��^�����ɍs��
�A���L�l�̈ʒu����O���v��ŋ��߂�ꂽ�����ڒn�ʒu�ւƓ��������[�V�����f�[�^�����
�i���̂Ƃ��ŏ��̗V�r�͓_�Ώ̈ʒu�ɔz�u����j
�B���̏�Ԃ��琧����J�n


(��  �I�[�O��  ��
�@�O���v�悩�狁�߂���Ō�̎x���r�ƍŌ�̏d�S�ʒu�̑Ώ̓_�ɏI�����̎x���_��p��
�A�����Ɍ������čŌ�̗V�r�𓮂����悤�Ƀ��[�V�����쐬 )

*/


namespace DiMP2
{;
//-------------------------------------------------------------------------------------------------
// BipedLIPKey

BipedLIPKey::BipedLIPKey()
{
	phase = BipedLIP::Left;
}



// memo�F�����ŏd�S�ʒu�E�ڒn�ʒu�̌v�Z���s�킹�Ă��� �ŏ��Ɉ��Ăяo���ďI��
void BipedLIPKey::AddVar(Solver* solver)
{
	BipedLIP* obj = GetNode();

	pos_t[0] = new SVar(solver, ID(0, node, tick, name + "_pos0"), node->graph->scale.pos_t); //0:x,1:y
	pos_t[1] = new SVar(solver, ID(0, node, tick, name + "_pos1"), node->graph->scale.pos_t);
	vel_t[0] = new SVar(solver, ID(12, node, tick, name + "_vel0"), node->graph->scale.vel_t);
	vel_t[1] = new SVar(solver, ID(0, node, tick, name + "_vel1"), node->graph->scale.vel_t);

	//addition acc_t
	acc_t[0] = new SVar(solver, ID(0, node, tick, name + "_acc0"), node->graph->scale.acc_t);
	acc_t[1] = new SVar(solver, ID(0, node, tick, name + "_acc1"), node->graph->scale.acc_t);


	pos_cop[0] = new SVar(solver, ID(0, node, tick, name + "_cop0"), node->graph->scale.pos_t);
	pos_cop[1] = new SVar(solver, ID(0, node, tick, name + "_cop1"), node->graph->scale.pos_t);
	
	period  = new SVar (solver, ID(VarTag::ObjectTP, node, tick, name + "_T"), node->graph->scale.time );

}




void BipedLIPKey::AddCon(Solver* solver)
{
	BipedLIPKey* nextObj = (BipedLIPKey*)next;

	if(next)
	{

		//LIP���f��
		con_lip_pos[0] = new LIPConP(solver, name + "_lip_p0", this, 0, node->graph->scale.pos_t);
		con_lip_pos[1] = new LIPConP(solver, name + "_lip_p1", this, 1, node->graph->scale.pos_t);
		con_lip_vel[0] = new LIPConV(solver, name + "_lip_v0", this, 0, node->graph->scale.vel_t);
		con_lip_vel[1] = new LIPConV(solver, name + "_lip_v1", this, 1, node->graph->scale.vel_t);
		

		//1phase�̎���
		con_range_period = new RangeConS(solver, ID(0, node, tick, name + "_range_period"), period, node->graph->scale.time);

		
		//
		con_diff_cop[0][0] = new DiffConS(solver, ID(0, node, tick, name + "_range_cop0x"), pos_t[0], pos_cop[0], node->graph->scale.pos_t);
		con_diff_cop[0][1] = new DiffConS(solver, ID(0, node, tick, name + "_range_cop0y"), pos_t[1], pos_cop[1], node->graph->scale.pos_t);
		con_diff_cop[1][0] = new DiffConS(solver, ID(0, node, tick, name + "_range_cop1x"), nextObj->pos_t[0], pos_cop[0], node->graph->scale.pos_t);
		con_diff_cop[1][1] = new DiffConS(solver, ID(0, node, tick, name + "_range_cop1y"), nextObj->pos_t[1], pos_cop[1], node->graph->scale.pos_t);
		


	}

	//�I�[�ʒu
	if(!next)
	{
		if ((bool)GetNode()->param.use_terminal_pos == true){
		con_fix_pos[0] = new FixConS(solver, ID(0, node, tick, name + "_fix_pos0"), pos_t[0], node->graph->scale.pos_t);
		con_fix_pos[1] = new FixConS(solver, ID(0, node, tick, name + "_fix_pos1"), pos_t[1], node->graph->scale.pos_t);
		}

		con_fix_vel[0] = new FixConS(solver, ID(0, node, tick, name + "_fix_vel0"), vel_t[0], node->graph->scale.vel_t);
		con_fix_vel[1] = new FixConS(solver, ID(0, node, tick, name + "_fix_vel1"), vel_t[1], node->graph->scale.vel_t);
	}

}

//memo : �l�X�ȃp�����[�^ �ŏ��Ɉ��Ăяo���ďI��
BipedLIP::Param::Param()
{
//	gravity         = 9.8;
//	height          = 0.7;
//	swing_vel_max   = 1.0;
//	step_period_min = 0.0;
//	step_period_max = 1.0;
	terminal_pos    = vec2_t();
	terminal_vel    = vec2_t();
	

	support_min[0]  = vec2_t( -0.03, -0.13 );  /// ����(-0.3,-0.2) <���� of Left Phase	(�i�s����,�������j
	support_max[0]  = vec2_t( 0.03, -0.08);		/// ����( 0.3,-0.05)
	
	support_min[1]  = vec2_t( -0.03,  0.08);  /// ����(-0.3,0.05) <���� of Right Phase
	support_max[1]  = vec2_t( 0.03,  0.13 );	/// ����(0.3,0.2)


	

}
;


void BipedLIPKey::Prepare()
{	
	if(!prev){
		 tick->time = 0.0;
	}else{
		BipedLIPKey* prevObj = (BipedLIPKey*)prev;
		tick->time = prevObj->tick->time + prevObj->period->val;
	}
}





//���r�x�����̏d�S��ڒn�_�̈ʒu�̕`��
void BipedLIPKey::Draw(GRRenderIf* render, DrawConfig* conf)
{
	
	render->SetPointSize(8.0f,0);
	Vec3f m,p,v;											
	// �d�S�ʒu
	m.x = (float)pos_t[0]->val;
	m.y = (float)pos_t[1]->val;
	m.z = (float)GetNode()->param.height;
	render->DrawPoint(m);

	cout << "com:" << m << "\n";
	
	render->SetPointSize(8.0f,1);
	// COP�ʒu(�ڒn�ʒu)
	p.x = (float)pos_cop[0]->val;
	p.y = (float)pos_cop[1]->val;
	p.z = 0.0f ;
	
	cout << "cop:" << p << "\n";


	if(p.x!= 0.0 || p.y != 0.0) {
	render->DrawPoint(p);
	render->SetLineWidth(3.0f);		//������z��\��
	render->DrawLine(vec3_t(0.0,0.0,0.0),vec3_t(0.0,0.0,(float)GetNode()->param.height)) ;
	}
	
}



// BipedLIPKey �����܂�
//-------------------------------------------------------------------------------------------------
// BipedLIP




BipedLIP::BipedLIP(Graph* g, string n):TrajectoryNode(g, n){
	}



//------------------------------------------------------------------------------------------------------//
//////////////////////////(�|���U�q���f���p�����[�^)/////////////////////////////////


// �e�����ɂ�����S�������^����i���v�Z���ďI���j

void BipedLIP::Init()				
{
	TrajectoryNode::Init();

	for(uint k = 0; k < graph->ticks.size(); k++)
	{

		BipedLIPKey* key = GetKeypoint(graph->ticks[k]);  //key��tick�ɃA�N�Z�X
		
		std::map<int, int>::iterator it = param.phase.find(k);
		
		if(it == param.phase.end())
			 key->phase = BipedLIP::Left;
		else key->phase = it->second;


		
		if(!key->prev)
		{
			key->pos_t[0]->Lock();
			key->pos_t[1]->Lock();
			
			//������Ԃ�^����
			if(param.use_beginning == true)
			{
				key->vel_t[0]->val =  param.initial_velocity.x;
				key->vel_t[1]->val =  param.initial_velocity.y;

				key->pos_cop[0]->val = param.rfoot_place.x;
				key->pos_cop[1]->val = param.rfoot_place.y;

				key->pos_cop[0]->Lock();
				key->pos_cop[1]->Lock();

			}
			
			
			key->vel_t[0]->Lock();
			key->vel_t[1]->Lock();

		}

		
		//�i�d�S���I�[�ʒu�ȊO�ł�key->next��1�ȊO�ł͂Ȃ����j

		if (key->next)
		{
			// �����̏����l�͉����Ə���̒��Ԓl
			key->period->val = (param.step_period_min + param.step_period_max) / 2.0  ;



			//
////////////////////////////////// 1Phase�̎��Ԃ̍ŏ��ƍő�̃p�����[�^//////////////////////////////

			key->con_range_period->_min = param.step_period_min;
			key->con_range_period->_max = param.step_period_max;


/////////////////////////////////////�E���̍S���̐؂�ւ�///////////////////////////////////////
			
			bool lr = (key->phase == Left ? 0 : 1);			//Phase��Left�Ȃ�lr=0,Right�Ȃ�lr=1
						
			key->con_diff_cop[lr][0]->_min = param.support_min[lr].x;	
			key->con_diff_cop[lr][0]->_max = param.support_max[lr].x;	
			key->con_diff_cop[lr][1]->_min = param.support_min[lr].y;	
			key->con_diff_cop[lr][1]->_max = param.support_max[lr].y;	

		}


/////////////////////////////////////////�I�[�S��////////////////////////////////////////////////
		if (key->next == 0)
		{
			if (param.use_terminal_pos == true){
			key->con_fix_pos[0]->desired = param.terminal_pos[0];
			key->con_fix_pos[1]->desired = param.terminal_pos[1];
			}

			key->con_fix_vel[0]->desired = param.terminal_vel[0];
			key->con_fix_vel[1]->desired = param.terminal_vel[1];
			
		
		}


	}

}




void BipedLIP::Prepare()
{
	TrajectoryNode::Prepare();
}



void BipedLIP::SetPhase(int step, int phase)
{
	param.phase[step] = phase;
}



//-----------------------------------------------------------------------------------------
//�ʒu
vec3_t BipedLIP::Pos(real_t t){
	BipedLIPKey* key = GetSegment(t).first;

	real_t T   = sqrt(param.height/param.gravity);
	real_t t0  = key->tick->time;			//�x�������؂�ւ�����u�Ԃ̎��ԁi�x���r���ς��܂ŌŒ�j
	real_t px   = key->pos_t[0]->val;		//�x�������؂�ւ�����u�Ԃ̈ʒu�i�x���r���ς��܂ŌŒ�j
	real_t py   = key->pos_t[1]->val;
	real_t vx   = key->vel_t[0]->val;		//�x�������؂�ւ�����u�Ԃ̑��x�i�x���r���ς��܂ŌŒ�j
	real_t vy   = key->vel_t[1]->val;
	real_t copx = key->pos_cop[0]->val;		//�ڒn�ʒu�@�i�x���r���ς��܂ŌŒ�j
	real_t copy = key->pos_cop[1]->val;
	vec3_t p;

	p.x = copx + (px - copx) * cosh((t-t0)/T)  + (vx*T) * sinh((t-t0)/T)  ;
	p.y = copy + (py - copy) * cosh((t-t0)/T)  + (vy*T) * sinh((t-t0)/T)  ;
	p.z = param.height;

	return p;			//t���󂯎��Cp(�ʒu)��Ԃ�
}



//------------------------------------------------------------------------------------
//���x
vec3_t BipedLIP::Vel(real_t t){
	BipedLIPKey* key = GetSegment(t).first;

	real_t T   = sqrt(param.height/param.gravity);
	real_t t0  = key->tick->time;
	real_t px   = key->pos_t[0]->val;
	real_t py   = key->pos_t[1]->val;
	real_t vx   = key->vel_t[0]->val;
	real_t vy   = key->vel_t[1]->val;
	real_t copx = key->pos_cop[0]->val;
	real_t copy = key->pos_cop[1]->val;
	vec3_t v;

	v.x = ((px-copx)/T) * sinh((t-t0)/T) + (vx) * cosh((t-t0)/T);
	v.y = ((py-copy)/T) * sinh((t-t0)/T) + (vy) * cosh((t-t0)/T);
	v.z = 0.0;
	

	return v;
}


//-------------------------------------------------------------------------------------------
//�����x

vec3_t BipedLIP::Acc(real_t t){
	BipedLIPKey* key = GetSegment(t).first;

	real_t T    = sqrt(param.height/param.gravity);
	real_t copx = key->pos_cop[0]->val;
	real_t copy = key->pos_cop[1]->val;
	vec3_t p    = Pos(t);
	vec3_t a;

	a.x = (p.x - copx)/(T*T);
	a.y = (p.y - copy)/(T*T);
	a.z = 0.0;

	return a;
}



//�t�������C�d�S�ʒu
vec3_t BipedLIP::PosCOM(real_t t)
{
	BipedLIPKey* key = GetSegment(t).first;

	return vec3_t(key->pos_t[0]->val, key->pos_t[1]->val,param.height);	//������ς���Ƃ��͕ύX����K�v����
}

//�ڒn�ʒu
vec3_t BipedLIP::PosCOP(real_t t)
{
	BipedLIPKey* key = GetSegment(t).first;

	return vec3_t(key->pos_cop[0]->val, key->pos_cop[1]->val,0.0);
}





//------------------------------------------------------------------------------------------------
//�d�S�O���̌v�Z�C�`�ʁ@�iBiped3����̏��������͈ȉ����)


//�i�����x�����j���ڒn�ʒu�A�E�ڒn�ʒu�A�d�S�ʒu���󂯎��A�������ɂ��ē��̒��S����̑��拗����Ԃ�

void BipedLIP::Footcalc(Vec3d left_cop,Vec3d right_cop,Vec3d com, Vec3d *left_pos, Vec3d *right_pos)
{
	Vec3d left_calc_pos,right_calc_pos ;	//�v�Z���ʂ������Ɋi�[	
	real_t beta2 = 1 - 1 / param.beta;


	//----------�i�s����----------
	real_t delta_front,center_front,delta_com_front;
	real_t delta_foot_front ;		//���S����̑���ړ���
	
	delta_front = abs(left_cop.x - right_cop.x) ;	//������ԋ���
	center_front = (left_cop.x + right_cop.x ) / 2;	//�����撆�S�̍��W
	delta_com_front = com.x - center_front;			//�����撆�S����̂���

	delta_foot_front = (2 + param.alpha) * delta_com_front / (2 * beta2 + param.alpha) ;  //���S����̊e����ʒu�ω���
	
	//--- �r�̑O��ŕς��
	//���r���O�̂Ƃ�
	if(left_cop.x > right_cop.x){
		left_calc_pos.x  = delta_front / 2.0 - delta_foot_front;
		right_calc_pos.x = - (delta_front / 2.0 + delta_foot_front ) ;
	}

	//�E�����O�̂Ƃ�
	else{
		left_calc_pos.x  = - (delta_front / 2.0 + delta_foot_front);
		right_calc_pos.x = delta_front / 2.0 - delta_foot_front;
	}



	//---------- ������ ----------
	real_t delta_side,center_side,delta_com_side;
	real_t delta_foot_side;		//���S����̑���ړ���


	delta_side = left_cop.y - right_cop.y;			//������ԋ���
	center_side = (left_cop.y + right_cop.y) / 2;	//�����撆�S�̍��W
	delta_com_side = com.y - center_side;			//�����撆�S����̂��� 

	delta_foot_side = (2 + param.alpha) * delta_com_side / (2 * beta2 + param.alpha) ;  //���S����̊e����ʒu�ω���

	left_calc_pos.y  = delta_side / 2.0 - delta_foot_side ;	
	right_calc_pos.y = delta_side / 2.0 + delta_foot_side ;	


	//----------��������(�����ł͕ύX�Ȃ�)-----------
	left_calc_pos.z = 0.0 ;
	right_calc_pos.z = 0.0;



	//---�Ō�ɂ܂Ƃ߂ĕԂ�
	*left_pos = left_calc_pos;
	*right_pos = right_calc_pos;

}



//�i�Б��x�����j���̒��S����̗V�r�̒����A�x���_����̏d�S�����������Ƃ��A���̒��S����̎x���r������Ԃ�
real_t BipedLIP::SupPos(real_t swing_pos, real_t com_pos){
	real_t sup_pos;	// �x���r����

	sup_pos = ((2 + param.alpha) * com_pos - swing_pos / param.beta) / ( 2 + param.alpha - (1 / param.beta)) ;

	return sup_pos;
}



void BipedLIP::Draw(GRRenderIf* render, DrawConfig* conf)
{
	TrajectoryNode::Draw(render, conf);


	//----------------------------- ������Ԃ�^����N���X -------------------------------




	//////////////////////////���W���̕\��//////////////////////////////////
	//render->SetLineWidth(0.1f);			
	//render->DrawLine(vec3_t(0.0,0.0,0.0),vec3_t(1.0,0.0,0.0));	//�i�s����
	//render->DrawLine(vec3_t(0.0,-0.2,0.0),vec3_t(0.0,0.4,0.0));	//������(���͉E�����j
	//render->DrawLine(vec3_t(0.0,0.0,0.0),vec3_t(0.0,0.0,0.8));	//��������
	///////////////////////////////////////////////////////////////////////


	//��������ʒu�̕`��
	//render->SetPointSize(12.0f);
	//render->DrawPoint(vec3_t(0,param.lfoot_place.y,0));
	//render->DrawPoint(vec3_t(0,-param.lfoot_place.y,0));




	if(conf->Set(render, DrawItem::ObjectTrajectory, this))
	{

		//////////////////// �g�p����ϐ����`
		real_t tf =  traj.back() -> tick -> time;	//�I�[����
		real_t dt = param.calc_period;		//�v�Z�����
		real_t cdt = param.control_period;	//�������
		real_t t0 = 0.0; 
		real_t t1 = dt;
		Vec3f p0 , p1, p_cop, a ;
		
		
		// \\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\    ����ʒu�E�d�S�ʒu�̌Ăяo���@\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\ 
		
		//-----------------------------------		 �e�z��̒�`			---------------------------------------------

		//�z��̗p��
		Vec3d *cop_position;				// �ڒn�ʒu
		cop_position = (Vec3d *)malloc(sizeof(Vec3d) * param.total_step + 2);

		Vec3d *mass_position;				// �x���r�������̏d�S�ʒu
		mass_position = (Vec3d *)malloc(sizeof(Vec3d) * param.total_step + 1);

		Vec3d *mass_trajectory;				// �d�S�O��
		real_t total_period = (int)(param.step_period_max / param.control_period) * param.total_step ;	// ��������ɏ]���d�S�ʒu��
		mass_trajectory = (Vec3d *)malloc(sizeof(Vec3d) * total_period);

		real_t *change_time;				// �x���r�������̎���
		change_time = (real_t *)malloc(sizeof(real_t) * total_period + 1);

		real_t *control_time;				// ����
		control_time = (real_t *)malloc(sizeof(real_t) * total_period); 


		//-------------------------- �����Őڒn�ʒu�E�������̏d�S�ʒu���i�[ ------------------------------
		int mass_number = 0;	//�d�S�O�� �C���f�b�N�X
		int i = 1 ;			
		int phase_check;		//phase��1�Ȃ�E�r�V�r
		BipedLIPKey* start_get_phase = GetSegment(0).first;
		phase_check = start_get_phase->phase;	//�ŏ���phase


		while ( t0 < tf ){
			BipedLIPKey* get_phase = GetSegment(t0).first;

			cop_position[0] = - PosCOP(0);	//�ŏ��̗V�r�ƂȂ�ʒu
			cop_position[1] = PosCOP(0);	//�ŏ��̎x���r�ʒu
			mass_position[0] = PosCOM(0);	//�ŏ��̏d�S�ʒu
			change_time[0] = 0.0;			//�ŏ��̎���

			// phase���`�F�b�N���A�؂�ւ�����Ƃ���̑���ʒu�A�d�S�ʒu
			if ( phase_check != get_phase->phase ){

				cop_position[i+1].x = get_phase->pos_cop[0]->val;	//�x���r�������̑���ʒu
				cop_position[i+1].y = get_phase->pos_cop[1]->val;
				cop_position[i+1].z = 0.0;							//�����ɕύX���Ȃ��ꍇ
				
				mass_position[i].x = get_phase->pos_t[0]->val;		//�x���r�������̏d�S�ʒu
				mass_position[i].y = get_phase->pos_t[1]->val;
				mass_position[i].z = param.height;					//�����ɕύX���Ȃ��ꍇ

			change_time[i] = t0;

			phase_check = get_phase->phase ;
			i++	;
			}
			t0 = t0 + cdt;
		}
		
		change_time[i] = t0;				//�I�[������ݒ�
		mass_position[i] = PosCOM(tf) ;	//�I�[�̏d�S�ʒu���쐬


		//�I�[�̐ڒn�ʒu(�Ō�ɋr�����낷�ꏊ�A�Ώ̓_)
		for (int ter=0 ; ter<2 ; ter++){
			cop_position[i+1][ter] = 2 * mass_position[i][ter] - cop_position[i][ter] ;
		}
		cop_position[i+1].z = 0.0 ;


		//phase�؂�ւ����ԃ`�F�b�N
		cout <<"\n"<< "tf : " << tf << "\n" ;
		for(int ch = 0 ; ch < param.total_step ; ch++)
		cout << change_time[ch] << "," ;
		cout << "\n" ;


		// ------------------------------ ���Z�b�g ----------------------------------------
		t0 = 0.0;	 
		p0 = Pos(t0);

		// -------------------------- �e�����ɂ�����d�S�O���̓��o ---------------------------
		while(true)
		{
			p_cop = PosCOP(t0);						//
			p1 = Pos(t1);							//


			render->SetLineWidth(1.0f);			
			render->DrawLine(p0, p_cop);			// �d�S����ڒn�_�܂ł���Ō���

			render->SetPointSize(5.0f);
			render->DrawPoint(p1);					// �d�S�̈ʒu��`�ʂ���


			// ---------------------------- �d�S�O���̊i�[  -------------------------

			//�d�S�ʒu�̕ۑ��i���{�b�g�̐�������ɍ��킹��)
			//dt���ω�����(?)�̂ŁA�K����臒l�i�v�����j
			if(fmod(t0,cdt) < dt/4 || fmod(t0,cdt) > dt * (cdt/dt-0.5)){

				mass_trajectory[mass_number].x = p0.x ;
				mass_trajectory[mass_number].y = p0.y ;
				mass_trajectory[mass_number].z = param.height ;	//�������̏ꍇ
				control_time[mass_number] = t0 ;
										
				mass_number ++ ;
			}
			
			p0 = p1;	
			t0 = t1;								
			
			if(t1 == tf)
				break;				//�I�[���ԂŏI��

			t1 = std::min( tf, t1 + dt);		//�������ق��̒l��Ԃ�

		}


		// \\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\�@����O���̐����i���̒��S����̑��拗�����o�j�@\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\

		real_t phase_number;		// ����phase�ɂ����鐧��_
		real_t all_number;			// ����J�n����̐���_
		real_t number_of_step;		// phase���̏d�S�ʒu�̐�
		real_t time;				// ����

		real_t mass_pos;			// �x���_����d�S�ʒu�܂ł̋���

		Vec3d right_foot_pos;		//phase�J�n���̖ڕW�E������
		Vec3d left_foot_pos;		//phase�J�n���̖ڕW�������� 
			
		Vec3d right_foot_terminal;	//phase�I�����̖ڕW�E������
		Vec3d left_foot_terminal;	//phase�I�����̖ڕW���r����
			
		Vec3d out_right_foot;		//�ڕW�E������
		Vec3d out_left_foot;		//�ڕW���r����


		BipedLIPKey* start = GetSegment(0).first;	
		int phase = start->phase ;					//�ŏ���phase���i�[ (1�Ȃ�E���V�r�A������؂�ւ���΃��[�V�����̍��E������ւ��)



		for (int l=0 ; l<param.total_step - 1  ; l++){	
			
			//----- phase�J�n���̎�����change_time[l]�Aphase�I�����̎�����change_time[l+1]  
			time = change_time[l];
			number_of_step = (change_time[l+1] - change_time[l] ) / cdt ;
			phase_number = 0.0 ;

			//---- ���r�V�r�̂Ƃ�
			if (phase == 0){
				
				// �r�������̑��拗��
				// �����͍��r��A�E����A�d�S���W�i���ꂼ��DiMP���W�j�ƂȂ��Ă���
				Footcalc(cop_position[l],cop_position[l+1],mass_position[l], &left_foot_pos, &right_foot_pos) ;
				Footcalc(cop_position[l+2],cop_position[l+1],mass_position[l+1], &left_foot_terminal, &right_foot_terminal) ;

				while (time < change_time[l+1]){
					
					all_number = time / cdt;

					//^^^^ ���� (�����̎w��͏����p������̕ω��ʂƂ���j
					out_right_foot.z = 0.0;

					if(phase_number == 0.0)	//phase�͂��߂͍���0
					out_left_foot.z = 0.0;
					
					else{
						if (phase_number < number_of_step / 2 )
						out_left_foot.z = param.max_swing_height ;

						else{
						out_left_foot.z = param.max_swing_height - (param.max_swing_height * 2 / number_of_step) * (phase_number - number_of_step / 2) ; 
						}
					}

					
					//^^^^ �O��
					//-- �V�r�͎��̖ڕW����ʒu�Ɍ������Ĉ�ӂɓ�����
					out_left_foot.x = left_foot_pos.x + (left_foot_terminal.x - left_foot_pos.x) * phase_number / number_of_step ;
					
					

					// �x���r�͋r�������ȊO�͌v�悳�ꂽ�d�S��ۂ悤�ɓ�����
					if(phase_number == 0.0)
					out_right_foot.x = right_foot_pos.x ;

					// �d�S�Ǝx���r�̑O��֌W�ŏꍇ����
					else{
						
						if(cop_position[l+1].x > mass_trajectory[(int)all_number].x){
						mass_pos = cop_position[l+1].x - mass_trajectory[(int)all_number].x ;
						out_right_foot.x =SupPos( abs(out_left_foot.x), mass_pos );

						}
						
						else{
						mass_pos = mass_trajectory[(int)all_number].x -  cop_position[l+1].x;
						out_right_foot.x = - SupPos( abs(out_left_foot.x),mass_pos);
						}

					}



					//^^^^ ���E
					//-- �V�r�͎��̖ڕW����ʒu�Ɍ������Ĉ�ӂɓ�����
					out_left_foot.y = left_foot_pos.y + (left_foot_terminal.y - left_foot_pos.y) * phase_number / number_of_step ;

					
					// �x���r�͋r�������ȊO�͌v�悳�ꂽ�d�S��ۂ悤�ɓ�����
					if(phase_number == 0.0)
					out_right_foot.y = right_foot_pos.y;

					else{
						mass_pos = mass_trajectory[(int)all_number].y - cop_position[l+1].y;
						out_right_foot.y =SupPos(mass_pos , out_left_foot.y);

					}


					//�o�͂��ăC���N�������g
					Out_motion(out_right_foot,out_left_foot,time,phase);
					time = time + cdt;
					phase_number ++;
				}

				cout << "\n" ;
				//����I���������͕ʓr�o��
				if(l == param.total_step - 2)
				Out_motion(right_foot_terminal,left_foot_terminal,time,phase);
				


			}

			//---- �E�r�V�r�̂Ƃ�
			else{		

				Footcalc(cop_position[l+1],cop_position[l],mass_position[l], &left_foot_pos, &right_foot_pos) ;
				Footcalc(cop_position[l+1],cop_position[l+2],mass_position[l+1], &left_foot_terminal, &right_foot_terminal) ;
				

				while (time < change_time[l+1]){
					all_number = time / cdt;
					
					//^^^^ ����
					out_left_foot.z = 0.0;
					
					if(phase_number == 0.0)	//phase�͂��߂͍���0
					out_right_foot.z = 0.0;
					
					else{
						if (phase_number < number_of_step / 2 )
						out_right_foot.z = param.max_swing_height ;

						else{
						out_right_foot.z = param.max_swing_height - (param.max_swing_height * 2 / number_of_step) * (phase_number - number_of_step / 2) ; 
						}
					}


					//^^^^ �O��
					out_right_foot.x = right_foot_pos.x + (right_foot_terminal.x - right_foot_pos.x) * phase_number / number_of_step ;
					
					// �x���r�͋r�������ȊO�͌v�悳�ꂽ�d�S��ۂ悤�ɓ�����
					if(phase_number == 0.0)
					out_left_foot.x = left_foot_pos.x ;
					
					// �d�S�Ǝx���r�̑O��֌W�ŏꍇ����
					else{
						
						if(cop_position[l+1].x > mass_trajectory[(int)all_number].x){
						mass_pos = cop_position[l+1].x - mass_trajectory[(int)all_number].x ;
						out_left_foot.x =SupPos( abs(out_right_foot.x), mass_pos );

						}
						
						else{
						mass_pos = mass_trajectory[(int)all_number].x -  cop_position[l+1].x;
						out_left_foot.x = - SupPos( abs(out_right_foot.x),mass_pos);
						}

					}


					//^^^^ ���E 
					//-- �V�r�͎��̖ڕW����ʒu�Ɍ������Ĉ�ӂɓ�����
					out_right_foot.y = right_foot_pos.y + (right_foot_terminal.y - right_foot_pos.y) * phase_number / number_of_step ;

					// �x���r�͋r�������ȊO�͌v�悳�ꂽ�d�S��ۂ悤�ɓ�����
					if(phase_number == 0.0)
					out_left_foot.y = left_foot_pos.y;

					else{
						mass_pos = cop_position[l+1].y - mass_trajectory[(int)all_number].y;
						out_left_foot.y =SupPos(mass_pos , out_right_foot.y);

					}


					//�o�͂��ăC���N�������g
					Out_motion(out_right_foot,out_left_foot,time,phase);
					time = time + cdt;
					phase_number ++;
				}
				

				//����I�����͕ʓr�o��
				if(l == param.total_step - 2)
				Out_motion(right_foot_terminal,left_foot_terminal,time,phase);

			}
			
			phase = (phase == 0 ? 1 : 0);		//phase�̐؂�ւ�
		}

	
	}
}
	


// enuvo2��ł� x:front y:height z:side
// enuvo2�ɂ�����0���E�r�C1�����r
//\\\\\\\\\\\\\\\\\\\\\\\\\\\ ����ڕW�ʒu���o�͂���N���X \\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\


// �����͉E���拗���A�����拗���A����
// ������phase��ǉ�����
void BipedLIP::Out_motion(Vec3d p_r,Vec3d p_l,real_t t,int phase){
	if(!MPC::calc.startcalc){	//�ȍ~���s���Ȃ�
		return;
	}
	

	if(t == 0){
		if (param.out_csv){
			ofstream ofs( "../../../enuvo/enuvo2/bin/motion/delta_pos.csv", ios::out );	//�t�@�C���V�K�쐬
			ofs << "time, foot0x, foot0y, foot0z, foot1x, foot1y, foot1z\n";
		}
		
		if(MPC::calc.firstcalc){
			if (param.out_enuvo){
				// ���L���������J��
				MPC::cntrl.enuvoCtrl  = MPC::cntrl.smCtrl .TryOpen<ENuvo2::Control>("ENUVO2_CONTROL_SHARED_MEMORY");
			}
		}
		
		int n = param.step_period_max * param.total_step / param.control_period;	//�v�f��
		MPC::cntrl.foot0 = new Vec3f[n];
		MPC::cntrl.foot1 = new Vec3f[n];

		counter = 0 ;
		out_check = 0;

	}

	//���撷�����e���[�J�����W�ւƕϊ�
	pos_right.x = p_r.x + 0.03 ;	//�i�s����
	pos_left.x = p_l.x + 0.03 ;

	pos_right.y = p_r.y - 80e-3;	//������
	pos_left.y = - (p_l.y - 80e-3) ;

	pos_right.z = p_r.z ;//����
	pos_left.z = p_l.z ;


 
	if (param.out_csv){
		ofstream ofs( "../../../enuvo/enuvo2/bin/motion/delta_pos.csv", ios::app );	//�t�@�C���X�V
		ofs << t * 1000 << ","  << pos_right.x << "," <<  pos_right.z << "," <<  pos_right.y << "," ;
		ofs << pos_left.x << "," << pos_left.z << "," << pos_left.y <<"," <<phase<<"\n" ;
	}

	if (param.out_enuvo){
		MPC::cntrl.foot0[MPC::cntrl.j].x = pos_right.x;
		MPC::cntrl.foot0[MPC::cntrl.j].y = pos_right.z;					
		MPC::cntrl.foot0[MPC::cntrl.j].z = pos_right.y;
		MPC::cntrl.foot1[MPC::cntrl.j].x = pos_left.x;
		MPC::cntrl.foot1[MPC::cntrl.j].y = pos_left.z;
		MPC::cntrl.foot1[MPC::cntrl.j].z = pos_left.y;
		MPC::cntrl.j++;
	}



	/*

	//�I����
	if ( t+param.calc_period > traj.back() -> tick -> time){
		if (param.out_csv){

		}

		if (param.out_enuvo){
			
			if(MPC::calc.firstcalc){
				MPC::timer.SetCallback(&MPC::myCallback);
				MPC::timer2.SetCallback(&MPC::myCallback2);
			}
			// �^�C�}�n��
			MPC::timer.Start(MPC::cntrl.timerPeriod);
			// �^�C�}2�n��
			MPC::timer2.Start(MPC::cntrl.timerPeriod2);

		}

//		cout<< "complete!" <<endl;
		MPC::calc.startcalc = false;
		MPC::calc.firstcalc = false;
		MPC::calc.firstout = true;
	}
	
	*/
	counter ++; 

}






void BipedLIP::DrawSnapshot(real_t time, GRRenderIf* render, DrawConfig* conf){
}


//-------------------------------------------------------------------------------------------------
// Constructors

LIPCon::LIPCon(Solver* solver, string _name, BipedLIPKey* _obj, uint _idx, real_t _scale):
	Constraint(solver, 1, ID(0, _obj->node, _obj->tick, _name), _scale){
	obj[0] = _obj;
	obj[1] = (BipedLIPKey*)_obj->next;
	idx    = _idx;
}

LIPConP::LIPConP(Solver* solver, string _name, BipedLIPKey* _obj, uint _idx, real_t _scale):
	LIPCon(solver, _name, _obj, _idx, _scale){

	AddSLink(obj[0]->pos_t  [idx]);
	AddSLink(obj[0]->vel_t  [idx]);
	AddSLink(obj[0]->pos_cop[idx]);
	AddSLink(obj[0]->period);
	AddSLink(obj[1]->pos_t  [idx]);

}

LIPConV::LIPConV(Solver* solver, string _name, BipedLIPKey* _obj, uint _idx, real_t _scale):
	LIPCon(solver, _name, _obj, _idx, _scale){
	AddSLink(obj[0]->pos_t  [idx]);
	AddSLink(obj[0]->vel_t  [idx]);
	AddSLink(obj[0]->pos_cop[idx]);
	AddSLink(obj[0]->period);
	AddSLink(obj[1]->vel_t  [idx]);

}


//-------------------------------------------------------------------------------------------------
// CalcCoef(�W���j


void LIPCon::Prepare()
{
	BipedLIP::Param& param = obj[0]->GetNode()->param;
	T  = sqrt(param.height/param.gravity);
	t  = obj[0]->period->val/T;
	ch = cosh(t);
	sh = sinh(t);
	p0 = obj[0]->pos_t  [idx]->val;
	v0 = obj[0]->vel_t  [idx]->val;
	c  = obj[0]->pos_cop[idx]->val;
	p1 = obj[1]->pos_t  [idx]->val;
	v1 = obj[1]->vel_t  [idx]->val;


}


void LIPConP::CalcCoef()
{
	Prepare();
	((SLink*)links[0])->SetCoef(-ch);
	((SLink*)links[1])->SetCoef(-T * sh);
	((SLink*)links[2])->SetCoef(ch - 1.0);
	((SLink*)links[3])->SetCoef(- ((p0-c)/T)*sh - v0*ch);
	((SLink*)links[4])->SetCoef(1.0);
}


void LIPConV::CalcCoef()
{
	Prepare();
	((SLink*)links[0])->SetCoef(-(1/T)*sh);
	((SLink*)links[1])->SetCoef(-ch);
	((SLink*)links[2])->SetCoef( (1/T)*sh);
	((SLink*)links[3])->SetCoef(- ((p0-c)/(T*T))*ch - (v0/T)*sh);
	((SLink*)links[4])->SetCoef(1.0);
}



//-------------------------------------------------------------------------------------------------
// CalcDeviation

void LIPConP::CalcDeviation(){
	y[0] =p1 - c - (p0-c)*ch - (v0*T)*sh;	//�S���덷�C������ŏ���������

}

void LIPConV::CalcDeviation(){
	y[0] = v1 - ((p0-c)/T)*sh - v0*ch;
	}
}
