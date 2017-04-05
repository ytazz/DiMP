#include <DiMP2/Graph.h>
#include <DiMP2/Mpc.h>


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
//		con_fix_pos[0] = new FixConS(solver, ID(0, node, tick, name + "_fix_pos0"), pos_t[0], node->graph->scale.pos_t);
//		con_fix_pos[1] = new FixConS(solver, ID(0, node, tick, name + "_fix_pos1"), pos_t[1], node->graph->scale.pos_t);
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
	

	support_min[0]  = vec2_t( -0.03, -0.2 );  /// ����(-0.3,-0.2) <���� of Left Phase	(�i�s����,�������j
	support_max[0]  = vec2_t( 0.03, -0.05);		/// ����( 0.3,-0.05)
	
	support_min[1]  = vec2_t( -0.03,  0.05);  /// ����(-0.3,0.05) <���� of Right Phase
	support_max[1]  = vec2_t( 0.03,  0.2 );	/// ����(0.3,0.2)


	

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
	Vec3f m,p,v;											//(Vec3f:float�^�O�����x�N�g��)

	// �d�S�ʒu
	m.x = (float)pos_t[0]->val;
	m.y = (float)pos_t[1]->val;
	m.z = (float)GetNode()->param.height;
	render->DrawPoint(m);					//p�_��`�ʂ���
	
	render->SetPointSize(8.0f,1);
	// COP�ʒu(�ڒn�ʒu)
	p.x = (float)pos_cop[0]->val;
	p.y = (float)pos_cop[1]->val;
	p.z = 0.0f ;

	
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
						
//			key->con_fix_pos[0]->desired = param.terminal_pos[0];
//			key->con_fix_pos[1]->desired = param.terminal_pos[1];
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
//�d�S�O���̌v�Z�C�`��


void BipedLIP::Draw(GRRenderIf* render, DrawConfig* conf)
{
	TrajectoryNode::Draw(render, conf);


	//----------------------------- ������Ԃ�^����N���X -------------------------------
	//if (param.use_beginning == true)
	//	BipedLIP::beginning();

	//////////////////////////���W���̕\��//////////////////////////////////
	//render->SetLineWidth(0.1f);			
	//render->DrawLine(vec3_t(0.0,0.0,0.0),vec3_t(1.0,0.0,0.0));	//�i�s����
	//render->DrawLine(vec3_t(0.0,-0.2,0.0),vec3_t(0.0,0.4,0.0));	//������(���͉E�����j
	//render->DrawLine(vec3_t(0.0,0.0,0.0),vec3_t(0.0,0.0,0.8));	//��������
	///////////////////////////////////////////////////////////////////////

	
	//csv�t�@�C���֏o�͂��鑫��ڕW�ʒu�̏�����
	pos_right = vec3_t(0.0,0.0,0.0);
	pos_left = vec3_t(0.0,0.0,0.0);


	//��������ʒu�̕`��
	render->SetPointSize(12.0f);
	render->DrawPoint(vec3_t(0,param.lfoot_place.y,0));
	render->DrawPoint(vec3_t(0,-param.lfoot_place.y,0));



	if(conf->Set(render, DrawItem::ObjectTrajectory, this))
	{

		//////////////////// �g�p����ϐ����`
		real_t tf =  traj.back() -> tick -> time;	//�I�[����
		real_t dt = param.calc_period;							//����(����:0.05)
		real_t t0 = 0.0; 
		real_t t1 = dt;
		Vec3f p0 , p1, p_cop, a ;
		
		// Write_csv�ɓn���l��p��
		Vec3f delta_swing_leg ;		//�V�r�ʒu��

		Vec3f delta_support_leg ;		//�x���r�ʒu��
		
		
		// \\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\    ����ʒu�E�d�S�ʒu�̌Ăяo���@\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\ 
		
		//-----------------------------------		 �e�z��̒�`			---------------------------------------------

		//�z��̗p��
		Vec3f *cop_position;				// �ڒn�ʒu
		cop_position = (Vec3f *)malloc(sizeof(Vec3f) * param.total_step);

		Vec3f *mass_position;				// �x���r�������̏d�S�ʒu
		mass_position = (Vec3f *)malloc(sizeof(Vec3f) * param.total_step);

		Vec3f *mass_trajectory;				// �d�S�O��
		real_t total_period = (int)(param.step_period_max / dt) * param.total_step ;	// ��������ɏ]���d�S�ʒu��
		mass_trajectory = (Vec3f *)malloc(sizeof(Vec3f) * total_period);

		real_t *change_time;				// �x���r�������̎���
		change_time = (real_t *)malloc(sizeof(Vec3f) * total_period);


		//-------------------------- �����Őڒn�ʒu�E�������̏d�S�ʒu���i�[ ------------------------------
		int mass_number = 0;	//�d�S�O�� �C���f�b�N�X
		int i = 0 ;			
		int phase_check;		//phase��1�Ȃ�E�r�V�r
		BipedLIPKey* start_get_phase = GetSegment(t0).first;
		phase_check = start_get_phase->phase;


		while ( t0 < tf ){
			BipedLIPKey* get_phase = GetSegment(t0).first;

			cop_position[0] = PosCOP(0);
			mass_position[0] = PosCOM(0);
			change_time[0] = 0.0;

			if ( phase_check != get_phase->phase ){

				i ++;
				cop_position[i] = PosCOP(t0);	
				mass_position[i] = PosCOM(t0);
				change_time[i] = t0;
				phase_check = get_phase->phase ;

			}
			
		t0 = t0 + dt;
		}

		change_time[i+1] = tf;				//�I�[�������`���Ă���
	



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

			//�����ŏd�S�ʒu��ۑ�����
			mass_trajectory[mass_number].x = p0.x ;
			mass_trajectory[mass_number].y = p0.y ;
			mass_trajectory[mass_number].z = 0.0 ;

			mass_number ++ ;



			p0 = p1;	
			t0 = t1;								

			
			if(t1 == tf)
				break;				//�I�[���ԂŏI��

		t1 = std::min(tf, t1 + dt);		//�������ق��̒l��Ԃ�


		
		}


		// \\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\�@���[�v�ŏd�S�O�������߂���̂ł���𗘗p�@\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\

		real_t phase_time = 0.0;			// phase start time
		real_t phase_number;		// phase�̃X�^�[�g���ۑ������d�S�ʒu�̉��Ԗڂɓ����邩
		real_t number_of_step;		// phase���̏d�S�ʒu�̐�

		Vec3f delta_cop_position;		
		Vec3f delta_mass_position;

		Vec3f d_cop_position;			//�V�r�O��


		for (int l=0 ; l<param.total_step - 1 ; l++){
	
		phase_time = change_time[l];
		phase_number = change_time[l] / dt ;	
		number_of_step = (change_time[l+1] - change_time[l]) / dt;	

		delta_mass_position = mass_position[l+1] - mass_position[l];

		//-------------------------------------- �����̏d�S�ʒu�̌��� ---------------------------------
		if(l==0){

			BipedLIPKey* start = GetSegment(phase_time).first;	//	1�Ȃ�E�r�V�r

			//�ŏ����E�r�V�r�̂Ƃ�
			if(start->phase == 1){
				d_cop_position = vec3_t(0,-param.lfoot_place.y,0);
			}

			else{
				d_cop_position = vec3_t(0,param.lfoot_place.y,0);
			}

		}

		if(l==1){
			BipedLIPKey* next = GetSegment(phase_time).first;	//	1�Ȃ�E�r�V�r
			if(next->phase == 1){
				d_cop_position = vec3_t(0,-param.lfoot_place.y,0);
			}

			else{
				d_cop_position = vec3_t(0,param.lfoot_place.y,0);
			}
		}





		//---------------------------------------	�����̋���	---------------------------------------
		if(l==0){

			BipedLIPKey* start = GetSegment(phase_time).first;	//	1�Ȃ�E�r�V�r


			while (phase_time < change_time[1]){

				Vec3f delta_mass_trajectory = mass_trajectory[(int)phase_number+1] - mass_trajectory[(int)phase_number] ;

				// ---------------�i�s�������o---------------
				d_cop_position.x = d_cop_position.x + delta_mass_trajectory.x * 2;

				
				// csv�t�@�C���p�i�s������
				delta_support_leg.x = delta_mass_trajectory.x;		// �x���r�͏d�S�O���ɓ�����
				delta_swing_leg.x = - delta_mass_trajectory.x;		// �V�r�͏d�S�O���̃}�C�i�X�ɓ�����
				

				// ----------------���������o---------------------
				d_cop_position.y = d_cop_position.y + (cop_position[1].y - param.lfoot_place.y) / number_of_step; 

				//csv�t�@�C���p��������
				// ���ӁF�u�d�S���������ɓ������d�S����͕��ɓ����v���ƂɂȂ�
				delta_support_leg.y = (-1) * delta_mass_trajectory.y;							//�i�x���r�O���j�d�S�O�����Q�Ƃ���
				//delta_swing_leg.y = (cop_position[1].y - param.lfoot_place.y) / number_of_step;	//�i�V�r�O���j�ݒu�ʒu���ɑ΂��ď��X�ɓ������Ă���
				delta_swing_leg.y = 	 delta_mass_trajectory.y ;								//�i�V�r�O���j�d�S�O�����Q�Ƃ���


				// -----------------�����������o------------------
				if(phase_number  < number_of_step / 2 )	//����ւ�莞�Ԃ̔����Ő؂�ւ���
				{
					d_cop_position.z = d_cop_position.z + param.max_swing_height / (number_of_step / 2) ;
					delta_swing_leg.z =  param.max_swing_height / (number_of_step / 2);
				}
				
				else
				{
					d_cop_position.z = d_cop_position.z - param.max_swing_height / (number_of_step / 2) ;
					delta_swing_leg.z =  - param.max_swing_height / (number_of_step / 2);
				}
				
				

				//�`��
				render->SetPointSize(5.0f);
				render->DrawPoint(d_cop_position);
				render->SetLineWidth(0.5f);			
				render->DrawLine(d_cop_position, Pos(phase_time));		//�d�S����ڒn�_�܂ł���Ō���



				//csv�֏o��
				if (start->phase ==1){
					BipedLIP::Write_csv(delta_swing_leg,delta_support_leg,phase_time,dt);
				}
				else
				{
					BipedLIP::Write_csv(delta_support_leg,delta_swing_leg,phase_time,dt);
				}
			
				phase_number++	;
				phase_time = phase_time + dt;

			}
		}




		// --------------------------------------   �I�[�̋O��   ---------------------------------------- 
		else if(l == param.total_step-2 )
		{
			BipedLIPKey* finish= GetSegment(phase_time).first;	//	1�Ȃ�E�r�V�r
			d_cop_position = cop_position[l-1];		//�����̗V�r�ʒu�͑O�̎x���r�ʒu�ɓ�����
			int tra_number = 0  ;	// ��Ԃ̉��Ԗڂ̏d�S�O����


			while (phase_time < change_time[l+1]){

				Vec3f delta_mass_trajectory = mass_trajectory[(int)phase_number] - mass_trajectory[(int)phase_number-1] ;

				// ---------------�i�s�������o---------------
				d_cop_position.x = d_cop_position.x + delta_mass_trajectory.x * 2;

				
				// csv�t�@�C���p�i�s������
				delta_support_leg.x = delta_mass_trajectory.x;		// �x���r�͏d�S�O���ɓ�����
				delta_swing_leg.x = - delta_mass_trajectory.x;		// �V�r�͏d�S�O���̃}�C�i�X�ɓ�����
				

				// ----------------���������o---------------------
				d_cop_position.y = d_cop_position.y + delta_mass_trajectory.y;		//�ЂƂ܂����������d�S�O���ɏ]�킹�� 

				//csv�t�@�C���p��������
				// ���ӁF�u�d�S���������ɓ������d�S����͕��ɓ����v���ƂɂȂ�
				delta_support_leg.y = (-1) * delta_mass_trajectory.y;		
				delta_swing_leg.y = 	 delta_mass_trajectory.y ;

				// -----------------�����������o------------------
				if(tra_number < number_of_step / 2 )	//����ւ�莞�Ԃ̔����Ő؂�ւ���
				{
					d_cop_position.z = d_cop_position.z + param.max_swing_height / (number_of_step / 2) ;
					delta_swing_leg.z =  param.max_swing_height / (number_of_step / 2);
				}

				else
				{
					d_cop_position.z = d_cop_position.z - param.max_swing_height / (number_of_step / 2) ;
					delta_swing_leg.z =  - param.max_swing_height / (number_of_step / 2);
				}
				
				
				//�`��
				render->SetPointSize(5.0f);
				render->DrawPoint(d_cop_position);
				render->SetLineWidth(0.5f);			
				render->DrawLine(d_cop_position, Pos(phase_time));		//�d�S����ڒn�_�܂ł���Ō���
				


				//csv�֏o��
				if (finish->phase ==1){
					BipedLIP::Write_csv(delta_swing_leg,delta_support_leg,phase_time,dt);
				}
				else
				{
					BipedLIP::Write_csv(delta_support_leg,delta_swing_leg,phase_time,dt);
				}
			
				phase_number++	;
				phase_time = phase_time + dt;
				tra_number ++ ;



				}
			}





		// --------------------------------------   ���Ԃ̋���  ---------------------------------------

		else{
			delta_cop_position = cop_position[l+1] - cop_position[l-1];		//�O�̐ڒn�ʒu�Ƃ̍������
			d_cop_position = cop_position[l-1];		//�����̗V�r�ʒu�͑O�̎x���r�ʒu�ɓ�����
			int tra_number = 0  ;	// ��Ԃ̉��Ԗڂ̏d�S�O����

			while ( phase_time < change_time[l+1] )
				{
			
					Vec3f delta_mass_trajectory = mass_trajectory[(int)phase_number] - mass_trajectory[(int)phase_number - 1] ;


					//--------------------- �i�s�������o  ----------------------------
					// �O���o�[�����ł͗V�r�̐i�s�����͏d�S�O����2�{�̋�����i��
					d_cop_position.x = d_cop_position.x + delta_mass_trajectory.x * 2 ;

					// csv�p
					delta_support_leg.x = delta_mass_trajectory.x;		// �x���r�͏d�S�O���ɓ�����
					delta_swing_leg.x = - delta_mass_trajectory.x;		// �V�r�͏d�S�O���̃}�C�i�X�ɓ�����
			
					
					//---------------------- ���������o -----------------------------
					// �O���[�o�����ł͉��������d�S�O����2�{�̋�����i��
					// ������[�J�����ł͐^���������̐ڒn�ʒu�Ɍ������ē�����
					d_cop_position.y = d_cop_position.y +  delta_mass_trajectory.y * 2;

					// �������������߂�
					delta_support_leg.y = (-1) * delta_mass_trajectory.y;			//�x���r�O���͏d�S�O���̕��̒l�ɓ�����
					//delta_swing_leg.y = delta_cop_position.y / number_of_step;	//�i�V�r�O���j�ڒn�ʒu���ɑ΂��ď��X�ɓ�����
					delta_swing_leg.y = 	 delta_mass_trajectory.y ;				//�i�U�q�N���j�d�S�O�����Q��


					
					//---------------------- �����������o ---------------------------
					
					if( tra_number < number_of_step / 2 )	//����ւ�莞�Ԃ̔����Ő؂�ւ���
					{
						d_cop_position.z = d_cop_position.z + param.max_swing_height / (number_of_step / 2) ;
						delta_swing_leg.z = param.max_swing_height / (number_of_step / 2);
					}

					else
					{
						d_cop_position.z = d_cop_position.z - param.max_swing_height / (number_of_step / 2) ;
						delta_swing_leg.z =  - param.max_swing_height / (number_of_step / 2);
					}
					

					render->SetPointSize(5.0f);
					render->DrawPoint(d_cop_position);

					render->SetLineWidth(0.5f);			
					render->DrawLine(d_cop_position, Pos(phase_time));			//�d�S����ڒn�_�܂ł���Ō���



					BipedLIPKey* key = GetSegment(phase_time).first;	//	1�Ȃ�E�r�V�r

					// enuvo2��ł� x:front y:height z:side
					// enuvo2�ɂ�����0���E�r�C1�����r
					if (key->phase ==1){
						BipedLIP::Write_csv(delta_swing_leg,delta_support_leg,phase_time,dt);
					}

					else
					{
						BipedLIP::Write_csv(delta_support_leg,delta_swing_leg,phase_time,dt);
					}
			

					// \\\\\\\\\\\\\\\\\\\\\\\\\\\\\\  �����܂Ł@\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\

					phase_number++	;
					phase_time = phase_time + dt;
					tra_number ++;
				}
		
			}
		}
	
	}
}

// enuvo2��ł� x:front y:height z:side
// enuvo2�ɂ�����0���E�r�C1�����r
//\\\\\\\\\\\\\\\\\\\\\\\\\\\ ����ڕW�ʒu���o�͂���N���X \\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\

int cnt = 0;	//cnt=2�̂Ƃ��o��

void BipedLIP::Write_csv(Vec3f p_r,Vec3f p_l,real_t t,real_t dt){

	if(MPC::startmpc == false || MPC::endcalc == true){	//�ȍ~���s���Ȃ�
		return;
	}
	
	if (param.out_csv == true){
		if ( t == 0 && cnt == 0 ){
			ofstream ofs( "../Biped3/data/delta_pos.csv", ios::out );	//�t�@�C���V�K�쐬
			ofs << "time, foot0x, foot0y, foot0z, foot1x, foot1y, foot1z\n";
		}
	}
			
	if(t == 0){	//�����Ȃ�(������)�Ƃ�
		cnt = 2;
	}

	if( t == 0){
		for (int i = 0 ; i<3 ; i++){	//������
			pos_right[i]  = 0.0;
			pos_left[i] = 0.0;
		}

	counter = 0 ;
	out_check = 0;

	/*
	// ---------------------------- �����ŏ����ʒu�ɑ΂���␳���s�� --------------------------------

	BipedLIPKey* start_cor = GetSegment(0).first;
			
	if(start_cor->phase == 1)	//���r�ɕ␳��������
	{
			for(real_t cor_time = 0 ; cor_time < param.start_move_time ; cor_time = cor_time + dt)
			{
			
			}
		}
	
		//(����͂��������g���Ă���)
		else	//�E�r�ɕ␳��������
		{
			for(real_t cor_time = 0; cor_time < param.start_move_time ; cor_time = cor_time + dt)
			{
			
			}
			
		}
	*/

	}



	pos_right.x = pos_right.x + p_r.x ;	//�i�s����
	pos_left.x = pos_left.x + p_l.x ;

	pos_right.y = pos_right.y + p_r.y;	//������
	pos_left.y = pos_left.y + p_l.y;

	pos_right.z = pos_right.z + p_r.z ;	//����
	pos_left.z = pos_left.z + p_l.z ;

	

	// -------------------------- �x���r�̍�����0�ɂ��� ---------------------

	BipedLIPKey* reset_height = GetSegment(t).first;
	
	if(reset_height->phase == 1){
		pos_left.z = 0.0;
	}else{
		pos_right.z = 0.0; 
	}

	//�����ɉ�����csv�ɏo�͂���f�[�^�����߂�
	int output_count = param.control_period / param.calc_period ;

	//��������DiMP��enuvo�ō��W���t�Ȃ̂�-��������
	//���Ԃɂ͏����ʒu�֑���𓮂������Ԃ�������

	if (counter % output_count == 0 && cnt == 2){ 
		if (param.out_csv == true){
				ofstream ofs( "../Biped3/data/delta_pos.csv", ios::app );	//�t�@�C���X�V
				ofs << (counter * param.control_period / output_count  + param.start_move_time) * 1000 << ","  << pos_right.x << "," <<  pos_right.z << "," <<  (-1) * pos_right.y << "," ;
				ofs << pos_left.x << "," << pos_left.z << "," << (-1) *  pos_left.y << "\n" ;
		}	
		out_check ++ ;
	}


	//�I�[���
	//�v�Z��Ō�̏�Ԃɂ���
	if ( t+dt > traj.back() -> tick -> time && cnt == 2){
		if (param.out_csv == true){
			ofstream ofs( "../Biped3/data/delta_pos.csv", ios::app );	//�t�@�C���X�V
			ofs << out_check * param.control_period * 1000<< ","  << pos_right.x << "," <<  0 << "," <<  (-1) * pos_right.y << "," ;
			ofs << pos_left.x << "," << 0 << "," << (-1) *  pos_left.y << "\n" ;
		}	
		MPC::endcalc = true;
		cout<< "complete" <<endl;
	}

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
