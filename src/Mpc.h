#pragma once

/// Scenebuilder
#include <sbmessage.h>
#include <sbshared.h>
#include <sbtimer.h>

/// enuvo2
#include <base/enuvo2.h>

using namespace Scenebuilder;

namespace MPC{
class Calc{
public:
	bool	startcalc;					///< �O���v��v�Z���J�n������
	bool	firstcalc;					///< �N���㏉��̌v�Z���ǂ���
	bool	timerfunc;					///< TimerFunc���Ȃ������Ă΂��̂ł��̑Ή�
	bool	timeron;					///< ���̌v�Z���J�n
	bool	firststep;					///< �ŏ��̈�����ǂ���
	double	testvel_x, testvel_y;		///< �e�X�g�p
	Vec3f	firstrate;					///< �O��������̌��m�p���x
	vec2_t	rfoot_place_log[5];			///< log�p
	vec2_t	lfoot_place_log[5];
	Vec3f	rate_log[5];
	float	deltax_log[5];
	float	deltay_log[5];
	vec2_t	initial_velocity_log[5];
	bool	rl_log[5];
	Vec3f	*foot0_log[5];
	Vec3f	*foot1_log[5];
	vec2_t	*vel_log[5];
	int		j_log[5];
	int		stepcount;					///< �v��J�n���������ڂ�
	int		footcount;					///< ����������
	int		total_step;					///< �v�����
	int		end_step;					///< ���ۂɕ�������
	int		deletecount;				///< tick����������
	vec2_t	last_velocity;				///< ����I�������Ƃ��̑��x
	bool	judge_grounded;				///< �ڒn�_�����p���邩
	vec2_t	moving_distance;			///< �d�S���i�񂾋���
	vec2_t	last_cop;					///< ����ł̐i�񂾋���

	Calc(){
		startcalc		= false;
		firstcalc		= true;
		timerfunc		= true;
		timeron			= false;
		firststep		= false;
		testvel_x		= 0.0;
		testvel_y		= 0.0;
		stepcount		= 1;
		footcount		= 1;
		total_step		= 7;			///< �O���v��ł̌v�����
		end_step		= 7;			///< ���ۂɕ��������i���ꂾ�������Ɛ��䂪�Ƃ܂�j
		judge_grounded	= false;		///< �ڒn�_�����p���邩�ǂ���

	}
};

extern Calc				calc;

class Cntrl{
public:
	SharedMemory		smState;				///< ��Ԏ擾�p�̋��L������
	SharedMemory		smCtrl;					///< ������͗p�̋��L������
	ENuvo2::State*		enuvoState;				///< ��Ԃ̍\���̂ւ̃|�C���^
	ENuvo2::Control*	enuvoCtrl;				///< ������͂̍\���̂ւ̃|�C���^
	Event				evStateUpdate;			///< ��ԍX�V�C�x���g
	Event				evCtrlUpdate;			///< ����X�V�C�x���g
	Event				evTimerUpdate;			///< �^�C�}�[�X�V�C�x���g
	Event				evMpcStart;				///< MPC�J�n�C�x���g
	Event				evFforwardEnd;			///< �t�B�[�h�t�H���[�h�I���C�x���g

	uint				timerPeriod;			///< �^�C�}����
	uint				timeElapsed;			///< �o�ߎ���
	uint				timerPeriod2;			///< �^�C�}����
	uint				timeElapsed2;			///< �o�ߎ���
	uint				timerPeriod3;			///< �^�C�}����
	uint				timeElapsed3;			///< �o�ߎ���
	Vec3f				*foot0;					///< �E����ʒu
	Vec3f				*foot1;					///< ������ʒu
	int					i,j,m,l;				///< �v�f��
	ENuvo2::JointID		jointid;				///< �֐߂�ID
	bool				rl;						///< �O����������ڂ̎x���r���ǂ��炩(Left=0,Right=1)
	bool				realphase;				///< ���@�ł̎x���r���ǂ��炩
	bool				*calcphase;				///< �v��ł̎x���r���ǂ��炩
	int					*timing_changephase;	///< �v��ł̎x���r�����̃^�C�~���O
	int					prephase;
	FILE				*fp;
	char				*fname;
	int					ret;
	float				swing[16], support[16], swingplus[16];
	int					csv_num, csv_i;
	bool				accdirection;
	vec3_t				initialangle[2];		///< �����p�x
	vec3_t				initiallength[2];		///< ����������
	vec3_t				ratetovel;				///< �p���x����ϊ��������i���x
	float				height, radius;			///< �]�|���ʗp
	bool				fforwardend;			///< ���}�[�u�p
	float				detectCoe_swing,detectCoe_support;	///< ���グ����p

	Cntrl(){
		enuvoCtrl			= smCtrl .TryOpen<ENuvo2::Control>("ENUVO2_CONTROL_SHARED_MEMORY");
		evStateUpdate .name = "ENUVO2_STATE_UPDATE_EVENT";
		evCtrlUpdate  .name = "ENUVO2_CTRL_UPDATE_EVENT";
		evTimerUpdate .name = "ENUVO2_TIMER_UPDATE_EVENT";
		evMpcStart    .name = "ENUVO2_MPC_START_EVENT";
		evFforwardEnd .name = "ENUVO2_MPC_END_EVENT";
		timerPeriod			= 5;   ///< �^�C�}����
		timeElapsed			= 0;    ///< �o�ߎ���
		timerPeriod2		= 5;	///< �^�C�}����
		timeElapsed2		= 0;	///< �o�ߎ���
		timerPeriod3		= 5;	///< �^�C�}����
		timeElapsed3		= 0;	///< �o�ߎ���
		i	= 0;
		j	= 0;
		m	= 0;
		l	= 0;
		rl	= 0;
		accdirection = 0;
		fname	= "../../../enuvo/enuvo2/bin/motion/mpc.csv";
		fp		= fopen( fname, "r" );
		if( fp == NULL ){
		  cout<<"failed to open "<<fname<<endl;
		}
		height	= 0.5445;
	}

};

extern Cntrl			cntrl;




class MyTimerCallback : public TimerCallback{
public:
	/// �^�C�}����
	virtual void OnTimer(){
		cntrl.timeElapsed += cntrl.timerPeriod;

		if(cntrl.timeElapsed % 5 == 0){

			
			/// �ڒn�_�����p����ꍇ�̏���
			if (MPC::calc.judge_grounded){
				cntrl.evCtrlUpdate.Wait(1000);

				if(cntrl.j != 0){
					if(MPC::cntrl.l <= MPC::cntrl.m && cntrl.i == MPC::cntrl.timing_changephase[MPC::cntrl.l] - 1){
						if(cntrl.calcphase[MPC::cntrl.l] == 0){
							if(cntrl.enuvoState->force[4] > 50){	//�P�����o�͂��āC�x���r���������^�C�~���O�ŏI��
								cntrl.i ++;
								cntrl.enuvoCtrl->ikFoot[0].x = cntrl.foot0[cntrl.i].x;
								cntrl.enuvoCtrl->ikFoot[0].y = cntrl.foot0[cntrl.i].y;
								cntrl.enuvoCtrl->ikFoot[0].z = cntrl.foot0[cntrl.i].z;
								cntrl.enuvoCtrl->ikFoot[1].x = cntrl.foot1[cntrl.i].x;
								cntrl.enuvoCtrl->ikFoot[1].y = cntrl.foot1[cntrl.i].y;
								cntrl.enuvoCtrl->ikFoot[1].z = cntrl.foot1[cntrl.i].z;

								MPC::cntrl.l++;
								if(MPC::calc.total_step != 2)
									MPC::calc.firststep = true;
								cntrl.evTimerUpdate.Set();
							}else{																//���̎x���r���ڒn����܂ł͍��̏�Ԃ̂܂ܑ҂�
								cntrl.enuvoCtrl->ikFoot[0].x = cntrl.foot0[cntrl.i].x;
								cntrl.enuvoCtrl->ikFoot[0].y = cntrl.foot0[cntrl.i].y;
								cntrl.enuvoCtrl->ikFoot[0].z = cntrl.foot0[cntrl.i].z;
								cntrl.enuvoCtrl->ikFoot[1].x = cntrl.foot1[cntrl.i].x;
								cntrl.enuvoCtrl->ikFoot[1].y = cntrl.foot1[cntrl.i].y;
								cntrl.enuvoCtrl->ikFoot[1].z = cntrl.foot1[cntrl.i].z;

								if (MPC::calc.footcount > MPC::calc.end_step - 1){
									cntrl.evTimerUpdate.Set();
									cout << "end" ;
								}


							}
						}else{
							if(cntrl.enuvoState->force[5] > 50){	//�P�����o�͂��āC�x���r���������^�C�~���O�ŏI��
								cntrl.i ++;
								cntrl.enuvoCtrl->ikFoot[0].x = cntrl.foot0[cntrl.i].x;
								cntrl.enuvoCtrl->ikFoot[0].y = cntrl.foot0[cntrl.i].y;
								cntrl.enuvoCtrl->ikFoot[0].z = cntrl.foot0[cntrl.i].z;
								cntrl.enuvoCtrl->ikFoot[1].x = cntrl.foot1[cntrl.i].x;
								cntrl.enuvoCtrl->ikFoot[1].y = cntrl.foot1[cntrl.i].y;
								cntrl.enuvoCtrl->ikFoot[1].z = cntrl.foot1[cntrl.i].z;

								MPC::cntrl.l++;
								if(MPC::calc.total_step != 2)
									MPC::calc.firststep = true;
								cntrl.evTimerUpdate.Set();
							}else{																//���̎x���r���ڒn����܂ł͍��̏�Ԃ̂܂ܑ҂�
								cntrl.enuvoCtrl->ikFoot[0].x = cntrl.foot0[cntrl.i].x;
								cntrl.enuvoCtrl->ikFoot[0].y = cntrl.foot0[cntrl.i].y;
								cntrl.enuvoCtrl->ikFoot[0].z = cntrl.foot0[cntrl.i].z;
								cntrl.enuvoCtrl->ikFoot[1].x = cntrl.foot1[cntrl.i].x;
								cntrl.enuvoCtrl->ikFoot[1].y = cntrl.foot1[cntrl.i].y;
								cntrl.enuvoCtrl->ikFoot[1].z = cntrl.foot1[cntrl.i].z;

								if (MPC::calc.footcount > MPC::calc.end_step - 1){
									cntrl.evTimerUpdate.Set();
									cout << "end" ;
								}

							}
						}

					}else{
						cntrl.i ++;
						cntrl.enuvoCtrl->ikFoot[0].x = cntrl.foot0[cntrl.i].x;
						cntrl.enuvoCtrl->ikFoot[0].y = cntrl.foot0[cntrl.i].y;
						cntrl.enuvoCtrl->ikFoot[0].z = cntrl.foot0[cntrl.i].z;
						cntrl.enuvoCtrl->ikFoot[1].x = cntrl.foot1[cntrl.i].x;
						cntrl.enuvoCtrl->ikFoot[1].y = cntrl.foot1[cntrl.i].y;
						cntrl.enuvoCtrl->ikFoot[1].z = cntrl.foot1[cntrl.i].z;


					}
			
				}
			}

			/// �ڒn�_������s��Ȃ��ꍇ�̏���
			else{
			cntrl.evCtrlUpdate.Wait(1000);
			if(cntrl.j != 0){
				if(MPC::cntrl.l <= MPC::cntrl.m && cntrl.i == MPC::cntrl.timing_changephase[MPC::cntrl.l] - 1){

						cntrl.i ++;
						cntrl.enuvoCtrl->ikFoot[0].x = cntrl.foot0[cntrl.i].x;
						cntrl.enuvoCtrl->ikFoot[0].y = cntrl.foot0[cntrl.i].y;
						cntrl.enuvoCtrl->ikFoot[0].z = cntrl.foot0[cntrl.i].z;
						cntrl.enuvoCtrl->ikFoot[1].x = cntrl.foot1[cntrl.i].x;
						cntrl.enuvoCtrl->ikFoot[1].y = cntrl.foot1[cntrl.i].y;
						cntrl.enuvoCtrl->ikFoot[1].z = cntrl.foot1[cntrl.i].z;
						

						MPC::cntrl.l++;
						if(MPC::calc.total_step != 2)
							MPC::calc.firststep = true;
						cntrl.evTimerUpdate.Set();
						
					}else{
						cntrl.i ++;
						cntrl.enuvoCtrl->ikFoot[0].x = cntrl.foot0[cntrl.i].x;
						cntrl.enuvoCtrl->ikFoot[0].y = cntrl.foot0[cntrl.i].y;
						cntrl.enuvoCtrl->ikFoot[0].z = cntrl.foot0[cntrl.i].z;
						cntrl.enuvoCtrl->ikFoot[1].x = cntrl.foot1[cntrl.i].x;
						cntrl.enuvoCtrl->ikFoot[1].y = cntrl.foot1[cntrl.i].y;
						cntrl.enuvoCtrl->ikFoot[1].z = cntrl.foot1[cntrl.i].z;
						

						//�Ō�͑������낵�ĐÎ~
						if (MPC::calc.footcount > MPC::calc.end_step - 1){
							cntrl.enuvoCtrl->ikFoot[0].y = 0.0;
							cntrl.enuvoCtrl->ikFoot[1].y = 0.0;
							cntrl.evTimerUpdate.Set();
							cout << "end\n\n" ;	
						}	
					}

				}
			}

		}
	}
};

extern Timer			timer;			///< �^�C�}
extern MyTimerCallback  myCallback;		///< �^�C�}�R�[���o�b�N

class MyTimerCallback2 : public TimerCallback{
public:
	/// �^�C�}����
	virtual void OnTimer(){
		cntrl.timeElapsed2 += cntrl.timerPeriod2;

		if(cntrl.timeElapsed2 % 5 == 0 ){
			//csv�f�[�^���g���ꍇ
			cntrl.evCtrlUpdate.Wait(1000);

			if(!cntrl.fforwardend){
				if(cntrl.accdirection  == 0){
					cntrl.enuvoCtrl->ikFoot[0].y += cntrl.support[cntrl.csv_i];
					cntrl.enuvoCtrl->ikFoot[1].y += cntrl.swing[cntrl.csv_i];
				}else{
					cntrl.enuvoCtrl->ikFoot[0].y += cntrl.swing[cntrl.csv_i];
					cntrl.enuvoCtrl->ikFoot[1].y += cntrl.support[cntrl.csv_i];
				}
				cntrl.csv_i++;
			}

			if(cntrl.csv_i == cntrl.csv_num){
				cntrl.fforwardend = true;
				MPC::cntrl.evFforwardEnd.Set();
			}
		}
	}
};

extern Timer			timer2;			///< �^�C�}
extern MyTimerCallback2  myCallback2;	///< �^�C�}�R�[���o�b�N

class MyTimerCallback3 : public TimerCallback{
public:
	/// �^�C�}����
	virtual void OnTimer(){
		cntrl.timeElapsed3 += cntrl.timerPeriod3;

		if(cntrl.timeElapsed3 % 5 == 0 ){
			cntrl.evStateUpdate.Wait(1000);	//�Z���T��񂪍X�V�����܂ő҂�

			////�������̓]�|����
			//if((- cntrl.enuvoState->rate.x) > 0){
			//	cntrl.radius = MPC::cntrl.initiallength[0].z;
			//	cntrl.accdirection = 1;
			//}else{
			//	cntrl.radius = MPC::cntrl.initiallength[1].z;
			//	cntrl.accdirection = 0;
			//}
			//if(cntrl.enuvoState->rate.x * cntrl.enuvoState->rate.x > 0.09/*2.0 * 9.8 * (cntrl.radius * cos(M_PI/180*9) - cntrl.height) / (cntrl.radius * cntrl.radius)*/){
			//	cntrl.evMpcStart.Set();
			//}
			//
			////�O������̓]�|����
			//if(cntrl.enuvoState->rate.z > 0){
			//	if((- cntrl.enuvoState->rate.x) > 0)	cntrl.radius = MPC::cntrl.initiallength[0].y;
			//	else									cntrl.radius = MPC::cntrl.initiallength[1].y;
			//}else{
			//	if((- cntrl.enuvoState->rate.x) > 0)	cntrl.radius = MPC::cntrl.initiallength[0].x;
			//	else									cntrl.radius = MPC::cntrl.initiallength[1].x;
			//}
			//if(cntrl.enuvoState->rate.z * cntrl.enuvoState->rate.z > 0.09/*2.0 * 9.8 * (cntrl.radius * cos(M_PI/180*9) - cntrl.height) / (cntrl.radius * cntrl.radius)*/){
			//	cntrl.evMpcStart.Set();
			//}

			cntrl.evMpcStart.Set();
		}
	}
};

extern Timer			 timer3;		///< �^�C�}
extern MyTimerCallback3  myCallback3;	///< �^�C�}�R�[���o�b�N

//timer�̏I������
extern void Timerend();

//timer2�̏I������
extern void Timer2end();

};