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
	bool	startcalc;					///< 軌道計画計算を開始したか
	bool	firstcalc;					///< 起動後初回の計算かどうか
	bool	timerfunc;					///< TimerFuncがなぜか二回呼ばれるのでその対応
	bool	timeron;					///< 次の計算を開始
	bool	firststep;					///< 最初の一歩かどうか
	double	testvel_x, testvel_y;		///< テスト用
	Vec3f	firstrate;					///< 外乱印加時の検知角速度
	vec2_t	rfoot_place_log[5];			///< log用
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
	int		stepcount;					///< 計画開始時が何歩目か
	int		footcount;					///< 歩いた歩数
	int		total_step;					///< 計画歩数
	int		end_step;					///< 実際に歩く歩数
	int		deletecount;				///< tickを消した回数
	vec2_t	last_velocity;				///< 一歩終了したときの速度
	bool	judge_grounded;				///< 接地点判定を用いるか
	vec2_t	moving_distance;			///< 重心が進んだ距離
	vec2_t	last_cop;					///< 一歩での進んだ距離

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
		total_step		= 7;			///< 軌道計画での計画歩数
		end_step		= 7;			///< 実際に歩く歩数（これだけ歩くと制御がとまる）
		judge_grounded	= false;		///< 接地点判定を用いるかどうか

	}
};

extern Calc				calc;

class Cntrl{
public:
	SharedMemory		smState;				///< 状態取得用の共有メモリ
	SharedMemory		smCtrl;					///< 制御入力用の共有メモリ
	ENuvo2::State*		enuvoState;				///< 状態の構造体へのポインタ
	ENuvo2::Control*	enuvoCtrl;				///< 制御入力の構造体へのポインタ
	Event				evStateUpdate;			///< 状態更新イベント
	Event				evCtrlUpdate;			///< 制御更新イベント
	Event				evTimerUpdate;			///< タイマー更新イベント
	Event				evMpcStart;				///< MPC開始イベント
	Event				evFforwardEnd;			///< フィードフォワード終了イベント

	uint				timerPeriod;			///< タイマ周期
	uint				timeElapsed;			///< 経過時間
	uint				timerPeriod2;			///< タイマ周期
	uint				timeElapsed2;			///< 経過時間
	uint				timerPeriod3;			///< タイマ周期
	uint				timeElapsed3;			///< 経過時間
	Vec3f				*foot0;					///< 右足先位置
	Vec3f				*foot1;					///< 左足先位置
	int					i,j,m,l;				///< 要素数
	ENuvo2::JointID		jointid;				///< 関節のID
	bool				rl;						///< 外乱印加後一歩目の支持脚がどちらか(Left=0,Right=1)
	bool				realphase;				///< 実機での支持脚がどちらか
	bool				*calcphase;				///< 計画での支持脚がどちらか
	int					*timing_changephase;	///< 計画での支持脚交換のタイミング
	int					prephase;
	FILE				*fp;
	char				*fname;
	int					ret;
	float				swing[16], support[16], swingplus[16];
	int					csv_num, csv_i;
	bool				accdirection;
	vec3_t				initialangle[2];		///< 初期角度
	vec3_t				initiallength[2];		///< 初期足長さ
	vec3_t				ratetovel;				///< 角速度から変換した並進速度
	float				height, radius;			///< 転倒判別用
	bool				fforwardend;			///< 応急措置用
	float				detectCoe_swing,detectCoe_support;	///< 足上げ動作用

	Cntrl(){
		enuvoCtrl			= smCtrl .TryOpen<ENuvo2::Control>("ENUVO2_CONTROL_SHARED_MEMORY");
		evStateUpdate .name = "ENUVO2_STATE_UPDATE_EVENT";
		evCtrlUpdate  .name = "ENUVO2_CTRL_UPDATE_EVENT";
		evTimerUpdate .name = "ENUVO2_TIMER_UPDATE_EVENT";
		evMpcStart    .name = "ENUVO2_MPC_START_EVENT";
		evFforwardEnd .name = "ENUVO2_MPC_END_EVENT";
		timerPeriod			= 5;   ///< タイマ周期
		timeElapsed			= 0;    ///< 経過時間
		timerPeriod2		= 5;	///< タイマ周期
		timeElapsed2		= 0;	///< 経過時間
		timerPeriod3		= 5;	///< タイマ周期
		timeElapsed3		= 0;	///< 経過時間
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
	/// タイマ処理
	virtual void OnTimer(){
		cntrl.timeElapsed += cntrl.timerPeriod;

		if(cntrl.timeElapsed % 5 == 0){

			
			/// 接地点判定を用いる場合の処理
			if (MPC::calc.judge_grounded){
				cntrl.evCtrlUpdate.Wait(1000);

				if(cntrl.j != 0){
					if(MPC::cntrl.l <= MPC::cntrl.m && cntrl.i == MPC::cntrl.timing_changephase[MPC::cntrl.l] - 1){
						if(cntrl.calcphase[MPC::cntrl.l] == 0){
							if(cntrl.enuvoState->force[4] > 50){	//１歩分出力して，支持脚交換したタイミングで終了
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
							}else{																//次の支持脚が接地するまでは今の状態のまま待つ
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
							if(cntrl.enuvoState->force[5] > 50){	//１歩分出力して，支持脚交換したタイミングで終了
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
							}else{																//次の支持脚が接地するまでは今の状態のまま待つ
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

			/// 接地点判定を行わない場合の処理
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
						

						//最後は足をおろして静止
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

extern Timer			timer;			///< タイマ
extern MyTimerCallback  myCallback;		///< タイマコールバック

class MyTimerCallback2 : public TimerCallback{
public:
	/// タイマ処理
	virtual void OnTimer(){
		cntrl.timeElapsed2 += cntrl.timerPeriod2;

		if(cntrl.timeElapsed2 % 5 == 0 ){
			//csvデータを使う場合
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

extern Timer			timer2;			///< タイマ
extern MyTimerCallback2  myCallback2;	///< タイマコールバック

class MyTimerCallback3 : public TimerCallback{
public:
	/// タイマ処理
	virtual void OnTimer(){
		cntrl.timeElapsed3 += cntrl.timerPeriod3;

		if(cntrl.timeElapsed3 % 5 == 0 ){
			cntrl.evStateUpdate.Wait(1000);	//センサ情報が更新されるまで待つ

			////横方向の転倒判別
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
			////前後方向の転倒判別
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

extern Timer			 timer3;		///< タイマ
extern MyTimerCallback3  myCallback3;	///< タイマコールバック

//timerの終了判定
extern void Timerend();

//timer2の終了判定
extern void Timer2end();

};