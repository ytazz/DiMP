#ifndef __TP_SETTING_H
#define __TP_SETTING_H

// タスクスケジューリングの際に軌道計画器に渡す計画仕様
// ラック・テーブル・テーブル系用


struct TP_setting
{
	const static int nTSS = 5;	// リーチングタスクの数

	// タイムスロットの設定
	struct TSS {
		double ts, te;	// 開始時間、終了時間
	};


	double ts, dt, te;	// 時刻列
	
	// タイムスロット
	struct TSS tss[nTSS];

	TP_setting( double ts=0, double dt=0.5, double te=10)
		: ts(ts), dt(dt), te(te)
	{
		tss[0].ts = 0;	tss[0].te = 0;	// タスク無効にしたいときは(0,0)としとく
		tss[1].ts = 0;	tss[0].te = 0;
		tss[2].ts = 0;	tss[0].te = 0;
		tss[3].ts = 0;	tss[0].te = 0;
		tss[4].ts = 0;	tss[0].te = 0;
	}
};


#endif