#ifndef __TP_SETTING_H
#define __TP_SETTING_H

// �^�X�N�X�P�W���[�����O�̍ۂɋO���v���ɓn���v��d�l
// ���b�N�E�e�[�u���E�e�[�u���n�p


struct TP_setting
{
	const static int nTSS = 5;	// ���[�`���O�^�X�N�̐�

	// �^�C���X���b�g�̐ݒ�
	struct TSS {
		double ts, te;	// �J�n���ԁA�I������
	};


	double ts, dt, te;	// ������
	
	// �^�C���X���b�g
	struct TSS tss[nTSS];

	TP_setting( double ts=0, double dt=0.5, double te=10)
		: ts(ts), dt(dt), te(te)
	{
		tss[0].ts = 0;	tss[0].te = 0;	// �^�X�N�����ɂ������Ƃ���(0,0)�Ƃ��Ƃ�
		tss[1].ts = 0;	tss[0].te = 0;
		tss[2].ts = 0;	tss[0].te = 0;
		tss[3].ts = 0;	tss[0].te = 0;
		tss[4].ts = 0;	tss[0].te = 0;
	}
};


#endif