#ifndef __TP_SETTING_H_
#define __TP_SETTING_H_


// �^�X�N�X�P�W���[������̃��X�|���X���i�[����N���X


#include <time.h>


struct TP_response
{
    enum RES {
        SUCCESS = 0,    // ����   
        FAIL = -1,              // ���s
        INVALID         // �s�����
    };

    bool          response;   // �^�X�N�X�P�W���[������l��Ԃ��Ƃ��� true

    RES              ret;              // �^�X�N�X�P�W���[�����O�̕Ԃ�l
    unsigned    cycle;          // ������
    double        conError;   // �S���덷
    clock_t        calcTime;   // �v�Z����


    TP_response()
        : response(false), ret(RES::INVALID), cycle(2^12), conError(2^12), calcTime(2^12)   // �K���ɑ傫���l
    {}
};


#endif

