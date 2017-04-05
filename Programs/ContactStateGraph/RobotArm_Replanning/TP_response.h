#ifndef __TP_SETTING_H_
#define __TP_SETTING_H_


// タスクスケジューラからのレスポンスを格納するクラス


#include <time.h>


struct TP_response
{
    enum RES {
        SUCCESS = 0,    // 成功   
        FAIL = -1,              // 失敗
        INVALID         // 不正状態
    };

    bool          response;   // タスクスケジューラから値を返すときに true

    RES              ret;              // タスクスケジューリングの返り値
    unsigned    cycle;          // 反復回数
    double        conError;   // 拘束誤差
    clock_t        calcTime;   // 計算時間


    TP_response()
        : response(false), ret(RES::INVALID), cycle(2^12), conError(2^12), calcTime(2^12)   // 適当に大きい値
    {}
};


#endif

