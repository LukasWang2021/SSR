/**
 * @file error_process.h
 * @brief 
 * @author WangWei
 * @version 1.0.0
 * @date 2016-12-05
 */
#ifndef TP_INTERFACE_ERROR_PROCESS_H_
#define TP_INTERFACE_ERROR_PROCESS_H_

#include "fst_error.h"

class ErrorProcess
{
  public:
    ErrorProcess();
    ~ErrorProcess();

    U64 getInfoCode();
    U64 getPauseCode();
    U64 getStopCode();
    U64 getServo01Code();
    U64 getServo02Code();
    U64 getAbortCode();
    U64 getSystemCode();

    bool setInfoCode(U64 code);
    bool setPauseCode(U64 code);
    bool setStopCode(U64 code);
    bool setServo01Code(U64 code);
    bool setServo02Code(U64 code);
    bool setAbortCode(U64 code);
    bool setSystemCode(U64 code);
  private:
    U64 err_info_;
    U64 err_pause_;
    U64 err_stop_;
    U64 err_servo01_;
    U64 err_servo02_;
    U64 err_abort_;
    U64 err_system_;
};

#endif
