/**
 * @file safety_interface.h
 * @brief 
 * @author WangWei
 * @version 1.0.0
 * @date 2016-12-05
 */

#ifndef TP_INTERFACE_SAFETY_INTERFACE_H_
#define TP_INTERFACE_SAFETY_INTERFACE_H_
#include "fst_error.h"


typedef struct _SaftyInputFrm2
{
    char pause:1;
    char e_stop0:1;
    char e_stop1:1;
    char brake3:1;
    char brake2:1;
    char brake1:1;
    char D1:1;
    char D0:1;
}SaftyInputFrm2;

typedef struct _SaftyInputFrm3
{
    char e_stop:1;
    char D6:1;
    char D5:1;
    char D4:1;
    char D3:1;
    char D2:1;
    char D1:1;
    char D0:1;
}SaftyInputFrm3;

typedef struct _SaftyOutputFrm2
{
    char warn:1;
    char D6:1;
    char D5:1;
    char D4:1;
    char D3:1;
    char D2:1;
    char D1:1;
    char D0:1;
}SaftyOutputFrm2;

typedef struct _SaftyOutputFrm3
{
    char D7:1;
    char D6:1;
    char D5:1;
    char D4:1;
    char D3:1;
    char D2:1;
    char D1:1;
    char D0:1;
}SaftyOutputFrm3;

class SafetyInterface
{
  public:
    SafetyInterface();
    ~SafetyInterface();

    char getDigitalValue(int frame_id, int index, U64 &err);
    U64 setDigitalOutput(int frame_id, int index, char data);

    char getDIStatusPause(U64 &err);
    char getDIStatusBrake3(U64 &err);
    char getDIStatusBrake2(U64 &err);
    char getDIStatusBrake1(U64 &err);
    char getDIDeadmanPanic(U64 &err);
    char getDIDeadmanNormal(U64 &err);
    char getDITPManual(U64 &err);
    char getDITPLimitedManual(U64 &err);
    char getDITPAuto(U64 &err);
    char getDIStatusEStop(U64 &err);
    char getDIStatusLimitedEStop(U64 &err);
    char getDIStatusExtEStop(U64 &err);
    char getDOStatusWarn(U64 &err);
    U64 setDOStatusWarn(char data);

    U64 setSafetyHeartBeat();
    bool isSafetyValid();
  private:
    bool valid_flag_;
};

#endif

