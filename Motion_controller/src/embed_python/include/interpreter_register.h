/* This module provide the interface to control the axis group.
 * In this case it is a robot arm in PUMA structure.
 */


#ifndef INTERPRETER_REGISTER_H
#define INTERPRETER_REGISTER_H

#include "common_error_code.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
    int coord; /* the posture is in joint or cartesian coodinate*/
    int posture[4];
    int turn[9];
    double pos[9]; /* the posture value of j1~j9/x-y-z-a-b-c */
}PrInfo;

typedef union 
{
	char sr[256];
    int mr;
    double rr;
    PrInfo pr;
}RegValue;

bool InterpReg_Init(void);

ErrorCode InterpReg_GetRR(int id, RegValue *val);
ErrorCode InterpReg_SetRR(int id, RegValue *val);

ErrorCode InterpReg_GetSR(int id, RegValue *val);
ErrorCode InterpReg_SetSR(int id, RegValue *val);

ErrorCode InterpReg_GetMR(int id, RegValue *val);
ErrorCode InterpReg_SetMR(int id, RegValue *val);

ErrorCode InterpReg_GetPR(int id, RegValue *val);
ErrorCode InterpReg_SetPR(int id, RegValue *val);

#ifdef __cplusplus
}
#endif

#endif