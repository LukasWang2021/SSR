#pragma once

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    ERR_CONFIG = 1,
    ERR_RPC = 2,
    ERR_TOUCH = 3,
    ERR_TIMER = 4,
    ERR_READ_THREAD = 5,
    ERR_MAX = 6,
}TouchErr_e;

void PushErrInfo(int module, const char* err_str);

int GetErrPtr(char** ptr, int index);

char* GetErrInfoPtr(void);

int GetErrInfo(char* ptr, int size);

void ErrInfoReset(void);

#ifdef __cplusplus
}
#endif