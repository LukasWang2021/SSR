#ifndef LOG_MANAGER_PRODUCER_BARE_H
#define LOG_MANAGER_PRODUCER_BARE_H

#include <sys/types.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdint.h>
#include "log_manager_datatype.h"

#define SELF_CPU_ID 2

namespace virtual_servo_device{

#ifdef __cplusplus
extern "C"{
    #endif

    int32_t initLogProducerBare(const char *thread_name, uint32_t *isr_count_ptr);

    void setLoggingLevelBare(int32_t fd, MessageLevel level);

    void debugBare(int32_t fd, const char *string);
    void infoBare(int32_t fd, const char *string);
    void warnBare(int32_t fd, const char *string);
    void errorBare(int32_t fd, const char *string);

    int32_t unblindLogProducerBare(int32_t fd);

    #ifdef __cplusplus

}
#endif
}

#endif

