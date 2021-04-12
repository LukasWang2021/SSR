#ifndef LOG_MANAGER_PRODUCER_BARE_H
#define LOG_MANAGER_PRODUCER_BARE_H

/**
 * @file log_manager_producer_bare.h
 * @brief The file is the header file of functions used in Bare Core.
 * @author Feng.Wu
 */

#include <sys/types.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdint.h>
#include "log_manager_datatype.h"

#define SELF_CPU_ID 2

#ifdef __cplusplus
	extern "C"{
#endif

/**
 * @brief Register one memory block for log.
 * @details Find one available memory block and initialize its control head.\n
 * @param [in] thread_name The thread name which is using this log block.
 * @param [in] isr_count_ptr The address pointer of Interrupt Service Routine. ISR is one of the log elements.
 * @return The index to the resource when operating on the log block.
 */
int32_t initLogProducerBare(const char *thread_name, uint32_t *isr_count_ptr);
/**
 * @brief Set which level up information will be logged.
 * @details typedef enum{
 *          LOG_DEBUG = 0,
 *          LOG_INFO  = 1,
 *          LOG_WARN  = 2,
 *          LOG_ERROR = 3,
 *          }MessageLevel;
 * @param [in] fd The index return from initLogProducerBare(...).
 * @param [in] level This level of the log information. The default value is LOG_INFO.
 * @return void
 */
void setLoggingLevelBare(int32_t fd, MessageLevel level);
/**
 * @brief Log the debug information.
 * @param [in] fd The index return from initLogProducerBare(...).
 * @param [in] string The logging string.
 * @return void
 */
void debugBare(int32_t fd, const char *string);
/**
 * @brief Log the info information.
 * @param [in] fd The index return from initLogProducerBare(...).
 * @param [in] string The info string.
 * @return void
 */
void infoBare(int32_t fd, const char *string);
/**
 * @brief Log the warn information.
 * @param [in] fd The index return from initLogProducerBare(...).
 * @param [in] string The warn string.
 * @return void
 */
void warnBare(int32_t fd, const char *string);
/**
 * @brief Log the error information.
 * @param [in] fd The index return from initLogProducerBare(...).
 * @param [in] string The error string.
 * @return void
 */
void errorBare(int32_t fd, const char *string);
/**
 * @brief clear the resource to opearte on the memory block.
 * @param [in] fd The index return from initLogProducerBare(...).
 * @retval 0 success.
 * @retval -1 failed.
 */
int32_t unblindLogProducerBare(int32_t fd);

#ifdef __cplusplus
}
#endif


#endif
