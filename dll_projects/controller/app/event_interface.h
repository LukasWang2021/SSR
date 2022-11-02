#pragma once
#include "comm_def.h"
#include "protoc.h"



/**
 * @brief Initialize the client to pull event.
 * @details Initialize the socket.\n
 *          Start a thread to receive event from the server.\n
 * @param [in] server_ip the IP of the publishing server.
 * @retval 0 Success.
 * @retval -1 Failed to initialize.
 */
extern "C" COMM_INTERFACE_API uint64_t c_initEvent(char* server_ip);

/**
 * @brief The client exits.
 * @details 
 * @retval 0 Success.
 * @retval -1 Failure.
 */
extern "C" COMM_INTERFACE_API uint64_t c_exitEvent(void);

/**
 * @brief Get the error code from Controller.
 * @details Should be cyclical called to check if there is error.
 * @param [out] error The error code buffer. The maximum size is eight.
 * @param [out] time_stamp The ISR of the error. The maximum size is eight.
 * @param [out] size The number of the error codes in the buffer.
 * @retval 0 Success.
 * @retval -1 Failure.
 */
extern "C" COMM_INTERFACE_API uint64_t c_getEventErrorList(uint64_t error[8], uint64_t time_stamp[8], int32_t* size);

