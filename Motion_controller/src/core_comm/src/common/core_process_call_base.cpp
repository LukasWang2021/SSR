#ifdef COMPILE_IN_BARE
#include "./core_protocal_inc/core_process_call_base.h"
#include <string.h>
#include <stdlib.h>
#include <assert.h>
#else
#include "common/core_process_call_base.h"
#include <string.h>
#include <stdlib.h>
#include <assert.h>
#include <stdio.h>
#endif

void initCoreProcessCallMemoryByFrom(CommBlockData_t* block_ptr, CoreProcessCallCommData_t* data_ptr)
{
    assert(block_ptr != NULL);
    assert(data_ptr != NULL);
    memcpy(block_ptr->memory_ptr, data_ptr, sizeof(CoreProcessCallCommData_t));
    memset(block_ptr->memory_ptr + sizeof(CoreProcessCallCommData_t), 0, data_ptr->max_data_byte_offset);
}

bool isCoreProcessCallChannelIdle(CommBlockData_t* block_ptr)
{
    assert(block_ptr != NULL);
    return CORE_PROCESS_CALL_REQUEST_TOGGLE == CORE_PROCESS_CALL_RESPONSE_TOGGLE ? true : false;
}

bool sendCoreProcessCallRequest(CommBlockData_t* block_ptr, uint8_t* data_ptr, int32_t data_size, bool enable_async)
{
    assert(block_ptr != NULL);
    assert(data_ptr != NULL);
    assert(data_size > 0);
    
    if(!isCoreProcessCallChannelIdle(block_ptr)
        || data_size > CORE_PROCESS_CALL_MAX_BYTE_OFFSET)
    {
        return false;
    }
    memcpy(CORE_PROCESS_CALL_DATA_PTR, data_ptr, data_size);
    CORE_PROCESS_CALL_VALID_BYTE_OFFSET = data_size;
    CORE_PROCESS_CALL_TIMEOUT_COUNT = block_ptr->param3;
    if(enable_async)
    {
        CORE_PROCESS_CALL_ASYNC_ACK = 0;
    }
    CORE_PROCESS_CALL_REQUEST_TOGGLE = CORE_PROCESS_CALL_REQUEST_TOGGLE + 1;
    return true;
}

bool recvCoreProcessCallResponse(CommBlockData_t* block_ptr, uint8_t* data_ptr, int32_t* data_size_ptr, bool* timeout_ptr)
{
    assert(block_ptr != NULL);
    assert(data_ptr != NULL);
    assert(data_size_ptr != NULL);

    if(CORE_PROCESS_CALL_TIMEOUT_COUNT > 0)
    {
        CORE_PROCESS_CALL_TIMEOUT_COUNT = CORE_PROCESS_CALL_TIMEOUT_COUNT - 1;
        *timeout_ptr = false;
    }
    else
    {
        *timeout_ptr = true;
    }    

    if(!isCoreProcessCallChannelIdle(block_ptr))
    {
        return false;
    }

    *data_size_ptr = CORE_PROCESS_CALL_VALID_BYTE_OFFSET;
    memcpy(data_ptr, CORE_PROCESS_CALL_DATA_PTR, *data_size_ptr);
    return true;
}

bool isCoreProcessCallAsyncAckActived(CommBlockData_t* block_ptr)
{
    assert(block_ptr != NULL);
    return CORE_PROCESS_CALL_ASYNC_ACK == 0 ? false : true;
}

bool recvCoreProcessCallRequest(CommBlockData_t* block_ptr, uint8_t* data_ptr, int32_t* data_size_ptr)
{
    assert(block_ptr != NULL);
    assert(data_ptr != NULL);
    assert(data_size_ptr != NULL);

    if(isCoreProcessCallChannelIdle(block_ptr))
    {
        return false;
    }

    *data_size_ptr = CORE_PROCESS_CALL_VALID_BYTE_OFFSET;
    memcpy(data_ptr, CORE_PROCESS_CALL_DATA_PTR, *data_size_ptr);
    return true;
}

bool sendCoreProcessCallResponse(CommBlockData_t* block_ptr, uint8_t* data_ptr, int32_t data_size)
{
    assert(block_ptr != NULL);
    assert(data_ptr != NULL);
    assert(data_size > 0);

    if(isCoreProcessCallChannelIdle(block_ptr)
        || data_size > CORE_PROCESS_CALL_MAX_BYTE_OFFSET)
    {
        return false;
    }
    memcpy(CORE_PROCESS_CALL_DATA_PTR, data_ptr, data_size);
    CORE_PROCESS_CALL_VALID_BYTE_OFFSET = data_size;
    CORE_PROCESS_CALL_RESPONSE_TOGGLE = CORE_PROCESS_CALL_RESPONSE_TOGGLE + 1;
    return true;
}

void sendCoreProcessCallAsyncAck(CommBlockData_t* block_ptr)
{
    assert(block_ptr != NULL);
    CORE_PROCESS_CALL_ASYNC_ACK = 1;
}


