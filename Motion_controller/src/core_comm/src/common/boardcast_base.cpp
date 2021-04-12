#ifdef COMPILE_IN_BARE
#include "./core_protocal_inc/boardcast_base.h"
#include <string.h>
#else
#include "common/boardcast_base.h"
#include <string.h>
#endif

bool setBoardcast(char* memory_ptr, BoardcastCommData_t* data_ptr)
{
    if(memory_ptr != NULL)
    {
        memcpy(memory_ptr, data_ptr, sizeof(BoardcastCommData_t));
        return true;
    }
    else
    {
        return false;
    }
}

bool getBoardcast(char* memory_ptr, BoardcastCommData_t* data_ptr)
{
    if(memory_ptr != NULL)
    {
        memcpy(data_ptr, memory_ptr, sizeof(BoardcastCommData_t));
        return true;
    }
    else
    {
        return false;
    }
}



