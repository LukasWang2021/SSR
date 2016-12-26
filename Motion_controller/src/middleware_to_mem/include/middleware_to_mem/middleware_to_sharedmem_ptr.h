/**********************************************
Copyright © 2016 Foresight-Robotics Ltd. All rights reserved.
File:       middleware_to_sharedmem_ptr.h
Author:     Feng.Wu / Yan.He 
Create:     16-Aug-2016
Modify:     03-Nov-2016
Summary:    Init the middleware
**********************************************/
#ifndef MIDDLEWARE_TO_MEM_MIDDLEWARE_TO_SHAREDMEM_PTR_H_
#define MIDDLEWARE_TO_MEM_MIDDLEWARE_TO_SHAREDMEM_PTR_H_

#include <sys/types.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "struct_to_mem/data_type.h"

#ifndef CPU1_SHAREDMEM  
#include <sys/mman.h>
#include <unistd.h>
#include <fcntl.h>
#else                      //for compile in bare-metal core
#define printf(arg,...)
#endif //CPU1_SHAREDMEM

typedef struct
{
    char* ptr;
    const FunctionTable *table;
    const int table_length;
    const int map_size;
}HandleTable;
#define HANDLE_TABLE_LEN 3 // need to change if add another sharedmem area.

#define MEM_PROCESS 0 // passed into openMem().
#define MEM_CORE 1
#define MEM_BARE 2

#ifdef __cplusplus
	extern "C"{
#endif
//------------------------------------------------------------
// Function:    openMem
// Summary: Open the shared mem.
// In:      type -> indicate which memory area will be open.
//                  parameters are MEM_PROCESS.
// Out:     None
// Return:  hanlde -> hide the pointer and other info.
//          -1 -> failed to map the memory area.
//------------------------------------------------------------
int openMem(int type);

//------------------------------------------------------------
// Function:  getPtrOfMem
// Summary: Get the pointer according to the handle.
// In:      handle -> Passing the handle got from openMem() function.
// Out:     None
// Return:  pointer -> success
//          null -> failed.
//------------------------------------------------------------
char* getPtrOfMem(int handle);

//------------------------------------------------------------
// Function:  clearSharedmem
// Summary: Set the shared memory to zero. 
// In:      handle -> Passing the handle got from openMem() function.
// Out:     None
// Return:  1 -> success
//          0 -> failed.
//------------------------------------------------------------
int clearSharedmem(int handle);

#ifdef __cplusplus
}
#endif

#endif // MIDDLEWARE_TO_MEM_MIDDLEWARE_TO_SHAREDMEM_PTR_H_
