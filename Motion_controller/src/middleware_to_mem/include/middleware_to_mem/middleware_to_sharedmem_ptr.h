#ifndef _MIDDLEWARE_TO_SHAREDMEM_PTR_H_
#define _MIDDLEWARE_TO_SHAREDMEM_PTR_H_

#include <sys/types.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "struct_to_mem/shared_mem_process.h" // for communication between processes 
#include "struct_to_mem/shared_mem_core.h" //for communication between cores (running in linux)

#ifndef CPU1_SHAREDMEM  
#include <sys/mman.h>
#include <unistd.h>
#include <fcntl.h>
#else                      //for compile in bare-metal core
#define printf(arg,...)
#endif

#define MEM_PROCESS 0
#define MEM_CORE 1
#define MEM_BARE 2
#define HANDLE_TABLE_LEN 3
typedef struct
{
    char* ptr;
    const FunctionTable *table;
    const int table_length;
    const int map_size;
}HandleTable;

#ifdef __cplusplus
	extern "C"{
#endif
int openMem(int type);

char* getPtrOfMem(const int handle);

int clearSharedmem(const int handle);


#ifdef __cplusplus
}
#endif

#endif
