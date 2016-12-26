/**********************************************
Copyright © 2016 Foresight-Robotics Ltd. All rights reserved.
File:       middleware_to_sharedmem_ptr.h
Author:     Feng.Wu / Yan.He
Create:     16-Aug-2016
Modify:     03-Nov-2016
Summary:    Init the middleware
**********************************************/
#ifndef MIDDLEWARE_TO_MEM_MIDDLEWARE_TO_SHAREDMEM_PTR_C_
#define MIDDLEWARE_TO_MEM_MIDDLEWARE_TO_SHAREDMEM_PTR_C_

#include "middleware_to_mem/middleware_to_sharedmem_ptr.h"
#include "struct_to_mem/shared_mem_process.h" // for communication between processes 
#include "struct_to_mem/shared_mem_core.h" //for communication between cores (running in linux)

//for communication between processes
#define MEM_MAP_SIZE_PROCESS 65536
//for communication between cores (running in linux)
#define MEM_MAP_SIZE_CORE 65536
#define MEM_ADDRESS_CORE 0x1D100000

HandleTable handleTable[HANDLE_TABLE_LEN] = 
{
    {NULL, tableProcess, MEM_TABLE_PROCESS_LEN, MEM_MAP_SIZE_PROCESS},
    {NULL, tableCore, MEM_TABLE_CORE_LEN, MEM_MAP_SIZE_CORE},
    {NULL, tableCore, MEM_TABLE_CORE_LEN, MEM_MAP_SIZE_CORE},
};

//------------------------------------------------------------
// Function:    openMem
// Summary: There are two shared memory area. One is for processes in linux 
//          when passing MEM_PROCESS. The other memory area is for cores 
//          when passing MEM_CORE or MEM_BARE(used in core1 only).
// In:      type -> indicate which memory area will be open.
//                  parameters are MEM_PROCESS/MEM_CORE/MEM_BARE.
// Out:     None
// Return:  hanlde -> hide the pointer and other info.
//          -1 -> failed to map the memory area.
//------------------------------------------------------------
int openMem(int type)
{
    int handle = 0;
    char *ptr;

    //to compile in Linux
    #ifndef CPU1_SHAREDMEM

    //to compile for processes communication in Linux
    if (type == MEM_PROCESS)
    {
        int fd = shm_open("fst_process_shm", O_CREAT|O_RDWR, 00777);
        if (fd == -1)
        {
            printf("\nError in openMem(): failed on opening sharedmem\n");
            close(fd);
            return -1;
        }
        ftruncate(fd, MEM_MAP_SIZE_PROCESS);
        ptr = (char*) mmap(NULL, MEM_MAP_SIZE_PROCESS, PROT_READ|PROT_WRITE, MAP_SHARED, fd, 0);
        if (ptr == MAP_FAILED) 
        {
            close(fd);
            printf("\nError in openMem(): failed on mapping process sharedmem\n");
            return -1;
        }
        handleTable[type].ptr = ptr;
        handle = MEM_PROCESS;
    }
    else if (type == MEM_CORE)
    //to compile for cores communication in Linux
    {
        int fd = open("/dev/globalmem_device", O_RDWR);
        ptr = (char*) mmap(NULL, MEM_MAP_SIZE_CORE, PROT_READ|PROT_WRITE, MAP_SHARED, fd, MEM_ADDRESS_CORE);
        if (ptr == MAP_FAILED) 
        {
            close(fd);
            printf("\nError in openMem(): failed on mapping core sharedmem\n");
            return -1;
        }
        handleTable[type].ptr = ptr;
        handle = MEM_CORE;
    }
    else
    { 
        printf("\nError in openMem(): Please input 'MEM_PROCESS' or 'MEM_CORE'\n");
        return -1;
    }

    //to compile in bare-metal core 
    #else  
    if (type == MEM_BARE)
    { 
        handleTable[type].ptr = (char*)MEM_ADDRESS;
        handle = MEM_BARE;
    }
    else
    { 
        return -1;
    }
    #endif
    return handle;
}

//------------------------------------------------------------
// Function:  getPtrOfMem
// Summary: Get the pointer according to the handle.
// In:      handle -> Passing the handle got from openMem() function.
// Out:     None
// Return:  pointer -> success
//          null -> failed.
//------------------------------------------------------------
char* getPtrOfMem(int handle)
{
    if ((handle < 0 )||(handle >= HANDLE_TABLE_LEN))
    {
        printf("\nError in getPtrOfMem(): Bad mapping or wrong handle!\n");
        return NULL;
    }
    return handleTable[handle].ptr;
}

//------------------------------------------------------------
// Function:  clearSharedmem
// Summary: Set the shared memory to zero. 
// In:      handle -> Passing the handle got from openMem() function.
// Out:     None
// Return:  1 -> success
//          0 -> failed.
//------------------------------------------------------------
int clearSharedmem(int handle)
{
    if ((handle < 0 )||(handle >= HANDLE_TABLE_LEN))
    {
        printf("\nError in clearSharedmem(): Bad mapping or wrong handle!\n");
        return 0;
    }
    char *ptr = getPtrOfMem(handle);
    if (NULL == ptr) 
        return 0;
    int size = handleTable[handle].map_size;
  
    memset(ptr, 0, size);
    return 1;
}

#endif // MIDDLEWARE_TO_MEM_MIDDLEWARE_TO_SHAREDMEM_PTR_C_
