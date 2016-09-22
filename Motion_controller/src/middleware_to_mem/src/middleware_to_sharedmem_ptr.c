#include "middleware_to_mem/middleware_to_sharedmem_ptr.h"

//for communication between processes
#define MEM_MAP_SIZE_PROCESS 65536
#define MEM_ADDRESS_PROCESS 0  
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
// Summary: Initialize the mmap shared memory.
//------------------------------------------------------------
int openMem(int type)
{
    int handle = 0;
    char *ptr;

    //to compile in Linux
    #ifndef CPU1_SHAREDMEM

    //to compile for processes communication in Linux
    if(type == MEM_PROCESS)
    {
        int fd = open("/opt/memfile", O_CREAT|O_RDWR, 00777);
        lseek(fd,MEM_MAP_SIZE_PROCESS-1, SEEK_SET);
        write(fd,"",1);
        ptr = (char*) mmap(NULL, MEM_MAP_SIZE_PROCESS, PROT_READ|PROT_WRITE, MAP_SHARED, fd, MEM_ADDRESS_PROCESS);
        if (ptr == MAP_FAILED) 
        {
            close(fd);
            printf("\nError in openMem(): Wrong mmapping for process\n");
            return -1;
        }
        handleTable[type].ptr = ptr;
        handle = MEM_PROCESS;
    }else if(type == MEM_CORE)
    //to compile for cores communication in Linux
    {
        int fd = open("/dev/globalmem_device", O_RDWR);
        ptr = (char*) mmap(NULL, MEM_MAP_SIZE_CORE, PROT_READ|PROT_WRITE, MAP_SHARED, fd, MEM_ADDRESS_CORE);
        if (ptr == MAP_FAILED) 
        {
            close(fd);
            printf("\nError in openMem(): Wrong mmapping for core\n");
            return -1;
        }
        handleTable[type].ptr = ptr;
        handle = MEM_CORE;
    }else
    { 
        printf("\nError in openMem(): Please input 'MEM_PROCESS' or 'MEM_CORE'\n");
        return -1;
    }

    //to compile in bare-metal core 
    #else  
    if(type == MEM_BARE)
    { 
        handleTable[type].ptr = (char*)MEM_ADDRESS;
        handle = MEM_BARE;
    }else
    { 
        return -1;
    }
    #endif
    return handle;
}

//------------------------------------------------------------
// Function:  getPtrOfMem
// Summary: Get the ptr of mem. 
//------------------------------------------------------------
char* getPtrOfMem(const int handle)
{
    if((handle < 0 )||(handle >= HANDLE_TABLE_LEN))
    {
        printf("\nError in getPtrOfMem(): Wrong handle!\n");
        return NULL;
    }
    return handleTable[handle].ptr;
}

//------------------------------------------------------------
// Function:  clearSharedmem
// Summary: Set the shared memory to zero. 
//------------------------------------------------------------
int clearSharedmem(const int handle)
{
    if((handle < 0 )||(handle >= HANDLE_TABLE_LEN))
    {
        printf("\nError in clearSharedmem(): Wrong handle!\n");
        return 0;
    }
    char *ptr = getPtrOfMem(handle);
    if(NULL == ptr) return 0;
    int size = handleTable[handle].map_size;
  
    memset(ptr, 0, size);
    return 1;
}


