/**********************************************
File: middleware_to_sharedmem.c
Copyright © 2016 Foresight-Robotics Ltd. All rights reserved.
Instruction: Main algorithm to operate on RAM
Author: Feng.Wu/Yan.He 16-Aug-2016
Modifier:
**********************************************/
#ifndef MIDDLEWARE_TO_MEM_MIDDLEWARE_TO_SHAREDMEM_C_
#define MIDDLEWARE_TO_MEM_MIDDLEWARE_TO_SHAREDMEM_C_

#include "middleware_to_mem/middleware_to_sharedmem.h"

#define MEM_FALSE 0
#define MEM_TRUE 1
#define MEM_WRITE_TURN 11
#define MEM_READ_TURN 12
#define MEM_READ_ALREADY 0
#define MEM_WRITE_ALREADY 1

extern HandleTable handleTable[];

//------------------------------------------------------------
// Function:  searchIndex
// Summary: Search the index of function table. The function
//          table records the whole data structure in the memory area.
// In:      handle -> passing the handle got from openMem() function.
//          *name -> the structure name of the variable
// Out:     None
// Return:  result -> the seq of structure in the memory area.
//          -1 -> failed.
//------------------------------------------------------------
int searchIndex(int handle, const char *name)
{ 
    int i;
    int result = -1; 
    if ((handle < 0 )||(handle >= HANDLE_TABLE_LEN))
    {
        printf("\nError in SearchIndex(): Bad mapping or wrong handle!\n");
        return result;
    }
 
    for (i = 0; i < handleTable[handle].table_length; ++i)
    {
        if (strcmp(name, handleTable[handle].table[i].name) == 0)
        {
            result = i;
            break;
        }
    }
    if (result == -1)
    {
        printf("\nErorr in SearchIndex(): The structure name might be wrong!\n");
    }
    return result;
}

//------------------------------------------------------------
// Function:  readWriteSharedMemByIndex
// Summary: Main API to read and write the data by table index. 
// In:      handle -> passing the handle got from openMem() function.
//          *structure -> the variable data which will be passed through 
//                        the memory area.
//          index -> the index number got from searchIndex() function.
//          access_type -> MEM_WRITE/MEM_READ
// Out:     None
// Return:  1 -> success.
//          0 -> failed.
//------------------------------------------------------------
int readWriteSharedMemByIndex(int handle, void *structure, int index, int access_type)
{
    if ((handle < 0 )||(handle >= HANDLE_TABLE_LEN))
    {
        printf("\nError in readWriteSharedMemByIndex(): Bad mapping or wrong handle!\n");
        return 0;
    }
    if ((index < 0)||(index >= handleTable[handle].table_length)) 
    {
        printf("Error in readWriteSharedMemByIndex(): Wrong index!\n");
        return 0;
    }
    if ((access_type != MEM_WRITE) && (access_type != MEM_READ))
    {
        printf("\nError in readWriteSharedMemByIndex(): Wrong access type!\n");
        return 0;
    } 

    //init the table and pointer 
    int access = access_type;  
    const FunctionTable *table = handleTable[handle].table;
    char *ptr = handleTable[handle].ptr;
    if (ptr == NULL){return 0;}

    volatile unsigned int *ptr_read, *ptr_write, *ptr_turn, *ptr_latest;
    AccessFlag *flag = (AccessFlag *)(ptr + table[index].offset_flag);
    ptr_read = &(flag->read);
    ptr_write = &(flag->write);
    ptr_turn = &(flag->turn);
    ptr_latest = &(flag->latest);

    if (table[index].handshake == MEM_HANDSHAKE && access == MEM_WRITE && *(ptr_latest) == MEM_WRITE_ALREADY){return 0;}
    if (table[index].handshake == MEM_HANDSHAKE && access == MEM_READ && *(ptr_latest) == MEM_READ_ALREADY){return 0;}

    if (access == MEM_WRITE && *(ptr_read) == MEM_FALSE && *(ptr_write)  == MEM_FALSE)
    {
        *(ptr_write) = MEM_TRUE;  //Indicating that it is going to be the writing state
        *(ptr_turn) = MEM_WRITE_TURN; //mark that which process is using the memory
        if (*(ptr_read) == MEM_TRUE &&  *(ptr_turn) != MEM_WRITE_TURN )
        {
            *(ptr_write) = MEM_FALSE; 
            return 0;
        } //check if writing process is working on.
        memcpy((ptr+table[index].offset) , structure, table[index].size);
        *(ptr_latest) = MEM_WRITE_ALREADY;
        *(ptr_write) = MEM_FALSE;       
    } 
    else if (access == MEM_READ && *(ptr_read) == MEM_FALSE && *(ptr_write)  ==  MEM_FALSE)
    {
        *(ptr_read) = MEM_TRUE;  //Indicating that it is going to be the reading state
        *(ptr_turn) = MEM_READ_TURN; //mark that which process is using the memory
        if (*(ptr_write) == MEM_TRUE &&  *(ptr_turn) != MEM_READ_TURN )
        {
            *(ptr_read) = MEM_FALSE; 
            return 0;
        } //check if writing process is working on.
        memcpy(structure, (ptr+table[index].offset), table[index].size);
        *(ptr_latest) = MEM_READ_ALREADY;
        *(ptr_read) = MEM_FALSE;
    }
    else
    {
        return 0;
    } 
    return 1;

}

//------------------------------------------------------------
// Function:  readWriteSharedMem
// Summary: Main API to read and write the data by the structure name. 
// In:      handle -> passing the handle got from openMem() function.
//          *structure -> the variable data which will be passed through 
//                        the memory area.
//          *name -> the structure name of the variable
//          access_type -> MEM_WRITE/MEM_READ
// Out:     None
// Return:  1 -> success.
//          0 -> failed.
//------------------------------------------------------------
int readWriteSharedMem(int handle, void *structure, const char *name, int access_type)
{   
    if ((handle < 0 )||(handle >= HANDLE_TABLE_LEN))
    {
        printf("\nError in readWriteSharedMem(): Bad mapping or wrong handle!\n");
        return 0;
    }
    if ((access_type != MEM_WRITE) && (access_type != MEM_READ))
    {
        printf("\nError in readWriteSharedMem(): Wrong access type!\n");
        return 0;
    } 

    int index = searchIndex(handle, name);
    if (index == -1) return 0;

    int result = readWriteSharedMemByIndex(handle, structure, index, access_type);

    return result;
}

//------------------------------------------------------------
// Function:  clientSendRequest
// Summary: The client sends the service request. 
// In:      handle -> passing the handle got from openMem() function.
//          *service_request 
// Out:     None
// Return:  1 -> success.
//          0 -> failed.
//------------------------------------------------------------
int clientSendRequest(int handle, ServiceRequest *service_request)
{
    if ((handle < 0 ) || (handle >= HANDLE_TABLE_LEN))
    {
        printf("\nError in clientSendRequest(): Bad mapping or wrong handle!\n");
        return 0;
    }
    if ((*service_request).req_id < 0)
    {
        printf("\nError in clientSendRequest(): Wrong request service ID!\n");
        return 0;
    }
    return readWriteSharedMem(handle, service_request, "ServiceRequest", MEM_WRITE);;
}

//------------------------------------------------------------
// Function:  clientGetResponse
// Summary: The client gets the service response. 
// In:      handle -> passing the handle got from openMem() function.
// Out:     *service_response 
// Return:  1 -> success.
//          0 -> failed.
//------------------------------------------------------------
int clientGetResponse(int handle, ServiceResponse *service_response)
{
    if ((handle < 0 ) || (handle >= HANDLE_TABLE_LEN))
    {
        printf("\nError in clientGetResponse(): Bad mapping or wrong handle!\n");
        return 0;
    }
    
    return readWriteSharedMem(handle, service_response, "ServiceResponse", MEM_READ);
}

//------------------------------------------------------------
// Function:  serverGetRequest
// Summary: The server gets the service request. 
// In:      handle -> passing the handle got from openMem() function.
// Out:     *service_request
// Return:  1 -> success.
//          0 -> failed.
//------------------------------------------------------------
int serverGetRequest(int handle, ServiceRequest *service_request)
{
    if ((handle < 0 ) ||  (handle >= HANDLE_TABLE_LEN))
    {
        printf("\nError in serverGetResponse(): Bad mapping or wrong handle!\n");
        return 0;
    }

    return readWriteSharedMem(handle, service_request, "ServiceRequest", MEM_READ);
}

//------------------------------------------------------------
// Function:  serverSendResponse
// Summary: The server sends the service response. 
// In:      handle -> passing the handle got from openMem() function.
//          *service_response
// Out:     None
// Return:  1 -> success.
//          0 -> failed.
//------------------------------------------------------------
int serverSendResponse(int handle, ServiceResponse *service_response)
{
    if ((handle < 0 ) || (handle >= HANDLE_TABLE_LEN))
    {
        printf("\nError in serverSendResponse(): Bad mapping or wrong handle!\n");
        return 0;
    }
    if ((*service_response).res_id < 0)
    {
        printf("\nError in serverSendResponse(): Wrong response service ID!\n");
        return 0;
    }
  
    return readWriteSharedMem(handle, service_response, "ServiceResponse", MEM_WRITE);;
}


#endif //MIDDLEWARE_TO_MEM_MIDDLEWARE_TO_SHAREDMEM_C_

