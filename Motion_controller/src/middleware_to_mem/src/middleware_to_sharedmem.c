#include "middleware_to_mem/middleware_to_sharedmem.h"

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
int searchIndex(const int handle, const char *name)
{ 
    int i;
    int result = -1; 
    if ((handle < 0 )||(handle >= HANDLE_TABLE_LEN))
    {
        printf("\nError in SearchIndex(): Bad mapping or wrong handle!\n");
        return result;
    }
 
    for (i = 0; i < handleTable[handle].table_length; i++)
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
int readWriteSharedMemByIndex(const int handle, void *structure, int index, int access_type)
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
    } else if (access == MEM_READ && *(ptr_read) == MEM_FALSE && *(ptr_write)  ==  MEM_FALSE)
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
    } else
    {
        return 0;
    } 
    return 1;

}

//------------------------------------------------------------
// Function:  readWriteSharedMem
// Summary: Main API to read and write the data. 
// In:      handle -> passing the handle got from openMem() function.
//          *structure -> the variable data which will be passed through 
//                        the memory area.
//          *name -> the structure name of the variable
//          access_type -> MEM_WRITE/MEM_READ
// Out:     None
// Return:  1 -> success.
//          0 -> failed.
//------------------------------------------------------------
int readWriteSharedMem(const int handle, void *structure, const char *name, int access_type)
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
//          req_id -> every service has an id.
//          *req_buff -> the parameters come with the service id.
//          req_size -> the size of the *req_buff.
// Out:     None
// Return:  1 -> success.
//          0 -> failed.
//------------------------------------------------------------
int clientSendRequest(const int handle, int req_id, char *req_buff, int req_size)
{
    if ((handle < 0 ) || (handle >= HANDLE_TABLE_LEN))
    {
        printf("\nError in clientSendRequest(): Bad mapping or wrong handle!\n");
        return 0;
    }
    if (req_id < 0)
    {
        printf("\nError in clientSendRequest(): Wrong request service ID!\n");
        return 0;
    }
    if ((req_size > REQ_BUFF_LEN) || (req_size < 0)) 
    {
        printf("\nError in clientSendRequest(): Beyond request buff size!\n");
        return 0;
    }

    //write the request data to the shared memory.
    ServiceRequest service_request;
    service_request.req_id = req_id;
    memcpy(service_request.req_buff, req_buff, req_size);

    return readWriteSharedMem(handle, &service_request, "ServiceRequest", MEM_WRITE);;
}

//------------------------------------------------------------
// Function:  clientGetResponse
// Summary: The client gets the service response. 
// In:      handle -> passing the handle got from openMem() function.
// Out:     *res_id -> the service id according to the response.
//          *res_buff -> the parameters come with the response id.
// Return:  1 -> success.
//          0 -> failed.
//------------------------------------------------------------
int clientGetResponse(const int handle, int *res_id, char *res_buff)
{
    if ((handle < 0 ) || (handle >= HANDLE_TABLE_LEN))
    {
        printf("\nError in clientGetResponse(): Bad mapping or wrong handle!\n");
        return 0;
    }
    
    //read the response service data from the shared memory.
    ServiceResponse service_response;
    int read_res = readWriteSharedMem(handle, &service_response, "ServiceResponse", MEM_READ);
    if (read_res == 0) return 0;

    //output.
    *res_id = service_response.res_id;
    memcpy(res_buff, service_response.res_buff, sizeof(service_response.res_buff));
    
    return 1;
}

//------------------------------------------------------------
// Function:  serveruGetRequest
// Summary: The server gets the service request. 
// In:      handle -> passing the handle got from openMem() function.
// Out:     *req_id -> the request service id.
//          *req_buff -> the parameters come with the request service id.
// Return:  1 -> success.
//          0 -> failed.
//------------------------------------------------------------
int serverGetRequest(const int handle, int *req_id, char *req_buff)
{
    if ((handle < 0 ) ||  (handle >= HANDLE_TABLE_LEN))
    {
        printf("\nError in serverGetResponse(): Bad mapping or wrong handle!\n");
        return 0;
    }

    //read the request service data from the shared memory.
    ServiceRequest service_request;
    int read_req = readWriteSharedMem(handle, &service_request, "ServiceRequest", MEM_READ);
    if (read_req == 0) return 0;   

    //output.
    *req_id = service_request.req_id; 
    memcpy(req_buff, &service_request.req_buff, sizeof(service_request.req_buff));

    return 1;
}

//------------------------------------------------------------
// Function:  serverSendResponse
// Summary: The server sends the service response. 
// In:      handle -> passing the handle got from openMem() function.
//          res_id -> the response service id.
//          *res_buff -> the parameters come with the service id.
//          res_size -> the size of the *res_buff.
// Out:     None
// Return:  1 -> success.
//          0 -> failed.
//------------------------------------------------------------
int serverSendResponse(const int handle, int res_id, char *res_buff, int res_size)
{
    if ((handle < 0 ) || (handle >= HANDLE_TABLE_LEN))
    {
        printf("\nError in serverSendResponse(): Bad mapping or wrong handle!\n");
        return 0;
    }
    if (res_id < 0)
    {
        printf("\nError in serverSendResponse(): Wrong response service ID!\n");
        return 0;
    }
    if ((res_size > RES_BUFF_LEN) || (res_size < 0)) 
    {
        printf("\nError in serverSendResponse(): Beyond response buff size!\n");
        return 0;
    }
  
    //Write the response data to the shared memory.
    ServiceResponse service_response;
    service_response.res_id = res_id;
    memcpy(service_response.res_buff, res_buff, res_size);
  
    return readWriteSharedMem(handle, &service_response, "ServiceResponse", MEM_WRITE);;
}




