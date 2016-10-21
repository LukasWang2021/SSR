/**********************************************
File: middleware_to_sharedmem.h
Copyright © 2016 Foresight-Robotics Ltd. All rights reserved.
Instruction: Main algorithm to operate on RAM
Author: Feng.Wu 16-Aug-2016
Modifier:
**********************************************/
#ifndef MIDDLEWARE_TO_MEM_MIDDLEWARE_TO_SHAREDMEM_H_
#define MIDDLEWARE_TO_MEM_MIDDLEWARE_TO_SHAREDMEM_H_

#include "middleware_to_sharedmem_ptr.h"
#include "struct_to_mem/struct_service_request.h"
#include "struct_to_mem/struct_service_response.h"

#define MEM_WRITE 0
#define MEM_READ 1

#ifdef __cplusplus
	extern "C"{
#endif

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
int searchIndex(int handle, const char *name);

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
int readWriteSharedMemByIndex(int handle, void *structure, int index, int access_type);

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
int readWriteSharedMem(int handle, void *structure, const char *name, int access_type);

//------------------------------------------------------------
// Function:  clientSendRequest
// Summary: The client sends the service request. 
// In:      handle -> passing the handle got from openMem() function.
//          *service_request 
// Out:     None
// Return:  1 -> success.
//          0 -> failed.
//------------------------------------------------------------
int clientSendRequest(int handle, ServiceRequest *service_request);

//------------------------------------------------------------
// Function:  clientGetResponse
// Summary: The client gets the service response. 
// In:      handle -> passing the handle got from openMem() function.
// Out:     *service_response 
// Return:  1 -> success.
//          0 -> failed.
//------------------------------------------------------------
int clientGetResponse(int handle, ServiceResponse *service_response);

//------------------------------------------------------------
// Function:  serverGetRequest
// Summary: The server gets the service request. 
// In:      handle -> passing the handle got from openMem() function.
// Out:     *service_request
// Return:  1 -> success.
//          0 -> failed.
//------------------------------------------------------------
int serverGetRequest(int handle, ServiceRequest *service_request);

//------------------------------------------------------------
// Function:  serverSendResponse
// Summary: The server sends the service response. 
// In:      handle -> passing the handle got from openMem() function.
//          *service_response
// Out:     None
// Return:  1 -> success.
//          0 -> failed.
//------------------------------------------------------------
int serverSendResponse(int handle, ServiceResponse *service_response);


#ifdef __cplusplus
}
#endif

#endif //MIDDLEWARE_TO_MEM_MIDDLEWARE_TO_SHAREDMEM_H_
