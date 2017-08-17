/**********************************************
File: service_test.c
Copyright © 2016 Foresight-Robotics Ltd. All rights reserved.
Instruction: test the service functions
Author: Feng.Wu 10-oct-2016
Modifier:
**********************************************/

#ifndef SERVICE_TEST_C_
#define SERVICE_TEST_C_

#include <string.h>
#include "middleware_to_mem/middleware_to_sharedmem.h"
#include "struct_to_mem/struct_service_request.h"
#include "struct_to_mem/struct_service_response.h"

int VersionInfo(ServiceRequest *service_request, ServiceResponse *service_response)
{
    printf("in function VersionInfo(): req_buff: %s \n", (*service_request).req_buff);
    (*service_response).res_id = (*service_request).req_id; 
    strcpy((*service_response).res_buff, "helloworld");
    return 1;
}

int main(int argc, char** argv)
{
    int handle = openMem(MEM_PROCESS);
    if (handle == -1) return 1;
 
    sleep(1);
 
    ServiceRequest client_service_request = {110, "my request is version info"};

    //1.client send request.
    clientSendRequest(handle, &client_service_request);

/*server action*/
    //2.server get the request and print it.
    ServiceRequest server_service_request;
    serverGetRequest(handle, &server_service_request);

    //3.call the service function.
    ServiceResponse server_service_response;
    VersionInfo(&server_service_request, &server_service_response);

    //4.server call the function.
    serverSendResponse(handle, &server_service_response);
/*end server action*/  
   
    //5.client get response.
    ServiceResponse client_service_response;   
    clientGetResponse(handle, &client_service_response);
    printf("5.the client_res_id = %d,\nclient_res_buff = %s\n", client_service_response.res_id, client_service_response.res_buff);

    return 0;
}

#endif

