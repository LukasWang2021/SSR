/**********************************************
File: service_test.c
Copyright © 2016 Foresight-Robotics Ltd. All rights reserved.
Instruction: test the service functions
Author: Feng.Wu 28-Sep-2016
Modifier:
**********************************************/

#ifndef SERVICE_TEST_C_
#define SERVICE_TEST_C_

#include <string.h>
#include "middleware_to_mem/middleware_to_sharedmem.h"
#include "struct_to_mem/struct_service_request.h"
#include "struct_to_mem/struct_service_response.h"

int VersionInfo(char *req_buff, char *res_buff)
{
    printf("in function VersionInfo(): req_buff: %s \n", req_buff);
    char act_buff[1024] = "helloworld";  
    memcpy(res_buff, act_buff, sizeof(act_buff));
    return 1;
}

int main(int argc, char** argv)
{
    int handle = openMem(MEM_PROCESS);
    if (handle == -1) return 1;
 
    sleep(1);
 
    int req_id = 110;
    char req_buff[] = "my request is version info";

    //1.client send request.
    clientSendRequest(handle, req_id, req_buff, sizeof(req_buff));

    //2.server get the request and print it.
    int server_req_id = 0;
    char server_req_buff[1024] = "";
    serverGetRequest(handle, &server_req_id, server_req_buff);


    //3.call the service function.
    char server_res_buff[1024] = "";
    VersionInfo(server_req_buff, server_res_buff);

    //4.server call the function.
    serverSendResponse(handle, server_req_id, server_res_buff, sizeof(server_res_buff));
     
    //5.client get response.
    int client_res_id = 0;
    char client_res_buff[1024] = "";   
    clientGetResponse(handle, &client_res_id, client_res_buff);
    printf("5.the client_res_id = %d,\nclient_res_buff = %s\n", client_res_id, client_res_buff);

    return 0;
}

#endif

