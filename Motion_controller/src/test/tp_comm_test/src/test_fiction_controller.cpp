#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <iostream>

#include <nanomsg/nn.h>
#include <nanomsg/reqrep.h>
#include <nanomsg/ws.h>
#include <time.h>
#include <pb_encode.h>
#include <pb_decode.h>
#include <pb_common.h>
#include "protoc.h"
#include <time.h>
#include "tp_comm.h"
#include "common_log.h"

#include "tp_comm_test.h"


#define MAX_REQ_BUFFER_SIZE     (65535)

using namespace std;

int main()
{
    TpComm yy_tp_comm_;
    MessageType_Int32 yy_publish_data1;
    MessageType_Int32 yy_publish_data2;
    std::mutex yy_request_list_mutex_;
    std::mutex yy_response_list_mutex_;
    std::mutex yy_publish_list_mutex_;

    if(!yy_tp_comm_.init()
        || !yy_tp_comm_.open())
    {
        FST_ERROR("failed to open tp_comm");
    }

    while(1)
    {
        std::vector<TpRequestResponse> request_list = yy_tp_comm_.popTaskFromRequestList();
        std::vector<TpRequestResponse>::iterator it;

       for(it = request_list.begin(); it != request_list.end(); ++it)
        {
            switch(it->hash)
            {
                case 0x00000C05: 
                {
                    if(!yy_tp_comm_.getResponseSucceed(it->response_data_ptr))
                    {
                        yy_response_list_mutex_.lock();
                        yy_tp_comm_.pushTaskToResponseList(*it);
                        yy_response_list_mutex_.unlock();
                    }
                    else
                    {
                        RequestMessageType_Void* request_data_ptr = static_cast<RequestMessageType_Void*>(it->request_data_ptr);
                        ResponseMessageType_Int32* response_data_ptr = static_cast<ResponseMessageType_Int32*>(it->response_data_ptr);
                        response_data_ptr->data.data = 1;
                        yy_response_list_mutex_.lock();
                        yy_tp_comm_.pushTaskToResponseList(*it);
                        yy_response_list_mutex_.unlock();
                    }
                }
                    break;
                //"/rpc/controller/getUserOpMode",
                case 0x00000AB3: 
                {
                    if(!yy_tp_comm_.getResponseSucceed(it->response_data_ptr))
                    {
                        yy_response_list_mutex_.lock();
                        yy_tp_comm_.pushTaskToResponseList(*it);
                        yy_response_list_mutex_.unlock();
                    }
                    else
                    {
                        RequestMessageType_Void* request_data_ptr = static_cast<RequestMessageType_Void*>(it->request_data_ptr);
                        ResponseMessageType_Int32* response_data_ptr = static_cast<ResponseMessageType_Int32*>(it->response_data_ptr);
                        response_data_ptr->data.data = 2;
                        yy_response_list_mutex_.lock();
                        yy_tp_comm_.pushTaskToResponseList(*it);
                        yy_response_list_mutex_.unlock();
                    }
                    FST_INFO("Controller : request push over");
                }
                    break;
                //"/rpc/controller/getRunningStatus",	0x00000AB3,
                case 0x00016483:
                {
                    if(!yy_tp_comm_.getResponseSucceed(it->response_data_ptr))
                    {
                        yy_response_list_mutex_.lock();
                        yy_tp_comm_.pushTaskToResponseList(*it);
                        yy_response_list_mutex_.unlock();
                    }
                    else
                    {
                        RequestMessageType_Void* request_data_ptr = static_cast<RequestMessageType_Void*>(it->request_data_ptr);
                        ResponseMessageType_Int32* response_data_ptr = static_cast<ResponseMessageType_Int32*>(it->response_data_ptr);
                        response_data_ptr->data.data = 3;
                        yy_response_list_mutex_.lock();
                        yy_tp_comm_.pushTaskToResponseList(*it);
                        yy_response_list_mutex_.unlock();
                    }
                }
                    break;
                case 0x00006F83: 
                {
                    if(!yy_tp_comm_.getResponseSucceed(it->response_data_ptr))
                    {
                        yy_response_list_mutex_.lock();
                        yy_tp_comm_.pushTaskToResponseList(*it);
                        yy_response_list_mutex_.unlock();
                    }
                    else
                    {
                        RequestMessageType_Void* request_data_ptr = static_cast<RequestMessageType_Void*>(it->request_data_ptr);
                        ResponseMessageType_Int32* response_data_ptr = static_cast<ResponseMessageType_Int32*>(it->response_data_ptr);
                        response_data_ptr->data.data = 4;
                        yy_response_list_mutex_.lock();
                        yy_tp_comm_.pushTaskToResponseList(*it);
                        yy_response_list_mutex_.unlock();
                    }
                }
                    break;
                case 0x0000E9D3: // "/get local time "
                {
                    if(!yy_tp_comm_.getResponseSucceed(it->response_data_ptr))
                    {
                        yy_response_list_mutex_.lock();
                        yy_tp_comm_.pushTaskToResponseList(*it);
                        yy_response_list_mutex_.unlock();
                    }
                    else
                    {
                        RequestMessageType_Void* request_data_ptr = static_cast<RequestMessageType_Void*>(it->request_data_ptr);
                        ResponseMessageType_Int32* response_data_ptr = static_cast<ResponseMessageType_Int32*>(it->response_data_ptr);
                        response_data_ptr->data.data = 5;
                        yy_response_list_mutex_.lock();
                        yy_tp_comm_.pushTaskToResponseList(*it);
                        yy_response_list_mutex_.unlock();
                    }
                }
                    break;
                case 0x0000D113: 
                {
                    if(!yy_tp_comm_.getResponseSucceed(it->response_data_ptr))
                    {
                        yy_response_list_mutex_.lock();
                        yy_tp_comm_.pushTaskToResponseList(*it);
                        yy_response_list_mutex_.unlock();
                    }
                    else
                    {
                        RequestMessageType_Void* request_data_ptr = static_cast<RequestMessageType_Void*>(it->request_data_ptr);
                        ResponseMessageType_Int32* response_data_ptr = static_cast<ResponseMessageType_Int32*>(it->response_data_ptr);
                        response_data_ptr->data.data = 6;
                        yy_response_list_mutex_.lock();
                        yy_tp_comm_.pushTaskToResponseList(*it);
                        yy_response_list_mutex_.unlock();
                    }
                }
                    break;
                case 0x0000C00D: 
                {
                    if(!yy_tp_comm_.getResponseSucceed(it->response_data_ptr))
                    {
                        yy_response_list_mutex_.lock();
                        yy_tp_comm_.pushTaskToResponseList(*it);
                        yy_response_list_mutex_.unlock();
                    }
                    else
                    {
                        RequestMessageType_Void* request_data_ptr = static_cast<RequestMessageType_Void*>(it->request_data_ptr);
                        ResponseMessageType_Int32* response_data_ptr = static_cast<ResponseMessageType_Int32*>(it->response_data_ptr);
                        response_data_ptr->data.data = 7;
                        yy_response_list_mutex_.lock();
                        yy_tp_comm_.pushTaskToResponseList(*it);
                        yy_response_list_mutex_.unlock();
                    }
                }
                    break;
                case 0x00013940: 
                {
                    if(!yy_tp_comm_.getResponseSucceed(it->response_data_ptr))
                    {
                        yy_response_list_mutex_.lock();
                        yy_tp_comm_.pushTaskToResponseList(*it);
                        yy_response_list_mutex_.unlock();
                    }
                    else
                    {
                        RequestMessageType_Void* request_data_ptr = static_cast<RequestMessageType_Void*>(it->request_data_ptr);
                        ResponseMessageType_Bool* response_data_ptr = static_cast<ResponseMessageType_Bool*>(it->response_data_ptr);
                        response_data_ptr->data.data = true;
                        yy_response_list_mutex_.lock();
                        yy_tp_comm_.pushTaskToResponseList(*it);
                        yy_response_list_mutex_.unlock();
                    }
                }
                    break;
                case 0x000161E4: 
                {
                    if(!yy_tp_comm_.getResponseSucceed(it->response_data_ptr))
                    {
                        yy_response_list_mutex_.lock();
                        yy_tp_comm_.pushTaskToResponseList(*it);
                        yy_response_list_mutex_.unlock();
                    }
                    else
                    {
                        RequestMessageType_Void* request_data_ptr = static_cast<RequestMessageType_Void*>(it->request_data_ptr);
                        ResponseMessageType_Bool* response_data_ptr = static_cast<ResponseMessageType_Bool*>(it->response_data_ptr);
                        response_data_ptr->data.data = true;
                        yy_response_list_mutex_.lock();
                        yy_tp_comm_.pushTaskToResponseList(*it);
                        yy_response_list_mutex_.unlock();
                    }
                }
                    break;
                case 0x00002ED5: 
                {
                    if(!yy_tp_comm_.getResponseSucceed(it->response_data_ptr))
                    {
                        yy_response_list_mutex_.lock();
                        yy_tp_comm_.pushTaskToResponseList(*it);
                        yy_response_list_mutex_.unlock();
                    }
                    else
                    {
                        RequestMessageType_Int32* request_data_ptr = static_cast<RequestMessageType_Int32*>(it->request_data_ptr);
                        ResponseMessageType_Bool* response_data_ptr = static_cast<ResponseMessageType_Bool*>(it->response_data_ptr);
                        response_data_ptr->data.data = true;
                        yy_response_list_mutex_.lock();
                        yy_tp_comm_.pushTaskToResponseList(*it);
                        yy_response_list_mutex_.unlock();
                    }
                }
                    break;
                case 0x00000773:
                {
                    if(!yy_tp_comm_.getResponseSucceed(it->response_data_ptr))
                    {
                        yy_response_list_mutex_.lock();
                        yy_tp_comm_.pushTaskToResponseList(*it);
                        yy_response_list_mutex_.unlock();
                    }
                    else
                    {
                        RequestMessageType_Topic* request_data_ptr = static_cast<RequestMessageType_Topic*>(it->request_data_ptr);
                        ResponseMessageType_Bool* response_data_ptr = static_cast<ResponseMessageType_Bool*>(it->response_data_ptr);
                        response_data_ptr->data.data = true;
                        yy_response_list_mutex_.lock();
                        yy_tp_comm_.pushTaskToResponseList(*it);
                        yy_response_list_mutex_.unlock();
                    }
                }
                    break;
                default:
                    ;
            }
        }

    usleep(10000);
    }
}
