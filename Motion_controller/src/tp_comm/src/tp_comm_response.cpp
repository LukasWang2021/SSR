#include "tp_comm_component.h"
#include "common/common.h"


using namespace std;


// "/tp_comm/test_request"
void TpCommComponent::handleResponse0xcf0be243(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size)
{
    if(!encodeResponsePackage(task->hash, comm_ResponseMessageTypeInt32_fields, task->response_data_ptr, send_buffer_size))
    {
        FST_INFO("handleResponse0xcf0be243: failed to encode response package");// send
    }
    if(task->request_data_ptr != NULL)
    {
        delete (comm_RequestMessageTypeInt32*)task->request_data_ptr;
    }
    if(task->response_data_ptr != NULL)
    {
        delete (comm_ResponseMessageTypeInt32*)task->response_data_ptr;
    }
}

void TpCommComponent::handleResponse0x0000F933(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size){}
void TpCommComponent::handleResponse0x0000E9D3(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size){}
void TpCommComponent::handleResponse0x00000AB3(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size){}
void TpCommComponent::handleResponse0x00010945(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size){}
void TpCommComponent::handleResponse0x000067A4(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size){}
void TpCommComponent::handleResponse0x00010685(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size){}
void TpCommComponent::handleResponse0x00010AB3(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size){}
void TpCommComponent::handleResponse0x00017C25(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size){}
void TpCommComponent::handleResponse0x000093EE(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size){}



