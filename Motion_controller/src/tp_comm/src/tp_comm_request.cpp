#include "tp_comm_component.h"
#include "common/common.h"


using namespace std;


// "/tp_comm/test_request"
void TpCommComponent::handleRequest0xcf0be243(int recv_bytes)
{
    // create object for request and response package
    comm_RequestMessageTypeInt32* request_data_ptr = new comm_RequestMessageTypeInt32;
    if(request_data_ptr == NULL)
    {
        FST_ERROR("handleRequest: can't allocate memory for request_data");
        return;
    }
    comm_ResponseMessageTypeInt32* response_data_ptr = new comm_ResponseMessageTypeInt32;
    if(response_data_ptr == NULL)
    {
        FST_ERROR("handleRequest: can't allocate memory for response_data");
        delete request_data_ptr;
        return;
    }
    
    handleRequestPackage(0xcf0be243, (void*)request_data_ptr, (void*)response_data_ptr, recv_bytes, 
                                comm_RequestMessageTypeInt32_fields, comm_Authority_TP, -1);
}

void TpCommComponent::handleRequest0x0000F933(int recv_bytes) {}
void TpCommComponent::handleRequest0x0000E9D3(int recv_bytes) {}
void TpCommComponent::handleRequest0x00000AB3(int recv_bytes) {}
void TpCommComponent::handleRequest0x00010945(int recv_bytes) {}
void TpCommComponent::handleRequest0x000067A4(int recv_bytes) {}
void TpCommComponent::handleRequest0x00010685(int recv_bytes) {}
void TpCommComponent::handleRequest0x00010AB3(int recv_bytes) {}
void TpCommComponent::handleRequest0x00017C25(int recv_bytes) {}
void TpCommComponent::handleRequest0x000093EE(int recv_bytes) {}

