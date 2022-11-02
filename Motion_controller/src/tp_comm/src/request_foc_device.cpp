#include <pb_encode.h>
#include <pb_decode.h>
#include <pb_common.h>

#include "common_error_code.h"
#include "tp_comm.h"

using namespace base_space;
using namespace user_space;
using namespace log_space;



/********rpc/fio_device/sendFioCmdPack, RequestMessageType_Uint32List(count = 2) **********/
void TpComm::handleRequest0x0000175B(int recv_bytes){
    RequestMessageType_Uint32List *request_data_ptr  = new RequestMessageType_Uint32List;
    if(request_data_ptr == NULL)
    {
        ErrorQueue::instance().push(TP_COMM_MEMORY_OPERATION_FAILED);
        LogProducer::error("comm", "handleRequest: can't allocate memory for request_data\n");
        return;
    }
    ResponseMessageType_Uint32List* response_data_ptr = new ResponseMessageType_Uint32List;
    if(response_data_ptr == NULL)
    {
        ErrorQueue::instance().push(TP_COMM_MEMORY_OPERATION_FAILED);
        LogProducer::error("comm", "handleRequest: can't allocate memory for response_data\n");
        delete request_data_ptr;
        return;
    }
    handleRequestPackage(0x0000175B, (void*)request_data_ptr, (void*)response_data_ptr, recv_bytes, RequestMessageType_Uint32List_fields, -1);
}






