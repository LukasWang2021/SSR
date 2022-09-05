#include "controller_rpc.h"

using namespace hal_space;
using namespace user_space;
using namespace log_space;

//"/rpc/fio_device/sendFioCmdPack"	
void ControllerRpc::handleRpc0x0000175B(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_Uint32List* rq_data_ptr = static_cast<RequestMessageType_Uint32List*>(request_data_ptr);
    ResponseMessageType_Uint32List* rs_data_ptr = static_cast<ResponseMessageType_Uint32List*>(response_data_ptr);

    LogProducer::info("rpc","cmd=%d,value=%d", rq_data_ptr->data.data[0],rq_data_ptr->data.data[1]);

    uint32_t cmd_type = static_cast<uint16_t>(rq_data_ptr->data.data[0]);;
    uint32_t cmd_val = rq_data_ptr->data.data[1];
    uint32_t cmd_rpl = 0;

    ErrorCode err = fio_device_ptr_->sendCmdRcvRpl(cmd_type, cmd_val, &cmd_rpl);
    rs_data_ptr->header.error_code = err;
    rs_data_ptr->data.data[0] = cmd_rpl;
    rs_data_ptr->data.data_count = 1;

    if(err) 
        LogProducer::error("rpc","/rpc/fio_device/sendFioCmdPack failed = 0x%llX, reply %d", err);
    else
        LogProducer::info("rpc","/rpc/fio_device/sendFioCmdPack success reply %d", rs_data_ptr->data.data[0]);
}
