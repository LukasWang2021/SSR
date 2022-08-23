#include "controller_rpc.h"

using namespace hal_space;
using namespace user_space;
using namespace log_space;

//"/rpc/fio_device/sendFioCmdPack"	
void ControllerRpc::handleRpc0x0000175B(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_Uint32List* rq_data_ptr = static_cast<RequestMessageType_Uint32List*>(request_data_ptr);
    ResponseMessageType_Uint32List* rs_data_ptr = static_cast<ResponseMessageType_Uint32List*>(response_data_ptr);
    int res=0;
    uint32_t rep_dat1, rep_dat2;
    if(rq_data_ptr->data.data_count == 2)
    {
        LogProducer::info("t_fioHw.cmd_regs","cmd=0x%x,value=0x%x", rq_data_ptr->data.data[0],rq_data_ptr->data.data[1]);
        uint32_t t_cmd,t_val;
        t_cmd = static_cast<uint16_t>(rq_data_ptr->data.data[0]);
        t_val = rq_data_ptr->data.data[1];
        res = fio_device_ptr_->FioSendCmdPack(t_cmd, t_val);
        if(res == 0)
        {
            res  = fio_device_ptr_->FioRecvReplyPack(&rep_dat1,&rep_dat2);
            if(res==0)
            {
                rs_data_ptr->data.data_count = 2;
                rs_data_ptr->data.data[0] = rep_dat1;
                rs_data_ptr->data.data[1] = rep_dat2;
                LogProducer::info("rpc", "/rpc/fio_device/sendFioCmdPackt success  dat1=0x%x, dat2=0x%x", rs_data_ptr->data.data[0], rs_data_ptr->data.data[1]);
            }
            else
            {
                LogProducer::warn("handleRpc0x0000175B","FioRecvReplyPack error");
            }
        }
        else
        {
            LogProducer::warn("handleRpc0x0000175B","FioSendCmdPack error");
        }
    }
    else
    {
        rs_data_ptr->data.data_count = 0;
        LogProducer::error("rpc", "/rpc/fio_device/sendFioCmdPack failed.");
    }
}


