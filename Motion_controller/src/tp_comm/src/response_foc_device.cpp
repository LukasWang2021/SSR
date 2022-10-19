#include "tp_comm.h"

using namespace user_space;
using namespace log_space;
using namespace std;



/********rpc/fio_device/sendFioCmdPack, ResponseMessageType_Uint32List(count=2)**********/
void TpComm::handleResponse0x0000175B(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size){
    if(!encodeResponsePackage(task->hash, ResponseMessageType_Uint32List_fields, task->response_data_ptr, send_buffer_size))
    {
        LogProducer::error("comm", "handleResponse0x0000175B: failed to encode response package");
    }
    if(task->request_data_ptr != NULL)
    {
        delete (RequestMessageType_Uint32List*)task->request_data_ptr;
    }
    if(task->response_data_ptr != NULL)
    {
        delete (ResponseMessageType_Uint32List*)task->response_data_ptr;
    }
}

