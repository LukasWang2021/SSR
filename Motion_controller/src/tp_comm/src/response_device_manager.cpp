#include "tp_comm.h"

using namespace fst_comm;

using namespace std;


//"/rpc/device_manager/getDeviceList"
void TpComm::handleResponse0x0000C1E0(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size)
{
    if(!encodeResponsePackage(task->hash, ResponseMessageType_Uint64_DeviceInfoList_fields, task->response_data_ptr, send_buffer_size))
    {
        FST_ERROR("handleResponseGetUserOpMode: failed to encode response package");// send
    }
    if(task->request_data_ptr != NULL)
    {
        delete (RequestMessageType_Void*)task->request_data_ptr;
    }
    if(task->response_data_ptr != NULL)
    {
        delete (ResponseMessageType_Uint64_DeviceInfoList*)task->response_data_ptr;
    }
}

//"/rpc/device_manager/get_FRP8A_IoDeviceInfo"
void TpComm::handleResponse0x00006BAF(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size)
{
    if(!encodeResponsePackage(task->hash, ResponseMessageType_Uint64_IoDeviceInfo_fields, task->response_data_ptr, send_buffer_size))
    {
        FST_ERROR("handleResponseGetUserOpMode: failed to encode response package");// send
    }
    if(task->request_data_ptr != NULL)
    {
        delete (RequestMessageType_Int32*)task->request_data_ptr;
    }
    if(task->response_data_ptr != NULL)
    {
        delete (ResponseMessageType_Uint64_IoDeviceInfo*)task->response_data_ptr;
    }
}
//"/rpc/device_manager/getModbusIoDeviceInfo"
void TpComm::handleResponse0x0001421F(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size)
{
    if(!encodeResponsePackage(task->hash, ResponseMessageType_Uint64_IoDeviceInfo_fields, task->response_data_ptr, send_buffer_size))
    {
        FST_ERROR("handleResponseGetUserOpMode: failed to encode response package");// send
    }
    if(task->request_data_ptr != NULL)
    {
        delete (RequestMessageType_Void*)task->request_data_ptr;
    }
    if(task->response_data_ptr != NULL)
    {
        delete (ResponseMessageType_Uint64_IoDeviceInfo*)task->response_data_ptr;
    }
}
