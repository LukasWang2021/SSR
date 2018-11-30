#include "controller_rpc.h"
#include "error_code.h"

using namespace fst_ctrl;
using namespace fst_hal;


// "/rpc/device_manager/getDeviceList"
void ControllerRpc::handleRpc0x0000C1E0(void* request_data_ptr, void* response_data_ptr)
{
    ResponseMessageType_Uint64_DeviceInfoList* rs_data_ptr = static_cast<ResponseMessageType_Uint64_DeviceInfoList*>(response_data_ptr);
    
    std::vector<fst_hal::DeviceInfo> device_list = device_manager_ptr_->getDeviceList();
    for(unsigned int i=0; i<device_list.size(); ++i)
    {
        rs_data_ptr->data.device_info[i].index = device_list[i].index;
        rs_data_ptr->data.device_info[i].address = device_list[i].address;
        rs_data_ptr->data.device_info[i].type = device_list[i].type;
        //rs_data_ptr->data.device_info[i].is_valid = device_list[i].is_valid;
        rs_data_ptr->data.device_info[i].is_valid = true;//feng add
    }
    rs_data_ptr->error_code.data = SUCCESS;
    rs_data_ptr->data.device_info_count = device_list.size();
    recordLog(DEVICE_MANAGER_LOG, rs_data_ptr->error_code.data, std::string("/rpc/device_manager/getDeviceList"));
}

// "/rpc/device_manager/get_FRP8A_IoDeviceInfo"
void ControllerRpc::handleRpc0x00006BAF(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_Int32* rq_data_ptr = static_cast<RequestMessageType_Int32*>(request_data_ptr);
    ResponseMessageType_Uint64_IoDeviceInfo* rs_data_ptr = static_cast<ResponseMessageType_Uint64_IoDeviceInfo*>(response_data_ptr);

    std::vector<fst_hal::DeviceInfo> device_list = device_manager_ptr_->getDeviceList();

    int address = -1;
    for (unsigned int i = 0; i < device_list.size(); ++i)
    {
        if (device_list[i].index == rq_data_ptr->data.data)
        {
            address = device_list[i].address;
            break;
        }
    }

    fst_hal::IODeviceInfo info;
    rs_data_ptr->error_code.data = io_device_ptr_->getDevInfoByID(address, info);
    if(rs_data_ptr->error_code.data == SUCCESS)
    {
        strcpy(rs_data_ptr->data.device_type, info.device_type.c_str());
        strcpy(rs_data_ptr->data.comm_type, info.comm_type.c_str());
        rs_data_ptr->data.device_index = rq_data_ptr->data.data;
        rs_data_ptr->data.address = address;
        rs_data_ptr->data.input_num = info.DI_num;
        rs_data_ptr->data.output_num = info.DO_num;
        rs_data_ptr->data.is_valid = info.is_valid;//feng add
    }else
    {
        memset(&rs_data_ptr->data, 0, sizeof(MessageType_IoDeviceInfo));
    }

    recordLog(DEVICE_MANAGER_LOG, rs_data_ptr->error_code.data, std::string("/rpc/device_manager/get_FRP8A_IoDeviceInfo"));

}

// "/rpc/device_manager/getModbusIoDeviceInfo"
void ControllerRpc::handleRpc0x0000215F(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_Void* rq_data_ptr = static_cast<RequestMessageType_Void*>(request_data_ptr);
    ResponseMessageType_Uint64_IoDeviceInfo* rs_data_ptr = static_cast<ResponseMessageType_Uint64_IoDeviceInfo*>(response_data_ptr);

    fst_hal::ServerInfo info;
    strcpy(rs_data_ptr->data.device_type, "modbus");
    strcpy(rs_data_ptr->data.comm_type, "TCP");
    rs_data_ptr->data.device_index = 0;
    rs_data_ptr->data.address = 0;
    rs_data_ptr->data.is_valid = modbus_manager_ptr_->isValid();

    rs_data_ptr->error_code.data = modbus_manager_ptr_->getServerInfo(info);
    if (rs_data_ptr->error_code.data != SUCCESS)
    {
        rs_data_ptr->data.input_num = 0;
        rs_data_ptr->data.output_num = 0;
        recordLog(DEVICE_MANAGER_LOG, rs_data_ptr->error_code.data, std::string("/rpc/device_manager/get_FRP8A_IoDeviceInfo"));
        return;
    }

    rs_data_ptr->data.input_num = info.discrepte_input.max_nb;
    rs_data_ptr->data.output_num = info.coil.max_nb;

    recordLog(DEVICE_MANAGER_LOG, rs_data_ptr->error_code.data, std::string("/rpc/device_manager/get_FRP8A_IoDeviceInfo"));
}

