#include "controller_rpc.h"
#include "error_code.h"

using namespace fst_ctrl;
using namespace fst_hal;


// "/rpc/device_manager/getDeviceList"
void ControllerRpc::handleRpc0x0000C1E0(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_Void* rq_data_ptr = static_cast<RequestMessageType_Void*>(request_data_ptr);
    ResponseMessageType_Uint64_DeviceInfoList* rs_data_ptr = static_cast<ResponseMessageType_Uint64_DeviceInfoList*>(response_data_ptr);
    modbus_manager_ptr_->isModbusValid();
    std::vector<fst_hal::DeviceInfo> device_list = device_manager_ptr_->getDeviceList();
    for(unsigned int i=0; i<device_list.size(); ++i)
    {
        rs_data_ptr->data.device_info[i].index = device_list[i].index;
        rs_data_ptr->data.device_info[i].address = device_list[i].address;
        rs_data_ptr->data.device_info[i].type = device_list[i].type;
        rs_data_ptr->data.device_info[i].is_valid = device_list[i].is_valid; 
    }
    rs_data_ptr->error_code.data = SUCCESS;
    rs_data_ptr->data.device_info_count = device_list.size();

    //if (rs_data_ptr->error_code.data != SUCCESS)
        recordLog(DEVICE_MANAGER_LOG, rs_data_ptr->error_code.data, std::string("/rpc/device_manager/getDeviceList"));
}

// "/rpc/device_manager/get_FRP8A_IoDeviceInfo"
void ControllerRpc::handleRpc0x00006BAF(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_Int32* rq_data_ptr = static_cast<RequestMessageType_Int32*>(request_data_ptr);
    ResponseMessageType_Uint64_IoDeviceInfo* rs_data_ptr = static_cast<ResponseMessageType_Uint64_IoDeviceInfo*>(response_data_ptr);

    std::vector<fst_hal::DeviceInfo> device_list = device_manager_ptr_->getDeviceList();

    for (unsigned int i = 0; i < device_list.size(); ++i)
    {
        if (device_list[i].index == rq_data_ptr->data.data && device_list[i].type == DEVICE_TYPE_FST_IO)
        {
            fst_hal::IODeviceInfo info;
            rs_data_ptr->error_code.data = io_manager_ptr_->getIODeviceInfo(device_list[i].address, info);
            if(rs_data_ptr->error_code.data == SUCCESS)
            {
                strcpy(rs_data_ptr->data.device_type, info.device_type.c_str());
                strcpy(rs_data_ptr->data.comm_type, info.comm_type.c_str());
                rs_data_ptr->data.device_index = rq_data_ptr->data.data;
                rs_data_ptr->data.address = info.address;
                rs_data_ptr->data.input_num = info.DI_num;
                rs_data_ptr->data.output_num = info.DO_num;
                rs_data_ptr->data.is_valid = info.is_valid;
            }else
            {
                memset(&rs_data_ptr->data, 0, sizeof(MessageType_IoDeviceInfo));
            }

            break;
        }
        else
        {
            memset(&rs_data_ptr->data, 0, sizeof(MessageType_IoDeviceInfo));
        }
        rs_data_ptr->error_code.data = IO_INVALID_PARAM_ID;
    }
    
    //if (rs_data_ptr->error_code.data != SUCCESS)
        recordLog(DEVICE_MANAGER_LOG, rs_data_ptr->error_code.data, std::string("/rpc/device_manager/get_FRP8A_IoDeviceInfo"));
}

// "/rpc/device_manager/getModbusIoDeviceInfo"
void ControllerRpc::handleRpc0x0001421F(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_Void* rq_data_ptr = static_cast<RequestMessageType_Void*>(request_data_ptr);
    ResponseMessageType_Uint64_IoDeviceInfo* rs_data_ptr = static_cast<ResponseMessageType_Uint64_IoDeviceInfo*>(response_data_ptr);

    std::vector<fst_hal::DeviceInfo> device_list = device_manager_ptr_->getDeviceList();

    for (unsigned int i = 0; i < device_list.size(); ++i)
    {
        if (device_list[i].type == DEVICE_TYPE_MODBUS)
        {
            fst_hal::IODeviceInfo info;
            rs_data_ptr->error_code.data = io_manager_ptr_->getModbusDeviceInfo(info, modbus_manager_ptr_);
            if(rs_data_ptr->error_code.data == SUCCESS)
            {
                strcpy(rs_data_ptr->data.device_type, info.device_type.c_str());
                strcpy(rs_data_ptr->data.comm_type, info.comm_type.c_str());
                rs_data_ptr->data.device_index = 0;
                rs_data_ptr->data.address = info.address;
                rs_data_ptr->data.input_num = info.DI_num;
                rs_data_ptr->data.output_num = info.DO_num;
                rs_data_ptr->data.is_valid = info.is_valid;
            }
            else
            {
                memset(&rs_data_ptr->data, 0, sizeof(MessageType_IoDeviceInfo));
            }

            break;
        }
        else
        {
            memset(&rs_data_ptr->data, 0, sizeof(MessageType_IoDeviceInfo));
        }
        rs_data_ptr->error_code.data = IO_INVALID_PARAM_ID;
    }

    //if (rs_data_ptr->error_code.data != SUCCESS)
        recordLog(DEVICE_MANAGER_LOG, rs_data_ptr->error_code.data, std::string("/rpc/device_manager/get_FRP8A_IoDeviceInfo"));
}

//"/rpc/device_manager/getIoDeviceInfoList"	
void ControllerRpc::handleRpc0x000024A4(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_Void* rq_data_ptr = static_cast<RequestMessageType_Void*>(request_data_ptr);
    ResponseMessageType_Uint64_IoDeviceInfoList* rs_data_ptr = static_cast<ResponseMessageType_Uint64_IoDeviceInfoList*>(response_data_ptr);
    
    modbus_manager_ptr_->isModbusValid();
    // establish the map btween address to index of device.xml
    std::vector<fst_hal::DeviceInfo> device_list = device_manager_ptr_->getDeviceList();
    std::map<int, int> address_to_index;
    for (int i = 0; i < device_list.size(); ++i)
    {
        address_to_index[device_list[i].address] = device_list[i].index;
    }

    // get all the info of io device.
    std::vector<fst_hal::IODeviceInfo> io_device_list = io_manager_ptr_->getIODeviceInfoList();
    rs_data_ptr->data.io_device_info_count = io_device_list.size();

    for(int i = 0; i < rs_data_ptr->data.io_device_info_count; ++i)
    {      
        strcpy(rs_data_ptr->data.io_device_info[i].device_type, io_device_list[i].device_type.c_str());
        strcpy(rs_data_ptr->data.io_device_info[i].comm_type, io_device_list[i].comm_type.c_str());
        rs_data_ptr->data.io_device_info[i].device_index = address_to_index[io_device_list[i].address];
        rs_data_ptr->data.io_device_info[i].address = io_device_list[i].address;
        rs_data_ptr->data.io_device_info[i].input_num = io_device_list[i].DI_num;
        rs_data_ptr->data.io_device_info[i].output_num = io_device_list[i].DO_num;
        rs_data_ptr->data.io_device_info[i].is_valid = io_device_list[i].is_valid;
    }

    rs_data_ptr->error_code.data = SUCCESS;

    //if (rs_data_ptr->error_code.data != SUCCESS)
        recordLog(DEVICE_MANAGER_LOG, rs_data_ptr->error_code.data, std::string("/rpc/device_manager/getIoDeviceInfoList"));
}

//"/rpc/device_manager/getDeviceVersionList"
void ControllerRpc::handleRpc0x0000F574(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_Void* rq_data_ptr = static_cast<RequestMessageType_Void*>(request_data_ptr);
    ResponseMessageType_Uint64_DeviceVersionList* rs_data_ptr = static_cast<ResponseMessageType_Uint64_DeviceVersionList*>(response_data_ptr);
    
    std::map<std::string, std::string> version_list = device_version_.getDeviceVersionList();
    rs_data_ptr->data.device_version_count = version_list.size();
    int count = 0;
    for(std::map<std::string, std::string>::iterator iter = version_list.begin();iter != version_list.end(); ++iter)
    {
        strcpy(rs_data_ptr->data.device_version[count].name, iter->first.c_str());
        strcpy(rs_data_ptr->data.device_version[count].version, iter->second.c_str());
        ++count;
    }

    rs_data_ptr->error_code.data = SUCCESS;

    //if (rs_data_ptr->error_code.data != SUCCESS)
        recordLog(DEVICE_MANAGER_LOG, rs_data_ptr->error_code.data, std::string("/rpc/device_manager/getDeviceVersionList"));
}

