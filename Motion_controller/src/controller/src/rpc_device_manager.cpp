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
        rs_data_ptr->data.device_info[i].is_valid = device_list[i].is_valid;
    }
    rs_data_ptr->error_code.data = SUCCESS;
    rs_data_ptr->data.device_info_count = device_list.size();
    recordLog(DEVICE_MANAGER_LOG, rs_data_ptr->error_code.data, std::string("/rpc/device_manager/getDeviceList"));
}

