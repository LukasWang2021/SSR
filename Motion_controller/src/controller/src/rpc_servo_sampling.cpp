#include "controller_rpc.h"

using namespace user_space;
using namespace log_space;

//"/rpc/servo_sampling/setSamplingConfiguration"    
void ControllerRpc::handleRpc0x0000845E(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_Int32_Uint32List* rq_data_ptr = static_cast<RequestMessageType_Int32_Uint32List*>(request_data_ptr);
    ResponseMessageType_Uint64* rs_data_ptr = static_cast<ResponseMessageType_Uint64*>(response_data_ptr);

    if (rq_data_ptr->data2.data_count != 2)
    {
        rs_data_ptr->data.data = RPC_PARAM_INVALID;
        LogProducer::error("rpc", "/rpc/servo_sampling/setSamplingConfiguration input invalid params");
        return;
    }
    int32_t cpu = rq_data_ptr->data1.data;//todo with axis_manager
    uint32_t sampling_interval = rq_data_ptr->data2.data[0];
    uint32_t sampling_max_times = rq_data_ptr->data2.data[1];
    cpu_comm_ptr_->setSamplingInterval(sampling_interval);
    cpu_comm_ptr_->setSamplingMaxTimes(sampling_max_times);
    rs_data_ptr->data.data = SUCCESS;
    LogProducer::info("rpc", "/rpc/servo_sampling/setSamplingConfiguration called cpu(%d) success, setting interval:%u, times:%u", 
        cpu, sampling_interval, sampling_max_times);
}
//"/rpc/servo_sampling/getSamplingConfiguration"    
void ControllerRpc::handleRpc0x000106EE(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_Int32* rq_data_ptr = static_cast<RequestMessageType_Int32*>(request_data_ptr);
    ResponseMessageType_Uint64_Uint32List* rs_data_ptr = static_cast<ResponseMessageType_Uint64_Uint32List*>(response_data_ptr);

    int32_t cpu = rq_data_ptr->data.data;//todo with axis_manager
    rs_data_ptr->data.data_count = 2;
    rs_data_ptr->data.data[0] = cpu_comm_ptr_->getSamplingInterval();
    rs_data_ptr->data.data[1] = cpu_comm_ptr_->getSamplingMaxTimes();
    rs_data_ptr->error_code.data = SUCCESS;
    LogProducer::info("rpc", "/rpc/servo_sampling/getSamplingConfiguration called cpu(%d) success, interval:%u, times:%u", 
        cpu, rs_data_ptr->data.data[0], rs_data_ptr->data.data[1]);
}

//"/rpc/servo_sampling/activateSamplingConfiguration"	
void ControllerRpc::handleRpc0x0000CDDE(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_Int32* rq_data_ptr = static_cast<RequestMessageType_Int32*>(request_data_ptr);
    ResponseMessageType_Uint64* rs_data_ptr = static_cast<ResponseMessageType_Uint64*>(response_data_ptr);

    int32_t cpu = rq_data_ptr->data.data;//todo with axis_manager
    cpu_comm_ptr_->enableSamplingCfg();
    rs_data_ptr->data.data = SUCCESS;
    LogProducer::info("rpc", "/rpc/servo_sampling/activateSamplingConfiguration called cpu(%d) success", cpu);
}

//"/rpc/servo_sampling/setSamplingSync" 
void ControllerRpc::handleRpc0x00003743(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_Int32_Uint32* rq_data_ptr = static_cast<RequestMessageType_Int32_Uint32*>(request_data_ptr);
    ResponseMessageType_Uint64* rs_data_ptr = static_cast<ResponseMessageType_Uint64*>(response_data_ptr);

    int32_t cpu = rq_data_ptr->data1.data;//todo with axis_manager
    uint32_t sampling_sync = rq_data_ptr->data2.data;
    cpu_comm_ptr_->setSamplingSync(sampling_sync);
    rs_data_ptr->data.data = SUCCESS;
    LogProducer::info("rpc", "/rpc/servo_sampling/setSamplingSync called cpu(%d) success, setting sync:%x", 
        cpu, sampling_sync);
}

//"/rpc/servo_sampling/getSamplingSync" 
void ControllerRpc::handleRpc0x00006343(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_Int32* rq_data_ptr = static_cast<RequestMessageType_Int32*>(request_data_ptr);
    ResponseMessageType_Uint64_Uint32* rs_data_ptr = static_cast<ResponseMessageType_Uint64_Uint32*>(response_data_ptr);

    int32_t cpu = rq_data_ptr->data.data;//todo with axis_manager
    rs_data_ptr->data.data = cpu_comm_ptr_->getSamplingSync();
    rs_data_ptr->error_code.data = SUCCESS;
    LogProducer::info("rpc", "/rpc/servo_sampling/getSamplingSync called cpu(%d) success, syc:%x", 
        cpu, rs_data_ptr->data.data);
}

//"/rpc/servo_sampling/setSamplingChannel"  
void ControllerRpc::handleRpc0x0000BACC(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_Int32List* rq_data_ptr = static_cast<RequestMessageType_Int32List*>(request_data_ptr);
    ResponseMessageType_Uint64* rs_data_ptr = static_cast<ResponseMessageType_Uint64*>(response_data_ptr);

    if (rq_data_ptr->data.data_count != 4)
    {
        rs_data_ptr->data.data = RPC_PARAM_INVALID;
        LogProducer::error("rpc", "/rpc/servo_sampling/setSamplingChannel input invalid params");
        return;
    }

    uint32_t cpu = rq_data_ptr->data.data[0];//todo with axis_manager
    uint32_t channel_index = rq_data_ptr->data.data[1];
    uint32_t servo_index = rq_data_ptr->data.data[2];
    uint32_t servo_param_index = rq_data_ptr->data.data[3];
    uint32_t channel_value = servo_param_index + (servo_index << 16);    
    cpu_comm_ptr_->setSamplingChannel(channel_index, channel_value);
    rs_data_ptr->data.data = SUCCESS;
    LogProducer::info("rpc", "/rpc/servo_sampling/setSamplingChannel called cpu(%d) success, setting channel_index=%u, servo_index:%u, servo_param_index:%u, channel_value:%u",
        cpu, channel_index, servo_index, servo_param_index, channel_value);
}

//"/rpc/servo_sampling/getSamplingChannel"  
void ControllerRpc::handleRpc0x0000556C(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_Int32* rq_data_ptr = static_cast<RequestMessageType_Int32*>(request_data_ptr);
    ResponseMessageType_Uint64_Uint32List* rs_data_ptr = static_cast<ResponseMessageType_Uint64_Uint32List*>(response_data_ptr);

    int32_t cpu = rq_data_ptr->data.data;//todo with axis_manager

    std::vector<uint32_t> vec = cpu_comm_ptr_->getSamplingChannel();
    rs_data_ptr->data.data_count = 16;
    for (size_t i = 0; i < vec.size(); ++i)
    {
        rs_data_ptr->data.data[i] = vec[i];
    }
    rs_data_ptr->error_code.data = SUCCESS;
    LogProducer::info("rpc", "/rpc/servo_sampling/getSamplingChannel called cpu(%d) success", cpu);
}

//"/rpc/servo_sampling/saveSamplingBufferData"  
uint8_t g_sampling_data[16*1024*1024];
void ControllerRpc::handleRpc0x00004E41(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_Int32_String* rq_data_ptr = static_cast<RequestMessageType_Int32_String*>(request_data_ptr);
    ResponseMessageType_Uint64* rs_data_ptr = static_cast<ResponseMessageType_Uint64*>(response_data_ptr);

    int32_t cpu = rq_data_ptr->data1.data;//todo with axis_manager
    sampling_file_path = rq_data_ptr->data2.data;

    // check if destination directory is existed.
    std::string dest = rq_data_ptr->data2.data;
    dest = dest.substr(0, dest.rfind('/') + 1);                   
    if (access(dest.c_str(), 0) == -1)
    {
        rs_data_ptr->data.data = RPC_EXECUTE_FAILED;
        LogProducer::error("rpc", "/rpc/servo_sampling/saveSamplingBufferData path not exist: %s", dest.c_str());
        return;
    }

    cpu_comm_ptr_->getSamplingBufferData(&g_sampling_data[0], &sampling_data_byte_size);
   
    if(!save_file_thread_.run(rpcSaveSamplingDataToFileThreadFunc, this, 20))
    {
        rs_data_ptr->data.data = RPC_EXECUTE_FAILED;
        LogProducer::error("rpc", "/rpc/servo_sampling/saveSamplingBufferData start thread failed");
        return;
    }
 
    rs_data_ptr->data.data = SUCCESS;
    LogProducer::info("rpc", "/rpc/servo_sampling/saveSamplingBufferData called cpu(%d) success", cpu);
}

void ControllerRpc::saveSamplingDataToFile()
{
    FILE *fp = fopen(sampling_file_path.c_str(), "w+");
    if (fp == NULL)
    {
        printf("saveSamplingDataToFile open file failed:%s",sampling_file_path.c_str());
        return;
    }
    for(int32_t i = 0; i < sampling_data_byte_size; i += 4)
    {
        fprintf(fp, "%d\n", *(int32_t*)(&g_sampling_data[i]));
    }   
    fflush(fp);
    fclose(fp);
}

void* rpcSaveSamplingDataToFileThreadFunc(void* arg)
{
    ControllerRpc* rpc_ptr = static_cast<ControllerRpc*>(arg);
    rpc_ptr->saveSamplingDataToFile();
    return NULL;
}
