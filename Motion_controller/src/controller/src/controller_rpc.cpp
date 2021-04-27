#include "controller_rpc.h"
#include <sstream>
#include <string>

using namespace user_space;
using namespace base_space;
using namespace std;


ControllerRpc::ControllerRpc():
    tp_comm_ptr_(NULL),
    publish_ptr_(NULL),
    cpu_comm_ptr_(NULL),
    servo_comm_ptr_(NULL),
    file_manager_ptr_(NULL),
    sync_ack_ptr_(NULL)
{
    for(size_t i = 0; i < AXIS_NUM; ++i)
    {
        axis_ptr_[i] = NULL;
    }
    for(size_t i = 0; i < GROUP_NUM; ++i)
    {
        group_ptr_[i] = NULL;
    }

}

ControllerRpc::~ControllerRpc()
{

}

void ControllerRpc::init(TpComm* tp_comm_ptr, ControllerPublish* publish_ptr, servo_comm_space::ServoCpuCommBase* cpu_comm_ptr,
        servo_comm_space::ServoCommBase* servo_comm_ptr[], axis_space::Axis* axis_ptr[AXIS_NUM],
        system_model_space::AxisModel_t* axis_model_ptr[AXIS_NUM], group_space::MotionControl* group_ptr[GROUP_NUM],
        base_space::FileManager* file_manager_ptr, hal_space::Io1000* io_dev_ptr)
{
    tp_comm_ptr_ = tp_comm_ptr;
	publish_ptr_ = publish_ptr;
    cpu_comm_ptr_ = cpu_comm_ptr;
    servo_comm_ptr_ = servo_comm_ptr;
    for(size_t i = 0; i < AXIS_NUM; ++i)
    {
        axis_ptr_[i] = axis_ptr[i];
        axis_model_ptr_[i] = axis_model_ptr[i];
    }
    for(size_t i = 0; i < GROUP_NUM; ++i)
    {
        group_ptr_[i] = group_ptr[i];
    }

    file_manager_ptr_ = file_manager_ptr;
    io_dev_ptr_ = io_dev_ptr;

    device_version_.init();

    initRpcTable();
    initRpcQuickSearchTable();
}

void ControllerRpc::processRpc()
{
    HandleRpcFuncPtr func_ptr;
    TpRequestResponse task;
    if(tp_comm_ptr_->popTaskFromRequestList(&task))
    {
        if(tp_comm_ptr_->getResponseSucceed(task.response_data_ptr) == 0)
        {
            func_ptr = getRpcHandlerByHash(task.hash);
            if(func_ptr != NULL)
            {
                (this->*func_ptr)(task.request_data_ptr, task.response_data_ptr);
            }
        }
        tp_comm_ptr_->pushTaskToResponseList(task);
    }
}

void ControllerRpc::initRpcQuickSearchTable()
{
    unsigned int remainder;
    for(unsigned int i = 0; i < rpc_table_.size(); ++i)
    {
        remainder = rpc_table_[i].hash % QUICK_SEARCH_TABLE_SIZE;
        rpc_quick_search_table_[remainder].push_back(rpc_table_[i]);
    }
}

ControllerRpc::HandleRpcFuncPtr ControllerRpc::getRpcHandlerByHash(unsigned int hash)
{
    unsigned int remainder = hash % QUICK_SEARCH_TABLE_SIZE;
    for(unsigned int i = 0; i < rpc_quick_search_table_[remainder].size(); ++i)
    {
        if(rpc_quick_search_table_[remainder][i].hash == hash)
        {
            return rpc_quick_search_table_[remainder][i].rpc_func_ptr;
        }
    }
    return NULL;
}

void ControllerRpc::recordLog(ErrorCode log_code, ErrorCode error_code, std::string rpc_path)
{
    std::stringstream stream;
    std::string log_str("run ");
    log_str += rpc_path;
    if(error_code == SUCCESS)
    {
        log_str += " success";
        stream<<"Log_Code: 0x"<<std::hex<<log_code<<" : "<<log_str;
        std::cout<<stream.str().c_str()<<std::endl;
    }
    else
    {
        log_str += " failed";
        stream<<"Log_Code: 0x"<<std::hex<<error_code<<" : "<<log_str;
        std::cout<<stream.str().c_str()<<std::endl;
    }    
}

