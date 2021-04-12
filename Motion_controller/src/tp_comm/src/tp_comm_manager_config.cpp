#include "tp_comm_manager_config.h"
#include "common_file_path.h"
#include <string>

using namespace user_space;

TpCommManagerConfig::TpCommManagerConfig():    
    cycle_time_(10000),
    recv_buffer_size_(65536),
    send_buffer_size_(65535),
    rpc_list_max_size_(10),
    event_list_max_size_(10),
    file_path_(COMPONENT_PARAM_FILE_DIR)
{
    file_path_ += "tp_comm.yaml";
}


bool TpCommManagerConfig::loadParam()
{

    if (!yaml_help_.loadParamFile(file_path_.c_str())
        || !yaml_help_.getParam("cycle_time", cycle_time_)
        || !yaml_help_.getParam("recv_buffer_size", recv_buffer_size_)
        || !yaml_help_.getParam("send_buffer_size", send_buffer_size_)
        || !yaml_help_.getParam("rpc_list_max_size", rpc_list_max_size_)
        || !yaml_help_.getParam("event_list_max_size", event_list_max_size_))
    {
        return false;
    }
    else    
    {
        return true;
    }
}

bool TpCommManagerConfig::saveParam()
{
    if(!yaml_help_.setParam("cycle_time", cycle_time_)
        || !yaml_help_.setParam("recv_buffer_size", recv_buffer_size_)
        || !yaml_help_.setParam("send_buffer_size", send_buffer_size_)
        || !yaml_help_.setParam("rpc_list_max_size", rpc_list_max_size_)
        || !yaml_help_.setParam("event_list_max_size", event_list_max_size_)
        || !yaml_help_.dumpParamFile(file_path_.c_str()))
    {
        return false;
    }
    else
    {
        return true;
    }
}
