#include "tp_comm_manager_param.h"
#include "common_file_path.h"
#include "common_log.h"
#include <string>

using namespace fst_comm;

TpCommManagerParam::TpCommManagerParam():
    file_path_(COMPONENT_PARAM_FILE_DIR),
    log_level_(fst_log::MSG_LEVEL_ERROR),
    cycle_time_(10000),
    recv_buffer_size_(65536),
    send_buffer_size_(65535),
    rpc_list_max_size_(10)
{
    file_path_ += "tp_comm.yaml";
}


bool TpCommManagerParam::loadParam()
{

    if (!yaml_help_.loadParamFile(file_path_.c_str())
        || !yaml_help_.getParam("log_level", log_level_)
        || !yaml_help_.getParam("cycle_time", cycle_time_)
        || !yaml_help_.getParam("recv_buffer_size", recv_buffer_size_)
        || !yaml_help_.getParam("send_buffer_size", send_buffer_size_)
        || !yaml_help_.getParam("rpc_list_max_size", rpc_list_max_size_))
    {
        return false;
    }
    else    
    {
        return true;
    }
}

bool TpCommManagerParam::saveParam()
{
    if(!yaml_help_.setParam("log_level", log_level_)
        || !yaml_help_.setParam("cycle_time", cycle_time_)
        || !yaml_help_.setParam("recv_buffer_size", recv_buffer_size_)
        || !yaml_help_.setParam("send_buffer_size", send_buffer_size_)
        || !yaml_help_.setParam("rpc_list_max_size", rpc_list_max_size_)
        || !yaml_help_.dumpParamFile(file_path_.c_str()))
    {
        return false;
    }
    else
    {
        return true;
    }
}
