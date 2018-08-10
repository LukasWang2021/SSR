#include "process_comm_param.h"
#include "common_file_path.h"
#include "common_log.h"
#include <string>

using namespace fst_base;

ProcessCommParam::ProcessCommParam():
    file_path_(COMPONENT_PARAM_FILE_DIR),
    log_level_(fst_log::MSG_LEVEL_ERROR),
    controller_server_thread_priority_(0),
    controller_server_cycle_time_(0),
    i2c_req_res_ip_(""),
    interpreter_server_thread_priority_(0),
    interpreter_server_cycle_time_(0),
    c2i_req_res_ip_(""),
    c2i_pub_ip_(""),
    c2i_event_ip_(""),
    interpreter_server_event_buffer_size_(0),
    recv_buffer_size_(0),
    send_buffer_size_(0)
    
{
    file_path_ += "process_comm.yaml";
}

ProcessCommParam::~ProcessCommParam()
{

}

bool ProcessCommParam::loadParam()
{    
    if (!yaml_help_.loadParamFile(file_path_.c_str())
        || !yaml_help_.getParam("log_level", log_level_)
        || !yaml_help_.getParam("controller_server_thread_priority", controller_server_thread_priority_)
        || !yaml_help_.getParam("controller_server_cycle_time", controller_server_cycle_time_)
        || !yaml_help_.getParam("i2c_req_res_ip", i2c_req_res_ip_)
        || !yaml_help_.getParam("interpreter_server_thread_priority", interpreter_server_thread_priority_)
        || !yaml_help_.getParam("interpreter_server_cycle_time", interpreter_server_cycle_time_)
        || !yaml_help_.getParam("c2i_req_res_ip", c2i_req_res_ip_)
        || !yaml_help_.getParam("c2i_pub_ip", c2i_pub_ip_)
        || !yaml_help_.getParam("c2i_event_ip", c2i_event_ip_)
        || !yaml_help_.getParam("interpreter_server_event_buffer_size", interpreter_server_event_buffer_size_)
        || !yaml_help_.getParam("recv_buffer_size", recv_buffer_size_)
        || !yaml_help_.getParam("send_buffer_size", send_buffer_size_))
    {
        return false;
    }
    else    
    {
        return true;
    }
}

bool ProcessCommParam::saveParam()
{
    if(!yaml_help_.setParam("log_level", log_level_)
        || !yaml_help_.setParam("controller_server_thread_priority", controller_server_thread_priority_)
        || !yaml_help_.setParam("controller_server_cycle_time", controller_server_cycle_time_)
        || !yaml_help_.setParam("i2c_req_res_ip", i2c_req_res_ip_)
        || !yaml_help_.setParam("interpreter_server_thread_priority", interpreter_server_thread_priority_)
        || !yaml_help_.setParam("interpreter_server_cycle_time", interpreter_server_cycle_time_)
        || !yaml_help_.setParam("c2i_req_res_ip", c2i_req_res_ip_)
        || !yaml_help_.setParam("c2i_pub_ip", c2i_pub_ip_)
        || !yaml_help_.setParam("c2i_event_ip", c2i_event_ip_)
        || !yaml_help_.setParam("interpreter_server_event_buffer_size", interpreter_server_event_buffer_size_)
        || !yaml_help_.setParam("recv_buffer_size", recv_buffer_size_)
        || !yaml_help_.setParam("send_buffer_size", send_buffer_size_)
        || !yaml_help_.dumpParamFile(file_path_.c_str()))
    {
        return false;
    }
    else
    {
        return true;
    }
}

