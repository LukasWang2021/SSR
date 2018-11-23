#include "modbus_client_param.h"
#include "common_file_path.h"
#include "common_log.h"
#include <string>
#include <iostream>
using namespace fst_hal;
using namespace std;

ModbusClientParam::ModbusClientParam(string file_path):
    file_path_(file_path),
    log_level_(fst_log::MSG_LEVEL_ERROR)
{
    ip_ = "192.168.1.102";
    port_ = 502;
    comm_type_ = "TCP";
    is_debug_ = true;
    bytes_timeout_.tv_sec = 0;
    bytes_timeout_.tv_usec = 250000;
    response_timeout_.tv_sec = 1;
    response_timeout_.tv_usec = 0;
}

bool ModbusClientParam::loadParam()
{
    double bytes_timeout_tv_sec = 0;
    double bytes_timeout_tv_usec = 0;
    double response_timeout_tv_sec = 0;
    double response_timeout_tv_usec = 0;

    if (!yaml_help_.loadParamFile(file_path_.c_str())
        || !yaml_help_.getParam("log_level", log_level_)
        || !yaml_help_.getParam("ip", ip_)
        || !yaml_help_.getParam("port", port_)
        || !yaml_help_.getParam("comm_type", comm_type_)
        || !yaml_help_.getParam("is_debug", is_debug_)
        || !yaml_help_.getParam("bytes_timeout/tv_sec", bytes_timeout_tv_sec)
        || !yaml_help_.getParam("bytes_timeout/tv_usec", bytes_timeout_tv_usec)
        || !yaml_help_.getParam("response_timeout/tv_sec", response_timeout_tv_sec)
        || !yaml_help_.getParam("response_timeout/tv_usec", response_timeout_tv_usec))
    {
        cout << " Failed load modbus tcp_client.yaml " << endl;
        return false;
    }

    bytes_timeout_.tv_sec = static_cast<time_t>(bytes_timeout_tv_sec);
    bytes_timeout_.tv_usec = static_cast<suseconds_t>(bytes_timeout_tv_usec);
    response_timeout_.tv_sec = static_cast<time_t>(response_timeout_tv_sec);
    response_timeout_.tv_usec = static_cast<suseconds_t>(response_timeout_tv_usec);

    return true;
}

bool ModbusClientParam::saveParam()
{
    double bytes_timeout_tv_sec = static_cast<double>(bytes_timeout_.tv_sec);
    double bytes_timeout_tv_usec = static_cast<double>(bytes_timeout_.tv_usec);
    double response_timeout_tv_sec = static_cast<double>(response_timeout_.tv_sec);
    double response_timeout_tv_usec = static_cast<double>(response_timeout_.tv_usec);

    if (!yaml_help_.setParam("ip", ip_)
        || !yaml_help_.setParam("log_level", log_level_)
        || !yaml_help_.setParam("port", port_)
        || !yaml_help_.setParam("comm_type", comm_type_)
        || !yaml_help_.setParam("is_debug", is_debug_)
        || !yaml_help_.setParam("bytes_timeout/tv_sec", bytes_timeout_tv_sec)
        || !yaml_help_.setParam("bytes_timeout/tv_usec", bytes_timeout_tv_usec)
        || !yaml_help_.setParam("response_timeout/tv_sec", response_timeout_tv_sec)
        || !yaml_help_.setParam("response_timeout/tv_usec", response_timeout_tv_usec)
        || !yaml_help_.dumpParamFile(file_path_.c_str()))
    {
        cout << " Failed save modbus tcp_client.yaml " << endl;
        return false;
    }

    return true;
}

bool ModbusClientParam::saveIp()
{
    if (!yaml_help_.setParam("ip", ip_)
        || !yaml_help_.dumpParamFile())
    {
        cout << " Failed save ip param to tcp_client.yaml " << endl;
        return false;
    }

    return true;
}


bool ModbusClientParam::savePort()
{
    if (!yaml_help_.setParam("port", port_)
        || !yaml_help_.dumpParamFile(file_path_.c_str()))
    {
        cout << " Failed save modbus tcp_client.yaml " << endl;
        return false;
    }

    return true;
}

bool ModbusClientParam::saveResponseTimeoutParam()
{
    double response_timeout_tv_sec = static_cast<double>(response_timeout_.tv_sec);
    double response_timeout_tv_usec = static_cast<double>(response_timeout_.tv_usec);

    if (!yaml_help_.setParam("response_timeout/tv_sec", response_timeout_tv_sec)
        || !yaml_help_.setParam("response_timeout/tv_usec", response_timeout_tv_usec)
        )
    {
        cout << " Failed save param to tcp_client.yaml " << endl;
        return false;
    }

    if (!yaml_help_.dumpParamFile(file_path_))
    {
        cout << " Failed dump tcp_client.yaml " << endl;
        return false;
    }

    return true;
}

bool ModbusClientParam::saveBytesTimeoutParam()
{
    double bytes_timeout_tv_sec = static_cast<double>(bytes_timeout_.tv_sec);
    double bytes_timeout_tv_usec = static_cast<double>(bytes_timeout_.tv_usec);

    if (!yaml_help_.setParam("bytes_timeout/tv_sec", bytes_timeout_tv_sec)
        || !yaml_help_.setParam("bytes_timeout/tv_usec", bytes_timeout_tv_usec)
        || !yaml_help_.dumpParamFile(file_path_.c_str()))
    {
        cout << " Failed save bytes to tcp_client.yaml " << endl;
        return false;
    }

    return true;
}
