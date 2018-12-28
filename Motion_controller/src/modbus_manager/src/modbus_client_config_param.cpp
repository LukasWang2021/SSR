#include "modbus_client_config_param.h"
#include "common_file_path.h"
#include "common_log.h"
#include <string>
#include <iostream>
using namespace fst_hal;
using namespace std;

ModbusClientConfigParam::ModbusClientConfigParam(string &file_path, int client_nb):
    client_nb_(client_nb)
{
    client_config_list_.resize(client_nb_);
    file_path_ = file_path;
}

ModbusClientConfigParam::ModbusClientConfigParam()
{}

bool ModbusClientConfigParam::loadParam()
{
    if (!yaml_help_.loadParamFile(file_path_.c_str()))
    {
        return false;
    }

    std::string id_str;
    std::stringstream stream;
    vector<ModbusClientConfigParams>::iterator it = client_config_list_.begin();

    for(int client_id = 1; client_id != client_config_list_.size() + 1; client_id++)
    {
        cout << "client_id = " << client_id << endl;
        cout << "client_config_list_.size() = " << client_config_list_.size() << endl;

        stream << client_id;
        stream >> id_str;
        string client_path = std::string("ModbusClient") + id_str;

        cout << "client_path = " << client_path << endl;

        if (!yaml_help_.getParam(client_path + "/id", it->start_info.id)
            || !yaml_help_.getParam(client_path + "/name", it->start_info.name)
            || !yaml_help_.getParam(client_path + "/ip", it->start_info.ip)
            || !yaml_help_.getParam(client_path + "/port", it->start_info.port)
            || !yaml_help_.getParam(client_path + "/scan_rate", it->start_info.scan_rate)
            || !yaml_help_.getParam(client_path + "/response_timeout", it->start_info.response_timeout)
            || !yaml_help_.getParam(client_path + "/reg_info/coil/addr", it->reg_info.coil.addr)
            || !yaml_help_.getParam(client_path + "/reg_info/coil/max_nb", it->reg_info.coil.max_nb)
            || !yaml_help_.getParam(client_path + "/reg_info/discrepte_input/addr", it->reg_info.discrepte_input.addr)
            || !yaml_help_.getParam(client_path + "/reg_info/discrepte_input/max_nb", it->reg_info.discrepte_input.max_nb)
            || !yaml_help_.getParam(client_path + "/reg_info/input_register/addr", it->reg_info.input_reg.addr)
            || !yaml_help_.getParam(client_path + "/reg_info/input_register/max_nb", it->reg_info.input_reg.max_nb)
            || !yaml_help_.getParam(client_path + "/reg_info/holding_register/addr", it->reg_info.holding_reg.addr)
            || !yaml_help_.getParam(client_path + "/reg_info/holding_register/max_nb", it->reg_info.holding_reg.max_nb)
            || !yaml_help_.getParam(client_path + "/is_enable", it->is_enable))
        {
            cout << "Failed to load client_config.yaml" << endl;
            return false;
        }

        it++;
    }

    return true;
}

bool ModbusClientConfigParam::saveParam()
{
    vector<ModbusClientConfigParams>::iterator it = client_config_list_.begin();

    int client_id = 1;
    for(;it != client_config_list_.end(); ++it)
    {
        std::string id_str;
        std::stringstream stream;
        stream << client_id;
        stream >> id_str;
        string client_path = std::string("ModbusClient") + id_str;

        if (!yaml_help_.setParam(client_path + "/id", it->start_info.id)
            || !yaml_help_.setParam(client_path + "/name", it->start_info.name)
            || !yaml_help_.setParam(client_path + "/ip", it->start_info.ip)
            || !yaml_help_.setParam(client_path + "/port", it->start_info.port)
            || !yaml_help_.setParam(client_path + "/scan_rate", it->start_info.scan_rate)
            || !yaml_help_.setParam(client_path + "/response_timeout", it->start_info.response_timeout)
            || !yaml_help_.setParam(client_path + "/reg_info/coil/addr", it->reg_info.coil.addr)
            || !yaml_help_.setParam(client_path + "/reg_info/coil/max_nb", it->reg_info.coil.max_nb)
            || !yaml_help_.setParam(client_path + "/reg_info/discrepte_input/addr", it->reg_info.discrepte_input.addr)
            || !yaml_help_.setParam(client_path + "/reg_info/discrepte_input/max_nb", it->reg_info.discrepte_input.max_nb)
            || !yaml_help_.setParam(client_path + "/reg_info/input_register/addr", it->reg_info.input_reg.addr)
            || !yaml_help_.setParam(client_path + "/reg_info/input_register/max_nb", it->reg_info.input_reg.max_nb)
            || !yaml_help_.setParam(client_path + "/reg_info/holding_register/addr", it->reg_info.holding_reg.addr)
            || !yaml_help_.setParam(client_path + "/reg_info/holding_register/max_nb", it->reg_info.holding_reg.max_nb)
            || !yaml_help_.setParam(client_path + "/is_enable", it->is_enable))
        {
            return false;
        }

        client_id++;
    }

    if (!yaml_help_.dumpParamFile(file_path_.c_str()))
    {
        return false;
    }

    return true;
}


bool ModbusClientConfigParam::saveStartInfo(ModbusClientStartInfo start_info)
{
    vector<ModbusClientConfigParams>::iterator it = client_config_list_.begin();

    for(;it != client_config_list_.end(); ++it)
    {
        if (it->start_info.id == start_info.id)
        {
            std::string id_str;
            std::stringstream stream;
            stream << start_info.id;
            stream >> id_str;
            string client_path = std::string("ModbusClient") + id_str;

            if (!yaml_help_.setParam(client_path + "/name", start_info.name)
                || !yaml_help_.setParam(client_path + "/ip", start_info.ip)
                || !yaml_help_.setParam(client_path + "/port", start_info.port)
                || !yaml_help_.setParam(client_path + "/scan_rate", start_info.scan_rate)
                || !yaml_help_.setParam(client_path + "/response_timeout", start_info.response_timeout)
                || !yaml_help_.dumpParamFile(file_path_.c_str()))
            {
                cout << " Failed save modbus client_config.yaml " << endl;
                return false;
            }

            it->start_info.name = start_info.name;
            it->start_info.ip = start_info.ip;
            it->start_info.port = start_info.port;
            it->start_info.scan_rate = start_info.scan_rate;
            it->start_info.response_timeout = start_info.response_timeout;

            return true;
        }
    }

    return false;
}

bool ModbusClientConfigParam::saveEnableStatus(int client_id, bool status)
{
    vector<ModbusClientConfigParams>::iterator it = client_config_list_.begin();

    for(;it != client_config_list_.end(); ++it)
    {
        if (it->start_info.id == client_id)
        {
            std::string id_str;
            std::stringstream stream;
            stream << client_id;
            stream >> id_str;
            string client_path = std::string("ModbusClient") + id_str;

            if (!yaml_help_.setParam(client_path + "/id", status)
                || !yaml_help_.dumpParamFile(file_path_.c_str()))
            {
                cout << " Failed save modbus client_config.yaml " << endl;
                return false;
            }

            it->is_enable = status;
            return true;
        }
    }

    return false;
}


bool ModbusClientConfigParam::saveRegInfo(int client_id, ModbusClientRegInfo &reg_info)
{
    vector<ModbusClientConfigParams>::iterator it = client_config_list_.begin();

    for(;it != client_config_list_.end(); ++it)
    {
        if (it->start_info.id == client_id)
        {
            std::string id_str;
            std::stringstream stream;
            stream << client_id;
            stream >> id_str;
            string client_path = std::string("ModbusClient") + id_str;

            if (!yaml_help_.setParam(client_path + "/reg_info/coil/addr", reg_info.coil.addr)
                || !yaml_help_.setParam(client_path + "/reg_info/coil/max_nb", reg_info.coil.max_nb)
                || !yaml_help_.setParam(client_path + "/reg_info/discrepte_input/addr", reg_info.discrepte_input.addr)
                || !yaml_help_.setParam(client_path + "/reg_info/discrepte_input/max_nb", reg_info.discrepte_input.max_nb)
                || !yaml_help_.setParam(client_path + "/reg_info/input_register/addr", reg_info.input_reg.addr)
                || !yaml_help_.setParam(client_path + "/reg_info/input_register/max_nb", reg_info.input_reg.max_nb)
                || !yaml_help_.setParam(client_path + "/reg_info/holding_register/addr", reg_info.holding_reg.addr)
                || !yaml_help_.setParam(client_path + "/reg_info/holding_register/max_nb", reg_info.holding_reg.max_nb)
                || !yaml_help_.dumpParamFile(file_path_.c_str()))
            {
                cout << " Failed save modbus client_config.yaml " << endl;
                return false;
            }

            it->reg_info.coil.addr = reg_info.coil.addr;
            it->reg_info.coil.max_nb = reg_info.coil.max_nb;
            it->reg_info.discrepte_input.addr = reg_info.discrepte_input.addr;
            it->reg_info.discrepte_input.max_nb = reg_info.discrepte_input.max_nb;
            it->reg_info.input_reg.addr = reg_info.input_reg.addr;
            it->reg_info.input_reg.max_nb = reg_info.input_reg.max_nb;
            it->reg_info.holding_reg.addr = reg_info.holding_reg.addr;
            it->reg_info.holding_reg.max_nb = reg_info.holding_reg.max_nb;

            return true;
        }
    }

    return false;
}
