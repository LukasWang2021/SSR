#include "modbus_client_manager.h"

#include "error_monitor.h"
#include "error_code.h"
#include "serverAlarmApi.h"

using namespace fst_hal;

ModbusClientManager::ModbusClientManager(string param_file_path, string config_file_path)
{
    log_ptr_ = new fst_log::Logger();
    param_ptr_ = new ModbusClientParam(param_file_path);
    FST_LOG_INIT("ModbusClientManager");
    FST_LOG_SET_LEVEL((fst_log::MessageLevel)param_ptr_->log_level_);
    config_param_ptr_ = NULL;
    //client_config_param_list_.clear();
    client_list_.clear();
    config_param_file_path_ = config_file_path;

    comm_type_ = "";
    is_debug_ = false;
    client_number_ = 0;
    scan_rate_min_ = 0;
    scan_rate_max_ = 0;
    port_min_ = 0;
    port_max_ = 0;
    response_timeout_min_ = 0;
    response_timeout_max_ = 0;
    coil_addr_min_ = 0;
    coil_addr_max_ = 0;
    discrepte_input_min_ = 0;
    discrepte_input_max_ = 0;
    holding_reg_min_ = 0;
    holding_reg_max_ = 0;
    input_reg_min_= 0;
    input_reg_max_= 0;
}

ModbusClientManager::ModbusClientManager()
{
    log_ptr_ = new fst_log::Logger();
    param_ptr_ = new ModbusClientParam("");
    FST_LOG_INIT("ModbusClientManager");
    FST_LOG_SET_LEVEL((fst_log::MessageLevel)param_ptr_->log_level_);
    config_param_ptr_ = NULL;

    client_list_.clear();

    comm_type_ = "";
    is_debug_ = false;
    client_number_ = 0;
    scan_rate_min_ = 0;
    scan_rate_max_ = 0;
    port_min_ = 0;
    port_max_ = 0;
    response_timeout_min_ = 0;
    response_timeout_max_ = 0;
    coil_addr_min_ = 0;
    coil_addr_max_ = 0;
    discrepte_input_min_ = 0;
    discrepte_input_max_ = 0;
    holding_reg_min_ = 0;
    holding_reg_max_ = 0;
    input_reg_min_= 0;
    input_reg_max_= 0;
}

ModbusClientManager::~ModbusClientManager()
{
    client_list_.clear();

    if (log_ptr_ != NULL)
    {
        delete log_ptr_;
        log_ptr_ = NULL;
    }

    if (param_ptr_ != NULL)
    {
        delete param_ptr_;
        param_ptr_ = NULL;
    }

    if (config_param_ptr_ != NULL)
    {
        delete config_param_ptr_;
        config_param_ptr_ = NULL;
    }
}

ErrorCode ModbusClientManager::initParam()
{
    if (!param_ptr_->loadParam())
    {
        return 0x12345678; // MODBUS_CLIENT_MANAGER_LOAD_PARAM_FAILED
    }

    comm_type_ = param_ptr_->comm_type_;
    is_debug_ = param_ptr_->is_debug_;
    client_number_ = param_ptr_->client_number_;

    scan_rate_min_ =  param_ptr_->scan_rate_min_;
    scan_rate_max_ =  param_ptr_->scan_rate_max_;
    port_min_ =  param_ptr_->port_min_;
    port_max_ =  param_ptr_->port_max_;
    response_timeout_min_ =  param_ptr_->response_timeout_min_;
    response_timeout_max_ =  param_ptr_->response_timeout_max_;

    coil_addr_min_ =  param_ptr_->coil_addr_min_;
    coil_addr_max_ =  param_ptr_->coil_addr_max_;
    discrepte_input_min_ = param_ptr_->discrepte_input_min_;
    discrepte_input_max_ = param_ptr_->discrepte_input_max_;
    holding_reg_min_ = param_ptr_->holding_reg_min_;
    holding_reg_max_ = param_ptr_->holding_reg_max_;
    input_reg_min_= param_ptr_->input_reg_min_;
    input_reg_max_= param_ptr_->input_reg_max_;

    if (config_param_ptr_ == NULL)
        config_param_ptr_ = new ModbusClientConfigParam(config_param_file_path_, client_number_);

    FST_ERROR("config_param_file_path_ = %s", config_param_file_path_.c_str());
    FST_ERROR("client_number_ = %d", client_number_);

    if (!config_param_ptr_->loadParam())
    {
        return 0x12345678; // MODBUS_CLIENT_MANAGER_LOAD_CONFIG_PARAM_FAILED
    }

    // if (!initClientListByConfigParamList())
        // return 0x12345678; //MODBUS_CLIENT_LIST_INIT_FALIED;

    return SUCCESS;
}

bool ModbusClientManager::initClientListByConfigParamList()
{
    return true;
}

ErrorCode ModbusClientManager::addClient(ModbusClientStartInfo start_info)
{
    vector<ModbusClient>::iterator it;

    for(it = client_list_.begin(); it != client_list_.end(); ++it)
    {
        if (it->getId() == start_info.id)
        {
            return SUCCESS; // MODBUS_CLIENT_IS_ADDED
        }
    }

    if ((scan_rate_min_ <= start_info.scan_rate && start_info.scan_rate <= scan_rate_max_)
        || (port_min_ <= start_info.port && start_info.port <= port_max_)
        || (response_timeout_min_ <= start_info.response_timeout 
            && start_info.response_timeout <= response_timeout_max_)
        || 0 < start_info.ip.length()
        || 0 < start_info.name.length())
    {
        return 0x12345678; // MODBUS_CLIENT_MANAGER_PARAM_INVALID
    }

    if (config_param_ptr_->saveStartInfo(start_info))
    {
        return 0x12345678; // MODBUS_CLIENT_MANAGER_SAEV_PARAM_FAILED
    }

    ModbusClient client(start_info.id, is_debug_, param_ptr_->log_level_);

    ErrorCode error_code = client.setStartInfo(start_info);
    if (error_code != SUCCESS) return error_code;

    error_code = client.init();
    if (error_code != SUCCESS) return error_code;

    client_list_.push_back(client);
    return SUCCESS;
}

ErrorCode ModbusClientManager::deleteClient(int client_id)
{
    vector<ModbusClient>::iterator it;

    for(it = client_list_.begin(); it != client_list_.end();)
    {
        if (it->getId() == client_id)
        {
            if (it->isRunning()) return 0x123456768;// MODBUS_CLIENT_IS_OPENED
            it = client_list_.erase(it);
            return SUCCESS;
        }
        else
        {
            ++it;
        }
    }

    return 0x12345678; //MODBUS_CLIENT_NOT_EXISTED;
}

ErrorCode ModbusClientManager::setEnableStatus(int client_id, bool status)
{
    vector<ModbusClient>::iterator it;

    for(it = client_list_.begin(); it != client_list_.end();)
    {
        if (it->getId() == client_id)
        {
            return it->setEnableStatus(status);
        }
        else 
        {
            ++it;
        }
    }

    return 0x12345678; //MODBUS_CLIENT_NOT_EXISTED;
}

ErrorCode ModbusClientManager::getEnableStatus(int client_id, bool status)
{
    vector<ModbusClient>::iterator it;

    for(it = client_list_.begin(); it != client_list_.end();)
    {
        if (it->getId() == client_id)
        {
            status = it->getEnableStatus();
            return SUCCESS;
        }
        else 
        {
            ++it;
        }
    }

    return 0x12345678; //MODBUS_CLIENT_NOT_EXISTED;
}

ErrorCode ModbusClientManager::getStartInfo(int client_id, ModbusClientStartInfo start_info)
{
    vector<ModbusClient>::iterator it;

    for(it = client_list_.begin(); it != client_list_.end();)
    {
        if (it->getId() == client_id)
        {
            start_info = it->getStartInfo();
            return SUCCESS;
        }
        else 
        {
            ++it;
        }
    }

    return 0x12345678; //MODBUS_CLIENT_NOT_EXISTED;
}

ErrorCode ModbusClientManager::setRegInfo(int client_id, ModbusClientRegInfo reg_info)
{
    vector<ModbusClient>::iterator it;

    for(it = client_list_.begin(); it != client_list_.end();)
    {
        if (it->getId() == client_id)
        {
            return it->setRegInfo(reg_info);
        }
        else 
        {
            ++it;
        }
    }

    return 0x12345678; //MODBUS_CLIENT_NOT_EXISTED;
}

ErrorCode ModbusClientManager::getRegInfo(int client_id, ModbusClientRegInfo reg_info)
{
    vector<ModbusClient>::iterator it;

    for(it = client_list_.begin(); it != client_list_.end();)
    {
        if (it->getId() == client_id)
        {
            reg_info = it->getRegInfo();
            return SUCCESS;
        }
        else 
        {
            ++it;
        }
    }

    return 0x12345678; //MODBUS_CLIENT_NOT_EXISTED;
}

vector<int> ModbusClientManager::getRunningClientIdList()
{
    vector<ModbusClient>::iterator it;
    vector<int> id_list;

    for(it = client_list_.begin(); it != client_list_.end(); ++it)
    {
        if (it->isRunning())
            id_list.push_back(it->getId());
    }

    return id_list;
}

vector<int> ModbusClientManager::getClientIdList()
{
    vector<ModbusClient>::iterator it;
    vector<int> id_list;

    for(it = client_list_.begin(); it != client_list_.end(); ++it)
    {
        id_list.push_back(it->getId());
    }

    return id_list;
}

bool ModbusClientManager::isAnyClientClosed()
{
    vector<ModbusClient>::iterator it;

    for(it = client_list_.begin(); it != client_list_.end();)
    {
        if (it->isRunning())
        {
            return false;
        }
        else
        {
             ++it;
        }
    }

    return true;
}

vector<int> ModbusClientManager::getScanRateList()
{
    vector<ModbusClient>::iterator it;
    vector<int> scan_rate_list;

    for(it = client_list_.begin(); it != client_list_.end(); ++it)
    {
       scan_rate_list.push_back(it->getScanRate());
    }

    return scan_rate_list;
}

int ModbusClientManager::getCtrlState(int client_id)
{
    vector<ModbusClient>::iterator it;

    for(it = client_list_.begin(); it != client_list_.end();)
    {
        if (it->getId() == client_id)
        {
            return it->getCtrlState();
        }
        else 
        {
            ++it;
        }
    }

    return 0x12345678; //MODBUS_CLIENT_NOT_EXISTED;
}

ErrorCode ModbusClientManager::openClient(int client_id)
{
    vector<ModbusClient>::iterator it;

    for(it = client_list_.begin(); it != client_list_.end();)
    {
        if (it->getId() == client_id)
        {
            return it->open();
        }
        else 
        {
            ++it;
        }
    }

    return 0x12345678; //MODBUS_CLIENT_NOT_EXISTED;
}

ErrorCode ModbusClientManager::closeClient(int client_id)
{
    vector<ModbusClient>::iterator it;

    for(it = client_list_.begin(); it != client_list_.end();)
    {
        if (it->getId() == client_id)
        {
            it->close();
            return SUCCESS;
        }
        else 
        {
            ++it;
        }
    }

    return 0x12345678; //MODBUS_CLIENT_NOT_EXISTED;
}

ErrorCode ModbusClientManager::isRunning(int client_id, bool is_running)
{
    vector<ModbusClient>::iterator it;

    for(it = client_list_.begin(); it != client_list_.end();)
    {
        if (it->getId() == client_id)
        {
            is_running = it->isRunning();
            return SUCCESS;
        }
        else
        {
            ++it;
        }
    }

    return 0x12345678; //MODBUS_CLIENT_NOT_EXISTED;
}

ErrorCode ModbusClientManager::writeCoils(int client_id, int addr, int nb, uint8_t *dest)
{
   vector<ModbusClient>::iterator it;

    for(it = client_list_.begin(); it != client_list_.end();)
    {
        if (it->getId() == client_id)
        {
            return it->writeCoils(addr, nb, dest);
        }
        else 
        {
            ++it;
        }
    }

    return 0x12345678; //MODBUS_CLIENT_NOT_EXISTED;
}

ErrorCode ModbusClientManager::readCoils(int client_id, int addr, int nb, uint8_t *dest)
{
   vector<ModbusClient>::iterator it;

    for(it = client_list_.begin(); it != client_list_.end();)
    {
        if (it->getId() == client_id)
        {
            return it->readCoils(addr, nb, dest);
        }
        else 
        {
            ++it;
        }
    }

    return 0x12345678; //MODBUS_CLIENT_NOT_EXISTED;
}

ErrorCode ModbusClientManager::readDiscreteInputs(int client_id, int addr, int nb, uint8_t *dest)
{
   vector<ModbusClient>::iterator it;

    for(it = client_list_.begin(); it != client_list_.end();)
    {
        if (it->getId() == client_id)
        {
            return it->readDiscreteInputs(addr, nb, dest);
        }
        else 
        {
            ++it;
        }
    }

    return 0x12345678; //MODBUS_CLIENT_NOT_EXISTED;
}

ErrorCode ModbusClientManager::writeHoldingRegs(int client_id, int addr, int nb, uint16_t *dest)
{
   vector<ModbusClient>::iterator it;

    for(it = client_list_.begin(); it != client_list_.end();)
    {
        if (it->getId() == client_id)
        {
            return it->writeHoldingRegs(addr, nb, dest);
        }
        else 
        {
            ++it;
        }
    }

    return 0x12345678; //MODBUS_CLIENT_NOT_EXISTED;
}

ErrorCode ModbusClientManager::readHoldingRegs(int client_id, int addr, int nb, uint16_t *dest)
{
   vector<ModbusClient>::iterator it;

    for(it = client_list_.begin(); it != client_list_.end();)
    {
        if (it->getId() == client_id)
        {
            return it->readHoldingRegs(addr, nb, dest);
        }
        else 
        {
            ++it;
        }
    }

    return 0x12345678; //MODBUS_CLIENT_NOT_EXISTED;
}

ErrorCode ModbusClientManager::writeAndReadHoldingRegs(int client_id, int write_addr, int write_nb, const uint16_t *write_dest,
    int read_addr, int read_nb, uint16_t *read_dest)
{
   vector<ModbusClient>::iterator it;

    for(it = client_list_.begin(); it != client_list_.end();)
    {
        if (it->getId() == client_id)
        {
            return it->writeAndReadHoldingRegs(write_addr, write_nb, write_dest, read_addr, read_nb, read_dest);
        }
        else 
        {
            ++it;
        }
    }

    return 0x12345678; //MODBUS_CLIENT_NOT_EXISTED;
}

ErrorCode ModbusClientManager::readInputRegs(int client_id, int addr, int nb, uint16_t *dest)
{
   vector<ModbusClient>::iterator it;

    for(it = client_list_.begin(); it != client_list_.end();)
    {
        if (it->getId() == client_id)
        {
            return it->readInputRegs(addr, nb, dest);
        }
        else 
        {
            ++it;
        }
    }

    return 0x12345678; //MODBUS_CLIENT_NOT_EXISTED;
}


vector<ModbusClientConfigParams> ModbusClientManager::getConfigParamsList()
{
    vector<ModbusClientConfigParams> params_list;
    vector<ModbusClientConfigParams>::iterator params_it = params_list.begin();

   vector<ModbusClient>::iterator client_it;

    for(client_it = client_list_.begin(); client_it != client_list_.end(); ++client_it)
    {
        *(params_it) = client_it->getConfigParams();
        ++ params_it;
    }

    return params_list;
}

ErrorCode ModbusClientManager::updateStartInfo(int client_id, ModbusClientStartInfo start_info)
{
    vector<ModbusClient>::iterator it;

    if (client_id == start_info.id)
    {
        for(it = client_list_.begin(); it != client_list_.end();)
        {
           if (it->getId() == client_id)
           {
                if ((scan_rate_min_ <= start_info.scan_rate && start_info.scan_rate <= scan_rate_max_)
                    || (port_min_ <= start_info.port && start_info.port <= port_max_)
                    || (response_timeout_min_ <= start_info.response_timeout 
                        && start_info.response_timeout <= response_timeout_max_)
                    || 0 < start_info.ip.length()
                    || 0 < start_info.name.length())
                {
                    return it->setStartInfo(start_info);
                }

                return 0x12345678; //MODBUS_CLIENT_PARAM_INVALID;
           }
           else 
           {
                ++it;
           }
        }

        return 0x12345678; // MODBUS_CLIENT_PARAM_INVALID
    }

    for(it = client_list_.begin(); it != client_list_.end();)
    {
       if (it->getId() == start_info.id)
       {
            return 0x12345678; //MODBUS_CLIENT_PARAM_INVALID;
       }
       else
       {
            ++it;
       }
    }

    for(it = client_list_.begin(); it != client_list_.end(); ++it)
    {
       if (it->getId() != client_id)
       {
            ++it;
       }
       else 
       {
            ErrorCode error_code = deleteClient(client_id);
            if (error_code != SUCCESS) return error_code;
            return addClient(start_info);
       }
    }

    return 0x12345678; // MODBUS_CLIENT_PARAM_INVALID
}