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

    list<ModbusClient*>::iterator iter;

    client_list_mutex_.lock();
    for(iter = client_list_.begin(); iter != client_list_.end();)
    {
        delete (*iter);
        iter = client_list_.erase(iter);
        iter++;
    }

    client_list_mutex_.unlock();
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
        return MODBUS_CLIENT_MANAGER_LOAD_PARAM_FAILED;
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

    if (!config_param_ptr_->loadParam())
    {
        return MODBUS_CLIENT_MANAGER_LOAD_PARAM_FAILED;
    }

    return SUCCESS;
}

ErrorCode ModbusClientManager::initCLientListByParams()
{
    ErrorCode error_code = SUCCESS;
    for(int client_id = 1; client_id != client_number_ + 1; ++client_id)
    {
        bool is_added = false;
        bool is_enable = false;
        ModbusClientStartInfo start_info;
        start_info.id = client_id;
        if (!config_param_ptr_->getIsAdded(client_id, is_added)
            || !config_param_ptr_->getEnableStatus(client_id, is_enable)
            || !config_param_ptr_->getStartInfo(start_info))
        {
            error_code = MODBUS_CLIENT_MANAGER_LOAD_PARAM_FAILED;
            continue;
        }

        if (is_added)
        {
            error_code = addClient(start_info);
            if (error_code == SUCCESS && is_enable)
            {
                error_code = setEnableStatus(client_id, is_enable);
            }
        }
    }

    return error_code;
}


bool ModbusClientManager::updateClientEnableStatus(int client_id, bool status)
{
    list<ModbusClient*>::iterator it;

    client_list_mutex_.lock();
    for(it = client_list_.begin(); it != client_list_.end(); ++it)
    {
        if ((*it)->getId() == client_id)
        {
            if ((*it)->setEnableStatus(status) == SUCCESS) 
            {
                client_list_mutex_.unlock();
                return true;
            }
            client_list_mutex_.unlock();
            return false;
        }
    }
    client_list_mutex_.unlock();
    return false;
}

bool ModbusClientManager::updateClientRegInfo(int client_id, ModbusClientRegInfo &reg_info)
{
    list<ModbusClient*>::iterator it;

    client_list_mutex_.lock();
    for(it = client_list_.begin(); it != client_list_.end(); ++it)
    {
        if ((*it)->getId() == client_id)
        {
            if ((*it)->setRegInfo(reg_info) == SUCCESS)
            {
                client_list_mutex_.unlock();
                return true;
            }

            client_list_mutex_.unlock();
            return false;
        }
    }
    client_list_mutex_.unlock();
    return false;
}

ErrorCode ModbusClientManager::addClient(ModbusClientStartInfo &start_info)
{
    list<ModbusClient*>::iterator it;

    client_list_mutex_.lock();
    for(it = client_list_.begin(); it != client_list_.end();)
    {
        if ((*it)->getId() == start_info.id)
        {
            client_list_mutex_.unlock();
            return MODBUS_CLIENT_ID_EXISTED;
        }

        ++it;
    }
    client_list_mutex_.unlock();

    if (start_info.scan_rate < scan_rate_min_
        || scan_rate_max_ < start_info.scan_rate
        || start_info.port < port_min_ 
        || port_max_ < start_info.port
        || start_info.response_timeout < response_timeout_min_
        || response_timeout_max_ < start_info.response_timeout
        || start_info.ip.length() == 0
        || start_info.name.length() == 0)
    {
        return MODBUS_CLIENT_MANAGER_INVALID_ARG;
    }

    if (!config_param_ptr_->saveStartInfo(start_info))
    {
        return MODBUS_CLIENT_MANAGER_SAVE_PARAM_FAILED;
    }

    ModbusClient* client = new ModbusClient(start_info.id, is_debug_, param_ptr_->log_level_);

    ErrorCode error_code = client->setStartInfo(start_info);
    if (error_code != SUCCESS) return error_code;

    ModbusClientRegInfo reg_info;
    if (!config_param_ptr_->getRegInfo(start_info.id, reg_info))
        return MODBUS_CLIENT_ID_NOT_EXISTED;

    error_code = client->setRegInfo(reg_info);
    if (error_code != SUCCESS) error_code;

    // bool is_enable = false;
    // if (!config_param_ptr_->getEnableStatus(start_info.id, is_enable))
        // return MODBUS_CLIENT_ID_NOT_EXISTED;
// 
    // error_code = client->setEnableStatus(is_enable);
    // if (error_code != SUCCESS) error_code;

    client_list_mutex_.lock();
    client_list_.push_back(client);
    client_list_mutex_.unlock();

    bool is_added = true;
    if (!config_param_ptr_->saveIsAdded(start_info.id, is_added))
        return MODBUS_CLIENT_ID_NOT_EXISTED;
    return SUCCESS;
}

ErrorCode ModbusClientManager::deleteClient(int client_id)
{
    ErrorCode error_code = SUCCESS;
    bool is_id_found = false;
    list<ModbusClient*>::iterator iter;

    client_list_mutex_.lock();
    for(iter = client_list_.begin(); iter != client_list_.end();)
    {
        if ((*iter)->getId() == client_id)
        {
            is_id_found = true;

            if ((*iter)->isConnected())
            {
                error_code = MODBUS_CLIENT_CONNECTED;
            }
            else if ((*iter)->getEnableStatus())
            {
                error_code = MODBUS_CLIENT_ENABLED;
            }
            else
            {
                delete (*iter);
                iter = client_list_.erase(iter);
                bool is_added = false;
                if (!config_param_ptr_->saveIsAdded(client_id, is_added))
                    error_code = MODBUS_CLIENT_ID_NOT_EXISTED;
                error_code = SUCCESS;
            }
            break;
        }
        else
        {
            iter++;
        }
    }

    client_list_mutex_.unlock();
    if (!is_id_found)
    {
        error_code = MODBUS_CLIENT_ID_NOT_EXISTED;
    }

    return error_code;
}

ErrorCode ModbusClientManager::setEnableStatus(int client_id, bool &status)
{
    list<ModbusClient*>::iterator it;

    client_list_mutex_.lock();
    for(it = client_list_.begin(); it != client_list_.end();)
    {
        if ((*it)->getId() == client_id)
        {
            if (!config_param_ptr_->saveEnableStatus(client_id, status))
            {
                client_list_mutex_.unlock();
                return MODBUS_CLIENT_MANAGER_SAVE_PARAM_FAILED;
            }

            ErrorCode error_code = (*it)->setEnableStatus(status);
            client_list_mutex_.unlock();
            return error_code;
        }

        ++it;
    }

    client_list_mutex_.unlock();
    return MODBUS_CLIENT_ID_NOT_EXISTED;
}

ErrorCode ModbusClientManager::getEnableStatus(int client_id, bool &status)
{
    list<ModbusClient*>::iterator it;

    client_list_mutex_.lock();
    for(it = client_list_.begin(); it != client_list_.end();)
    {
        if ((*it)->getId() == client_id)
        {
            status = (*it)->getEnableStatus();
            client_list_mutex_.unlock();
            return SUCCESS;
        }

        ++it;
    }

    client_list_mutex_.unlock();
    return MODBUS_CLIENT_ID_NOT_EXISTED;
}

ErrorCode ModbusClientManager::getStartInfo(int client_id, ModbusClientStartInfo &start_info)
{
    list<ModbusClient*>::iterator it;

    client_list_mutex_.lock();
    for(it = client_list_.begin(); it != client_list_.end();)
    {
        if ((*it)->getId() == client_id)
        {
            start_info = (*it)->getStartInfo();
            client_list_mutex_.unlock();
            return SUCCESS;
        }

        ++it;
    }

    client_list_mutex_.unlock();
    return MODBUS_CLIENT_ID_NOT_EXISTED;
}

ErrorCode ModbusClientManager::setRegInfo(int client_id, ModbusClientRegInfo &reg_info)
{
    list<ModbusClient*>::iterator it;

    client_list_mutex_.lock();
    for(it = client_list_.begin(); it != client_list_.end();)
    {
        if ((*it)->getId() == client_id)
        {
            ErrorCode error_code = (*it)->setRegInfo(reg_info);
            client_list_mutex_.unlock();
            return error_code;

            if (!config_param_ptr_->saveRegInfo(client_id, reg_info))
            {
                client_list_mutex_.unlock();
                return MODBUS_CLIENT_MANAGER_SAVE_PARAM_FAILED;
            }
        }
        ++it;
    }

    client_list_mutex_.unlock();
    return MODBUS_CLIENT_ID_NOT_EXISTED;
}

ErrorCode ModbusClientManager::getRegInfo(int client_id, ModbusClientRegInfo &reg_info)
{
    list<ModbusClient*>::iterator it;

    client_list_mutex_.lock();
    for(it = client_list_.begin(); it != client_list_.end();)
    {
        if ((*it)->getId() == client_id)
        {
            reg_info = (*it)->getRegInfo();
            client_list_mutex_.unlock();
            return SUCCESS;
        }
        else 
        {
            ++it;
        }
    }

    client_list_mutex_.unlock();
    return MODBUS_CLIENT_ID_NOT_EXISTED;
}

vector<int> ModbusClientManager::getConnectedClientIdList()
{
    list<ModbusClient*>::iterator it;
    vector<int> id_list;

    client_list_mutex_.lock();
    for(it = client_list_.begin(); it != client_list_.end(); ++it)
    {
        if ((*it)->isConnected())
            id_list.push_back((*it)->getId());
    }

    client_list_mutex_.unlock();
    return id_list;
}

vector<int> ModbusClientManager::getClientIdList()
{
    list<ModbusClient*>::iterator it;
    vector<int> id_list;

    client_list_mutex_.lock();
    for(it = client_list_.begin(); it != client_list_.end(); ++it)
    {
        id_list.push_back((*it)->getId());
    }

    client_list_mutex_.unlock();
    return id_list;
}

bool ModbusClientManager::isAllClientClosed()
{
    list<ModbusClient*>::iterator it;

    client_list_mutex_.lock();
    for(it = client_list_.begin(); it != client_list_.end();)
    {
        if ((*it)->isConnected())
        {
            client_list_mutex_.unlock();
            return false;
        }
        ++it;
    }

    client_list_mutex_.unlock();
    return true;
}

vector<int> ModbusClientManager::getScanRateList()
{
    list<ModbusClient*>::iterator it;
    vector<int> scan_rate_list;

    client_list_mutex_.lock();
    for(it = client_list_.begin(); it != client_list_.end(); ++it)
    {
       scan_rate_list.push_back((*it)->getScanRate());
    }

    client_list_mutex_.unlock();
    return scan_rate_list;
}

ErrorCode ModbusClientManager::getCtrlState(int client_id, int &ctrl_state)
{
    list<ModbusClient*>::iterator it;

    client_list_mutex_.lock();

    for(it = client_list_.begin(); it != client_list_.end();)
    {
        if ((*it)->getId() == client_id)
        {
            ctrl_state = (*it)->getCtrlState();
            client_list_mutex_.unlock();
            return SUCCESS;
        }
        else 
        {
            ++it;
        }
    }
    client_list_mutex_.unlock();
    return MODBUS_CLIENT_ID_NOT_EXISTED;
}

ErrorCode ModbusClientManager::connectClient(int client_id)
{
    list<ModbusClient*>::iterator it;

    client_list_mutex_.lock();
    for(it = client_list_.begin(); it != client_list_.end();)
    {
        if ((*it)->getId() == client_id)
        {
            ErrorCode error_code = (*it)->connect();
            client_list_mutex_.unlock();
            return error_code;
        }
        ++it;
    }

    client_list_mutex_.unlock();
    return MODBUS_CLIENT_ID_NOT_EXISTED;
}

ErrorCode ModbusClientManager::closeClient(int client_id)
{
    list<ModbusClient*>::iterator it;

    client_list_mutex_.lock();
    for(it = client_list_.begin(); it != client_list_.end();)
    {
        if ((*it)->getId() == client_id)
        {
            (*it)->close();
            client_list_mutex_.unlock();
            return SUCCESS;
        }

        ++it;
    }
    client_list_mutex_.unlock();
    return MODBUS_CLIENT_ID_NOT_EXISTED;
}

ErrorCode ModbusClientManager::isConnected(int client_id, bool &is_connected)
{
    list<ModbusClient*>::iterator it;

    client_list_mutex_.lock();
    for(it = client_list_.begin(); it != client_list_.end();)
    {
        if ((*it)->getId() == client_id)
        {
            is_connected = (*it)->isConnected();
            client_list_mutex_.unlock();
            return SUCCESS;
        }
        ++it;
    }

    client_list_mutex_.unlock();
    return MODBUS_CLIENT_ID_NOT_EXISTED;
}

ErrorCode ModbusClientManager::scanDataArea()
{
    list<ModbusClient*>::iterator it;
    ErrorCode error_code = SUCCESS;

    client_list_mutex_.lock();
    for(it = client_list_.begin(); it != client_list_.end(); ++it)
    {
        error_code = (*it)->scanDataArea();
    }

    client_list_mutex_.unlock();
    return error_code;
}

ErrorCode ModbusClientManager::writeCoils(int client_id, int addr, int nb, uint8_t *dest)
{
    list<ModbusClient*>::iterator it;

    client_list_mutex_.lock();
    for(it = client_list_.begin(); it != client_list_.end();)
    {
        if ((*it)->getId() == client_id)
        {
            ErrorCode error_code = (*it)->writeCoils(addr, nb, dest);
            client_list_mutex_.unlock();
            return error_code;
        }
        ++it;
    }
    client_list_mutex_.unlock();
    return MODBUS_CLIENT_ID_NOT_EXISTED;
}

ErrorCode ModbusClientManager::readCoils(int client_id, int addr, int nb, uint8_t *dest)
{
    list<ModbusClient*>::iterator it;

    client_list_mutex_.lock();
    for(it = client_list_.begin(); it != client_list_.end();)
    {
        if ((*it)->getId() == client_id)
        {
            ErrorCode error_code = (*it)->readCoils(addr, nb, dest);
            client_list_mutex_.unlock();
            return error_code;
        }
        ++it;
    }

    client_list_mutex_.unlock();
    return MODBUS_CLIENT_ID_NOT_EXISTED;
}

ErrorCode ModbusClientManager::readDiscreteInputs(int client_id, int addr, int nb, uint8_t *dest)
{
    list<ModbusClient*>::iterator it;

    client_list_mutex_.lock();
    for(it = client_list_.begin(); it != client_list_.end();)
    {
        if ((*it)->getId() == client_id)
        {
            ErrorCode error_code = (*it)->readDiscreteInputs(addr, nb, dest);
            client_list_mutex_.unlock();
            return error_code;
        }
        ++it;
    }

    client_list_mutex_.unlock();
    return MODBUS_CLIENT_ID_NOT_EXISTED;
}

ErrorCode ModbusClientManager::writeHoldingRegs(int client_id, int addr, int nb, uint16_t *dest)
{
    list<ModbusClient*>::iterator it;

    client_list_mutex_.lock();
    for(it = client_list_.begin(); it != client_list_.end();)
    {
        if ((*it)->getId() == client_id)
        {
            ErrorCode error_code = (*it)->writeHoldingRegs(addr, nb, dest);
            client_list_mutex_.unlock();
            return error_code;
        }
        ++it;
    }

    client_list_mutex_.unlock();
    return MODBUS_CLIENT_ID_NOT_EXISTED;
}

ErrorCode ModbusClientManager::readHoldingRegs(int client_id, int addr, int nb, uint16_t *dest)
{
    list<ModbusClient*>::iterator it;

    client_list_mutex_.lock();
    for(it = client_list_.begin(); it != client_list_.end();)
    {
        if ((*it)->getId() == client_id)
        {
            ErrorCode error_code = (*it)->readHoldingRegs(addr, nb, dest);
            client_list_mutex_.unlock();
            return error_code;
        }
            ++it;
    }

    client_list_mutex_.unlock();
    return MODBUS_CLIENT_ID_NOT_EXISTED;
}

ErrorCode ModbusClientManager::writeAndReadHoldingRegs(int client_id, int write_addr, int write_nb, const uint16_t *write_dest,
    int read_addr, int read_nb, uint16_t *read_dest)
{
    list<ModbusClient*>::iterator it;

    client_list_mutex_.lock();
    for(it = client_list_.begin(); it != client_list_.end();)
    {
        if ((*it)->getId() == client_id)
        {
            ErrorCode error_code = (*it)->writeAndReadHoldingRegs(write_addr, write_nb, write_dest, read_addr, read_nb, read_dest);
            client_list_mutex_.unlock();
            return error_code;
        }
        ++it;
    }

    client_list_mutex_.unlock();
    return MODBUS_CLIENT_ID_NOT_EXISTED;
}

ErrorCode ModbusClientManager::readInputRegs(int client_id, int addr, int nb, uint16_t *dest)
{
    list<ModbusClient*>::iterator it;

    client_list_mutex_.lock();
    for(it = client_list_.begin(); it != client_list_.end();)
    {
        if ((*it)->getId() == client_id)
        {
            ErrorCode error_code = (*it)->readInputRegs(addr, nb, dest);
            client_list_mutex_.unlock();
            return error_code;
        }
        ++it;
    }

    client_list_mutex_.unlock();
    return MODBUS_CLIENT_ID_NOT_EXISTED;
}


vector<ModbusClientConfigParams> ModbusClientManager::getConfigParamsList()
{
    vector<ModbusClientConfigParams> params_list;
    vector<ModbusClientConfigParams>::iterator params_it = params_list.begin();

    list<ModbusClient*>::iterator client_it;

    client_list_mutex_.lock();
    for(client_it = client_list_.begin(); client_it != client_list_.end(); ++client_it)
    {
        *(params_it) = (*client_it)->getConfigParams();
        ++params_it;
    }

    client_list_mutex_.unlock();
    return params_list;
}

ErrorCode ModbusClientManager::getConfigParams(int client_id, ModbusClientConfigParams &params)
{
    list<ModbusClient*>::iterator client_it;

    client_list_mutex_.lock();
    for(client_it = client_list_.begin(); client_it != client_list_.end();)
    {
        if ((*client_it)->getId() == client_id)
        {
            params = (*client_it)->getConfigParams();
            client_list_mutex_.unlock();
            return SUCCESS;
        }

        client_id++;
    }

    client_list_mutex_.unlock();
    return MODBUS_CLIENT_ID_NOT_EXISTED;
}

ErrorCode ModbusClientManager::updateStartInfo(ModbusClientStartInfo &start_info)
{
    list<ModbusClient*>::iterator it;

    client_list_mutex_.lock();
    for(it = client_list_.begin(); it != client_list_.end();)
    {
       if ((*it)->getId() == start_info.id)
       {
            if ((*it)->getEnableStatus())
            {
                client_list_mutex_.unlock();
                return MODBUS_CLIENT_ENABLED;
            } 

            if ((scan_rate_min_ <= start_info.scan_rate && start_info.scan_rate <= scan_rate_max_)
                || (port_min_ <= start_info.port && start_info.port <= port_max_)
                || (response_timeout_min_ <= start_info.response_timeout 
                    && start_info.response_timeout <= response_timeout_max_)
                || 0 < start_info.ip.length()
                || 0 < start_info.name.length())
            {
                ErrorCode error_code = (*it)->setStartInfo(start_info);
                client_list_mutex_.unlock();
                return error_code;
            }

            if (!config_param_ptr_->saveStartInfo(start_info))
            {
                client_list_mutex_.unlock();
                return MODBUS_CLIENT_MANAGER_SAVE_PARAM_FAILED;
            }

            client_list_mutex_.unlock();
            return MODBUS_CLIENT_MANAGER_INVALID_ARG;
       }

        ++it;
    }

    client_list_mutex_.unlock();
    return MODBUS_CLIENT_MANAGER_INVALID_ARG;
}

ErrorCode ModbusClientManager::replaceClient(int &replaced_id, ModbusClientStartInfo &start_info)
{
    if (replaced_id == start_info.id) 
        return MODBUS_CLIENT_MANAGER_INVALID_ARG;

    ErrorCode error_code = SUCCESS;
    int ctrl_state = MODBUS_CLIENT_CTRL_DISABLED;

    error_code = getCtrlState(replaced_id, ctrl_state);
    if (error_code != SUCCESS) return error_code;

    if (MODBUS_CLIENT_CTRL_ENABLED <= ctrl_state)
        return MODBUS_CLIENT_ENABLED;

    error_code = addClient(start_info);
    if (error_code != SUCCESS) return error_code;

    return deleteClient(replaced_id);
}

ErrorCode ModbusClientManager::getClientScanRate(int client_id, int &scan_rate)
{
    list<ModbusClient*>::iterator client_it;

    client_list_mutex_.lock();
    for(client_it = client_list_.begin(); client_it != client_list_.end();)
    {
        if ((*client_it)->getId() == client_id)
        {
            scan_rate = (*client_it)->getScanRate();
            client_list_mutex_.unlock();
            return SUCCESS;
        }

        client_id++;
    }

    client_list_mutex_.unlock();
    return MODBUS_CLIENT_ID_NOT_EXISTED;
}

ErrorCode ModbusClientManager::scanAllClientDataArea()
{
    ErrorCode error_code = SUCCESS;
#if 0
    std::list<ModbusClient*>::iterator it;

    client_list_mutex_.lock();
    for (it = client_list_.begin(); it != client_list_.end(); it++)
    {
        error_code = (*it)->scanDataArea();
        if (error_code != SUCCESS)
        {
            if (!(*it)->isSocketValid()) (*it)->close();
            continue;
        }
    }

    client_list_mutex_.unlock();
#endif
    return error_code;
}
