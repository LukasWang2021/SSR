#include <string>
#include <boost/date_time/posix_time/posix_time.hpp>

#include "error_monitor.h"
#include "error_code.h"
#include "common_log.h"

#include "modbus_manager.h"

using namespace std;
using namespace fst_modbus;

ModbusManager::ModbusManager():
    log_ptr_(NULL), param_ptr_(NULL)
{
    log_ptr_ = new fst_log::Logger();
    param_ptr_ = new ModbusManagerParam();
    FST_LOG_INIT("ModbusManager");
    FST_LOG_SET_LEVEL((fst_log::MessageLevel)param_ptr_->log_level_);

    config_ip_ = "192.168.1.166";param_ptr_->server_ip_;
    config_port_ = 1502;//param_ptr_->server_port_;
    device_number_ = param_ptr_->device_number_;
    connection_number_ = param_ptr_->connection_number_;
    cycle_time_ = param_ptr_->cycle_time_;

    start_mode_ = START_MODE_CLIENT;
    tcp_client_ = NULL;
    tcp_server_ = NULL;
}

ModbusManager::~ModbusManager()
{
    modbus_thread.interrupt();
    modbus_thread.join();
    if (tcp_client_ != NULL)
    {
        delete tcp_client_;
        tcp_client_ = NULL;
    }
    if (tcp_server_ != NULL)
    {
        delete tcp_server_;
        tcp_server_ = NULL;
    }
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
}

bool ModbusManager::checkModbusRegsNum(DeviceModbusInfo &device_modbus_info)
{
    bool result = true;
    RegisterInfo reg_info = getModbusRegInfo();
    switch(device_modbus_info.modbus_reg_type)
    {
        case COIL:
        {
            int coil_addr_max = reg_info.coil_nb + reg_info.coil_addr - 1;
            if (device_modbus_info.modbus_reg_addr < reg_info.coil_addr
                || coil_addr_max < device_modbus_info.modbus_reg_addr)
            {
                result = false;
            }
        }
            break;
        case DISCREPTE_INPUT:
        {
            int discrepte_input_addr_max = reg_info.discrepte_input_addr + reg_info.discrepte_input_nb - 1;
            if (device_modbus_info.modbus_reg_addr < reg_info.discrepte_input_addr
                || discrepte_input_addr_max < device_modbus_info.modbus_reg_addr)
            {
                result = false;
            }
        }
            break;
        case HOLDING_REGISTER:
        {
            int holding_reg_addr_max = reg_info.holding_register_addr + reg_info.holding_register_nb - 1;
            if (device_modbus_info.modbus_reg_addr < reg_info.holding_register_addr
                || holding_reg_addr_max < device_modbus_info.modbus_reg_addr)
            {
                result = false;
            }
        }
            break;
        case INPUT_REGISTER:
        {
            int input_reg_addr_max = reg_info.input_register_addr + reg_info.input_register_nb - 1;
            if (device_modbus_info.modbus_reg_addr < reg_info.input_register_addr
                || input_reg_addr_max < device_modbus_info.modbus_reg_addr)
            {
                result = false;
            }
        }
            break;
        default:
            result = false;
    };

    return result;
}

bool ModbusManager::pushToModbusMap(DeviceModbusInfo &element)
{
    modbus_map_mutex_.lock();
    modbus_map_.push_back(element);
    modbus_map_mutex_.unlock();
}

DeviceModbusInfo ModbusManager::eraseFromModbusMap(DeviceInfo &device_info)
{
    std::vector<DeviceModbusInfo>::iterator it;  
    DeviceModbusInfo modbus_map_element;

    modbus_map_mutex_.lock();
    for(it = modbus_map_.begin(); it != modbus_map_.end(); ++it)
    {
        if(device_info.device_type == it->device_info.device_type
            && device_info.device_index == it->device_info.device_index
            && device_info.device_addr == it->device_info.device_addr)
        {
            modbus_map_element.device_info.device_type = it->device_info.device_type;
            modbus_map_element.device_info.device_index = it->device_info.device_index;
            modbus_map_element.device_info.device_addr = it->device_info.device_addr;
            modbus_map_element.port_type = it->port_type;
            modbus_map_element.modbus_reg_type = it->modbus_reg_type;
            modbus_map_element.modbus_reg_addr = it->modbus_reg_addr;

            it = modbus_map_.erase(it);
            break;
        }
        else 
            ++it;
    }

    modbus_map_mutex_.unlock();
    return modbus_map_element;
}

void ModbusManager::clearModbusMap()
{
    modbus_map_mutex_.lock();
    modbus_map_.clear();
    modbus_map_mutex_.unlock();
}


ErrorCode ModbusManager::init(int mode, bool is_fake, bool is_debug)
{
    ErrorCode error_code = SUCCESS;

    if (!param_ptr_->loadDeviceParam())
    {
        error_code = 1;//load param error
        return error_code;
    }

    if (!param_ptr_->loadParam())
    {
        error_code = 1;//error_code
        return error_code;
    }

    config_ip_ = param_ptr_->server_ip_;
    config_port_ = param_ptr_->server_port_;
    device_number_ = param_ptr_->device_number_;
    connection_number_ = param_ptr_->connection_number_;
    cycle_time_ = param_ptr_->cycle_time_;
    is_debug_ = is_debug;

    start_mode_ = static_cast<StartMode>(mode);
    if ((START_MODE_SERVER != start_mode_)
        && (START_MODE_CLIENT != start_mode_))
    {
        error_code = 1; // error code
        return error_code;
    }

    if (START_MODE_SERVER == start_mode_)
    {
        startThread();
        usleep(10000);
    }

    tcp_client_ = new ModbusTCPClient(getConfigIP(), getConfigPort());
    if (tcp_client_ == NULL)
    {
        error_code = 1; // error code
        return error_code;
    }

    setResponseTimeout(param_ptr_->response_timeout_);
    setByteTimeout(param_ptr_->bytes_timeout_);

    if (!tcp_client_->init())
    {
        FST_ERROR("Modbus Manager : TCP Client init failed");
        return -1;
    }

    tcp_client_->setDebug(true);

    if (is_fake)
    {
        // to do ...
    }
    else
    {
        // to do...
    }

    return error_code;
}

ErrorCode ModbusManager::getDeviceStatus(DeviceInfo &device_info, int addr_nb, uint8_t* dest)
{
    ErrorCode error_code = SUCCESS;
    if (STATUS_ONE_OP_NUM < addr_nb)
    {
        error_code = 1; // error_code
        return error_code;
    }

    ModbusStatus status;
    status.nb = addr_nb;
    status.dest = dest;
    status.addr = getRegStartAddrFromModbusMap(device_info);

    if (-1 == status.addr)
    {
        error_code = 1; // error_code, to do ...
        return error_code;
    }

    RegisterType type = getRegTypeFromModbusMap(device_info);
    switch(type)
    {
        case COIL:
        {
            if (!tcp_client_->readCoils(status))
            {
                error_code = 1; // error code. todo...
                return error_code;
            }
            memcpy(dest, status.dest, status.nb);
        }
            break;
        case DISCREPTE_INPUT:
        {
            if (!tcp_client_->readDiscreteInputs(status))
            {
                error_code = 1; // error code
                return error_code;
            }
            memcpy(dest, status.dest, status.nb);
        }
            break;
        default:
        {
            error_code = 1; // error_code
            return error_code;
        }
    }
}

ErrorCode ModbusManager::setDeviceStatus(DeviceInfo &device_info, int addr_nb, uint8_t* dest)
{
    ErrorCode error_code = SUCCESS;
    if (STATUS_ONE_OP_NUM < addr_nb)
    {
        error_code = 1; // error_code
        return error_code;
    }

    ModbusStatus status;
    status.nb = addr_nb;
    status.dest = dest;
    status.addr = getRegStartAddrFromModbusMap(device_info);

    if (-1 == status.addr)
    {
        error_code = 1; // error_code
        return error_code;
    }

    RegisterType type = getRegTypeFromModbusMap(device_info);
    if (COIL != type)
    {
        FST_ERROR("Modbus Manger : write coil : reg type error");
        error_code = 1; // error_code
        return error_code;
    }

    if (!tcp_client_->writeCoils(status))
    {
        FST_ERROR("Modbus Manger : write coil : failed");
        error_code = 1; // error code
    }

    return error_code;
}

ErrorCode ModbusManager::setDeviceRegisters(DeviceInfo &device_info, int addr_nb, uint16_t* dest)
{
    ErrorCode error_code = SUCCESS;
    if (REGISTER_ONE_OP_NUM < addr_nb)
    {
        error_code = 1; // error_code
        return error_code;
    }

    ModbusRegisters reg;
    reg.nb = addr_nb;
    reg.dest = dest;
    reg.addr = getRegStartAddrFromModbusMap(device_info);

    if (-1 == reg.addr)
    {
        error_code = 1; // error_code
        return error_code;
    }

    RegisterType type = getRegTypeFromModbusMap(device_info);
    if (HOLDING_REGISTER != type)
    {
        FST_ERROR("Modbus Manger : failed write holding register : reg type error");
        error_code = 1; // error_code
        return error_code;
    }

    if (!tcp_client_->writeHoldingRegs(reg))
    {
        FST_ERROR("Modbus Manger : failed write coil.");
        error_code = 1; // error code
    }

    return error_code;
}

ErrorCode ModbusManager::getDeviceRegisters(DeviceInfo &device_info, int addr_nb, uint16_t* dest)
{
    ErrorCode error_code = SUCCESS;
    if (REGISTER_ONE_OP_NUM < addr_nb)
    {
        error_code = 1; // error_code
        return error_code;
    }

    ModbusRegisters reg;
    reg.nb = addr_nb;
    reg.dest = dest;
    reg.addr = getRegStartAddrFromModbusMap(device_info);

    if (-1 == reg.addr)
    {
        error_code = 1; // error_code, to do ...
        return error_code;
    }

    RegisterType type = getRegTypeFromModbusMap(device_info);
    switch(type)
    {
        case HOLDING_REGISTER:
        {
            if (!tcp_client_->readHoldingRegs(reg))
            {
                error_code = 1; // error code. todo...
                return error_code;
            }
            memcpy(dest, reg.dest, reg.nb);
        }
            break;
        case INPUT_REGISTER:
        {
            if (!tcp_client_->readInputRegs(reg))
            {
                error_code = 1; // error code
                return error_code;
            }
            memcpy(dest, reg.dest, reg.nb);
        }
            break;
        default:
        {
            error_code = 1; // error_code
            return error_code;
        }
    }
}

RegisterInfo ModbusManager::getModbusRegInfo()
{
    return param_ptr_->modbus_register_info_;
}

int ModbusManager::getRegStartAddrFromModbusMap(DeviceInfo &device_info)
{
    std::vector<DeviceModbusInfo>::iterator it;
    int addr = -1;
    modbus_map_mutex_.lock();
    for(it = modbus_map_.begin(); it != modbus_map_.end(); ++it)
    {
        if(device_info.device_type == it->device_info.device_type
            && device_info.device_index == it->device_info.device_index
            && device_info.device_addr == it->device_info.device_addr)
        {
            addr = it->modbus_reg_addr;
            break;
        }
        else 
            ++it;
    }

    modbus_map_mutex_.unlock();
    return addr;
}

RegisterType ModbusManager::getRegTypeFromModbusMap(DeviceInfo &device_info)
{
    std::vector<DeviceModbusInfo>::iterator it;
    RegisterType type = DEFAULT;
    modbus_map_mutex_.lock();
    for(it = modbus_map_.begin(); it != modbus_map_.end(); ++it)
    {
        if(device_info.device_type == it->device_info.device_type
            && device_info.device_index == it->device_info.device_index
            && device_info.device_addr == it->device_info.device_addr)
        {
            type = it->modbus_reg_type;
            break;
        }
        else 
            ++it;
    }

    modbus_map_mutex_.unlock();
    return type;
}

ErrorCode ModbusManager::getRegInfoOfDeviceFromModbusMap(DeviceType device_type, int &num)
{
    ErrorCode error_code = SUCCESS;
    return error_code;
}

ErrorCode ModbusManager::setResponseTimeout(timeval& timeout)
{
    ErrorCode error_code = SUCCESS;
    if(!tcp_client_->setResponseTimeout(timeout))
    {
        FST_ERROR("Modbus Manager : failed set response timeout.");
        error_code = 1;//error code
    }
    return error_code;
}

ErrorCode ModbusManager::setByteTimeout(timeval& timeout)
{
    ErrorCode error_code = SUCCESS;
    if(!tcp_client_->setByteTimeout(timeout))
    {
        FST_ERROR("Modbus Manager : failed set bytes timeout.");
        error_code = 1;//error code
    }
    return error_code;
}

void ModbusManager::startThread(void)
{
    modbus_thread = boost::thread(boost::bind(&ModbusManager::runThread, this));
}

void ModbusManager::runThread(void)
{
    ErrorCode error_code = SUCCESS;
    try
    {
        tcp_server_ = new ModbusTCPServer(local_ip_.get(), config_port_);
        config_ip_ = local_ip_.get();
        if (tcp_server_ == NULL)
        {
            FST_ERROR("Modbus Manager : new server failed");
            error_code = 1; // error code, add error_code
            return;
        }

        RegisterInfo modbus_reg_info = getModbusRegInfo();
        if (!tcp_server_->mapping_new_start_address(
            modbus_reg_info.coil_addr, modbus_reg_info.coil_nb, 
            modbus_reg_info.discrepte_input_addr, modbus_reg_info.discrepte_input_nb, 
            modbus_reg_info.holding_register_addr, modbus_reg_info.holding_register_nb, 
            modbus_reg_info.input_register_addr, modbus_reg_info.input_register_nb))
        {
            FST_ERROR("Modbus Manager : failed to new mapping for modbus");
            return;
        }
        if (!tcp_server_->init(connection_number_))
        {
            printf("Modbus: Failed to init tcp server : %d!\n", connection_number_);
            error_code = 1; // error code
            return ;
        }

        tcp_server_->setDebug(is_debug_);

        while (true)
        {
            uint8_t query[MODBUS_TCP_MAX_ADU_LENGTH];
            if (0 <= tcp_server_->receive(query))
            {
                tcp_server_->reply(query, MODBUS_TCP_MAX_ADU_LENGTH);
            }
            else
            {
                error_code = 1; // error code
                tcp_server_->close(); //close ctx
                tcp_server_->accept(); //accept next frames from client
            }

            boost::this_thread::sleep(boost::posix_time::microseconds(cycle_time_));
        }
        tcp_server_->mapping_free();
        tcp_server_->close();
    }
    catch (boost::thread_interrupted &)
    {
        std::cout<< "Stop Modbus Server Thread. "<<std::endl;
        tcp_server_->mapping_free();
        tcp_server_->close();
    }
}

