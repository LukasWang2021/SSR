#include <map>
#include <vector>
#include "interpreter_device.h"
#include "interpreter_control.h"
#include "io_1000.h"
#include "fio_device.h"
#include "log_manager_producer.h"
#include "force_sensor.h"

using namespace std;
using namespace hal_space;
using namespace log_space;
using namespace sensors_space;

// vector<BaseDevice *> io_dev;
static map<DeviceType, BaseDevice *> io_ptr_map;
static ForceSensor *force_sensor_ptr;

bool InterpDevice_Init(vector<BaseDevice *> io_ptr, ForceSensor *force_sn_ptr)
{
    io_ptr_map.clear();
    for(auto iter = io_ptr.begin(); iter != io_ptr.end(); ++iter)
    {
        if(*iter == NULL) return false;
        io_ptr_map.insert(make_pair((*iter)->getDeviceType(), *iter));
    }

    if(force_sn_ptr == NULL) return false;

    force_sensor_ptr = force_sn_ptr;

    return true;
}

ErrorCode InterpDevice_GetDIBit(uint32_t offset, uint8_t &value)
{
    Io1000 *io1000_dev_ptr = (Io1000 *)(io_ptr_map[DEVICE_TYPE_DIO]);
    if(!InterpCtrl::instance().runExecSyncCallback())
        return INTERPRETER_ERROR_SYNC_CALL_FAILED;

    ErrorCode ret = io1000_dev_ptr->readDiBit(offset, value);
    if(ret != 0)
    {
        LogProducer::error("interpreter","get DI[%u] fail 0x%llX", offset, ret);
    }
    return ret;
}
ErrorCode InterpDevice_GetDOBit(uint32_t offset, uint8_t &value)
{
    Io1000 *io1000_dev_ptr = (Io1000 *)(io_ptr_map[DEVICE_TYPE_DIO]);
    if(!InterpCtrl::instance().runExecSyncCallback())
        return INTERPRETER_ERROR_SYNC_CALL_FAILED;

    ErrorCode ret = io1000_dev_ptr->readDoBit(offset, value);
    if(ret != 0)
    {
        LogProducer::error("interpreter","get DO[%u] fail 0x%llX", offset, ret);
    }
    return ret;
}

ErrorCode InterpDevice_SetDOBit(uint32_t offset, uint8_t value)
{
    Io1000 *io1000_dev_ptr = (Io1000 *)(io_ptr_map[DEVICE_TYPE_DIO]);
    if(!InterpCtrl::instance().runExecSyncCallback())
        return INTERPRETER_ERROR_SYNC_CALL_FAILED;

    ErrorCode ret = io1000_dev_ptr->writeDoBit(offset, value);
    if(ret != 0)
    {
        LogProducer::error("interpreter","set DO[%u] fail 0x%llX", offset, ret);
    }
    return ret;
}

ErrorCode InterpDevice_GetForceRawValue(uint32_t id, double value[6])
{
    if(!InterpCtrl::instance().runExecSyncCallback())
        return INTERPRETER_ERROR_SYNC_CALL_FAILED;

    if(!force_sensor_ptr->getSourceValue(id, value, 6))
    {
        return -1;
    }
    return SUCCESS;
}

ErrorCode InterpDevice_GetForceCalibValue(uint32_t id, double value[6])
{
    if(!InterpCtrl::instance().runExecSyncCallback())
        return INTERPRETER_ERROR_SYNC_CALL_FAILED;

    if(!force_sensor_ptr->getCalibratedValue(id, value, 6))
    {
        return -1;
    }
    return SUCCESS;
}

ErrorCode InterpDevice_ReloadForceParam(uint32_t id)
{
    if(!InterpCtrl::instance().runExecSyncCallback())
        return INTERPRETER_ERROR_SYNC_CALL_FAILED;

    if(!force_sensor_ptr->loadCalibrationParams(id))
    {
        return -1;
    }
    return SUCCESS;
}

ErrorCode InterpDevice_FioControl(uint32_t cmd_type, uint32_t cmd_value, uint32_t *cmd_result)
{
    FioDevice *fio_dev_ptr = (FioDevice *)(io_ptr_map[DEVICE_TYPE_FIO]);

    if(!InterpCtrl::instance().runExecSyncCallback())
        return INTERPRETER_ERROR_SYNC_CALL_FAILED;

    ErrorCode ret = fio_dev_ptr->sendCmdRcvRpl(cmd_type, cmd_value, cmd_result);
    if(ret != 0)
    {
        LogProducer::error("interpreter","FIO send cmd[%u][%u] fail 0x%llX", cmd_type, cmd_value, ret);
    }

    return ret;
}
