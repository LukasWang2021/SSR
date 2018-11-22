/**********************************************
Copyright © 2016 Foresight-Robotics Ltd. All rights reserved.
File:       io_manager.cpp
Author:     Feng.Wu 
Create:     16-Nov-2018
Modify:     16-Nov-2018
Summary:    dealing with IO virtual board
**********************************************/

#include "fst_io_manager.h"
#include "error_monitor.h"

namespace fst_hal
{

bool IOManager::addVirtualBoard(void)
{

    IODeviceUnit unit;
    initIODeviceUnit(unit);

    unit.info.id = param_ptr_->virtual_board_address_;
    unit.info.dev_type = DEVICE_TYPE_FST_IO;
    unit.info.device_type = "virtual_Board";
    unit.info.comm_type = "RS485";
    unit.info.DI_num = param_ptr_->virtual_DI_number_;
    unit.info.DO_num = param_ptr_->virtual_DO_number_;
    unit.info.RI_num = 0;
    unit.info.RO_num = 0;
    unit.port_values.id = unit.info.id;
    memset(unit.port_values.virtual_DI, 0, sizeof(unit.port_values.virtual_DI));
    memset(unit.port_values.virtual_DO, 0, sizeof(unit.port_values.virtual_DO));
    vector_dev_.push_back(unit);

    return true;
}


ErrorCode IOManager::virtualGetModuleValue(uint32_t physics_id, uint8_t &port_value, int index, std::vector<IODeviceUnit> &io)
{
    PhysicsID id;
    id.number = physics_id;
    int frame = (id.info.port - 1) / 8;
    int shift = (id.info.port - 1) % 8;

    if (id.info.port_type == IO_TYPE_DI) {
        if((id.info.port > io[index].info.DI_num) || (id.info.port == 0)) {
            FST_ERROR("IOManager::virtualGetModuleValue(): invalid port seq for DI - %d", id.info.port);
            fst_base::ErrorMonitor::instance()->add(IO_INVALID_PORT_SEQ);
            return IO_INVALID_PORT_SEQ;
        }

        port_value = (io[index].port_values.virtual_DI[frame] >> shift) & 0x01;
    } else if (id.info.port_type == IO_TYPE_DO) {
        if (id.info.port > io[index].info.DO_num || id.info.port == 0) {
            FST_ERROR("IOManager::virtualGetModuleValue(): invalid port seq for DO - %d", id.info.port);
            fst_base::ErrorMonitor::instance()->add(IO_INVALID_PORT_SEQ);
            return IO_INVALID_PORT_SEQ;
        }
        port_value = (io[index].port_values.virtual_DO[frame] >> shift) & 0x01;
    }

    return SUCCESS;

}


ErrorCode IOManager::virtualSetModuleValue(uint32_t physics_id, uint8_t port_value, int index, std::vector<IODeviceUnit> &io)
{
    PhysicsID id;
    id.number = physics_id;
    int frame = (id.info.port - 1) / 8;
    int shift = (id.info.port - 1) % 8;
    if (id.info.port_type == IO_TYPE_DO) {
        if((id.info.port > io[index].info.DO_num) || (id.info.port == 0)) {
            FST_ERROR("IOManager::setModuleValue(): invalid port seq for DO - %d", id.info.port);
            fst_base::ErrorMonitor::instance()->add(IO_INVALID_PORT_SEQ);
            return IO_INVALID_PORT_SEQ;
        }
        {
            boost::mutex::scoped_lock lock(mutex_); //------lock mutex-----//
            if (port_value == 0) {
                vector_dev_[index].port_values.virtual_DO[frame] &= ~(0x01 << shift);
            }
            else {
                vector_dev_[index].port_values.virtual_DO[frame] |= 0x01 << shift;
            }
        }

    } else if (id.info.port_type == IO_TYPE_DI) {
        if((id.info.port > io[index].info.DI_num) || (id.info.port == 0)) {
            FST_ERROR("IOManager::setModuleValue(): invalid port seq for RO - %d", id.info.port);
            fst_base::ErrorMonitor::instance()->add(IO_INVALID_PORT_SEQ);
            return IO_INVALID_PORT_SEQ;
        }
        {
            boost::mutex::scoped_lock lock(mutex_); //------lock mutex-----//
            if (port_value == 0) {
                vector_dev_[index].port_values.virtual_DI[frame] &= ~(0x01 << shift);
            }
            else {
                vector_dev_[index].port_values.virtual_DI[frame] |= 0x01 << shift;
            }
        }
    }
    return SUCCESS;

}





} //namespace fst_io_manager

