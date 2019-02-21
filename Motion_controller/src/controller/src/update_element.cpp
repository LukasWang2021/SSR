#include "controller_publish.h"
#include "basic_alg_datatype.h"

using namespace fst_ctrl;
using namespace fst_mc;
using namespace fst_alg;

void ControllerPublish::updateAxisGroupJointFeedback()
{
    Joint joint_feedback = motion_control_ptr_->getServoJoint();
    joint_feedback_.data1.data = 1;
    joint_feedback_.data2.data_count = 9;
    joint_feedback_.data2.data[0] = joint_feedback.j1;
    joint_feedback_.data2.data[1] = joint_feedback.j2;
    joint_feedback_.data2.data[2] = joint_feedback.j3;
    joint_feedback_.data2.data[3] = joint_feedback.j4;
    joint_feedback_.data2.data[4] = joint_feedback.j5;
    joint_feedback_.data2.data[5] = joint_feedback.j6;
    joint_feedback_.data2.data[6] = joint_feedback.j7;
    joint_feedback_.data2.data[7] = joint_feedback.j8;
    joint_feedback_.data2.data[8] = joint_feedback.j9;
}

void ControllerPublish::updateAxisGroupTcpWorldCartesian()
{
    tcp_world_cartesian_.data1.data = 1;
    tcp_world_cartesian_.data2.data_count = 6;
    tcp_world_cartesian_.data2.data[0] = 111;
    tcp_world_cartesian_.data2.data[1] = 222;
    tcp_world_cartesian_.data2.data[2] = 333;
    tcp_world_cartesian_.data2.data[3] = 1.11;
    tcp_world_cartesian_.data2.data[4] = 2.22;
    tcp_world_cartesian_.data2.data[5] = 3.33;
}

void ControllerPublish::updateAxisGroupTcpBaseCartesian()
{
    tcp_base_cartesian_.data1.data = 1;
    tcp_base_cartesian_.data2.data_count = 6;
    tcp_base_cartesian_.data2.data[0] = 111;
    tcp_base_cartesian_.data2.data[1] = 222;
    tcp_base_cartesian_.data2.data[2] = 333;
    tcp_base_cartesian_.data2.data[3] = 1.11;
    tcp_base_cartesian_.data2.data[4] = 2.22;
    tcp_base_cartesian_.data2.data[5] = 3.33;
}

void ControllerPublish::updateAxisGroupTcpCurrentCartesian()
{
    PoseEuler pos_feedback = motion_control_ptr_->getCurrentPose();
    tcp_current_cartesian_.data1.data = 1;
    tcp_current_cartesian_.data2.data_count = 6;
    tcp_current_cartesian_.data2.data[0] = pos_feedback.position.x;
    tcp_current_cartesian_.data2.data[1] = pos_feedback.position.y;
    tcp_current_cartesian_.data2.data[2] = pos_feedback.position.z;
    tcp_current_cartesian_.data2.data[3] = pos_feedback.orientation.a;
    tcp_current_cartesian_.data2.data[4] = pos_feedback.orientation.b;
    tcp_current_cartesian_.data2.data[5] = pos_feedback.orientation.c;
}

void ControllerPublish::updateAxisGroupCurrentCoordinate()
{
    current_coordinate_.data_count = 3;
    current_coordinate_.data[0] = 1;
    current_coordinate_.data[1] = (int32_t)motion_control_ptr_->getManualFrame();
    motion_control_ptr_->getUserFrame(current_coordinate_.data[2]);
}

void ControllerPublish::updateAxisGroupCurrentTool()
{
    current_tool_.data_count = 2;
    current_tool_.data[0] = 1;
    motion_control_ptr_->getToolFrame(current_tool_.data[1]);
}

void ControllerPublish::updateGlobalVelRatio()
{
    global_vel_ratio_.data = motion_control_ptr_->getGlobalVelRatio();
}

void ControllerPublish::updateGlobalAccRatio()
{
    global_acc_ratio_.data = motion_control_ptr_->getGlobalAccRatio();
}

void ControllerPublish::updateProgramStatus()
{
    InterpreterPublish* data_ptr = controller_client_ptr_->getInterpreterPublishPtr();
    memcpy(&program_status_.data1.data[0], data_ptr->program_name, 256);
    program_status_.data2.data = data_ptr->current_line_num;
}

void ControllerPublish::updateTpProgramStatus()
{
    InterpreterPublish* data_ptr = controller_client_ptr_->getInterpreterPublishPtr();
    tp_program_status_.data_count = 2;
    memcpy(&tp_program_status_.data[0].data[0], data_ptr->program_name, 256);
    memcpy(&tp_program_status_.data[1].data[0], data_ptr->current_line_path, 256);
}


void ControllerPublish::updateSafetyBoardStatus()
{
    //todo new
    uint32_t data = safety_device_ptr_->getDIFrm1();  //old board is getDIfrm2; new board is getDiFrm1
    memcpy(&safety_board_status_.data, &data, sizeof(data));
}

void ControllerPublish::updateIoBoardStatus()
{
    std::vector<fst_hal::IODeviceInfo> info_list = io_manager_ptr_->getIODeviceInfoList();
    fst_hal::IODevicePortValues values;

    io_board_status_.io_board_count = 4;
    for (int i = 0; i < io_board_status_.io_board_count; ++i)
    {
        if (info_list[i].dev_type == DEVICE_TYPE_FST_IO)
        {
            ErrorCode ret = io_manager_ptr_->getDevicePortValues(info_list[i].address, values);
            if (ret == SUCCESS)
            {
                io_board_status_.io_board[i].id = info_list[i].address;
                memcpy(&io_board_status_.io_board[i].DI, &values.DI, sizeof(uint32_t));
                memcpy(&io_board_status_.io_board[i].DO, &values.DO, sizeof(uint32_t));
                memcpy(&io_board_status_.io_board[i].RI, &values.RI, sizeof(uint8_t));
                memcpy(&io_board_status_.io_board[i].RO, &values.RO, sizeof(uint8_t));
                io_board_status_.io_board[i].valid = info_list[i].is_valid;
            }
            else
            {
                io_board_status_.io_board[i].valid = 0;
            }
        }
        else
        {
            io_board_status_.io_board[i].valid = 0;
        }
    }

}

void ControllerPublish::updateReg()
{
    void* value_ptr;
    std::list<RegPublishUpdate>::iterator it;
    for(it = reg_update_list_.begin(); it != reg_update_list_.end(); ++it)
    {
        switch(it->reg_type)
        {
            case REG_TYPE_PR:
            {
                value_ptr = reg_manager_ptr_->getPrRegValueById(it->reg_index);
                if(value_ptr != NULL)
                {
                    PrValue* pr_value_ptr = static_cast<PrValue*>(value_ptr);
                    it->pr_value.is_valid = true;
                    it->pr_value.group_id = pr_value_ptr->group_id;
                    it->pr_value.pos_type = pr_value_ptr->pos_type;
                    memcpy(&it->pr_value.pos.data[0], &pr_value_ptr->pos[0], 9*sizeof(double));
                    memcpy(&it->pr_value.posture.data[0], &pr_value_ptr->posture[0], 4*sizeof(bool));
                }
                else
                {
                    it->pr_value.is_valid = false;
                }
                it->pr_value.pos.data_count = 9;
                it->pr_value.posture.data_count = 4;
                break;               
            }
            case REG_TYPE_HR:
            {
                value_ptr = reg_manager_ptr_->getHrRegValueById(it->reg_index);
                if(value_ptr != NULL)
                {
                    HrValue* hr_value_ptr = static_cast<HrValue*>(value_ptr);
                    it->hr_value.is_valid = true;
                    it->hr_value.group_id = hr_value_ptr->group_id;
                    memcpy(&it->hr_value.joint_pos.data[0], &hr_value_ptr->joint_pos[0], 9*sizeof(double));
                    memcpy(&it->hr_value.diff_pos.data[0], &hr_value_ptr->diff_pos[0], 9*sizeof(double));
                }
                else
                {
                    it->hr_value.is_valid = false;
                }
                it->hr_value.joint_pos.data_count = 9;
                it->hr_value.diff_pos.data_count = 9;
                break;               
            }                
            case REG_TYPE_SR:
            {
                value_ptr = reg_manager_ptr_->getSrRegValueById(it->reg_index);
                if(value_ptr != NULL)
                {
                    char* sr_value_ptr = static_cast<char*>(value_ptr);
                    it->sr_value.is_valid = true;
                    unsigned int str_length = strlen(sr_value_ptr);
                    memcpy(&it->sr_value.data[0], sr_value_ptr, str_length);
                    it->sr_value.data[str_length] = 0;
                }
                else
                {
                    it->sr_value.is_valid = false;
                }
                break;               
            }
            case REG_TYPE_MR:
            {
                value_ptr = reg_manager_ptr_->getMrRegValueById(it->reg_index);
                if(value_ptr != NULL)
                {
                    int* mr_value_ptr = static_cast<int*>(value_ptr);
                    it->mr_value.is_valid = true;
                    it->mr_value.data = *mr_value_ptr;
                }
                else
                {
                    it->mr_value.is_valid = false;
                }
                break;               
            }
            case REG_TYPE_R:
            {
                value_ptr = reg_manager_ptr_->getRRegValueById(it->reg_index);
                if(value_ptr != NULL)
                {
                    double* r_value_ptr = static_cast<double*>(value_ptr);
                    it->r_value.is_valid = true;
                    it->r_value.data = *r_value_ptr;
                }
                else
                {
                    it->r_value.is_valid = false;
                }
                break;               
            }                
        }
    }
}

void ControllerPublish::updateIo()
{
    ErrorCode ret = SUCCESS;
    std::list<IoPublishUpdate>::iterator it;
    for(it = io_update_list_.begin(); it != io_update_list_.end(); ++it)
    {
        uint8_t value = 0;
        switch(it->port_type)
        {
            case MessageType_IoType_DI://fst_hal::IO_TYPE_DI:
            {
                ret = io_mapping_ptr_->getDIByBit(it->port_offset, value);
                if(ret == SUCCESS)
                {
                    it->is_valid = true;
                    it->value.data = static_cast<uint32_t>(value);
                }
                else
                {
                    it->is_valid = false;
                }
                break;
            }
            case MessageType_IoType_DO://fst_hal::IO_TYPE_DO:
            {
                ret = io_mapping_ptr_->getDOByBit(it->port_offset, value);
                if(ret == SUCCESS)
                {
                    it->is_valid = true;
                    it->value.data = static_cast<uint32_t>(value);
                }
                else
                {
                    it->is_valid = false;
                }
                break;
            }
            case MessageType_IoType_RI://fst_hal::IO_TYPE_RI:
            {
                ret = io_mapping_ptr_->getRIByBit(it->port_offset, value);
                if(ret == SUCCESS)
                {
                    it->is_valid = true;
                    it->value.data = static_cast<uint32_t>(value);
                }
                else
                {
                    it->is_valid = false;
                }
                break;
            }
            case MessageType_IoType_RO://fst_hal::IO_TYPE_RO:
            {
                ret = io_mapping_ptr_->getROByBit(it->port_offset, value);
                if(ret == SUCCESS)
                {
                    it->is_valid = true;
                    it->value.data = static_cast<uint32_t>(value);
                }
                else
                {
                    it->is_valid = false;
                }
                break;
            }
            case MessageType_IoType_UI://fst_hal::IO_TYPE_DI:
            {
                ret = io_mapping_ptr_->getUIByBit(it->port_offset, value);
                if(ret == SUCCESS)
                {
                    it->is_valid = true;
                    it->value.data = static_cast<uint32_t>(value);
                }
                else
                {
                    it->is_valid = false;
                }
                break;
            }
            case MessageType_IoType_UO://fst_hal::IO_TYPE_DO:
            {
                ret = io_mapping_ptr_->getUOByBit(it->port_offset, value);
                if(ret == SUCCESS)
                {
                    it->is_valid = true;
                    it->value.data = static_cast<uint32_t>(value);
                }
                else
                {
                    it->is_valid = false;
                }
                break;
            }
        }
    }
}

void ControllerPublish::updateModbusClientCtrlStatus()
{
    modbus_client_ctrl_status_ = MessageType_ModbusClientCtrlStatusList_init_default;
    vector<int> id_list;
    id_list.clear();
    
    ErrorCode error_code = modbus_manager_ptr_->getClientIdList(id_list);

    if (error_code == SUCCESS)
    {
        vector<int>::iterator it = id_list.begin();
        modbus_client_ctrl_status_.ctrl_status_count = id_list.size();

        for (int i = 0; i != id_list.size(); ++i)
        {
            modbus_client_ctrl_status_.ctrl_status[i].id = *it;
            error_code = modbus_manager_ptr_->getClientCtrlState(*it, modbus_client_ctrl_status_.ctrl_status[i].status);
            it++;
        }
    }
}
