#include "controller_publish.h"

using namespace fst_ctrl;
using namespace fst_mc;

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
    tcp_current_cartesian_.data1.data = 1;
    tcp_current_cartesian_.data2.data_count = 6;
    tcp_current_cartesian_.data2.data[0] = 111;
    tcp_current_cartesian_.data2.data[1] = 222;
    tcp_current_cartesian_.data2.data[2] = 333;
    tcp_current_cartesian_.data2.data[3] = 1.11;
    tcp_current_cartesian_.data2.data[4] = 2.22;
    tcp_current_cartesian_.data2.data[5] = 3.33;
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
                    memcpy(&it->sr_value.data[0], &sr_value_ptr, str_length);
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


