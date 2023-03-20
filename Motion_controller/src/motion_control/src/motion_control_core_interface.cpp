/*************************************************************************
	> File Name: motion_control_core_interface.cpp
	> Author: 
	> Mail: 
	> Created Time: 2018年08月08日 星期三 15时35分37秒
 ************************************************************************/

#include <iostream>
#include <motion_control_core_interface.h>

using namespace std;
using namespace basic_alg;
using namespace axis_space;
using namespace group_space;
using namespace log_space;
using namespace servo_comm_space;
using namespace system_model_space;

namespace group_space
{


BareCoreInterface::BareCoreInterface(void)
{
    joint_num_ = 0;
    point_cache_.is_empty = true;
    sync_index_ = 0;
}

BareCoreInterface::~BareCoreInterface(void)
{}

bool BareCoreInterface::initInterface(uint32_t joint_num, std::map<int32_t, axis_space::Axis*>* axis_group_ptr, GroupSm* sm_ptr,
    servo_comm_space::ServoCpuCommBase* cpu_comm_ptr, system_model_space::GroupModel_t* db_ptr)
{
    joint_num_ = joint_num;
    axis_group_ptr_ = axis_group_ptr;
    sm_ptr_ = sm_ptr;
    cpu_comm_ptr_ = cpu_comm_ptr;
    db_ptr_ = db_ptr;
    if (joint_num_ != axis_group_ptr->size())
        return false;

    //compute coupling factor from 5 to 6 axis.
    int32_t coupling_numerator_5to6 = 0;
    if (!db_ptr_->application_ptr->get(GroupApplication1000__coupling_numerator_5to6, &coupling_numerator_5to6))
		return false;
    int32_t coupling_denominator_5to6 = 0;
    if (!db_ptr_->application_ptr->get(GroupApplication1000__coupling_denominator_5to6, &coupling_denominator_5to6))
		return false;
    int32_t coupling_direction_5to6 = 0;
    if (!db_ptr_->application_ptr->get(GroupApplication1000__coupling_direction_5to6, &coupling_direction_5to6))
		return false;
    coupling_factor_ = ((double)coupling_numerator_5to6) / coupling_denominator_5to6;

    // 0: positive effect; 1: negative effect
    if (0 == coupling_direction_5to6)
    {
        // use opposite to compensate the positive effect of axis4 on axis3.
        coupling_factor_ = -coupling_factor_;
    }
    LogProducer::info("mc_core", "group coupling_factor_ is %lf", coupling_factor_);
    

    //setting pdo_sync index
    std::map<int, Axis*>::iterator it;
    int32_t param_value = 0;
    for (it = axis_group_ptr_->begin(); it != axis_group_ptr_->end(); ++it)
    {
        if (it->second->mcReadParamter(SERVO_PARAM_CTRL_PDO_SYNC_INDEX, param_value) == SUCCESS)
        {
            if (it == axis_group_ptr_->begin())
            {
                sync_index_ = param_value;
                continue;
            }
            if (sync_index_ != param_value)
            {
                LogProducer::error("mc_core", "mc_group has different pdo_sync index(%d) for axis[%d].", 
                    param_value, it->second->getID());
            }
        }
        else
        {
            LogProducer::error("mc_core", "mc_group failed to read pdo_sync index for axis[%d].", it->second->getID());
        }
    }

    return true;
}

bool BareCoreInterface::isPointCacheEmpty(void)
{
    return point_cache_.is_empty;
}

bool BareCoreInterface::clearPointCache(void)
{
    point_cache_.is_empty = true;
    std::map<int, axis_space::Axis*>::iterator it;
    int32_t i = 0;
    for (it = axis_group_ptr_->begin(), i = 0; it != axis_group_ptr_->end(); ++it, ++i)
    {
        it->second->getServoCommPtr()->clearCtrlPdoBuffer();
    }
    return true;
}

bool BareCoreInterface::fillPointCache(TrajectoryPoint *points, size_t length, PointProperty property)
{
    if(!point_cache_.is_empty || length <= 0 || length > JC_POINT_NUM)
    {
        return false;
    }

    std::map<int, Axis*>::iterator it;
    int32_t i = 0;
    double coupling_pos = 0;
    for (it = axis_group_ptr_->begin(), i = 0; it != axis_group_ptr_->end(); ++it, ++i)
    {
        for (unsigned int j = 0; j < length; ++j)
        {
            if (INDEX_JOINT6 == i)
            {
                coupling_pos = couplingAxis5To6ByRad(points[j].state.angle[i - 1], points[j].state.angle[i]);
                point_cache_.axis[i].set_point[j].cmd_position = it->second->getAxisConvPtr()->convertPosA2M(coupling_pos);
            }
            else
            {
                point_cache_.axis[i].set_point[j].cmd_position = it->second->getAxisConvPtr()->convertPosA2M(points[j].state.angle[i]);
            }
            point_cache_.axis[i].set_point[j].feedforward_velocity = it->second->getAxisConvPtr()->convertVelA2M(points[j].state.omega[i]);
            point_cache_.axis[i].set_point[j].feedforward_acc = it->second->getAxisConvPtr()->convertAccA2M(points[j].state.alpha[i]);
            // LogProducer::info("fillPointCache","joint%d-%d position=%d,velocity=%d",i,j,point_cache_.axis[i].set_point[j].cmd_position, point_cache_.axis[i].set_point[j].feedforward_velocity);
        }
        point_cache_.axis[i].current_point = 0;
        point_cache_.axis[i].total_points = length;
        point_cache_.is_start = false;
        if(points[0].level == POINT_START)
        {
            point_cache_.is_start = true;
        }
    }

    point_cache_.is_empty = false;
    point_cache_.property = property;
    return true;
}

double BareCoreInterface::couplingAxis5To6ByRad(double fifth_pos, double sixth_pos)
{
    // Coupling: turn of axis5 outputs to axis6. The unit is rad.
    return sixth_pos + coupling_factor_*fifth_pos;
}

double BareCoreInterface::decouplingAxis6ByRad(double fifth_pos, double sixth_pos)
{
    return sixth_pos - coupling_factor_ * fifth_pos;
}

bool BareCoreInterface::sendPoint(void)
{
    if (!point_cache_.is_empty)
    {
        if (WriteShareMem(point_cache_, point_cache_.property))
        {
            point_cache_.is_empty = true;
            return true;
        }
        else
        {
            return false;
        }
    }
    else
    {
        return false;
    }

    point_cache_.is_empty = true;
    return true;
}


bool BareCoreInterface::WriteShareMem(PointCache& cache, unsigned int valid_level)
{
    //如果有新的轨迹，检查上一条轨迹是否已经执行完毕。
    if(cache.is_start)
    {
        //同步信号还是１，未被伺服置为０，说明上条轨迹的buffer数据未取完。
        if (cpu_comm_ptr_->getCtrlPdoSync(sync_index_) == 1)
        {
            LogProducer::warn("mc_core","WriteShareMem: Last trajectory is not finished.");
            return false;
        }
    }
    //填充各轴位置控制缓冲
    std::map<int, axis_space::Axis*>::iterator it;
    int32_t i = 0;
    bool result = true;
    //bool flag_actual_element_number_zero = false;
    for (it = axis_group_ptr_->begin(), i = 0; it != axis_group_ptr_->end(); ++it, ++i)
    {
        int32_t current_index = cache.axis[i].current_point;
        int32_t expect_element_number = cache.axis[i].total_points - current_index;
        int32_t actual_element_number = 0;
        it->second->getServoCommPtr()->processCtrlPdoBufferMode((uint8_t*)&cache.axis[i].set_point[current_index], expect_element_number, &actual_element_number);
        cache.axis[i].current_point += actual_element_number;
        if (cache.axis[i].current_point != cache.axis[i].total_points)
        {
            result = false;
        }

    }
    //如果是新轨迹，置各轴同步信号
    if (cache.is_start)
    {
        cpu_comm_ptr_->setCtrlPdoSync(sync_index_, 1);
        sm_ptr_->transferStateToGroupMoving();
        LogProducer::warn("mc_core", "WriteShareMem new trajectory, index=%d, setSync=1, getSync=%d", sync_index_, cpu_comm_ptr_->getCtrlPdoSync(sync_index_));
    } 
    return result;
}

bool BareCoreInterface::getLatestJoint(Joint &joint, uint32_t (&encoder_state)[NUM_OF_JOINT], ServoState &state)
{
    std::map<int32_t, Axis*>::iterator it;
    uint32_t i = 0;
    for (it = axis_group_ptr_->begin(); it != axis_group_ptr_->end(); ++it, ++i)
    {
        it->second->mcReadActualPosition(joint[i]);
        encoder_state[i] = it->second->rtmGetEncoderState();
    }
    state = (ServoState)sm_ptr_->getGroupStatus();
    
    return true;
}

ErrorCode BareCoreInterface::resetEncoderError(void)
{
    ErrorCode result = SUCCESS;
    std::map<int32_t, Axis*>::iterator it;
    for (it = axis_group_ptr_->begin(); it != axis_group_ptr_->end(); ++it)
    {
        result = it->second->rtmResetEncoder();
        if (result != SUCCESS)
            return result;
    }
    return result;
}

ErrorCode BareCoreInterface::setOffsetPositions(uint32_t index, double offset)
{
    if (index >= axis_group_ptr_->size())
    {
        LogProducer::error("mc_core", "setOffsetPositions, axis index(%d) is invalid.", index);
        return MC_LOAD_PARAM_FAILED;
    }

    std::map<int32_t, Axis*>::iterator it = axis_group_ptr_->find(index);
    if (it != axis_group_ptr_->end())
    {      
        return it->second->mcSetPosition();
    }

    LogProducer::error("mc_core", "setOffsetPositions, axis index(%d) not found in group.", index);
    return MC_LOAD_PARAM_FAILED;
}

}

