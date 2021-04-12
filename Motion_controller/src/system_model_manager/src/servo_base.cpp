#include "servo_base.h"

using namespace system_model_space;
using namespace base_space;


ServoBase::ServoBase(std::string style, std::string file_path, uint32_t param_number,
                        uint32_t power_param_number, uint32_t power_param_start,
                        uint32_t encoder_param_number, uint32_t encoder_param_start,
                        uint32_t motor_param_number, uint32_t motor_param_start):
    style_(style), file_path_(file_path), param_number_(param_number),
    power_param_number_(power_param_number), power_param_start_(power_param_start),
    encoder_param_number_(encoder_param_number), encoder_param_start_(encoder_param_start),
    motor_param_number_(motor_param_number), motor_param_start_(motor_param_start)
{

}

ServoBase::~ServoBase()
{

}

bool ServoBase::load()
{
    if(!yaml_help_.loadParamFile(file_path_))
    {
        return false;
    }

    param_.clear();
    if(!yaml_help_.getParam("servo_param", param_))
    {
        return false;
    }

    if(param_.size() != param_number_)
    {
        return false;
    }
    return true;
}

bool ServoBase::save()
{
    if(!yaml_help_.setParam("servo_param", param_))
    {
        return false;
    }

    if(!yaml_help_.dumpParamFile(file_path_))
    {
        return false;
    }
    return true;
}

std::string ServoBase::getStyle()
{
    return style_;
}

bool ServoBase::set(const uint32_t param_index, const int32_t param_value)
{
    // param_value is not checked here
    if(param_index < param_number_)
    {
        param_[param_index] = param_value;
        return true;
    }
    else
    {
        return false;
    }
}

bool ServoBase::get(const uint32_t param_index, int32_t* param_value_ptr)
{
    if(param_index < param_number_)
    {
        *param_value_ptr = param_[param_index];
        return true;
    }
    else
    {
        return false;
    }
}

bool ServoBase::syncPowerToServo(ModelBase* model_ptr)
{
    if(model_ptr->getType() != MODEL_TYPE_POWER)
    {
        return false;
    }

    uint32_t param_numer = model_ptr->getParamNumber();
    if(param_numer != power_param_number_)
    {
        return false;
    }
    int32_t value;
    size_t power_param_start = power_param_start_;
    for(size_t i = 0; i < param_numer; ++i)
    {
        if(model_ptr->get(i, &value))
        {
            set(power_param_start, value);
            ++power_param_start;
        }
        else
        {
            return false;
        }
    }
    return true;
}

bool ServoBase::syncEncoderToServo(ModelBase* model_ptr)
{
    if(model_ptr->getType() != MODEL_TYPE_ENCODER)
    {
        return false;
    }

    uint32_t param_numer = model_ptr->getParamNumber();
    if(param_numer != encoder_param_number_)
    {
        return false;
    }
    int32_t value;
    size_t param_offset = encoder_param_start_;
    for(size_t i = 0; i < param_numer; ++i)
    {
        if(model_ptr->get(i, &value))
        {
            set(param_offset, value);
            ++param_offset;
        }
        else
        {
            return false;
        }
    }
    return true;
}

bool ServoBase::syncMotorToServo(ModelBase* model_ptr)
{
    if(model_ptr->getType() != MODEL_TYPE_MOTOR)
    {
        return false;
    }

    uint32_t param_numer = model_ptr->getParamNumber();
    if(param_numer != motor_param_number_)
    {
        return false;
    }
    int32_t value;
    size_t param_offset = motor_param_start_;
    for(size_t i = 0; i < param_numer; ++i)
    {
        if(model_ptr->get(i, &value))
        {
            set(param_offset, value);
            ++param_offset;
        }
        else
        {
            return false;
        }
    }
    return true;
}

