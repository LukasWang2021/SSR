#include "servo_param_1001.h"

using namespace system_model_space;
using namespace base_space;


ServoParam1001::ServoParam1001(std::string style_str, std::string file_path):
    ServoBase(style_str, file_path, Servo1001__number,
                Servo1001_PowerParamNumber, Servo1001_PowerParamStart,
                Servo1001_EncoderParamNumber, Servo1001_EncoderParamStart,
                Servo1001_MotorParamNumber, Servo1001_MotorParamStart)
{

}

ServoParam1001::~ServoParam1001()
{

}


