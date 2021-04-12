#include "servo_param_1000.h"

using namespace system_model_space;
using namespace base_space;


ServoParam1000::ServoParam1000(std::string style_str, std::string file_path):
    ServoBase(style_str, file_path, Servo1000__number,
                Servo1000_PowerParamNumber, Servo1000_PowerParamStart,
                Servo1000_EncoderParamNumber, Servo1000_EncoderParamStart,
                Servo1000_MotorParamNumber, Servo1000_MotorParamStart)
{

}

ServoParam1000::~ServoParam1000()
{

}


