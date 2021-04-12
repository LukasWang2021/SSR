#ifndef SERVO_PARAM_1000_H
#define SERVO_PARAM_1000_H

/**
 * @file servo_param_1000.h
 * @brief The file defines the class ServoParam1000.
 * @author zhengyu.shen
 */

#include "servo_base.h"
#include "model_base.h"
#include <string>
/**
 * @brief system_model_space includes all system model related implementation.
 */
namespace system_model_space
{
/**
 * @brief Defines parameters for servo_1000 model. Up to 512 parameters is supported.
 */
typedef enum
{
    Servo1000__number = 512,
}ServoParam1000_e;
/**
 * @brief Defines the parameter section for power/encoder/motor in servo parameter list. 
 */
typedef enum
{
    Servo1000_PowerParamNumber = 6,     /**< Number of power parameters.*/
    Servo1000_PowerParamStart = 49,     /**< Index of the first power parameter.*/
    Servo1000_EncoderParamNumber = 9,   /**< Number of encoder parameters.*/
    Servo1000_EncoderParamStart = 65,   /**< Index of the first encoder parameter.*/
    Servo1000_MotorParamNumber = 17,    /**< Number of motor parameters.*/
    Servo1000_MotorParamStart = 81,     /**< Index of the first motor parameter.*/
}ServoParam1000_ParamSearch_e;
/**
 * @brief Defines the servo_1000 model.
 */
class ServoParam1000 : public ServoBase
{
public:
    /**
     * @brief Constructor of the class.
     * @param [in] style_str Model style.
     * @param [in] file_path The absolute file path of the yaml file.
     */    
    ServoParam1000(std::string style_str, std::string file_path);
    /**
     * @brief Destructor of the class.
     */
    ~ServoParam1000();
    
private:
};

}


#endif

