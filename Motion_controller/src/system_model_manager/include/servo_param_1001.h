#ifndef SERVO_PARAM_1001_H
#define SERVO_PARAM_1001_H

/**
 * @file servo_param_1001.h
 * @brief The file defines the class ServoParam1000.
 * @author feng.wu
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
 * @brief Defines parameters for servo_1001 model. Up to 512 parameters is supported.
 */
typedef enum
{
    Servo1001__number = 512,
}ServoParam1001_e;
/**
 * @brief Defines the parameter section for power/encoder/motor in servo parameter list. 
 */
typedef enum
{
    Servo1001_PowerParamNumber = 6,     /**< Number of power parameters.*/
    Servo1001_PowerParamStart = 49,     /**< Index of the first power parameter.*/
    Servo1001_EncoderParamNumber = 9,   /**< Number of encoder parameters.*/
    Servo1001_EncoderParamStart = 65,   /**< Index of the first encoder parameter.*/
    Servo1001_MotorParamNumber = 17,    /**< Number of motor parameters.*/
    Servo1001_MotorParamStart = 81,     /**< Index of the first motor parameter.*/
}ServoParam1001_ParamSearch_e;
/**
 * @brief Defines the servo_1000 model.
 */
class ServoParam1001 : public ServoBase
{
public:
    /**
     * @brief Constructor of the class.
     * @param [in] style_str Model style.
     * @param [in] file_path The absolute file path of the yaml file.
     */    
    ServoParam1001(std::string style_str, std::string file_path);
    /**
     * @brief Destructor of the class.
     */
    ~ServoParam1001();
    
private:
};

}


#endif

