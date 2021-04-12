#ifndef SERVO_MODEL_BASE_H
#define SERVO_MODEL_BASE_H

/**
 * @file servo_base.h
 * @brief The file defines the class ServoBase.
 * @author zhengyu.shen
 */

#include "yaml_help.h"
#include "model_base.h"
#include <stdint.h>
#include <string>
#include <vector>
/**
 * @brief system_model_space includes all system model related implementation.
 */
namespace system_model_space
{
class ServoBase
{
public:
    /**
     * @brief Constructor of the class.
     * @param [in] style_str Model style.
     * @param [in] file_path The absolute file path of the yaml file.
     * @param [in] param_number Total number of servo parameters.
     * @param [in] power_param_number Number of power parameters.
     * @param [in] power_param_start Index of the first power parameter.
     * @param [in] encoder_param_number Number of encoder parameters.
     * @param [in] encoder_param_start Index of the first encoder parameter.
     * @param [in] motor_param_number Number of motor parameters.
     * @param [in] motor_param_start Index of the first motor parameter.
     */       
    ServoBase(std::string style_str, std::string file_path, uint32_t param_number,
                    uint32_t power_param_number, uint32_t power_param_start,
                    uint32_t encoder_param_number, uint32_t encoder_param_start,
                    uint32_t motor_param_number, uint32_t motor_param_start);
    /**
     * @brief Destructor of the class.
     */
    ~ServoBase();
    /**
     * @brief Load yaml file to local memory.
     * @retval true Operation succeed.
     * @retval false Operation failed.
     */
    bool load();
    /**
     * @brief Save parameters to yaml file.
     * @retval true Operation succeed.
     * @retval false Operation failed.
     */       
    bool save();
    /**
     * @brief Get servo style.
     * @details A servo model is expressed in format "Name.Style". "Name" is servo name and "Style" is servo style.\n
     * @return Servo style.
     */     
    std::string getStyle();
    /**
     * @brief Set the operation value of a parameter.
     * @param [in] param_index Parameter index.
     * @param [in] param_value Parameter value.
     * @retval true Operation succeed.
     * @retval false Operation failed.
     */
    bool set(const uint32_t param_index, const int32_t param_value);
    /**
     * @brief Get the operation value of a parameter.
     * @param [in] param_index Parameter index.
     * @param [out] param_value_ptr Pointer of parameter value.
     * @retval true Operation succeed.
     * @retval false Operation failed.
     */ 
    bool get(const uint32_t param_index, int32_t* param_value_ptr);
    /**
     * @brief Synchronize the paramters of power model to servo parameters.
     * @param [in] model_ptr Pointer of power model.
     * @retval true Operation succeed.
     * @retval false Operation failed.
     */ 
    bool syncPowerToServo(ModelBase* model_ptr);
    /**
     * @brief Synchronize the paramters of encoder model to servo parameters.
     * @param [in] model_ptr Pointer of encoder model.
     * @retval true Operation succeed.
     * @retval false Operation failed.
     */
    bool syncEncoderToServo(ModelBase* model_ptr);
    /**
     * @brief Synchronize the paramters of motor model to servo parameters.
     * @param [in] model_ptr Pointer of motor model.
     * @retval true Operation succeed.
     * @retval false Operation failed.
     */
    bool syncMotorToServo(ModelBase* model_ptr);
        
private:
    std::string style_;
    std::string file_path_;
    uint32_t param_number_;
    base_space::YamlHelp yaml_help_;  
    std::vector<int32_t> param_;
    uint32_t power_param_number_;
    uint32_t power_param_start_;
    uint32_t encoder_param_number_;
    uint32_t encoder_param_start_;
    uint32_t motor_param_number_;
    uint32_t motor_param_start_;    
};

}


#endif

