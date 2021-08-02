#ifndef FORCE_PARAM_H
#define FORCE_PARAM_H

/**
 * @file force_param.h
 * @brief The file defines the class ForceParam.
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
 * @brief Defines parameters for servo_1000 model. Up to 32 parameters is supported.
 */
typedef enum
{
    Force_param_number = 512,
}ForceParam_e;

/**
 * @brief Defines the stator parameter model.
 */
class ForceParam : public ServoBase
{
public:
    /**
     * @brief Constructor of the class.
     * @param [in] style_str Model style.
     * @param [in] file_path The absolute file path of the yaml file.
     */    
    ForceParam(std::string style_str, std::string file_path);
    /**
     * @brief Destructor of the class.
     */
    ~ForceParam();
    
private:
};

}


#endif

