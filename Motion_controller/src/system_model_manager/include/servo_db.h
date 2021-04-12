#ifndef SERVO_DB_H
#define SERVO_DB_H

/**
 * @file servo_db.h
 * @brief The file defines the class handling data base of servo.
 * @author zhengyu.shen
 */

#include "servo_base.h"
#include <string>
/**
 * @brief system_model_space includes all system model related implementation.
 */
namespace system_model_space
{
/**
 * @brief Defines the data base of servo model.
 */
class ServoDb
{
public:
    /**
     * @brief Constructor of the class.
     */    
    ServoDb();
    /**
     * @brief Destructor of the class.
     */
    ~ServoDb();
    /**
     * @brief Create a model object by specified name and configuration yaml file.
     * @details The returned model object pointer should be freed after use.\n
     * @param [in] model_name Model name is extracted from axes_config.xml in format "name.style".
     * @param [in] file_path The absolute file path of the yaml file.
     * @return Pointer of the model object.
     */
    static ServoBase* newModel(std::string model_name, std::string file_path);
private:
    
};

}

#endif

