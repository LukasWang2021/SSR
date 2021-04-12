#ifndef GROUP_APPLICATION_DB_H
#define GROUP_APPLICATION_DB_H

/**
 * @file group_application_db.h
 * @brief The file defines the class handling data base of group application.
 * @author zhengyu.shen
 */

#include "model_base.h"
#include <string>
/**
 * @brief system_model_space includes all system model related implementation.
 */
namespace system_model_space
{
/**
 * @brief Defines the data base of group application model.
 */
class GroupApplicationDb
{
public:
    /**
     * @brief Constructor of the class.
     */    
    GroupApplicationDb();
    /**
     * @brief Destructor of the class.
     */    
    ~GroupApplicationDb();
    /**
     * @brief Create a model object by specified name and configuration yaml file.
     * @details The returned model object pointer should be freed after use.\n
     * @param [in] model_name Model name is extracted from groups_config.xml in format "name.style".
     * @param [in] file_path The absolute file path of the yaml file.
     * @return Pointer of the model object.
     */
    static ModelBase* newModel(std::string model_name, std::string file_path);
private:
    
};

}

#endif

