#ifndef MODEL_BASE_H
#define MODEL_BASE_H

/**
 * @file model_base.h
 * @brief The file defines the class ModelBase.
 * @author zhengyu.shen
 */

#include "common_datatype.h"
#include "yaml_help.h"
#include <string>
#include <vector>
/**
 * @brief system_model_space includes all system model related implementation.
 */
namespace system_model_space
{
/**
 * @brief Defines model type.
 */
typedef enum
{
    MODEL_TYPE_POWER = 0,               /**< Model name is like power_XXX.YYY.*/
    MODEL_TYPE_ENCODER = 1,             /**< Model name is like encoder_XXX.YYY.*/
    MODEL_TYPE_MOTOR = 2,               /**< Model name is like motor_XXX.YYY.*/
    MODEL_TYPE_AXIS_MODEL = 3,          /**< Model name is like axis_model_XXX.YYY.*/
    MODEL_TYPE_AXIS_APPLICATION = 4,    /**< Model name is like axis_application_XXX.YYY.*/
    MODEL_TYPE_AXIS_ALGORITHM = 5,      /**< Model name is like axis_algorithm_XXX.YYY.*/
    MODEL_TYPE_GROUP_KINEMATICS = 6,    /**< Model name is like group_kinematics_XXX.YYY.*/
    MODEL_TYPE_GROUP_DYNAMICS = 7,      /**< Model name is like group_dynamics_XXX.YYY.*/
    MODEL_TYPE_GROUP_APPLICATION = 8,   /**< Model name is like group_application_XXX.YYY.*/
    MODEL_TYPE_GROUP_ALGORITHM = 9,     /**< Model name is like group_algorithm_XXX.YYY.*/
}ModelType_e;
/**
 * @brief Data struct to mapping parameters in pair of name and index.
 */
typedef struct
{
    std::string param_name; /**< Parameter name.*/
    uint32_t param_index;   /**< Parameter index.*/
}ModelParamMap_t;
/**
 * @brief Base class for all models except servo_model.
 */
class ModelBase
{
public:
    /**
     * @brief Constructor of the class.
     * @param [in] type Model type.
     * @param [in] style_str Model style.
     * @param [in] file_path The absolute file path of the yaml file.
     * @param [in] param_ptr Pointer of the model parameter list.
     * @param [in] param_number Number of the model parameters.
     */    
    ModelBase(ModelType_e type, std::string style_str, std::string file_path, ParamDetail_t* param_ptr, uint32_t param_number);
    /**
     * @brief Destructor of the class.
     */
    ~ModelBase();
    /**
     * @brief Map a parameter with its name and its index in the parameter list.
     * @param [in] param_name Parameter name.
     * @param [in] param_index Parameter index in the list.
     * @return void
     */
    void addParam(const std::string& param_name, const uint32_t param_index);
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
     * @brief Get model type.
     * @return Model type.
     */    
    ModelType_e getType();
    /**
     * @brief Get model name.
     * @details A model is expressed in format "Name.Style". "Name" is model name and "Style" is model style.\n
     * @return Model name.
     */      
    std::string getName();
    /**
     * @brief Get model style.
     * @details A model is expressed in format "Name.Style". "Name" is model name and "Style" is model style.\n
     * @return Model style.
     */     
    std::string getStyle();
    /**
     * @brief Set model name.
     * @details A model is expressed in format "Name.Style". "Name" is model name and "Style" is model style.\n
     * @param [in] name Model name.
     * @return void.
     */ 
    void setName(const std::string& name);
    /**
     * @brief Get the total number of model parameters.
     * @return Parameter number.
     */    
    uint32_t getParamNumber();
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
     * @brief Get details of a parameter.
     * @param [in] param_index Parameter index.
     * @param [out] param_detail_ptr Pointer of parameter details.
     * @retval true Operation succeed.
     * @retval false Operation failed.
     */    
    bool getDetail(const uint32_t param_index, ParamDetail_t* param_detail_ptr);
    /**
     * @brief Get details of all parameters.
     * @return Pointer of the parameter detail list.
     */ 
    ParamDetail_t* getAll();

private:
    ModelType_e type_;
    std::string name_;
    std::string style_;
    std::string file_path_;
    ParamDetail_t* param_ptr_;
    uint32_t param_number_;
    std::vector<ModelParamMap_t> param_map_;
    base_space::YamlHelp yaml_help_;  

    void formatUnitStr2Char(const std::string& unit_str, char* unit_char16);
    void formatUnitChar2Str(const char* unit_char16, std::string& unit_str);
};

}


#endif

