#ifndef GROUP_ALGORITHM_PARAM_1000_H
#define GROUP_ALGORITHM_PARAM_1000_H

/**
 * @file group_algorithm_param_1000.h
 * @brief The file defines the class GroupAlgorithmParam1000.
 * @author zhengyu.shen
 */

#include "model_base.h"
/**
 * @brief system_model_space includes all system model related implementation.
 */
namespace system_model_space
{
/**
 * @brief Defines parameters for group_algorithm_1000 model.
 */
typedef enum
{
    GroupAlgorithm1000__model_id = 0,
    GroupAlgorithm1000__sampling_interval = 1,
    GroupAlgorithm1000__dof = 2,
    GroupAlgorithm1000__number = 3,
}GroupAlgorithmParam1000_e;
/**
 * @brief Defines the group_algorithm_1000 model.
 */
class GroupAlgorithmParam1000 : public ModelBase
{
public:
    /**
     * @brief Constructor of the class.
     * @param [in] style_str Model style.
     * @param [in] file_path The absolute file path of the yaml file.
     */    
    GroupAlgorithmParam1000(std::string style_str, std::string file_path);
    /**
     * @brief Destructor of the class.
     */    
    ~GroupAlgorithmParam1000();
    
private:
    ParamDetail_t param_[GroupAlgorithm1000__number];
};

}


#endif

