#ifndef TP_COMM_MANAGER_CONFIG
#define TP_COMM_MANAGER_CONFIG

/**
 * @file tp_comm_manager_config.h
 * @brief The file includes the class for handling configuration of the tp_comm module.
 * @author zhengyu.shen
 */

#include "yaml_help.h"
#include <string>
/**
 * @brief user_space includes all user-defined implementations.
 */
namespace user_space
{
/**
 * @brief TpCommManagerConfig is the object to handle configuration of the tp_comm module.
 */
class TpCommManagerConfig
{
public:
    /**
     * @brief Constructor of the class.
     */      
    TpCommManagerConfig();
    /**
     * @brief Destructor of the class.
     */    
    ~TpCommManagerConfig(){}
    /**
     * @brief Load configuration file of the tp_comm module.
     * @retval true Operation succeed.
     * @retval false Operation failed.
     */
    bool loadParam();
    /**
     * @brief Save configuration file of the tp_comm module.
     * @retval true Operation succeed.
     * @retval false Operation failed.
     */    
    bool saveParam();

    // param to load & save   
    int cycle_time_;        /**< Cycle time for the routine thread of the module, unit in ms.*/
    int recv_buffer_size_;  /**< Byte size of receiveing buffer.*/
    int send_buffer_size_;  /**< Byte size of sending buffer.*/
    int rpc_list_max_size_; /**< Maximum number of rpc that the requset and response lists can hold.*/
	int event_list_max_size_;   /**< Maximum number of event that the event list can hold.*/

private:
    base_space::YamlHelp yaml_help_;
    std::string file_path_;
};
}

#endif


