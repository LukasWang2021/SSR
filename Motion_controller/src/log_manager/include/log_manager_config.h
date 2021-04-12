#ifndef LOG_MANAGER_CONFIG
#define LOG_MANAGER_CONFIG

/**
 * @file log_manager_config.h
 * @brief The file is the header file of class "LogManagerConfig".
 * @author Feng.Wu
 */

#include "yaml_help.h"
#include <string>

/**
 * @brief log_space includes all log related definitions and implementation.
 */
namespace log_space
{
/**
 * @brief LogManagerConfig stores the parameters of the comsumer.
 * @details LogManagerConfig is used by log_comsumer.
 */
class LogManagerConfig
{
public:
    /**
     * @brief Constructor of the class.
     */ 
    LogManagerConfig();
    /**
     * @brief Destructor of the class. 
     */  
    ~LogManagerConfig(){}

    /**
     * @brief Load all the paramters.
     * @retval true Load all the parameters successfully.
     * @retval false Failed to load all the parameters.
     */
    bool loadParam();
    /**
     * @brief Save all the paramters.
     * @retval true Save all the parameters successfully.
     * @retval false Failed to save all the parameters.
     */
    bool saveParam();

    bool display_enable_;   /**< Enable the log_comsumer to display the log.*/
	bool log_enable_;       /**< Enable the log_comsumer to write the log info files.*/
	int cycle_time_;        /**< The cycle time of the log_comsumer. The unit is ms.*/

	int max_file_log_item_; /**< The maximum number of the log items written in one file.*/
	int max_log_size_;      /**< The maximum size of all log files. The uint is MB.*/
	int percent_log_retain_;/**< The retain percent of all log files when the size exceeds. The uint is %.*/
	std::string log_path_;  /**< The path of the log files.*/

private:
    base_space::YamlHelp yaml_help_;
    std::string file_path_;

};
}

#endif


