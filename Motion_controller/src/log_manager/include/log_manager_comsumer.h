#ifndef _LOG_MANAGER_COMSUMER_H
#define _LOG_MANAGER_COMSUMER_H

/**
 * @file log_manager_comsumer.h
 * @brief The file is the header file of class "LogComsumer".
 * @author Feng.Wu
 */

#include <string>
#include <vector>
#include "log_manager_datatype.h"
#include "log_manager_config.h"

/**
 * @brief log_space includes all log related definitions and implementation.
 */
namespace log_space {

/**
 * @brief LogControlBlock is data structure used by comsumer.
 * @details The data structure contains the resources to read the log and write down a file.
 */
struct LogControlBlock {
	LogControlArea*     ctrl_area_ptr;  /**< The pointer of the control header of the log block.*/
    char*               text_area_ptr;  /**< The pointer of the text area of the log block.*/
	int32_t             log_item_cnt;   /**< The counter of the text item number.*/
	uint32_t            lost_item_cnt;  /**< The counter of the lost texts.*/
    int32_t             file_fd;        /**< The file descriptor of the current log file.*/
	std::string         file_name;      /**< The full path and name of the current log file.*/
};
/**
 * @brief LogComsumer is a proccess to read the log and write files.
 * @details LogComsumer iterates all the log blocks and write the log into files.
 */
class LogComsumer {
  public:
    /**
     * @brief Destructor of the class. 
     */  
    ~LogComsumer(void);

    /**
     * @brief Singleton pattern of the log_comsumer.
     * @return The pointer of the singleton.
     */
    static LogComsumer* getInstance();
    /**
     * @brief Initialize log_comsumer.
     * @details Load the configuration.\n
     *          Get the address pointer of the log blocks.\n
     *          Create the log directory if not existing.\n
     * @param [in] level The least level of the log to be displayed. All log will be displayed by default.
     * @param [in] thread_list Display the specified log according to the list of specified thread names.
     * @retval true success.
     * @retval false Failed to initialize.
     */
	bool init(int32_t level, std::vector<std::string> &thread_list);
    /**
     * @brief Main function of the log_comsumer.
     * @details Check if the log directory is existed.\n
     *          Check log directory space.\n
     *          Read, display and write the log into files.\n
     * @return void
     */
    void runLogComsuming(void);
    /**
     * @brief Set the log_comsumer proccess to exit the cycle.
     * @details The action is triggered by keyboard "Ctrl+C".\n
     * @return void
     */
    void setExit(void);
    /**
     * @brief Check if the log_comsumer proccess is about to exit the cycle.
     * @retval false The proccess is not setted to exit the cycle.
     * @retval true The proccess is setted to exit the cycle.
     */
    bool isExit(void);
  
  private:
  	static LogComsumer* instance_;
	LogManagerConfig* param_ptr_;

	bool display_enable_;
	bool log_enable_;
	bool is_exit_;
	uint32_t block_step_;
	int64_t max_log_size_;
	int64_t retain_log_size_;

	int32_t display_level_;
	std::vector<std::string> display_thread_list_;

	LogControlBlock *log_queue_ptr_;
	uint8_t* shmem_ptr_;

    LogComsumer(void);

	void writeLogFiles(LogControlBlock *log_block_ptr);
	void displayLog(char *item_ptr, std::string thread_name);
    void displayItem(int32_t level, char *item_ptr);
	void initLogQueue(void);
	bool createLogPath(const char *dir_path);

    //check log directory size and take actions.
    bool checkLogDirSpace(const char *dir_path);
    //get the size of the log directory.
    int64_t getLogDirSize(const char *dir_path);
    //delete the oldest log file.
    bool deleteOldestLogFile(const char *dir_path);

};
}


#endif
