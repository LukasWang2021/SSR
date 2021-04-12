#ifndef LOG_MANAGER_PRODUCER_H
#define LOG_MANAGER_PRODUCER_H

/**
 * @file log_manager_producer.h
 * @brief The file is the header file of class "LogProducer".
 * @author Feng.Wu
 */

#include <mutex>
#include <map>
#include "log_manager_datatype.h"
#include "log_manager_config.h"

/**
 * @brief log_space includes all log related definitions and implementation.
 */
namespace log_space {
/**
 * @brief LogProducer can be used to log information by a proccess or a thread.
 * @details LogProducer has static functions which can be used by all threads without instantiation.
 */
class LogProducer {
  public:
    /**
     * @brief Constructor of the class.
     */
    LogProducer(void);
    /**
     * @brief Destructor of the class. 
     */    
    ~LogProducer(void);

    /**
     * @brief Register one memory block for log.
     * @details Find one available memory block and initialize its control head.\n
     *          Map the thread pid with the block address pointer.\n
     * @param [in] thread_name The thread name which is using this log block.
     * @param [in] isr_count_ptr The address pointer of Interrupt Service Routine. ISR is one of the log elements.
     * @retval true One available log block is registered.
     * @retval false Failed to register a log block.
     */
    bool init(const char *thread_name, uint32_t *isr_count_ptr);
    /**
     * @brief Set which level up information will be logged.
     * @details typedef enum{
     *          LOG_DEBUG = 0,
     *          LOG_INFO  = 1,
     *          LOG_WARN  = 2,
     *          LOG_ERROR = 3,
     *          }MessageLevel;
     * @param [in] level This level of the log information. The default value is LOG_INFO.
     * @return void
     */
    static void setLoggingLevel(MessageLevel level);
    /**
     * @brief Log the debug information.
     * @param [in] module_name The module name which is logging.
     * @param [in] format The logging string.
     * @return void
     */
    static void debug(const char *module_name, const char *format, ...);
    /**
     * @brief Log the info information.
     * @param [in] module_name The module name which is logging.
     * @param [in] format The logging string.
     * @return void
     */
    static void info(const char *module_name, const char *format, ...);
    /**
     * @brief Log the warn information.
     * @param [in] module_name The module name which is logging.
     * @param [in] format The logging string.
     * @return void
     */
    static void warn(const char *module_name, const char *format, ...);
    /**
     * @brief Log the error information.
     * @param [in] module_name The module name which is logging.
     * @param [in] format The logging string.
     * @return void
     */
    static void error(const char *module_name, const char *format, ...);
  
  private:    
	bool is_valid_;
	uint8_t* shmem_ptr_;
    LogControlArea* ctrl_area_ptr_;

	static std::mutex occupied_mutex_;
    static std::map<unsigned long, LogControlArea*> pid_map_ptr_;
    static int32_t log_level_;
    static uint32_t* isr_count_ptr_;

    static bool relocatePtrByThreadId(LogControlArea * &ctrl_area_using_ptr);
    static void constructItem(const char *module_name, const LogControlArea *ctrl_area_using_ptr, LogItemArea *pitem);
	static void writeShareMemory(LogControlArea *ctrl_area_using_ptr, const LogItemArea *pitem);
};

}
#endif
