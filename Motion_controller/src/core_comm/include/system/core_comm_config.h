#ifndef CORE_COMM_CONFIG_H
#define CORE_COMM_CONFIG_H

/**
 * @file core_comm_config.h
 * @brief The file includes the classs for parsing the configuration file of inter core communication.
 * @author zhengyu.shen
 */

#include "common/core_comm_datatype.h"
#include "xml_help.h"
#include <stdint.h>
#include <vector>
#include <string>

/**
 * @brief core_comm_space includes all inter core communication related implementation.
 */
namespace core_comm_space
{
/**
 * @brief CoreCommConfig is the object to handle the configuration file which defines the inter core communication pattern.
 */
class CoreCommConfig
{
public:
    /**
     * @brief Constructor of the class.
     */
    CoreCommConfig();
     /**
     * @brief Destructor of the class.
     */
    ~CoreCommConfig();

    /**
     * @brief Read configuration file and transform it into local memroy.
     * @details The configuration file is in xml format. The result is stroed in member variable 'CoreCommConfig_t config_data_'.\n
     * @param [in] file_name Configuration file name.
     * @param [out] error_msg Error dignose message.
     * @retval true Operation is successful.
     * @retval false Operation is failed.
     */
    bool parseConfigXml(const std::string& file_name, std::string& error_msg);
    /**
     * @brief Print content of the member variable 'CoreCommConfig_t config_data_'.
     * @return void
     */    
    void printCoreCommConfig();

    CoreCommConfig_t config_data_;  /**< Store the configuration of inter core communication.*/
private:
    base_space::XmlHelp xml_help_;  /**< File parser for XML format.*/
    
    bool getBoardcastBlockData(const xmlDocPtr doc_ptr, const xmlNodePtr target_node_ptr, BoardcastBlockData_t& data, std::string& error_msg);
    bool getCpuAckBlockData(const xmlDocPtr doc_ptr, const xmlNodePtr target_node_ptr, CpuAckBlockData_t& data, std::string& error_msg);
    bool getCommBlockData(const xmlDocPtr doc_ptr, const xmlNodePtr target_node_ptr, CommBlockData_t& data, std::string& error_msg);
    bool convertTypeToInt32(const std::string& type_str, int32_t& type_int32);
    bool convertTypeToString(const int32_t& type_int32, std::string& type_str);
    bool checkConfig(std::string& error_msg);
};

}


#endif

