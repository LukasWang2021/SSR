#ifndef YAML_HELP_H
#define YAML_HELP_H

/**
 * @file yaml_help.h
 * @brief The file includes the class for handling file in YAML format.
 * @author zhengyu.shen
 */

#include <yaml-cpp/yaml.h>
#include <string>

/**
 * @brief base_space includes all foundational definitions and realizations.
 */
namespace base_space
{
/**
 * @brief YamlHelp is the object to handle files in YAML format.
 */
class YamlHelp
{
public:
    /**
     * @brief Constructor of the class.
     */    
    YamlHelp();
    /**
     * @brief Destructor of the class.
     */
    ~YamlHelp();
    /**
     * @brief Load yaml file to local memory.
     * @param [in] file_path The full path and the name of the text file.
     * @retval true Operation succeed.
     * @retval false Operation failed.
     */
    bool loadParamFile(const std::string file_path);
    /**
     * @brief Load local memory to yaml file.
     * @param [in] file_path The full path and the name of the text file.
     * @retval true Operation succeed.
     * @retval false Operation failed.
     */    
    bool dumpParamFile(const std::string file_path);
    /**
     * @brief Get parameter value by key string.
     * @param [in] key Key string.
     * @param [out] value Parameter value.
     * @retval true Operation succeed.
     * @retval false Operation failed.
     */
    bool getParam(const std::string& key, bool& value);
    bool getParam(const std::string& key, int& value);
    bool getParam(const std::string& key, double& value);
    bool getParam(const std::string& key, std::string& value);
    bool getParam(const std::string& key, std::vector<bool>& value);
    bool getParam(const std::string& key, std::vector<int>& value);
    bool getParam(const std::string& key, std::vector<double>& value);
    bool getParam(const std::string& key, std::vector<std::string>& value);
    /**
     * @brief Set parameter value by key string.
     * @param [in] key Key string.
     * @param [in] value Parameter value.
     * @retval true Operation succeed.
     * @retval false Operation failed.
     */       
    bool setParam(const std::string& key, const bool& value);
    bool setParam(const std::string& key, const int& value);
    bool setParam(const std::string& key, const double& value);
    bool setParam(const std::string& key, const std::string& value);
    bool setParam(const std::string& key, const std::vector<bool>& value);
    bool setParam(const std::string& key, const std::vector<int>& value);
    bool setParam(const std::string& key, const std::vector<double>& value);
    bool setParam(const std::string& key, const std::vector<std::string>& value);
    /**
     * @brief Add a parameter by key string.
     * @param [in] key Key string.
     * @param [in] value Parameter value.
     * @retval true Operation succeed.
     * @retval false Operation failed.
     */
    bool addParam(const std::string& key, const bool& value);
    bool addParam(const std::string& key, const int& value);
    bool addParam(const std::string& key, const double& value);
    bool addParam(const std::string& key, const std::string& value);
    bool addParam(const std::string& key, const std::vector<bool>& value);
    bool addParam(const std::string& key, const std::vector<int>& value);
    bool addParam(const std::string& key, const std::vector<double>& value);
    bool addParam(const std::string& key, const std::vector<std::string>& value);
    /**
     * @brief Delete a parameter by key string.
     * @param [in] key Key string.
     * @retval true Operation succeed.
     * @retval false Operation failed.
     */    
     bool deleteParam(const std::string& key);

private:
    YAML::Node node_;   /**< Data struct to store yaml file in local memory.*/

    void getKeys(const std::string& key_str, std::vector<std::string>& key_set);
};

}


#endif


