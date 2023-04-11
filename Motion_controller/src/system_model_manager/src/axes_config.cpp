#include "axes_config.h"


using namespace system_model_space;
using namespace base_space;
using namespace std;

AxesConfig::AxesConfig(std::string file_path):
    file_path_(file_path), is_valid_(false)
{

}

AxesConfig::~AxesConfig()
{

}

bool AxesConfig::load()
{
    xmlDocPtr doc_ptr;
    doc_ptr = xmlParseFile(file_path_.c_str());
    if(doc_ptr == NULL)
    {
        return false;
    } 

    axis_config_.clear();
    xmlXPathObjectPtr node_set = xml_help_.getNodeObject(doc_ptr, BAD_CAST("/AxesConfig/AxisConfig"));
    for(int i = 0; i < node_set->nodesetval->nodeNr; i++)
    {
        AxisConfig_t axis_config;//axis_config是一个数据结构体，里面有轴的ID，伺服类型等。
        if(!loadAxisConfig(doc_ptr, node_set->nodesetval->nodeTab[i], axis_config)) //将读取的配置数据写入axis_config中
        {
            return false;
        }
        axis_config_.push_back(axis_config);//axis_config_是一个存放axis_config类型数据结构体的容器
    }
    xmlFree(node_set);
    xmlFreeDoc(doc_ptr);
    is_valid_ = true;
    return true;
}

bool AxesConfig::isValid()
{
    return is_valid_;
}

std::vector<AxisConfig_t>& AxesConfig::getRef()
{
    return axis_config_;
}

bool AxesConfig::loadAxisConfig(const xmlDocPtr doc_ptr, const xmlNodePtr child_node_ptr, AxisConfig_t& axis_config)
{
	if(!xml_help_.getDataFromNode<int32_t>(doc_ptr, child_node_ptr, axis_config.axis_id, BAD_CAST("./AxisId"))
        || !xml_help_.getDataFromNode<int32_t>(doc_ptr, child_node_ptr, axis_config.core_id, BAD_CAST("./CoreId"))
        || !xml_help_.getDataFromNode<int32_t>(doc_ptr, child_node_ptr, axis_config.servo_id, BAD_CAST("./ServoId"))
        || !xml_help_.getDataFromNode<std::string>(doc_ptr, child_node_ptr, axis_config.servo_type, BAD_CAST("./ServoType"))
        || !xml_help_.getDataFromNode<std::string>(doc_ptr, child_node_ptr, axis_config.root_dir, BAD_CAST("./RootDir"))
        || !xml_help_.getDataFromNode<std::string>(doc_ptr, child_node_ptr, axis_config.application, BAD_CAST("./Application"))
        || !xml_help_.getDataFromNode<std::string>(doc_ptr, child_node_ptr, axis_config.actuator.servo, BAD_CAST("./Actuator/Servo")))
    {
        return false;
    }
   
    return true;
}

