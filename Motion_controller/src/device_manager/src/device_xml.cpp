#include "device_xml.h"
#include "common_file_path.h"


using namespace fst_hal;

DeviceXml::DeviceXml(fst_log::Logger* log_ptr, DeviceManagerParam* param_ptr):
    log_ptr_(log_ptr), param_ptr_(param_ptr), config_file_path_(DEVICE_DIR)
{
    config_file_path_ += param_ptr->device_config_file_name_;
}

DeviceXml::~DeviceXml()
{

}

bool DeviceXml::loadDeviceConfig()
{
	xmlDocPtr doc_ptr;
	doc_ptr = xmlParseFile(config_file_path_.c_str());
	if(doc_ptr == NULL)
	{
		return false;
	} 

    DeviceConfig device_config;
    device_config_list_.clear();
    xmlXPathObjectPtr node_set = xml_help_.getNodeObject(doc_ptr, BAD_CAST("/Devices/Device"));
    for(int i=0; i<node_set->nodesetval->nodeNr; i++)
    {
        if(!createDeviceConfig(doc_ptr, node_set->nodesetval->nodeTab[i], device_config))
        {
            return false;
        }
        device_config_list_.push_back(device_config);
    }
	xmlFree(node_set);
	xmlFreeDoc(doc_ptr);
	return true;
}

bool DeviceXml::createDeviceConfig(const xmlDocPtr doc_ptr,
	                                    const xmlNodePtr slave_node_ptr, DeviceConfig& slave_data)
{
    std::string device_type_str;
	if(!xml_help_.getDataFromNode(doc_ptr, slave_node_ptr, slave_data.device_index, BAD_CAST("./DeviceIndex"))
        || !xml_help_.getDataFromNode(doc_ptr, slave_node_ptr, slave_data.address, BAD_CAST("./Address"))
        || !xml_help_.getDataFromNode(doc_ptr, slave_node_ptr, device_type_str, BAD_CAST("./DeviceType")))
    {
        return false;
    }   
        
    slave_data.device_type = convertDeviceTypeStrToEnum(device_type_str);
    if(slave_data.device_type == DEVICE_TYPE_INVALID)
    {
        return false;
    }

    switch(slave_data.device_type)
    {
        case DEVICE_TYPE_FST_AXIS: return false;
        case DEVICE_TYPE_FST_IO: return false;
        case DEVICE_TYPE_FST_SAFETY: return false;
        case DEVICE_TYPE_FST_ANYBUS: return false;
        case DEVICE_TYPE_VIRTUAL_AXIS: break;
        case DEVICE_TYPE_VIRTUAL_IO: return false;
        case DEVICE_TYPE_VIRTUAL_SAFETY: return false;
        case DEVICE_TYPE_NORMAL: return false;
        default: return false;
    }
	return true;
}


bool DeviceXml::saveDeviceConfig()
{
    return false;
}

DeviceXml::DeviceXml():
    log_ptr_(NULL), param_ptr_(NULL)
{

}


DeviceType DeviceXml::convertDeviceTypeStrToEnum(std::string device_type_str)
{
    if(device_type_str == std::string("FstAxis"))
    {
        return DEVICE_TYPE_FST_AXIS;
    }
    else if(device_type_str == std::string("FstIo"))
    {
        return DEVICE_TYPE_FST_IO;
    }
    else if(device_type_str == std::string("FstSafety"))
    {
        return DEVICE_TYPE_FST_SAFETY;
    }
    else if(device_type_str == std::string("AnyBus"))
    {
        return DEVICE_TYPE_FST_ANYBUS;
    }
    else if(device_type_str == std::string("VirtualAxis"))
    {
        return DEVICE_TYPE_VIRTUAL_AXIS;
    }
    else if(device_type_str == std::string("VirtualIo"))
    {
        return DEVICE_TYPE_VIRTUAL_IO;
    }
    else if(device_type_str == std::string("VirtualSafety"))
    {
        return DEVICE_TYPE_VIRTUAL_SAFETY;
    }
    else if(device_type_str == std::string("Normal"))
    {
        return DEVICE_TYPE_NORMAL;
    }
    else
    {
        return DEVICE_TYPE_INVALID;
    }
}

