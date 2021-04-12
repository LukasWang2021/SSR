#include "system/core_comm_config.h"
#include "common/core_comm_config_base.h"
#include "common_file_path.h"
#include <iostream>
#include <stdlib.h>
#include <string.h>


using namespace std;
using namespace base_space;
using namespace core_comm_space;


CoreCommConfig::CoreCommConfig()
{

}

CoreCommConfig::~CoreCommConfig()
{

}

bool CoreCommConfig::parseConfigXml(const std::string& file_name, std::string& error_msg)
{
    std::string file_path(COMM_CONFIG_DIR);
    file_path.append(file_name);
    
    error_msg.clear();
	xmlDocPtr doc_ptr;
    
	doc_ptr = xmlParseFile(file_path.c_str());
	if(doc_ptr == NULL)
	{
	    error_msg.append("invalid xml format");
		return false;
	} 

    CpuAckBlockData_t cpu_ack_block_data;
    CommBlockData_t comm_block_data;

    xmlXPathObjectPtr core_comm_config_node = xml_help_.getNodeObject(doc_ptr, BAD_CAST("/CoreCommConfig"));
    if(core_comm_config_node->nodesetval->nodeNr != 1)
    {
        error_msg.append("multiple 'CoreCommConfig' element");
        return false;
    }

    if(!xml_help_.getDataFromNode(doc_ptr, core_comm_config_node->nodesetval->nodeTab[0], config_data_.base_address, BAD_CAST("./BaseAddress")))
    {
        error_msg.append("invalid BaseAddress");
        return false;
    }
       
    xmlXPathObjectPtr boardcast_block_node_set = xml_help_.getNodeObject(doc_ptr, core_comm_config_node->nodesetval->nodeTab[0], BAD_CAST("./BoardcastBlock"));
    config_data_.boardcast_block_num = boardcast_block_node_set->nodesetval->nodeNr;
    if(config_data_.boardcast_block_num != 1
        || !getBoardcastBlockData(doc_ptr, boardcast_block_node_set->nodesetval->nodeTab[0], config_data_.boardcast_data, error_msg))
    {
        return false;
    }

    xmlXPathObjectPtr cpu_ack_block_node_set = xml_help_.getNodeObject(doc_ptr, core_comm_config_node->nodesetval->nodeTab[0], BAD_CAST("./CpuAckBlock"));
    config_data_.cpu_ack_block_num = cpu_ack_block_node_set->nodesetval->nodeNr;
    for(size_t i = 0; i < config_data_.cpu_ack_block_num; i++)
    {
        if(!getCpuAckBlockData(doc_ptr, cpu_ack_block_node_set->nodesetval->nodeTab[i], cpu_ack_block_data, error_msg))
        {
            error_msg.append(" at block ");
            error_msg.append(std::to_string(i));
            return false;
        }
        memcpy(&config_data_.cpu_ack_data[i], &cpu_ack_block_data, sizeof(CpuAckBlockData_t));
    }

    xmlXPathObjectPtr comm_block_node_set = xml_help_.getNodeObject(doc_ptr, core_comm_config_node->nodesetval->nodeTab[0], BAD_CAST("./CommBlock"));
    config_data_.comm_block_num = comm_block_node_set->nodesetval->nodeNr;
    for(size_t i = 0; i < config_data_.comm_block_num; i++)
    {
        if(!getCommBlockData(doc_ptr, comm_block_node_set->nodesetval->nodeTab[i], comm_block_data, error_msg))
        {
            error_msg.append(" at block ");
            error_msg.append(std::to_string(i));
            return false;
        }
        memcpy(&config_data_.comm_data[i], &comm_block_data, sizeof(CommBlockData_t));
    }
  
	xmlFree(boardcast_block_node_set);
    xmlFree(cpu_ack_block_node_set);
    xmlFree(comm_block_node_set);
	xmlFreeDoc(doc_ptr);

    if(!checkConfig(error_msg))
    {
        error_msg.append(", configuration is invalid");
        return false;
    }
    
    error_msg.append("success");
    return true;
}

void CoreCommConfig::printCoreCommConfig()
{
    std::cout<<"------------CoreCommConfig------------"<<std::endl
             <<"BaseAddress = 0x"<<std::hex<<config_data_.base_address<<std::endl;

    std::cout<<"BoardcastBlock:"<<std::endl
             <<"    ByteOffset = 0x"<<std::hex<<config_data_.boardcast_data.byte_offset<<std::endl
             <<"    From = "<<std::dec<<config_data_.boardcast_data.from<<std::endl;

    for(size_t i = 0; i < config_data_.cpu_ack_block_num; ++i)
    {
        std::cout<<"CpuAckBlock"<<i<<":"<<std::endl
                 <<"    ByteOffset = 0x"<<std::hex<<config_data_.cpu_ack_data[i].byte_offset<<std::endl
                 <<"    From = "<<std::dec<<config_data_.cpu_ack_data[i].from<<std::endl
                 <<"    To = "<<std::dec<<config_data_.cpu_ack_data[i].to<<std::endl;
    }
    std::string type_str;
    for(size_t i = 0; i < config_data_.comm_block_num; ++i)
    {
        
        if(!convertTypeToString(config_data_.comm_data[i].type, type_str))
        {
            type_str = "Unknown type";
        }
        std::cout<<"CommBlock"<<i<<":"<<std::endl
                 <<"    Type = "<<std::dec<<type_str<<std::endl
                 <<"    ApplicationID = "<<std::dec<<config_data_.comm_data[i].application_id<<std::endl
                 <<"    ByteOffset = 0x"<<std::hex<<config_data_.comm_data[i].byte_offset<<std::endl
                 <<"    From = "<<std::dec<<config_data_.comm_data[i].from<<std::endl
                 <<"    To = "<<std::dec<<config_data_.comm_data[i].to<<std::endl
                 <<"    Param1 = "<<std::dec<<config_data_.comm_data[i].param1<<std::endl
                 <<"    Param2 = "<<std::dec<<config_data_.comm_data[i].param2<<std::endl
                 <<"    Param3 = "<<std::dec<<config_data_.comm_data[i].param3<<std::endl
                 <<"    Param4 = "<<std::dec<<config_data_.comm_data[i].param4<<std::endl;
    }
}

bool CoreCommConfig::getBoardcastBlockData(const xmlDocPtr doc_ptr, const xmlNodePtr target_node_ptr, BoardcastBlockData_t& data, std::string& error_msg)
{
    if(!xml_help_.getDataFromNode(doc_ptr, target_node_ptr, data.byte_offset, BAD_CAST("./ByteOffset")))
    {
        error_msg.append("invalid BoardcastBlock/ByteOffset");
        return false;
    }

    if(!xml_help_.getDataFromNode(doc_ptr, target_node_ptr, data.from, BAD_CAST("./From")))
    {
        error_msg.append("invalid BoardcastBlock/From");
        return false;
    }

    return true;
}

bool CoreCommConfig::getCpuAckBlockData(const xmlDocPtr doc_ptr, const xmlNodePtr target_node_ptr, CpuAckBlockData_t& data, std::string& error_msg)
{
    if(!xml_help_.getDataFromNode(doc_ptr, target_node_ptr, data.byte_offset, BAD_CAST("./ByteOffset")))
    {
        error_msg.append("invalid EventBlock/OffsetAddress");
        return false;
    }

    if(!xml_help_.getDataFromNode(doc_ptr, target_node_ptr, data.from, BAD_CAST("./From")))
    {
        error_msg.append("invalid EventBlock/From");
        return false;
    }

    if(!xml_help_.getDataFromNode(doc_ptr, target_node_ptr, data.to, BAD_CAST("./To")))
    {
        error_msg.append("invalid EventBlock/To");
        return false;
    }    

    return true;
}

bool CoreCommConfig::getCommBlockData(const xmlDocPtr doc_ptr, const xmlNodePtr target_node_ptr, CommBlockData_t& data, std::string& error_msg)
{
    std::string type_str;
    if(!xml_help_.getDataFromNode(doc_ptr, target_node_ptr, type_str, BAD_CAST("./Type"))
        || !convertTypeToInt32(type_str, data.type))
    {
        error_msg.append("invalid CommBlock/Type");
        return false;
    }

    if(!xml_help_.getDataFromNode(doc_ptr, target_node_ptr, data.application_id, BAD_CAST("./ApplicationID")))
    {
        error_msg.append("invalid CommBlock/ApplicationID");
        return false;
    }

    if(!xml_help_.getDataFromNode(doc_ptr, target_node_ptr, data.byte_offset, BAD_CAST("./ByteOffset")))
    {
        error_msg.append("invalid CommBlock/ByteOffset");
        return false;
    }

    if(!xml_help_.getDataFromNode(doc_ptr, target_node_ptr, data.from, BAD_CAST("./From")))
    {
        error_msg.append("invalid CommBlock/From");
        return false;
    }

    if(!xml_help_.getDataFromNode(doc_ptr, target_node_ptr, data.to, BAD_CAST("./To")))
    {
        error_msg.append("invalid CommBlock/To");
        return false;
    }    

    if(!xml_help_.getDataFromNode(doc_ptr, target_node_ptr, data.param1, BAD_CAST("./Param1")))
    {
        data.param1 = 0;
    }
    
    if(!xml_help_.getDataFromNode(doc_ptr, target_node_ptr, data.param2, BAD_CAST("./Param2")))
    {
        data.param2 = 0;
    }
    
    if(!xml_help_.getDataFromNode(doc_ptr, target_node_ptr, data.param3, BAD_CAST("./Param3")))
    {
        data.param3 = 0;
    }
    
    if(!xml_help_.getDataFromNode(doc_ptr, target_node_ptr, data.param4, BAD_CAST("./Param4")))
    {
        data.param4 = 0;
    }

    return true;
}

bool CoreCommConfig::convertTypeToInt32(const std::string& type_str, int32_t& type_int32)
{
    if(type_str == COMM_BLOCK_TYPE_COMM_REG_STR)
    {
        type_int32 = COMM_BLOCK_TYPE_COMM_REG;
    }
    else if(type_str == COMM_BLOCK_TYPE_BUFFER_STR)
    {
        type_int32 = COMM_BLOCK_TYPE_BUFFER;
    }
    else if(type_str == COMM_BLOCK_TYPE_CIRCLE_BUFFER_STR)
    {
        type_int32 = COMM_BLOCK_TYPE_CIRCLE_BUFFER;
    }
    else if(type_str == COMM_BLOCK_TYPE_FPGA_BUFFER_STR)
    {
        type_int32 = COMM_BLOCK_TYPE_FPGA_BUFFER;
    }
    else if(type_str == COMM_BLOCK_TYPE_CORE_PROCESS_CALL_STR)
    {
        type_int32 = COMM_BLOCK_TYPE_CORE_PROCESS_CALL;
    }
    else
    {
        return false;
    }
    return true;
}

bool CoreCommConfig::convertTypeToString(const int32_t& type_int32, std::string& type_str)
{
    if(type_int32 == COMM_BLOCK_TYPE_COMM_REG)
    {
        type_str = COMM_BLOCK_TYPE_COMM_REG_STR;
    }
    else if(type_int32 == COMM_BLOCK_TYPE_BUFFER)
    {
        type_str = COMM_BLOCK_TYPE_BUFFER_STR;
    }
    else if(type_int32 == COMM_BLOCK_TYPE_CIRCLE_BUFFER)
    {
        type_str = COMM_BLOCK_TYPE_CIRCLE_BUFFER_STR;
    }
    else if(type_int32 == COMM_BLOCK_TYPE_FPGA_BUFFER)
    {
        type_str = COMM_BLOCK_TYPE_FPGA_BUFFER_STR;
    }
    else if(type_int32 == COMM_BLOCK_TYPE_CORE_PROCESS_CALL)
    {
        type_str = COMM_BLOCK_TYPE_CORE_PROCESS_CALL_STR;
    }
    else
    {
        return false;
    }
    return true;
}

bool CoreCommConfig::checkConfig(std::string& error_msg)
{
    return true;
}


