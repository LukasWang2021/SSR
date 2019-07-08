#include "dynamic_alg_payload.h"
#include "common_file_path.h"
#include "error_code.h"
#include <cstring>
#include <sstream>

using namespace basic_alg;

DynamicAlgPayload::DynamicAlgPayload(fst_log::Logger *logger):
    file_path_(COMPONENT_PARAM_FILE_DIR),
    max_number_of_payloads_(10)
{
    log_ptr_ = logger;
    file_path_ += "dynamic_payload_info.yaml";
}

DynamicAlgPayload::~DynamicAlgPayload()
{
}

bool DynamicAlgPayload::init()
{
    if(!readAllPayloadInfoFromYaml())
    {
        FST_ERROR("Failed to load dynamic_payload_info");
        return false;
    }
    return true;
}

ErrorCode DynamicAlgPayload::addPayload(const PayloadInfo& info)
{
    if(info.id >= payload_set_.size()
        || info.id == 0
        || payload_set_[info.id].is_valid)
    {
        return DYNAMIC_PAYLOAD_INVALID_ARG;
    }
        
    payload_set_[info.id].id = info.id;
    payload_set_[info.id].is_valid = true;
    if(info.comment.size() == 0)
    {
        payload_set_[info.id].comment = std::string("default");
    }
    else
    {
        payload_set_[info.id].comment = info.comment;
    }
    payload_set_[info.id].m_load = info.m_load;
    payload_set_[info.id].lcx_load = info.lcx_load;
    payload_set_[info.id].lcy_load = info.lcy_load;
    payload_set_[info.id].lcz_load = info.lcz_load;
    payload_set_[info.id].Ixx_load = info.Ixx_load;
    payload_set_[info.id].Iyy_load = info.Iyy_load;
    payload_set_[info.id].Izz_load = info.Izz_load;

    if(!writePayloadInfoToYaml(payload_set_[info.id]))
    {
        return DYNAMIC_PAYLOAD_INFO_FILE_WRITE_FAILED;
    }
    return SUCCESS;
}

ErrorCode DynamicAlgPayload::deletePayload(int id)
{
    if(id >= payload_set_.size()
        || id == 0)
    {
        return DYNAMIC_PAYLOAD_INVALID_ARG;
    }
        
    payload_set_[id].is_valid = false;
    payload_set_[id].comment = std::string("default");
    payload_set_[id].m_load = 0.0;
    payload_set_[id].lcx_load = 0.0;
    payload_set_[id].lcy_load = 0.0;
    payload_set_[id].lcz_load = 0.0;
    payload_set_[id].Ixx_load = 0.0;
    payload_set_[id].Iyy_load = 0.0;
    payload_set_[id].Izz_load = 0.0;
    if(!writePayloadInfoToYaml(payload_set_[id]))
    {
        return DYNAMIC_PAYLOAD_INFO_FILE_WRITE_FAILED;
    }
    return SUCCESS;
}

ErrorCode DynamicAlgPayload::updatePayload(const PayloadInfo& info)
{
    if(info.id >= payload_set_.size()
        || info.id == 0
        || !payload_set_[info.id].is_valid)
    {
        return DYNAMIC_PAYLOAD_INVALID_ARG;
    }
        
    payload_set_[info.id].id = info.id;
    payload_set_[info.id].is_valid = true;
    if(info.comment.size() == 0)
    {
        payload_set_[info.id].comment = std::string("default");
    }
    else
    {
        payload_set_[info.id].comment = info.comment;
    }
    payload_set_[info.id].m_load = info.m_load;
    payload_set_[info.id].lcx_load = info.lcx_load;
    payload_set_[info.id].lcy_load = info.lcy_load;
    payload_set_[info.id].lcz_load = info.lcz_load;
    payload_set_[info.id].Ixx_load = info.Ixx_load;
    payload_set_[info.id].Iyy_load = info.Iyy_load;
    payload_set_[info.id].Izz_load = info.Izz_load;

    if(!writePayloadInfoToYaml(payload_set_[info.id]))
    {
        return DYNAMIC_PAYLOAD_INFO_FILE_WRITE_FAILED;
    }
    return SUCCESS;
}

ErrorCode DynamicAlgPayload::movePayload(int expect_id, int original_id)
{
    if(payload_set_[expect_id].is_valid
        || original_id == expect_id
        || !payload_set_[original_id].is_valid)
    {
        return DYNAMIC_PAYLOAD_INVALID_ARG;
    }

    payload_set_[expect_id].id = expect_id;
    payload_set_[expect_id].is_valid = true;
    payload_set_[expect_id].comment = payload_set_[original_id].comment;
    payload_set_[expect_id].m_load = payload_set_[original_id].m_load;
    payload_set_[expect_id].lcx_load = payload_set_[original_id].lcx_load;
    payload_set_[expect_id].lcy_load = payload_set_[original_id].lcy_load;
    payload_set_[expect_id].lcz_load = payload_set_[original_id].lcz_load;
    payload_set_[expect_id].Ixx_load = payload_set_[original_id].Ixx_load;
    payload_set_[expect_id].Iyy_load = payload_set_[original_id].Iyy_load;
    payload_set_[expect_id].Izz_load = payload_set_[original_id].Izz_load;

    payload_set_[original_id].id = original_id;
    payload_set_[original_id].is_valid = false;
    payload_set_[original_id].comment = std::string("default");
    payload_set_[original_id].m_load = 0.0;
    payload_set_[original_id].lcx_load = 0.0;
    payload_set_[original_id].lcy_load = 0.0;
    payload_set_[original_id].lcz_load = 0.0;
    payload_set_[original_id].Ixx_load = 0.0;
    payload_set_[original_id].Iyy_load = 0.0;
    payload_set_[original_id].Izz_load = 0.0;

    if(!writePayloadInfoToYaml(payload_set_[original_id]) 
        || !writePayloadInfoToYaml(payload_set_[expect_id]))
    {
        return DYNAMIC_PAYLOAD_INFO_FILE_WRITE_FAILED;
    }
    return SUCCESS;
}

ErrorCode DynamicAlgPayload::getPayloadInfoById(int id, PayloadInfo& info)
{
    if(id >= payload_set_.size()
        || id < 0)
    {
        return DYNAMIC_PAYLOAD_INVALID_ARG;
    }

    memcpy(&info, &payload_set_[id], sizeof(PayloadInfo));

    return SUCCESS;
}

std::vector<PayloadSummaryInfo> DynamicAlgPayload::getAllValidPayloadSummaryInfo(void)
{
    PayloadSummaryInfo summary_info;
    std::vector<PayloadSummaryInfo> summary_list;
    for(unsigned int i = 1; i < payload_set_.size(); ++i)
    {
        if(payload_set_[i].is_valid)
        {
            summary_info.id = i;
            summary_info.comment = payload_set_[i].comment;
            summary_list.push_back(summary_info);
        }
    }
    return summary_list;
}

void DynamicAlgPayload::getAllValidPayloadSummaryInfo(std::vector<PayloadSummaryInfo>& info_list)
{
    info_list.resize(0);
    PayloadSummaryInfo summary_info;

    for(unsigned int i = 1; i < payload_set_.size(); ++i)
    {
        if(payload_set_[i].is_valid)
        {
            summary_info.id = i;
            summary_info.comment = payload_set_[i].comment;
            info_list.push_back(summary_info);
        }
    }
}

void DynamicAlgPayload::packDummyPayloadInfo(PayloadInfo& info)
{
    info.id = 0;
    info.is_valid = true;
    info.comment = std::string("No payload");
    info.m_load = 0;
    info.lcx_load = 0;
    info.lcy_load = 0;
    info.lcz_load = 0;
    info.Ixx_load = 0;
    info.Iyy_load = 0;
    info.Izz_load = 0;
}

std::string DynamicAlgPayload::getPayloadInfoPath(int payload_id)
{
    std::string id_str;
    std::stringstream stream;
    stream << payload_id;
    stream >> id_str;
    return (std::string("payload") + id_str);
}

bool DynamicAlgPayload::readAllPayloadInfoFromYaml(void)
{
    PayloadInfo info;
    payload_set_.clear();
    packDummyPayloadInfo(info);
    payload_set_.push_back(info);
    
    if (yaml_help_.loadParamFile(file_path_.c_str()))
    {
        yaml_help_.getParam("max_number_of_payloads", max_number_of_payloads_);

        for(unsigned int i = 1; i <= max_number_of_payloads_; ++i)
        {
            std::string payload_info_path;
            payload_info_path = getPayloadInfoPath(i);
            yaml_help_.getParam(payload_info_path + "/id", info.id);
            if(info.id != i)
            {
                return false;
            }
            yaml_help_.getParam(payload_info_path + "/is_valid", info.is_valid);
            yaml_help_.getParam(payload_info_path + "/comment", info.comment);
            yaml_help_.getParam(payload_info_path + "/m_load", info.m_load);
            yaml_help_.getParam(payload_info_path + "/lcx_load", info.lcx_load);
            yaml_help_.getParam(payload_info_path + "/lcy_load", info.lcy_load);
            yaml_help_.getParam(payload_info_path + "/lcz_load", info.lcz_load);
            yaml_help_.getParam(payload_info_path + "/Ixx_load", info.Ixx_load);
            yaml_help_.getParam(payload_info_path + "/Iyy_load", info.Iyy_load);
            yaml_help_.getParam(payload_info_path + "/Izz_load", info.Izz_load);
            payload_set_.push_back(info);
        }
	    return true;
    }
    else
    {
        FST_ERROR("lost config file: %s", file_path_.c_str());
	    return false;
    }
}

bool DynamicAlgPayload::writePayloadInfoToYaml(PayloadInfo& info)
{
    std::string payload_info_path = getPayloadInfoPath(info.id);
    yaml_help_.setParam(payload_info_path + "/id", info.id);
    yaml_help_.setParam(payload_info_path + "/is_valid", info.is_valid);
    yaml_help_.setParam(payload_info_path + "/comment", info.comment);
    yaml_help_.setParam(payload_info_path + "/m_load", info.m_load);
    yaml_help_.setParam(payload_info_path + "/lcx_load", info.lcx_load);
    yaml_help_.setParam(payload_info_path + "/lcy_load", info.lcy_load);
    yaml_help_.setParam(payload_info_path + "/lcz_load", info.lcz_load);
    yaml_help_.setParam(payload_info_path + "/Ixx_load", info.Ixx_load);
    yaml_help_.setParam(payload_info_path + "/Iyy_load", info.Iyy_load);
    yaml_help_.setParam(payload_info_path + "/Izz_load", info.Izz_load);

    return yaml_help_.dumpParamFile(file_path_.c_str());
}

void DynamicAlgPayload::printfPayload(void)
{
    for(unsigned int i = 0; i <= max_number_of_payloads_; ++i)
    {
        printf("id=%d, valid=%d, comment=%s, load=%lf, %lf, %lf, %lf, %lf, %lf, %lf\n",
               payload_set_[i].id, payload_set_[i].is_valid, (payload_set_[i].comment).c_str(), payload_set_[i].m_load, 
               payload_set_[i].lcx_load, payload_set_[i].lcy_load, payload_set_[i].lcz_load,
               payload_set_[i].Ixx_load, payload_set_[i].Iyy_load, payload_set_[i].Izz_load);
    }
}


