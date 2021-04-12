#include "coordinate_manager.h"
#include "common_file_path.h"
#include "copy_file.h" 
#include <cstring>
#include <sstream>
#include "log_manager_producer.h"

using namespace fst_ctrl;
using namespace log_space;

CoordinateManager::CoordinateManager():
    param_ptr_(NULL)
{
    param_ptr_ = new CoordinateManagerParam();
}

CoordinateManager::~CoordinateManager()
{
    if(param_ptr_ != NULL)
    {
        delete param_ptr_;
        param_ptr_ = NULL;
    }
}

ErrorCode CoordinateManager::init()
{
    if(!param_ptr_->loadParam())
    {
        LogProducer::error("Coord","Failed to load CoordinateManager component parameters");
        return COORDINATE_MANAGER_LOAD_PARAM_FAILED;
    } 

    coord_info_file_path_ = std::string(COORD_DIR);
    coord_info_file_path_modified_ = std::string(COORD_DIR_MODIFIED);
    coord_info_file_path_ += param_ptr_->coord_info_file_name_;
    coord_info_file_path_modified_ += param_ptr_->coord_info_file_name_;

    if (!copyFile(coord_info_file_path_.c_str(), coord_info_file_path_modified_.c_str()))
    {
        LogProducer::error("Coord","Failed to backup file");
        return COORDINATE_MANAGER_LOAD_PARAM_FAILED;
    }

    if(!readAllCoordInfoFromYaml(param_ptr_->max_number_of_coords_))
    {
        LogProducer::error("Coord","Failed to load coord info");
        return COORDINATE_MANAGER_LOAD_COORDINFO_FAILED;
    }
    return SUCCESS;
}

ErrorCode CoordinateManager::addCoord(CoordInfo& info)
{
    if(info.id >= static_cast<int>(coord_set_.size())
        || info.id == 0
        || coord_set_[info.id].is_valid)
    {
        LogProducer::error("Coord","Failed to add coord %d, invalid coord id", info.id);
        return COORDINATE_MANAGER_INVALID_ARG;
    }
        
    coord_set_[info.id].id = info.id;
    coord_set_[info.id].is_valid = true;
    coord_set_[info.id].name = info.name;
    coord_set_[info.id].comment = info.comment;
    coord_set_[info.id].group_id = info.group_id;
    coord_set_[info.id].data = info.data;
    if(!writeCoordInfoToYaml(coord_set_[info.id]))
    {
        LogProducer::error("Coord","Failed to add coord %d, write yaml failed", info.id);
        return COORDINATE_MANAGER_COORDINFO_FILE_WRITE_FAILED;
    }
    return SUCCESS;
}

ErrorCode CoordinateManager::deleteCoord(int id)
{
    if(id >= static_cast<int>(coord_set_.size())
        || id == 0)
    {
        LogProducer::error("Coord","Failed to delete coord %d, invalid coord id", id);
        return COORDINATE_MANAGER_INVALID_ARG;
    }
        
    coord_set_[id].is_valid = false;
    coord_set_[id].name = "";
    coord_set_[id].comment = "";
    coord_set_[id].group_id = -1;
    memset(&coord_set_[id].data, 0, sizeof(basic_alg::PoseEuler));
    if(!writeCoordInfoToYaml(coord_set_[id]))
    {
        LogProducer::error("Coord","Failed to delete coord %d, write yaml failed", id);
        return COORDINATE_MANAGER_COORDINFO_FILE_WRITE_FAILED;
    }
    return SUCCESS;
}

ErrorCode CoordinateManager::updateCoord(CoordInfo& info)
{
    if(info.id >= static_cast<int>(coord_set_.size())
        || info.id == 0
        || !coord_set_[info.id].is_valid)
    {
        LogProducer::error("Coord","Failed to update coord %d, invalid coord id", info.id);
        return COORDINATE_MANAGER_INVALID_ARG;
    }
        
    coord_set_[info.id].id = info.id;
    coord_set_[info.id].is_valid = true;
    coord_set_[info.id].name = info.name;
    coord_set_[info.id].comment = info.comment;
    coord_set_[info.id].group_id = info.group_id;
    coord_set_[info.id].data = info.data;

    if(!writeCoordInfoToYaml(coord_set_[info.id]))
    {
        LogProducer::error("Coord","Failed to update coord %d, write yaml failed", info.id);
        return COORDINATE_MANAGER_COORDINFO_FILE_WRITE_FAILED;
    }
    return SUCCESS;
}

ErrorCode CoordinateManager::moveCoord(int expect_id, int original_id)
{
    if(coord_set_[expect_id].is_valid
        || original_id == expect_id
        || !coord_set_[original_id].is_valid)
    {
        LogProducer::error("Coord","Failed to move coord %d to %d, invalid coord id", expect_id, original_id);
        return COORDINATE_MANAGER_INVALID_ARG;
    }

    coord_set_[expect_id].id = expect_id;
    coord_set_[expect_id].is_valid = true;
    coord_set_[expect_id].name = coord_set_[original_id].name;
    coord_set_[expect_id].comment = coord_set_[original_id].comment;
    coord_set_[expect_id].group_id = coord_set_[original_id].group_id;
    coord_set_[expect_id].data = coord_set_[original_id].data;

    coord_set_[original_id].id = original_id;
    coord_set_[original_id].is_valid = false;
    coord_set_[original_id].name = "";
    coord_set_[original_id].comment = "";
    coord_set_[original_id].group_id = -1;
    memset(&coord_set_[original_id].data, 0, sizeof(basic_alg::PoseEuler));

    if(!writeCoordInfoToYaml(coord_set_[original_id]) 
        || !writeCoordInfoToYaml(coord_set_[expect_id]))
    {
        LogProducer::error("Coord","Failed to move coord %d to %d, write yaml failed", expect_id, original_id);
        return COORDINATE_MANAGER_COORDINFO_FILE_WRITE_FAILED;
    }
    return SUCCESS;
}

ErrorCode CoordinateManager::getCoordInfoById(int id, CoordInfo& info)
{
    if(id >= static_cast<int>(coord_set_.size())
        || id <= 0)
    {
        LogProducer::error("Coord","Failed to get coord %d information, invalid coord id", id);
        return COORDINATE_MANAGER_INVALID_ARG;
    }

    info = coord_set_[id];
    return SUCCESS;
}

std::vector<CoordSummaryInfo> CoordinateManager::getAllValidCoordSummaryInfo()
{
    CoordSummaryInfo summary_info;
    std::vector<CoordSummaryInfo> summary_list;
    for(unsigned int i = 1; i < coord_set_.size(); ++i)
    {
        if(coord_set_[i].is_valid)
        {
            summary_info.id = i;
            summary_info.name = coord_set_[i].name;
            summary_info.comment = coord_set_[i].comment;
            summary_info.group_id = coord_set_[i].group_id;
            summary_list.push_back(summary_info);
        }
    }
    return summary_list;
}

void CoordinateManager::packDummyCoordInfo(CoordInfo& info)
{
    info.id = 0;
    info.is_valid = true;
    info.name = std::string("no coordinate");
    info.comment = std::string("no coordinate");
    info.group_id = -1;
    memset(&info.data, 0, sizeof(basic_alg::PoseEuler));
}

std::string CoordinateManager::getCoordInfoPath(int coord_id)
{
    std::string id_str;
    std::stringstream stream;
    stream << coord_id;
    stream >> id_str;
    return (std::string("coord") + id_str);
}

bool CoordinateManager::readAllCoordInfoFromYaml(int number_of_coords)
{
    CoordInfo info;

    coord_set_.clear();
    packDummyCoordInfo(info);
    coord_set_.push_back(info);
    
    if (coord_info_yaml_help_.loadParamFile(coord_info_file_path_modified_.c_str()))
    {
        for(int i = 1; i <= number_of_coords; ++i)
        {
            std::string coord_info_path;
            coord_info_path = getCoordInfoPath(i);
            coord_info_yaml_help_.getParam(coord_info_path + "/id", info.id);
            if(info.id != i)
            {
                return false;
            }
            coord_info_yaml_help_.getParam(coord_info_path + "/is_valid", info.is_valid);
            coord_info_yaml_help_.getParam(coord_info_path + "/name", info.name);
            coord_info_yaml_help_.getParam(coord_info_path + "/comment", info.comment);
            coord_info_yaml_help_.getParam(coord_info_path + "/group_id", info.group_id);
            std::string point_path = coord_info_path + std::string("/point");
            coord_info_yaml_help_.getParam(point_path + "/x", info.data.point_.x_);
            coord_info_yaml_help_.getParam(point_path + "/y", info.data.point_.y_);
            coord_info_yaml_help_.getParam(point_path + "/z", info.data.point_.z_);
            std::string euler_path = coord_info_path + std::string("/euler");
            coord_info_yaml_help_.getParam(euler_path + "/a", info.data.euler_.a_);
            coord_info_yaml_help_.getParam(euler_path + "/b", info.data.euler_.b_);
            coord_info_yaml_help_.getParam(euler_path + "/c", info.data.euler_.c_);
            coord_set_.push_back(info);
        }
	    return true;
    }
    else
    {
        LogProducer::error("Coord","Failed to read coord info from yaml: %s", coord_info_file_path_modified_.c_str());
	    return false;
    }
}

bool CoordinateManager::writeCoordInfoToYaml(CoordInfo& info)
{
    std::string coord_info_path = getCoordInfoPath(info.id);
    coord_info_yaml_help_.setParam(coord_info_path + "/id", info.id);
    coord_info_yaml_help_.setParam(coord_info_path + "/is_valid", info.is_valid);
    coord_info_yaml_help_.setParam(coord_info_path + "/name", info.name);
    coord_info_yaml_help_.setParam(coord_info_path + "/comment", info.comment);
    coord_info_yaml_help_.setParam(coord_info_path + "/group_id", info.group_id);
    std::string point_path = coord_info_path + std::string("/point");
    coord_info_yaml_help_.setParam(point_path + "/x", info.data.point_.x_);
    coord_info_yaml_help_.setParam(point_path + "/y", info.data.point_.y_);
    coord_info_yaml_help_.setParam(point_path + "/z", info.data.point_.z_);
    std::string euler_path = coord_info_path + std::string("/euler");
    coord_info_yaml_help_.setParam(euler_path + "/a", info.data.euler_.a_);
    coord_info_yaml_help_.setParam(euler_path + "/b", info.data.euler_.b_);
    coord_info_yaml_help_.setParam(euler_path + "/c", info.data.euler_.c_);
    if(coord_info_yaml_help_.dumpParamFile(coord_info_file_path_modified_.c_str()))
    {
        return true;
    }
    else
    {
        LogProducer::error("Coord","Failed to write coord info to yaml: %s", coord_info_file_path_modified_.c_str());
        return false;
    }
}


