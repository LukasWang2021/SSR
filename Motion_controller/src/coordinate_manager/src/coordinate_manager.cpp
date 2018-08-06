#include "coordinate_manager.h"
#include "common_file_path.h"
#include <cstring>
#include <sstream>


using namespace fst_ctrl;


CoordinateManager::CoordinateManager():
    log_ptr_(NULL),
    param_ptr_(NULL)
{
    log_ptr_ = new fst_log::Logger();
    param_ptr_ = new CoordinateManagerParam();
    FST_LOG_INIT("CoordinateManager");
    FST_LOG_SET_LEVEL((fst_log::MessageLevel)param_ptr_->log_level_);
}

CoordinateManager::~CoordinateManager()
{

}

bool CoordinateManager::init()
{
    if(!param_ptr_->loadParam())
    {
        FST_ERROR("Failed to load CoordinateManager component parameters");
        return false;
    } 
    FST_LOG_SET_LEVEL((fst_log::MessageLevel)param_ptr_->log_level_);   

    coord_info_file_path_ = std::string(COORD_DIR);
    coord_info_file_path_ += param_ptr_->coord_info_file_name_;
    if(!readAllCoordInfoFromYaml(param_ptr_->max_number_of_coords_))
    {
        FST_ERROR("Failed to load coord info");
        return false;
    }
    return true;
}

bool CoordinateManager::addCoord(CoordInfo& info)
{
    if(info.id >= coord_set_.size()
        || info.id == 0
        || coord_set_[info.id].is_valid)
    {
        return false;
    }
        
    coord_set_[info.id].id = info.id;
    coord_set_[info.id].is_valid = true;
    if(coord_set_[info.id].name.size() == 0)
    {
        coord_set_[info.id].name = std::string("default");
    }
    else
    {
        coord_set_[info.id].name = info.name;
    }
    if(coord_set_[info.id].comment.size() == 0)
    {
        coord_set_[info.id].comment = std::string("default");
    }
    else
    {
        coord_set_[info.id].comment = info.comment;
    }
    coord_set_[info.id].group_id = info.group_id;
    coord_set_[info.id].data = info.data;
    return writeCoordInfoToYaml(coord_set_[info.id]);
}

bool CoordinateManager::deleteCoord(int id)
{
    if(id >= coord_set_.size()
        || id == 0)
    {
        return false;
    }
        
    coord_set_[id].is_valid = false;
    coord_set_[id].name = std::string("default");
    coord_set_[id].comment = std::string("default");
    coord_set_[id].group_id = -1;
    memset(&coord_set_[id].data, 0, sizeof(fst_base::PoseEuler));
    return writeCoordInfoToYaml(coord_set_[id]);
}

bool CoordinateManager::updateCoord(CoordInfo& info)
{
    if(info.id >= coord_set_.size()
        || info.id == 0
        || !coord_set_[info.id].is_valid)
    {
        return false;
    }
        
    coord_set_[info.id].id = info.id;
    coord_set_[info.id].is_valid = true;
    if(coord_set_[info.id].name.size() == 0)
    {
        coord_set_[info.id].name = std::string("default");
    }
    else
    {
        coord_set_[info.id].name = info.name;
    }
    if(coord_set_[info.id].comment.size() == 0)
    {
        coord_set_[info.id].comment = std::string("default");
    }
    else
    {
        coord_set_[info.id].comment = info.comment;
    }
    coord_set_[info.id].group_id = info.group_id;
    coord_set_[info.id].data = info.data;

    return writeCoordInfoToYaml(coord_set_[info.id]);
}

bool CoordinateManager::moveCoord(int expect_id, int original_id)
{
    if(coord_set_[expect_id].is_valid
        || original_id == expect_id
        || !coord_set_[original_id].is_valid)
    {
        return false;
    }

    coord_set_[expect_id].id = expect_id;
    coord_set_[expect_id].is_valid = true;
    coord_set_[expect_id].name = coord_set_[original_id].name;
    coord_set_[expect_id].comment = coord_set_[original_id].comment;
    coord_set_[expect_id].group_id = coord_set_[original_id].group_id;
    coord_set_[expect_id].data = coord_set_[original_id].data;

    coord_set_[original_id].id = original_id;
    coord_set_[original_id].is_valid = false;
    coord_set_[original_id].name = std::string("default");
    coord_set_[original_id].comment = std::string("default");
    coord_set_[original_id].group_id = -1;
    memset(&coord_set_[original_id].data, 0, sizeof(fst_base::PoseEuler));

    return (writeCoordInfoToYaml(coord_set_[original_id]) 
            && writeCoordInfoToYaml(coord_set_[expect_id]));
}

bool CoordinateManager::getCoordInfoById(int id, CoordInfo& info)
{
    if(info.id >= coord_set_.size()
        || info.id <= 0)
    {
        return false;
    }

    info = coord_set_[id];
    return true;
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
    memset(&info.data, 0, sizeof(fst_base::PoseEuler));
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
    
    if (coord_info_yaml_help_.loadParamFile(coord_info_file_path_.c_str()))
    {
        for(unsigned int i = 1; i <= number_of_coords; ++i)
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
            coord_info_yaml_help_.getParam(point_path + "/x", info.data.position.x);
            coord_info_yaml_help_.getParam(point_path + "/y", info.data.position.y);
            coord_info_yaml_help_.getParam(point_path + "/z", info.data.position.z);
            std::string euler_path = coord_info_path + std::string("/euler");
            coord_info_yaml_help_.getParam(euler_path + "/a", info.data.orientation.a);
            coord_info_yaml_help_.getParam(euler_path + "/b", info.data.orientation.b);
            coord_info_yaml_help_.getParam(euler_path + "/c", info.data.orientation.c);
            coord_set_.push_back(info);
        }
	    return true;
    }
    else
    {
        FST_ERROR("lost config file: %s", coord_info_file_path_.c_str());
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
    coord_info_yaml_help_.setParam(point_path + "/x", info.data.position.x);
    coord_info_yaml_help_.setParam(point_path + "/y", info.data.position.y);
    coord_info_yaml_help_.setParam(point_path + "/z", info.data.position.z);
    std::string euler_path = coord_info_path + std::string("/euler");
    coord_info_yaml_help_.setParam(euler_path + "/a", info.data.orientation.a);
    coord_info_yaml_help_.setParam(euler_path + "/b", info.data.orientation.b);
    coord_info_yaml_help_.setParam(euler_path + "/c", info.data.orientation.c);
    return coord_info_yaml_help_.dumpParamFile(coord_info_file_path_.c_str());
}


