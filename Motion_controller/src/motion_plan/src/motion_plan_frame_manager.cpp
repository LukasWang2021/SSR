#include "motion_plan_frame_manager.h"
#include "motion_plan_variable.h"
#include "motion_plan_reuse.h"
#include <sstream>
#include <cstring>

using namespace fst_controller;
using fst_parameter::ParamGroup;
using fst_parameter::ParamValue;
using namespace fst_algorithm;

FrameManager::FrameManager(std::string frame_name, int frame_set_size, std::string file_path,
                                fst_algorithm::Matrix& frame_matrix, fst_algorithm::Matrix& inverse_frame_matrix):
    is_ready_(false), activated_frame_id_(0), file_path_(file_path), frame_name_(frame_name),
    frame_matrix_(frame_matrix), inverse_frame_matrix_(inverse_frame_matrix)
{
    if(readAllFrameFromYaml(frame_set_size))
    {
        is_ready_ = true;
    }
    else
    {
        is_ready_ = false;
    }
}

FrameManager::~FrameManager()
{
    frame_set_.clear();
}

bool FrameManager::isReady()
{
    return is_ready_;
}

bool FrameManager::activateFrame(int id)
{
    if(id >= frame_set_.size()
        || !frame_set_[id].is_valid)
    {
        return false;
    }

    if(activated_frame_id_ != id)
    {
        if(updateGlobalFrame(id))
        {
            activated_frame_id_ = id;
        }
        else
        {
            return false;
        }
    }
    
    return true;
}

int FrameManager::getActivatedFrame()
{
    return activated_frame_id_;
}

bool FrameManager::writeFrameToYaml(Frame& frame)
{
    std::string frame_path = getFramePath(frame.id);
    param_.setParam(frame_path + "/id", frame.id);
    param_.setParam(frame_path + "/is_valid", frame.is_valid);
    param_.setParam(frame_path + "/comment", frame.comment);
    std::string point_path = frame_path + std::string("/point");
    param_.setParam(point_path + "/x", frame.data.position.x);
    param_.setParam(point_path + "/y", frame.data.position.y);
    param_.setParam(point_path + "/z", frame.data.position.z);
    std::string euler_path = frame_path + std::string("/euler");
    param_.setParam(euler_path + "/a", frame.data.orientation.a);
    param_.setParam(euler_path + "/b", frame.data.orientation.b);
    param_.setParam(euler_path + "/c", frame.data.orientation.c);

    return param_.dumpParamFile(file_path_.c_str());
}

bool FrameManager::addFrame(Frame& frame)
{
    if(frame.id >= frame_set_.size()
        || frame.id == 0
        || frame_set_[frame.id].is_valid)
    {
        return false;
    }
        
    frame_set_[frame.id].id = frame.id;
    frame_set_[frame.id].is_valid = true;
    memcpy(frame_set_[frame.id].comment, frame.comment, MAX_COMMENT_LENGTH);
    frame_set_[frame.id].data = frame.data;

    return writeFrameToYaml(frame_set_[frame.id]);
}

bool FrameManager::deleteFrame(int id)
{
    if(id >= frame_set_.size()
        || id == 0
        || !frame_set_[id].is_valid
        || activated_frame_id_ == id)   // can't delete the activated frame
    {
        return false;
    }
        
    frame_set_[id].is_valid = false;
    std::string comment("Not used");
    comment.resize(MAX_COMMENT_LENGTH - 1, 0);
    memcpy(frame_set_[id].comment, comment.c_str(), MAX_COMMENT_LENGTH);
    memset(&frame_set_[id].data, 0, sizeof(PoseEuler));

    return writeFrameToYaml(frame_set_[id]);
}

bool FrameManager::updateFrame(Frame& frame)
{
    if(frame.id >= frame_set_.size()
        || frame.id == 0
        || !frame_set_[frame.id].is_valid)
    {
        return false;
    }
        
    frame_set_[frame.id].id = frame.id;
    memcpy(frame_set_[frame.id].comment, frame.comment, MAX_COMMENT_LENGTH);
    frame_set_[frame.id].data = frame.data;

    if(!writeFrameToYaml(frame_set_[frame.id]))
    {
        return false;
    }

    if(frame.id == activated_frame_id_)
    {
        if(!updateGlobalFrame(frame.id))
        {
            return false;
        }
    }
    
    return true;
}

bool FrameManager::getFrame(int id, Frame& frame)
{
    if(frame.id >= frame_set_.size())
    {
        return false;
    }

    frame.id = frame_set_[id].id;
    frame.is_valid = frame_set_[id].is_valid;
    frame.data = frame_set_[id].data;
    memcpy(frame.comment, frame_set_[id].comment, MAX_COMMENT_LENGTH);
    return true;
}

std::string FrameManager::getFramePath(int frame_index)
{
    std::string index_str;
    std::stringstream stream;
    stream << frame_index;
    stream >> index_str;
    return (frame_name_ + index_str);
}

bool FrameManager::readAllFrameFromYaml(int frame_set_size)
{
    if (param_.loadParamFile(file_path_.c_str()))
    {
        for(unsigned int i = 0; i < frame_set_size; ++i)
        {
            std::string frame_path;
            std::string comment;
            Frame frame;
            frame_path = getFramePath(i);
            param_.getParam(frame_path + "/id", frame.id);
            if(frame.id != i)
            {
                is_ready_ = false;
                return false;
            }
            param_.getParam(frame_path + "/is_valid", frame.is_valid);
            param_.getParam(frame_path + "/comment", comment);
            comment.resize(MAX_COMMENT_LENGTH - 1, 0);
            memcpy(frame.comment, comment.c_str(), MAX_COMMENT_LENGTH);
            std::string point_path = frame_path + std::string("/point");
            param_.getParam(point_path + "/x", frame.data.position.x);
            param_.getParam(point_path + "/y", frame.data.position.y);
            param_.getParam(point_path + "/z", frame.data.position.z);
            std::string euler_path = frame_path + std::string("/euler");
            param_.getParam(euler_path + "/a", frame.data.orientation.a);
            param_.getParam(euler_path + "/b", frame.data.orientation.b);
            param_.getParam(euler_path + "/c", frame.data.orientation.c);
            frame_set_.push_back(frame);
        }
        is_ready_ = true;
    }
    else
    {
        FST_ERROR("Lost config file: %s", file_path_.c_str());
        return param_.getLastError();
    }
}

bool FrameManager::updateGlobalFrame(int frame_index)
{
    frame_matrix_.fromPoseEuler(frame_set_[frame_index].data);
    inverse_frame_matrix_ = frame_matrix_;
    return inverse_frame_matrix_.inverse();
}
