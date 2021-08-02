#include "system_model_manager.h"
#include "axis_application_db.h"
#include "servo_db.h"
#include "group_kinematics_db.h"
#include "group_application_db.h"
#include "log_manager_producer.h"

using namespace system_model_space;
using namespace std;
using namespace log_space;

SystemModelManager::SystemModelManager():
    axes_config_ptr_(NULL), 
    groups_config_ptr_(NULL),
    forces_config_ptr_(NULL)
{

}

SystemModelManager::~SystemModelManager()
{
    std::map<int32_t, AxisModel_t>::iterator axis_it;
    for(axis_it = axis_model_set_.begin(); axis_it != axis_model_set_.end(); ++axis_it)
    {
        if(axis_it->second.application_ptr != NULL)
        {
            delete axis_it->second.application_ptr;
            axis_it->second.application_ptr = NULL;
        }        
        if(axis_it->second.actuator.servo_ptr != NULL)
        {
            delete axis_it->second.actuator.servo_ptr;
            axis_it->second.actuator.servo_ptr = NULL;
        }
    }
    axis_model_set_.clear();

    std::map<int32_t, GroupModel_t>::iterator group_it;
    for(group_it = group_model_set_.begin(); group_it != group_model_set_.end(); ++group_it)
    {
        if(group_it->second.kinematics_ptr != NULL)
        {
            delete group_it->second.kinematics_ptr;
            group_it->second.kinematics_ptr = NULL;
        }
        if(group_it->second.application_ptr != NULL)
        {
            delete group_it->second.application_ptr;
            group_it->second.application_ptr = NULL;
        }        
        group_it->second.axis_set.clear();
    }
    group_model_set_.clear();

    std::map<int32_t, ForceModel_t>::iterator force_it;
    for(force_it = force_model_set_.begin(); force_it != force_model_set_.end(); ++force_it)
    {
        if(force_it->second.force_param_ptr != NULL)
        {
            delete force_it->second.force_param_ptr;
            force_it->second.force_param_ptr = NULL;
        }        
    }
    force_model_set_.clear();

    if(axes_config_ptr_ != NULL)
    {
        delete axes_config_ptr_;
        axes_config_ptr_ = NULL;
    }

    if(groups_config_ptr_ != NULL)
    {
        delete groups_config_ptr_;
        groups_config_ptr_ = NULL;
    }    

    if(forces_config_ptr_ != NULL)
    {
        delete forces_config_ptr_;
        forces_config_ptr_ = NULL;
    }
}

bool SystemModelManager::init()
{
    return config_.load();
}

bool SystemModelManager::load()
{     
    if(axes_config_ptr_ == NULL)
    {
        std::string axes_config_file_path;
        axes_config_file_path.append(config_.system_model_root_dir_).append("/axis/axes_config.xml");
        axes_config_ptr_ = new AxesConfig(axes_config_file_path);
        if(axes_config_ptr_ == NULL)
        {
            LogProducer::error("SystemModel", "Initializing axes_config failed.");
            return false;
        }
    }
    if(!axes_config_ptr_->load())
    {
        LogProducer::error("SystemModel", "Loading axes_config failed.");
        return false;
    }
    if(groups_config_ptr_ == NULL)
    {
        std::string groups_config_file_path;
        groups_config_file_path.append(config_.system_model_root_dir_).append("/group/groups_config.xml");
        groups_config_ptr_ = new GroupsConfig(groups_config_file_path);
        if(groups_config_ptr_ == NULL)
        {
            LogProducer::error("SystemModel", "Initialzing groups_config failed.");
            return false;
        }
    }
    if(!groups_config_ptr_->load())
    {
        LogProducer::error("SystemModel", "Loading groups_config failed.");
        return false;
    }
    if(forces_config_ptr_ == NULL)
    {
        std::string forces_config_file_path;
        forces_config_file_path.append(config_.system_model_root_dir_).append("/force/forces_config.xml");
        forces_config_ptr_ = new ForcesConfig(forces_config_file_path);
        if(forces_config_ptr_ == NULL)
        {
            LogProducer::error("SystemModel", "Initializing forces_config failed.");
            return false;
        }
    }
    if(!forces_config_ptr_->load())
    {
        LogProducer::error("SystemModel", "Loading forces_config failed.");
        return false;
    }

    // load axis info
    std::vector<AxisConfig_t>& axis_config_ref = axes_config_ptr_->getRef();
    for(size_t i = 0; i < axis_config_ref.size(); ++i)
    {
        AxisModel_t axis_model;
        // axis application
        std::string axis_application_file_path;
        axis_application_file_path.append(axis_config_ref[i].root_dir).append("/").append(axis_config_ref[i].application).append(".yaml");
        axis_model.application_ptr = AxisApplicationDb::newModel(axis_config_ref[i].application, axis_application_file_path);
        if(axis_model.application_ptr == NULL
            || !axis_model.application_ptr->load())
        {
            LogProducer::error("SystemModel", "Loading %s failed.", axis_application_file_path.c_str());
            return false;
        }
        // actuator dir
        std::string actuator_root_dir;
        actuator_root_dir.append(axis_config_ref[i].root_dir).append("/actuator/");
        // servo
        std::string servo_model_file_path;
        servo_model_file_path.append(actuator_root_dir).append(axis_config_ref[i].actuator.servo).append(".yaml");  
        axis_model.actuator.servo_ptr = ServoDb::newModel(axis_config_ref[i].actuator.servo, servo_model_file_path);
        if(axis_model.actuator.servo_ptr == NULL
            || !axis_model.actuator.servo_ptr->load())
        {
            LogProducer::error("SystemModel", "Loading %s failed.", servo_model_file_path.c_str());
            return false;
        }

        axis_model_set_.insert(std::pair<int32_t, AxisModel_t>(axis_config_ref[i].axis_id, axis_model));
    }
    // load group info
    std::vector<GroupConfig_t>& group_config_ref = groups_config_ptr_->getRef();
    for(size_t i = 0; i < group_config_ref.size(); ++i)
    {
        GroupModel_t group_model;
        // group kinematics
        std::string group_kinematics_file_path;
        group_kinematics_file_path.append(group_config_ref[i].root_dir).append("/").append(group_config_ref[i].kinematics).append(".yaml");
        group_model.kinematics_ptr = GroupKinematicsDb::newModel(group_config_ref[i].kinematics, group_kinematics_file_path);
        if(group_model.kinematics_ptr == NULL
            || !group_model.kinematics_ptr->load())
        {
            LogProducer::error("SystemModel", "Loading kinematics %s failed.", group_kinematics_file_path.c_str());
            return false;
        }
        // group application
        std::string group_application_file_path;
        group_application_file_path.append(group_config_ref[i].root_dir).append("/").append(group_config_ref[i].application).append(".yaml");
        group_model.application_ptr = GroupApplicationDb::newModel(group_config_ref[i].application, group_application_file_path);
        if(group_model.application_ptr == NULL
            || !group_model.application_ptr->load())
        {
            LogProducer::error("SystemModel", "Loading application %s failed.", group_application_file_path.c_str());
            return false;
        }                    
        // axis set
        for(size_t j = 0; j < group_config_ref[i].axis_id.size(); ++j)
        {
            AxisModel_t* axis_model_ptr = getAxisModel(group_config_ref[i].axis_id[j]);
            if(axis_model_ptr == NULL)
            {
                LogProducer::error("SystemModel", "Get Axis model failed.");
                return false;
            }
            group_model.axis_set.push_back(axis_model_ptr);
        }
        
        group_model_set_.insert(std::pair<int32_t, GroupModel_t>(group_config_ref[i].group_id, group_model));
    }

    // load forcee info
    std::vector<ForceConfig_t>& force_config_ref = forces_config_ptr_->getRef();
    for(size_t i = 0; i < force_config_ref.size(); ++i)
    {
        ForceModel_t force_model;
        // stator param
        std::string force_model_file_path;
        force_model_file_path.append(force_config_ref[i].root_dir).append("/").append(force_config_ref[i].parameter).append(".yaml");  
        force_model.force_param_ptr = ServoDb::newModel(force_config_ref[i].parameter, force_model_file_path);
        if(force_model.force_param_ptr == NULL
            || !force_model.force_param_ptr->load())
        {
            LogProducer::error("SystemModel", "Loading %s failed.", force_model_file_path.c_str());
            return false;
        }
        force_model_set_.insert(std::pair<int32_t, ForceModel_t>(force_config_ref[i].force_id, force_model));
    }

    return true;
}

bool SystemModelManager::save()
{
    std::map<int32_t, AxisModel_t>::iterator axis_it;
    for(axis_it = axis_model_set_.begin(); axis_it != axis_model_set_.end(); ++axis_it)
    {
        if((axis_it->second.application_ptr != NULL && !axis_it->second.application_ptr->save())
            || (axis_it->second.actuator.servo_ptr != NULL && !axis_it->second.actuator.servo_ptr->save()))
        {
            return false;
        }
    }
    
    std::map<int32_t, GroupModel_t>::iterator group_it;
    for(group_it = group_model_set_.begin(); group_it != group_model_set_.end(); ++group_it)
    {
        if((group_it->second.kinematics_ptr != NULL && !group_it->second.kinematics_ptr->save())
            || (group_it->second.application_ptr != NULL && !group_it->second.application_ptr->save()))
        {
            return false;
        }   
    }
    return true;
}

AxesConfig* SystemModelManager::getAxesConfig()
{
    return axes_config_ptr_;
}

GroupsConfig* SystemModelManager::getGroupsConfig()
{
    return groups_config_ptr_;
}

ForcesConfig* SystemModelManager::getForcesConfig()
{
    return forces_config_ptr_;
}

AxisModel_t* SystemModelManager::getAxisModel(int32_t axis_id)
{
    std::map<int32_t, AxisModel_t>::iterator it = axis_model_set_.find(axis_id);
    if(it != axis_model_set_.end())
    {
        return &it->second;
    }
    else
    {
        return NULL;
    }
}

GroupModel_t* SystemModelManager::getGroupModel(int32_t group_id)
{
    std::map<int32_t, GroupModel_t>::iterator it = group_model_set_.find(group_id);
    if(it != group_model_set_.end())
    {
        return &it->second;
    }
    else
    {
        return NULL;
    }
}

ForceModel_t* SystemModelManager::getForceModel(int32_t force_id)
{
    std::map<int32_t, ForceModel_t>::iterator it = force_model_set_.find(force_id);
    if(it != force_model_set_.end())
    {
        return &it->second;
    }
    else
    {
        return NULL;
    }
}


