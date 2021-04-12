#include "system_model_manager.h"
#include "axis_application_db.h"
#include "servo_db.h"
#include "group_kinematics_db.h"
#include "group_dynamics_db.h"
#include "group_application_db.h"
#include "group_algorithm_db.h"
#include "log_manager_producer.h"

using namespace system_model_space;
using namespace std;
using namespace log_space;

SystemModelManager::SystemModelManager():
    axes_config_ptr_(NULL), groups_config_ptr_(NULL)
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
        if(group_it->second.dynamics_ptr != NULL)
        {
            delete group_it->second.dynamics_ptr;
            group_it->second.dynamics_ptr = NULL;
        }
        if(group_it->second.application_ptr != NULL)
        {
            delete group_it->second.application_ptr;
            group_it->second.application_ptr = NULL;
        }        
        group_it->second.axis_set.clear();
        std::map<std::string, ModelBase*>::iterator algorithm_it;
        for(algorithm_it = group_it->second.algorithm_set.begin(); algorithm_it != group_it->second.algorithm_set.end(); algorithm_it++)
        {
            if(algorithm_it->second != NULL)
            {
                delete algorithm_it->second;
                algorithm_it->second = NULL;
            }
        }        
        group_it->second.algorithm_set.clear();
    }
    group_model_set_.clear();

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
        // sync power/encoder/motor to servo
        /*if(!axis_model.actuator.servo_ptr->syncPowerToServo(axis_model.actuator.power_ptr)
            || !axis_model.actuator.servo_ptr->syncEncoderToServo(axis_model.actuator.encoder_ptr)
            || !axis_model.actuator.servo_ptr->syncMotorToServo(axis_model.actuator.motor_ptr))
        {
            LogProducer::error("SystemModel", "synchronize to servo failed.");
            return false;
        }*/
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
            LogProducer::error("SystemModel", "Loading %s failed.", group_kinematics_file_path.c_str());
            return false;
        }
        // group dynamics
        if(group_config_ref[i].dynamics.compare("NULL") != 0)
        {
            std::string group_dynamics_file_path;
            group_dynamics_file_path.append(group_config_ref[i].root_dir).append("/").append(group_config_ref[i].dynamics).append(".yaml");
            group_model.dynamics_ptr = GroupDynamicsDb::newModel(group_config_ref[i].dynamics, group_dynamics_file_path);
            if(group_model.dynamics_ptr == NULL
                || !group_model.dynamics_ptr->load())
            {
                LogProducer::error("SystemModel", "Loading %s failed.", group_dynamics_file_path.c_str());
                return false;
            }
        }
        else
        {
            group_model.dynamics_ptr = NULL;
        }
        // group application
        std::string group_application_file_path;
        group_application_file_path.append(group_config_ref[i].root_dir).append("/").append(group_config_ref[i].application).append(".yaml");
        group_model.application_ptr = GroupApplicationDb::newModel(group_config_ref[i].application, group_application_file_path);
        if(group_model.application_ptr == NULL
            || !group_model.application_ptr->load())
        {
            LogProducer::error("SystemModel", "Loading %s failed.", group_application_file_path.c_str());
            return false;
        }                    
        // axis set
        for(size_t j = 0; j < group_config_ref[i].axis_id.size(); ++j)
        {
            AxisModel_t* axis_model_ptr = getAxisModel(group_config_ref[i].axis_id[j]);
            if(axis_model_ptr == NULL)
            {
                LogProducer::error("SystemModel", "Get Axie model failed.");
                return false;
            }
            group_model.axis_set.push_back(axis_model_ptr);
        }
        // algorithms dir
        std::string algorithms_root_dir;
        algorithms_root_dir.append(group_config_ref[i].root_dir).append("/algorithms/");
        for(size_t j = 0; j < group_config_ref[i].algorithm.size(); ++j)
        {
            std::string algorithm_file_path;
            algorithm_file_path.append(algorithms_root_dir).append(group_config_ref[i].algorithm[j]).append(".yaml");
            ModelBase* algorithm_ptr = GroupAlgorithmDb::newModel(group_config_ref[i].algorithm[j], algorithm_file_path);
            if(algorithm_ptr == NULL
                || !algorithm_ptr->load())
            {
                LogProducer::error("SystemModel", "Loading %s failed.", algorithm_file_path.c_str());
                return false;
            }
            std::string name_str = group_config_ref[i].algorithm[j];
            size_t dot_pos = name_str.find_first_of('.');
            if(dot_pos != std::string::npos)
            {
                name_str = name_str.substr(0, dot_pos);
            }
            group_model.algorithm_set.insert(std::pair<std::string, ModelBase*>(name_str, algorithm_ptr));
        }
        
        group_model_set_.insert(std::pair<int32_t, GroupModel_t>(group_config_ref[i].group_id, group_model));
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
            || (group_it->second.dynamics_ptr != NULL && !group_it->second.dynamics_ptr->save())
            || (group_it->second.application_ptr != NULL && !group_it->second.application_ptr->save()))
        {
            return false;
        }
        std::map<std::string, ModelBase*>::iterator algorithm_it;
        for(algorithm_it = group_it->second.algorithm_set.begin(); algorithm_it != group_it->second.algorithm_set.end(); algorithm_it++)
        {
            if(algorithm_it->second != NULL && !algorithm_it->second->save())
            {
                return false;
            }
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



