#include "system_model_manager.h"
#include "axis_application_param_1001.h"
#include "group_kinematics_param_1000.h"
#include <iostream>

using namespace system_model_space;
using namespace std;

int main()
{
    SystemModelManager* manager_ptr = new SystemModelManager();   
    if(!manager_ptr->init())
    {
        std::cout<<"manager_ptr init failed"<<std::endl;
        return -1;
    }  
    if(!manager_ptr->load())
    {
        std::cout<<"manager_ptr load failed"<<std::endl;
        return -1;
    }
    AxisModel_t* axis_model_ptr = manager_ptr->getAxisModel(1);
    if (axis_model_ptr == NULL)
        return -1;
    int32_t param_value;
    if(axis_model_ptr->application_ptr->get(AxisApplication1001__app_max_vel, &param_value))
    {
        std::cout<<"get value = "<<param_value<<std::endl;
    }
    param_value = 2000;
    GroupModel_t* group_model_ptr = manager_ptr->getGroupModel(0);
    if(group_model_ptr != NULL
        && group_model_ptr->kinematics_ptr->set(GroupKinematics1000__A0, param_value))
    {
        std::cout<<"set value = "<<param_value<<std::endl;
        if(manager_ptr->save())
        {
            std::cout<<"manager_ptr save success"<<std::endl;
        }
    }
    else
    {
        std::cout<<"group_model_ptr == NULL"<<std::endl;
    }
    delete manager_ptr;
    manager_ptr = NULL;
    return 0;
}

