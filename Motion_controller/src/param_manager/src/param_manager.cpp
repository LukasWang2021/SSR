#include "param_manager.h"
#include "common_file_path.h"
#include <cstring>
#include <iostream>


using namespace std;
using namespace fst_mc;
using namespace fst_parameter;

#define USER_PARAM_NAME_1 "fine positioning check threshold" 
#define USER_PARAM_NAME_2 "fine stabilization time check threshold"
#define USER_PARAM_NAME_3 "PR reg max number"
#define USER_PARAM_NAME_4 "MR reg max number"
#define USER_PARAM_NAME_5 "SR reg max number"
#define USER_PARAM_NAME_6 "R reg max number"
#define USER_PARAM_NAME_7 "user coordinate max number"
#define USER_PARAM_NAME_8 "tool coordinate max number"
#define USER_PARAM_NAME_9 "io mapping max number"

#define MANU_PARAM_NAME_1 "trajectory fifo size"
#define MANU_PARAM_NAME_2 "duration of segment in trajectory fifo"
#define MANU_PARAM_NAME_3 "cartesian accuracy factor"
#define MANU_PARAM_NAME_4 "joint accuracy factor"
#define MANU_PARAM_NAME_5 "trajectory piece max number"
#define MANU_PARAM_NAME_6 "cartesian path interval"
#define MANU_PARAM_NAME_7 "joint path interval"
#define MANU_PARAM_NAME_8 "orientation path interval"
#define MANU_PARAM_NAME_9 "orientation interpolation valve"
#define MANU_PARAM_NAME_10 "smooth conservative acceleration"
#define MANU_PARAM_NAME_11 "start time factor"
#define MANU_PARAM_NAME_12 "stop time factor"
#define MANU_PARAM_NAME_13 "switch of dynamics"
#define MANU_PARAM_NAME_14 "max cartesian accerleration"
#define MANU_PARAM_NAME_15 "axis1 - switch of friction compensation"
#define MANU_PARAM_NAME_16 "axis1 - switch of acceleration feedforward"
#define MANU_PARAM_NAME_17 "axis1 - switch of gravity compensation"
#define MANU_PARAM_NAME_18 "axis1 - switch of velocity compensation"
#define MANU_PARAM_NAME_19 "axis2 - switch of friction compensation"
#define MANU_PARAM_NAME_20 "axis2 - switch of acceleration feedforward"
#define MANU_PARAM_NAME_21 "axis2 - switch of gravity compensation"
#define MANU_PARAM_NAME_22 "axis2 - switch of velocity compensation"
#define MANU_PARAM_NAME_23 "axis3 - switch of friction compensation"
#define MANU_PARAM_NAME_24 "axis3 - switch of acceleration feedforward"
#define MANU_PARAM_NAME_25 "axis3 - switch of gravity compensation"
#define MANU_PARAM_NAME_26 "axis3 - switch of velocity compensation"
#define MANU_PARAM_NAME_27 "axis4 - switch of friction compensation"
#define MANU_PARAM_NAME_28 "axis4 - switch of acceleration feedforward"
#define MANU_PARAM_NAME_29 "axis4 - switch of gravity compensation"
#define MANU_PARAM_NAME_30 "axis4 - switch of velocity compensation"
#define MANU_PARAM_NAME_31 "axis5 - switch of friction compensation"
#define MANU_PARAM_NAME_32 "axis5 - switch of acceleration feedforward"
#define MANU_PARAM_NAME_33 "axis5 - switch of gravity compensation"
#define MANU_PARAM_NAME_34 "axis5 - switch of velocity compensation"
#define MANU_PARAM_NAME_35 "axis6 - switch of friction compensation"
#define MANU_PARAM_NAME_36 "axis6 - switch of acceleration feedforward"
#define MANU_PARAM_NAME_37 "axis6 - switch of gravity compensation"
#define MANU_PARAM_NAME_38 "axis6 - switch of velocity compensation"

#define USER_PARAM_DATA_1 "stable_with_fine/threshold"
#define USER_PARAM_DATA_2 "stable_with_fine/cycle"
#define USER_PARAM_DATA_3 "pr_reg_number"
#define USER_PARAM_DATA_4 "mr_reg_number"
#define USER_PARAM_DATA_5 "sr_reg_number"
#define USER_PARAM_DATA_6 "r_reg_number"
#define USER_PARAM_DATA_7 "max_number_of_coords"
#define USER_PARAM_DATA_8 "max_number_of_tools"
#define USER_PARAM_DATA_9 "max_mapping_number"

#define MANU_PARAM_DATA_1 "trajectory_fifo_size"
#define MANU_PARAM_DATA_2 "duration_of_segment_in_trajectory_fifo"
#define MANU_PARAM_DATA_3 "accuracy_cartesian_factor"
#define MANU_PARAM_DATA_4 "accuracy_joint_factor"
#define MANU_PARAM_DATA_5 "max_traj_points_num"
#define MANU_PARAM_DATA_6 "path_interval"
#define MANU_PARAM_DATA_7 "joint_interval"
#define MANU_PARAM_DATA_8 "angle_interval"
#define MANU_PARAM_DATA_9 "angle_valve"
#define MANU_PARAM_DATA_10 "conservative_acc"
#define MANU_PARAM_DATA_11 "time_factor_first"
#define MANU_PARAM_DATA_12 "time_factor_last"
#define MANU_PARAM_DATA_13 "is_fake_dynamics"
#define MANU_PARAM_DATA_14 "max_cartesian_acc"
#define MANU_PARAM_DATA_SERVO "servo/stored_param"

#define FILE_BASE_GROUP (std::string(AXIS_GROUP_MODEL_DIR) + "base_group.yaml")
#define FILE_REG_MANAGER (std::string(COMPONENT_PARAM_FILE_DIR) + "reg_manager.yaml")
#define FILE_COORDINATE_MANAGER (std::string(COMPONENT_PARAM_FILE_DIR) + "coordinate_manager.yaml")
#define FILE_TOOL_MANAGER (std::string(COMPONENT_PARAM_FILE_DIR) + "tool_manager.yaml")
#define FILE_IO_MAPPING (std::string(COMPONENT_PARAM_FILE_DIR) + "io_mapping.yaml")
#define FILE_SEGMENT_ALG (std::string(COMPONENT_PARAM_FILE_DIR) + "segment_alg.yaml")
#define FILE_SERVO_PARAM (std::string(SERVO_DIR) + "servo_param.yaml")

ParamManager::ParamManager()
{

}

ParamManager::~ParamManager()
{

}

ErrorCode ParamManager::init()
{
    user_param_list_.clear();
    manu_param_list_.clear();
    
    ParamInfo_t param_info;
    int data_int;
    double data_double;
    bool data_bool;

    // user param
    if(!yaml_help_.loadParamFile(FILE_BASE_GROUP)) return PARAM_MANAGER_INIT_FAILED;    

    if(!yaml_help_.getParam(USER_PARAM_DATA_1, data_double)) return PARAM_MANAGER_INIT_FAILED;
    strcpy(param_info.name, USER_PARAM_NAME_1);
    param_info.type = PARAM_INFO_DOUBLE;
    memcpy(param_info.data, &data_double, sizeof(double));
    user_param_list_.push_back(param_info);

    if(!yaml_help_.getParam(USER_PARAM_DATA_2, data_int)) return PARAM_MANAGER_INIT_FAILED;
    strcpy(param_info.name, USER_PARAM_NAME_2);
    param_info.type = PARAM_INFO_INT;
    memcpy(param_info.data, &data_int, sizeof(int));
    user_param_list_.push_back(param_info);

    // manu param
    if(!yaml_help_.getParam(MANU_PARAM_DATA_1, data_int)) return PARAM_MANAGER_INIT_FAILED;
    strcpy(param_info.name, MANU_PARAM_NAME_1);
    param_info.type = PARAM_INFO_INT;
    memcpy(param_info.data, &data_int, sizeof(int));
    manu_param_list_.push_back(param_info);

    if(!yaml_help_.getParam(MANU_PARAM_DATA_2, data_double)) return PARAM_MANAGER_INIT_FAILED;
    strcpy(param_info.name, MANU_PARAM_NAME_2);
    param_info.type = PARAM_INFO_DOUBLE;
    memcpy(param_info.data, &data_double, sizeof(double));
    manu_param_list_.push_back(param_info);    

    // user param
    if(!yaml_help_.loadParamFile(FILE_REG_MANAGER)) return PARAM_MANAGER_INIT_FAILED;
    
    if(!yaml_help_.getParam(USER_PARAM_DATA_3, data_int)) return PARAM_MANAGER_INIT_FAILED;
    strcpy(param_info.name, USER_PARAM_NAME_3);
    param_info.type = PARAM_INFO_INT;
    memcpy(param_info.data, &data_int, sizeof(int));
    user_param_list_.push_back(param_info);

    if(!yaml_help_.getParam(USER_PARAM_DATA_4, data_int)) return PARAM_MANAGER_INIT_FAILED;
    strcpy(param_info.name, USER_PARAM_NAME_4);
    param_info.type = PARAM_INFO_INT;
    memcpy(param_info.data, &data_int, sizeof(int));
    user_param_list_.push_back(param_info);

    if(!yaml_help_.getParam(USER_PARAM_DATA_5, data_int)) return PARAM_MANAGER_INIT_FAILED;
    strcpy(param_info.name, USER_PARAM_NAME_5);
    param_info.type = PARAM_INFO_INT;
    memcpy(param_info.data, &data_int, sizeof(int));
    user_param_list_.push_back(param_info);    

    if(!yaml_help_.getParam(USER_PARAM_DATA_6, data_int)) return PARAM_MANAGER_INIT_FAILED;
    strcpy(param_info.name, USER_PARAM_NAME_6);
    param_info.type = PARAM_INFO_INT;
    memcpy(param_info.data, &data_int, sizeof(int));
    user_param_list_.push_back(param_info);

    if(!yaml_help_.loadParamFile(FILE_COORDINATE_MANAGER)) return PARAM_MANAGER_INIT_FAILED;
    
    if(!yaml_help_.getParam(USER_PARAM_DATA_7, data_int)) return PARAM_MANAGER_INIT_FAILED;
    strcpy(param_info.name, USER_PARAM_NAME_7);
    param_info.type = PARAM_INFO_INT;
    memcpy(param_info.data, &data_int, sizeof(int));
    user_param_list_.push_back(param_info);

    if(!yaml_help_.loadParamFile(FILE_TOOL_MANAGER)) return PARAM_MANAGER_INIT_FAILED;
    
    if(!yaml_help_.getParam(USER_PARAM_DATA_8, data_int)) return PARAM_MANAGER_INIT_FAILED;
    strcpy(param_info.name, USER_PARAM_NAME_8);
    param_info.type = PARAM_INFO_INT;
    memcpy(param_info.data, &data_int, sizeof(int));
    user_param_list_.push_back(param_info);

    if(!yaml_help_.loadParamFile(FILE_IO_MAPPING)) return PARAM_MANAGER_INIT_FAILED;
    
    if(!yaml_help_.getParam(USER_PARAM_DATA_9, data_int)) return PARAM_MANAGER_INIT_FAILED;
    strcpy(param_info.name, USER_PARAM_NAME_9);
    param_info.type = PARAM_INFO_INT;
    memcpy(param_info.data, &data_int, sizeof(int));
    user_param_list_.push_back(param_info);    

    // manu param
    if(!yaml_help_.loadParamFile(FILE_SEGMENT_ALG)) return PARAM_MANAGER_INIT_FAILED;
    
    if(!yaml_help_.getParam(MANU_PARAM_DATA_3, data_int)) return PARAM_MANAGER_INIT_FAILED;
    strcpy(param_info.name, MANU_PARAM_NAME_3);
    param_info.type = PARAM_INFO_INT;
    memcpy(param_info.data, &data_int, sizeof(int));
    manu_param_list_.push_back(param_info); 

    if(!yaml_help_.getParam(MANU_PARAM_DATA_4, data_int)) return PARAM_MANAGER_INIT_FAILED;
    strcpy(param_info.name, MANU_PARAM_NAME_4);
    param_info.type = PARAM_INFO_INT;
    memcpy(param_info.data, &data_int, sizeof(int));
    manu_param_list_.push_back(param_info); 

    if(!yaml_help_.getParam(MANU_PARAM_DATA_5, data_int)) return PARAM_MANAGER_INIT_FAILED;
    strcpy(param_info.name, MANU_PARAM_NAME_5);
    param_info.type = PARAM_INFO_INT;
    memcpy(param_info.data, &data_int, sizeof(int));
    manu_param_list_.push_back(param_info); 

    if(!yaml_help_.getParam(MANU_PARAM_DATA_6, data_double)) return PARAM_MANAGER_INIT_FAILED;
    strcpy(param_info.name, MANU_PARAM_NAME_6);
    param_info.type = PARAM_INFO_DOUBLE;
    memcpy(param_info.data, &data_double, sizeof(double));
    manu_param_list_.push_back(param_info); 

    if(!yaml_help_.getParam(MANU_PARAM_DATA_7, data_double)) return PARAM_MANAGER_INIT_FAILED;
    strcpy(param_info.name, MANU_PARAM_NAME_7);
    param_info.type = PARAM_INFO_DOUBLE;
    memcpy(param_info.data, &data_double, sizeof(double));
    manu_param_list_.push_back(param_info); 

    if(!yaml_help_.getParam(MANU_PARAM_DATA_8, data_double)) return PARAM_MANAGER_INIT_FAILED;
    strcpy(param_info.name, MANU_PARAM_NAME_8);
    param_info.type = PARAM_INFO_DOUBLE;
    memcpy(param_info.data, &data_double, sizeof(double));
    manu_param_list_.push_back(param_info);     

    if(!yaml_help_.getParam(MANU_PARAM_DATA_9, data_double)) return PARAM_MANAGER_INIT_FAILED;
    strcpy(param_info.name, MANU_PARAM_NAME_9);
    param_info.type = PARAM_INFO_DOUBLE;
    memcpy(param_info.data, &data_double, sizeof(double));
    manu_param_list_.push_back(param_info); 

    if(!yaml_help_.getParam(MANU_PARAM_DATA_10, data_double)) return PARAM_MANAGER_INIT_FAILED;
    strcpy(param_info.name, MANU_PARAM_NAME_10);
    param_info.type = PARAM_INFO_DOUBLE;
    memcpy(param_info.data, &data_double, sizeof(double));
    manu_param_list_.push_back(param_info);     

    if(!yaml_help_.getParam(MANU_PARAM_DATA_11, data_double)) return PARAM_MANAGER_INIT_FAILED;
    strcpy(param_info.name, MANU_PARAM_NAME_11);
    param_info.type = PARAM_INFO_DOUBLE;
    memcpy(param_info.data, &data_double, sizeof(double));
    manu_param_list_.push_back(param_info); 

    if(!yaml_help_.getParam(MANU_PARAM_DATA_12, data_double)) return PARAM_MANAGER_INIT_FAILED;
    strcpy(param_info.name, MANU_PARAM_NAME_12);
    param_info.type = PARAM_INFO_DOUBLE;
    memcpy(param_info.data, &data_double, sizeof(double));
    manu_param_list_.push_back(param_info); 

    if(!yaml_help_.getParam(MANU_PARAM_DATA_13, data_bool)) return PARAM_MANAGER_INIT_FAILED;
    strcpy(param_info.name, MANU_PARAM_NAME_13);
    param_info.type = PARAM_INFO_BOOL;
    memcpy(param_info.data, &data_bool, sizeof(bool));
    manu_param_list_.push_back(param_info);    

    if(!yaml_help_.getParam(MANU_PARAM_DATA_14, data_double)) return PARAM_MANAGER_INIT_FAILED;
    strcpy(param_info.name, MANU_PARAM_NAME_14);
    param_info.type = PARAM_INFO_DOUBLE;
    memcpy(param_info.data, &data_double, sizeof(double));
    manu_param_list_.push_back(param_info); 

    if(!yaml_help_.loadParamFile(FILE_SERVO_PARAM)) return PARAM_MANAGER_INIT_FAILED;
    fst_parameter::ParamValue servo_data;
    if(!yaml_help_.getParam(MANU_PARAM_DATA_SERVO, servo_data)) return PARAM_MANAGER_INIT_FAILED;

    strcpy(param_info.name, MANU_PARAM_NAME_15);
    param_info.type = PARAM_INFO_INT;
    memcpy(param_info.data, &servo_data[93], sizeof(int));
    manu_param_list_.push_back(param_info);

    strcpy(param_info.name, MANU_PARAM_NAME_16);
    param_info.type = PARAM_INFO_INT;
    memcpy(param_info.data, &servo_data[96], sizeof(int));
    manu_param_list_.push_back(param_info);

    strcpy(param_info.name, MANU_PARAM_NAME_17);
    param_info.type = PARAM_INFO_INT;
    memcpy(param_info.data, &servo_data[98], sizeof(int));
    manu_param_list_.push_back(param_info); 

    strcpy(param_info.name, MANU_PARAM_NAME_18);
    param_info.type = PARAM_INFO_INT;
    memcpy(param_info.data, &servo_data[132], sizeof(int));
    manu_param_list_.push_back(param_info);
    
    strcpy(param_info.name, MANU_PARAM_NAME_19);
    param_info.type = PARAM_INFO_INT;
    memcpy(param_info.data, &servo_data[285], sizeof(int));
    manu_param_list_.push_back(param_info);

    strcpy(param_info.name, MANU_PARAM_NAME_20);
    param_info.type = PARAM_INFO_INT;
    memcpy(param_info.data, &servo_data[288], sizeof(int));
    manu_param_list_.push_back(param_info);

    strcpy(param_info.name, MANU_PARAM_NAME_21);
    param_info.type = PARAM_INFO_INT;
    memcpy(param_info.data, &servo_data[290], sizeof(int));
    manu_param_list_.push_back(param_info); 

    strcpy(param_info.name, MANU_PARAM_NAME_22);
    param_info.type = PARAM_INFO_INT;
    memcpy(param_info.data, &servo_data[324], sizeof(int));
    manu_param_list_.push_back(param_info);

    strcpy(param_info.name, MANU_PARAM_NAME_23);
    param_info.type = PARAM_INFO_INT;
    memcpy(param_info.data, &servo_data[477], sizeof(int));
    manu_param_list_.push_back(param_info);

    strcpy(param_info.name, MANU_PARAM_NAME_24);
    param_info.type = PARAM_INFO_INT;
    memcpy(param_info.data, &servo_data[480], sizeof(int));
    manu_param_list_.push_back(param_info);

    strcpy(param_info.name, MANU_PARAM_NAME_25);
    param_info.type = PARAM_INFO_INT;
    memcpy(param_info.data, &servo_data[482], sizeof(int));
    manu_param_list_.push_back(param_info); 

    strcpy(param_info.name, MANU_PARAM_NAME_26);
    param_info.type = PARAM_INFO_INT;
    memcpy(param_info.data, &servo_data[516], sizeof(int));
    manu_param_list_.push_back(param_info);

    strcpy(param_info.name, MANU_PARAM_NAME_27);
    param_info.type = PARAM_INFO_INT;
    memcpy(param_info.data, &servo_data[669], sizeof(int));
    manu_param_list_.push_back(param_info);

    strcpy(param_info.name, MANU_PARAM_NAME_28);
    param_info.type = PARAM_INFO_INT;
    memcpy(param_info.data, &servo_data[672], sizeof(int));
    manu_param_list_.push_back(param_info);

    strcpy(param_info.name, MANU_PARAM_NAME_29);
    param_info.type = PARAM_INFO_INT;
    memcpy(param_info.data, &servo_data[674], sizeof(int));
    manu_param_list_.push_back(param_info); 

    strcpy(param_info.name, MANU_PARAM_NAME_30);
    param_info.type = PARAM_INFO_INT;
    memcpy(param_info.data, &servo_data[708], sizeof(int));
    manu_param_list_.push_back(param_info);   

    strcpy(param_info.name, MANU_PARAM_NAME_31);
    param_info.type = PARAM_INFO_INT;
    memcpy(param_info.data, &servo_data[861], sizeof(int));
    manu_param_list_.push_back(param_info);

    strcpy(param_info.name, MANU_PARAM_NAME_32);
    param_info.type = PARAM_INFO_INT;
    memcpy(param_info.data, &servo_data[864], sizeof(int));
    manu_param_list_.push_back(param_info);

    strcpy(param_info.name, MANU_PARAM_NAME_33);
    param_info.type = PARAM_INFO_INT;
    memcpy(param_info.data, &servo_data[866], sizeof(int));
    manu_param_list_.push_back(param_info); 

    strcpy(param_info.name, MANU_PARAM_NAME_34);
    param_info.type = PARAM_INFO_INT;
    memcpy(param_info.data, &servo_data[900], sizeof(int));
    manu_param_list_.push_back(param_info);      

    strcpy(param_info.name, MANU_PARAM_NAME_35);
    param_info.type = PARAM_INFO_INT;
    memcpy(param_info.data, &servo_data[1053], sizeof(int));
    manu_param_list_.push_back(param_info);

    strcpy(param_info.name, MANU_PARAM_NAME_36);
    param_info.type = PARAM_INFO_INT;
    memcpy(param_info.data, &servo_data[1056], sizeof(int));
    manu_param_list_.push_back(param_info);

    strcpy(param_info.name, MANU_PARAM_NAME_37);
    param_info.type = PARAM_INFO_INT;
    memcpy(param_info.data, &servo_data[1058], sizeof(int));
    manu_param_list_.push_back(param_info); 

    strcpy(param_info.name, MANU_PARAM_NAME_38);
    param_info.type = PARAM_INFO_INT;
    memcpy(param_info.data, &servo_data[1092], sizeof(int));
    manu_param_list_.push_back(param_info);

    return SUCCESS;
}

std::vector<ParamInfo_t> ParamManager::getParamInfoList(ParamGroup_e param_group)
{
    switch(param_group)
    {
        case PARAM_GROUP_MANU:
            return manu_param_list_;
        case PARAM_GROUP_USER:
            return user_param_list_;
        default:
            return dummy_list_;            
    }
}

ErrorCode ParamManager::setParamInfo(ParamGroup_e param_group, ParamInfo_t& param_info)
{
    int data_int;
    double data_double;
    bool data_bool;
    switch(param_group)
    {
        case PARAM_GROUP_MANU:
        {
            if(strcmp(param_info.name, MANU_PARAM_NAME_1) == 0)
            {
                data_int = *((int*)param_info.data);
                if(!yaml_help_.loadParamFile(FILE_BASE_GROUP)
                    || !yaml_help_.setParam(MANU_PARAM_DATA_1, data_int)
                    || !yaml_help_.dumpParamFile(FILE_BASE_GROUP)) 
                {
                    return PARAM_MANAGER_SET_PARAM_FAILED;
                }
                memcpy(manu_param_list_[0].data, &data_int, sizeof(int));
                return SUCCESS;
            }
            if(strcmp(param_info.name, MANU_PARAM_NAME_2) == 0)
            {
                data_double = *((double*)param_info.data);
                if(!yaml_help_.loadParamFile(FILE_BASE_GROUP)
                    || !yaml_help_.setParam(MANU_PARAM_DATA_2, data_double)
                    || !yaml_help_.dumpParamFile(FILE_BASE_GROUP)) 
                {
                    return PARAM_MANAGER_SET_PARAM_FAILED;
                }
                memcpy(manu_param_list_[1].data, &data_double, sizeof(double));
                return SUCCESS;
            }    
            if(strcmp(param_info.name, MANU_PARAM_NAME_3) == 0)
            {
                data_int = *((int*)param_info.data);
                if(!yaml_help_.loadParamFile(FILE_SEGMENT_ALG)
                    || !yaml_help_.setParam(MANU_PARAM_DATA_3, data_int)
                    || !yaml_help_.dumpParamFile(FILE_SEGMENT_ALG)) 
                {
                    return PARAM_MANAGER_SET_PARAM_FAILED;
                }
                memcpy(manu_param_list_[2].data, &data_int, sizeof(int));
                return SUCCESS;
            }
            if(strcmp(param_info.name, MANU_PARAM_NAME_4) == 0)
            {
                data_int = *((int*)param_info.data);
                if(!yaml_help_.loadParamFile(FILE_SEGMENT_ALG)
                    || !yaml_help_.setParam(MANU_PARAM_DATA_4, data_int)
                    || !yaml_help_.dumpParamFile(FILE_SEGMENT_ALG)) 
                {
                    return PARAM_MANAGER_SET_PARAM_FAILED;
                }
                memcpy(manu_param_list_[3].data, &data_int, sizeof(int));
                return SUCCESS;
            }
            if(strcmp(param_info.name, MANU_PARAM_NAME_5) == 0)
            {
                data_int = *((int*)param_info.data);
                if(!yaml_help_.loadParamFile(FILE_SEGMENT_ALG)
                    || !yaml_help_.setParam(MANU_PARAM_DATA_5, data_int)
                    || !yaml_help_.dumpParamFile(FILE_SEGMENT_ALG)) 
                {
                    return PARAM_MANAGER_SET_PARAM_FAILED;
                }
                memcpy(manu_param_list_[4].data, &data_int, sizeof(int));
                return SUCCESS;
            }
            if(strcmp(param_info.name, MANU_PARAM_NAME_6) == 0)
            {
                data_double = *((double*)param_info.data);
                if(!yaml_help_.loadParamFile(FILE_SEGMENT_ALG)
                    || !yaml_help_.setParam(MANU_PARAM_DATA_6, data_double)
                    || !yaml_help_.dumpParamFile(FILE_SEGMENT_ALG)) 
                {
                    return PARAM_MANAGER_SET_PARAM_FAILED;
                }
                memcpy(manu_param_list_[5].data, &data_double, sizeof(double));
                return SUCCESS;
            }
            if(strcmp(param_info.name, MANU_PARAM_NAME_7) == 0)
            {
                data_double = *((double*)param_info.data);
                if(!yaml_help_.loadParamFile(FILE_SEGMENT_ALG)
                    || !yaml_help_.setParam(MANU_PARAM_DATA_7, data_double)
                    || !yaml_help_.dumpParamFile(FILE_SEGMENT_ALG)) 
                {
                    return PARAM_MANAGER_SET_PARAM_FAILED;
                }
                memcpy(manu_param_list_[6].data, &data_double, sizeof(double));
                return SUCCESS;
            }
            if(strcmp(param_info.name, MANU_PARAM_NAME_8) == 0)
            {
                data_double = *((double*)param_info.data);
                if(!yaml_help_.loadParamFile(FILE_SEGMENT_ALG)
                    || !yaml_help_.setParam(MANU_PARAM_DATA_8, data_double)
                    || !yaml_help_.dumpParamFile(FILE_SEGMENT_ALG)) 
                {
                    return PARAM_MANAGER_SET_PARAM_FAILED;
                }
                memcpy(manu_param_list_[7].data, &data_double, sizeof(double));
                return SUCCESS;
            }
            if(strcmp(param_info.name, MANU_PARAM_NAME_9) == 0)
            {
                data_double = *((double*)param_info.data);
                if(!yaml_help_.loadParamFile(FILE_SEGMENT_ALG)
                    || !yaml_help_.setParam(MANU_PARAM_DATA_9, data_double)
                    || !yaml_help_.dumpParamFile(FILE_SEGMENT_ALG)) 
                {
                    return PARAM_MANAGER_SET_PARAM_FAILED;
                }
                memcpy(manu_param_list_[8].data, &data_double, sizeof(double));
                return SUCCESS;
            }
            if(strcmp(param_info.name, MANU_PARAM_NAME_10) == 0)
            {
                data_double = *((double*)param_info.data);
                if(!yaml_help_.loadParamFile(FILE_SEGMENT_ALG)
                    || !yaml_help_.setParam(MANU_PARAM_DATA_10, data_double)
                    || !yaml_help_.dumpParamFile(FILE_SEGMENT_ALG)) 
                {
                    return PARAM_MANAGER_SET_PARAM_FAILED;
                }
                memcpy(manu_param_list_[9].data, &data_double, sizeof(double));
                return SUCCESS;
            }
            if(strcmp(param_info.name, MANU_PARAM_NAME_11) == 0)
            {
                data_double = *((double*)param_info.data);
                if(!yaml_help_.loadParamFile(FILE_SEGMENT_ALG)
                    || !yaml_help_.setParam(MANU_PARAM_DATA_11, data_double)
                    || !yaml_help_.dumpParamFile(FILE_SEGMENT_ALG)) 
                {
                    return PARAM_MANAGER_SET_PARAM_FAILED;
                }
                memcpy(manu_param_list_[10].data, &data_double, sizeof(double));
                return SUCCESS;
            }
            if(strcmp(param_info.name, MANU_PARAM_NAME_12) == 0)
            {
                data_double = *((double*)param_info.data);
                if(!yaml_help_.loadParamFile(FILE_SEGMENT_ALG)
                    || !yaml_help_.setParam(MANU_PARAM_DATA_12, data_double)
                    || !yaml_help_.dumpParamFile(FILE_SEGMENT_ALG)) 
                {
                    return PARAM_MANAGER_SET_PARAM_FAILED;
                }
                memcpy(manu_param_list_[11].data, &data_double, sizeof(double));
                return SUCCESS;
            }
            if(strcmp(param_info.name, MANU_PARAM_NAME_13) == 0)
            {
                data_bool = *((bool*)param_info.data);
                if(!yaml_help_.loadParamFile(FILE_SEGMENT_ALG)
                    || !yaml_help_.setParam(MANU_PARAM_DATA_13, data_bool)
                    || !yaml_help_.dumpParamFile(FILE_SEGMENT_ALG)) 
                {
                    return PARAM_MANAGER_SET_PARAM_FAILED;
                }
                memcpy(manu_param_list_[12].data, &data_bool, sizeof(bool));
                return SUCCESS;
            }
            if(strcmp(param_info.name, MANU_PARAM_NAME_14) == 0)
            {
                data_double = *((double*)param_info.data);
                if(!yaml_help_.loadParamFile(FILE_SEGMENT_ALG)
                    || !yaml_help_.setParam(MANU_PARAM_DATA_14, data_double)
                    || !yaml_help_.dumpParamFile(FILE_SEGMENT_ALG)) 
                {
                    return PARAM_MANAGER_SET_PARAM_FAILED;
                }
                memcpy(manu_param_list_[13].data, &data_double, sizeof(double));
                return SUCCESS;
            }
            
            fst_parameter::ParamValue servo_data;
            // servo param - axis1
            if(strcmp(param_info.name, MANU_PARAM_NAME_15) == 0)
            {
                data_int = *((int*)param_info.data);
                if(!yaml_help_.loadParamFile(FILE_SERVO_PARAM)
                    || !yaml_help_.getParam(MANU_PARAM_DATA_SERVO, servo_data))
                {
                    return PARAM_MANAGER_SET_PARAM_FAILED;                    
                }
                else
                {
                    servo_data[93] = data_int;                    
                }
                if(!yaml_help_.setParam(MANU_PARAM_DATA_SERVO, servo_data)
                    || !yaml_help_.dumpParamFile(FILE_SERVO_PARAM))
                {
                    return PARAM_MANAGER_SET_PARAM_FAILED;
                }
                memcpy(manu_param_list_[14].data, &data_int, sizeof(int));
                return SUCCESS;
            }
            if(strcmp(param_info.name, MANU_PARAM_NAME_16) == 0)
            {
                data_int = *((int*)param_info.data);
                if(!yaml_help_.loadParamFile(FILE_SERVO_PARAM)
                    || !yaml_help_.getParam(MANU_PARAM_DATA_SERVO, servo_data))
                {
                    return PARAM_MANAGER_SET_PARAM_FAILED;                    
                }
                else
                {
                    servo_data[96] = data_int;                    
                }
                if(!yaml_help_.setParam(MANU_PARAM_DATA_SERVO, servo_data)
                    || !yaml_help_.dumpParamFile(FILE_SERVO_PARAM))
                {
                    return PARAM_MANAGER_SET_PARAM_FAILED;
                }
                memcpy(manu_param_list_[15].data, &data_int, sizeof(int));
                return SUCCESS;
            }  
            if(strcmp(param_info.name, MANU_PARAM_NAME_17) == 0)
            {
                data_int = *((int*)param_info.data);
                if(!yaml_help_.loadParamFile(FILE_SERVO_PARAM)
                    || !yaml_help_.getParam(MANU_PARAM_DATA_SERVO, servo_data))
                {
                    return PARAM_MANAGER_SET_PARAM_FAILED;                    
                }
                else
                {
                    servo_data[98] = data_int;                    
                }
                if(!yaml_help_.setParam(MANU_PARAM_DATA_SERVO, servo_data)
                    || !yaml_help_.dumpParamFile(FILE_SERVO_PARAM))
                {
                    return PARAM_MANAGER_SET_PARAM_FAILED;
                }
                memcpy(manu_param_list_[16].data, &data_int, sizeof(int));
                return SUCCESS;
            }
            if(strcmp(param_info.name, MANU_PARAM_NAME_18) == 0)
            {
                data_int = *((int*)param_info.data);
                if(!yaml_help_.loadParamFile(FILE_SERVO_PARAM)
                    || !yaml_help_.getParam(MANU_PARAM_DATA_SERVO, servo_data))
                {
                    return PARAM_MANAGER_SET_PARAM_FAILED;                    
                }
                else
                {
                    servo_data[132] = data_int;                    
                }
                if(!yaml_help_.setParam(MANU_PARAM_DATA_SERVO, servo_data)
                    || !yaml_help_.dumpParamFile(FILE_SERVO_PARAM))
                {
                    return PARAM_MANAGER_SET_PARAM_FAILED;
                }
                memcpy(manu_param_list_[17].data, &data_int, sizeof(int));
                return SUCCESS;
            }
            // servo param - axis2
            if(strcmp(param_info.name, MANU_PARAM_NAME_19) == 0)
            {
                data_int = *((int*)param_info.data);
                if(!yaml_help_.loadParamFile(FILE_SERVO_PARAM)
                    || !yaml_help_.getParam(MANU_PARAM_DATA_SERVO, servo_data))
                {
                    return PARAM_MANAGER_SET_PARAM_FAILED;                    
                }
                else
                {
                    servo_data[285] = data_int;                    
                }
                if(!yaml_help_.setParam(MANU_PARAM_DATA_SERVO, servo_data)
                    || !yaml_help_.dumpParamFile(FILE_SERVO_PARAM))
                {
                    return PARAM_MANAGER_SET_PARAM_FAILED;
                }
                memcpy(manu_param_list_[18].data, &data_int, sizeof(int));
                return SUCCESS;
            }
            if(strcmp(param_info.name, MANU_PARAM_NAME_20) == 0)
            {
                data_int = *((int*)param_info.data);
                if(!yaml_help_.loadParamFile(FILE_SERVO_PARAM)
                    || !yaml_help_.getParam(MANU_PARAM_DATA_SERVO, servo_data))
                {
                    return PARAM_MANAGER_SET_PARAM_FAILED;                    
                }
                else
                {
                    servo_data[288] = data_int;                    
                }
                if(!yaml_help_.setParam(MANU_PARAM_DATA_SERVO, servo_data)
                    || !yaml_help_.dumpParamFile(FILE_SERVO_PARAM))
                {
                    return PARAM_MANAGER_SET_PARAM_FAILED;
                }
                memcpy(manu_param_list_[19].data, &data_int, sizeof(int));
                return SUCCESS;
            }  
            if(strcmp(param_info.name, MANU_PARAM_NAME_21) == 0)
            {
                data_int = *((int*)param_info.data);
                if(!yaml_help_.loadParamFile(FILE_SERVO_PARAM)
                    || !yaml_help_.getParam(MANU_PARAM_DATA_SERVO, servo_data))
                {
                    return PARAM_MANAGER_SET_PARAM_FAILED;                    
                }
                else
                {
                    servo_data[290] = data_int;                    
                }
                if(!yaml_help_.setParam(MANU_PARAM_DATA_SERVO, servo_data)
                    || !yaml_help_.dumpParamFile(FILE_SERVO_PARAM))
                {
                    return PARAM_MANAGER_SET_PARAM_FAILED;
                }
                memcpy(manu_param_list_[20].data, &data_int, sizeof(int));
                return SUCCESS;
            }
            if(strcmp(param_info.name, MANU_PARAM_NAME_22) == 0)
            {
                data_int = *((int*)param_info.data);
                if(!yaml_help_.loadParamFile(FILE_SERVO_PARAM)
                    || !yaml_help_.getParam(MANU_PARAM_DATA_SERVO, servo_data))
                {
                    return PARAM_MANAGER_SET_PARAM_FAILED;                    
                }
                else
                {
                    servo_data[324] = data_int;                    
                }
                if(!yaml_help_.setParam(MANU_PARAM_DATA_SERVO, servo_data)
                    || !yaml_help_.dumpParamFile(FILE_SERVO_PARAM))
                {
                    return PARAM_MANAGER_SET_PARAM_FAILED;
                }
                memcpy(manu_param_list_[21].data, &data_int, sizeof(int));
                return SUCCESS;
            }
            // servo param - axis3
            if(strcmp(param_info.name, MANU_PARAM_NAME_23) == 0)
            {
                data_int = *((int*)param_info.data);
                if(!yaml_help_.loadParamFile(FILE_SERVO_PARAM)
                    || !yaml_help_.getParam(MANU_PARAM_DATA_SERVO, servo_data))
                {
                    return PARAM_MANAGER_SET_PARAM_FAILED;                    
                }
                else
                {
                    servo_data[477] = data_int;                    
                }
                if(!yaml_help_.setParam(MANU_PARAM_DATA_SERVO, servo_data)
                    || !yaml_help_.dumpParamFile(FILE_SERVO_PARAM))
                {
                    return PARAM_MANAGER_SET_PARAM_FAILED;
                }
                memcpy(manu_param_list_[22].data, &data_int, sizeof(int));
                return SUCCESS;
            }
            if(strcmp(param_info.name, MANU_PARAM_NAME_24) == 0)
            {
                data_int = *((int*)param_info.data);
                if(!yaml_help_.loadParamFile(FILE_SERVO_PARAM)
                    || !yaml_help_.getParam(MANU_PARAM_DATA_SERVO, servo_data))
                {
                    return PARAM_MANAGER_SET_PARAM_FAILED;                    
                }
                else
                {
                    servo_data[480] = data_int;                    
                }
                if(!yaml_help_.setParam(MANU_PARAM_DATA_SERVO, servo_data)
                    || !yaml_help_.dumpParamFile(FILE_SERVO_PARAM))
                {
                    return PARAM_MANAGER_SET_PARAM_FAILED;
                }
                memcpy(manu_param_list_[23].data, &data_int, sizeof(int));
                return SUCCESS;
            }  
            if(strcmp(param_info.name, MANU_PARAM_NAME_25) == 0)
            {
                data_int = *((int*)param_info.data);
                if(!yaml_help_.loadParamFile(FILE_SERVO_PARAM)
                    || !yaml_help_.getParam(MANU_PARAM_DATA_SERVO, servo_data))
                {
                    return PARAM_MANAGER_SET_PARAM_FAILED;                    
                }
                else
                {
                    servo_data[482] = data_int;                    
                }
                if(!yaml_help_.setParam(MANU_PARAM_DATA_SERVO, servo_data)
                    || !yaml_help_.dumpParamFile(FILE_SERVO_PARAM))
                {
                    return PARAM_MANAGER_SET_PARAM_FAILED;
                }
                memcpy(manu_param_list_[24].data, &data_int, sizeof(int));
                return SUCCESS;
            }
            if(strcmp(param_info.name, MANU_PARAM_NAME_26) == 0)
            {
                data_int = *((int*)param_info.data);
                if(!yaml_help_.loadParamFile(FILE_SERVO_PARAM)
                    || !yaml_help_.getParam(MANU_PARAM_DATA_SERVO, servo_data))
                {
                    return PARAM_MANAGER_SET_PARAM_FAILED;                    
                }
                else
                {
                    servo_data[516] = data_int;                    
                }
                if(!yaml_help_.setParam(MANU_PARAM_DATA_SERVO, servo_data)
                    || !yaml_help_.dumpParamFile(FILE_SERVO_PARAM))
                {
                    return PARAM_MANAGER_SET_PARAM_FAILED;
                }
                memcpy(manu_param_list_[25].data, &data_int, sizeof(int));
                return SUCCESS;
            }
            // servo param - axis4
            if(strcmp(param_info.name, MANU_PARAM_NAME_27) == 0)
            {
                data_int = *((int*)param_info.data);
                if(!yaml_help_.loadParamFile(FILE_SERVO_PARAM)
                    || !yaml_help_.getParam(MANU_PARAM_DATA_SERVO, servo_data))
                {
                    return PARAM_MANAGER_SET_PARAM_FAILED;                    
                }
                else
                {
                    servo_data[669] = data_int;                    
                }
                if(!yaml_help_.setParam(MANU_PARAM_DATA_SERVO, servo_data)
                    || !yaml_help_.dumpParamFile(FILE_SERVO_PARAM))
                {
                    return PARAM_MANAGER_SET_PARAM_FAILED;
                }
                memcpy(manu_param_list_[26].data, &data_int, sizeof(int));
                return SUCCESS;
            }
            if(strcmp(param_info.name, MANU_PARAM_NAME_28) == 0)
            {
                data_int = *((int*)param_info.data);
                if(!yaml_help_.loadParamFile(FILE_SERVO_PARAM)
                    || !yaml_help_.getParam(MANU_PARAM_DATA_SERVO, servo_data))
                {
                    return PARAM_MANAGER_SET_PARAM_FAILED;                    
                }
                else
                {
                    servo_data[672] = data_int;                    
                }
                if(!yaml_help_.setParam(MANU_PARAM_DATA_SERVO, servo_data)
                    || !yaml_help_.dumpParamFile(FILE_SERVO_PARAM))
                {
                    return PARAM_MANAGER_SET_PARAM_FAILED;
                }
                memcpy(manu_param_list_[27].data, &data_int, sizeof(int));
                return SUCCESS;
            }  
            if(strcmp(param_info.name, MANU_PARAM_NAME_29) == 0)
            {
                data_int = *((int*)param_info.data);
                if(!yaml_help_.loadParamFile(FILE_SERVO_PARAM)
                    || !yaml_help_.getParam(MANU_PARAM_DATA_SERVO, servo_data))
                {
                    return PARAM_MANAGER_SET_PARAM_FAILED;                    
                }
                else
                {
                    servo_data[674] = data_int;                    
                }
                if(!yaml_help_.setParam(MANU_PARAM_DATA_SERVO, servo_data)
                    || !yaml_help_.dumpParamFile(FILE_SERVO_PARAM))
                {
                    return PARAM_MANAGER_SET_PARAM_FAILED;
                }
                memcpy(manu_param_list_[28].data, &data_int, sizeof(int));
                return SUCCESS;
            }
            if(strcmp(param_info.name, MANU_PARAM_NAME_30) == 0)
            {
                data_int = *((int*)param_info.data);
                if(!yaml_help_.loadParamFile(FILE_SERVO_PARAM)
                    || !yaml_help_.getParam(MANU_PARAM_DATA_SERVO, servo_data))
                {
                    return PARAM_MANAGER_SET_PARAM_FAILED;                    
                }
                else
                {
                    servo_data[708] = data_int;                    
                }
                if(!yaml_help_.setParam(MANU_PARAM_DATA_SERVO, servo_data)
                    || !yaml_help_.dumpParamFile(FILE_SERVO_PARAM))
                {
                    return PARAM_MANAGER_SET_PARAM_FAILED;
                }
                memcpy(manu_param_list_[29].data, &data_int, sizeof(int));
                return SUCCESS;
            }
            // servo param - axis5
            if(strcmp(param_info.name, MANU_PARAM_NAME_31) == 0)
            {
                data_int = *((int*)param_info.data);
                if(!yaml_help_.loadParamFile(FILE_SERVO_PARAM)
                    || !yaml_help_.getParam(MANU_PARAM_DATA_SERVO, servo_data))
                {
                    return PARAM_MANAGER_SET_PARAM_FAILED;                    
                }
                else
                {
                    servo_data[861] = data_int;                    
                }
                if(!yaml_help_.setParam(MANU_PARAM_DATA_SERVO, servo_data)
                    || !yaml_help_.dumpParamFile(FILE_SERVO_PARAM))
                {
                    return PARAM_MANAGER_SET_PARAM_FAILED;
                }
                memcpy(manu_param_list_[30].data, &data_int, sizeof(int));
                return SUCCESS;
            }
            if(strcmp(param_info.name, MANU_PARAM_NAME_32) == 0)
            {
                data_int = *((int*)param_info.data);
                if(!yaml_help_.loadParamFile(FILE_SERVO_PARAM)
                    || !yaml_help_.getParam(MANU_PARAM_DATA_SERVO, servo_data))
                {
                    return PARAM_MANAGER_SET_PARAM_FAILED;                    
                }
                else
                {
                    servo_data[864] = data_int;                    
                }
                if(!yaml_help_.setParam(MANU_PARAM_DATA_SERVO, servo_data)
                    || !yaml_help_.dumpParamFile(FILE_SERVO_PARAM))
                {
                    return PARAM_MANAGER_SET_PARAM_FAILED;
                }
                memcpy(manu_param_list_[31].data, &data_int, sizeof(int));
                return SUCCESS;
            }  
            if(strcmp(param_info.name, MANU_PARAM_NAME_33) == 0)
            {
                data_int = *((int*)param_info.data);
                if(!yaml_help_.loadParamFile(FILE_SERVO_PARAM)
                    || !yaml_help_.getParam(MANU_PARAM_DATA_SERVO, servo_data))
                {
                    return PARAM_MANAGER_SET_PARAM_FAILED;                    
                }
                else
                {
                    servo_data[866] = data_int;                    
                }
                if(!yaml_help_.setParam(MANU_PARAM_DATA_SERVO, servo_data)
                    || !yaml_help_.dumpParamFile(FILE_SERVO_PARAM))
                {
                    return PARAM_MANAGER_SET_PARAM_FAILED;
                }
                memcpy(manu_param_list_[32].data, &data_int, sizeof(int));
                return SUCCESS;
            }
            if(strcmp(param_info.name, MANU_PARAM_NAME_34) == 0)
            {
                data_int = *((int*)param_info.data);
                if(!yaml_help_.loadParamFile(FILE_SERVO_PARAM)
                    || !yaml_help_.getParam(MANU_PARAM_DATA_SERVO, servo_data))
                {
                    return PARAM_MANAGER_SET_PARAM_FAILED;                    
                }
                else
                {
                    servo_data[900] = data_int;                    
                }
                if(!yaml_help_.setParam(MANU_PARAM_DATA_SERVO, servo_data)
                    || !yaml_help_.dumpParamFile(FILE_SERVO_PARAM))
                {
                    return PARAM_MANAGER_SET_PARAM_FAILED;
                }
                memcpy(manu_param_list_[33].data, &data_int, sizeof(int));
                return SUCCESS;
            }
            // servo param - axis6
            if(strcmp(param_info.name, MANU_PARAM_NAME_35) == 0)
            {
                data_int = *((int*)param_info.data);
                if(!yaml_help_.loadParamFile(FILE_SERVO_PARAM)
                    || !yaml_help_.getParam(MANU_PARAM_DATA_SERVO, servo_data))
                {
                    return PARAM_MANAGER_SET_PARAM_FAILED;                    
                }
                else
                {
                    servo_data[1053] = data_int;                    
                }
                if(!yaml_help_.setParam(MANU_PARAM_DATA_SERVO, servo_data)
                    || !yaml_help_.dumpParamFile(FILE_SERVO_PARAM))
                {
                    return PARAM_MANAGER_SET_PARAM_FAILED;
                }
                memcpy(manu_param_list_[34].data, &data_int, sizeof(int));
                return SUCCESS;
            }
            if(strcmp(param_info.name, MANU_PARAM_NAME_36) == 0)
            {
                data_int = *((int*)param_info.data);
                if(!yaml_help_.loadParamFile(FILE_SERVO_PARAM)
                    || !yaml_help_.getParam(MANU_PARAM_DATA_SERVO, servo_data))
                {
                    return PARAM_MANAGER_SET_PARAM_FAILED;                    
                }
                else
                {
                    servo_data[1056] = data_int;                    
                }
                if(!yaml_help_.setParam(MANU_PARAM_DATA_SERVO, servo_data)
                    || !yaml_help_.dumpParamFile(FILE_SERVO_PARAM))
                {
                    return PARAM_MANAGER_SET_PARAM_FAILED;
                }
                memcpy(manu_param_list_[35].data, &data_int, sizeof(int));
                return SUCCESS;
            }  
            if(strcmp(param_info.name, MANU_PARAM_NAME_37) == 0)
            {
                data_int = *((int*)param_info.data);
                if(!yaml_help_.loadParamFile(FILE_SERVO_PARAM)
                    || !yaml_help_.getParam(MANU_PARAM_DATA_SERVO, servo_data))
                {
                    return PARAM_MANAGER_SET_PARAM_FAILED;                    
                }
                else
                {
                    servo_data[1058] = data_int;                    
                }
                if(!yaml_help_.setParam(MANU_PARAM_DATA_SERVO, servo_data)
                    || !yaml_help_.dumpParamFile(FILE_SERVO_PARAM))
                {
                    return PARAM_MANAGER_SET_PARAM_FAILED;
                }
                memcpy(manu_param_list_[36].data, &data_int, sizeof(int));
                return SUCCESS;
            }
            if(strcmp(param_info.name, MANU_PARAM_NAME_38) == 0)
            {
                data_int = *((int*)param_info.data);
                if(!yaml_help_.loadParamFile(FILE_SERVO_PARAM)
                    || !yaml_help_.getParam(MANU_PARAM_DATA_SERVO, servo_data))
                {
                    return PARAM_MANAGER_SET_PARAM_FAILED;                    
                }
                else
                {
                    servo_data[1092] = data_int;                    
                }
                if(!yaml_help_.setParam(MANU_PARAM_DATA_SERVO, servo_data)
                    || !yaml_help_.dumpParamFile(FILE_SERVO_PARAM))
                {
                    return PARAM_MANAGER_SET_PARAM_FAILED;
                }
                memcpy(manu_param_list_[37].data, &data_int, sizeof(int));
                return SUCCESS;
            }
            
            return PARAM_MANAGER_SET_PARAM_FAILED;
        }   
        case PARAM_GROUP_USER:
        {
            if(strcmp(param_info.name, USER_PARAM_NAME_1) == 0)
            {
                data_double = *((double*)param_info.data);
                if(!yaml_help_.loadParamFile(FILE_BASE_GROUP)
                    || !yaml_help_.setParam(USER_PARAM_DATA_1, data_double)
                    || !yaml_help_.dumpParamFile(FILE_BASE_GROUP)) 
                {
                    return PARAM_MANAGER_SET_PARAM_FAILED;
                }
                memcpy(user_param_list_[0].data, &data_double, sizeof(double));
                return SUCCESS;
            }
            if(strcmp(param_info.name, USER_PARAM_NAME_2) == 0)
            {
                data_int = *((int*)param_info.data);
                if(!yaml_help_.loadParamFile(FILE_BASE_GROUP)
                    || !yaml_help_.setParam(USER_PARAM_DATA_2, data_int)
                    || !yaml_help_.dumpParamFile(FILE_BASE_GROUP)) 
                {
                    return PARAM_MANAGER_SET_PARAM_FAILED;
                }
                memcpy(user_param_list_[1].data, &data_int, sizeof(int));
                return SUCCESS;
            }
            if(strcmp(param_info.name, USER_PARAM_NAME_3) == 0)
            {
                data_int = *((int*)param_info.data);
                if(!yaml_help_.loadParamFile(FILE_REG_MANAGER)
                    || !yaml_help_.setParam(USER_PARAM_DATA_3, data_int)
                    || !yaml_help_.dumpParamFile(FILE_REG_MANAGER)) 
                {
                    return PARAM_MANAGER_SET_PARAM_FAILED;
                }
                memcpy(user_param_list_[2].data, &data_int, sizeof(int));
                return SUCCESS;
            }
            if(strcmp(param_info.name, USER_PARAM_NAME_4) == 0)
            {
                data_int = *((int*)param_info.data);
                if(!yaml_help_.loadParamFile(FILE_REG_MANAGER)
                    || !yaml_help_.setParam(USER_PARAM_DATA_4, data_int)
                    || !yaml_help_.dumpParamFile(FILE_REG_MANAGER)) 
                {
                    return PARAM_MANAGER_SET_PARAM_FAILED;
                }
                memcpy(user_param_list_[3].data, &data_int, sizeof(int));
                return SUCCESS;
            }
            if(strcmp(param_info.name, USER_PARAM_NAME_5) == 0)
            {
                data_int = *((int*)param_info.data);
                if(!yaml_help_.loadParamFile(FILE_REG_MANAGER)
                    || !yaml_help_.setParam(USER_PARAM_DATA_5, data_int)
                    || !yaml_help_.dumpParamFile(FILE_REG_MANAGER)) 
                {
                    return PARAM_MANAGER_SET_PARAM_FAILED;
                }
                memcpy(user_param_list_[4].data, &data_int, sizeof(int));
                return SUCCESS;
            }
            if(strcmp(param_info.name, USER_PARAM_NAME_6) == 0)
            {
                data_int = *((int*)param_info.data);
                if(!yaml_help_.loadParamFile(FILE_REG_MANAGER)
                    || !yaml_help_.setParam(USER_PARAM_DATA_6, data_int)
                    || !yaml_help_.dumpParamFile(FILE_REG_MANAGER)) 
                {
                    return PARAM_MANAGER_SET_PARAM_FAILED;
                }
                memcpy(user_param_list_[5].data, &data_int, sizeof(int));
                return SUCCESS;
            }
            if(strcmp(param_info.name, USER_PARAM_NAME_7) == 0)
            {
                data_int = *((int*)param_info.data);
                if(!yaml_help_.loadParamFile(FILE_COORDINATE_MANAGER)
                    || !yaml_help_.setParam(USER_PARAM_DATA_7, data_int)
                    || !yaml_help_.dumpParamFile(FILE_COORDINATE_MANAGER)) 
                {
                    return PARAM_MANAGER_SET_PARAM_FAILED;
                }
                memcpy(user_param_list_[6].data, &data_int, sizeof(int));
                return SUCCESS;
            }
            if(strcmp(param_info.name, USER_PARAM_NAME_8) == 0)
            {
                data_int = *((int*)param_info.data);
                if(!yaml_help_.loadParamFile(FILE_TOOL_MANAGER)
                    || !yaml_help_.setParam(USER_PARAM_DATA_8, data_int)
                    || !yaml_help_.dumpParamFile(FILE_TOOL_MANAGER)) 
                {
                    return PARAM_MANAGER_SET_PARAM_FAILED;
                }
                memcpy(user_param_list_[7].data, &data_int, sizeof(int));
                return SUCCESS;
            }
            if(strcmp(param_info.name, USER_PARAM_NAME_9) == 0)
            {
                data_int = *((int*)param_info.data);
                if(!yaml_help_.loadParamFile(FILE_IO_MAPPING)
                    || !yaml_help_.setParam(USER_PARAM_DATA_9, data_int)
                    || !yaml_help_.dumpParamFile(FILE_IO_MAPPING)) 
                {
                    return PARAM_MANAGER_SET_PARAM_FAILED;
                }
                memcpy(user_param_list_[8].data, &data_int, sizeof(int));
                return SUCCESS;
            }

            return PARAM_MANAGER_SET_PARAM_FAILED;
        }
        default:
            return PARAM_MANAGER_SET_PARAM_FAILED;
    }
}

void ParamManager::printParamInfoList(ParamGroup_e param_group)
{
    int i;
    switch(param_group)
    {
        case PARAM_GROUP_MANU:
            for(i = 0; i < manu_param_list_.size(); ++i)
            {
                std::cout<<manu_param_list_[i].name;
                switch(manu_param_list_[i].type)
                {
                    case PARAM_INFO_INT:
                        std::cout<<" type = INT value = "<<*((int*)manu_param_list_[i].data); break;
                    case PARAM_INFO_DOUBLE:
                        std::cout<<" type = DOUBLE value = "<<*((double*)manu_param_list_[i].data); break;
                    case PARAM_INFO_BOOL:
                        std::cout<<" type = BOOL value = "<<*((bool*)manu_param_list_[i].data); break;
                    default:
                        std::cout<<" unknown type";
                }
                std::cout<<std::endl;
            }
            break;
        case PARAM_GROUP_USER:
             for(i = 0; i < user_param_list_.size(); ++i)
            {
                std::cout<<user_param_list_[i].name;
                switch(user_param_list_[i].type)
                {
                    case PARAM_INFO_INT:
                        std::cout<<" type = INT value = "<<*((int*)user_param_list_[i].data); break;
                    case PARAM_INFO_DOUBLE:
                        std::cout<<" type = DOUBLE value = "<<*((double*)user_param_list_[i].data); break;
                    case PARAM_INFO_BOOL:
                        std::cout<<" type = BOOL value = "<<*((bool*)user_param_list_[i].data); break;
                    default:
                        std::cout<<" unknown type";
                }
                std::cout<<std::endl;
            }
            break;           
    }
}

