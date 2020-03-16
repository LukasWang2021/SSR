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
#define USER_PARAM_NAME_10 "enable set IO in auto mode"
#define USER_PARAM_NAME_11 "enable set velocity in auto mode"
#define USER_PARAM_NAME_12 "auto mode DO port number"
#define USER_PARAM_NAME_13 "limited manual mode DO port number"
#define USER_PARAM_NAME_14 "manual mode DO port number"
#define USER_PARAM_NAME_15 "condition wait timeout"

#define MANU_PARAM_NAME_1 "trajectory fifo size"
#define MANU_PARAM_NAME_2 "trajectory fifo limit size"
#define MANU_PARAM_NAME_3 "axis1 - switch of friction compensation"
#define MANU_PARAM_NAME_4 "axis1 - switch of acceleration feedforward"
#define MANU_PARAM_NAME_5 "axis1 - switch of gravity compensation"
#define MANU_PARAM_NAME_6 "axis1 - switch of velocity compensation"
#define MANU_PARAM_NAME_7 "axis2 - switch of friction compensation"
#define MANU_PARAM_NAME_8 "axis2 - switch of acceleration feedforward"
#define MANU_PARAM_NAME_9 "axis2 - switch of gravity compensation"
#define MANU_PARAM_NAME_10 "axis2 - switch of velocity compensation"
#define MANU_PARAM_NAME_11 "axis3 - switch of friction compensation"
#define MANU_PARAM_NAME_12 "axis3 - switch of acceleration feedforward"
#define MANU_PARAM_NAME_13 "axis3 - switch of gravity compensation"
#define MANU_PARAM_NAME_14 "axis3 - switch of velocity compensation"
#define MANU_PARAM_NAME_15 "axis4 - switch of friction compensation"
#define MANU_PARAM_NAME_16 "axis4 - switch of acceleration feedforward"
#define MANU_PARAM_NAME_17 "axis4 - switch of gravity compensation"
#define MANU_PARAM_NAME_18 "axis4 - switch of velocity compensation"
#define MANU_PARAM_NAME_19 "axis5 - switch of friction compensation"
#define MANU_PARAM_NAME_20 "axis5 - switch of acceleration feedforward"
#define MANU_PARAM_NAME_21 "axis5 - switch of gravity compensation"
#define MANU_PARAM_NAME_22 "axis5 - switch of velocity compensation"
#define MANU_PARAM_NAME_23 "axis6 - switch of friction compensation"
#define MANU_PARAM_NAME_24 "axis6 - switch of acceleration feedforward"
#define MANU_PARAM_NAME_25 "axis6 - switch of gravity compensation"
#define MANU_PARAM_NAME_26 "axis6 - switch of velocity compensation"
#define MANU_PARAM_NAME_27 "max jerk num"
#define MANU_PARAM_NAME_28 "inverse dynamics check"
#define MANU_PARAM_NAME_29 "adjust acc by vel"
#define MANU_PARAM_NAME_30 "axis1 - torque max"
#define MANU_PARAM_NAME_31 "axis2 - torque max"
#define MANU_PARAM_NAME_32 "axis3 - torque max"
#define MANU_PARAM_NAME_33 "axis4 - torque max"
#define MANU_PARAM_NAME_34 "axis5 - torque max"
#define MANU_PARAM_NAME_35 "axis6 - torque max"
#define MANU_PARAM_NAME_36 "axis1 - omega max"
#define MANU_PARAM_NAME_37 "axis2 - omega max"
#define MANU_PARAM_NAME_38 "axis3 - omega max"
#define MANU_PARAM_NAME_39 "axis4 - omega max"
#define MANU_PARAM_NAME_40 "axis5 - omega max"
#define MANU_PARAM_NAME_41 "axis6 - omega max"
#define MANU_PARAM_NAME_42 "axis1 - alpha max"
#define MANU_PARAM_NAME_43 "axis2 - alpha max"
#define MANU_PARAM_NAME_44 "axis3 - alpha max"
#define MANU_PARAM_NAME_45 "axis4 - alpha max"
#define MANU_PARAM_NAME_46 "axis5 - alpha max"
#define MANU_PARAM_NAME_47 "axis6 - alpha max"
#define MANU_PARAM_NAME_48 "axis1 - beta max acc"
#define MANU_PARAM_NAME_49 "axis2 - beta max acc"
#define MANU_PARAM_NAME_50 "axis3 - beta max acc"
#define MANU_PARAM_NAME_51 "axis4 - beta max acc"
#define MANU_PARAM_NAME_52 "axis5 - beta max acc"
#define MANU_PARAM_NAME_53 "axis6 - beta max acc"
#define MANU_PARAM_NAME_54 "axis1 - beta max running"
#define MANU_PARAM_NAME_55 "axis2 - beta max running"
#define MANU_PARAM_NAME_56 "axis3 - beta max running"
#define MANU_PARAM_NAME_57 "axis4 - beta max running"
#define MANU_PARAM_NAME_58 "axis5 - beta max running"
#define MANU_PARAM_NAME_59 "axis6 - beta max running"
#define MANU_PARAM_NAME_60 "axis1 - beta max dec"
#define MANU_PARAM_NAME_61 "axis2 - beta max dec"
#define MANU_PARAM_NAME_62 "axis3 - beta max dec"
#define MANU_PARAM_NAME_63 "axis4 - beta max dec"
#define MANU_PARAM_NAME_64 "axis5 - beta max dec"
#define MANU_PARAM_NAME_65 "axis6 - beta max dec"
#define MANU_PARAM_NAME_66 "cart position vel max"
#define MANU_PARAM_NAME_67 "cart position acc max"
#define MANU_PARAM_NAME_68 "cart position jerk max acc"
#define MANU_PARAM_NAME_69 "cart position jerk max dec"
#define MANU_PARAM_NAME_70 "cart position jerk max running"
#define MANU_PARAM_NAME_71 "cart quaternion vel max"
#define MANU_PARAM_NAME_72 "cart quaternion acc max"
#define MANU_PARAM_NAME_73 "cart quaternion jerk max acc"
#define MANU_PARAM_NAME_74 "cart quaternion jerk max dec"
#define MANU_PARAM_NAME_75 "cart quaternion jerk max running"

#define USER_PARAM_DATA_1 "stable_with_fine/threshold"
#define USER_PARAM_DATA_2 "stable_with_fine/cycle"
#define USER_PARAM_DATA_3 "pr_reg_number"
#define USER_PARAM_DATA_4 "mr_reg_number"
#define USER_PARAM_DATA_5 "sr_reg_number"
#define USER_PARAM_DATA_6 "r_reg_number"
#define USER_PARAM_DATA_7 "max_number_of_coords"
#define USER_PARAM_DATA_8 "max_number_of_tools"
#define USER_PARAM_DATA_9 "max_mapping_number"
#define USER_PARAM_DATA_10 "enable_set_io_in_auto"
#define USER_PARAM_DATA_11 "enable_set_vel_in_auto"
#define USER_PARAM_DATA_12 "auto_mode_DO"
#define USER_PARAM_DATA_13 "limited_manual_mode_DO"
#define USER_PARAM_DATA_14 "manual_mode_DO"
#define USER_PARAM_DATA_15 "wait_time/time_out"

#define MANU_PARAM_DATA_1 "trajectory_fifo/fifo_size"
#define MANU_PARAM_DATA_2 "trajectory_fifo/lower_limit"
#define MANU_PARAM_DATA_SERVO "servo/stored_param"
#define MANU_PARAM_DATA_27 "max_jerk_num"
#define MANU_PARAM_DATA_28 "inverse_dynamics_check"
#define MANU_PARAM_DATA_29 "adjust_acc_by_vel"
#define MANU_PARAM_DATA_30 "joint/torque_max"
#define MANU_PARAM_DATA_36 "joint/omega_max"
#define MANU_PARAM_DATA_42 "joint/alpha_max"
#define MANU_PARAM_DATA_48 "joint/beta_max_acc"
#define MANU_PARAM_DATA_54 "joint/beta_max_running"
#define MANU_PARAM_DATA_60 "joint/beta_max_dec"
#define MANU_PARAM_DATA_66 "cart/position/vel_max"
#define MANU_PARAM_DATA_67 "cart/position/acc_max"
#define MANU_PARAM_DATA_68 "cart/position/jerk_max_acc"
#define MANU_PARAM_DATA_69 "cart/position/jerk_max_dec"
#define MANU_PARAM_DATA_70 "cart/position/jerk_max_running"
#define MANU_PARAM_DATA_71 "cart/quaternion/vel_max"
#define MANU_PARAM_DATA_72 "cart/quaternion/acc_max"
#define MANU_PARAM_DATA_73 "cart/quaternion/jerk_max_acc"
#define MANU_PARAM_DATA_74 "cart/quaternion/jerk_max_dec"
#define MANU_PARAM_DATA_75 "cart/quaternion/jerk_max_running"


#define FILE_BASE_GROUP (std::string(AXIS_GROUP_DIR) + "base_group.yaml")
#define FILE_REG_MANAGER (std::string(COMPONENT_PARAM_FILE_DIR) + "reg_manager.yaml")
#define FILE_COORDINATE_MANAGER (std::string(COMPONENT_PARAM_FILE_DIR) + "coordinate_manager.yaml")
#define FILE_TOOL_MANAGER (std::string(COMPONENT_PARAM_FILE_DIR) + "tool_manager.yaml")
#define FILE_IO_MAPPING (std::string(COMPONENT_PARAM_FILE_DIR) + "io_mapping.yaml")
#define FILE_ALGORITHM_CONFIG (std::string(ALGORITHM_DIR) + "config.yaml")
#define FILE_ALGORITHM_CONSTRAINT (std::string(ALGORITHM_DIR) + "constraint.yaml")
#define FILE_SERVO_PARAM (std::string(SERVO_DIR) + "servo_param.yaml")
#define FILE_CONTROLLER (std::string(COMPONENT_PARAM_FILE_DIR) + "controller.yaml")
#define FILE_FST_SAFETY_DEVICE (std::string(COMPONENT_PARAM_FILE_DIR) + "fst_safety_device.yaml")
#define FILE_PRG_INTERPRETER (std::string(COMPONENT_PARAM_FILE_DIR) + "prg_interpreter_config.yaml")

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

    if(!yaml_help_.getParam(MANU_PARAM_DATA_2, data_int)) return PARAM_MANAGER_INIT_FAILED;
    strcpy(param_info.name, MANU_PARAM_NAME_2);
    param_info.type = PARAM_INFO_INT;
    memcpy(param_info.data, &data_int, sizeof(int));
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

    if(!yaml_help_.getParam(USER_PARAM_DATA_10, data_bool)) return PARAM_MANAGER_INIT_FAILED;
    strcpy(param_info.name, USER_PARAM_NAME_10);
    param_info.type = PARAM_INFO_BOOL;
    memcpy(param_info.data, &data_int, sizeof(bool));
    user_param_list_.push_back(param_info);  

    if(!yaml_help_.loadParamFile(FILE_CONTROLLER)) return PARAM_MANAGER_INIT_FAILED;

    if(!yaml_help_.getParam(USER_PARAM_DATA_11, data_bool)) return PARAM_MANAGER_INIT_FAILED;
    strcpy(param_info.name, USER_PARAM_NAME_11);
    param_info.type = PARAM_INFO_BOOL;
    memcpy(param_info.data, &data_int, sizeof(bool));
    user_param_list_.push_back(param_info); 

    if(!yaml_help_.loadParamFile(FILE_FST_SAFETY_DEVICE)) return PARAM_MANAGER_INIT_FAILED;
    
    if(!yaml_help_.getParam(USER_PARAM_DATA_12, data_int)) return PARAM_MANAGER_INIT_FAILED;
    strcpy(param_info.name, USER_PARAM_NAME_12);
    param_info.type = PARAM_INFO_INT;
    memcpy(param_info.data, &data_int, sizeof(int));
    user_param_list_.push_back(param_info); 

    if(!yaml_help_.getParam(USER_PARAM_DATA_13, data_int)) return PARAM_MANAGER_INIT_FAILED;
    strcpy(param_info.name, USER_PARAM_NAME_13);
    param_info.type = PARAM_INFO_INT;
    memcpy(param_info.data, &data_int, sizeof(int));
    user_param_list_.push_back(param_info);

    if(!yaml_help_.getParam(USER_PARAM_DATA_14, data_int)) return PARAM_MANAGER_INIT_FAILED;
    strcpy(param_info.name, USER_PARAM_NAME_14);
    param_info.type = PARAM_INFO_INT;
    memcpy(param_info.data, &data_int, sizeof(int));
    user_param_list_.push_back(param_info);  

    if(!yaml_help_.loadParamFile(FILE_PRG_INTERPRETER)) return PARAM_MANAGER_INIT_FAILED;
    
    if(!yaml_help_.getParam(USER_PARAM_DATA_15, data_int)) return PARAM_MANAGER_INIT_FAILED;
    strcpy(param_info.name, USER_PARAM_NAME_15);
    param_info.type = PARAM_INFO_INT;
    memcpy(param_info.data, &data_int, sizeof(int));
    user_param_list_.push_back(param_info); 

    // manu param
    if(!yaml_help_.loadParamFile(FILE_SERVO_PARAM)) return PARAM_MANAGER_INIT_FAILED;
    fst_parameter::ParamValue servo_data;
    if(!yaml_help_.getParam(MANU_PARAM_DATA_SERVO, servo_data)) return PARAM_MANAGER_INIT_FAILED;

    int data;

    strcpy(param_info.name, MANU_PARAM_NAME_3);
    param_info.type = PARAM_INFO_INT;
    data = (int)servo_data[93];
    memcpy(param_info.data, &data, sizeof(int));
    manu_param_list_.push_back(param_info);

    strcpy(param_info.name, MANU_PARAM_NAME_4);
    param_info.type = PARAM_INFO_INT;
    data = (int)servo_data[96];
    memcpy(param_info.data, &data, sizeof(int));
    manu_param_list_.push_back(param_info);

    strcpy(param_info.name, MANU_PARAM_NAME_5);
    param_info.type = PARAM_INFO_INT;
    data = (int)servo_data[98];
    memcpy(param_info.data, &data, sizeof(int));
    manu_param_list_.push_back(param_info); 

    strcpy(param_info.name, MANU_PARAM_NAME_6);
    param_info.type = PARAM_INFO_INT;
    data = (int)servo_data[132];
    memcpy(param_info.data, &data, sizeof(int));
    manu_param_list_.push_back(param_info);
    
    strcpy(param_info.name, MANU_PARAM_NAME_7);
    param_info.type = PARAM_INFO_INT;
    data = (int)servo_data[285];
    memcpy(param_info.data, &data, sizeof(int));
    manu_param_list_.push_back(param_info);

    strcpy(param_info.name, MANU_PARAM_NAME_8);
    param_info.type = PARAM_INFO_INT;
    data = (int)servo_data[288];
    memcpy(param_info.data, &data, sizeof(int));
    manu_param_list_.push_back(param_info);

    strcpy(param_info.name, MANU_PARAM_NAME_9);
    param_info.type = PARAM_INFO_INT;
    data = (int)servo_data[290];
    memcpy(param_info.data, &data, sizeof(int));
    manu_param_list_.push_back(param_info); 

    strcpy(param_info.name, MANU_PARAM_NAME_10);
    param_info.type = PARAM_INFO_INT;
    data = (int)servo_data[324];
    memcpy(param_info.data, &data, sizeof(int));
    manu_param_list_.push_back(param_info);

    strcpy(param_info.name, MANU_PARAM_NAME_11);
    param_info.type = PARAM_INFO_INT;
    data = (int)servo_data[477];
    memcpy(param_info.data, &data, sizeof(int));
    manu_param_list_.push_back(param_info);

    strcpy(param_info.name, MANU_PARAM_NAME_12);
    param_info.type = PARAM_INFO_INT;
    data = (int)servo_data[480];
    memcpy(param_info.data, &data, sizeof(int));
    manu_param_list_.push_back(param_info);

    strcpy(param_info.name, MANU_PARAM_NAME_13);
    param_info.type = PARAM_INFO_INT;
    data = (int)servo_data[482];
    memcpy(param_info.data, &data, sizeof(int));
    manu_param_list_.push_back(param_info); 

    strcpy(param_info.name, MANU_PARAM_NAME_14);
    param_info.type = PARAM_INFO_INT;
    data = (int)servo_data[516];
    memcpy(param_info.data, &data, sizeof(int));
    manu_param_list_.push_back(param_info);

    strcpy(param_info.name, MANU_PARAM_NAME_15);
    param_info.type = PARAM_INFO_INT;
    data = (int)servo_data[669];
    memcpy(param_info.data, &data, sizeof(int));
    manu_param_list_.push_back(param_info);

    strcpy(param_info.name, MANU_PARAM_NAME_16);
    param_info.type = PARAM_INFO_INT;
    data = (int)servo_data[672];
    memcpy(param_info.data, &data, sizeof(int));
    manu_param_list_.push_back(param_info);

    strcpy(param_info.name, MANU_PARAM_NAME_17);
    param_info.type = PARAM_INFO_INT;
    data = (int)servo_data[674];
    memcpy(param_info.data, &data, sizeof(int));
    manu_param_list_.push_back(param_info); 

    strcpy(param_info.name, MANU_PARAM_NAME_18);
    param_info.type = PARAM_INFO_INT;
    data = (int)servo_data[708];
    memcpy(param_info.data, &data, sizeof(int));
    manu_param_list_.push_back(param_info);   

    strcpy(param_info.name, MANU_PARAM_NAME_19);
    param_info.type = PARAM_INFO_INT;
    data = (int)servo_data[861];
    memcpy(param_info.data, &data, sizeof(int));
    manu_param_list_.push_back(param_info);

    strcpy(param_info.name, MANU_PARAM_NAME_20);
    param_info.type = PARAM_INFO_INT;
    data = (int)servo_data[864];
    memcpy(param_info.data, &data, sizeof(int));
    manu_param_list_.push_back(param_info);

    strcpy(param_info.name, MANU_PARAM_NAME_21);
    param_info.type = PARAM_INFO_INT;
    data = (int)servo_data[866];
    memcpy(param_info.data, &data, sizeof(int));
    manu_param_list_.push_back(param_info); 

    strcpy(param_info.name, MANU_PARAM_NAME_22);
    param_info.type = PARAM_INFO_INT;
    data = (int)servo_data[900];
    memcpy(param_info.data, &data, sizeof(int));
    manu_param_list_.push_back(param_info);      

    strcpy(param_info.name, MANU_PARAM_NAME_23);
    param_info.type = PARAM_INFO_INT;
    data = (int)servo_data[1053];
    memcpy(param_info.data, &data, sizeof(int));
    manu_param_list_.push_back(param_info);

    strcpy(param_info.name, MANU_PARAM_NAME_24);
    param_info.type = PARAM_INFO_INT;
    data = (int)servo_data[1056];
    memcpy(param_info.data, &data, sizeof(int));
    manu_param_list_.push_back(param_info);

    strcpy(param_info.name, MANU_PARAM_NAME_25);
    param_info.type = PARAM_INFO_INT;
    data = (int)servo_data[1058];
    memcpy(param_info.data, &data, sizeof(int));
    manu_param_list_.push_back(param_info); 

    strcpy(param_info.name, MANU_PARAM_NAME_26);
    param_info.type = PARAM_INFO_INT;
    data = (int)servo_data[1092];
    memcpy(param_info.data, &data, sizeof(int));
    manu_param_list_.push_back(param_info);

    if(!yaml_help_.loadParamFile(FILE_ALGORITHM_CONFIG)) return PARAM_MANAGER_INIT_FAILED;
    
    if(!yaml_help_.getParam(MANU_PARAM_DATA_27, data_int)) return PARAM_MANAGER_INIT_FAILED;
    strcpy(param_info.name, MANU_PARAM_NAME_27);
    param_info.type = PARAM_INFO_INT;
    memcpy(param_info.data, &data_int, sizeof(int));
    manu_param_list_.push_back(param_info); 

    if(!yaml_help_.getParam(MANU_PARAM_DATA_28, data_bool)) return PARAM_MANAGER_INIT_FAILED;
    strcpy(param_info.name, MANU_PARAM_NAME_28);
    param_info.type = PARAM_INFO_BOOL;
    memcpy(param_info.data, &data_bool, sizeof(bool));
    manu_param_list_.push_back(param_info); 

    if(!yaml_help_.getParam(MANU_PARAM_DATA_29, data_bool)) return PARAM_MANAGER_INIT_FAILED;
    strcpy(param_info.name, MANU_PARAM_NAME_29);
    param_info.type = PARAM_INFO_BOOL;
    memcpy(param_info.data, &data_bool, sizeof(bool));
    manu_param_list_.push_back(param_info); 

    if(!yaml_help_.loadParamFile(FILE_ALGORITHM_CONSTRAINT)) return PARAM_MANAGER_INIT_FAILED;
    fst_parameter::ParamValue array_data;    
    if(!yaml_help_.getParam(MANU_PARAM_DATA_30, array_data)) return PARAM_MANAGER_INIT_FAILED;

    strcpy(param_info.name, MANU_PARAM_NAME_30);
    param_info.type = PARAM_INFO_DOUBLE;
    data_double = (double)array_data[0];
    memcpy(param_info.data, &data_double, sizeof(double));
    manu_param_list_.push_back(param_info);

    strcpy(param_info.name, MANU_PARAM_NAME_31);
    param_info.type = PARAM_INFO_DOUBLE;
    data_double = (double)array_data[1];
    memcpy(param_info.data, &data_double, sizeof(double));
    manu_param_list_.push_back(param_info);

    strcpy(param_info.name, MANU_PARAM_NAME_32);
    param_info.type = PARAM_INFO_DOUBLE;
    data_double = (double)array_data[2];
    memcpy(param_info.data, &data_double, sizeof(double));
    manu_param_list_.push_back(param_info);

    strcpy(param_info.name, MANU_PARAM_NAME_33);
    param_info.type = PARAM_INFO_DOUBLE;
    data_double = (double)array_data[3];
    memcpy(param_info.data, &data_double, sizeof(double));
    manu_param_list_.push_back(param_info);

    strcpy(param_info.name, MANU_PARAM_NAME_34);
    param_info.type = PARAM_INFO_DOUBLE;
    data_double = (double)array_data[4];
    memcpy(param_info.data, &data_double, sizeof(double));
    manu_param_list_.push_back(param_info);

    strcpy(param_info.name, MANU_PARAM_NAME_35);
    param_info.type = PARAM_INFO_DOUBLE;
    data_double = (double)array_data[5];
    memcpy(param_info.data, &data_double, sizeof(double));
    manu_param_list_.push_back(param_info);

    if(!yaml_help_.getParam(MANU_PARAM_DATA_36, array_data)) return PARAM_MANAGER_INIT_FAILED;

    strcpy(param_info.name, MANU_PARAM_NAME_36);
    param_info.type = PARAM_INFO_DOUBLE;
    data_double = (double)array_data[0];
    memcpy(param_info.data, &data_double, sizeof(double));
    manu_param_list_.push_back(param_info);

    strcpy(param_info.name, MANU_PARAM_NAME_37);
    param_info.type = PARAM_INFO_DOUBLE;
    data_double = (double)array_data[1];
    memcpy(param_info.data, &data_double, sizeof(double));
    manu_param_list_.push_back(param_info);

    strcpy(param_info.name, MANU_PARAM_NAME_38);
    param_info.type = PARAM_INFO_DOUBLE;
    data_double = (double)array_data[2];
    memcpy(param_info.data, &data_double, sizeof(double));
    manu_param_list_.push_back(param_info);

    strcpy(param_info.name, MANU_PARAM_NAME_39);
    param_info.type = PARAM_INFO_DOUBLE;
    data_double = (double)array_data[3];
    memcpy(param_info.data, &data_double, sizeof(double));
    manu_param_list_.push_back(param_info);

    strcpy(param_info.name, MANU_PARAM_NAME_40);
    param_info.type = PARAM_INFO_DOUBLE;
    data_double = (double)array_data[4];
    memcpy(param_info.data, &data_double, sizeof(double));
    manu_param_list_.push_back(param_info);

    strcpy(param_info.name, MANU_PARAM_NAME_41);
    param_info.type = PARAM_INFO_DOUBLE;
    data_double = (double)array_data[5];
    memcpy(param_info.data, &data_double, sizeof(double));
    manu_param_list_.push_back(param_info);

    if(!yaml_help_.getParam(MANU_PARAM_DATA_42, array_data)) return PARAM_MANAGER_INIT_FAILED;

    strcpy(param_info.name, MANU_PARAM_NAME_42);
    param_info.type = PARAM_INFO_DOUBLE;
    data_double = (double)array_data[0];
    memcpy(param_info.data, &data_double, sizeof(double));
    manu_param_list_.push_back(param_info);

    strcpy(param_info.name, MANU_PARAM_NAME_43);
    param_info.type = PARAM_INFO_DOUBLE;
    data_double = (double)array_data[1];
    memcpy(param_info.data, &data_double, sizeof(double));
    manu_param_list_.push_back(param_info);

    strcpy(param_info.name, MANU_PARAM_NAME_44);
    param_info.type = PARAM_INFO_DOUBLE;
    data_double = (double)array_data[2];
    memcpy(param_info.data, &data_double, sizeof(double));
    manu_param_list_.push_back(param_info);

    strcpy(param_info.name, MANU_PARAM_NAME_45);
    param_info.type = PARAM_INFO_DOUBLE;
    data_double = (double)array_data[3];
    memcpy(param_info.data, &data_double, sizeof(double));
    manu_param_list_.push_back(param_info);

    strcpy(param_info.name, MANU_PARAM_NAME_46);
    param_info.type = PARAM_INFO_DOUBLE;
    data_double = (double)array_data[4];
    memcpy(param_info.data, &data_double, sizeof(double));
    manu_param_list_.push_back(param_info);

    strcpy(param_info.name, MANU_PARAM_NAME_47);
    param_info.type = PARAM_INFO_DOUBLE;
    data_double = (double)array_data[5];
    memcpy(param_info.data, &data_double, sizeof(double));
    manu_param_list_.push_back(param_info);

    if(!yaml_help_.getParam(MANU_PARAM_DATA_48, array_data)) return PARAM_MANAGER_INIT_FAILED;

    strcpy(param_info.name, MANU_PARAM_NAME_48);
    param_info.type = PARAM_INFO_DOUBLE;
    data_double = (double)array_data[0];
    memcpy(param_info.data, &data_double, sizeof(double));
    manu_param_list_.push_back(param_info);

    strcpy(param_info.name, MANU_PARAM_NAME_49);
    param_info.type = PARAM_INFO_DOUBLE;
    data_double = (double)array_data[1];
    memcpy(param_info.data, &data_double, sizeof(double));
    manu_param_list_.push_back(param_info);

    strcpy(param_info.name, MANU_PARAM_NAME_50);
    param_info.type = PARAM_INFO_DOUBLE;
    data_double = (double)array_data[2];
    memcpy(param_info.data, &data_double, sizeof(double));
    manu_param_list_.push_back(param_info);

    strcpy(param_info.name, MANU_PARAM_NAME_51);
    param_info.type = PARAM_INFO_DOUBLE;
    data_double = (double)array_data[3];
    memcpy(param_info.data, &data_double, sizeof(double));
    manu_param_list_.push_back(param_info);

    strcpy(param_info.name, MANU_PARAM_NAME_52);
    param_info.type = PARAM_INFO_DOUBLE;
    data_double = (double)array_data[4];
    memcpy(param_info.data, &data_double, sizeof(double));
    manu_param_list_.push_back(param_info);

    strcpy(param_info.name, MANU_PARAM_NAME_53);
    param_info.type = PARAM_INFO_DOUBLE;
    data_double = (double)array_data[5];
    memcpy(param_info.data, &data_double, sizeof(double));
    manu_param_list_.push_back(param_info);

    if(!yaml_help_.getParam(MANU_PARAM_DATA_54, array_data)) return PARAM_MANAGER_INIT_FAILED;

    strcpy(param_info.name, MANU_PARAM_NAME_54);
    param_info.type = PARAM_INFO_DOUBLE;
    data_double = (double)array_data[0];
    memcpy(param_info.data, &data_double, sizeof(double));
    manu_param_list_.push_back(param_info);

    strcpy(param_info.name, MANU_PARAM_NAME_55);
    param_info.type = PARAM_INFO_DOUBLE;
    data_double = (double)array_data[1];
    memcpy(param_info.data, &data_double, sizeof(double));
    manu_param_list_.push_back(param_info);

    strcpy(param_info.name, MANU_PARAM_NAME_56);
    param_info.type = PARAM_INFO_DOUBLE;
    data_double = (double)array_data[2];
    memcpy(param_info.data, &data_double, sizeof(double));
    manu_param_list_.push_back(param_info);

    strcpy(param_info.name, MANU_PARAM_NAME_57);
    param_info.type = PARAM_INFO_DOUBLE;
    data_double = (double)array_data[3];
    memcpy(param_info.data, &data_double, sizeof(double));
    manu_param_list_.push_back(param_info);

    strcpy(param_info.name, MANU_PARAM_NAME_58);
    param_info.type = PARAM_INFO_DOUBLE;
    data_double = (double)array_data[4];
    memcpy(param_info.data, &data_double, sizeof(double));
    manu_param_list_.push_back(param_info);

    strcpy(param_info.name, MANU_PARAM_NAME_59);
    param_info.type = PARAM_INFO_DOUBLE;
    data_double = (double)array_data[5];
    memcpy(param_info.data, &data_double, sizeof(double));
    manu_param_list_.push_back(param_info);
    
    if(!yaml_help_.getParam(MANU_PARAM_DATA_60, array_data)) return PARAM_MANAGER_INIT_FAILED;

    strcpy(param_info.name, MANU_PARAM_NAME_60);
    param_info.type = PARAM_INFO_DOUBLE;
    data_double = (double)array_data[0];
    memcpy(param_info.data, &data_double, sizeof(double));
    manu_param_list_.push_back(param_info);

    strcpy(param_info.name, MANU_PARAM_NAME_61);
    param_info.type = PARAM_INFO_DOUBLE;
    data_double = (double)array_data[1];
    memcpy(param_info.data, &data_double, sizeof(double));
    manu_param_list_.push_back(param_info);

    strcpy(param_info.name, MANU_PARAM_NAME_62);
    param_info.type = PARAM_INFO_DOUBLE;
    data_double = (double)array_data[2];
    memcpy(param_info.data, &data_double, sizeof(double));
    manu_param_list_.push_back(param_info);

    strcpy(param_info.name, MANU_PARAM_NAME_63);
    param_info.type = PARAM_INFO_DOUBLE;
    data_double = (double)array_data[3];
    memcpy(param_info.data, &data_double, sizeof(double));
    manu_param_list_.push_back(param_info);

    strcpy(param_info.name, MANU_PARAM_NAME_64);
    param_info.type = PARAM_INFO_DOUBLE;
    data_double = (double)array_data[4];
    memcpy(param_info.data, &data_double, sizeof(double));
    manu_param_list_.push_back(param_info);

    strcpy(param_info.name, MANU_PARAM_NAME_65);
    param_info.type = PARAM_INFO_DOUBLE;
    data_double = (double)array_data[5];
    memcpy(param_info.data, &data_double, sizeof(double));
    manu_param_list_.push_back(param_info);

    if(!yaml_help_.getParam(MANU_PARAM_DATA_66, data_double)) return PARAM_MANAGER_INIT_FAILED;
    strcpy(param_info.name, MANU_PARAM_NAME_66);
    param_info.type = PARAM_INFO_DOUBLE;
    memcpy(param_info.data, &data_double, sizeof(double));
    manu_param_list_.push_back(param_info); 

    if(!yaml_help_.getParam(MANU_PARAM_DATA_67, data_double)) return PARAM_MANAGER_INIT_FAILED;
    strcpy(param_info.name, MANU_PARAM_NAME_67);
    param_info.type = PARAM_INFO_DOUBLE;
    memcpy(param_info.data, &data_double, sizeof(double));
    manu_param_list_.push_back(param_info); 

    if(!yaml_help_.getParam(MANU_PARAM_DATA_68, data_double)) return PARAM_MANAGER_INIT_FAILED;
    strcpy(param_info.name, MANU_PARAM_NAME_68);
    param_info.type = PARAM_INFO_DOUBLE;
    memcpy(param_info.data, &data_double, sizeof(double));
    manu_param_list_.push_back(param_info); 

    if(!yaml_help_.getParam(MANU_PARAM_DATA_69, data_double)) return PARAM_MANAGER_INIT_FAILED;
    strcpy(param_info.name, MANU_PARAM_NAME_69);
    param_info.type = PARAM_INFO_DOUBLE;
    memcpy(param_info.data, &data_double, sizeof(double));
    manu_param_list_.push_back(param_info); 

    if(!yaml_help_.getParam(MANU_PARAM_DATA_70, data_double)) return PARAM_MANAGER_INIT_FAILED;
    strcpy(param_info.name, MANU_PARAM_NAME_70);
    param_info.type = PARAM_INFO_DOUBLE;
    memcpy(param_info.data, &data_double, sizeof(double));
    manu_param_list_.push_back(param_info); 

    if(!yaml_help_.getParam(MANU_PARAM_DATA_71, data_double)) return PARAM_MANAGER_INIT_FAILED;
    strcpy(param_info.name, MANU_PARAM_NAME_71);
    param_info.type = PARAM_INFO_DOUBLE;
    memcpy(param_info.data, &data_double, sizeof(double));
    manu_param_list_.push_back(param_info); 

    if(!yaml_help_.getParam(MANU_PARAM_DATA_72, data_double)) return PARAM_MANAGER_INIT_FAILED;
    strcpy(param_info.name, MANU_PARAM_NAME_72);
    param_info.type = PARAM_INFO_DOUBLE;
    memcpy(param_info.data, &data_double, sizeof(double));
    manu_param_list_.push_back(param_info); 

    if(!yaml_help_.getParam(MANU_PARAM_DATA_73, data_double)) return PARAM_MANAGER_INIT_FAILED;
    strcpy(param_info.name, MANU_PARAM_NAME_73);
    param_info.type = PARAM_INFO_DOUBLE;
    memcpy(param_info.data, &data_double, sizeof(double));
    manu_param_list_.push_back(param_info); 

    if(!yaml_help_.getParam(MANU_PARAM_DATA_74, data_double)) return PARAM_MANAGER_INIT_FAILED;
    strcpy(param_info.name, MANU_PARAM_NAME_74);
    param_info.type = PARAM_INFO_DOUBLE;
    memcpy(param_info.data, &data_double, sizeof(double));
    manu_param_list_.push_back(param_info); 

    if(!yaml_help_.getParam(MANU_PARAM_DATA_75, data_double)) return PARAM_MANAGER_INIT_FAILED;
    strcpy(param_info.name, MANU_PARAM_NAME_75);
    param_info.type = PARAM_INFO_DOUBLE;
    memcpy(param_info.data, &data_double, sizeof(double));
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
                data_int = *((int*)param_info.data);
                if(!yaml_help_.loadParamFile(FILE_BASE_GROUP)
                    || !yaml_help_.setParam(MANU_PARAM_DATA_2, data_int)
                    || !yaml_help_.dumpParamFile(FILE_BASE_GROUP)) 
                {
                    return PARAM_MANAGER_SET_PARAM_FAILED;
                }
                memcpy(manu_param_list_[1].data, &data_int, sizeof(int));
                return SUCCESS;
            }

            fst_parameter::ParamValue servo_data;
            fst_parameter::ParamValue array_data;
            // servo param - axis1
            if(strcmp(param_info.name, MANU_PARAM_NAME_3) == 0)
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
                memcpy(manu_param_list_[2].data, &data_int, sizeof(int));
                return SUCCESS;
            }
            if(strcmp(param_info.name, MANU_PARAM_NAME_4) == 0)
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
                memcpy(manu_param_list_[3].data, &data_int, sizeof(int));
                return SUCCESS;
            }  
            if(strcmp(param_info.name, MANU_PARAM_NAME_5) == 0)
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
                memcpy(manu_param_list_[4].data, &data_int, sizeof(int));
                return SUCCESS;
            }
            if(strcmp(param_info.name, MANU_PARAM_NAME_6) == 0)
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
                memcpy(manu_param_list_[5].data, &data_int, sizeof(int));
                return SUCCESS;
            }
            // servo param - axis2
            if(strcmp(param_info.name, MANU_PARAM_NAME_7) == 0)
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
                memcpy(manu_param_list_[6].data, &data_int, sizeof(int));
                return SUCCESS;
            }
            if(strcmp(param_info.name, MANU_PARAM_NAME_8) == 0)
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
                memcpy(manu_param_list_[7].data, &data_int, sizeof(int));
                return SUCCESS;
            }  
            if(strcmp(param_info.name, MANU_PARAM_NAME_9) == 0)
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
                memcpy(manu_param_list_[8].data, &data_int, sizeof(int));
                return SUCCESS;
            }
            if(strcmp(param_info.name, MANU_PARAM_NAME_10) == 0)
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
                memcpy(manu_param_list_[9].data, &data_int, sizeof(int));
                return SUCCESS;
            }
            // servo param - axis3
            if(strcmp(param_info.name, MANU_PARAM_NAME_11) == 0)
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
                memcpy(manu_param_list_[10].data, &data_int, sizeof(int));
                return SUCCESS;
            }
            if(strcmp(param_info.name, MANU_PARAM_NAME_12) == 0)
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
                memcpy(manu_param_list_[11].data, &data_int, sizeof(int));
                return SUCCESS;
            }  
            if(strcmp(param_info.name, MANU_PARAM_NAME_13) == 0)
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
                memcpy(manu_param_list_[12].data, &data_int, sizeof(int));
                return SUCCESS;
            }
            if(strcmp(param_info.name, MANU_PARAM_NAME_14) == 0)
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
                memcpy(manu_param_list_[13].data, &data_int, sizeof(int));
                return SUCCESS;
            }
            // servo param - axis4
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
                    servo_data[669] = data_int;                    
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
                    servo_data[672] = data_int;                    
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
                    servo_data[674] = data_int;                    
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
                    servo_data[708] = data_int;                    
                }
                if(!yaml_help_.setParam(MANU_PARAM_DATA_SERVO, servo_data)
                    || !yaml_help_.dumpParamFile(FILE_SERVO_PARAM))
                {
                    return PARAM_MANAGER_SET_PARAM_FAILED;
                }
                memcpy(manu_param_list_[17].data, &data_int, sizeof(int));
                return SUCCESS;
            }
            // servo param - axis5
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
                    servo_data[861] = data_int;                    
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
                    servo_data[864] = data_int;                    
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
                    servo_data[866] = data_int;                    
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
                    servo_data[900] = data_int;                    
                }
                if(!yaml_help_.setParam(MANU_PARAM_DATA_SERVO, servo_data)
                    || !yaml_help_.dumpParamFile(FILE_SERVO_PARAM))
                {
                    return PARAM_MANAGER_SET_PARAM_FAILED;
                }
                memcpy(manu_param_list_[21].data, &data_int, sizeof(int));
                return SUCCESS;
            }
            // servo param - axis6
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
                    servo_data[1053] = data_int;                    
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
                    servo_data[1056] = data_int;                    
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
                    servo_data[1058] = data_int;                    
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
                    servo_data[1092] = data_int;                    
                }
                if(!yaml_help_.setParam(MANU_PARAM_DATA_SERVO, servo_data)
                    || !yaml_help_.dumpParamFile(FILE_SERVO_PARAM))
                {
                    return PARAM_MANAGER_SET_PARAM_FAILED;
                }
                memcpy(manu_param_list_[25].data, &data_int, sizeof(int));
                return SUCCESS;
            }
            // algorithm config
            if(strcmp(param_info.name, MANU_PARAM_NAME_27) == 0)
            {
                data_int = *((int*)param_info.data);
                if(!yaml_help_.loadParamFile(FILE_ALGORITHM_CONFIG)
                    || !yaml_help_.setParam(MANU_PARAM_DATA_27, data_int)
                    || !yaml_help_.dumpParamFile(FILE_ALGORITHM_CONFIG)) 
                {
                    return PARAM_MANAGER_SET_PARAM_FAILED;
                }
                memcpy(manu_param_list_[26].data, &data_int, sizeof(int));
                return SUCCESS;
            }            
            if(strcmp(param_info.name, MANU_PARAM_NAME_28) == 0)
            {
                data_bool = *((bool*)param_info.data);
                if(!yaml_help_.loadParamFile(FILE_ALGORITHM_CONFIG)
                    || !yaml_help_.setParam(MANU_PARAM_DATA_28, data_bool)
                    || !yaml_help_.dumpParamFile(FILE_ALGORITHM_CONFIG)) 
                {
                    return PARAM_MANAGER_SET_PARAM_FAILED;
                }
                memcpy(manu_param_list_[27].data, &data_bool, sizeof(bool));
                return SUCCESS;
            } 
            if(strcmp(param_info.name, MANU_PARAM_NAME_29) == 0)
            {
                data_bool = *((bool*)param_info.data);
                if(!yaml_help_.loadParamFile(FILE_ALGORITHM_CONFIG)
                    || !yaml_help_.setParam(MANU_PARAM_DATA_29, data_bool)
                    || !yaml_help_.dumpParamFile(FILE_ALGORITHM_CONFIG)) 
                {
                    return PARAM_MANAGER_SET_PARAM_FAILED;
                }
                memcpy(manu_param_list_[28].data, &data_bool, sizeof(bool));
                return SUCCESS;
            }            
            if(strcmp(param_info.name, MANU_PARAM_NAME_30) == 0)
            {
                data_double = *((double*)param_info.data);
                if(!yaml_help_.loadParamFile(FILE_ALGORITHM_CONSTRAINT)
                    || !yaml_help_.getParam(MANU_PARAM_DATA_30, array_data))
                {
                    return PARAM_MANAGER_SET_PARAM_FAILED;                    
                }
                else
                {
                    array_data[0] = data_double;                    
                }
                if(!yaml_help_.setParam(MANU_PARAM_DATA_30, array_data)
                    || !yaml_help_.dumpParamFile(FILE_ALGORITHM_CONSTRAINT))
                {
                    return PARAM_MANAGER_SET_PARAM_FAILED;
                }
                memcpy(manu_param_list_[29].data, &data_double, sizeof(double));
                return SUCCESS;
            }
            if(strcmp(param_info.name, MANU_PARAM_NAME_31) == 0)
            {
                data_double = *((double*)param_info.data);
                if(!yaml_help_.loadParamFile(FILE_ALGORITHM_CONSTRAINT)
                    || !yaml_help_.getParam(MANU_PARAM_DATA_30, array_data))
                {
                    return PARAM_MANAGER_SET_PARAM_FAILED;                    
                }
                else
                {
                    array_data[1] = data_double;                    
                }
                if(!yaml_help_.setParam(MANU_PARAM_DATA_30, array_data)
                    || !yaml_help_.dumpParamFile(FILE_ALGORITHM_CONSTRAINT))
                {
                    return PARAM_MANAGER_SET_PARAM_FAILED;
                }
                memcpy(manu_param_list_[30].data, &data_double, sizeof(double));
                return SUCCESS;
            }
            if(strcmp(param_info.name, MANU_PARAM_NAME_32) == 0)
            {
                data_double = *((double*)param_info.data);
                if(!yaml_help_.loadParamFile(FILE_ALGORITHM_CONSTRAINT)
                    || !yaml_help_.getParam(MANU_PARAM_DATA_30, array_data))
                {
                    return PARAM_MANAGER_SET_PARAM_FAILED;                    
                }
                else
                {
                    array_data[2] = data_double;                    
                }
                if(!yaml_help_.setParam(MANU_PARAM_DATA_30, array_data)
                    || !yaml_help_.dumpParamFile(FILE_ALGORITHM_CONSTRAINT))
                {
                    return PARAM_MANAGER_SET_PARAM_FAILED;
                }
                memcpy(manu_param_list_[31].data, &data_double, sizeof(double));
                return SUCCESS;
            }
            if(strcmp(param_info.name, MANU_PARAM_NAME_33) == 0)
            {
                data_double = *((double*)param_info.data);
                if(!yaml_help_.loadParamFile(FILE_ALGORITHM_CONSTRAINT)
                    || !yaml_help_.getParam(MANU_PARAM_DATA_30, array_data))
                {
                    return PARAM_MANAGER_SET_PARAM_FAILED;                    
                }
                else
                {
                    array_data[3] = data_double;                    
                }
                if(!yaml_help_.setParam(MANU_PARAM_DATA_30, array_data)
                    || !yaml_help_.dumpParamFile(FILE_ALGORITHM_CONSTRAINT))
                {
                    return PARAM_MANAGER_SET_PARAM_FAILED;
                }
                memcpy(manu_param_list_[32].data, &data_double, sizeof(double));
                return SUCCESS;
            }
            if(strcmp(param_info.name, MANU_PARAM_NAME_34) == 0)
            {
                data_double = *((double*)param_info.data);
                if(!yaml_help_.loadParamFile(FILE_ALGORITHM_CONSTRAINT)
                    || !yaml_help_.getParam(MANU_PARAM_DATA_30, array_data))
                {
                    return PARAM_MANAGER_SET_PARAM_FAILED;                    
                }
                else
                {
                    array_data[4] = data_double;                    
                }
                if(!yaml_help_.setParam(MANU_PARAM_DATA_30, array_data)
                    || !yaml_help_.dumpParamFile(FILE_ALGORITHM_CONSTRAINT))
                {
                    return PARAM_MANAGER_SET_PARAM_FAILED;
                }
                memcpy(manu_param_list_[33].data, &data_double, sizeof(double));
                return SUCCESS;
            }
            if(strcmp(param_info.name, MANU_PARAM_NAME_35) == 0)
            {
                data_double = *((double*)param_info.data);
                if(!yaml_help_.loadParamFile(FILE_ALGORITHM_CONSTRAINT)
                    || !yaml_help_.getParam(MANU_PARAM_DATA_30, array_data))
                {
                    return PARAM_MANAGER_SET_PARAM_FAILED;                    
                }
                else
                {
                    array_data[5] = data_double;                    
                }
                if(!yaml_help_.setParam(MANU_PARAM_DATA_30, array_data)
                    || !yaml_help_.dumpParamFile(FILE_ALGORITHM_CONSTRAINT))
                {
                    return PARAM_MANAGER_SET_PARAM_FAILED;
                }
                memcpy(manu_param_list_[34].data, &data_double, sizeof(double));
                return SUCCESS;
            }
            if(strcmp(param_info.name, MANU_PARAM_NAME_36) == 0)
            {
                data_double = *((double*)param_info.data);
                if(!yaml_help_.loadParamFile(FILE_ALGORITHM_CONSTRAINT)
                    || !yaml_help_.getParam(MANU_PARAM_DATA_36, array_data))
                {
                    return PARAM_MANAGER_SET_PARAM_FAILED;                    
                }
                else
                {
                    array_data[0] = data_double;                    
                }
                if(!yaml_help_.setParam(MANU_PARAM_DATA_36, array_data)
                    || !yaml_help_.dumpParamFile(FILE_ALGORITHM_CONSTRAINT))
                {
                    return PARAM_MANAGER_SET_PARAM_FAILED;
                }
                memcpy(manu_param_list_[35].data, &data_double, sizeof(double));
                return SUCCESS;
            }
            if(strcmp(param_info.name, MANU_PARAM_NAME_37) == 0)
            {
                data_double = *((double*)param_info.data);
                if(!yaml_help_.loadParamFile(FILE_ALGORITHM_CONSTRAINT)
                    || !yaml_help_.getParam(MANU_PARAM_DATA_36, array_data))
                {
                    return PARAM_MANAGER_SET_PARAM_FAILED;                    
                }
                else
                {
                    array_data[1] = data_double;                    
                }
                if(!yaml_help_.setParam(MANU_PARAM_DATA_36, array_data)
                    || !yaml_help_.dumpParamFile(FILE_ALGORITHM_CONSTRAINT))
                {
                    return PARAM_MANAGER_SET_PARAM_FAILED;
                }
                memcpy(manu_param_list_[36].data, &data_double, sizeof(double));
                return SUCCESS;
            }
            if(strcmp(param_info.name, MANU_PARAM_NAME_38) == 0)
            {
                data_double = *((double*)param_info.data);
                if(!yaml_help_.loadParamFile(FILE_ALGORITHM_CONSTRAINT)
                    || !yaml_help_.getParam(MANU_PARAM_DATA_36, array_data))
                {
                    return PARAM_MANAGER_SET_PARAM_FAILED;                    
                }
                else
                {
                    array_data[2] = data_double;                    
                }
                if(!yaml_help_.setParam(MANU_PARAM_DATA_36, array_data)
                    || !yaml_help_.dumpParamFile(FILE_ALGORITHM_CONSTRAINT))
                {
                    return PARAM_MANAGER_SET_PARAM_FAILED;
                }
                memcpy(manu_param_list_[37].data, &data_double, sizeof(double));
                return SUCCESS;
            }
            if(strcmp(param_info.name, MANU_PARAM_NAME_39) == 0)
            {
                data_double = *((double*)param_info.data);
                if(!yaml_help_.loadParamFile(FILE_ALGORITHM_CONSTRAINT)
                    || !yaml_help_.getParam(MANU_PARAM_DATA_36, array_data))
                {
                    return PARAM_MANAGER_SET_PARAM_FAILED;                    
                }
                else
                {
                    array_data[3] = data_double;                    
                }
                if(!yaml_help_.setParam(MANU_PARAM_DATA_36, array_data)
                    || !yaml_help_.dumpParamFile(FILE_ALGORITHM_CONSTRAINT))
                {
                    return PARAM_MANAGER_SET_PARAM_FAILED;
                }
                memcpy(manu_param_list_[38].data, &data_double, sizeof(double));
                return SUCCESS;
            }
            if(strcmp(param_info.name, MANU_PARAM_NAME_40) == 0)
            {
                data_double = *((double*)param_info.data);
                if(!yaml_help_.loadParamFile(FILE_ALGORITHM_CONSTRAINT)
                    || !yaml_help_.getParam(MANU_PARAM_DATA_36, array_data))
                {
                    return PARAM_MANAGER_SET_PARAM_FAILED;                    
                }
                else
                {
                    array_data[4] = data_double;                    
                }
                if(!yaml_help_.setParam(MANU_PARAM_DATA_36, array_data)
                    || !yaml_help_.dumpParamFile(FILE_ALGORITHM_CONSTRAINT))
                {
                    return PARAM_MANAGER_SET_PARAM_FAILED;
                }
                memcpy(manu_param_list_[39].data, &data_double, sizeof(double));
                return SUCCESS;
            }
            if(strcmp(param_info.name, MANU_PARAM_NAME_41) == 0)
            {
                data_double = *((double*)param_info.data);
                if(!yaml_help_.loadParamFile(FILE_ALGORITHM_CONSTRAINT)
                    || !yaml_help_.getParam(MANU_PARAM_DATA_36, array_data))
                {
                    return PARAM_MANAGER_SET_PARAM_FAILED;                    
                }
                else
                {
                    array_data[5] = data_double;                    
                }
                if(!yaml_help_.setParam(MANU_PARAM_DATA_36, array_data)
                    || !yaml_help_.dumpParamFile(FILE_ALGORITHM_CONSTRAINT))
                {
                    return PARAM_MANAGER_SET_PARAM_FAILED;
                }
                memcpy(manu_param_list_[40].data, &data_double, sizeof(double));
                return SUCCESS;
            }
            if(strcmp(param_info.name, MANU_PARAM_NAME_42) == 0)
            {
                data_double = *((double*)param_info.data);
                if(!yaml_help_.loadParamFile(FILE_ALGORITHM_CONSTRAINT)
                    || !yaml_help_.getParam(MANU_PARAM_DATA_42, array_data))
                {
                    return PARAM_MANAGER_SET_PARAM_FAILED;                    
                }
                else
                {
                    array_data[0] = data_double;                    
                }
                if(!yaml_help_.setParam(MANU_PARAM_DATA_42, array_data)
                    || !yaml_help_.dumpParamFile(FILE_ALGORITHM_CONSTRAINT))
                {
                    return PARAM_MANAGER_SET_PARAM_FAILED;
                }
                memcpy(manu_param_list_[41].data, &data_double, sizeof(double));
                return SUCCESS;
            }
            if(strcmp(param_info.name, MANU_PARAM_NAME_43) == 0)
            {
                data_double = *((double*)param_info.data);
                if(!yaml_help_.loadParamFile(FILE_ALGORITHM_CONSTRAINT)
                    || !yaml_help_.getParam(MANU_PARAM_DATA_42, array_data))
                {
                    return PARAM_MANAGER_SET_PARAM_FAILED;                    
                }
                else
                {
                    array_data[1] = data_double;                    
                }
                if(!yaml_help_.setParam(MANU_PARAM_DATA_42, array_data)
                    || !yaml_help_.dumpParamFile(FILE_ALGORITHM_CONSTRAINT))
                {
                    return PARAM_MANAGER_SET_PARAM_FAILED;
                }
                memcpy(manu_param_list_[42].data, &data_double, sizeof(double));
                return SUCCESS;
            }
            if(strcmp(param_info.name, MANU_PARAM_NAME_44) == 0)
            {
                data_double = *((double*)param_info.data);
                if(!yaml_help_.loadParamFile(FILE_ALGORITHM_CONSTRAINT)
                    || !yaml_help_.getParam(MANU_PARAM_DATA_42, array_data))
                {
                    return PARAM_MANAGER_SET_PARAM_FAILED;                    
                }
                else
                {
                    array_data[2] = data_double;                    
                }
                if(!yaml_help_.setParam(MANU_PARAM_DATA_42, array_data)
                    || !yaml_help_.dumpParamFile(FILE_ALGORITHM_CONSTRAINT))
                {
                    return PARAM_MANAGER_SET_PARAM_FAILED;
                }
                memcpy(manu_param_list_[43].data, &data_double, sizeof(double));
                return SUCCESS;
            }
            if(strcmp(param_info.name, MANU_PARAM_NAME_45) == 0)
            {
                data_double = *((double*)param_info.data);
                if(!yaml_help_.loadParamFile(FILE_ALGORITHM_CONSTRAINT)
                    || !yaml_help_.getParam(MANU_PARAM_DATA_42, array_data))
                {
                    return PARAM_MANAGER_SET_PARAM_FAILED;                    
                }
                else
                {
                    array_data[3] = data_double;                    
                }
                if(!yaml_help_.setParam(MANU_PARAM_DATA_42, array_data)
                    || !yaml_help_.dumpParamFile(FILE_ALGORITHM_CONSTRAINT))
                {
                    return PARAM_MANAGER_SET_PARAM_FAILED;
                }
                memcpy(manu_param_list_[44].data, &data_double, sizeof(double));
                return SUCCESS;
            }
            if(strcmp(param_info.name, MANU_PARAM_NAME_46) == 0)
            {
                data_double = *((double*)param_info.data);
                if(!yaml_help_.loadParamFile(FILE_ALGORITHM_CONSTRAINT)
                    || !yaml_help_.getParam(MANU_PARAM_DATA_42, array_data))
                {
                    return PARAM_MANAGER_SET_PARAM_FAILED;                    
                }
                else
                {
                    array_data[4] = data_double;                    
                }
                if(!yaml_help_.setParam(MANU_PARAM_DATA_42, array_data)
                    || !yaml_help_.dumpParamFile(FILE_ALGORITHM_CONSTRAINT))
                {
                    return PARAM_MANAGER_SET_PARAM_FAILED;
                }
                memcpy(manu_param_list_[45].data, &data_double, sizeof(double));
                return SUCCESS;
            }
            if(strcmp(param_info.name, MANU_PARAM_NAME_47) == 0)
            {
                data_double = *((double*)param_info.data);
                if(!yaml_help_.loadParamFile(FILE_ALGORITHM_CONSTRAINT)
                    || !yaml_help_.getParam(MANU_PARAM_DATA_42, array_data))
                {
                    return PARAM_MANAGER_SET_PARAM_FAILED;                    
                }
                else
                {
                    array_data[5] = data_double;                    
                }
                if(!yaml_help_.setParam(MANU_PARAM_DATA_42, array_data)
                    || !yaml_help_.dumpParamFile(FILE_ALGORITHM_CONSTRAINT))
                {
                    return PARAM_MANAGER_SET_PARAM_FAILED;
                }
                memcpy(manu_param_list_[46].data, &data_double, sizeof(double));
                return SUCCESS;
            }
            if(strcmp(param_info.name, MANU_PARAM_NAME_48) == 0)
            {
                data_double = *((double*)param_info.data);
                if(!yaml_help_.loadParamFile(FILE_ALGORITHM_CONSTRAINT)
                    || !yaml_help_.getParam(MANU_PARAM_DATA_48, array_data))
                {
                    return PARAM_MANAGER_SET_PARAM_FAILED;                    
                }
                else
                {
                    array_data[0] = data_double;                    
                }
                if(!yaml_help_.setParam(MANU_PARAM_DATA_48, array_data)
                    || !yaml_help_.dumpParamFile(FILE_ALGORITHM_CONSTRAINT))
                {
                    return PARAM_MANAGER_SET_PARAM_FAILED;
                }
                memcpy(manu_param_list_[47].data, &data_double, sizeof(double));
                return SUCCESS;
            }
            if(strcmp(param_info.name, MANU_PARAM_NAME_49) == 0)
            {
                data_double = *((double*)param_info.data);
                if(!yaml_help_.loadParamFile(FILE_ALGORITHM_CONSTRAINT)
                    || !yaml_help_.getParam(MANU_PARAM_DATA_48, array_data))
                {
                    return PARAM_MANAGER_SET_PARAM_FAILED;                    
                }
                else
                {
                    array_data[1] = data_double;                    
                }
                if(!yaml_help_.setParam(MANU_PARAM_DATA_48, array_data)
                    || !yaml_help_.dumpParamFile(FILE_ALGORITHM_CONSTRAINT))
                {
                    return PARAM_MANAGER_SET_PARAM_FAILED;
                }
                memcpy(manu_param_list_[48].data, &data_double, sizeof(double));
                return SUCCESS;
            }
            if(strcmp(param_info.name, MANU_PARAM_NAME_50) == 0)
            {
                data_double = *((double*)param_info.data);
                if(!yaml_help_.loadParamFile(FILE_ALGORITHM_CONSTRAINT)
                    || !yaml_help_.getParam(MANU_PARAM_DATA_48, array_data))
                {
                    return PARAM_MANAGER_SET_PARAM_FAILED;                    
                }
                else
                {
                    array_data[2] = data_double;                    
                }
                if(!yaml_help_.setParam(MANU_PARAM_DATA_48, array_data)
                    || !yaml_help_.dumpParamFile(FILE_ALGORITHM_CONSTRAINT))
                {
                    return PARAM_MANAGER_SET_PARAM_FAILED;
                }
                memcpy(manu_param_list_[49].data, &data_double, sizeof(double));
                return SUCCESS;
            }
            if(strcmp(param_info.name, MANU_PARAM_NAME_51) == 0)
            {
                data_double = *((double*)param_info.data);
                if(!yaml_help_.loadParamFile(FILE_ALGORITHM_CONSTRAINT)
                    || !yaml_help_.getParam(MANU_PARAM_DATA_48, array_data))
                {
                    return PARAM_MANAGER_SET_PARAM_FAILED;                    
                }
                else
                {
                    array_data[3] = data_double;                    
                }
                if(!yaml_help_.setParam(MANU_PARAM_DATA_48, array_data)
                    || !yaml_help_.dumpParamFile(FILE_ALGORITHM_CONSTRAINT))
                {
                    return PARAM_MANAGER_SET_PARAM_FAILED;
                }
                memcpy(manu_param_list_[50].data, &data_double, sizeof(double));
                return SUCCESS;
            }
            if(strcmp(param_info.name, MANU_PARAM_NAME_52) == 0)
            {
                data_double = *((double*)param_info.data);
                if(!yaml_help_.loadParamFile(FILE_ALGORITHM_CONSTRAINT)
                    || !yaml_help_.getParam(MANU_PARAM_DATA_48, array_data))
                {
                    return PARAM_MANAGER_SET_PARAM_FAILED;                    
                }
                else
                {
                    array_data[4] = data_double;                    
                }
                if(!yaml_help_.setParam(MANU_PARAM_DATA_48, array_data)
                    || !yaml_help_.dumpParamFile(FILE_ALGORITHM_CONSTRAINT))
                {
                    return PARAM_MANAGER_SET_PARAM_FAILED;
                }
                memcpy(manu_param_list_[51].data, &data_double, sizeof(double));
                return SUCCESS;
            }
            if(strcmp(param_info.name, MANU_PARAM_NAME_53) == 0)
            {
                data_double = *((double*)param_info.data);
                if(!yaml_help_.loadParamFile(FILE_ALGORITHM_CONSTRAINT)
                    || !yaml_help_.getParam(MANU_PARAM_DATA_48, array_data))
                {
                    return PARAM_MANAGER_SET_PARAM_FAILED;                    
                }
                else
                {
                    array_data[5] = data_double;                    
                }
                if(!yaml_help_.setParam(MANU_PARAM_DATA_48, array_data)
                    || !yaml_help_.dumpParamFile(FILE_ALGORITHM_CONSTRAINT))
                {
                    return PARAM_MANAGER_SET_PARAM_FAILED;
                }
                memcpy(manu_param_list_[52].data, &data_double, sizeof(double));
                return SUCCESS;
            }
            if(strcmp(param_info.name, MANU_PARAM_NAME_54) == 0)
            {
                data_double = *((double*)param_info.data);
                if(!yaml_help_.loadParamFile(FILE_ALGORITHM_CONSTRAINT)
                    || !yaml_help_.getParam(MANU_PARAM_DATA_54, array_data))
                {
                    return PARAM_MANAGER_SET_PARAM_FAILED;                    
                }
                else
                {
                    array_data[0] = data_double;                    
                }
                if(!yaml_help_.setParam(MANU_PARAM_DATA_54, array_data)
                    || !yaml_help_.dumpParamFile(FILE_ALGORITHM_CONSTRAINT))
                {
                    return PARAM_MANAGER_SET_PARAM_FAILED;
                }
                memcpy(manu_param_list_[53].data, &data_double, sizeof(double));
                return SUCCESS;
            }
            if(strcmp(param_info.name, MANU_PARAM_NAME_55) == 0)
            {
                data_double = *((double*)param_info.data);
                if(!yaml_help_.loadParamFile(FILE_ALGORITHM_CONSTRAINT)
                    || !yaml_help_.getParam(MANU_PARAM_DATA_54, array_data))
                {
                    return PARAM_MANAGER_SET_PARAM_FAILED;                    
                }
                else
                {
                    array_data[1] = data_double;                    
                }
                if(!yaml_help_.setParam(MANU_PARAM_DATA_54, array_data)
                    || !yaml_help_.dumpParamFile(FILE_ALGORITHM_CONSTRAINT))
                {
                    return PARAM_MANAGER_SET_PARAM_FAILED;
                }
                memcpy(manu_param_list_[54].data, &data_double, sizeof(double));
                return SUCCESS;
            }
            if(strcmp(param_info.name, MANU_PARAM_NAME_56) == 0)
            {
                data_double = *((double*)param_info.data);
                if(!yaml_help_.loadParamFile(FILE_ALGORITHM_CONSTRAINT)
                    || !yaml_help_.getParam(MANU_PARAM_DATA_54, array_data))
                {
                    return PARAM_MANAGER_SET_PARAM_FAILED;                    
                }
                else
                {
                    array_data[2] = data_double;                    
                }
                if(!yaml_help_.setParam(MANU_PARAM_DATA_54, array_data)
                    || !yaml_help_.dumpParamFile(FILE_ALGORITHM_CONSTRAINT))
                {
                    return PARAM_MANAGER_SET_PARAM_FAILED;
                }
                memcpy(manu_param_list_[55].data, &data_double, sizeof(double));
                return SUCCESS;
            }
            if(strcmp(param_info.name, MANU_PARAM_NAME_57) == 0)
            {
                data_double = *((double*)param_info.data);
                if(!yaml_help_.loadParamFile(FILE_ALGORITHM_CONSTRAINT)
                    || !yaml_help_.getParam(MANU_PARAM_DATA_54, array_data))
                {
                    return PARAM_MANAGER_SET_PARAM_FAILED;                    
                }
                else
                {
                    array_data[3] = data_double;                    
                }
                if(!yaml_help_.setParam(MANU_PARAM_DATA_54, array_data)
                    || !yaml_help_.dumpParamFile(FILE_ALGORITHM_CONSTRAINT))
                {
                    return PARAM_MANAGER_SET_PARAM_FAILED;
                }
                memcpy(manu_param_list_[56].data, &data_double, sizeof(double));
                return SUCCESS;
            }
            if(strcmp(param_info.name, MANU_PARAM_NAME_58) == 0)
            {
                data_double = *((double*)param_info.data);
                if(!yaml_help_.loadParamFile(FILE_ALGORITHM_CONSTRAINT)
                    || !yaml_help_.getParam(MANU_PARAM_DATA_54, array_data))
                {
                    return PARAM_MANAGER_SET_PARAM_FAILED;                    
                }
                else
                {
                    array_data[4] = data_double;                    
                }
                if(!yaml_help_.setParam(MANU_PARAM_DATA_54, array_data)
                    || !yaml_help_.dumpParamFile(FILE_ALGORITHM_CONSTRAINT))
                {
                    return PARAM_MANAGER_SET_PARAM_FAILED;
                }
                memcpy(manu_param_list_[57].data, &data_double, sizeof(double));
                return SUCCESS;
            }
            if(strcmp(param_info.name, MANU_PARAM_NAME_59) == 0)
            {
                data_double = *((double*)param_info.data);
                if(!yaml_help_.loadParamFile(FILE_ALGORITHM_CONSTRAINT)
                    || !yaml_help_.getParam(MANU_PARAM_DATA_54, array_data))
                {
                    return PARAM_MANAGER_SET_PARAM_FAILED;                    
                }
                else
                {
                    array_data[5] = data_double;                    
                }
                if(!yaml_help_.setParam(MANU_PARAM_DATA_54, array_data)
                    || !yaml_help_.dumpParamFile(FILE_ALGORITHM_CONSTRAINT))
                {
                    return PARAM_MANAGER_SET_PARAM_FAILED;
                }
                memcpy(manu_param_list_[58].data, &data_double, sizeof(double));
                return SUCCESS;
            }
            if(strcmp(param_info.name, MANU_PARAM_NAME_60) == 0)
            {
                data_double = *((double*)param_info.data);
                if(!yaml_help_.loadParamFile(FILE_ALGORITHM_CONSTRAINT)
                    || !yaml_help_.getParam(MANU_PARAM_DATA_60, array_data))
                {
                    return PARAM_MANAGER_SET_PARAM_FAILED;                    
                }
                else
                {
                    array_data[0] = data_double;                    
                }
                if(!yaml_help_.setParam(MANU_PARAM_DATA_60, array_data)
                    || !yaml_help_.dumpParamFile(FILE_ALGORITHM_CONSTRAINT))
                {
                    return PARAM_MANAGER_SET_PARAM_FAILED;
                }
                memcpy(manu_param_list_[59].data, &data_double, sizeof(double));
                return SUCCESS;
            }
            if(strcmp(param_info.name, MANU_PARAM_NAME_61) == 0)
            {
                data_double = *((double*)param_info.data);
                if(!yaml_help_.loadParamFile(FILE_ALGORITHM_CONSTRAINT)
                    || !yaml_help_.getParam(MANU_PARAM_DATA_60, array_data))
                {
                    return PARAM_MANAGER_SET_PARAM_FAILED;                    
                }
                else
                {
                    array_data[1] = data_double;                    
                }
                if(!yaml_help_.setParam(MANU_PARAM_DATA_60, array_data)
                    || !yaml_help_.dumpParamFile(FILE_ALGORITHM_CONSTRAINT))
                {
                    return PARAM_MANAGER_SET_PARAM_FAILED;
                }
                memcpy(manu_param_list_[60].data, &data_double, sizeof(double));
                return SUCCESS;
            }
            if(strcmp(param_info.name, MANU_PARAM_NAME_62) == 0)
            {
                data_double = *((double*)param_info.data);
                if(!yaml_help_.loadParamFile(FILE_ALGORITHM_CONSTRAINT)
                    || !yaml_help_.getParam(MANU_PARAM_DATA_60, array_data))
                {
                    return PARAM_MANAGER_SET_PARAM_FAILED;                    
                }
                else
                {
                    array_data[2] = data_double;                    
                }
                if(!yaml_help_.setParam(MANU_PARAM_DATA_60, array_data)
                    || !yaml_help_.dumpParamFile(FILE_ALGORITHM_CONSTRAINT))
                {
                    return PARAM_MANAGER_SET_PARAM_FAILED;
                }
                memcpy(manu_param_list_[61].data, &data_double, sizeof(double));
                return SUCCESS;
            }
            if(strcmp(param_info.name, MANU_PARAM_NAME_63) == 0)
            {
                data_double = *((double*)param_info.data);
                if(!yaml_help_.loadParamFile(FILE_ALGORITHM_CONSTRAINT)
                    || !yaml_help_.getParam(MANU_PARAM_DATA_60, array_data))
                {
                    return PARAM_MANAGER_SET_PARAM_FAILED;                    
                }
                else
                {
                    array_data[3] = data_double;                    
                }
                if(!yaml_help_.setParam(MANU_PARAM_DATA_60, array_data)
                    || !yaml_help_.dumpParamFile(FILE_ALGORITHM_CONSTRAINT))
                {
                    return PARAM_MANAGER_SET_PARAM_FAILED;
                }
                memcpy(manu_param_list_[62].data, &data_double, sizeof(double));
                return SUCCESS;
            }
            if(strcmp(param_info.name, MANU_PARAM_NAME_64) == 0)
            {
                data_double = *((double*)param_info.data);
                if(!yaml_help_.loadParamFile(FILE_ALGORITHM_CONSTRAINT)
                    || !yaml_help_.getParam(MANU_PARAM_DATA_60, array_data))
                {
                    return PARAM_MANAGER_SET_PARAM_FAILED;                    
                }
                else
                {
                    array_data[4] = data_double;                    
                }
                if(!yaml_help_.setParam(MANU_PARAM_DATA_60, array_data)
                    || !yaml_help_.dumpParamFile(FILE_ALGORITHM_CONSTRAINT))
                {
                    return PARAM_MANAGER_SET_PARAM_FAILED;
                }
                memcpy(manu_param_list_[63].data, &data_double, sizeof(double));
                return SUCCESS;
            }
            if(strcmp(param_info.name, MANU_PARAM_NAME_65) == 0)
            {
                data_double = *((double*)param_info.data);
                if(!yaml_help_.loadParamFile(FILE_ALGORITHM_CONSTRAINT)
                    || !yaml_help_.getParam(MANU_PARAM_DATA_60, array_data))
                {
                    return PARAM_MANAGER_SET_PARAM_FAILED;                    
                }
                else
                {
                    array_data[5] = data_double;                    
                }
                if(!yaml_help_.setParam(MANU_PARAM_DATA_60, array_data)
                    || !yaml_help_.dumpParamFile(FILE_ALGORITHM_CONSTRAINT))
                {
                    return PARAM_MANAGER_SET_PARAM_FAILED;
                }
                memcpy(manu_param_list_[64].data, &data_double, sizeof(double));
                return SUCCESS;
            }
            if(strcmp(param_info.name, MANU_PARAM_NAME_66) == 0)
            {
                data_double = *((double*)param_info.data);
                if(!yaml_help_.loadParamFile(FILE_ALGORITHM_CONSTRAINT)
                    || !yaml_help_.setParam(MANU_PARAM_DATA_66, data_int)
                    || !yaml_help_.dumpParamFile(FILE_ALGORITHM_CONSTRAINT)) 
                {
                    return PARAM_MANAGER_SET_PARAM_FAILED;
                }
                memcpy(manu_param_list_[65].data, &data_double, sizeof(double));
                return SUCCESS;
            }
            if(strcmp(param_info.name, MANU_PARAM_NAME_67) == 0)
            {
                data_double = *((double*)param_info.data);
                if(!yaml_help_.loadParamFile(FILE_ALGORITHM_CONSTRAINT)
                    || !yaml_help_.setParam(MANU_PARAM_DATA_67, data_int)
                    || !yaml_help_.dumpParamFile(FILE_ALGORITHM_CONSTRAINT)) 
                {
                    return PARAM_MANAGER_SET_PARAM_FAILED;
                }
                memcpy(manu_param_list_[66].data, &data_double, sizeof(double));
                return SUCCESS;
            }
            if(strcmp(param_info.name, MANU_PARAM_NAME_68) == 0)
            {
                data_double = *((double*)param_info.data);
                if(!yaml_help_.loadParamFile(FILE_ALGORITHM_CONSTRAINT)
                    || !yaml_help_.setParam(MANU_PARAM_DATA_68, data_int)
                    || !yaml_help_.dumpParamFile(FILE_ALGORITHM_CONSTRAINT)) 
                {
                    return PARAM_MANAGER_SET_PARAM_FAILED;
                }
                memcpy(manu_param_list_[67].data, &data_double, sizeof(double));
                return SUCCESS;
            }
            if(strcmp(param_info.name, MANU_PARAM_NAME_69) == 0)
            {
                data_double = *((double*)param_info.data);
                if(!yaml_help_.loadParamFile(FILE_ALGORITHM_CONSTRAINT)
                    || !yaml_help_.setParam(MANU_PARAM_DATA_69, data_int)
                    || !yaml_help_.dumpParamFile(FILE_ALGORITHM_CONSTRAINT)) 
                {
                    return PARAM_MANAGER_SET_PARAM_FAILED;
                }
                memcpy(manu_param_list_[68].data, &data_double, sizeof(double));
                return SUCCESS;
            }
            if(strcmp(param_info.name, MANU_PARAM_NAME_70) == 0)
            {
                data_double = *((double*)param_info.data);
                if(!yaml_help_.loadParamFile(FILE_ALGORITHM_CONSTRAINT)
                    || !yaml_help_.setParam(MANU_PARAM_DATA_70, data_int)
                    || !yaml_help_.dumpParamFile(FILE_ALGORITHM_CONSTRAINT)) 
                {
                    return PARAM_MANAGER_SET_PARAM_FAILED;
                }
                memcpy(manu_param_list_[69].data, &data_double, sizeof(double));
                return SUCCESS;
            }
            if(strcmp(param_info.name, MANU_PARAM_NAME_71) == 0)
            {
                data_double = *((double*)param_info.data);
                if(!yaml_help_.loadParamFile(FILE_ALGORITHM_CONSTRAINT)
                    || !yaml_help_.setParam(MANU_PARAM_DATA_71, data_int)
                    || !yaml_help_.dumpParamFile(FILE_ALGORITHM_CONSTRAINT)) 
                {
                    return PARAM_MANAGER_SET_PARAM_FAILED;
                }
                memcpy(manu_param_list_[70].data, &data_double, sizeof(double));
                return SUCCESS;
            }
            if(strcmp(param_info.name, MANU_PARAM_NAME_72) == 0)
            {
                data_double = *((double*)param_info.data);
                if(!yaml_help_.loadParamFile(FILE_ALGORITHM_CONSTRAINT)
                    || !yaml_help_.setParam(MANU_PARAM_DATA_72, data_int)
                    || !yaml_help_.dumpParamFile(FILE_ALGORITHM_CONSTRAINT)) 
                {
                    return PARAM_MANAGER_SET_PARAM_FAILED;
                }
                memcpy(manu_param_list_[71].data, &data_double, sizeof(double));
                return SUCCESS;
            }
            if(strcmp(param_info.name, MANU_PARAM_NAME_73) == 0)
            {
                data_double = *((double*)param_info.data);
                if(!yaml_help_.loadParamFile(FILE_ALGORITHM_CONSTRAINT)
                    || !yaml_help_.setParam(MANU_PARAM_DATA_73, data_int)
                    || !yaml_help_.dumpParamFile(FILE_ALGORITHM_CONSTRAINT)) 
                {
                    return PARAM_MANAGER_SET_PARAM_FAILED;
                }
                memcpy(manu_param_list_[72].data, &data_double, sizeof(double));
                return SUCCESS;
            }
            if(strcmp(param_info.name, MANU_PARAM_NAME_74) == 0)
            {
                data_double = *((double*)param_info.data);
                if(!yaml_help_.loadParamFile(FILE_ALGORITHM_CONSTRAINT)
                    || !yaml_help_.setParam(MANU_PARAM_DATA_74, data_int)
                    || !yaml_help_.dumpParamFile(FILE_ALGORITHM_CONSTRAINT)) 
                {
                    return PARAM_MANAGER_SET_PARAM_FAILED;
                }
                memcpy(manu_param_list_[73].data, &data_double, sizeof(double));
                return SUCCESS;
            }
            if(strcmp(param_info.name, MANU_PARAM_NAME_75) == 0)
            {
                data_double = *((double*)param_info.data);
                if(!yaml_help_.loadParamFile(FILE_ALGORITHM_CONSTRAINT)
                    || !yaml_help_.setParam(MANU_PARAM_DATA_75, data_int)
                    || !yaml_help_.dumpParamFile(FILE_ALGORITHM_CONSTRAINT)) 
                {
                    return PARAM_MANAGER_SET_PARAM_FAILED;
                }
                memcpy(manu_param_list_[74].data, &data_double, sizeof(double));
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
            if(strcmp(param_info.name, USER_PARAM_NAME_10) == 0)
            {
                data_bool = *((bool*)param_info.data);
                if(!yaml_help_.loadParamFile(FILE_IO_MAPPING)
                    || !yaml_help_.setParam(USER_PARAM_DATA_10, data_bool)
                    || !yaml_help_.dumpParamFile(FILE_IO_MAPPING)) 
                {
                    return PARAM_MANAGER_SET_PARAM_FAILED;
                }
                memcpy(user_param_list_[9].data, &data_bool, sizeof(bool));
                return SUCCESS;
            }
            if(strcmp(param_info.name, USER_PARAM_NAME_11) == 0)
            {
                data_bool = *((bool*)param_info.data);
                if(!yaml_help_.loadParamFile(FILE_CONTROLLER)
                    || !yaml_help_.setParam(USER_PARAM_DATA_11, data_bool)
                    || !yaml_help_.dumpParamFile(FILE_CONTROLLER)) 
                {
                    return PARAM_MANAGER_SET_PARAM_FAILED;
                }
                memcpy(user_param_list_[10].data, &data_bool, sizeof(bool));
                return SUCCESS;
            }
            if(strcmp(param_info.name, USER_PARAM_NAME_12) == 0)
            {
                data_int = *((int*)param_info.data);
                if(!yaml_help_.loadParamFile(FILE_FST_SAFETY_DEVICE)
                    || !yaml_help_.setParam(USER_PARAM_DATA_12, data_int)
                    || !yaml_help_.dumpParamFile(FILE_FST_SAFETY_DEVICE)) 
                {
                    return PARAM_MANAGER_SET_PARAM_FAILED;
                }
                memcpy(user_param_list_[11].data, &data_int, sizeof(int));
                return SUCCESS;
            }
            if(strcmp(param_info.name, USER_PARAM_NAME_13) == 0)
            {
                data_int = *((int*)param_info.data);
                if(!yaml_help_.loadParamFile(FILE_FST_SAFETY_DEVICE)
                    || !yaml_help_.setParam(USER_PARAM_DATA_13, data_int)
                    || !yaml_help_.dumpParamFile(FILE_FST_SAFETY_DEVICE)) 
                {
                    return PARAM_MANAGER_SET_PARAM_FAILED;
                }
                memcpy(user_param_list_[12].data, &data_int, sizeof(int));
                return SUCCESS;
            }
            if(strcmp(param_info.name, USER_PARAM_NAME_14) == 0)
            {
                data_int = *((int*)param_info.data);
                if(!yaml_help_.loadParamFile(FILE_FST_SAFETY_DEVICE)
                    || !yaml_help_.setParam(USER_PARAM_DATA_14, data_int)
                    || !yaml_help_.dumpParamFile(FILE_FST_SAFETY_DEVICE)) 
                {
                    return PARAM_MANAGER_SET_PARAM_FAILED;
                }
                memcpy(user_param_list_[13].data, &data_int, sizeof(int));
                return SUCCESS;
            }
            if(strcmp(param_info.name, USER_PARAM_NAME_15) == 0)
            {
                data_int = *((int*)param_info.data);
                if(!yaml_help_.loadParamFile(FILE_PRG_INTERPRETER)
                    || !yaml_help_.setParam(USER_PARAM_DATA_15, data_int)
                    || !yaml_help_.dumpParamFile(FILE_PRG_INTERPRETER)) 
                {
                    return PARAM_MANAGER_SET_PARAM_FAILED;
                }
                memcpy(user_param_list_[14].data, &data_int, sizeof(int));
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
    switch(param_group)
    {
        case PARAM_GROUP_MANU:
            for(uint32_t i = 0; i < manu_param_list_.size(); ++i)
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
             for(uint32_t i = 0; i < user_param_list_.size(); ++i)
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

