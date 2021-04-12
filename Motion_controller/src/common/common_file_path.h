#ifndef COMMON_FILE_PATH_H
#define COMMON_FILE_PATH_H

/**
 * @file common_file_path.h
 * @brief The file includes the dir paths of various configuration files.
 * @author zhengyu.shen
 */

#if 1
/**
 * @def COMPONENT_PARAM_FILE_DIR
 *      The dir path for all configuration files of all components.
 */
#define COMPONENT_PARAM_FILE_DIR "/root/install/runtime/component/"
/**
 * @def COMM_CONFIG_DIR
 *      The dir path for the configuration file of core communication. This file should be generated automatically by upper software.
 */
#define COMM_CONFIG_DIR "/root/install/runtime/comm/"
/**
 * @def SYSTEM_MODEL_FILE_DIR
 *      The dir path for the modeling configuration of the system. This file should be generated automatically by upper software.
 */
#define SYSTEM_MODEL_FILE_DIR "/root/install/runtime/system_model/"
#else
#define COMPONENT_PARAM_FILE_DIR "/home/fst/rtm_workspace/install/runtime/component/"
#define COMM_CONFIG_DIR "/home/fst/rtm_workspace/install/runtime/comm/"
#define SYSTEM_MODEL_FILE_DIR "/home/fst/rtm_workspace/install/runtime/system_model/"
#endif

#define MOTOR_MODEL_DIR             "/root/install/runtime/robot/model/motor/"
#define SERVO_MODEL_DIR             "/root/install/runtime/robot/model/servo/"
#define ENCODER_MODEL_DIR           "/root/install/runtime/robot/model/encoder/"
#define CHAIN_MODEL_DIR             "/root/install/runtime/robot/model/chain/"
#define APP_MODEL_DIR               "/root/install/runtime/robot/model/app/"
#define AXIS_GROUP_MODEL_DIR        "/root/install/runtime/robot/model/axis_group/"
#define AXIS_DIR                    "/root/install/runtime/robot/axis/"
#define AXIS_GROUP_DIR              "/root/install/runtime/robot/axis_group/"
#define TOOL_DIR                    "/root/install/runtime/robot/tool/"
#define COORD_DIR                   "/root/install/runtime/robot/coord/"
#define REG_DIR                     "/root/install/runtime/robot/reg/"
#define DEVICE_DIR                  "/root/install/runtime/robot/device/"
#define MODBUS_DIR                  "/root/install/runtime/robot/modbus/"
#define SERVO_DIR                   "/root/install/configuration/machine/"
#define DYNAMICS_DIR                "/root/install/runtime/robot/dynamics/"
#define ALGORITHM_DIR               "/root/install/runtime/robot/algorithm/"

#define TOOL_DIR_MODIFIED            "/root/robot_data/tool/"
#define COORD_DIR_MODIFIED           "/root/robot_data/coord/"
#define REG_DIR_MODIFIED             "/root/robot_data/reg/"
#define NVRAM_DIR                    "/root/robot_data/nvram/"

#endif

