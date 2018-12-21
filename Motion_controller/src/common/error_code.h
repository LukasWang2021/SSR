#ifndef COMMON_ERROR_CODE_H_
#define COMMON_ERROR_CODE_H_


#ifndef SUCCESS
#define SUCCESS 0
#endif

#ifndef FST_SUCCESS
#define FST_SUCCESS 0
#endif

#define BM_INVALID_DTC (unsigned long long int)0x0
#define BM_NUMBER_OF_DTC (int)170
#define BM_DTC_E01 (unsigned long long int)0x0001000B00A10001   /*Controller,inner problem,there might be system damagement, need to change controller*/
#define BM_DTC_E02 (unsigned long long int)0x0001000900A10002   /*Core 1 detected trajectory data timeout when running a continuous trajectory*/
#define BM_DTC_E03 (unsigned long long int)0x0011000B00A10003   /*Core 1 interrupt ISR not run!! */
#define BM_DTC_E04 (unsigned long long int)0x0001000900A10004   /*Core 1 detected Core 0 hear beat missing*/
#define BM_DTC_E05 (unsigned long long int)0x0000000200A10005   /*Core 1 ISR timeout error*/
#define BM_DTC_E06 (unsigned long long int)0x0000000200A10006   /*Core 1 ISR timeout warning*/
#define BM_DTC_E07 (unsigned long long int)0x0000000100A10007   /*Print buf is full,just for debug*/
#define BM_DTC_E08 (unsigned long long int)0x0000000100A10008   /*Log buf is full,just for debug*/
#define BM_DTC_E09 (unsigned long long int)0x0001000900A10009   /*Distance between consecutive points is over limit*/
#define BM_DTC_E10 (unsigned long long int)0x0001000900A1000A   /*Acceleration is over limit*/
#define BM_DTC_E11 (unsigned long long int)0x0001000900A1000B   /*Time stamp error*/
#define BM_DTC_E12 (unsigned long long int)0x0001000900A1000C   /*Motor Setpoint Speed over limit*/
#define OPEN_CORE_MEM_FAIL (unsigned long long int)0x0011000B007103E9   /*fail to open sharedmem of cores when initialization,interaction between cores is not available.*/
#define WRITE_CORE_MEM_FAIL (unsigned long long int)0x00000002007103EA   /*fail to write data on sharedmem of cores.*/
#define READ_CORE_MEM_FAIL (unsigned long long int)0x00000002007103EB   /*fail to read date from sharedmem of cores.*/
#define CREATE_CHANNEL_FAIL (unsigned long long int)0x00110006006F03F3   /*fail to create channel between processes.*/
#define SEND_MSG_FAIL (unsigned long long int)0x00000002006F03F4   /*fail to send msg to the other process.*/
#define RECV_MSG_FAIL (unsigned long long int)0x00000002006F03F5   /*fail to recv msg to the other process.*/
#define BARE_CORE_TIMEOUT (unsigned long long int)0x0001000B0071044C   /*no heartbeat from BARE CORE within a limited time.*/
#define INVALID_SERVICE_ID (unsigned long long int)0x00000002006F044D   /*invalid service ID received from other processes.*/
#define SEND_RESP_FAIL (unsigned long long int)0x00000002006F044E   /*fail to send response to other processes within limited tries.*/
#define MCS_TIMEOUT (unsigned long long int)0x00000002006F044F   /*no heartbeat from Motion Controller within a limited time.*/
#define TOOL_MANAGER_LOG (unsigned long long int)0x0001000100A20000   /*ToolManager log{0}*/
#define TOOL_MANAGER_LOAD_PARAM_FAILED (unsigned long long int)0x0011000B00A20001   /*ToolManager load param failed in initialization phase*/
#define TOOL_MANAGER_LOAD_TOOLINFO_FAILED (unsigned long long int)0x0011000B00A20002   /*ToolManager load tool info failed in initialization phase*/
#define TOOL_MANAGER_INVALID_ARG (unsigned long long int)0x0001000200A20003   /*ToolManager has invalid argument*/
#define TOOL_MANAGER_TOOLINFO_FILE_WRITE_FAILED (unsigned long long int)0x0001000200A20004   /*ToolManager failed to write ToolInfo config file*/
#define COORDINATE_MANAGER_LOG (unsigned long long int)0x0001000100A30000   /*CoordinateManager log{0}*/
#define COORDINATE_MANAGER_LOAD_PARAM_FAILED (unsigned long long int)0x0011000B00A30001   /*CoordinateManager load param failed in initialization phase*/
#define COORDINATE_MANAGER_LOAD_COORDINFO_FAILED (unsigned long long int)0x0011000B00A30002   /*CoordinateManager load user cooridnate info failed in initialization phase*/
#define COORDINATE_MANAGER_INVALID_ARG (unsigned long long int)0x0001000200A30003   /*CoordinateManager has invalid argument*/
#define COORDINATE_MANAGER_COORDINFO_FILE_WRITE_FAILED (unsigned long long int)0x0001000200A30004   /*CoordinateManager failed to write ToolInfo config file*/
#define REG_MANAGER_LOG (unsigned long long int)0x0001000100A40000   /*RegManager log{0}*/
#define REG_MANAGER_LOAD_PARAM_FAILED (unsigned long long int)0x0011000B00A40001   /*RegManager load param failed in initialization phase*/
#define REG_MANAGER_LOAD_PR_FAILED (unsigned long long int)0x0011000B00A40002   /*RegManager load PrReg info failed in initialization phase*/
#define REG_MANAGER_LOAD_HR_FAILED (unsigned long long int)0x0011000B00A40003   /*RegManager load HrReg info failed in initialization phase*/
#define REG_MANAGER_LOAD_MR_FAILED (unsigned long long int)0x0011000B00A40004   /*RegManager load MrReg info failed in initialization phase*/
#define REG_MANAGER_LOAD_SR_FAILED (unsigned long long int)0x0011000B00A40005   /*RegManager load SrReg info failed in initialization phase*/
#define REG_MANAGER_LOAD_R_FAILED (unsigned long long int)0x0011000B00A40006   /*RegManager load RReg info failed in initialization phase*/
#define REG_MANAGER_INVALID_ARG (unsigned long long int)0x0001000200A40007   /*RegManager has invalid argument*/
#define REG_MANAGER_REG_FILE_WRITE_FAILED (unsigned long long int)0x0001000200A40008   /*RegManager failed to write reg config file*/
#define REG_MANAGER_INIT_OBJECT_FAILED (unsigned long long int)0x0011000B00A40009   /*RegManager failed to initialize internal variables*/
#define DEVICE_MANAGER_LOG (unsigned long long int)0x0001000100A50000   /*DeviceManager log{0}*/
#define DEVICE_MANAGER_LOAD_PARAM_FAILED (unsigned long long int)0x0011000B00A50001   /*DeviceManager load param failed in initialization phase*/
#define DEVICE_MANAGER_LOAD_DEVICE_CONFIG_FAILED (unsigned long long int)0x0011000B00A50002   /*DeviceManager load device config failed in initialization phase*/
#define DEVICE_MANAGER_INVALID_DEVICE_TYPE (unsigned long long int)0x0011000B00A50003   /*DeviceManager load invalid type of device from device config file in initialization phase*/
#define DEVICE_MANAGER_INIT_DEVICE_FAILED (unsigned long long int)0x0011000B00A50004   /*DeviceManager failed to init device according to device config file*/
#define DEVICE_MANAGER_DEVICE_ALREADY_EXIST (unsigned long long int)0x0001000200A50005   /*DeviceManager failed to add device because the device has already been exist*/
#define DEVICE_MANAGER_INVALID_ARG (unsigned long long int)0x0001000200A50006   /*DeviceManager has invalid argument*/
#define PROCESS_COMM_LOAD_PARAM_FAILED (unsigned long long int)0x0011000B00A60001   /*ProcessComm load param failed in initialization phase*/
#define PROCESS_COMM_INIT_OBJECT_FAILED (unsigned long long int)0x0011000B00A60002   /*ProcessComm failed to initialize internal variables*/
#define PROCESS_COMM_CONTROLLER_CLIENT_INIT_FAILED (unsigned long long int)0x0011000B00A60003   /*ControllerClient init failed*/
#define PROCESS_COMM_CONTROLLER_SERVER_INIT_FAILED (unsigned long long int)0x0011000B00A60004   /*ControllerServer init failed*/
#define PROCESS_COMM_CONTROLLER_SERVER_OPEN_FAILED (unsigned long long int)0x0011000B00A60005   /*ControllerServer open failed*/
#define PROCESS_COMM_INTERPRETER_CLIENT_INIT_FAILED (unsigned long long int)0x0011000B00A60006   /*InterpreterClient init failed*/
#define PROCESS_COMM_INTERPRETER_SERVER_INIT_FAILED (unsigned long long int)0x0011000B00A60007   /*InterpreterServer init failed*/
#define PROCESS_COMM_INTERPRETER_SERVER_OPEN_FAILED (unsigned long long int)0x0011000B00A60008   /*InterpreterServer open failed*/
#define PROCESS_COMM_HEARTBEAT_CLIENT_INIT_FAILED (unsigned long long int)0x0011000B00A60009   /*HeartbeatClient init failed*/
#define PROCESS_COMM_OPERATION_FAILED (unsigned long long int)0x0000000200A6000A   /*ProcessComm operation failed*/
#define TP_COMM_LOG (unsigned long long int)0x0001000100A70000   /*TpComm log{0}*/
#define TP_COMM_LOAD_PARAM_FAILED (unsigned long long int)0x0011000B00A70001   /*TpComm load param failed in initialization phase*/
#define TP_COMM_INIT_OBJECT_FAILED (unsigned long long int)0x0011000B00A70002   /*TpComm failed to initialize internal variables*/
#define TP_COMM_OPEN_FAILED (unsigned long long int)0x0011000B00A70003   /*TpComm open failed*/
#define TP_COMM_INVALID_REQUEST (unsigned long long int)0x0001000200A70004   /*TpComm receive invalid hash for RPC*/
#define TP_COMM_ENCODE_FAILED (unsigned long long int)0x0001000200A70005   /*TpComm failed to encode data to send out*/
#define TP_COMM_DECODE_FAILED (unsigned long long int)0x0001000200A70006   /*TpComm failed to decode data that has been received*/
#define TP_COMM_MEMORY_OPERATION_FAILED (unsigned long long int)0x0001000200A70007   /*TpComm failed to operate memory*/
#define TP_COMM_AUTHORITY_CHECK_FAILED (unsigned long long int)0x0001000200A70008   /*TpComm failed to run unauthorized operation*/
#define TP_COMM_SEND_FAILED (unsigned long long int)0x0001000200A70009   /*TpComm failed to send package*/
#define TP_COMM_RECEIVE_FAILED (unsigned long long int)0x0001000200A7000A   /*TpComm failed to receive package*/
#define TP_COMM_RPC_OVERLOAD (unsigned long long int)0x0000000200A7000B   /*"TpComm failed to handle too much rpc request"*/
#define CONTROLLER_LOG (unsigned long long int)0x0001000100A80000   /*Controller log{0}*/
#define CONTROLLER_LOAD_PARAM_FAILED (unsigned long long int)0x0011000B00A80001   /*Controller load param failed in initialization phase*/
#define CONTROLLER_INIT_OBJECT_FAILED (unsigned long long int)0x0011000B00A80002   /*Controller failed to initialize internal object*/
#define CONTROLLER_CREATE_ROUTINE_THREAD_FAILED (unsigned long long int)0x0011000B00A80003   /*Controller failed to create routine thread*/
#define CONTROLLER_CREATE_HEARTBEAT_THREAD_FAILED (unsigned long long int)0x0011000B00A80004   /*Controller failed to create heartbeat thread*/
#define CONTROLLER_INVALID_ARG (unsigned long long int)0x0001000200A80005   /*Controller has invalid argument*/
#define CONTROLLER_PUBLISH_FAILED (unsigned long long int)0x0001000200A80006   /*Controller failed to echo the request of publish something*/
#define CONTROLLER_INVALID_OPERATION (unsigned long long int)0x0001000200A80007   /*Controller failed to operate command because of invalid pre-condition*/
#define CONTROLLER_UNKNOWN_USER_OP_MODE (unsigned long long int)0x0001000B00A80008   /*User Op Mode is in unknown state*/
#define INTERPRETER_LOG       (unsigned long long int)0x0001000100AA0000   /*Interpreter log{0}*/
#define FAIL_INTERPRETER_SYNTAX_ERROR (unsigned long long int)0x0001000900AA0001   /*No syntax*/
#define FAIL_INTERPRETER_UNBALANCED_PARENTHESES (unsigned long long int)0x0001000900AA0002   /*unbalanced parentheses*/
#define FAIL_INTERPRETER_NO_EXPRESSION_PRESENT (unsigned long long int)0x0001000900AA0003   /*no expression present*/
#define FAIL_INTERPRETER_EQUALS_SIGN_EXPECTED (unsigned long long int)0x0001000900AA0004   /*equals sign expected*/
#define FAIL_INTERPRETER_NOT_A_VARIABLE (unsigned long long int)0x0001000900AA0005   /*not a variable*/
#define FAIL_INTERPRETER_LABEL_TABLE_FULL (unsigned long long int)0x0001000900AA0006   /*Label table full*/
#define FAIL_INTERPRETER_DUPLICATE_SUB_LABEL (unsigned long long int)0x0001000900AA0007   /*duplicate sub_label*/
#define FAIL_INTERPRETER_UNDEFINED_SUB_LABEL (unsigned long long int)0x0001000900AA0008   /*undefined sub_label*/
#define FAIL_INTERPRETER_THEN_EXPECTED (unsigned long long int)0x0001000900AA0009   /*THEN expected*/
#define FAIL_INTERPRETER_TO_EXPECTED (unsigned long long int)0x0001000900AA000A   /*TO expected*/
#define FAIL_INTERPRETER_TOO_MANY_NESTED_FOR_LOOPS (unsigned long long int)0x0001000900AA000B   /*too many nested FOR loops*/
#define FAIL_INTERPRETER_NEXT_WITHOUT_FOR (unsigned long long int)0x0001000900AA000C   /*NEXT without FOR*/
#define FAIL_INTERPRETER_TOO_MANY_NESTED_GOSUB (unsigned long long int)0x0001000900AA000D   /*too many nested GOSUBs*/
#define FAIL_INTERPRETER_RETURN_WITHOUT_GOSUB (unsigned long long int)0x0001000900AA000E   /*RETURN without GOSUB*/
#define FAIL_INTERPRETER_FILE_NOT_FOUND (unsigned long long int)0x0001000900AA000F   /*no program file*/
#define FAIL_INTERPRETER_MOVL_WITH_JOINT (unsigned long long int)0x0001000900AA0010   /*movl with joint*/
#define FAIL_INTERPRETER_MOVJ_WITH_POINT (unsigned long long int)0x0001000900AA0011   /*movj with point*/
#define FAIL_INTERPRETER_ILLEGAL_LINE_NUMBER (unsigned long long int)0x0001000900AA0012   /*movj with point*/
#define FAIL_INTERPRETER_FUNC_PARAMS_MISMATCH (unsigned long long int)0x0001000900AA0013   /*movj with point*/
#define FAIL_INTERPRETER_DUPLICATE_EXEC_MACRO (unsigned long long int)0x0001000900AA0014   /*movj with point*/
#define INFO_INTERPRETER_BACK_TO_BEGIN    (unsigned long long int)0x0001000200AA0015   /*movj with point*/
#define INFO_INTERPRETER_THREAD_NOT_EXIST (unsigned long long int)0x0001000200AA0016   /*THREAD NOT EXIST */
#define FAIL_INTERPRETER_ALARM_EXEC_BASE (unsigned long long int)0x0001000900AA0100   /*User Alarm BASE*/
#define FAIL_INTERPRETER_USER_ALARM1 (unsigned long long int)0x0001000900AA0101   /*User Alarm 1*/
#define FAIL_INTERPRETER_USER_ALARM2 (unsigned long long int)0x0001000900AA0102   /*User Alarm 2*/
#define FAIL_INTERPRETER_USER_ALARM3 (unsigned long long int)0x0001000900AA0103   /*User Alarm 3*/
#define FAIL_INTERPRETER_USER_ALARM4 (unsigned long long int)0x0001000900AA0104   /*User Alarm 4*/
#define FAIL_INTERPRETER_USER_ALARM5 (unsigned long long int)0x0001000900AA0105   /*User Alarm 5*/
#define FAIL_INTERPRETER_USER_ALARM6 (unsigned long long int)0x0001000900AA0106   /*User Alarm 6*/
#define FAIL_INTERPRETER_USER_ALARM7 (unsigned long long int)0x0001000900AA0107   /*User Alarm 7*/
#define FAIL_INTERPRETER_USER_ALARM8 (unsigned long long int)0x0001000900AA0108   /*User Alarm 8*/
#define FAIL_INTERPRETER_USER_ALARM9 (unsigned long long int)0x0001000900AA0109   /*User Alarm 9*/
#define FAIL_INTERPRETER_USER_ALARM10 (unsigned long long int)0x0001000900AA010A   /*User Alarm 10*/
#define FAIL_INTERPRETER_NOT_IN_PAUSE (unsigned long long int)0x0001000900AA010B   /*Not in PAUSE*/
#define MOTION_CONTROL_LOG (unsigned long long int)0x0001000100A90000   /*MotionControl log{0}*/
#define IK_OUT_OF_WORKSPACE (unsigned long long int)0x0001000400A903E9   /*IK failed for Axis 1*/
#define IK_JOINT_OUT_OF_LIMIT (unsigned long long int)0x0001000400A903EA   /*IK failed for Axis 2~6*/
#define IK_EXCESSIVE_DISTANCE (unsigned long long int)0x0001000400A903EB   /*IK result far away from reference*/
#define FK_JOINT_OUT_OF_LIMIT (unsigned long long int)0x0000000400A903F3   /*joint out of limit computing FK*/
#define AXIS_OVERSHOOT (unsigned long long int)0x0001000400A90407   /*axis run over target position in moveJ*/
#define AXIS_APPROACHING_LIMIT (unsigned long long int)0x0001000400A90408   /*axis approaching and going to crash a limit*/
#define CUBIC_CURVE_PLANNING_FAILED (unsigned long long int)0x0001000400A90400   /*planning failed using cubic curve*/
#define MOTION_SPEED_TOO_LOW (unsigned long long int)0x0001000400A90401   /*motion speed is too low*/
#define CURVE_PRECISION_TOO_LOW (unsigned long long int)0x0001000400A9041D   /*get a low precision curve when planning MoveC*/
#define THREE_POINTS_COLINEAR (unsigned long long int)0x0001000400A9041E   /*given points colinear, cannot define a circle*/
#define MOTION_INTERNAL_FAULT (unsigned long long int)0x0001000400A90001   /*program internal fault*/
#define MOTION_FAIL_IN_INIT (unsigned long long int)0x0011000200A903E9   /*initialization failed*/
#define FAIL_LOADING_CONSTRAINT (unsigned long long int)0x0011000200A903EA   /*load joint constraint failed*/
#define FAIL_LOADING_PARAMETER (unsigned long long int)0x0011000200A903EB   /*load parameter failed*/
#define JOINT_OUT_OF_CONSTRAINT (unsigned long long int)0x0001000400A903F3   /*joint out of constraint*/
#define TARGET_OUT_OF_CONSTRAINT (unsigned long long int)0x0001000400A903F4   /*target joint out of constraint*/
#define INVALID_PARAMETER (unsigned long long int)0x0001000400A903FD   /*APIs called with an invalid parameter*/
#define INVALID_SEQUENCE (unsigned long long int)0x0001000400A903FE   /*APIs called in a invalid sequence*/
#define CALIBRATION_FAULT (unsigned long long int)0x0001000A00A907D1   /*error while calibrating zero offset*/
#define ZERO_OFFSET_LOST (unsigned long long int)0x0001000200A907D2   /*one or more axis lost its zero offset*/
#define ZERO_OFFSET_DEVIATE (unsigned long long int)0x0001000200A907D3   /*axis zero offset deviated*/
#define FAIL_GET_FEEDBACK_JOINT (unsigned long long int)0x0001000600A907E6   /*fail to get FeedbackJointState*/
#define NEED_INITIALIZATION (unsigned long long int)0x0010000200A90411   /*ArmGroup need to initialize */
#define NEED_CALIBRATION (unsigned long long int)0x0001000400A90412   /*ArmGroup need to calibrate*/
#define IPC_COMMUNICATION_ERROR (unsigned long long int)0x0001000200A907E7   /*fail to communication with other process*/
#define PLANNING_MAJOR_ARC (unsigned long long int)0x0001000400A9041F   /*given circle targets define a major arc*/
#define PARAM_LENGTH_ERROR (unsigned long long int)0x00010002007903F5   /*array index beyond range*/
#define PARAM_INTERNAL_FAULT (unsigned long long int)0x0001000B00790001   /*program internal fault*/
#define PARAM_FAIL_IN_INIT (unsigned long long int)0x00010002007903E9   /*initialization failed*/
#define PARAM_NOT_FOUND (unsigned long long int)0x00010002007903F3   /*cannot find the param*/
#define PARAM_TYPE_ERROR (unsigned long long int)0x00010002007903F4   /*param has a type beyond expectation*/
#define PARAM_PARSE_ERROR (unsigned long long int)0x00010002007903FD   /*cannot parse a scalar to expected value type*/
#define COMMUNICATION_ERROR (unsigned long long int)0x0001000200790407   /*cannot communicate with remote server*/
#define BAD_FILE_PATH (unsigned long long int)0x00000002007907D1   /*bad path of config file*/
#define BAD_FILE_EXTENSION (unsigned long long int)0x00000002007907D2   /*bad extension of config file*/
#define BAD_FILE_NAME (unsigned long long int)0x00000002007907D3   /*bad name of config file*/
#define FAIL_OPENNING_FILE (unsigned long long int)0x00010002007907DB   /*open config file failed*/
#define FAIL_BUILDING_PARAM_TREE (unsigned long long int)0x00010002007907DC   /*build param tree failed*/
#define FAIL_RESTORING_YAML (unsigned long long int)0x00010002007907DD   /*restore YAML from backup failed*/
#define FAIL_UPDATING_BACKUP (unsigned long long int)0x00010002007907DE   /*update backup file falled*/
#define FAIL_DUMPING_PARAM (unsigned long long int)0x00010002007907DF   /*fail to dump parameter into a file*/
#define GET_IO_FAIL (unsigned long long int)0x0001000400AC03E9   /*fail to get io data by driver.*/
#define LOAD_IO_CONFIG_FAIL (unsigned long long int)0x0011000400AC03EA   /*fail to load io configuration file，the device can not be used.*/
#define IO_DEVICE_CHANGED (unsigned long long int)0x0001000400AC03EB   /*IO devices are removed when machine is running.*/
#define IO_VERIFY_FALSE (unsigned long long int)0x0000000200AC03EC   /*io data is verified to be false.*/
#define IO_INIT_FAIL (unsigned long long int)0x0011000400AC03F3   /*fail to initialize the IO module.*/
#define IO_INVALID_PARAM_ID (unsigned long long int)0x0000000200AC03F4   /*invalid parameter id as function argument.*/
#define IO_INVALID_PORT_SEQ (unsigned long long int)0x0000000200AC03F5   /*invalid port sequence number as function argument.*/
#define IO_INVALID_DEV_INDEX (unsigned long long int)0x0000000200AC03F6   /*invalid index to get the device info as function argument.*/
#define IO_INVALID_PORT_LEN (unsigned long long int)0x0000000200AC03F7   /*invalid port number of device as function argument.*/
#define IO_THREAD_INIT_STATUS (unsigned long long int)0x0000000200AC03F8   /*io thread is in initial status and not ready.*/
#define INVALID_PATH_FROM_TP (unsigned long long int)0x0000000200AC03F9   /*invalid path*/
#define PARSE_IO_PATH_FAILED (unsigned long long int)0x0000000200AC03FA   /*IO not exist*/
#define IO_MAPPING_LOG (unsigned long long int)0x0001000100AD0000   /*IoMapping log{0}*/

#define MODBUS_LOG (unsigned long long int)0x0011000200AE0000   /*ModbusManager log{0}*/																										
#define MODBUS_MANAGER_SAVE_PARAM_FAILED (unsigned long long int)0x0001000200AE0002   /*modbus_manager save param failed*/																										
#define MODBUS_START_MODE_ERROR (unsigned long long int)0x0001000200AE0003   /*modbus start mode error*/																										
#define MODBUS_INVALID (unsigned long long int)0x0001000200AE0004   /*modbus invalid*/																										
#define MODBUS_CLIENT_EXISTED (unsigned long long int)0x0001000200AE0008   /*modbus client is existed*/																										
#define MODBUS_CLIENT_INVALID_ARG (unsigned long long int)0x0001000200AE0009   /*modbus client param error*/																										
#define MODBUS_CLIENT_CONNECT_FAILED (unsigned long long int)0x0001000200AE000A   /*modbus client connect failed*/																										
#define MODBUS_CLIENT_LOAD_PARAM_FAILED (unsigned long long int)0x0001000200AE000C   /*modbus client load param failed*/																										
#define MODBUS_CLIENT_SAVE_PARAM_FAILED (unsigned long long int)0x0001000200AE000D   /*modbus client save param failed*/																										
#define MODBUS_CLIENT_INIT_FAILED (unsigned long long int)0x0001000200AE000F   /*modbus client init failed*/																										
#define MODBUS_CLIENT_READ_FAILED (unsigned long long int)0x0001000200AE0010   /*modbus client read failed*/																										
#define MODBUS_CLIENT_WRITE_FAILED (unsigned long long int)0x0001000200AE0011   /*modbus client write failed*/																										
#define MODBUS_CLIENT_BE_NOT_OPENED (unsigned long long int)0x0001000200AE0012   /*modbus client be not opened*/																										
#define MODBUS_CLIENT_IS_RUNNING (unsigned long long int)0x0001000200AE0013   /*modbus client is  running*/																										
#define MODBUS_CLIENT_IS_ADDED (unsigned long long int)0x0001000200AE0014   /*modbus client is added*/																										
#define MODBUS_SERVER_BE_NOT_OPENED (unsigned long long int)0x0001000200AE0019   /*modbus server is not be opened*/																										
#define MODBUS_SERVER_SAVE_PARAM_FALIED (unsigned long long int)0x0001000200AE001A   /*modbus server save param failed*/																										
#define MODBUS_SERVER_CONNECT_FALIED (unsigned long long int)0x0001000200AE001B   /*modbus server connect failed */																										
#define MODBUS_SERVER_INVALID_ARG (unsigned long long int)0x0001000200AE001C   /*modbus server invalid param*/																										
#define MODBUS_SERVER_LOAD_PARAM_FALIED (unsigned long long int)0x0001000200AE001D   /*modbus server load param failed*/																										
#define MODBUS_SERVER_OPEN_FAILED (unsigned long long int)0x0001000200AE001E   /*modbus server open failed*/																										
#define MODBUS_SERVER_INIT_FAILED (unsigned long long int)0x0001000200AE001F   /*modbus server init failed*/																										
#define MODBUS_SERVER_FUNCTION_INVALID (unsigned long long int)0x0001000200AE0020   /*modbus server reg invalid*/																										
#define MODBUS_SERVER_IS_RUNNING (unsigned long long int)0x0001000200AE0021   /*modbus server is running*/																										

#define IO_MAPPING_LOAD_PARAM_FAILED (unsigned long long int)0x0010000200AD0001   /*failed to load io_mapping yaml paramters*/
#define IO_MAPPING_LOAD_MAP_FILE_FAILED (unsigned long long int)0x0010000200AD0002   /*failed to load io_mapping json files*/
#define IO_MAPPING_LOAD_SIM_FILE_FAILED (unsigned long long int)0x0010000200AD0003   /*failed to load simused status json files*/
#define PROGRAM_LAUNCHING_LOAD_PARAM_FAILED (unsigned long long int)0x0011000200B00001   /*failed to load program_launching yaml paramters*/
#define PROGRAM_LANNCHING_LOAD_MODE_FILE_FAILED (unsigned long long int)0x0011000200B00002   /*failed to load launch_mode_setting json files*/
#define PROGRAM_LAUNCHING_LOAD_MACRO_FILE_FAILED (unsigned long long int)0x0011000200B00003   /*failed to load macro_io_launch json files*/
#define FILE_MANAGER_READ_FILE_FAILED (unsigned long long int)0x0010000200B10001   /*failed to read file*/
#define FILE_MANAGER_WRITE_FILE_FAILED (unsigned long long int)0x0010000200B10002   /*failed to write file*/



#define TRAJECTORY_FIFO_EMPTY (unsigned long long int)0x0001000400A90BB8   /*trajectory FIFO is empty*/
#define TRAJECTORY_FIFO_FULL (unsigned long long int)0x0001000400A90BB9   /*trajectory FIFO is full*/
#define TRAJECTORY_SEGMENT_ERROR (unsigned long long int)0x0001000400A90BBA   /*trajectory segment is invalid*/
#define TARGET_COINCIDENCE (unsigned long long int)0x0001000200A90BC2   /*target coincidence with start or another target*/

#define PATH_PLANNING_INVALID_TARGET (unsigned long long int)0x0000000400B10001   /*the expected target point is invalid*/
#define TRAJ_PLANNING_INVALID_PATHCACHE (unsigned long long int)0x0000000400B10002   /*invalid path cache*/
#define TRAJ_PLANNING_INVALID_MOTION_TYPE (unsigned long long int)0x0000000400B10003   /*invalid motion type*/
#define TRAJ_PLANNING_INVALID_SMOOTH_IN_INDEX (unsigned long long int)0x0000000400B10004   /*invalid smooth in index*/

#endif

