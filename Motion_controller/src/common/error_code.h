#ifndef COMMON_ERROR_CODE_H_
#define COMMON_ERROR_CODE_H_


#define SUCCESS (0)

#define BM_INVALID_DTC (unsigned long long int)0x0
#define BM_NUMBER_OF_DTC (int)110
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

#define TOOL_MANAGER_LOAD_PARAM_FAILED (unsigned long long int)0x0011000B00A20001   /*ToolManager load param failed in initialization phase*/
#define TOOL_MANAGER_LOAD_TOOLINFO_FAILED (unsigned long long int)0x0011000B00A20002   /*ToolManager load tool info failed in initialization phase*/
#define TOOL_MANAGER_INVALID_ARG (unsigned long long int)0x0001000200A20003   /*ToolManager has invalid argument*/
#define TOOL_MANAGER_TOOLINFO_FILE_WRITE_FAILED (unsigned long long int)0x0001000200A20004   /*ToolManager failed to write ToolInfo config file*/
#define COORDINATE_MANAGER_LOAD_PARAM_FAILED (unsigned long long int)0x0011000B00A30001   /*CoordinateManager load param failed in initialization phase*/
#define COORDINATE_MANAGER_LOAD_COORDINFO_FAILED (unsigned long long int)0x0011000B00A30002   /*CoordinateManager load tool info failed in initialization phase*/
#define COORDINATE_MANAGER_INVALID_ARG (unsigned long long int)0x0001000200A30003   /*CoordinateManager has invalid argument*/
#define COORDINATE_MANAGER_COORDINFO_FILE_WRITE_FAILED (unsigned long long int)0x0001000200A30004   /*CoordinateManager failed to write ToolInfo config file*/
#define REG_MANAGER_LOAD_PARAM_FAILED (unsigned long long int)0x0011000B00A40001   /*RegManager load param failed in initialization phase*/
#define REG_MANAGER_LOAD_PR_FAILED (unsigned long long int)0x0011000B00A40002   /*RegManager load PrReg info failed in initialization phase*/
#define REG_MANAGER_LOAD_HR_FAILED (unsigned long long int)0x0011000B00A40003   /*RegManager load HrReg info failed in initialization phase*/
#define REG_MANAGER_LOAD_MR_FAILED (unsigned long long int)0x0011000B00A40004   /*RegManager load MrReg info failed in initialization phase*/
#define REG_MANAGER_LOAD_SR_FAILED (unsigned long long int)0x0011000B00A40005   /*RegManager load SrReg info failed in initialization phase*/
#define REG_MANAGER_LOAD_R_FAILED (unsigned long long int)0x0011000B00A40006   /*RegManager load RReg info failed in initialization phase*/
#define REG_MANAGER_INVALID_ARG (unsigned long long int)0x0001000200A40007   /*RegManager has invalid argument*/
#define REG_MANAGER_REG_FILE_WRITE_FAILED (unsigned long long int)0x0001000200A40008   /*RegManager failed to write reg config file*/
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
#define CONTROLLER_LOAD_PARAM_FAILED (unsigned long long int)0x0011000B00A80001   /*Controller load param failed in initialization phase*/
#define CONTROLLER_INIT_OBJECT_FAILED (unsigned long long int)0x0011000B00A80002   /*Controller failed to initialize internal object*/
#define CONTROLLER_CREATE_ROUTINE_THREAD_FAILED (unsigned long long int)0x0011000B00A80003   /*Controller failed to create routine thread*/
#define CONTROLLER_CREATE_HEARTBEAT_THREAD_FAILED (unsigned long long int)0x0011000B00A80003   /*Controller failed to create heartbeat thread*/

#define IK_OUT_OF_WORKSPACE (unsigned long long int)0x00010004006503E9   /*IK failed for Axis 1*/
#define IK_JOINT_OUT_OF_LIMIT (unsigned long long int)0x00010004006503EA   /*IK failed for Axis 2~6*/
#define IK_EXCESSIVE_DISTANCE (unsigned long long int)0x00010004006503EB   /*IK result far away from reference*/
#define FK_JOINT_OUT_OF_LIMIT (unsigned long long int)0x00010004006503F3   /*joint out of limit computing FK*/
#define TARGET_REPEATED (unsigned long long int)0x00000001006603F5   /*Motion Command planned with 0 point*/
#define AXIS_OVERSHOOT (unsigned long long int)0x0001000400650407   /*axis run over target position in moveJ*/
#define AXIS_APPROACHING_LIMIT (unsigned long long int)0x0001000400650408   /*axis approaching and going to crash a limit*/
#define CUBIC_CURVE_PLANNING_FAILED (unsigned long long int)0x0001000400650400   /*planning failed using cubic curve*/
#define MOTION_SPEED_TOO_LOW (unsigned long long int)0x0001000400650401   /*motion speed is too low*/
#define CURVE_PRECISION_TOO_LOW (unsigned long long int)0x000100040065041D   /*get a low precision curve when planning MoveC*/
#define THREE_POINTS_COLINEAR (unsigned long long int)0x000100040065041E   /*given points colinear, cannot define a circle*/
#define PLANNING_MAJOR_ARC (unsigned long long int)0x000100040065041F   /*given circle targets define a major arc*/
#define S_CURVE_PLANNING_FAILED (unsigned long long int)0x0001000400650403   /*failed to plan a S-CURVE*/
#define INVALID_CNT (unsigned long long int)0x0001000400650405   /*given CNT parameter is invalid*/
#define AXIS_INCONSISTENT (unsigned long long int)0x0001000400650409   /*failed to coordinate every axis*/
#define MOTION_INTERNAL_FAULT (unsigned long long int)0x0001000400660001   /*program internal fault*/
#define MOTION_FAIL_IN_INIT (unsigned long long int)0x00010002006603E9   /*initialization failed*/
#define FAIL_LOADING_CONSTRAINT (unsigned long long int)0x00010002006603EA   /*load joint constraint failed*/
#define FAIL_LOADING_PARAMETER (unsigned long long int)0x00010002006603EB   /*load parameter failed*/
#define JOINT_OUT_OF_CONSTRAINT (unsigned long long int)0x00010004006603F3   /*joint out of constraint*/
#define TARGET_OUT_OF_CONSTRAINT (unsigned long long int)0x00010004006603F4   /*target joint out of constraint*/
#define INVALID_PARAMETER (unsigned long long int)0x00010004006603FD   /*APIs called with an invalid parameter*/
#define INVALID_SEQUENCE (unsigned long long int)0x00010004006603FE   /*APIs called in a invalid sequence*/
#define CURRENT_JOINT_OUT_OF_CONSTRAINT (unsigned long long int)0x00000001006603FF   /*current joint is out of constraint when resetArmGroup*/
#define TRAJECTORY_FIFO_FULL (unsigned long long int)0x0000000200660401   /*trajectory FIFO full, cannot fill points into FIFO2*/
#define CARTESIAN_PATH_EXIST (unsigned long long int)0x0001000200660402   /*cartesian point exist, cannot plan joint path*/
#define NO_ENOUGH_POINTS_FIFO1 (unsigned long long int)0x0001000200660407   /*have no enough points in FIFO1*/
#define NO_ENOUGH_POINTS_FIFO2 (unsigned long long int)0x0001000200660408   /*have no enough points in FIFO2*/
#define CALIBRATION_FAULT (unsigned long long int)0x00010002006607D1   /*error while calibrating zero offset*/
#define ZERO_OFFSET_LOST (unsigned long long int)0x00010004006607D2   /*one or more axis lost its zero offset*/
#define ZERO_OFFSET_DEVIATE (unsigned long long int)0x00010004006607D3   /*axis zero offset deviated*/
#define CALIBRATION_FAIL_IN_INIT (unsigned long long int)0x00000002006607DB   /*calibrator initialization failed*/
#define FAIL_GET_FEEDBACK_JOINT (unsigned long long int)0x00010002006607E6   /*fail to get FeedbackJointState*/
#define NEED_INITIALIZATION (unsigned long long int)0x0001000200660411   /*ArmGroup need to initialize */
#define NEED_CALIBRATION (unsigned long long int)0x0001000400660412   /*ArmGroup need to calibrate*/
#define IPC_COMMUNICATION_ERROR (unsigned long long int)0x00010002006607E7   /*fail to communicate with other process*/
#define BARE_CORE_TIMEOUT (unsigned long long int)0x00010004006607E8   /*fail to communicate with bare core*/

#define FAIL_INTERPRETER_BASE                       (unsigned long long int)0x0001000900B50000   /*fail to dump parameter into a file*/
#define FAIL_INTERPRETER_SYNTAX_ERROR               (unsigned long long int)0x0001000900B50001 
#define FAIL_INTERPRETER_UNBALANCED_PARENTHESES     (unsigned long long int)0x0001000900B50002 
#define FAIL_INTERPRETER_NO_EXPRESSION_PRESENT      (unsigned long long int)0x0001000900B50003 
#define FAIL_INTERPRETER_EQUALS_SIGN_EXPECTED       (unsigned long long int)0x0001000900B50004 
#define FAIL_INTERPRETER_NOT_A_VARIABLE             (unsigned long long int)0x0001000900B50005 
#define FAIL_INTERPRETER_LABEL_TABLE_FULL           (unsigned long long int)0x0001000900B50006 
#define FAIL_INTERPRETER_DUPLICATE_SUB_LABEL        (unsigned long long int)0x0001000900B50007 
#define FAIL_INTERPRETER_UNDEFINED_SUB_LABEL        (unsigned long long int)0x0001000900B50008
#define FAIL_INTERPRETER_THEN_EXPECTED              (unsigned long long int)0x0001000900B50009 
#define FAIL_INTERPRETER_TO_EXPECTED                (unsigned long long int)0x0001000900B5000A 
#define FAIL_INTERPRETER_TOO_MANY_NESTED_FOR_LOOPS  (unsigned long long int)0x0001000900B5000B 
#define FAIL_INTERPRETER_NEXT_WITHOUT_FOR           (unsigned long long int)0x0001000900B5000C 
#define FAIL_INTERPRETER_TOO_MANY_NESTED_GOSUB      (unsigned long long int)0x0001000900B5000D 
#define FAIL_INTERPRETER_RETURN_WITHOUT_GOSUB       (unsigned long long int)0x0001000900B5000E 
#define FAIL_INTERPRETER_FILE_NOT_FOUND             (unsigned long long int)0x0001000900B5000F 
#define FAIL_INTERPRETER_MOVL_WITH_JOINT            (unsigned long long int)0x0001000900B50010
#define FAIL_INTERPRETER_MOVJ_WITH_POINT            (unsigned long long int)0x0001000900B50011 
#define FAIL_INTERPRETER_ILLEGAL_LINE_NUMBER        (unsigned long long int)0x0001000900B50012 
#define FAIL_INTERPRETER_FUNC_PARAMS_MISMATCH       (unsigned long long int)0x0001000900B50013 
#define FAIL_INTERPRETER_DUPLICATE_EXEC_MACRO       (unsigned long long int)0x0001000900B50014 
#define INFO_INTERPRETER_BACK_TO_BEGIN              (unsigned long long int)0x0001000200B50015

#define FAIL_INTERPRETER_ALARM_EXEC_BASE            (unsigned long long int)0x0001000900B50100 
#define FAIL_INTERPRETER_USER_ALARM1                (unsigned long long int)0x0001000900B50101
#define FAIL_INTERPRETER_USER_ALARM2                (unsigned long long int)0x0001000900B50102
#define FAIL_INTERPRETER_USER_ALARM3                (unsigned long long int)0x0001000900B50103
#define FAIL_INTERPRETER_USER_ALARM4                (unsigned long long int)0x0001000900B50104
#define FAIL_INTERPRETER_USER_ALARM5                (unsigned long long int)0x0001000900B50105
#define FAIL_INTERPRETER_USER_ALARM6                (unsigned long long int)0x0001000900B50106
#define FAIL_INTERPRETER_USER_ALARM7                (unsigned long long int)0x0001000900B50107
#define FAIL_INTERPRETER_USER_ALARM8                (unsigned long long int)0x0001000900B50108
#define FAIL_INTERPRETER_USER_ALARM9                (unsigned long long int)0x0001000900B50109
#define FAIL_INTERPRETER_USER_ALARM10               (unsigned long long int)0x0001000900B5010A
#define FAIL_INTERPRETER_NOT_IN_PAUSE               (unsigned long long int)0x0001000900B5010B

#define PARSE_IO_PATH_FAILED                    (unsigned long long int)0x0001000400670009   /*cant use current path to set IO*/


#endif

