#ifndef COMMON_ERROR_CODE_H
#define COMMON_ERROR_CODE_H


typedef unsigned long long int ErrorCode;

#define SUCCESS 0

#define CORE_COMM_LOAD_PARAM_FAILED 0x1000
#define CORE_COMM_LOAD_CORE_COMM_CONFIG_FAILED 0x1001
#define CORE_COMM_OPEN_INIT_DEVICE_FAILED    0x1002
#define CORE_COMM_OPEN_CONFIG_DEVICE_FAILED    0x1003
#define CORE_COMM_OPEN_COMM_DEVICE_FAILED    0x1004
#define CORE_COMM_COPY_CONFIG_TO_DEVICE_FAILED    0x1005
#define CORE_COMM_COPY_DEVICE_TO_CONFIG_FAILED    0x1006
#define CORE_COMM_SET_BOARDCAST_FAILED  0x1007
#define CORE_COMM_GET_BOARDCAST_FAILED  0x1008
#define CORE_COMM_SEND_EVENT_FAILED 0x1009
#define CORE_COMM_SLAVE_EVENT_CONFIG_INVALID    0x100A

#define CORE_COMM_INVALID_INIT_TYPE    0x100B
#define CORE_COMM_CONFIGURATION_MISMATCH 0x100C

#define CORE_COMM_SEND_CORE_PROCESS_CALL_FAILED 0x100D
#define CORE_COMM_RECV_CORE_PROCESS_CALL_FAILED 0x100E
#define CORE_COMM_RECV_CORE_PROCESS_CALL_TIMEOUT 0x100F
#define CORE_COMM_EXEC_CORE_PROCESS_CALL_FAILED 0x1010
#define BASE_GROUP_PICK_POINTS_FROM_OFFLINECACHE_NULL 0x1111//从离线轨迹缓存取点数据为空
#define BASE_GROUP_PICK_POINTS_FROM_ONLINECACHE_NULL 0x1112//从在线轨迹缓存取点数据为空
#define BASE_GROUP_FILL_ONLINE_FIFO_ERROR 0x1113 //填充在线轨迹数据队列失败
#define BASE_GROUP_RECV_ONLINE_DOIK_ERROR 0x1114 //在线轨迹 逆解失败
#define BASE_GROUP_RECV_ONLINE_JOINT_OUTLIMIT 0x1115 //填充在线轨迹数据队列失败
#define BASE_GROUP_RECV_ONLINE_NORMAL_END 0x1116 //填充在线轨迹数据队列失败
#define CONTROLLER_INIT_FAILED 0x2000
#define CONTROLLER_CREATE_ROUTINE_THREAD_FAILED 0x2001
#define CONTROLLER_CREATE_ALG_THREAD_FAILED 0x2002
#define CONTROLLER_CREATE_RT_THREAD_FAILED 0x2003
#define CONTROLLER_CREATE_RPC_THREAD_FAILED 0x2004
#define CONTROLLER_CREATE_ONLIE_THREAD_FAILED 0x2005

#define CONTROLLER_PUBLISH_EXIST 0x2011   /*the publishing element of request is exist*/
#define CONTROLLER_INVALID_OPERATION 0x2012   /*Controller failed to operate command because of invalid pre-condition*/
#define CONTROLLER_PUBLISH_NONE 0x2013   /*Controller failed to find the publishing elements of request*/
//#define CONTROLLER_INVALID_OPERATION_SET_TIME (unsigned long long int)0x0001000200A8000A    /* Controller模块不能设置系统时间因运行状态不满足，使控制器进入ESTOP状态后设置 */
#define CONTROLLER_INVALID_OPERATION_START (unsigned long long int)0x0001000200A8000B    /* Controller模块不能start程序因运行状态不满足，使控制器进入ENGAGE状态并且解释器IDLE状态 */
//#define CONTROLLER_INVALID_OPERATION_LAUNCH (unsigned long long int)0x0001000200A8000C    /* Controller模块不能launch程序因运行状态不满足，使控制器进入ENGAGE状态并且解释器IDLE状态 */
//#define CONTROLLER_INVALID_OPERATION_FORWARD (unsigned long long int)0x0001000200A8000D    /* Controller模块不能前进一步程序因运行状态不满足，使控制器进入ENGAGE状态并且用户模式为手动 */
//#define CONTROLLER_INVALID_OPERATION_BACKWARD (unsigned long long int)0x0001000200A8000E    /* Controller模块不能后退一步执行程序因运行状态不满足，使控制器进入ENGAGE状态并且用户模式为手动 */
//#define CONTROLLER_INVALID_OPERATION_JUMP (unsigned long long int)0x0001000200A8000F    /* Controller模块不能跳行执行程序因运行状态不满足，使控制器进入ENGAGE状态并且用户模式为手动 */
// #define CONTROLLER_INVALID_OPERATION_PAUSE (unsigned long long int)0x0001000200A80010    /* Controller模块不能暂停程序因运行状态不满足，控制器在ENGAGE状态并且解释器在执行状态才能执行暂停操作 */
// #define CONTROLLER_INVALID_OPERATION_RESUME (unsigned long long int)0x0001000200A80011    /* Controller模块不能重新执行程序因运行状态不满足，控制器在ENGAGE状态、解释器在暂停状态并且机器人进入IDLE状态 */
//#define CONTROLLER_INVALID_OPERATION_SET_IO (unsigned long long int)0x0001000200A80012    /* Controller模块不能设置IO因运行状态不满足，检查参数（是否允许自动模式下设置IO）：false->不可执行;true->可执行。 */
//#define CONTROLLER_INVALID_OPERATION_SET_LAUNCH (unsigned long long int)0x0001000200A80013    /* Controller模块不能设置启动模式因运行状态不满足，使控制器进入ESTOP状态后设置 */
//#define CONTROLLER_INVALID_OPERATION_SET_VEL (unsigned long long int)0x0001000200A80014    /* Controller模块不能设置全局速度比率因运行状态不满足，检查参数（是否允许在自动模式下设置速度）和限速手动模式的速度限值 */
//#define CONTROLLER_INVALID_OPERATION_SET_ACC (unsigned long long int)0x0001000200A80015    /* Controller模块不能设置全局加速度比率因运行状态不满足，限速手动模式的加速度限值 */
#define CONTROLLER_INVALID_OPERATION_MOVE_STEP (unsigned long long int)0x0001000200A80016    /* Controller模块不能执行步进因运行状态不满足，使控制器进入ENAGED，机器人进入IDLE状态，解释器非执行状态，手动模式。 */
#define CONTROLLER_INVALID_OPERATION_MOVE_CONTINUOUS (unsigned long long int)0x0001000200A80017    /* Controller模块不能执行连续运动因运行状态不满足，使控制器进入ENAGED，解释器非执行状态，手动模式。 */
#define CONTROLLER_INVALID_OPERATION_GOTO_CARTESIAN (unsigned long long int)0x0001000200A80018    /* Controller模块不能执行到笛卡尔坐标的运动因运行状态不满足，使控制器进入ENAGED，解释器非执行状态，手动模式。 */
#define CONTROLLER_INVALID_OPERATION_GOTO_JOINT (unsigned long long int)0x0001000200A80019    /* Controller模块不能执行到关节坐标的运动因运行状态不满足，使控制器进入ENAGED，解释器非执行状态，手动模式。 */
#define CONTROLLER_INVALID_OPERATION_MANUAL_STOP (unsigned long long int)0x0001000200A8001A    /* Controller模块不能执行停止因运行状态不满足，使控制器进入ENAGED，手动模式。 */
//#define CONTROLLER_INVALID_OPERATION_RESET (unsigned long long int)0x0001000200A8001B    /* Controller模块不能执行RESET因运行状态不满足，控制器在INIT，ENAGED，ESTOP状态下可做RESET操作 */
#define CONTROLLER_OFFSET_NEED_CALIBRATE (unsigned long long int)0x0001000200A8001C    /* Controller模块需要对零位进行重新标定 */

#define TP_COMM_LOAD_PARAM_FAILED 0x3001   /*TpComm loading param is failed in initialization phase*/
#define TP_COMM_INIT_OBJECT_FAILED 0x3002   /*TpComm failed to initialize internal variables*/
#define TP_COMM_CREATE_ROUTINE_THREAD_FAILED 0x3003   /*TpComm failed to create routine thread*/
#define TP_COMM_INVALID_REQUEST 0x3004   /*TpComm receives a invalid hash for RPC*/
#define TP_COMM_ENCODE_FAILED 0x3005   /*TpComm failed to encode data to send out*/
#define TP_COMM_DECODE_FAILED 0x3006   /*TpComm failed to decode data that has been received*/
#define TP_COMM_MEMORY_OPERATION_FAILED 0x3007   /*TpComm failed to operate memory*/
#define TP_COMM_AUTHORITY_CHECK_FAILED 0x3008   /*TpComm failed to run a unauthorized operation*/
#define TP_COMM_SEND_FAILED 0x3009   /*TpComm failed to send a package*/
#define TP_COMM_RECEIVE_FAILED 0x300A   /*TpComm failed to receive a package*/
#define TP_COMM_RPC_OVERLOAD 0x300B   /*TpComm failed to handle too much rpc request*/

#define RPC_PARAM_INVALID  0x4001
#define RPC_EXECUTE_FAILED 0x4002

#define AXIS_STATE_TRANSFER_INVALID 0x5001
#define AXIS_SEND_CORE_POWER_FAILED 0x5002
#define AXIS_SEND_CORE_RESET_FAILED 0x5003
#define AXIS_SEND_CORE_STOP_FAILED 0x5004
#define AXIS_SEND_CORE_HALT_FAILED 0x5005
#define AXIS_ALG_NOT_DEFINED 0x5006
#define AXIS_ALG_COMPUTE_FAILED 0x5007
#define AXIS_PDO_TIMEOUT 0x5008
#define AXIS_POWR_ON_TIMEOUT 0x5009
#define AXIS_SET_ZERO_FAILED 0x500A

#define GROUP_INVALID_PARAM 0x6001
#define GROUP_STATE_EXE_INVALID 0x6002
#define GROUP_ADD_AXIS_FAILED 0x6003
#define GROUP_REMOVE_AXIS_FAILED 0x6004
#define GROUP_READ_CONFIG_FAILED 0x6005
#define GROUP_ALG_NOT_DEFINED 0x5006
#define GROUP_ALG_COMPUTE_FAILED 0x6007
#define GROUP_PDO_TIMEOUT 0x6008

#define FILE_MANAGER_READ_FILE_FAILED 0x7001   /*failed to read file*/
#define FILE_MANAGER_WRITE_FILE_FAILED 0x7002   /*failed to write file*/

#define IO_DIGITAL_DEV_DISABLE 0xB001
#define IO_DIGITAL_DATA_VIRIFY_FAILED 0xB002
#define IO_DIGITAL_INVALID_OFFSET 0xB003
#define IO_STEPPER_DIGITAL_DEV_DISABLE 0xB004

#define IO_ANALOG_DEV_LOST 0xB011
#define IO_ANALOG_DEV_LINE_ERR 0xB012
#define IO_ANALOG_DEV_CRC_ERR 0xB013
#define IO_ANALOG_DEV_FRAME_ERR 0xB014
#define IO_ANALOG_INVALID_OFFSET 0xB015

#define IO_SAFETY_DEV_LOST 0xB021
#define IO_SAFETY_DEV_LINE_ERR 0xB022
#define IO_SAFETY_DEV_CRC_ERR 0xB023
#define IO_SAFETY_DEV_FRAME_ERR 0xB024
#define IO_SAFETY_INVALID_OFFSET 0xB025

#define FIO_UNKNOWN 0xC000
#define FIO_DEVICE_CH_BUSY 0xC001
#define FIO_DEVICE_NO_RPL 0xC002
#define FIO_DEVICE_BUSY 0xC003
#define FIO_CMD_CRC_ERR 0xC004
#define FIO_CMD_INVALID 0xC005
#define FIO_CMD_TYPE_ERR 0xC006
#define FIO_CMD_VALUE_ERR 0xC007
#define FIO_EEPROM_LOCKED 0xC008
#define FIO_CMD_NOT_AVAILABLE 0xC009
#define FIO_CMD_EXEC_FAILED 0xC00A
#define FIO_MAX_EXCEEDED 0xC00B
#define FIO_DOWNLOAD_NOT_POSSIBLE 0xC00C
#define FIO_REG_WRITE_PROTECTED 0xC00D
#define FIO_CHIP_READ_FAILED 0xC00E
#define FIO_GET_ID_FAILED 0xC00F

#define REG_MANAGER_LOG (unsigned long long int)0x0000000100A40000    /* RegManager模块日志{0} */
#define REG_MANAGER_LOAD_PARAM_FAILED (unsigned long long int)0x0011000B00A40001    /* RegManager模块在初始化阶段载入模块参数失败 */
#define REG_MANAGER_LOAD_PR_FAILED (unsigned long long int)0x0011000B00A40002    /* RegManager模块在初始化阶段载入Pr寄存器参数失败 */
#define REG_MANAGER_LOAD_HR_FAILED (unsigned long long int)0x0011000B00A40003    /* RegManager模块在初始化阶段载入Hr寄存器参数失败 */
#define REG_MANAGER_LOAD_MR_FAILED (unsigned long long int)0x0011000B00A40004    /* RegManager模块在初始化阶段载入Mr寄存器参数失败 */
#define REG_MANAGER_LOAD_SR_FAILED (unsigned long long int)0x0011000B00A40005    /* RegManager模块在初始化阶段载入Sr寄存器参数失败 */
#define REG_MANAGER_LOAD_R_FAILED (unsigned long long int)0x0011000B00A40006    /* RegManager模块在初始化阶段载入R寄存器参数失败 */
#define REG_MANAGER_INVALID_ARG (unsigned long long int)0x0001000200A40007    /* RegManager模块在操作时传入非法参数 */
#define REG_MANAGER_REG_FILE_WRITE_FAILED (unsigned long long int)0x0001000200A40008    /* RegManager模块在操作时写Reg参数文件失败 */
#define REG_MANAGER_INIT_OBJECT_FAILED (unsigned long long int)0x0011000B00A40009    /* RegManager模块在初始化阶段初始化内部变量失败 */
#define REG_MANAGER_OPERATE_NVRAM_FAILED (unsigned long long int)0x0001000B00A4000A    /* RegManager模块在读写NVRAM数据时发生错误 */

#define TOOL_MANAGER_LOG (unsigned long long int)0x0000000100A20000    /* ToolManager模块日志{0} */
#define TOOL_MANAGER_LOAD_PARAM_FAILED (unsigned long long int)0x0011000B00A20001    /* ToolManager模块在初始化阶段载入模块参数失败 */
#define TOOL_MANAGER_LOAD_TOOLINFO_FAILED (unsigned long long int)0x0011000B00A20002    /* ToolManager模块在初始化阶段载入工具参数失败 */
#define TOOL_MANAGER_INVALID_ARG (unsigned long long int)0x0001000200A20003    /* ToolManager模块在操作时传入非法参数 */
#define TOOL_MANAGER_TOOLINFO_FILE_WRITE_FAILED (unsigned long long int)0x0001000200A20004    /* ToolManager模块在操作时写ToolInfo文件失败 */
#define COORDINATE_MANAGER_LOG (unsigned long long int)0x0000000100A30000    /* CoordinateManager模块日志{0} */
#define COORDINATE_MANAGER_LOAD_PARAM_FAILED (unsigned long long int)0x0011000B00A30001    /* CoordinateManager模块在初始化阶段载入模块参数失败 */
#define COORDINATE_MANAGER_LOAD_COORDINFO_FAILED (unsigned long long int)0x0011000B00A30002    /* CoordinateManager模块在初始化阶段载入用户坐标系参数失败 */
#define COORDINATE_MANAGER_INVALID_ARG (unsigned long long int)0x0001000200A30003    /* CoordinateManager模块在操作时传入非法参数 */
#define COORDINATE_MANAGER_COORDINFO_FILE_WRITE_FAILED (unsigned long long int)0x0001000200A30004    /* CoordinateManager模块在操作时写CoordInfo文件失败 */

#define MOTION_CONTROL_LOG (unsigned long long int)0x0000000100A90000    /* MotionControl模块日志{0} */
#define MC_INTERNAL_FAULT (unsigned long long int)0x0001000B00A90001    /* 运控程序发生严重错误，可能导致不可预知的行为 */
#define MC_FAIL_IN_INIT (unsigned long long int)0x0011000B00A90002    /* 运控程序初始化失败，需要重启 */
// #define MC_NO_ENOUGH_CACHE (unsigned long long int)0x0001000400A90003    /* 路径或轨迹缓存不足 */
#define MC_COMMUNICATION_WITH_BARECORE_FAIL (unsigned long long int)0x0001000600A90004    /* 通过服务通道与裸核通信失败 */
#define MC_OPERATE_NVRAM_FAILED (unsigned long long int)0x0001000600A90005    /* NvRam读写失败 */
#define MC_RECORD_JOINT_TIMEOUT (unsigned long long int)0x0001000400A90006    /* 记录当前关节坐标超时 */
#define MC_SWITCH_STATE_TIMEOUT (unsigned long long int)0x0001000400A90007    /* 状态机跳转超时 */
#define MC_NEAR_SINGULAR_POSITION (unsigned long long int)0x0001000900A90008    /* 运控检测到机器人正在接近奇异位置 */
#define MC_SEND_TRAJECTORY_FAIL (unsigned long long int)0x0001000700A90009    /* 运控向裸核下发轨迹失败 */
#define MC_TOOL_MISMATCH (unsigned long long int)0x0001000400A9000A    /* 运控检测到目标点工具与当前激活工具不匹配 */
#define MC_FRAME_MISMATCH (unsigned long long int)0x0001000400A9000B    /* 运控检测到目标点所用坐标系与当前激活的坐标系不匹配 */
#define MC_POSTURE_MISMATCH (unsigned long long int)0x0001000400A9000C    /* 运控检测到运动起点和终点的形态参数不一致 */
#define MC_TURN_MISMATCH (unsigned long long int)0x0001000400A9000D    /* 运控检测到运动路径上的终点回转圈数和目标点不一致 */
#define MC_TRAJECTORY_OUT_OF_TARGET (unsigned long long int)0x0001000400A9000E    /* 轨迹的终末点与目标点的关节坐标不一致 */
#define MC_COMPUTE_IK_FAIL (unsigned long long int)0x0001000400A903E9    /* 运控求解机器人逆运动学失败 */
#define MC_MANUAL_TO_SINGULAR_POSITION (unsigned long long int)0x0001000400A9000F    /* 运控检测到机器人正在接近奇异位置 */
#define JOINT_OUT_OF_CONSTRAINT (unsigned long long int)0x0001000400A903F3    /* 运控关节角度超出约束条件 */
#define INVALID_PARAMETER (unsigned long long int)0x0001000400A903FD    /* 运控获得的参数不合理，当前操作被中止 */
#define INVALID_SEQUENCE (unsigned long long int)0x0001000400A903FE    /* 运控在当前状态下无法执行指定的指令 */
// #define CALIBRATION_FAULT (unsigned long long int)0x0001000A00A907D1    /* 运控由于文件丢失或者程序错误导致的零位校验失败 */
#define ZERO_OFFSET_LOST (unsigned long long int)0x0001000200A907D2    /* 运控检测到{0}轴零位丢失 */
#define ZERO_OFFSET_INVALID (unsigned long long int)0x0001000200A907D4    /* 运控检测到{0}轴零位异常 */
#define MC_FAIL_GET_FEEDBACK_JOINT (unsigned long long int)0x0001000B00A907E6    /* 运控无法获取当前机器人的关节反馈值 */
#define INVALID_AXIS_ID (unsigned long long int)0x0001000400A907E7    /* 使用了错误的轴ID标号 */
#define MC_COMMUNICATION_WITH_ENCODER_FAIL (unsigned long long int)0x0001000700A907E8    /* 编码器通讯断开 */
// #define TRAJECTORY_FIFO_EMPTY (unsigned long long int)0x0001000400A90BB8    /* 运控轨迹FIFO被消耗完，后续轨迹未就绪 */
#define MC_ARC_PLANNING_FAIL (unsigned long long int)0x0001000400A90BBA    /* 圆弧规划失败 */
#define TARGET_COINCIDENCE (unsigned long long int)0x0001000100A90BC2    /* 运控检测到给定目标位置重合 */
#define MC_PATH_PLANNING_FAIL (unsigned long long int)0x0001000400A90BBB    /* 运控路径规划失败 */
#define MC_TRAJECTORY_PLANNING_FAIL (unsigned long long int)0x0001000400A90BBC    /* 运控轨迹规划失败 */
#define MC_TRAJECTORY_SMOOTH_FAIL (unsigned long long int)0x0001000400A90BBD    /* 运控轨迹平滑失败 */
#define MC_FAIL_MANUAL_TO_POINT (unsigned long long int)0x0001000200A90FA0    /* 运控示教到目标点运动失败 */
#define MC_FAIL_MANUAL_STEP (unsigned long long int)0x0001000200A90FA1    /* 运控示教步进运动失败 */
#define MC_FAIL_MANUAL_CONTINUOUS (unsigned long long int)0x0001000200A90FA2    /* 运控示教连续运动失败 */
#define MC_MANUAL_FRAME_ERROR (unsigned long long int)0x0001000400A90FA3    /* 运控手动示教坐标系错误 */
#define MC_PAUSE_FAILED (unsigned long long int)0x0001000700A90BBE    /* 规划器规划减速暂停轨迹失败 */
#define MC_RESUME_FAILED (unsigned long long int)0x0001000700A90BBF    /* 规划器规划加速恢复轨迹失败 */
#define MC_LOAD_PARAM_FAILED (unsigned long long int)0x0001000700A90BC0
#define MC_SET_PARAM_FAILED (unsigned long long int)0x0001000700A90BC1
#define MC_VP2TRAJ_PLAN_FAILED (unsigned long long int)0x0001000700A90BC2

#define DYNAMIC_PAYLOAD_INVALID_ARG (unsigned long long int)0x0001000200B40001    /* Dynamic模块的payload传入非法参数 */
#define DYNAMIC_PAYLOAD_INFO_FILE_WRITE_FAILED (unsigned long long int)0x0001000200B40002    /* Dynamic模块的payload数值写入文件失败 */
#define DYNAMIC_PAYLOAD_UPDATE_PARAM_FAILED (unsigned long long int)0x0001000200B40003    /* Dynamic模块的动力学参数更新失败 */

/*interpreter error code*/
#define INTERPRETER_ERROR_API_NULL 0x90A00001
#define INTERPRETER_ERROR_INVALID_ARG 0x90A00002
#define INTERPRETER_ERROR_INVALID_MODE 0x10A00003
#define INTERPRETER_ERROR_INVALID_STATE 0x10A00004
#define INTERPRETER_ERROR_HOLD_FAILED 0x90A00005
#define INTERPRETER_ERROR_RELEASE_FAILED 0x90A00006
#define INTERPRETER_ERROR_PROG_NOT_EXIST 0x90A00007
#define INTERPRETER_ERROR_MOD_INVALID_ARG 0x50A00008
#define INTERPRETER_ERROR_INVALID_COORD_TYPE 0x50A00009
#define INTERPRETER_ERROR_SYNC_CALL_FAILED 0x50A0000A
#define INTERPRETER_ERROR_TRAJ_INFO_INVALID 0x50A0000B
#define INTERPRETER_ERROR_CONFIG_LOAD_FAILED 0x50A0000C
#define INTERPRETER_ERROR_START_THREAD_FAILED 0x50A0000D
#define INTERPRETER_ERROR_MEM_ALLOCATE_FAILED 0x50A0000E
#define INTERPRETER_ERROR_CREATE_SUB_FAILED 0x50A0000F
#define INTERPRETER_ERROR_RESET_FAILED 0x50A00010

#endif

