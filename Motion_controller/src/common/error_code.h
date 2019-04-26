#ifndef COMMON_ERROR_CODE_H_
#define COMMON_ERROR_CODE_H_


#ifndef SUCCESS
#define SUCCESS 0
#endif

#ifndef FST_SUCCESS
#define FST_SUCCESS 0
#endif

typedef unsigned long long int ErrorCode;

#define BM_INVALID_DTC (unsigned long long int)0x0
#define BM_NUMBER_OF_DTC (int)261
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
#define BM_DTC_E13 (unsigned long long int)0x0001000900A1000D   /*Core 1 waiting servo down timeout*/
#define MOTION_CONTROL_LOG (unsigned long long int)0x0000000100A90000   /*MotionControl log{0}*/
#define MC_INTERNAL_FAULT (unsigned long long int)0x0001000400A90001   /*program internal fault*/
#define MC_FAIL_IN_INIT (unsigned long long int)0x0011000200A90002   /*initialization failed*/
#define MC_NO_ENOUGH_CACHE (unsigned long long int)0x0001000400A90003   /*no enough path or trajectory cache*/
#define MC_COMMUNICATION_WITH_BARECORE_FAIL (unsigned long long int)0x0001000400A90004   /*lost service communication with barecore*/
#define MC_NVRAM_DATA_INVALID (unsigned long long int)0x0001000400A90005   /*data in NVRAM is invalid*/
#define MC_RECORD_JOINT_TIMEOUT (unsigned long long int)0x0001000400A90006   /*record joint to nvram timeout*/
#define MC_SWITCH_STATE_TIMEOUT (unsigned long long int)0x0001000400A90007   /*group switch from one state to other timeout*/
#define MC_JOINT_TRACKING_ERROR (unsigned long long int)0x0001000400A90008   /*joint tracking error beyond threshold*/
#define MC_COMPUTE_IK_FAIL (unsigned long long int)0x0001000400A903E9   /*compute IK failed*/
#define FAIL_LOADING_PARAMETER (unsigned long long int)0x0011000200A903EB   /*load parameter failed*/
#define JOINT_OUT_OF_CONSTRAINT (unsigned long long int)0x0001000400A903F3   /*joint out of constraint*/
#define INVALID_PARAMETER (unsigned long long int)0x0001000400A903FD   /*APIs called with an invalid parameter*/
#define INVALID_SEQUENCE (unsigned long long int)0x0001000400A903FE   /*APIs called in a invalid sequence*/
#define CALIBRATION_FAULT (unsigned long long int)0x0001000A00A907D1   /*error while calibrating zero offset*/
#define ZERO_OFFSET_LOST (unsigned long long int)0x0001000200A907D2   /*one or more axis lost its zero offset*/
#define ZERO_OFFSET_DEVIATE (unsigned long long int)0x0001000200A907D3   /*axis zero offset deviated*/
#define FAIL_GET_FEEDBACK_JOINT (unsigned long long int)0x0001000600A907E6   /*fail to get FeedbackJointState*/
#define IPC_COMMUNICATION_ERROR (unsigned long long int)0x0001000200A907E7   /*fail to communication with other process*/
#define TRAJECTORY_FIFO_FULL (unsigned long long int)0x0000000100A90BB9   /*trajectory fifo is full*/
#define TRAJECTORY_FIFO_EMPTY (unsigned long long int)0x0001000400A90BB8   /*trajectory fifo is empty*/
#define TRAJECTORY_SEGMENT_ERROR (unsigned long long int)0x0001000400A90BBA   /*duration of trajectory segment is invalid*/
#define TARGET_COINCIDENCE (unsigned long long int)0x0001000200A90BC2   /*target pose is same as target or start pose*/
#define MC_PATH_PLANNING_FAIL (unsigned long long int)0x0001000400A90BBB   /*fail to plan a valid path*/
#define MC_TRAJECTORY_PLANNING_FAIL (unsigned long long int)0x0001000400A90BBC   /*fail to plan a valid trajectory*/
#define MC_FAIL_MANUAL_TO_POINT (unsigned long long int)0x0001000400A90FA0   /*fail to manual move to target point*/
#define MC_FAIL_MANUAL_STEP (unsigned long long int)0x0001000400A90FA1   /*fail to manual move step*/
#define MC_FAIL_MANUAL_CONTINUOUS (unsigned long long int)0x0001000400A90FA2   /*fail to manual move continuous*/
#define MC_MANUAL_FRAME_ERROR (unsigned long long int)0x0001000400A90FA3   /*invalid manual frame*/
#define PARAM_LENGTH_ERROR (unsigned long long int)0x00010002007903F5   /*array index beyond range*/
#define PARAM_INTERNAL_FAULT (unsigned long long int)0x0001000B00790001   /*program internal fault*/
#define PARAM_FAIL_IN_INIT (unsigned long long int)0x00010002007903E9   /*initialization failed*/
#define PARAM_NOT_FOUND (unsigned long long int)0x00010002007903F3   /*cannot find the param*/
#define PARAM_TYPE_ERROR (unsigned long long int)0x00010002007903F4   /*param has a type beyond expectation*/
#define PARAM_PARSE_ERROR (unsigned long long int)0x00010002007903FD   /*cannot parse a scalar to expected value type*/
#define BAD_FILE_PATH (unsigned long long int)0x00000002007907D1   /*bad path of config file*/
#define BAD_FILE_EXTENSION (unsigned long long int)0x00000002007907D2   /*bad extension of config file*/
#define BAD_FILE_NAME (unsigned long long int)0x00000002007907D3   /*bad name of config file*/
#define FAIL_OPENNING_FILE (unsigned long long int)0x00010002007907DB   /*open config file failed*/
#define FAIL_BUILDING_PARAM_TREE (unsigned long long int)0x00010002007907DC   /*build param tree failed*/
#define FAIL_RESTORING_YAML (unsigned long long int)0x00010002007907DD   /*restore YAML from backup failed*/
#define FAIL_UPDATING_BACKUP (unsigned long long int)0x00010002007907DE   /*update backup file falled*/
#define FAIL_DUMPING_PARAM (unsigned long long int)0x00010002007907DF   /*fail to dump parameter into a file*/
#define OPEN_CORE_MEM_FAIL (unsigned long long int)0x0011000B009F0001   /*MiddleWare fail to open sharedmem of cores when initialization,interaction between cores is not available.*/
#define WRITE_CORE_MEM_FAIL (unsigned long long int)0x00000002009F0002   /*MiddleWare fail to write data on sharedmem of cores.*/
#define READ_CORE_MEM_FAIL (unsigned long long int)0x00000002009F0003   /*MiddleWare fail to read date from sharedmem of cores.*/
#define CREATE_CHANNEL_FAIL (unsigned long long int)0x00110006009F0065   /*MiddleWare fail to create channel between processes.*/
#define SEND_MSG_FAIL (unsigned long long int)0x00000002009F0066   /*MiddleWare fail to send msg to the other process.*/
#define RECV_MSG_FAIL (unsigned long long int)0x00000002009F0067   /*MiddleWare fail to recv msg to the other process.*/
#define BARE_CORE_TIMEOUT (unsigned long long int)0x0001000B00A00001   /*ServiceManager can't get heartbeat from BARE CORE within a limited time.*/
#define INVALID_SERVICE_ID (unsigned long long int)0x0000000200A00002   /*ServiceManager invalid service ID received from other processes.*/
#define SEND_RESP_FAIL (unsigned long long int)0x0000000200A00003   /*ServiceManager fail to send response to other processes within limited tries.*/
#define MCS_TIMEOUT (unsigned long long int)0x0000000200A00004   /*ServiceManager can't get heartbeat from Motion Controller within a limited time.*/
#define ERR_SAFETY_FILE_OPEN (unsigned long long int)0x0011000B00AB0001   /*SafetyDevice can't open memory device file*/
#define ERR_SAFETY_FILE_MAP (unsigned long long int)0x0011000B00AB0002   /*SafetyDevice can't map memory device file*/
#define ERR_SAFETY_FPGA_MCU_NOT_CONNECT (unsigned long long int)0x0001000B00AB0003   /*SafetyDevice has no communication between FPGA and MCU*/
#define ERR_SAFETY_FPGA_CORE0_NOT_CONNECT (unsigned long long int)0x0001000B00AB0004   /*SafetyDevice detects that FPGA can't read heartbeat from core0*/
#define ERR_SAFETY_FPGA_CORE1_NOT_CONNECT (unsigned long long int)0x0001000B00AB0005   /*SafetyDevice detects that FPGA can't read heartbeat from core1*/
#define ERR_SAFETY_PTHREAD_INIT (unsigned long long int)0x0011000B00AB0006   /*SafetyDevice Mutex initialization is failed*/
#define ERR_SAFETY_PTHREAD_LOCK (unsigned long long int)0x0001000B00AB0007   /*SafetyDevice Mutex lock is failed*/
#define ERR_SAFETY_PTHREAD_UNLOCK (unsigned long long int)0x0001000B00AB0008   /*SafetyDevice Mutex unlock is failed*/
#define ERR_SAFETY_FRAME (unsigned long long int)0x0001000B00AB0009   /*SafetyDevice inner safety param is out of range*/
#define LOAD_IO_CONFIG_FAIL (unsigned long long int)0x0011000600AC0001   /*IoDevice fail to load IO configuration file，the device can not be used.*/
#define IO_INIT_FAIL (unsigned long long int)0x0011000600AC0002   /*Iodevice fail to initialize the sharedmem.*/
#define GET_IO_FAIL (unsigned long long int)0x0001000600AC0003   /*IoDevice fail to get IO data.*/
#define IO_DEVICE_UNFOUND (unsigned long long int)0x0001000600AC0004   /*Iodevice is unfound when machine is running.*/
#define IO_VERIFY_FALSE (unsigned long long int)0x0000000200AC0005   /*Iodevice data is verified to be false.*/
#define IO_INVALID_PORT_SEQ (unsigned long long int)0x0001000600AC0006   /*IoDevice invalid port sequence number */
#define IO_MAPPING_LOAD_PARAM_FAILED (unsigned long long int)0x0011000200AD0001   /*IoMapping failed to load io_mapping yaml paramters*/
#define IO_MAPPING_LOAD_MAP_FILE_FAILED (unsigned long long int)0x0011000200AD0002   /*IoMapping failed to load io_mapping json files*/
#define IO_MAPPING_LOAD_SIM_FILE_FAILED (unsigned long long int)0x0011000200AD0003   /*IoMapping failed to load simused status json files*/
#define IO_MAPPING_LOG (unsigned long long int)0x0000000100AD0000   /*IoMapping log{0}*/
#define IO_MANAGER_LOG (unsigned long long int)0x0000000100AF0000   /*IoManager log{0}*/
#define IO_INVALID_PARAM_ID (unsigned long long int)0x0001000600AC0001   /*IoManager invalid parameter pysical_id*/
#define PROGRAM_LAUNCHING_LOG (unsigned long long int)0x0000000100B00000   /*ProgramLaunching log{0}*/
#define PROGRAM_LAUNCHING_LOAD_PARAM_FAILED (unsigned long long int)0x0011000200B00001   /*ProgramLaunching failed to load program_launching yaml paramters*/
#define PROGRAM_LANNCHING_LOAD_MODE_FILE_FAILED (unsigned long long int)0x0011000200B00002   /*ProgramLaunching failed to load launch_mode_setting json files*/
#define PROGRAM_LAUNCHING_LOAD_MACRO_FILE_FAILED (unsigned long long int)0x0011000200B00003   /*ProgramLaunching failed to load macro_io_launch json files*/
#define FILE_MANAGER_READ_FILE_FAILED (unsigned long long int)0x0010000200B10001   /*FileManager fail to read file*/
#define FILE_MANAGER_WRITE_FILE_FAILED (unsigned long long int)0x0010000200B10002   /*FileManager fail to write file*/
#define SAFETY_BOARD_CABINET_RESET (unsigned long long int)0x0001000200C80000   /*the cabinet reset button is triggered by the user.*/
#define INFO_RESET_SUCCESS (unsigned long long int)0x0001000100000000   /*The controller is reset successfully.*/
#define SAFETY_BOARD_RELAY_DUAL_FAULTY (unsigned long long int)0x0001000B00C80001   /*the safety board dual channel data are not the same.*/
#define SAFETY_BOARD_EXTERNAL_STOP (unsigned long long int)0x0001000B00C80002   /*the safety board detects external stop signal.*/
#define SAFETY_BOARD_SAFETY_DOOR_STOP (unsigned long long int)0x0001000B00C80003   /*the safety board detects safty door stop signal.*/
#define SAFETY_BOARD_LIMITED_STOP (unsigned long long int)0x0001000B00C80004   /*the safety board detects limited stop signal.*/
#define SAFETY_BOARD_DEADMAN_NORMAL_FAULTY (unsigned long long int)0x0001000600C80005   /*the safety board detects deadman normal is abnormal when manaul mode.*/
#define SAFETY_BOARD_DEADMAN_PANIC (unsigned long long int)0x0001000B00C80006   /*the safety board detects deadman panic signal.*/
#define SAFETY_BOARD_TP_ESTOP (unsigned long long int)0x0001000B00C80007   /*the safety board detects TP-ESTOP signal.*/
#define SAFETY_BOARD_OP_MODE_FAULTY (unsigned long long int)0x0001000600C80008   /*the safety board detects abnormal operation mode.*/
#define SAFETY_BOARD_MAIN_CONTACTOR_FAULTY (unsigned long long int)0x0001000B00C80009   /*the safety board detects the main contactor is abnormal.*/
#define SAFETY_BOARD_MAIN_BRAKE_RELAY_FAULTY (unsigned long long int)0x0001000B00C8000A   /*the safety board detects the relays on 1-6 axes are abnormal.*/
#define SAFETY_BOARD_AUX_BRAKE_RELAY_ONE_FAULTY (unsigned long long int)0x0001000B00C8000B   /*the safety board detects the relays of the first auxiliary axis is abnormal.*/
#define SAFETY_BOARD_AUX_BRAKE_RELAY_TWO_FAULTY (unsigned long long int)0x0001000B00C8000C   /*the safety board detects the relay of the second auxiliary axis is abnormal.*/
#define SAFETY_BOARD_CONTACTOR_RELAY_ZERO_FAULTY (unsigned long long int)0x0001000B00C8000D   /*the safety board detects the first relay of the main contactor is abnormal.*/
#define SAFETY_BOARD_CONTACTOR_RELAY_ONE_FAULTY (unsigned long long int)0x0001000B00C8000E   /*the safety board detects the second relay of the main contactor is abnormal.*/
#define SAFETY_BOARD_CABINET_STOP (unsigned long long int)0x0001000B00C8000F   /*the safety board detects cabinet estop signal.*/
#define SAFETY_BOARD_COMM_ERROR (unsigned long long int)0x0001000B00C80010   /*the safety board detects data error between FPGA and safety_board.*/
#define SAFETY_BOARD_KEY_SWITCH_UNDER_ENGAGED (unsigned long long int)0x0001000600C80011   /*the safety board detects key switch under engaged*/
#define TOOL_MANAGER_LOG (unsigned long long int)0x0000000100A20000   /*ToolManager log{0}*/
#define TOOL_MANAGER_LOAD_PARAM_FAILED (unsigned long long int)0x0011000B00A20001   /*ToolManager load param failed in initialization phase*/
#define TOOL_MANAGER_LOAD_TOOLINFO_FAILED (unsigned long long int)0x0011000B00A20002   /*ToolManager load tool info failed in initialization phase*/
#define TOOL_MANAGER_INVALID_ARG (unsigned long long int)0x0001000200A20003   /*ToolManager has invalid argument*/
#define TOOL_MANAGER_TOOLINFO_FILE_WRITE_FAILED (unsigned long long int)0x0001000200A20004   /*ToolManager failed to write ToolInfo config file*/
#define COORDINATE_MANAGER_LOG (unsigned long long int)0x0000000100A30000   /*CoordinateManager log{0}*/
#define COORDINATE_MANAGER_LOAD_PARAM_FAILED (unsigned long long int)0x0011000B00A30001   /*CoordinateManager load param failed in initialization phase*/
#define COORDINATE_MANAGER_LOAD_COORDINFO_FAILED (unsigned long long int)0x0011000B00A30002   /*CoordinateManager load user cooridnate info failed in initialization phase*/
#define COORDINATE_MANAGER_INVALID_ARG (unsigned long long int)0x0001000200A30003   /*CoordinateManager has invalid argument*/
#define COORDINATE_MANAGER_COORDINFO_FILE_WRITE_FAILED (unsigned long long int)0x0001000200A30004   /*CoordinateManager failed to write ToolInfo config file*/
#define REG_MANAGER_LOG (unsigned long long int)0x0000000100A40000   /*RegManager log{0}*/
#define REG_MANAGER_LOAD_PARAM_FAILED (unsigned long long int)0x0011000B00A40001   /*RegManager load param failed in initialization phase*/
#define REG_MANAGER_LOAD_PR_FAILED (unsigned long long int)0x0011000B00A40002   /*RegManager load PrReg info failed in initialization phase*/
#define REG_MANAGER_LOAD_HR_FAILED (unsigned long long int)0x0011000B00A40003   /*RegManager load HrReg info failed in initialization phase*/
#define REG_MANAGER_LOAD_MR_FAILED (unsigned long long int)0x0011000B00A40004   /*RegManager load MrReg info failed in initialization phase*/
#define REG_MANAGER_LOAD_SR_FAILED (unsigned long long int)0x0011000B00A40005   /*RegManager load SrReg info failed in initialization phase*/
#define REG_MANAGER_LOAD_R_FAILED (unsigned long long int)0x0011000B00A40006   /*RegManager load RReg info failed in initialization phase*/
#define REG_MANAGER_INVALID_ARG (unsigned long long int)0x0001000200A40007   /*RegManager has invalid argument*/
#define REG_MANAGER_REG_FILE_WRITE_FAILED (unsigned long long int)0x0001000200A40008   /*RegManager failed to write reg config file*/
#define REG_MANAGER_INIT_OBJECT_FAILED (unsigned long long int)0x0011000B00A40009   /*RegManager failed to initialize internal variables*/
#define REG_MANAGER_LOAD_NVRAM_FAILED (unsigned long long int)0x0011000B00A4000A   /*RegManager load param failed in initialization phase*/
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
#define CONTROLLER_LOG (unsigned long long int)0x0000000100A80000   /*Controller log{0}*/
#define CONTROLLER_LOAD_PARAM_FAILED (unsigned long long int)0x0011000B00A80001   /*Controller load param failed in initialization phase*/
#define CONTROLLER_INIT_OBJECT_FAILED (unsigned long long int)0x0011000B00A80002   /*Controller failed to initialize internal object*/
#define CONTROLLER_CREATE_ROUTINE_THREAD_FAILED (unsigned long long int)0x0011000B00A80003   /*Controller failed to create routine thread*/
#define CONTROLLER_CREATE_HEARTBEAT_THREAD_FAILED (unsigned long long int)0x0011000B00A80004   /*Controller failed to create heartbeat thread*/
#define CONTROLLER_INVALID_ARG (unsigned long long int)0x0001000200A80005   /*Controller has invalid argument*/
#define CONTROLLER_PUBLISH_EXIST (unsigned long long int)0x0001000200A80006   /*the publishing element of request is exist*/
#define CONTROLLER_INVALID_OPERATION (unsigned long long int)0x0001000200A80007   /*Controller failed to operate command because of invalid pre-condition*/
#define CONTROLLER_UNKNOWN_USER_OP_MODE (unsigned long long int)0x0001000B00A80008   /*User Op Mode is in unknown state*/
#define CONTROLLER_PUBLISH_NONE (unsigned long long int)0x0001000200A80009   /*Controller failed to find the publishing elements of request*/
#define CONTROLLER_INVALID_OPERATION_SET_TIME (unsigned long long int)0x0001000200A8000A   /*Controller failed to set the time.*/
#define CONTROLLER_INVALID_OPERATION_START (unsigned long long int)0x0001000200A8000B   /*Controller failed to start the program.*/
#define CONTROLLER_INVALID_OPERATION_LAUNCH (unsigned long long int)0x0001000200A8000C   /*Controller failed to launch the program.*/
#define CONTROLLER_INVALID_OPERATION_FORWARD (unsigned long long int)0x0001000200A8000D   /*Controller failed to forward the program.*/
#define CONTROLLER_INVALID_OPERATION_BACKWARD (unsigned long long int)0x0001000200A8000E   /*Controller failed to backward the program.*/
#define CONTROLLER_INVALID_OPERATION_JUMP (unsigned long long int)0x0001000200A8000F   /*Controller failed to jump the program.*/
#define CONTROLLER_INVALID_OPERATION_PAUSE (unsigned long long int)0x0001000200A80010   /*Controller failed to pause the program.*/
#define CONTROLLER_INVALID_OPERATION_RESUME (unsigned long long int)0x0001000200A80011   /*Controller failed to resume the program.*/
#define CONTROLLER_INVALID_OPERATION_SET_IO (unsigned long long int)0x0001000200A80012   /*Controller failed to set IO*/
#define CONTROLLER_INVALID_OPERATION_SET_LAUNCH (unsigned long long int)0x0001000200A80013   /*Controller failed to set launching mode*/
#define CONTROLLER_INVALID_OPERATION_SET_VEL (unsigned long long int)0x0001000200A80014   /*Controller failed to set global velocity ratio*/
#define CONTROLLER_INVALID_OPERATION_SET_ACC (unsigned long long int)0x0001000200A80015   /*Controller failed to set global acceleration ratio*/
#define CONTROLLER_INVALID_OPERATION_MOVE_STEP (unsigned long long int)0x0001000200A80016   /*Controller failed to do step manual move*/
#define CONTROLLER_INVALID_OPERATION_MOVE_CONTINUOUS (unsigned long long int)0x0001000200A80017   /*Controller failed to docontinuous manual move*/
#define CONTROLLER_INVALID_OPERATION_GOTO_CARTESIAN (unsigned long long int)0x0001000200A80018   /*Controller failed to goto cartesian point in manual mode*/
#define CONTROLLER_INVALID_OPERATION_GOTO_JOINT (unsigned long long int)0x0001000200A80019   /*Controller failed to goto joint point in manual mode*/
#define CONTROLLER_INVALID_OPERATION_MANUAL_STOP (unsigned long long int)0x0001000200A8001A   /*Controller failed to do manual stop*/
#define CONTROLLER_INVALID_OPERATION_RESET (unsigned long long int)0x0001000200A8001B   /*Controller failed to do reset*/
#define CONTROLLER_OFFSET_NEED_CALIBRATE (unsigned long long int)0x0001000200A8001C   /*Controller needs to calibrate the offset */
#define CONTROLLER_SAFETY_NOT_READY (unsigned long long int)0x0001000200A8001D   /*Controller detects the safety board is not ready when reset operation*/
#define TP_COMM_LOG (unsigned long long int)0x0000000100A70000   /*TpComm log{0}*/
#define TP_COMM_LOAD_PARAM_FAILED (unsigned long long int)0x0011000B00A70001   /*TpComm loading param is failed in initialization phase*/
#define TP_COMM_INIT_OBJECT_FAILED (unsigned long long int)0x0011000B00A70002   /*TpComm failed to initialize internal variables*/
#define TP_COMM_CREATE_ROUTINE_THREAD_FAILED (unsigned long long int)0x0011000B00A70003   /*TpComm failed to create routine thread*/
#define TP_COMM_INVALID_REQUEST (unsigned long long int)0x0001000200A70004   /*TpComm receives a invalid hash for RPC*/
#define TP_COMM_ENCODE_FAILED (unsigned long long int)0x0001000200A70005   /*TpComm failed to encode data to send out*/
#define TP_COMM_DECODE_FAILED (unsigned long long int)0x0001000200A70006   /*TpComm failed to decode data that has been received*/
#define TP_COMM_MEMORY_OPERATION_FAILED (unsigned long long int)0x0001000200A70007   /*TpComm failed to operate memory*/
#define TP_COMM_AUTHORITY_CHECK_FAILED (unsigned long long int)0x0001000200A70008   /*TpComm failed to run a unauthorized operation*/
#define TP_COMM_SEND_FAILED (unsigned long long int)0x0001000200A70009   /*TpComm failed to send a package*/
#define TP_COMM_RECEIVE_FAILED (unsigned long long int)0x0001000200A7000A   /*TpComm failed to receive a package*/
#define TP_COMM_RPC_OVERLOAD (unsigned long long int)0x0000000200A7000B   /*TpComm failed to handle too much rpc request*/
#define PATH_PLANNING_INVALID_TARGET (unsigned long long int)0x0001000400B20001   /*PathTrajAlg detected that the expected target point is invalid*/
#define TRAJ_PLANNING_INVALID_PATHCACHE (unsigned long long int)0x0001000400B20002   /*PathTrajAlg detected a invalid path that was not in a RPC table. */
#define TRAJ_PLANNING_INVALID_MOTION_TYPE (unsigned long long int)0x0001000400B20003   /*PathTrajAlg detected a invalid motion type*/
#define TRAJ_PLANNING_INVALID_SMOOTH_IN_INDEX (unsigned long long int)0x0001000400B20004   /*PathTrajAlg detected that smooth_in index was invalid */
#define TRAJ_PLANNING_INVALID_IK_FAILED (unsigned long long int)0x0001000400B20005   /*PathTrajAlg detected that ik failed*/
#define TRAJ_PLANNING_PAUSE_FAILED (unsigned long long int)0x0001000200B20006   /*PathTrajAlg failed to pause */
#define MODBUS_LOG (unsigned long long int)0x0000000100AE0000   /*ModbusManager log{0}*/
#define MODBUS_START_MODE_ERROR (unsigned long long int)0x0001000200AE0001   /*modbus start mode error*/
#define MODBUS_MANAGER_SAVE_PARAM_FAILED (unsigned long long int)0x0001000200AE0002   /*modbus manager failed to save parameters*/
#define MODBUS_INVALID (unsigned long long int)0x0001000200AE0003   /*modbus is invalid*/
#define MODBUS_SERVER_NOT_OPENED (unsigned long long int)0x0001000200AE0019   /*modbus server is closed*/
#define MODBUS_SERVER_SAVE_PARAM_FALIED (unsigned long long int)0x0001000200AE001A   /*modbus server failed to save parameter*/
#define MODBUS_SERVER_INVALID_ARG (unsigned long long int)0x0001000200AE001C   /*modbus server has invalid parameters*/
#define MODBUS_SERVER_LOAD_PARAM_FALIED (unsigned long long int)0x0001000200AE001D   /*modbus server failed to load parameters*/
#define MODBUS_SERVER_OPEN_FAILED (unsigned long long int)0x0001000200AE001E   /*modbus server failed to open*/
#define MODBUS_SERVER_INIT_FAILED (unsigned long long int)0x0001000200AE001F   /*modbus server initialization failed*/
#define MODBUS_SERVER_ENABLED (unsigned long long int)0x0001000200AE0020   /*modbus server is enabled*/
#define MODBUS_SERVER_DISABLED (unsigned long long int)0x0001000200AE0021   /*modbus server is disabled */
#define MODBUS_SERVER_IS_RUNNING (unsigned long long int)0x0001000200AE0022   /*modbus server is running*/
#define MODBUS_CLIENT_ENABLED (unsigned long long int)0x0001000200AE0028   /*modbus client is enabled*/
#define MODBUS_CLIENT_DISABLED (unsigned long long int)0x0001000200AE0029   /*modbus client is disabled*/
#define MODBUS_CLIENT_CONNECTED (unsigned long long int)0x0001000200AE002A   /*modbus client is connected*/
#define MODBUS_CLIENT_NOT_CONNECT (unsigned long long int)0x0001000200AE002B   /*modbus client is not connected*/
#define MODBUS_CLIENT_INVALID_ARG (unsigned long long int)0x0001000200AE002C   /*modbus client has received a invalid parameter */
#define MODBUS_CLIENT_ENABLE_FAILED (unsigned long long int)0x0001000200AE002D   /*modbus client failed to enable*/
#define MODBUS_CLIENT_INIT_FAILED (unsigned long long int)0x0001000200AE002E   /*modbus client initialization failed*/
#define MODBUS_CLIENT_CONNECT_FAILED (unsigned long long int)0x0001000200AE002F   /*modbus client failed to connect*/
#define MODBUS_CLIENT_OPERATION_FAILED (unsigned long long int)0x0001000200AE0030   /*modbus client failed to read or write data*/
#define MODBUS_CLIENT_MANAGER_LOAD_PARAM_FAILED (unsigned long long int)0x0001000200AE0031   /*modbus client manager failed to load parameter*/
#define MODBUS_CLIENT_MANAGER_SAVE_PARAM_FAILED (unsigned long long int)0x0001000200AE0032   /*modbus client manager failed to save parameter*/
#define MODBUS_CLIENT_ID_EXISTED (unsigned long long int)0x0001000200AE0033   /*modbus client is existing*/
#define MODBUS_CLIENT_ID_NOT_EXISTED (unsigned long long int)0x0001000200AE0034   /*modbus client can't be found existing*/
#define MODBUS_CLIENT_MANAGER_INVALID_ARG (unsigned long long int)0x0001000200AE0035   /*modbus client received a invalid parameter*/
#define MODBUS_CLIENT_NOT_ALL_CLOSED (unsigned long long int)0x0001000200AE0036   /*modbus clients are not all closed*/
#define INTERPRETER_LOG (unsigned long long int)0x0000000100AA0000   /*Interpreter log{0}*/
#define FAIL_INTERPRETER_SYNTAX_ERROR (unsigned long long int)0x0001000900AA0001   /*not a syntax in the program*/
#define FAIL_INTERPRETER_UNBALANCED_PARENTHESES (unsigned long long int)0x0001000900AA0002   /*unbalanced parentheses in the program*/
#define FAIL_INTERPRETER_NO_EXPRESSION_PRESENT (unsigned long long int)0x0001000900AA0003   /*no expression present in the program*/
#define FAIL_INTERPRETER_EQUALS_SIGN_EXPECTED (unsigned long long int)0x0001000900AA0004   /*expected equals sign in the program*/
#define FAIL_INTERPRETER_NOT_A_VARIABLE (unsigned long long int)0x0001000900AA0005   /*not a defined or initialized variable in the program*/
#define FAIL_INTERPRETER_LABEL_TABLE_FULL (unsigned long long int)0x0001000900AA0006   /*defined too much variables in the program*/
#define FAIL_INTERPRETER_DUPLICATE_SUB_LABEL (unsigned long long int)0x0001000900AA0007   /*duplicate function in the program*/
#define FAIL_INTERPRETER_UNDEFINED_SUB_LABEL (unsigned long long int)0x0001000900AA0008   /*undefined function in the program*/
#define FAIL_INTERPRETER_THEN_EXPECTED (unsigned long long int)0x0001000900AA0009   /*THEN expected after IF in the program*/
#define FAIL_INTERPRETER_TO_EXPECTED (unsigned long long int)0x0001000900AA000A   /*TO expected after FOR in the program*/
#define FAIL_INTERPRETER_TOO_MANY_NESTED_FOR_LOOPS (unsigned long long int)0x0001000900AA000B   /*too many nested FOR loops in the program*/
#define FAIL_INTERPRETER_NEXT_WITHOUT_FOR (unsigned long long int)0x0001000900AA000C   /*NEXT expected after WHILE in the program*/
#define FAIL_INTERPRETER_TOO_MANY_NESTED_GOSUB (unsigned long long int)0x0001000900AA000D   /*too many nested GOSUBs*/
#define FAIL_INTERPRETER_RETURN_WITHOUT_GOSUB (unsigned long long int)0x0001000900AA000E   /*Without RETURN in the GOSUB in the program*/
#define FAIL_INTERPRETER_FILE_NOT_FOUND (unsigned long long int)0x0001000900AA000F   /*Program file does not exist*/
#define FAIL_INTERPRETER_MOVL_WITH_JOINT (unsigned long long int)0x0001000900AA0010   /*movl with joint in the program*/
#define FAIL_INTERPRETER_MOVJ_WITH_POINT (unsigned long long int)0x0001000900AA0011   /*movj with point in the program*/
#define FAIL_INTERPRETER_ILLEGAL_LINE_NUMBER (unsigned long long int)0x0001000900AA0012   /*Illegal Line Number*/
#define FAIL_INTERPRETER_FUNC_PARAMS_MISMATCH (unsigned long long int)0x0001000900AA0013   /*Function params mismatch in the program*/
#define FAIL_INTERPRETER_DUPLICATE_EXEC_MACRO (unsigned long long int)0x0001000900AA0014   /*Duplicate Macro is executing*/
#define INFO_INTERPRETER_BACK_TO_BEGIN (unsigned long long int)0x0001000200AA0015   /*reverted executing to begin*/
#define INFO_INTERPRETER_THREAD_NOT_EXIST (unsigned long long int)0x0001000200AA0016   /*Program thread does not exist*/
#define INFO_INTERPRETER_TOO_MANY_IMPORT (unsigned long long int)0x0001000900AA0017   /*Too many import file in the program*/
#define INFO_INTERPRETER_TOO_LONG_PROJECT_NAME (unsigned long long int)0x0001000900AA0018   /*Project name does too long*/
#define INFO_INTERPRETER_ARITHMETIC_EXCEPTION (unsigned long long int)0x0001000900AA0019   /*Arithm Exception(Ex:Division By Zero)*/
#define INFO_INTERPRETER_UNKNOWN_ARITHM (unsigned long long int)0x0001000900AA001A   /*Unknown Arithm Operator in the program*/
#define INFO_INTERPRETER_WAIT_TIMEOUT (unsigned long long int)0x0001000900AA001B   /*WAIT Timeout in the program*/
#define INFO_INTERPRETER_OVERRUN_HOME_POSE (unsigned long long int)0x0001000900AA001C   /*Home Pose of program out of range*/
#define INFO_INTERPRETER_HOME_POSE_NOT_EXIST (unsigned long long int)0x0001000900AA001D   /*Home Pose of program does not exist*/
#define FAIL_INTERPRETER_ALARM_EXEC_BASE (unsigned long long int)0x0000000200AA0100   /*User defined WARNING：User Alarm BASE*/
#define FAIL_INTERPRETER_USER_ALARM1 (unsigned long long int)0x0001000200AA0101   /*User defined WARNING：User Alarm 1*/
#define FAIL_INTERPRETER_USER_ALARM2 (unsigned long long int)0x0001000200AA0102   /*User defined WARNING：User Alarm 2*/
#define FAIL_INTERPRETER_USER_ALARM3 (unsigned long long int)0x0001000200AA0103   /*User defined WARNING：User Alarm 3*/
#define FAIL_INTERPRETER_USER_ALARM4 (unsigned long long int)0x0001000200AA0104   /*User defined WARNING：User Alarm 4*/
#define FAIL_INTERPRETER_USER_ALARM5 (unsigned long long int)0x0001000200AA0105   /*User defined WARNING：User Alarm 5*/
#define FAIL_INTERPRETER_USER_ALARM6 (unsigned long long int)0x0001000200AA0106   /*User defined WARNING：User Alarm 6*/
#define FAIL_INTERPRETER_USER_ALARM7 (unsigned long long int)0x0001000200AA0107   /*User defined WARNING：User Alarm 7*/
#define FAIL_INTERPRETER_USER_ALARM8 (unsigned long long int)0x0001000200AA0108   /*User defined WARNING：User Alarm 8*/
#define FAIL_INTERPRETER_USER_ALARM9 (unsigned long long int)0x0001000200AA0109   /*User defined WARNING：User Alarm 9*/
#define FAIL_INTERPRETER_USER_ALARM10 (unsigned long long int)0x0001000200AA010A   /*User defined WARNING：User Alarm 10*/
#define FAIL_INTERPRETER_NOT_IN_PAUSE (unsigned long long int)0x0001000200AA010B   /*Can not continue to execute while program is not in the PAUSE status*/
#define PARAM_MANAGER_LOG (unsigned long long int)0x0000000100B30000   /*ParamManager log{0}*/
#define PARAM_MANAGER_INIT_FAILED (unsigned long long int)0x0011000200B30001   /*Param Manager load parameters failed*/
#define PARAM_MANAGER_SET_PARAM_FAILED (unsigned long long int)0x0001000200B30002   /*Param Manager set parameter failed*/
#define DEVICE_MANAGER_LOG (unsigned long long int)0x0000000100A50000   /*DeviceManager log{0}*/
#define DEVICE_MANAGER_LOAD_PARAM_FAILED (unsigned long long int)0x0011000B00A50001   /*DeviceManager load param failed in initialization phase*/
#define DEVICE_MANAGER_LOAD_DEVICE_CONFIG_FAILED (unsigned long long int)0x0011000B00A50002   /*DeviceManager load device config failed in initialization phase*/
#define DEVICE_MANAGER_INVALID_DEVICE_TYPE (unsigned long long int)0x0011000B00A50003   /*DeviceManager load invalid type of device from device config file in initialization phase*/
#define DEVICE_MANAGER_INIT_DEVICE_FAILED (unsigned long long int)0x0011000B00A50004   /*DeviceManager failed to init device according to device config file*/
#define DEVICE_MANAGER_DEVICE_ALREADY_EXIST (unsigned long long int)0x0001000200A50005   /*DeviceManager failed to add device because the device has already been exist*/
#define DEVICE_MANAGER_INVALID_ARG (unsigned long long int)0x0001000200A50006   /*DeviceManager has invalid argument*/

#endif

