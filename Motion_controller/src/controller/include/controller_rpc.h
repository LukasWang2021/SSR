#ifndef CONTROLLER_RPC_H
#define CONTROLLER_RPC_H

/**
 * @file controller_rpc.h
 * @brief The file is the header file of class "ControllerRpc".
 * @author Feng.Wu
 */

#include <vector>
#include "tp_comm.h"
#include "protoc.h"
#include "file_manager.h"
#include "device_version.h"
#include "controller_publish.h"
#include "log_manager_producer.h"
#include "system/core_comm_system.h"
#include "system/servo_comm_base.h"
#include "system/servo_cpu_comm_base.h"
#include "axis.h"
#include "io_1000.h"
#include "motion_control.h"
#include "tool_manager.h"
#include "coordinate_manager.h"
#include "reg_manager.h"

/**
 * @brief user_space includes the user level implementation.
 */
namespace user_space
{

/**
 * @brief ControllerRpc deals with the Remote Procedure Call.
 * @details 
 */
class ControllerRpc
{
public:
    /**
     * @brief Constructor of the class.
     */
    ControllerRpc();
    /**
     * @brief Destructor of the class. 
     */  
    ~ControllerRpc();

    /**
     * @brief Initialization.
     * @details
     * @param [in] tp_comm_ptr The pointer of the communication with the upper computer.
     * @param [in] publish_ptr The pointer of the ControllerPublish.
     * @param [in] cpu_comm_ptr The pointer to communicate with the other cpu.
     * @param [in] servo_comm_ptr The pointer to communicate with servos.
     * @param [in] axis_ptr The pointer of all the axes.
     * @param [in] group_ptr The pointer of all the groups.
     * @param [in] file_manager_ptr The pointer of the file manager.
     * @return void.
     */
    void init(TpComm* tp_comm_ptr, ControllerPublish* publish_ptr, servo_comm_space::ServoCpuCommBase* cpu_comm_ptr, 
        servo_comm_space::ServoCommBase* servo_comm_ptr[], axis_space::Axis* axis_ptr[AXIS_NUM],
        system_model_space::AxisModel_t* axis_model_ptr[AXIS_NUM], group_space::MotionControl* group_ptr[GROUP_NUM],
        base_space::FileManager* file_manager_ptr, hal_space::Io1000* io_dev_ptr,
        fst_ctrl::ToolManager* tool_manager_ptr, fst_ctrl::CoordinateManager* coordinate_manager_ptr, fst_ctrl::RegManager* reg_manager_ptr,
        system_model_space::ForceModel_t* force_model_ptr);

    /**
     * @brief Process the service request in case the rpc comes.
     * @details It should be cyclically called by Controller.
     * @return void.
     */ 
    void processRpc();

private:
    user_space::TpComm* tp_comm_ptr_;
	user_space::ControllerPublish* publish_ptr_;
    servo_comm_space::ServoCpuCommBase* cpu_comm_ptr_;
    servo_comm_space::ServoCommBase** servo_comm_ptr_;
    system_model_space::AxisModel_t* axis_model_ptr_[AXIS_NUM];
    system_model_space::ForceModel_t* force_model_ptr_;
    axis_space::Axis* axis_ptr_[AXIS_NUM];
    base_space::FileManager* file_manager_ptr_;
    int32_t* sync_ack_ptr_;
    hal_space::Io1000* io_dev_ptr_;
    DeviceVersion device_version_;
    fst_ctrl::ToolManager* tool_manager_ptr_;
    fst_ctrl::CoordinateManager* coordinate_manager_ptr_;
    fst_ctrl::RegManager* reg_manager_ptr_;

    group_space::MotionControl* group_ptr_[GROUP_NUM];

    enum {HASH_BYTE_SIZE = 4,};
    enum {QUICK_SEARCH_TABLE_SIZE = 128,};

    typedef void (ControllerRpc::*HandleRpcFuncPtr)(void* request_data_ptr, void* response_data_ptr);

    typedef struct
    {
        std::string path;
        unsigned int hash;
        HandleRpcFuncPtr rpc_func_ptr;
    }RpcService;
    std::vector<RpcService> rpc_table_;
    std::vector<RpcService> rpc_quick_search_table_[QUICK_SEARCH_TABLE_SIZE]; 

    void initRpcTable();
    void initRpcQuickSearchTable();
    HandleRpcFuncPtr getRpcHandlerByHash(unsigned int hash);
    void recordLog(ErrorCode log_code, ErrorCode error_code, std::string rpc_path);

    //"/rpc/publish/addTopic"	
    void handleRpc0x000050E3(void* request_data_ptr, void* response_data_ptr);
    //"/rpc/publish/deleteTopic"	
    void handleRpc0x00004403(void* request_data_ptr, void* response_data_ptr);
        
    //"/rpc/tp_comm/getRpcTable"	
    void handleRpc0x00004FA5(void* request_data_ptr, void* response_data_ptr);
    //"/rpc/tp_comm/getPublishTable"	
    void handleRpc0x000147A5(void* request_data_ptr, void* response_data_ptr);

    //"/rpc/file_manager/readFile"	
    void handleRpc0x0000A545(void* request_data_ptr, void* response_data_ptr);
    //"/rpc/file_manager/writeFile"	
    void handleRpc0x00010D95(void* request_data_ptr, void* response_data_ptr);
        
    //"/rpc/controller/getVersion"	
    void handleRpc0x000093EE(void* request_data_ptr, void* response_data_ptr);
    //"/rpc/controller/setSystemTime"	
    void handleRpc0x000167C5(void* request_data_ptr, void* response_data_ptr);
    //"/rpc/controller/getSystemTime"	
    void handleRpc0x000003F5(void* request_data_ptr, void* response_data_ptr);
    //"/rpc/controller/setWorkMode"	
    void handleRpc0x00006825(void* request_data_ptr, void* response_data_ptr);
    //"/rpc/controller/getWorkMode"	
    void handleRpc0x00003325(void* request_data_ptr, void* response_data_ptr);
    //"/rpc/controller/setControlMode"	
    void handleRpc0x0000B555(void* request_data_ptr, void* response_data_ptr);
    //"/rpc/controller/getControlMode"	
    void handleRpc0x0000B695(void* request_data_ptr, void* response_data_ptr);

    //"/rpc/axis/mcPower"	
    void handleRpc0x000053E2(void* request_data_ptr, void* response_data_ptr);
    //"/rpc/axis/mcReset"	
    void handleRpc0x000180C4(void* request_data_ptr, void* response_data_ptr);
    //"/rpc/axis/mcStop"	
    void handleRpc0x00002820(void* request_data_ptr, void* response_data_ptr);
    //"/rpc/axis/mcHalt"	
    void handleRpc0x00004BB4(void* request_data_ptr, void* response_data_ptr);
    //"/rpc/axis/mcSetPosition"	
    void handleRpc0x0001798E(void* request_data_ptr, void* response_data_ptr);
    //"/rpc/axis/mcReadParameter"	
    void handleRpc0x00016BF2(void* request_data_ptr, void* response_data_ptr);
    //"/rpc/axis/mcWriteParameter"	
    void handleRpc0x00005732(void* request_data_ptr, void* response_data_ptr);
    //"/rpc/axis/mcMoveAbsolute"	
    void handleRpc0x000051F5(void* request_data_ptr, void* response_data_ptr);
    //"/rpc/axis/mcMoveVelocity"	
    void handleRpc0x00016CF9(void* request_data_ptr, void* response_data_ptr);
    //"/rpc/axis/mcReadActualPosition"	
    void handleRpc0x000012BE(void* request_data_ptr, void* response_data_ptr);
    //"/rpc/axis/mcReadActualVelocity"	
    void handleRpc0x00002EA9(void* request_data_ptr, void* response_data_ptr);
    //"/rpc/axis/mcReadActualTorque"	
    void handleRpc0x00014265(void* request_data_ptr, void* response_data_ptr);
    //"/rpc/axis/mcReadAxisInfo"	
    void handleRpc0x0000314F(void* request_data_ptr, void* response_data_ptr);
    //"/rpc/axis/mcReadStatus"	
    void handleRpc0x00003E53(void* request_data_ptr, void* response_data_ptr);
    //"/rpc/axis/mcReadAxisError"	
    void handleRpc0x000063C2(void* request_data_ptr, void* response_data_ptr);
    //"/rpc/axis/mcReadAxisErrorHistory"	
    void handleRpc0x00018469(void* request_data_ptr, void* response_data_ptr);
    //"/rpc/axis/mcMoveRelative"	
    void handleRpc0x0000CC85(void* request_data_ptr, void* response_data_ptr);
    //"/rpc/axis/mcHome"	
    void handleRpc0x000059B5(void* request_data_ptr, void* response_data_ptr);
    //"/rpc/axis/rtmAbortHoming"	
    void handleRpc0x0000E4B7(void* request_data_ptr, void* response_data_ptr);
    //"/rpc/axis/rtmReadAxisFdbPdoPtr"	
    void handleRpc0x0000A632(void* request_data_ptr, void* response_data_ptr);
    //"/rpc/axis/rtmResetEncoder"	
    void handleRpc0x00000BA2(void* request_data_ptr, void* response_data_ptr);

    //"/rpc/group/mcGroupReset"	
    void handleRpc0x00016FF4(void* request_data_ptr, void* response_data_ptr);
    //"/rpc/group/mcGroupEnable"	
    void handleRpc0x00003615(void* request_data_ptr, void* response_data_ptr);
    //"/rpc/group/mcGroupDisable"	
    void handleRpc0x0000D185(void* request_data_ptr, void* response_data_ptr);
    //"/rpc/group/mcGroupReadError"	
    void handleRpc0x00004BE2(void* request_data_ptr, void* response_data_ptr);
    //"/rpc/group/mcGroupReadStatus"	
    void handleRpc0x00002A83(void* request_data_ptr, void* response_data_ptr);
    //"/rpc/group/resetAllEncoder"	
    void handleRpc0x000019D2(void* request_data_ptr, void* response_data_ptr);

    //"/rpc/servo_sampling/setSamplingConfiguration"	
    void handleRpc0x0000845E(void* request_data_ptr, void* response_data_ptr);
    //"/rpc/servo_sampling/getSamplingConfiguration"	
    void handleRpc0x000106EE(void* request_data_ptr, void* response_data_ptr);
    //"/rpc/servo_sampling/activateSamplingConfiguration"	
    void handleRpc0x0000CDDE(void* request_data_ptr, void* response_data_ptr);
    //"/rpc/servo_sampling/setSamplingSync"	
    void handleRpc0x00003743(void* request_data_ptr, void* response_data_ptr);
    //"/rpc/servo_sampling/getSamplingSync"	
    void handleRpc0x00006343(void* request_data_ptr, void* response_data_ptr);
    //"/rpc/servo_sampling/setSamplingChannel"	
    void handleRpc0x0000BACC(void* request_data_ptr, void* response_data_ptr);
    //"/rpc/servo_sampling/getSamplingChannel"	
    void handleRpc0x0000556C(void* request_data_ptr, void* response_data_ptr);
    //"/rpc/servo_sampling/saveSamplingBufferData"	
    void handleRpc0x00004E41(void* request_data_ptr, void* response_data_ptr);

    //"/rpc/servo1001/servo/shutDown"	
    void handleRpc0x0000863E(void* request_data_ptr, void* response_data_ptr);
    //"/rpc/servo1001/servo/switchOn"	
    void handleRpc0x0000E5CE(void* request_data_ptr, void* response_data_ptr);
    //"/rpc/servo1001/servo/disableVoltage"	
    void handleRpc0x00004755(void* request_data_ptr, void* response_data_ptr);
    //"/rpc/servo1001/servo/enableOperation"	
    void handleRpc0x0000313E(void* request_data_ptr, void* response_data_ptr);
    //"/rpc/servo1001/servo/switchOnAndEnableOperation"	
    void handleRpc0x000177CE(void* request_data_ptr, void* response_data_ptr);
    //"/rpc/servo1001/servo/disableOperation"	
    void handleRpc0x000026AE(void* request_data_ptr, void* response_data_ptr);
    //"/rpc/servo1001/servo/quickStop"	
    void handleRpc0x00000580(void* request_data_ptr, void* response_data_ptr);
    //"/rpc/servo1001/servo/resetFault"	
    void handleRpc0x00010584(void* request_data_ptr, void* response_data_ptr);
    //"/rpc/servo1001/servo/transCommState"	
    void handleRpc0x000153C5(void* request_data_ptr, void* response_data_ptr);
    //"/rpc/servo1001/servo/readParameter"	
    void handleRpc0x00006892(void* request_data_ptr, void* response_data_ptr);
    //"/rpc/servo1001/servo/writeParameter"	
    void handleRpc0x00007C32(void* request_data_ptr, void* response_data_ptr);
    //"/rpc/servo1001/servo/moveVelocity"	
    void handleRpc0x000164D9(void* request_data_ptr, void* response_data_ptr);
    //"/rpc/servo1001/servo/moveAbsolute"	
    void handleRpc0x00004DD5(void* request_data_ptr, void* response_data_ptr);
    //"/rpc/servo1001/servo/triggerUploadParameters"	
    void handleRpc0x000020B3(void* request_data_ptr, void* response_data_ptr);
    //"/rpc/servo1001/servo/uploadParameters"	
    void handleRpc0x0000E003(void* request_data_ptr, void* response_data_ptr);
    //"/rpc/servo1001/servo/triggerDownloadParameters"	
    void handleRpc0x00011C53(void* request_data_ptr, void* response_data_ptr);
    //"/rpc/servo1001/servo/downloadParameters"	
    void handleRpc0x00017063(void* request_data_ptr, void* response_data_ptr);
    //"/rpc/servo1001/servo/isAsyncServiceFinish"	
    void handleRpc0x000043B8(void* request_data_ptr, void* response_data_ptr);
    //"/rpc/servo1001/servo/getCommState"	
    void handleRpc0x0000F485(void* request_data_ptr, void* response_data_ptr);
    //"/rpc/servo1001/servo/getServoState"	
    void handleRpc0x000032F5(void* request_data_ptr, void* response_data_ptr);
    //"/rpc/servo1001/servo/mcMoveRelative"	
    void handleRpc0x000172C5(void* request_data_ptr, void* response_data_ptr);
    //"/rpc/servo1001/servo/resetEncoder"	
    void handleRpc0x0000EFE2(void* request_data_ptr, void* response_data_ptr);
    //"/rpc/servo1001/servo/goHome"	
    void handleRpc0x00013BB5(void* request_data_ptr, void* response_data_ptr);
    //"/rpc/servo1001/servo/abortHoming"	
    void handleRpc0x00015AB7(void* request_data_ptr, void* response_data_ptr);
    //"/rpc/servo1001/servo/getServoCommInfo"	
    void handleRpc0x0000BF1F(void* request_data_ptr, void* response_data_ptr);
    //"/rpc/servo1001/servo/getServoDefinedInfo"	
    void handleRpc0x0000C87F(void* request_data_ptr, void* response_data_ptr);

    //"/rpc/servo1001/cpu/getVersion"	
    void handleRpc0x0001192E(void* request_data_ptr, void* response_data_ptr);
    //"/rpc/servo1001/cpu/setCtrlPdoSync"	
    void handleRpc0x00005123(void* request_data_ptr, void* response_data_ptr);
    //"/rpc/servo1001/cpu/getCtrlPdoSync"	
    void handleRpc0x00005463(void* request_data_ptr, void* response_data_ptr);
    //"/rpc/servo1001/cpu/setSamplingSync"	
    void handleRpc0x00004023(void* request_data_ptr, void* response_data_ptr);
    //"/rpc/servo1001/cpu/getSamplingSync"	
    void handleRpc0x00006C23(void* request_data_ptr, void* response_data_ptr);
    //"/rpc/servo1001/cpu/setSamplingInterval"	
    void handleRpc0x00003EEC(void* request_data_ptr, void* response_data_ptr);
    //"/rpc/servo1001/cpu/getSamplingInterval"	
    void handleRpc0x00001C2C(void* request_data_ptr, void* response_data_ptr);
    //"/rpc/servo1001/cpu/setSamplingMaxTimes"	
    void handleRpc0x000110A3(void* request_data_ptr, void* response_data_ptr);
    //"/rpc/servo1001/cpu/getSamplingMaxTimes"	
    void handleRpc0x00013363(void* request_data_ptr, void* response_data_ptr);
    //"/rpc/servo1001/cpu/setSamplingChannel"	
    void handleRpc0x00008E5C(void* request_data_ptr, void* response_data_ptr);
    //"/rpc/servo1001/cpu/getSamplingChannel"	
    void handleRpc0x0000FD9C(void* request_data_ptr, void* response_data_ptr);
    //"/rpc/servo1001/cpu/activateSamplingConfiguration"	
    void handleRpc0x0000939E(void* request_data_ptr, void* response_data_ptr);
    //"/rpc/servo1001/cpu/saveSamplingBufferData"	
    void handleRpc0x00015621(void* request_data_ptr, void* response_data_ptr);
    //"/rpc/servo1001/cpu/getServoCpuCommInfo"	
    void handleRpc0x0000FE5F(void* request_data_ptr, void* response_data_ptr);
    //"/rpc/servo1001/cpu/setForceControlParameters"	
    void handleRpc0x00005F53(void* request_data_ptr, void* response_data_ptr);
    //"/rpc/servo1001/cpu/getForceControlParameters"	
    void handleRpc0x00008203(void* request_data_ptr, void* response_data_ptr);


    //"/rpc/io/readDI"	
    void handleRpc0x000185A9(void* request_data_ptr, void* response_data_ptr);
    //"/rpc/io/readDO"	
    void handleRpc0x000185AF(void* request_data_ptr, void* response_data_ptr);
    //"/rpc/io/writeDO"	
    void handleRpc0x00000C1F(void* request_data_ptr, void* response_data_ptr);
    
    //"/rpc/tool_manager/addTool"	
    void handleRpc0x0000A22C(void* request_data_ptr, void* response_data_ptr);
    //"/rpc/tool_manager/deleteTool"	
    void handleRpc0x00010E4C(void* request_data_ptr, void* response_data_ptr);
    //"/rpc/tool_manager/updateTool"	
    void handleRpc0x0000C78C(void* request_data_ptr, void* response_data_ptr);
    //"/rpc/tool_manager/moveTool"	
    void handleRpc0x000085FC(void* request_data_ptr, void* response_data_ptr);
    //"/rpc/tool_manager/getToolInfoById"	
    void handleRpc0x00009E34(void* request_data_ptr, void* response_data_ptr);
    //"/rpc/tool_manager/getAllValidToolSummaryInfo"	
    void handleRpc0x0001104F(void* request_data_ptr, void* response_data_ptr);

    //"/rpc/coordinate_manager/addUserCoord"	
    void handleRpc0x00016764(void* request_data_ptr, void* response_data_ptr);
    //"/rpc/coordinate_manager/deleteUserCoord"	
    void handleRpc0x0000BAF4(void* request_data_ptr, void* response_data_ptr);
    //"/rpc/coordinate_manager/updateUserCoord"	
    void handleRpc0x0000EC14(void* request_data_ptr, void* response_data_ptr);
    //"/rpc/coordinate_manager/moveUserCoord"	
    void handleRpc0x0000E104(void* request_data_ptr, void* response_data_ptr);
    //"/rpc/coordinate_manager/getUserCoordInfoById"	
    void handleRpc0x00004324(void* request_data_ptr, void* response_data_ptr);
    //"/rpc/coordinate_manager/getAllValidUserCoordSummaryInfo"	
    void handleRpc0x0001838F(void* request_data_ptr, void* response_data_ptr);

    //"/rpc/motion_control/setGlobalVelRatio"	
    void handleRpc0x000005EF(void* request_data_ptr, void* response_data_ptr);
    //"/rpc/motion_control/getGlobalVelRatio"	
    void handleRpc0x0001578F(void* request_data_ptr, void* response_data_ptr);
    //"/rpc/motion_control/setGlobalAccRatio"	
    void handleRpc0x0000271F(void* request_data_ptr, void* response_data_ptr);
    //"/rpc/motion_control/getGlobalAccRatio"	
    void handleRpc0x00016D9F(void* request_data_ptr, void* response_data_ptr);
    //"/rpc/motion_control/axis_group/doStepManualMove"	
    void handleRpc0x000085D5(void* request_data_ptr, void* response_data_ptr);
    //"/rpc/motion_control/axis_group/doContinuousManualMove"	
    void handleRpc0x0000D3F5(void* request_data_ptr, void* response_data_ptr);
    //"/rpc/motion_control/axis_group/doGotoCartesianPointManualMove"	
    void handleRpc0x00010C05(void* request_data_ptr, void* response_data_ptr);
    //"/rpc/motion_control/axis_group/doGotoJointPointManualMove"	
    void handleRpc0x00008075(void* request_data_ptr, void* response_data_ptr);
    //"/rpc/motion_control/axis_group/doManualStop"	
    void handleRpc0x0000A9A0(void* request_data_ptr, void* response_data_ptr);
    //"/rpc/motion_control/axis_group/getJointsFeedBack"	
    void handleRpc0x0000DFBB(void* request_data_ptr, void* response_data_ptr);
    //"/rpc/motion_control/axis_group/setUserSoftLimit"	
    void handleRpc0x000114A4(void* request_data_ptr, void* response_data_ptr);
    //"/rpc/motion_control/axis_group/getUserSoftLimit"	
    void handleRpc0x0000C764(void* request_data_ptr, void* response_data_ptr);
    //"/rpc/motion_control/axis_group/setManuSoftLimit"	
    void handleRpc0x000108E4(void* request_data_ptr, void* response_data_ptr);
    //"/rpc/motion_control/axis_group/getManuSoftLimit"	
    void handleRpc0x0000C244(void* request_data_ptr, void* response_data_ptr);
    //"/rpc/motion_control/axis_group/setHardLimit"	
    void handleRpc0x0000C454(void* request_data_ptr, void* response_data_ptr);
    //"/rpc/motion_control/axis_group/getHardLimit"	
    void handleRpc0x00013394(void* request_data_ptr, void* response_data_ptr);
    //"/rpc/motion_control/axis_group/setCoordinate"	
    void handleRpc0x0000A845(void* request_data_ptr, void* response_data_ptr);
    //"/rpc/motion_control/axis_group/getCoordinate"	
    void handleRpc0x00008595(void* request_data_ptr, void* response_data_ptr);
    //"/rpc/motion_control/axis_group/setUserCoordId"	
    void handleRpc0x00005CF4(void* request_data_ptr, void* response_data_ptr);
    //"/rpc/motion_control/axis_group/getUserCoordId"	
    void handleRpc0x00005BB4(void* request_data_ptr, void* response_data_ptr);
    //"/rpc/motion_control/axis_group/setTool"	
    void handleRpc0x0001581C(void* request_data_ptr, void* response_data_ptr);
    //"/rpc/motion_control/axis_group/getTool"	
    void handleRpc0x0001354C(void* request_data_ptr, void* response_data_ptr);
    //"/rpc/motion_control/axis_group/convertCartToJoint"	
    void handleRpc0x00010FD4(void* request_data_ptr, void* response_data_ptr);
    //"/rpc/motion_control/axis_group/convertJointToCart"	
    void handleRpc0x0000B6D4(void* request_data_ptr, void* response_data_ptr);
    //"/rpc/motion_control/axis_group/ignoreLostZeroError"	
    void handleRpc0x00014952(void* request_data_ptr, void* response_data_ptr);
    //"/rpc/motion_control/axis_group/setSingleZeroPointOffset"	
    void handleRpc0x00012404(void* request_data_ptr, void* response_data_ptr);
    //"/rpc/motion_control/axis_group/setAllZeroPointOffsets"	
    void handleRpc0x00011853(void* request_data_ptr, void* response_data_ptr);
    //"/rpc/motion_control/axis_group/getAllZeroPointOffsets"	
    void handleRpc0x00012353(void* request_data_ptr, void* response_data_ptr);
    //"/rpc/motion_control/axis_group/getAllZeroErrorMaskStatus"	
    void handleRpc0x0000C183(void* request_data_ptr, void* response_data_ptr);
    //"/rpc/motion_control/axis_group/setSingleZeroPointStatus"	
    void handleRpc0x00010E43(void* request_data_ptr, void* response_data_ptr);
    //"/rpc/motion_control/axis_group/getAllZeroPointStatus"	
    void handleRpc0x000102F3(void* request_data_ptr, void* response_data_ptr);
    //"/rpc/motion_control/axis_group/calibrateAllZeroPointOffsets"	
    void handleRpc0x00011B03(void* request_data_ptr, void* response_data_ptr);
    //"/rpc/motion_control/axis_group/calibrateSingleZeroPointOffset"	
    void handleRpc0x000131D4(void* request_data_ptr, void* response_data_ptr);
    //"/rpc/motion_control/axis_group/calibrateZeroPointOffsets"	
    void handleRpc0x00005AE3(void* request_data_ptr, void* response_data_ptr);
    //"/rpc/motion_control/axis_group/setJointManualStep"	
    void handleRpc0x00018470(void* request_data_ptr, void* response_data_ptr);
    //"/rpc/motion_control/axis_group/getJointManualStep"	
    void handleRpc0x00006D10(void* request_data_ptr, void* response_data_ptr);
    //"/rpc/motion_control/axis_group/setCartesianManualStep"	
    void handleRpc0x0000A420(void* request_data_ptr, void* response_data_ptr);
    //"/rpc/motion_control/axis_group/getCartesianManualStep"	
    void handleRpc0x0000EAC0(void* request_data_ptr, void* response_data_ptr);
    //"/rpc/motion_control/axis_group/setOrientationManualStep"	
    void handleRpc0x00002940(void* request_data_ptr, void* response_data_ptr);
    //"/rpc/motion_control/axis_group/getOrientationManualStep"	
    void handleRpc0x00016D20(void* request_data_ptr, void* response_data_ptr);
    //"/rpc/motion_control/axis_group/getFcpBasePose"	
    void handleRpc0x000016B5(void* request_data_ptr, void* response_data_ptr);
    //"/rpc/motion_control/axis_group/getTcpCurrentPose"	
    void handleRpc0x00003B45(void* request_data_ptr, void* response_data_ptr);
    //"/rpc/motion_control/getPostureByJoint"	
    void handleRpc0x0000EC64(void* request_data_ptr, void* response_data_ptr);
    //"/rpc/motion_control/axis_group/setOfflineTrajectoryFile"	
    void handleRpc0x00011275(void* request_data_ptr, void* response_data_ptr);
    //"/rpc/motion_control/axis_group/PrepareOfflineTrajectory"	
    void handleRpc0x000051E9(void* request_data_ptr, void* response_data_ptr);
    //"/rpc/motion_control/axis_group/moveOfflineTrajectory"	
    void handleRpc0x0000C4D9(void* request_data_ptr, void* response_data_ptr);

    //"/rpc/reg_manager/pr/addReg"	
    void handleRpc0x000154E7(void* request_data_ptr, void* response_data_ptr);
    //"/rpc/reg_manager/pr/deleteReg"	
    void handleRpc0x00001097(void* request_data_ptr, void* response_data_ptr);
    //"/rpc/reg_manager/pr/updateReg"	
    void handleRpc0x00009EF7(void* request_data_ptr, void* response_data_ptr);
    //"/rpc/reg_manager/pr/getReg"	
    void handleRpc0x00017207(void* request_data_ptr, void* response_data_ptr);
    //"/rpc/reg_manager/pr/moveReg"	
    void handleRpc0x0000D7C7(void* request_data_ptr, void* response_data_ptr);
    //"/rpc/reg_manager/pr/getChangedList"	
    void handleRpc0x0000B454(void* request_data_ptr, void* response_data_ptr);
    //"/rpc/reg_manager/pr/getValidList"	
    void handleRpc0x00009354(void* request_data_ptr, void* response_data_ptr);


};

}


#endif

