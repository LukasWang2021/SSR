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
        system_model_space::AxisModel_t* axis_model_ptr[AXIS_NUM], 
        base_space::FileManager* file_manager_ptr,
        hal_space::Io1000* io_dev_ptr);

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
    axis_space::Axis* axis_ptr_[AXIS_NUM];
    base_space::FileManager* file_manager_ptr_;
    int32_t* sync_ack_ptr_;
    hal_space::Io1000* io_dev_ptr_;
    DeviceVersion device_version_;

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

    //"/rpc/io/readDI"	
    void handleRpc0x000185A9(void* request_data_ptr, void* response_data_ptr);
    //"/rpc/io/readDO"	
    void handleRpc0x000185AF(void* request_data_ptr, void* response_data_ptr);
    //"/rpc/io/writeDO"	
    void handleRpc0x00000C1F(void* request_data_ptr, void* response_data_ptr);
    

};

}


#endif

