#ifndef CONTROLLER_RPC_H
#define CONTROLLER_RPC_H

#include "controller_param.h"
#include "common_log.h"
#include "virtual_core1.h"
#include "tp_comm.h"
#include "controller_sm.h"
#include "controller_publish.h"
#include "tool_manager.h"
#include "coordinate_manager.h"
#include "device_manager.h"
#include "reg_manager.h"
#include "process_comm.h"
#include "motion_control.h"
#include "serverAlarmApi.h"
#include "io_mapping.h"
#include "program_launching.h"
#include "file_manager.h"
#include "device_version.h"
#include <vector>

namespace fst_ctrl
{
class ControllerRpc
{
public:
    ControllerRpc();
    ~ControllerRpc();

    void init(fst_log::Logger* log_ptr, ControllerParam* param_ptr, ControllerPublish* publish_ptr, VirtualCore1* virtual_core1_ptr, 
                    fst_comm::TpComm* tp_comm_ptr, ControllerSm* state_machine_ptr, ToolManager* tool_manager_ptr, 
                    CoordinateManager* coordinate_manager_ptr, RegManager* reg_manager_ptr, fst_hal::DeviceManager* device_manager_ptr, 
                    fst_mc::MotionControl* motion_control_ptr, fst_base::ControllerClient* controller_client_ptr,
                    IoMapping* io_mapping_ptr, fst_hal::IoManager* io_manager_ptr, ProgramLaunching* program_launching, 
                    fst_base::FileManager* file_manager);

    void processRpc();

private:
    fst_log::Logger* log_ptr_;
    ControllerParam* param_ptr_;
    VirtualCore1* virtual_core1_ptr_;
    fst_comm::TpComm* tp_comm_ptr_;
    ControllerSm* state_machine_ptr_;
    ToolManager* tool_manager_ptr_;
    CoordinateManager* coordinate_manager_ptr_;
    RegManager* reg_manager_ptr_;
    fst_hal::DeviceManager* device_manager_ptr_;
    fst_mc::MotionControl* motion_control_ptr_;
    fst_base::ControllerClient* controller_client_ptr_;
    ControllerPublish* publish_ptr_;
    IoMapping* io_mapping_ptr_; 
    IoManager* io_manager_ptr_;
    fst_hal::ModbusManager* modbus_manager_ptr_; 
    fst_hal::FstSafetyDevice* safety_device_ptr_;
    ProgramLaunching* program_launching_;
    fst_base::FileManager* file_manager_ptr_;

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

    /* publish rpc */
    // "/rpc/publish/addTopic"
    void handleRpc0x000050E3(void* request_data_ptr, void* response_data_ptr);
    // "/rpc/publish/addRegTopic"
    void handleRpc0x000163A3(void* request_data_ptr, void* response_data_ptr);
    // "/rpc/publish/addIoTopic"
    void handleRpc0x000058F3(void* request_data_ptr, void* response_data_ptr);
    // "/rpc/publish/deleteTopic"
    void handleRpc0x00004403(void* request_data_ptr, void* response_data_ptr);
    // "/rpc/publish/deleteRegTopic"
    void handleRpc0x00010353(void* request_data_ptr, void* response_data_ptr);
    // "/rpc/publish/deleteIoTopic"
    void handleRpc0x0000DD03(void* request_data_ptr, void* response_data_ptr);   

    /* controller rpc */
    // "/rpc/controller/getUserOpMode"
    void handleRpc0x00000C05(void* request_data_ptr, void* response_data_ptr);
    // "/rpc/controller/getRunningStatus"
    void handleRpc0x00000AB3(void* request_data_ptr, void* response_data_ptr);
    // "/rpc/controller/getInterpreterStatus"
    void handleRpc0x00016483(void* request_data_ptr, void* response_data_ptr);
    // "/rpc/controller/getRobotStatus"
    void handleRpc0x00006F83(void* request_data_ptr, void* response_data_ptr);
    // "/rpc/controller/getCtrlStatus"
    void handleRpc0x0000E9D3(void* request_data_ptr, void* response_data_ptr);
    // "/rpc/controller/getServoStatus"
    void handleRpc0x0000D113(void* request_data_ptr, void* response_data_ptr);
    // "/rpc/controller/getSafetyAlarm"
    void handleRpc0x0000C00D(void* request_data_ptr, void* response_data_ptr);
    // "/rpc/controller/callEstop"
    void handleRpc0x00013940(void* request_data_ptr, void* response_data_ptr);
    // "/rpc/controller/callReset"
    void handleRpc0x000161E4(void* request_data_ptr, void* response_data_ptr);
    // "/rpc/controller/setUserOpMode"
    void handleRpc0x00002ED5(void* request_data_ptr, void* response_data_ptr);
    // "/rpc/controller/shutdown"
    void handleRpc0x0000899E(void* request_data_ptr, void* response_data_ptr);
    // "/rpc/controller/setSystemTime"
    void handleRpc0x000167C5(void* request_data_ptr, void* response_data_ptr);
    // "/rpc/controller/getSystemTime"
    void handleRpc0x000003F5(void* request_data_ptr, void* response_data_ptr);
    // "/rpc/controller/getVersion"
    void handleRpc0x000093EE(void* request_data_ptr, void* response_data_ptr);
    // "/rpc/controller/getErrorCodeList"
    void handleRpc0x00015F44(void* request_data_ptr, void* response_data_ptr);

    /* tool manager rpc */
    // "/rpc/tool_manager/addTool"
    void handleRpc0x0000A22C(void* request_data_ptr, void* response_data_ptr);
    // "/rpc/tool_manager/deleteTool"
    void handleRpc0x00010E4C(void* request_data_ptr, void* response_data_ptr);
    // "/rpc/tool_manager/updateTool"
    void handleRpc0x0000C78C(void* request_data_ptr, void* response_data_ptr);
    // "/rpc/tool_manager/moveTool"
    void handleRpc0x000085FC(void* request_data_ptr, void* response_data_ptr);
    // "/rpc/tool_manager/getToolInfoById"
    void handleRpc0x00009E34(void* request_data_ptr, void* response_data_ptr);
    // "/rpc/tool_manager/getAllValidToolSummaryInfo"
    void handleRpc0x0001104F(void* request_data_ptr, void* response_data_ptr);

    /* coordinate manager rpc */
    // "/rpc/coordinate_manager/addUserCoord"
    void handleRpc0x00016764(void* request_data_ptr, void* response_data_ptr);
    // "/rpc/coordinate_manager/deleteUserCoord"
    void handleRpc0x0000BAF4(void* request_data_ptr, void* response_data_ptr);
    // "/rpc/coordinate_manager/updateUserCoord"
    void handleRpc0x0000EC14(void* request_data_ptr, void* response_data_ptr);
    // "/rpc/coordinate_manager/moveUserCoord"
    void handleRpc0x0000E104(void* request_data_ptr, void* response_data_ptr);
    // "/rpc/coordinate_manager/getUserCoordInfoById"
    void handleRpc0x00004324(void* request_data_ptr, void* response_data_ptr);
    // "/rpc/coordinate_manager/getAllValidUserCoordSummaryInfo"
    void handleRpc0x0001838F(void* request_data_ptr, void* response_data_ptr);

    /* reg manager rpc*/
    // "/rpc/reg_manager/r/addReg"
    void handleRpc0x00004FF7(void* request_data_ptr, void* response_data_ptr);
    // "/rpc/reg_manager/r/deleteReg"
    void handleRpc0x000012F7(void* request_data_ptr, void* response_data_ptr);
    // "/rpc/reg_manager/r/updateReg"
    void handleRpc0x00005757(void* request_data_ptr, void* response_data_ptr);
    // "/rpc/reg_manager/r/getReg"
    void handleRpc0x0000EAB7(void* request_data_ptr, void* response_data_ptr);
    // "/rpc/reg_manager/r/moveReg"
    void handleRpc0x0000C877(void* request_data_ptr, void* response_data_ptr);
    // "/rpc/reg_manager/r/getChangedList"
    void handleRpc0x0000A904(void* request_data_ptr, void* response_data_ptr);
    // "/rpc/reg_manager/r/getValidList"
    void handleRpc0x00008CE4(void* request_data_ptr, void* response_data_ptr);
    // "/rpc/reg_manager/mr/addReg"
    void handleRpc0x000097E7(void* request_data_ptr, void* response_data_ptr);
    // "/rpc/reg_manager/mr/deleteReg"
    void handleRpc0x0000E5D7(void* request_data_ptr, void* response_data_ptr);
    // "/rpc/reg_manager/mr/updateReg"
    void handleRpc0x0000E9B7(void* request_data_ptr, void* response_data_ptr);
    // "/rpc/reg_manager/mr/getReg"
    void handleRpc0x0000B507(void* request_data_ptr, void* response_data_ptr);
    // "/rpc/reg_manager/mr/moveReg"
    void handleRpc0x00015BA7(void* request_data_ptr, void* response_data_ptr);
    // "/rpc/reg_manager/mr/getChangedList"
    void handleRpc0x00001774(void* request_data_ptr, void* response_data_ptr);
    // "/rpc/reg_manager/mr/getValidList"
    void handleRpc0x00015CF4(void* request_data_ptr, void* response_data_ptr);
    // "/rpc/reg_manager/sr/addReg"
    void handleRpc0x000161E7(void* request_data_ptr, void* response_data_ptr);
    // "/rpc/reg_manager/sr/deleteReg"
    void handleRpc0x0000B817(void* request_data_ptr, void* response_data_ptr);
    // "/rpc/reg_manager/sr/updateReg"
    void handleRpc0x000119F7(void* request_data_ptr, void* response_data_ptr);
    // "/rpc/reg_manager/sr/getReg"
    void handleRpc0x00017F07(void* request_data_ptr, void* response_data_ptr);
    // "/rpc/reg_manager/sr/moveReg"
    void handleRpc0x00002127(void* request_data_ptr, void* response_data_ptr);
    // "/rpc/reg_manager/sr/getChangedList"
    void handleRpc0x00004834(void* request_data_ptr, void* response_data_ptr);
    // "/rpc/reg_manager/sr/getValidList"
    void handleRpc0x00009854(void* request_data_ptr, void* response_data_ptr);
    // "/rpc/reg_manager/pr/addReg"
    void handleRpc0x000154E7(void* request_data_ptr, void* response_data_ptr);
    // "/rpc/reg_manager/pr/deleteReg"
    void handleRpc0x00001097(void* request_data_ptr, void* response_data_ptr);
    // "/rpc/reg_manager/pr/updateReg"
    void handleRpc0x00009EF7(void* request_data_ptr, void* response_data_ptr);
    // "/rpc/reg_manager/pr/getReg"
    void handleRpc0x00017207(void* request_data_ptr, void* response_data_ptr);
    // "/rpc/reg_manager/pr/moveReg"
    void handleRpc0x0000D7C7(void* request_data_ptr, void* response_data_ptr);
    // "/rpc/reg_manager/pr/getChangedList"
    void handleRpc0x0000B454(void* request_data_ptr, void* response_data_ptr);
    // "/rpc/reg_manager/pr/getValidList"
    void handleRpc0x00009354(void* request_data_ptr, void* response_data_ptr);
    // "/rpc/reg_manager/hr/addReg"
    void handleRpc0x00016CE7(void* request_data_ptr, void* response_data_ptr);
    // "/rpc/reg_manager/hr/deleteReg"
    void handleRpc0x00003D17(void* request_data_ptr, void* response_data_ptr);
    // "/rpc/reg_manager/hr/updateReg"
    void handleRpc0x0000CB77(void* request_data_ptr, void* response_data_ptr);
    // "/rpc/reg_manager/hr/getReg"
    void handleRpc0x00000367(void* request_data_ptr, void* response_data_ptr);
    // "/rpc/reg_manager/hr/moveReg"
    void handleRpc0x00014A87(void* request_data_ptr, void* response_data_ptr);
    // "/rpc/reg_manager/hr/getChangedList"
    void handleRpc0x00012974(void* request_data_ptr, void* response_data_ptr);
    // "/rpc/reg_manager/hr/getValidList"
    void handleRpc0x00006B54(void* request_data_ptr, void* response_data_ptr);

    /* motion control rpc */
    // "/rpc/motion_control/stop"
    void handleRpc0x00001E70(void* request_data_ptr, void* response_data_ptr);
    // "/rpc/motion_control/reset"
    void handleRpc0x00001D14(void* request_data_ptr, void* response_data_ptr);
    // "/rpc/motion_control/setGlobalVelRatio"
    void handleRpc0x000005EF(void* request_data_ptr, void* response_data_ptr);
    // "/rpc/motion_control/getGlobalVelRatio"
    void handleRpc0x0001578F(void* request_data_ptr, void* response_data_ptr);
    // "/rpc/motion_control/setGlobalAccRatio"
    void handleRpc0x0000271F(void* request_data_ptr, void* response_data_ptr);
    // "/rpc/motion_control/getGlobalAccRatio"
    void handleRpc0x00016D9F(void* request_data_ptr, void* response_data_ptr); 
    // "/rpc/motion_control/getAxisGroupInfoList"
    void handleRpc0x00010F54(void* request_data_ptr, void* response_data_ptr);    
    // "/rpc/motion_control/axis_group/doStepManualMove"
    void handleRpc0x000085D5(void* request_data_ptr, void* response_data_ptr);
    // "/rpc/motion_control/axis_group/doContinuousManualMove"
    void handleRpc0x0000D3F5(void* request_data_ptr, void* response_data_ptr);
    // "/rpc/motion_control/axis_group/doGotoCartesianPointManualMove"
    void handleRpc0x00010C05(void* request_data_ptr, void* response_data_ptr);
    // "/rpc/motion_control/axis_group/doGotoJointPointManualMove"
    void handleRpc0x00008075(void* request_data_ptr, void* response_data_ptr);
    // "/rpc/motion_control/axis_group/doManualStop"
    void handleRpc0x0000A9A0(void* request_data_ptr, void* response_data_ptr);
    // "/rpc/motion_control/axis_group/getJointFeedBack"
    void handleRpc0x0000DFBB(void* request_data_ptr, void* response_data_ptr);
    // "/rpc/motion_control/axis_group/setUserSoftLimit"
    void handleRpc0x000114A4(void* request_data_ptr, void* response_data_ptr);
    // "/rpc/motion_control/axis_group/getUserSoftLimit"
    void handleRpc0x0000C764(void* request_data_ptr, void* response_data_ptr);
    // "/rpc/motion_control/axis_group/setManuSoftLimit"
    void handleRpc0x000108E4(void* request_data_ptr, void* response_data_ptr);
    // "/rpc/motion_control/axis_group/getManuSoftLimit"
    void handleRpc0x0000C244(void* request_data_ptr, void* response_data_ptr);
    // "/rpc/motion_control/axis_group/setHardLimit"
    void handleRpc0x0000C454(void* request_data_ptr, void* response_data_ptr);  
    // "/rpc/motion_control/axis_group/getHardLimit"
    void handleRpc0x00013394(void* request_data_ptr, void* response_data_ptr);   
    // "/rpc/motion_control/axis_group/setCoordinate"
    void handleRpc0x0000A845(void* request_data_ptr, void* response_data_ptr);
    // "/rpc/motion_control/axis_group/getCoordinate"
    void handleRpc0x00008595(void* request_data_ptr, void* response_data_ptr);
    // "/rpc/motion_control/axis_group/setUserCoordId"
    void handleRpc0x00005CF4(void* request_data_ptr, void* response_data_ptr);
    // "/rpc/motion_control/axis_group/getUserCoordId"
    void handleRpc0x00005BB4(void* request_data_ptr, void* response_data_ptr);    
    // "/rpc/motion_control/axis_group/setTool"
    void handleRpc0x0001581C(void* request_data_ptr, void* response_data_ptr);
    // "/rpc/motion_control/axis_group/getTool"
    void handleRpc0x0001354C(void* request_data_ptr, void* response_data_ptr);
    // "/rpc/motion_control/axis_group/convertCartToJoint"
    void handleRpc0x00010FD4(void* request_data_ptr, void* response_data_ptr);
    // "/rpc/motion_control/axis_group/convertJointToCart"
    void handleRpc0x0000B6D4(void* request_data_ptr, void* response_data_ptr);
    // "/rpc/motion_control/axis_group/ignoreLostZeroError"
    void handleRpc0x00014952(void* request_data_ptr, void* response_data_ptr);
    // "/rpc/motion_control/axis_group/setSingleZeroPointOffset"
    void handleRpc0x00012404(void* request_data_ptr, void* response_data_ptr);
    // "/rpc/motion_control/axis_group/setAllZeroPointOffsets"
    void handleRpc0x00008AB4(void* request_data_ptr, void* response_data_ptr);
    // "/rpc/motion_control/axis_group/getAllZeroPointOffsets"
    void handleRpc0x00012353(void* request_data_ptr, void* response_data_ptr);
    // "/rpc/motion_control/axis_group/getAllZeroErrorMaskStatus"
    void handleRpc0x0000C183(void* request_data_ptr, void* response_data_ptr);
    // "/rpc/motion_control/axis_group/saveAllZeroPointOffsets"
    void handleRpc0x000171D3(void* request_data_ptr, void* response_data_ptr);
    // "/rpc/motion_control/axis_group/setSingleZeroPointStatus"
    void handleRpc0x00010E43(void* request_data_ptr, void* response_data_ptr);
    // "/rpc/motion_control/axis_group/getAllZeroPointStatus"
    void handleRpc0x000102F3(void* request_data_ptr, void* response_data_ptr);
    // "/rpc/motion_control/axis_group/calibrateAllZeroPointOffsets"
    void handleRpc0x00011B03(void* request_data_ptr, void* response_data_ptr);
    // "/rpc/motion_control/axis_group/calibrateSingleZeroPointOffset"
    void handleRpc0x000131D4(void* request_data_ptr, void* response_data_ptr);
    // "/rpc/motion_control/axis_group/calibrateZeroPointOffsets"
    void handleRpc0x00005AE3(void* request_data_ptr, void* response_data_ptr);
    // "/rpc/motion_control/axis_group/isReferencePointExist"
    void handleRpc0x0000D344(void* request_data_ptr, void* response_data_ptr);
    // "/rpc/motion_control/axis_group/deleteReferencePoint"
    void handleRpc0x00008744(void* request_data_ptr, void* response_data_ptr);
    // "/rpc/motion_control/axis_group/saveReferencePoint"
    void handleRpc0x00006744(void* request_data_ptr, void* response_data_ptr);
    // "/rpc/motion_control/axis_group/fastCalibrateAllZeroPointOffsets"
    void handleRpc0x0000E913(void* request_data_ptr, void* response_data_ptr);
    // "/rpc/motion_control/axis_group/fastCalibrateSingleZeroPointOffset"
    void handleRpc0x00004754(void* request_data_ptr, void* response_data_ptr);
    // "/rpc/motion_control/axis_group/fastCalibrateZeroPointOffsets"
    void handleRpc0x00007EC3(void* request_data_ptr, void* response_data_ptr);
    // "/rpc/motion_control/axis_group/getUserSoftLimitWithUnit"
    void handleRpc0x00008ED4(void* request_data_ptr, void* response_data_ptr);
    // "/rpc/motion_control/axis_group/getManuSoftLimitWithUnit"
    void handleRpc0x000124E4(void* request_data_ptr, void* response_data_ptr);
    // "/rpc/motion_control/axis_group/getHardLimitWithUnit"
    void handleRpc0x000092B4(void* request_data_ptr, void* response_data_ptr);
    // "/rpc/motion_control/axis_group/setRotateManualStep"
    void handleRpc0x00005290(void* request_data_ptr, void* response_data_ptr);
    // "/rpc/motion_control/axis_group/getRotateManualStep"
    void handleRpc0x00003000(void* request_data_ptr, void* response_data_ptr);
    // "/rpc/motion_control/axis_group/setPrismaticManualStep"
    void handleRpc0x0000B640(void* request_data_ptr, void* response_data_ptr);
    // "/rpc/motion_control/axis_group/getPrismaticManualStep"
    void handleRpc0x0000FCE0(void* request_data_ptr, void* response_data_ptr);
    // "/rpc/motion_control/axis_group/setCartesianManualStep"
    void handleRpc0x0000A420(void* request_data_ptr, void* response_data_ptr);
    // "/rpc/motion_control/axis_group/getCartesianManualStep"
    void handleRpc0x0000EAC0(void* request_data_ptr, void* response_data_ptr);
    // "/rpc/motion_control/axis_group/setOrientationManualStep"
    void handleRpc0x00002940(void* request_data_ptr, void* response_data_ptr);
    // "/rpc/motion_control/axis_group/getOrientationManualStep"
    void handleRpc0x00016D20(void* request_data_ptr, void* response_data_ptr);

    /* interpreter rpc */
    // "/rpc/interpreter/start"
    void handleRpc0x00006154(void* request_data_ptr, void* response_data_ptr);
    // "/rpc/interpreter/debug"
    void handleRpc0x000102D7(void* request_data_ptr, void* response_data_ptr);
    // "/rpc/interpreter/forward"
    void handleRpc0x0000D974(void* request_data_ptr, void* response_data_ptr);
    // "/rpc/interpreter/backward"
    void handleRpc0x00008E74(void* request_data_ptr, void* response_data_ptr);
    // "/rpc/interpreter/jump"
    void handleRpc0x00015930(void* request_data_ptr, void* response_data_ptr);
    // "/rpc/interpreter/pause"
    void handleRpc0x0000BA55(void* request_data_ptr, void* response_data_ptr);
    // "/rpc/interpreter/resume"
    void handleRpc0x0000CF55(void* request_data_ptr, void* response_data_ptr);
    // "/rpc/interpreter/abort"
    void handleRpc0x000086F4(void* request_data_ptr, void* response_data_ptr);
    // "/rpc/interpreter/switchStep"
    void handleRpc0x000140F0(void* request_data_ptr, void* response_data_ptr);

    /* io mapping rpc */
    // "/rpc/io_mapping/getDIByBit"
    void handleRpc0x000050B4(void* request_data_ptr, void* response_data_ptr);
    // "/rpc/io_mapping/setDIByBit"
    void handleRpc0x00011754(void* request_data_ptr, void* response_data_ptr);
    // "/rpc/io_mapping/getDOByBit"
    void handleRpc0x00013074(void* request_data_ptr, void* response_data_ptr);
    // "/rpc/io_mapping/setDOByBit"
    void handleRpc0x00007074(void* request_data_ptr, void* response_data_ptr);
    // "/rpc/io_mapping/getRIByBit"
    void handleRpc0x00000684(void* request_data_ptr, void* response_data_ptr);
    // "/rpc/io_mapping/setRIByBit"
    void handleRpc0x0000CD24(void* request_data_ptr, void* response_data_ptr);
    // "/rpc/io_mapping/getROByBit"
    void handleRpc0x00005BD4(void* request_data_ptr, void* response_data_ptr);
    // "/rpc/io_mapping/setROByBit"
    void handleRpc0x00012274(void* request_data_ptr, void* response_data_ptr);
    //"/rpc/io_mapping/getUIByBit"
    void handleRpc0x0000A9A4(void* request_data_ptr, void* response_data_ptr);
    //"/rpc/io_mapping/setUIByBit"
    void handleRpc0x00017044(void* request_data_ptr, void* response_data_ptr);
    //"/rpc/io_mapping/getUOByBit"
    void handleRpc0x000002C4(void* request_data_ptr, void* response_data_ptr);


    // "/rpc/io_mapping/syncFileIoStatus"
    void handleRpc0x0000BA73(void* request_data_ptr, void* response_data_ptr);
    // "/rpc/io_mapping/syncFileIoMapping"
    void handleRpc0x0000C2A7(void* request_data_ptr, void* response_data_ptr);

    /* device manager rpc */
    // "/rpc/device_manager/getDeviceList"
    void handleRpc0x0000C1E0(void* request_data_ptr, void* response_data_ptr);
    // "/rpc/device_manager/get_FRP8A_IoDeviceInfo"
    void handleRpc0x00006BAF(void* request_data_ptr, void* response_data_ptr);
    // "/rpc/device_manager/getModbusIoDeviceInfo"
    void handleRpc0x0001421F(void* request_data_ptr, void* response_data_ptr);
    // "/rpc/device_manager/getIoDeviceInfoList"
    void handleRpc0x000024A4(void* request_data_ptr, void* response_data_ptr);
    // "/rpc/device_manager/getDeviceVersionList"
    void handleRpc0x0000F574(void* request_data_ptr, void* response_data_ptr);


    /* program launching rpc */
    // "/rpc/program_launching/setMethod"
    void handleRpc0x00011544(void* request_data_ptr, void* response_data_ptr);
    // "/rpc/program_launching/getMethod"
    void handleRpc0x00010944(void* request_data_ptr, void* response_data_ptr);
    // "/rpc/program_launching/syncFileMacroConfig"
    void handleRpc0x00016B27(void* request_data_ptr, void* response_data_ptr);

    //"/rpc/file_manager/readFile"
    void handleRpc0x0000A545(void* request_data_ptr, void* response_data_ptr);
    //"/rpc/file_manager/writeFile"
    void handleRpc0x00010D95(void* request_data_ptr, void* response_data_ptr);

	//"/rpc/modbus/setStartMode"	
    void handleRpc0x0000D3A5(void* request_data_ptr, void* response_data_ptr);
	//"/rpc/modbus/getStartMode"	
    void handleRpc0x000041C5(void* request_data_ptr, void* response_data_ptr);
	//"/rpc/modbus/setServerEnableStatus"	
    void handleRpc0x00004033(void* request_data_ptr, void* response_data_ptr);
	//"/rpc/modbus/getServerEnableStatus"	
    void handleRpc0x00004C33(void* request_data_ptr, void* response_data_ptr);
	//"/rpc/modbus/setServerStartInfo"	
    void handleRpc0x0001300F(void* request_data_ptr, void* response_data_ptr);
	//"/rpc/modbus/getServerStartInfo"	
    void handleRpc0x000018AF(void* request_data_ptr, void* response_data_ptr);
	//"/rpc/modbus/setServerAllFunctionAddrInfo"	
    void handleRpc0x0000A4BF(void* request_data_ptr, void* response_data_ptr);
	//"/rpc/modbus/getServerAllFunctionAddrInfo"	
    void handleRpc0x00005E1F(void* request_data_ptr, void* response_data_ptr);
	//"/rpc/modbus/getServerConfigParams"	
    void handleRpc0x0000E2E3(void* request_data_ptr, void* response_data_ptr);
	//"/rpc/modbus/openServer"	
    void handleRpc0x00010912(void* request_data_ptr, void* response_data_ptr);
	//"/rpc/modbus/closeServer"	
    void handleRpc0x000045B2(void* request_data_ptr, void* response_data_ptr);
	//"/rpc/modbus/getServerRunningStatus"	
    void handleRpc0x00000953(void* request_data_ptr, void* response_data_ptr);

    //"/rpc/modbus/addClient"
    void handleRpc0x00012E44(void* request_data_ptr, void* response_data_ptr);
    //"/rpc/modbus/deleteClient"
    void handleRpc0x00014CF4(void* request_data_ptr, void* response_data_ptr);
    //"/rpc/modbus/replaceClient"
    void handleRpc0x0000C2F4(void* request_data_ptr, void* response_data_ptr);
    //"/rpc/modbus/getClientIdList"
    void handleRpc0x000046C4(void* request_data_ptr, void* response_data_ptr);
    //"/rpc/modbus/setClientEnableStatus"
    void handleRpc0x00002AD3(void* request_data_ptr, void* response_data_ptr);
    //"/rpc/modbus/getClientEnableStatus"
    void handleRpc0x00018573(void* request_data_ptr, void* response_data_ptr);
    //"/rpc/modbus/setClientAllFunctionAddrInfo"
    void handleRpc0x0000A4CF(void* request_data_ptr, void* response_data_ptr);
    //"/rpc/modbus/getClientAllFunctionAddrInfo"
    void handleRpc0x0000132F(void* request_data_ptr, void* response_data_ptr);
    //"/rpc/modbus/updateClientStartInfo"
    void handleRpc0x00008C7F(void* request_data_ptr, void* response_data_ptr);
    //"/rpc/modbus/getClientStartInfo"
    void handleRpc0x0000084F(void* request_data_ptr, void* response_data_ptr);
    //"/rpc/modbus/getClientConfigParams"
    void handleRpc0x00009833(void* request_data_ptr, void* response_data_ptr);
    //"/rpc/modbus/connectClient"
    void handleRpc0x00014594(void* request_data_ptr, void* response_data_ptr);
    //"/rpc/modbus/closeClient"
    void handleRpc0x00006CA4(void* request_data_ptr, void* response_data_ptr);
    //"/rpc/modbus/isClientConnected"
    void handleRpc0x00002FC4(void* request_data_ptr, void* response_data_ptr);
    //"/rpc/modbus/getClientCtrlStatus"
    void handleRpc0x000170E3(void* request_data_ptr, void* response_data_ptr);
    //"/rpc/modbus/getConnectedClientList"
    void handleRpc0x00001DC4(void* request_data_ptr, void* response_data_ptr);
    //"/rpc/modbus/getClientSummaryStartInfoList"
    void handleRpc0x00005564(void* request_data_ptr, void* response_data_ptr);

	//"/rpc/modbus/writeCoils"
    void handleRpc0x0000BD83(void* request_data_ptr, void* response_data_ptr);
	//"/rpc/modbus/readCoils"
    void handleRpc0x0000A433(void* request_data_ptr, void* response_data_ptr);
	//"/rpc/modbus/readDiscreteInputs"
    void handleRpc0x0000C063(void* request_data_ptr, void* response_data_ptr);
	//"/rpc/modbus/writeHoldingRegs"
    void handleRpc0x00008C43(void* request_data_ptr, void* response_data_ptr);
	//"/rpc/modbus/readHoldingRegs"
    void handleRpc0x00003583(void* request_data_ptr, void* response_data_ptr);
	//"/rpc/modbus/readInputRegs"
    void handleRpc0x000072C3(void* request_data_ptr, void* response_data_ptr);


};

}


#endif

