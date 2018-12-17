#include "controller_rpc.h"


using namespace fst_ctrl;


void ControllerRpc::initRpcTable()
{
    RpcService rpc_service;
    
    rpc_service = {"/rpc/publish/addTopic", 0x000050E3, &ControllerRpc::handleRpc0x000050E3}; rpc_table_.push_back(rpc_service);
    rpc_service = {"/rpc/publish/addRegTopic", 0x000163A3, &ControllerRpc::handleRpc0x000163A3}; rpc_table_.push_back(rpc_service);
    rpc_service = {"/rpc/publish/addIoTopic", 0x000058F3, &ControllerRpc::handleRpc0x000058F3}; rpc_table_.push_back(rpc_service);
    rpc_service = {"/rpc/publish/deleteTopic", 0x00004403, &ControllerRpc::handleRpc0x00004403}; rpc_table_.push_back(rpc_service);
    rpc_service = {"/rpc/publish/deleteRegTopic", 0x00010353, &ControllerRpc::handleRpc0x00010353}; rpc_table_.push_back(rpc_service);
    rpc_service = {"/rpc/publish/deleteIoTopic", 0x0000DD03, &ControllerRpc::handleRpc0x0000DD03}; rpc_table_.push_back(rpc_service);
    
    rpc_service = {"/rpc/controller/getUserOpMode", 0x00000C05, &ControllerRpc::handleRpc0x00000C05}; rpc_table_.push_back(rpc_service);
    rpc_service = {"/rpc/controller/getRunningStatus", 0x00000AB3, &ControllerRpc::handleRpc0x00000AB3}; rpc_table_.push_back(rpc_service);
    rpc_service = {"/rpc/controller/getInterpreterStatus", 0x00016483, &ControllerRpc::handleRpc0x00016483}; rpc_table_.push_back(rpc_service);
    rpc_service = {"/rpc/controller/getRobotStatus", 0x00006F83, &ControllerRpc::handleRpc0x00006F83}; rpc_table_.push_back(rpc_service);
    rpc_service = {"/rpc/controller/getCtrlStatus", 0x0000E9D3, &ControllerRpc::handleRpc0x0000E9D3}; rpc_table_.push_back(rpc_service);
    rpc_service = {"/rpc/controller/getServoStatus", 0x0000D113, &ControllerRpc::handleRpc0x0000D113}; rpc_table_.push_back(rpc_service);
    rpc_service = {"/rpc/controller/getSafetyAlarm", 0x0000C00D, &ControllerRpc::handleRpc0x0000C00D}; rpc_table_.push_back(rpc_service);
    rpc_service = {"/rpc/controller/callEstop", 0x00013940, &ControllerRpc::handleRpc0x00013940}; rpc_table_.push_back(rpc_service);
    rpc_service = {"/rpc/controller/callReset", 0x000161E4, &ControllerRpc::handleRpc0x000161E4}; rpc_table_.push_back(rpc_service);
    rpc_service = {"/rpc/controller/setUserOpMode", 0x00002ED5, &ControllerRpc::handleRpc0x00002ED5}; rpc_table_.push_back(rpc_service);
    rpc_service = {"/rpc/controller/shutdown", 0x0000899E, &ControllerRpc::handleRpc0x0000899E}; rpc_table_.push_back(rpc_service);
    rpc_service = {"/rpc/controller/setSystemTime", 0x000167C5, &ControllerRpc::handleRpc0x000167C5}; rpc_table_.push_back(rpc_service);
    rpc_service = {"/rpc/controller/getSystemTime", 0x000003F5, &ControllerRpc::handleRpc0x000003F5}; rpc_table_.push_back(rpc_service);
    rpc_service = {"/rpc/controller/getVersion", 0x000093EE, &ControllerRpc::handleRpc0x000093EE}; rpc_table_.push_back(rpc_service);
    // todo delete
    //rpc_service = {"/rpc/controller/setStartMode", 0x00010225, &ControllerRpc::handleRpc0x00010225}; rpc_table_.push_back(rpc_service);
    //rpc_service = {"/rpc/controller/getStartMode", 0x000092E5, &ControllerRpc::handleRpc0x000092E5}; rpc_table_.push_back(rpc_service);
    rpc_service = {"/rpc/controller/getErrorCodeList", 0x00015F44, &ControllerRpc::handleRpc0x00015F44}; rpc_table_.push_back(rpc_service);


    rpc_service = {"/rpc/tool_manager/addTool", 0x0000A22C, &ControllerRpc::handleRpc0x0000A22C}; rpc_table_.push_back(rpc_service);
    rpc_service = {"/rpc/tool_manager/deleteTool", 0x00010E4C, &ControllerRpc::handleRpc0x00010E4C}; rpc_table_.push_back(rpc_service);
    rpc_service = {"/rpc/tool_manager/updateTool", 0x0000C78C, &ControllerRpc::handleRpc0x0000C78C}; rpc_table_.push_back(rpc_service);
    rpc_service = {"/rpc/tool_manager/moveTool", 0x000085FC, &ControllerRpc::handleRpc0x000085FC}; rpc_table_.push_back(rpc_service);
    rpc_service = {"/rpc/tool_manager/getToolInfoById", 0x00009E34, &ControllerRpc::handleRpc0x00009E34}; rpc_table_.push_back(rpc_service);
    rpc_service = {"/rpc/tool_manager/getAllValidToolSummaryInfo", 0x0001104F, &ControllerRpc::handleRpc0x0001104F}; rpc_table_.push_back(rpc_service);

    rpc_service = {"/rpc/coordinate_manager/addUserCoord", 0x00016764, &ControllerRpc::handleRpc0x00016764}; rpc_table_.push_back(rpc_service);
    rpc_service = {"/rpc/coordinate_manager/deleteUserCoord", 0x0000BAF4, &ControllerRpc::handleRpc0x0000BAF4}; rpc_table_.push_back(rpc_service);
    rpc_service = {"/rpc/coordinate_manager/updateUserCoord", 0x0000EC14, &ControllerRpc::handleRpc0x0000EC14}; rpc_table_.push_back(rpc_service);
    rpc_service = {"/rpc/coordinate_manager/moveUserCoord", 0x0000E104, &ControllerRpc::handleRpc0x0000E104}; rpc_table_.push_back(rpc_service);
    rpc_service = {"/rpc/coordinate_manager/getUserCoordInfoById", 0x00004324, &ControllerRpc::handleRpc0x00004324}; rpc_table_.push_back(rpc_service);
    rpc_service = {"/rpc/coordinate_manager/getAllValidUserCoordSummaryInfo", 0x0001838F, &ControllerRpc::handleRpc0x0001838F}; rpc_table_.push_back(rpc_service);

    rpc_service = {"/rpc/reg_manager/r/addReg", 0x00004FF7, &ControllerRpc::handleRpc0x00004FF7}; rpc_table_.push_back(rpc_service);
    rpc_service = {"/rpc/reg_manager/r/deleteReg", 0x000012F7, &ControllerRpc::handleRpc0x000012F7}; rpc_table_.push_back(rpc_service);
    rpc_service = {"/rpc/reg_manager/r/updateReg", 0x00005757, &ControllerRpc::handleRpc0x00005757}; rpc_table_.push_back(rpc_service);
    rpc_service = {"/rpc/reg_manager/r/getReg", 0x0000EAB7, &ControllerRpc::handleRpc0x0000EAB7}; rpc_table_.push_back(rpc_service);
    rpc_service = {"/rpc/reg_manager/r/moveReg", 0x0000C877, &ControllerRpc::handleRpc0x0000C877}; rpc_table_.push_back(rpc_service);
    rpc_service = {"/rpc/reg_manager/r/getChangedList", 0x0000A904, &ControllerRpc::handleRpc0x0000A904}; rpc_table_.push_back(rpc_service);
    rpc_service = {"/rpc/reg_manager/r/getValidList", 0x00008CE4, &ControllerRpc::handleRpc0x00008CE4}; rpc_table_.push_back(rpc_service);
    rpc_service = {"/rpc/reg_manager/mr/addReg", 0x000097E7, &ControllerRpc::handleRpc0x000097E7}; rpc_table_.push_back(rpc_service);
    rpc_service = {"/rpc/reg_manager/mr/deleteReg", 0x0000E5D7, &ControllerRpc::handleRpc0x0000E5D7}; rpc_table_.push_back(rpc_service);
    rpc_service = {"/rpc/reg_manager/mr/updateReg", 0x0000E9B7, &ControllerRpc::handleRpc0x0000E9B7}; rpc_table_.push_back(rpc_service);
    rpc_service = {"/rpc/reg_manager/mr/getReg", 0x0000B507, &ControllerRpc::handleRpc0x0000B507}; rpc_table_.push_back(rpc_service);
    rpc_service = {"/rpc/reg_manager/mr/moveReg", 0x00015BA7, &ControllerRpc::handleRpc0x00015BA7}; rpc_table_.push_back(rpc_service);
    rpc_service = {"/rpc/reg_manager/mr/getChangedList", 0x00001774, &ControllerRpc::handleRpc0x00001774}; rpc_table_.push_back(rpc_service);
    rpc_service = {"/rpc/reg_manager/mr/getValidList", 0x00015CF4, &ControllerRpc::handleRpc0x00015CF4}; rpc_table_.push_back(rpc_service);
    rpc_service = {"/rpc/reg_manager/sr/addReg", 0x000161E7, &ControllerRpc::handleRpc0x000161E7}; rpc_table_.push_back(rpc_service);
    rpc_service = {"/rpc/reg_manager/sr/deleteReg", 0x0000B817, &ControllerRpc::handleRpc0x0000B817}; rpc_table_.push_back(rpc_service);
    rpc_service = {"/rpc/reg_manager/sr/updateReg", 0x000119F7, &ControllerRpc::handleRpc0x000119F7}; rpc_table_.push_back(rpc_service);
    rpc_service = {"/rpc/reg_manager/sr/getReg", 0x00017F07, &ControllerRpc::handleRpc0x00017F07}; rpc_table_.push_back(rpc_service);
    rpc_service = {"/rpc/reg_manager/sr/moveReg", 0x00002127, &ControllerRpc::handleRpc0x00002127}; rpc_table_.push_back(rpc_service);
    rpc_service = {"/rpc/reg_manager/sr/getChangedList", 0x00004834, &ControllerRpc::handleRpc0x00004834}; rpc_table_.push_back(rpc_service);
    rpc_service = {"/rpc/reg_manager/sr/getValidList", 0x00009854, &ControllerRpc::handleRpc0x00009854}; rpc_table_.push_back(rpc_service);
    rpc_service = {"/rpc/reg_manager/pr/addReg", 0x000154E7, &ControllerRpc::handleRpc0x000154E7}; rpc_table_.push_back(rpc_service);
    rpc_service = {"/rpc/reg_manager/pr/deleteReg", 0x00001097, &ControllerRpc::handleRpc0x00001097}; rpc_table_.push_back(rpc_service);
    rpc_service = {"/rpc/reg_manager/pr/updateReg", 0x00009EF7, &ControllerRpc::handleRpc0x00009EF7}; rpc_table_.push_back(rpc_service);
    rpc_service = {"/rpc/reg_manager/pr/getReg", 0x00017207, &ControllerRpc::handleRpc0x00017207}; rpc_table_.push_back(rpc_service);
    rpc_service = {"/rpc/reg_manager/pr/moveReg", 0x0000D7C7, &ControllerRpc::handleRpc0x0000D7C7}; rpc_table_.push_back(rpc_service);
    rpc_service = {"/rpc/reg_manager/pr/getChangedList", 0x0000B454, &ControllerRpc::handleRpc0x0000B454}; rpc_table_.push_back(rpc_service);
    rpc_service = {"/rpc/reg_manager/pr/getValidList", 0x00009354, &ControllerRpc::handleRpc0x00009354}; rpc_table_.push_back(rpc_service);
    rpc_service = {"/rpc/reg_manager/hr/addReg", 0x00016CE7, &ControllerRpc::handleRpc0x00016CE7}; rpc_table_.push_back(rpc_service);
    rpc_service = {"/rpc/reg_manager/hr/deleteReg", 0x00003D17, &ControllerRpc::handleRpc0x00003D17}; rpc_table_.push_back(rpc_service);
    rpc_service = {"/rpc/reg_manager/hr/updateReg", 0x0000CB77, &ControllerRpc::handleRpc0x0000CB77}; rpc_table_.push_back(rpc_service);
    rpc_service = {"/rpc/reg_manager/hr/getReg", 0x00000367, &ControllerRpc::handleRpc0x00000367}; rpc_table_.push_back(rpc_service);
    rpc_service = {"/rpc/reg_manager/hr/moveReg", 0x00014A87, &ControllerRpc::handleRpc0x00014A87}; rpc_table_.push_back(rpc_service);
    rpc_service = {"/rpc/reg_manager/hr/getChangedList", 0x00012974, &ControllerRpc::handleRpc0x00012974}; rpc_table_.push_back(rpc_service);
    rpc_service = {"/rpc/reg_manager/hr/getValidList", 0x00006B54, &ControllerRpc::handleRpc0x00006B54}; rpc_table_.push_back(rpc_service);

    rpc_service = {"/rpc/motion_control/stop", 0x00001E70, &ControllerRpc::handleRpc0x00001E70}; rpc_table_.push_back(rpc_service);
    rpc_service = {"/rpc/motion_control/reset", 0x00001D14, &ControllerRpc::handleRpc0x00001D14}; rpc_table_.push_back(rpc_service);
    rpc_service = {"/rpc/motion_control/setGlobalVelRatio", 0x000005EF, &ControllerRpc::handleRpc0x000005EF}; rpc_table_.push_back(rpc_service);
    rpc_service = {"/rpc/motion_control/getGlobalVelRatio", 0x0001578F, &ControllerRpc::handleRpc0x0001578F}; rpc_table_.push_back(rpc_service);
    rpc_service = {"/rpc/motion_control/setGlobalAccRatio", 0x0000271F, &ControllerRpc::handleRpc0x0000271F}; rpc_table_.push_back(rpc_service);
    rpc_service = {"/rpc/motion_control/getGlobalAccRatio", 0x00016D9F, &ControllerRpc::handleRpc0x00016D9F}; rpc_table_.push_back(rpc_service);
    rpc_service = {"/rpc/motion_control/getAxisGroupInfoList", 0x00010F54, &ControllerRpc::handleRpc0x00010F54}; rpc_table_.push_back(rpc_service);
    rpc_service = {"/rpc/motion_control/axis_group/doStepManualMove", 0x000085D5, &ControllerRpc::handleRpc0x000085D5}; rpc_table_.push_back(rpc_service);
    rpc_service = {"/rpc/motion_control/axis_group/doContinuousManualMove", 0x0000D3F5, &ControllerRpc::handleRpc0x0000D3F5}; rpc_table_.push_back(rpc_service);
    rpc_service = {"/rpc/motion_control/axis_group/doGotoCartesianPointManualMove", 0x00010C05, &ControllerRpc::handleRpc0x00010C05}; rpc_table_.push_back(rpc_service);
    rpc_service = {"/rpc/motion_control/axis_group/doGotoJointPointManualMove", 0x00008075, &ControllerRpc::handleRpc0x00008075}; rpc_table_.push_back(rpc_service);
    rpc_service = {"/rpc/motion_control/axis_group/doManualStop", 0x0000A9A0, &ControllerRpc::handleRpc0x0000A9A0}; rpc_table_.push_back(rpc_service);
    rpc_service = {"/rpc/motion_control/axis_group/getJointFeedBack", 0x0000DFBB, &ControllerRpc::handleRpc0x0000DFBB}; rpc_table_.push_back(rpc_service);
    rpc_service = {"/rpc/motion_control/axis_group/setUserSoftLimit", 0x000114A4, &ControllerRpc::handleRpc0x000114A4}; rpc_table_.push_back(rpc_service);
    rpc_service = {"/rpc/motion_control/axis_group/getUserSoftLimit", 0x0000C764, &ControllerRpc::handleRpc0x0000C764}; rpc_table_.push_back(rpc_service);
    rpc_service = {"/rpc/motion_control/axis_group/setManuSoftLimit", 0x000108E4, &ControllerRpc::handleRpc0x000108E4}; rpc_table_.push_back(rpc_service);
    rpc_service = {"/rpc/motion_control/axis_group/getManuSoftLimit", 0x0000C244, &ControllerRpc::handleRpc0x0000C244}; rpc_table_.push_back(rpc_service);
    rpc_service = {"/rpc/motion_control/axis_group/setHardLimit", 0x0000C454, &ControllerRpc::handleRpc0x0000C454}; rpc_table_.push_back(rpc_service);
    rpc_service = {"/rpc/motion_control/axis_group/getHardLimit", 0x00013394, &ControllerRpc::handleRpc0x00013394}; rpc_table_.push_back(rpc_service);
    rpc_service = {"/rpc/motion_control/axis_group/setCoordinate", 0x0000A845, &ControllerRpc::handleRpc0x0000A845}; rpc_table_.push_back(rpc_service);
    rpc_service = {"/rpc/motion_control/axis_group/getCoordinate", 0x00008595, &ControllerRpc::handleRpc0x00008595}; rpc_table_.push_back(rpc_service);
    rpc_service = {"/rpc/motion_control/axis_group/setUserCoordId", 0x00005CF4, &ControllerRpc::handleRpc0x00005CF4}; rpc_table_.push_back(rpc_service);
    rpc_service = {"/rpc/motion_control/axis_group/getUserCoordId", 0x00005BB4, &ControllerRpc::handleRpc0x00005BB4}; rpc_table_.push_back(rpc_service);
    rpc_service = {"/rpc/motion_control/axis_group/setTool", 0x0001581C, &ControllerRpc::handleRpc0x0001581C}; rpc_table_.push_back(rpc_service);
    rpc_service = {"/rpc/motion_control/axis_group/getTool", 0x0001354C, &ControllerRpc::handleRpc0x0001354C}; rpc_table_.push_back(rpc_service);
    rpc_service = {"/rpc/motion_control/axis_group/convertCartToJoint", 0x00010FD4, &ControllerRpc::handleRpc0x00010FD4}; rpc_table_.push_back(rpc_service);
    rpc_service = {"/rpc/motion_control/axis_group/convertJointToCart", 0x0000B6D4, &ControllerRpc::handleRpc0x0000B6D4}; rpc_table_.push_back(rpc_service);
    rpc_service = {"/rpc/motion_control/axis_group/ignoreLostZeroError", 0x00014952, &ControllerRpc::handleRpc0x00014952}; rpc_table_.push_back(rpc_service);
    rpc_service = {"/rpc/motion_control/axis_group/setSingleZeroPointOffset", 0x00012404, &ControllerRpc::handleRpc0x00012404}; rpc_table_.push_back(rpc_service);
    rpc_service = {"/rpc/motion_control/axis_group/setAllZeroPointOffsets", 0x00008AB4, &ControllerRpc::handleRpc0x00008AB4}; rpc_table_.push_back(rpc_service);
    rpc_service = {"/rpc/motion_control/axis_group/getAllZeroPointOffsets", 0x00012353, &ControllerRpc::handleRpc0x00012353}; rpc_table_.push_back(rpc_service);
    rpc_service = {"/rpc/motion_control/axis_group/getAllZeroErrorMaskStatus", 0x0000C183, &ControllerRpc::handleRpc0x0000C183}; rpc_table_.push_back(rpc_service);
    rpc_service = {"/rpc/motion_control/axis_group/saveAllZeroPointOffsets", 0x000171D3, &ControllerRpc::handleRpc0x000171D3}; rpc_table_.push_back(rpc_service);
    rpc_service = {"/rpc/motion_control/axis_group/setSingleZeroPointStatus", 0x00010E43, &ControllerRpc::handleRpc0x00010E43}; rpc_table_.push_back(rpc_service);
    rpc_service = {"/rpc/motion_control/axis_group/getAllZeroPointStatus", 0x000102F3, &ControllerRpc::handleRpc0x000102F3}; rpc_table_.push_back(rpc_service);
    rpc_service = {"/rpc/motion_control/axis_group/calibrateAllZeroPointOffsets", 0x00011B03, &ControllerRpc::handleRpc0x00011B03}; rpc_table_.push_back(rpc_service);
    rpc_service = {"/rpc/motion_control/axis_group/calibrateSingleZeroPointOffset", 0x000131D4, &ControllerRpc::handleRpc0x000131D4}; rpc_table_.push_back(rpc_service);
    rpc_service = {"/rpc/motion_control/axis_group/calibrateZeroPointOffsets", 0x00005AE3, &ControllerRpc::handleRpc0x00005AE3}; rpc_table_.push_back(rpc_service);
    rpc_service = {"/rpc/motion_control/axis_group/isReferencePointExist", 0x0000D344, &ControllerRpc::handleRpc0x0000D344}; rpc_table_.push_back(rpc_service);
    rpc_service = {"/rpc/motion_control/axis_group/deleteReferencePoint", 0x00008744, &ControllerRpc::handleRpc0x00008744}; rpc_table_.push_back(rpc_service);
    rpc_service = {"/rpc/motion_control/axis_group/saveReferencePoint", 0x00006744, &ControllerRpc::handleRpc0x00006744}; rpc_table_.push_back(rpc_service);
    rpc_service = {"/rpc/motion_control/axis_group/fastCalibrateAllZeroPointOffsets", 0x0000E913, &ControllerRpc::handleRpc0x0000E913}; rpc_table_.push_back(rpc_service);
    rpc_service = {"/rpc/motion_control/axis_group/fastCalibrateSingleZeroPointOffset", 0x00004754, &ControllerRpc::handleRpc0x00004754}; rpc_table_.push_back(rpc_service);
    rpc_service = {"/rpc/motion_control/axis_group/fastCalibrateZeroPointOffsets", 0x00007EC3, &ControllerRpc::handleRpc0x00007EC3}; rpc_table_.push_back(rpc_service);
    rpc_service = {"/rpc/motion_control/axis_group/getUserSoftLimitWithUnit", 0x00008ED4, &ControllerRpc::handleRpc0x00008ED4}; rpc_table_.push_back(rpc_service);
    rpc_service = {"/rpc/motion_control/axis_group/getManuSoftLimitWithUnit", 0x000124E4, &ControllerRpc::handleRpc0x000124E4}; rpc_table_.push_back(rpc_service);
    rpc_service = {"/rpc/motion_control/axis_group/getHardLimitWithUnit", 0x000092B4, &ControllerRpc::handleRpc0x000092B4}; rpc_table_.push_back(rpc_service);
    rpc_service = {"/rpc/motion_control/axis_group/setRotateManualStep", 0x00005290, &ControllerRpc::handleRpc0x00005290}; rpc_table_.push_back(rpc_service);
    rpc_service = {"/rpc/motion_control/axis_group/getRotateManualStep", 0x00003000, &ControllerRpc::handleRpc0x00003000}; rpc_table_.push_back(rpc_service);
    rpc_service = {"/rpc/motion_control/axis_group/setPrismaticManualStep", 0x0000B640, &ControllerRpc::handleRpc0x0000B640}; rpc_table_.push_back(rpc_service);
    rpc_service = {"/rpc/motion_control/axis_group/getPrismaticManualStep", 0x0000FCE0, &ControllerRpc::handleRpc0x0000FCE0}; rpc_table_.push_back(rpc_service);
    rpc_service = {"/rpc/motion_control/axis_group/setCartesianManualStep", 0x0000A420, &ControllerRpc::handleRpc0x0000A420}; rpc_table_.push_back(rpc_service);
    rpc_service = {"/rpc/motion_control/axis_group/getCartesianManualStep", 0x0000EAC0, &ControllerRpc::handleRpc0x0000EAC0}; rpc_table_.push_back(rpc_service);
    rpc_service = {"/rpc/motion_control/axis_group/setOrientationManualStep", 0x00002940, &ControllerRpc::handleRpc0x00002940}; rpc_table_.push_back(rpc_service);
    rpc_service = {"/rpc/motion_control/axis_group/getOrientationManualStep", 0x00016D20, &ControllerRpc::handleRpc0x00016D20}; rpc_table_.push_back(rpc_service);

    rpc_service = {"/rpc/interpreter/start", 0x00006154, &ControllerRpc::handleRpc0x00006154}; rpc_table_.push_back(rpc_service);
    rpc_service = {"/rpc/interpreter/debug", 0x000102D7, &ControllerRpc::handleRpc0x000102D7}; rpc_table_.push_back(rpc_service);
    rpc_service = {"/rpc/interpreter/forward", 0x0000D974, &ControllerRpc::handleRpc0x0000D974}; rpc_table_.push_back(rpc_service);
    rpc_service = {"/rpc/interpreter/backward", 0x00008E74, &ControllerRpc::handleRpc0x00008E74}; rpc_table_.push_back(rpc_service);
    rpc_service = {"/rpc/interpreter/jump", 0x00015930, &ControllerRpc::handleRpc0x00015930}; rpc_table_.push_back(rpc_service);
    rpc_service = {"/rpc/interpreter/pause", 0x0000BA55, &ControllerRpc::handleRpc0x0000BA55}; rpc_table_.push_back(rpc_service);
    rpc_service = {"/rpc/interpreter/resume", 0x0000CF55, &ControllerRpc::handleRpc0x0000CF55}; rpc_table_.push_back(rpc_service);
    rpc_service = {"/rpc/interpreter/abort", 0x000086F4, &ControllerRpc::handleRpc0x000086F4}; rpc_table_.push_back(rpc_service);
    rpc_service = {"/rpc/interpreter/switchStep", 0x000140F0, &ControllerRpc::handleRpc0x000140F0}; rpc_table_.push_back(rpc_service);

    rpc_service = {"/rpc/io_mapping/getDIByBit", 0x000050B4, &ControllerRpc::handleRpc0x000050B4}; rpc_table_.push_back(rpc_service);
    rpc_service = {"/rpc/io_mapping/setDIByBit", 0x00011754, &ControllerRpc::handleRpc0x00011754}; rpc_table_.push_back(rpc_service);
    rpc_service = {"/rpc/io_mapping/getDOByBit", 0x00013074, &ControllerRpc::handleRpc0x00013074}; rpc_table_.push_back(rpc_service);
    rpc_service = {"/rpc/io_mapping/setDOByBit", 0x00007074, &ControllerRpc::handleRpc0x00007074}; rpc_table_.push_back(rpc_service);
    rpc_service = {"/rpc/io_mapping/getRIByBit", 0x00000684, &ControllerRpc::handleRpc0x00000684}; rpc_table_.push_back(rpc_service);
    rpc_service = {"/rpc/io_mapping/setRIByBit", 0x0000CD24, &ControllerRpc::handleRpc0x0000CD24}; rpc_table_.push_back(rpc_service);
    rpc_service = {"/rpc/io_mapping/getROByBit", 0x00005BD4, &ControllerRpc::handleRpc0x00005BD4}; rpc_table_.push_back(rpc_service);
    rpc_service = {"/rpc/io_mapping/setROByBit", 0x00012274, &ControllerRpc::handleRpc0x00012274}; rpc_table_.push_back(rpc_service);
    rpc_service = {"/rpc/io_mapping/syncFileIoStatus", 0x0000BA73, &ControllerRpc::handleRpc0x0000BA73}; rpc_table_.push_back(rpc_service);
    rpc_service = {"/rpc/io_mapping/syncFileIoMapping", 0x0000C2A7, &ControllerRpc::handleRpc0x0000C2A7}; rpc_table_.push_back(rpc_service);

    rpc_service = {"/rpc/device_manager/getDeviceList", 0x0000C1E0, &ControllerRpc::handleRpc0x0000C1E0}; rpc_table_.push_back(rpc_service);
    rpc_service = {"/rpc/device_manager/get_FRP8A_IoDeviceInfo", 0x00006BAF, &ControllerRpc::handleRpc0x00006BAF}; rpc_table_.push_back(rpc_service);
    rpc_service = {"/rpc/device_manager/getModbusIoDeviceInfo", 0x0000215F, &ControllerRpc::handleRpc0x0000215F}; rpc_table_.push_back(rpc_service);

    rpc_service = {"/rpc/program_launching/setMethod", 0x00011544, &ControllerRpc::handleRpc0x00011544}; rpc_table_.push_back(rpc_service);
    rpc_service = {"/rpc/program_launching/getMethod", 0x00010944, &ControllerRpc::handleRpc0x00010944}; rpc_table_.push_back(rpc_service);
    rpc_service = {"/rpc/program_launching/syncFileMacroConfig", 0x00016B27, &ControllerRpc::handleRpc0x00016B27}; rpc_table_.push_back(rpc_service);

    rpc_service =	{	"/rpc/file_manager/readFile",	0x0000A545,	&ControllerRpc::handleRpc0x0000A545	};	rpc_table_.push_back(rpc_service);
    rpc_service =	{	"/rpc/file_manager/writeFile",	0x00010D95,	&ControllerRpc::handleRpc0x00010D95	};	rpc_table_.push_back(rpc_service);

	rpc_service =	{	"/rpc/modbus/setStartMode",	0x0000D3A5,	&ControllerRpc::handleRpc0x0000D3A5	};	rpc_table_.push_back(rpc_service);
	rpc_service =	{	"/rpc/modbus/getStartMode",	0x000041C5,	&ControllerRpc::handleRpc0x000041C5	};	rpc_table_.push_back(rpc_service);
	rpc_service =	{	"/rpc/modbus/setServerConnectStatus",	0x0000DB23,	&ControllerRpc::handleRpc0x0000DB23	};	rpc_table_.push_back(rpc_service);
	rpc_service =	{	"/rpc/modbus/getServerConnectStatus",	0x00001B23,	&ControllerRpc::handleRpc0x00001B23	};	rpc_table_.push_back(rpc_service);
	rpc_service =	{	"/rpc/modbus/setServerConfig",	0x00017547,	&ControllerRpc::handleRpc0x00017547	};	rpc_table_.push_back(rpc_service);
	rpc_service =	{	"/rpc/modbus/getServerConfig",	0x00016947,	&ControllerRpc::handleRpc0x00016947	};	rpc_table_.push_back(rpc_service);
	rpc_service =	{	"/rpc/modbus/getServerStartInfo",	0x000018AF,	&ControllerRpc::handleRpc0x000018AF	};	rpc_table_.push_back(rpc_service);
	rpc_service =	{	"/rpc/modbus/openServer",	0x00010912,	&ControllerRpc::handleRpc0x00010912	};	rpc_table_.push_back(rpc_service);
	rpc_service =	{	"/rpc/modbus/closeServer",	0x000045B2,	&ControllerRpc::handleRpc0x000045B2	};	rpc_table_.push_back(rpc_service);
	rpc_service =	{	"/rpc/modbus/addClient",	0x00012E44,	&ControllerRpc::handleRpc0x00012E44	};	rpc_table_.push_back(rpc_service);
	rpc_service =	{	"/rpc/modbus/setClientConnectStatus",	0x000099D3,	&ControllerRpc::handleRpc0x000099D3	};	rpc_table_.push_back(rpc_service);
	rpc_service =	{	"/rpc/modbus/getClientConnectStatus",	0x00010A53,	&ControllerRpc::handleRpc0x00010A53	};	rpc_table_.push_back(rpc_service);
	rpc_service =	{	"/rpc/modbus/setClientConfig",	0x0000D017,	&ControllerRpc::handleRpc0x0000D017	};	rpc_table_.push_back(rpc_service);
	rpc_service =	{	"/rpc/modbus/getClientConfig",	0x0000FC17,	&ControllerRpc::handleRpc0x0000FC17	};	rpc_table_.push_back(rpc_service);
	rpc_service =	{	"/rpc/modbus/openClient",	0x00000544,	&ControllerRpc::handleRpc0x00000544	};	rpc_table_.push_back(rpc_service);
	rpc_service =	{	"/rpc/modbus/closeClient",	0x00006CA4,	&ControllerRpc::handleRpc0x00006CA4	};	rpc_table_.push_back(rpc_service);
	rpc_service =	{	"/rpc/modbus/deleteClient",	0x00014CF4,	&ControllerRpc::handleRpc0x00014CF4	};	rpc_table_.push_back(rpc_service);
	rpc_service =	{	"/rpc/modbus/getClientSummaryInfoList",	0x0000B424,	&ControllerRpc::handleRpc0x0000B424	};	rpc_table_.push_back(rpc_service);
	rpc_service =	{	"/rpc/modbus/writeCoils",	0x0000BD83,	&ControllerRpc::handleRpc0x0000BD83	};	rpc_table_.push_back(rpc_service);
	rpc_service =	{	"/rpc/modbus/readCoils",	0x0000A433,	&ControllerRpc::handleRpc0x0000A433	};	rpc_table_.push_back(rpc_service);
	rpc_service =	{	"/rpc/modbus/readDiscreteInputs",	0x0000C063,	&ControllerRpc::handleRpc0x0000C063	};	rpc_table_.push_back(rpc_service);
	rpc_service =	{	"/rpc/modbus/writeHoldingRegs",	0x00008C43,	&ControllerRpc::handleRpc0x00008C43	};	rpc_table_.push_back(rpc_service);
	rpc_service =	{	"/rpc/modbus/readHoldingRegs",	0x00003583,	&ControllerRpc::handleRpc0x00003583	};	rpc_table_.push_back(rpc_service);
	rpc_service =	{	"/rpc/modbus/readInputRegs",	0x000072C3,	&ControllerRpc::handleRpc0x000072C3	};	rpc_table_.push_back(rpc_service);

	rpc_service =	{	"/rpc/modbus/getServerValidFuctionInfo",	0x00008E7F,	&ControllerRpc::handleRpc0x00008E7F	};	rpc_table_.push_back(rpc_service);
	rpc_service =	{	"/rpc/modbus/getServerResponseDelay",	0x00000329,	&ControllerRpc::handleRpc0x00000329	};	rpc_table_.push_back(rpc_service);
	rpc_service =	{	"/rpc/modbus/getServerRunningStatus",	0x00000953,	&ControllerRpc::handleRpc0x00000953	};	rpc_table_.push_back(rpc_service);

}

