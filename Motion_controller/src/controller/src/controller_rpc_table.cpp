#include "controller_rpc.h"


using namespace user_space;


void ControllerRpc::initRpcTable()
{
    RpcService rpc_service;   

    rpc_service =	{	"/rpc/publish/addTopic",	0x000050E3,	&ControllerRpc::handleRpc0x000050E3	};	rpc_table_.push_back(rpc_service);
    rpc_service =	{	"/rpc/publish/deleteTopic",	0x00004403,	&ControllerRpc::handleRpc0x00004403	};	rpc_table_.push_back(rpc_service);

    rpc_service =	{	"/rpc/file_manager/readFile",	0x0000A545,	&ControllerRpc::handleRpc0x0000A545	};	rpc_table_.push_back(rpc_service);
    rpc_service =	{	"/rpc/file_manager/writeFile",	0x00010D95,	&ControllerRpc::handleRpc0x00010D95	};	rpc_table_.push_back(rpc_service);

    rpc_service =	{	"/rpc/controller/getVersion",	0x000093EE,	&ControllerRpc::handleRpc0x000093EE	};	rpc_table_.push_back(rpc_service);
    rpc_service =	{	"/rpc/controller/setSystemTime",	0x000167C5,	&ControllerRpc::handleRpc0x000167C5	};	rpc_table_.push_back(rpc_service);
    rpc_service =	{	"/rpc/controller/getSystemTime",	0x000003F5,	&ControllerRpc::handleRpc0x000003F5	};	rpc_table_.push_back(rpc_service);
    rpc_service =	{	"/rpc/controller/setWorkMode",	0x00006825,	&ControllerRpc::handleRpc0x00006825	};	rpc_table_.push_back(rpc_service);
    rpc_service =	{	"/rpc/controller/getWorkMode",	0x00003325,	&ControllerRpc::handleRpc0x00003325	};	rpc_table_.push_back(rpc_service);

    rpc_service =	{	"/rpc/axis/mcPower",	0x000053E2,	&ControllerRpc::handleRpc0x000053E2	};	rpc_table_.push_back(rpc_service);
    rpc_service =	{	"/rpc/axis/mcReset",	0x000180C4,	&ControllerRpc::handleRpc0x000180C4	};	rpc_table_.push_back(rpc_service);
    rpc_service =	{	"/rpc/axis/mcStop",	0x00002820,	&ControllerRpc::handleRpc0x00002820	};	rpc_table_.push_back(rpc_service);
    rpc_service =	{	"/rpc/axis/mcHalt",	0x00004BB4,	&ControllerRpc::handleRpc0x00004BB4	};	rpc_table_.push_back(rpc_service);
    rpc_service =	{	"/rpc/axis/mcSetPosition",	0x0001798E,	&ControllerRpc::handleRpc0x0001798E	};	rpc_table_.push_back(rpc_service);
    rpc_service =	{	"/rpc/axis/mcReadParameter",	0x00016BF2,	&ControllerRpc::handleRpc0x00016BF2	};	rpc_table_.push_back(rpc_service);
    rpc_service =	{	"/rpc/axis/mcWriteParameter",	0x00005732,	&ControllerRpc::handleRpc0x00005732	};	rpc_table_.push_back(rpc_service);
    rpc_service =	{	"/rpc/axis/mcMoveAbsolute",	0x000051F5,	&ControllerRpc::handleRpc0x000051F5	};	rpc_table_.push_back(rpc_service);
    rpc_service =	{	"/rpc/axis/mcMoveVelocity",	0x00016CF9,	&ControllerRpc::handleRpc0x00016CF9	};	rpc_table_.push_back(rpc_service);
    rpc_service =	{	"/rpc/axis/mcReadActualPosition",	0x000012BE,	&ControllerRpc::handleRpc0x000012BE	};	rpc_table_.push_back(rpc_service);
    rpc_service =	{	"/rpc/axis/mcReadActualVelocity",	0x00002EA9,	&ControllerRpc::handleRpc0x00002EA9	};	rpc_table_.push_back(rpc_service);
    rpc_service =	{	"/rpc/axis/mcReadActualTorque",	0x00014265,	&ControllerRpc::handleRpc0x00014265	};	rpc_table_.push_back(rpc_service);
    rpc_service =	{	"/rpc/axis/mcReadAxisInfo",	0x0000314F,	&ControllerRpc::handleRpc0x0000314F	};	rpc_table_.push_back(rpc_service);
    rpc_service =	{	"/rpc/axis/mcReadStatus",	0x00003E53,	&ControllerRpc::handleRpc0x00003E53	};	rpc_table_.push_back(rpc_service);
    rpc_service =	{	"/rpc/axis/mcReadAxisError",	0x000063C2,	&ControllerRpc::handleRpc0x000063C2	};	rpc_table_.push_back(rpc_service);
    rpc_service =	{	"/rpc/axis/mcReadAxisErrorHistory",	0x00018469,	&ControllerRpc::handleRpc0x00018469	};	rpc_table_.push_back(rpc_service);
    rpc_service =	{	"/rpc/axis/mcMoveRelative",	0x0000CC85,	&ControllerRpc::handleRpc0x0000CC85	};	rpc_table_.push_back(rpc_service);
    rpc_service =	{	"/rpc/axis/mcHome",	0x000059B5,	&ControllerRpc::handleRpc0x000059B5	};	rpc_table_.push_back(rpc_service);
    rpc_service =	{	"/rpc/axis/rtmAbortHoming",	0x0000E4B7,	&ControllerRpc::handleRpc0x0000E4B7	};	rpc_table_.push_back(rpc_service);
    rpc_service =	{	"/rpc/axis/rtmReadAxisFdbPdoPtr",	0x0000A632,	&ControllerRpc::handleRpc0x0000A632	};	rpc_table_.push_back(rpc_service);
    rpc_service =	{	"/rpc/axis/rtmResetEncoder",	0x00000BA2,	&ControllerRpc::handleRpc0x00000BA2	};	rpc_table_.push_back(rpc_service);

    rpc_service =	{	"/rpc/group/mcGroupReset",	0x00016FF4,	&ControllerRpc::handleRpc0x00016FF4	};	rpc_table_.push_back(rpc_service);
    rpc_service =	{	"/rpc/group/mcGroupEnable",	0x00003615,	&ControllerRpc::handleRpc0x00003615	};	rpc_table_.push_back(rpc_service);
    rpc_service =	{	"/rpc/group/mcGroupDisable",	0x0000D185,	&ControllerRpc::handleRpc0x0000D185	};	rpc_table_.push_back(rpc_service);
    rpc_service =	{	"/rpc/group/mcGroupReadError",	0x00004BE2,	&ControllerRpc::handleRpc0x00004BE2	};	rpc_table_.push_back(rpc_service);
    rpc_service =	{	"/rpc/group/mcGroupReadStatus",	0x00002A83,	&ControllerRpc::handleRpc0x00002A83	};	rpc_table_.push_back(rpc_service);
    rpc_service =	{	"/rpc/group/resetAllEncoder",	0x000019D2,	&ControllerRpc::handleRpc0x000019D2	};	rpc_table_.push_back(rpc_service);

    rpc_service =	{	"/rpc/servo_sampling/setSamplingConfiguration",	0x0000845E,	&ControllerRpc::handleRpc0x0000845E	};	rpc_table_.push_back(rpc_service);
    rpc_service =	{	"/rpc/servo_sampling/getSamplingConfiguration",	0x000106EE,	&ControllerRpc::handleRpc0x000106EE	};	rpc_table_.push_back(rpc_service);
    rpc_service =	{	"/rpc/servo_sampling/activateSamplingConfiguration",	0x0000CDDE,	&ControllerRpc::handleRpc0x0000CDDE	};	rpc_table_.push_back(rpc_service);
    rpc_service =	{	"/rpc/servo_sampling/setSamplingSync",	0x00003743,	&ControllerRpc::handleRpc0x00003743	};	rpc_table_.push_back(rpc_service);
    rpc_service =	{	"/rpc/servo_sampling/getSamplingSync",	0x00006343,	&ControllerRpc::handleRpc0x00006343	};	rpc_table_.push_back(rpc_service);
    rpc_service =	{	"/rpc/servo_sampling/setSamplingChannel",	0x0000BACC,	&ControllerRpc::handleRpc0x0000BACC	};	rpc_table_.push_back(rpc_service);
    rpc_service =	{	"/rpc/servo_sampling/getSamplingChannel",	0x0000556C,	&ControllerRpc::handleRpc0x0000556C	};	rpc_table_.push_back(rpc_service);
    rpc_service =	{	"/rpc/servo_sampling/saveSamplingBufferData",	0x00004E41,	&ControllerRpc::handleRpc0x00004E41	};	rpc_table_.push_back(rpc_service);

    rpc_service =	{	"/rpc/servo1001/servo/shutDown",	0x0000863E,	&ControllerRpc::handleRpc0x0000863E	};	rpc_table_.push_back(rpc_service);
    rpc_service =	{	"/rpc/servo1001/servo/switchOn",	0x0000E5CE,	&ControllerRpc::handleRpc0x0000E5CE	};	rpc_table_.push_back(rpc_service);
    rpc_service =	{	"/rpc/servo1001/servo/disableVoltage",	0x00004755,	&ControllerRpc::handleRpc0x00004755	};	rpc_table_.push_back(rpc_service);
    rpc_service =	{	"/rpc/servo1001/servo/enableOperation",	0x0000313E,	&ControllerRpc::handleRpc0x0000313E	};	rpc_table_.push_back(rpc_service);
    rpc_service =	{	"/rpc/servo1001/servo/switchOnAndEnableOperation",	0x000177CE,	&ControllerRpc::handleRpc0x000177CE	};	rpc_table_.push_back(rpc_service);
    rpc_service =	{	"/rpc/servo1001/servo/disableOperation",	0x000026AE,	&ControllerRpc::handleRpc0x000026AE	};	rpc_table_.push_back(rpc_service);
    rpc_service =	{	"/rpc/servo1001/servo/quickStop",	0x00000580,	&ControllerRpc::handleRpc0x00000580	};	rpc_table_.push_back(rpc_service);
    rpc_service =	{	"/rpc/servo1001/servo/resetFault",	0x00010584,	&ControllerRpc::handleRpc0x00010584	};	rpc_table_.push_back(rpc_service);
    rpc_service =	{	"/rpc/servo1001/servo/transCommState",	0x000153C5,	&ControllerRpc::handleRpc0x000153C5	};	rpc_table_.push_back(rpc_service);
    rpc_service =	{	"/rpc/servo1001/servo/readParameter",	0x00006892,	&ControllerRpc::handleRpc0x00006892	};	rpc_table_.push_back(rpc_service);
    rpc_service =	{	"/rpc/servo1001/servo/writeParameter",	0x00007C32,	&ControllerRpc::handleRpc0x00007C32	};	rpc_table_.push_back(rpc_service);
    rpc_service =	{	"/rpc/servo1001/servo/moveVelocity",	0x000164D9,	&ControllerRpc::handleRpc0x000164D9	};	rpc_table_.push_back(rpc_service);
    rpc_service =	{	"/rpc/servo1001/servo/moveAbsolute",	0x00004DD5,	&ControllerRpc::handleRpc0x00004DD5	};	rpc_table_.push_back(rpc_service);
    rpc_service =	{	"/rpc/servo1001/servo/triggerUploadParameters",	0x000020B3,	&ControllerRpc::handleRpc0x000020B3	};	rpc_table_.push_back(rpc_service);
    rpc_service =	{	"/rpc/servo1001/servo/uploadParameters",	0x0000E003,	&ControllerRpc::handleRpc0x0000E003	};	rpc_table_.push_back(rpc_service);
    rpc_service =	{	"/rpc/servo1001/servo/triggerDownloadParameters",	0x00011C53,	&ControllerRpc::handleRpc0x00011C53	};	rpc_table_.push_back(rpc_service);
    rpc_service =	{	"/rpc/servo1001/servo/downloadParameters",	0x00017063,	&ControllerRpc::handleRpc0x00017063	};	rpc_table_.push_back(rpc_service);
    rpc_service =	{	"/rpc/servo1001/servo/isAsyncServiceFinish",	0x000043B8,	&ControllerRpc::handleRpc0x000043B8	};	rpc_table_.push_back(rpc_service);
    rpc_service =	{	"/rpc/servo1001/servo/getCommState",	0x0000F485,	&ControllerRpc::handleRpc0x0000F485	};	rpc_table_.push_back(rpc_service);
    rpc_service =	{	"/rpc/servo1001/servo/getServoState",	0x000032F5,	&ControllerRpc::handleRpc0x000032F5	};	rpc_table_.push_back(rpc_service);
    rpc_service =	{	"/rpc/servo1001/servo/mcMoveRelative",	0x000172C5,	&ControllerRpc::handleRpc0x000172C5	};	rpc_table_.push_back(rpc_service);
    rpc_service =	{	"/rpc/servo1001/servo/resetEncoder",	0x0000EFE2,	&ControllerRpc::handleRpc0x0000EFE2	};	rpc_table_.push_back(rpc_service);
    rpc_service =	{	"/rpc/servo1001/servo/goHome",	0x00013BB5,	&ControllerRpc::handleRpc0x00013BB5	};	rpc_table_.push_back(rpc_service);
    rpc_service =	{	"/rpc/servo1001/servo/abortHoming",	0x00015AB7,	&ControllerRpc::handleRpc0x00015AB7	};	rpc_table_.push_back(rpc_service);
    rpc_service =	{	"/rpc/servo1001/servo/getServoCommInfo",	0x0000BF1F,	&ControllerRpc::handleRpc0x0000BF1F	};	rpc_table_.push_back(rpc_service);
    rpc_service =	{	"/rpc/servo1001/servo/getServoDefinedInfo",	0x0000C87F,	&ControllerRpc::handleRpc0x0000C87F	};	rpc_table_.push_back(rpc_service);

    rpc_service =	{	"/rpc/servo1001/cpu/getVersion",	0x0001192E,	&ControllerRpc::handleRpc0x0001192E	};	rpc_table_.push_back(rpc_service);
    rpc_service =	{	"/rpc/servo1001/cpu/setCtrlPdoSync",	0x00005123,	&ControllerRpc::handleRpc0x00005123	};	rpc_table_.push_back(rpc_service);
    rpc_service =	{	"/rpc/servo1001/cpu/getCtrlPdoSync",	0x00005463,	&ControllerRpc::handleRpc0x00005463	};	rpc_table_.push_back(rpc_service);
    rpc_service =	{	"/rpc/servo1001/cpu/setSamplingSync",	0x00004023,	&ControllerRpc::handleRpc0x00004023	};	rpc_table_.push_back(rpc_service);
    rpc_service =	{	"/rpc/servo1001/cpu/getSamplingSync",	0x00006C23,	&ControllerRpc::handleRpc0x00006C23	};	rpc_table_.push_back(rpc_service);
    rpc_service =	{	"/rpc/servo1001/cpu/setSamplingInterval",	0x00003EEC,	&ControllerRpc::handleRpc0x00003EEC	};	rpc_table_.push_back(rpc_service);
    rpc_service =	{	"/rpc/servo1001/cpu/getSamplingInterval",	0x00001C2C,	&ControllerRpc::handleRpc0x00001C2C	};	rpc_table_.push_back(rpc_service);
    rpc_service =	{	"/rpc/servo1001/cpu/setSamplingMaxTimes",	0x000110A3,	&ControllerRpc::handleRpc0x000110A3	};	rpc_table_.push_back(rpc_service);
    rpc_service =	{	"/rpc/servo1001/cpu/getSamplingMaxTimes",	0x00013363,	&ControllerRpc::handleRpc0x00013363	};	rpc_table_.push_back(rpc_service);
    rpc_service =	{	"/rpc/servo1001/cpu/setSamplingChannel",	0x00008E5C,	&ControllerRpc::handleRpc0x00008E5C	};	rpc_table_.push_back(rpc_service);
    rpc_service =	{	"/rpc/servo1001/cpu/getSamplingChannel",	0x0000FD9C,	&ControllerRpc::handleRpc0x0000FD9C	};	rpc_table_.push_back(rpc_service);
    rpc_service =	{	"/rpc/servo1001/cpu/activateSamplingConfiguration",	0x0000939E,	&ControllerRpc::handleRpc0x0000939E	};	rpc_table_.push_back(rpc_service);
    rpc_service =	{	"/rpc/servo1001/cpu/saveSamplingBufferData",	0x00015621,	&ControllerRpc::handleRpc0x00015621	};	rpc_table_.push_back(rpc_service);
    rpc_service =	{	"/rpc/servo1001/cpu/getServoCpuCommInfo",	0x0000FE5F,	&ControllerRpc::handleRpc0x0000FE5F	};	rpc_table_.push_back(rpc_service);

    rpc_service =	{	"/rpc/io/readDI",	0x000185A9,	&ControllerRpc::handleRpc0x000185A9	};	rpc_table_.push_back(rpc_service);
    rpc_service =	{	"/rpc/io/readDO",	0x000185AF,	&ControllerRpc::handleRpc0x000185AF	};	rpc_table_.push_back(rpc_service);
    rpc_service =	{	"/rpc/io/writeDO",	0x00000C1F,	&ControllerRpc::handleRpc0x00000C1F	};	rpc_table_.push_back(rpc_service);
    
    rpc_service =	{	"/rpc/tool_manager/addTool",	0x0000A22C,	&ControllerRpc::handleRpc0x0000A22C	};	rpc_table_.push_back(rpc_service);
    rpc_service =	{	"/rpc/tool_manager/deleteTool",	0x00010E4C,	&ControllerRpc::handleRpc0x00010E4C	};	rpc_table_.push_back(rpc_service);
    rpc_service =	{	"/rpc/tool_manager/updateTool",	0x0000C78C,	&ControllerRpc::handleRpc0x0000C78C	};	rpc_table_.push_back(rpc_service);
    rpc_service =	{	"/rpc/tool_manager/moveTool",	0x000085FC,	&ControllerRpc::handleRpc0x000085FC	};	rpc_table_.push_back(rpc_service);
    rpc_service =	{	"/rpc/tool_manager/getToolInfoById",	0x00009E34,	&ControllerRpc::handleRpc0x00009E34	};	rpc_table_.push_back(rpc_service);
    rpc_service =	{	"/rpc/tool_manager/getAllValidToolSummaryInfo",	0x0001104F,	&ControllerRpc::handleRpc0x0001104F	};	rpc_table_.push_back(rpc_service);

    rpc_service =	{	"/rpc/coordinate_manager/addUserCoord",	0x00016764,	&ControllerRpc::handleRpc0x00016764	};	rpc_table_.push_back(rpc_service);
    rpc_service =	{	"/rpc/coordinate_manager/deleteUserCoord",	0x0000BAF4,	&ControllerRpc::handleRpc0x0000BAF4	};	rpc_table_.push_back(rpc_service);
    rpc_service =	{	"/rpc/coordinate_manager/updateUserCoord",	0x0000EC14,	&ControllerRpc::handleRpc0x0000EC14	};	rpc_table_.push_back(rpc_service);
    rpc_service =	{	"/rpc/coordinate_manager/moveUserCoord",	0x0000E104,	&ControllerRpc::handleRpc0x0000E104	};	rpc_table_.push_back(rpc_service);
    rpc_service =	{	"/rpc/coordinate_manager/getUserCoordInfoById",	0x00004324,	&ControllerRpc::handleRpc0x00004324	};	rpc_table_.push_back(rpc_service);
    rpc_service =	{	"/rpc/coordinate_manager/getAllValidUserCoordSummaryInfo",	0x0001838F,	&ControllerRpc::handleRpc0x0001838F	};	rpc_table_.push_back(rpc_service);

    rpc_service =	{	"/rpc/motion_control/setGlobalVelRatio",	0x000005EF,	&ControllerRpc::handleRpc0x000005EF	};	rpc_table_.push_back(rpc_service);
    rpc_service =	{	"/rpc/motion_control/getGlobalVelRatio",	0x0001578F,	&ControllerRpc::handleRpc0x0001578F	};	rpc_table_.push_back(rpc_service);
    rpc_service =	{	"/rpc/motion_control/setGlobalAccRatio",	0x0000271F,	&ControllerRpc::handleRpc0x0000271F	};	rpc_table_.push_back(rpc_service);
    rpc_service =	{	"/rpc/motion_control/getGlobalAccRatio",	0x00016D9F,	&ControllerRpc::handleRpc0x00016D9F	};	rpc_table_.push_back(rpc_service);
    rpc_service =	{	"/rpc/motion_control/axis_group/doStepManualMove",	0x000085D5,	&ControllerRpc::handleRpc0x000085D5	};	rpc_table_.push_back(rpc_service);
    rpc_service =	{	"/rpc/motion_control/axis_group/doContinuousManualMove",	0x0000D3F5,	&ControllerRpc::handleRpc0x0000D3F5	};	rpc_table_.push_back(rpc_service);
    rpc_service =	{	"/rpc/motion_control/axis_group/doGotoCartesianPointManualMove",	0x00010C05,	&ControllerRpc::handleRpc0x00010C05	};	rpc_table_.push_back(rpc_service);
    rpc_service =	{	"/rpc/motion_control/axis_group/doGotoJointPointManualMove",	0x00008075,	&ControllerRpc::handleRpc0x00008075	};	rpc_table_.push_back(rpc_service);
    rpc_service =	{	"/rpc/motion_control/axis_group/doManualStop",	0x0000A9A0,	&ControllerRpc::handleRpc0x0000A9A0	};	rpc_table_.push_back(rpc_service);
    rpc_service =	{	"/rpc/motion_control/axis_group/getJointsFeedBack",	0x0000DFBB,	&ControllerRpc::handleRpc0x0000DFBB	};	rpc_table_.push_back(rpc_service);
    rpc_service =	{	"/rpc/motion_control/axis_group/setUserSoftLimit",	0x000114A4,	&ControllerRpc::handleRpc0x000114A4	};	rpc_table_.push_back(rpc_service);
    rpc_service =	{	"/rpc/motion_control/axis_group/getUserSoftLimit",	0x0000C764,	&ControllerRpc::handleRpc0x0000C764	};	rpc_table_.push_back(rpc_service);
    rpc_service =	{	"/rpc/motion_control/axis_group/setManuSoftLimit",	0x000108E4,	&ControllerRpc::handleRpc0x000108E4	};	rpc_table_.push_back(rpc_service);
    rpc_service =	{	"/rpc/motion_control/axis_group/getManuSoftLimit",	0x0000C244,	&ControllerRpc::handleRpc0x0000C244	};	rpc_table_.push_back(rpc_service);
    rpc_service =	{	"/rpc/motion_control/axis_group/setHardLimit",	0x0000C454,	&ControllerRpc::handleRpc0x0000C454	};	rpc_table_.push_back(rpc_service);
    rpc_service =	{	"/rpc/motion_control/axis_group/getHardLimit",	0x00013394,	&ControllerRpc::handleRpc0x00013394	};	rpc_table_.push_back(rpc_service);
    rpc_service =	{	"/rpc/motion_control/axis_group/setCoordinate",	0x0000A845,	&ControllerRpc::handleRpc0x0000A845	};	rpc_table_.push_back(rpc_service);
    rpc_service =	{	"/rpc/motion_control/axis_group/getCoordinate",	0x00008595,	&ControllerRpc::handleRpc0x00008595	};	rpc_table_.push_back(rpc_service);
    rpc_service =	{	"/rpc/motion_control/axis_group/setUserCoordId",	0x00005CF4,	&ControllerRpc::handleRpc0x00005CF4	};	rpc_table_.push_back(rpc_service);
    rpc_service =	{	"/rpc/motion_control/axis_group/getUserCoordId",	0x00005BB4,	&ControllerRpc::handleRpc0x00005BB4	};	rpc_table_.push_back(rpc_service);
    rpc_service =	{	"/rpc/motion_control/axis_group/setTool",	0x0001581C,	&ControllerRpc::handleRpc0x0001581C	};	rpc_table_.push_back(rpc_service);
    rpc_service =	{	"/rpc/motion_control/axis_group/getTool",	0x0001354C,	&ControllerRpc::handleRpc0x0001354C	};	rpc_table_.push_back(rpc_service);
    rpc_service =	{	"/rpc/motion_control/axis_group/convertCartToJoint",	0x00010FD4,	&ControllerRpc::handleRpc0x00010FD4	};	rpc_table_.push_back(rpc_service);
    rpc_service =	{	"/rpc/motion_control/axis_group/convertJointToCart",	0x0000B6D4,	&ControllerRpc::handleRpc0x0000B6D4	};	rpc_table_.push_back(rpc_service);
    rpc_service =	{	"/rpc/motion_control/axis_group/ignoreLostZeroError",	0x00014952,	&ControllerRpc::handleRpc0x00014952	};	rpc_table_.push_back(rpc_service);
    rpc_service =	{	"/rpc/motion_control/axis_group/setSingleZeroPointOffset",	0x00012404,	&ControllerRpc::handleRpc0x00012404	};	rpc_table_.push_back(rpc_service);
    rpc_service =	{	"/rpc/motion_control/axis_group/setAllZeroPointOffsets",	0x00011853,	&ControllerRpc::handleRpc0x00011853	};	rpc_table_.push_back(rpc_service);
    rpc_service =	{	"/rpc/motion_control/axis_group/getAllZeroPointOffsets",	0x00012353,	&ControllerRpc::handleRpc0x00012353	};	rpc_table_.push_back(rpc_service);
    rpc_service =	{	"/rpc/motion_control/axis_group/getAllZeroErrorMaskStatus",	0x0000C183,	&ControllerRpc::handleRpc0x0000C183	};	rpc_table_.push_back(rpc_service);
    rpc_service =	{	"/rpc/motion_control/axis_group/setSingleZeroPointStatus",	0x00010E43,	&ControllerRpc::handleRpc0x00010E43	};	rpc_table_.push_back(rpc_service);
    rpc_service =	{	"/rpc/motion_control/axis_group/getAllZeroPointStatus",	0x000102F3,	&ControllerRpc::handleRpc0x000102F3	};	rpc_table_.push_back(rpc_service);
    rpc_service =	{	"/rpc/motion_control/axis_group/calibrateAllZeroPointOffsets",	0x00011B03,	&ControllerRpc::handleRpc0x00011B03	};	rpc_table_.push_back(rpc_service);
    rpc_service =	{	"/rpc/motion_control/axis_group/calibrateSingleZeroPointOffset",	0x000131D4,	&ControllerRpc::handleRpc0x000131D4	};	rpc_table_.push_back(rpc_service);
    rpc_service =	{	"/rpc/motion_control/axis_group/calibrateZeroPointOffsets",	0x00005AE3,	&ControllerRpc::handleRpc0x00005AE3	};	rpc_table_.push_back(rpc_service);
    rpc_service =	{	"/rpc/motion_control/axis_group/setJointManualStep",	0x00018470,	&ControllerRpc::handleRpc0x00018470	};	rpc_table_.push_back(rpc_service);
    rpc_service =	{	"/rpc/motion_control/axis_group/getJointManualStep",	0x00006D10,	&ControllerRpc::handleRpc0x00006D10	};	rpc_table_.push_back(rpc_service);
    rpc_service =	{	"/rpc/motion_control/axis_group/setCartesianManualStep",	0x0000A420,	&ControllerRpc::handleRpc0x0000A420	};	rpc_table_.push_back(rpc_service);
    rpc_service =	{	"/rpc/motion_control/axis_group/getCartesianManualStep",	0x0000EAC0,	&ControllerRpc::handleRpc0x0000EAC0	};	rpc_table_.push_back(rpc_service);
    rpc_service =	{	"/rpc/motion_control/axis_group/setOrientationManualStep",	0x00002940,	&ControllerRpc::handleRpc0x00002940	};	rpc_table_.push_back(rpc_service);
    rpc_service =	{	"/rpc/motion_control/axis_group/getOrientationManualStep",	0x00016D20,	&ControllerRpc::handleRpc0x00016D20	};	rpc_table_.push_back(rpc_service);
    rpc_service =	{	"/rpc/motion_control/axis_group/getFcpBasePose",	0x000016B5,	&ControllerRpc::handleRpc0x000016B5	};	rpc_table_.push_back(rpc_service);
    rpc_service =	{	"/rpc/motion_control/axis_group/getTcpCurrentPose",	0x00003B45,	&ControllerRpc::handleRpc0x00003B45	};	rpc_table_.push_back(rpc_service);
    rpc_service =	{	"/rpc/motion_control/getPostureByJoint",	0x0000EC64,	&ControllerRpc::handleRpc0x0000EC64	};	rpc_table_.push_back(rpc_service);
    rpc_service =	{	"/rpc/motion_control/axis_group/setOfflineTrajectoryFile",	0x00011275,	&ControllerRpc::handleRpc0x00011275	};	rpc_table_.push_back(rpc_service);
    rpc_service =	{	"/rpc/motion_control/axis_group/PrepareOfflineTrajectory",	0x000051E9,	&ControllerRpc::handleRpc0x000051E9	};	rpc_table_.push_back(rpc_service);
    rpc_service =	{	"/rpc/motion_control/axis_group/moveOfflineTrajectory",	0x0000C4D9,	&ControllerRpc::handleRpc0x0000C4D9	};	rpc_table_.push_back(rpc_service);


}

