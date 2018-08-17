#include "tp_comm.h"
using namespace fst_comm;

void TpComm::initRpcTable()
{
    RpcService rpc_service;
	rpc_service =	{	"/rpc/controller/getUserOpMode",	0x00000C05,	"RequestMessageType.Void",	"ResponseMessageType.Int32",	&TpComm::handleRequest0x00000C05,	&TpComm::handleResponse0x00000C05,	Comm_Authority_TP,	};	 rpc_table_.push_back(rpc_service);
	rpc_service =	{	"/rpc/controller/getRunningStatus",	0x00000AB3,	"RequestMessageType.Void",	"ResponseMessageType.Int32",	&TpComm::handleRequest0x00000AB3,	&TpComm::handleResponse0x00000AB3,	Comm_Authority_TP,	};	 rpc_table_.push_back(rpc_service);
	rpc_service =	{	"/rpc/controller/getInterpreterStatus",	0x00016483,	"RequestMessageType.Void",	"ResponseMessageType.Int32",	&TpComm::handleRequest0x00016483,	&TpComm::handleResponse0x00016483,	Comm_Authority_TP,	};	 rpc_table_.push_back(rpc_service);
	rpc_service =	{	"/rpc/controller/getRobotStatus",	0x00006F83,	"RequestMessageType.Void",	"ResponseMessageType.Int32",	&TpComm::handleRequest0x00006F83,	&TpComm::handleResponse0x00006F83,	Comm_Authority_TP,	};	 rpc_table_.push_back(rpc_service);
	rpc_service =	{	"/rpc/controller/getCtrlStatus",	0x0000E9D3,	"RequestMessageType.Void",	"ResponseMessageType.Int32",	&TpComm::handleRequest0x0000E9D3,	&TpComm::handleResponse0x0000E9D3,	Comm_Authority_TP,	};	 rpc_table_.push_back(rpc_service);
	rpc_service =	{	"/rpc/controller/getServoStatus",	0x0000D113,	"RequestMessageType.Void",	"ResponseMessageType.Int32",	&TpComm::handleRequest0x0000D113,	&TpComm::handleResponse0x0000D113,	Comm_Authority_TP,	};	 rpc_table_.push_back(rpc_service);
	rpc_service =	{	"/rpc/controller/getSafetyAlarm",	0x0000C00D,	"RequestMessageType.Void",	"ResponseMessageType.Int32",	&TpComm::handleRequest0x0000C00D,	&TpComm::handleResponse0x0000C00D,	Comm_Authority_TP,	};	 rpc_table_.push_back(rpc_service);
	rpc_service =	{	"/rpc/controller/callEstop",	0x00013940,	"RequestMessageType.Void",	"ResponseMessageType.Bool",	&TpComm::handleRequest0x00013940,	&TpComm::handleResponse0x00013940,	Comm_Authority_TP,	};	 rpc_table_.push_back(rpc_service);
	rpc_service =	{	"/rpc/controller/callReset",	0x000161E4,	"RequestMessageType.Void",	"ResponseMessageType.Bool",	&TpComm::handleRequest0x000161E4,	&TpComm::handleResponse0x000161E4,	Comm_Authority_TP,	};	 rpc_table_.push_back(rpc_service);
	rpc_service =	{	"/rpc/controller/setUserOpMode",	0x00002ED5,	"RequestMessageType.Int32",	"ResponseMessageType.Bool",	&TpComm::handleRequest0x00002ED5,	&TpComm::handleResponse0x00002ED5,	Comm_Authority_TP,	};	 rpc_table_.push_back(rpc_service);

	rpc_service =	{	"/rpc/publish/addTopic",	0x000050E3,	"RequestMessageType.Topic",	"ResponseMessageType.Bool",	&TpComm::handleRequest0x000050E3,	&TpComm::handleResponse0x000050E3,	Comm_Authority_TP,	};	 rpc_table_.push_back(rpc_service);
	rpc_service =	{	"/rpc/publish/deleteTopic",	0x00004403,	"RequestMessageType.UnsignedInt32",	"ResponseMessageType.Bool",	&TpComm::handleRequest0x00004403,	&TpComm::handleResponse0x00004403,	Comm_Authority_TP,	};	 rpc_table_.push_back(rpc_service);
	rpc_service =	{	"/rpc/publish/addRegTopic",	0x000163A3,	"RequestMessageType.Topic",	"ResponseMessageType.Bool",	&TpComm::handleRequest0x000163A3,	&TpComm::handleResponse0x000163A3,	Comm_Authority_TP,	};	 rpc_table_.push_back(rpc_service);
	rpc_service =	{	"/rpc/publish/addIoTopic",	0x000058F3,	"RequestMessageType.Topic",	"ResponseMessageType.Bool",	&TpComm::handleRequest0x000058F3,	&TpComm::handleResponse0x000058F3,	Comm_Authority_TP,	};	 rpc_table_.push_back(rpc_service);

	rpc_service =	{	"/rpc/tp_comm/getRpcTable",	0x00004FA5,	"RequestMessageType.Void",	"ResponseMessageType.RpcTable",	&TpComm::handleRequest0x00004FA5,	&TpComm::handleResponse0x00004FA5,	Comm_Authority_TP,	};	 rpc_table_.push_back(rpc_service);
	rpc_service =	{	"/rpc/tp_comm/getPublishTable",	0x000147A5,	"RequestMessageType.Void",	"ResponseMessageType.PublishTable",	&TpComm::handleRequest0x000147A5,	&TpComm::handleResponse0x000147A5,	Comm_Authority_TP,	};	 rpc_table_.push_back(rpc_service);

	rpc_service =	{	"/rpc/tool_manager/addTool",	0x0000A22C,	"RequestMessageType.ToolInfo",	"ResponseMessageType.Bool",	&TpComm::handleRequest0x0000A22C,	&TpComm::handleResponse0x0000A22C,	Comm_Authority_TP,	};	 rpc_table_.push_back(rpc_service);
	rpc_service =	{	"/rpc/tool_manager/deleteTool",	0x00010E4C,	"RequestMessageType.Int32",	"ResponseMessageType.Bool",	&TpComm::handleRequest0x00010E4C,	&TpComm::handleResponse0x00010E4C,	Comm_Authority_TP,	};	 rpc_table_.push_back(rpc_service);
	rpc_service =	{	"/rpc/tool_manager/updateTool",	0x0000C78C,	"RequestMessageType.ToolInfo",	"ResponseMessageType.Bool",	&TpComm::handleRequest0x0000C78C,	&TpComm::handleResponse0x0000C78C,	Comm_Authority_TP,	};	 rpc_table_.push_back(rpc_service);
	rpc_service =	{	"/rpc/tool_manager/moveTool",	0x000085FC,	"RequestMessageType.Int32List",	"ResponseMessageType.Bool",	&TpComm::handleRequest0x000085FC,	&TpComm::handleResponse0x000085FC,	Comm_Authority_TP,	};	 rpc_table_.push_back(rpc_service);
	rpc_service =	{	"/rpc/tool_manager/getToolInfoById",	0x00009E34,	"RequestMessageType.Int32",	"ResponseMessageType.Bool_ToolInfo",	&TpComm::handleRequest0x00009E34,	&TpComm::handleResponse0x00009E34,	Comm_Authority_TP,	};	 rpc_table_.push_back(rpc_service);
	rpc_service =	{	"/rpc/tool_manager/getAllValidToolSummaryInfo",	0x0001104F,	"RequestMessageType.Void",	"ResponseMessageType.ToolSummaryList",	&TpComm::handleRequest0x0001104F,	&TpComm::handleResponse0x0001104F,	Comm_Authority_TP,	};	 rpc_table_.push_back(rpc_service);
											
	rpc_service =	{	"/rpc/coordinate_manager/addUserCoord",	0x00016764,	"RequestMessageType.UserInfo",	"ResponseMessageType.Bool",	&TpComm::handleRequest0x00016764,	&TpComm::handleResponse0x00016764,	Comm_Authority_TP,	};	 rpc_table_.push_back(rpc_service);
	rpc_service =	{	"/rpc/coordinate_manager/deleteUserCoord",	0x0000BAF4,	"RequestMessageType.Int32",	"ResponseMessageType.Bool",	&TpComm::handleRequest0x0000BAF4,	&TpComm::handleResponse0x0000BAF4,	Comm_Authority_TP,	};	 rpc_table_.push_back(rpc_service);
	rpc_service =	{	"/rpc/coordinate_manager/updateUserCoord",	0x0000EC14,	"RequestMessageType.UserInfo",	"ResponseMessageType.Bool",	&TpComm::handleRequest0x0000EC14,	&TpComm::handleResponse0x0000EC14,	Comm_Authority_TP,	};	 rpc_table_.push_back(rpc_service);
	rpc_service =	{	"/rpc/coordinate_manager/moveUserCoord",	0x0000E104,	"RequestMessageType.Int32List",	"ResponseMessageType.Bool",	&TpComm::handleRequest0x0000E104,	&TpComm::handleResponse0x0000E104,	Comm_Authority_TP,	};	 rpc_table_.push_back(rpc_service);
	rpc_service =	{	"/rpc/coordinate_manager/getUserCoordInfoById",	0x00004324,	"RequestMessageType.Int32",	"ResponseMessageType.Bool_UserInfo",	&TpComm::handleRequest0x00004324,	&TpComm::handleResponse0x00004324,	Comm_Authority_TP,	};	 rpc_table_.push_back(rpc_service);
	rpc_service =	{	"/rpc/coordinate_manager/getAllValidUserCoordSummaryInfo",	0x0001838F,	"RequestMessageType.Void",	"ResponseMessageType.UserSummaryList",	&TpComm::handleRequest0x0001838F,	&TpComm::handleResponse0x0001838F,	Comm_Authority_TP,	};	 rpc_table_.push_back(rpc_service);

	rpc_service =	{	"/rpc/reg_manager/r/addReg",	0x00004FF7,	"RequestMessageType.RRegData",	"ResponseMessageType.Bool",	&TpComm::handleRequest0x00004FF7,	&TpComm::handleResponse0x00004FF7,	Comm_Authority_TP,	};	 rpc_table_.push_back(rpc_service);
	rpc_service =	{	"/rpc/reg_manager/r/deleteReg",	0x000012F7,	"RequestMessageType.Int32",	"ResponseMessageType.Bool",	&TpComm::handleRequest0x000012F7,	&TpComm::handleResponse0x000012F7,	Comm_Authority_TP,	};	 rpc_table_.push_back(rpc_service);
	rpc_service =	{	"/rpc/reg_manager/r/updateReg",	0x00005757,	"RequestMessageType.RRegData",	"ResponseMessageType.Bool",	&TpComm::handleRequest0x00005757,	&TpComm::handleResponse0x00005757,	Comm_Authority_TP,	};	 rpc_table_.push_back(rpc_service);
	rpc_service =	{	"/rpc/reg_manager/r/getReg",	0x0000EAB7,	"RequestMessageType.Int32",	"ResponseMessageType.Bool_RRegData",	&TpComm::handleRequest0x0000EAB7,	&TpComm::handleResponse0x0000EAB7,	Comm_Authority_TP,	};	 rpc_table_.push_back(rpc_service);
	rpc_service =	{	"/rpc/reg_manager/r/moveReg",	0x0000C877,	"RequestMessageType.Int32List(count = 2) ",	"ResponseMessageType.Bool",	&TpComm::handleRequest0x0000C877,	&TpComm::handleResponse0x0000C877,	Comm_Authority_TP,	};	 rpc_table_.push_back(rpc_service);
	rpc_service =	{	"/rpc/reg_manager/r/getChangedList",	0x0000A904,	"RequestMessageType.Int32List(count = 2) ",	"ResponseMessageType.BaseRegSummaryList",	&TpComm::handleRequest0x0000A904,	&TpComm::handleResponse0x0000A904,	Comm_Authority_TP,	};	 rpc_table_.push_back(rpc_service);
	rpc_service =	{	"/rpc/reg_manager/r/getValidList",	0x00008CE4,	"RequestMessageType.Int32List(count = 2) ",	"ResponseMessageType.BaseRegSummaryList",	&TpComm::handleRequest0x00008CE4,	&TpComm::handleResponse0x00008CE4,	Comm_Authority_TP,	};	 rpc_table_.push_back(rpc_service);
											
	rpc_service =	{	"/rpc/reg_manager/mr/addReg",	0x000097E7,	"RequestMessageType.MrRegData",	"ResponseMessageType.Bool",	&TpComm::handleRequest0x000097E7,	&TpComm::handleResponse0x000097E7,	Comm_Authority_TP,	};	 rpc_table_.push_back(rpc_service);
	rpc_service =	{	"/rpc/reg_manager/mr/deleteReg",	0x0000E5D7,	"RequestMessageType.Int32",	"ResponseMessageType.Bool",	&TpComm::handleRequest0x0000E5D7,	&TpComm::handleResponse0x0000E5D7,	Comm_Authority_TP,	};	 rpc_table_.push_back(rpc_service);
	rpc_service =	{	"/rpc/reg_manager/mr/updateReg",	0x0000E9B7,	"RequestMessageType.MrRegData",	"ResponseMessageType.Bool",	&TpComm::handleRequest0x0000E9B7,	&TpComm::handleResponse0x0000E9B7,	Comm_Authority_TP,	};	 rpc_table_.push_back(rpc_service);
	rpc_service =	{	"/rpc/reg_manager/mr/getReg",	0x0000B507,	"RequestMessageType.Int32",	"ResponseMessageType.Bool_MrRegData",	&TpComm::handleRequest0x0000B507,	&TpComm::handleResponse0x0000B507,	Comm_Authority_TP,	};	 rpc_table_.push_back(rpc_service);
	rpc_service =	{	"/rpc/reg_manager/mr/moveReg",	0x00015BA7,	"RequestMessageType.Int32List(count = 2) ",	"ResponseMessageType.Bool",	&TpComm::handleRequest0x00015BA7,	&TpComm::handleResponse0x00015BA7,	Comm_Authority_TP,	};	 rpc_table_.push_back(rpc_service);
	rpc_service =	{	"/rpc/reg_manager/mr/getChangedList",	0x00001774,	"RequestMessageType.Int32List(count = 2) ",	"ResponseMessageType.BaseRegSummaryList",	&TpComm::handleRequest0x00001774,	&TpComm::handleResponse0x00001774,	Comm_Authority_TP,	};	 rpc_table_.push_back(rpc_service);
	rpc_service =	{	"/rpc/reg_manager/mr/getValidList",	0x00015CF4,	"RequestMessageType.Int32List(count = 2) ",	"ResponseMessageType.BaseRegSummaryList",	&TpComm::handleRequest0x00015CF4,	&TpComm::handleResponse0x00015CF4,	Comm_Authority_TP,	};	 rpc_table_.push_back(rpc_service);
											
	rpc_service =	{	"/rpc/reg_manager/sr/addReg",	0x000161E7,	"RequestMessageType.SrRegData",	"ResponseMessageType.Bool",	&TpComm::handleRequest0x000161E7,	&TpComm::handleResponse0x000161E7,	Comm_Authority_TP,	};	 rpc_table_.push_back(rpc_service);
	rpc_service =	{	"/rpc/reg_manager/sr/deleteReg",	0x0000B817,	"RequestMessageType.Int32",	"ResponseMessageType.Bool",	&TpComm::handleRequest0x0000B817,	&TpComm::handleResponse0x0000B817,	Comm_Authority_TP,	};	 rpc_table_.push_back(rpc_service);
	rpc_service =	{	"/rpc/reg_manager/sr/updateReg",	0x000119F7,	"RequestMessageType.SrRegData",	"ResponseMessageType.Bool",	&TpComm::handleRequest0x000119F7,	&TpComm::handleResponse0x000119F7,	Comm_Authority_TP,	};	 rpc_table_.push_back(rpc_service);
	rpc_service =	{	"/rpc/reg_manager/sr/getReg",	0x00017F07,	"RequestMessageType.Int32",	"ResponseMessageType.Bool_SrRegData",	&TpComm::handleRequest0x00017F07,	&TpComm::handleResponse0x00017F07,	Comm_Authority_TP,	};	 rpc_table_.push_back(rpc_service);
	rpc_service =	{	"/rpc/reg_manager/sr/moveReg",	0x00002127,	"RequestMessageType.Int32List(count = 2) ",	"ResponseMessageType.Bool",	&TpComm::handleRequest0x00002127,	&TpComm::handleResponse0x00002127,	Comm_Authority_TP,	};	 rpc_table_.push_back(rpc_service);
	rpc_service =	{	"/rpc/reg_manager/sr/getChangedList",	0x00004834,	"RequestMessageType.Int32List(count = 2) ",	"ResponseMessageType.BaseRegSummaryList",	&TpComm::handleRequest0x00004834,	&TpComm::handleResponse0x00004834,	Comm_Authority_TP,	};	 rpc_table_.push_back(rpc_service);
	rpc_service =	{	"/rpc/reg_manager/sr/getValidList",	0x00009854,	"RequestMessageType.Int32List(count = 2) ",	"ResponseMessageType.BaseRegSummaryList",	&TpComm::handleRequest0x00009854,	&TpComm::handleResponse0x00009854,	Comm_Authority_TP,	};	 rpc_table_.push_back(rpc_service);
											
	rpc_service =	{	"/rpc/reg_manager/pr/addReg",	0x000154E7,	"RequestMessageType.PrRegData",	"ResponseMessageType.Bool",	&TpComm::handleRequest0x000154E7,	&TpComm::handleResponse0x000154E7,	Comm_Authority_TP,	};	 rpc_table_.push_back(rpc_service);
	rpc_service =	{	"/rpc/reg_manager/pr/deleteReg",	0x00001097,	"RequestMessageType.Int32",	"ResponseMessageType.Bool",	&TpComm::handleRequest0x00001097,	&TpComm::handleResponse0x00001097,	Comm_Authority_TP,	};	 rpc_table_.push_back(rpc_service);
	rpc_service =	{	"/rpc/reg_manager/pr/updateReg",	0x00009EF7,	"RequestMessageType.PrRegData",	"ResponseMessageType.Bool",	&TpComm::handleRequest0x00009EF7,	&TpComm::handleResponse0x00009EF7,	Comm_Authority_TP,	};	 rpc_table_.push_back(rpc_service);
	rpc_service =	{	"/rpc/reg_manager/pr/getReg",	0x00017207,	"RequestMessageType.Int32",	"ResponseMessageType.Bool_PrRegData",	&TpComm::handleRequest0x00017207,	&TpComm::handleResponse0x00017207,	Comm_Authority_TP,	};	 rpc_table_.push_back(rpc_service);
	rpc_service =	{	"/rpc/reg_manager/pr/moveReg",	0x0000D7C7,	"RequestMessageType.Int32List(count = 2) ",	"ResponseMessageType.Bool",	&TpComm::handleRequest0x0000D7C7,	&TpComm::handleResponse0x0000D7C7,	Comm_Authority_TP,	};	 rpc_table_.push_back(rpc_service);
	rpc_service =	{	"/rpc/reg_manager/pr/getChangedList",	0x0000B454,	"RequestMessageType.Int32List(count = 2) ",	"ResponseMessageType.BaseRegSummaryList",	&TpComm::handleRequest0x0000B454,	&TpComm::handleResponse0x0000B454,	Comm_Authority_TP,	};	 rpc_table_.push_back(rpc_service);
	rpc_service =	{	"/rpc/reg_manager/pr/getValidList",	0x00009354,	"RequestMessageType.Int32List(count = 2) ",	"ResponseMessageType.BaseRegSummaryList",	&TpComm::handleRequest0x00009354,	&TpComm::handleResponse0x00009354,	Comm_Authority_TP,	};	 rpc_table_.push_back(rpc_service);
											
	rpc_service =	{	"/rpc/reg_manager/hr/addReg",	0x00016CE7,	"RequestMessageType.HrRegData",	"ResponseMessageType.Bool",	&TpComm::handleRequest0x00016CE7,	&TpComm::handleResponse0x00016CE7,	Comm_Authority_TP,	};	 rpc_table_.push_back(rpc_service);
	rpc_service =	{	"/rpc/reg_manager/hr/deleteReg",	0x00003D17,	"RequestMessageType.Int32",	"ResponseMessageType.Bool",	&TpComm::handleRequest0x00003D17,	&TpComm::handleResponse0x00003D17,	Comm_Authority_TP,	};	 rpc_table_.push_back(rpc_service);
	rpc_service =	{	"/rpc/reg_manager/hr/updateReg",	0x0000CB77,	"RequestMessageType.HrRegData",	"ResponseMessageType.Bool",	&TpComm::handleRequest0x0000CB77,	&TpComm::handleResponse0x0000CB77,	Comm_Authority_TP,	};	 rpc_table_.push_back(rpc_service);
	rpc_service =	{	"/rpc/reg_manager/hr/getReg",	0x00000367,	"RequestMessageType.Int32",	"ResponseMessageType.Bool_HrRegData",	&TpComm::handleRequest0x00000367,	&TpComm::handleResponse0x00000367,	Comm_Authority_TP,	};	 rpc_table_.push_back(rpc_service);
	rpc_service =	{	"/rpc/reg_manager/hr/moveReg",	0x00014A87,	"RequestMessageType.Int32List(count = 2) ",	"ResponseMessageType.Bool",	&TpComm::handleRequest0x00014A87,	&TpComm::handleResponse0x00014A87,	Comm_Authority_TP,	};	 rpc_table_.push_back(rpc_service);
	rpc_service =	{	"/rpc/reg_manager/hr/getChangedList",	0x00012974,	"RequestMessageType.Int32List(count = 2) ",	"ResponseMessageType.BaseRegSummaryList",	&TpComm::handleRequest0x00012974,	&TpComm::handleResponse0x00012974,	Comm_Authority_TP,	};	 rpc_table_.push_back(rpc_service);
	rpc_service =	{	"/rpc/reg_manager/hr/getValidList",	0x00006B54,	"RequestMessageType.Int32List(count = 2) ",	"ResponseMessageType.BaseRegSummaryList",	&TpComm::handleRequest0x00006B54,	&TpComm::handleResponse0x00006B54,	Comm_Authority_TP,	};	 rpc_table_.push_back(rpc_service);

	rpc_service =	{	"/rpc/motion_control/stop",	0x00001E70,	"RequestMessageType.Void",	"ResponseMessageType.Bool",	&TpComm::handleRequest0x00001E70,	&TpComm::handleResponse0x00001E70,	Comm_Authority_TP,	};	 rpc_table_.push_back(rpc_service);
	rpc_service =	{	"/rpc/motion_control/reset",	0x00001D14,	"RequestMessageType.Void",	"ResponseMessageType.Bool",	&TpComm::handleRequest0x00001D14,	&TpComm::handleResponse0x00001D14,	Comm_Authority_TP,	};	 rpc_table_.push_back(rpc_service);
	rpc_service =	{	"/rpc/motion_control/axis_group/setManualFrame",	0x00009D05,	"RequestMessageType.Int32List(count = 2) ",	"ResponseMessageType.Bool",	&TpComm::handleRequest0x00009D05,	&TpComm::handleResponse0x00009D05,	Comm_Authority_TP,	};	 rpc_table_.push_back(rpc_service);
	rpc_service =	{	"/rpc/motion_control/axis_group/doStepManualMove",	0x000085D5,	"RequestMessageType.Int32_Int32List(count = 9) ",	"ResponseMessageType.Bool",	&TpComm::handleRequest0x000085D5,	&TpComm::handleResponse0x000085D5,	Comm_Authority_TP,	};	 rpc_table_.push_back(rpc_service);
	rpc_service =	{	"/rpc/motion_control/axis_group/doContinuousManualMove",	0x0000D3F5,	"RequestMessageType.Int32_Int32List(count = 9)",	"ResponseMessageType.Bool",	&TpComm::handleRequest0x0000D3F5,	&TpComm::handleResponse0x0000D3F5,	Comm_Authority_TP,	};	 rpc_table_.push_back(rpc_service);
	rpc_service =	{	"/rpc/motion_control/axis_group/getJointsFeedBack",	0x0000DFBB,	"RequestMessageType.Int32",	"ResponseMessageType.Bool_DoubleList(count = 9) ",	&TpComm::handleRequest0x0000DFBB,	&TpComm::handleResponse0x0000DFBB,	Comm_Authority_TP,	};	 rpc_table_.push_back(rpc_service);
	rpc_service =	{	"/rpc/motion_control/axis_group/setUserSoftLimit",	0x000114A4,	"RequestMessageType.Int32_JointLimit",	"ResponseMessageType.Bool",	&TpComm::handleRequest0x000114A4,	&TpComm::handleResponse0x000114A4,	Comm_Authority_TP,	};	 rpc_table_.push_back(rpc_service);
	rpc_service =	{	"/rpc/motion_control/axis_group/getUserSoftLimit",	0x0000C764,	"RequestMessageType.Int32",	"ResponseMessageType.Bool_JointLimit",	&TpComm::handleRequest0x0000C764,	&TpComm::handleResponse0x0000C764,	Comm_Authority_TP,	};	 rpc_table_.push_back(rpc_service);
	rpc_service =	{	"/rpc/motion_control/axis_group/setManuSoftLimit",	0x000108E4,	"RequestMessageType.Int32_JointLimit",	"ResponseMessageType.Bool",	&TpComm::handleRequest0x000108E4,	&TpComm::handleResponse0x000108E4,	Comm_Authority_ADMINISTRATOR,	};	 rpc_table_.push_back(rpc_service);
	rpc_service =	{	"/rpc/motion_control/axis_group/getManuSoftLimit",	0x0000C244,	"RequestMessageType.Int32",	"ResponseMessageType.Bool_JointLimit",	&TpComm::handleRequest0x0000C244,	&TpComm::handleResponse0x0000C244,	Comm_Authority_TP,	};	 rpc_table_.push_back(rpc_service);
	rpc_service =	{	"/rpc/motion_control/axis_group/doGotoCartesianPointManualMove",	0x00010C05,	"RequestMessageType.Int32_DoubleList(count = 6) ",	"ResponseMessageType.Bool",	&TpComm::handleRequest0x00010C05,	&TpComm::handleResponse0x00010C05,	Comm_Authority_TP,	};	 rpc_table_.push_back(rpc_service);
	rpc_service =	{	"/rpc/motion_control/axis_group/doGotoJointPointManualMove",	0x00008075,	"RequestMessageType.Int32_DoubleList(count = 9) ",	"ResponseMessageType.Bool",	&TpComm::handleRequest0x00008075,	&TpComm::handleResponse0x00008075,	Comm_Authority_TP,	};	 rpc_table_.push_back(rpc_service);
	rpc_service =	{	"/rpc/motion_control/axis_group/doManualStop",	0x0000A9A0,	"RequestMessageType.Int32",	"ResponseMessageType.Bool",	&TpComm::handleRequest0x0000A9A0,	&TpComm::handleResponse0x0000A9A0,	Comm_Authority_TP,	};	 rpc_table_.push_back(rpc_service);

	rpc_service =	{	"/rpc/interpreter/start",	0x00006154,	"RequestMessageType.String",	"ResponseMessageType.Bool",	&TpComm::handleRequest0x00006154,	&TpComm::handleResponse0x00006154,	Comm_Authority_TP,	};	 rpc_table_.push_back(rpc_service);
	rpc_service =	{	"/rpc/interpreter/debug",	0x000102D7,	"RequestMessageType.String",	"ResponseMessageType.Bool",	&TpComm::handleRequest0x000102D7,	&TpComm::handleResponse0x000102D7,	Comm_Authority_TP,	};	 rpc_table_.push_back(rpc_service);
	rpc_service =	{	"/rpc/interpreter/forward",	0x0000D974,	"RequestMessageType.Void",	"ResponseMessageType.Bool",	&TpComm::handleRequest0x0000D974,	&TpComm::handleResponse0x0000D974,	Comm_Authority_TP,	};	 rpc_table_.push_back(rpc_service);
	rpc_service =	{	"/rpc/interpreter/backward",	0x00008E74,	"RequestMessageType.Void",	"ResponseMessageType.Bool",	&TpComm::handleRequest0x00008E74,	&TpComm::handleResponse0x00008E74,	Comm_Authority_TP,	};	 rpc_table_.push_back(rpc_service);
	rpc_service =	{	"/rpc/interpreter/jump",	0x00015930,	"RequestMessageType.Int32",	"ResponseMessageType.Bool",	&TpComm::handleRequest0x00015930,	&TpComm::handleResponse0x00015930,	Comm_Authority_TP,	};	 rpc_table_.push_back(rpc_service);
	rpc_service =	{	"/rpc/interpreter/pause",	0x0000BA55,	"RequestMessageType.Void",	"ResponseMessageType.Bool",	&TpComm::handleRequest0x0000BA55,	&TpComm::handleResponse0x0000BA55,	Comm_Authority_TP,	};	 rpc_table_.push_back(rpc_service);
	rpc_service =	{	"/rpc/interpreter/resume",	0x0000CF55,	"RequestMessageType.Void",	"ResponseMessageType.Bool",	&TpComm::handleRequest0x0000CF55,	&TpComm::handleResponse0x0000CF55,	Comm_Authority_TP,	};	 rpc_table_.push_back(rpc_service);
	rpc_service =	{	"/rpc/interpreter/abort",	0x000086F4,	"RequestMessageType.Void",	"ResponseMessageType.Bool",	&TpComm::handleRequest0x000086F4,	&TpComm::handleResponse0x000086F4,	Comm_Authority_TP,	};	 rpc_table_.push_back(rpc_service);

	rpc_service =	{	"/rpc/io_mapping/getDIByBit",	0x000050B4,	"RequestMessageType.Int32",	"ResponseMessageType.Bool_Int32",	&TpComm::handleRequest0x000050B4,	&TpComm::handleResponse0x000050B4,	Comm_Authority_TP,	};	 rpc_table_.push_back(rpc_service);
	rpc_service =	{	"/rpc/io_mapping/setDIByBit",	0x00011754,	"RequestMessageType.Int32List(count = 2) ",	"ResponseMessageType.Bool",	&TpComm::handleRequest0x00011754,	&TpComm::handleResponse0x00011754,	Comm_Authority_TP,	};	 rpc_table_.push_back(rpc_service);
	rpc_service =	{	"/rpc/io_mapping/getDOByBit",	0x00013074,	"RequestMessageType.Int32",	"ResponseMessageType.Bool_Int32",	&TpComm::handleRequest0x00013074,	&TpComm::handleResponse0x00013074,	Comm_Authority_TP,	};	 rpc_table_.push_back(rpc_service);
	rpc_service =	{	"/rpc/io_mapping/setDOByBit",	0x00007074,	"RequestMessageType.Int32List(count = 2) ",	"ResponseMessageType.Bool",	&TpComm::handleRequest0x00007074,	&TpComm::handleResponse0x00007074,	Comm_Authority_TP,	};	 rpc_table_.push_back(rpc_service);

	rpc_service =	{	"/rpc/device_manager/getDeviceList",	0x0000C1E0,	"RequestMessageType.Void",	"ResponseMessageType.DeviceInfoList",	&TpComm::handleRequest0x0000C1E0,	&TpComm::handleResponse0x0000C1E0,	Comm_Authority_TP,	};	 rpc_table_.push_back(rpc_service);
}

void TpComm::initPublishElementTable()
{
    PublishService publish_service;
	publish_service =	{	"/publish/controller/UserOpMode",	0x00015255,	"MessageType.Int32",	&TpComm::handlePublishElement0x00015255,	Comm_Authority_TP,	};	publish_element_table_.push_back(publish_service); 
	publish_service =	{	"/publish/controller/RunningStatus",	0x00001F33,	"MessageType.Int32",	&TpComm::handlePublishElement0x00001F33,	Comm_Authority_TP,	};	publish_element_table_.push_back(publish_service); 
	publish_service =	{	"/publish/controller/InterpreterStatus",	0x00003203,	"MessageType.Int32",	&TpComm::handlePublishElement0x00003203,	Comm_Authority_TP,	};	publish_element_table_.push_back(publish_service); 
	publish_service =	{	"/publish/controller/RobotStatus",	0x00012943,	"MessageType.Int32",	&TpComm::handlePublishElement0x00012943,	Comm_Authority_TP,	};	publish_element_table_.push_back(publish_service); 
	publish_service =	{	"/publish/controller/CtrlStatus",	0x0000E8E3,	"MessageType.Int32",	&TpComm::handlePublishElement0x0000E8E3,	Comm_Authority_TP,	};	publish_element_table_.push_back(publish_service); 
	publish_service =	{	"/publish/controller/ServoStatus",	0x00002053,	"MessageType.Int32",	&TpComm::handlePublishElement0x00002053,	Comm_Authority_TP,	};	publish_element_table_.push_back(publish_service); 
	publish_service =	{	"/publish/controller/SafetyAlarm",	0x0000D0AD,	"MessageType.Int32",	&TpComm::handlePublishElement0x0000D0AD,	Comm_Authority_TP,	};	publish_element_table_.push_back(publish_service); 
	publish_service =	{	"/publish/motion_control/axis_group/feedback/joints",	0x000161F3,	"MessageType.Int32_Int32List(count=9)",	&TpComm::handlePublishElement0x000161F3,	Comm_Authority_TP,	};	publish_element_table_.push_back(publish_service); 

}