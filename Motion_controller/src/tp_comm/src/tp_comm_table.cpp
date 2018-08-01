#include "tp_comm.h"


void TpComm::initRpcTable()
{
    RpcService rpc_service;
	rpc_service =	{	"/rpc/controller/getWorkStatus",	0x0000F933,	"RequestMessageType_Void",	"ResponseMessageType_Int32",	&TpComm::handleRequest0x0000F933,	&TpComm::handleResponse0x0000F933,	Comm_Authority_TP,	};	 rpc_table_.push_back(rpc_service);
	rpc_service =	{	"/rpc/controller/getCtrlStatus",	0x0000E9D3,	"RequestMessageType_Void",	"ResponseMessageType_Int32",	&TpComm::handleRequest0x0000E9D3,	&TpComm::handleResponse0x0000E9D3,	Comm_Authority_TP,	};	 rpc_table_.push_back(rpc_service);
	rpc_service =	{	"/rpc/controller/getRunningStatus",	0x00000AB3,	"RequestMessageType_Void",	"ResponseMessageType_Int32",	&TpComm::handleRequest0x00000AB3,	&TpComm::handleResponse0x00000AB3,	Comm_Authority_TP,	};	 rpc_table_.push_back(rpc_service);
	rpc_service =	{	"/rpc/controller/getUserOperationMode",	0x00010945,	"RequestMessageType_Void",	"ResponseMessageType_Int32",	&TpComm::handleRequest0x00010945,	&TpComm::handleResponse0x00010945,	Comm_Authority_TP,	};	 rpc_table_.push_back(rpc_service);
	rpc_service =	{	"/rpc/controller/setRobotStateCmd",	0x000067A4,	"RequestMessageType_Int32",	"ResponseMessageType_Bool",	&TpComm::handleRequest0x000067A4,	&TpComm::handleResponse0x000067A4,	Comm_Authority_TP,	};	 rpc_table_.push_back(rpc_service);
	rpc_service =	{	"/rpc/controller/setUserOperationMode",	0x00010685,	"RequestMessageType_Int32",	"ResponseMessageType_Bool",	&TpComm::handleRequest0x00010685,	&TpComm::handleResponse0x00010685,	Comm_Authority_TP,	};	 rpc_table_.push_back(rpc_service);
	//rpc_service =	{	"/rpc/controller/getWarnings",	0x00010AB3,	"RequestMessageType_Void",	"ResponseMessageType_String",	&TpComm::handleRequest0x00010AB3,	&TpComm::handleResponse0x00010AB3,	Comm_Authority_TP,	};	 rpc_table_.push_back(rpc_service);
	rpc_service =	{	"/rpc/controller/getLocalTime",	0x00017C25,	"RequestMessageType_Void",	"ResponseMessageType_UnsignedInt64",	&TpComm::handleRequest0x00017C25,	&TpComm::handleResponse0x00017C25,	Comm_Authority_TP,	};	 rpc_table_.push_back(rpc_service);
	//rpc_service =	{	"/rpc/controller/getVersion",	0x000093EE,	"RequestMessageType_Void",	"ResponseMessageType_Int32List",	&TpComm::handleRequest0x000093EE,	&TpComm::handleResponse0x000093EE,	Comm_Authority_TP,	};	 rpc_table_.push_back(rpc_service);
	//rpc_service =	{	"/rpc/tpComm/getPublishTable",	0x0000FC15,	"RequestMessageType_Void",	"ResponseMessageType_PublishTable",	&TpComm::handleRequest0x0000FC15,	&TpComm::handleResponse0x0000FC15,	Comm_Authority_TP,	};	 rpc_table_.push_back(rpc_service);
	//rpc_service =	{	"/rpc/tpComm/addTopic",	0x00013EA3,	"RequestMessageType_Topic",	"ResponseMessageType_Bool",	&TpComm::handleRequest0x00013EA3,	&TpComm::handleResponse0x00013EA3,	Comm_Authority_TP,	};	 rpc_table_.push_back(rpc_service);
	rpc_service =	{	"/rpc/tpComm/getRpcTable",	0x00009BC5,	"RequestMessageType_Void",	"ResponseMessageType_RpcTable",	&TpComm::handleRequest0x00009BC5,	&TpComm::handleResponse0x00009BC5,	Comm_Authority_TP,	};	 rpc_table_.push_back(rpc_service);

	rpc_service =	{	"/rpc/register/addR",	0x00007CF2,	"RequestMessageType_RegisterR",	"ResponseMessageType_Bool",	&TpComm::handleRequest0x00007CF2,	&TpComm::handleResponse0x00007CF2,	Comm_Authority_TP,	};	 rpc_table_.push_back(rpc_service);
	rpc_service =	{	"/rpc/register/updateR",	0x000031B2,	"RequestMessageType_RegisterR",	"ResponseMessageType_Bool",	&TpComm::handleRequest0x000031B2,	&TpComm::handleResponse0x000031B2,	Comm_Authority_TP,	};	 rpc_table_.push_back(rpc_service);
	rpc_service =	{	"/rpc/register/deleteR",	0x00013062,	"RequestMessageType_UnsignedInt32",	"ResponseMessageType_Bool",	&TpComm::handleRequest0x00013062,	&TpComm::handleResponse0x00013062,	Comm_Authority_TP,	};	 rpc_table_.push_back(rpc_service);
	rpc_service =	{	"/rpc/register/getR",	0x000116F2,	"RequestMessageType_UnsignedInt32",	"ResponseMessageType_RegisterR",	&TpComm::handleRequest0x000116F2,	&TpComm::handleResponse0x000116F2,	Comm_Authority_TP,	};	 rpc_table_.push_back(rpc_service);
	rpc_service =	{	"/rpc/register/setActivateR",	0x000098C2,	"RequestMessageType_UnsignedInt32",	"ResponseMessageType_Bool",	&TpComm::handleRequest0x000098C2,	&TpComm::handleResponse0x000098C2,	Comm_Authority_TP,	};	 rpc_table_.push_back(rpc_service);
	rpc_service =	{	"/rpc/register/getActivateR",	0x00005602,	"RequestMessageType_Void",	"ResponseMessageType_UnsignedInt32",	&TpComm::handleRequest0x00005602,	&TpComm::handleResponse0x00005602,	Comm_Authority_TP,	};	 rpc_table_.push_back(rpc_service);
											
	rpc_service =	{	"/rpc/register/addPR",	0x0000F862,	"RequestMessageType_RegisterPR",	"ResponseMessageType_Bool",	&TpComm::handleRequest0x0000F862,	&TpComm::handleResponse0x0000F862,	Comm_Authority_TP,	};	 rpc_table_.push_back(rpc_service);
	rpc_service =	{	"/rpc/register/updatePR",	0x0000C032,	"RequestMessageType_RegisterPR",	"ResponseMessageType_Bool",	&TpComm::handleRequest0x0000C032,	&TpComm::handleResponse0x0000C032,	Comm_Authority_TP,	};	 rpc_table_.push_back(rpc_service);
	rpc_service =	{	"/rpc/register/deletePR",	0x00005392,	"RequestMessageType_UnsignedInt32",	"ResponseMessageType_Bool",	&TpComm::handleRequest0x00005392,	&TpComm::handleResponse0x00005392,	Comm_Authority_TP,	};	 rpc_table_.push_back(rpc_service);
	rpc_service =	{	"/rpc/register/getPR",	0x000170A2,	"RequestMessageType_UnsignedInt32",	"RespnseMessageType_RegisterPR",	&TpComm::handleRequest0x000170A2,	&TpComm::handleResponse0x000170A2,	Comm_Authority_TP,	};	 rpc_table_.push_back(rpc_service);
	rpc_service =	{	"/rpc/register/setActivatePR",	0x000116B2,	"RequestMessageType_UnsignedInt32",	"ResponseMessageType_Bool",	&TpComm::handleRequest0x000116B2,	&TpComm::handleResponse0x000116B2,	Comm_Authority_TP,	};	 rpc_table_.push_back(rpc_service);
	rpc_service =	{	"/rpc/register/getActivatePR",	0x0000F402,	"RequestMessageType_Void",	"ResponseMessageType_UnsignedInt32",	&TpComm::handleRequest0x0000F402,	&TpComm::handleResponse0x0000F402,	Comm_Authority_TP,	};	 rpc_table_.push_back(rpc_service);
											
	rpc_service =	{	"/rpc/register/addMR",	0x0000F892,	"RequestMessageType_RegisterMR",	"ResponseMessageType_Bool",	&TpComm::handleRequest0x0000F892,	&TpComm::handleResponse0x0000F892,	Comm_Authority_TP,	};	 rpc_table_.push_back(rpc_service);
	rpc_service =	{	"/rpc/register/updateMR",	0x0000C1C2,	"RequestMessageType_RegisterMR",	"ResponseMessageType_Bool",	&TpComm::handleRequest0x0000C1C2,	&TpComm::handleResponse0x0000C1C2,	Comm_Authority_TP,	};	 rpc_table_.push_back(rpc_service);
	rpc_service =	{	"/rpc/register/deleteMR",	0x000053E2,	"RequestMessageType_UnsignedInt32",	"ResponseMessageType_Bool",	&TpComm::handleRequest0x000053E2,	&TpComm::handleResponse0x000053E2,	Comm_Authority_TP,	};	 rpc_table_.push_back(rpc_service);
	rpc_service =	{	"/rpc/register/getMR",	0x000170D2,	"RequestMessageType_UnsignedInt32",	"RespnseMessageType_RegisterMR",	&TpComm::handleRequest0x000170D2,	&TpComm::handleResponse0x000170D2,	Comm_Authority_TP,	};	 rpc_table_.push_back(rpc_service);
	rpc_service =	{	"/rpc/register/setActivateMR",	0x00011642,	"RequestMessageType_UnsignedInt32",	"ResponseMessageType_Bool",	&TpComm::handleRequest0x00011642,	&TpComm::handleResponse0x00011642,	Comm_Authority_TP,	};	 rpc_table_.push_back(rpc_service);
	rpc_service =	{	"/rpc/register/getActivateMR",	0x0000F3B2,	"RequestMessageType_Void",	"ResponseMessageType_UnsignedInt32",	&TpComm::handleRequest0x0000F3B2,	&TpComm::handleResponse0x0000F3B2,	Comm_Authority_TP,	};	 rpc_table_.push_back(rpc_service);
											
	rpc_service =	{	"/rpc/register/addSR",	0x0000F932,	"RequestMessageType_RegisterSR",	"ResponseMessageType_Bool",	&TpComm::handleRequest0x0000F932,	&TpComm::handleResponse0x0000F932,	Comm_Authority_TP,	};	 rpc_table_.push_back(rpc_service);
	rpc_service =	{	"/rpc/register/updateSR",	0x0000BF62,	"RequestMessageType_RegisterSR",	"ResponseMessageType_Bool",	&TpComm::handleRequest0x0000BF62,	&TpComm::handleResponse0x0000BF62,	Comm_Authority_TP,	};	 rpc_table_.push_back(rpc_service);
	rpc_service =	{	"/rpc/register/deleteSR",	0x00005342,	"RequestMessageType_UnsignedInt32",	"ResponseMessageType_Bool",	&TpComm::handleRequest0x00005342,	&TpComm::handleResponse0x00005342,	Comm_Authority_TP,	};	 rpc_table_.push_back(rpc_service);
	rpc_service =	{	"/rpc/register/getSR",	0x00017172,	"RequestMessageType_UnsignedInt32",	"RespnseMessageType_RegisterSR",	&TpComm::handleRequest0x00017172,	&TpComm::handleResponse0x00017172,	Comm_Authority_TP,	};	 rpc_table_.push_back(rpc_service);
	rpc_service =	{	"/rpc/register/setActivateSR",	0x000115E2,	"RequestMessageType_UnsignedInt32",	"ResponseMessageType_Bool",	&TpComm::handleRequest0x000115E2,	&TpComm::handleResponse0x000115E2,	Comm_Authority_TP,	};	 rpc_table_.push_back(rpc_service);
	rpc_service =	{	"/rpc/register/getActivateSR",	0x0000F352,	"RequestMessageType_Void",	"ResponseMessageType_UnsignedInt32",	&TpComm::handleRequest0x0000F352,	&TpComm::handleResponse0x0000F352,	Comm_Authority_TP,	};	 rpc_table_.push_back(rpc_service);

}

void TpComm::initPublishElementTable()
{
    PublishService publish_service;
	publish_service =	{	"/publish/controller/workStatus",	0x00011423,	"MessageType_Int32",	&TpComm::handlePublishElement0x00011423,	Comm_Authority_ADMINISTRATOR,	};	publish_element_table_.push_back(publish_service); 
	publish_service =	{	"/publish/controller/ctrlStatus",	0x00010363,	"MessageType_Int32",	&TpComm::handlePublishElement0x00010363,	Comm_Authority_TP,	};	publish_element_table_.push_back(publish_service); 
	publish_service =	{	"/publish/controller/runningStatus",	0x00001F13,	"MessageType_Int32",	&TpComm::handlePublishElement0x00001F13,	Comm_Authority_TP,	};	publish_element_table_.push_back(publish_service); 
	publish_service =	{	"/publish/controller/userOperationMode",	0x0000D175,	"MessageType_Int32",	&TpComm::handlePublishElement0x0000D175,	Comm_Authority_TP,	};	publish_element_table_.push_back(publish_service); 
	publish_service =	{	"/publish/controller/warnings",	0x00015453,	"MessageType_String",	&TpComm::handlePublishElement0x00015453,	Comm_Authority_TP,	};	publish_element_table_.push_back(publish_service); 
	publish_service =	{	"/publish/controller/localTime",	0x0000AB25,	"MessageType_Int32",	&TpComm::handlePublishElement0x0000AB25,	Comm_Authority_TP,	};	publish_element_table_.push_back(publish_service); 

}