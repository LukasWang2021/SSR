#include "tp_comm_component.h"





void TpCommComponent::initRpcTable()
{
    RpcService rpc_service;
    rpc_service = {"/tp_comm/test_request", 0xcf0be243, "comm_RequestMessageTypeInt32", "comm_ResponseMessageTypeInt32", &TpCommComponent::handleRequest0xcf0be243, &TpCommComponent::handleResponse0xcf0be243}; rpc_table_.push_back(rpc_service);
    rpc_service =	{	"/rpc/controller/getWorkStatus",	0x0000F933,	"RequestMessageType_Void",	"ResponseMessageType_Int32",	 &TpCommComponent::handleRequest0x0000F933,	 &TpCommComponent::handleResponse0x0000F933,	};	 rpc_table_.push_back(rpc_service);
    rpc_service =	{	"/rpc/controller/getCtrlStatus",	0x0000E9D3,	"RequestMessageType_Void",	"ResponseMessageType_Int32",	 &TpCommComponent::handleRequest0x0000E9D3,	 &TpCommComponent::handleResponse0x0000E9D3,	};	 rpc_table_.push_back(rpc_service);
    rpc_service =	{	"/rpc/controller/getRunningStatus",	0x00000AB3,	"RequestMessageType_Void",	"ResponseMessageType_Int32",	 &TpCommComponent::handleRequest0x00000AB3,	 &TpCommComponent::handleResponse0x00000AB3,	};	 rpc_table_.push_back(rpc_service);
    rpc_service =	{	"/rpc/controller/getUserOperationMode",	0x00010945,	"RequestMessageType_Void",	"ResponseMessageType_Int32",	 &TpCommComponent::handleRequest0x00010945,	 &TpCommComponent::handleResponse0x00010945,	};	 rpc_table_.push_back(rpc_service);
    rpc_service =	{	"/rpc/controller/setRobotStateCmd",	0x000067A4,	"RequestMessageType_Int32",	"ResponseMessageType_Bool",	 &TpCommComponent::handleRequest0x000067A4,	 &TpCommComponent::handleResponse0x000067A4,	};	 rpc_table_.push_back(rpc_service);
    rpc_service =	{	"/rpc/controller/setUserOperationMode",	0x00010685,	"RequestMessageType_Int32",	"ResponseMessageType_Bool",	 &TpCommComponent::handleRequest0x00010685,	 &TpCommComponent::handleResponse0x00010685,	};	 rpc_table_.push_back(rpc_service);
    rpc_service =	{	"/rpc/controller/getWarnings",	0x00010AB3,	"RequestMessageType_Void",	"ResponseMessageType_String",	 &TpCommComponent::handleRequest0x00010AB3,	 &TpCommComponent::handleResponse0x00010AB3,	};	 rpc_table_.push_back(rpc_service);
    rpc_service =	{	"/rpc/controller/getLocalTime",	0x00017C25,	"RequestMessageType_Void",	"ResponseMessageType_Int32",	 &TpCommComponent::handleRequest0x00017C25,	 &TpCommComponent::handleResponse0x00017C25,	};	 rpc_table_.push_back(rpc_service);
    rpc_service =	{	"/rpc/controller/getVersion",	0x000093EE,	"RequestMessageType_Void",	"ResponseMessageType_Int32List",	 &TpCommComponent::handleRequest0x000093EE,	 &TpCommComponent::handleResponse0x000093EE,	};	 rpc_table_.push_back(rpc_service);

}

void TpCommComponent::initPublishElementTable()
{
    PublishService publish_service;
    publish_service = {"/comm/basetype/int32", 0x7622aa34, "comm_MessageTypeInt32", &TpCommComponent::handlePublishElement0x7622aa34}; publish_element_table_.push_back(publish_service);
    publish_service =	{	"/publish/controller/workStatus",	0x00011423,	"MessageType_Int32",	&TpCommComponent::handlePublishElement0x00011423,	};	publish_element_table_.push_back(publish_service); 
    publish_service =	{	"/publish/controller/ctrlStatus",	0x00010363,	"MessageType_Int32",	&TpCommComponent::handlePublishElement0x00010363,	};	publish_element_table_.push_back(publish_service); 
    publish_service =	{	"/publish/controller/runningStatus",	0x00001F13,	"MessageType_Int32",	&TpCommComponent::handlePublishElement0x00001F13,	};	publish_element_table_.push_back(publish_service); 
    publish_service =	{	"/publish/controller/userOperationMode",	0x0000D175,	"MessageType_Int32",	&TpCommComponent::handlePublishElement0x0000D175,	};	publish_element_table_.push_back(publish_service); 
    publish_service =	{	"/publish/controller/warnings",	0x00015453,	"MessageType_String",	&TpCommComponent::handlePublishElement0x00015453,	};	publish_element_table_.push_back(publish_service); 
    publish_service =	{	"/publish/controller/localTime",	0x0000AB25,	"MessageType_Int32",	&TpCommComponent::handlePublishElement0x0000AB25,	};	publish_element_table_.push_back(publish_service); 

}


