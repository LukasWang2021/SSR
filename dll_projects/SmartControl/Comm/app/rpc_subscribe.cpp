//#include "stdafx.h"
#include "rpc_interface.h"
#include "comm_def.h"
#include "rpc_basic.h"
#include "common_error_code.h"

uint64_t c_addTopic()
{
	RpcBasic* rpc_ptr = RpcBasic::getInstance();
	RequestMessageType_Topic req_data;
	ResponseMessageType_Uint64 rep_data;

	req_data.data.topic_hash = TOPIC_HASH;
	req_data.header.time_stamp = 122;
	req_data.property.authority = Comm_Authority_TP_SIMMULATOR;
	req_data.data.time_max = 100;//update if time reach
	req_data.data.time_min = 100;//update if element changed
	req_data.data.element_hash_list_count = TOPIC_ELEM_NUM;
	req_data.data.element_hash_list[0] = 0x0001715B;	// axis_feedback
	req_data.data.element_hash_list[1] = 0x0001128B;	// servo_feedback
	req_data.data.element_hash_list[2] = 0x00012FFB;	// cpu_feedback
	req_data.data.element_hash_list[2] = 0x00013C8B;	// io1000_feedback
	req_data.data.element_hash_list[2] = 0x0001472B;	// iosafety_feedback

	if (!rpc_ptr->handleRpc(0x000050E3, &req_data, RequestMessageType_Topic_fields, &rep_data, ResponseMessageType_Uint64_fields))
	{
		return HANDLE_RPC_FAILED;
	}

	return rep_data.data.data;
}

uint64_t c_deleteTopic()
{
	RpcBasic* rpc_ptr = RpcBasic::getInstance();
	RequestMessageType_Uint32 req_data;
	ResponseMessageType_Uint64 rep_data;

	req_data.header.time_stamp = 122;
	req_data.property.authority = Comm_Authority_TP_SIMMULATOR;
	req_data.data.data = TOPIC_HASH;

	if (!rpc_ptr->handleRpc(0x00004403, &req_data, RequestMessageType_Uint32_fields, &rep_data, ResponseMessageType_Uint64_fields))
	{
		return HANDLE_RPC_FAILED;
	}

	return rep_data.data.data;
}
