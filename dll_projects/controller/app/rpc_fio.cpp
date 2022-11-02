#include "rpc_interface.h"
#include "rpc_basic.h"
#include "common_error_code.h"
#include "protoc.h"


uint64_t c_sendFioCmdValue(uint32_t opcode_type, uint32_t set_value, uint32_t ret_data[2])
{
	if (!rpc_valid)
		return HANDLE_RPC_FAILED;
	RpcBasic* rpc_ptr = RpcBasic::getInstance();
	RequestMessageType_Uint32List req_data;
	ResponseMessageType_Uint32List rep_data;
	req_data.header.time_stamp = 122;
	req_data.property.authority = Comm_Authority_TP_SIMMULATOR;

	req_data.data.data_count = 2;
	req_data.data.data[0] = opcode_type;
	req_data.data.data[1] = set_value;

	if (!rpc_ptr->handleRpc(0x0000175B, &req_data, RequestMessageType_Uint32List_fields, &rep_data, ResponseMessageType_Uint32List_fields))
	{
		return HANDLE_RPC_FAILED;
	}
	ret_data[0] = rep_data.data.data[0];

	return rep_data.header.error_code;
}




