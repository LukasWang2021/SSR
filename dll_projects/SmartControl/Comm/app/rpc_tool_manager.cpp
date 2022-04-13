#include "rpc_interface.h"
#include "rpc_basic.h"
#include "common_error_code.h"
#include "protoc.h"

uint64_t c_tfUploadToolFrame(int32_t id, double* x, double* y, double* z, double* a, double* b, double *c, char* name, char* comment, int32_t* group_id)
{
	if (!rpc_valid)
		return HANDLE_RPC_FAILED;
	RpcBasic* rpc_ptr = RpcBasic::getInstance();
	RequestMessageType_Int32 req_data;
	ResponseMessageType_Uint64_ToolInfo rep_data;

	req_data.data.data = id;
	if (!rpc_ptr->handleRpc(0x00009E34, &req_data, RequestMessageType_Int32_fields, &rep_data, ResponseMessageType_Uint64_ToolInfo_fields))
	{
		return HANDLE_RPC_FAILED;
	}

	if (rep_data.data.data.data_count != 6)
		return HANDLE_RPC_FAILED;
	*x = rep_data.data.data.data[0];
	*y = rep_data.data.data.data[1];
	*z = rep_data.data.data.data[2];
	*a = rep_data.data.data.data[3];
	*b = rep_data.data.data.data[4];
	*c = rep_data.data.data.data[5];
	memcpy(name, rep_data.data.name, strlen(rep_data.data.name));
	name[strlen(rep_data.data.name)] = '\0';
	memcpy(comment, rep_data.data.comment, strlen(rep_data.data.comment));
	comment[strlen(rep_data.data.comment)] = '\0';
	*group_id = rep_data.data.group_id;

	return rep_data.error_code.data;
}

uint64_t c_tfDownloadToolFrame(int32_t id, double x, double y, double z, double a, double b, double c, char* name, char* comment, int32_t group_id)
{
	if (!rpc_valid)
		return HANDLE_RPC_FAILED;
	RpcBasic* rpc_ptr = RpcBasic::getInstance();
	// "/rpc/tool_manager/getAllValidToolSummaryInfo"
	RequestMessageType_Void req_data1;
	ResponseMessageType_Uint64_ToolSummaryList rep_data1;
	if (!rpc_ptr->handleRpc(0x0001104F, &req_data1, RequestMessageType_Void_fields, &rep_data1, ResponseMessageType_Uint64_ToolSummaryList_fields))
	{
		return HANDLE_RPC_FAILED;
	}

	RequestMessageType_ToolInfo req_data2;
	ResponseMessageType_Uint64 rep_data2;
	req_data2.data.id = id;
	req_data2.data.group_id = group_id;
	req_data2.data.data.data_count = 6;
	req_data2.data.data.data[0] = x;
	req_data2.data.data.data[1] = y;
	req_data2.data.data.data[2] = z;
	req_data2.data.data.data[3] = a;
	req_data2.data.data.data[4] = b;
	req_data2.data.data.data[5] = c;
	memcpy(req_data2.data.name, name, strlen(name));
	req_data2.data.name[strlen(name)] = '\0';
	memcpy(req_data2.data.comment, comment, strlen(comment));
	req_data2.data.comment[strlen(comment)] = '\0';

	bool is_exist = false;
	for (uint32_t i = 0; i < rep_data1.data.tool_summary_info_count; ++i)
	{
		if (id == rep_data1.data.tool_summary_info[i].id)
		{
			is_exist = true;
			break;
		}
	}
	if (is_exist)
	{
		// "/rpc/tool_manager/updateTool"
		if (!rpc_ptr->handleRpc(0x0000C78C, &req_data2, RequestMessageType_ToolInfo_fields, &rep_data2, ResponseMessageType_Uint64_fields))
		{
			return HANDLE_RPC_FAILED;
		}
	}
	else
	{
		// "/rpc/tool_manager/addTool"
		if (!rpc_ptr->handleRpc(0x0000A22C, &req_data2, RequestMessageType_ToolInfo_fields, &rep_data2, ResponseMessageType_Uint64_fields))
		{
			return HANDLE_RPC_FAILED;
		}
	}
	return rep_data2.data.data;
}

uint64_t c_tfDeleteToolFrame(int32_t id)
{
	if (!rpc_valid)
		return HANDLE_RPC_FAILED;
	RpcBasic* rpc_ptr = RpcBasic::getInstance();
	RequestMessageType_Int32 req_data;
	ResponseMessageType_Uint64 rep_data;

	req_data.data.data = id;
	if (!rpc_ptr->handleRpc(0x00010E4C, &req_data, RequestMessageType_Int32_fields, &rep_data, ResponseMessageType_Uint64_fields))
	{
		return HANDLE_RPC_FAILED;
	}
	return rep_data.data.data;
}