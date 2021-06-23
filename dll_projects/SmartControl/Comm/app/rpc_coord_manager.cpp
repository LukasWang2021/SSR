#include "rpc_interface.h"
#include "rpc_basic.h"
#include "common_error_code.h"

uint64_t c_ufUploadUserFrame(int32_t id, double* x, double* y, double* z, double* a, double* b, double *c, char* name, char* comment, int32_t* group_id)
{
	if (!rpc_valid)
		return HANDLE_RPC_FAILED;
	RpcBasic* rpc_ptr = RpcBasic::getInstance();
	RequestMessageType_Int32 req_data;
	ResponseMessageType_Uint64_UserCoordInfo rep_data;

	req_data.data.data = id;
	if (!rpc_ptr->handleRpc(0x00004324, &req_data, RequestMessageType_Int32_fields, &rep_data, ResponseMessageType_Uint64_UserCoordInfo_fields))
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

uint64_t c_ufDownloadUserFrame(int32_t id, double x, double y, double z, double a, double b, double c, char* name, char* comment, int32_t group_id)
{
	if (!rpc_valid)
		return HANDLE_RPC_FAILED;
	RpcBasic* rpc_ptr = RpcBasic::getInstance();
	// "/rpc/coordinate_manager/getAllValidUserCoordSummaryInfo"
	RequestMessageType_Void req_data1;
	ResponseMessageType_Uint64_UserCoordSummaryList rep_data1;
	if (!rpc_ptr->handleRpc(0x0001838F, &req_data1, RequestMessageType_Void_fields, &rep_data1, ResponseMessageType_Uint64_UserCoordSummaryList_fields))
	{
		return HANDLE_RPC_FAILED;
	}

	RequestMessageType_UserCoordInfo req_data2;
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
	for (uint32_t i = 0; i < rep_data1.data.user_coord_summary_count; ++i)
	{
		if (id == rep_data1.data.user_coord_summary[i].id)
		{
			is_exist = true;
			break;
		}
	}
	if (is_exist)
	{
		// "/rpc/coordinate_manager/updateUserCoord"
		if (!rpc_ptr->handleRpc(0x0000EC14, &req_data2, RequestMessageType_UserCoordInfo_fields, &rep_data2, ResponseMessageType_Uint64_fields))
		{
			return HANDLE_RPC_FAILED;
		}
	}
	else
	{
		// "/rpc/coordinate_manager/addUserCoord"
		if (!rpc_ptr->handleRpc(0x00016764, &req_data2, RequestMessageType_UserCoordInfo_fields, &rep_data2, ResponseMessageType_Uint64_fields))
		{
			return HANDLE_RPC_FAILED;
		}
	}
	return rep_data2.data.data;
}

uint64_t c_ufDeleteUserFrame(int32_t id)
{
	if (!rpc_valid)
		return HANDLE_RPC_FAILED;
	RpcBasic* rpc_ptr = RpcBasic::getInstance();
	RequestMessageType_Int32 req_data;
	ResponseMessageType_Uint64 rep_data;

	req_data.data.data = id;
	if (!rpc_ptr->handleRpc(0x0000BAF4, &req_data, RequestMessageType_Int32_fields, &rep_data, ResponseMessageType_Uint64_fields))
	{
		return HANDLE_RPC_FAILED;
	}
	return rep_data.data.data;
}

