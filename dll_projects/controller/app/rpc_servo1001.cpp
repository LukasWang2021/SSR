//#include "stdafx.h"
#include "rpc_interface.h"
#include "rpc_basic.h"
#include "common_error_code.h"
#include <string.h>
#include <time.h>
#include "protoc.h"

uint64_t c_servo1001ServoShutDown(int32_t cpu_id, int32_t servo_id)
{
	if (!rpc_valid)
		return HANDLE_RPC_FAILED;
	RpcBasic* rpc_ptr = RpcBasic::getInstance();
	RequestMessageType_Int32List req_data;
	ResponseMessageType_Uint64 rep_data;

	req_data.header.time_stamp = 122;
	req_data.property.authority = Comm_Authority_TP_SIMMULATOR;
	req_data.data.data_count = 2;
	req_data.data.data[0] = cpu_id;
	req_data.data.data[1] = servo_id;
	if (!rpc_ptr->handleRpc(0x0000863E, &req_data, RequestMessageType_Int32List_fields, &rep_data, ResponseMessageType_Uint64_fields))
	{
		return HANDLE_RPC_FAILED;
	}
	return rep_data.data.data;
}

uint64_t c_servo1001ServoSwitchOn(int32_t cpu_id, int32_t servo_id)
{
	if (!rpc_valid)
		return HANDLE_RPC_FAILED;
	RpcBasic* rpc_ptr = RpcBasic::getInstance();
	RequestMessageType_Int32List req_data;
	ResponseMessageType_Uint64 rep_data;

	req_data.header.time_stamp = 122;
	req_data.property.authority = Comm_Authority_TP_SIMMULATOR;
	req_data.data.data_count = 2;
	req_data.data.data[0] = cpu_id;
	req_data.data.data[1] = servo_id;
	if (!rpc_ptr->handleRpc(0x0000E5CE, &req_data, RequestMessageType_Int32List_fields, &rep_data, ResponseMessageType_Uint64_fields))
	{
		return HANDLE_RPC_FAILED;
	}
	return rep_data.data.data;
}

uint64_t c_servo1001ServoDisableVoltage(int32_t cpu_id, int32_t servo_id)
{
	if (!rpc_valid)
		return HANDLE_RPC_FAILED;
	RpcBasic* rpc_ptr = RpcBasic::getInstance();
	RequestMessageType_Int32List req_data;
	ResponseMessageType_Uint64 rep_data;

	req_data.header.time_stamp = 122;
	req_data.property.authority = Comm_Authority_TP_SIMMULATOR;
	req_data.data.data_count = 2;
	req_data.data.data[0] = cpu_id;
	req_data.data.data[1] = servo_id;
	if (!rpc_ptr->handleRpc(0x00004755, &req_data, RequestMessageType_Int32List_fields, &rep_data, ResponseMessageType_Uint64_fields))
	{
		return HANDLE_RPC_FAILED;
	}
	return rep_data.data.data;
}

uint64_t c_servo1001ServoEnableOperation(int32_t cpu_id, int32_t servo_id)
{
	if (!rpc_valid)
		return HANDLE_RPC_FAILED;
	RpcBasic* rpc_ptr = RpcBasic::getInstance();
	RequestMessageType_Int32List req_data;
	ResponseMessageType_Uint64 rep_data;

	req_data.header.time_stamp = 122;
	req_data.property.authority = Comm_Authority_TP_SIMMULATOR;
	req_data.data.data_count = 2;
	req_data.data.data[0] = cpu_id;
	req_data.data.data[1] = servo_id;
	if (!rpc_ptr->handleRpc(0x0000313E, &req_data, RequestMessageType_Int32List_fields, &rep_data, ResponseMessageType_Uint64_fields))
	{
		return HANDLE_RPC_FAILED;
	}
	return rep_data.data.data;
}

uint64_t c_servo1001ServoSwitchOnAndEnableOperation(int32_t cpu_id, int32_t servo_id)
{
	if (!rpc_valid)
		return HANDLE_RPC_FAILED;
	RpcBasic* rpc_ptr = RpcBasic::getInstance();
	RequestMessageType_Int32List req_data;
	ResponseMessageType_Uint64 rep_data;

	req_data.header.time_stamp = 122;
	req_data.property.authority = Comm_Authority_TP_SIMMULATOR;
	req_data.data.data_count = 2;
	req_data.data.data[0] = cpu_id;
	req_data.data.data[1] = servo_id;
	if (!rpc_ptr->handleRpc(0x000177CE, &req_data, RequestMessageType_Int32List_fields, &rep_data, ResponseMessageType_Uint64_fields))
	{
		return HANDLE_RPC_FAILED;
	}
	return rep_data.data.data;
}


uint64_t c_servo1001ServoDisableOperation(int32_t cpu_id, int32_t servo_id)
{
	if (!rpc_valid)
		return HANDLE_RPC_FAILED;
	RpcBasic* rpc_ptr = RpcBasic::getInstance();
	RequestMessageType_Int32List req_data;
	ResponseMessageType_Uint64 rep_data;

	req_data.header.time_stamp = 122;
	req_data.property.authority = Comm_Authority_TP_SIMMULATOR;
	req_data.data.data_count = 2;
	req_data.data.data[0] = cpu_id;
	req_data.data.data[1] = servo_id;
	if (!rpc_ptr->handleRpc(0x000026AE, &req_data, RequestMessageType_Int32List_fields, &rep_data, ResponseMessageType_Uint64_fields))
	{
		return HANDLE_RPC_FAILED;
	}
	return rep_data.data.data;
}

uint64_t c_servo1001ServoQuickStop(int32_t cpu_id, int32_t servo_id)
{
	if (!rpc_valid)
		return HANDLE_RPC_FAILED;
	RpcBasic* rpc_ptr = RpcBasic::getInstance();
	RequestMessageType_Int32List req_data;
	ResponseMessageType_Uint64 rep_data;

	req_data.header.time_stamp = 122;
	req_data.property.authority = Comm_Authority_TP_SIMMULATOR;
	req_data.data.data_count = 2;
	req_data.data.data[0] = cpu_id;
	req_data.data.data[1] = servo_id;
	if (!rpc_ptr->handleRpc(0x00000580, &req_data, RequestMessageType_Int32List_fields, &rep_data, ResponseMessageType_Uint64_fields))
	{
		return HANDLE_RPC_FAILED;
	}
	return rep_data.data.data;
}

uint64_t c_servo1001ServoResetFault(int32_t cpu_id, int32_t servo_id)
{
	if (!rpc_valid)
		return HANDLE_RPC_FAILED;
	RpcBasic* rpc_ptr = RpcBasic::getInstance();
	RequestMessageType_Int32List req_data;
	ResponseMessageType_Uint64 rep_data;

	req_data.header.time_stamp = 122;
	req_data.property.authority = Comm_Authority_TP_SIMMULATOR;
	req_data.data.data_count = 2;
	req_data.data.data[0] = cpu_id;
	req_data.data.data[1] = servo_id;
	if (!rpc_ptr->handleRpc(0x00010584, &req_data, RequestMessageType_Int32List_fields, &rep_data, ResponseMessageType_Uint64_fields))
	{
		return HANDLE_RPC_FAILED;
	}
	return rep_data.data.data;
}

uint64_t c_servo1001ServoTransCommState(int32_t cpu_id, int32_t servo_id, int32_t state)
{
	if (!rpc_valid)
		return HANDLE_RPC_FAILED;
	RpcBasic* rpc_ptr = RpcBasic::getInstance();
	RequestMessageType_Int32List_CoreCommState req_data;
	ResponseMessageType_Uint64 rep_data;

	req_data.header.time_stamp = 122;
	req_data.property.authority = Comm_Authority_TP_SIMMULATOR;
	req_data.data1.data_count = 2;
	req_data.data1.data[0] = cpu_id;
	req_data.data1.data[1] = servo_id;
	req_data.data2 = (MessageType_CoreCommState)state;
	if (!rpc_ptr->handleRpc(0x000153C5, &req_data, RequestMessageType_Int32List_CoreCommState_fields, &rep_data, ResponseMessageType_Uint64_fields))
	{
		return HANDLE_RPC_FAILED;
	}
	return rep_data.data.data;
}

uint64_t c_servo1001ServoReadParameter(int32_t cpu_id, int32_t servo_id, int32_t param_index, int32_t* value)
{
	if (!rpc_valid)
		return HANDLE_RPC_FAILED;
	RpcBasic* rpc_ptr = RpcBasic::getInstance();
	RequestMessageType_Int32List req_data;
	ResponseMessageType_Uint64_Int32 rep_data;

	req_data.header.time_stamp = 122;
	req_data.property.authority = Comm_Authority_TP_SIMMULATOR;
	req_data.data.data_count = 3;
	req_data.data.data[0] = cpu_id;
	req_data.data.data[1] = servo_id;
	req_data.data.data[2] = param_index;
	if (!rpc_ptr->handleRpc(0x00006892, &req_data, RequestMessageType_Int32List_fields, &rep_data, ResponseMessageType_Uint64_Int32_fields))
	{
		return HANDLE_RPC_FAILED;
	}
	*value = rep_data.data.data;
	return rep_data.error_code.data;
}

uint64_t c_servo1001ServoWriteParameter(int32_t cpu_id, int32_t servo_id, int32_t param_index, int32_t param_value)
{
	if (!rpc_valid)
		return HANDLE_RPC_FAILED;
	RpcBasic* rpc_ptr = RpcBasic::getInstance();
	RequestMessageType_Int32List req_data;
	ResponseMessageType_Uint64 rep_data;

	req_data.header.time_stamp = 122;
	req_data.property.authority = Comm_Authority_TP_SIMMULATOR;
	req_data.data.data_count = 4;
	req_data.data.data[0] = cpu_id;
	req_data.data.data[1] = servo_id;
	req_data.data.data[2] = param_index;
	req_data.data.data[3] = param_value;
	if (!rpc_ptr->handleRpc(0x00007C32, &req_data, RequestMessageType_Int32List_fields, &rep_data, ResponseMessageType_Uint64_fields))
	{
		return HANDLE_RPC_FAILED;
	}
	return rep_data.data.data;
}


uint64_t c_servo1001ServoMoveVelocity(int32_t cpu_id, int32_t servo_id, int32_t velocity, int32_t acc, int32_t dec, int32_t jerk, int32_t direction)
{
	if (!rpc_valid)
		return HANDLE_RPC_FAILED;
	RpcBasic* rpc_ptr = RpcBasic::getInstance();
	RequestMessageType_Int32List req_data;
	ResponseMessageType_Uint64 rep_data;

	req_data.header.time_stamp = 122;
	req_data.property.authority = Comm_Authority_TP_SIMMULATOR;
	req_data.data.data_count = 7;
	req_data.data.data[0] = cpu_id;
	req_data.data.data[1] = servo_id;
	req_data.data.data[2] = velocity;
	req_data.data.data[3] = acc;
	req_data.data.data[4] = dec;
	req_data.data.data[5] = jerk;
	req_data.data.data[6] = direction;
	if (!rpc_ptr->handleRpc(0x000164D9, &req_data, RequestMessageType_Int32List_fields, &rep_data, ResponseMessageType_Uint64_fields))
	{
		return HANDLE_RPC_FAILED;
	}
	return rep_data.data.data;
}


uint64_t c_servo1001ServoMoveAbsolute(int32_t cpu_id, int32_t servo_id, int64_t position, int32_t velocity, int32_t acc, int32_t dec, int32_t jerk)
{
	if (!rpc_valid)
		return HANDLE_RPC_FAILED;
	RpcBasic* rpc_ptr = RpcBasic::getInstance();
	RequestMessageType_Int32List_Int64 req_data;
	ResponseMessageType_Uint64 rep_data;

	req_data.header.time_stamp = 122;
	req_data.property.authority = Comm_Authority_TP_SIMMULATOR;
	req_data.data1.data_count = 6;
	req_data.data1.data[0] = cpu_id;
	req_data.data1.data[1] = servo_id;
	req_data.data1.data[2] = velocity;
	req_data.data1.data[3] = acc;
	req_data.data1.data[4] = dec;
	req_data.data1.data[5] = jerk;
	req_data.data2.data = position;
	if (!rpc_ptr->handleRpc(0x00004DD5, &req_data, RequestMessageType_Int32List_Int64_fields, &rep_data, ResponseMessageType_Uint64_fields))
	{
		return HANDLE_RPC_FAILED;
	}
	return rep_data.data.data;
}


uint64_t c_servo1001ServoTriggerUploadParameters(int32_t cpu_id, int32_t servo_id)
{
	if (!rpc_valid)
		return HANDLE_RPC_FAILED;
	RpcBasic* rpc_ptr = RpcBasic::getInstance();
	RequestMessageType_Int32List req_data;
	ResponseMessageType_Uint64 rep_data;

	req_data.header.time_stamp = 122;
	req_data.property.authority = Comm_Authority_TP_SIMMULATOR;
	req_data.data.data_count = 2;
	req_data.data.data[0] = cpu_id;
	req_data.data.data[1] = servo_id;
	if (!rpc_ptr->handleRpc(0x000020B3, &req_data, RequestMessageType_Int32List_fields, &rep_data, ResponseMessageType_Uint64_fields))
	{
		return HANDLE_RPC_FAILED;
	}
	return rep_data.data.data;
}


uint64_t c_servo1001ServoUploadParameters(int32_t cpu_id, int32_t servo_id, int32_t value[512])
{
	if (!rpc_valid)
		return HANDLE_RPC_FAILED;
	RpcBasic* rpc_ptr = RpcBasic::getInstance();
	RequestMessageType_Int32List req_data;
	ResponseMessageType_Uint64_Int32List rep_data;

	req_data.header.time_stamp = 122;
	req_data.property.authority = Comm_Authority_TP_SIMMULATOR;
	req_data.data.data_count = 2;
	req_data.data.data[0] = cpu_id;
	req_data.data.data[1] = servo_id;
	if (!rpc_ptr->handleRpc(0x0000E003, &req_data, RequestMessageType_Int32List_fields, &rep_data, ResponseMessageType_Uint64_Int32List_fields))
	{
		return HANDLE_RPC_FAILED;
	}

	for (unsigned int i = 0; i < rep_data.data.data_count; ++i)
	{
		value[i] = rep_data.data.data[i];
	}
	return rep_data.error_code.data;
}


uint64_t c_servo1001ServoTriggerDownloadParameters(int32_t cpu_id, int32_t servo_id)
{
	if (!rpc_valid)
		return HANDLE_RPC_FAILED;
	RpcBasic* rpc_ptr = RpcBasic::getInstance();
	RequestMessageType_Int32List req_data;
	ResponseMessageType_Uint64 rep_data;

	req_data.header.time_stamp = 122;
	req_data.property.authority = Comm_Authority_TP_SIMMULATOR;
	req_data.data.data_count = 2;
	req_data.data.data[0] = cpu_id;
	req_data.data.data[1] = servo_id;
	if (!rpc_ptr->handleRpc(0x00011C53, &req_data, RequestMessageType_Int32List_fields, &rep_data, ResponseMessageType_Uint64_fields))
	{
		return HANDLE_RPC_FAILED;
	}
	return rep_data.data.data;
}

uint64_t c_servo1001ServoDownloadParameters(int32_t cpu_id, int32_t servo_id, int32_t param_value[512], int32_t param_size)
{
	if (!rpc_valid)
		return HANDLE_RPC_FAILED;
	RpcBasic* rpc_ptr = RpcBasic::getInstance();
	RequestMessageType_Int32List_Int32List req_data;
	ResponseMessageType_Uint64 rep_data;

	req_data.header.time_stamp = 122;
	req_data.property.authority = Comm_Authority_TP_SIMMULATOR;
	req_data.data1.data_count = 2;
	req_data.data1.data[0] = cpu_id;
	req_data.data1.data[1] = servo_id;
	req_data.data2.data_count = 512;
	for (int32_t i = 0; i < param_size; ++i)
	{
		req_data.data2.data[i] = param_value[i];
	}
	if (!rpc_ptr->handleRpc(0x00017063, &req_data, RequestMessageType_Int32List_Int32List_fields, &rep_data, ResponseMessageType_Uint64_fields))
	{
		return HANDLE_RPC_FAILED;
	}
	return rep_data.data.data;
}

uint64_t c_servo1001ServoIsAsyncServiceFinish(int32_t cpu_id, int32_t servo_id, int32_t* sync)
{
	if (!rpc_valid)
		return HANDLE_RPC_FAILED;
	RpcBasic* rpc_ptr = RpcBasic::getInstance();
	RequestMessageType_Int32List req_data;
	ResponseMessageType_Uint64_Bool rep_data;

	req_data.header.time_stamp = 122;
	req_data.property.authority = Comm_Authority_TP_SIMMULATOR;
	req_data.data.data_count = 2;
	req_data.data.data[0] = cpu_id;
	req_data.data.data[1] = servo_id;
	if (!rpc_ptr->handleRpc(0x000043B8, &req_data, RequestMessageType_Int32List_fields, &rep_data, ResponseMessageType_Uint64_Bool_fields))
	{
		return HANDLE_RPC_FAILED;
	}
	*sync = rep_data.data.data;
	return rep_data.error_code.data;
}

uint64_t c_servo1001ServoGetCommState(int32_t cpu_id, int32_t servo_id, int32_t* state)
{
	if (!rpc_valid)
		return HANDLE_RPC_FAILED;
	RpcBasic* rpc_ptr = RpcBasic::getInstance();
	RequestMessageType_Int32List req_data;
	ResponseMessageType_Uint64_CoreCommState rep_data;

	req_data.header.time_stamp = 122;
	req_data.property.authority = Comm_Authority_TP_SIMMULATOR;
	req_data.data.data_count = 2;
	req_data.data.data[0] = cpu_id;
	req_data.data.data[1] = servo_id;
	if (!rpc_ptr->handleRpc(0x0000F485, &req_data, RequestMessageType_Int32List_fields, &rep_data, ResponseMessageType_Uint64_CoreCommState_fields))
	{
		return HANDLE_RPC_FAILED;
	}
	*state = rep_data.data;
	return rep_data.error_code.data;
}


uint64_t c_servo1001ServoMoveRelative(int32_t cpu_id, int32_t servo_id, int64_t position, int32_t velocity, int32_t acc, int32_t dec, int32_t jerk)
{
	if (!rpc_valid)
		return HANDLE_RPC_FAILED;
	RpcBasic* rpc_ptr = RpcBasic::getInstance();
	RequestMessageType_Int32List_Int64 req_data;
	ResponseMessageType_Uint64 rep_data;

	req_data.header.time_stamp = 122;
	req_data.property.authority = Comm_Authority_TP_SIMMULATOR;
	req_data.data1.data_count = 6;
	req_data.data1.data[0] = cpu_id;
	req_data.data1.data[1] = servo_id;
	req_data.data1.data[2] = velocity;
	req_data.data1.data[3] = acc;
	req_data.data1.data[4] = dec;
	req_data.data1.data[5] = jerk;
	req_data.data2.data = position;
	if (!rpc_ptr->handleRpc(0x000172C5, &req_data, RequestMessageType_Int32List_Int64_fields, &rep_data, ResponseMessageType_Uint64_fields))
	{
		return HANDLE_RPC_FAILED;
	}
	return rep_data.data.data;
}

uint64_t c_servo1001ServoHome(int32_t cpu_id, int32_t servo_id)
{
	if (!rpc_valid)
		return HANDLE_RPC_FAILED;
	RpcBasic* rpc_ptr = RpcBasic::getInstance();
	RequestMessageType_Int32List req_data;
	ResponseMessageType_Uint64 rep_data;

	req_data.header.time_stamp = 122;
	req_data.property.authority = Comm_Authority_TP_SIMMULATOR;
	req_data.data.data_count = 2;
	req_data.data.data[0] = cpu_id;
	req_data.data.data[1] = servo_id;
	if (!rpc_ptr->handleRpc(0x00013BB5, &req_data, RequestMessageType_Int32List_fields, &rep_data, ResponseMessageType_Uint64_fields))
	{
		return HANDLE_RPC_FAILED;
	}
	return rep_data.data.data;
}

uint64_t c_servo1001ServoAbortHoming(int32_t cpu_id, int32_t servo_id)
{
	if (!rpc_valid)
		return HANDLE_RPC_FAILED;
	RpcBasic* rpc_ptr = RpcBasic::getInstance();
	RequestMessageType_Int32List req_data;
	ResponseMessageType_Uint64 rep_data;

	req_data.header.time_stamp = 122;
	req_data.property.authority = Comm_Authority_TP_SIMMULATOR;
	req_data.data.data_count = 2;
	req_data.data.data[0] = cpu_id;
	req_data.data.data[1] = servo_id;
	if (!rpc_ptr->handleRpc(0x00015AB7, &req_data, RequestMessageType_Int32List_fields, &rep_data, ResponseMessageType_Uint64_fields))
	{
		return HANDLE_RPC_FAILED;
	}
	return rep_data.data.data;
}


uint64_t c_servo1001ServoGetServoCommInfo(int32_t cpu_id, int32_t servo_id, int32_t* comm_reg_id, int32_t* service_id,
	int32_t* download_param_id, int32_t* upload_param_id, int32_t* ctrl_pdo_id, int32_t* fdb_pdo_id)
{
	if (!rpc_valid)
		return HANDLE_RPC_FAILED;
	RpcBasic* rpc_ptr = RpcBasic::getInstance();
	RequestMessageType_Int32List req_data;
	ResponseMessageType_Uint64_Int32List rep_data;

	req_data.header.time_stamp = 122;
	req_data.property.authority = Comm_Authority_TP_SIMMULATOR;
	req_data.data.data_count = 2;
	req_data.data.data[0] = cpu_id;
	req_data.data.data[1] = servo_id;
	if (!rpc_ptr->handleRpc(0x0000BF1F, &req_data, RequestMessageType_Int32List_fields, &rep_data, ResponseMessageType_Uint64_Int32List_fields))
	{
		return HANDLE_RPC_FAILED;
	}
	*comm_reg_id = rep_data.data.data[0];
	*service_id = rep_data.data.data[1];
	*download_param_id = rep_data.data.data[2];
	*upload_param_id = rep_data.data.data[3];
	*ctrl_pdo_id = rep_data.data.data[4];
	*fdb_pdo_id = rep_data.data.data[5];
	return rep_data.error_code.data;
}

uint64_t c_servo1001ServoGetServoDefinedInfo(int32_t cpu_id, int32_t servo_id, int32_t req[7], int32_t res[7])
{
	if (!rpc_valid)
		return HANDLE_RPC_FAILED;
	RpcBasic* rpc_ptr = RpcBasic::getInstance();
	RequestMessageType_Int32_Int32List req_data;
	ResponseMessageType_Uint64_Int32List rep_data;

	req_data.header.time_stamp = 122;
	req_data.property.authority = Comm_Authority_TP_SIMMULATOR;
	req_data.data1.data = cpu_id;
	req_data.data2.data_count = 9;
	req_data.data2.data[0] = servo_id;
	for (unsigned int i = 0; i < 7; ++i)
	{
		req_data.data2.data[i + 1] = req[i];
	}
	if (!rpc_ptr->handleRpc(0x0000C87F, &req_data, RequestMessageType_Int32_Int32List_fields, &rep_data, ResponseMessageType_Uint64_Int32List_fields))
	{
		return HANDLE_RPC_FAILED;
	}
	for (unsigned int i = 0; i < 7; ++i)
	{
		res[i] = rep_data.data.data[i + 1];
	}
	return rep_data.error_code.data;
}

uint64_t c_servo1001CpuGetVersion(int32_t cpu_id, int32_t* major_version, int32_t* minor_version)
{
	if (!rpc_valid)
		return HANDLE_RPC_FAILED;
	RpcBasic* rpc_ptr = RpcBasic::getInstance();
	RequestMessageType_Int32 req_data;
	ResponseMessageType_Uint64_Uint32List rep_data;

	req_data.header.time_stamp = 122;
	req_data.property.authority = Comm_Authority_TP_SIMMULATOR;
	req_data.data.data = cpu_id;
	if (!rpc_ptr->handleRpc(0x0001192E, &req_data, RequestMessageType_Int32List_fields, &rep_data, ResponseMessageType_Uint64_Uint32List_fields))
	{
		return HANDLE_RPC_FAILED;
	}
	*major_version = rep_data.data.data[0];
	*minor_version = rep_data.data.data[1];
	return rep_data.error_code.data;
}

uint64_t c_servo1001CpuSetCtrlPdoSync(int32_t cpu_id, int32_t servo_id, uint32_t sync)
{
	if (!rpc_valid)
		return HANDLE_RPC_FAILED;
	RpcBasic* rpc_ptr = RpcBasic::getInstance();
	RequestMessageType_Int32List_Uint32 req_data;
	ResponseMessageType_Uint64 rep_data;

	req_data.header.time_stamp = 122;
	req_data.property.authority = Comm_Authority_TP_SIMMULATOR;
	req_data.data1.data_count = 2;
	req_data.data1.data[0] = cpu_id;
	req_data.data1.data[1] = servo_id;
	req_data.data2.data = sync;
	if (!rpc_ptr->handleRpc(0x00005123, &req_data, RequestMessageType_Int32List_Uint32_fields, &rep_data, ResponseMessageType_Uint64_fields))
	{
		return HANDLE_RPC_FAILED;
	}
	return rep_data.data.data;
}

uint64_t c_servo1001CpuGetCtrlPdoSync(int32_t cpu_id, int32_t servo_id, uint32_t* sync)
{
	if (!rpc_valid)
		return HANDLE_RPC_FAILED;
	RpcBasic* rpc_ptr = RpcBasic::getInstance();
	RequestMessageType_Int32List req_data;
	ResponseMessageType_Uint64_Int32_Uint32 rep_data;

	req_data.header.time_stamp = 122;
	req_data.property.authority = Comm_Authority_TP_SIMMULATOR;
	req_data.data.data_count = 2;
	req_data.data.data[0] = cpu_id;
	req_data.data.data[1] = servo_id;
	if (!rpc_ptr->handleRpc(0x00005463, &req_data, RequestMessageType_Int32List_fields, &rep_data, ResponseMessageType_Uint64_Int32_Uint32_fields))
	{
		return HANDLE_RPC_FAILED;
	}
	if (servo_id != rep_data.data1.data)
	{
		return HANDLE_RPC_FAILED;
	}
	*sync = rep_data.data2.data;
	return rep_data.error_code.data;
}

uint64_t c_servo1001CpuSetSamplingSync(int32_t cpu_id, uint32_t sync)
{
	if (!rpc_valid)
		return HANDLE_RPC_FAILED;
	RpcBasic* rpc_ptr = RpcBasic::getInstance();
	RequestMessageType_Int32_Uint32 req_data;
	ResponseMessageType_Uint64 rep_data;

	req_data.header.time_stamp = 122;
	req_data.property.authority = Comm_Authority_TP_SIMMULATOR;
	req_data.data1.data = cpu_id;
	req_data.data2.data = sync;
	if (!rpc_ptr->handleRpc(0x00004023, &req_data, RequestMessageType_Int32_Uint32_fields, &rep_data, ResponseMessageType_Uint64_fields))
	{
		return HANDLE_RPC_FAILED;
	}
	return rep_data.data.data;
}

uint64_t c_servo1001CpuGetSamplingSync(int32_t cpu_id, uint32_t* sync)
{
	if (!rpc_valid)
		return HANDLE_RPC_FAILED;
	RpcBasic* rpc_ptr = RpcBasic::getInstance();
	RequestMessageType_Int32 req_data;
	ResponseMessageType_Uint64_Uint32 rep_data;

	req_data.header.time_stamp = 122;
	req_data.property.authority = Comm_Authority_TP_SIMMULATOR;
	req_data.data.data = cpu_id;
	if (!rpc_ptr->handleRpc(0x00006C23, &req_data, RequestMessageType_Int32_fields, &rep_data, ResponseMessageType_Uint64_Uint32_fields))
	{
		return HANDLE_RPC_FAILED;
	}
	*sync = rep_data.data.data;
	return rep_data.error_code.data;
}

uint64_t c_servo1001CpuSetSamplingInterval(int32_t cpu_id, uint32_t interval)
{
	if (!rpc_valid)
		return HANDLE_RPC_FAILED;
	RpcBasic* rpc_ptr = RpcBasic::getInstance();
	RequestMessageType_Int32_Uint32 req_data;
	ResponseMessageType_Uint64 rep_data;

	req_data.header.time_stamp = 122;
	req_data.property.authority = Comm_Authority_TP_SIMMULATOR;
	req_data.data1.data = cpu_id;
	req_data.data2.data = interval;
	if (!rpc_ptr->handleRpc(0x00003EEC, &req_data, RequestMessageType_Int32_Uint32_fields, &rep_data, ResponseMessageType_Uint64_fields))
	{
		return HANDLE_RPC_FAILED;
	}
	return rep_data.data.data;
}

uint64_t c_servo1001CpuGetSamplingInterval(int32_t cpu_id, uint32_t* interval)
{
	if (!rpc_valid)
		return HANDLE_RPC_FAILED;
	RpcBasic* rpc_ptr = RpcBasic::getInstance();
	RequestMessageType_Int32 req_data;
	ResponseMessageType_Uint64_Uint32 rep_data;

	req_data.header.time_stamp = 122;
	req_data.property.authority = Comm_Authority_TP_SIMMULATOR;
	req_data.data.data = cpu_id;
	if (!rpc_ptr->handleRpc(0x00001C2C, &req_data, RequestMessageType_Int32_fields, &rep_data, ResponseMessageType_Uint64_Uint32_fields))
	{
		return HANDLE_RPC_FAILED;
	}
	*interval = rep_data.data.data;
	return rep_data.error_code.data;
}

uint64_t c_servo1001CpuSetSamplingMaxTimes(int32_t cpu_id, uint32_t max_times)
{
	if (!rpc_valid)
		return HANDLE_RPC_FAILED;
	RpcBasic* rpc_ptr = RpcBasic::getInstance();
	RequestMessageType_Int32_Uint32 req_data;
	ResponseMessageType_Uint64 rep_data;

	req_data.header.time_stamp = 122;
	req_data.property.authority = Comm_Authority_TP_SIMMULATOR;
	req_data.data1.data = cpu_id;
	req_data.data2.data = max_times;
	if (!rpc_ptr->handleRpc(0x000110A3, &req_data, RequestMessageType_Int32_Uint32_fields, &rep_data, ResponseMessageType_Uint64_fields))
	{
		return HANDLE_RPC_FAILED;
	}
	return rep_data.data.data;
}

uint64_t c_servo1001CpuGetSamplingMaxTimes(int32_t cpu_id, uint32_t* max_times)
{
	if (!rpc_valid)
		return HANDLE_RPC_FAILED;
	RpcBasic* rpc_ptr = RpcBasic::getInstance();
	RequestMessageType_Int32 req_data;
	ResponseMessageType_Uint64_Uint32 rep_data;

	req_data.header.time_stamp = 122;
	req_data.property.authority = Comm_Authority_TP_SIMMULATOR;
	req_data.data.data = cpu_id;
	if (!rpc_ptr->handleRpc(0x00013363, &req_data, RequestMessageType_Int32_fields, &rep_data, ResponseMessageType_Uint64_Uint32_fields))
	{
		return HANDLE_RPC_FAILED;
	}
	*max_times = rep_data.data.data;
	return rep_data.error_code.data;
}

uint64_t c_servo1001CpuSetSamplingChannel(int32_t cpu_id, uint32_t channel_index, int32_t servo_index, int32_t servo_param_index)
{
	if (!rpc_valid)
		return HANDLE_RPC_FAILED;
	RpcBasic* rpc_ptr = RpcBasic::getInstance();
	RequestMessageType_Int32_Uint32List req_data;
	ResponseMessageType_Uint64 rep_data;

	req_data.header.time_stamp = 122;
	req_data.property.authority = Comm_Authority_TP_SIMMULATOR;
	req_data.data1.data = cpu_id;
	req_data.data2.data_count = 3;
	req_data.data2.data[0] = channel_index;
	req_data.data2.data[1] = servo_index;
	req_data.data2.data[2] = servo_param_index;
	if (!rpc_ptr->handleRpc(0x00008E5C, &req_data, RequestMessageType_Int32_Uint32List_fields, &rep_data, ResponseMessageType_Uint64_fields))
	{
		return HANDLE_RPC_FAILED;
	}
	return rep_data.data.data;
}
uint64_t c_servo1001CpuGetSamplingChannel(int32_t cpu_id, uint32_t value[16], int32_t* size)
{
	if (!rpc_valid)
		return HANDLE_RPC_FAILED;
	RpcBasic* rpc_ptr = RpcBasic::getInstance();
	RequestMessageType_Int32 req_data;
	ResponseMessageType_Uint64_Uint32List rep_data;

	req_data.header.time_stamp = 122;
	req_data.property.authority = Comm_Authority_TP_SIMMULATOR;
	req_data.data.data = cpu_id;
	if (!rpc_ptr->handleRpc(0x0000FD9C, &req_data, RequestMessageType_Int32_fields, &rep_data, ResponseMessageType_Uint64_Uint32List_fields))
	{
		return HANDLE_RPC_FAILED;
	}
	*size = rep_data.data.data_count;
	for (unsigned int i = 0; i < rep_data.data.data_count; ++i)
	{
		value[i] = rep_data.data.data[i];
	}
	return rep_data.error_code.data;
}
uint64_t c_servo1001CpuActivateSamplingConfiguration(int32_t cpu_id)
{
	if (!rpc_valid)
		return HANDLE_RPC_FAILED;
	RpcBasic* rpc_ptr = RpcBasic::getInstance();
	RequestMessageType_Int32 req_data;
	ResponseMessageType_Uint64 rep_data;

	req_data.header.time_stamp = 122;
	req_data.property.authority = Comm_Authority_TP_SIMMULATOR;
	req_data.data.data = cpu_id;
	if (!rpc_ptr->handleRpc(0x0000939E, &req_data, RequestMessageType_Int32_fields, &rep_data, ResponseMessageType_Uint64_fields))
	{
		return HANDLE_RPC_FAILED;
	}
	return rep_data.data.data;
}


uint64_t c_servo1001CpuSaveSamplingBufferData(int32_t cpu_id, char* file_path_ptr)
{
	if (!rpc_valid)
		return HANDLE_RPC_FAILED;
	RpcBasic* rpc_ptr = RpcBasic::getInstance();
	RequestMessageType_Int32_String req_data;
	ResponseMessageType_Uint64 rep_data;

	req_data.header.time_stamp = 122;
	req_data.property.authority = Comm_Authority_TP_SIMMULATOR;
	req_data.data1.data = cpu_id;
	int file_path_size = (int)strlen(file_path_ptr);
	memcpy(req_data.data2.data, file_path_ptr, file_path_size);
	req_data.data2.data[file_path_size] = 0;
	if (!rpc_ptr->handleRpc(0x00015621, &req_data, RequestMessageType_Int32_String_fields, &rep_data, ResponseMessageType_Uint64_fields))
	{
		return HANDLE_RPC_FAILED;
	}
	return rep_data.data.data;
}

uint64_t c_servo1001CpuGetServoCpuCommInfo(int32_t cpu_id, int32_t* comm_reg_id, int32_t* sampling_buffer_id)
{
	if (!rpc_valid)
		return HANDLE_RPC_FAILED;
	RpcBasic* rpc_ptr = RpcBasic::getInstance();
	RequestMessageType_Int32 req_data;
	ResponseMessageType_Uint64_Int32List rep_data;

	req_data.header.time_stamp = 122;
	req_data.property.authority = Comm_Authority_TP_SIMMULATOR;
	req_data.data.data = cpu_id;
	if (!rpc_ptr->handleRpc(0x0000FE5F, &req_data, RequestMessageType_Int32_fields, &rep_data, ResponseMessageType_Uint64_Int32List_fields))
	{
		return HANDLE_RPC_FAILED;
	}
	*comm_reg_id = rep_data.data.data[0];
	*sampling_buffer_id = rep_data.data.data[1];
	return rep_data.error_code.data;
}

uint64_t c_servo1001CpuSetForceControlParameters(int32_t cpu_id, int32_t param_value[512], int32_t param_size)
{
	if (!rpc_valid)
		return HANDLE_RPC_FAILED;
	RpcBasic* rpc_ptr = RpcBasic::getInstance();
	RequestMessageType_Int32_Int32List req_data;
	ResponseMessageType_Uint64 rep_data;

	req_data.header.time_stamp = 122;
	req_data.property.authority = Comm_Authority_TP_SIMMULATOR;
	req_data.data1.data = cpu_id;
	req_data.data2.data_count = param_size;
	for (size_t i = 0; i < req_data.data2.data_count; ++i)
	{
		req_data.data2.data[i] = param_value[i];
	}
	if (!rpc_ptr->handleRpc(0x00005F53, &req_data, RequestMessageType_Int32_Int32List_fields, &rep_data, ResponseMessageType_Uint64_fields))
	{
		return HANDLE_RPC_FAILED;
	}
	return rep_data.data.data;
}

uint64_t c_servo1001CpuGetForceControlParameters(int32_t cpu_id, int32_t param_value[512], int32_t* param_size_ptr)
{
	if (!rpc_valid)
		return HANDLE_RPC_FAILED;
	RpcBasic* rpc_ptr = RpcBasic::getInstance();
	RequestMessageType_Int32 req_data;
	ResponseMessageType_Uint64_Int32List rep_data;

	req_data.header.time_stamp = 122;
	req_data.property.authority = Comm_Authority_TP_SIMMULATOR;
	req_data.data.data = cpu_id;
	if (!rpc_ptr->handleRpc(0x00008203, &req_data, RequestMessageType_Int32_fields, &rep_data, ResponseMessageType_Uint64_Int32List_fields))
	{
		return HANDLE_RPC_FAILED;
	}
	*param_size_ptr = rep_data.data.data_count;
	for (size_t i = 0; i < rep_data.data.data_count; ++i)
	{
		param_value[i] = rep_data.data.data[i];
	}
	return rep_data.error_code.data;
}

uint64_t c_servo1001CpuGetTorqueSensorData(int32_t cpu_id, double t_data[6], int32_t* data_size_ptr)
{
	if (!rpc_valid)
		return HANDLE_RPC_FAILED;
	RpcBasic* rpc_ptr = RpcBasic::getInstance();
	RequestMessageType_Int32 req_data;
	ResponseMessageType_Uint64_DoubleList rep_data;
	req_data.header.time_stamp = 122;
	req_data.property.authority = Comm_Authority_TP_SIMMULATOR;
	req_data.data.data = cpu_id;
	if (!rpc_ptr->handleRpc(0x000003E1, &req_data, RequestMessageType_Int32_fields, &rep_data, ResponseMessageType_Uint64_DoubleList_fields))
	{
		return HANDLE_RPC_FAILED;
	}
	*data_size_ptr = rep_data.data.data_count;

	for (size_t i = 0; i < rep_data.data.data_count; ++i)
	{
		t_data[i] = rep_data.data.data[i];
	}
	return rep_data.error_code.data;
}

uint64_t c_servo1001CpuSetTorqueSensorSync(int32_t cpu_id)
{
	if (!rpc_valid)
		return HANDLE_RPC_FAILED;
	RpcBasic* rpc_ptr = RpcBasic::getInstance();
	RequestMessageType_Int32 req_data;
	ResponseMessageType_Uint64_Int32 rep_data;
	req_data.header.time_stamp = 122;
	req_data.property.authority = Comm_Authority_TP_SIMMULATOR;
	req_data.data.data = cpu_id;
	if (!rpc_ptr->handleRpc(0x00012853, &req_data, RequestMessageType_Int32_fields, &rep_data, ResponseMessageType_Uint64_Int32_fields))
	{
		return HANDLE_RPC_FAILED;
	}
	return rep_data.error_code.data;
}

uint64_t c_servo1001CpuGetTorqueSensorSync(int32_t cpu_id, int32_t * p_updateFlag)
{
	if (!rpc_valid)
		return HANDLE_RPC_FAILED;
	RpcBasic* rpc_ptr = RpcBasic::getInstance();
	RequestMessageType_Int32 req_data;
	ResponseMessageType_Uint64_Int32 rep_data;
	req_data.header.time_stamp = 122;
	req_data.property.authority = Comm_Authority_TP_SIMMULATOR;
	req_data.data.data = cpu_id;
	if (!rpc_ptr->handleRpc(0x00010583, &req_data, RequestMessageType_Int32_fields, &rep_data, ResponseMessageType_Uint64_Int32_fields))
	{
		return HANDLE_RPC_FAILED;
	}
	*p_updateFlag = rep_data.data.data;
	return rep_data.error_code.data;
}




