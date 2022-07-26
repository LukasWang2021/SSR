//#include "stdafx.h"
#include "rpc_interface.h"
#include "rpc_basic.h"
#include "common_error_code.h"
#include <string.h>
#include <time.h>
#include "protoc.h"

uint64_t c_servo1000ServoShutDown(int32_t cpu_id, int32_t servo_id)
{
	RpcBasic* rpc_ptr = RpcBasic::getInstance();
	RequestMessageType_Int32List req_data;
	ResponseMessageType_Uint64 rep_data;

	req_data.header.time_stamp = 122;
	req_data.property.authority = Comm_Authority_TP_SIMMULATOR;
	req_data.data.data_count = 2;
	req_data.data.data[0] = cpu_id;
	req_data.data.data[1] = servo_id;
	if (!rpc_ptr->handleRpc(0x0000563E, &req_data, RequestMessageType_Int32List_fields, &rep_data, ResponseMessageType_Uint64_fields))
	{
		return HANDLE_RPC_FAILED;
	}
	return rep_data.data.data;
}

uint64_t c_servo1000ServoSwitchOn(int32_t cpu_id, int32_t servo_id)
{
	RpcBasic* rpc_ptr = RpcBasic::getInstance();
	RequestMessageType_Int32List req_data;
	ResponseMessageType_Uint64 rep_data;

	req_data.header.time_stamp = 122;
	req_data.property.authority = Comm_Authority_TP_SIMMULATOR;
	req_data.data.data_count = 2;
	req_data.data.data[0] = cpu_id;
	req_data.data.data[1] = servo_id;
	if (!rpc_ptr->handleRpc(0x000095CE, &req_data, RequestMessageType_Int32List_fields, &rep_data, ResponseMessageType_Uint64_fields))
	{
		return HANDLE_RPC_FAILED;
	}
	return rep_data.data.data;
}

uint64_t c_servo1000ServoDisableVoltage(int32_t cpu_id, int32_t servo_id)
{
	RpcBasic* rpc_ptr = RpcBasic::getInstance();
	RequestMessageType_Int32List req_data;
	ResponseMessageType_Uint64 rep_data;

	req_data.header.time_stamp = 122;
	req_data.property.authority = Comm_Authority_TP_SIMMULATOR;
	req_data.data.data_count = 2;
	req_data.data.data[0] = cpu_id;
	req_data.data.data[1] = servo_id;
	if (!rpc_ptr->handleRpc(0x00005755, &req_data, RequestMessageType_Int32List_fields, &rep_data, ResponseMessageType_Uint64_fields))
	{
		return HANDLE_RPC_FAILED;
	}
	return rep_data.data.data;
}

uint64_t c_servo1000ServoEnableOperation(int32_t cpu_id, int32_t servo_id)
{
	RpcBasic* rpc_ptr = RpcBasic::getInstance();
	RequestMessageType_Int32List req_data;
	ResponseMessageType_Uint64 rep_data;

	req_data.header.time_stamp = 122;
	req_data.property.authority = Comm_Authority_TP_SIMMULATOR;
	req_data.data.data_count = 2;
	req_data.data.data[0] = cpu_id;
	req_data.data.data[1] = servo_id;
	if (!rpc_ptr->handleRpc(0x00003E7E, &req_data, RequestMessageType_Int32List_fields, &rep_data, ResponseMessageType_Uint64_fields))
	{
		return HANDLE_RPC_FAILED;
	}
	return rep_data.data.data;
}

uint64_t c_servo1000ServoSwitchOnAndEnableOperation(int32_t cpu_id, int32_t servo_id)
{
	RpcBasic* rpc_ptr = RpcBasic::getInstance();
	RequestMessageType_Int32List req_data;
	ResponseMessageType_Uint64 rep_data;

	req_data.header.time_stamp = 122;
	req_data.property.authority = Comm_Authority_TP_SIMMULATOR;
	req_data.data.data_count = 2;
	req_data.data.data[0] = cpu_id;
	req_data.data.data[1] = servo_id;
	if (!rpc_ptr->handleRpc(0x0000012E, &req_data, RequestMessageType_Int32List_fields, &rep_data, ResponseMessageType_Uint64_fields))
	{
		return HANDLE_RPC_FAILED;
	}
	return rep_data.data.data;
}


uint64_t c_servo1000ServoDisableOperation(int32_t cpu_id, int32_t servo_id)
{
	RpcBasic* rpc_ptr = RpcBasic::getInstance();
	RequestMessageType_Int32List req_data;
	ResponseMessageType_Uint64 rep_data;

	req_data.header.time_stamp = 122;
	req_data.property.authority = Comm_Authority_TP_SIMMULATOR;
	req_data.data.data_count = 2;
	req_data.data.data[0] = cpu_id;
	req_data.data.data[1] = servo_id;
	if (!rpc_ptr->handleRpc(0x0000FAAE, &req_data, RequestMessageType_Int32List_fields, &rep_data, ResponseMessageType_Uint64_fields))
	{
		return HANDLE_RPC_FAILED;
	}
	return rep_data.data.data;
}

uint64_t c_servo1000ServoQuickStop(int32_t cpu_id, int32_t servo_id)
{
	RpcBasic* rpc_ptr = RpcBasic::getInstance();
	RequestMessageType_Int32List req_data;
	ResponseMessageType_Uint64 rep_data;

	req_data.header.time_stamp = 122;
	req_data.property.authority = Comm_Authority_TP_SIMMULATOR;
	req_data.data.data_count = 2;
	req_data.data.data[0] = cpu_id;
	req_data.data.data[1] = servo_id;
	if (!rpc_ptr->handleRpc(0x000012C0, &req_data, RequestMessageType_Int32List_fields, &rep_data, ResponseMessageType_Uint64_fields))
	{
		return HANDLE_RPC_FAILED;
	}
	return rep_data.data.data;
}

uint64_t c_servo1000ServoResetFault(int32_t cpu_id, int32_t servo_id)
{
	RpcBasic* rpc_ptr = RpcBasic::getInstance();
	RequestMessageType_Int32List req_data;
	ResponseMessageType_Uint64 rep_data;

	req_data.header.time_stamp = 122;
	req_data.property.authority = Comm_Authority_TP_SIMMULATOR;
	req_data.data.data_count = 2;
	req_data.data.data[0] = cpu_id;
	req_data.data.data[1] = servo_id;
	if (!rpc_ptr->handleRpc(0x000052E4, &req_data, RequestMessageType_Int32List_fields, &rep_data, ResponseMessageType_Uint64_fields))
	{
		return HANDLE_RPC_FAILED;
	}
	return rep_data.data.data;
}

uint64_t c_servo1000ServoTransCommState(int32_t cpu_id, int32_t servo_id, int32_t state)
{
	RpcBasic* rpc_ptr = RpcBasic::getInstance();
	RequestMessageType_Int32List_CoreCommState req_data;
	ResponseMessageType_Uint64 rep_data;

	req_data.header.time_stamp = 122;
	req_data.property.authority = Comm_Authority_TP_SIMMULATOR;
	req_data.data1.data_count = 2;
	req_data.data1.data[0] = cpu_id;
	req_data.data1.data[1] = servo_id;
	req_data.data2 = (MessageType_CoreCommState)state;
	if (!rpc_ptr->handleRpc(0x000183C5, &req_data, RequestMessageType_Int32List_CoreCommState_fields, &rep_data, ResponseMessageType_Uint64_fields))
	{
		return HANDLE_RPC_FAILED;
	}
	return rep_data.data.data;
}

uint64_t c_servo1000ServoReadParameter(int32_t cpu_id, int32_t servo_id, int32_t param_index, int32_t* value)
{
	RpcBasic* rpc_ptr = RpcBasic::getInstance();
	RequestMessageType_Int32List req_data;
	ResponseMessageType_Uint64_Int32 rep_data;

	req_data.header.time_stamp = 122;
	req_data.property.authority = Comm_Authority_TP_SIMMULATOR;
	req_data.data.data_count = 3;
	req_data.data.data[0] = cpu_id;
	req_data.data.data[1] = servo_id;
	req_data.data.data[2] = param_index;
	if (!rpc_ptr->handleRpc(0x00006D92, &req_data, RequestMessageType_Int32List_fields, &rep_data, ResponseMessageType_Uint64_Int32_fields))
	{
		return HANDLE_RPC_FAILED;
	}
	*value = rep_data.data.data;
	return rep_data.error_code.data;
}

uint64_t c_servo1000ServoWriteParameter(int32_t cpu_id, int32_t servo_id, int32_t param_index, int32_t param_value)
{
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
	if (!rpc_ptr->handleRpc(0x0000CC32, &req_data, RequestMessageType_Int32List_fields, &rep_data, ResponseMessageType_Uint64_fields))
	{
		return HANDLE_RPC_FAILED;
	}
	return rep_data.data.data;
}


uint64_t c_servo1000ServoMoveVelocity(int32_t cpu_id, int32_t servo_id, int32_t velocity, int32_t acc, int32_t dec, int32_t jerk, int32_t direction)
{
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
	if (!rpc_ptr->handleRpc(0x00016509, &req_data, RequestMessageType_Int32List_fields, &rep_data, ResponseMessageType_Uint64_fields))
	{
		return HANDLE_RPC_FAILED;
	}
	return rep_data.data.data;
}


uint64_t c_servo1000ServoMoveAbsolute(int32_t cpu_id, int32_t servo_id, int64_t position, int32_t velocity, int32_t acc, int32_t dec, int32_t jerk)
{
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
	if (!rpc_ptr->handleRpc(0x00004E05, &req_data, RequestMessageType_Int32List_Int64_fields, &rep_data, ResponseMessageType_Uint64_fields))
	{
		return HANDLE_RPC_FAILED;
	}
	return rep_data.data.data;
}

uint64_t c_servo1000ServoTriggerUploadParameters(int32_t cpu_id, int32_t servo_id)
{
	RpcBasic* rpc_ptr = RpcBasic::getInstance();
	RequestMessageType_Int32List req_data;
	ResponseMessageType_Uint64 rep_data;

	req_data.header.time_stamp = 122;
	req_data.property.authority = Comm_Authority_TP_SIMMULATOR;
	req_data.data.data_count = 2;
	req_data.data.data[0] = cpu_id;
	req_data.data.data[1] = servo_id;
	if (!rpc_ptr->handleRpc(0x00009C53, &req_data, RequestMessageType_Int32List_fields, &rep_data, ResponseMessageType_Uint64_fields))
	{
		return HANDLE_RPC_FAILED;
	}
	return rep_data.data.data;
}


uint64_t c_servo1000ServoUploadParameters(int32_t cpu_id, int32_t servo_id, int32_t value[512])
{
	RpcBasic* rpc_ptr = RpcBasic::getInstance();
	RequestMessageType_Int32List req_data;
	ResponseMessageType_Uint64_ParamDetailList rep_data;

	req_data.header.time_stamp = 122;
	req_data.property.authority = Comm_Authority_TP_SIMMULATOR;
	req_data.data.data_count = 2;
	req_data.data.data[0] = cpu_id;
	req_data.data.data[1] = servo_id;
	if (!rpc_ptr->handleRpc(0x000000E3, &req_data, RequestMessageType_Int32List_fields, &rep_data, ResponseMessageType_Uint64_ParamDetailList_fields))
	{
		return HANDLE_RPC_FAILED;
	}

	for (unsigned int i = 0; i < rep_data.data.data_count; ++i)
	{
		value[i] = rep_data.data.data[i].operation_value;
		//value[i][1] = rep_data.data.data[i].default_value;
		//value[i][2] = rep_data.data.data[i].upper_limit_value;
		//value[i][3] = rep_data.data.data[i].lower_limit_value;
		//value[i][4] = rep_data.data.data[i].attr;
		//value[i][5] = rep_data.data.data[i].validity;
	}
	return rep_data.error_code.data;
}

/*
uint64_t c_servo1000ServoUploadParameters(int32_t cpu_id, int32_t servo_id, char file_path[128])
{
	RpcBasic* rpc_ptr = RpcBasic::getInstance();
	RequestMessageType_Int32List req_data;
	ResponseMessageType_Uint64_ParamDetailList rep_data;

	req_data.header.time_stamp = 122;
	req_data.property.authority = Comm_Authority_TP_SIMMULATOR;
	req_data.data.data_count = 2;
	req_data.data.data[0] = cpu_id;
	req_data.data.data[1] = servo_id;
	if (!rpc_ptr->handleRpc(0x000000E3, &req_data, RequestMessageType_Int32List_fields, &rep_data, ResponseMessageType_Uint64_ParamDetailList_fields))
	{
		return HANDLE_RPC_FAILED;
	}

	time_t now;
	struct tm t;
	time(&now);
	localtime_s(&t, &now);
	char buf[128] = {0};
	sprintf_s(buf, "ParamList_%04d-%02d-%02d_%02d:%02d:%02d.csv", t.tm_year, t.tm_mon, t.tm_mday, t.tm_hour, t.tm_min, t.tm_sec);
	strcpy_s(file_path, strlen(buf) + 1, buf);

	FILE *pfile = fopen(file_path, "w+");
	if (pfile == NULL)
	{
		return 1;
	}
    for (unsigned int i = 0; i < rep_data.data.data_count; ++i)
	{
		fprintf(pfile, "%d,%d,%d,%d,%d,%d,%d,%s\n", i, rep_data.data.data[i].operation_value, rep_data.data.data[i].default_value,
				rep_data.data.data[i].upper_limit_value, rep_data.data.data[i].lower_limit_value, rep_data.data.data[i].attr,
				rep_data.data.data[i].validity, rep_data.data.data[i].unit);
	}
	fclose(pfile);
	
	return rep_data.error_code.data;
}
*/


uint64_t c_servo1000ServoTriggerDownloadParameters(int32_t cpu_id, int32_t servo_id)
{
	RpcBasic* rpc_ptr = RpcBasic::getInstance();
	RequestMessageType_Int32List req_data;
	ResponseMessageType_Uint64 rep_data;

	req_data.header.time_stamp = 122;
	req_data.property.authority = Comm_Authority_TP_SIMMULATOR;
	req_data.data.data_count = 2;
	req_data.data.data[0] = cpu_id;
	req_data.data.data[1] = servo_id;
	if (!rpc_ptr->handleRpc(0x000010B3, &req_data, RequestMessageType_Int32List_fields, &rep_data, ResponseMessageType_Uint64_fields))
	{
		return HANDLE_RPC_FAILED;
	}
	return rep_data.data.data;
}

uint64_t c_servo1000ServoDownloadParameters(int32_t cpu_id, int32_t servo_id, int32_t param_value[512], int32_t param_size)
{
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
	if (!rpc_ptr->handleRpc(0x00017053, &req_data, RequestMessageType_Int32List_Int32List_fields, &rep_data, ResponseMessageType_Uint64_fields))
	{
		return HANDLE_RPC_FAILED;
	}
	return rep_data.data.data;
}

uint64_t c_servo1000ServoIsAsyncServiceFinish(int32_t cpu_id, int32_t servo_id, int32_t* sync)
{
	RpcBasic* rpc_ptr = RpcBasic::getInstance();
	RequestMessageType_Int32List req_data;
	ResponseMessageType_Uint64_Bool rep_data;

	req_data.header.time_stamp = 122;
	req_data.property.authority = Comm_Authority_TP_SIMMULATOR;
	req_data.data.data_count = 2;
	req_data.data.data[0] = cpu_id;
	req_data.data.data[1] = servo_id;
	if (!rpc_ptr->handleRpc(0x000073B8, &req_data, RequestMessageType_Int32List_fields, &rep_data, ResponseMessageType_Uint64_Bool_fields))
	{
		return HANDLE_RPC_FAILED;
	}
	*sync = rep_data.data.data;
	return rep_data.error_code.data;
}

uint64_t c_servo1000ServoGetCommState(int32_t cpu_id, int32_t servo_id, int32_t* state)
{
	RpcBasic* rpc_ptr = RpcBasic::getInstance();
	RequestMessageType_Int32List req_data;
	ResponseMessageType_Uint64_CoreCommState rep_data;

	req_data.header.time_stamp = 122;
	req_data.property.authority = Comm_Authority_TP_SIMMULATOR;
	req_data.data.data_count = 2;
	req_data.data.data[0] = cpu_id;
	req_data.data.data[1] = servo_id;
	if (!rpc_ptr->handleRpc(0x0000F475, &req_data, RequestMessageType_Int32List_fields, &rep_data, ResponseMessageType_Uint64_CoreCommState_fields))
	{
		return HANDLE_RPC_FAILED;
	}
	*state = rep_data.data;
	return rep_data.error_code.data;
}




uint64_t c_servo1000CpuGetVersion(int32_t cpu_id, int32_t* major_version, int32_t* minor_version)
{
	RpcBasic* rpc_ptr = RpcBasic::getInstance();
	RequestMessageType_Int32 req_data;
	ResponseMessageType_Uint64_Uint32List rep_data;

	req_data.header.time_stamp = 122;
	req_data.property.authority = Comm_Authority_TP_SIMMULATOR;
	req_data.data.data = cpu_id;
	if (!rpc_ptr->handleRpc(0x0000028E, &req_data, RequestMessageType_Int32List_fields, &rep_data, ResponseMessageType_Uint64_Uint32List_fields))
	{
		return HANDLE_RPC_FAILED;
	}
	*major_version = rep_data.data.data[0];
	*minor_version = rep_data.data.data[1];
	return rep_data.error_code.data;
}

uint64_t c_servo1000CpuSetCtrlPdoSync(int32_t cpu_id, int32_t servo_id, uint32_t sync)
{
	RpcBasic* rpc_ptr = RpcBasic::getInstance();
	RequestMessageType_Int32List_Uint32 req_data;
	ResponseMessageType_Uint64 rep_data;

	req_data.header.time_stamp = 122;
	req_data.property.authority = Comm_Authority_TP_SIMMULATOR;
	req_data.data1.data_count = 2;
	req_data.data1.data[0] = cpu_id;
	req_data.data1.data[1] = servo_id;
	req_data.data2.data = sync;
	if (!rpc_ptr->handleRpc(0x00005193, &req_data, RequestMessageType_Int32List_Uint32_fields, &rep_data, ResponseMessageType_Uint64_fields))
	{
		return HANDLE_RPC_FAILED;
	}
	return rep_data.data.data;
}

uint64_t c_servo1000CpuGetCtrlPdoSync(int32_t cpu_id, int32_t servo_id, uint32_t* sync)
{
	RpcBasic* rpc_ptr = RpcBasic::getInstance();
	RequestMessageType_Int32List req_data;
	ResponseMessageType_Uint64_Int32_Uint32 rep_data;

	req_data.header.time_stamp = 122;
	req_data.property.authority = Comm_Authority_TP_SIMMULATOR;
	req_data.data.data_count = 2;
	req_data.data.data[0] = cpu_id;
	req_data.data.data[1] = servo_id;
	if (!rpc_ptr->handleRpc(0x00005453, &req_data, RequestMessageType_Int32List_fields, &rep_data, ResponseMessageType_Uint64_Int32_Uint32_fields))
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

uint64_t c_servo1000CpuSetSamplingSync(int32_t cpu_id, uint32_t sync)
{
	RpcBasic* rpc_ptr = RpcBasic::getInstance();
	RequestMessageType_Int32_Uint32 req_data;
	ResponseMessageType_Uint64 rep_data;

	req_data.header.time_stamp = 122;
	req_data.property.authority = Comm_Authority_TP_SIMMULATOR;
	req_data.data1.data = cpu_id;
	req_data.data2.data = sync;
	if (!rpc_ptr->handleRpc(0x00003F23, &req_data, RequestMessageType_Int32_Uint32_fields, &rep_data, ResponseMessageType_Uint64_fields))
	{
		return HANDLE_RPC_FAILED;
	}
	return rep_data.data.data;
}

uint64_t c_servo1000CpuGetSamplingSync(int32_t cpu_id, uint32_t* sync)
{
	RpcBasic* rpc_ptr = RpcBasic::getInstance();
	RequestMessageType_Int32 req_data;
	ResponseMessageType_Uint64_Uint32 rep_data;

	req_data.header.time_stamp = 122;
	req_data.property.authority = Comm_Authority_TP_SIMMULATOR;
	req_data.data.data = cpu_id;
	if (!rpc_ptr->handleRpc(0x00005323, &req_data, RequestMessageType_Int32_fields, &rep_data, ResponseMessageType_Uint64_Uint32_fields))
	{
		return HANDLE_RPC_FAILED;
	}
	*sync = rep_data.data.data;
	return rep_data.error_code.data;
}

uint64_t c_servo1000CpuSetSamplingInterval(int32_t cpu_id, uint32_t interval)
{
	RpcBasic* rpc_ptr = RpcBasic::getInstance();
	RequestMessageType_Int32_Uint32 req_data;
	ResponseMessageType_Uint64 rep_data;

	req_data.header.time_stamp = 122;
	req_data.property.authority = Comm_Authority_TP_SIMMULATOR;
	req_data.data1.data = cpu_id;
	req_data.data2.data = interval;
	if (!rpc_ptr->handleRpc(0x000097EC, &req_data, RequestMessageType_Int32_Uint32_fields, &rep_data, ResponseMessageType_Uint64_fields))
	{
		return HANDLE_RPC_FAILED;
	}
	return rep_data.data.data;
}

uint64_t c_servo1000CpuGetSamplingInterval(int32_t cpu_id, uint32_t* interval)
{
	RpcBasic* rpc_ptr = RpcBasic::getInstance();
	RequestMessageType_Int32 req_data;
	ResponseMessageType_Uint64_Uint32 rep_data;

	req_data.header.time_stamp = 122;
	req_data.property.authority = Comm_Authority_TP_SIMMULATOR;
	req_data.data.data = cpu_id;
	if (!rpc_ptr->handleRpc(0x0000BB5C, &req_data, RequestMessageType_Int32_fields, &rep_data, ResponseMessageType_Uint64_Uint32_fields))
	{
		return HANDLE_RPC_FAILED;
	}
	*interval = rep_data.data.data;
	return rep_data.error_code.data;
}

uint64_t c_servo1000CpuSetSamplingMaxTimes(int32_t cpu_id, uint32_t max_times)
{
	RpcBasic* rpc_ptr = RpcBasic::getInstance();
	RequestMessageType_Int32_Uint32 req_data;
	ResponseMessageType_Uint64 rep_data;

	req_data.header.time_stamp = 122;
	req_data.property.authority = Comm_Authority_TP_SIMMULATOR;
	req_data.data1.data = cpu_id;
	req_data.data2.data = max_times;
	if (!rpc_ptr->handleRpc(0x0000B7A3, &req_data, RequestMessageType_Int32_Uint32_fields, &rep_data, ResponseMessageType_Uint64_fields))
	{
		return HANDLE_RPC_FAILED;
	}
	return rep_data.data.data;
}

uint64_t c_servo1000CpuGetSamplingMaxTimes(int32_t cpu_id, uint32_t* max_times)
{
	RpcBasic* rpc_ptr = RpcBasic::getInstance();
	RequestMessageType_Int32 req_data;
	ResponseMessageType_Uint64_Uint32 rep_data;

	req_data.header.time_stamp = 122;
	req_data.property.authority = Comm_Authority_TP_SIMMULATOR;
	req_data.data.data = cpu_id;
	if (!rpc_ptr->handleRpc(0x00009533, &req_data, RequestMessageType_Int32_fields, &rep_data, ResponseMessageType_Uint64_Uint32_fields))
	{
		return HANDLE_RPC_FAILED;
	}
	*max_times = rep_data.data.data;
	return rep_data.error_code.data;
}

uint64_t c_servo1000CpuSetSamplingChannel(int32_t cpu_id, uint32_t channel_index, int32_t servo_index, int32_t servo_param_index)
{
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
	if (!rpc_ptr->handleRpc(0x00014C1C, &req_data, RequestMessageType_Int32_Uint32List_fields, &rep_data, ResponseMessageType_Uint64_fields))
	{
		return HANDLE_RPC_FAILED;
	}
	return rep_data.data.data;
}
uint64_t c_servo1000CpuGetSamplingChannel(int32_t cpu_id, uint32_t value[16], int32_t* size)
{
	RpcBasic* rpc_ptr = RpcBasic::getInstance();
	RequestMessageType_Int32 req_data;
	ResponseMessageType_Uint64_Uint32List rep_data;

	req_data.header.time_stamp = 122;
	req_data.property.authority = Comm_Authority_TP_SIMMULATOR;
	req_data.data.data = cpu_id;
	if (!rpc_ptr->handleRpc(0x0000083C, &req_data, RequestMessageType_Int32_fields, &rep_data, ResponseMessageType_Uint64_Uint32List_fields))
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
uint64_t c_servo1000CpuActivateSamplingConfiguration(int32_t cpu_id)
{
	RpcBasic* rpc_ptr = RpcBasic::getInstance();
	RequestMessageType_Int32 req_data;
	ResponseMessageType_Uint64 rep_data;

	req_data.header.time_stamp = 122;
	req_data.property.authority = Comm_Authority_TP_SIMMULATOR;
	req_data.data.data = cpu_id;
	if (!rpc_ptr->handleRpc(0x000134BE, &req_data, RequestMessageType_Int32_fields, &rep_data, ResponseMessageType_Uint64_fields))
	{
		return HANDLE_RPC_FAILED;
	}
	return rep_data.data.data;
}


uint64_t c_servo1000CpuSaveSamplingBufferData(int32_t cpu_id, char* file_path_ptr)
{
	RpcBasic* rpc_ptr = RpcBasic::getInstance();
	RequestMessageType_Int32_String req_data;
	ResponseMessageType_Uint64 rep_data;

	req_data.header.time_stamp = 122;
	req_data.property.authority = Comm_Authority_TP_SIMMULATOR;
	req_data.data1.data = cpu_id;
	int file_path_size = (int)strlen(file_path_ptr);
	memcpy(req_data.data2.data, file_path_ptr, file_path_size);
	req_data.data2.data[file_path_size] = 0;
	if (!rpc_ptr->handleRpc(0x00018621, &req_data, RequestMessageType_Int32_String_fields, &rep_data, ResponseMessageType_Uint64_fields))
	{
		return HANDLE_RPC_FAILED;
	}
	return rep_data.data.data;
}

uint64_t c_servo1000ServoGetServoCommInfo(int32_t cpu_id, int32_t servo_id, int32_t* comm_reg_id, int32_t* service_id,
	int32_t* download_param_id, int32_t* upload_param_id, int32_t* ctrl_pdo_id, int32_t* fdb_pdo_id)
{
	RpcBasic* rpc_ptr = RpcBasic::getInstance();
	RequestMessageType_Int32List req_data;
	ResponseMessageType_Uint64_Int32List rep_data;

	req_data.header.time_stamp = 122;
	req_data.property.authority = Comm_Authority_TP_SIMMULATOR;
	req_data.data.data_count = 2;
	req_data.data.data[0] = cpu_id;
	req_data.data.data[1] = servo_id;
	if (!rpc_ptr->handleRpc(0x000171BF, &req_data, RequestMessageType_Int32List_fields, &rep_data, ResponseMessageType_Uint64_Int32List_fields))
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

uint64_t c_servo1000CpuGetServoCpuCommInfo(int32_t cpu_id, int32_t* comm_reg_id, int32_t* sampling_buffer_id)
{
	RpcBasic* rpc_ptr = RpcBasic::getInstance();
	RequestMessageType_Int32 req_data;
	ResponseMessageType_Uint64_Int32List rep_data;

	req_data.header.time_stamp = 122;
	req_data.property.authority = Comm_Authority_TP_SIMMULATOR;
	req_data.data.data = cpu_id;
	if (!rpc_ptr->handleRpc(0x000179FF, &req_data, RequestMessageType_Int32_fields, &rep_data, ResponseMessageType_Uint64_Int32List_fields))
	{
		return HANDLE_RPC_FAILED;
	}
	*comm_reg_id = rep_data.data.data[0];
	*sampling_buffer_id = rep_data.data.data[1];
	return rep_data.error_code.data;
}




