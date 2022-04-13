#include "rpc_interface.h"
#include "rpc_basic.h"
#include "common_error_code.h"
#include "protoc.h"


uint64_t c_axisPower(int32_t axis_id, int32_t enable)
{
	if (!rpc_valid)
		return HANDLE_RPC_FAILED;
	RpcBasic* rpc_ptr = RpcBasic::getInstance();
	RequestMessageType_Int32_Bool req_data;
	ResponseMessageType_Uint64 rep_data;

	req_data.header.time_stamp = 122;
	req_data.property.authority = Comm_Authority_TP_SIMMULATOR;
	req_data.data1.data = axis_id;
	req_data.data2.data = enable;
	if (!rpc_ptr->handleRpc(0x000053E2, &req_data, RequestMessageType_Int32_Bool_fields, &rep_data, ResponseMessageType_Uint64_fields))
	{
		return HANDLE_RPC_FAILED;
	}
	return rep_data.data.data;
}

uint64_t c_axisReset(int32_t axis_id)
{
	if (!rpc_valid)
		return HANDLE_RPC_FAILED;
	RpcBasic* rpc_ptr = RpcBasic::getInstance();
	RequestMessageType_Int32 req_data;
	ResponseMessageType_Uint64 rep_data;

	req_data.header.time_stamp = 122;
	req_data.property.authority = Comm_Authority_TP_SIMMULATOR;
	req_data.data.data = axis_id;
	if (!rpc_ptr->handleRpc(0x000180C4, &req_data, RequestMessageType_Int32_fields, &rep_data, ResponseMessageType_Uint64_fields))
	{
		return HANDLE_RPC_FAILED;
	}
	return rep_data.data.data;
}

uint64_t c_axisStop(int32_t axis_id)
{
	if (!rpc_valid)
		return HANDLE_RPC_FAILED;
	RpcBasic* rpc_ptr = RpcBasic::getInstance();
	RequestMessageType_Int32 req_data;
	ResponseMessageType_Uint64 rep_data;

	req_data.header.time_stamp = 122;
	req_data.property.authority = Comm_Authority_TP_SIMMULATOR;
	req_data.data.data = axis_id;
	if (!rpc_ptr->handleRpc(0x00002820, &req_data, RequestMessageType_Int32_fields, &rep_data, ResponseMessageType_Uint64_fields))
	{
		return HANDLE_RPC_FAILED;
	}
	return rep_data.data.data;
}

uint64_t c_axisHalt(int32_t axis_id)
{
	if (!rpc_valid)
		return HANDLE_RPC_FAILED;
	RpcBasic* rpc_ptr = RpcBasic::getInstance();
	RequestMessageType_Int32 req_data;
	ResponseMessageType_Uint64 rep_data;

	req_data.header.time_stamp = 122;
	req_data.property.authority = Comm_Authority_TP_SIMMULATOR;
	req_data.data.data = axis_id;
	if (!rpc_ptr->handleRpc(0x00004BB4, &req_data, RequestMessageType_Int32_fields, &rep_data, ResponseMessageType_Uint64_fields))
	{
		return HANDLE_RPC_FAILED;
	}
	return rep_data.data.data;
}

uint64_t c_axisSetPosition(int32_t axis_id, double position)
{
	if (!rpc_valid)
		return HANDLE_RPC_FAILED;
	RpcBasic* rpc_ptr = RpcBasic::getInstance();
	RequestMessageType_Int32_Double req_data;
	ResponseMessageType_Uint64 rep_data;

	req_data.header.time_stamp = 122;
	req_data.property.authority = Comm_Authority_TP_SIMMULATOR;
	req_data.data1.data = axis_id;
	req_data.data2.data = position;
	if (!rpc_ptr->handleRpc(0x0001798E, &req_data, RequestMessageType_Int32_Double_fields, &rep_data, ResponseMessageType_Uint64_fields))
	{
		return HANDLE_RPC_FAILED;
	}
	return rep_data.data.data;
}

uint64_t c_axisReadParameter(int32_t axis_id, int32_t param_index, int32_t* param_value)
{
	if (!rpc_valid)
		return HANDLE_RPC_FAILED;
	RpcBasic* rpc_ptr = RpcBasic::getInstance();
	RequestMessageType_Int32List req_data;
	ResponseMessageType_Uint64_Int32 rep_data;
	
	req_data.header.time_stamp = 122;
	req_data.property.authority = Comm_Authority_TP_SIMMULATOR;
	req_data.data.data_count = 2;
	req_data.data.data[0] = axis_id;
	req_data.data.data[1] = param_index;
	if (!rpc_ptr->handleRpc(0x00016BF2, &req_data, RequestMessageType_Int32List_fields, &rep_data, ResponseMessageType_Uint64_Int32_fields))
	{
		return HANDLE_RPC_FAILED;
	}
	*param_value = rep_data.data.data;
	return rep_data.error_code.data;
}

uint64_t c_axisWriteParameter(int32_t axis_id, int32_t param_index, int32_t param_value)
{
	if (!rpc_valid)
		return HANDLE_RPC_FAILED;
	RpcBasic* rpc_ptr = RpcBasic::getInstance();
	RequestMessageType_Int32List req_data;
	ResponseMessageType_Uint64 rep_data;

	req_data.header.time_stamp = 122;
	req_data.property.authority = Comm_Authority_TP_SIMMULATOR;
	req_data.data.data_count = 3;
	req_data.data.data[0] = axis_id;
	req_data.data.data[1] = param_index;
	req_data.data.data[2] = param_value;
	if (!rpc_ptr->handleRpc(0x00005732, &req_data, RequestMessageType_Int32List_fields, &rep_data, ResponseMessageType_Uint64_fields))
	{
		return HANDLE_RPC_FAILED;
	}
	return rep_data.data.data;
}

uint64_t c_axisMoveAbsolute(int32_t axis_id, double position, double velocity, double acc, double dec, double jerk)
{
	if (!rpc_valid)
		return HANDLE_RPC_FAILED;
	RpcBasic* rpc_ptr = RpcBasic::getInstance();
	RequestMessageType_Int32_DoubleList req_data;
	ResponseMessageType_Uint64 rep_data;

	req_data.header.time_stamp = 122;
	req_data.property.authority = Comm_Authority_TP_SIMMULATOR;
	req_data.data1.data = axis_id;
	req_data.data2.data_count = 5;
	req_data.data2.data[0] = position;
	req_data.data2.data[1] = velocity;
	req_data.data2.data[2] = acc;
	req_data.data2.data[3] = dec;
	req_data.data2.data[4] = jerk;
	if (!rpc_ptr->handleRpc(0x000051F5, &req_data, RequestMessageType_Int32_DoubleList_fields, &rep_data, ResponseMessageType_Uint64_fields))
	{
		return HANDLE_RPC_FAILED;
	}
	return rep_data.data.data;
}

uint64_t c_axisMoveVelocity(int32_t axis_id, int32_t direction, double velocity, double acc, double dec, double jerk)
{
	if (!rpc_valid)
		return HANDLE_RPC_FAILED;
	RpcBasic* rpc_ptr = RpcBasic::getInstance();
	RequestMessageType_Int32List_DoubleList req_data;
	ResponseMessageType_Uint64 rep_data;

	req_data.header.time_stamp = 122;
	req_data.property.authority = Comm_Authority_TP_SIMMULATOR;
	req_data.data1.data_count = 2;
	req_data.data1.data[0] = axis_id;
	req_data.data1.data[1] = direction;
	req_data.data2.data_count = 4;
	req_data.data2.data[0] = velocity;
	req_data.data2.data[1] = acc;
	req_data.data2.data[2] = dec;
	req_data.data2.data[3] = jerk;
	if (!rpc_ptr->handleRpc(0x00016CF9, &req_data, RequestMessageType_Int32List_DoubleList_fields, &rep_data, ResponseMessageType_Uint64_fields))
	{
		return HANDLE_RPC_FAILED;
	}
	return rep_data.data.data;
}

uint64_t c_axisReadActualPosition(int32_t axis_id, double* position)
{
	if (!rpc_valid)
		return HANDLE_RPC_FAILED;
	RpcBasic* rpc_ptr = RpcBasic::getInstance();
	RequestMessageType_Int32 req_data;
	ResponseMessageType_Uint64_Double rep_data;

	req_data.header.time_stamp = 122;
	req_data.property.authority = Comm_Authority_TP_SIMMULATOR;
	req_data.data.data = axis_id;
	if (!rpc_ptr->handleRpc(0x000012BE, &req_data, RequestMessageType_Int32_fields, &rep_data, ResponseMessageType_Uint64_Double_fields))
	{
		return HANDLE_RPC_FAILED;
	}
	*position = rep_data.data.data;
	return rep_data.error_code.data;
}

uint64_t c_axisReadActualVelocity(int32_t axis_id, double* velocity)
{
	if (!rpc_valid)
		return HANDLE_RPC_FAILED;
	RpcBasic* rpc_ptr = RpcBasic::getInstance();
	RequestMessageType_Int32 req_data;
	ResponseMessageType_Uint64_Double rep_data;

	req_data.header.time_stamp = 122;
	req_data.property.authority = Comm_Authority_TP_SIMMULATOR;
	req_data.data.data = axis_id;
	if (!rpc_ptr->handleRpc(0x00002EA9, &req_data, RequestMessageType_Int32_fields, &rep_data, ResponseMessageType_Uint64_Double_fields))
	{
		return HANDLE_RPC_FAILED;
	}
	*velocity = rep_data.data.data;
	return rep_data.error_code.data;
}

uint64_t c_axisReadActualTorque(int32_t axis_id, double* torque)
{
	if (!rpc_valid)
		return HANDLE_RPC_FAILED;
	RpcBasic* rpc_ptr = RpcBasic::getInstance();
	RequestMessageType_Int32 req_data;
	ResponseMessageType_Uint64_Double rep_data;

	req_data.header.time_stamp = 122;
	req_data.property.authority = Comm_Authority_TP_SIMMULATOR;
	req_data.data.data = axis_id;
	if (!rpc_ptr->handleRpc(0x00014265, &req_data, RequestMessageType_Int32_fields, &rep_data, ResponseMessageType_Uint64_Double_fields))
	{
		return HANDLE_RPC_FAILED;
	}
	*torque = rep_data.data.data;
	return rep_data.error_code.data;
}

uint64_t c_axisReadAxisInfo(int32_t axis_id, int32_t* simulation, int32_t* comm_ready, int32_t* ready_power_on, int32_t* power_on)
{
	if (!rpc_valid)
		return HANDLE_RPC_FAILED;
	RpcBasic* rpc_ptr = RpcBasic::getInstance();
	RequestMessageType_Int32 req_data;
	ResponseMessageType_Uint64_AxisInfo rep_data;

	req_data.header.time_stamp = 122;
	req_data.property.authority = Comm_Authority_TP_SIMMULATOR;
	req_data.data.data = axis_id;
	if (!rpc_ptr->handleRpc(0x0000314F, &req_data, RequestMessageType_Int32_fields, &rep_data, ResponseMessageType_Uint64_AxisInfo_fields))
	{
		return HANDLE_RPC_FAILED;
	}
	*simulation = rep_data.data.simulation;
	*comm_ready = rep_data.data.communication_ready;
	*ready_power_on = rep_data.data.ready_for_power_on;
	*power_on = rep_data.data.power_on;
	return rep_data.error_code.data;
}

uint64_t c_axisReadAxisStatus(int32_t axis_id, int32_t* status)
{
	if (!rpc_valid)
		return HANDLE_RPC_FAILED;
	RpcBasic* rpc_ptr = RpcBasic::getInstance();
	RequestMessageType_Int32 req_data;
	ResponseMessageType_Uint64_AxisStatus rep_data;

	req_data.header.time_stamp = 122;
	req_data.property.authority = Comm_Authority_TP_SIMMULATOR;
	req_data.data.data = axis_id;
	if (!rpc_ptr->handleRpc(0x00003E53, &req_data, RequestMessageType_Int32_fields, &rep_data, ResponseMessageType_Uint64_AxisStatus_fields))
	{
		return HANDLE_RPC_FAILED;
	}
	*status = rep_data.data;
	return rep_data.error_code.data;
}



uint64_t c_axisReadAxisError(int32_t axis_id, uint64_t* error)
{
	if (!rpc_valid)
		return HANDLE_RPC_FAILED;
	RpcBasic* rpc_ptr = RpcBasic::getInstance();
	RequestMessageType_Int32 req_data;
	ResponseMessageType_Uint64_Uint64 rep_data;

	req_data.header.time_stamp = 122;
	req_data.property.authority = Comm_Authority_TP_SIMMULATOR;
	req_data.data.data = axis_id;
	if (!rpc_ptr->handleRpc(0x000063C2, &req_data, RequestMessageType_Int32_fields, &rep_data, ResponseMessageType_Uint64_Uint64_fields))
	{
		return HANDLE_RPC_FAILED;
	}

	*error = rep_data.data.data;
	return rep_data.error_code.data;
}

uint64_t c_axisReadAxisErrorHistory(int32_t axis_id, uint64_t error_list[8], int32_t* error_list_size)
{
	if (!rpc_valid)
		return HANDLE_RPC_FAILED;
	RpcBasic* rpc_ptr = RpcBasic::getInstance();
	RequestMessageType_Int32 req_data;
	ResponseMessageType_Uint64_Uint64List rep_data;

	req_data.header.time_stamp = 122;
	req_data.property.authority = Comm_Authority_TP_SIMMULATOR;
	req_data.data.data = axis_id;
	if (!rpc_ptr->handleRpc(0x00018469, &req_data, RequestMessageType_Int32_fields, &rep_data, ResponseMessageType_Uint64_Uint64List_fields))
	{
		return HANDLE_RPC_FAILED;
	}

	// note: error_list_size should be smaller than 8
	for (unsigned int i = 0; i < rep_data.data.data_count; ++i)
	{
		error_list[i] = rep_data.data.data[i];
	}
	*error_list_size = rep_data.data.data_count;
	return rep_data.error_code.data;
}

uint64_t c_axisMoveRelative(int32_t axis_id, double position, double velocity, double acc, double dec, double jerk)
{
	if (!rpc_valid)
		return HANDLE_RPC_FAILED;
	RpcBasic* rpc_ptr = RpcBasic::getInstance();
	RequestMessageType_Int32_DoubleList req_data;
	ResponseMessageType_Uint64 rep_data;

	req_data.header.time_stamp = 122;
	req_data.property.authority = Comm_Authority_TP_SIMMULATOR;
	req_data.data1.data = axis_id;
	req_data.data2.data_count = 5;
	req_data.data2.data[0] = position;
	req_data.data2.data[1] = velocity;
	req_data.data2.data[2] = acc;
	req_data.data2.data[3] = dec;
	req_data.data2.data[4] = jerk;
	if (!rpc_ptr->handleRpc(0x0000CC85, &req_data, RequestMessageType_Int32_DoubleList_fields, &rep_data, ResponseMessageType_Uint64_fields))
	{
		return HANDLE_RPC_FAILED;
	}
	return rep_data.data.data;
}

uint64_t c_axisHome(int32_t axis_id)
{
	if (!rpc_valid)
		return HANDLE_RPC_FAILED;
	RpcBasic* rpc_ptr = RpcBasic::getInstance();
	RequestMessageType_Int32 req_data;
	ResponseMessageType_Uint64 rep_data;

	req_data.header.time_stamp = 122;
	req_data.property.authority = Comm_Authority_TP_SIMMULATOR;
	req_data.data.data = axis_id;
	if (!rpc_ptr->handleRpc(0x000059B5, &req_data, RequestMessageType_Int32_fields, &rep_data, ResponseMessageType_Uint64_fields))
	{
		return HANDLE_RPC_FAILED;
	}
	return rep_data.data.data;
}

uint64_t c_axisAbortHoming(int32_t axis_id)
{
	if (!rpc_valid)
		return HANDLE_RPC_FAILED;
	RpcBasic* rpc_ptr = RpcBasic::getInstance();
	RequestMessageType_Int32 req_data;
	ResponseMessageType_Uint64 rep_data;

	req_data.header.time_stamp = 122;
	req_data.property.authority = Comm_Authority_TP_SIMMULATOR;
	req_data.data.data = axis_id;
	if (!rpc_ptr->handleRpc(0x0000E4B7, &req_data, RequestMessageType_Int32_fields, &rep_data, ResponseMessageType_Uint64_fields))
	{
		return HANDLE_RPC_FAILED;
	}
	return rep_data.data.data;
}

uint64_t c_axisResetEncoder(int32_t axis_id)
{
	if (!rpc_valid)
		return HANDLE_RPC_FAILED;
	RpcBasic* rpc_ptr = RpcBasic::getInstance();
	RequestMessageType_Int32 req_data;
	ResponseMessageType_Uint64 rep_data;

	req_data.header.time_stamp = 122;
	req_data.property.authority = Comm_Authority_TP_SIMMULATOR;
	req_data.data.data = axis_id;
	if (!rpc_ptr->handleRpc(0x00000BA2, &req_data, RequestMessageType_Int32_fields, &rep_data, ResponseMessageType_Uint64_fields))
	{
		return HANDLE_RPC_FAILED;
	}
	return rep_data.data.data;
}

