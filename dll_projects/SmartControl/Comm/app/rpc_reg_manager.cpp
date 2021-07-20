#include "stdafx.h"
#include <windows.h>
#include <thread>
#include "rpc_interface.h"
#include "rpc_basic.h"
#include "common_error_code.h"
#include <string>
#include <sstream>

using namespace std;

uint64_t c_getPrRegValidList(int32_t start_id, int32_t max_list_size, char* valid_list_str)
{
	RpcBasic* rpc_ptr = RpcBasic::getInstance();
	RequestMessageType_Int32List req_data;
	ResponseMessageType_Uint64_BaseRegSummaryList rep_data;

	req_data.property.authority = Comm_Authority_TP_SIMMULATOR;
	req_data.data.data_count = 2;
	req_data.data.data[0] = start_id;
	req_data.data.data[1] = max_list_size;
	if (!rpc_ptr->handleRpc(0x00009354, &req_data, RequestMessageType_Int32List_fields, &rep_data, ResponseMessageType_Uint64_BaseRegSummaryList_fields))
	{
		return HANDLE_RPC_FAILED;
	}

	std::string result;
	std::string id_str;
	for (unsigned int i = 0; i < rep_data.data.summary_count; ++i)
	{
		std::stringstream ss;
		ss << rep_data.data.summary[i].id;
		ss >> id_str;
		result += "PrReg ";
		result += id_str;
		result += "\r\n";
	}

	memcpy(valid_list_str, result.c_str(), result.size());
	valid_list_str[result.size()] = 0;

	return rep_data.error_code.data;
}

int32_t getTurnByJoint(double joint)
{
	if (joint >= 0)
	{
		return (int)floor((joint + 3.1415926535897932) / 6.2831853071795864);
	}
	else
	{
		return (int)ceil((joint - 3.1415926535897932) / 6.2831853071795864);
	}
}

uint64_t c_addPrReg(int32_t id, char* name, char* comment, int32_t pos_type,
	double j1, double j2, double j3, double j4, double j5, double j6, double j7, double j8, double j9,
	int32_t p_left_right, int32_t p_back_front, int32_t p_up_down, int32_t p_wrist_flip, 
	int32_t group_id)
{
	RpcBasic* rpc_ptr = RpcBasic::getInstance();
	RequestMessageType_PrRegData req_data;
	ResponseMessageType_Uint64 rep_data;

	req_data.property.authority = Comm_Authority_TP_SIMMULATOR;
	req_data.data.id = id;
	memcpy(req_data.data.name, name, strlen(name));
	req_data.data.name[strlen(name)] = 0;
	memcpy(req_data.data.comment, comment, strlen(comment));
	req_data.data.comment[strlen(comment)] = 0;
	req_data.data.pos_type = pos_type;
	req_data.data.pos.data_count = 9;
	req_data.data.pos.data[0] = j1;
	req_data.data.pos.data[1] = j2;
	req_data.data.pos.data[2] = j3;
	req_data.data.pos.data[3] = j4;
	req_data.data.pos.data[4] = j5;
	req_data.data.pos.data[5] = j6;
	req_data.data.pos.data[6] = j7;
	req_data.data.pos.data[7] = j8;
	req_data.data.pos.data[8] = j9;
	req_data.data.group_id = group_id;
	req_data.data.posture.arm_left_right = p_left_right;
	req_data.data.posture.arm_back_front = p_back_front;
	req_data.data.posture.arm_up_down = p_up_down;
	req_data.data.posture.wrist_flip = p_wrist_flip;
	req_data.data.posture.turn_cycle.data_count = 9;
	if (pos_type == 0)
	{
		req_data.data.posture.turn_cycle.data[0] = getTurnByJoint(j1);
		req_data.data.posture.turn_cycle.data[1] = getTurnByJoint(j2);
		req_data.data.posture.turn_cycle.data[2] = getTurnByJoint(j3);
		req_data.data.posture.turn_cycle.data[3] = getTurnByJoint(j4);
		req_data.data.posture.turn_cycle.data[4] = getTurnByJoint(j5);
		req_data.data.posture.turn_cycle.data[5] = getTurnByJoint(j6);
		req_data.data.posture.turn_cycle.data[6] = getTurnByJoint(j7);
		req_data.data.posture.turn_cycle.data[7] = getTurnByJoint(j8);
		req_data.data.posture.turn_cycle.data[8] = getTurnByJoint(j9);
	}
	else
	{
		req_data.data.posture.turn_cycle.data[0] = 0;
		req_data.data.posture.turn_cycle.data[1] = 0;
		req_data.data.posture.turn_cycle.data[2] = 0;
		req_data.data.posture.turn_cycle.data[3] = 0;
		req_data.data.posture.turn_cycle.data[4] = 0;
		req_data.data.posture.turn_cycle.data[5] = 0;
		req_data.data.posture.turn_cycle.data[6] = 0;
		req_data.data.posture.turn_cycle.data[7] = 0;
		req_data.data.posture.turn_cycle.data[8] = 0;
	}

	if (!rpc_ptr->handleRpc(0x000154E7, &req_data, RequestMessageType_PrRegData_fields, &rep_data, ResponseMessageType_Uint64_fields))
	{
		return HANDLE_RPC_FAILED;
	}
	return rep_data.data.data;
}

uint64_t c_setPrReg(int32_t id, char* name, char* comment, int32_t pos_type,
	double j1, double j2, double j3, double j4, double j5, double j6, double j7, double j8, double j9,
	int32_t p_left_right, int32_t p_back_front, int32_t p_up_down, int32_t p_wrist_flip,
	int32_t group_id)
{
	RpcBasic* rpc_ptr = RpcBasic::getInstance();
	RequestMessageType_PrRegData req_data;
	ResponseMessageType_Uint64 rep_data;

	req_data.property.authority = Comm_Authority_TP_SIMMULATOR;
	req_data.data.id = id;
	memcpy(req_data.data.name, name, strlen(name));
	req_data.data.name[strlen(name)] = 0;
	memcpy(req_data.data.comment, comment, strlen(comment));
	req_data.data.comment[strlen(comment)] = 0;
	req_data.data.pos_type = pos_type;
	req_data.data.pos.data_count = 9;
	req_data.data.pos.data[0] = j1;
	req_data.data.pos.data[1] = j2;
	req_data.data.pos.data[2] = j3;
	req_data.data.pos.data[3] = j4;
	req_data.data.pos.data[4] = j5;
	req_data.data.pos.data[5] = j6;
	req_data.data.pos.data[6] = j7;
	req_data.data.pos.data[7] = j8;
	req_data.data.pos.data[8] = j9;
	req_data.data.group_id = group_id;
	req_data.data.posture.arm_left_right = p_left_right;
	req_data.data.posture.arm_back_front = p_back_front;
	req_data.data.posture.arm_up_down = p_up_down;
	req_data.data.posture.wrist_flip = p_wrist_flip;
	req_data.data.posture.turn_cycle.data_count = 9;
	if (pos_type == 0)
	{
		req_data.data.posture.turn_cycle.data[0] = getTurnByJoint(j1);
		req_data.data.posture.turn_cycle.data[1] = getTurnByJoint(j2);
		req_data.data.posture.turn_cycle.data[2] = getTurnByJoint(j3);
		req_data.data.posture.turn_cycle.data[3] = getTurnByJoint(j4);
		req_data.data.posture.turn_cycle.data[4] = getTurnByJoint(j5);
		req_data.data.posture.turn_cycle.data[5] = getTurnByJoint(j6);
		req_data.data.posture.turn_cycle.data[6] = getTurnByJoint(j7);
		req_data.data.posture.turn_cycle.data[7] = getTurnByJoint(j8);
		req_data.data.posture.turn_cycle.data[8] = getTurnByJoint(j9);
	}
	else
	{
		req_data.data.posture.turn_cycle.data[0] = 0;
		req_data.data.posture.turn_cycle.data[1] = 0;
		req_data.data.posture.turn_cycle.data[2] = 0;
		req_data.data.posture.turn_cycle.data[3] = 0;
		req_data.data.posture.turn_cycle.data[4] = 0;
		req_data.data.posture.turn_cycle.data[5] = 0;
		req_data.data.posture.turn_cycle.data[6] = 0;
		req_data.data.posture.turn_cycle.data[7] = 0;
		req_data.data.posture.turn_cycle.data[8] = 0;
	}

	if (!rpc_ptr->handleRpc(0x00009EF7, &req_data, RequestMessageType_PrRegData_fields, &rep_data, ResponseMessageType_Uint64_fields))
	{
		return HANDLE_RPC_FAILED;
	}
	return rep_data.data.data;
}

uint64_t c_deletePrReg(int32_t id)
{
	RpcBasic* rpc_ptr = RpcBasic::getInstance();
	RequestMessageType_Int32 req_data;
	ResponseMessageType_Uint64 rep_data;

	req_data.property.authority = Comm_Authority_TP_SIMMULATOR;
	req_data.data.data = id;
	if (!rpc_ptr->handleRpc(0x00001097, &req_data, RequestMessageType_Int32_fields, &rep_data, ResponseMessageType_Uint64_fields))
	{
		return HANDLE_RPC_FAILED;
	}
	return rep_data.data.data;
}

uint64_t c_getPrReg(int32_t id, char* name, char* comment, int32_t* pos_type,
					double* j1, double* j2, double* j3, double* j4, double* j5, double* j6, double* j7, double* j8, double* j9,
					int32_t* p_left_right, int32_t* p_back_front, int32_t* p_up_down, int32_t* p_wrist_flip,
					int32_t* group_id)
{
	RpcBasic* rpc_ptr = RpcBasic::getInstance();
	RequestMessageType_Int32 req_data;
	ResponseMessageType_Uint64_PrRegData rep_data;

	req_data.property.authority = Comm_Authority_TP_SIMMULATOR;
	req_data.data.data = id;
	if (!rpc_ptr->handleRpc(0x00017207, &req_data, RequestMessageType_Int32_fields, &rep_data, ResponseMessageType_Uint64_PrRegData_fields))
	{
		return HANDLE_RPC_FAILED;
	}
	memcpy(name, rep_data.data.name, strlen(rep_data.data.name));
	name[strlen(rep_data.data.name)] = 0;
	memcpy(comment, rep_data.data.comment, strlen(rep_data.data.comment));
	comment[strlen(rep_data.data.comment)] = 0;
	*pos_type = rep_data.data.pos_type;
	*j1 = rep_data.data.pos.data[0];
	*j2 = rep_data.data.pos.data[1];
	*j3 = rep_data.data.pos.data[2];
	*j4 = rep_data.data.pos.data[3];
	*j5 = rep_data.data.pos.data[4];
	*j6 = rep_data.data.pos.data[5];
	*j7 = rep_data.data.pos.data[6];
	*j8 = rep_data.data.pos.data[7];
	*j9 = rep_data.data.pos.data[8];
	*p_left_right = rep_data.data.posture.arm_left_right;
	*p_back_front = rep_data.data.posture.arm_back_front;
	*p_up_down = rep_data.data.posture.arm_up_down;
	*p_wrist_flip = rep_data.data.posture.wrist_flip;
	*group_id = rep_data.data.group_id;

	return rep_data.error_code.data;
}



