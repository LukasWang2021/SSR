#include <thread>
#include "rpc_interface.h"
#include "rpc_basic.h"
#include "common_error_code.h"
#include "protoc.h"
#include "trans_matrix_list.pb.h"
#include "request_transmatrixlist.pb.h"



uint64_t c_mcSetGlobalVelRatio(int32_t vel_ratio)
{
	if (!rpc_valid) return HANDLE_RPC_FAILED;
	RpcBasic* rpc_ptr = RpcBasic::getInstance();
	RequestMessageType_Double req_data;
	ResponseMessageType_Uint64 rep_data;

	req_data.property.authority = Comm_Authority_TP_SIMMULATOR;
	req_data.data.data = ((double)vel_ratio) / 100.0;

	if (!rpc_ptr->handleRpc(0x000005EF, &req_data, RequestMessageType_Double_fields, &rep_data, ResponseMessageType_Uint64_fields))
	{
		return HANDLE_RPC_FAILED;
	}
	return rep_data.data.data;
}

uint64_t c_mcSetGlobalAccRatio(int32_t acc_ratio)
{
	if (!rpc_valid) return HANDLE_RPC_FAILED;
	RpcBasic* rpc_ptr = RpcBasic::getInstance();
	RequestMessageType_Double req_data;
	ResponseMessageType_Uint64 rep_data;

	req_data.property.authority = Comm_Authority_TP_SIMMULATOR;
	req_data.data.data = ((double)acc_ratio) / 100.0;

	if (!rpc_ptr->handleRpc(0x0000271F, &req_data, RequestMessageType_Double_fields, &rep_data, ResponseMessageType_Uint64_fields))
	{
		return HANDLE_RPC_FAILED;
	}
	return rep_data.data.data;
}

uint64_t c_mcDoStepManualMove(int32_t axis_id, int32_t direction)
{
	if (!rpc_valid) return HANDLE_RPC_FAILED;
	RpcBasic* rpc_ptr = RpcBasic::getInstance();
	RequestMessageType_Int32_Int32List req_data;
	ResponseMessageType_Uint64 rep_data;

	req_data.property.authority = Comm_Authority_TP_SIMMULATOR;
	req_data.data1.data = 0;
	req_data.data2.data_count = 9;
	memset(req_data.data2.data, 0, 9 * sizeof(int32_t));
	if (axis_id >= 9 || axis_id < 0) return HANDLE_RPC_FAILED;
	req_data.data2.data[axis_id] = direction;

	if (!rpc_ptr->handleRpc(0x000085D5, &req_data, RequestMessageType_Int32_Int32List_fields, &rep_data, ResponseMessageType_Uint64_fields))
	{
		return HANDLE_RPC_FAILED;
	}
	return rep_data.data.data;
}


std::thread* g_manual_move_thread_ptr = NULL;
RequestMessageType_Int32_Int32List g_manual_move_data;
ResponseMessageType_Uint64 g_manual_move_relpy;
bool is_g_manual_move_thread_alive = false;
void ManualMoveThread(void)
{
	RpcBasic* rpc_ptr = RpcBasic::getInstance();
	while (is_g_manual_move_thread_alive)
	{
#ifdef _WIN_PLAT
        Sleep(200);
#else
        usleep(200000);
#endif		
        rpc_ptr->handleRpc(0x0000D3F5, &g_manual_move_data, RequestMessageType_Int32_Int32List_fields, &g_manual_move_relpy, RequestMessageType_Uint64_fields);
	}

	RequestMessageType_Int32_Int32List req_data;
	req_data.property.authority = Comm_Authority_TP_SIMMULATOR;
	req_data.data1.data = 0;
	req_data.data2.data_count = 9;
	req_data.data2.data[0] = 0;
	req_data.data2.data[1] = 0;
	req_data.data2.data[2] = 0;
	req_data.data2.data[3] = 0;
	req_data.data2.data[4] = 0;
	req_data.data2.data[5] = 0;
	req_data.data2.data[6] = 0;
	req_data.data2.data[7] = 0;
	req_data.data2.data[8] = 0;
	rpc_ptr->handleRpc(0x0000D3F5, &req_data, RequestMessageType_Int32_Int32List_fields, &g_manual_move_relpy, RequestMessageType_Uint64_fields);
	
}

uint64_t c_mcDoContinuousManualMove(int32_t axis_id, int32_t direction)
{
	if (!rpc_valid) return HANDLE_RPC_FAILED;
	RpcBasic* rpc_ptr = RpcBasic::getInstance();

	g_manual_move_data.property.authority = Comm_Authority_TP_SIMMULATOR;
	g_manual_move_data.data1.data = 0;
	g_manual_move_data.data2.data_count = 9;
	memset(g_manual_move_data.data2.data, 0, 9 * sizeof(int32_t));
	g_manual_move_data.data2.data[axis_id] = direction;

	if (!rpc_ptr->handleRpc(0x0000D3F5, &g_manual_move_data, RequestMessageType_Int32_Int32List_fields, &g_manual_move_relpy, RequestMessageType_Uint64_fields))
	{
		return HANDLE_RPC_FAILED;
	}
	if(g_manual_move_relpy.data.data != 0)
		return g_manual_move_relpy.data.data; 

	if (g_manual_move_thread_ptr != NULL)
	{
		is_g_manual_move_thread_alive = false;
		g_manual_move_thread_ptr->join();
		delete g_manual_move_thread_ptr;
		g_manual_move_thread_ptr = NULL;
	}
	is_g_manual_move_thread_alive = true;
	g_manual_move_thread_ptr = new std::thread(&ManualMoveThread);
	return SUCCESS;
}

void c_mcDoContinuousManualToStandstill()
{
	is_g_manual_move_thread_alive = false;
	if (g_manual_move_thread_ptr != NULL)
	{
		g_manual_move_thread_ptr->join();
		delete g_manual_move_thread_ptr;
		g_manual_move_thread_ptr = NULL;
	}
}

uint64_t c_mcDoGotoCartesianMove(int32_t group_index, double x, double y, double z, double a, double b, double c,
	int32_t arm_front_back, int32_t elbow_up_down, int32_t wrist_down_up, int32_t uf_id, int32_t tf_id)
{
	if (!rpc_valid) return HANDLE_RPC_FAILED;
	RpcBasic* rpc_ptr = RpcBasic::getInstance();
	RequestMessageType_Int32_UFTF_PoseAndPosture req_data;
	ResponseMessageType_Uint64 rep_data;

	req_data.property.authority = Comm_Authority_TP_SIMMULATOR;
	req_data.data1.data = group_index;
	req_data.data2.uf_id.data = uf_id;
	req_data.data2.tf_id.data = tf_id;
	req_data.data2.pose_and_posture.pose.data_count = 6;
	req_data.data2.pose_and_posture.pose.data[0] = x;
	req_data.data2.pose_and_posture.pose.data[1] = y;
	req_data.data2.pose_and_posture.pose.data[2] = z;
	req_data.data2.pose_and_posture.pose.data[3] = a;
	req_data.data2.pose_and_posture.pose.data[4] = b;
	req_data.data2.pose_and_posture.pose.data[5] = c;
	req_data.data2.pose_and_posture.posture.arm_back_front = arm_front_back; // 1: front arm, -1:back arm
	req_data.data2.pose_and_posture.posture.arm_up_down = elbow_up_down;     // 1: elbow above wrist, -1:elbow below wrist
	req_data.data2.pose_and_posture.posture.wrist_flip = wrist_down_up;      // 1: wrist down, -1: wrist up
	req_data.data2.pose_and_posture.posture.arm_left_right = 0;              // 0: not flip wrist, 1: flip wrist
	req_data.data2.pose_and_posture.posture.turn_cycle.data_count = 9;
	for (size_t i = 0; i < req_data.data2.pose_and_posture.posture.turn_cycle.data_count; ++i)
	{
		req_data.data2.pose_and_posture.posture.turn_cycle.data[i] = 0;
	}

	if (!rpc_ptr->handleRpc(0x00010C05, &req_data, RequestMessageType_Int32_UFTF_PoseAndPosture_fields, &rep_data, ResponseMessageType_Uint64_fields))
	{
		return HANDLE_RPC_FAILED;
	}
	return rep_data.data.data;
}

uint64_t c_mcDoGotoJointMove(int32_t group_index, double j1, double j2, double j3, double j4, double j5, double j6, double j7, double j8, double j9)
{
	if (!rpc_valid) return HANDLE_RPC_FAILED;
	RpcBasic* rpc_ptr = RpcBasic::getInstance();
	RequestMessageType_Int32_DoubleList req_data;
	ResponseMessageType_Uint64 rep_data;

	req_data.property.authority = Comm_Authority_TP_SIMMULATOR;
	req_data.data1.data = group_index;
	req_data.data2.data_count = 9;
	req_data.data2.data[0] = j1;
	req_data.data2.data[1] = j2;
	req_data.data2.data[2] = j3;
	req_data.data2.data[3] = j4;
	req_data.data2.data[4] = j5;
	req_data.data2.data[5] = j6;
	req_data.data2.data[6] = j7;
	req_data.data2.data[7] = j8;
	req_data.data2.data[8] = j9;

	if (!rpc_ptr->handleRpc(0x00008075, &req_data, RequestMessageType_Int32_DoubleList_fields, &rep_data, ResponseMessageType_Uint64_fields))
	{
		return HANDLE_RPC_FAILED;
	}
	return rep_data.data.data;
}

uint64_t c_mcSetCoordinate(char* coord_type)
{
	if (!rpc_valid) return HANDLE_RPC_FAILED;
	RpcBasic* rpc_ptr = RpcBasic::getInstance();
	RequestMessageType_Int32List req_data;
	ResponseMessageType_Uint64 rep_data;

	req_data.property.authority = Comm_Authority_TP_SIMMULATOR;
	req_data.data.data_count = 2;
	req_data.data.data[0] = 0;
	if (strcmp(coord_type, "Joint") == 0) req_data.data.data[1] = 0;
	else if (strcmp(coord_type, "Base") == 0) req_data.data.data[1] = 1;
	else if (strcmp(coord_type, "World") == 0) req_data.data.data[1] = 2;
	else if (strcmp(coord_type, "User") == 0) req_data.data.data[1] = 3;
	else if (strcmp(coord_type, "Tool") == 0) req_data.data.data[1] = 4;
	else req_data.data.data[1] = 5;

	if (!rpc_ptr->handleRpc(0x0000A845, &req_data, RequestMessageType_Int32List_fields, &rep_data, ResponseMessageType_Uint64_fields))
	{
		return HANDLE_RPC_FAILED;
	}
	return rep_data.data.data;
}

uint64_t c_mcSetUserCoordId(int32_t user_coord_id)
{
	if (!rpc_valid) return HANDLE_RPC_FAILED;
	RpcBasic* rpc_ptr = RpcBasic::getInstance();
	RequestMessageType_Int32List req_data;
	ResponseMessageType_Uint64 rep_data;

	req_data.property.authority = Comm_Authority_TP_SIMMULATOR;
	req_data.data.data_count = 2;
	req_data.data.data[0] = 0;
	req_data.data.data[1] = user_coord_id;

	if (!rpc_ptr->handleRpc(0x00005CF4, &req_data, RequestMessageType_Int32List_fields, &rep_data, ResponseMessageType_Uint64_fields))
	{
		return HANDLE_RPC_FAILED;
	}
	return rep_data.data.data;
}

uint64_t c_mcSetToolId(int32_t tool_id)
{
	if (!rpc_valid) return HANDLE_RPC_FAILED;
	RpcBasic* rpc_ptr = RpcBasic::getInstance();
	RequestMessageType_Int32List req_data;
	ResponseMessageType_Uint64 rep_data;

	req_data.property.authority = Comm_Authority_TP_SIMMULATOR;
	req_data.data.data_count = 2;
	req_data.data.data[0] = 0;
	req_data.data.data[1] = tool_id;

	if (!rpc_ptr->handleRpc(0x0001581C, &req_data, RequestMessageType_Int32List_fields, &rep_data, ResponseMessageType_Uint64_fields))
	{
		return HANDLE_RPC_FAILED;
	}
	return rep_data.data.data;
}

uint64_t c_mcIgnoreLostZeroError()
{
	if (!rpc_valid) return HANDLE_RPC_FAILED;
	RpcBasic* rpc_ptr = RpcBasic::getInstance();
	RequestMessageType_Int32 req_data;
	ResponseMessageType_Uint64 rep_data;

	req_data.property.authority = Comm_Authority_TP_SIMMULATOR;
	req_data.data.data = 0;//group_id

	if (!rpc_ptr->handleRpc(0x00014952, &req_data, RequestMessageType_Int32_fields, &rep_data, ResponseMessageType_Uint64_fields))
	{
		return HANDLE_RPC_FAILED;
	}
	return rep_data.data.data;
}

uint64_t c_mcSetAllZeroPointOffsets()
{
	if (!rpc_valid) return HANDLE_RPC_FAILED;
	RpcBasic* rpc_ptr = RpcBasic::getInstance();
	RequestMessageType_Int32_DoubleList req_data;
	ResponseMessageType_Uint64 rep_data;

	req_data.property.authority = Comm_Authority_TP_SIMMULATOR;
	req_data.data1.data = 0;//group_id
	req_data.data2.data_count = 9;
	for (size_t i = 0; i < req_data.data2.data_count; ++i)
	{
		req_data.data2.data[i] = 0;
	}

	if (!rpc_ptr->handleRpc(0x00011853, &req_data, RequestMessageType_Int32_DoubleList_fields, &rep_data, ResponseMessageType_Uint64_fields))
	{
		return HANDLE_RPC_FAILED;
	}
	return rep_data.data.data;
}


uint64_t c_mcSetStep(double joint_step, double cartesian_step, double orientation_step)
{
	if (!rpc_valid) return HANDLE_RPC_FAILED;
	RpcBasic* rpc_ptr = RpcBasic::getInstance();
	//set joint manual step.
	RequestMessageType_Int32_DoubleList req_data1;
	RequestMessageType_Int32_Double req_data2;
	ResponseMessageType_Uint64 rep_data;

	//set cartesian manual step
	req_data1.property.authority = Comm_Authority_TP_SIMMULATOR;
	req_data1.data1.data = 0;
	req_data1.data2.data_count = 9;
	for (size_t i = 0; i < req_data1.data2.data_count; ++i)
	{
		req_data1.data2.data[i] = joint_step;
	}
	if (!rpc_ptr->handleRpc(0x00018470, &req_data1, RequestMessageType_Int32_DoubleList_fields, &rep_data, ResponseMessageType_Uint64_fields))
	{
		return HANDLE_RPC_FAILED;
	}
	if (rep_data.data.data != 0) return rep_data.data.data;
#ifdef _WIN_PLAT
        Sleep(100);
#else
        usleep(100000);
#endif
	//set orientation manual step
	req_data2.property.authority = Comm_Authority_TP_SIMMULATOR;
	req_data2.data1.data = 0;
	req_data2.data2.data = cartesian_step;
	if (!rpc_ptr->handleRpc(0x0000A420, &req_data2, RequestMessageType_Int32_Double_fields, &rep_data, ResponseMessageType_Uint64_fields))
	{
		return HANDLE_RPC_FAILED;
	}
	if (rep_data.data.data != 0) return rep_data.data.data;
#ifdef _WIN_PLAT
        Sleep(100);
#else
        usleep(100000);
#endif
	req_data2.data1.data = 0;
	req_data2.data2.data = orientation_step;
	if (!rpc_ptr->handleRpc(0x00002940, &req_data2, RequestMessageType_Int32_Double_fields, &rep_data, ResponseMessageType_Uint64_fields))
	{
		return HANDLE_RPC_FAILED;
	}

	return rep_data.data.data;
}

uint64_t c_mcGetPostureByJoint(int32_t group_index, double j1, double j2, double j3, double j4, double j5, double j6, double j7, double j8, double j9,
							int32_t* arm_front_back, int32_t* elbow_up_down, int32_t* wrist_down_up)
{
	if (!rpc_valid) return HANDLE_RPC_FAILED;
	RpcBasic* rpc_ptr = RpcBasic::getInstance();
	RequestMessageType_Int32_DoubleList req_data;
	ResponseMessageType_Uint64_Posture rep_data;

	req_data.property.authority = Comm_Authority_TP_SIMMULATOR;
	req_data.data1.data = group_index;
	req_data.data2.data_count = 9;
	req_data.data2.data[0] = j1;
	req_data.data2.data[1] = j2;
	req_data.data2.data[2] = j3;
	req_data.data2.data[3] = j4;
	req_data.data2.data[4] = j5;
	req_data.data2.data[5] = j6;
	req_data.data2.data[6] = j7;
	req_data.data2.data[7] = j8;
	req_data.data2.data[8] = j9;

	if (!rpc_ptr->handleRpc(0x0000EC64, &req_data, RequestMessageType_Int32_DoubleList_fields, &rep_data, ResponseMessageType_Uint64_Int32List_fields))
	{
		return HANDLE_RPC_FAILED;
	}

	*arm_front_back = rep_data.data.arm_back_front;
	*elbow_up_down = rep_data.data.arm_up_down;
	*wrist_down_up = rep_data.data.wrist_flip;

	return rep_data.error_code.data;
}

uint64_t c_OfflineTrajectory_eulerFileConvert2JointFile(char* file_name_ptr)
{
	if (!rpc_valid)
		return HANDLE_RPC_FAILED;
	RpcBasic* rpc_ptr = RpcBasic::getInstance();
	RequestMessageType_String req_data;
	ResponseMessageType_Uint64 rep_data;

	req_data.header.time_stamp = 122;
	req_data.property.authority = Comm_Authority_TP_SIMMULATOR;
	size_t file_name_size = strlen(file_name_ptr);
	memcpy(req_data.data.data, file_name_ptr, file_name_size);
	req_data.data.data[file_name_size] = 0;//在文件名字符串最后一字节设置结束符
	if (!rpc_ptr->handleRpc(0x0000E375, &req_data, RequestMessageType_String_fields, &rep_data, ResponseMessageType_Uint64_fields))
	{
		return HANDLE_RPC_FAILED;
	}
	return rep_data.data.data;
}

uint64_t c_OfflineTrajectoryFileSet(char* file_name_ptr)
{
	if (!rpc_valid)
		return HANDLE_RPC_FAILED;
	RpcBasic* rpc_ptr = RpcBasic::getInstance();
	RequestMessageType_String req_data;
	ResponseMessageType_Uint64 rep_data;

	req_data.header.time_stamp = 122;
	req_data.property.authority = Comm_Authority_TP_SIMMULATOR;
	size_t file_name_size = strlen(file_name_ptr);
	memcpy(req_data.data.data, file_name_ptr, file_name_size);
	req_data.data.data[file_name_size] = 0;
	if (!rpc_ptr->handleRpc(0x00011275, &req_data, RequestMessageType_String_fields, &rep_data, ResponseMessageType_Uint64_fields))
	{
		return HANDLE_RPC_FAILED;
	}
	return rep_data.data.data;
}

uint64_t c_OfflineTrajectoryPrepare(void)
{
	if (!rpc_valid)
		return HANDLE_RPC_FAILED;
	RpcBasic* rpc_ptr = RpcBasic::getInstance();
	RequestMessageType_Int32 req_data;
	ResponseMessageType_Uint64 rep_data;
	req_data.header.time_stamp = 122;
	req_data.property.authority = Comm_Authority_TP_SIMMULATOR;
	if (!rpc_ptr->handleRpc(0x000051E9, &req_data, RequestMessageType_Void_fields, &rep_data, ResponseMessageType_Uint64_fields))
	{
		return HANDLE_RPC_FAILED;
	}
	return rep_data.data.data;
}
uint64_t c_OfflineTrajectoryMove(void)
{
	if (!rpc_valid)
		return HANDLE_RPC_FAILED;
	RpcBasic* rpc_ptr = RpcBasic::getInstance();
	RequestMessageType_Int32 req_data;
	ResponseMessageType_Uint64 rep_data;
	req_data.header.time_stamp = 122;
	req_data.property.authority = Comm_Authority_TP_SIMMULATOR;
	if (!rpc_ptr->handleRpc(0x0000C4D9, &req_data, RequestMessageType_Void_fields, &rep_data, ResponseMessageType_Uint64_fields))
	{
		return HANDLE_RPC_FAILED;
	}
	return rep_data.data.data;
}

uint64_t c_sendOnlineTrajectory(double *m_list, int *m_stat, int matrix_cnt)
{
	if (!rpc_valid)
		return HANDLE_RPC_FAILED;
	RpcBasic* rpc_ptr = RpcBasic::getInstance();
	RequestMessageType_TransMatrixList req_data;
	ResponseMessageType_Uint64 rep_data;
	//req_data = RequestMessageType_TransMatrixList_init_zero;
	req_data.header.time_stamp = 122;
	req_data.property.authority = Comm_Authority_TP_SIMMULATOR;
	//printf("c_sendOnlineTrajectory matrix_cnt=%d\n", matrix_cnt);
	if (matrix_cnt > 32)
	{
		return TP_COMM_RPC_TOO_MUCH_DATA;
	}
	req_data.data.matrices_count = matrix_cnt;
	for (int i = 0; i < matrix_cnt; i++)
	{
		req_data.data.matrices[i].state = m_stat[i];
		req_data.data.matrices[i].matrix_count = 16;//此处必须是16,不然无法保证数据传输的完整性
		/*printf("\nmatrix[%d]=%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf", i,
			m_list[16 * i + 0], m_list[16 * i + 1], m_list[16 * i + 2], m_list[16 * i + 3],
			m_list[16 * i + 4], m_list[16 * i + 5], m_list[16 * i + 6], m_list[16 * i + 7],
			m_list[16 * i + 8], m_list[16 * i + 9], m_list[16 * i + 10], m_list[16 * i + 11],
			m_list[16 * i + 12], m_list[16 * i + 13], m_list[16 * i + 14], m_list[16 * i + 15]);*/
		memcpy(&req_data.data.matrices[i].matrix[0], &m_list[16*i], 16*sizeof(double));
		/*printf("req_data.data.matrices[%d]=%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf", i,
			req_data.data.matrices[i].matrix[0],req_data.data.matrices[i].matrix[1],req_data.data.matrices[i].matrix[2],req_data.data.matrices[i].matrix[3],
			req_data.data.matrices[i].matrix[4], req_data.data.matrices[i].matrix[5],req_data.data.matrices[i].matrix[6],req_data.data.matrices[i].matrix[7],
			req_data.data.matrices[i].matrix[8],req_data.data.matrices[i].matrix[9],req_data.data.matrices[i].matrix[10],req_data.data.matrices[i].matrix[11],
			req_data.data.matrices[i].matrix[12],req_data.data.matrices[i].matrix[13],req_data.data.matrices[i].matrix[14],req_data.data.matrices[i].matrix[15]);*/
	}
	if (!rpc_ptr->handleRpc(0x00008A31, &req_data, RequestMessageType_TransMatrixList_fields, &rep_data, ResponseMessageType_Uint64_fields))
	{
		return HANDLE_RPC_FAILED;
	}
	return rep_data.data.data;
}

uint64_t c_OfflineTrajectoryPlan(char* traj_name, double traj_vel, double via_points[][6], int32_t number_of_vp)
{
	if (!rpc_valid) return HANDLE_RPC_FAILED;
	if (traj_name == NULL || via_points == NULL) return RPC_PARAM_INVALID;

	RpcBasic* rpc_ptr = RpcBasic::getInstance();
	RequestMessageType_Int32_DoubleList send_vp_req_data;
	ResponseMessageType_Uint64 send_vp_rep_data;

	send_vp_req_data.header.time_stamp = 122;
	send_vp_req_data.property.authority = Comm_Authority_TP_SIMMULATOR;
	send_vp_req_data.data1.data = 1; // first time is 1 means this is a new trajectory

	double* p_vp = &via_points[0][0];
	int32_t vps_to_copy = number_of_vp;
	size_t copy_cnts = (size_t)ceil((number_of_vp * 6.0 * sizeof(double)) / sizeof(send_vp_req_data.data2.data)); // these vps copy coast how many times
	int32_t copy_grps = sizeof(send_vp_req_data.data2.data) / (6 * sizeof(double)); //copy groups: buffer can copy how many vps at one time
	do
	{
		vps_to_copy = vps_to_copy > copy_grps ? copy_grps : vps_to_copy;
		send_vp_req_data.data2.data_count = vps_to_copy * 6;
		memcpy(send_vp_req_data.data2.data, p_vp, send_vp_req_data.data2.data_count * sizeof(double));
		if (!rpc_ptr->handleRpc(0x0000A063, &send_vp_req_data, RequestMessageType_Int32_DoubleList_fields, &send_vp_rep_data, ResponseMessageType_Uint64_fields))
		{
			return CORE_COMM_LOAD_PARAM_FAILED;
		}
		p_vp += vps_to_copy;
		send_vp_req_data.data1.data = 0; // first time is 1 means this is a new trajectory
		vps_to_copy = number_of_vp - vps_to_copy;
	}while (--copy_cnts);

	RequestMessageType_String_Double traj_plan_req_data;
	ResponseMessageType_Uint64_String traj_plan_rep_data;
	traj_plan_req_data.header.time_stamp = 122;
	traj_plan_req_data.property.authority = Comm_Authority_TP_SIMMULATOR;
	strcpy(traj_plan_req_data.data1.data, traj_name);
	traj_plan_req_data.data2.data = traj_vel;

	if (!rpc_ptr->handleRpc(0x0000E479, &traj_plan_req_data, RequestMessageType_String_Double_fields, &traj_plan_rep_data, ResponseMessageType_Uint64_String_fields))
	{
		return HANDLE_RPC_FAILED;
	}

	return traj_plan_rep_data.error_code.data;
}

uint64_t c_mcGetDH(int32_t group_id, double  dh[7][4])
{
	if (!rpc_valid)
		return HANDLE_RPC_FAILED;
	RpcBasic* rpc_ptr = RpcBasic::getInstance();
	RequestMessageType_Int32 req_data;
	ResponseMessageType_Uint64_DoubleList rep_data;

	req_data.header.time_stamp = 122;
	req_data.property.authority = Comm_Authority_TP_SIMMULATOR;
	req_data.data.data = group_id;
	if (!rpc_ptr->handleRpc(0x00016078, &req_data, RequestMessageType_Int32_fields, &rep_data, ResponseMessageType_Uint64_DoubleList_fields))
	{
		return HANDLE_RPC_FAILED;
	}
	memcpy(dh, rep_data.data.data, rep_data.data.data_count * sizeof(double));
	return rep_data.error_code.data;
}



