#include "rpc_interface.h"
#include "sub_interface.h"
#include "event_interface.h"
#include <stdio.h>
#include "common_error_code.h"
#include <iostream>
#include "request_transmatrixlist.pb.h"

void handle(void* request_data_ptr)
{
	RequestMessageType_TransMatrixList* rq_data_ptr = static_cast<RequestMessageType_TransMatrixList*>(request_data_ptr);
	int recv_matrixLen = rq_data_ptr->data.matrices_count;
	for (int i = 0; i < recv_matrixLen; i++)
	{
		printf("status=%d, \n[%lf,%lf,%lf,%lf]\n\n[%lf,%lf,%lf,%lf]\n\n[%lf,%lf,%lf,%lf]\n\n[%lf,%lf,%lf,%lf]\n",
			rq_data_ptr->data.matrices[i].state,
			rq_data_ptr->data.matrices[i].matrix[0],
			rq_data_ptr->data.matrices[i].matrix[1],
			rq_data_ptr->data.matrices[i].matrix[2],
			rq_data_ptr->data.matrices[i].matrix[3],
			rq_data_ptr->data.matrices[i].matrix[4],
			rq_data_ptr->data.matrices[i].matrix[5],
			rq_data_ptr->data.matrices[i].matrix[6],
			rq_data_ptr->data.matrices[i].matrix[7],
			rq_data_ptr->data.matrices[i].matrix[8],
			rq_data_ptr->data.matrices[i].matrix[9],
			rq_data_ptr->data.matrices[i].matrix[10],
			rq_data_ptr->data.matrices[i].matrix[11],
			rq_data_ptr->data.matrices[i].matrix[12],
			rq_data_ptr->data.matrices[i].matrix[13],
			rq_data_ptr->data.matrices[i].matrix[14],
			rq_data_ptr->data.matrices[i].matrix[15]
			);
	}
}

void set_RequestMessageType_TransMatrixList(double* m_list, int* m_stat, int matrix_cnt)
{
	RequestMessageType_TransMatrixList req_data;
	//req_data = RequestMessageType_TransMatrixList_init_zero;
	req_data.header.time_stamp = 122;
	req_data.property.authority = Comm_Authority_TP_SIMMULATOR;
	req_data.data.matrices_count = matrix_cnt;
	int i = 0, j = 0;
	for (i = 0; i < matrix_cnt; i++)
	{
		req_data.data.matrices[i].state = m_stat[i];
		req_data.data.matrices[i].matrix_count = i;
		memcpy(&req_data.data.matrices[i].matrix[0], &m_list[16 * i], 16 * sizeof(double));
	}
	handle(&req_data);
}
int main()
{
	RequestMessageType_TransMatrixList aaa;
	double m_data[32] = {1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16, 17,18,19,20,21,22,23,24,25,26,27,28,29,30,31,32};
	int m_state[2] = {1,2};
	set_RequestMessageType_TransMatrixList(m_data, m_state,2);
	return 0;
}