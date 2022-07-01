//#include "stdafx.h"
#include "rpc_basic.h"
#include "request_base.pb.h"
#include "response_base.pb.h"
#include <iostream>
#include "request_transmatrixlist.pb.h"
using namespace std;

RpcBasic* RpcBasic::instance_ = NULL;

RpcBasic::RpcBasic():
	socket_(-1)
{
	memset(send_buffer_, 0, sizeof(send_buffer_));
	memset(recv_buffer_, 0, sizeof(send_buffer_));
}

RpcBasic::~RpcBasic()
{
	nn_close(socket_);
}

RpcBasic* RpcBasic::getInstance()
{
	if (instance_ == NULL)
	{
		instance_ = new RpcBasic();
	}
	return instance_;
}

int32_t RpcBasic::init(std::string server_ip)
{
	server_path_ = "ws://" + server_ip + ":5600";

	if ((socket_ = nn_socket(AF_SP, NN_REQ)) < 0) return -1;
	int32_t recv_timeout = 10000, send_timeout = 2000;
    nn_setsockopt(socket_, NN_SOL_SOCKET, NN_RCVTIMEO, &recv_timeout, sizeof(int32_t));
	nn_setsockopt(socket_, NN_SOL_SOCKET, NN_SNDTIMEO, &send_timeout, sizeof(int32_t));
	if (nn_connect(socket_, server_path_.c_str()) < 0) return -1;
	return 0;
}

bool RpcBasic::handleRpc(unsigned int hash, void* req_data_ptr, const pb_field_t req_fields[], void* rep_data_ptr, const pb_field_t rep_fields[])
{
	ResponseMessageType_Uint64* head_data_ptr = static_cast<ResponseMessageType_Uint64*>(rep_data_ptr);
	//initRequestPackage(req_data_ptr);
	mutex_.lock();
	int send_size, recv_size;
	if (!encodeRequestPackage(hash, req_fields, req_data_ptr, send_size)
		|| !send(send_size)
		|| !recv(recv_size)
		|| !decodeReplyPackage(rep_fields, rep_data_ptr, recv_size)
		|| head_data_ptr->header.error_code != 0)
	{
		mutex_.unlock();
		return false;
	}
	else
	{
		mutex_.unlock();
		return true;
	}
}

bool RpcBasic::handleRpc32(unsigned int hash, void* req_data_ptr, const pb_field_t req_fields[], void* rep_data_ptr, const pb_field_t rep_fields[])
{
	ResponseMessageType_Int32* head_data_ptr = static_cast<ResponseMessageType_Int32*>(rep_data_ptr);
	//initRequestPackage(req_data_ptr);

	int send_size, recv_size;
	if (!encodeRequestPackage(hash, req_fields, req_data_ptr, send_size)
		|| !send(send_size)
		|| !recv(recv_size)
		|| !decodeReplyPackage(rep_fields, rep_data_ptr, recv_size)
		|| head_data_ptr->header.error_code != 0)
	{
		return false;
	}
	else
	{
		return true;
	}
}

bool RpcBasic::handleRpcWithoutReply(unsigned int hash, void* req_data_ptr, const pb_field_t req_fields[])
{
	initRequestPackage(req_data_ptr);

	int send_size;
	if (!encodeRequestPackage(hash, req_fields, req_data_ptr, send_size)
		|| !send(send_size))
	{
		return false;
	}
	else
	{
		return true;
	}
}

bool RpcBasic::send(int expect_size)
{
	int send_size = nn_send(socket_, &send_buffer_, expect_size, 0);
	if (send_size < 0
		|| send_size != expect_size)
	{
		return false;
	}
	else
	{
		return true;
	}
}

bool RpcBasic::recv(int& recv_size)
{
	recv_size = nn_recv(socket_, &recv_buffer_, COMM_BUFFER_SIZE, 0);
	if (recv_size < 0)
	{
		return false;
	}
	else
	{
		return true;
	}
}

bool RpcBasic::encodeRequestPackage(unsigned int hash, const pb_field_t fields[], void* request_data_ptr, int& send_buffer_size)
{
	send_buffer_size = 0;
	*((unsigned int*)(&send_buffer_[0])) = hash;
	pb_ostream_t stream = pb_ostream_from_buffer(&send_buffer_[0] + HASH_BYTE_SIZE, COMM_BUFFER_SIZE - HASH_BYTE_SIZE);

	if (!pb_encode(&stream, fields, request_data_ptr))
	{
		send_buffer_size = 0;
		return false;
	}

	send_buffer_size = (int)(stream.bytes_written + HASH_BYTE_SIZE);
	return true;
}

bool RpcBasic::decodeReplyPackage(const pb_field_t fields[], void* reply_data_ptr, int recv_bytes)
{
	pb_istream_t stream = pb_istream_from_buffer(&recv_buffer_[0] + HASH_BYTE_SIZE, recv_bytes - HASH_BYTE_SIZE);
	return pb_decode(&stream, fields, (void*)reply_data_ptr);
}

void RpcBasic::initRequestPackage(void* request_package_ptr)
{
	RequestMessageType_Void* package_ptr = static_cast<RequestMessageType_Void*>(request_package_ptr);
	//package_ptr->header.time_stamp = 0;
	package_ptr->property.authority = Comm_Authority_TP_SIMMULATOR;
}
