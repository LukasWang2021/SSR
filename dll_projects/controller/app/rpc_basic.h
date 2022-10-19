#pragma once
#include "comm_def.h"
#include "nanomsg/nn.h"
#include "nanomsg/reqrep.h"
#include "nanomsg/ws.h"
#include "protocol/comm.pb.h"
#include "pb_decode.h"
#include "pb_encode.h"
#include <string>
#include <mutex>


class RpcBasic
{
public:
	RpcBasic();
	~RpcBasic();

	static RpcBasic* getInstance();
	int32_t init(std::string server_ip);
	bool handleRpc(unsigned int hash, void* req_data_ptr, const pb_field_t req_fields[], void* rep_data_ptr, const pb_field_t rep_fields[]);
	bool handleRpc32(unsigned int hash, void* req_data_ptr, const pb_field_t req_fields[], void* rep_data_ptr, const pb_field_t rep_fields[]);
	bool handleRpcWithoutReply(unsigned int hash, void* req_data_ptr, const pb_field_t req_fields[]);

private:
	static RpcBasic* instance_;
	std::string server_path_;
	int socket_;
	uint8_t send_buffer_[COMM_BUFFER_SIZE];
	uint8_t recv_buffer_[COMM_BUFFER_SIZE];
	std::mutex mutex_;

	bool send(int expect_size);
	bool recv(int& recv_size);
	bool encodeRequestPackage(unsigned int hash, const pb_field_t fields[], void* request_data_ptr, int& send_buffer_size);
	bool decodeReplyPackage(const pb_field_t fields[], void* reply_data_ptr, int recv_bytes);
	void initRequestPackage(void* request_package_ptr);

};
