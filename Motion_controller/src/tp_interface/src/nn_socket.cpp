/**
 * @file nn_socket_.cpp
 * @brief 
 * @author WangWei
 * @version 1.0.0
 * @date 2016-08-18
 */
#include <stdio.h>
#include "common.h"
#include "nn_socket.h"

NNSocket::NNSocket(string server_ip, NNSockType cmd_type,
			NNSockType state_type, int cmd_port, int state_port):
			server_ip_(server_ip), cmd_type_(cmd_type),
			tate_type_(state_type),cmd_port_(cmd_port),
			state_port_(state_port)
{
	nnSocketInit();
}

NNSocket::~NNSocket()
{
	nn_close(command_socket_);
	nn_close(state_socket_);
}
/**
 * @brief get request buffer pointer
 *
 * @return :ther pointer of the buffer
 */
uint8_t* NNSocket::getRequestBufPtr()
{
	return request_buffer_;
}
/**
 * @brief: get reply_buffer_
 *
 * @return: address of reply_buffer_ 
 */
uint8_t* NNSocket::getReplyBufPtr()
{
	return reply_buffer_;
}
/**
 * @brief: get publish_buffer_
 *
 * @return: address of publish_buffer_
 */
uint8_t* NNSocket::getPublishBufPtr()
{
	return publish_buffer_;
}

/**
 * @brief get request buffer length
 *
 * @return :the length of request buffer
 */
int NNSocket::getRequestBufLen()
{
	return request_bytes_;
}


/**
 * @brief: socket init
 *
 * @return: true if success 
 */
int NNSocket::nnSocketInit()
{
	char cmd_addr[256];
	char state_addr[256];

	string str_sock_type;
	command_socket_ = nn_socket(AF_SP, NN_REP);
	FST_ASSERT(command_socket_ >= 0);

	state_socket_ = nn_socket(AF_SP, NN_PUB);
	FST_ASSERT(state_socket_ >= 0);
	//FST_INFO("commandSocket:%d, statesocket:%d", command_socket_, state_socket_);

	if (cmd_type_ == NN_SOCK_TCP)
		str_sock_type = "tcp://";
	else if (cmd_type_ == NN_SOCK_WS)
		str_sock_type = "ws://";
	sprintf(cmd_addr, "%s%s:%d",\
		str_sock_type.c_str(), server_ip_.c_str(), cmd_port_);
	FST_INFO("command address:%s",cmd_addr);
	FST_ASSERT(nn_bind(command_socket_, cmd_addr) >= 0);

	if (tate_type_ == NN_SOCK_TCP)
		str_sock_type = "tcp://";
	else if (tate_type_ == NN_SOCK_WS)
		str_sock_type = "ws://";
	sprintf(state_addr, "%s%s:%d",\
		str_sock_type.c_str(), server_ip_.c_str(), state_port_);
	FST_INFO("state address:%s",state_addr);
	FST_ASSERT(nn_bind(state_socket_, state_addr) >= 0);

	usleep(100000); // wait for connections
	 
	pfd_[0].fd = command_socket_;
	pfd_ [0].events = NN_POLLIN | NN_POLLOUT;
	pfd_[1].fd = state_socket_;
	pfd_[1].events = NN_POLLOUT;
}

bool NNSocket::nnPoll(int sec)
{
	int rc = nn_poll (pfd_, 2, sec*1000);
	if(rc == -1)
	{
		return false;
	}
	return true;
}

/**
 * @brief :publish cotent of buffer
 *
 * @param buffer: input 
 * @param buffer_size: input
 *
 * @return: the actual bytes that send to TP 
 */
int NNSocket::nnSocketPublish(const uint8_t *buffer, int buffer_size)
{
	if (pfd_[1].revents & NN_POLLOUT)
	{
		int bytes = nn_send(state_socket_, (const char*)buffer, buffer_size, 0);

		return bytes;
	}
	//printf("can't publish buffer\n");
	return 0;
}

/**
 * @brief: receive TP request
 *
 * @return: true if success 
 */
bool NNSocket::nnSocketRecieve()
{
	if (pfd_[0].revents & NN_POLLIN)
	{
		request_bytes_ = nn_recv(command_socket_, request_buffer_, MAX_BUFFER_SIZE, 0);
		return true;
	}

	return false;
}



/**
 * @brief: after get the request ,reply cotent of buffer to TP
 *
 * @param: buffer:input
 * @param: buffer_size: input
 *
 * @return: the actual bytes that send to TP 
 */
int NNSocket::nnSocketReply(const uint8_t *buffer, int buffer_size)
{
	int bytes = nn_send(command_socket_, (const char*)buffer, buffer_size, 0);
	return bytes;
}

