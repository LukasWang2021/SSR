/**
 * @file nn_socket.cpp
 * @brief 
 * @author WangWei
 * @version 1.0.0
 * @date 2016-08-18
 */
#include <stdio.h>
#include "nn_socket.h"

int NN_Socket::NN_Socket_Init()
{
	char cmd_addr[256];
	char state_addr[256];

	string str_sock_type;
	m_command_socket = nn_socket(AF_SP, NN_REP);
	if(m_command_socket < 0)
		return -1;
	m_state_socket = nn_socket(AF_SP, NN_PUB);
	if(m_state_socket < 0)
		return -1;
	printf("commandSocket:%d, statesocket:%d\n", m_command_socket, m_state_socket);

	if(m_cmd_type == NN_SOCK_TCP)
		str_sock_type = "tcp://";
	else if(m_cmd_type == NN_SOCK_WS)
		str_sock_type = "ws://";
	sprintf(cmd_addr, "%s%s:%d",\
		str_sock_type.c_str(), m_server_ip.c_str(), m_cmd_port);
	printf("command address:%s\n",cmd_addr);
	if(nn_bind(m_command_socket, cmd_addr) < 0)
		return -1;

	if(m_state_type == NN_SOCK_TCP)
		str_sock_type = "tcp://";
	else if(m_state_type == NN_SOCK_WS)
		str_sock_type = "ws://";
	sprintf(state_addr, "%s%s:%d",\
		str_sock_type.c_str(), m_server_ip.c_str(), m_state_port);
	printf("state address:%s\n",state_addr);
	if(nn_bind(m_state_socket, state_addr) < 0)
		return -1;
}


int NN_Socket::NN_Socket_Publish(uint8_t *buffer, int buffer_size)
{
	int result = nn_send(m_state_socket, (const char*)buffer, buffer_size, 0);
	return result;
}
int NN_Socket::NN_Socket_Recieve()
{
	recv_req_data_size = nn_recv(m_command_socket, req_buffer, sizeof(req_buffer), 0);

	printf("reced:%d\n",recv_req_data_size);
}

int NN_Socket::NN_socket_Reply(uint8_t *buffer, int buffer_size)
{
	int result = nn_send(m_command_socket, (const char*)buffer, buffer_size, 0);
	return result;
}

