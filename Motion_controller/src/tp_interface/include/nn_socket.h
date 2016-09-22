/**
 * @file nn_socket.cpp
 * @brief 
 * @author WangWei
 * @version 1.0.0
 * @date 2016-08-18
 */
#include <string>
#include <assert.h>
#include <nanomsg/nn.h>
#include <nanomsg/pubsub.h>
#include <nanomsg/reqrep.h>

using namespace std;
#define MAX_REQ_BUFFER_SIZE		(1024)

#define COMMAND_PORT		(5556)
#define STATE_PORT			(5555)

typedef enum _NN_Sock_Type
{
	NN_SOCK_TCP	= 0,
	NN_SOCK_WS	= 1
}NN_Sock_Type;


class NN_Socket
{
public:
	string m_server_ip;

	NN_Sock_Type m_cmd_type;
	NN_Sock_Type m_state_type;
	
	int m_cmd_port;
	int m_state_port;

	int m_command_socket;
	int m_state_socket;

	uint8_t req_buffer[MAX_REQ_BUFFER_SIZE];
	int recv_req_data_size;

	NN_Socket(string server_ip, NN_Sock_Type cmd_type,
			NN_Sock_Type state_type, int cmd_port, int state_port):
			m_server_ip(server_ip), m_cmd_type(cmd_type),
			m_state_type(state_type),m_cmd_port(cmd_port),
			m_state_port(state_port)
	{
		
	}

	int NN_Socket_Init();
	int NN_Socket_Publish(uint8_t *buffer, int buffer_size);	
	int NN_Socket_Recieve();
	int NN_socket_Reply(uint8_t *buffer, int buffer_size);
};
