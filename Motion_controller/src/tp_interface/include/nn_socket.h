/**
 * @file nn_socket_.cpp
 * @brief 
 * @author WangWei
 * @version 1.0.0
 * @date 2016-08-18 *
 */
#ifndef TP_INTERFACE_NN_SOCKET_H_
#define TP_INTERFACE_NN_SOCKET_H_


#include <string>
#include <assert.h>
#include <nanomsg/nn.h>
#include <nanomsg/pubsub.h>
#include <nanomsg/reqrep.h>

using std::string;

#define MAX_BUFFER_SIZE		(1024*100)

#define COMMAND_PORT		(5556)
#define STATE_PORT			(5555)

typedef enum _NNSockType
{
	NN_SOCK_TCP	= 0,
	NN_SOCK_WS	= 1
}NNSockType;


class NNSocket
{
  public:		
	NNSocket(string server_ip, NNSockType cmd_type,
			NNSockType state_type, int cmd_port, int state_port);
	~NNSocket();
	/**
	 * @brief: get request_buffer_
	 *
	 * @return: address of request_buffer_
	 */
	uint8_t *getRequestBufPtr();

	/**
	 * @brief: get reply_buffer_
	 *
	 * @return: address of reply_buffer_ 
	 */
	uint8_t *getReplyBufPtr();

	/**
	 * @brief: get publish_buffer_
	 *
	 * @return: address of publish_buffer_
	 */
	uint8_t *getPublishBufPtr();

	/**
	 * @brief: get the actual length of request_buffer_ 
	 *
	 * @return: length 
	 */
	int getRequestBufLen();

	/**
	 * @brief: socket init
	 *
	 */
	void nnSocketInit();
	/**
	 * @brief: publish cotent of buffer
	 *
	 * @param buffer: input 
	 * @param buffer_size: input
	 *
	 * @return: the actual bytes that send to TP 
	 */
	bool nnPoll(int sec);
	int nnSocketPublish(const uint8_t *buffer, int buffer_size);	
	/**
	 * @brief: receive TP request
	 *
	 * @return: true if success
	 */
	bool nnSocketRecieve();
	/**
	 * @brief: after get the request ,reply cotent of buffer to TP
	 *
	 * @param buffer: input
	 * @param buffer_size: input
	 *
	 * @return: the actual bytes that send to TP 
	 */
	int nnSocketReply(const uint8_t *buffer, int buffer_size);
	

  private:
	string server_ip_;		//server ip address

	NNSockType cmd_type_;	//command socket type "tcp" or "ws"
	NNSockType tate_type_;  //state socket type "tcp" or "ws"
	
	int cmd_port_;			//command socket port
	int state_port_;		//state socket port

	int command_socket_;
	int state_socket_;
	struct nn_pollfd pfd_[2];

	
	uint8_t request_buffer_[MAX_BUFFER_SIZE];
	uint8_t reply_buffer_[MAX_BUFFER_SIZE];
	uint8_t publish_buffer_[MAX_BUFFER_SIZE];
	int request_bytes_;		//sizeof bytes from request_buffer_
	
};

#endif
