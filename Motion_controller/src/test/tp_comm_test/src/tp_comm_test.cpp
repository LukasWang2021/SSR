#include <sys/ioctl.h>
#include <netinet/in.h>
#include <net/if.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <nanomsg/nn.h>
#include <nanomsg/reqrep.h>
#include <nanomsg/pubsub.h>
#include <nanomsg/pipeline.h>
#include <nanomsg/ws.h>
#include <iostream>
#include <pb_encode.h>
#include <pb_decode.h>
#include <pb_common.h>

#include "tp_comm_test.h"

using namespace std;

TpCommTest ::TpCommTest()
{
    rpc_socket_ = -1;
    sub_socket_ = -1;
	event_socket_ = -1;
}

TpCommTest ::~TpCommTest()
{
    if (0 <= rpc_socket_)
    {
        nn_shutdown(rpc_socket_, 0);
        rpc_socket_ = -1;
    }
    if (0 <= sub_socket_)
    {
        nn_shutdown(sub_socket_, 0);
        sub_socket_ = -1;
    }
	if (0 <= event_socket_)
    {
        nn_shutdown(event_socket_, 0);
        sub_socket_ = -1;
    }
}

string TpCommTest::getLocalIP()
{
    int fd = socket(AF_INET, SOCK_DGRAM, 0);

    struct ifreq ifr;
    ifr.ifr_addr.sa_family = AF_INET;

    char iface[] = "eth0";
    strncpy(ifr.ifr_name, iface, IFNAMSIZ-1);

    ioctl(fd, SIOCGIFADDR, &ifr);

    close(fd);

    string ip = inet_ntoa(((struct sockaddr_in*) & ifr.ifr_addr)->sin_addr);
    return ip;
}

bool TpCommTest::initRpcSocket()
{
    if ((rpc_socket_ = nn_socket(AF_SP, NN_REQ)) < 0) return false;
    cout << "Request : socket is " << rpc_socket_ << endl;

    string ip = getLocalIP();
    string addr = "ws://"+ip+":5600";
    if (nn_connect (rpc_socket_, addr.c_str()) < 0) return false;
    cout << "Request : addr : " << addr << endl;

    return true;
}

bool TpCommTest::initPublishSocket()
{
    sub_socket_ = nn_socket(AF_SP, NN_SUB);
    if (sub_socket_ < 0) return false;
    cout << "Subscribe : socket is " << sub_socket_ << endl;

    string ip = getLocalIP();
    string addr = "ws://"+ip+":5601";
    if (nn_connect (sub_socket_, addr.c_str()) < 0) return false;
    cout << "Subscribe : addr : " << addr << endl;
    
    return true;
}

bool TpCommTest::initEventSocket()
{
    event_socket_ = nn_socket(AF_SP, NN_PULL);
    if (event_socket_ < 0) return false;
    cout << "Event : socket is " << event_socket_ << endl;

    string ip = getLocalIP();
    string addr = "ws://"+ip+":5602";
    if (nn_connect (event_socket_, addr.c_str()) < 0) return false;
    cout << "Event : addr : " << addr << endl;

	poll_event_fd_.fd = event_socket_;
	poll_event_fd_.events = NN_POLLIN;
    
    return true;
}


bool TpCommTest::sendRequestBuf(uint8_t* buf, int &buf_size)
{
    int bytes = -1;
    if ((bytes = nn_send(rpc_socket_, buf, buf_size, 0)) < 0) return false;

    return true;
}

bool TpCommTest::recvResponseBuf(uint8_t* buf, int &buf_size)
{
    if ((buf_size = nn_recv(rpc_socket_, buf, buf_size, 0)) < 0) return false;

    return true;
}

bool TpCommTest::checkHash(unsigned int &request_hash, unsigned int &response_hash)
{
    if (request_hash != response_hash) return false;

    return true;
}


int TpCommTest::recvPublishBuf(uint8_t* buf)
{
    return (nn_recv(sub_socket_, buf, SUBSCRIE_BUFFER_SIZE, 0));
}

bool TpCommTest::enablePublishTopic(unsigned int &hash)
{
    if (nn_setsockopt(sub_socket_, NN_SUB, NN_SUB_SUBSCRIBE, &hash, HASH_BYTE_SIZE) < 0) {
        cout << "Sub socket : setsockopt error" << endl;
        return false;
    }

    return true;
}


bool TpCommTest::recvEventBuf(uint8_t* buf, int &buf_size)
{
    if(nn_poll(&poll_event_fd_, 1, 0) <= 0)
		return false;

	if(poll_event_fd_.revents & NN_POLLIN)
    {
    	buf_size = nn_recv(event_socket_, buf, buf_size, 0);
		if (buf_size == -1)
		{
		    cout<<"Event socket: recv error"<<endl;
			return false;
		}
		
		return true;
    }
    return false;
}

bool TpCommTest::decodeEventMessageType(const pb_field_t fields[], uint8_t* buf, int &buf_size, void *data_ptr)
{
    pb_istream_t stream = {0};
    stream = pb_istream_from_buffer(buf, buf_size);
    bool decode_success = pb_decode(&stream, fields, data_ptr);
    if(!decode_success) return false;
    
    return true;
}



int TpCommTest::shutdownRpcSocket()
{
    return (nn_shutdown(rpc_socket_, 0));
}

int TpCommTest::shutdownSubscribeSocket()
{
    return (nn_shutdown(sub_socket_, 0));
}

bool TpCommTest::decodeMessageType(pb_byte_t* data_buf, pb_size_t &data_size, void *data, const pb_field_t fields[])
{
    pb_istream_t stream = pb_istream_from_buffer(data_buf, data_size);
    return (pb_decode(&stream, fields, data));
}

bool TpCommTest::decodeCommPublishBuf(uint8_t *buf, int &buf_size, unsigned int &hash, Comm_Publish &msg)
{
    memcpy(&hash, buf, HASH_BYTE_SIZE);
    pb_istream_t stream = {0};
    stream = pb_istream_from_buffer(buf + HASH_BYTE_SIZE, buf_size - HASH_BYTE_SIZE);

    bool decode_success = pb_decode(&stream, Comm_Publish_fields, &msg);
    if(!decode_success) return false;

    return true;
}


bool TpCommTest::generateRequestMessageType(unsigned int &hash,
    void *request_ptr, const pb_field_t fields[], uint8_t* buf, int &buf_size)
{
    memcpy(buf, &hash, HASH_BYTE_SIZE);
    buf_size = HASH_BYTE_SIZE;

    pb_ostream_t ostream = pb_ostream_from_buffer(buf + HASH_BYTE_SIZE, REQUEST_BUFFER_SIZE - HASH_BYTE_SIZE);
    
    bool encode_success = pb_encode(&ostream, fields, request_ptr);
    if(!encode_success) return false;

    buf_size += ostream.bytes_written;

    return true;
}

bool TpCommTest::decodeResponseMessageType(unsigned int &hash, void *response_ptr, 
const pb_field_t fields[], uint8_t* buf, int &buf_size)
{
    memcpy(&hash, buf, HASH_BYTE_SIZE);

    pb_istream_t stream = {0};
    stream = pb_istream_from_buffer(buf + HASH_BYTE_SIZE, buf_size - HASH_BYTE_SIZE);
    bool decode_success = pb_decode(&stream, fields, response_ptr);
    if(!decode_success) return false;
    
    return true;
}
