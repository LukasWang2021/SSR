#ifndef TP_COMM_TEST_H
#define TP_COMM_TEST_H


#include <thread>
#include <mutex>
#include <string>
#include <vector>
#include <sys/time.h>
#include <nanomsg/nn.h>
#include "protoc.h"

using namespace std;

class TpCommTest
{
public:
    TpCommTest();
    ~TpCommTest();

    bool initRpcSocket();
    bool initPublishSocket();

    bool sendRequestBuf(uint8_t* buf, int &buf_size);
    bool recvResponseBuf(uint8_t* buf, int &buf_size);
    bool checkHash(unsigned int &request_hash, unsigned int &response_hash);

    bool generateRequestMessageType(unsigned int &hash,
        void *request_ptr, const pb_field_t fields[], uint8_t* buf, int &buf_size);
    bool decodeResponseMessageType(unsigned int &hash, 
        void *response_ptr, const pb_field_t fields[], uint8_t* buf, int &buf_size);

    int recvPublishBuf(uint8_t* buf);
    bool decodeCommPublishBuf(uint8_t *buf, int &buf_size, unsigned int &hash, Comm_Publish &msg);
    bool decodeMessageType(pb_byte_t* data_buf, pb_size_t &data_size, void *data, const pb_field_t fields[]);
    bool enablePublishTopic(unsigned int &hash);

    int shutdownRpcSocket();
    int shutdownSubscribeSocket();

private:
    enum {HASH_BYTE_SIZE = 4, REQUEST_BUFFER_SIZE = 65535, RESPONSE_BUFFER_SIZE = 65535, SUBSCRIE_BUFFER_SIZE = 65535,};
    int rpc_socket_;
    int sub_socket_;

    string getLocalIP(); 
};

#endif


