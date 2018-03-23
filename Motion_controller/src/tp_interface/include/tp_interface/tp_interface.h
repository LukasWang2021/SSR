/**
 * @file proto_parse.h
 * @brief: change some of protocol 
 * @author Wang Wei
 * @version 2.0.1
 * @date 2016-08-21
 */
#ifndef RCS_TP_INTERFACE_H_
#define RCS_TP_INTERFACE_H_

#include <mutex>
#include "task.h"
#include "common.h"
#include "nn_socket.h"
#include "base_types.pb.h"
#include "tpif_data.h"
#include "proto_parse.h"



typedef struct _PublishUpdate
{
    int count;          //time counter
    int base_interval;  //time interval to check if publish this parameter
    int max_interval;   //if time expired this interval this parameter must be published
    bool update_flag;   //publish flag 
    int buf_len;        //buffer length of parameter
}PublishUpdate;


typedef struct _CmdParams
{
    BaseTypes_CommandType   cmd;        //TP command
    PublishUpdate           pub_update; //publish parameter
}CmdParams;


class TPInterface
{
  public:
    TPInterface();
	~TPInterface();

  public:    
    void destroy();

    /**
     * @brief: get pointer of this instance 
     *
     * @return: pointer of this class 
     */
    static TPInterface* instance();

    /**
     * @brief: get pointer of request data 
     *
     * @return: pointer of request data 
     */
    TPIFReqData<>* getReqDataPtr();

    /**
     * @brief: get pointer of reply data
     *
     * @return: pointer of reply data 
     */
    TPIFRepData* getRepDataPtr();

    /**
     * @brief: get pointer of publish data 
     *
     * @return: pointer of publish data 
     */
    TPIPubData* getPubDataPtr();
    /**
     * @brief: send reply to TP 
     *
     * @param status_code: status code to send
     * @param id: id
     */
    void setReply(BaseTypes_StatusCode status_code, int id = 0);

    /**
     * @brief: send reply to TP 
     *
     * @param type: type to send
     * @param id: id
     */
    void setReply(TPRepType type, int id = 0);

    /**
     * @brief: get request id 
     *
     * @return: request id 
     */
	int getRequestID(); 
	
  private:
    NNSocket	*nn_socket_;    //pointer of NNSocket class 
    ProtoParse  *proto_parser_; //pointer of ProtoParse class
    rcs::Task   *task_;         //pointer of Task class
    TPIFReqData<>   request_;   //request data struct
    TPIFRepData     reply_;     //reply data struct
    TPIPubData      publish_;   //publish data struct
    std::mutex      mtx_;       
    

    /**
     * @brief: thread to request/reply and publish 
     *
     * @param params
     *
     * @return 
     */
    void* updateInterface(void *params);

    /**
     * @brief: parse TP command  
     *
     * @param buffer: buffer to parse
     * @param len: length of buffer
     */
    void parseTPCommand(const uint8_t* buffer, int len);

    /**
     * @brief: send reply to TP 
     */
    void sendReply();

    /**
     * @brief: send publish to TP 
     */
    void sendPublish();

    /**
     * @brief: compare two streams of integer 
     *
     * @param cmp_a: stream one
     * @param cmp_b: stream two
     *
     * @return: true if they are the same 
     */
    bool compareInt(const unsigned char *cmp_a, const uint8_t *cmp_b);
};

#endif //RCS_TP_INTERFACE_H_
