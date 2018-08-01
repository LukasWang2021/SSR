#ifndef TP_COMM_COMPONENT_H
#define TP_COMM_COMPONENT_H


#include <thread>
#include <mutex>
#include <string>
#include <vector>
#include <sys/time.h>
#include <nanomsg/nn.h>
#include "protoc.h"
#include "parameter_manager/parameter_manager_param_group.h"


typedef struct
{
    unsigned int hash;
    void* data_ptr;
}TpPublishElement;

typedef struct
{
    unsigned int hash;
    std::vector<TpPublishElement> element_list_;
    Comm_Publish package;
    int interval_min;   // ms
    int interval_max;   // ms
    bool is_element_changed;
    struct timeval last_publish_time;
}TpPublish;

typedef struct
{
    unsigned int hash;
    void* request_data_ptr;
    void* response_data_ptr;
}TpRequestResponse;

class TpComm
{
public:
    TpComm();
    ~TpComm();

    bool init();

    bool open();
    void close();

    std::vector<TpRequestResponse> popTaskFromRequestList();
    int32_t getResponseSucceed(void* response_data_ptr);
    void pushTaskToResponseList(TpRequestResponse& package);
    TpPublish generateTpPublishTask(unsigned int topic_hash, int interval_min, int interval_max);
    void addTpPublishElement(TpPublish& task, unsigned int element_hash, void* element_data_ptr);
    void pushTaskToPublishList(TpPublish& package);

private:
    enum {HASH_BYTE_SIZE = 4,};
    enum {QUICK_SEARCH_TABLE_SIZE = 128,};

    typedef void (TpComm::*HandleRequestFuncPtr)(int);
    typedef void (TpComm::*HandleResponseFuncPtr)(std::vector<TpRequestResponse>::iterator&, int&);
    typedef void (TpComm::*HandlePublishElementFuncPtr)(Comm_Publish&, int, TpPublishElement&);

    void initRpcTable();
    void initRpcQuickSearchTable();
    void initPublishElementTable();
    void initPublishElementQuickSearchTable();
    bool initParamsFromYaml();
    HandleRequestFuncPtr getRequestHandlerByHash(unsigned int hash);
    HandleResponseFuncPtr getResponseHandlerByHash(unsigned int hash);
    HandlePublishElementFuncPtr getPublishElementHandlerByHash(unsigned int hash);
    Comm_Authority getPublishElementAuthorityByHash(unsigned int hash);
    Comm_Authority getRpcTableElementAuthorityByHash(unsigned int hash);
    void tpCommThreadFunc();
    void handleRequest();
    void pushTaskToRequestList(unsigned int hash, void* request_data_ptr, void* response_data_ptr);
    void handleResponseList();
    void handlePublishList();

    long computeTimeElapsed(struct timeval& current_time_val, struct timeval& last_time_val);
    long computeTimeForTp(struct timeval& current_time_val);
    bool checkPublishCondition(long time_elapsed, bool is_element_changed, int interval_min, int interval_max);

    bool decodeRequestPackage(const pb_field_t fields[], void* request_data_ptr, int recv_bytes);
    bool checkAuthority(Comm_Authority request_authority, Comm_Authority controller_authority);
    void initResponsePackage(void* request_data_ptr, void* response_data_ptr, int package_left);
    void initCommFailedResponsePackage(void* request_data_ptr, void* response_data_ptr);
    void handleRequestPackage(unsigned int hash, void* request_data_ptr, void* response_data_ptr, int recv_bytes, 
                                    const pb_field_t fields[], int package_left);
    ResponseMessageType_PublishTable getResponseSucceedPublishTable();

    bool encodeResponsePackage(unsigned int hash, const pb_field_t fields[], void* response_data_ptr, int& send_buffer_size);
    bool encodePublishElement(Comm_PublishElement_data_t& element, const pb_field_t fields[], void* element_data_ptr);
    bool encodePublishPackage(Comm_Publish& package, unsigned int hash, struct timeval& current_time_val, int& send_buffer_size);

    // request & response handler
    void handleRequest0xcf0be243(int recv_bytes); // "/tp_comm/test_request"
    void handleResponse0xcf0be243(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);

    /********"RequestMessageType_Void",**********/
    void handleRequest0x0000F933(int recv_bytes);
    /********"RequestMessageType_Void",**********/
    void handleRequest0x0000E9D3(int recv_bytes);
    /********"RequestMessageType_Void",**********/
    void handleRequest0x00000AB3(int recv_bytes);
    /********"RequestMessageType_Void",**********/
    void handleRequest0x00010945(int recv_bytes);
    /********"RequestMessageType_Int32",**********/
    void handleRequest0x000067A4(int recv_bytes);
    /********"RequestMessageType_Int32",**********/
    void handleRequest0x00010685(int recv_bytes);
    /********"RequestMessageType_Void",**********/
    void handleRequest0x00010AB3(int recv_bytes);
    /********"RequestMessageType_Void",**********/
    void handleRequest0x00017C25(int recv_bytes);
    /********"RequestMessageType_Void",**********/
    void handleRequest0x000093EE(int recv_bytes);
    /********"RequestMessageType_Void",**********/
    void handleRequest0x0000FC15(int recv_bytes);
    /********"RequestMessageType_Topic",**********/
    void handleRequest0x00013EA3(int recv_bytes);
    /********"RequestMessageType_Void",**********/
    void handleRequest0x00009BC5(int recv_bytes);

    /********"RequestMessageType_RegisterR",**********/	
    void handleRequest0x00007CF2(int recv_bytes);
    /********"RequestMessageType_RegisterR",**********/	
    void handleRequest0x000031B2(int recv_bytes);
    /********"RequestMessageType_UnsignedInt32",**********/	
    void handleRequest0x00013062(int recv_bytes);
    /********"RequestMessageType_UnsignedInt32",**********/	
    void handleRequest0x000116F2(int recv_bytes);
    /********"RequestMessageType_UnsignedInt32",**********/	
    void handleRequest0x000098C2(int recv_bytes);
    /********"RequestMessageType_Void",**********/
    	void handleRequest0x00005602(int recv_bytes);

    /********"RequestMessageType_RegisterPR",**********/	
    void handleRequest0x0000F862(int recv_bytes);
    /********"RequestMessageType_RegisterPR",**********/	
    void handleRequest0x0000C032(int recv_bytes);
    /********"RequestMessageType_UnsignedInt32",**********/	
    void handleRequest0x00005392(int recv_bytes);
    /********"RequestMessageType_UnsignedInt32",**********/	
    void handleRequest0x000170A2(int recv_bytes);
    /********"RequestMessageType_UnsignedInt32",**********/	
    void handleRequest0x000116B2(int recv_bytes);
    /********"RequestMessageType_Void",**********/	
    void handleRequest0x0000F402(int recv_bytes);

    /********"RequestMessageType_RegisterMR",**********/	
    void handleRequest0x0000F892(int recv_bytes);
    /********"RequestMessageType_RegisterMR",**********/
    void handleRequest0x0000C1C2(int recv_bytes);
    /********"RequestMessageType_UnsignedInt32",**********/	
    void handleRequest0x000053E2(int recv_bytes);
    /********"RequestMessageType_UnsignedInt32",**********/	
    void handleRequest0x000170D2(int recv_bytes);
    /********"RequestMessageType_UnsignedInt32",**********/	
    void handleRequest0x00011642(int recv_bytes);
    /********"RequestMessageType_Void",**********/	
    void handleRequest0x0000F3B2(int recv_bytes);

    /********"RequestMessageType_RegisterSR",**********/	
    void handleRequest0x0000F932(int recv_bytes);
    /********"RequestMessageType_RegisterSR",**********/	
    void handleRequest0x0000BF62(int recv_bytes);
    /********"RequestMessageType_UnsignedInt32",**********/	
    void handleRequest0x00005342(int recv_bytes);
    /********"RequestMessageType_UnsignedInt32",**********/	
    void handleRequest0x00017172(int recv_bytes);
    /********"RequestMessageType_UnsignedInt32",**********/	
    void handleRequest0x000115E2(int recv_bytes);
    /********"RequestMessageType_Void",**********/	
    void handleRequest0x0000F352(int recv_bytes);



    /********"ResponseMessageType_Int32",**********/
    void handleResponse0x0000F933(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);
    /********"ResponseMessageType_Int32",**********/
    void handleResponse0x0000E9D3(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);
    /********"ResponseMessageType_Int32",**********/
    void handleResponse0x00000AB3(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);
    /********"ResponseMessageType_Int32",**********/
    void handleResponse0x00010945(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);
    /********"ResponseMessageType_Bool",**********/
    void handleResponse0x000067A4(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);
    /********"ResponseMessageType_Bool",**********/
    void handleResponse0x00010685(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);
    /********"ResponseMessageType_String",**********/
    void handleResponse0x00010AB3(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);
    /********"ResponseMessageType_Int32",**********/
    void handleResponse0x00017C25(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);
    /********"ResponseMessageType_Int32List",**********/
    void handleResponse0x000093EE(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);
    /********"ResponseMessageType_PublishTable",**********/
    void handleResponse0x0000FC15(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);
    /********"ResponseMessageType_Bool",**********/
    void handleResponse0x00013EA3(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);
    /********"ResponseMessageType_RpcTable",**********/
    void handleResponse0x00009BC5(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);
    /********"ResponseMessageType_Bool",**********/	
    void handleResponse0x00007CF2(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);
    /********"ResponseMessageType_Bool",**********/	
    void handleResponse0x000031B2(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);
    /********"ResponseMessageType_Bool",**********/	
    void handleResponse0x00013062(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);
    /********"ResponseMessageType_RegisterR",**********/	
    void handleResponse0x000116F2(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);
    /********"ResponseMessageType_Bool",**********/	
    void handleResponse0x000098C2(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);
    /********"ResponseMessageType_UnsignedInt32",**********/	
    void handleResponse0x00005602(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);

    /********"ResponseMessageType_Bool",**********/	
    void handleResponse0x0000F862(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);
    /********"ResponseMessageType_Bool",**********/	
    void handleResponse0x0000C032(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);
    /********"ResponseMessageType_Bool",**********/	
    void handleResponse0x00005392(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);
    /********"RespnseMessageType_RegistePR",**********/	
    void handleResponse0x000170A2(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);
    /********"ResponseMessageType_Bool",**********/	
    void handleResponse0x000116B2(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);
    /********"ResponseMessageType_UnsignedInt32",**********/	
    void handleResponse0x0000F402(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);

    /********"ResponseMessageType_Bool",**********/	
    void handleResponse0x0000F892(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);
    /********"ResponseMessageType_Bool",**********/	
    void handleResponse0x0000C1C2(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);
    /********"ResponseMessageType_Bool",**********/	
    void handleResponse0x000053E2(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);
    /********"RespnseMessageType_RegisteMR",**********/	
    void handleResponse0x000170D2(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);
    /********"ResponseMessageType_Bool",**********/
    void handleResponse0x00011642(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);
    /********"ResponseMessageType_UnsignedInt32",**********/	
    void handleResponse0x0000F3B2(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);
    /********"ResponseMessageType_Bool",**********/	
    void handleResponse0x0000F932(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);
    /********"ResponseMessageType_Bool",**********/	
    void handleResponse0x0000BF62(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);
    /********"ResponseMessageType_Bool",**********/	
    void handleResponse0x00005342(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);
    /********"RespnseMessageType_RegisteSR",**********/	
    void handleResponse0x00017172(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);
    /********"ResponseMessageType_Bool",**********/	    
    void handleResponse0x000115E2(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);
    /********"ResponseMessageType_UnsignedInt32",**********/	
    void handleResponse0x0000F352(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);







    // publish element handler
    void handlePublishElement0x7622aa34(Comm_Publish& package, int element_index, TpPublishElement& list_element);
    /********"MessageType_Int32",**********/
    void handlePublishElement0x00011423(Comm_Publish& package, int element_index, TpPublishElement& list_element);
    /********"MessageType_Int32",**********/
    void handlePublishElement0x00010363(Comm_Publish& package, int element_index, TpPublishElement& list_element);
    /********"MessageType_Int32",**********/
    void handlePublishElement0x00001F13(Comm_Publish& package, int element_index, TpPublishElement& list_element);
    /********"MessageType_Int32",**********/
    void handlePublishElement0x0000D175(Comm_Publish& package, int element_index, TpPublishElement& list_element);
    /********"MessageType_String",**********/
    void handlePublishElement0x00015453(Comm_Publish& package, int element_index, TpPublishElement& list_element);
    /********"MessageType_Int32",**********/
    void handlePublishElement0x0000AB25(Comm_Publish& package, int element_index, TpPublishElement& list_element);


    // component parameters
    bool is_running_;
    int cycle_time_;    //ms
    std::string req_resp_ip_;
    std::string publish_ip_;
    int recv_buffer_size_;
    int send_buffer_size_;

    // runtime variables
    std::thread* thread_ptr_;
    int req_resp_socket_;
    int publish_socket_;
    int req_resp_endpoint_id_;
    int publish_endpoint_id_;
    struct nn_pollfd poll_fd_;
    uint8_t* recv_buffer_ptr_;
    uint8_t* send_buffer_ptr_;
    fst_parameter::ParamGroup param_;

    // runtime table    
    typedef struct
    {
        std::string path;
        unsigned int hash;
        std::string request_type;
        std::string response_type;
        HandleRequestFuncPtr request_func_ptr;
        HandleResponseFuncPtr response_func_ptr;
        Comm_Authority authority;
    }RpcService;
    std::vector<RpcService> rpc_table_;
    std::vector<RpcService> rpc_quick_search_table_[QUICK_SEARCH_TABLE_SIZE];

    typedef struct
    {
        std::string element_path;
        unsigned int hash;
        std::string element_type;
        HandlePublishElementFuncPtr publish_element_func_ptr;
        Comm_Authority authority;
    }PublishService;
    std::vector<PublishService> publish_element_table_;
    std::vector<PublishService> publish_element_quick_search_table_[QUICK_SEARCH_TABLE_SIZE];

    // runtime list & list mutex
    std::mutex request_list_mutex_;
    std::mutex response_list_mutex_;
    std::mutex publish_list_mutex_;
    std::vector<TpRequestResponse>  request_list_;
    std::vector<TpRequestResponse>  response_list_;
    std::vector<TpPublish>  publish_list_;
};

#endif

