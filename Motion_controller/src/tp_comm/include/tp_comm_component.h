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
    comm_Publish package;
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

class TpCommComponent
{
public:
    TpCommComponent();
    ~TpCommComponent();

    bool init();

    bool open();
    void close();

    std::vector<TpRequestResponse> popTaskFromRequestList();
    bool getResponseSucceed(void* response_data_ptr);
    void pushTaskToResponseList(TpRequestResponse& package);
    TpPublish generateTpPublishTask(unsigned int topic_hash, int interval_min, int interval_max);
    void addTpPublishElement(TpPublish& task, unsigned int element_hash, void* element_data_ptr);
    void pushTaskToPublishList(TpPublish& package);
    
private:
    enum {HASH_BYTE_SIZE = 4,};
    enum {QUICK_SEARCH_TABLE_SIZE = 128,};

    typedef void (TpCommComponent::*HandleRequestFuncPtr)(int);
    typedef void (TpCommComponent::*HandleResponseFuncPtr)(std::vector<TpRequestResponse>::iterator&, int&);
    typedef void (TpCommComponent::*HandlePublishElementFuncPtr)(comm_Publish&, int, TpPublishElement&);

    void initRpcTable();
    void initRpcQuickSearchTable();
    void initPublishElementTable();
    void initPublishElementQuickSearchTable();
    bool initParamsFromYaml();
    HandleRequestFuncPtr getRequestHandlerByHash(unsigned int hash);
    HandleResponseFuncPtr getResponseHandlerByHash(unsigned int hash);
    HandlePublishElementFuncPtr getPublishElementHandlerByHash(unsigned int hash);
    
    void tpCommThreadFunc();
    void handleRequest();
    void pushTaskToRequestList(unsigned int hash, void* request_data_ptr, void* response_data_ptr);
    void handleResponseList();
    void handlePublishList();

    long computeTimeElapsed(struct timeval& current_time_val, struct timeval& last_time_val);
    long computeTimeForTp(struct timeval& current_time_val);
    bool checkPublishCondition(long time_elapsed, bool is_element_changed, int interval_min, int interval_max);

    bool decodeRequestPackage(const pb_field_t fields[], void* request_data_ptr, int recv_bytes);
    bool checkAuthority(comm_Authority request_authority, comm_Authority controller_authority);
    void initResponsePackage(void* request_data_ptr, void* response_data_ptr, int package_left);
    void initCommFailedResponsePackage(void* request_data_ptr, void* response_data_ptr);
    void handleRequestPackage(unsigned int hash, void* request_data_ptr, void* response_data_ptr, int recv_bytes, 
                                    const pb_field_t fields[], comm_Authority controller_authority, int package_left);

    bool encodeResponsePackage(unsigned int hash, const pb_field_t fields[], void* response_data_ptr, int& send_buffer_size);
    bool encodePublishElement(comm_PublishElement_data_t& element, const pb_field_t fields[], void* element_data_ptr);
    bool encodePublishPackage(comm_Publish& package, unsigned int hash, struct timeval& current_time_val, int& send_buffer_size);

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

    // publish element handler
    void handlePublishElement0x7622aa34(comm_Publish& package, int element_index, TpPublishElement& list_element);
    /********"MessageType_Int32",**********/
    void handlePublishElement0x00011423(comm_Publish& package, int element_index, TpPublishElement& list_element);
    /********"MessageType_Int32",**********/
    void handlePublishElement0x00010363(comm_Publish& package, int element_index, TpPublishElement& list_element);
    /********"MessageType_Int32",**********/
    void handlePublishElement0x00001F13(comm_Publish& package, int element_index, TpPublishElement& list_element);
    /********"MessageType_Int32",**********/
    void handlePublishElement0x0000D175(comm_Publish& package, int element_index, TpPublishElement& list_element);
    /********"MessageType_String",**********/
    void handlePublishElement0x00015453(comm_Publish& package, int element_index, TpPublishElement& list_element);
    /********"MessageType_Int32",**********/
    void handlePublishElement0x0000AB25(comm_Publish& package, int element_index, TpPublishElement& list_element);


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
    }RpcService;
    std::vector<RpcService> rpc_table_;
    std::vector<RpcService> rpc_quick_search_table_[QUICK_SEARCH_TABLE_SIZE];

    typedef struct
    {
        std::string element_path;
        unsigned int hash;
        std::string element_type;
        HandlePublishElementFuncPtr publish_element_func_ptr;
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

