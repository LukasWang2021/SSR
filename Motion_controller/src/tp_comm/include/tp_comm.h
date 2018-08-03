#ifndef TP_COMM_H
#define TP_COMM_H


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

    /********GetUserOpMode, ResponseMessageType_Int32**********/	
    void handleRequest0x00000C05(int recv_bytes);
    /********GetRunningStatus, ResponseMessageType_Int32**********/	
    void handleRequest0x00000AB3(int recv_bytes);
    /********GetInterpreterStatus, ResponseMessageType_Int32**********/	
    void handleRequest0x00016483(int recv_bytes);
    /********GetRobotStatus, ResponseMessageType_Int32**********/	
    void handleRequest0x00006F83(int recv_bytes);
    /********GetCtrlStatus, ResponseMessageType_Int32**********/	
    void handleRequest0x0000E9D3(int recv_bytes);
    /********GetServoStatus, ResponseMessageType_Int32**********/	
    void handleRequest0x0000D113(int recv_bytes);
    /********GetSafetyAlarm, ResponseMessageType_Int32**********/	
    void handleRequest0x0000C00D(int recv_bytes);
    /********CallEstop, ResponseMessageType_Bool**********/	        
    void handleRequest0x00013940(int recv_bytes);
    /********CallReset, ResponseMessageType_Bool**********/     	
    void handleRequest0x000161E4(int recv_bytes);
    /********SetUserOpMode, ResponseMessageType_Bool**********/	    
    void handleRequest0x00002ED5(int recv_bytes);
    /********addTopic, ResponseMessageType_Bool**********/	
    void handleRequest0x00000773(int recv_bytes);


    /********GetUserOpMode, ResponseMessageType_Int32**********/	
    void handleResponse0x00000C05(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);
    /********GetRunningStatus, ResponseMessageType_Int32**********/	
    void handleResponse0x00000AB3(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);
    /********GetInterpreterStatus, ResponseMessageType_Int32**********/	
    void handleResponse0x00016483(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);
    /********GetRobotStatus, ResponseMessageType_Int32**********/	
    void handleResponse0x00006F83(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);
    /********GetCtrlStatus, ResponseMessageType_Int32**********/	
    void handleResponse0x0000E9D3(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);
    /********GetServoStatus, ResponseMessageType_Int32**********/	
    void handleResponse0x0000D113(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);
    /********GetSafetyAlarm, ResponseMessageType_Int32**********/	
    void handleResponse0x0000C00D(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);
    /********CallEstop, ResponseMessageType_Bool**********/	        
    void handleResponse0x00013940(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);
    /********CallReset, ResponseMessageType_Bool**********/	        
    void handleResponse0x000161E4(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);
    /********SetUserOpMode, ResponseMessageType_Bool**********/	    
    void handleResponse0x00002ED5(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);
    /********addTopic, ResponseMessageType_Bool**********/
    void handleResponse0x00000773(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);


    /********UserOpMode, MessageType_Int32**********/  
    void handlePublishElement0x00015255(Comm_Publish& package, int element_index, TpPublishElement& list_element);
    /********RunningStatus, MessageType_Int32**********/
    void handlePublishElement0x00001F33(Comm_Publish& package, int element_index, TpPublishElement& list_element);
    /********InterpreterStatus, MessageType_Int32**********/
    void handlePublishElement0x00003203(Comm_Publish& package, int element_index, TpPublishElement& list_element);
    /********RobotStatus, MessageType_Int32**********/
    void handlePublishElement0x00012943(Comm_Publish& package, int element_index, TpPublishElement& list_element);
    /********CtrlStatus, MessageType_Int32**********/
    void handlePublishElement0x0000E8E3(Comm_Publish& package, int element_index, TpPublishElement& list_element);
    /********ServoStatus, MessageType_Int32**********/
    void handlePublishElement0x00002053(Comm_Publish& package, int element_index, TpPublishElement& list_element);
    /********SafetyAlarm, MessageType_Int32**********/
    void handlePublishElement0x0000D0AD(Comm_Publish& package, int element_index, TpPublishElement& list_element);

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

