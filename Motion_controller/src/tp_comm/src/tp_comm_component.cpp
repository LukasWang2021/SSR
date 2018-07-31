#include <unistd.h>
#include <string>
#include <nanomsg/pubsub.h>
#include <nanomsg/reqrep.h>
#include <pb_decode.h>
#include <pb_encode.h>
#include "tp_comm_component.h"
#include "common/common.h"

using namespace std;


TpCommComponent::TpCommComponent():
    is_running_(false), cycle_time_(10), 
    req_resp_socket_(-1), publish_socket_(-1),
    req_resp_endpoint_id_(-1), publish_endpoint_id_(-1),
    req_resp_ip_("ws://192.168.1.166:5600"), publish_ip_("ws://192.168.1.166:5601"),
    recv_buffer_size_(65536), send_buffer_size_(65536)
{
}

TpCommComponent::~TpCommComponent()
{
    this->close();
}

bool TpCommComponent::initParamsFromYaml()
{
    std::string file_path = "share/configuration/configurable/tp_comm.yaml";
    if (!param_.loadParamFile(file_path.c_str()))
    {
        FST_ERROR("TP COMM : yaml file load failed!");
        return false;
    }

    param_.getParam("/rpc_ip", req_resp_ip_);
    param_.getParam("/pub_ip", publish_ip_);
    param_.getParam("/recv_buffer_size", recv_buffer_size_);
    param_.getParam("/send_buffer_size", send_buffer_size_);
    param_.getParam("/cycle_time", cycle_time_);

    FST_INFO("TP Comm : rpc ip = %c", req_resp_ip_.c_str());
    FST_INFO("TP Comm : pub ip = %c", publish_ip_.c_str());
    FST_INFO("TP Comm : recv buffer size = %d", recv_buffer_size_);
    FST_INFO("TP Comm : send buffer size = %d", send_buffer_size_);
    FST_INFO("TP Comm : cycle time = %d", cycle_time_);
    return true;
}

bool TpCommComponent::init()
{
    // load parameter from yaml
    if (!initParamsFromYaml())
    {
        return false;
    }

    // initialize runtime variables
    req_resp_socket_ = nn_socket(AF_SP, NN_REP);
    if(req_resp_socket_ == -1)
    {
        return false;
    }
    
    req_resp_endpoint_id_ = nn_bind(req_resp_socket_, req_resp_ip_.c_str());
    if(req_resp_endpoint_id_ == -1)
    {
        return false;
    }
    
    publish_socket_ = nn_socket(AF_SP, NN_PUB);
    if(publish_socket_ == -1)
    {
        return false;
    }
    publish_endpoint_id_ = nn_bind(publish_socket_, publish_ip_.c_str());
    if(publish_endpoint_id_ == -1)
    {
        return false;
    }

    // it is critical to set poll fd here to make the model work correctly
	poll_fd_.fd = req_resp_socket_;
	poll_fd_.events = NN_POLLIN | NN_POLLOUT;    

    recv_buffer_ptr_ = new uint8_t[recv_buffer_size_]();
    send_buffer_ptr_ = new uint8_t[send_buffer_size_]();

    initRpcTable();
    initRpcQuickSearchTable();
    initPublishElementTable();
    initPublishElementQuickSearchTable();

    FST_INFO("TpComm init success");
    return true;
}

bool TpCommComponent::open()
{
    is_running_ = true;
    thread_ptr_ = new std::thread(std::bind(&TpCommComponent::tpCommThreadFunc, this));
    if(thread_ptr_ != NULL)
    {
        return true;
    }
    else
    {
        return false;
    }
}

void TpCommComponent::close()
{
    if(thread_ptr_ != NULL)
    {
        is_running_ = false;
        thread_ptr_->join();
        delete thread_ptr_;
        thread_ptr_ = NULL;
    }

    if(req_resp_socket_ != -1 && req_resp_endpoint_id_ != -1)
    {
        nn_shutdown(req_resp_socket_, req_resp_endpoint_id_);
    }
    
    if(publish_socket_ != -1 && publish_endpoint_id_ != -1)
    {
        nn_shutdown(publish_socket_, publish_endpoint_id_);
    }    
}

std::vector<TpRequestResponse> TpCommComponent::popTaskFromRequestList()
{
    std::vector<TpRequestResponse> request_list;
    std::vector<TpRequestResponse>::iterator it;
    request_list_mutex_.lock();
    for(it = request_list_.begin(); it != request_list_.end(); ++it)
    {
        request_list.push_back(*it);
    }
    request_list_.clear();
    request_list_mutex_.unlock();
    return request_list;
}

bool TpCommComponent::getResponseSucceed(void* response_data_ptr)
{
    return ((comm_ResponseMessageTypeInt32*)response_data_ptr)->header.succeed;
}

void TpCommComponent::pushTaskToResponseList(TpRequestResponse& package)
{
    response_list_mutex_.lock();
    response_list_.push_back(package);
    response_list_mutex_.unlock();
}

TpPublish TpCommComponent::generateTpPublishTask(unsigned int topic_hash, int interval_min, int interval_max)
{
    TpPublish task;
    task.hash = topic_hash;
    task.interval_min = interval_min;
    task.interval_max = interval_max;
    task.is_element_changed = true;
    task.last_publish_time.tv_sec = 0;
    task.last_publish_time.tv_usec = 0;
    task.package.element_count = 0;
    return task;
}

void TpCommComponent::addTpPublishElement(TpPublish& task, unsigned int element_hash, void* element_data_ptr)
{
    TpPublishElement element;
    element.hash = element_hash;
    element.data_ptr = element_data_ptr;
    task.element_list_.push_back(element);
    task.package.element_count = task.element_list_.size();
    task.package.element[task.package.element_count - 1].hash = element_hash;
}

void TpCommComponent::pushTaskToPublishList(TpPublish& package)
{
    publish_list_mutex_.lock();
    publish_list_.push_back(package);
    publish_list_mutex_.unlock();
}

void TpCommComponent::initRpcQuickSearchTable()
{
    unsigned int remainder;
    for(unsigned int i = 0; i < rpc_table_.size(); ++i)
    {
        remainder = rpc_table_[i].hash % QUICK_SEARCH_TABLE_SIZE;
        rpc_quick_search_table_[remainder].push_back(rpc_table_[i]);
    }
}

void TpCommComponent::initPublishElementQuickSearchTable()
{
    unsigned int remainder;
    for(unsigned int i = 0; i < publish_element_table_.size(); ++i)
    {
        remainder = publish_element_table_[i].hash % QUICK_SEARCH_TABLE_SIZE;
        publish_element_quick_search_table_[remainder].push_back(publish_element_table_[i]);
    }
}

TpCommComponent::HandleRequestFuncPtr TpCommComponent::getRequestHandlerByHash(unsigned int hash)
{
    unsigned int remainder = hash % QUICK_SEARCH_TABLE_SIZE;
    for(unsigned int i = 0; i < rpc_quick_search_table_[remainder].size(); ++i)
    {
        if(rpc_quick_search_table_[remainder][i].hash == hash)
        {
            return rpc_quick_search_table_[remainder][i].request_func_ptr;
        }
    }
    return NULL;
}

TpCommComponent::HandleResponseFuncPtr TpCommComponent::getResponseHandlerByHash(unsigned int hash)
{
    unsigned int remainder = hash % QUICK_SEARCH_TABLE_SIZE;
    for(unsigned int i = 0; i < rpc_quick_search_table_[remainder].size(); ++i)
    {
        if(rpc_quick_search_table_[remainder][i].hash == hash)
        {
            return rpc_quick_search_table_[remainder][i].response_func_ptr;
        }
    }
    return NULL;
}

TpCommComponent::HandlePublishElementFuncPtr TpCommComponent::getPublishElementHandlerByHash(unsigned int hash)
{
    unsigned int remainder = hash % QUICK_SEARCH_TABLE_SIZE;
    for(unsigned int i = 0; i < publish_element_quick_search_table_[remainder].size(); ++i)
    {
        if(publish_element_quick_search_table_[remainder][i].hash == hash)
        {
            return publish_element_quick_search_table_[remainder][i].publish_element_func_ptr;
        }
    }
    return NULL;
}



void TpCommComponent::tpCommThreadFunc()
{
    while(is_running_)
    {
        handleRequest();
        handleResponseList();
        handlePublishList();
        usleep(cycle_time_);
    }
}

void TpCommComponent::handleRequest()

{
    if(nn_poll (&poll_fd_, 1, 0) == -1)
    {
        return;
    }

    int recv_bytes;
    if(poll_fd_.revents & NN_POLLIN)
    {
        recv_bytes = nn_recv(req_resp_socket_, recv_buffer_ptr_, recv_buffer_size_, 0);
        if(recv_bytes == -1)
        {
            return;
        }
    }
    else
    {
        return;
    }

    // hash is 128bits long, only take lsb 32bits
    unsigned int hash = *((unsigned int*)recv_buffer_ptr_);
    HandleRequestFuncPtr func_ptr = getRequestHandlerByHash(hash);
    if(func_ptr != NULL)
    {
        (this->*func_ptr)(recv_bytes);
    }
}

void TpCommComponent::pushTaskToRequestList(unsigned int hash, void* request_data_ptr, void* response_data_ptr)
{
    TpRequestResponse package;
    package.hash = hash;
    package.request_data_ptr = request_data_ptr;
    package.response_data_ptr = response_data_ptr;
    request_list_mutex_.lock();
    request_list_.push_back(package);
    request_list_mutex_.unlock();
}

void TpCommComponent::handleResponseList()
{
    // it is tricky here, if no usleep, nn_send will fail. fix me later
    int send_buffer_size;
    std::vector<TpRequestResponse>::iterator it;
    HandleResponseFuncPtr func_ptr;
    response_list_mutex_.lock();
    for(it = response_list_.begin(); it != response_list_.end(); ++it)
    {
        func_ptr = getResponseHandlerByHash(it->hash);
        if(func_ptr != NULL)
        {
            (this->*func_ptr)(it, send_buffer_size);
        }
        else
        {
            return;
        }
        if(send_buffer_size == 0)
        {
            return;
        }

        int send_bytes = nn_send(req_resp_socket_, send_buffer_ptr_, send_buffer_size, 0); // block send
        if(send_bytes == -1)
        {
            FST_ERROR("handleResponseList: send response failed, nn_error = %d", nn_errno());
        }
    }
    response_list_.clear();
    response_list_mutex_.unlock();
}

void TpCommComponent::handlePublishList()
{
    std::vector<TpPublish>::iterator it;
    struct timeval time_val;
    long time_elapsed;
    HandlePublishElementFuncPtr func_ptr; 
    publish_list_mutex_.lock();
    gettimeofday(&time_val, NULL);
    for(it = publish_list_.begin(); it != publish_list_.end(); ++it)
    {
        time_elapsed = computeTimeElapsed(time_val, it->last_publish_time);
        if(checkPublishCondition(time_elapsed, it->is_element_changed, it->interval_min, it->interval_max))
        {            
            // encode TpPublishElement
            for(int i = 0; i < it->package.element_count; ++i)
            {
                func_ptr = getPublishElementHandlerByHash(it->package.element[i].hash);
                if(func_ptr != NULL)
                {
                    (this->*func_ptr)(it->package, i, it->element_list_[i]);
                }
            }

            // encode TpPublish
            int send_buffer_size;
            if(!encodePublishPackage(it->package, it->hash, time_val, send_buffer_size))
            {
                FST_ERROR("handlePublishList: encode data failed");
                break;
            }

            int send_bytes = nn_send(publish_socket_, send_buffer_ptr_, send_buffer_size, 0); // block send
            if(send_bytes == -1)
            {
                FST_INFO("handlePublishList: send publish failed, error = %d", nn_errno());
                break;
            }

            it->is_element_changed = false;
            it->last_publish_time = time_val;         
        }
    }
    publish_list_mutex_.unlock();
}

long TpCommComponent::computeTimeElapsed(struct timeval& current_time_val, struct timeval& last_time_val)
{
    long delta_tv_sec = current_time_val.tv_sec - last_time_val.tv_sec;
    long delta_tv_usec = current_time_val.tv_usec - last_time_val.tv_usec;
    return delta_tv_sec * 1000 + delta_tv_usec / 1000;
}

long TpCommComponent::computeTimeForTp(struct timeval& current_time_val)
{
    return current_time_val.tv_sec * 1000000 + current_time_val.tv_usec;
}

bool TpCommComponent::checkPublishCondition(long time_elapsed, bool is_element_changed, int interval_min, int interval_max)
{
    if(is_element_changed)
    {
        if(time_elapsed >= interval_min)
        {
            return true;
        }
        else
        {
            return false;
        }
    }
    else if(time_elapsed >= interval_max)
    {
        return true;
    }
    else
    {
        return false;
    }
}

bool TpCommComponent::decodeRequestPackage(const pb_field_t fields[], void* request_data_ptr, int recv_bytes)
{
    pb_istream_t stream = pb_istream_from_buffer(recv_buffer_ptr_ + HASH_BYTE_SIZE, recv_bytes - HASH_BYTE_SIZE);
    return pb_decode(&stream, fields, (void*)request_data_ptr);
}

bool TpCommComponent::checkAuthority(comm_Authority request_authority, comm_Authority controller_authority)
{
    return (request_authority >= controller_authority ? true:false); 
}

void TpCommComponent::initResponsePackage(void* request_data_ptr, void* response_data_ptr, int package_left)
{
    // it is tricky here that all request & response have the same package header and property, no matter what data they have
    ((comm_ResponseMessageTypeInt32*)response_data_ptr)->header.time_stamp = ((comm_RequestMessageTypeInt32*)request_data_ptr)->header.time_stamp;
    ((comm_ResponseMessageTypeInt32*)response_data_ptr)->header.package_left = package_left;
    ((comm_ResponseMessageTypeInt32*)response_data_ptr)->header.succeed = true;
    ((comm_ResponseMessageTypeInt32*)response_data_ptr)->property.authority = ((comm_RequestMessageTypeInt32*)request_data_ptr)->property.authority;
}

void TpCommComponent::initCommFailedResponsePackage(void* request_data_ptr, void* response_data_ptr)
{
    // it is tricky here that all request & response have the same package header and property, no matter what data they have
    ((comm_ResponseMessageTypeInt32*)response_data_ptr)->header.time_stamp = ((comm_RequestMessageTypeInt32*)request_data_ptr)->header.time_stamp;
    ((comm_ResponseMessageTypeInt32*)response_data_ptr)->header.package_left = -1;
    ((comm_ResponseMessageTypeInt32*)response_data_ptr)->header.succeed = false;
    ((comm_ResponseMessageTypeInt32*)response_data_ptr)->property.authority = ((comm_RequestMessageTypeInt32*)request_data_ptr)->property.authority;
}

void TpCommComponent::handleRequestPackage(unsigned int hash, void* request_data_ptr, void* response_data_ptr, int recv_bytes, 
                                                const pb_field_t fields[], comm_Authority controller_authority, int package_left)
{
    if(!decodeRequestPackage(fields, (void*)request_data_ptr, recv_bytes))
    {
        FST_ERROR("handleRequestPackage: /tp_comm/test_request: decode data failed");
        return ;
    }

    if(!checkAuthority(((comm_RequestMessageTypeInt32*)request_data_ptr)->property.authority, controller_authority))
    {
        FST_ERROR("handleRequestPackage: /tp_comm/test_request: operation is not authorized");
        // If operation not authorized, don't throw a response to response list directly but reuse the normal channel,
        // that is throwing the task to request list. User should judge if response_data_ptr.header.succeed is false.
        // If it is false, then don't process anything, throw it to response list directly caling by pushTaskToRequestList().
        // Although it seems to be detour, one can trace the program processing more easily.
        initCommFailedResponsePackage(request_data_ptr, response_data_ptr);
    }
    else
    {
        initResponsePackage(request_data_ptr, response_data_ptr, -1);
    }
    pushTaskToRequestList(hash, request_data_ptr, response_data_ptr);
}

bool TpCommComponent::encodeResponsePackage(unsigned int hash, const pb_field_t fields[], void* response_data_ptr, int& send_buffer_size)
{
    *((unsigned int*)send_buffer_ptr_) = hash;
    pb_ostream_t stream = pb_ostream_from_buffer(send_buffer_ptr_ + HASH_BYTE_SIZE, send_buffer_size_ - HASH_BYTE_SIZE);
    if(!pb_encode(&stream, fields, response_data_ptr))
    {
        FST_ERROR("encodeResponsePackage: encode data failed");
        send_buffer_size = 0;
        return false;
    }
    send_buffer_size = stream.bytes_written + HASH_BYTE_SIZE;
    return true;
}

bool TpCommComponent::encodePublishElement(comm_PublishElement_data_t& element, const pb_field_t fields[], void* element_data_ptr)
{
    pb_ostream_t element_stream = pb_ostream_from_buffer(element.bytes, sizeof(element.bytes));
    if(!pb_encode(&element_stream, fields, element_data_ptr))
    {
        FST_ERROR("encodePublishElement: encode element data failed");
        element.size = 0;
        return false;
    }
    element.size = element_stream.bytes_written;
    return true;
}

bool TpCommComponent::encodePublishPackage(comm_Publish& package, unsigned int hash, struct timeval& current_time_val, int& send_buffer_size)
{
    *((unsigned int*)send_buffer_ptr_) = hash;
    package.time_stamp = computeTimeForTp(current_time_val);
    pb_ostream_t stream = pb_ostream_from_buffer(send_buffer_ptr_ + HASH_BYTE_SIZE, send_buffer_size_ - HASH_BYTE_SIZE);
    if(!pb_encode(&stream, comm_Publish_fields, &package))
    {
        FST_ERROR("encodePublishPackage: encode data failed");
        return false;
    }            
    send_buffer_size = stream.bytes_written + HASH_BYTE_SIZE;
    return true;
}

