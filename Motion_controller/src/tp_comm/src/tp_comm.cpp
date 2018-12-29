#include <unistd.h>
#include <string>
#include <iostream>
#include <nanomsg/nn.h>
#include <nanomsg/ws.h>
#include <nanomsg/pubsub.h>
#include <nanomsg/reqrep.h>
#include <pb_decode.h>
#include <pb_encode.h>
#include "error_monitor.h"
#include "error_code.h"
#include "serverAlarmApi.h"
#include "tp_comm.h"

using namespace fst_base;
using namespace std;
using namespace fst_comm;


TpComm::TpComm():
    log_ptr_(NULL), param_ptr_(NULL),
    is_running_(false), cycle_time_(10), 
    req_resp_socket_(-1), publish_socket_(-1),
    req_resp_endpoint_id_(-1), publish_endpoint_id_(-1),
    recv_buffer_size_(65536), send_buffer_size_(65536)
{
    string ip = local_ip_.get();
    req_resp_ip_ = "ws://" + ip + ":5600";
    publish_ip_ = "ws://" + ip + ":5601";
    is_received_ = false;

    log_ptr_ = new fst_log::Logger();
    param_ptr_ = new TpCommManagerParam();
    FST_LOG_INIT("TpComm");
    FST_LOG_SET_LEVEL((fst_log::MessageLevel)param_ptr_->log_level_);
}

TpComm::~TpComm()
{
    this->close();

    if(log_ptr_ != NULL)
    {
        delete log_ptr_;
        log_ptr_ = NULL;
    }
    if(param_ptr_ != NULL)
    {
        delete param_ptr_;
        param_ptr_ = NULL;
    };
}

bool TpComm::initComponentParams()
{
    if(!param_ptr_->loadParam()) return false;

    FST_LOG_SET_LEVEL((fst_log::MessageLevel)param_ptr_->log_level_);   

    recv_buffer_size_ = param_ptr_->recv_buffer_size_;
    send_buffer_size_ = param_ptr_->send_buffer_size_;
    cycle_time_ = param_ptr_->cycle_time_;

    return true;
}

unsigned long long int TpComm::init()
{
    if (!initComponentParams()) 
    {
        FST_ERROR("Failed to load TpCommManager component parameters");
        return TP_COMM_LOAD_PARAM_FAILED;
    }

    req_resp_socket_ = nn_socket(AF_SP, NN_REP);
    if(req_resp_socket_ == -1)
    {
        FST_ERROR("Failed to init rpc socket");
        return TP_COMM_INIT_OBJECT_FAILED;
    }

    req_resp_endpoint_id_ = nn_bind(req_resp_socket_, req_resp_ip_.c_str());
    if(req_resp_endpoint_id_ == -1)
    {
        FST_ERROR("Failed to bind rpc socket");
        return TP_COMM_INIT_OBJECT_FAILED;
    }

    publish_socket_ = nn_socket(AF_SP, NN_PUB);
    if(publish_socket_ == -1)
    {
        FST_ERROR("Failed to init pub socket");
        return TP_COMM_INIT_OBJECT_FAILED;
    }

    publish_endpoint_id_ = nn_bind(publish_socket_, publish_ip_.c_str());
    if(publish_endpoint_id_ == -1)
    {
        FST_ERROR("Failed to bind pub socket");
        return TP_COMM_INIT_OBJECT_FAILED;
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
    return SUCCESS;
}

unsigned long long int TpComm::open()
{
    is_running_ = true;

    if(!thread_ptr_.run(&tpCommRoutineThreadFunc, this, 50))
    {
        FST_ERROR("Failed to open tpcomm");
        return TP_COMM_OPEN_FAILED;
    }

    return SUCCESS;
}

bool TpComm::isRunning()
{
    return is_running_;
}

void TpComm::close()
{
    is_running_ = false;
    thread_ptr_.join();

    if(req_resp_socket_ != -1 && req_resp_endpoint_id_ != -1)
    {
        nn_shutdown(req_resp_socket_, req_resp_endpoint_id_);
    }
    
    if(publish_socket_ != -1 && publish_endpoint_id_ != -1)
    {
        nn_shutdown(publish_socket_, publish_endpoint_id_);
    }
}

 bool TpComm::popTaskFromRequestList(TpRequestResponse* task)
{
    if(task == NULL)
    {
        return false;
    }
    
    request_list_mutex_.lock();

    if(!request_list_.empty())
    {
        *task = request_list_.front();
        request_list_.pop_front();
        request_list_mutex_.unlock();
        return true;
    }
    else
    {
        request_list_mutex_.unlock();
        return false;
    }
}

int32_t TpComm::getResponseSucceed(void* response_data_ptr)
{
    return ((ResponseMessageType_Int32*)response_data_ptr)->header.error_code;
}

void TpComm::pushTaskToResponseList(TpRequestResponse& package)
{
    response_list_mutex_.lock();
    response_list_.push_back(package);
    response_list_mutex_.unlock();
}

void TpComm::lockPublishMutex()
{
    publish_list_mutex_.lock();
}

void TpComm::unlockPublishMutex()
{
    publish_list_mutex_.unlock();
}

TpPublish TpComm::generateTpPublishTask(unsigned int topic_hash, int interval_min, int interval_max)
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

void TpComm::addTpPublishElement(TpPublish& task, unsigned int element_hash, void* element_data_ptr)
{
    TpPublishElement element;
    element.hash = element_hash;
    element.data_ptr = element_data_ptr;
    task.element_list_.push_back(element);
    task.package.element_count = task.element_list_.size();
    task.package.element[task.package.element_count - 1].hash = element_hash;
}

void TpComm::pushTaskToPublishList(TpPublish& package)
{
    publish_list_mutex_.lock();
    publish_list_.push_back(package);
    publish_list_mutex_.unlock();
}

void TpComm::initRpcQuickSearchTable()
{
    unsigned int remainder;
    for(unsigned int i = 0; i < rpc_table_.size(); ++i)
    {
        remainder = rpc_table_[i].hash % QUICK_SEARCH_TABLE_SIZE;
        rpc_quick_search_table_[remainder].push_back(rpc_table_[i]);
    }
}

void TpComm::initPublishElementQuickSearchTable()
{
    unsigned int remainder;
    for(unsigned int i = 0; i < publish_element_table_.size(); ++i)
    {
        remainder = publish_element_table_[i].hash % QUICK_SEARCH_TABLE_SIZE;
        publish_element_quick_search_table_[remainder].push_back(publish_element_table_[i]);
    }
}

TpComm::HandleRequestFuncPtr TpComm::getRequestHandlerByHash(unsigned int hash)
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

Comm_Authority TpComm::getRpcTableElementAuthorityByHash(unsigned int hash)
{
    unsigned int remainder = hash % QUICK_SEARCH_TABLE_SIZE;
    for(unsigned int i = 0; i < rpc_quick_search_table_[remainder].size(); ++i)
    {
        if(rpc_quick_search_table_[remainder][i].hash == hash)
        {
            return rpc_quick_search_table_[remainder][i].authority;
        }
    }
    return NULL;
}

TpComm::HandleResponseFuncPtr TpComm::getResponseHandlerByHash(unsigned int hash)
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

TpComm::HandlePublishElementFuncPtr TpComm::getPublishElementHandlerByHash(unsigned int hash)
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

Comm_Authority TpComm::getPublishElementAuthorityByHash(unsigned int hash)
{
    unsigned int remainder = hash % QUICK_SEARCH_TABLE_SIZE;
    for(unsigned int i = 0; i < publish_element_quick_search_table_[remainder].size(); ++i)
    {
        if(publish_element_quick_search_table_[remainder][i].hash == hash)
        {
            return publish_element_quick_search_table_[remainder][i].authority;
        }
    }
    return NULL;
}

void TpComm::tpCommThreadFunc()
{
    handleRequest();
    handleResponseList();
    handlePublishList();
    handleIoPublishList();
    handleRegPublishList();
    usleep(cycle_time_);
}

void TpComm::handleRequest()
{
    if (is_received_) return;

    if(nn_poll (&poll_fd_, 1, 0) <= 0)
    {
        return;
    }
    int recv_bytes;
    if(poll_fd_.revents & NN_POLLIN)
    {
        recv_bytes = nn_recv(req_resp_socket_, recv_buffer_ptr_, recv_buffer_size_, 0);
        if(recv_bytes == -1)
        {
            ErrorMonitor::instance()->add(TP_COMM_RECEIVE_FAILED);
            FST_ERROR("Failed to receive request.");
            return;
        }
        is_received_ = true;
    }
    else
    {
        return;
    }

    if(request_list_.size() > param_ptr_->rpc_list_max_size_)
    {
        ErrorMonitor::instance()->add(TP_COMM_RPC_OVERLOAD);
        FST_ERROR("Too much rpc to handle.");
        return;
    }

    unsigned int hash = *((unsigned int*)recv_buffer_ptr_);
    FST_INFO("---handleRequest: %x %x %x %x", recv_buffer_ptr_[0], recv_buffer_ptr_[1], recv_buffer_ptr_[2], recv_buffer_ptr_[3]);

    HandleRequestFuncPtr func_ptr = getRequestHandlerByHash(hash);
    if(func_ptr != NULL)
    {
        (this->*func_ptr)(recv_bytes);
    }
    else
    {
        handleRequestNonexistentHash(hash, recv_bytes);
        ErrorMonitor::instance()->add(TP_COMM_INVALID_REQUEST);
        FST_ERROR("Request invalid.");
    }
}

void TpComm::pushTaskToRequestList(unsigned int hash, void* request_data_ptr, void* response_data_ptr)
{
    TpRequestResponse package;
    package.hash = hash;
    package.request_data_ptr = request_data_ptr;
    package.response_data_ptr = response_data_ptr;
    request_list_mutex_.lock();
    request_list_.push_back(package);
    request_list_mutex_.unlock();
}

void TpComm::handleResponseList()
{
    int send_buffer_size = 0;
    std::vector<TpRequestResponse>::iterator it;
    HandleResponseFuncPtr func_ptr;
    response_list_mutex_.lock();

    if (response_list_.empty())
    {
        response_list_mutex_.unlock();
        return;
    }

    for(it = response_list_.begin(); it != response_list_.end(); ++it)
    {
        func_ptr = getResponseHandlerByHash(it->hash);
        if(func_ptr != NULL)
        {
            (this->*func_ptr)(it, send_buffer_size);
        }
        else
        {
            handleResponseNonexistentHash(it, send_buffer_size);
        }

        int send_bytes = nn_send(req_resp_socket_, send_buffer_ptr_, send_buffer_size, 0); // block send
        if(send_bytes == -1)
        {
            ErrorMonitor::instance()->add(TP_COMM_SEND_FAILED);
            FST_ERROR("Send response failed, nn_error = %d", nn_errno());
        }
    }
    is_received_ = false;
    response_list_.clear();
    response_list_mutex_.unlock();
}

void TpComm::handlePublishList()
{
    std::vector<TpPublish>::iterator it;
    struct timeval time_val;
    long long time_elapsed;
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
                FST_ERROR("Encode data failed");
                ErrorMonitor::instance()->add(TP_COMM_ENCODE_FAILED);
                break;
            }

            int send_bytes = nn_send(publish_socket_, send_buffer_ptr_, send_buffer_size, 0); // block send
            if(send_bytes == -1)
            {
                ErrorMonitor::instance()->add(TP_COMM_SEND_FAILED);
                FST_INFO("Send publish failed, error = %d", nn_errno());
                break;
            }
            it->is_element_changed = false;
            it->last_publish_time = time_val;
        }
    }

    publish_list_mutex_.unlock();
}

long long TpComm::computeTimeElapsed(struct timeval& current_time_val, struct timeval& last_time_val)
{
    long long delta_tv_sec = current_time_val.tv_sec - last_time_val.tv_sec;
    long long delta_tv_usec = current_time_val.tv_usec - last_time_val.tv_usec;

    return (delta_tv_sec * 1000 + delta_tv_usec / 1000);
}

long long TpComm::computeTimeForTp(struct timeval& current_time_val)
{
    return current_time_val.tv_sec * 1000000 + current_time_val.tv_usec;
}

bool TpComm::checkPublishCondition(long long time_elapsed, bool is_element_changed, int interval_min, int interval_max)
{
    if(is_element_changed)
    {
        if(time_elapsed >= (long long)interval_min)
        {
            return true;
        }
        else
        {
            return false;
        }
    }
    else if(time_elapsed >= (long long)interval_max)
    {
        return true;
    }
    else
    {
        return false;
    }
}

bool TpComm::decodeRequestPackage(const pb_field_t fields[], void* request_data_ptr, int recv_bytes)
{
    pb_istream_t stream = pb_istream_from_buffer(recv_buffer_ptr_ + HASH_BYTE_SIZE, recv_bytes - HASH_BYTE_SIZE);
    return pb_decode(&stream, fields, (void*)request_data_ptr);
}

bool TpComm::checkAuthority(Comm_Authority request_authority, Comm_Authority controller_authority)
{
    return (request_authority >= controller_authority ? true:false); 
}

void TpComm::initResponsePackage(void* request_data_ptr, void* response_data_ptr, int package_left)
{
    // it is tricky here that all request & response have the same package header and property, no matter what data they have
    ((ResponseMessageType_Int32*)response_data_ptr)->header.time_stamp = ((RequestMessageType_Int32*)request_data_ptr)->header.time_stamp;
    ((ResponseMessageType_Int32*)response_data_ptr)->header.package_left = package_left;
    ((ResponseMessageType_Int32*)response_data_ptr)->header.error_code = SUCCESS;
    ((ResponseMessageType_Int32*)response_data_ptr)->property.authority = ((RequestMessageType_Int32*)request_data_ptr)->property.authority;
}

void TpComm::initCommFailedResponsePackage(void* request_data_ptr, void* response_data_ptr)
{
    // it is tricky here that all request & response have the same package header and property, no matter what data they have
    ((ResponseMessageType_Int32*)response_data_ptr)->header.time_stamp = ((RequestMessageType_Int32*)request_data_ptr)->header.time_stamp;
    ((ResponseMessageType_Int32*)response_data_ptr)->header.package_left = -1;
    ((ResponseMessageType_Int32*)response_data_ptr)->header.error_code = TP_COMM_AUTHORITY_CHECK_FAILED;
    ((ResponseMessageType_Int32*)response_data_ptr)->property.authority = ((RequestMessageType_Int32*)request_data_ptr)->property.authority;
}

void TpComm::initDecodeFailedResponsePackage(void* request_data_ptr, void* response_data_ptr)
{
    // it is tricky here that all request & response have the same package header and property, no matter what data they have
    ((ResponseMessageType_Int32*)response_data_ptr)->header.time_stamp = ((RequestMessageType_Int32*)request_data_ptr)->header.time_stamp;
    ((ResponseMessageType_Int32*)response_data_ptr)->header.package_left = -1;
    ((ResponseMessageType_Int32*)response_data_ptr)->header.error_code = TP_COMM_DECODE_FAILED;
    ((ResponseMessageType_Int32*)response_data_ptr)->property.authority = ((RequestMessageType_Int32*)request_data_ptr)->property.authority;
}

void TpComm::handleRequestPackage(unsigned int hash, void* request_data_ptr, void* response_data_ptr, 
                                    int recv_bytes, const pb_field_t fields[], int package_left)
{
    if(!decodeRequestPackage(fields, (void*)request_data_ptr, recv_bytes))
    {
        ErrorMonitor::instance()->add(TP_COMM_DECODE_FAILED);
        FST_ERROR("handleRequestPackage:  decode data failed");
        initDecodeFailedResponsePackage(request_data_ptr, response_data_ptr);
        pushTaskToRequestList(hash, request_data_ptr, response_data_ptr);
        return;
    }

    Comm_Authority controller_authority = getRpcTableElementAuthorityByHash(hash);

    if(!checkAuthority(((RequestMessageType_Int32*)request_data_ptr)->property.authority, controller_authority))
    {
        ErrorMonitor::instance()->add(TP_COMM_AUTHORITY_CHECK_FAILED);    
        FST_ERROR("Operation is not authorized");
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

bool TpComm::encodeResponsePackage(unsigned int hash, const pb_field_t fields[], void* response_data_ptr, int& send_buffer_size)
{
    send_buffer_size = 0;
    *((unsigned int*)send_buffer_ptr_) = hash;
    pb_ostream_t stream = pb_ostream_from_buffer(send_buffer_ptr_ + HASH_BYTE_SIZE, send_buffer_size_ - HASH_BYTE_SIZE);

    if(!pb_encode(&stream, fields, response_data_ptr))
    {
        ErrorMonitor::instance()->add(TP_COMM_ENCODE_FAILED);
        FST_ERROR("Encode data failed");

        send_buffer_size = 0;
        return false;
    }

    send_buffer_size = stream.bytes_written + HASH_BYTE_SIZE;
    return true;
}

bool TpComm::encodePublishElement(Comm_PublishElement_data_t& element, const pb_field_t fields[], void* element_data_ptr)
{
    pb_ostream_t element_stream = pb_ostream_from_buffer(element.bytes, sizeof(element.bytes));
    if(!pb_encode(&element_stream, fields, element_data_ptr))
    {
        ErrorMonitor::instance()->add(TP_COMM_ENCODE_FAILED);
        FST_ERROR("encodePublishElement: encode element data failed");
        element.size = 0;
        return false;
    }
    element.size = element_stream.bytes_written;
    return true;
}

bool TpComm::encodePublishPackage(Comm_Publish& package, unsigned int hash, struct timeval& current_time_val, int& send_buffer_size)
{
    *((unsigned int*)send_buffer_ptr_) = hash;
    package.time_stamp = computeTimeForTp(current_time_val);
    pb_ostream_t stream = pb_ostream_from_buffer(send_buffer_ptr_ + HASH_BYTE_SIZE, send_buffer_size_ - HASH_BYTE_SIZE);
    if(!pb_encode(&stream, Comm_Publish_fields, &package))
    {
        FST_ERROR("encodePublishPackage: encode data failed");
        return false;
    }
    send_buffer_size = stream.bytes_written + HASH_BYTE_SIZE;
    return true;
}

ResponseMessageType_Uint64_PublishTable TpComm::getResponseSucceedPublishTable()
{
    ResponseMessageType_Uint64_PublishTable response_data_ptr;
    response_data_ptr.header.time_stamp = 11;
    response_data_ptr.header.package_left = 12;
    response_data_ptr.header.error_code = SUCCESS;
    response_data_ptr.property.authority = Comm_Authority_TP;

    std::vector<PublishService>::iterator iter;
    int element_index = 0;

    for (iter = publish_element_table_.begin(); iter != publish_element_table_.end(); iter++)
    {
       iter->element_path.copy(response_data_ptr.data.element[element_index].path, iter->element_path.length(), 0);
       iter->element_type.copy(response_data_ptr.data.element[element_index].message_type, iter->element_type.length(), 0);
       response_data_ptr.data.element[element_index].hash = iter->hash;
       element_index++;
    }

    response_data_ptr.data.element_count = element_index;
    response_data_ptr.error_code.data = SUCCESS;

    return response_data_ptr;
}

std::vector<TpPublishElement> TpComm::eraseTaskFromPublishList(unsigned int &topic_hash)
{
    std::vector<TpPublish>::iterator it;  //publish_list_;
    std::vector<TpPublishElement> publish_element;
    publish_list_mutex_.lock();
    for(it = publish_list_.begin(); it != publish_list_.end();)
    {
        if(topic_hash == it->hash)
        {
            publish_element.assign(it->element_list_.begin(), it->element_list_.end());
            it = publish_list_.erase(it);
        }
        else 
            ++it;
    }
    publish_list_mutex_.unlock();

    return publish_element;
}


void tpCommRoutineThreadFunc(void* arg)
{
    TpComm* tp_comm = static_cast<TpComm*>(arg);
    while(tp_comm->isRunning())
    {
        tp_comm->tpCommThreadFunc();
    }
}

void TpComm::lockIoPublishMutex()
{
    io_publish_list_mutex_.lock();
}

void TpComm::unlockIoPublishMutex()
{
    io_publish_list_mutex_.unlock();
}

void TpComm::lockRegPublishMutex()
{
    reg_publish_list_mutex_.lock();
}

void TpComm::unlockRegPublishMutex()
{
    reg_publish_list_mutex_.unlock();
}

void TpComm::pushTaskToRegPublishList(TpPublish& package)
{
    reg_publish_list_mutex_.lock();
    reg_publish_list_.push_back(package);
    reg_publish_list_mutex_.unlock();
}

void TpComm::pushTaskToIoPublishList(TpPublish& package)
{
    io_publish_list_mutex_.lock();
    io_publish_list_.push_back(package);
    io_publish_list_mutex_.unlock();
}

void  TpComm::handleRegPublishList()
{
    std::vector<TpPublish>::iterator it;
    struct timeval time_val;
    long long time_elapsed;
 
    reg_publish_list_mutex_.lock();
    gettimeofday(&time_val, NULL);
    for(it = reg_publish_list_.begin(); it != reg_publish_list_.end(); ++it)
    {
        time_elapsed = computeTimeElapsed(time_val, it->last_publish_time);
        if(checkPublishCondition(time_elapsed, it->is_element_changed, it->interval_min, it->interval_max))
        {
            // encode TpPublishElement
            for(int i = 0; i < it->package.element_count; ++i)
            {
                int reg_type = (MessageType_RegType)(it->element_list_[i].hash >> 16);
                switch(reg_type)
                {
                    case MessageType_RegType_PR: handlePublishElementRegPr(it->package, i, it->element_list_[i]); break;
                    case MessageType_RegType_HR: handlePublishElementRegHr(it->package, i, it->element_list_[i]); break;
                    case MessageType_RegType_SR: handlePublishElementRegSr(it->package, i, it->element_list_[i]); break;
                    case MessageType_RegType_MR: handlePublishElementRegMr(it->package, i, it->element_list_[i]); break;
                    case MessageType_RegType_R:  handlePublishElementRegR(it->package, i, it->element_list_[i]); break;
                    default: ;
                }
            }

            // encode TpPublish
            int send_buffer_size;
            if(!encodePublishPackage(it->package, it->hash, time_val, send_buffer_size))
            {
                FST_ERROR("handleRegPublishList: encode data failed");
                break;
            }

            int send_bytes = nn_send(publish_socket_, send_buffer_ptr_, send_buffer_size, 0); // block send
            if(send_bytes == -1)
            {
                FST_INFO("handleRegPublishList: send publish failed, error = %d", nn_errno());
                break;
            }

            it->is_element_changed = false;
            it->last_publish_time = time_val;
        }
    }

    reg_publish_list_mutex_.unlock();
}

std::vector<TpPublishElement>  TpComm::eraseTaskFromRegPublishList(unsigned int &topic_hash)
{
    std::vector<TpPublish>::iterator it;  //reg_publish_list_;
    std::vector<TpPublishElement> reg_publish_element;
    reg_publish_list_mutex_.lock();
    for(it = reg_publish_list_.begin(); it != reg_publish_list_.end();)
    {
        if(topic_hash == it->hash)
        {
            reg_publish_element.assign(it->element_list_.begin(), it->element_list_.end());
            it = reg_publish_list_.erase(it);
        }
        else 
            ++it;
    }
    reg_publish_list_mutex_.unlock();

    return reg_publish_element;
}

void  TpComm::handleIoPublishList()
{
    std::vector<TpPublish>::iterator it;
    struct timeval time_val;
    long long time_elapsed;
 
    io_publish_list_mutex_.lock();
    gettimeofday(&time_val, NULL);
    for(it = io_publish_list_.begin(); it != io_publish_list_.end(); ++it)
    {
        time_elapsed = computeTimeElapsed(time_val, it->last_publish_time);
        if(checkPublishCondition(time_elapsed, it->is_element_changed, it->interval_min, it->interval_max))
        {
            // encode TpPublishElement
            for(int i = 0; i < it->package.element_count; ++i)
            {
                int io_type = (it->element_list_[i].hash >> 16) & 0x000000FF;

                if (io_type == MessageType_IoType_DI || io_type == MessageType_IoType_DO
                || io_type == MessageType_IoType_RI || io_type == MessageType_IoType_RO
                || io_type == MessageType_IoType_UI || io_type == MessageType_IoType_UO)
                {
                    handlePublishElementIo(it->package, i, it->element_list_[i]); continue;
                }
            }

            // encode TpPublish
            int send_buffer_size;
            if(!encodePublishPackage(it->package, it->hash, time_val, send_buffer_size))
            {
                FST_ERROR("handleIoPublishList: encode data failed");
                break;
            }

            int send_bytes = nn_send(publish_socket_, send_buffer_ptr_, send_buffer_size, 0); // block send
            if(send_bytes == -1)
            {
                FST_INFO("handleIoPublishList: send publish failed, error = %d", nn_errno());
                break;
            }

            it->is_element_changed = false;
            it->last_publish_time = time_val;
        }
    }

    io_publish_list_mutex_.unlock();
}

std::vector<TpPublishElement>  TpComm::eraseTaskFromIoPublishList(unsigned int &topic_hash)
{
    std::vector<TpPublishElement> io_publish_element;
    std::vector<TpPublish>::iterator it;  //io_publish_list_;
    io_publish_list_mutex_.lock();
    for(it = io_publish_list_.begin(); it != io_publish_list_.end();)
    {
        if(topic_hash == it->hash)
        {
            io_publish_element.assign(it->element_list_.begin(), it->element_list_.end());
            it = io_publish_list_.erase(it);
        }
        else 
            ++it;
    }
    io_publish_list_mutex_.unlock();

    return io_publish_element;
}


bool TpComm::isTopicExisted(unsigned int topic_hash)
{
    bool is_exist = false;    
    std::vector<TpPublish>::iterator itr;

    publish_list_mutex_.lock();

    for (itr = publish_list_.begin(); itr != publish_list_.end(); ++itr)
        if (itr->hash == topic_hash)
            is_exist = true;

    publish_list_mutex_.unlock();
    return is_exist;
}

bool TpComm::isRegTopicExisted(unsigned int topic_hash)
{
    bool is_exist = false;    
    std::vector<TpPublish>::iterator itr;

    reg_publish_list_mutex_.lock();

    for (itr = reg_publish_list_.begin(); itr != reg_publish_list_.end(); ++itr)
        if (itr->hash == topic_hash)
            is_exist = true;

    reg_publish_list_mutex_.unlock();
    return is_exist;
}

bool TpComm::isIoTopicExisted(unsigned int topic_hash)
{
    bool is_exist = false;
    std::vector<TpPublish>::iterator itr;

    io_publish_list_mutex_.lock();

    for (itr = io_publish_list_.begin(); itr != io_publish_list_.end(); ++itr)
        if (itr->hash == topic_hash)
            is_exist = true;

    io_publish_list_mutex_.unlock();
    return is_exist;
}


void TpComm::recordLog(ErrorCode log_code, ErrorCode error_code, std::string rpc_path)
{
    std::string log_str("run ");
    log_str += rpc_path;
    if(error_code == SUCCESS)
    {
        log_str += " success";
        ServerAlarmApi::GetInstance()->sendOneAlarm(log_code, log_str);
    }
    else
    {
        log_str += " failed";
        ServerAlarmApi::GetInstance()->sendOneAlarm(error_code, log_str);
    }
}
