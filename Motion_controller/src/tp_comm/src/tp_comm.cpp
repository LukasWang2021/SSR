#include <unistd.h>
#include <string>
#include <iostream>
#include <nanomsg/nn.h>
#include <nanomsg/ws.h>
#include <nanomsg/pubsub.h>
#include <nanomsg/reqrep.h>
#include <nanomsg/pipeline.h>
#include <pb_decode.h>
#include <pb_encode.h>
#include "common_error_code.h"
#include "tp_comm.h"

using namespace base_space;
using namespace user_space;
using namespace log_space;
using namespace std;
extern uint32_t* g_isr_ptr_;

TpComm::TpComm():
    param_ptr_(NULL),
    is_running_(false), 
    cycle_time_(10), 
    recv_buffer_size_(65536), 
    send_buffer_size_(65536),
    req_resp_socket_(-1), 
    publish_socket_(-1),
    req_resp_endpoint_id_(-1), 
    publish_endpoint_id_(-1),
    send_event_socket_(-1),
    send_event_endpoint_id_(-1)
{
    string ip = local_ip_.get();
    req_resp_ip_ = "ws://" + ip + ":5600";
    publish_ip_ = "ws://" + ip + ":5601";
	send_event_ip_ = "ws://" + ip + ":5602";
    is_received_ = false;

    param_ptr_ = new TpCommManagerConfig();
}

TpComm::~TpComm()
{
    this->close();

    if(param_ptr_ != NULL)
    {
        delete param_ptr_;
        param_ptr_ = NULL;
    };
    if(recv_buffer_ptr_ != NULL)
    {
        delete[] recv_buffer_ptr_;
        recv_buffer_ptr_ = NULL;
    };
    if(send_buffer_ptr_ != NULL)
    {
        delete[] send_buffer_ptr_;
        send_buffer_ptr_ = NULL;
    };
}

bool TpComm::initComponentParams()
{
    if(!param_ptr_->loadParam()) return false;

    recv_buffer_size_ = param_ptr_->recv_buffer_size_;
    send_buffer_size_ = param_ptr_->send_buffer_size_;
    cycle_time_ = param_ptr_->cycle_time_;

    return true;
}

unsigned long long int TpComm::init()
{
    if (!initComponentParams()) 
    {
        printf("Failed to load TpCommManager component parameters\n");
        return TP_COMM_LOAD_PARAM_FAILED;
    }

    req_resp_socket_ = nn_socket(AF_SP, NN_REP);
    if(req_resp_socket_ == -1)
    {
        printf("Failed to init rpc socket\n");
        return TP_COMM_INIT_OBJECT_FAILED;
    }

    req_resp_endpoint_id_ = nn_bind(req_resp_socket_, req_resp_ip_.c_str());
    if(req_resp_endpoint_id_ == -1)
    {
        printf("Failed to bind rpc socket\n");
        return TP_COMM_INIT_OBJECT_FAILED;
    }

    publish_socket_ = nn_socket(AF_SP, NN_PUB);
    if(publish_socket_ == -1)
    {
        printf("Failed to init pub socket\n");
        return TP_COMM_INIT_OBJECT_FAILED;
    }

    publish_endpoint_id_ = nn_bind(publish_socket_, publish_ip_.c_str());
    if(publish_endpoint_id_ == -1)
    {
        printf("Failed to bind pub socket\n");
        return TP_COMM_INIT_OBJECT_FAILED;
    }

	//event
	send_event_socket_ = nn_socket(AF_SP, NN_PUSH);
	if (send_event_socket_ == -1)
	{
		printf("Failed to init send event socket\n");
        return TP_COMM_INIT_OBJECT_FAILED;
	}
	send_event_endpoint_id_ = nn_bind(send_event_socket_, send_event_ip_.c_str());
	if (send_event_endpoint_id_ == -1)
	{
		printf("Failed to bind send event socket\n");
        return TP_COMM_INIT_OBJECT_FAILED;
	}
	poll_send_event_fd_.fd = send_event_socket_;
	poll_send_event_fd_.events = NN_POLLOUT;


    // it is critical to set poll fd here to make the model work correctly
	poll_fd_.fd = req_resp_socket_;
	poll_fd_.events = NN_POLLIN | NN_POLLOUT;

    recv_buffer_ptr_ = new uint8_t[recv_buffer_size_]();
    send_buffer_ptr_ = new uint8_t[send_buffer_size_]();

    initRpcTable();
    initRpcQuickSearchTable();
    initPublishElementTable();
    initPublishElementQuickSearchTable();

    ErrorCode err = open();
	if (err !=  SUCCESS)
	{
		return err;
	}

    return SUCCESS;
}

unsigned long long int TpComm::open()
{
    is_running_ = true;

    if(!thread_ptr_.run(&tpCommRoutineThreadFunc, this, 50))
    {
        printf("Failed to open tpcomm\n");
        return TP_COMM_CREATE_ROUTINE_THREAD_FAILED;
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

	if(send_event_socket_ != -1 && send_event_endpoint_id_ != -1)
    {
        nn_shutdown(send_event_socket_, send_event_endpoint_id_);
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

uint64_t TpComm::getResponseSucceed(void* response_data_ptr)
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
    task.package.time_stamp = 0;
    task.package.element_count = 0;
    task.interval_min = interval_min;
    task.interval_max = interval_max;
    task.is_element_changed = true;
    task.last_publish_time.tv_sec = 0;
    task.last_publish_time.tv_usec = 0;
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
	handleSendEventList();
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
            ErrorQueue::instance().push(TP_COMM_RECEIVE_FAILED);
            LogProducer::error("comm", "Failed to receive request");
            return;
        }
        is_received_ = true;
    }
    else
    {
        return;
    }

    if(request_list_.size() > static_cast<unsigned int>(param_ptr_->rpc_list_max_size_))
    {
        ErrorQueue::instance().push(TP_COMM_RPC_OVERLOAD);
        LogProducer::error("comm", "Too much rpc to handle");
        return;
    }

    unsigned int hash = *((unsigned int*)recv_buffer_ptr_);
    LogProducer::info("rpc", "---handleRequest: %x", hash);

    HandleRequestFuncPtr func_ptr = getRequestHandlerByHash(hash);
    if(func_ptr != NULL)
    {
        (this->*func_ptr)(recv_bytes);
    }
    else
    {
        handleRequestNonexistentHash(hash, recv_bytes);
        ErrorQueue::instance().push(TP_COMM_INVALID_REQUEST);
        LogProducer::warn("rpc", "Request invalid");
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
        //LogProducer::debug("rpc", "---handleResponse: %x", it->hash);
        if(send_bytes == -1)
        {
            ErrorQueue::instance().push(TP_COMM_SEND_FAILED);
            LogProducer::error("comm", "Send response failed, nn_error = %s", nn_strerror(nn_errno()));
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
            for(unsigned int i = 0; i < it->package.element_count; ++i)
            {
                func_ptr = getPublishElementHandlerByHash(it->package.element[i].hash);
                if(func_ptr != NULL)
                {
                    (this->*func_ptr)(it->package, i, it->element_list_[i]);
                }
            }

            // encode TpPublish
            int send_buffer_size = 0;
            if(!encodePublishPackage(it->package, it->hash, time_val, send_buffer_size))
            {
                LogProducer::error("comm", "Encode data failed");
                ErrorQueue::instance().push(TP_COMM_ENCODE_FAILED);
                break;
            }

            int send_bytes = nn_send(publish_socket_, send_buffer_ptr_, send_buffer_size, 0); // block send
            if(send_bytes == -1)
            {
                ErrorQueue::instance().push(TP_COMM_SEND_FAILED);
                LogProducer::error("comm", "Send publish failed, error = %d", nn_errno());
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
    ((ResponseMessageType_Int32*)response_data_ptr)->header.time_stamp = *g_isr_ptr_;
    ((ResponseMessageType_Int32*)response_data_ptr)->header.package_left = package_left;
    ((ResponseMessageType_Int32*)response_data_ptr)->header.error_code = SUCCESS;
    ((ResponseMessageType_Int32*)response_data_ptr)->property.authority = ((RequestMessageType_Int32*)request_data_ptr)->property.authority;
}

void TpComm::initCommFailedResponsePackage(void* request_data_ptr, void* response_data_ptr)
{
    // it is tricky here that all request & response have the same package header and property, no matter what data they have
    ((ResponseMessageType_Int32*)response_data_ptr)->header.time_stamp =  *g_isr_ptr_;
    ((ResponseMessageType_Int32*)response_data_ptr)->header.package_left = -1;
    ((ResponseMessageType_Int32*)response_data_ptr)->header.error_code = TP_COMM_AUTHORITY_CHECK_FAILED;
    ((ResponseMessageType_Int32*)response_data_ptr)->property.authority = ((RequestMessageType_Int32*)request_data_ptr)->property.authority;
}

void TpComm::initDecodeFailedResponsePackage(void* request_data_ptr, void* response_data_ptr)
{
    // it is tricky here that all request & response have the same package header and property, no matter what data they have
    ((ResponseMessageType_Int32*)response_data_ptr)->header.time_stamp =  *g_isr_ptr_;
    ((ResponseMessageType_Int32*)response_data_ptr)->header.package_left = -1;
    ((ResponseMessageType_Int32*)response_data_ptr)->header.error_code = TP_COMM_DECODE_FAILED;
    ((ResponseMessageType_Int32*)response_data_ptr)->property.authority = ((RequestMessageType_Int32*)request_data_ptr)->property.authority;
}

void TpComm::handleRequestPackage(unsigned int hash, void* request_data_ptr, void* response_data_ptr, 
                                    int recv_bytes, const pb_field_t fields[], int package_left)
{
    if(!decodeRequestPackage(fields, (void*)request_data_ptr, recv_bytes))
    {
        ErrorQueue::instance().push(TP_COMM_DECODE_FAILED);
        LogProducer::error("comm", "handleRequestPackage:  decode data failed");
        initDecodeFailedResponsePackage(request_data_ptr, response_data_ptr);
        pushTaskToRequestList(hash, request_data_ptr, response_data_ptr);
        return;
    }

    Comm_Authority controller_authority = getRpcTableElementAuthorityByHash(hash);

    if(!checkAuthority(((RequestMessageType_Int32*)request_data_ptr)->property.authority, controller_authority))
    {
        ErrorQueue::instance().push(TP_COMM_AUTHORITY_CHECK_FAILED);    
        printf("handleRequestPackage: Operation is not authorized\n");
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
        ErrorQueue::instance().push(TP_COMM_ENCODE_FAILED);
        LogProducer::error("comm", "encodeResponsePackage: encode data failed");

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
        ErrorQueue::instance().push(TP_COMM_ENCODE_FAILED);
        LogProducer::error("publish", "encodePublishElement: encode element data failed");
        element.size = 0;
        return false;
    }
    element.size = element_stream.bytes_written;
    return true;
}

bool TpComm::encodePublishPackage(Comm_Publish& package, unsigned int hash, struct timeval& current_time_val, int& send_buffer_size)
{
    *((unsigned int*)send_buffer_ptr_) = hash;
    package.time_stamp =  *g_isr_ptr_; 
    pb_ostream_t stream = pb_ostream_from_buffer(send_buffer_ptr_ + HASH_BYTE_SIZE, send_buffer_size_ - HASH_BYTE_SIZE);
    if(!pb_encode(&stream, Comm_Publish_fields, &package))
    {
        LogProducer::error("publish", "encodePublishPackage: encode data failed");
        return false;
    }
    send_buffer_size = stream.bytes_written + HASH_BYTE_SIZE;
    return true;
}

ResponseMessageType_Uint64_PublishTable TpComm::getResponseSucceedPublishTable()
{
    ResponseMessageType_Uint64_PublishTable response_data_ptr;
    response_data_ptr.header.time_stamp =  *g_isr_ptr_;
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

void TpComm::sendEvent(TpEventMsg& event)
{
	send_event_list_mutex_.lock();
	if(send_event_list_.size() < static_cast<unsigned int>(param_ptr_->event_list_max_size_))
	{
		send_event_list_.push_back(event);
	}
	send_event_list_mutex_.unlock();
}

void TpComm::handleSendEventList(void)
{
	if(nn_poll(&poll_send_event_fd_, 1, 0) <= 0)
	{
		return;
	}
	std::vector<TpEventMsg>::iterator it;
	send_event_list_mutex_.lock();
	for(it = send_event_list_.begin(); it != send_event_list_.end(); ++it)
	{
		EventMessageType_Uint64 msg;
        msg.header.time_stamp = *g_isr_ptr_;
        msg.header.type = it->type;
        msg.header.id = it->type;
		msg.data.data = it->event_data;
		int send_buffer_size = 0;
		if(!encodeEventPackage(EventMessageType_Uint64_fields, (void*)&msg, send_buffer_size))
		{
			break;
		}
		
		int send_bytes = nn_send(send_event_socket_, send_buffer_ptr_, send_buffer_size, 0);
		LogProducer::info("event", "handleSendEventList: 0x%llX, time_stamp = %llu", it->event_data, msg.header.time_stamp);
		if(send_bytes == -1)
		{
			LogProducer::error("event", "handleSendEventList: send failed, %s\n", nn_strerror(errno));
			break;
		}
	}
	send_event_list_.clear();
	send_event_list_mutex_.unlock();
}

bool TpComm::encodeEventPackage(const pb_field_t fields[], void* event_data_ptr, int& send_buffer_size)
{
    pb_ostream_t stream = pb_ostream_from_buffer(send_buffer_ptr_, send_buffer_size_);
	if(!pb_encode(&stream, fields, event_data_ptr))
    {
        LogProducer::error("event", "encodeEventPackage: encode data failed\n");
		send_buffer_size = 0;
        return false;
    }
	send_buffer_size = stream.bytes_written;

    return true;
}


void* tpCommRoutineThreadFunc(void* arg)
{
    TpComm* tp_comm = static_cast<TpComm*>(arg);
    log_space::LogProducer log_manager;
    log_manager.init("comm", g_isr_ptr_);
    while(tp_comm->isRunning())
    {
        tp_comm->tpCommThreadFunc();
    }
    std::cout<<"comm exit"<<std::endl;
	return NULL;
}

