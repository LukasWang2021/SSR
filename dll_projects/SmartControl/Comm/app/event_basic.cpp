#include "event_basic.h"
#include "protocol/publish.pb.h"
#include "base.pb.h"

using namespace std;

EventBasic* EventBasic::instance_ = NULL;

EventBasic::EventBasic()
{
	on_exit_ = false;
}

EventBasic::~EventBasic()
{
	nn_close(socket_);
}

EventBasic* EventBasic::getInstance()
{
	if (instance_ == NULL)
	{
		instance_ = new EventBasic();
	}
	return instance_;
}

int32_t EventBasic::init(std::string server_ip)
{
	server_path_ = "ws://" + server_ip + ":5602";

	if ((socket_ = nn_socket(AF_SP, NN_PULL)) < 0) return -1;
	if (nn_connect(socket_, server_path_.c_str()) < 0) return -1;

	poll_fd_.fd = socket_;
	poll_fd_.events = NN_POLLIN;

	thread_ptr_ = new thread(&EventThread, this);
	if (thread_ptr_ == NULL)
	{
		return -1;
	}
	return 0;
}

void EventBasic::exit()
{
	on_exit_ = true;
	if (thread_ptr_ != NULL)
	{
		thread_ptr_->join();
		delete thread_ptr_;
		thread_ptr_ = NULL;
	}
}

bool EventBasic::isOnExit()
{
	return on_exit_;
}

void EventBasic::handleRecvEvent()
{
	if (nn_poll(&poll_fd_, 1, 0) <= 0)
	{
		return;
	}

	int recv_size;
	EventMessageType_Uint64 msg; msg.data.data = 0;
	
	if (!recv(recv_size)
		|| !decodeMessageType(recv_buffer_,recv_size, &msg, EventMessageType_Uint64_fields))
	{
		return;
	}

	EventInfo event;
	event.time_stamp = msg.header.time_stamp;
	event.data = msg.data.data;
	event_mutex_.lock();
	if (event_list_.size() >= MAX_EVENT_NUMBER)
	{
		event_list_.pop_front();
	}
	event_list_.push_back(event);
	event_mutex_.unlock();
}

void EventBasic::popAll(EventInfo event[MAX_EVENT_NUMBER], int32_t* size)
{
	std::list<EventInfo>::iterator it;
	int32_t count = 0;
	event_mutex_.lock();
	for (it = event_list_.begin(); it != event_list_.end(); ++it)
	{
		event[count] = *it;
		count++;
	}
	event_list_.clear();
	event_mutex_.unlock();
	*size = count;
}

bool EventBasic::recv(int& recv_size)
{
	if (poll_fd_.revents & NN_POLLIN)
	{
		recv_size = nn_recv(socket_, &recv_buffer_, COMM_BUFFER_SIZE, 0);
		if (recv_size < 0)
		{
			return false;
		}
		else
		{
			return true;
		}
	}
	return false;
}

bool EventBasic::decodeMessageType(pb_byte_t* data_buf, int &data_size, void *data, const pb_field_t fields[])
{
	pb_istream_t stream = pb_istream_from_buffer(data_buf, data_size);
	return (pb_decode(&stream, fields, data));
}

void EventThread(void* arg)
{
	EventBasic* object_ptr = static_cast<EventBasic*>(arg);
	while (!object_ptr->isOnExit())
	{
		object_ptr->handleRecvEvent();
#ifdef _WIN_PLAT
        Sleep(100);
#else
        usleep(100000);
#endif
		
	}
}

