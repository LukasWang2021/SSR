//#include "stdafx.h"
#include "sub_basic.h"
#include "protocol/publish.pb.h"
#include "base.pb.h"
#include <windows.h>


using namespace std;

SubBasic* SubBasic::instance_ = NULL;

SubBasic::SubBasic()
{
	memset(&topic_data_, 0, sizeof(TopicData));
	on_exit_ = false;
}

SubBasic::~SubBasic()
{
	nn_close(socket_);
	if (comm_pub_ != NULL)
	{
		delete comm_pub_;
		comm_pub_ = NULL;
	}
}

SubBasic* SubBasic::getInstance()
{
	if (instance_ == NULL)
	{
		instance_ = new SubBasic();
	}
	return instance_;
}

int32_t SubBasic::init(std::string server_ip)
{
	server_path_ = "ws://" + server_ip + ":5601";

	if ((socket_ = nn_socket(AF_SP, NN_SUB)) < 0) return -1;
	if (nn_connect(socket_, server_path_.c_str()) < 0) return -1;
	unsigned int topic_hash = TOPIC_HASH;
	if (nn_setsockopt(socket_, NN_SUB, NN_SUB_SUBSCRIBE, &topic_hash, HASH_BYTE_SIZE) < 0) return -1;
	poll_fd_.fd = socket_;
	poll_fd_.events = NN_POLLIN;

	comm_pub_ = new Comm_Publish;
	if (comm_pub_ == NULL)
	{
		return -1;
	}

	thread_ptr_ = new thread(&TopicThread, this);
	if (thread_ptr_ == NULL)
	{
		return -1;
	}
	return 0;
}

void SubBasic::exit()
{
	on_exit_ = true;
	if (thread_ptr_ != NULL)
	{
		thread_ptr_->join();
		delete thread_ptr_;
		thread_ptr_ = NULL;
	}
	nn_close(socket_);
}

bool SubBasic::isOnExit()
{
	return on_exit_;
}

bool SubBasic::recv(int& recv_size)
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

bool SubBasic::decodeReplyPackage(const pb_field_t fields[], void* reply_data_ptr, size_t recv_bytes)
{
	pb_istream_t stream = pb_istream_from_buffer(&recv_buffer_[0] + HASH_BYTE_SIZE, recv_bytes - HASH_BYTE_SIZE);
	return pb_decode(&stream, fields, reply_data_ptr);
}

bool SubBasic::decodeMessageType(pb_byte_t* data_buf, pb_size_t &data_size, void *data, const pb_field_t fields[])
{
	pb_istream_t stream = pb_istream_from_buffer(data_buf, data_size);
	return (pb_decode(&stream, fields, data));
}

void SubBasic::pushCacheAxisFeedbackList()
{
	if (cache_axis_feedback_list_.size() == 0)
	{
		cache_axis_feedback_list_.push_back(topic_data_.axis_feedback);
		return;
	}

	std::list<MessageType_AxisFeedbackList>::iterator it = cache_axis_feedback_list_.end();
	it--;
	if (topic_data_.axis_feedback.data[0].data1.data[0] == it->data[0].data1.data[0])
	{
		return;
	}

	if (cache_axis_feedback_list_.size() > 100)
	{
		cache_axis_feedback_list_.pop_front();
	}
	cache_axis_feedback_list_.push_back(topic_data_.axis_feedback);
}

void SubBasic::handleSubscribe()
{
	if (nn_poll(&poll_fd_, 1, 0) == -1)
	{
		return;
	}
	int recv_size = 0;

	if (!recv(recv_size))
	{
		return;
	}
	if(!decodeReplyPackage(Comm_Publish_fields, comm_pub_, recv_size))
	{
		return;
	}
	if (comm_pub_->element_count != TOPIC_ELEM_NUM)
	{
		return;
	}
	topic_mutex_.lock();
	decodeMessageType(comm_pub_->element[0].data.bytes, comm_pub_->element[0].data.size, (void*)&(topic_data_.axis_feedback), MessageType_AxisFeedbackList_fields);
	decodeMessageType(comm_pub_->element[1].data.bytes, comm_pub_->element[1].data.size, (void*)&(topic_data_.servo_feedback), MessageType_ServoFeedbackList_fields);
	decodeMessageType(comm_pub_->element[2].data.bytes, comm_pub_->element[2].data.size, (void*)&(topic_data_.cpu_feedback), MessageType_Uint32List_fields);
	decodeMessageType(comm_pub_->element[3].data.bytes, comm_pub_->element[3].data.size, (void*)&(topic_data_.io1000_feedback), MessageType_Uint32List_fields);
	decodeMessageType(comm_pub_->element[4].data.bytes, comm_pub_->element[4].data.size, (void*)&(topic_data_.iosafety_feedback), MessageType_Uint32List_fields);

	cache_axis_feedback_mutex_.lock();
	pushCacheAxisFeedbackList();
	cache_axis_feedback_mutex_.unlock();

	topic_mutex_.unlock();
}

TopicData* SubBasic::getTopicDataPtr()
{
	return &topic_data_;
}

void SubBasic::lockTopicData()
{
	topic_mutex_.lock();
}

void SubBasic::unlockTopicData()
{
	topic_mutex_.unlock();
}

int32_t SubBasic::getAxisFeedBackByIsrCount(uint32_t isr_count, 
											uint32_t state[AXIS_NUM],
											double position[AXIS_NUM],
											double velocity[AXIS_NUM],
											double torque[AXIS_NUM])
{
	size_t list_size;
	cache_axis_feedback_mutex_.lock();
	list_size = cache_axis_feedback_list_.size();
	if (list_size < 2)
	{
		cache_axis_feedback_mutex_.unlock();
		return 2;
	}

	std::list<MessageType_AxisFeedbackList>::iterator it, it_prev;
	double factor;
	it = it_prev = cache_axis_feedback_list_.begin();
	it++;
	for (; it != cache_axis_feedback_list_.end(); it++, it_prev++)
	{
		if (it_prev->data[0].data1.data[0] <= isr_count
			&& it->data[0].data1.data[0] >= isr_count)
		{
			factor = (isr_count - it_prev->data[0].data1.data[0]) / (it->data[0].data1.data[0] - it_prev->data[0].data1.data[0]);
			for (uint32_t i = 0; i < AXIS_NUM; ++i)
			{
				state[i] = it_prev->data[i].data1.data[1];
				position[i] = it_prev->data[i].data2.data[0] + (it->data[i].data2.data[0] - it_prev->data[i].data2.data[0])*factor;
				velocity[i] = it_prev->data[i].data2.data[1] + (it->data[i].data2.data[1] - it_prev->data[i].data2.data[1])*factor;
				torque[i] = it_prev->data[i].data2.data[2] + (it->data[i].data2.data[2] - it_prev->data[i].data2.data[2])*factor;
			}
			cache_axis_feedback_mutex_.unlock();
			return 0;
		}
	}
	cache_axis_feedback_mutex_.unlock();
	return 3;
}

void TopicThread(void* arg)
{
	SubBasic* object_ptr = static_cast<SubBasic*>(arg);
	while (!object_ptr->isOnExit())
	{
		object_ptr->handleSubscribe();
		Sleep(90);
	}
}

