#pragma once
#include "comm_def.h"
#include "nanomsg/nn.h"
#include "nanomsg/pubsub.h"
#include "nanomsg/ws.h"
#include "protocol/comm.pb.h"
#include "protoc.h"
#include "pb_decode.h"
#include "pb_encode.h"
#include <string>
#include <thread>
#include <mutex>
#include <list>

typedef struct
{
	MessageType_AxisFeedbackList axis_feedback;
	MessageType_ServoFeedbackList servo_feedback;
	MessageType_Uint32List cpu_feedback;
	MessageType_Uint32List io1000_feedback;
	MessageType_Uint32List iosafety_feedback;
}TopicData;


class SubBasic
{
public:
	SubBasic();
	~SubBasic();

	static SubBasic* getInstance();
	int32_t init(std::string server_ip);
	void exit();
	bool isOnExit();
	void handleSubscribe();

	TopicData* getTopicDataPtr();
	void lockTopicData();
	void unlockTopicData();

	int32_t getAxisFeedBackByIsrCount(uint32_t isr_count,
										uint32_t state[AXIS_NUM],
										double position[AXIS_NUM],
										double velocity[AXIS_NUM],
										double torque[AXIS_NUM]);
	
private:
	static SubBasic* instance_;
	std::string server_path_;
	struct nn_pollfd poll_fd_;
	int socket_;
	uint8_t recv_buffer_[COMM_BUFFER_SIZE];
	std::thread* thread_ptr_;
	TopicData topic_data_;
	std::mutex topic_mutex_;
	std::list<MessageType_AxisFeedbackList> cache_axis_feedback_list_;
	std::mutex cache_axis_feedback_mutex_;
	bool on_exit_;
	Comm_Publish *comm_pub_;

	bool recv(int& recv_size);
	bool decodeReplyPackage(const pb_field_t fields[], void* reply_data_ptr, size_t recv_bytes);
	bool decodeMessageType(pb_byte_t* data_buf, pb_size_t &data_size, void *data, const pb_field_t fields[]);
	void pushCacheAxisFeedbackList();
};

void TopicThread(void* arg);