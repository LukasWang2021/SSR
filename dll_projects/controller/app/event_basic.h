#pragma once
#include "comm_def.h"
#include "nanomsg/nn.h"
#include "nanomsg/pipeline.h"
#include "nanomsg/ws.h"
#include "protocol/comm.pb.h"
#include "protoc.h"
#include "pb_decode.h"
#include "pb_encode.h"
#include <string>
#include <thread>
#include <list>
#include <mutex>

typedef struct
{
	unsigned long long int time_stamp;
	unsigned long long int data;
}EventInfo;

class EventBasic
{
public:
	EventBasic();
	~EventBasic();

	static EventBasic* getInstance();
	int32_t init(std::string server_ip);
	void exit();
	bool isOnExit();
	void handleRecvEvent();
	void popAll(EventInfo event[MAX_EVENT_NUMBER], int32_t* size);
	
private:
	static EventBasic* instance_;
	std::string server_path_;
	struct nn_pollfd poll_fd_;
	int socket_;
	uint8_t recv_buffer_[COMM_BUFFER_SIZE];
	std::thread* thread_ptr_;
	std::list<EventInfo> event_list_;
	std::mutex event_mutex_;
	bool on_exit_;
	
	bool recv(int& recv_size);
	bool decodeMessageType(pb_byte_t* data_buf, int &data_size, void *data, const pb_field_t fields[]);
};

void EventThread(void* arg);