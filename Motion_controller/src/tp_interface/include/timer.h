#ifndef TP_INTERFACE_TIMER_H_
#define TP_INTERFACE_TIMER_H_
/**
 * @file tp_interface_timer.h
 * @brief 
 * @author WangWei
 * @version 1.0.0
 * @date 2016-11-02
 */
#include <map>
#include <boost/asio.hpp>
#include "lw_signal.h"

using std::map;

typedef struct _TimeCntFlag
{
	int		count;
	bool	expire_flag;
}TimeCntFlag;

class Timer
{	
  public:
	Timer(int mseconds);	
	~Timer();
	bool isTimeOut();
	void start();
	void stop();
	bool isExpired(int id);
	void add(int id, int mseconds);
	void wait();

  private:
	boost::asio::io_service		io_;  
    boost::asio::deadline_timer *timer_;
	unsigned int				count_;
	int							msecond_;	
	bool						timeout_flag_;
	map<int, TimeCntFlag>		timer_id_list_;
	Semaphore					*sem_;


	void handler();  
	void IORun();
	void setExpireFlag();
};





#endif // TP_INTERFACE_TIMER_H_
