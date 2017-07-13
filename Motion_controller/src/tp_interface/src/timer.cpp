#include "timer.h"
#include <boost/thread.hpp>


Timer::Timer(int mseconds):msecond_(mseconds)
{
	timeout_flag_ = false;
	count_ = 0;
	sem_ = new Semaphore(0);
	timer_ = new boost::asio::deadline_timer(io_, boost::posix_time::milliseconds(mseconds));  

	timer_->async_wait(boost::bind(&Timer::handler,this));  
}		
Timer::~Timer()
{
	delete timer_;
	delete sem_;
	io_.stop();
}

bool Timer::isTimeOut()
{
	bool ret = timeout_flag_;
	timeout_flag_ = false;
	
	return ret;
}

void Timer::start()
{
	io_.run();	
}

void Timer::stop()
{
	io_.stop();
}

bool Timer::isExpired(int id)
{
	try
	{
		if (timer_id_list_[id].expire_flag == true)
		{
			timer_id_list_[id].expire_flag = false;
			return true;
		}
	}
	catch(std::exception e)
	{
		return false;
	}
	return false;
}

void Timer::add(int id, int mseconds)
{
	TimeCntFlag time_cnt_flag;
	time_cnt_flag.count = mseconds / msecond_; //cnt every time
	time_cnt_flag.expire_flag = false;
	timer_id_list_.insert(map<int, TimeCntFlag>::value_type(id, time_cnt_flag));  
}

void Timer::wait()
{
	sem_->wait();
}

void Timer::handler()  
{  
	timeout_flag_ = true;
	count_ ++;
	setExpireFlag();
	timer_->expires_at(timer_->expires_at() + boost::posix_time::milliseconds(msecond_));
	timer_->async_wait(boost::bind(&Timer::handler,this));
}  	

void Timer::setExpireFlag()
{
	bool cnt_clear_flag = true;
	bool cnt_set_flag = false;
	map<int,TimeCntFlag>::iterator it;
    for (it=timer_id_list_.begin();it!=timer_id_list_.end();++it)
	{
		if ((count_ % it->second.count) == 0)
		{
			it->second.expire_flag = true;
			cnt_set_flag = true;
		}
		else
		{
			cnt_clear_flag = false;
		}
	}
	if(cnt_clear_flag == true)
	{
		count_ = 0;   //clear count
	}
	if(cnt_set_flag == true)
	{
		sem_->signal();
	}
}
