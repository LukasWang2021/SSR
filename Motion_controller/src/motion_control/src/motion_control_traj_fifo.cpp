/*************************************************************************
	> File Name: motion_control_traj_fifo.cpp
	> Author: 
	> Mail: 
	> Created Time: 2018年05月15日 星期二 10时41分30秒
 ************************************************************************/

#include <motion_control_traj_fifo.h>


namespace fst_mc
{

TrajectoryFifo::TrajectoryFifo(void)
{
    traj_head_ = 0;
    traj_tail_ = 0;
    traj_duration_ = 0;
    time_from_start_ = 0;
}

TrajectoryFifo::~TrajectoryFifo(void)
{}

void TrajectoryFifo::clear(void)
{
    traj_head_ = 0;
    traj_tail_ = 0;
    traj_duration_ = 0;
    time_from_start_ = 0;
}

bool TrajectoryFifo::empty(void) const
{
    return traj_head_ == traj_tail_;
}

bool TrajectoryFifo::full(void) const
{
    size_t head = traj_head_;
    size_t tail = (traj_tail_ + 1) % TRAJECTORY_FIFO_CAPACITY;
    return tail == head;
}

size_t TrajectoryFifo::size(void) const
{
    size_t head = traj_head_;
    size_t tail = traj_tail_;
    return tail >= head ? tail - head : tail + TRAJECTORY_FIFO_CAPACITY - head;
}

MotionTime TrajectoryFifo::duration(void) const
{
    return traj_duration_;
}

MotionTime  TrajectoryFifo::timeFromStart(void) const
{
    return time_from_start_;
}

bool TrajectoryFifo::push(const TrajectoryItem &segment)
{
    if ((traj_tail_ + 1) % TRAJECTORY_FIFO_CAPACITY != traj_head_)
    {
        traj_fifo_[traj_tail_] = segment;
        traj_tail_ = (traj_tail_ + 1) % TRAJECTORY_FIFO_CAPACITY;
        updateDuration();
        return true;
    }
    else
    {
        return false;
    }
}

bool TrajectoryFifo::fetch(TrajectoryItem &segment)
{
    if (traj_head_ != traj_tail_)
    {
        segment = traj_fifo_[traj_head_];
        traj_head_ = (traj_head_ + 1) % TRAJECTORY_FIFO_CAPACITY;
        updateDuration();
        return true;
    }
    else
    {
        return false;
    }
}

bool TrajectoryFifo::dropBack(void)
{
    if (traj_head_ != traj_tail_)
    {
        traj_tail_ = traj_tail_ > 0 ? traj_tail_ - 1 : TRAJECTORY_FIFO_CAPACITY - 1;
        updateDuration();
        return true;
    }
    else
    {
        return false;
    }
}

bool TrajectoryFifo::dropFront(void)
{
    if (traj_head_ != traj_tail_)
    {
        traj_head_ = traj_head_ + 1 < TRAJECTORY_FIFO_CAPACITY ? traj_head_ + 1 : 0;
        updateDuration();
        return true;
    }
    else
    {
        return false;
    }
}

const TrajectoryItem& TrajectoryFifo::front(void) const
{
    return traj_fifo_[traj_head_];
}

const TrajectoryItem& TrajectoryFifo::back(void) const
{
    size_t back = traj_tail_ > 0 ? traj_tail_ - 1 : TRAJECTORY_FIFO_CAPACITY - 1;
    return traj_fifo_[back];
}

void TrajectoryFifo::updateDuration(void)
{
    MotionTime tm = 0;
    MotionTime tm_from_start = 0;

    for (size_t i = traj_head_; i != traj_tail_; i = (i + 1) % TRAJECTORY_FIFO_CAPACITY)
    {
        tm += traj_fifo_[i].duration;
        tm_from_start = traj_fifo_[i].time_from_start;
    }
    
    traj_duration_ = tm;
    time_from_start_ = tm_from_start;
}


}


