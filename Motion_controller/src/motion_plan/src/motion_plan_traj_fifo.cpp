/*************************************************************************
	> File Name: motion_plan_traj_fifo.cpp
	> Author: 
	> Mail: 
	> Created Time: 2018年05月15日 星期二 10时41分30秒
 ************************************************************************/

#include <motion_plan_reuse.h>
#include <motion_plan_traj_fifo.h>


namespace fst_controller
{

TrajectoryFifo::TrajectoryFifo(void)
{
    traj_head_ = 0;
    traj_tail_ = 0;
    traj_duration_ = 0;
}

TrajectoryFifo::~TrajectoryFifo(void)
{}

void TrajectoryFifo::clear(void)
{
    traj_head_ = 0;
    traj_tail_ = 0;
    traj_duration_ = 0;
}

bool TrajectoryFifo::empty(void)
{
    return traj_head_ == traj_tail_;
}

bool TrajectoryFifo::full(void)
{
    size_t head = traj_head_;
    size_t tail = (traj_tail_ + 1) % TRAJECTORY_FIFO_CAPACITY;
    return tail == head;
}

size_t TrajectoryFifo::size(void)
{
    size_t head = traj_head_;
    size_t tail = traj_tail_;
    return tail >= head ? tail - head : tail + TRAJECTORY_FIFO_CAPACITY - head;
}

MotionTime TrajectoryFifo::duration(void)
{
    return traj_duration_;
}

bool TrajectoryFifo::push(const TrajSegment &segment)
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

bool TrajectoryFifo::fetch(TrajSegment &segment)
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

const TrajSegment& TrajectoryFifo::front(void)
{
    return traj_fifo_[traj_head_];
}

const TrajSegment& TrajectoryFifo::back(void)
{
    size_t back = traj_tail_ > 0 ? traj_tail_ - 1 : TRAJECTORY_FIFO_CAPACITY - 1;
    return traj_fifo_[back];
}

void TrajectoryFifo::updateDuration(void)
{
    MotionTime tm = 0;

    for (size_t i = traj_head_; i != traj_tail_; i = (i + 1) % TRAJECTORY_FIFO_CAPACITY)
    {
        tm += traj_fifo_[i].duration;
    }
    
    traj_duration_ = tm;
}


}


