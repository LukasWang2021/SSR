/*************************************************************************
	> File Name: motion_control_traj_fifo.h
	> Author: 
	> Mail: 
	> Created Time: 2018年05月15日 星期二 10时39分59秒
 ************************************************************************/

#ifndef _MOTION_PLAN_TRAJ_FIFO_H
#define _MOTION_PLAN_TRAJ_FIFO_H

#include <motion_control_datatype.h>
#include <error_code.h>

#define     TRAJECTORY_FIFO_CAPACITY        256         // must be setted to 2^N

namespace fst_mc
{

class TrajectoryFifo
{
  public:
    TrajectoryFifo(void);
    ~TrajectoryFifo(void);

    bool    empty(void) const;
    bool    full(void) const;

    void    clear(void);
    bool    push(const TrajectoryItem &segment);
    bool    fetch(TrajectoryItem &segment);
    bool    dropBack(void);
    bool    dropFront(void);

    const TrajectoryItem& front(void) const;
    const TrajectoryItem& back(void) const;

    size_t      size(void) const;
    MotionTime  duration(void) const;
    MotionTime  timeFromStart(void) const;

  private:
    void updateDuration(void);

    size_t      traj_head_;
    size_t      traj_tail_;
    MotionTime  traj_duration_;
    MotionTime  time_from_start_;
    TrajectoryItem  traj_fifo_[TRAJECTORY_FIFO_CAPACITY];
};

}

#endif
