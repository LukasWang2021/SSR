/*************************************************************************
	> File Name: motion_plan_traj_fifo.h
	> Author: 
	> Mail: 
	> Created Time: 2018年05月15日 星期二 10时39分59秒
 ************************************************************************/

#ifndef _MOTION_PLAN_TRAJ_FIFO_H
#define _MOTION_PLAN_TRAJ_FIFO_H

#include <fst_datatype.h>
#include <motion_plan_error_code.h>

#define     TRAJECTORY_FIFO_CAPACITY        256         // must be setted to 2^N

namespace fst_controller
{

class TrajectoryFifo
{
  public:
    TrajectoryFifo(void);
    ~TrajectoryFifo(void);

    void    clear(void);
    bool    empty(void);
    bool    full(void);
    size_t  size(void);

    bool    push(const TrajSegment &segment);
    bool    fetch(TrajSegment &segment);
    bool    dropBack(void);
    bool    dropFront(void);

    const TrajSegment& front(void);
    const TrajSegment& back(void);


    MotionTime duration(void);

  private:
    void updateDuration(void);


    TrajSegment traj_fifo_[TRAJECTORY_FIFO_CAPACITY];
    size_t      traj_head_;
    size_t      traj_tail_;
    MotionTime  traj_duration_;
};

}

#endif
