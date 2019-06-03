/*************************************************************************
	> File Name: motion_control_traj_fifo.h
	> Author: 
	> Mail: 
	> Created Time: 2018年05月15日 星期二 10时39分59秒
 ************************************************************************/

#ifndef _MOTION_PLAN_TRAJ_FIFO_H
#define _MOTION_PLAN_TRAJ_FIFO_H

#include <error_code.h>
#include <common_log.h>
#include <lock_free_fifo.h>
#include <motion_control_datatype.h>

#define LOCK_FREE_FIFO_SIZE_MIN    2
#define LOCK_FREE_FIFO_SIZE_MAX    1024

namespace fst_mc
{

class TrajectoryFifo
{
  public:
    TrajectoryFifo(void);
    ~TrajectoryFifo(void);

    ErrorCode initTrajectoryFifo(size_t capacity, size_t joint_num, fst_log::Logger *plog);
    ErrorCode pushTrajectorySegment(const TrajectorySegment &segment);
    ErrorCode pickTrajectoryPoint(MotionTime time, TrajectoryPoint &point);

    void clear(void);
    bool empty(void) const;
    bool full(void) const;
    size_t  size(void) const;

  private:
    ErrorCode fetchSegmentByTime(MotionTime time);
    void samplePointFromSegment(MotionTime time, TrajectoryPoint &point);
    void sampleEndingPointFromSegment(TrajectoryPoint &point);

    size_t  joint_num_;
    fst_log::Logger     *log_ptr_;
    TrajectorySegment   trajectory_segment_;
    LockFreeFIFO<TrajectorySegment>   trajectory_fifo_;
};

}

#endif
