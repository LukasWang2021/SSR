/*************************************************************************
	> File Name: motion_control_traj_fifo.cpp
	> Author: 
	> Mail: 
	> Created Time: 2018年05月15日 星期二 10时41分30秒
 ************************************************************************/

#include <iostream>
#include <fstream>
#include <motion_control_traj_fifo.h>


using namespace std;
using namespace basic_alg;

namespace fst_mc
{


TrajectoryFifo::TrajectoryFifo(void)
{
    memset(&trajectory_segment_, 0, sizeof(trajectory_segment_));
}

TrajectoryFifo::~TrajectoryFifo(void)
{}

ErrorCode TrajectoryFifo::initTrajectoryFifo(size_t capacity, size_t joint_num, fst_log::Logger *plog)
{
    log_ptr_ = plog;
    joint_num_ = joint_num;
    trajectory_segment_.time_from_start = 0;
    trajectory_segment_.time_from_block = 0;
    trajectory_segment_.duration = -99.99;
    return trajectory_fifo_.init(capacity) ? SUCCESS : MC_INTERNAL_FAULT;
}

ErrorCode TrajectoryFifo::pushTrajectorySegment(const TrajectorySegment &segment)
{
    return trajectory_fifo_.push(segment) ? SUCCESS : TRAJECTORY_FIFO_FULL;
}

ErrorCode TrajectoryFifo::pickTrajectoryPoint(MotionTime time, TrajectoryPoint &point)
{
    ErrorCode err = fetchSegmentByTime(time);
    
    if (err == SUCCESS)
    {
        samplePointFromSegment(time, point);
    }
    else if (err == TRAJECTORY_FIFO_EMPTY)
    {
        if (trajectory_segment_.duration > 0)
        {
            sampleEndingPointFromSegment(point);
            trajectory_segment_.duration = -99.99;
            err = SUCCESS;
        }
        else
        {
            err = TRAJECTORY_FIFO_EMPTY;
        }
    }

    return err;
}

ErrorCode TrajectoryFifo::fetchSegmentByTime(MotionTime time)
{
    if (trajectory_segment_.duration < 0 && !trajectory_fifo_.empty())
    {
        trajectory_fifo_.fetch(trajectory_segment_);
    }

    while (trajectory_segment_.time_from_start + trajectory_segment_.duration < time && !trajectory_fifo_.empty())
    {
        trajectory_fifo_.fetch(trajectory_segment_);
        //printf("fetch: time = %.4f, start-time = %.4f, duration = %.4f\n", time, trajectory_segment_.time_from_start, trajectory_segment_.duration);
    }

    if (trajectory_segment_.time_from_start < time + MINIMUM_E9 && trajectory_segment_.time_from_start + trajectory_segment_.duration > time - MINIMUM_E9)
    {
        return SUCCESS;
    }
    else if (trajectory_fifo_.empty())
    {
        return TRAJECTORY_FIFO_EMPTY;
    }
    else
    {
        FST_ERROR("Error by fetchSegmentByTime: fifo-size = %d, time = %.6f, %.6f, %.6f, %.6f", trajectory_fifo_.size(), time, trajectory_segment_.time_from_start, trajectory_segment_.duration, trajectory_segment_.time_from_block);
        return TRAJECTORY_SEGMENT_ERROR;
    }
}

void TrajectoryFifo::samplePointFromSegment(MotionTime time, TrajectoryPoint &point)
{
    double *data;
    double sample_time = time - trajectory_segment_.time_from_start + trajectory_segment_.time_from_block;
    MotionTime tm[6];
    tm[0] = 1;
    tm[1] = sample_time;
    tm[2] = sample_time * tm[1];
    tm[3] = sample_time * tm[2];
    tm[4] = sample_time * tm[3];
    tm[5] = sample_time * tm[4];

    for (size_t i = 0; i < joint_num_; i++)
    {
        data = trajectory_segment_.axis[i].data;
        point.angle[i] = data[0] + data[1] * tm[1] + data[2] * tm[2] + data[3] * tm[3] + data[4] * tm[4] + data[5] * tm[5];
        point.omega[i] = data[1] + data[2] * tm[1] * 2 + data[3] * tm[2] * 3 + data[4] * tm[3] * 4 + data[5] * tm[4] * 5;
        point.alpha[i] = data[2] * 2 + data[3] * tm[1] * 6 + data[4] * tm[2] * 12 + data[5] * tm[3] * 20;
    }

    //printf("mid: %.4f - %.6f,%.6f,%.6f,%.6f,%.6f,%.6f - %.6f,%.6f,%.6f,%.6f,%.6f,%.6f - %.6f,%.6f,%.6f,%.6f,%.6f,%.6f\n", time,
    //       point.angle[0], point.angle[1], point.angle[2], point.angle[3], point.angle[4], point.angle[5],
    //       point.omega[0], point.omega[1], point.omega[2], point.omega[3], point.omega[4], point.omega[5],
    //       point.alpha[0], point.alpha[1], point.alpha[2], point.alpha[3], point.alpha[4], point.alpha[5]);

    point.level = POINT_MIDDLE;
}

void TrajectoryFifo::sampleEndingPointFromSegment(TrajectoryPoint &point)
{
    double *data;
    double sample_time = trajectory_segment_.time_from_block + trajectory_segment_.duration;
    MotionTime tm[6];
    tm[0] = 1;
    tm[1] = sample_time;
    tm[2] = sample_time * tm[1];
    tm[3] = sample_time * tm[2];
    tm[4] = sample_time * tm[3];
    tm[5] = sample_time * tm[4];

    for (size_t i = 0; i < joint_num_; i++)
    {
        data = trajectory_segment_.axis[i].data;
        point.angle[i] = data[0] + data[1] * tm[1] + data[2] * tm[2] + data[3] * tm[3] + data[4] * tm[4] + data[5] * tm[5];
        point.omega[i] = data[1] + data[2] * tm[1] * 2 + data[3] * tm[2] * 3 + data[4] * tm[3] * 4 + data[5] * tm[4] * 5;
        point.alpha[i] = data[2] * 2 + data[3] * tm[1] * 6 + data[4] * tm[2] * 12 + data[5] * tm[3] * 20;
    }

    // printf("end: %.4f - %.6f,%.6f,%.6f,%.6f,%.6f,%.6f - %.6f,%.6f,%.6f,%.6f,%.6f,%.6f - %.6f,%.6f,%.6f,%.6f,%.6f,%.6f\n",
    //        trajectory_segment_.time_from_start + trajectory_segment_.duration,
    //        point.angle[0], point.angle[1], point.angle[2], point.angle[3], point.angle[4], point.angle[5],
    //        point.omega[0], point.omega[1], point.omega[2], point.omega[3], point.omega[4], point.omega[5],
    //        point.alpha[0], point.alpha[1], point.alpha[2], point.alpha[3], point.alpha[4], point.alpha[5]);

    point.level = POINT_ENDING;
}

bool TrajectoryFifo::empty(void) const
{
    return trajectory_fifo_.empty() && trajectory_segment_.duration < 0;
}

bool TrajectoryFifo::full(void) const
{
    return trajectory_fifo_.full();
}

void TrajectoryFifo::clear(void)
{
    trajectory_fifo_.clear();
    trajectory_segment_.time_from_start = 0;
    trajectory_segment_.time_from_block = 0;
    trajectory_segment_.duration = -99.99;
}

size_t TrajectoryFifo::size(void) const
{
    return trajectory_fifo_.size();
}


}


