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

namespace fst_mc
{


TrajectoryFifo::TrajectoryFifo(void)
{
    memset(&trajectory_segment_, 0, sizeof(trajectory_segment_));
}

TrajectoryFifo::~TrajectoryFifo(void)
{}

ErrorCode TrajectoryFifo::initTrajectoryFifo(size_t capacity, size_t joint_num)
{
    joint_num_ = joint_num;
    trajectory_segment_.traj_from_start = 0;
    trajectory_segment_.segment_from_traj = 0;
    trajectory_segment_.duration = -99.99;
    return trajectory_fifo_.init(capacity) ? SUCCESS : MOTION_INTERNAL_FAULT;
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
    auto segment_from_start = trajectory_segment_.traj_from_start + trajectory_segment_.segment_from_traj;

    while (segment_from_start + trajectory_segment_.duration < time && !trajectory_fifo_.empty())
    {
        trajectory_fifo_.fetch(trajectory_segment_);
        segment_from_start = trajectory_segment_.traj_from_start + trajectory_segment_.segment_from_traj;
        //printf("fetch: time = %.4f, start-time = %.4f, duration = %.4f\n", time, segment_from_start, trajectory_segment_.duration);
    }

    if (segment_from_start < time + MINIMUM_E9 && segment_from_start + trajectory_segment_.duration > time - MINIMUM_E9)
    {
        return SUCCESS;
    }
    else if (trajectory_fifo_.empty())
    {
        return TRAJECTORY_FIFO_EMPTY;
    }
    else
    {
        return TRAJECTORY_SEGMENT_ERROR;
    }
}

void TrajectoryFifo::samplePointFromSegment(MotionTime time, TrajectoryPoint &point)
{
    double *data;
    double sample_time = time - trajectory_segment_.traj_from_start;
    double tm[4]  = {1.0, sample_time, sample_time * sample_time, sample_time * sample_time * sample_time};

    for (size_t i = 0; i < joint_num_; i++)
    {
        data = trajectory_segment_.axis[i].data;
        point.angle[i] = data[0] + data[1] * tm[1] + data[2] * tm[2] + data[3] * tm[3];
        point.omega[i] = data[1] + data[2] * tm[1] * 2 + data[3] * tm[2] * 3;
        point.alpha[i] = data[2] * 2 + data[3] * tm[1] * 6;
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
    double sample_time = trajectory_segment_.segment_from_traj + trajectory_segment_.duration;
    double tm[4]  = {1.0, sample_time, sample_time * sample_time, sample_time * sample_time * sample_time};

    for (size_t i = 0; i < joint_num_; i++)
    {
        data = trajectory_segment_.axis[i].data;
        point.angle[i] = data[0] + data[1] * tm[1] + data[2] * tm[2] + data[3] * tm[3];
        point.omega[i] = data[1] + data[2] * tm[1] * 2 + data[3] * tm[2] * 3;
        point.alpha[i] = data[2] * 2 + data[3] * tm[1] * 6;
    }

    //printf("end: %.4f - %.6f,%.6f,%.6f,%.6f,%.6f,%.6f - %.6f,%.6f,%.6f,%.6f,%.6f,%.6f - %.6f,%.6f,%.6f,%.6f,%.6f,%.6f\n",
    //       trajectory_segment_.traj_from_start + trajectory_segment_.segment_from_traj + trajectory_segment_.duration,
    //       point.angle[0], point.angle[1], point.angle[2], point.angle[3], point.angle[4], point.angle[5],
    //       point.omega[0], point.omega[1], point.omega[2], point.omega[3], point.omega[4], point.omega[5],
    //       point.alpha[0], point.alpha[1], point.alpha[2], point.alpha[3], point.alpha[4], point.alpha[5]);

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
    trajectory_segment_.traj_from_start = 0;
    trajectory_segment_.segment_from_traj = 0;
    trajectory_segment_.duration = -99.99;
}

size_t TrajectoryFifo::size(void) const
{
    return trajectory_fifo_.size();
}


}


