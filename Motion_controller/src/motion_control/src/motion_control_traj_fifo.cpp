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
{}

TrajectoryFifo::~TrajectoryFifo(void)
{}

ErrorCode TrajectoryFifo::initTrajectoryFifo(size_t capacity)
{
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
    else
    {
        if (trajectory_segment_.duration > 0)
        {
            sampleEndingPointFromSegment(point);
            trajectory_segment_.duration = -99.99;
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
    while (trajectory_segment_.time_from_start + trajectory_segment_.duration < time && !trajectory_fifo_.empty())
    {
        trajectory_fifo_.fetch(trajectory_segment_);
    }

    if (trajectory_segment_.time_from_start < time + MINIMUM_E9 && trajectory_segment_.time_from_start + trajectory_segment_.duration > time - MINIMUM_E9)
    {
        return SUCCESS;
    }
    else
    {
        return TRAJECTORY_SEGMENT_ERROR;
    }
}

void TrajectoryFifo::samplePointFromSegment(MotionTime time, TrajectoryPoint &point)
{
    // TODO
    memset(&point.angle, 0, sizeof(point.angle));
    memset(&point.omega, 0, sizeof(point.omega));
    memset(&point.alpha, 0, sizeof(point.alpha));

    //FST_LOG ("%.4f - %.6f,%.6f,%.6f,%.6f,%.6f,%.6f - %.6f,%.6f,%.6f,%.6f,%.6f,%.6f - %.6f,%.6f,%.6f,%.6f,%.6f,%.6f", time,
    //         point.angle[0], point.angle[1], point.angle[2], point.angle[3], point.angle[4], point.angle[5],
    //         point.omega[0], point.omega[1], point.omega[2], point.omega[3], point.omega[4], point.omega[5],
    //         point.alpha[0], point.alpha[1], point.alpha[2], point.alpha[3], point.alpha[4], point.alpha[5]);

    point.level = POINT_MIDDLE;
}

void TrajectoryFifo::sampleEndingPointFromSegment(TrajectoryPoint &point)
{
    // TODO
    memset(&point.angle, 0, sizeof(point.angle));
    memset(&point.omega, 0, sizeof(point.omega));
    memset(&point.alpha, 0, sizeof(point.alpha));

    //MotionTime time = trajectory_segment_.time_from_start + trajectory_segment_.duration;
    //FST_LOG ("%.4f - %.6f,%.6f,%.6f,%.6f,%.6f,%.6f - %.6f,%.6f,%.6f,%.6f,%.6f,%.6f - %.6f,%.6f,%.6f,%.6f,%.6f,%.6f", time,
    //         point.angle[0], point.angle[1], point.angle[2], point.angle[3], point.angle[4], point.angle[5],
    //         point.omega[0], point.omega[1], point.omega[2], point.omega[3], point.omega[4], point.omega[5],
    //         point.alpha[0], point.alpha[1], point.alpha[2], point.alpha[3], point.alpha[4], point.alpha[5]);

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
    trajectory_segment_.duration = -MINIMUM_E3;
}

size_t TrajectoryFifo::size(void) const
{
    return trajectory_fifo_.size();
}


}


