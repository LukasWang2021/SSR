/*************************************************************************
	> File Name: motion_control_traj_fifo.cpp
	> Author: 
	> Mail: 
	> Created Time: 2018年05月15日 星期二 10时41分30秒
 ************************************************************************/

#include <iostream>
#include <fstream>
#include <motion_control_traj_fifo.h>

//#define OUTPUT_TRAJECTORY_SEGMENT

using namespace std;

namespace fst_mc
{


#ifdef OUTPUT_TRAJECTORY_SEGMENT
#define SEGMENT_SIZE   (100000)

TrajectoryItem  s_segment[SEGMENT_SIZE];
static size_t   s_seg_index = 0;
ofstream        s_sout("seg_out.csv");

#endif


TrajectoryFifo::TrajectoryFifo(void)
{
    traj_head_ = 0;
    traj_tail_ = 0;
    traj_duration_ = 0;
    time_from_start_ = 0;
}

TrajectoryFifo::~TrajectoryFifo(void)
{
#ifdef OUTPUT_TRAJECTORY_SEGMENT
    printf("正在将缓存中的轨迹段录入文件：seg_out.csv ... 请稍后\n");
    s_sout << "time-of-start,duration,"
           <<"j0.duration0,j0.duration1,j0.duration2,j0.duration3,j0.coeff00,j0.coeff01,j0.coeff02,j0.coeff03,j0.coeff10,j0.coeff11,j0.coeff12,j0.coeff13,j0.coeff20,j0.coeff21,j0.coeff22,j0.coeff23,j0.coeff30,j0.coeff31,j0.coeff32,j0.coeff33,"
           <<"j1.duration0,j1.duration1,j1.duration2,j1.duration3,j1.coeff00,j1.coeff01,j1.coeff02,j1.coeff03,j1.coeff10,j1.coeff11,j1.coeff12,j1.coeff13,j1.coeff20,j1.coeff21,j1.coeff22,j1.coeff23,j1.coeff30,j1.coeff31,j1.coeff32,j1.coeff33,"
           <<"j2.duration0,j2.duration1,j2.duration2,j2.duration3,j2.coeff00,j2.coeff01,j2.coeff02,j2.coeff03,j2.coeff10,j2.coeff11,j2.coeff12,j2.coeff13,j2.coeff20,j2.coeff21,j2.coeff22,j2.coeff23,j2.coeff30,j2.coeff31,j2.coeff32,j2.coeff33,"
           <<"j3.duration0,j3.duration1,j3.duration2,j3.duration3,j3.coeff00,j3.coeff01,j3.coeff02,j3.coeff03,j3.coeff10,j3.coeff11,j3.coeff12,j3.coeff13,j3.coeff20,j3.coeff21,j3.coeff22,j3.coeff23,j3.coeff30,j3.coeff31,j3.coeff32,j3.coeff33,"
           <<"j4.duration0,j4.duration1,j4.duration2,j4.duration3,j4.coeff00,j4.coeff01,j4.coeff02,j4.coeff03,j4.coeff10,j4.coeff11,j4.coeff12,j4.coeff13,j4.coeff20,j4.coeff21,j4.coeff22,j4.coeff23,j4.coeff30,j4.coeff31,j4.coeff32,j4.coeff33,"
           <<"j5.duration0,j5.duration1,j5.duration2,j5.duration3,j5.coeff00,j5.coeff01,j5.coeff02,j5.coeff03,j5.coeff10,j5.coeff11,j5.coeff12,j5.coeff13,j5.coeff20,j5.coeff21,j5.coeff22,j5.coeff23,j5.coeff30,j5.coeff31,j5.coeff32,j5.coeff33,"
           << "m00,m01,m02,m03,m04,m05,m10,m11,m12,m13,m14,m15,m20,m21,m22,m23,m24,m25,m30,m31,m32,m33,m34,m35,m40,m41,m42,m43,m44,m45,m50,m51,m52,m53,m54,m55,"
           << "c00,c01,c02,c03,c04,c05,c10,c11,c12,c13,c14,c15,c20,c21,c22,c23,c24,c25,c30,c31,c32,c33,c34,c35,c40,c41,c42,c43,c44,c45,c50,c51,c52,c53,c54,c55,"
           << "g0,g1,g2,g3,g4,g5" << endl;

    for (size_t i = 0; i < s_seg_index; i++)
    {
        s_sout << s_segment[i].time_from_start << "," << s_segment[i].duration << ",";

        for (size_t j = 0; j < 6; j++)
        {
            s_sout << s_segment[i].traj_coeff[j].duration[0] << "," << s_segment[i].traj_coeff[j].duration[1] << "," << s_segment[i].traj_coeff[j].duration[2] << "," << s_segment[i].traj_coeff[j].duration[3] << ","
                   << s_segment[i].traj_coeff[j].coeff[0][0] << "," << s_segment[i].traj_coeff[j].coeff[0][1] << "," << s_segment[i].traj_coeff[j].coeff[0][2] << "," << s_segment[i].traj_coeff[j].coeff[0][3] << ","
                   << s_segment[i].traj_coeff[j].coeff[1][0] << "," << s_segment[i].traj_coeff[j].coeff[1][1] << "," << s_segment[i].traj_coeff[j].coeff[1][2] << "," << s_segment[i].traj_coeff[j].coeff[1][3] << ","
                   << s_segment[i].traj_coeff[j].coeff[2][0] << "," << s_segment[i].traj_coeff[j].coeff[2][1] << "," << s_segment[i].traj_coeff[j].coeff[2][2] << "," << s_segment[i].traj_coeff[j].coeff[2][3] << ","
                   << s_segment[i].traj_coeff[j].coeff[3][0] << "," << s_segment[i].traj_coeff[j].coeff[3][1] << "," << s_segment[i].traj_coeff[j].coeff[3][2] << "," << s_segment[i].traj_coeff[j].coeff[3][3] << ",";
        }

        s_sout << s_segment[i].dynamics_product.m[0][0] << "," << s_segment[i].dynamics_product.m[0][1] << "," << s_segment[i].dynamics_product.m[0][2] << "," << s_segment[i].dynamics_product.m[0][3] << "," << s_segment[i].dynamics_product.m[0][4] << "," << s_segment[i].dynamics_product.m[0][5] << ","
               << s_segment[i].dynamics_product.m[1][0] << "," << s_segment[i].dynamics_product.m[1][1] << "," << s_segment[i].dynamics_product.m[1][2] << "," << s_segment[i].dynamics_product.m[1][3] << "," << s_segment[i].dynamics_product.m[1][4] << "," << s_segment[i].dynamics_product.m[1][5] << ","
               << s_segment[i].dynamics_product.m[2][0] << "," << s_segment[i].dynamics_product.m[2][1] << "," << s_segment[i].dynamics_product.m[2][2] << "," << s_segment[i].dynamics_product.m[2][3] << "," << s_segment[i].dynamics_product.m[2][4] << "," << s_segment[i].dynamics_product.m[2][5] << ","
               << s_segment[i].dynamics_product.m[3][0] << "," << s_segment[i].dynamics_product.m[3][1] << "," << s_segment[i].dynamics_product.m[3][2] << "," << s_segment[i].dynamics_product.m[3][3] << "," << s_segment[i].dynamics_product.m[3][4] << "," << s_segment[i].dynamics_product.m[3][5] << ","
               << s_segment[i].dynamics_product.m[4][0] << "," << s_segment[i].dynamics_product.m[4][1] << "," << s_segment[i].dynamics_product.m[4][2] << "," << s_segment[i].dynamics_product.m[4][3] << "," << s_segment[i].dynamics_product.m[4][4] << "," << s_segment[i].dynamics_product.m[4][5] << ","
               << s_segment[i].dynamics_product.m[5][0] << "," << s_segment[i].dynamics_product.m[5][1] << "," << s_segment[i].dynamics_product.m[5][2] << "," << s_segment[i].dynamics_product.m[5][3] << "," << s_segment[i].dynamics_product.m[5][4] << "," << s_segment[i].dynamics_product.m[5][5] << ","
               << s_segment[i].dynamics_product.c[0][0] << "," << s_segment[i].dynamics_product.c[0][1] << "," << s_segment[i].dynamics_product.c[0][2] << "," << s_segment[i].dynamics_product.c[0][3] << "," << s_segment[i].dynamics_product.c[0][4] << "," << s_segment[i].dynamics_product.c[0][5] << ","
               << s_segment[i].dynamics_product.c[1][0] << "," << s_segment[i].dynamics_product.c[1][1] << "," << s_segment[i].dynamics_product.c[1][2] << "," << s_segment[i].dynamics_product.c[1][3] << "," << s_segment[i].dynamics_product.c[1][4] << "," << s_segment[i].dynamics_product.c[1][5] << ","
               << s_segment[i].dynamics_product.c[2][0] << "," << s_segment[i].dynamics_product.c[2][1] << "," << s_segment[i].dynamics_product.c[2][2] << "," << s_segment[i].dynamics_product.c[2][3] << "," << s_segment[i].dynamics_product.c[2][4] << "," << s_segment[i].dynamics_product.c[2][5] << ","
               << s_segment[i].dynamics_product.c[3][0] << "," << s_segment[i].dynamics_product.c[3][1] << "," << s_segment[i].dynamics_product.c[3][2] << "," << s_segment[i].dynamics_product.c[3][3] << "," << s_segment[i].dynamics_product.c[3][4] << "," << s_segment[i].dynamics_product.c[3][5] << ","
               << s_segment[i].dynamics_product.c[4][0] << "," << s_segment[i].dynamics_product.c[4][1] << "," << s_segment[i].dynamics_product.c[4][2] << "," << s_segment[i].dynamics_product.c[4][3] << "," << s_segment[i].dynamics_product.c[4][4] << "," << s_segment[i].dynamics_product.c[4][5] << ","
               << s_segment[i].dynamics_product.c[5][0] << "," << s_segment[i].dynamics_product.c[5][1] << "," << s_segment[i].dynamics_product.c[5][2] << "," << s_segment[i].dynamics_product.c[5][3] << "," << s_segment[i].dynamics_product.c[5][4] << "," << s_segment[i].dynamics_product.c[5][5] << ","
               << s_segment[i].dynamics_product.g[0] << "," << s_segment[i].dynamics_product.g[1] << "," << s_segment[i].dynamics_product.g[2] << "," << s_segment[i].dynamics_product.g[3] << "," << s_segment[i].dynamics_product.g[4] << "," << s_segment[i].dynamics_product.g[5] << endl;

        if (i > 0 && i % 10000 == 0)
        {
            printf("已完成%.2f%%，还剩余%d段\n", (double)i / s_seg_index * 100, s_seg_index - 1 - i);
        }
    }

    s_sout.close();
    printf("录入完成！\n");
#endif
}

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
#ifdef OUTPUT_TRAJECTORY_SEGMENT
    s_segment[s_seg_index] = segment;
    s_seg_index = (s_seg_index + 1) % SEGMENT_SIZE;
#endif

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
    return empty() ? traj_fifo_[traj_head_ > 0 ? traj_head_ - 1 : TRAJECTORY_FIFO_CAPACITY - 1] : traj_fifo_[traj_head_];
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


