/*************************************************************************
	> File Name: motion_plan_additional.cpp
	> Author: 
	> Mail: 
	> Created Time: 2018年05月24日 星期四 19时18分21秒
 ************************************************************************/

#include <string.h>
#include <fst_datatype.h>
#include <motion_plan_reuse.h>

using namespace fst_controller;

void trimArray(double *array, size_t size, double ratio, double *res)
{
    if (array != NULL && res != NULL && array != res)
        for (size_t i = 0; i < size; i++)
            res[i] = array[i] * ratio;
}

void logTrajSegment(const char *str, const size_t stamp, const TrajSegment &seg)
{
    FST_LOG("%s-%d: time-from-start=%.6f, duration=%.6f", str, stamp, seg.time_from_start, seg.duration);
    
    for (size_t i = 0; i < AXIS_IN_ALGORITHM; i++)
    {
        FST_LOG("  %.6f,%.6f,%.6f,%.6f,%.6f,%.6f",
                seg.coeff[i][5], seg.coeff[i][4], seg.coeff[i][3],
                seg.coeff[i][2], seg.coeff[i][1], seg.coeff[i][0]);
    } 
}

