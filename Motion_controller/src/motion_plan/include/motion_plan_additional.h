/*************************************************************************
	> File Name: motion_plan_additional.h
	> Author: 
	> Mail: 
	> Created Time: 2018年05月24日 星期四 19时28分19秒
 ************************************************************************/

#ifndef _MOTION_PLAN_ADDITIONAL_H
#define _MOTION_PLAN_ADDITIONAL_H

#include <fst_datatype.h>

extern void trimArray(double *array, size_t size, double ratio, double *res);
extern void logTrajSegment(const char *str, const size_t stamp, const fst_controller::TrajSegment &seg);
#endif
