/*************************************************************************
	> File Name: motion_plan_reuse.h
	> Author:   Feng Yun
	> Mail:     yun.feng@foresight-robotics.com
	> Created Time: 2018年02月27日 星期二 10时20分34秒
 ************************************************************************/

#ifndef _MOTION_PLAN_REUSE_H
#define _MOTION_PLAN_REUSE_H

#include <motion_plan_variable.h>

#define FST_LOG(fmt, ...)       fst_algorithm::g_log.log(fmt, ##__VA_ARGS__)
#define FST_INFO(fmt, ...)      fst_algorithm::g_log.info(fmt, ##__VA_ARGS__)
#define FST_WARN(fmt, ...)      fst_algorithm::g_log.warn(fmt, ##__VA_ARGS__)
#define FST_ERROR(fmt, ...)     fst_algorithm::g_log.error(fmt, ##__VA_ARGS__)

#endif
