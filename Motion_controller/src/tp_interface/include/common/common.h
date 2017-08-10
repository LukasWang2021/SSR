/**
 * @file common.h
 * @brief 
 * @author WangWei
 * @version 1.0.0
 * @date 2016-08-17
 */
#ifndef TP_INTERFACE_COMMON_H_
#define TP_INTERFACE_COMMON_H_
#include <ros/ros.h>

#ifdef PRINT
#define FST_INFO(format, ...) \
	do { \
		ROS_INFO("\033[36m" format "\033[0m", ##__VA_ARGS__); \
	} while (0) 
#else
#define FST_INFO(format, ...)
#endif

#ifdef _DEBUG
	#define FST_DEBUG(format, ...)\
		do{\
			ROS_INFO("\033[1m\033[34m" format "\033[0m", __func__, ##__VA_ARGS__); \
		} while (0) 
#else
	#define FST_DEBUG(format, ...)
#endif

#ifdef PRINT
#define FST_PRINT(format, ...) \
	do { \
		printf("\033[33m" format "\033[0m", ##__VA_ARGS__); \
	} while (0) 
#else
#define FST_PRINT(format, ...)
#endif


#define FST_ERROR ROS_ERROR

#define FST_ASSERT	ROS_ASSERT

//#define PI  3.1415926

#endif
