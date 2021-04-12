/* Automatically generated nanopb header */
/* Generated by nanopb-0.3.7 at Sun Sep 29 18:18:27 2019. */

#ifndef PB_MESSAGETYPE_AXIS_STATUS_PB_H_INCLUDED
#define PB_MESSAGETYPE_AXIS_STATUS_PB_H_INCLUDED
#include <pb.h>

/* @@protoc_insertion_point(includes) */
#if PB_PROTO_HEADER_VERSION != 30
#error Regenerate this file with the current version of nanopb generator.
#endif

#ifdef __cplusplus
extern "C" {
#endif

/* Enum definitions */
typedef enum _MessageType_AxisStatus {
    MessageType_AxisStatus_UNKNOWN = 0,
    MessageType_AxisStatus_ERRORSTOP = 1,
    MessageType_AxisStatus_DISABLED = 2,
    MessageType_AxisStatus_STANDSTILL = 3,
    MessageType_AxisStatus_STOPPING = 4,
    MessageType_AxisStatus_HOMING = 5,
    MessageType_AxisStatus_DISCRETE_MOTION = 6,
    MessageType_AxisStatus_CONTINUOUS_MOTION = 7,
    MessageType_AxisStatus_SYNCHRONIZED_MOTION = 8
} MessageType_AxisStatus;
#define _MessageType_AxisStatus_MIN MessageType_AxisStatus_UNKNOWN
#define _MessageType_AxisStatus_MAX MessageType_AxisStatus_SYNCHRONIZED_MOTION
#define _MessageType_AxisStatus_ARRAYSIZE ((MessageType_AxisStatus)(MessageType_AxisStatus_SYNCHRONIZED_MOTION+1))

#ifdef __cplusplus
} /* extern "C" */
#endif
/* @@protoc_insertion_point(eof) */

#endif
