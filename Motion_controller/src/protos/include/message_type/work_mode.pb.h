/* Automatically generated nanopb header */
/* Generated by nanopb-0.3.7 at Sun Sep 29 18:18:27 2019. */

#ifndef PB_MESSAGETYPE_WORK_MODE_PB_H_INCLUDED
#define PB_MESSAGETYPE_WORK_MODE_PB_H_INCLUDED
#include <pb.h>

/* @@protoc_insertion_point(includes) */
#if PB_PROTO_HEADER_VERSION != 30
#error Regenerate this file with the current version of nanopb generator.
#endif

#ifdef __cplusplus
extern "C" {
#endif

/* Enum definitions */
typedef enum _MessageType_WorkMode {
    MessageType_WorkMode_INIT = 0,
    MessageType_WorkMode_AUTO = 1,
    MessageType_WorkMode_MANUAL = 2
} MessageType_WorkMode;
#define _MessageType_WorkMode_MIN MessageType_WorkMode_INIT
#define _MessageType_WorkMode_MAX MessageType_WorkMode_MANUAL
#define _MessageType_WorkMode_ARRAYSIZE ((MessageType_WorkMode)(MessageType_WorkMode_MANUAL+1))

#ifdef __cplusplus
} /* extern "C" */
#endif
/* @@protoc_insertion_point(eof) */

#endif
