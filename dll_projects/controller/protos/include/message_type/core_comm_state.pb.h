/* Automatically generated nanopb header */
/* Generated by nanopb-0.3.7 at Thu Sep 19 16:40:42 2019. */

#ifndef PB_MESSAGETYPE_CORE_COMM_STATE_PB_H_INCLUDED
#define PB_MESSAGETYPE_CORE_COMM_STATE_PB_H_INCLUDED
#include <pb.h>

/* @@protoc_insertion_point(includes) */
#if PB_PROTO_HEADER_VERSION != 30
#error Regenerate this file with the current version of nanopb generator.
#endif

#ifdef __cplusplus
extern "C" {
#endif

/* Enum definitions */
typedef enum _MessageType_CoreCommState {
    MessageType_CoreCommState_INIT = 0,
    MessageType_CoreCommState_PREOP = 1,
    MessageType_CoreCommState_SAFEOP = 2,
    MessageType_CoreCommState_OP = 3,
    MessageType_CoreCommState_UNKNOWN = 4
} MessageType_CoreCommState;
#define _MessageType_CoreCommState_MIN MessageType_CoreCommState_INIT
#define _MessageType_CoreCommState_MAX MessageType_CoreCommState_UNKNOWN
#define _MessageType_CoreCommState_ARRAYSIZE ((MessageType_CoreCommState)(MessageType_CoreCommState_UNKNOWN+1))

#ifdef __cplusplus
} /* extern "C" */
#endif
/* @@protoc_insertion_point(eof) */

#endif
