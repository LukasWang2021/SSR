/* Automatically generated nanopb header */
/* Generated by nanopb-0.3.7 at Wed Dec 18 15:11:19 2019. */

#ifndef PB_EVENTMESSAGETYPE_EVENT_BASE_PB_H_INCLUDED
#define PB_EVENTMESSAGETYPE_EVENT_BASE_PB_H_INCLUDED
#include <pb.h>

#include "protocol/event_header.pb.h"

#include "message_type/base.pb.h"

/* @@protoc_insertion_point(includes) */
#if PB_PROTO_HEADER_VERSION != 30
#error Regenerate this file with the current version of nanopb generator.
#endif

#ifdef __cplusplus
extern "C" {
#endif

/* Struct definitions */
typedef struct _EventMessageType_Uint64 {
    Event_Header header;
    MessageType_Uint64 data;
/* @@protoc_insertion_point(struct:EventMessageType_Uint64) */
} EventMessageType_Uint64;

/* Default values for struct fields */

/* Initializer values for message structs */
#define EventMessageType_Uint64_init_default     {Event_Header_init_default, MessageType_Uint64_init_default}
#define EventMessageType_Uint64_init_zero        {Event_Header_init_zero, MessageType_Uint64_init_zero}

/* Field tags (for use in manual encoding/decoding) */
#define EventMessageType_Uint64_header_tag       1
#define EventMessageType_Uint64_data_tag         2

/* Struct field encoding specification for nanopb */
extern const pb_field_t EventMessageType_Uint64_fields[3];

/* Maximum encoded size of messages (where known) */
#define EventMessageType_Uint64_size             48

/* Message IDs (where set with "msgid" option) */
#ifdef PB_MSGID

#define EVENT_BASE_MESSAGES \


#endif

#ifdef __cplusplus
} /* extern "C" */
#endif
/* @@protoc_insertion_point(eof) */

#endif
