/* Automatically generated nanopb header */
/* Generated by nanopb-0.3.7 at Fri Nov  1 09:51:43 2019. */

#ifndef PB_MESSAGETYPE_AXIS_FEEDBACK_LIST_PB_H_INCLUDED
#define PB_MESSAGETYPE_AXIS_FEEDBACK_LIST_PB_H_INCLUDED
#include <pb.h>

#include "message_type/base.pb.h"

#include "message_type/uint32list_doublelist.pb.h"

/* @@protoc_insertion_point(includes) */
#if PB_PROTO_HEADER_VERSION != 30
#error Regenerate this file with the current version of nanopb generator.
#endif

#ifdef __cplusplus
extern "C" {
#endif

/* Struct definitions */
typedef struct _MessageType_AxisFeedbackList {
    pb_size_t data_count;
    MessageType_Uint32List_DoubleList data[64];
/* @@protoc_insertion_point(struct:MessageType_AxisFeedbackList) */
} MessageType_AxisFeedbackList;

/* Default values for struct fields */

/* Initializer values for message structs */
#define MessageType_AxisFeedbackList_init_default {0, {MessageType_Uint32List_DoubleList_init_default, MessageType_Uint32List_DoubleList_init_default, MessageType_Uint32List_DoubleList_init_default, MessageType_Uint32List_DoubleList_init_default, MessageType_Uint32List_DoubleList_init_default, MessageType_Uint32List_DoubleList_init_default, MessageType_Uint32List_DoubleList_init_default, MessageType_Uint32List_DoubleList_init_default, MessageType_Uint32List_DoubleList_init_default, MessageType_Uint32List_DoubleList_init_default, MessageType_Uint32List_DoubleList_init_default, MessageType_Uint32List_DoubleList_init_default, MessageType_Uint32List_DoubleList_init_default, MessageType_Uint32List_DoubleList_init_default, MessageType_Uint32List_DoubleList_init_default, MessageType_Uint32List_DoubleList_init_default, MessageType_Uint32List_DoubleList_init_default, MessageType_Uint32List_DoubleList_init_default, MessageType_Uint32List_DoubleList_init_default, MessageType_Uint32List_DoubleList_init_default, MessageType_Uint32List_DoubleList_init_default, MessageType_Uint32List_DoubleList_init_default, MessageType_Uint32List_DoubleList_init_default, MessageType_Uint32List_DoubleList_init_default, MessageType_Uint32List_DoubleList_init_default, MessageType_Uint32List_DoubleList_init_default, MessageType_Uint32List_DoubleList_init_default, MessageType_Uint32List_DoubleList_init_default, MessageType_Uint32List_DoubleList_init_default, MessageType_Uint32List_DoubleList_init_default, MessageType_Uint32List_DoubleList_init_default, MessageType_Uint32List_DoubleList_init_default, MessageType_Uint32List_DoubleList_init_default, MessageType_Uint32List_DoubleList_init_default, MessageType_Uint32List_DoubleList_init_default, MessageType_Uint32List_DoubleList_init_default, MessageType_Uint32List_DoubleList_init_default, MessageType_Uint32List_DoubleList_init_default, MessageType_Uint32List_DoubleList_init_default, MessageType_Uint32List_DoubleList_init_default, MessageType_Uint32List_DoubleList_init_default, MessageType_Uint32List_DoubleList_init_default, MessageType_Uint32List_DoubleList_init_default, MessageType_Uint32List_DoubleList_init_default, MessageType_Uint32List_DoubleList_init_default, MessageType_Uint32List_DoubleList_init_default, MessageType_Uint32List_DoubleList_init_default, MessageType_Uint32List_DoubleList_init_default, MessageType_Uint32List_DoubleList_init_default, MessageType_Uint32List_DoubleList_init_default, MessageType_Uint32List_DoubleList_init_default, MessageType_Uint32List_DoubleList_init_default, MessageType_Uint32List_DoubleList_init_default, MessageType_Uint32List_DoubleList_init_default, MessageType_Uint32List_DoubleList_init_default, MessageType_Uint32List_DoubleList_init_default, MessageType_Uint32List_DoubleList_init_default, MessageType_Uint32List_DoubleList_init_default, MessageType_Uint32List_DoubleList_init_default, MessageType_Uint32List_DoubleList_init_default, MessageType_Uint32List_DoubleList_init_default, MessageType_Uint32List_DoubleList_init_default, MessageType_Uint32List_DoubleList_init_default, MessageType_Uint32List_DoubleList_init_default}}
#define MessageType_AxisFeedbackList_init_zero   {0, {MessageType_Uint32List_DoubleList_init_zero, MessageType_Uint32List_DoubleList_init_zero, MessageType_Uint32List_DoubleList_init_zero, MessageType_Uint32List_DoubleList_init_zero, MessageType_Uint32List_DoubleList_init_zero, MessageType_Uint32List_DoubleList_init_zero, MessageType_Uint32List_DoubleList_init_zero, MessageType_Uint32List_DoubleList_init_zero, MessageType_Uint32List_DoubleList_init_zero, MessageType_Uint32List_DoubleList_init_zero, MessageType_Uint32List_DoubleList_init_zero, MessageType_Uint32List_DoubleList_init_zero, MessageType_Uint32List_DoubleList_init_zero, MessageType_Uint32List_DoubleList_init_zero, MessageType_Uint32List_DoubleList_init_zero, MessageType_Uint32List_DoubleList_init_zero, MessageType_Uint32List_DoubleList_init_zero, MessageType_Uint32List_DoubleList_init_zero, MessageType_Uint32List_DoubleList_init_zero, MessageType_Uint32List_DoubleList_init_zero, MessageType_Uint32List_DoubleList_init_zero, MessageType_Uint32List_DoubleList_init_zero, MessageType_Uint32List_DoubleList_init_zero, MessageType_Uint32List_DoubleList_init_zero, MessageType_Uint32List_DoubleList_init_zero, MessageType_Uint32List_DoubleList_init_zero, MessageType_Uint32List_DoubleList_init_zero, MessageType_Uint32List_DoubleList_init_zero, MessageType_Uint32List_DoubleList_init_zero, MessageType_Uint32List_DoubleList_init_zero, MessageType_Uint32List_DoubleList_init_zero, MessageType_Uint32List_DoubleList_init_zero, MessageType_Uint32List_DoubleList_init_zero, MessageType_Uint32List_DoubleList_init_zero, MessageType_Uint32List_DoubleList_init_zero, MessageType_Uint32List_DoubleList_init_zero, MessageType_Uint32List_DoubleList_init_zero, MessageType_Uint32List_DoubleList_init_zero, MessageType_Uint32List_DoubleList_init_zero, MessageType_Uint32List_DoubleList_init_zero, MessageType_Uint32List_DoubleList_init_zero, MessageType_Uint32List_DoubleList_init_zero, MessageType_Uint32List_DoubleList_init_zero, MessageType_Uint32List_DoubleList_init_zero, MessageType_Uint32List_DoubleList_init_zero, MessageType_Uint32List_DoubleList_init_zero, MessageType_Uint32List_DoubleList_init_zero, MessageType_Uint32List_DoubleList_init_zero, MessageType_Uint32List_DoubleList_init_zero, MessageType_Uint32List_DoubleList_init_zero, MessageType_Uint32List_DoubleList_init_zero, MessageType_Uint32List_DoubleList_init_zero, MessageType_Uint32List_DoubleList_init_zero, MessageType_Uint32List_DoubleList_init_zero, MessageType_Uint32List_DoubleList_init_zero, MessageType_Uint32List_DoubleList_init_zero, MessageType_Uint32List_DoubleList_init_zero, MessageType_Uint32List_DoubleList_init_zero, MessageType_Uint32List_DoubleList_init_zero, MessageType_Uint32List_DoubleList_init_zero, MessageType_Uint32List_DoubleList_init_zero, MessageType_Uint32List_DoubleList_init_zero, MessageType_Uint32List_DoubleList_init_zero, MessageType_Uint32List_DoubleList_init_zero}}

/* Field tags (for use in manual encoding/decoding) */
#define MessageType_AxisFeedbackList_data_tag    1

/* Struct field encoding specification for nanopb */
extern const pb_field_t MessageType_AxisFeedbackList_fields[2];

/* Maximum encoded size of messages (where known) */
#define MessageType_AxisFeedbackList_size        123456

/* Message IDs (where set with "msgid" option) */
#ifdef PB_MSGID

#define AXIS_FEEDBACK_LIST_MESSAGES \


#endif

#ifdef __cplusplus
} /* extern "C" */
#endif
/* @@protoc_insertion_point(eof) */

#endif
