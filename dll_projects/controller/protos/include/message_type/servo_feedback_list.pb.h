/* Automatically generated nanopb header */
/* Generated by nanopb-0.3.7 at Mon May 11 13:51:35 2020. */

#ifndef PB_MESSAGETYPE_SERVO_FEEDBACK_LIST_PB_H_INCLUDED
#define PB_MESSAGETYPE_SERVO_FEEDBACK_LIST_PB_H_INCLUDED
#include <pb.h>

#include "message_type/base.pb.h"

/* @@protoc_insertion_point(includes) */
#if PB_PROTO_HEADER_VERSION != 30
#error Regenerate this file with the current version of nanopb generator.
#endif

#ifdef __cplusplus
extern "C" {
#endif

/* Struct definitions */
typedef struct _MessageType_ServoFeedbackList {
    pb_size_t data_count;
    MessageType_Int32List data[64];
/* @@protoc_insertion_point(struct:MessageType_ServoFeedbackList) */
} MessageType_ServoFeedbackList;

/* Default values for struct fields */

/* Initializer values for message structs */
#define MessageType_ServoFeedbackList_init_default {0, {MessageType_Int32List_init_default, MessageType_Int32List_init_default, MessageType_Int32List_init_default, MessageType_Int32List_init_default, MessageType_Int32List_init_default, MessageType_Int32List_init_default, MessageType_Int32List_init_default, MessageType_Int32List_init_default, MessageType_Int32List_init_default, MessageType_Int32List_init_default, MessageType_Int32List_init_default, MessageType_Int32List_init_default, MessageType_Int32List_init_default, MessageType_Int32List_init_default, MessageType_Int32List_init_default, MessageType_Int32List_init_default, MessageType_Int32List_init_default, MessageType_Int32List_init_default, MessageType_Int32List_init_default, MessageType_Int32List_init_default, MessageType_Int32List_init_default, MessageType_Int32List_init_default, MessageType_Int32List_init_default, MessageType_Int32List_init_default, MessageType_Int32List_init_default, MessageType_Int32List_init_default, MessageType_Int32List_init_default, MessageType_Int32List_init_default, MessageType_Int32List_init_default, MessageType_Int32List_init_default, MessageType_Int32List_init_default, MessageType_Int32List_init_default, MessageType_Int32List_init_default, MessageType_Int32List_init_default, MessageType_Int32List_init_default, MessageType_Int32List_init_default, MessageType_Int32List_init_default, MessageType_Int32List_init_default, MessageType_Int32List_init_default, MessageType_Int32List_init_default, MessageType_Int32List_init_default, MessageType_Int32List_init_default, MessageType_Int32List_init_default, MessageType_Int32List_init_default, MessageType_Int32List_init_default, MessageType_Int32List_init_default, MessageType_Int32List_init_default, MessageType_Int32List_init_default, MessageType_Int32List_init_default, MessageType_Int32List_init_default, MessageType_Int32List_init_default, MessageType_Int32List_init_default, MessageType_Int32List_init_default, MessageType_Int32List_init_default, MessageType_Int32List_init_default, MessageType_Int32List_init_default, MessageType_Int32List_init_default, MessageType_Int32List_init_default, MessageType_Int32List_init_default, MessageType_Int32List_init_default, MessageType_Int32List_init_default, MessageType_Int32List_init_default, MessageType_Int32List_init_default, MessageType_Int32List_init_default}}
#define MessageType_ServoFeedbackList_init_zero  {0, {MessageType_Int32List_init_zero, MessageType_Int32List_init_zero, MessageType_Int32List_init_zero, MessageType_Int32List_init_zero, MessageType_Int32List_init_zero, MessageType_Int32List_init_zero, MessageType_Int32List_init_zero, MessageType_Int32List_init_zero, MessageType_Int32List_init_zero, MessageType_Int32List_init_zero, MessageType_Int32List_init_zero, MessageType_Int32List_init_zero, MessageType_Int32List_init_zero, MessageType_Int32List_init_zero, MessageType_Int32List_init_zero, MessageType_Int32List_init_zero, MessageType_Int32List_init_zero, MessageType_Int32List_init_zero, MessageType_Int32List_init_zero, MessageType_Int32List_init_zero, MessageType_Int32List_init_zero, MessageType_Int32List_init_zero, MessageType_Int32List_init_zero, MessageType_Int32List_init_zero, MessageType_Int32List_init_zero, MessageType_Int32List_init_zero, MessageType_Int32List_init_zero, MessageType_Int32List_init_zero, MessageType_Int32List_init_zero, MessageType_Int32List_init_zero, MessageType_Int32List_init_zero, MessageType_Int32List_init_zero, MessageType_Int32List_init_zero, MessageType_Int32List_init_zero, MessageType_Int32List_init_zero, MessageType_Int32List_init_zero, MessageType_Int32List_init_zero, MessageType_Int32List_init_zero, MessageType_Int32List_init_zero, MessageType_Int32List_init_zero, MessageType_Int32List_init_zero, MessageType_Int32List_init_zero, MessageType_Int32List_init_zero, MessageType_Int32List_init_zero, MessageType_Int32List_init_zero, MessageType_Int32List_init_zero, MessageType_Int32List_init_zero, MessageType_Int32List_init_zero, MessageType_Int32List_init_zero, MessageType_Int32List_init_zero, MessageType_Int32List_init_zero, MessageType_Int32List_init_zero, MessageType_Int32List_init_zero, MessageType_Int32List_init_zero, MessageType_Int32List_init_zero, MessageType_Int32List_init_zero, MessageType_Int32List_init_zero, MessageType_Int32List_init_zero, MessageType_Int32List_init_zero, MessageType_Int32List_init_zero, MessageType_Int32List_init_zero, MessageType_Int32List_init_zero, MessageType_Int32List_init_zero, MessageType_Int32List_init_zero}}

/* Field tags (for use in manual encoding/decoding) */
#define MessageType_ServoFeedbackList_data_tag   1

/* Struct field encoding specification for nanopb */
extern const pb_field_t MessageType_ServoFeedbackList_fields[2];

/* Maximum encoded size of messages (where known) */
#define MessageType_ServoFeedbackList_size       360640

/* Message IDs (where set with "msgid" option) */
#ifdef PB_MSGID

#define SERVO_FEEDBACK_LIST_MESSAGES \


#endif

#ifdef __cplusplus
} /* extern "C" */
#endif
/* @@protoc_insertion_point(eof) */

#endif
