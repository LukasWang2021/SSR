/* Automatically generated nanopb header */
/* Generated by nanopb-0.3.7 at Mon Dec  2 13:20:24 2019. */

#ifndef PB_MESSAGETYPE_SCARA_A_FEEDBACK_LIST_PB_H_INCLUDED
#define PB_MESSAGETYPE_SCARA_A_FEEDBACK_LIST_PB_H_INCLUDED
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
typedef struct _MessageType_ScaraAFeedbackList {
    pb_size_t data_count;
    MessageType_Uint32List_DoubleList data[16];
/* @@protoc_insertion_point(struct:MessageType_ScaraAFeedbackList) */
} MessageType_ScaraAFeedbackList;

/* Default values for struct fields */

/* Initializer values for message structs */
#define MessageType_ScaraAFeedbackList_init_default {0, {MessageType_Uint32List_DoubleList_init_default, MessageType_Uint32List_DoubleList_init_default, MessageType_Uint32List_DoubleList_init_default, MessageType_Uint32List_DoubleList_init_default, MessageType_Uint32List_DoubleList_init_default, MessageType_Uint32List_DoubleList_init_default, MessageType_Uint32List_DoubleList_init_default, MessageType_Uint32List_DoubleList_init_default, MessageType_Uint32List_DoubleList_init_default, MessageType_Uint32List_DoubleList_init_default, MessageType_Uint32List_DoubleList_init_default, MessageType_Uint32List_DoubleList_init_default, MessageType_Uint32List_DoubleList_init_default, MessageType_Uint32List_DoubleList_init_default, MessageType_Uint32List_DoubleList_init_default, MessageType_Uint32List_DoubleList_init_default}}
#define MessageType_ScaraAFeedbackList_init_zero {0, {MessageType_Uint32List_DoubleList_init_zero, MessageType_Uint32List_DoubleList_init_zero, MessageType_Uint32List_DoubleList_init_zero, MessageType_Uint32List_DoubleList_init_zero, MessageType_Uint32List_DoubleList_init_zero, MessageType_Uint32List_DoubleList_init_zero, MessageType_Uint32List_DoubleList_init_zero, MessageType_Uint32List_DoubleList_init_zero, MessageType_Uint32List_DoubleList_init_zero, MessageType_Uint32List_DoubleList_init_zero, MessageType_Uint32List_DoubleList_init_zero, MessageType_Uint32List_DoubleList_init_zero, MessageType_Uint32List_DoubleList_init_zero, MessageType_Uint32List_DoubleList_init_zero, MessageType_Uint32List_DoubleList_init_zero, MessageType_Uint32List_DoubleList_init_zero}}

/* Field tags (for use in manual encoding/decoding) */
#define MessageType_ScaraAFeedbackList_data_tag  1

/* Struct field encoding specification for nanopb */
extern const pb_field_t MessageType_ScaraAFeedbackList_fields[2];

/* Maximum encoded size of messages (where known) */
#define MessageType_ScaraAFeedbackList_size      30864

/* Message IDs (where set with "msgid" option) */
#ifdef PB_MSGID

#define SCARA_A_FEEDBACK_LIST_MESSAGES \


#endif

#ifdef __cplusplus
} /* extern "C" */
#endif
/* @@protoc_insertion_point(eof) */

#endif
