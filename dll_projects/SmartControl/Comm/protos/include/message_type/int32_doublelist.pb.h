/* Automatically generated nanopb header */
/* Generated by nanopb-0.3.7 at Wed Sep 25 15:42:04 2019. */

#ifndef PB_MESSAGETYPE_INT32_DOUBLELIST_PB_H_INCLUDED
#define PB_MESSAGETYPE_INT32_DOUBLELIST_PB_H_INCLUDED
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
typedef struct _MessageType_Int32_DoubleList {
    MessageType_Int32 data1;
    MessageType_DoubleList data2;
/* @@protoc_insertion_point(struct:MessageType_Int32_DoubleList) */
} MessageType_Int32_DoubleList;

/* Default values for struct fields */

/* Initializer values for message structs */
#define MessageType_Int32_DoubleList_init_default {MessageType_Int32_init_default, MessageType_DoubleList_init_default}
#define MessageType_Int32_DoubleList_init_zero   {MessageType_Int32_init_zero, MessageType_DoubleList_init_zero}

/* Field tags (for use in manual encoding/decoding) */
#define MessageType_Int32_DoubleList_data1_tag   1
#define MessageType_Int32_DoubleList_data2_tag   2

/* Struct field encoding specification for nanopb */
extern const pb_field_t MessageType_Int32_DoubleList_fields[3];

/* Maximum encoded size of messages (where known) */
#define MessageType_Int32_DoubleList_size        1168

/* Message IDs (where set with "msgid" option) */
#ifdef PB_MSGID

#define INT32_DOUBLELIST_MESSAGES \


#endif

#ifdef __cplusplus
} /* extern "C" */
#endif
/* @@protoc_insertion_point(eof) */

#endif
