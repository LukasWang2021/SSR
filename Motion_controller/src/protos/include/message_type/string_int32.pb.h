/* Automatically generated nanopb header */
/* Generated by nanopb-0.3.7 at Sun Sep 29 18:18:26 2019. */

#ifndef PB_MESSAGETYPE_STRING_INT32_PB_H_INCLUDED
#define PB_MESSAGETYPE_STRING_INT32_PB_H_INCLUDED
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
typedef struct _MessageType_String_Int32 {
    MessageType_String data1;
    MessageType_Int32 data2;
/* @@protoc_insertion_point(struct:MessageType_String_Int32) */
} MessageType_String_Int32;

/* Default values for struct fields */

/* Initializer values for message structs */
#define MessageType_String_Int32_init_default    {MessageType_String_init_default, MessageType_Int32_init_default}
#define MessageType_String_Int32_init_zero       {MessageType_String_init_zero, MessageType_Int32_init_zero}

/* Field tags (for use in manual encoding/decoding) */
#define MessageType_String_Int32_data1_tag       1
#define MessageType_String_Int32_data2_tag       2

/* Struct field encoding specification for nanopb */
extern const pb_field_t MessageType_String_Int32_fields[3];

/* Maximum encoded size of messages (where known) */
#define MessageType_String_Int32_size            531

/* Message IDs (where set with "msgid" option) */
#ifdef PB_MSGID

#define STRING_INT32_MESSAGES \


#endif

#ifdef __cplusplus
} /* extern "C" */
#endif
/* @@protoc_insertion_point(eof) */

#endif
