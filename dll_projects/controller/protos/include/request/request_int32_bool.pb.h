/* Automatically generated nanopb header */
/* Generated by nanopb-0.3.7 at Thu Sep 19 11:38:03 2019. */

#ifndef PB_REQUESTMESSAGETYPE_REQUEST_INT32_BOOL_PB_H_INCLUDED
#define PB_REQUESTMESSAGETYPE_REQUEST_INT32_BOOL_PB_H_INCLUDED
#include <pb.h>

#include "protocol/request_header.pb.h"

#include "protocol/comm.pb.h"

#include "message_type/base.pb.h"

/* @@protoc_insertion_point(includes) */
#if PB_PROTO_HEADER_VERSION != 30
#error Regenerate this file with the current version of nanopb generator.
#endif

#ifdef __cplusplus
extern "C" {
#endif

/* Struct definitions */
typedef struct _RequestMessageType_Int32_Bool {
    Request_Header header;
    Comm_Property property;
    MessageType_Int32 data1;
    MessageType_Bool data2;
/* @@protoc_insertion_point(struct:RequestMessageType_Int32_Bool) */
} RequestMessageType_Int32_Bool;

/* Default values for struct fields */

/* Initializer values for message structs */
#define RequestMessageType_Int32_Bool_init_default {Request_Header_init_default, Comm_Property_init_default, MessageType_Int32_init_default, MessageType_Bool_init_default}
#define RequestMessageType_Int32_Bool_init_zero  {Request_Header_init_zero, Comm_Property_init_zero, MessageType_Int32_init_zero, MessageType_Bool_init_zero}

/* Field tags (for use in manual encoding/decoding) */
#define RequestMessageType_Int32_Bool_header_tag 1
#define RequestMessageType_Int32_Bool_property_tag 2
#define RequestMessageType_Int32_Bool_data1_tag  3
#define RequestMessageType_Int32_Bool_data2_tag  4

/* Struct field encoding specification for nanopb */
extern const pb_field_t RequestMessageType_Int32_Bool_fields[5];

/* Maximum encoded size of messages (where known) */
#define RequestMessageType_Int32_Bool_size       34

/* Message IDs (where set with "msgid" option) */
#ifdef PB_MSGID

#define REQUEST_INT32_BOOL_MESSAGES \


#endif

#ifdef __cplusplus
} /* extern "C" */
#endif
/* @@protoc_insertion_point(eof) */

#endif
