/* Automatically generated nanopb header */
/* Generated by nanopb-0.3.7-dev at Thu Dec 13 16:57:21 2018. */

#ifndef PB_REQUESTMESSAGETYPE_REQUEST_STRING_BYTES_PB_H_INCLUDED
#define PB_REQUESTMESSAGETYPE_REQUEST_STRING_BYTES_PB_H_INCLUDED
#include <pb.h>

#include "protocal/request_header.pb.h"

#include "protocal/comm.pb.h"

#include "message_type/base.pb.h"

/* @@protoc_insertion_point(includes) */
#if PB_PROTO_HEADER_VERSION != 30
#error Regenerate this file with the current version of nanopb generator.
#endif

#ifdef __cplusplus
extern "C" {
#endif

/* Struct definitions */
typedef struct _RequestMessageType_String_Bytes {
    Request_Header header;
    Comm_Property property;
    MessageType_String data1;
    MessageType_Bytes data2;
/* @@protoc_insertion_point(struct:RequestMessageType_String_Bytes) */
} RequestMessageType_String_Bytes;

/* Default values for struct fields */

/* Initializer values for message structs */
#define RequestMessageType_String_Bytes_init_default {Request_Header_init_default, Comm_Property_init_default, MessageType_String_init_default, MessageType_Bytes_init_default}
#define RequestMessageType_String_Bytes_init_zero {Request_Header_init_zero, Comm_Property_init_zero, MessageType_String_init_zero, MessageType_Bytes_init_zero}

/* Field tags (for use in manual encoding/decoding) */
#define RequestMessageType_String_Bytes_header_tag 1
#define RequestMessageType_String_Bytes_property_tag 2
#define RequestMessageType_String_Bytes_data1_tag 3
#define RequestMessageType_String_Bytes_data2_tag 4

/* Struct field encoding specification for nanopb */
extern const pb_field_t RequestMessageType_String_Bytes_fields[5];

/* Maximum encoded size of messages (where known) */
#define RequestMessageType_String_Bytes_size     66079

/* Message IDs (where set with "msgid" option) */
#ifdef PB_MSGID

#define REQUEST_STRING_BYTES_MESSAGES \


#endif

#ifdef __cplusplus
} /* extern "C" */
#endif
/* @@protoc_insertion_point(eof) */

#endif
