/* Automatically generated nanopb header */
/* Generated by nanopb-0.3.7 at Thu Sep 19 11:38:03 2019. */

#ifndef PB_REQUESTMESSAGETYPE_REQUEST_STRINGLIST_PB_H_INCLUDED
#define PB_REQUESTMESSAGETYPE_REQUEST_STRINGLIST_PB_H_INCLUDED
#include <pb.h>

#include "protocol/request_header.pb.h"

#include "protocol/comm.pb.h"

#include "message_type/string_list.pb.h"

/* @@protoc_insertion_point(includes) */
#if PB_PROTO_HEADER_VERSION != 30
#error Regenerate this file with the current version of nanopb generator.
#endif

#ifdef __cplusplus
extern "C" {
#endif

/* Struct definitions */
typedef struct _RequestMessageType_StringList {
    Request_Header header;
    Comm_Property property;
    MessageType_StringList data;
/* @@protoc_insertion_point(struct:RequestMessageType_StringList) */
} RequestMessageType_StringList;

/* Default values for struct fields */

/* Initializer values for message structs */
#define RequestMessageType_StringList_init_default {Request_Header_init_default, Comm_Property_init_default, MessageType_StringList_init_default}
#define RequestMessageType_StringList_init_zero  {Request_Header_init_zero, Comm_Property_init_zero, MessageType_StringList_init_zero}

/* Field tags (for use in manual encoding/decoding) */
#define RequestMessageType_StringList_header_tag 1
#define RequestMessageType_StringList_property_tag 2
#define RequestMessageType_StringList_data_tag   3

/* Struct field encoding specification for nanopb */
extern const pb_field_t RequestMessageType_StringList_fields[4];

/* Maximum encoded size of messages (where known) */
#define RequestMessageType_StringList_size       (215 + 32*MessageType_String_size)

/* Message IDs (where set with "msgid" option) */
#ifdef PB_MSGID

#define REQUEST_STRINGLIST_MESSAGES \


#endif

#ifdef __cplusplus
} /* extern "C" */
#endif
/* @@protoc_insertion_point(eof) */

#endif
