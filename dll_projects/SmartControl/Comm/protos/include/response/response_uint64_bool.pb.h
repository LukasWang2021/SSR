/* Automatically generated nanopb header */
/* Generated by nanopb-0.3.7 at Thu Sep 19 11:38:04 2019. */

#ifndef PB_RESPONSEMESSAGETYPE_RESPONSE_UINT64_BOOL_PB_H_INCLUDED
#define PB_RESPONSEMESSAGETYPE_RESPONSE_UINT64_BOOL_PB_H_INCLUDED
#include <pb.h>

#include "protocol/response_header.pb.h"

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
typedef struct _ResponseMessageType_Uint64_Bool {
    Response_Header header;
    Comm_Property property;
    MessageType_Uint64 error_code;
    MessageType_Bool data;
/* @@protoc_insertion_point(struct:ResponseMessageType_Uint64_Bool) */
} ResponseMessageType_Uint64_Bool;

/* Default values for struct fields */

/* Initializer values for message structs */
#define ResponseMessageType_Uint64_Bool_init_default {Response_Header_init_default, Comm_Property_init_default, MessageType_Uint64_init_default, MessageType_Bool_init_default}
#define ResponseMessageType_Uint64_Bool_init_zero {Response_Header_init_zero, Comm_Property_init_zero, MessageType_Uint64_init_zero, MessageType_Bool_init_zero}

/* Field tags (for use in manual encoding/decoding) */
#define ResponseMessageType_Uint64_Bool_header_tag 1
#define ResponseMessageType_Uint64_Bool_property_tag 2
#define ResponseMessageType_Uint64_Bool_error_code_tag 3
#define ResponseMessageType_Uint64_Bool_data_tag 4

/* Struct field encoding specification for nanopb */
extern const pb_field_t ResponseMessageType_Uint64_Bool_fields[5];

/* Maximum encoded size of messages (where known) */
#define ResponseMessageType_Uint64_Bool_size     56

/* Message IDs (where set with "msgid" option) */
#ifdef PB_MSGID

#define RESPONSE_UINT64_BOOL_MESSAGES \


#endif

#ifdef __cplusplus
} /* extern "C" */
#endif
/* @@protoc_insertion_point(eof) */

#endif