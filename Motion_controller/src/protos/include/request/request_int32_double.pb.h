/* Automatically generated nanopb header */
/* Generated by nanopb-0.3.7-dev at Tue Aug 28 15:30:36 2018. */

#ifndef PB_REQUESTMESSAGETYPE_REQUEST_INT32_DOUBLE_PB_H_INCLUDED
#define PB_REQUESTMESSAGETYPE_REQUEST_INT32_DOUBLE_PB_H_INCLUDED
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
typedef struct _RequestMessageType_Int32_Double {
    Request_Header header;
    Comm_Property property;
    MessageType_Int32 data1;
    MessageType_Double data2;
/* @@protoc_insertion_point(struct:RequestMessageType_Int32_Double) */
} RequestMessageType_Int32_Double;

/* Default values for struct fields */

/* Initializer values for message structs */
#define RequestMessageType_Int32_Double_init_default {Request_Header_init_default, Comm_Property_init_default, MessageType_Int32_init_default, MessageType_Double_init_default}
#define RequestMessageType_Int32_Double_init_zero {Request_Header_init_zero, Comm_Property_init_zero, MessageType_Int32_init_zero, MessageType_Double_init_zero}

/* Field tags (for use in manual encoding/decoding) */
#define RequestMessageType_Int32_Double_header_tag 1
#define RequestMessageType_Int32_Double_property_tag 2
#define RequestMessageType_Int32_Double_data1_tag 3
#define RequestMessageType_Int32_Double_data2_tag 4

/* Struct field encoding specification for nanopb */
extern const pb_field_t RequestMessageType_Int32_Double_fields[5];

/* Maximum encoded size of messages (where known) */
#define RequestMessageType_Int32_Double_size     41

/* Message IDs (where set with "msgid" option) */
#ifdef PB_MSGID

#define REQUEST_INT32_DOUBLE_MESSAGES \


#endif

#ifdef __cplusplus
} /* extern "C" */
#endif
/* @@protoc_insertion_point(eof) */

#endif
