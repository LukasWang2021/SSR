/* Automatically generated nanopb header */
/* Generated by nanopb-0.3.7 at Thu Sep 19 13:40:58 2019. */

#ifndef PB_REQUESTMESSAGETYPE_REQUEST_INT32_COORD_TYPE_PB_H_INCLUDED
#define PB_REQUESTMESSAGETYPE_REQUEST_INT32_COORD_TYPE_PB_H_INCLUDED
#include <pb.h>

#include "protocol/request_header.pb.h"

#include "protocol/comm.pb.h"

#include "message_type/base.pb.h"

#include "message_type/coord_type.pb.h"

/* @@protoc_insertion_point(includes) */
#if PB_PROTO_HEADER_VERSION != 30
#error Regenerate this file with the current version of nanopb generator.
#endif

#ifdef __cplusplus
extern "C" {
#endif

/* Struct definitions */
typedef struct _RequestMessageType_Int32_CoordType {
    Request_Header header;
    Comm_Property property;
    MessageType_Int32 data1;
    MessageType_CoordType data2;
/* @@protoc_insertion_point(struct:RequestMessageType_Int32_CoordType) */
} RequestMessageType_Int32_CoordType;

/* Default values for struct fields */

/* Initializer values for message structs */
#define RequestMessageType_Int32_CoordType_init_default {Request_Header_init_default, Comm_Property_init_default, MessageType_Int32_init_default, (MessageType_CoordType)0}
#define RequestMessageType_Int32_CoordType_init_zero {Request_Header_init_zero, Comm_Property_init_zero, MessageType_Int32_init_zero, (MessageType_CoordType)0}

/* Field tags (for use in manual encoding/decoding) */
#define RequestMessageType_Int32_CoordType_header_tag 1
#define RequestMessageType_Int32_CoordType_property_tag 2
#define RequestMessageType_Int32_CoordType_data1_tag 3
#define RequestMessageType_Int32_CoordType_data2_tag 4

/* Struct field encoding specification for nanopb */
extern const pb_field_t RequestMessageType_Int32_CoordType_fields[5];

/* Maximum encoded size of messages (where known) */
#define RequestMessageType_Int32_CoordType_size  32

/* Message IDs (where set with "msgid" option) */
#ifdef PB_MSGID

#define REQUEST_INT32_COORD_TYPE_MESSAGES \


#endif

#ifdef __cplusplus
} /* extern "C" */
#endif
/* @@protoc_insertion_point(eof) */

#endif
