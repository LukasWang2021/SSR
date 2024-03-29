/* Automatically generated nanopb header */
/* Generated by nanopb-0.3.7 at Thu May  6 10:02:09 2021. */

#ifndef PB_RESPONSEMESSAGETYPE_RESPONSE_COORD_USER_PB_H_INCLUDED
#define PB_RESPONSEMESSAGETYPE_RESPONSE_COORD_USER_PB_H_INCLUDED
#include <pb.h>

#include "protocol/response_header.pb.h"

#include "protocol/comm.pb.h"

#include "message_type/coord_user.pb.h"

#include "message_type/base.pb.h"

/* @@protoc_insertion_point(includes) */
#if PB_PROTO_HEADER_VERSION != 30
#error Regenerate this file with the current version of nanopb generator.
#endif

#ifdef __cplusplus
extern "C" {
#endif

/* Struct definitions */
typedef struct _ResponseMessageType_Uint64_UserCoordInfo {
    Response_Header header;
    Comm_Property property;
    MessageType_Uint64 error_code;
    MessageType_UserCoordInfo data;
/* @@protoc_insertion_point(struct:ResponseMessageType_Uint64_UserCoordInfo) */
} ResponseMessageType_Uint64_UserCoordInfo;

typedef struct _ResponseMessageType_Uint64_UserCoordSummaryList {
    Response_Header header;
    Comm_Property property;
    MessageType_Uint64 error_code;
    MessageType_UserCoordSummaryList data;
/* @@protoc_insertion_point(struct:ResponseMessageType_Uint64_UserCoordSummaryList) */
} ResponseMessageType_Uint64_UserCoordSummaryList;

/* Default values for struct fields */

/* Initializer values for message structs */
#define ResponseMessageType_Uint64_UserCoordInfo_init_default {Response_Header_init_default, Comm_Property_init_default, MessageType_Uint64_init_default, MessageType_UserCoordInfo_init_default}
#define ResponseMessageType_Uint64_UserCoordSummaryList_init_default {Response_Header_init_default, Comm_Property_init_default, MessageType_Uint64_init_default, MessageType_UserCoordSummaryList_init_default}
#define ResponseMessageType_Uint64_UserCoordInfo_init_zero {Response_Header_init_zero, Comm_Property_init_zero, MessageType_Uint64_init_zero, MessageType_UserCoordInfo_init_zero}
#define ResponseMessageType_Uint64_UserCoordSummaryList_init_zero {Response_Header_init_zero, Comm_Property_init_zero, MessageType_Uint64_init_zero, MessageType_UserCoordSummaryList_init_zero}

/* Field tags (for use in manual encoding/decoding) */
#define ResponseMessageType_Uint64_UserCoordInfo_header_tag 1
#define ResponseMessageType_Uint64_UserCoordInfo_property_tag 2
#define ResponseMessageType_Uint64_UserCoordInfo_error_code_tag 3
#define ResponseMessageType_Uint64_UserCoordInfo_data_tag 4
#define ResponseMessageType_Uint64_UserCoordSummaryList_header_tag 1
#define ResponseMessageType_Uint64_UserCoordSummaryList_property_tag 2
#define ResponseMessageType_Uint64_UserCoordSummaryList_error_code_tag 3
#define ResponseMessageType_Uint64_UserCoordSummaryList_data_tag 4

/* Struct field encoding specification for nanopb */
extern const pb_field_t ResponseMessageType_Uint64_UserCoordInfo_fields[5];
extern const pb_field_t ResponseMessageType_Uint64_UserCoordSummaryList_fields[5];

/* Maximum encoded size of messages (where known) */
#define ResponseMessageType_Uint64_UserCoordInfo_size 1525
#define ResponseMessageType_Uint64_UserCoordSummaryList_size 5143

/* Message IDs (where set with "msgid" option) */
#ifdef PB_MSGID

#define RESPONSE_COORD_USER_MESSAGES \


#endif

#ifdef __cplusplus
} /* extern "C" */
#endif
/* @@protoc_insertion_point(eof) */

#endif
