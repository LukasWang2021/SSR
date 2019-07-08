/* Automatically generated nanopb header */
/* Generated by nanopb-0.3.7-dev at Mon Jul  8 14:53:53 2019. */

#ifndef PB_RESPONSEMESSAGETYPE_RESPONSE_UINT64_PAYLOAD_SUMMARY_LIST_PB_H_INCLUDED
#define PB_RESPONSEMESSAGETYPE_RESPONSE_UINT64_PAYLOAD_SUMMARY_LIST_PB_H_INCLUDED
#include <pb.h>

#include "protocal/response_header.pb.h"

#include "protocal/comm.pb.h"

#include "message_type/payload_info.pb.h"

/* @@protoc_insertion_point(includes) */
#if PB_PROTO_HEADER_VERSION != 30
#error Regenerate this file with the current version of nanopb generator.
#endif

#ifdef __cplusplus
extern "C" {
#endif

/* Struct definitions */
typedef struct _ResponseMessageType_Uint64_PayloadSummaryList {
    Response_Header header;
    Comm_Property property;
    MessageType_Uint64 error_code;
    MessageType_PayloadSummaryList data;
/* @@protoc_insertion_point(struct:ResponseMessageType_Uint64_PayloadSummaryList) */
} ResponseMessageType_Uint64_PayloadSummaryList;

/* Default values for struct fields */

/* Initializer values for message structs */
#define ResponseMessageType_Uint64_PayloadSummaryList_init_default {Response_Header_init_default, Comm_Property_init_default, MessageType_Uint64_init_default, MessageType_PayloadSummaryList_init_default}
#define ResponseMessageType_Uint64_PayloadSummaryList_init_zero {Response_Header_init_zero, Comm_Property_init_zero, MessageType_Uint64_init_zero, MessageType_PayloadSummaryList_init_zero}

/* Field tags (for use in manual encoding/decoding) */
#define ResponseMessageType_Uint64_PayloadSummaryList_header_tag 1
#define ResponseMessageType_Uint64_PayloadSummaryList_property_tag 2
#define ResponseMessageType_Uint64_PayloadSummaryList_error_code_tag 3
#define ResponseMessageType_Uint64_PayloadSummaryList_data_tag 4

/* Struct field encoding specification for nanopb */
extern const pb_field_t ResponseMessageType_Uint64_PayloadSummaryList_fields[5];

/* Maximum encoded size of messages (where known) */
#define ResponseMessageType_Uint64_PayloadSummaryList_size (51 + MessageType_Uint64_size + MessageType_PayloadSummaryList_size)

/* Message IDs (where set with "msgid" option) */
#ifdef PB_MSGID

#define RESPONSE_UINT64_PAYLOAD_SUMMARY_LIST_MESSAGES \


#endif

#ifdef __cplusplus
} /* extern "C" */
#endif
/* @@protoc_insertion_point(eof) */

#endif
