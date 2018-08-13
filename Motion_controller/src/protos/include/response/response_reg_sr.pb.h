/* Automatically generated nanopb header */
/* Generated by nanopb-0.3.7-dev at Mon Aug 13 10:49:45 2018. */

#ifndef PB_RESPONSEMESSAGETYPE_RESPONSE_REG_SR_PB_H_INCLUDED
#define PB_RESPONSEMESSAGETYPE_RESPONSE_REG_SR_PB_H_INCLUDED
#include <pb.h>

#include "protocal/response_header.pb.h"

#include "protocal/comm.pb.h"

#include "message_type/reg_sr.pb.h"

#include "message_type/base.pb.h"

/* @@protoc_insertion_point(includes) */
#if PB_PROTO_HEADER_VERSION != 30
#error Regenerate this file with the current version of nanopb generator.
#endif

#ifdef __cplusplus
extern "C" {
#endif

/* Struct definitions */
typedef struct _ResponseMessageType_Bool_SrRegData {
    Response_Header header;
    Comm_Property property;
    MessageType_Bool success;
    MessageType_SrRegData data;
/* @@protoc_insertion_point(struct:ResponseMessageType_Bool_SrRegData) */
} ResponseMessageType_Bool_SrRegData;

/* Default values for struct fields */

/* Initializer values for message structs */
#define ResponseMessageType_Bool_SrRegData_init_default {Response_Header_init_default, Comm_Property_init_default, MessageType_Bool_init_default, MessageType_SrRegData_init_default}
#define ResponseMessageType_Bool_SrRegData_init_zero {Response_Header_init_zero, Comm_Property_init_zero, MessageType_Bool_init_zero, MessageType_SrRegData_init_zero}

/* Field tags (for use in manual encoding/decoding) */
#define ResponseMessageType_Bool_SrRegData_header_tag 1
#define ResponseMessageType_Bool_SrRegData_property_tag 2
#define ResponseMessageType_Bool_SrRegData_success_tag 3
#define ResponseMessageType_Bool_SrRegData_data_tag 4

/* Struct field encoding specification for nanopb */
extern const pb_field_t ResponseMessageType_Bool_SrRegData_fields[5];

/* Maximum encoded size of messages (where known) */
#define ResponseMessageType_Bool_SrRegData_size  607

/* Message IDs (where set with "msgid" option) */
#ifdef PB_MSGID

#define RESPONSE_REG_SR_MESSAGES \


#endif

#ifdef __cplusplus
} /* extern "C" */
#endif
/* @@protoc_insertion_point(eof) */

#endif
