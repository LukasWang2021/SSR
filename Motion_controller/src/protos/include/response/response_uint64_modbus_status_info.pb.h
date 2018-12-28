/* Automatically generated nanopb header */
/* Generated by nanopb-0.3.7-dev at Fri Dec 28 10:08:35 2018. */

#ifndef PB_RESPONSEMESSAGETYPE_RESPONSE_UINT64_MODBUS_STATUS_INFO_PB_H_INCLUDED
#define PB_RESPONSEMESSAGETYPE_RESPONSE_UINT64_MODBUS_STATUS_INFO_PB_H_INCLUDED
#include <pb.h>

#include "protocal/response_header.pb.h"

#include "protocal/comm.pb.h"

#include "message_type/modbus_status_info.pb.h"

#include "message_type/base.pb.h"

/* @@protoc_insertion_point(includes) */
#if PB_PROTO_HEADER_VERSION != 30
#error Regenerate this file with the current version of nanopb generator.
#endif

#ifdef __cplusplus
extern "C" {
#endif

/* Struct definitions */
typedef struct _ResponseMessageType_Uint64_ModbusStatusInfo {
    Response_Header header;
    Comm_Property property;
    MessageType_Uint64 error_code;
    MessageType_ModbusStatusInfo data;
/* @@protoc_insertion_point(struct:ResponseMessageType_Uint64_ModbusStatusInfo) */
} ResponseMessageType_Uint64_ModbusStatusInfo;

/* Default values for struct fields */

/* Initializer values for message structs */
#define ResponseMessageType_Uint64_ModbusStatusInfo_init_default {Response_Header_init_default, Comm_Property_init_default, MessageType_Uint64_init_default, MessageType_ModbusStatusInfo_init_default}
#define ResponseMessageType_Uint64_ModbusStatusInfo_init_zero {Response_Header_init_zero, Comm_Property_init_zero, MessageType_Uint64_init_zero, MessageType_ModbusStatusInfo_init_zero}

/* Field tags (for use in manual encoding/decoding) */
#define ResponseMessageType_Uint64_ModbusStatusInfo_header_tag 1
#define ResponseMessageType_Uint64_ModbusStatusInfo_property_tag 2
#define ResponseMessageType_Uint64_ModbusStatusInfo_error_code_tag 3
#define ResponseMessageType_Uint64_ModbusStatusInfo_data_tag 4

/* Struct field encoding specification for nanopb */
extern const pb_field_t ResponseMessageType_Uint64_ModbusStatusInfo_fields[5];

/* Maximum encoded size of messages (where known) */
#define ResponseMessageType_Uint64_ModbusStatusInfo_size 2125

/* Message IDs (where set with "msgid" option) */
#ifdef PB_MSGID

#define RESPONSE_UINT64_MODBUS_STATUS_INFO_MESSAGES \


#endif

#ifdef __cplusplus
} /* extern "C" */
#endif
/* @@protoc_insertion_point(eof) */

#endif
