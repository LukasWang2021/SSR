/* Automatically generated nanopb header */
/* Generated by nanopb-0.3.7-dev at Wed Jan  2 16:06:10 2019. */

#ifndef PB_REQUESTMESSAGETYPE_REQUEST_MODBUS_CLIENT_START_INFO_PB_H_INCLUDED
#define PB_REQUESTMESSAGETYPE_REQUEST_MODBUS_CLIENT_START_INFO_PB_H_INCLUDED
#include <pb.h>

#include "protocal/request_header.pb.h"

#include "protocal/comm.pb.h"

#include "message_type/base.pb.h"

#include "message_type/modbus_client_start_info.pb.h"

/* @@protoc_insertion_point(includes) */
#if PB_PROTO_HEADER_VERSION != 30
#error Regenerate this file with the current version of nanopb generator.
#endif

#ifdef __cplusplus
extern "C" {
#endif

/* Struct definitions */
typedef struct _RequestMessageType_ModbusClientStartInfo {
    Request_Header header;
    Comm_Property property;
    MessageType_ModbusClientStartInfo data;
/* @@protoc_insertion_point(struct:RequestMessageType_ModbusClientStartInfo) */
} RequestMessageType_ModbusClientStartInfo;

/* Default values for struct fields */

/* Initializer values for message structs */
#define RequestMessageType_ModbusClientStartInfo_init_default {Request_Header_init_default, Comm_Property_init_default, MessageType_ModbusClientStartInfo_init_default}
#define RequestMessageType_ModbusClientStartInfo_init_zero {Request_Header_init_zero, Comm_Property_init_zero, MessageType_ModbusClientStartInfo_init_zero}

/* Field tags (for use in manual encoding/decoding) */
#define RequestMessageType_ModbusClientStartInfo_header_tag 1
#define RequestMessageType_ModbusClientStartInfo_property_tag 2
#define RequestMessageType_ModbusClientStartInfo_data_tag 3

/* Struct field encoding specification for nanopb */
extern const pb_field_t RequestMessageType_ModbusClientStartInfo_fields[4];

/* Maximum encoded size of messages (where known) */
#define RequestMessageType_ModbusClientStartInfo_size 326

/* Message IDs (where set with "msgid" option) */
#ifdef PB_MSGID

#define REQUEST_MODBUS_CLIENT_START_INFO_MESSAGES \


#endif

#ifdef __cplusplus
} /* extern "C" */
#endif
/* @@protoc_insertion_point(eof) */

#endif
