/* Automatically generated nanopb header */
/* Generated by nanopb-0.3.7-dev at Wed Jan  2 15:26:38 2019. */

#ifndef PB_MESSAGETYPE_MODBUS_CLIENT_CONFIG_PARAMS_PB_H_INCLUDED
#define PB_MESSAGETYPE_MODBUS_CLIENT_CONFIG_PARAMS_PB_H_INCLUDED
#include <pb.h>

#include "message_type/base.pb.h"

#include "message_type/modbus_function.pb.h"

#include "message_type/modbus_client_start_info.pb.h"

/* @@protoc_insertion_point(includes) */
#if PB_PROTO_HEADER_VERSION != 30
#error Regenerate this file with the current version of nanopb generator.
#endif

#ifdef __cplusplus
extern "C" {
#endif

/* Struct definitions */
typedef struct _MessageType_ModbusClientConfigParams {
    bool is_enable;
    MessageType_ModbusAllFucntionAddrInfo function_addr_info;
    MessageType_ModbusClientStartInfo start_info;
/* @@protoc_insertion_point(struct:MessageType_ModbusClientConfigParams) */
} MessageType_ModbusClientConfigParams;

/* Default values for struct fields */

/* Initializer values for message structs */
#define MessageType_ModbusClientConfigParams_init_default {0, MessageType_ModbusAllFucntionAddrInfo_init_default, MessageType_ModbusClientStartInfo_init_default}
#define MessageType_ModbusClientConfigParams_init_zero {0, MessageType_ModbusAllFucntionAddrInfo_init_zero, MessageType_ModbusClientStartInfo_init_zero}

/* Field tags (for use in manual encoding/decoding) */
#define MessageType_ModbusClientConfigParams_is_enable_tag 1
#define MessageType_ModbusClientConfigParams_function_addr_info_tag 2
#define MessageType_ModbusClientConfigParams_start_info_tag 3

/* Struct field encoding specification for nanopb */
extern const pb_field_t MessageType_ModbusClientConfigParams_fields[4];

/* Maximum encoded size of messages (where known) */
#define MessageType_ModbusClientConfigParams_size 409

/* Message IDs (where set with "msgid" option) */
#ifdef PB_MSGID

#define MODBUS_CLIENT_CONFIG_PARAMS_MESSAGES \


#endif

#ifdef __cplusplus
} /* extern "C" */
#endif
/* @@protoc_insertion_point(eof) */

#endif
