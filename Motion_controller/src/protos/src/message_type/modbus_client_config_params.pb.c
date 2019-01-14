/* Automatically generated nanopb constant definitions */
/* Generated by nanopb-0.3.7-dev at Wed Jan  2 15:26:38 2019. */

#include "modbus_client_config_params.pb.h"

/* @@protoc_insertion_point(includes) */
#if PB_PROTO_HEADER_VERSION != 30
#error Regenerate this file with the current version of nanopb generator.
#endif



const pb_field_t MessageType_ModbusClientConfigParams_fields[4] = {
    PB_FIELD(  1, BOOL    , REQUIRED, STATIC  , FIRST, MessageType_ModbusClientConfigParams, is_enable, is_enable, 0),
    PB_FIELD(  2, MESSAGE , REQUIRED, STATIC  , OTHER, MessageType_ModbusClientConfigParams, function_addr_info, is_enable, &MessageType_ModbusAllFucntionAddrInfo_fields),
    PB_FIELD(  3, MESSAGE , REQUIRED, STATIC  , OTHER, MessageType_ModbusClientConfigParams, start_info, function_addr_info, &MessageType_ModbusClientStartInfo_fields),
    PB_LAST_FIELD
};


/* Check that field information fits in pb_field_t */
#if !defined(PB_FIELD_32BIT)
/* If you get an error here, it means that you need to define PB_FIELD_32BIT
 * compile-time option. You can do that in pb.h or on compiler command line.
 * 
 * The reason you need to do this is that some of your messages contain tag
 * numbers or field sizes that are larger than what can fit in 8 or 16 bit
 * field descriptors.
 */
PB_STATIC_ASSERT((pb_membersize(MessageType_ModbusClientConfigParams, function_addr_info) < 65536 && pb_membersize(MessageType_ModbusClientConfigParams, start_info) < 65536), YOU_MUST_DEFINE_PB_FIELD_32BIT_FOR_MESSAGES_MessageType_ModbusClientConfigParams)
#endif

#if !defined(PB_FIELD_16BIT) && !defined(PB_FIELD_32BIT)
/* If you get an error here, it means that you need to define PB_FIELD_16BIT
 * compile-time option. You can do that in pb.h or on compiler command line.
 * 
 * The reason you need to do this is that some of your messages contain tag
 * numbers or field sizes that are larger than what can fit in the default
 * 8 bit descriptors.
 */
PB_STATIC_ASSERT((pb_membersize(MessageType_ModbusClientConfigParams, function_addr_info) < 256 && pb_membersize(MessageType_ModbusClientConfigParams, start_info) < 256), YOU_MUST_DEFINE_PB_FIELD_16BIT_FOR_MESSAGES_MessageType_ModbusClientConfigParams)
#endif


/* @@protoc_insertion_point(eof) */
