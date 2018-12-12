/* Automatically generated nanopb constant definitions */
/* Generated by nanopb-0.3.7-dev at Fri Dec  7 10:47:08 2018. */

#include "modbus_client_summaryinfo.pb.h"

/* @@protoc_insertion_point(includes) */
#if PB_PROTO_HEADER_VERSION != 30
#error Regenerate this file with the current version of nanopb generator.
#endif



const pb_field_t MessageType_ModbusClientSummaryInfo_fields[3] = {
    PB_FIELD(  1, INT32   , REQUIRED, STATIC  , FIRST, MessageType_ModbusClientSummaryInfo, id, id, 0),
    PB_FIELD(  2, STRING  , REQUIRED, STATIC  , OTHER, MessageType_ModbusClientSummaryInfo, name, id, 0),
    PB_LAST_FIELD
};

const pb_field_t MessageType_ModbusClientSummaryInfoList_fields[2] = {
    PB_FIELD(  1, MESSAGE , REPEATED, STATIC  , FIRST, MessageType_ModbusClientSummaryInfoList, info, info, &MessageType_ModbusClientSummaryInfo_fields),
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
PB_STATIC_ASSERT((pb_membersize(MessageType_ModbusClientSummaryInfoList, info[0]) < 65536), YOU_MUST_DEFINE_PB_FIELD_32BIT_FOR_MESSAGES_MessageType_ModbusClientSummaryInfo_MessageType_ModbusClientSummaryInfoList)
#endif

#if !defined(PB_FIELD_16BIT) && !defined(PB_FIELD_32BIT)
/* If you get an error here, it means that you need to define PB_FIELD_16BIT
 * compile-time option. You can do that in pb.h or on compiler command line.
 * 
 * The reason you need to do this is that some of your messages contain tag
 * numbers or field sizes that are larger than what can fit in the default
 * 8 bit descriptors.
 */
PB_STATIC_ASSERT((pb_membersize(MessageType_ModbusClientSummaryInfoList, info[0]) < 256), YOU_MUST_DEFINE_PB_FIELD_16BIT_FOR_MESSAGES_MessageType_ModbusClientSummaryInfo_MessageType_ModbusClientSummaryInfoList)
#endif


/* @@protoc_insertion_point(eof) */
