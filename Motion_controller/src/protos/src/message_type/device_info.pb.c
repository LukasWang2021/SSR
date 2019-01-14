/* Automatically generated nanopb constant definitions */
/* Generated by nanopb-0.3.7-dev at Tue Aug 14 16:41:11 2018. */

#include "device_info.pb.h"

/* @@protoc_insertion_point(includes) */
#if PB_PROTO_HEADER_VERSION != 30
#error Regenerate this file with the current version of nanopb generator.
#endif



const pb_field_t MessageType_DeviceInfo_fields[5] = {
    PB_FIELD(  1, INT32   , REQUIRED, STATIC  , FIRST, MessageType_DeviceInfo, index, index, 0),
    PB_FIELD(  2, INT32   , REQUIRED, STATIC  , OTHER, MessageType_DeviceInfo, address, index, 0),
    PB_FIELD(  3, INT32   , REQUIRED, STATIC  , OTHER, MessageType_DeviceInfo, type, address, 0),
    PB_FIELD(  4, BOOL    , REQUIRED, STATIC  , OTHER, MessageType_DeviceInfo, is_valid, type, 0),
    PB_LAST_FIELD
};

const pb_field_t MessageType_DeviceInfoList_fields[2] = {
    PB_FIELD(  1, MESSAGE , REPEATED, STATIC  , FIRST, MessageType_DeviceInfoList, device_info, device_info, &MessageType_DeviceInfo_fields),
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
PB_STATIC_ASSERT((pb_membersize(MessageType_DeviceInfoList, device_info[0]) < 65536), YOU_MUST_DEFINE_PB_FIELD_32BIT_FOR_MESSAGES_MessageType_DeviceInfo_MessageType_DeviceInfoList)
#endif

#if !defined(PB_FIELD_16BIT) && !defined(PB_FIELD_32BIT)
#error Field descriptor for MessageType_DeviceInfoList.device_info is too large. Define PB_FIELD_16BIT to fix this.
#endif


/* @@protoc_insertion_point(eof) */