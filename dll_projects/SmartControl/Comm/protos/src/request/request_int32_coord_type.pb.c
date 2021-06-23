/* Automatically generated nanopb constant definitions */
/* Generated by nanopb-0.3.7 at Thu Sep 19 13:40:58 2019. */

#include "request_int32_coord_type.pb.h"

/* @@protoc_insertion_point(includes) */
#if PB_PROTO_HEADER_VERSION != 30
#error Regenerate this file with the current version of nanopb generator.
#endif



const pb_field_t RequestMessageType_Int32_CoordType_fields[5] = {
    PB_FIELD(  1, MESSAGE , REQUIRED, STATIC  , FIRST, RequestMessageType_Int32_CoordType, header, header, &Request_Header_fields),
    PB_FIELD(  2, MESSAGE , REQUIRED, STATIC  , OTHER, RequestMessageType_Int32_CoordType, property, header, &Comm_Property_fields),
    PB_FIELD(  3, MESSAGE , REQUIRED, STATIC  , OTHER, RequestMessageType_Int32_CoordType, data1, property, &MessageType_Int32_fields),
    PB_FIELD(  4, UENUM   , REQUIRED, STATIC  , OTHER, RequestMessageType_Int32_CoordType, data2, data1, 0),
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
PB_STATIC_ASSERT((pb_membersize(RequestMessageType_Int32_CoordType, header) < 65536 && pb_membersize(RequestMessageType_Int32_CoordType, property) < 65536 && pb_membersize(RequestMessageType_Int32_CoordType, data1) < 65536), YOU_MUST_DEFINE_PB_FIELD_32BIT_FOR_MESSAGES_RequestMessageType_Int32_CoordType)
#endif

#if !defined(PB_FIELD_16BIT) && !defined(PB_FIELD_32BIT)
/* If you get an error here, it means that you need to define PB_FIELD_16BIT
 * compile-time option. You can do that in pb.h or on compiler command line.
 * 
 * The reason you need to do this is that some of your messages contain tag
 * numbers or field sizes that are larger than what can fit in the default
 * 8 bit descriptors.
 */
PB_STATIC_ASSERT((pb_membersize(RequestMessageType_Int32_CoordType, header) < 256 && pb_membersize(RequestMessageType_Int32_CoordType, property) < 256 && pb_membersize(RequestMessageType_Int32_CoordType, data1) < 256), YOU_MUST_DEFINE_PB_FIELD_16BIT_FOR_MESSAGES_RequestMessageType_Int32_CoordType)
#endif


/* @@protoc_insertion_point(eof) */
