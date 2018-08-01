/* Automatically generated nanopb constant definitions */
/* Generated by nanopb-0.3.7-dev at Mon Jul 30 15:41:17 2018. */

#include "response_register_r.pb.h"

/* @@protoc_insertion_point(includes) */
#if PB_PROTO_HEADER_VERSION != 30
#error Regenerate this file with the current version of nanopb generator.
#endif



const pb_field_t ResponseMessageType_RegisterR_fields[4] = {
    PB_FIELD(  1, MESSAGE , REQUIRED, STATIC  , FIRST, ResponseMessageType_RegisterR, header, header, &Response_Header_fields),
    PB_FIELD(  2, MESSAGE , REQUIRED, STATIC  , OTHER, ResponseMessageType_RegisterR, property, header, &Comm_Property_fields),
    PB_FIELD(  3, MESSAGE , REQUIRED, STATIC  , OTHER, ResponseMessageType_RegisterR, data, property, &MessageType_RegisterR_fields),
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
PB_STATIC_ASSERT((pb_membersize(ResponseMessageType_RegisterR, header) < 65536 && pb_membersize(ResponseMessageType_RegisterR, property) < 65536 && pb_membersize(ResponseMessageType_RegisterR, data) < 65536), YOU_MUST_DEFINE_PB_FIELD_32BIT_FOR_MESSAGES_ResponseMessageType_RegisterR)
#endif

#if !defined(PB_FIELD_16BIT) && !defined(PB_FIELD_32BIT)
/* If you get an error here, it means that you need to define PB_FIELD_16BIT
 * compile-time option. You can do that in pb.h or on compiler command line.
 * 
 * The reason you need to do this is that some of your messages contain tag
 * numbers or field sizes that are larger than what can fit in the default
 * 8 bit descriptors.
 */
PB_STATIC_ASSERT((pb_membersize(ResponseMessageType_RegisterR, header) < 256 && pb_membersize(ResponseMessageType_RegisterR, property) < 256 && pb_membersize(ResponseMessageType_RegisterR, data) < 256), YOU_MUST_DEFINE_PB_FIELD_16BIT_FOR_MESSAGES_ResponseMessageType_RegisterR)
#endif


/* @@protoc_insertion_point(eof) */
