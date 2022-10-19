/* Automatically generated nanopb constant definitions */
/* Generated by nanopb-0.3.7 at Tue Jul 20 10:18:31 2021. */

#include "response_reg_base_summary.pb.h"

/* @@protoc_insertion_point(includes) */
#if PB_PROTO_HEADER_VERSION != 30
#error Regenerate this file with the current version of nanopb generator.
#endif



const pb_field_t ResponseMessageType_Uint64_BaseRegSummaryList_fields[5] = {
    PB_FIELD(  1, MESSAGE , REQUIRED, STATIC  , FIRST, ResponseMessageType_Uint64_BaseRegSummaryList, header, header, &Response_Header_fields),
    PB_FIELD(  2, MESSAGE , REQUIRED, STATIC  , OTHER, ResponseMessageType_Uint64_BaseRegSummaryList, property, header, &Comm_Property_fields),
    PB_FIELD(  3, MESSAGE , REQUIRED, STATIC  , OTHER, ResponseMessageType_Uint64_BaseRegSummaryList, error_code, property, &MessageType_Uint64_fields),
    PB_FIELD(  4, MESSAGE , REQUIRED, STATIC  , OTHER, ResponseMessageType_Uint64_BaseRegSummaryList, data, error_code, &MessageType_BaseRegSummaryList_fields),
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
PB_STATIC_ASSERT((pb_membersize(ResponseMessageType_Uint64_BaseRegSummaryList, header) < 65536 && pb_membersize(ResponseMessageType_Uint64_BaseRegSummaryList, property) < 65536 && pb_membersize(ResponseMessageType_Uint64_BaseRegSummaryList, error_code) < 65536 && pb_membersize(ResponseMessageType_Uint64_BaseRegSummaryList, data) < 65536), YOU_MUST_DEFINE_PB_FIELD_32BIT_FOR_MESSAGES_ResponseMessageType_Uint64_BaseRegSummaryList)
#endif

#if !defined(PB_FIELD_16BIT) && !defined(PB_FIELD_32BIT)
/* If you get an error here, it means that you need to define PB_FIELD_16BIT
 * compile-time option. You can do that in pb.h or on compiler command line.
 * 
 * The reason you need to do this is that some of your messages contain tag
 * numbers or field sizes that are larger than what can fit in the default
 * 8 bit descriptors.
 */
PB_STATIC_ASSERT((pb_membersize(ResponseMessageType_Uint64_BaseRegSummaryList, header) < 256 && pb_membersize(ResponseMessageType_Uint64_BaseRegSummaryList, property) < 256 && pb_membersize(ResponseMessageType_Uint64_BaseRegSummaryList, error_code) < 256 && pb_membersize(ResponseMessageType_Uint64_BaseRegSummaryList, data) < 256), YOU_MUST_DEFINE_PB_FIELD_16BIT_FOR_MESSAGES_ResponseMessageType_Uint64_BaseRegSummaryList)
#endif


/* @@protoc_insertion_point(eof) */
