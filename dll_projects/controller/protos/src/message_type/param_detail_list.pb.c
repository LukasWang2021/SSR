/* Automatically generated nanopb constant definitions */
/* Generated by nanopb-0.3.7 at Thu Sep 19 13:40:58 2019. */

#include "param_detail_list.pb.h"

/* @@protoc_insertion_point(includes) */
#if PB_PROTO_HEADER_VERSION != 30
#error Regenerate this file with the current version of nanopb generator.
#endif



const pb_field_t MessageType_ParamDetail_fields[8] = {
    PB_FIELD(  1, INT32   , REQUIRED, STATIC  , FIRST, MessageType_ParamDetail, operation_value, operation_value, 0),
    PB_FIELD(  2, INT32   , REQUIRED, STATIC  , OTHER, MessageType_ParamDetail, default_value, operation_value, 0),
    PB_FIELD(  3, INT32   , REQUIRED, STATIC  , OTHER, MessageType_ParamDetail, upper_limit_value, default_value, 0),
    PB_FIELD(  4, INT32   , REQUIRED, STATIC  , OTHER, MessageType_ParamDetail, lower_limit_value, upper_limit_value, 0),
    PB_FIELD(  5, INT32   , REQUIRED, STATIC  , OTHER, MessageType_ParamDetail, attr, lower_limit_value, 0),
    PB_FIELD(  6, INT32   , REQUIRED, STATIC  , OTHER, MessageType_ParamDetail, validity, attr, 0),
    PB_FIELD(  7, STRING  , REQUIRED, STATIC  , OTHER, MessageType_ParamDetail, unit, validity, 0),
    PB_LAST_FIELD
};

const pb_field_t MessageType_ParamDetailList_fields[2] = {
    PB_FIELD(  1, MESSAGE , REPEATED, STATIC  , FIRST, MessageType_ParamDetailList, data, data, &MessageType_ParamDetail_fields),
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
PB_STATIC_ASSERT((pb_membersize(MessageType_ParamDetailList, data[0]) < 65536), YOU_MUST_DEFINE_PB_FIELD_32BIT_FOR_MESSAGES_MessageType_ParamDetail_MessageType_ParamDetailList)
#endif

#if !defined(PB_FIELD_16BIT) && !defined(PB_FIELD_32BIT)
#error Field descriptor for MessageType_ParamDetailList.data is too large. Define PB_FIELD_16BIT to fix this.
#endif


/* @@protoc_insertion_point(eof) */
