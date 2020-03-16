/* Automatically generated nanopb constant definitions */
/* Generated by nanopb-0.3.7-dev at Thu Jul 11 16:28:39 2019. */

#include "reg_pr.pb.h"

/* @@protoc_insertion_point(includes) */
#if PB_PROTO_HEADER_VERSION != 30
#error Regenerate this file with the current version of nanopb generator.
#endif



const pb_field_t MessageType_PrRegData_fields[8] = {
    PB_FIELD(  1, INT32   , REQUIRED, STATIC  , FIRST, MessageType_PrRegData, id, id, 0),
    PB_FIELD(  2, STRING  , REQUIRED, STATIC  , OTHER, MessageType_PrRegData, name, id, 0),
    PB_FIELD(  3, STRING  , REQUIRED, STATIC  , OTHER, MessageType_PrRegData, comment, name, 0),
    PB_FIELD(  4, INT32   , REQUIRED, STATIC  , OTHER, MessageType_PrRegData, group_id, comment, 0),
    PB_FIELD(  5, INT32   , REQUIRED, STATIC  , OTHER, MessageType_PrRegData, pos_type, group_id, 0),
    PB_FIELD(  6, MESSAGE , REQUIRED, STATIC  , OTHER, MessageType_PrRegData, pos, pos_type, &MessageType_DoubleList_fields),
    PB_FIELD(  7, MESSAGE , REQUIRED, STATIC  , OTHER, MessageType_PrRegData, posture, pos, &MessageType_Posture_fields),
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
PB_STATIC_ASSERT((pb_membersize(MessageType_PrRegData, pos) < 65536 && pb_membersize(MessageType_PrRegData, posture) < 65536), YOU_MUST_DEFINE_PB_FIELD_32BIT_FOR_MESSAGES_MessageType_PrRegData)
#endif

#if !defined(PB_FIELD_16BIT) && !defined(PB_FIELD_32BIT)
#error Field descriptor for MessageType_PrRegData.comment is too large. Define PB_FIELD_16BIT to fix this.
#endif


/* @@protoc_insertion_point(eof) */
