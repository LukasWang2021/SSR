/* Automatically generated nanopb constant definitions */
/* Generated by nanopb-0.3.7 at Mon Jul 19 13:49:17 2021. */

#include "reg_r.pb.h"

/* @@protoc_insertion_point(includes) */
#if PB_PROTO_HEADER_VERSION != 30
#error Regenerate this file with the current version of nanopb generator.
#endif



const pb_field_t MessageType_RRegData_fields[5] = {
    PB_FIELD(  1, INT32   , REQUIRED, STATIC  , FIRST, MessageType_RRegData, id, id, 0),
    PB_FIELD(  2, STRING  , REQUIRED, STATIC  , OTHER, MessageType_RRegData, name, id, 0),
    PB_FIELD(  3, STRING  , REQUIRED, STATIC  , OTHER, MessageType_RRegData, comment, name, 0),
    PB_FIELD(  4, DOUBLE  , REQUIRED, STATIC  , OTHER, MessageType_RRegData, value, comment, 0),
    PB_LAST_FIELD
};


/* Check that field information fits in pb_field_t */
#if !defined(PB_FIELD_16BIT) && !defined(PB_FIELD_32BIT)
#error Field descriptor for MessageType_RRegData.comment is too large. Define PB_FIELD_16BIT to fix this.
#endif


/* On some platforms (such as AVR), double is really float.
 * These are not directly supported by nanopb, but see example_avr_double.
 * To get rid of this error, remove any double fields from your .proto.
 */
PB_STATIC_ASSERT(sizeof(double) == 8, DOUBLE_MUST_BE_8_BYTES)

/* @@protoc_insertion_point(eof) */
