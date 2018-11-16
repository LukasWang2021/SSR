/* Automatically generated nanopb constant definitions */
/* Generated by nanopb-0.3.7-dev at Fri Nov 16 13:40:20 2018. */

#include "request_int32list_double.pb.h"

/* @@protoc_insertion_point(includes) */
#if PB_PROTO_HEADER_VERSION != 30
#error Regenerate this file with the current version of nanopb generator.
#endif



const pb_field_t RequestMessageType_Int32List_Double_fields[5] = {
    PB_FIELD(  1, MESSAGE , REQUIRED, STATIC  , FIRST, RequestMessageType_Int32List_Double, header, header, &Request_Header_fields),
    PB_FIELD(  2, MESSAGE , REQUIRED, STATIC  , OTHER, RequestMessageType_Int32List_Double, property, header, &Comm_Property_fields),
    PB_FIELD(  3, MESSAGE , REQUIRED, STATIC  , OTHER, RequestMessageType_Int32List_Double, data1, property, &MessageType_Int32List_fields),
    PB_FIELD(  4, MESSAGE , REQUIRED, STATIC  , OTHER, RequestMessageType_Int32List_Double, data2, data1, &MessageType_Double_fields),
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
PB_STATIC_ASSERT((pb_membersize(RequestMessageType_Int32List_Double, header) < 65536 && pb_membersize(RequestMessageType_Int32List_Double, property) < 65536 && pb_membersize(RequestMessageType_Int32List_Double, data1) < 65536 && pb_membersize(RequestMessageType_Int32List_Double, data2) < 65536), YOU_MUST_DEFINE_PB_FIELD_32BIT_FOR_MESSAGES_RequestMessageType_Int32List_Double)
#endif

#if !defined(PB_FIELD_16BIT) && !defined(PB_FIELD_32BIT)
/* If you get an error here, it means that you need to define PB_FIELD_16BIT
 * compile-time option. You can do that in pb.h or on compiler command line.
 * 
 * The reason you need to do this is that some of your messages contain tag
 * numbers or field sizes that are larger than what can fit in the default
 * 8 bit descriptors.
 */
PB_STATIC_ASSERT((pb_membersize(RequestMessageType_Int32List_Double, header) < 256 && pb_membersize(RequestMessageType_Int32List_Double, property) < 256 && pb_membersize(RequestMessageType_Int32List_Double, data1) < 256 && pb_membersize(RequestMessageType_Int32List_Double, data2) < 256), YOU_MUST_DEFINE_PB_FIELD_16BIT_FOR_MESSAGES_RequestMessageType_Int32List_Double)
#endif


/* @@protoc_insertion_point(eof) */
