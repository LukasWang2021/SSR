/* Automatically generated nanopb constant definitions */
/* Generated by nanopb-0.3.7 at Sun Sep 29 18:18:26 2019. */

#include "publish_table.pb.h"

/* @@protoc_insertion_point(includes) */
#if PB_PROTO_HEADER_VERSION != 30
#error Regenerate this file with the current version of nanopb generator.
#endif



const pb_field_t MessageType_PublishTableElement_fields[4] = {
    PB_FIELD(  1, STRING  , REQUIRED, STATIC  , FIRST, MessageType_PublishTableElement, path, path, 0),
    PB_FIELD(  2, UINT32  , REQUIRED, STATIC  , OTHER, MessageType_PublishTableElement, hash, path, 0),
    PB_FIELD(  3, STRING  , REQUIRED, STATIC  , OTHER, MessageType_PublishTableElement, message_type, hash, 0),
    PB_LAST_FIELD
};

const pb_field_t MessageType_PublishTable_fields[2] = {
    PB_FIELD(  1, MESSAGE , REPEATED, STATIC  , FIRST, MessageType_PublishTable, element, element, &MessageType_PublishTableElement_fields),
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
PB_STATIC_ASSERT((pb_membersize(MessageType_PublishTable, element[0]) < 65536), YOU_MUST_DEFINE_PB_FIELD_32BIT_FOR_MESSAGES_MessageType_PublishTableElement_MessageType_PublishTable)
#endif

#if !defined(PB_FIELD_16BIT) && !defined(PB_FIELD_32BIT)
#error Field descriptor for MessageType_PublishTableElement.message_type is too large. Define PB_FIELD_16BIT to fix this.
#endif


/* @@protoc_insertion_point(eof) */
