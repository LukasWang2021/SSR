/* Automatically generated nanopb header */
/* Generated by nanopb-0.3.7 at Thu Sep 19 16:40:41 2019. */

#ifndef PB_MESSAGETYPE_PUBLISH_TABLE_PB_H_INCLUDED
#define PB_MESSAGETYPE_PUBLISH_TABLE_PB_H_INCLUDED
#include <pb.h>

/* @@protoc_insertion_point(includes) */
#if PB_PROTO_HEADER_VERSION != 30
#error Regenerate this file with the current version of nanopb generator.
#endif

#ifdef __cplusplus
extern "C" {
#endif

/* Struct definitions */
typedef struct _MessageType_PublishTableElement {
    char path[256];
    uint32_t hash;
    char message_type[256];
/* @@protoc_insertion_point(struct:MessageType_PublishTableElement) */
} MessageType_PublishTableElement;

typedef struct _MessageType_PublishTable {
    pb_size_t element_count;
    MessageType_PublishTableElement element[128];
/* @@protoc_insertion_point(struct:MessageType_PublishTable) */
} MessageType_PublishTable;

/* Default values for struct fields */

/* Initializer values for message structs */
#define MessageType_PublishTableElement_init_default {"", 0, ""}
#define MessageType_PublishTable_init_default    {0, {MessageType_PublishTableElement_init_default, MessageType_PublishTableElement_init_default, MessageType_PublishTableElement_init_default, MessageType_PublishTableElement_init_default, MessageType_PublishTableElement_init_default, MessageType_PublishTableElement_init_default, MessageType_PublishTableElement_init_default, MessageType_PublishTableElement_init_default, MessageType_PublishTableElement_init_default, MessageType_PublishTableElement_init_default, MessageType_PublishTableElement_init_default, MessageType_PublishTableElement_init_default, MessageType_PublishTableElement_init_default, MessageType_PublishTableElement_init_default, MessageType_PublishTableElement_init_default, MessageType_PublishTableElement_init_default, MessageType_PublishTableElement_init_default, MessageType_PublishTableElement_init_default, MessageType_PublishTableElement_init_default, MessageType_PublishTableElement_init_default, MessageType_PublishTableElement_init_default, MessageType_PublishTableElement_init_default, MessageType_PublishTableElement_init_default, MessageType_PublishTableElement_init_default, MessageType_PublishTableElement_init_default, MessageType_PublishTableElement_init_default, MessageType_PublishTableElement_init_default, MessageType_PublishTableElement_init_default, MessageType_PublishTableElement_init_default, MessageType_PublishTableElement_init_default, MessageType_PublishTableElement_init_default, MessageType_PublishTableElement_init_default, MessageType_PublishTableElement_init_default, MessageType_PublishTableElement_init_default, MessageType_PublishTableElement_init_default, MessageType_PublishTableElement_init_default, MessageType_PublishTableElement_init_default, MessageType_PublishTableElement_init_default, MessageType_PublishTableElement_init_default, MessageType_PublishTableElement_init_default, MessageType_PublishTableElement_init_default, MessageType_PublishTableElement_init_default, MessageType_PublishTableElement_init_default, MessageType_PublishTableElement_init_default, MessageType_PublishTableElement_init_default, MessageType_PublishTableElement_init_default, MessageType_PublishTableElement_init_default, MessageType_PublishTableElement_init_default, MessageType_PublishTableElement_init_default, MessageType_PublishTableElement_init_default, MessageType_PublishTableElement_init_default, MessageType_PublishTableElement_init_default, MessageType_PublishTableElement_init_default, MessageType_PublishTableElement_init_default, MessageType_PublishTableElement_init_default, MessageType_PublishTableElement_init_default, MessageType_PublishTableElement_init_default, MessageType_PublishTableElement_init_default, MessageType_PublishTableElement_init_default, MessageType_PublishTableElement_init_default, MessageType_PublishTableElement_init_default, MessageType_PublishTableElement_init_default, MessageType_PublishTableElement_init_default, MessageType_PublishTableElement_init_default, MessageType_PublishTableElement_init_default, MessageType_PublishTableElement_init_default, MessageType_PublishTableElement_init_default, MessageType_PublishTableElement_init_default, MessageType_PublishTableElement_init_default, MessageType_PublishTableElement_init_default, MessageType_PublishTableElement_init_default, MessageType_PublishTableElement_init_default, MessageType_PublishTableElement_init_default, MessageType_PublishTableElement_init_default, MessageType_PublishTableElement_init_default, MessageType_PublishTableElement_init_default, MessageType_PublishTableElement_init_default, MessageType_PublishTableElement_init_default, MessageType_PublishTableElement_init_default, MessageType_PublishTableElement_init_default, MessageType_PublishTableElement_init_default, MessageType_PublishTableElement_init_default, MessageType_PublishTableElement_init_default, MessageType_PublishTableElement_init_default, MessageType_PublishTableElement_init_default, MessageType_PublishTableElement_init_default, MessageType_PublishTableElement_init_default, MessageType_PublishTableElement_init_default, MessageType_PublishTableElement_init_default, MessageType_PublishTableElement_init_default, MessageType_PublishTableElement_init_default, MessageType_PublishTableElement_init_default, MessageType_PublishTableElement_init_default, MessageType_PublishTableElement_init_default, MessageType_PublishTableElement_init_default, MessageType_PublishTableElement_init_default, MessageType_PublishTableElement_init_default, MessageType_PublishTableElement_init_default, MessageType_PublishTableElement_init_default, MessageType_PublishTableElement_init_default, MessageType_PublishTableElement_init_default, MessageType_PublishTableElement_init_default, MessageType_PublishTableElement_init_default, MessageType_PublishTableElement_init_default, MessageType_PublishTableElement_init_default, MessageType_PublishTableElement_init_default, MessageType_PublishTableElement_init_default, MessageType_PublishTableElement_init_default, MessageType_PublishTableElement_init_default, MessageType_PublishTableElement_init_default, MessageType_PublishTableElement_init_default, MessageType_PublishTableElement_init_default, MessageType_PublishTableElement_init_default, MessageType_PublishTableElement_init_default, MessageType_PublishTableElement_init_default, MessageType_PublishTableElement_init_default, MessageType_PublishTableElement_init_default, MessageType_PublishTableElement_init_default, MessageType_PublishTableElement_init_default, MessageType_PublishTableElement_init_default, MessageType_PublishTableElement_init_default, MessageType_PublishTableElement_init_default, MessageType_PublishTableElement_init_default, MessageType_PublishTableElement_init_default, MessageType_PublishTableElement_init_default, MessageType_PublishTableElement_init_default, MessageType_PublishTableElement_init_default, MessageType_PublishTableElement_init_default}}
#define MessageType_PublishTableElement_init_zero {"", 0, ""}
#define MessageType_PublishTable_init_zero       {0, {MessageType_PublishTableElement_init_zero, MessageType_PublishTableElement_init_zero, MessageType_PublishTableElement_init_zero, MessageType_PublishTableElement_init_zero, MessageType_PublishTableElement_init_zero, MessageType_PublishTableElement_init_zero, MessageType_PublishTableElement_init_zero, MessageType_PublishTableElement_init_zero, MessageType_PublishTableElement_init_zero, MessageType_PublishTableElement_init_zero, MessageType_PublishTableElement_init_zero, MessageType_PublishTableElement_init_zero, MessageType_PublishTableElement_init_zero, MessageType_PublishTableElement_init_zero, MessageType_PublishTableElement_init_zero, MessageType_PublishTableElement_init_zero, MessageType_PublishTableElement_init_zero, MessageType_PublishTableElement_init_zero, MessageType_PublishTableElement_init_zero, MessageType_PublishTableElement_init_zero, MessageType_PublishTableElement_init_zero, MessageType_PublishTableElement_init_zero, MessageType_PublishTableElement_init_zero, MessageType_PublishTableElement_init_zero, MessageType_PublishTableElement_init_zero, MessageType_PublishTableElement_init_zero, MessageType_PublishTableElement_init_zero, MessageType_PublishTableElement_init_zero, MessageType_PublishTableElement_init_zero, MessageType_PublishTableElement_init_zero, MessageType_PublishTableElement_init_zero, MessageType_PublishTableElement_init_zero, MessageType_PublishTableElement_init_zero, MessageType_PublishTableElement_init_zero, MessageType_PublishTableElement_init_zero, MessageType_PublishTableElement_init_zero, MessageType_PublishTableElement_init_zero, MessageType_PublishTableElement_init_zero, MessageType_PublishTableElement_init_zero, MessageType_PublishTableElement_init_zero, MessageType_PublishTableElement_init_zero, MessageType_PublishTableElement_init_zero, MessageType_PublishTableElement_init_zero, MessageType_PublishTableElement_init_zero, MessageType_PublishTableElement_init_zero, MessageType_PublishTableElement_init_zero, MessageType_PublishTableElement_init_zero, MessageType_PublishTableElement_init_zero, MessageType_PublishTableElement_init_zero, MessageType_PublishTableElement_init_zero, MessageType_PublishTableElement_init_zero, MessageType_PublishTableElement_init_zero, MessageType_PublishTableElement_init_zero, MessageType_PublishTableElement_init_zero, MessageType_PublishTableElement_init_zero, MessageType_PublishTableElement_init_zero, MessageType_PublishTableElement_init_zero, MessageType_PublishTableElement_init_zero, MessageType_PublishTableElement_init_zero, MessageType_PublishTableElement_init_zero, MessageType_PublishTableElement_init_zero, MessageType_PublishTableElement_init_zero, MessageType_PublishTableElement_init_zero, MessageType_PublishTableElement_init_zero, MessageType_PublishTableElement_init_zero, MessageType_PublishTableElement_init_zero, MessageType_PublishTableElement_init_zero, MessageType_PublishTableElement_init_zero, MessageType_PublishTableElement_init_zero, MessageType_PublishTableElement_init_zero, MessageType_PublishTableElement_init_zero, MessageType_PublishTableElement_init_zero, MessageType_PublishTableElement_init_zero, MessageType_PublishTableElement_init_zero, MessageType_PublishTableElement_init_zero, MessageType_PublishTableElement_init_zero, MessageType_PublishTableElement_init_zero, MessageType_PublishTableElement_init_zero, MessageType_PublishTableElement_init_zero, MessageType_PublishTableElement_init_zero, MessageType_PublishTableElement_init_zero, MessageType_PublishTableElement_init_zero, MessageType_PublishTableElement_init_zero, MessageType_PublishTableElement_init_zero, MessageType_PublishTableElement_init_zero, MessageType_PublishTableElement_init_zero, MessageType_PublishTableElement_init_zero, MessageType_PublishTableElement_init_zero, MessageType_PublishTableElement_init_zero, MessageType_PublishTableElement_init_zero, MessageType_PublishTableElement_init_zero, MessageType_PublishTableElement_init_zero, MessageType_PublishTableElement_init_zero, MessageType_PublishTableElement_init_zero, MessageType_PublishTableElement_init_zero, MessageType_PublishTableElement_init_zero, MessageType_PublishTableElement_init_zero, MessageType_PublishTableElement_init_zero, MessageType_PublishTableElement_init_zero, MessageType_PublishTableElement_init_zero, MessageType_PublishTableElement_init_zero, MessageType_PublishTableElement_init_zero, MessageType_PublishTableElement_init_zero, MessageType_PublishTableElement_init_zero, MessageType_PublishTableElement_init_zero, MessageType_PublishTableElement_init_zero, MessageType_PublishTableElement_init_zero, MessageType_PublishTableElement_init_zero, MessageType_PublishTableElement_init_zero, MessageType_PublishTableElement_init_zero, MessageType_PublishTableElement_init_zero, MessageType_PublishTableElement_init_zero, MessageType_PublishTableElement_init_zero, MessageType_PublishTableElement_init_zero, MessageType_PublishTableElement_init_zero, MessageType_PublishTableElement_init_zero, MessageType_PublishTableElement_init_zero, MessageType_PublishTableElement_init_zero, MessageType_PublishTableElement_init_zero, MessageType_PublishTableElement_init_zero, MessageType_PublishTableElement_init_zero, MessageType_PublishTableElement_init_zero, MessageType_PublishTableElement_init_zero, MessageType_PublishTableElement_init_zero, MessageType_PublishTableElement_init_zero, MessageType_PublishTableElement_init_zero, MessageType_PublishTableElement_init_zero, MessageType_PublishTableElement_init_zero}}

/* Field tags (for use in manual encoding/decoding) */
#define MessageType_PublishTableElement_path_tag 1
#define MessageType_PublishTableElement_hash_tag 2
#define MessageType_PublishTableElement_message_type_tag 3
#define MessageType_PublishTable_element_tag     1

/* Struct field encoding specification for nanopb */
extern const pb_field_t MessageType_PublishTableElement_fields[4];
extern const pb_field_t MessageType_PublishTable_fields[2];

/* Maximum encoded size of messages (where known) */
#define MessageType_PublishTableElement_size     524
#define MessageType_PublishTable_size            67456

/* Message IDs (where set with "msgid" option) */
#ifdef PB_MSGID

#define PUBLISH_TABLE_MESSAGES \


#endif

#ifdef __cplusplus
} /* extern "C" */
#endif
/* @@protoc_insertion_point(eof) */

#endif
