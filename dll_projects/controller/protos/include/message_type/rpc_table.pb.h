/* Automatically generated nanopb header */
/* Generated by nanopb-0.3.7 at Thu Sep 19 16:40:41 2019. */

#ifndef PB_MESSAGETYPE_RPC_TABLE_PB_H_INCLUDED
#define PB_MESSAGETYPE_RPC_TABLE_PB_H_INCLUDED
#include <pb.h>

/* @@protoc_insertion_point(includes) */
#if PB_PROTO_HEADER_VERSION != 30
#error Regenerate this file with the current version of nanopb generator.
#endif

#ifdef __cplusplus
extern "C" {
#endif

/* Struct definitions */
typedef struct _MessageType_RpcTableElement {
    char path[256];
    uint32_t hash;
    char request_message_type[256];
    char response_message_type[256];
/* @@protoc_insertion_point(struct:MessageType_RpcTableElement) */
} MessageType_RpcTableElement;

typedef struct _MessageType_RpcTable {
    pb_size_t element_count;
    MessageType_RpcTableElement element[256];
/* @@protoc_insertion_point(struct:MessageType_RpcTable) */
} MessageType_RpcTable;

/* Default values for struct fields */

/* Initializer values for message structs */
#define MessageType_RpcTableElement_init_default {"", 0, "", ""}
#define MessageType_RpcTable_init_default        {0, {MessageType_RpcTableElement_init_default, MessageType_RpcTableElement_init_default, MessageType_RpcTableElement_init_default, MessageType_RpcTableElement_init_default, MessageType_RpcTableElement_init_default, MessageType_RpcTableElement_init_default, MessageType_RpcTableElement_init_default, MessageType_RpcTableElement_init_default, MessageType_RpcTableElement_init_default, MessageType_RpcTableElement_init_default, MessageType_RpcTableElement_init_default, MessageType_RpcTableElement_init_default, MessageType_RpcTableElement_init_default, MessageType_RpcTableElement_init_default, MessageType_RpcTableElement_init_default, MessageType_RpcTableElement_init_default, MessageType_RpcTableElement_init_default, MessageType_RpcTableElement_init_default, MessageType_RpcTableElement_init_default, MessageType_RpcTableElement_init_default, MessageType_RpcTableElement_init_default, MessageType_RpcTableElement_init_default, MessageType_RpcTableElement_init_default, MessageType_RpcTableElement_init_default, MessageType_RpcTableElement_init_default, MessageType_RpcTableElement_init_default, MessageType_RpcTableElement_init_default, MessageType_RpcTableElement_init_default, MessageType_RpcTableElement_init_default, MessageType_RpcTableElement_init_default, MessageType_RpcTableElement_init_default, MessageType_RpcTableElement_init_default, MessageType_RpcTableElement_init_default, MessageType_RpcTableElement_init_default, MessageType_RpcTableElement_init_default, MessageType_RpcTableElement_init_default, MessageType_RpcTableElement_init_default, MessageType_RpcTableElement_init_default, MessageType_RpcTableElement_init_default, MessageType_RpcTableElement_init_default, MessageType_RpcTableElement_init_default, MessageType_RpcTableElement_init_default, MessageType_RpcTableElement_init_default, MessageType_RpcTableElement_init_default, MessageType_RpcTableElement_init_default, MessageType_RpcTableElement_init_default, MessageType_RpcTableElement_init_default, MessageType_RpcTableElement_init_default, MessageType_RpcTableElement_init_default, MessageType_RpcTableElement_init_default, MessageType_RpcTableElement_init_default, MessageType_RpcTableElement_init_default, MessageType_RpcTableElement_init_default, MessageType_RpcTableElement_init_default, MessageType_RpcTableElement_init_default, MessageType_RpcTableElement_init_default, MessageType_RpcTableElement_init_default, MessageType_RpcTableElement_init_default, MessageType_RpcTableElement_init_default, MessageType_RpcTableElement_init_default, MessageType_RpcTableElement_init_default, MessageType_RpcTableElement_init_default, MessageType_RpcTableElement_init_default, MessageType_RpcTableElement_init_default, MessageType_RpcTableElement_init_default, MessageType_RpcTableElement_init_default, MessageType_RpcTableElement_init_default, MessageType_RpcTableElement_init_default, MessageType_RpcTableElement_init_default, MessageType_RpcTableElement_init_default, MessageType_RpcTableElement_init_default, MessageType_RpcTableElement_init_default, MessageType_RpcTableElement_init_default, MessageType_RpcTableElement_init_default, MessageType_RpcTableElement_init_default, MessageType_RpcTableElement_init_default, MessageType_RpcTableElement_init_default, MessageType_RpcTableElement_init_default, MessageType_RpcTableElement_init_default, MessageType_RpcTableElement_init_default, MessageType_RpcTableElement_init_default, MessageType_RpcTableElement_init_default, MessageType_RpcTableElement_init_default, MessageType_RpcTableElement_init_default, MessageType_RpcTableElement_init_default, MessageType_RpcTableElement_init_default, MessageType_RpcTableElement_init_default, MessageType_RpcTableElement_init_default, MessageType_RpcTableElement_init_default, MessageType_RpcTableElement_init_default, MessageType_RpcTableElement_init_default, MessageType_RpcTableElement_init_default, MessageType_RpcTableElement_init_default, MessageType_RpcTableElement_init_default, MessageType_RpcTableElement_init_default, MessageType_RpcTableElement_init_default, MessageType_RpcTableElement_init_default, MessageType_RpcTableElement_init_default, MessageType_RpcTableElement_init_default, MessageType_RpcTableElement_init_default, MessageType_RpcTableElement_init_default, MessageType_RpcTableElement_init_default, MessageType_RpcTableElement_init_default, MessageType_RpcTableElement_init_default, MessageType_RpcTableElement_init_default, MessageType_RpcTableElement_init_default, MessageType_RpcTableElement_init_default, MessageType_RpcTableElement_init_default, MessageType_RpcTableElement_init_default, MessageType_RpcTableElement_init_default, MessageType_RpcTableElement_init_default, MessageType_RpcTableElement_init_default, MessageType_RpcTableElement_init_default, MessageType_RpcTableElement_init_default, MessageType_RpcTableElement_init_default, MessageType_RpcTableElement_init_default, MessageType_RpcTableElement_init_default, MessageType_RpcTableElement_init_default, MessageType_RpcTableElement_init_default, MessageType_RpcTableElement_init_default, MessageType_RpcTableElement_init_default, MessageType_RpcTableElement_init_default, MessageType_RpcTableElement_init_default, MessageType_RpcTableElement_init_default, MessageType_RpcTableElement_init_default, MessageType_RpcTableElement_init_default, MessageType_RpcTableElement_init_default, MessageType_RpcTableElement_init_default, MessageType_RpcTableElement_init_default, MessageType_RpcTableElement_init_default, MessageType_RpcTableElement_init_default, MessageType_RpcTableElement_init_default, MessageType_RpcTableElement_init_default, MessageType_RpcTableElement_init_default, MessageType_RpcTableElement_init_default, MessageType_RpcTableElement_init_default, MessageType_RpcTableElement_init_default, MessageType_RpcTableElement_init_default, MessageType_RpcTableElement_init_default, MessageType_RpcTableElement_init_default, MessageType_RpcTableElement_init_default, MessageType_RpcTableElement_init_default, MessageType_RpcTableElement_init_default, MessageType_RpcTableElement_init_default, MessageType_RpcTableElement_init_default, MessageType_RpcTableElement_init_default, MessageType_RpcTableElement_init_default, MessageType_RpcTableElement_init_default, MessageType_RpcTableElement_init_default, MessageType_RpcTableElement_init_default, MessageType_RpcTableElement_init_default, MessageType_RpcTableElement_init_default, MessageType_RpcTableElement_init_default, MessageType_RpcTableElement_init_default, MessageType_RpcTableElement_init_default, MessageType_RpcTableElement_init_default, MessageType_RpcTableElement_init_default, MessageType_RpcTableElement_init_default, MessageType_RpcTableElement_init_default, MessageType_RpcTableElement_init_default, MessageType_RpcTableElement_init_default, MessageType_RpcTableElement_init_default, MessageType_RpcTableElement_init_default, MessageType_RpcTableElement_init_default, MessageType_RpcTableElement_init_default, MessageType_RpcTableElement_init_default, MessageType_RpcTableElement_init_default, MessageType_RpcTableElement_init_default, MessageType_RpcTableElement_init_default, MessageType_RpcTableElement_init_default, MessageType_RpcTableElement_init_default, MessageType_RpcTableElement_init_default, MessageType_RpcTableElement_init_default, MessageType_RpcTableElement_init_default, MessageType_RpcTableElement_init_default, MessageType_RpcTableElement_init_default, MessageType_RpcTableElement_init_default, MessageType_RpcTableElement_init_default, MessageType_RpcTableElement_init_default, MessageType_RpcTableElement_init_default, MessageType_RpcTableElement_init_default, MessageType_RpcTableElement_init_default, MessageType_RpcTableElement_init_default, MessageType_RpcTableElement_init_default, MessageType_RpcTableElement_init_default, MessageType_RpcTableElement_init_default, MessageType_RpcTableElement_init_default, MessageType_RpcTableElement_init_default, MessageType_RpcTableElement_init_default, MessageType_RpcTableElement_init_default, MessageType_RpcTableElement_init_default, MessageType_RpcTableElement_init_default, MessageType_RpcTableElement_init_default, MessageType_RpcTableElement_init_default, MessageType_RpcTableElement_init_default, MessageType_RpcTableElement_init_default, MessageType_RpcTableElement_init_default, MessageType_RpcTableElement_init_default, MessageType_RpcTableElement_init_default, MessageType_RpcTableElement_init_default, MessageType_RpcTableElement_init_default, MessageType_RpcTableElement_init_default, MessageType_RpcTableElement_init_default, MessageType_RpcTableElement_init_default, MessageType_RpcTableElement_init_default, MessageType_RpcTableElement_init_default, MessageType_RpcTableElement_init_default, MessageType_RpcTableElement_init_default, MessageType_RpcTableElement_init_default, MessageType_RpcTableElement_init_default, MessageType_RpcTableElement_init_default, MessageType_RpcTableElement_init_default, MessageType_RpcTableElement_init_default, MessageType_RpcTableElement_init_default, MessageType_RpcTableElement_init_default, MessageType_RpcTableElement_init_default, MessageType_RpcTableElement_init_default, MessageType_RpcTableElement_init_default, MessageType_RpcTableElement_init_default, MessageType_RpcTableElement_init_default, MessageType_RpcTableElement_init_default, MessageType_RpcTableElement_init_default, MessageType_RpcTableElement_init_default, MessageType_RpcTableElement_init_default, MessageType_RpcTableElement_init_default, MessageType_RpcTableElement_init_default, MessageType_RpcTableElement_init_default, MessageType_RpcTableElement_init_default, MessageType_RpcTableElement_init_default, MessageType_RpcTableElement_init_default, MessageType_RpcTableElement_init_default, MessageType_RpcTableElement_init_default, MessageType_RpcTableElement_init_default, MessageType_RpcTableElement_init_default, MessageType_RpcTableElement_init_default, MessageType_RpcTableElement_init_default, MessageType_RpcTableElement_init_default, MessageType_RpcTableElement_init_default, MessageType_RpcTableElement_init_default, MessageType_RpcTableElement_init_default, MessageType_RpcTableElement_init_default, MessageType_RpcTableElement_init_default, MessageType_RpcTableElement_init_default, MessageType_RpcTableElement_init_default, MessageType_RpcTableElement_init_default, MessageType_RpcTableElement_init_default, MessageType_RpcTableElement_init_default, MessageType_RpcTableElement_init_default, MessageType_RpcTableElement_init_default, MessageType_RpcTableElement_init_default, MessageType_RpcTableElement_init_default, MessageType_RpcTableElement_init_default, MessageType_RpcTableElement_init_default, MessageType_RpcTableElement_init_default, MessageType_RpcTableElement_init_default, MessageType_RpcTableElement_init_default}}
#define MessageType_RpcTableElement_init_zero    {"", 0, "", ""}
#define MessageType_RpcTable_init_zero           {0, {MessageType_RpcTableElement_init_zero, MessageType_RpcTableElement_init_zero, MessageType_RpcTableElement_init_zero, MessageType_RpcTableElement_init_zero, MessageType_RpcTableElement_init_zero, MessageType_RpcTableElement_init_zero, MessageType_RpcTableElement_init_zero, MessageType_RpcTableElement_init_zero, MessageType_RpcTableElement_init_zero, MessageType_RpcTableElement_init_zero, MessageType_RpcTableElement_init_zero, MessageType_RpcTableElement_init_zero, MessageType_RpcTableElement_init_zero, MessageType_RpcTableElement_init_zero, MessageType_RpcTableElement_init_zero, MessageType_RpcTableElement_init_zero, MessageType_RpcTableElement_init_zero, MessageType_RpcTableElement_init_zero, MessageType_RpcTableElement_init_zero, MessageType_RpcTableElement_init_zero, MessageType_RpcTableElement_init_zero, MessageType_RpcTableElement_init_zero, MessageType_RpcTableElement_init_zero, MessageType_RpcTableElement_init_zero, MessageType_RpcTableElement_init_zero, MessageType_RpcTableElement_init_zero, MessageType_RpcTableElement_init_zero, MessageType_RpcTableElement_init_zero, MessageType_RpcTableElement_init_zero, MessageType_RpcTableElement_init_zero, MessageType_RpcTableElement_init_zero, MessageType_RpcTableElement_init_zero, MessageType_RpcTableElement_init_zero, MessageType_RpcTableElement_init_zero, MessageType_RpcTableElement_init_zero, MessageType_RpcTableElement_init_zero, MessageType_RpcTableElement_init_zero, MessageType_RpcTableElement_init_zero, MessageType_RpcTableElement_init_zero, MessageType_RpcTableElement_init_zero, MessageType_RpcTableElement_init_zero, MessageType_RpcTableElement_init_zero, MessageType_RpcTableElement_init_zero, MessageType_RpcTableElement_init_zero, MessageType_RpcTableElement_init_zero, MessageType_RpcTableElement_init_zero, MessageType_RpcTableElement_init_zero, MessageType_RpcTableElement_init_zero, MessageType_RpcTableElement_init_zero, MessageType_RpcTableElement_init_zero, MessageType_RpcTableElement_init_zero, MessageType_RpcTableElement_init_zero, MessageType_RpcTableElement_init_zero, MessageType_RpcTableElement_init_zero, MessageType_RpcTableElement_init_zero, MessageType_RpcTableElement_init_zero, MessageType_RpcTableElement_init_zero, MessageType_RpcTableElement_init_zero, MessageType_RpcTableElement_init_zero, MessageType_RpcTableElement_init_zero, MessageType_RpcTableElement_init_zero, MessageType_RpcTableElement_init_zero, MessageType_RpcTableElement_init_zero, MessageType_RpcTableElement_init_zero, MessageType_RpcTableElement_init_zero, MessageType_RpcTableElement_init_zero, MessageType_RpcTableElement_init_zero, MessageType_RpcTableElement_init_zero, MessageType_RpcTableElement_init_zero, MessageType_RpcTableElement_init_zero, MessageType_RpcTableElement_init_zero, MessageType_RpcTableElement_init_zero, MessageType_RpcTableElement_init_zero, MessageType_RpcTableElement_init_zero, MessageType_RpcTableElement_init_zero, MessageType_RpcTableElement_init_zero, MessageType_RpcTableElement_init_zero, MessageType_RpcTableElement_init_zero, MessageType_RpcTableElement_init_zero, MessageType_RpcTableElement_init_zero, MessageType_RpcTableElement_init_zero, MessageType_RpcTableElement_init_zero, MessageType_RpcTableElement_init_zero, MessageType_RpcTableElement_init_zero, MessageType_RpcTableElement_init_zero, MessageType_RpcTableElement_init_zero, MessageType_RpcTableElement_init_zero, MessageType_RpcTableElement_init_zero, MessageType_RpcTableElement_init_zero, MessageType_RpcTableElement_init_zero, MessageType_RpcTableElement_init_zero, MessageType_RpcTableElement_init_zero, MessageType_RpcTableElement_init_zero, MessageType_RpcTableElement_init_zero, MessageType_RpcTableElement_init_zero, MessageType_RpcTableElement_init_zero, MessageType_RpcTableElement_init_zero, MessageType_RpcTableElement_init_zero, MessageType_RpcTableElement_init_zero, MessageType_RpcTableElement_init_zero, MessageType_RpcTableElement_init_zero, MessageType_RpcTableElement_init_zero, MessageType_RpcTableElement_init_zero, MessageType_RpcTableElement_init_zero, MessageType_RpcTableElement_init_zero, MessageType_RpcTableElement_init_zero, MessageType_RpcTableElement_init_zero, MessageType_RpcTableElement_init_zero, MessageType_RpcTableElement_init_zero, MessageType_RpcTableElement_init_zero, MessageType_RpcTableElement_init_zero, MessageType_RpcTableElement_init_zero, MessageType_RpcTableElement_init_zero, MessageType_RpcTableElement_init_zero, MessageType_RpcTableElement_init_zero, MessageType_RpcTableElement_init_zero, MessageType_RpcTableElement_init_zero, MessageType_RpcTableElement_init_zero, MessageType_RpcTableElement_init_zero, MessageType_RpcTableElement_init_zero, MessageType_RpcTableElement_init_zero, MessageType_RpcTableElement_init_zero, MessageType_RpcTableElement_init_zero, MessageType_RpcTableElement_init_zero, MessageType_RpcTableElement_init_zero, MessageType_RpcTableElement_init_zero, MessageType_RpcTableElement_init_zero, MessageType_RpcTableElement_init_zero, MessageType_RpcTableElement_init_zero, MessageType_RpcTableElement_init_zero, MessageType_RpcTableElement_init_zero, MessageType_RpcTableElement_init_zero, MessageType_RpcTableElement_init_zero, MessageType_RpcTableElement_init_zero, MessageType_RpcTableElement_init_zero, MessageType_RpcTableElement_init_zero, MessageType_RpcTableElement_init_zero, MessageType_RpcTableElement_init_zero, MessageType_RpcTableElement_init_zero, MessageType_RpcTableElement_init_zero, MessageType_RpcTableElement_init_zero, MessageType_RpcTableElement_init_zero, MessageType_RpcTableElement_init_zero, MessageType_RpcTableElement_init_zero, MessageType_RpcTableElement_init_zero, MessageType_RpcTableElement_init_zero, MessageType_RpcTableElement_init_zero, MessageType_RpcTableElement_init_zero, MessageType_RpcTableElement_init_zero, MessageType_RpcTableElement_init_zero, MessageType_RpcTableElement_init_zero, MessageType_RpcTableElement_init_zero, MessageType_RpcTableElement_init_zero, MessageType_RpcTableElement_init_zero, MessageType_RpcTableElement_init_zero, MessageType_RpcTableElement_init_zero, MessageType_RpcTableElement_init_zero, MessageType_RpcTableElement_init_zero, MessageType_RpcTableElement_init_zero, MessageType_RpcTableElement_init_zero, MessageType_RpcTableElement_init_zero, MessageType_RpcTableElement_init_zero, MessageType_RpcTableElement_init_zero, MessageType_RpcTableElement_init_zero, MessageType_RpcTableElement_init_zero, MessageType_RpcTableElement_init_zero, MessageType_RpcTableElement_init_zero, MessageType_RpcTableElement_init_zero, MessageType_RpcTableElement_init_zero, MessageType_RpcTableElement_init_zero, MessageType_RpcTableElement_init_zero, MessageType_RpcTableElement_init_zero, MessageType_RpcTableElement_init_zero, MessageType_RpcTableElement_init_zero, MessageType_RpcTableElement_init_zero, MessageType_RpcTableElement_init_zero, MessageType_RpcTableElement_init_zero, MessageType_RpcTableElement_init_zero, MessageType_RpcTableElement_init_zero, MessageType_RpcTableElement_init_zero, MessageType_RpcTableElement_init_zero, MessageType_RpcTableElement_init_zero, MessageType_RpcTableElement_init_zero, MessageType_RpcTableElement_init_zero, MessageType_RpcTableElement_init_zero, MessageType_RpcTableElement_init_zero, MessageType_RpcTableElement_init_zero, MessageType_RpcTableElement_init_zero, MessageType_RpcTableElement_init_zero, MessageType_RpcTableElement_init_zero, MessageType_RpcTableElement_init_zero, MessageType_RpcTableElement_init_zero, MessageType_RpcTableElement_init_zero, MessageType_RpcTableElement_init_zero, MessageType_RpcTableElement_init_zero, MessageType_RpcTableElement_init_zero, MessageType_RpcTableElement_init_zero, MessageType_RpcTableElement_init_zero, MessageType_RpcTableElement_init_zero, MessageType_RpcTableElement_init_zero, MessageType_RpcTableElement_init_zero, MessageType_RpcTableElement_init_zero, MessageType_RpcTableElement_init_zero, MessageType_RpcTableElement_init_zero, MessageType_RpcTableElement_init_zero, MessageType_RpcTableElement_init_zero, MessageType_RpcTableElement_init_zero, MessageType_RpcTableElement_init_zero, MessageType_RpcTableElement_init_zero, MessageType_RpcTableElement_init_zero, MessageType_RpcTableElement_init_zero, MessageType_RpcTableElement_init_zero, MessageType_RpcTableElement_init_zero, MessageType_RpcTableElement_init_zero, MessageType_RpcTableElement_init_zero, MessageType_RpcTableElement_init_zero, MessageType_RpcTableElement_init_zero, MessageType_RpcTableElement_init_zero, MessageType_RpcTableElement_init_zero, MessageType_RpcTableElement_init_zero, MessageType_RpcTableElement_init_zero, MessageType_RpcTableElement_init_zero, MessageType_RpcTableElement_init_zero, MessageType_RpcTableElement_init_zero, MessageType_RpcTableElement_init_zero, MessageType_RpcTableElement_init_zero, MessageType_RpcTableElement_init_zero, MessageType_RpcTableElement_init_zero, MessageType_RpcTableElement_init_zero, MessageType_RpcTableElement_init_zero, MessageType_RpcTableElement_init_zero, MessageType_RpcTableElement_init_zero, MessageType_RpcTableElement_init_zero, MessageType_RpcTableElement_init_zero, MessageType_RpcTableElement_init_zero, MessageType_RpcTableElement_init_zero, MessageType_RpcTableElement_init_zero, MessageType_RpcTableElement_init_zero, MessageType_RpcTableElement_init_zero, MessageType_RpcTableElement_init_zero, MessageType_RpcTableElement_init_zero, MessageType_RpcTableElement_init_zero, MessageType_RpcTableElement_init_zero, MessageType_RpcTableElement_init_zero, MessageType_RpcTableElement_init_zero, MessageType_RpcTableElement_init_zero, MessageType_RpcTableElement_init_zero, MessageType_RpcTableElement_init_zero, MessageType_RpcTableElement_init_zero, MessageType_RpcTableElement_init_zero, MessageType_RpcTableElement_init_zero, MessageType_RpcTableElement_init_zero, MessageType_RpcTableElement_init_zero, MessageType_RpcTableElement_init_zero, MessageType_RpcTableElement_init_zero, MessageType_RpcTableElement_init_zero}}

/* Field tags (for use in manual encoding/decoding) */
#define MessageType_RpcTableElement_path_tag     1
#define MessageType_RpcTableElement_hash_tag     2
#define MessageType_RpcTableElement_request_message_type_tag 3
#define MessageType_RpcTableElement_response_message_type_tag 4
#define MessageType_RpcTable_element_tag         1

/* Struct field encoding specification for nanopb */
extern const pb_field_t MessageType_RpcTableElement_fields[5];
extern const pb_field_t MessageType_RpcTable_fields[2];

/* Maximum encoded size of messages (where known) */
#define MessageType_RpcTableElement_size         783
#define MessageType_RpcTable_size                201216

/* Message IDs (where set with "msgid" option) */
#ifdef PB_MSGID

#define RPC_TABLE_MESSAGES \


#endif

#ifdef __cplusplus
} /* extern "C" */
#endif
/* @@protoc_insertion_point(eof) */

#endif