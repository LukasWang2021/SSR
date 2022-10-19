/* Automatically generated nanopb header */
/* Generated by nanopb-0.3.7 at Tue Jul 20 10:18:31 2021. */

#ifndef PB_MESSAGETYPE_REG_BASE_SUMMARY_PB_H_INCLUDED
#define PB_MESSAGETYPE_REG_BASE_SUMMARY_PB_H_INCLUDED
#include <pb.h>

/* @@protoc_insertion_point(includes) */
#if PB_PROTO_HEADER_VERSION != 30
#error Regenerate this file with the current version of nanopb generator.
#endif

#ifdef __cplusplus
extern "C" {
#endif

/* Struct definitions */
typedef struct _MessageType_BaseRegSummary {
    int32_t id;
    char name[32];
    char comment[256];
/* @@protoc_insertion_point(struct:MessageType_BaseRegSummary) */
} MessageType_BaseRegSummary;

typedef struct _MessageType_BaseRegSummaryList {
    pb_size_t summary_count;
    MessageType_BaseRegSummary summary[128];
/* @@protoc_insertion_point(struct:MessageType_BaseRegSummaryList) */
} MessageType_BaseRegSummaryList;

/* Default values for struct fields */

/* Initializer values for message structs */
#define MessageType_BaseRegSummary_init_default  {0, "", ""}
#define MessageType_BaseRegSummaryList_init_default {0, {MessageType_BaseRegSummary_init_default, MessageType_BaseRegSummary_init_default, MessageType_BaseRegSummary_init_default, MessageType_BaseRegSummary_init_default, MessageType_BaseRegSummary_init_default, MessageType_BaseRegSummary_init_default, MessageType_BaseRegSummary_init_default, MessageType_BaseRegSummary_init_default, MessageType_BaseRegSummary_init_default, MessageType_BaseRegSummary_init_default, MessageType_BaseRegSummary_init_default, MessageType_BaseRegSummary_init_default, MessageType_BaseRegSummary_init_default, MessageType_BaseRegSummary_init_default, MessageType_BaseRegSummary_init_default, MessageType_BaseRegSummary_init_default, MessageType_BaseRegSummary_init_default, MessageType_BaseRegSummary_init_default, MessageType_BaseRegSummary_init_default, MessageType_BaseRegSummary_init_default, MessageType_BaseRegSummary_init_default, MessageType_BaseRegSummary_init_default, MessageType_BaseRegSummary_init_default, MessageType_BaseRegSummary_init_default, MessageType_BaseRegSummary_init_default, MessageType_BaseRegSummary_init_default, MessageType_BaseRegSummary_init_default, MessageType_BaseRegSummary_init_default, MessageType_BaseRegSummary_init_default, MessageType_BaseRegSummary_init_default, MessageType_BaseRegSummary_init_default, MessageType_BaseRegSummary_init_default, MessageType_BaseRegSummary_init_default, MessageType_BaseRegSummary_init_default, MessageType_BaseRegSummary_init_default, MessageType_BaseRegSummary_init_default, MessageType_BaseRegSummary_init_default, MessageType_BaseRegSummary_init_default, MessageType_BaseRegSummary_init_default, MessageType_BaseRegSummary_init_default, MessageType_BaseRegSummary_init_default, MessageType_BaseRegSummary_init_default, MessageType_BaseRegSummary_init_default, MessageType_BaseRegSummary_init_default, MessageType_BaseRegSummary_init_default, MessageType_BaseRegSummary_init_default, MessageType_BaseRegSummary_init_default, MessageType_BaseRegSummary_init_default, MessageType_BaseRegSummary_init_default, MessageType_BaseRegSummary_init_default, MessageType_BaseRegSummary_init_default, MessageType_BaseRegSummary_init_default, MessageType_BaseRegSummary_init_default, MessageType_BaseRegSummary_init_default, MessageType_BaseRegSummary_init_default, MessageType_BaseRegSummary_init_default, MessageType_BaseRegSummary_init_default, MessageType_BaseRegSummary_init_default, MessageType_BaseRegSummary_init_default, MessageType_BaseRegSummary_init_default, MessageType_BaseRegSummary_init_default, MessageType_BaseRegSummary_init_default, MessageType_BaseRegSummary_init_default, MessageType_BaseRegSummary_init_default, MessageType_BaseRegSummary_init_default, MessageType_BaseRegSummary_init_default, MessageType_BaseRegSummary_init_default, MessageType_BaseRegSummary_init_default, MessageType_BaseRegSummary_init_default, MessageType_BaseRegSummary_init_default, MessageType_BaseRegSummary_init_default, MessageType_BaseRegSummary_init_default, MessageType_BaseRegSummary_init_default, MessageType_BaseRegSummary_init_default, MessageType_BaseRegSummary_init_default, MessageType_BaseRegSummary_init_default, MessageType_BaseRegSummary_init_default, MessageType_BaseRegSummary_init_default, MessageType_BaseRegSummary_init_default, MessageType_BaseRegSummary_init_default, MessageType_BaseRegSummary_init_default, MessageType_BaseRegSummary_init_default, MessageType_BaseRegSummary_init_default, MessageType_BaseRegSummary_init_default, MessageType_BaseRegSummary_init_default, MessageType_BaseRegSummary_init_default, MessageType_BaseRegSummary_init_default, MessageType_BaseRegSummary_init_default, MessageType_BaseRegSummary_init_default, MessageType_BaseRegSummary_init_default, MessageType_BaseRegSummary_init_default, MessageType_BaseRegSummary_init_default, MessageType_BaseRegSummary_init_default, MessageType_BaseRegSummary_init_default, MessageType_BaseRegSummary_init_default, MessageType_BaseRegSummary_init_default, MessageType_BaseRegSummary_init_default, MessageType_BaseRegSummary_init_default, MessageType_BaseRegSummary_init_default, MessageType_BaseRegSummary_init_default, MessageType_BaseRegSummary_init_default, MessageType_BaseRegSummary_init_default, MessageType_BaseRegSummary_init_default, MessageType_BaseRegSummary_init_default, MessageType_BaseRegSummary_init_default, MessageType_BaseRegSummary_init_default, MessageType_BaseRegSummary_init_default, MessageType_BaseRegSummary_init_default, MessageType_BaseRegSummary_init_default, MessageType_BaseRegSummary_init_default, MessageType_BaseRegSummary_init_default, MessageType_BaseRegSummary_init_default, MessageType_BaseRegSummary_init_default, MessageType_BaseRegSummary_init_default, MessageType_BaseRegSummary_init_default, MessageType_BaseRegSummary_init_default, MessageType_BaseRegSummary_init_default, MessageType_BaseRegSummary_init_default, MessageType_BaseRegSummary_init_default, MessageType_BaseRegSummary_init_default, MessageType_BaseRegSummary_init_default, MessageType_BaseRegSummary_init_default, MessageType_BaseRegSummary_init_default, MessageType_BaseRegSummary_init_default, MessageType_BaseRegSummary_init_default, MessageType_BaseRegSummary_init_default, MessageType_BaseRegSummary_init_default, MessageType_BaseRegSummary_init_default}}
#define MessageType_BaseRegSummary_init_zero     {0, "", ""}
#define MessageType_BaseRegSummaryList_init_zero {0, {MessageType_BaseRegSummary_init_zero, MessageType_BaseRegSummary_init_zero, MessageType_BaseRegSummary_init_zero, MessageType_BaseRegSummary_init_zero, MessageType_BaseRegSummary_init_zero, MessageType_BaseRegSummary_init_zero, MessageType_BaseRegSummary_init_zero, MessageType_BaseRegSummary_init_zero, MessageType_BaseRegSummary_init_zero, MessageType_BaseRegSummary_init_zero, MessageType_BaseRegSummary_init_zero, MessageType_BaseRegSummary_init_zero, MessageType_BaseRegSummary_init_zero, MessageType_BaseRegSummary_init_zero, MessageType_BaseRegSummary_init_zero, MessageType_BaseRegSummary_init_zero, MessageType_BaseRegSummary_init_zero, MessageType_BaseRegSummary_init_zero, MessageType_BaseRegSummary_init_zero, MessageType_BaseRegSummary_init_zero, MessageType_BaseRegSummary_init_zero, MessageType_BaseRegSummary_init_zero, MessageType_BaseRegSummary_init_zero, MessageType_BaseRegSummary_init_zero, MessageType_BaseRegSummary_init_zero, MessageType_BaseRegSummary_init_zero, MessageType_BaseRegSummary_init_zero, MessageType_BaseRegSummary_init_zero, MessageType_BaseRegSummary_init_zero, MessageType_BaseRegSummary_init_zero, MessageType_BaseRegSummary_init_zero, MessageType_BaseRegSummary_init_zero, MessageType_BaseRegSummary_init_zero, MessageType_BaseRegSummary_init_zero, MessageType_BaseRegSummary_init_zero, MessageType_BaseRegSummary_init_zero, MessageType_BaseRegSummary_init_zero, MessageType_BaseRegSummary_init_zero, MessageType_BaseRegSummary_init_zero, MessageType_BaseRegSummary_init_zero, MessageType_BaseRegSummary_init_zero, MessageType_BaseRegSummary_init_zero, MessageType_BaseRegSummary_init_zero, MessageType_BaseRegSummary_init_zero, MessageType_BaseRegSummary_init_zero, MessageType_BaseRegSummary_init_zero, MessageType_BaseRegSummary_init_zero, MessageType_BaseRegSummary_init_zero, MessageType_BaseRegSummary_init_zero, MessageType_BaseRegSummary_init_zero, MessageType_BaseRegSummary_init_zero, MessageType_BaseRegSummary_init_zero, MessageType_BaseRegSummary_init_zero, MessageType_BaseRegSummary_init_zero, MessageType_BaseRegSummary_init_zero, MessageType_BaseRegSummary_init_zero, MessageType_BaseRegSummary_init_zero, MessageType_BaseRegSummary_init_zero, MessageType_BaseRegSummary_init_zero, MessageType_BaseRegSummary_init_zero, MessageType_BaseRegSummary_init_zero, MessageType_BaseRegSummary_init_zero, MessageType_BaseRegSummary_init_zero, MessageType_BaseRegSummary_init_zero, MessageType_BaseRegSummary_init_zero, MessageType_BaseRegSummary_init_zero, MessageType_BaseRegSummary_init_zero, MessageType_BaseRegSummary_init_zero, MessageType_BaseRegSummary_init_zero, MessageType_BaseRegSummary_init_zero, MessageType_BaseRegSummary_init_zero, MessageType_BaseRegSummary_init_zero, MessageType_BaseRegSummary_init_zero, MessageType_BaseRegSummary_init_zero, MessageType_BaseRegSummary_init_zero, MessageType_BaseRegSummary_init_zero, MessageType_BaseRegSummary_init_zero, MessageType_BaseRegSummary_init_zero, MessageType_BaseRegSummary_init_zero, MessageType_BaseRegSummary_init_zero, MessageType_BaseRegSummary_init_zero, MessageType_BaseRegSummary_init_zero, MessageType_BaseRegSummary_init_zero, MessageType_BaseRegSummary_init_zero, MessageType_BaseRegSummary_init_zero, MessageType_BaseRegSummary_init_zero, MessageType_BaseRegSummary_init_zero, MessageType_BaseRegSummary_init_zero, MessageType_BaseRegSummary_init_zero, MessageType_BaseRegSummary_init_zero, MessageType_BaseRegSummary_init_zero, MessageType_BaseRegSummary_init_zero, MessageType_BaseRegSummary_init_zero, MessageType_BaseRegSummary_init_zero, MessageType_BaseRegSummary_init_zero, MessageType_BaseRegSummary_init_zero, MessageType_BaseRegSummary_init_zero, MessageType_BaseRegSummary_init_zero, MessageType_BaseRegSummary_init_zero, MessageType_BaseRegSummary_init_zero, MessageType_BaseRegSummary_init_zero, MessageType_BaseRegSummary_init_zero, MessageType_BaseRegSummary_init_zero, MessageType_BaseRegSummary_init_zero, MessageType_BaseRegSummary_init_zero, MessageType_BaseRegSummary_init_zero, MessageType_BaseRegSummary_init_zero, MessageType_BaseRegSummary_init_zero, MessageType_BaseRegSummary_init_zero, MessageType_BaseRegSummary_init_zero, MessageType_BaseRegSummary_init_zero, MessageType_BaseRegSummary_init_zero, MessageType_BaseRegSummary_init_zero, MessageType_BaseRegSummary_init_zero, MessageType_BaseRegSummary_init_zero, MessageType_BaseRegSummary_init_zero, MessageType_BaseRegSummary_init_zero, MessageType_BaseRegSummary_init_zero, MessageType_BaseRegSummary_init_zero, MessageType_BaseRegSummary_init_zero, MessageType_BaseRegSummary_init_zero, MessageType_BaseRegSummary_init_zero, MessageType_BaseRegSummary_init_zero, MessageType_BaseRegSummary_init_zero, MessageType_BaseRegSummary_init_zero, MessageType_BaseRegSummary_init_zero, MessageType_BaseRegSummary_init_zero, MessageType_BaseRegSummary_init_zero}}

/* Field tags (for use in manual encoding/decoding) */
#define MessageType_BaseRegSummary_id_tag        1
#define MessageType_BaseRegSummary_name_tag      2
#define MessageType_BaseRegSummary_comment_tag   3
#define MessageType_BaseRegSummaryList_summary_tag 1

/* Struct field encoding specification for nanopb */
extern const pb_field_t MessageType_BaseRegSummary_fields[4];
extern const pb_field_t MessageType_BaseRegSummaryList_fields[2];

/* Maximum encoded size of messages (where known) */
#define MessageType_BaseRegSummary_size          304
#define MessageType_BaseRegSummaryList_size      39296

/* Message IDs (where set with "msgid" option) */
#ifdef PB_MSGID

#define REG_BASE_SUMMARY_MESSAGES \


#endif

#ifdef __cplusplus
} /* extern "C" */
#endif
/* @@protoc_insertion_point(eof) */

#endif
