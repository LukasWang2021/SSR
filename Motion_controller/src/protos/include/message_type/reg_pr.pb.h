/* Automatically generated nanopb header */
/* Generated by nanopb-0.3.7-dev at Fri Aug 10 15:58:13 2018. */

#ifndef PB_MESSAGETYPE_REG_PR_PB_H_INCLUDED
#define PB_MESSAGETYPE_REG_PR_PB_H_INCLUDED
#include <pb.h>

#include "message_type/base.pb.h"

/* @@protoc_insertion_point(includes) */
#if PB_PROTO_HEADER_VERSION != 30
#error Regenerate this file with the current version of nanopb generator.
#endif

#ifdef __cplusplus
extern "C" {
#endif

/* Struct definitions */
typedef struct _MessageType_PrRegData {
    int32_t id;
    char name[32];
    char comment[256];
    int32_t group_id;
    int32_t pos_type;
    MessageType_DoubleList pos;
    MessageType_BoolList posture;
/* @@protoc_insertion_point(struct:MessageType_PrRegData) */
} MessageType_PrRegData;

/* Default values for struct fields */

/* Initializer values for message structs */
#define MessageType_PrRegData_init_default       {0, "", "", 0, 0, MessageType_DoubleList_init_default, MessageType_BoolList_init_default}
#define MessageType_PrRegData_init_zero          {0, "", "", 0, 0, MessageType_DoubleList_init_zero, MessageType_BoolList_init_zero}

/* Field tags (for use in manual encoding/decoding) */
#define MessageType_PrRegData_id_tag             1
#define MessageType_PrRegData_name_tag           2
#define MessageType_PrRegData_comment_tag        3
#define MessageType_PrRegData_group_id_tag       4
#define MessageType_PrRegData_pos_type_tag       5
#define MessageType_PrRegData_pos_tag            6
#define MessageType_PrRegData_posture_tag        7

/* Struct field encoding specification for nanopb */
extern const pb_field_t MessageType_PrRegData_fields[8];

/* Maximum encoded size of messages (where known) */
#define MessageType_PrRegData_size               1740

/* Message IDs (where set with "msgid" option) */
#ifdef PB_MSGID

#define REG_PR_MESSAGES \


#endif

#ifdef __cplusplus
} /* extern "C" */
#endif
/* @@protoc_insertion_point(eof) */

#endif