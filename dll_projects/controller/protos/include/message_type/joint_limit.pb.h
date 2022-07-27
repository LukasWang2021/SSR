/* Automatically generated nanopb header */
/* Generated by nanopb-0.3.7 at Thu May  6 16:58:13 2021. */

#ifndef PB_MESSAGETYPE_JOINT_LIMIT_PB_H_INCLUDED
#define PB_MESSAGETYPE_JOINT_LIMIT_PB_H_INCLUDED
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
typedef struct _MessageType_JointLimit {
    MessageType_DoubleList positive_limit;
    MessageType_DoubleList negative_limit;
/* @@protoc_insertion_point(struct:MessageType_JointLimit) */
} MessageType_JointLimit;

/* Default values for struct fields */

/* Initializer values for message structs */
#define MessageType_JointLimit_init_default      {MessageType_DoubleList_init_default, MessageType_DoubleList_init_default}
#define MessageType_JointLimit_init_zero         {MessageType_DoubleList_init_zero, MessageType_DoubleList_init_zero}

/* Field tags (for use in manual encoding/decoding) */
#define MessageType_JointLimit_positive_limit_tag 1
#define MessageType_JointLimit_negative_limit_tag 2

/* Struct field encoding specification for nanopb */
extern const pb_field_t MessageType_JointLimit_fields[3];

/* Maximum encoded size of messages (where known) */
#define MessageType_JointLimit_size              2310

/* Message IDs (where set with "msgid" option) */
#ifdef PB_MSGID

#define JOINT_LIMIT_MESSAGES \


#endif

#ifdef __cplusplus
} /* extern "C" */
#endif
/* @@protoc_insertion_point(eof) */

#endif
