/* Automatically generated nanopb header */
/* Generated by nanopb-0.3.7-dev at Wed Jan 23 10:49:06 2019. */

#ifndef PB_MESSAGETYPE_DEVICE_VERSION_PB_H_INCLUDED
#define PB_MESSAGETYPE_DEVICE_VERSION_PB_H_INCLUDED
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
typedef struct _MessageType_DeviceVersion {
    char name[128];
    char version[128];
/* @@protoc_insertion_point(struct:MessageType_DeviceVersion) */
} MessageType_DeviceVersion;

typedef struct _MessageType_DeviceVersionList {
    pb_size_t device_version_count;
    MessageType_DeviceVersion device_version[32];
/* @@protoc_insertion_point(struct:MessageType_DeviceVersionList) */
} MessageType_DeviceVersionList;

/* Default values for struct fields */

/* Initializer values for message structs */
#define MessageType_DeviceVersion_init_default   {"", ""}
#define MessageType_DeviceVersionList_init_default {0, {MessageType_DeviceVersion_init_default, MessageType_DeviceVersion_init_default, MessageType_DeviceVersion_init_default, MessageType_DeviceVersion_init_default, MessageType_DeviceVersion_init_default, MessageType_DeviceVersion_init_default, MessageType_DeviceVersion_init_default, MessageType_DeviceVersion_init_default, MessageType_DeviceVersion_init_default, MessageType_DeviceVersion_init_default, MessageType_DeviceVersion_init_default, MessageType_DeviceVersion_init_default, MessageType_DeviceVersion_init_default, MessageType_DeviceVersion_init_default, MessageType_DeviceVersion_init_default, MessageType_DeviceVersion_init_default, MessageType_DeviceVersion_init_default, MessageType_DeviceVersion_init_default, MessageType_DeviceVersion_init_default, MessageType_DeviceVersion_init_default, MessageType_DeviceVersion_init_default, MessageType_DeviceVersion_init_default, MessageType_DeviceVersion_init_default, MessageType_DeviceVersion_init_default, MessageType_DeviceVersion_init_default, MessageType_DeviceVersion_init_default, MessageType_DeviceVersion_init_default, MessageType_DeviceVersion_init_default, MessageType_DeviceVersion_init_default, MessageType_DeviceVersion_init_default, MessageType_DeviceVersion_init_default, MessageType_DeviceVersion_init_default}}
#define MessageType_DeviceVersion_init_zero      {"", ""}
#define MessageType_DeviceVersionList_init_zero  {0, {MessageType_DeviceVersion_init_zero, MessageType_DeviceVersion_init_zero, MessageType_DeviceVersion_init_zero, MessageType_DeviceVersion_init_zero, MessageType_DeviceVersion_init_zero, MessageType_DeviceVersion_init_zero, MessageType_DeviceVersion_init_zero, MessageType_DeviceVersion_init_zero, MessageType_DeviceVersion_init_zero, MessageType_DeviceVersion_init_zero, MessageType_DeviceVersion_init_zero, MessageType_DeviceVersion_init_zero, MessageType_DeviceVersion_init_zero, MessageType_DeviceVersion_init_zero, MessageType_DeviceVersion_init_zero, MessageType_DeviceVersion_init_zero, MessageType_DeviceVersion_init_zero, MessageType_DeviceVersion_init_zero, MessageType_DeviceVersion_init_zero, MessageType_DeviceVersion_init_zero, MessageType_DeviceVersion_init_zero, MessageType_DeviceVersion_init_zero, MessageType_DeviceVersion_init_zero, MessageType_DeviceVersion_init_zero, MessageType_DeviceVersion_init_zero, MessageType_DeviceVersion_init_zero, MessageType_DeviceVersion_init_zero, MessageType_DeviceVersion_init_zero, MessageType_DeviceVersion_init_zero, MessageType_DeviceVersion_init_zero, MessageType_DeviceVersion_init_zero, MessageType_DeviceVersion_init_zero}}

/* Field tags (for use in manual encoding/decoding) */
#define MessageType_DeviceVersion_name_tag       1
#define MessageType_DeviceVersion_version_tag    2
#define MessageType_DeviceVersionList_device_version_tag 1

/* Struct field encoding specification for nanopb */
extern const pb_field_t MessageType_DeviceVersion_fields[3];
extern const pb_field_t MessageType_DeviceVersionList_fields[2];

/* Maximum encoded size of messages (where known) */
#define MessageType_DeviceVersion_size           262
#define MessageType_DeviceVersionList_size       8480

/* Message IDs (where set with "msgid" option) */
#ifdef PB_MSGID

#define DEVICE_VERSION_MESSAGES \


#endif

#ifdef __cplusplus
} /* extern "C" */
#endif
/* @@protoc_insertion_point(eof) */

#endif