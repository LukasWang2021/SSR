/* Automatically generated nanopb header */
/* Generated by nanopb-0.3.7 at Thu Sep 19 16:40:41 2019. */

#ifndef PB_MESSAGETYPE_AXIS_INFO_PB_H_INCLUDED
#define PB_MESSAGETYPE_AXIS_INFO_PB_H_INCLUDED
#include <pb.h>

/* @@protoc_insertion_point(includes) */
#if PB_PROTO_HEADER_VERSION != 30
#error Regenerate this file with the current version of nanopb generator.
#endif

#ifdef __cplusplus
extern "C" {
#endif

/* Struct definitions */
typedef struct _MessageType_AxisInfo {
    bool simulation;
    bool communication_ready;
    bool ready_for_power_on;
    bool power_on;
/* @@protoc_insertion_point(struct:MessageType_AxisInfo) */
} MessageType_AxisInfo;

/* Default values for struct fields */

/* Initializer values for message structs */
#define MessageType_AxisInfo_init_default        {0, 0, 0, 0}
#define MessageType_AxisInfo_init_zero           {0, 0, 0, 0}

/* Field tags (for use in manual encoding/decoding) */
#define MessageType_AxisInfo_simulation_tag      1
#define MessageType_AxisInfo_communication_ready_tag 2
#define MessageType_AxisInfo_ready_for_power_on_tag 3
#define MessageType_AxisInfo_power_on_tag        4

/* Struct field encoding specification for nanopb */
extern const pb_field_t MessageType_AxisInfo_fields[5];

/* Maximum encoded size of messages (where known) */
#define MessageType_AxisInfo_size                8

/* Message IDs (where set with "msgid" option) */
#ifdef PB_MSGID

#define AXIS_INFO_MESSAGES \


#endif

#ifdef __cplusplus
} /* extern "C" */
#endif
/* @@protoc_insertion_point(eof) */

#endif
