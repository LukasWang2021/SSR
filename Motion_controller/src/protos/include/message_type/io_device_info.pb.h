/* Automatically generated nanopb header */
/* Generated by nanopb-0.3.7-dev at Mon Nov 19 16:25:07 2018. */

#ifndef PB_MESSAGETYPE_IO_DEVICE_INFO_PB_H_INCLUDED
#define PB_MESSAGETYPE_IO_DEVICE_INFO_PB_H_INCLUDED
#include <pb.h>

/* @@protoc_insertion_point(includes) */
#if PB_PROTO_HEADER_VERSION != 30
#error Regenerate this file with the current version of nanopb generator.
#endif

#ifdef __cplusplus
extern "C" {
#endif

/* Struct definitions */
typedef struct _MessageType_IoDeviceInfo {
    char device_type[256];
    char comm_type[256];
    int32_t device_index;
    int32_t address;
    int32_t input_num;
    int32_t output_num;
    bool is_valid;
/* @@protoc_insertion_point(struct:MessageType_IoDeviceInfo) */
} MessageType_IoDeviceInfo;

typedef struct _MessageType_IoDeviceInfoList {
    pb_size_t io_device_info_count;
    MessageType_IoDeviceInfo io_device_info[32];
/* @@protoc_insertion_point(struct:MessageType_IoDeviceInfoList) */
} MessageType_IoDeviceInfoList;

/* Default values for struct fields */

/* Initializer values for message structs */
#define MessageType_IoDeviceInfo_init_default    {"", "", 0, 0, 0, 0, 0}
#define MessageType_IoDeviceInfoList_init_default {0, {MessageType_IoDeviceInfo_init_default, MessageType_IoDeviceInfo_init_default, MessageType_IoDeviceInfo_init_default, MessageType_IoDeviceInfo_init_default, MessageType_IoDeviceInfo_init_default, MessageType_IoDeviceInfo_init_default, MessageType_IoDeviceInfo_init_default, MessageType_IoDeviceInfo_init_default, MessageType_IoDeviceInfo_init_default, MessageType_IoDeviceInfo_init_default, MessageType_IoDeviceInfo_init_default, MessageType_IoDeviceInfo_init_default, MessageType_IoDeviceInfo_init_default, MessageType_IoDeviceInfo_init_default, MessageType_IoDeviceInfo_init_default, MessageType_IoDeviceInfo_init_default, MessageType_IoDeviceInfo_init_default, MessageType_IoDeviceInfo_init_default, MessageType_IoDeviceInfo_init_default, MessageType_IoDeviceInfo_init_default, MessageType_IoDeviceInfo_init_default, MessageType_IoDeviceInfo_init_default, MessageType_IoDeviceInfo_init_default, MessageType_IoDeviceInfo_init_default, MessageType_IoDeviceInfo_init_default, MessageType_IoDeviceInfo_init_default, MessageType_IoDeviceInfo_init_default, MessageType_IoDeviceInfo_init_default, MessageType_IoDeviceInfo_init_default, MessageType_IoDeviceInfo_init_default, MessageType_IoDeviceInfo_init_default, MessageType_IoDeviceInfo_init_default}}
#define MessageType_IoDeviceInfo_init_zero       {"", "", 0, 0, 0, 0, 0}
#define MessageType_IoDeviceInfoList_init_zero   {0, {MessageType_IoDeviceInfo_init_zero, MessageType_IoDeviceInfo_init_zero, MessageType_IoDeviceInfo_init_zero, MessageType_IoDeviceInfo_init_zero, MessageType_IoDeviceInfo_init_zero, MessageType_IoDeviceInfo_init_zero, MessageType_IoDeviceInfo_init_zero, MessageType_IoDeviceInfo_init_zero, MessageType_IoDeviceInfo_init_zero, MessageType_IoDeviceInfo_init_zero, MessageType_IoDeviceInfo_init_zero, MessageType_IoDeviceInfo_init_zero, MessageType_IoDeviceInfo_init_zero, MessageType_IoDeviceInfo_init_zero, MessageType_IoDeviceInfo_init_zero, MessageType_IoDeviceInfo_init_zero, MessageType_IoDeviceInfo_init_zero, MessageType_IoDeviceInfo_init_zero, MessageType_IoDeviceInfo_init_zero, MessageType_IoDeviceInfo_init_zero, MessageType_IoDeviceInfo_init_zero, MessageType_IoDeviceInfo_init_zero, MessageType_IoDeviceInfo_init_zero, MessageType_IoDeviceInfo_init_zero, MessageType_IoDeviceInfo_init_zero, MessageType_IoDeviceInfo_init_zero, MessageType_IoDeviceInfo_init_zero, MessageType_IoDeviceInfo_init_zero, MessageType_IoDeviceInfo_init_zero, MessageType_IoDeviceInfo_init_zero, MessageType_IoDeviceInfo_init_zero, MessageType_IoDeviceInfo_init_zero}}

/* Field tags (for use in manual encoding/decoding) */
#define MessageType_IoDeviceInfo_device_type_tag 1
#define MessageType_IoDeviceInfo_comm_type_tag   2
#define MessageType_IoDeviceInfo_device_index_tag 3
#define MessageType_IoDeviceInfo_address_tag     4
#define MessageType_IoDeviceInfo_input_num_tag   5
#define MessageType_IoDeviceInfo_output_num_tag  6
#define MessageType_IoDeviceInfo_is_valid_tag    7
#define MessageType_IoDeviceInfoList_io_device_info_tag 1

/* Struct field encoding specification for nanopb */
extern const pb_field_t MessageType_IoDeviceInfo_fields[8];
extern const pb_field_t MessageType_IoDeviceInfoList_fields[2];

/* Maximum encoded size of messages (where known) */
#define MessageType_IoDeviceInfo_size            564
#define MessageType_IoDeviceInfoList_size        18144

/* Message IDs (where set with "msgid" option) */
#ifdef PB_MSGID

#define IO_DEVICE_INFO_MESSAGES \


#endif

#ifdef __cplusplus
} /* extern "C" */
#endif
/* @@protoc_insertion_point(eof) */

#endif