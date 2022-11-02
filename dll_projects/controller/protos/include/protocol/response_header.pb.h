/* Automatically generated nanopb header */
/* Generated by nanopb-0.3.7-dev at Fri Dec 21 15:33:21 2018. */

#ifndef PB_RESPONSE_RESPONSE_HEADER_PB_H_INCLUDED
#define PB_RESPONSE_RESPONSE_HEADER_PB_H_INCLUDED
#include <pb.h>

/* @@protoc_insertion_point(includes) */
#if PB_PROTO_HEADER_VERSION != 30
#error Regenerate this file with the current version of nanopb generator.
#endif

#ifdef __cplusplus
extern "C" {
#endif

/* Struct definitions */
typedef struct _Response_Header {
    uint64_t time_stamp;
    int32_t package_left;
    uint64_t error_code;
/* @@protoc_insertion_point(struct:Response_Header) */
} Response_Header;

/* Default values for struct fields */

/* Initializer values for message structs */
#define Response_Header_init_default             {0, 0, 0}
#define Response_Header_init_zero                {0, 0, 0}

/* Field tags (for use in manual encoding/decoding) */
#define Response_Header_time_stamp_tag           1
#define Response_Header_package_left_tag         2
#define Response_Header_error_code_tag           3

/* Struct field encoding specification for nanopb */
extern const pb_field_t Response_Header_fields[4];

/* Maximum encoded size of messages (where known) */
#define Response_Header_size                     33

/* Message IDs (where set with "msgid" option) */
#ifdef PB_MSGID

#define RESPONSE_HEADER_MESSAGES \


#endif

#ifdef __cplusplus
} /* extern "C" */
#endif
/* @@protoc_insertion_point(eof) */

#endif
