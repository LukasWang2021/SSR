/* Automatically generated nanopb header */
/* Generated by nanopb-0.3.7 at Sun Sep 29 18:11:45 2019. */

#ifndef PB_REQUEST_REQUEST_HEADER_PB_H_INCLUDED
#define PB_REQUEST_REQUEST_HEADER_PB_H_INCLUDED
#include <pb.h>

/* @@protoc_insertion_point(includes) */
#if PB_PROTO_HEADER_VERSION != 30
#error Regenerate this file with the current version of nanopb generator.
#endif

#ifdef __cplusplus
extern "C" {
#endif

/* Struct definitions */
typedef struct _Request_Header {
    uint64_t time_stamp;
/* @@protoc_insertion_point(struct:Request_Header) */
} Request_Header;

/* Default values for struct fields */

/* Initializer values for message structs */
#define Request_Header_init_default              {0}
#define Request_Header_init_zero                 {0}

/* Field tags (for use in manual encoding/decoding) */
#define Request_Header_time_stamp_tag            1

/* Struct field encoding specification for nanopb */
extern const pb_field_t Request_Header_fields[2];

/* Maximum encoded size of messages (where known) */
#define Request_Header_size                      11

/* Message IDs (where set with "msgid" option) */
#ifdef PB_MSGID

#define REQUEST_HEADER_MESSAGES \


#endif

#ifdef __cplusplus
} /* extern "C" */
#endif
/* @@protoc_insertion_point(eof) */

#endif
