/* Automatically generated nanopb constant definitions */
/* Generated by nanopb-0.3.7-dev at Wed Sep 12 13:42:17 2018. */

#include "file.pb.h"

/* @@protoc_insertion_point(includes) */
#if PB_PROTO_HEADER_VERSION != 30
#error Regenerate this file with the current version of nanopb generator.
#endif



const pb_field_t MessageType_File_fields[3] = {
    PB_FIELD(  1, STRING  , REQUIRED, STATIC  , FIRST, MessageType_File, file_path, file_path, 0),
    PB_FIELD(  2, STRING  , REQUIRED, STATIC  , OTHER, MessageType_File, data, file_path, 0),
    PB_LAST_FIELD
};


/* Check that field information fits in pb_field_t */
#if !defined(PB_FIELD_16BIT) && !defined(PB_FIELD_32BIT)
#error Field descriptor for MessageType_File.data is too large. Define PB_FIELD_16BIT to fix this.
#endif


/* @@protoc_insertion_point(eof) */
