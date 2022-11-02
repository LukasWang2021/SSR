/* Automatically generated nanopb header */
/* Generated by nanopb-0.3.7 at Wed Jun 15 13:57:07 2022. */

#ifndef PB_MESSAGETYPE_TRANS_MATRIX_LIST_PB_H_INCLUDED
#define PB_MESSAGETYPE_TRANS_MATRIX_LIST_PB_H_INCLUDED
#include <pb.h>

/* @@protoc_insertion_point(includes) */
#if PB_PROTO_HEADER_VERSION != 30
#error Regenerate this file with the current version of nanopb generator.
#endif

#ifdef __cplusplus
extern "C" {
#endif

/* Struct definitions */
typedef struct _MessageType_TransMatrix {
    int32_t state;
    pb_size_t matrix_count;
    double matrix[16];
/* @@protoc_insertion_point(struct:MessageType_TransMatrix) */
} MessageType_TransMatrix;

typedef struct _MessageType_TransMatrixList {
    pb_size_t matrices_count;
    MessageType_TransMatrix matrices[32];
/* @@protoc_insertion_point(struct:MessageType_TransMatrixList) */
} MessageType_TransMatrixList;

/* Default values for struct fields */

/* Initializer values for message structs */
#define MessageType_TransMatrix_init_default     {0, 0, {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}}
#define MessageType_TransMatrixList_init_default {0, {MessageType_TransMatrix_init_default, MessageType_TransMatrix_init_default, MessageType_TransMatrix_init_default, MessageType_TransMatrix_init_default, MessageType_TransMatrix_init_default, MessageType_TransMatrix_init_default, MessageType_TransMatrix_init_default, MessageType_TransMatrix_init_default, MessageType_TransMatrix_init_default, MessageType_TransMatrix_init_default, MessageType_TransMatrix_init_default, MessageType_TransMatrix_init_default, MessageType_TransMatrix_init_default, MessageType_TransMatrix_init_default, MessageType_TransMatrix_init_default, MessageType_TransMatrix_init_default, MessageType_TransMatrix_init_default, MessageType_TransMatrix_init_default, MessageType_TransMatrix_init_default, MessageType_TransMatrix_init_default, MessageType_TransMatrix_init_default, MessageType_TransMatrix_init_default, MessageType_TransMatrix_init_default, MessageType_TransMatrix_init_default, MessageType_TransMatrix_init_default, MessageType_TransMatrix_init_default, MessageType_TransMatrix_init_default, MessageType_TransMatrix_init_default, MessageType_TransMatrix_init_default, MessageType_TransMatrix_init_default, MessageType_TransMatrix_init_default, MessageType_TransMatrix_init_default}}
#define MessageType_TransMatrix_init_zero        {0, 0, {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}}
#define MessageType_TransMatrixList_init_zero    {0, {MessageType_TransMatrix_init_zero, MessageType_TransMatrix_init_zero, MessageType_TransMatrix_init_zero, MessageType_TransMatrix_init_zero, MessageType_TransMatrix_init_zero, MessageType_TransMatrix_init_zero, MessageType_TransMatrix_init_zero, MessageType_TransMatrix_init_zero, MessageType_TransMatrix_init_zero, MessageType_TransMatrix_init_zero, MessageType_TransMatrix_init_zero, MessageType_TransMatrix_init_zero, MessageType_TransMatrix_init_zero, MessageType_TransMatrix_init_zero, MessageType_TransMatrix_init_zero, MessageType_TransMatrix_init_zero, MessageType_TransMatrix_init_zero, MessageType_TransMatrix_init_zero, MessageType_TransMatrix_init_zero, MessageType_TransMatrix_init_zero, MessageType_TransMatrix_init_zero, MessageType_TransMatrix_init_zero, MessageType_TransMatrix_init_zero, MessageType_TransMatrix_init_zero, MessageType_TransMatrix_init_zero, MessageType_TransMatrix_init_zero, MessageType_TransMatrix_init_zero, MessageType_TransMatrix_init_zero, MessageType_TransMatrix_init_zero, MessageType_TransMatrix_init_zero, MessageType_TransMatrix_init_zero, MessageType_TransMatrix_init_zero}}

/* Field tags (for use in manual encoding/decoding) */
#define MessageType_TransMatrix_state_tag        1
#define MessageType_TransMatrix_matrix_tag       2
#define MessageType_TransMatrixList_matrices_tag 1

/* Struct field encoding specification for nanopb */
extern const pb_field_t MessageType_TransMatrix_fields[3];
extern const pb_field_t MessageType_TransMatrixList_fields[2];

/* Maximum encoded size of messages (where known) */
#define MessageType_TransMatrix_size             155
#define MessageType_TransMatrixList_size         5056

/* Message IDs (where set with "msgid" option) */
#ifdef PB_MSGID

#define TRANS_MATRIX_LIST_MESSAGES \


#endif

#ifdef __cplusplus
} /* extern "C" */
#endif
/* @@protoc_insertion_point(eof) */

#endif
