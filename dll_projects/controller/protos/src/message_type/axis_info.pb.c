/* Automatically generated nanopb constant definitions */
/* Generated by nanopb-0.3.7 at Thu Sep 19 13:40:57 2019. */

#include "axis_info.pb.h"

/* @@protoc_insertion_point(includes) */
#if PB_PROTO_HEADER_VERSION != 30
#error Regenerate this file with the current version of nanopb generator.
#endif



const pb_field_t MessageType_AxisInfo_fields[5] = {
    PB_FIELD(  1, BOOL    , REQUIRED, STATIC  , FIRST, MessageType_AxisInfo, simulation, simulation, 0),
    PB_FIELD(  2, BOOL    , REQUIRED, STATIC  , OTHER, MessageType_AxisInfo, communication_ready, simulation, 0),
    PB_FIELD(  3, BOOL    , REQUIRED, STATIC  , OTHER, MessageType_AxisInfo, ready_for_power_on, communication_ready, 0),
    PB_FIELD(  4, BOOL    , REQUIRED, STATIC  , OTHER, MessageType_AxisInfo, power_on, ready_for_power_on, 0),
    PB_LAST_FIELD
};


/* @@protoc_insertion_point(eof) */
