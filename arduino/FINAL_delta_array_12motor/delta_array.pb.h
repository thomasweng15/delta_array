/* Automatically generated nanopb header */
/* Generated by nanopb-0.4.5 */

#ifndef PB_DELTA_ARRAY_PB_H_INCLUDED
#define PB_DELTA_ARRAY_PB_H_INCLUDED
#include <pb.h>

#if PB_PROTO_HEADER_VERSION != 40
#error Regenerate this file with the current version of nanopb generator.
#endif

/* Struct definitions */
typedef struct _DeltaMessage { 
    int32_t id; 
    pb_size_t joint_pos_count;
    float joint_pos[12]; 
    bool request_done_state; 
    bool request_joint_pose; 
    bool reset; 
} DeltaMessage;


#ifdef __cplusplus
extern "C" {
#endif

/* Initializer values for message structs */
#define DeltaMessage_init_default                {0, 0, {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, 0, 0, 0}
#define DeltaMessage_init_zero                   {0, 0, {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, 0, 0, 0}

/* Field tags (for use in manual encoding/decoding) */
#define DeltaMessage_id_tag                      1
#define DeltaMessage_joint_pos_tag               2
#define DeltaMessage_request_done_state_tag      3
#define DeltaMessage_request_joint_pose_tag      4
#define DeltaMessage_reset_tag                   5

/* Struct field encoding specification for nanopb */
#define DeltaMessage_FIELDLIST(X, a) \
X(a, STATIC,   SINGULAR, INT32,    id,                1) \
X(a, STATIC,   REPEATED, FLOAT,    joint_pos,         2) \
X(a, STATIC,   SINGULAR, BOOL,     request_done_state,   3) \
X(a, STATIC,   SINGULAR, BOOL,     request_joint_pose,   4) \
X(a, STATIC,   SINGULAR, BOOL,     reset,             5)
#define DeltaMessage_CALLBACK NULL
#define DeltaMessage_DEFAULT NULL

extern const pb_msgdesc_t DeltaMessage_msg;

/* Defines for backwards compatibility with code written before nanopb-0.4.0 */
#define DeltaMessage_fields &DeltaMessage_msg

/* Maximum encoded size of messages (where known) */
#define DeltaMessage_size                        77

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif