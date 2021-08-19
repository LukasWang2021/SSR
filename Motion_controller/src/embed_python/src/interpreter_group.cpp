#include "interpreter_group.h"
#include "trajectory_datatype.h"
#include "interpreter_control.h"

using namespace group_space;
using namespace log_space;

static Instruction *mv_inst_;
static MotionControl **group_ptr_;

static void InterpGroup_PauseCall(void)
{
    InterpCtrl::instance().pause(0);
}

bool InterpGroup_Init(MotionControl **group_ptr)
{
    if(group_ptr == NULL) return false;
    group_ptr_ = group_ptr;

    mv_inst_ = new Instruction;
    if(mv_inst_ == NULL) return false;
    // initialize the instruction !!! do not memset
    mv_inst_->type = COMMON;
    mv_inst_->target.type = MOTION_NONE;
    mv_inst_->target.smooth_type = SMOOTH_NONE;
    mv_inst_->target.is_swift = false;
    mv_inst_->target.cnt = -1;
    mv_inst_->target.vel = 0;
    mv_inst_->target.acc = 1.0;
    mv_inst_->target.user_frame_id = -1;
    mv_inst_->target.tool_frame_id = -1;
    mv_inst_->interp_pause = InterpGroup_PauseCall;

    return true;
}

static bool trajInfo2MoveInst(MoveTrajInfo *traj)
{
    LogProducer::info("interpgroup", "SmoothType=%d,SmoothValue=%f,VEL=%f,TgtType=%d,ViaType=%d,ACC=%f,UFID=%d,TFID=%d",
    mv_inst_->target.smooth_type = (SmoothType)traj->smooth_type,
    mv_inst_->target.cnt = traj->smooth_value,
    mv_inst_->target.vel = traj->vel,
    mv_inst_->target.target.type = (CoordinateType)traj->tgt.coord,
    mv_inst_->target.via.type    = (CoordinateType)traj->aux.coord,
    mv_inst_->target.acc = traj->acc,
    mv_inst_->target.user_frame_id = traj->uf_id,
    mv_inst_->target.tool_frame_id = traj->tf_id);

    if(mv_inst_->target.target.type == COORDINATE_JOINT)
    {
        LogProducer::info("interpgroup", "Target JOINT:%f,%f,%f,%f,%f,%f,%f,%f,%f",
        mv_inst_->target.target.joint.j1_ = traj->tgt.pos[0],
        mv_inst_->target.target.joint.j2_ = traj->tgt.pos[1],
        mv_inst_->target.target.joint.j3_ = traj->tgt.pos[2],
        mv_inst_->target.target.joint.j4_ = traj->tgt.pos[3],
        mv_inst_->target.target.joint.j5_ = traj->tgt.pos[4],
        mv_inst_->target.target.joint.j6_ = traj->tgt.pos[5],
        mv_inst_->target.target.joint.j7_ = traj->tgt.pos[6],
        mv_inst_->target.target.joint.j8_ = traj->tgt.pos[7],
        mv_inst_->target.target.joint.j9_ = traj->tgt.pos[8]);
    }
    else if(mv_inst_->target.target.type == COORDINATE_CARTESIAN)
    {
        LogProducer::info("interpgroup", "Target POSE:%f,%f,%f,%f,%f,%f",
        mv_inst_->target.target.pose.pose.point_.x_ = traj->tgt.pos[0],
        mv_inst_->target.target.pose.pose.point_.y_ = traj->tgt.pos[1],
        mv_inst_->target.target.pose.pose.point_.z_ = traj->tgt.pos[2],
        mv_inst_->target.target.pose.pose.euler_.a_ = traj->tgt.pos[3],
        mv_inst_->target.target.pose.pose.euler_.b_ = traj->tgt.pos[4],
        mv_inst_->target.target.pose.pose.euler_.c_ = traj->tgt.pos[5]);

        LogProducer::info("interpgroup", "Target POSTURE:%d,%d,%d,%d",
        mv_inst_->target.target.pose.posture.arm = traj->tgt.posture[0],
        mv_inst_->target.target.pose.posture.elbow = traj->tgt.posture[1],
        mv_inst_->target.target.pose.posture.wrist = traj->tgt.posture[2],
        mv_inst_->target.target.pose.posture.flip = traj->tgt.posture[3]);
        
        LogProducer::info("interpgroup", "Target TURN:%d,%d,%d,%d,%d,%d,%d,%d,%d",
        mv_inst_->target.target.pose.turn.j1 = traj->tgt.turn[0],
        mv_inst_->target.target.pose.turn.j2 = traj->tgt.turn[1],
        mv_inst_->target.target.pose.turn.j3 = traj->tgt.turn[2],
        mv_inst_->target.target.pose.turn.j4 = traj->tgt.turn[3],
        mv_inst_->target.target.pose.turn.j5 = traj->tgt.turn[4],
        mv_inst_->target.target.pose.turn.j6 = traj->tgt.turn[5],
        mv_inst_->target.target.pose.turn.j7 = traj->tgt.turn[6],
        mv_inst_->target.target.pose.turn.j8 = traj->tgt.turn[7],
        mv_inst_->target.target.pose.turn.j9 = traj->tgt.turn[8]);
    }
    else
    {
        LogProducer::error("interpgroup", "target coordiante type invalid");
        return false;
    }
    

    if(mv_inst_->target.via.type == COORDINATE_JOINT)
    {
        LogProducer::info("interpgroup", "Via JOINT:%f,%f,%f,%f,%f,%f,%f,%f,%f",
        mv_inst_->target.via.joint.j1_ = traj->aux.pos[0],
        mv_inst_->target.via.joint.j2_ = traj->aux.pos[1],
        mv_inst_->target.via.joint.j3_ = traj->aux.pos[2],
        mv_inst_->target.via.joint.j4_ = traj->aux.pos[3],
        mv_inst_->target.via.joint.j5_ = traj->aux.pos[4],
        mv_inst_->target.via.joint.j6_ = traj->aux.pos[5],
        mv_inst_->target.via.joint.j7_ = traj->aux.pos[6],
        mv_inst_->target.via.joint.j8_ = traj->aux.pos[7],
        mv_inst_->target.via.joint.j9_ = traj->aux.pos[8]);
    }
    else if(mv_inst_->target.via.type == COORDINATE_CARTESIAN)
    {
        LogProducer::info("interpgroup", "Via POSE:%f,%f,%f,%f,%f,%f",
        mv_inst_->target.via.pose.pose.point_.x_ = traj->aux.pos[0],
        mv_inst_->target.via.pose.pose.point_.y_ = traj->aux.pos[1],
        mv_inst_->target.via.pose.pose.point_.z_ = traj->aux.pos[2],
        mv_inst_->target.via.pose.pose.euler_.a_ = traj->aux.pos[3],
        mv_inst_->target.via.pose.pose.euler_.b_ = traj->aux.pos[4],
        mv_inst_->target.via.pose.pose.euler_.c_ = traj->aux.pos[5]);

        LogProducer::info("interpgroup", "Via POSTURE:%d,%d,%d,%d",
        mv_inst_->target.via.pose.posture.arm = traj->aux.posture[0],
        mv_inst_->target.via.pose.posture.elbow = traj->aux.posture[1],
        mv_inst_->target.via.pose.posture.wrist = traj->aux.posture[2],
        mv_inst_->target.via.pose.posture.flip = traj->aux.posture[3]);
        
        LogProducer::info("interpgroup", "Via TURN:%d,%d,%d,%d,%d,%d,%d,%d,%d",
        mv_inst_->target.via.pose.turn.j1 = traj->aux.turn[0],
        mv_inst_->target.via.pose.turn.j2 = traj->aux.turn[1],
        mv_inst_->target.via.pose.turn.j3 = traj->aux.turn[2],
        mv_inst_->target.via.pose.turn.j4 = traj->aux.turn[3],
        mv_inst_->target.via.pose.turn.j5 = traj->aux.turn[4],
        mv_inst_->target.via.pose.turn.j6 = traj->aux.turn[5],
        mv_inst_->target.via.pose.turn.j7 = traj->aux.turn[6],
        mv_inst_->target.via.pose.turn.j8 = traj->aux.turn[7],
        mv_inst_->target.via.pose.turn.j9 = traj->aux.turn[8]);
    }
    else
    {
        LogProducer::error("interpgroup", "via coordiante type invalid");
        return false;
    }
    return true;
}

ErrorCode InterpGroup_MoveJoint(int gid, MoveTrajInfo *traj)
{
    ErrorCode ret = 0;
    mv_inst_->type = MOTION;
    mv_inst_->target.type = MOTION_JOINT;
    if(!trajInfo2MoveInst(traj))
        return INTERPRETER_ERROR_TRAJ_INFO_INVALID;

    if(!InterpCtrl::instance().runSyncCallback())
        return INTERPRETER_ERROR_SYNC_CALL_FAILED;

    LogProducer::info("interpgroup", "move joint");
    ret = group_ptr_[gid]->autoMove(*mv_inst_);
    return ret;
}

ErrorCode InterpGroup_MoveLiner(int gid, MoveTrajInfo *traj)
{
    ErrorCode ret = 0;
    mv_inst_->type = MOTION;
    mv_inst_->target.type = MOTION_LINE;
    if(!trajInfo2MoveInst(traj))
        return INTERPRETER_ERROR_TRAJ_INFO_INVALID;

    if(!InterpCtrl::instance().runSyncCallback())
        return INTERPRETER_ERROR_SYNC_CALL_FAILED;

    LogProducer::info("interpgroup", "move liner");
    ret = group_ptr_[gid]->autoMove(*mv_inst_);
    return ret;
}

ErrorCode InterpGroup_MoveCircl(int gid, MoveTrajInfo *traj)
{
    ErrorCode ret = 0;
    mv_inst_->type = MOTION;
    mv_inst_->target.type = MOTION_CIRCLE;
    if(!trajInfo2MoveInst(traj))
        return INTERPRETER_ERROR_TRAJ_INFO_INVALID;

    if(!InterpCtrl::instance().runSyncCallback())
        return INTERPRETER_ERROR_SYNC_CALL_FAILED;

    LogProducer::info("interpgroup", "move circle");
    ret = group_ptr_[gid]->autoMove(*mv_inst_);
    return ret;
}

ErrorCode InterpGroup_SetOVC(int gid, double val)
{
    ErrorCode ret = 0;
    mv_inst_->type = SET_OVC;
    mv_inst_->ovc = val;
    if(!InterpCtrl::instance().runSyncCallback())
        return INTERPRETER_ERROR_SYNC_CALL_FAILED;
    LogProducer::info("interpgroup", "set ovc %f", mv_inst_->ovc);
    ret = group_ptr_[gid]->autoMove(*mv_inst_);
    return ret;
}

ErrorCode InterpGroup_SetOAC(int gid, double val)
{
    ErrorCode ret = 0;
    mv_inst_->type = SET_OAC;
    mv_inst_->oac = val;
    if(!InterpCtrl::instance().runSyncCallback())
        return INTERPRETER_ERROR_SYNC_CALL_FAILED;
    LogProducer::info("interpgroup", "set oac %f", mv_inst_->oac);
    ret = group_ptr_[gid]->autoMove(*mv_inst_);
    return ret;
}

ErrorCode InterpGroup_SetPLD(int gid, int val)
{
    ErrorCode ret = 0;
    mv_inst_->type = SET_PAYLOAD;
    mv_inst_->payload_id = val;
    if(!InterpCtrl::instance().runSyncCallback())
        return INTERPRETER_ERROR_SYNC_CALL_FAILED;
    LogProducer::info("interpgroup", "set payload %d", mv_inst_->payload_id);
    ret = group_ptr_[gid]->autoMove(*mv_inst_);
    return ret;
}

ErrorCode InterpGroup_SetUF(int gid, int val)
{
    ErrorCode ret = 0;
    mv_inst_->type = SET_UF;
    mv_inst_->uf_id = val;
    if(!InterpCtrl::instance().runSyncCallback())
        return INTERPRETER_ERROR_SYNC_CALL_FAILED;
    LogProducer::info("interpgroup", "set uf %d", mv_inst_->uf_id);
    ret = group_ptr_[gid]->autoMove(*mv_inst_);
    return ret;
}

ErrorCode InterpGroup_SetTF(int gid, int val)
{
    ErrorCode ret = 0;
    mv_inst_->type = SET_TF;
    mv_inst_->tf_id = val;
    if(!InterpCtrl::instance().runSyncCallback())
        return INTERPRETER_ERROR_SYNC_CALL_FAILED;
    LogProducer::info("interpgroup", "set tf %d", mv_inst_->tf_id);
    ret = group_ptr_[gid]->autoMove(*mv_inst_);
    return ret;
}