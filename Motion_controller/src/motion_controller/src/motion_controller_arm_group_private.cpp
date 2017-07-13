/*************************************************************************
	> File Name: motion_controller_arm_group_private.cpp
	> Author: 
	> Mail: 
	> Created Time: 2017年03月02日 星期四 15时48分17秒
 ************************************************************************/

#include <motion_controller/motion_controller_arm_group.h>

using std::cout;
using std::endl;
using std::string;
using std::vector;

namespace fst_controller {

static const unsigned int UNDEFINED     = 0x5555;
static const unsigned int INITIALIZED   = 0x5556;

//------------------------------------------------------------------------------
// Function:    setCycleTime
// Summary: To set cycle time of interpolation algorithm.
// In:      tc  -> desired cycle time
// Out:     None
// Return:  true    -> cycle time setted to given value
//          false   -> cycle time NOT changed
//------------------------------------------------------------------------------
bool ArmGroup::setCycleTime(double tc) {
    // if (current_state_ != INITIALIZED)  return false;

    if (planning_interface_->setCycleTime(tc)) {
        log.info("Set cycle time to %.4fs", tc);
        return true;
    }
    else {
        log.error("Cannot set cycle time to %.4fs", tc);
        return false;
    }
}

//------------------------------------------------------------------------------
// Function:    setJointConstraints
// Summary: To set joint constraints in Kinematics algorithm.
// In:      constraints -> joint constraints
// Out:     None
// Return:  true    -> set successfully
//          false   -> set UNsuccessfully
//------------------------------------------------------------------------------
bool ArmGroup::setJointConstraints(const JointConstraints &constraints) {
    // if (current_state_ != INITIALIZED)  return false;
    
    joint_constraints_ = constraints;
    printJointConstraints(constraints);
    return planning_interface_->setJointConstraints(constraints);
}


//------------------------------------------------------------------------------
// Function:    setJointOvershoot
// Summary: To set joint overshoot.
// In:      angle   -> desired angle
// Out:     None
// Return:  true    -> joint overshoot changed to given value
//          false   -> joint overshoot NOT changed
//------------------------------------------------------------------------------
bool ArmGroup::setJointOvershoot(double angle) {
    // if (current_state_ != INITIALIZED)  return false;

    if (planning_interface_->setJointOvershoot(angle)) {
        log.info("Set joint overshoot to %.4f rad", angle);
        return true;
    }
    else {
        log.error("Cannot set joint overshoot to %.4f rad", angle);
        return false;
    }
}

//------------------------------------------------------------------------------
// Function:    setJointErrorAngle
// Summary: To set joint error angle.
// In:      angle   -> desired angle
// Out:     None
// Return:  true    -> joint error angle changed to given value
//          false   -> joint error angle NOT changed
//------------------------------------------------------------------------------
bool ArmGroup::setJointErrorAngle(double angle) {
    // if (current_state_ != INITIALIZED)  return false;

    if (planning_interface_->setJointErrorAngle(angle)) {
        log.info("Set joint error angle to %.4f rad", angle);
        return true;
    }
    else {
        log.error("Cannot set joint error angle to %.4f rad", angle);
        return false;
    }
}

//------------------------------------------------------------------------------
// Function:    setOmegaOverload
// Summary: To set omega overload.
// In:      value   -> desired value
// Out:     None
// Return:  true    -> omega overload changed to given value
//          false   -> omega overload NOT changed
//------------------------------------------------------------------------------
bool ArmGroup::setOmegaOverload(double value) {
    // if (current_state_ != INITIALIZED)  return false;

    if (planning_interface_->setOmegaOverload(value)) {
        log.info("Set omega overload to %.2f%%", value * 100);
        return true;
    }
    else {
        log.error("Cannot set omega overload to %.2f%%", value * 100);
        return false;
    }
}

//------------------------------------------------------------------------------
// Function:    setAlphaOverload
// Summary: To set alpha overload.
// In:      value   -> desired value
// Out:     None
// Return:  true    -> alpha overload changed to given value
//          false   -> alpha overload NOT changed
//------------------------------------------------------------------------------
bool ArmGroup::setAlphaOverload(double value) {
    // if (current_state_ != INITIALIZED)  return false;

    if (planning_interface_->setAlphaOverload(value)) {
        log.info("Set alpha overload to %.2f%%", value * 100);
        return true;
    }
    else {
        log.error("Cannot set alpha overload to %.2f%%", value * 100);
        return false;
    }
}

//------------------------------------------------------------------------------
// Function:    setSmoothRadiusCoefficient
// Summary: To set smooth radius coefficient.
// In:      coeff   -> desired value
// Out:     None
// Return:  true    -> smooth radius coefficient changed to given value
//          false   -> smooth radius coefficient NOT changed
//------------------------------------------------------------------------------
bool ArmGroup::setSmoothRadiusCoefficient(double coeff) {
    // if (current_state_ != INITIALIZED)  return false;

    if (planning_interface_->setSmoothRadiusCoefficient(coeff)) {
        log.info("Set smooth radius coefficient to %.2f", coeff);
        return true;
    }
    else {
        log.error("Cannot set smooth radius coefficient to %.2f", coeff);
        return false;
    }
}

void ArmGroup::setStartJointImpl(const JointValues &joint) {
    m_joint_start.joints = joint;
    memset(&m_joint_start.omegas, 0, sizeof(JointOmegas));
}

void ArmGroup::setStartPoseImpl(const Pose &pose) {
    m_pose_start    = pose;
    m_pose_previous = pose;
    m_v_start       = 0.0;
    m_vu_start      = 0.0;
}

bool ArmGroup::setStartStateImpl(const JointValues &joint, ErrorCode &err) {
    Pose pose;
    if (planning_interface_->computeForwardKinematics(joint, pose, err)) {
        setStartJointImpl(joint);
        setStartPoseImpl(pose);
        return true;
    }
    else {
        return false;
    }
}

bool ArmGroup::rebuildPlanningVariable(ErrorCode &err) {
    //allowed_motion_type_ = MOTION_UNDEFINED;
    if (!planned_path_fifo_.empty()) {
        if (planned_path_fifo_.back().type == MOTION_JOINT) {
            return setStartStateImpl(planned_path_fifo_.back().joints, err);
        }
        else if (planned_path_fifo_.back().type == MOTION_LINE
                 || planned_path_fifo_.back().type == MOTION_CIRCLE) {
            m_pose_start    = planned_path_fifo_.back().pose;
            m_pose_previous = m_pose_start;
            m_v_start   = 0.0;
            m_vu_start  = 0.0;
            return true;
        }
        else {
            err = MOTION_INTERNAL_FAULT;
            return false;
        }
    }
    else if (!trajectory_fifo_.empty()) {
        return setStartStateImpl(trajectory_fifo_.back().joints, err);
    }
    else {
        return setStartStateImpl(getLatestIKReference(), err);
    }
}

void ArmGroup::abstractLastMotion(int id, MotionType motion_t, SmoothType smooth_t) {
    last_motion_.id = id;
    last_motion_.smooth_type = smooth_t;
    last_motion_.motion_type = motion_t;
}

/*
bool ArmGroup::fillPlannedPathFIFO(int id, MotionType motion_t, SmoothType smooth_t, vector<Pose> &path) {
    if (motion_t != MOTION_LINE && motion_t != MOTION_CIRCLE || path.empty()) {
        return false;
    }

    PathPoint pp;
    pp.type = motion_t;

    if (last_motion_.smooth_type == SMOOTH_NONE) {
        pp.id = (id << 2) + POINT_START;
        pp.pose = path.front();
        planned_path_fifo_.push_back(pp);
        pp.id = (id << 2) + POINT_MIDDLE;

        vector<Pose>::iterator itr = path.begin() + 1;
        for ( ; itr != path.end(); ++itr) {
            pp.pose = *itr;
            planned_path_fifo_.push_back(pp);
        }
    }
    else {
        pp.id = (id << 2) + POINT_MIDDLE;

        vector<Pose>::iterator itr = path.begin();
        for ( ; itr != path.end(); ++itr) {
            pp.pose = *itr;
            planned_path_fifo_.push_back(pp);
        }
    }

    if (smooth_t == SMOOTH_NONE) {
        planned_path_fifo_.back().id = (id << 2) + POINT_ENDING;
    }

    return true;
}

bool ArmGroup::fillPlannedPathFIFO(int id, MotionType motion_t, SmoothType smooth_t, vector<JointValues> &path) {
    if (motion_t != MOTION_JOINT || path.empty()) {
        return false;
    }

    PathPoint pp;
    pp.type = motion_t;

    if (last_motion_.smooth_type == SMOOTH_NONE) {
        pp.id = (id << 2) + POINT_START;
        pp.joints = path.front();
        planned_path_fifo_.push_back(pp);
        pp.id = (id << 2) + POINT_MIDDLE;

        vector<JointValues>::iterator itr = path.begin() + 1;
        for ( ; itr != path.end(); ++itr) {
            pp.joints = *itr;
            planned_path_fifo_.push_back(pp);
        }
    }
    else {
        pp.id = (id << 2) + POINT_MIDDLE;

        vector<JointValues>::iterator itr = path.begin();
        for ( ; itr != path.end(); ++itr) {
            pp.joints = *itr;
            planned_path_fifo_.push_back(pp);
        }
    }

    if (smooth_t == SMOOTH_NONE) {
        planned_path_fifo_.back().id = (id << 2) + POINT_ENDING;
    }

    return true;
}*/

bool ArmGroup::managePlanningResult(int id, MotionType motion_t, SmoothType smooth_t, ErrorCode &err) {
    bool result = true;
    switch (smooth_t) {
        case SMOOTH_J2J:
        case SMOOTH_L2J:
        case SMOOTH_C2J:    
                allowed_motion_type_ = MOTION_JOINT;
                break;
        case SMOOTH_J2L:
        case SMOOTH_L2L:
        case SMOOTH_C2L:
                allowed_motion_type_ = MOTION_LINE;
                break;
        case SMOOTH_J2C:
        case SMOOTH_L2C:
        case SMOOTH_C2C:
                allowed_motion_type_ = MOTION_CIRCLE;
                break;
        case SMOOTH_NONE:
                allowed_motion_type_ = MOTION_UNDEFINED;
                break;
        default:
                err    = MOTION_INTERNAL_FAULT;
                result = false;
    }

    if (last_motion_.smooth_type == SMOOTH_NONE) {
        size_t length = planned_path_fifo_.size();
        abstractLastMotion(id, motion_t, smooth_t);
        result = result && pickPoints(err);
        if (result) planned_path_fifo_.at(length).id = last_motion_.id << 2 | POINT_START;
    }
    else {
        abstractLastMotion(id, motion_t, smooth_t);
        result = result && pickPoints(err);
    }

    return result;
}

//------------------------------------------------------------------------------
// Function:    setLatestIKReference
// Summary: To set latest IK reference.
// In:      joint_reference -> new IK reference
// Out:     error_code  -> error code
// Return:  true    -> latest IK reference setted to joint_reference scucessfully
//          false   -> failed to set latest IK reference
//------------------------------------------------------------------------------
bool ArmGroup::setLatestIKReference(const JointValues &joint_reference, ErrorCode &err) {
    // if (current_state_ != INITIALIZED) {err = NEED_INITIALIZATION; return false;}

    if (checkJointBoundary(joint_reference)) {
        latest_ik_reference_ = joint_reference;
        return true;
    }
    else {
        err = JOINT_OUT_OF_CONSTRAINT;
        return false;
    }
}

//------------------------------------------------------------------------------
// Function:    getLatestIKReference
// Summary: To get latest IK reference values.
// In:      None
// Out:     None
// Return:  a group of joint values used as IK reference
//------------------------------------------------------------------------------
const JointValues& ArmGroup::getLatestIKReference(void) {
    return latest_ik_reference_;
}

//------------------------------------------------------------------------------
// Function:    checkJointBoundary
// Summary: To check whether a group of joint values are valid according to
//          joint constraints.
// In:      joint_values -> joint_values needed to be checked
// Out:     None
// Return:  true  -> valid
//------------------------------------------------------------------------------
bool ArmGroup::checkJointBoundary(const JointValues &joint_values) {
    return joint_values.j1 > joint_constraints_.j1.lower &&
           joint_values.j1 < joint_constraints_.j1.upper &&
           joint_values.j2 > joint_constraints_.j2.lower &&
           joint_values.j2 < joint_constraints_.j2.upper &&
           joint_values.j3 > joint_constraints_.j3.lower &&
           joint_values.j3 < joint_constraints_.j3.upper &&
           joint_values.j4 > joint_constraints_.j4.lower &&
           joint_values.j4 < joint_constraints_.j4.upper &&
           joint_values.j5 > joint_constraints_.j5.lower &&
           joint_values.j5 < joint_constraints_.j5.upper &&
           joint_values.j6 > joint_constraints_.j6.lower &&
           joint_values.j6 < joint_constraints_.j6.upper;
}

//------------------------------------------------------------------------------
// Function:    convertPath2Trajectory
// Summary: To convert numbers of posepoint in m_cartesian_path_FIFO
//          into jointpoint in trajectory_fifo_.
// In:      num        -> number of pose that needed to be converted
// Out:     error_code -> error code
// Return:  <0         -> ERROR occurred during converting
//          >=0        -> number of pose that convered actually
//------------------------------------------------------------------------------
int ArmGroup::convertPath2Trajectory(int num, ErrorCode &err) {
    if (planned_path_fifo_.size() < planning_interface_->getTrajectorySegmentLength() &&
            planning_interface_->getFIFOLength() > 0) {
        pickPoints(err);
    }

    if (trajectory_fifo_dynamic_length_ <= getJointTrajectoryFIFOLength()) {
        num = 0;
    }
    else if (num > trajectory_fifo_dynamic_length_ - getJointTrajectoryFIFOLength()) {
        num = trajectory_fifo_dynamic_length_ - getJointTrajectoryFIFOLength();
    }

    if (num > planned_path_fifo_.size()) {
        num = planned_path_fifo_.size();
    }

    int conv_cnt = 0;
    vector<PathPoint>::iterator itr = planned_path_fifo_.begin();
    JointPoint jp;
    
    for (conv_cnt = 0; conv_cnt < num; ++conv_cnt) {
        if (itr->type == MOTION_LINE || itr->type == MOTION_CIRCLE) {
            if (computeIK(itr->pose, jp.joints, err)) {
                if (itr == planned_path_fifo_.end() - 1) {
                    if (last_motion_.smooth_type == SMOOTH_NONE) {
                        setStartStateImpl(jp.joints, err);
                    }
                    else if (last_motion_.smooth_type == SMOOTH_L2J ||
                             last_motion_.smooth_type == SMOOTH_C2J) {
                        m_joint_start.joints = jp.joints;
                        JointValues joints = getLatestIKReference();
                        double cycle_time = planning_interface_->getCycleTime();
                        m_joint_start.omegas.j1 = (jp.joints.j1 - joints.j1) / cycle_time;
                        m_joint_start.omegas.j2 = (jp.joints.j2 - joints.j2) / cycle_time;
                        m_joint_start.omegas.j3 = (jp.joints.j3 - joints.j3) / cycle_time;
                        m_joint_start.omegas.j4 = (jp.joints.j4 - joints.j4) / cycle_time;
                        m_joint_start.omegas.j5 = (jp.joints.j5 - joints.j5) / cycle_time;
                        m_joint_start.omegas.j6 = (jp.joints.j6 - joints.j6) / cycle_time;
                    }
                    else {
                        // Nothing to do
                    }
                    /*
                    if (last_motion_.addition_smooth) {
                        insertAdditionSmooth(err);
                        // last_motion_.smooth_type = SMOOTH_NONE;
                        last_motion_.addition_smooth = false;
                    }*/
                }
                jp.id = itr->id;
                trajectory_fifo_.push_back(jp);
                latest_ik_reference_ = jp.joints;
            }
            else {
                log.error("Error while converting cartesian point to joint space, Error code=0x%llx", err);
                log.error("%d points has been converted before the error", conv_cnt);
                printJointValues("IK reference: ", latest_ik_reference_);
                printJointValues("IK result:    ", jp.joints);
                printPose(       "Target Pose:  ", itr->pose);
                // ------------------- FOR TEST ONLY --------------------
                /*
                Pose tmp;
                computeFK(m_latest_ik_reference, tmp, err);
                printPose("reference: ", tmp);
                computeFK(jp.joints, tmp, err);
                printPose("result: ", tmp);
                */
                // ^^^^^^^^^^^^^^^^^^^ FOR TEST ONLY ^^^^^^^^^^^^^^^^^^^^
                break;
            }
        }
        else if (itr->type == MOTION_JOINT) {
            jp.id = itr->id;
            jp.joints = itr->joints;
            trajectory_fifo_.push_back(jp);
            if (itr == planned_path_fifo_.end() - 1) {
                if (last_motion_.smooth_type == SMOOTH_NONE) {
                    setStartStateImpl(itr->joints, err);
                }
                else if (last_motion_.smooth_type == SMOOTH_J2J) {
                    m_joint_start.joints = itr->joints;
                    JointValues joints = getLatestIKReference();
                    double cycle_time = planning_interface_->getCycleTime();
                    m_joint_start.omegas.j1 = (jp.joints.j1 - joints.j1) / cycle_time;
                    m_joint_start.omegas.j2 = (jp.joints.j2 - joints.j2) / cycle_time;
                    m_joint_start.omegas.j3 = (jp.joints.j3 - joints.j3) / cycle_time;
                    m_joint_start.omegas.j4 = (jp.joints.j4 - joints.j4) / cycle_time;
                    m_joint_start.omegas.j5 = (jp.joints.j5 - joints.j5) / cycle_time;
                    m_joint_start.omegas.j6 = (jp.joints.j6 - joints.j6) / cycle_time;
                }
                else {
                    // Nothing to do
                }
            }
            latest_ik_reference_ = jp.joints;
        }
        else {
            err = MOTION_INTERNAL_FAULT;
            break;
        }

        ++itr;
    }
    
    if (conv_cnt != 0) {
        itr = planned_path_fifo_.begin();
        planned_path_fifo_.erase(itr, itr + conv_cnt);
    }

    return conv_cnt;
}

bool ArmGroup::isMotionExecutable(MotionType motion_type, ErrorCode &err) {
    if (planning_interface_->getFIFOLength() > planning_interface_->getTrajectorySegmentLength()) {
        err = INVALID_SEQUENCE;
        return false;
    }
    if (planning_interface_->getFIFOLength() > 0) {
        if (pickPoints(err) != true)    return false;
    }

    switch (motion_type) {
        case MOTION_JOINT:
            if (allowed_motion_type_ != MOTION_JOINT && allowed_motion_type_ != MOTION_UNDEFINED) {
                err = INVALID_SEQUENCE;
                return false;
            }
            if (!planned_path_fifo_.empty() &&
                    (planned_path_fifo_.back().type == MOTION_LINE ||
                     planned_path_fifo_.back().type == MOTION_CIRCLE)) {
                err = CARTESIAN_PATH_EXIST;
                return false;
            }
            return true;
        case MOTION_LINE:
            if (allowed_motion_type_ != MOTION_LINE && allowed_motion_type_ != MOTION_UNDEFINED) {
                err = INVALID_SEQUENCE;
                return false;
            }
            if (enable_calibration_ == true && calibrator_->getCurrentState() < CALIBRATED) {
                err = NEED_CALIBRATION;
                return false;
            }
            /*
            if (last_motion_.addition_smooth && !planned_path_fifo_.empty()) {
                err = CARTESIAN_PATH_EXIST;
                return false;
            }
            else {
                return true;
            }*/
            return true;
        case MOTION_CIRCLE:
            if (allowed_motion_type_ != MOTION_CIRCLE && allowed_motion_type_ != MOTION_UNDEFINED) {
                err = INVALID_SEQUENCE;
                return false;
            }
            if (enable_calibration_ == true && calibrator_->getCurrentState() < CALIBRATED) {
                err = NEED_CALIBRATION;
                return false;
            }
            /*
            if (last_motion_.addition_smooth && !planned_path_fifo_.empty()) {
                err = CARTESIAN_PATH_EXIST;
                return false;
            }
            else {
                return true;
            }*/
            return true;
        default:
            err = MOTION_INTERNAL_FAULT;
            log.error("API:'isMotionExecutable' received an invalid motion type: %d", motion_type);
            return false;
    }
}

bool ArmGroup::pickPoints(ErrorCode &err) {
    resizeJointTrajectoryFIFO();
    log.info("Pick points ...");
    if (planning_interface_->getFIFOLength() > 0) {
        if (last_motion_.motion_type == MOTION_JOINT) {
            vector<JointValues> points;
            if (planning_interface_->pickPoints(points, err)) {
                PathPoint pp;
                pp.id   = last_motion_.id << 2 | POINT_MIDDLE;
                pp.type = last_motion_.motion_type;
                vector<JointValues>::iterator it;
                for(it = points.begin(); it != points.end(); ++it) {
                    pp.joints = *it;
                    planned_path_fifo_.push_back(pp);
                }
                if (planning_interface_->getFIFOLength() == 0 && last_motion_.smooth_type == SMOOTH_NONE)
                    planned_path_fifo_.back().id = last_motion_.id << 2 | POINT_ENDING;
                log.info("  %d points have been picked into FIFO1, and %d points left",
                        points.size(), planning_interface_->getFIFOLength());
                return true;
            }
            else {
                log.error("Fail to pick points, error_code=0x%llx", err);
                return false;
            }
        }
        else {
            vector<Pose> points;
            if (planning_interface_->pickPoints(points, err)) {
                PathPoint pp;
                pp.id   = last_motion_.id << 2 | POINT_MIDDLE;
                pp.type = last_motion_.motion_type;
                vector<Pose>::iterator it;
                for(it = points.begin(); it != points.end(); ++it) {
                    pp.pose = *it;
                    planned_path_fifo_.push_back(pp);
                }
                if (planning_interface_->getFIFOLength() == 0 && last_motion_.smooth_type == SMOOTH_NONE)
                    planned_path_fifo_.back().id = last_motion_.id << 2 | POINT_ENDING;
                log.info("  %d points have been picked into FIFO1, and %d points left",
                        points.size(), planning_interface_->getFIFOLength());
                return true;
            }
            else {
                log.error("Fail to pick points, error_code=0x%llx", err);
                return false;
            }
        }
    }
    else {
        return false;
    }
}

bool ArmGroup::resizeJointTrajectoryFIFO(void) {
    log.info("Resize FIFO2 ...");
    if (trajectory_fifo_.size() >= 2) {
        vector<JointPoint>::iterator it1 = trajectory_fifo_.end() - 1;
        vector<JointPoint>::iterator it2 = trajectory_fifo_.end() - 2;
        int size = planning_interface_->estimateFIFOLength(it1->joints, it2->joints);
        if      (size < 50)     size = 50;
        else if (size > 500)    size = 500;
        
        trajectory_fifo_dynamic_length_ = size;
        log.info("  success! FIFO2 resized to %d", trajectory_fifo_dynamic_length_);
        return true;
    }
    else {
        log.error("  failed: no enough points in FIFO2, need 2 but has %d only", trajectory_fifo_.size());
        return false;
    }
}

/*
bool ArmGroup::insertAdditionSmooth(ErrorCode &err) {
    log.info("Insert additional smooth ...");
    vector<JointValues> planned_path;

    if (last_motion_.smooth_type == SMOOTH_L2C) {
        log.info("  Planning smooth path from MoevL to MoveC");
        if (planning_interface_->MoveL2CAdditionSmooth(m_joint_start, m_pose_start,
                                                       m_pose_start_past, planned_path, err)) {
            if (setLatestIKReference(planned_path.back(), err)) {
                PathPoint pp;
                pp.type = MOTION_JOINT;
                pp.id = (last_motion_.id << 2) + POINT_MIDDLE;

                vector<JointValues>::iterator itr = planned_path.begin();
                for (; itr != planned_path.end(); ++itr) {
                    pp.joints = *itr;
                    planned_path_fifo_.push_back(pp);
                }
                log.info("  Smooth path generated successfully with %zd points, and added into planned_path_FIFO.",
                         planned_path.size());
                return true;
            }
            else {
                log.error("  Cannot set new IK reference, error code=0x%llx", err);
                return false;
            }
        }
        else {
            log.error("  Fail to plan additional smooth path, error code=0x%llx", err);
            return false;
        }
    }
    else if (last_motion_.smooth_type == SMOOTH_C2L) {
        log.info("  Planning smooth path from MoevC to MoveL");
        if (planning_interface_->MoveC2LAdditionSmooth(m_joint_start, m_pose_start,
                                                       m_pose_start_past, planned_path, err)) {
            if (setLatestIKReference(planned_path.back(), err)) {
                PathPoint pp;
                pp.type = MOTION_JOINT;
                pp.id = (last_motion_.id << 2) + POINT_MIDDLE;

                vector<JointValues>::iterator itr = planned_path.begin();
                for (; itr != planned_path.end(); ++itr) {
                    pp.joints = *itr;
                    planned_path_fifo_.push_back(pp);
                }
                log.info("  Smooth path generated successfully with %zd points, and added into planned_path_FIFO.",
                         planned_path.size());
                return true;
            }
            else {
                log.error("  Cannot set new IK reference, error code=0x%llx", err);
                return false;
            }
        }
        else {
            log.error("  Fail to plan additional smooth path, error code=0x%llx", err);
            return false;
        }
    }
    else if (last_motion_.smooth_type == SMOOTH_C2C) {
        log.info("  Planning smooth path from MoevC to MoveC");
        if (planning_interface_->MoveC2CAdditionSmooth(m_joint_start, m_pose_start,
                                                       m_pose_start_past, planned_path, err)) {
            if (setLatestIKReference(planned_path.back(), err)) {
                PathPoint pp;
                pp.type = MOTION_JOINT;
                pp.id = (last_motion_.id << 2) + POINT_MIDDLE;

                vector<JointValues>::iterator itr = planned_path.begin();
                for (; itr != planned_path.end(); ++itr) {
                    pp.joints = *itr;
                    planned_path_fifo_.push_back(pp);
                }
                log.info("  Smooth path generated successfully with %zd points,  and added into planned_path_FIFO.",
                         planned_path.size());
                return true;
            }
            else {
                log.error("  Cannot set new IK reference, error code=0x%llx", err);
                return false;
            }
        }
        else {
            log.error("  Fail to plan additional smooth path, error code=0x%llx", err);
            return false;
        }
    }
    else {
        err = MOTION_INTERNAL_FAULT;
        log.error("  Unexpected smooth type, error code=0x%llx", err);
        return false;
    }
}*/

bool ArmGroup::resumeByOrigin(ErrorCode &err) {
    trajectory_fifo_.insert(trajectory_fifo_.begin(),
                            suspend_state_.fifo2_cache.begin(),
                            suspend_state_.fifo2_cache.end());
    suspend_state_.is_suspended = false;
    log.info("Resume successfully, pattern=origin, trajectory length=%zd", trajectory_fifo_.size());
    return true;
}

bool ArmGroup::resumeByReplan(ErrorCode &err) {
    int num = planned_path_fifo_.size() < 100 ? planned_path_fifo_.size() : 100;
    if (convertPath2Trajectory(num, err) == num) {
        bool last_point_is_ending_point = false;
        suspend_state_.replan_trajectory.clear();
        vector<JointPoint>::iterator itr = trajectory_fifo_.begin();
        for ( ; itr != trajectory_fifo_.end(); ++itr) {
            suspend_state_.replan_trajectory.push_back(itr->joints);
            if ((itr->id & POINT_LEVEL_MASK) == POINT_ENDING) {
                log.info("The last point is ending point.");
                last_point_is_ending_point = true;
            }
        }
        bool result = planning_interface_->replanRestartTrajectory(suspend_state_.replan_trajectory,
                                                                   suspend_state_.last_point,
                                                                   err);
        if (result == true) {
            JointPoint jp;
            memset(&jp.omegas, 0, sizeof(JointOmegas));
            jp.id = trajectory_fifo_.front().id;
            jp.id = jp.id & ~POINT_LEVEL_MASK | POINT_MIDDLE;
            trajectory_fifo_.clear();
            vector<JointValues>::iterator itr = suspend_state_.replan_trajectory.begin();
            for ( ; itr != suspend_state_.replan_trajectory.end(); ++itr) {
                jp.joints = *itr;
                trajectory_fifo_.push_back(jp);
            }
            log.info("Resume pattern: replan, trajectory length: %d", trajectory_fifo_.size());
            if (last_point_is_ending_point && !trajectory_fifo_.empty()) {
                trajectory_fifo_.back().id &= ~POINT_LEVEL_MASK;
                trajectory_fifo_.back().id |= POINT_ENDING;
                log.info("Turning the last point to ending point.");
            }
        }
        else {
            suspend_state_.is_suspended = false;
            log.error("ERROR while replanning resume trajectory, error_code=%d", result);
            return false;
        }
    }
    else {
        suspend_state_.is_suspended = false;
        log.error("ERROR while filling FIFO2, error_code=0x%llx", err);
        return false;
    }
    suspend_state_.is_suspended = false;
    log.info("Resume successfully.");
    return true;
}

void ArmGroup::printJointValues(const JointValues &joint) {
    log.info("%lf, %lf, %lf, %lf, %lf, %lf",
             joint.j1, joint.j2, joint.j3, joint.j4, joint.j5, joint.j6);
}

void ArmGroup::printJointValues(const char *str, const JointValues &joint) {
    log.info("%s%lf, %lf, %lf, %lf, %lf, %lf", str,
             joint.j1, joint.j2, joint.j3, joint.j4, joint.j5, joint.j6);
}

void ArmGroup::printJointLimit(const JointLimit &joint_limit) {
    log.info("lower=%lf, home=%lf, upper=%lf, max_omega=%lf, max_alpha=%lf",
             joint_limit.lower, joint_limit.home, joint_limit.upper,
             joint_limit.max_omega, joint_limit.max_alpha);
}

void ArmGroup::printJointLimit(const char *str, const JointLimit &joint_limit) {
    log.info("%slower=%lf, home=%lf, upper=%lf, max_omega=%lf, max_alpha=%lf", str,
             joint_limit.lower, joint_limit.home, joint_limit.upper,
             joint_limit.max_omega, joint_limit.max_alpha);
}

void ArmGroup::printJointConstraints(const JointConstraints &constraint) {
    log.info("Joint constraints:");
    printJointLimit("  J1 limit: ", constraint.j1);
    printJointLimit("  J2 limit: ", constraint.j2);
    printJointLimit("  J3 limit: ", constraint.j3);
    printJointLimit("  J4 limit: ", constraint.j4);
    printJointLimit("  J5 limit: ", constraint.j5);
    printJointLimit("  J6 limit: ", constraint.j6);
}

void ArmGroup::printPose(const Pose &pose) {
    log.info("position=%lf,%lf,%lf orientation=%lf,%lf,%lf,%lf",
            pose.position.x, pose.position.y, pose.position.z,
            pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z);
}

void ArmGroup::printPose(const char *str, const Pose &pose) {
    log.info("%sposition=%lf,%lf,%lf orientation=%lf,%lf,%lf,%lf", str,
             pose.position.x, pose.position.y, pose.position.z,
             pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z);
}

void ArmGroup::printPoseEuler(const PoseEuler &pose_euler) {
    log.info("position=%lf,%lf,%lf orientation=%lf,%lf,%lf",
            pose_euler.position.x, pose_euler.position.y, pose_euler.position.z,
            pose_euler.orientation.a, pose_euler.orientation.b, pose_euler.orientation.c);
}

void ArmGroup::printPoseEuler(const char *str, const PoseEuler &pose_euler) {
    log.info("%sposition=%lf,%lf,%lf orientation=%lf,%lf,%lf", str,
            pose_euler.position.x, pose_euler.position.y, pose_euler.position.z,
            pose_euler.orientation.a, pose_euler.orientation.b, pose_euler.orientation.c);
}

void ArmGroup::printJointPoint(const JointPoint &point) {
    log.info("Joint point: ID=%d", point.id);
    log.info("  joints=%lf, %lf, %lf, %lf, %lf, %lf",
             point.joints.j1, point.joints.j2, point.joints.j3, 
             point.joints.j4, point.joints.j5, point.joints.j6);
    log.info("  omegas=%lf, %lf, %lf, %lf, %lf, %lf",
             point.omegas.j1, point.omegas.j2, point.omegas.j3,
             point.omegas.j4, point.omegas.j5, point.omegas.j6);
}

void ArmGroup::printJointPoint(const char *str, const JointPoint &point) {
    log.info(str);
    printJointPoint(point);
}

void ArmGroup::printPathPoint(const PathPoint &point) {
    switch (point.type) {
        case MOTION_JOINT:
            log.info("Path point: ID=%d, type=Joint", point.id);
            printJointValues("  joints=", point.joints);
            break;

        case MOTION_LINE:
            log.info("Path point: ID=%d, type=Line", point.id);
            printPose("  pose: ", point.pose);
            break;

        case MOTION_CIRCLE:
            log.info("Path point: ID=%d, type=Circle", point.id);
            printPose("  pose: ", point.pose);
            break;

        default:
            log.error("Type error: undefined point type.");
    }
}

void ArmGroup::printPathPoint(const char *str, const PathPoint &point) {
    log.info(str);
    printPathPoint(point);
}


void ArmGroup::lockArmGroup(void) {
    //log.info("Try to lock arm group.");
    pthread_mutex_lock(&group_mutex_);
}

void ArmGroup::unlockArmGroup(void) {
    pthread_mutex_unlock(&group_mutex_);
    //log.info("Arm group unlocked.");
}

}
