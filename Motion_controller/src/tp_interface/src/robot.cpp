#include "robot.h"
#include "error_code.h"
#include "error_monitor.h"

Robot::Robot(ArmGroup *arm_group):arm_group_(arm_group)
{
}

Robot::~Robot()
{
}

PoseEuler* Robot::getTCPPosePtr()
{
    return &tcp_pose_;
}
PoseEuler* Robot::getFlangePosePtr(const Joint &joints)
{
    PoseEuler flange_pose_;
    PoseEuler tcp_pose;
    arm_group_->getPoseFromJointInWorld(joints, flange_pose_, tcp_pose);

    return &flange_pose_;
}
void Robot::updatePose(const Joint &joints)
{
    getPoseFromJoint(joints, tcp_pose_);
}


bool Robot::getPoseFromJoint(const Joint &joints, PoseEuler &pose)
{
    Pose p;
    U64 result = arm_group_->getPoseFromJoint(joints, pose);     
    if (result != TPI_SUCCESS)
    {
        rcs::Error::instance()->add(result);
        return false;
    }
    return true;
}

bool Robot::getJointFromPose(const PoseEuler &pose, Joint &joints, double time_val)
{
    U64 result = arm_group_->getJointFromPose(pose, joints);
    //printDbLine("new is:", (double*)&joints, 6);
    if (result != TPI_SUCCESS)
    {
            FST_ERROR("computeIK failed");
            rcs::Error::instance()->add(result);
            return false;
    }

    return true;
}

motion_spec_userFrame Robot::getUserFrame()
{
    Transformation transform = arm_group_->getUserFrame();
    motion_spec_userFrame user_frame;
    user_frame.X = transform.position.x;
    user_frame.Y = transform.position.y;
    user_frame.Z = transform.position.z;
    
    user_frame.A = transform.orientation.a;
    user_frame.B = transform.orientation.b;
    user_frame.C = transform.orientation.c;

    return user_frame;
}

bool Robot::setUserFrame(motion_spec_userFrame *user_frame)
{
    Transformation transform;
    transform.position.x = user_frame->X;
    transform.position.y = user_frame->Y;
    transform.position.z = user_frame->Z;
    
    transform.orientation.a = user_frame->A;
    transform.orientation.b = user_frame->B;
    transform.orientation.c = user_frame->C;

    arm_group_->setUserFrame(transform);

    return true;
}

motion_spec_toolFrame Robot::getToolFrame()
{
    Transformation transform = arm_group_->getToolFrame();
    motion_spec_toolFrame too_frame;
    too_frame.X = transform.position.x;
    too_frame.Y = transform.position.y;
    too_frame.Z = transform.position.z;
    
    too_frame.A = transform.orientation.a;
    too_frame.B = transform.orientation.b;
    too_frame.C = transform.orientation.c;

    return too_frame;
}

bool Robot::setToolFrame(motion_spec_toolFrame *tool_frame)
{
    Transformation transform;
    transform.position.x = tool_frame->X;
    transform.position.y = tool_frame->Y;
    transform.position.z = tool_frame->Z;
    
    transform.orientation.a = tool_frame->A;
    transform.orientation.b = tool_frame->B;
    transform.orientation.c = tool_frame->C;

    arm_group_->setToolFrame(transform);    

    return true;
}


motion_spec_JointConstraint Robot::getSoftConstraint()
{
    JointConstraint jc = arm_group_->getSoftConstraint();
    //FST_INFO("J1=>maxv:%f, maxa:%f", jc.j1.max_omega, jc.j1.max_alpha);
    motion_spec_JointConstraint jnt_constraint;
    jnt_constraint.jntLmt_count = MAX_JOINTS;

    jnt_constraint.jntLmt[0].has_zero = true;
    jnt_constraint.jntLmt[0].zero = jc.j1.home;
    jnt_constraint.jntLmt[0].has_upper = true;
    jnt_constraint.jntLmt[0].upper = jc.j1.upper;
    jnt_constraint.jntLmt[0].has_lower = true;
    jnt_constraint.jntLmt[0].lower = jc.j1.lower;
    jnt_constraint.jntLmt[0].has_max_omega = true;
    jnt_constraint.jntLmt[0].max_omega = jc.j1.max_omega;
    jnt_constraint.jntLmt[0].has_max_alpha = true;
    jnt_constraint.jntLmt[0].max_alpha = jc.j1.max_alpha;

    jnt_constraint.jntLmt[1].has_zero = true;
    jnt_constraint.jntLmt[1].zero = jc.j2.home;
    jnt_constraint.jntLmt[1].has_upper = true;
    jnt_constraint.jntLmt[1].upper = jc.j2.upper;
    jnt_constraint.jntLmt[1].has_lower = true;
    jnt_constraint.jntLmt[1].lower = jc.j2.lower;
    jnt_constraint.jntLmt[1].has_max_omega = true;
    jnt_constraint.jntLmt[1].max_omega = jc.j2.max_omega;
    jnt_constraint.jntLmt[1].has_max_alpha = true;
    jnt_constraint.jntLmt[1].max_alpha = jc.j2.max_alpha;

    jnt_constraint.jntLmt[2].has_zero = true;
    jnt_constraint.jntLmt[2].zero = jc.j3.home;
    jnt_constraint.jntLmt[2].has_upper = true;
    jnt_constraint.jntLmt[2].upper = jc.j3.upper;
    jnt_constraint.jntLmt[2].has_lower = true;
    jnt_constraint.jntLmt[2].lower = jc.j3.lower;
    jnt_constraint.jntLmt[2].has_max_omega = true;
    jnt_constraint.jntLmt[2].max_omega = jc.j3.max_omega;
    jnt_constraint.jntLmt[2].has_max_alpha = true;
    jnt_constraint.jntLmt[2].max_alpha = jc.j3.max_alpha;

    jnt_constraint.jntLmt[3].has_zero = true;
    jnt_constraint.jntLmt[3].zero = jc.j4.home;
    jnt_constraint.jntLmt[3].has_upper = true;
    jnt_constraint.jntLmt[3].upper = jc.j4.upper;
    jnt_constraint.jntLmt[3].has_lower = true;
    jnt_constraint.jntLmt[3].lower = jc.j4.lower;
    jnt_constraint.jntLmt[3].has_max_omega = true;
    jnt_constraint.jntLmt[3].max_omega = jc.j4.max_omega;
    jnt_constraint.jntLmt[3].has_max_alpha = true;
    jnt_constraint.jntLmt[3].max_alpha = jc.j4.max_alpha;

    jnt_constraint.jntLmt[4].has_zero = true;
    jnt_constraint.jntLmt[4].zero = jc.j5.home;
    jnt_constraint.jntLmt[4].has_upper = true;
    jnt_constraint.jntLmt[4].upper = jc.j5.upper;
    jnt_constraint.jntLmt[4].has_lower = true;
    jnt_constraint.jntLmt[4].lower = jc.j5.lower;
    jnt_constraint.jntLmt[4].has_max_omega = true;
    jnt_constraint.jntLmt[4].max_omega = jc.j5.max_omega;
    jnt_constraint.jntLmt[4].has_max_alpha = true;
    jnt_constraint.jntLmt[4].max_alpha = jc.j5.max_alpha;

    jnt_constraint.jntLmt[5].has_zero = true;
    jnt_constraint.jntLmt[5].zero = jc.j6.home;
    jnt_constraint.jntLmt[5].has_upper = true;
    jnt_constraint.jntLmt[5].upper = jc.j6.upper;
    jnt_constraint.jntLmt[5].has_lower = true;
    jnt_constraint.jntLmt[5].lower = jc.j6.lower;
    jnt_constraint.jntLmt[5].has_max_omega = true;
    jnt_constraint.jntLmt[5].max_omega = jc.j6.max_omega;
    jnt_constraint.jntLmt[5].has_max_alpha = true;
    jnt_constraint.jntLmt[5].max_alpha = jc.j6.max_alpha;

    return jnt_constraint;
}
bool Robot::setSoftConstraint(motion_spec_JointConstraint *jnt_constraint)
{
    JointConstraint jc = arm_group_->getSoftConstraint();
    if (jnt_constraint->jntLmt_count < 6)
    {
        rcs::Error::instance()->add(INVALID_PARAM_FROM_TP);
        return false;
    }

    if (jnt_constraint->jntLmt[0].has_zero)
        jc.j1.home = jnt_constraint->jntLmt[0].zero;
    if (jnt_constraint->jntLmt[0].has_upper)
        jc.j1.upper = jnt_constraint->jntLmt[0].upper;
    if (jnt_constraint->jntLmt[0].has_lower)
        jc.j1.lower = jnt_constraint->jntLmt[0].lower;
    if (jnt_constraint->jntLmt[0].has_max_omega)
        jc.j1.max_omega = jnt_constraint->jntLmt[0].max_omega;
    if (jnt_constraint->jntLmt[0].has_max_alpha)
        jc.j1.max_alpha = jnt_constraint->jntLmt[0].max_alpha;

    if (jnt_constraint->jntLmt[1].has_zero)
        jc.j2.home = jnt_constraint->jntLmt[1].zero;
    if (jnt_constraint->jntLmt[1].has_upper)
        jc.j2.upper = jnt_constraint->jntLmt[1].upper;
    if (jnt_constraint->jntLmt[1].has_lower)
        jc.j2.lower = jnt_constraint->jntLmt[1].lower;
    if (jnt_constraint->jntLmt[1].has_max_omega)
        jc.j2.max_omega = jnt_constraint->jntLmt[1].max_omega;
    if (jnt_constraint->jntLmt[1].has_max_alpha)
        jc.j2.max_alpha = jnt_constraint->jntLmt[1].max_alpha;

    if (jnt_constraint->jntLmt[2].has_zero)
        jc.j3.home = jnt_constraint->jntLmt[2].zero;
    if (jnt_constraint->jntLmt[2].has_upper)
        jc.j3.upper = jnt_constraint->jntLmt[2].upper;
    if (jnt_constraint->jntLmt[2].has_lower)
        jc.j3.lower = jnt_constraint->jntLmt[2].lower;
    if (jnt_constraint->jntLmt[2].has_max_omega)
        jc.j3.max_omega = jnt_constraint->jntLmt[2].max_omega;
    if (jnt_constraint->jntLmt[2].has_max_alpha)
        jc.j3.max_alpha = jnt_constraint->jntLmt[2].max_alpha;

    if (jnt_constraint->jntLmt[3].has_zero)
        jc.j4.home = jnt_constraint->jntLmt[3].zero;
    if (jnt_constraint->jntLmt[3].has_upper)
        jc.j4.upper = jnt_constraint->jntLmt[3].upper;
    if (jnt_constraint->jntLmt[3].has_lower)
        jc.j4.lower = jnt_constraint->jntLmt[3].lower;
    if (jnt_constraint->jntLmt[3].has_max_omega)
        jc.j4.max_omega = jnt_constraint->jntLmt[3].max_omega;
    if (jnt_constraint->jntLmt[3].has_max_alpha)
        jc.j4.max_alpha = jnt_constraint->jntLmt[3].max_alpha;

    if (jnt_constraint->jntLmt[4].has_zero)
        jc.j5.home = jnt_constraint->jntLmt[4].zero;
    if (jnt_constraint->jntLmt[4].has_upper)
        jc.j5.upper = jnt_constraint->jntLmt[4].upper;
    if (jnt_constraint->jntLmt[4].has_lower)
        jc.j5.lower = jnt_constraint->jntLmt[4].lower;
    if (jnt_constraint->jntLmt[4].has_max_omega)
        jc.j5.max_omega = jnt_constraint->jntLmt[4].max_omega;
    if (jnt_constraint->jntLmt[4].has_max_alpha)
        jc.j5.max_alpha = jnt_constraint->jntLmt[4].max_alpha;

    if (jnt_constraint->jntLmt[5].has_zero)
        jc.j6.home = jnt_constraint->jntLmt[5].zero;
    if (jnt_constraint->jntLmt[5].has_upper)
        jc.j6.upper = jnt_constraint->jntLmt[5].upper;
    if (jnt_constraint->jntLmt[5].has_lower)
        jc.j6.lower = jnt_constraint->jntLmt[5].lower;
    if (jnt_constraint->jntLmt[5].has_max_omega)
        jc.j6.max_omega = jnt_constraint->jntLmt[5].max_omega;
    if (jnt_constraint->jntLmt[5].has_max_alpha)
        jc.j6.max_alpha = jnt_constraint->jntLmt[5].max_alpha;

    if (arm_group_->setSoftConstraint(jc) == false)
    {
        rcs::Error::instance()->add(INVALID_PARAM_FROM_TP);
        return false;
    }

    return true;
}

motion_spec_JointConstraint Robot::getHardConstraint()
{
    JointConstraint jc = arm_group_->getHardConstraint();
    motion_spec_JointConstraint jnt_constraint;
    jnt_constraint.jntLmt_count = MAX_JOINTS;

    jnt_constraint.jntLmt[0].has_upper = true;
    jnt_constraint.jntLmt[0].upper = jc.j1.upper;
    jnt_constraint.jntLmt[0].has_lower = true;
    jnt_constraint.jntLmt[0].lower = jc.j1.lower;

    jnt_constraint.jntLmt[1].has_upper = true;
    jnt_constraint.jntLmt[1].upper = jc.j2.upper;
    jnt_constraint.jntLmt[1].has_lower = true;
    jnt_constraint.jntLmt[1].lower = jc.j2.lower;

    jnt_constraint.jntLmt[2].has_upper = true;
    jnt_constraint.jntLmt[2].upper = jc.j3.upper;
    jnt_constraint.jntLmt[2].has_lower = true;
    jnt_constraint.jntLmt[2].lower = jc.j3.lower;

    jnt_constraint.jntLmt[3].has_upper = true;
    jnt_constraint.jntLmt[3].upper = jc.j4.upper;
    jnt_constraint.jntLmt[3].has_lower = true;
    jnt_constraint.jntLmt[3].lower = jc.j4.lower;

    jnt_constraint.jntLmt[4].has_upper = true;
    jnt_constraint.jntLmt[4].upper = jc.j5.upper;
    jnt_constraint.jntLmt[4].has_lower = true;
    jnt_constraint.jntLmt[4].lower = jc.j5.lower;

    jnt_constraint.jntLmt[5].has_upper = true;
    jnt_constraint.jntLmt[5].upper = jc.j6.upper;
    jnt_constraint.jntLmt[5].has_lower = true;
    jnt_constraint.jntLmt[5].lower = jc.j6.lower;

    return jnt_constraint;
}




motion_spec_DHGroup Robot::getDHGroup()
{
    DHGroup dh = arm_group_->getDH();
    motion_spec_DHGroup dh_group;
    dh_group.coord_offset_count = 6;
    dh_group.coord_offset[0].alpha = dh.j1.alpha;
    dh_group.coord_offset[0].a = dh.j1.a;
    dh_group.coord_offset[0].d = dh.j1.d;
    dh_group.coord_offset[0].theta = dh.j1.theta;

    dh_group.coord_offset[1].alpha = dh.j2.alpha;
    dh_group.coord_offset[1].a = dh.j2.a;
    dh_group.coord_offset[1].d = dh.j2.d;
    dh_group.coord_offset[1].theta = dh.j2.theta;

    dh_group.coord_offset[2].alpha = dh.j3.alpha;
    dh_group.coord_offset[2].a = dh.j3.a;
    dh_group.coord_offset[2].d = dh.j3.d;
    dh_group.coord_offset[2].theta = dh.j3.theta;

    dh_group.coord_offset[3].alpha = dh.j4.alpha;
    dh_group.coord_offset[3].a = dh.j4.a;
    dh_group.coord_offset[3].d = dh.j4.d;
    dh_group.coord_offset[3].theta = dh.j4.theta;
    
    dh_group.coord_offset[4].alpha = dh.j5.alpha;
    dh_group.coord_offset[4].a = dh.j5.a;
    dh_group.coord_offset[4].d = dh.j5.d;
    dh_group.coord_offset[4].theta = dh.j5.theta;

    dh_group.coord_offset[5].alpha = dh.j6.alpha;
    dh_group.coord_offset[5].a = dh.j6.a;
    dh_group.coord_offset[5].d = dh.j6.d;
    dh_group.coord_offset[5].theta = dh.j6.theta;

    return dh_group;
}




