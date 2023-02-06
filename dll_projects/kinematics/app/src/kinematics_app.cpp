#include "kinematics_app.h"
#include "kinematics_rtm.h"
#include "transformation.h"
#include "basic_alg.h"
#include "trans_matrix.h"
#include "kinematics_err.h"
#include <string.h>

using namespace basic_alg;

static KinematicsRTM* p_kinematics = NULL;
static Transformation* p_transform = NULL;
static PoseEuler tool_frame;
static PoseEuler user_frame;

static bool initialized = false;

uint64_t c_km_initKinematics(double dh[7][4])
{
	if (initialized) return 0;

	DH base_dh, arm_dh[6];

	base_dh.d      = dh[0][0];
	base_dh.a      = dh[0][1];
	base_dh.alpha  = dh[0][2];
	base_dh.offset = dh[0][3];

	for (size_t i = 0; i < 6; ++i)
	{
		arm_dh[i].d      = dh[i+1][0];
		arm_dh[i].a      = dh[i+1][1];
		arm_dh[i].alpha  = dh[i+1][2];
		arm_dh[i].offset = dh[i+1][3];
	}

	p_kinematics = new KinematicsRTM(base_dh, arm_dh, 0);
	p_transform = new Transformation();
	if (p_kinematics == NULL || p_transform == NULL)
	{
		return KINEMATICS_MEM_ERR;
	}
	p_transform->init(p_kinematics);

	tool_frame.point_.zero();
	tool_frame.euler_.zero();
	user_frame.point_.zero();
	user_frame.euler_.zero();

	initialized = true;

	return 0;
}

uint64_t c_km_exitKinematics(void)
{
	if (p_kinematics != NULL) delete p_kinematics;
	if (p_transform != NULL) delete p_transform;

	initialized = false;

	return 0;
}

uint64_t c_km_setDH(double  dh[7][4])
{
	if (!initialized) return KINEMATICS_NOT_INIT_ERR;

	DH base_dh, arm_dh[6];
	memcpy(&base_dh, dh, sizeof(DH) * 7);

	p_kinematics->setDH(base_dh, arm_dh);
	return 0;
}

uint64_t c_km_setToolFrame(double pose[6])
{
	tool_frame.point_.x_ = pose[0];
	tool_frame.point_.y_ = pose[1];
	tool_frame.point_.z_ = pose[2];
	tool_frame.euler_.c_ = pose[3];
	tool_frame.euler_.b_ = pose[4];
	tool_frame.euler_.a_ = pose[5];
	return 0;
}

uint64_t c_km_setUserFrame(double pose[6])
{
    user_frame.point_.x_ = pose[0];
	user_frame.point_.y_ = pose[1];
	user_frame.point_.z_ = pose[2];
	user_frame.euler_.c_ = pose[3];
	user_frame.euler_.b_ = pose[4];
	user_frame.euler_.a_ = pose[5];
	return 0;
}

uint64_t c_km_getCartLink(double joint_pos[6], uint32_t from, uint32_t to, double cart_pos[6])
{
	if (!initialized) return KINEMATICS_NOT_INIT_ERR;

	Joint joint;
	PoseEuler pose_euler;

	for (size_t i = 0; i < 6; ++i)
	{
		joint[i] = joint_pos[i];
	}

	p_kinematics->doFK(joint, pose_euler, from, to);

	cart_pos[0] = pose_euler.point_.x_;
	cart_pos[1] = pose_euler.point_.y_;
	cart_pos[2] = pose_euler.point_.z_;
	cart_pos[3] = pose_euler.euler_.c_;
	cart_pos[4] = pose_euler.euler_.b_;
	cart_pos[5] = pose_euler.euler_.a_;

	return 0;
}

uint64_t c_km_getTcpByBase(double joint_pos[6], double pose_tcp[6])
{
	if (!initialized) return KINEMATICS_NOT_INIT_ERR;

	Joint joint;
	PoseEuler pose_euler;

	for (size_t i = 0; i < 6; ++i)
	{
		joint[i] = joint_pos[i];
	}

	p_transform->getTcpByBase(joint, tool_frame, pose_euler);

	pose_tcp[0] = pose_euler.point_.x_;
	pose_tcp[1] = pose_euler.point_.y_;
	pose_tcp[2] = pose_euler.point_.z_;
	pose_tcp[3] = pose_euler.euler_.c_;
	pose_tcp[4] = pose_euler.euler_.b_;
	pose_tcp[5] = pose_euler.euler_.a_;

	return 0;
}

uint64_t c_km_getPostureByJoint(double joint_pos[6], int32_t posture[4])
{
	if (!initialized) return KINEMATICS_NOT_INIT_ERR;

	Posture p;
	Joint j; j.zero();
	for (int i = 0; i < 6; ++i)
	{
		j[i] = joint_pos[i];
	}
	p = p_kinematics->getPostureByJoint(j);

	posture[0] = p.arm;
	posture[1] = p.elbow;
	posture[2] = p.wrist;
	posture[3] = p.flip;

	return 0;
}


uint64_t c_km_getJointByPoseEuler(const double pose_euler[6], const int32_t posture[4], double joint[6])
{
	Posture pt;
	pt.arm = posture[0];
	pt.elbow = posture[1];
	pt.wrist = posture[2];
	pt.flip = posture[3];

	Joint jt; jt.zero();
	PoseEuler pe;
	pe.point_.x_ = pose_euler[0];
	pe.point_.y_ = pose_euler[1];
	pe.point_.z_ = pose_euler[2];
	pe.euler_.a_ = pose_euler[5];
	pe.euler_.b_ = pose_euler[4];
	pe.euler_.c_ = pose_euler[3];

	if (!p_kinematics->doIK(pe, pt, jt))
	{
		return 1;
	}
	for (int i = 0; i < 6; ++i)
	{
		joint[i] = jt[i];
	}
	return 0;
}

uint64_t c_km_getJointByPoseQuat(const double pose_quat[7], const int32_t posture[4], double joint[6])
{
	Posture pt;
	pt.arm = posture[0];
	pt.elbow = posture[1];
	pt.wrist = posture[2];
	pt.flip = posture[3];

	Joint jt; jt.zero();
	PoseQuaternion pq;
	pq.point_.x_ = pose_quat[0];
	pq.point_.y_ = pose_quat[1];
	pq.point_.z_ = pose_quat[2];
	pq.quaternion_.w_ = pose_quat[3];
	pq.quaternion_.x_ = pose_quat[4];
	pq.quaternion_.y_ = pose_quat[5];
	pq.quaternion_.z_ = pose_quat[5];

	if (!p_kinematics->doIK(pq, pt, jt))
	{
		return 1;
	}
	for (int i = 0; i < 6; ++i)
	{
		joint[i] = jt[i];
	}
	return 0;
}

uint64_t c_km_getJointByTransMatrix(const double trans_matrix[16], const int32_t posture[4], double joint[6])
{
	Posture pt;
	pt.arm = posture[0];
	pt.elbow = posture[1];
	pt.wrist = posture[2];
	pt.flip = posture[3];

	Joint jt; jt.zero();
	TransMatrix tm;
	tm.trans_vector_.x_ = trans_matrix[3];
	tm.trans_vector_.y_ = trans_matrix[7];
	tm.trans_vector_.z_ = trans_matrix[11];

	tm.rotation_matrix_.matrix_[0][0] = trans_matrix[0];
	tm.rotation_matrix_.matrix_[0][1] = trans_matrix[1];
	tm.rotation_matrix_.matrix_[0][2] = trans_matrix[2];
	tm.rotation_matrix_.matrix_[1][0] = trans_matrix[4];
	tm.rotation_matrix_.matrix_[1][1] = trans_matrix[5];
	tm.rotation_matrix_.matrix_[1][2] = trans_matrix[6];
	tm.rotation_matrix_.matrix_[2][0] = trans_matrix[8];
	tm.rotation_matrix_.matrix_[2][1] = trans_matrix[9];
	tm.rotation_matrix_.matrix_[2][2] = trans_matrix[10];

	if (!p_kinematics->doIK(tm, pt, jt))
	{
		return 1;
	}
	for (int i = 0; i < 6; ++i)
	{
		joint[i] = jt[i];
	}
	return 0;
}
