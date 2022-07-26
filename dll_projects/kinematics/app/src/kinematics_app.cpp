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

