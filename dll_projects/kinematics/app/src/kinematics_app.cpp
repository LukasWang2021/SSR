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


uint64_t c_km_getMatrixInv(const double* p_matrix, int dim, double* p_inv)
{
	inverse(p_matrix, dim, p_inv);
	return 0;
}

uint64_t c_km_turnQuat2Euler(const double (&quaternion_)[4], double (&res)[3])
{
	Quaternion quaternion;
	quaternion.w_ = quaternion_[0];
	quaternion.x_ = quaternion_[1];
	quaternion.y_ = quaternion_[2];
	quaternion.z_ = quaternion_[3];

	Euler euler;
	Quaternion q(quaternion);

	// if (fabs(sqrt(q.w_ * q.w_ + q.x_ * q.x_ + q.y_ * q.y_ + q.z_ * q.z_) - 1) > 0.0005)
	// {
	normalizeQuaternion(q);
	// }

	euler.c_ = atan2((q.w_ * q.x_ + q.y_ * q.z_) * 2, 1 - (q.x_ * q.x_ + q.y_ * q.y_) * 2);
	euler.b_ = asin((q.w_ * q.y_ - q.z_ * q.x_) * 2);
	euler.a_ = atan2((q.w_ * q.z_ + q.x_ * q.y_) * 2, 1 - (q.y_ * q.y_ + q.z_ * q.z_) * 2);
	res[0] = euler.a_;
	res[1] = euler.b_;
	res[2] = euler.c_;
	return 0;
}

uint64_t c_km_turnEuler2Quat(const double (&euler_)[3], double (&res)[4])
{
	Euler euler;
	euler.a_ = euler_[0];
	euler.b_ = euler_[1];
	euler.c_ = euler_[2];

	Quaternion quat;

	quat = Euler2Quaternion(euler);

	res[0] = quat.w_;
	res[1] = quat.x_;
	res[2] = quat.y_;
	res[3] = quat.z_;

	return 0;
}

uint64_t c_km_turnPoseEuler2Matrix(const double (&pose_)[6], double(&m)[4][4])
{
	PoseEuler pose;
	pose.point_.x_ = pose_[0];
	pose.point_.y_ = pose_[1];
	pose.point_.z_ = pose_[2];
	pose.euler_.a_ = pose_[3];
	pose.euler_.b_ = pose_[4];
	pose.euler_.c_ = pose_[5];

	PoseEuler2Matrix(pose, m);

	return 0;

}

uint64_t c_km_turnMatrix2PoseEuler(const double(&m)[4][4], double (&pose_)[6])
{
	PoseEuler res;
	Matrix2PoseEuler(m, res);
	pose_[0] = res.point_.x_;
	pose_[1] = res.point_.y_;
	pose_[2] = res.point_.z_;

	pose_[3] = res.euler_.a_;
	pose_[4] = res.euler_.b_;
	pose_[5] = res.euler_.c_;

	return 0;
}

uint64_t c_km_mulMatrix2Matrix(const double(&m)[4][4], const double(&n)[4][4], double(&res)[4][4])
{
	mulMatrix2Matrix(m,n,res);
	return 0;
}


