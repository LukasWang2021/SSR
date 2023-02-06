#ifndef KINEMATICS_APP_H
#define KINEMATICS_APP_H

#include "comm_def.h"

#ifdef __cplusplus
extern "C"
{
#endif

	/**
	 * @brief Initialize the kinematics module.
	 * @details
	 * @param [in] dh The arm DH parameters.
	 * @retval 0 Success.
	 * @retval others Failure.
	 */
	COMM_INTERFACE_API uint64_t c_km_initKinematics(double dh[7][4]);

	/**
	 * @brief Finalize the kinematics module.
	 * @details
	 * @param [in] None.
	 * @retval 0 Success.
	 * @retval others Failure.
	 */
	COMM_INTERFACE_API uint64_t c_km_exitKinematics(void);

	/**
	 * @brief Set the kinematics module's DH parameters.
	 * @details
	 * @param [in] dh The arm DH values..
	 * @retval 0 Success.
	 * @retval others Failure.
	 */
	COMM_INTERFACE_API uint64_t c_km_setDH(double dh[7][4]);

	/**
	 * @brief Set the tool frame parameters.
	 * @details
	 * @param [in] pose The tool frame value with x,y,z,a,b,c.
	 * @retval 0 Success.
	 * @retval others Failure.
	 */
	COMM_INTERFACE_API uint64_t c_km_setToolFrame(double pose[6]);

	/**
	 * @brief Set the user frame parameters.
	 * @details
	 * @param [in] The user frame value with x,y,z,a,b,c.
	 * @retval 0 Success.
	 * @retval others Failure.
	 */
	COMM_INTERFACE_API uint64_t c_km_setUserFrame(double pose[6]);

	/**
	 * @brief Get the link posture with given joint position.
	 * @details
	 * @param [in] joint_pos The current position of the joints.rad.
	 * @param [in] from The index of the link. To be zero if from the base.
	 * @param [in] to The index of the link.
	 * @param [out] cart_pos The posture {x, y, z, a, b, c} of link_0 under Base Coordinate System. The unit is mm.
	 * @retval 0 Success.
	 * @retval others Failure.
	 */
	COMM_INTERFACE_API uint64_t c_km_getCartLink(double joint_pos[6], uint32_t from, uint32_t to, double cart_pos[6]);

	/**
	 * @brief Get the link posture with given joint position.
	 * @details
	 * @param [in] joint_pos The current position of the joints.rad.
	 * @param [out] pose_tcp The posture {x, y, z, a, b, c} of current tool center position. The unit is mm.
	 * @retval 0 Success.
	 * @retval others Failure.
	 */
	COMM_INTERFACE_API uint64_t c_km_getTcpByBase(double joint_pos[6], double pose_tcp[6]);

	/**
	 * @brief Get the arm posture with given joint position.
	 * @details
	 * @param [in] joint_pos The current position of the joints.rad.
	 * @param [out] posture The posture {arm, elbow, wrist, flip}.
	 * @retval 0 Success.
	 * @retval others Failure.
	 */
	COMM_INTERFACE_API uint64_t c_km_getPostureByJoint(double joint_pos[6], int32_t posture[4]);

	/**
	 * @brief Get the joint pose with given arm position.
	 * @details
	 * @param [in] joint_pos The current position of the joints.rad.
	 * @param [out] posture The posture {arm, elbow, wrist, flip}.
	 * @retval 0 Success.
	 * @retval others Failure.
	 */
	COMM_INTERFACE_API uint64_t c_km_getJointByPoseEuler(const double pose_euler[6], const int32_t posture[4], double joint[6]);
	COMM_INTERFACE_API uint64_t c_km_getJointByPoseQuat(const double pose_quat[7], const int32_t posture[4], double joint[6]);
	COMM_INTERFACE_API uint64_t c_km_getJointByTransMatrix(const double trans_matrix[16], const int32_t posture[4], double joint[6]);

	//COMM_INTERFACE_API uint64_t c_km_convertCartToJoint(double joint_pos[6], double joint_pos[6]);
	//COMM_INTERFACE_API uint64_t c_km_convertJointToCart(double joint_pos[6], double joint_pos[6]);

#ifdef __cplusplus
}
#endif


#endif

