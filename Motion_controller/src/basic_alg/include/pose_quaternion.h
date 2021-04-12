#ifndef POSE_QUATERNION_H
#define POSE_QUATERNION_H

#include "point.h"
#include "quaternion.h"
#include "pose_euler.h"
#include "trans_matrix.h"
#include <string>

namespace basic_alg
{
class PoseEuler;
class TransMatrix;

class PoseQuaternion
{
/**
 * @brief PoseQuaternion is the object to represent a pose in 3D space. It includes two member variables {point_, quaternion_}.
 * @details A pose consists of a transition part and an orientation part.\n
            The transition part is described by member variable point_.\n
            The orientation part is described by member variable quaternion_.\n
 */
public:
    Point point_;
    Quaternion quaternion_;
    /**
     * @brief Compare if two poses are equal.
     * @details Compare both trans_vector_ and quaternion_ with given valve value.\n
     * @param [in] pose_quaternion Another pose object, used as the comparing object.
     * @param [in] valve The maximum permissible deviations for comparing trans_vector_ and quaternion_.
     * @retval true Self pose is equal to the comparing pose.
     * @retval false Self pose is not equal to the comparing pose.
     */
    bool isEqual(const PoseQuaternion& pose_quaternion, double valve = 0.001) const;
    bool isEquivalent(const PoseQuaternion& pose_quaternion, double valve = 0.001) const;
    /**
     * @brief Convert the PoseQuaternion object to PoseEuler object.
     * @details Keep transition part and do convertion for orientation part.
     * @param [out] pose_euler The PoseEuler object.
     * @return void
     * @see basic_alg::PoseEuler
     */
    void convertToPoseEuler(basic_alg::PoseEuler& pose_euler) const;
    /**
     * @brief Convert the PoseQuaternion object to TransMatrix object.
     * @details Combine transition part and orientation part together to form a 4 by 4 transformation matrix.\n
     * @param [out] matrix The TransMatrix object.
     * @return void
     * @see basic_alg::TransMatrix
     */
    void convertToTransMatrix(basic_alg::TransMatrix& matrix) const;  

    void print(std::string comment = "") const;
};

}


#endif

