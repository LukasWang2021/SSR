/*************************************************************************
	> File Name: path_plan.cpp
	> Author: 
	> Mail: 
	> Created Time: 2018年09月05日 星期三 17时55分57秒
 ************************************************************************/

#include <math.h>
#include <path_plan.h>
#include <iostream>
#include <basic_alg.h>

using namespace basic_alg;

namespace fst_mc
{

struct LinePathCoeff
{
    double position_coeff_x;
    double position_coeff_y;
    double position_coeff_z;
    double orientation_angle;
};

ErrorCode planJointPath(const Joint &start, const Joint &target, double &precision, size_t &index, Joint (&path)[MAX_PATH_SIZE], size_t &valid_length)
{
    size_t trip_index = 0;
    MotionTime trip_max = -1;

    for (size_t i = 0; i < NUM_OF_JOINT; i++)
    {
        if (fabs(target[i] - start[i]) > trip_max)
        {
            trip_max = fabs(target[i] - start[i]);
            trip_index = i;
        }
    }

    size_t max_stamp;
    max_stamp = ceil(fabs(target[trip_index] - start[trip_index]) / precision);
    max_stamp = max_stamp < MAX_PATH_SIZE ? max_stamp : MAX_PATH_SIZE - 1;
    precision = fabs(target[trip_index] - start[trip_index]) / max_stamp;
    index = trip_index;
    valid_length = max_stamp + 1;

    double coeff[NUM_OF_JOINT] = {
            (target[0] - start[0]) / max_stamp,
            (target[1] - start[1]) / max_stamp,
            (target[2] - start[2]) / max_stamp,
            (target[3] - start[3]) / max_stamp,
            (target[4] - start[4]) / max_stamp,
            (target[5] - start[5]) / max_stamp,
            (target[6] - start[6]) / max_stamp,
            (target[7] - start[7]) / max_stamp,
            (target[8] - start[8]) / max_stamp,
    };

    //printf("max-stamp=%d, precision=%.6f\n", max_stamp, precision);

    for (size_t i = 0; i <= max_stamp; i++)
    {
        path[i].j1 = start[0] + coeff[0] * i;
        path[i].j2 = start[1] + coeff[1] * i;
        path[i].j3 = start[2] + coeff[2] * i;
        path[i].j4 = start[3] + coeff[3] * i;
        path[i].j5 = start[4] + coeff[4] * i;
        path[i].j6 = start[5] + coeff[5] * i;
        path[i].j7 = start[6] + coeff[6] * i;
        path[i].j8 = start[7] + coeff[7] * i;
        path[i].j9 = start[8] + coeff[8] * i;

        //printf("path[%d]: %.4f, %.4f, %.4f, %.4f, %.4f, %.4f\n", i,
        //       path[i].j1, path[i].j2, path[i].j3, path[i].j4, path[i].j5, path[i].j6);
    }

    return SUCCESS;
}


ErrorCode planLinePath(const PoseEuler &start, const PoseEuler &target, double &precision, Pose (&path)[MAX_PATH_SIZE], size_t &valid_length)
{
    Pose pose0 = PoseEuler2Pose(start);
    Pose pose1 = PoseEuler2Pose(target);

    if (innerProductQuatern(pose0.orientation, pose1.orientation) < 0)
    {
        pose0.orientation.w = -pose0.orientation.w;
        pose0.orientation.x = -pose0.orientation.x;
        pose0.orientation.y = -pose0.orientation.y;
        pose0.orientation.z = -pose0.orientation.z;
    }

    // Get position distance and orientateion rotate angle
    // Resolve the max stamp of this path
    double distance = getDistance(pose0.position, pose1.position);
    double rotation = getOrientationAngle(pose0, pose1);
    double stamp_position    = distance / precision;
    double stamp_orientation = rotation / 0.01;
    size_t max_stamp = stamp_position > stamp_orientation ? ceil(stamp_position) : ceil(stamp_orientation);
    valid_length = max_stamp + 1;
    precision = distance / max_stamp;

    LinePathCoeff coeff;

    coeff.position_coeff_x  = (pose1.position.x - pose0.position.x) / max_stamp;
    coeff.position_coeff_y  = (pose1.position.y - pose0.position.y) / max_stamp;
    coeff.position_coeff_z  = (pose1.position.z - pose0.position.z) / max_stamp;
    coeff.orientation_angle = rotation;

    for (size_t i = 0; i <= max_stamp; i++)
    {
        path[i].position.x = pose0.position.x + coeff.position_coeff_x * i;
        path[i].position.y = pose0.position.y + coeff.position_coeff_y * i;
        path[i].position.z = pose0.position.z + coeff.position_coeff_z * i;
    }

    double a1, a2;

    if (rotation > 0.1)
    {
        // Spherical interpolation
        for (size_t i = 0; i <= max_stamp; i++)
        {
            a1 = sin(coeff.orientation_angle * (max_stamp - i ) / max_stamp) / sin(coeff.orientation_angle);
            a2 = sin(coeff.orientation_angle * i / max_stamp) / sin(coeff.orientation_angle);
            path[i].orientation.w = a1 * pose0.orientation.w + a2 * pose1.orientation.w;
            path[i].orientation.x = a1 * pose0.orientation.x + a2 * pose1.orientation.x;
            path[i].orientation.y = a1 * pose0.orientation.y + a2 * pose1.orientation.y;
            path[i].orientation.z = a1 * pose0.orientation.z + a2 * pose1.orientation.z;
        }
    }
    else
    {
        // Linear interpolation
        for (size_t i = 0; i <= max_stamp; i++)
        {
            a1 = (double)(max_stamp - i ) / max_stamp;
            a2 = (double)i / max_stamp;
            path[i].orientation.w = a1 * pose0.orientation.w + a2 * pose1.orientation.w;
            path[i].orientation.x = a1 * pose0.orientation.x + a2 * pose1.orientation.x;
            path[i].orientation.y = a1 * pose0.orientation.y + a2 * pose1.orientation.y;
            path[i].orientation.z = a1 * pose0.orientation.z + a2 * pose1.orientation.z;
        }
    }

    return SUCCESS;
}


}