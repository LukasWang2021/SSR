/*************************************************************************
	> File Name: path_plan.cpp
	> Author: 
	> Mail: 
	> Created Time: 2018年09月05日 星期三 17时55分57秒
 ************************************************************************/

#include <math.h>
#include <path_plan.h>
#include <iostream>

namespace fst_mc
{

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


}