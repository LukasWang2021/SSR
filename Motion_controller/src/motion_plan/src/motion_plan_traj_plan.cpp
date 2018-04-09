/*************************************************************************
    > File Name: motion_plan_traj_plan.cpp
    > Author: Feng Yun
    > Mail:   yun.feng@foresight-robotics.com
    > Created Time: 2018年02月08日 星期四 18时17分35秒
 ************************************************************************/

#include <iostream>
#include <float.h>
#include <fstream>
#include <string.h>

#include <fst_datatype.h>
#include <motion_plan_kinematics.h>
#include <motion_plan_motion_command.h>
#include <motion_plan_traj_plan.h>
#include <motion_plan_basic_function.h>
#include <motion_plan_variable.h>
#include <motion_plan_reuse.h>

using namespace fst_algorithm;


namespace fst_controller
{

void caculateDuration(const JointState je, const JointState js, double *alpha, double *t_min, double *t_max)
{
    // FST_INFO("alpha=%f, omega=%f, js=%f, je=%f", alpha[2], js.omega[2], js.joint[2], je.joint[2]);

    double a_max, a_min;
    double delta;
    double j, w, a;

    for (int i = 0; i < AXIS_IN_ALGORITHM; i++) {
        j = je.joint[i] - js.joint[i];
        w = js.omega[i];

        if (j > 0) {
            if (w > 0) {
                // caculate t_min:
                // j ->
                // w ->
                // a ->
                a = alpha[i];
                delta = w * w + a * j * 2;
                t_min[i] = (sqrt(delta) - w) / a;

                // caculate t_max
                // j ->
                // w ->
                // a <-
                a = -alpha[i];
                delta = w * w + a * j * 2;
                if (delta >= 0) {
                    t_max[i] = (sqrt(delta) - w) / a;
                }
                else {
                    t_max[i] = DBL_MAX;
                }
            }
            else if (w < 0) {
                // caculate t_min:
                // j ->
                // w <-
                // a ->
                a = alpha[i];
                delta = w * w + a * j * 2;
                t_min[i] = (sqrt(delta) - w) / a;

                // caculate t_max
                // j ->
                // w <-
                // a <-
                t_max[i] = DBL_MAX;
            }
            else {
                t_min[i] = sqrt(j * 2 / alpha[i]);
                t_max[i] = DBL_MAX;
            }
        }
        else if (j < 0) {
            if (w > 0) {
                // caculate t_min:
                // j <-
                // w ->
                // a <-
                a = -alpha[i];
                delta = w * w + a * j * 2;
                t_min[i] = (-sqrt(delta) - w) / a;

                // caculate t_max
                // j <-
                // w ->
                // a ->
                t_max[i] = DBL_MAX;
            }
            else if (w < 0) {
                // caculate t_min:
                // j <-
                // w <-
                // a <-
                a = -alpha[i];
                delta = w * w + a * j * 2;
                t_min[i] = (-sqrt(delta) - w) / a;

                // caculate t_max
                // j <-
                // w <-
                // a ->
                a = alpha[i];
                delta = w * w + a * j * 2;
                if (delta >= 0) {
                    t_max[i] = (-sqrt(delta) - w) / a;
                }
                else {
                    t_max[i] = DBL_MAX;
                }
            }
            else {
                t_min[i] = sqrt(j * 2 / (-alpha[i]));
                t_max[i] = DBL_MAX;
            }
        }
        else {
            if (w > 0) {
                t_min[i] = w / alpha[i] * 2;
                t_max[i] = DBL_MAX;
            }
            else if (w < 0) {
                t_min[i] = -w / alpha[i] * 2;
                t_max[i] = DBL_MAX;
            }
            else {
                t_min[i] = 0;
                t_max[i] = DBL_MAX;
            }
        }
    }
}

ErrorCode createTrajectoryFromPath(const ControlPoint &prev_point, ControlPoint &this_point)
{
    ErrorCode err;
    return err;
}


void forwardMinimumDuration(const JointState je, const JointState js, double *alpha_min, double *alpha_max, double *t_min)
{
    double delta;
    double j, w, a;

    double dj = je.joint[0] - js.joint[0];
    double ww = js.omega[0];
    double a2max = alpha_max[0];
    double a2min = alpha_min[0];

    //FST_WARN("j=%.6f, w=%.6f, a_min=%.6f, a_max=%.6f", dj, ww, a2min, a2max);

    for (int i = 0; i < AXIS_IN_ALGORITHM; i++)
    {
        j = je.joint[i] - js.joint[i];
        w = js.omega[i];

        if (j > 0)
        {
            if (w > 0)
            {
                // caculate t_min:
                // j ->
                // w ->
                if (alpha_max[i] > 0)
                {
                    a = alpha_max[i];
                    delta = w * w + a * j * 2;
                    t_min[i] = (sqrt(delta) - w) / a;
                }
                else if (alpha_max[i] < 0)
                {
                    a = alpha_max[i];
                    delta = w * w + a * j * 2;
                    t_min[i] = (sqrt(delta) - w) / a;
                }
                else
                {
                    t_min[i] = j / w;
                }
            }
            else if (w < 0)
            {
                // caculate t_min:
                // j ->
                // w <-
                if (alpha_max[i] > 0)
                {
                    a = alpha_max[i];
                    delta = w * w + a * j * 2;
                    t_min[i] = (sqrt(delta) - w) / a;
                }
                else if (alpha_max[i] <= 0)
                {
                    t_min[i] = DBL_MAX;
                }
            }
            else
            {
                if (alpha_max[i] > 0)
                {
                    t_min[i] = sqrt(j * 2 / alpha_max[i]);
                }
                else
                {
                    t_min[i] = DBL_MAX;
                }
            }
        }
        else if (j < 0)
        {
            if (w > 0)
            {
                // caculate t_min:
                // j <-
                // w ->
                if (alpha_min[i] < 0)
                {
                    a = alpha_min[i];
                    delta = w * w + a * j * 2;
                    t_min[i] = (-sqrt(delta) - w) / a;
                }
                else
                {
                    t_min[i] = DBL_MAX;
                }
            }
            else if (w < 0)
            {
                // caculate t_min:
                // j <-
                // w <-
                if (alpha_min[i] < -MINIMUM_E9)
                {
                    a = alpha_min[i];
                    delta = w * w + a * j * 2;
                    t_min[i] = (-sqrt(delta) - w) / a;
                }
                else if (alpha_min[i] > MINIMUM_E9)
                {
                    a = alpha_min[i];
                    delta = w * w + a * j * 2;
                    if (delta < 0)
                        t_min[i] = DBL_MAX;
                    else
                        t_min[i] = (-sqrt(delta) - w) / a;
                }
                else
                {
                    t_min[i] = j / w;
                }
            }
            else
            {
                if (alpha_min[i] < 0)
                    t_min[i] = sqrt(j * 2 / alpha_min[i]);
                else
                    t_min[i] = DBL_MAX;
            }
        }
        else
        {
            if (w > 0)
            {
                if (alpha_min[i] < 0)
                {
                    t_min[i] = w / -alpha_min[i] * 2;
                }
                else
                {
                    t_min[i] = DBL_MAX;
                }
            }
            else if (w < 0)
            {
                if (alpha_max[i] > 0)
                {
                    t_min[i] = -w / alpha_max[i] * 2;
                }
                else
                {
                    t_min[i] = DBL_MAX;
                }
            }
            else
            {
                if (alpha_min[i] <= 0 && alpha_max[i] >= 0)
                {
                    t_min[i] = 0;
                }
                else
                {
                    t_min[i] = DBL_MAX;
                }
            }
        }
    }
}

void forwardMaximumDuration(const JointState je, const JointState js, double *alpha_min, double *alpha_max, double *t_max)
{
    double delta;
    double j, w, a;

    double dj = je.joint[0] - js.joint[0];
    double ww = js.omega[0];
    double a2max = alpha_max[0];
    double a2min = alpha_min[0];

    //FST_WARN("j=%.6f, w=%.6f, a_min=%.6f, a_max=%.6f", dj, ww, a2min, a2max);
    //FST_WARN("alpha max: %.4f, %.4f, %.4f, %.4f, %.4f, %.4f",\
             alpha_max[0], alpha_max[1], alpha_max[2], alpha_max[3], alpha_max[4], alpha_max[5]);
    //FST_WARN("alpha min: %.4f, %.4f, %.4f, %.4f, %.4f, %.4f",\
             alpha_min[0], alpha_min[1], alpha_min[2], alpha_min[3], alpha_min[4], alpha_min[5]);

    for (int i = 0; i < AXIS_IN_ALGORITHM; i++)
    {
        j = je.joint[i] - js.joint[i];
        w = js.omega[i];

        if (j > 0)
        {
            if (w > 0)
            {
                // caculate t_max:
                // j ->
                // w ->
                if (alpha_min[i] > MINIMUM_E9)
                {
                    a = alpha_min[i];
                    delta = w * w + a * j * 2;
                    t_max[i] = (sqrt(delta) - w) / a;
                }
                else if (alpha_min[i] < -MINIMUM_E9)
                {
                    a = alpha_min[i];
                    delta = w * w + a * j * 2;
                    if (delta < 0)
                        t_max[i] = DBL_MAX;
                    else
                        t_max[i] = (sqrt(delta) - w) / a;
                }
                else
                {
                    t_max[i] = j / w;
                }
            }
            else if (w < 0)
            {
                // caculate t_max:
                // j ->
                // w <-
                if (alpha_min[i] > MINIMUM_E9)
                {
                    a = alpha_min[i];
                    delta = w * w + a * j * 2;
                    t_max[i] = (sqrt(delta) - w) / a;
                }
                else
                {
                    t_max[i] = DBL_MAX;
                }
            }
            else
            {
                // caculate t_max:
                // j ->
                // w = 0
                if (alpha_min[i] > MINIMUM_E9)
                {
                    t_max[i] = sqrt(j * 2 / alpha_min[i]);
                }
                else
                {
                    t_max[i] = DBL_MAX;
                }
            }
        }
        else if (j < 0)
        {
            if (w > 0)
            {
                // caculate t_max:
                // j <-
                // w ->
                if (alpha_max[i] < 0)
                {
                    a = alpha_max[i];
                    delta = w * w + a * j * 2;
                    t_max[i] = (-sqrt(delta) - w) / a;
                }
                else
                {
                    t_max[i] = DBL_MAX;
                }
            }
            else if (w < 0)
            {
                // caculate t_max:
                // j <-
                // w <-
                if (alpha_max[i] < 0)
                {
                    a = alpha_max[i];
                    delta = w * w + a * j * 2;
                    t_max[i] = (-sqrt(delta) - w) / a;
                }
                else if (alpha_max[i] > 0)
                {
                    a = alpha_max[i];
                    delta = w * w + a * j * 2;
                    if (delta < 0)
                        t_max[i] = DBL_MAX;
                    else
                        t_max[i] = (-sqrt(delta) - w) / a;
                }
                else
                {
                    t_max[i] = j / w;
                }
            }
            else
            {
                if (alpha_max[i] < 0)
                    t_max[i] = sqrt(j * 2 / alpha_max[i]);
                else
                    t_max[i] = DBL_MAX;
            }
        }
        else
        {
            if (w > 0)
            {
                if (alpha_min[i] < 0)
                {
                    t_max[i] = w / -alpha_min[i] * 2;
                }
                else
                {
                    t_max[i] = DBL_MAX;
                }
            }
            else if (w < 0)
            {
                if (alpha_min[i] > 0)
                {
                    t_max[i] = -w / alpha_min[i] * 2;
                }
                else
                {
                    t_max[i] = DBL_MAX;
                }
            }
            else
            {
                    t_max[i] = DBL_MAX;
            }
        }
    }
}

std::ofstream os("cart_vel.csv");

extern bool tflag;
int cc = 0;
ErrorCode foreCycle(const ControlPoint &prev_point, ControlPoint &this_point, int flg)
{
    ErrorCode err = SUCCESS;

    // determin expect duration time
    double distance = getDistance(prev_point.path_point.pose, this_point.path_point.pose);
    this_point.expect_duration = distance / this_point.path_point.source->getCommandVelocity();

        // increase speed or constant speed
    double t_max[AXIS_IN_ALGORITHM];
    double t_min[AXIS_IN_ALGORITHM];
    double alpha_max[AXIS_IN_ALGORITHM];
    double alpha_min[AXIS_IN_ALGORITHM];

    // for test only
    double jerk[AXIS_IN_ALGORITHM] = {5.258, 4.445, 11.737, 197.644, 75.21, 105.61};

    /*
    double tm;
    if (prev_point.duration > MINIMUM_E9)
    {
        tm = prev_point.duration;
    }
    else
    {
        tm = distance / 50;
    }

    for (int i = 0; i < AXIS_IN_ALGORITHM; i++)
    {
        if (flg == 2)
        {
            alpha_max[i] = prev_point.point.alpha[i] + tm * jerk[i] * 2;
            alpha_min[i] = prev_point.point.alpha[i] - tm * jerk[i] * 2;
        }
        else
        {
            alpha_max[i] = prev_point.point.alpha[i] + tm * jerk[i];
            alpha_min[i] = prev_point.point.alpha[i] - tm * jerk[i];
        }
    }
    */

    for (int i = 0; i < AXIS_IN_ALGORITHM; i++)
    {
        if (flg == 2)
        {
            alpha_max[i] = prev_point.point.alpha[i] + jerk[i] * 2;
            alpha_min[i] = prev_point.point.alpha[i] - jerk[i] * 2;
        }
        else
        {
            alpha_max[i] = prev_point.point.alpha[i] + jerk[i];
            alpha_min[i] = prev_point.point.alpha[i] - jerk[i];
        }
    }

    alpha_max[0] = alpha_max[0] < g_soft_constraint.j1.max_alpha ? alpha_max[0] : g_soft_constraint.j1.max_alpha;
    alpha_max[1] = alpha_max[1] < g_soft_constraint.j2.max_alpha ? alpha_max[1] : g_soft_constraint.j2.max_alpha;
    alpha_max[2] = alpha_max[2] < g_soft_constraint.j3.max_alpha ? alpha_max[2] : g_soft_constraint.j3.max_alpha;
    alpha_max[3] = alpha_max[3] < g_soft_constraint.j4.max_alpha ? alpha_max[3] : g_soft_constraint.j4.max_alpha;
    alpha_max[4] = alpha_max[4] < g_soft_constraint.j5.max_alpha ? alpha_max[4] : g_soft_constraint.j5.max_alpha;
    alpha_max[5] = alpha_max[5] < g_soft_constraint.j6.max_alpha ? alpha_max[5] : g_soft_constraint.j6.max_alpha;

    alpha_min[0] = alpha_min[0] > -g_soft_constraint.j1.max_alpha ? alpha_min[0] : -g_soft_constraint.j1.max_alpha;
    alpha_min[1] = alpha_min[1] > -g_soft_constraint.j2.max_alpha ? alpha_min[1] : -g_soft_constraint.j1.max_alpha;
    alpha_min[2] = alpha_min[2] > -g_soft_constraint.j3.max_alpha ? alpha_min[2] : -g_soft_constraint.j1.max_alpha;
    alpha_min[3] = alpha_min[3] > -g_soft_constraint.j4.max_alpha ? alpha_min[3] : -g_soft_constraint.j1.max_alpha;
    alpha_min[4] = alpha_min[4] > -g_soft_constraint.j5.max_alpha ? alpha_min[4] : -g_soft_constraint.j1.max_alpha;
    alpha_min[5] = alpha_min[5] > -g_soft_constraint.j6.max_alpha ? alpha_min[5] : -g_soft_constraint.j1.max_alpha;

    //FST_WARN("alpha max: %.4f, %.4f, %.4f, %.4f, %.4f, %.4f",\
             alpha_max[0], alpha_max[1], alpha_max[2], alpha_max[3], alpha_max[4], alpha_max[5]);
    //FST_WARN("alpha min: %.4f, %.4f, %.4f, %.4f, %.4f, %.4f",\
             alpha_min[0], alpha_min[1], alpha_min[2], alpha_min[3], alpha_min[4], alpha_min[5]);

    forwardMinimumDuration(this_point.point, prev_point.point, alpha_min, alpha_max, t_min);

    //FST_INFO("min duration: %f,%f,%f,%f,%f,%f", t_min[0], t_min[1], t_min[2], t_min[3], t_min[4], t_min[5]);

    forwardMaximumDuration(this_point.point, prev_point.point, alpha_min, alpha_max, t_max);

    //FST_INFO("max duration: %f,%f,%f,%f,%f,%f", t_max[0], t_max[1], t_max[2], t_max[3], t_max[4], t_max[5]);

    // determin duration Time
    double min_duration_max = t_min[0];
    double max_duration_min = t_max[0];
    size_t min_duration_limit_index  = 0;
    size_t max_duration_limit_index  = 0;
    
    for (int i = 1; i < AXIS_IN_ALGORITHM; i++)
    {
        if (t_min[i] > min_duration_max) {
            min_duration_limit_index = i;
            min_duration_max  = t_min[i];
        }
        if (t_max[i] < max_duration_min) {
            max_duration_limit_index = i;
            max_duration_min  = t_max[i];
        }
    }

    //FST_INFO("#### flg=%d###########", flg);
    //FST_INFO("#### min=%f, max=%f, exp=%f", min_duration_max, max_duration_min, this_point.expect_duration);
    if (min_duration_max < max_duration_min)
    {
        if (flg == 0)
        {
            if (min_duration_max > this_point.expect_duration)
            {
                this_point.duration = min_duration_max;
            }
            else if (max_duration_min < this_point.expect_duration)
            {
                this_point.duration = max_duration_min;
            }
            else
            {
                this_point.duration = this_point.expect_duration;
            }
        }
        else if (flg == 1)
        {
            if (min_duration_max > this_point.expect_duration)
            {
                this_point.duration = min_duration_max;
            }
            else if (max_duration_min < this_point.expect_duration)
            {
                this_point.duration = max_duration_min;
            }
            else
            {
                this_point.duration = this_point.expect_duration;
            }
        }
        else if (flg == 2)
        {
            this_point.duration = max_duration_min;
        }
    }
    else
    {
        FST_ERROR("duration error, fail to create trajectory !!!");
        err = MOTION_INTERNAL_FAULT;
    }


    this_point.time_from_start = prev_point.time_from_start + this_point.duration;

    this_point.point.alpha[0] = (this_point.point.joint[0] - prev_point.point.joint[0] - prev_point.point.omega[0] * this_point.duration) * 2 / this_point.duration / this_point.duration;
    this_point.point.alpha[1] = (this_point.point.joint[1] - prev_point.point.joint[1] - prev_point.point.omega[1] * this_point.duration) * 2 / this_point.duration / this_point.duration;
    this_point.point.alpha[2] = (this_point.point.joint[2] - prev_point.point.joint[2] - prev_point.point.omega[2] * this_point.duration) * 2 / this_point.duration / this_point.duration;
    this_point.point.alpha[3] = (this_point.point.joint[3] - prev_point.point.joint[3] - prev_point.point.omega[3] * this_point.duration) * 2 / this_point.duration / this_point.duration;
    this_point.point.alpha[4] = (this_point.point.joint[4] - prev_point.point.joint[4] - prev_point.point.omega[4] * this_point.duration) * 2 / this_point.duration / this_point.duration;
    this_point.point.alpha[5] = (this_point.point.joint[5] - prev_point.point.joint[5] - prev_point.point.omega[5] * this_point.duration) * 2 / this_point.duration / this_point.duration;

    this_point.point.omega[0] = prev_point.point.omega[0] + this_point.point.alpha[0] * this_point.duration;
    this_point.point.omega[1] = prev_point.point.omega[1] + this_point.point.alpha[1] * this_point.duration;
    this_point.point.omega[2] = prev_point.point.omega[2] + this_point.point.alpha[2] * this_point.duration;
    this_point.point.omega[3] = prev_point.point.omega[3] + this_point.point.alpha[3] * this_point.duration;
    this_point.point.omega[4] = prev_point.point.omega[4] + this_point.point.alpha[4] * this_point.duration;
    this_point.point.omega[5] = prev_point.point.omega[5] + this_point.point.alpha[5] * this_point.duration;

    this_point.time_from_start = prev_point.time_from_start + this_point.duration;
        
    JointState &p = this_point.point;
    //FST_INFO("stamp=%d, time_from_start=%.6f", this_point.path_point.stamp, this_point.time_from_start);
    //FST_INFO("min_duration_max=%.6f, max_duration_min=%.6f", min_duration_max, max_duration_min);
    //FST_INFO("duration=%.6f, exp duration=%.6f", this_point.duration, this_point.expect_duration);
    //FST_WARN("dis=%f, velocity=%.6f, exp velocity=%.6f", distance, distance / this_point.duration, distance / this_point.expect_duration);
    //FST_INFO("j1-j6: %.6f, %.6f, %.6f, %.6f, %.6f, %.6f", p.joint[0], p.joint[1], p.joint[2], p.joint[3], p.joint[4], p.joint[5]);
    //FST_INFO("w1-w6: %.6f, %.6f, %.6f, %.6f, %.6f, %.6f", p.omega[0], p.omega[1], p.omega[2], p.omega[3], p.omega[4], p.omega[5]);
    //FST_INFO("a1-a6: %.6f, %.6f, %.6f, %.6f, %.6f, %.6f", p.alpha[0], p.alpha[1], p.alpha[2], p.alpha[3], p.alpha[4], p.alpha[5]);
    
    /*
    os << this_point.time_from_start << "," 
    << distance / this_point.duration << "," 
    << distance / this_point.expect_duration << ","
    << min_duration_max << ","
    << max_duration_min << ","
    << std::endl;
    */
    return err;
}

ErrorCode backCycle(ControlPoint &next_point, ControlPoint &this_point, int flg)
{
    ErrorCode err = SUCCESS;

    // determin expect duration time
    double distance = getDistance(this_point.path_point.pose, next_point.path_point.pose);
    next_point.expect_duration = distance / next_point.path_point.source->getCommandVelocity();

    double t_max[AXIS_IN_ALGORITHM];
    double t_min[AXIS_IN_ALGORITHM];
    double alpha_max[AXIS_IN_ALGORITHM];
    double alpha_min[AXIS_IN_ALGORITHM];

    // for test only
    double jerk[AXIS_IN_ALGORITHM] = {5.258, 4.445, 11.737, 197.644, 75.21, 105.61};

    for (int i = 0; i < AXIS_IN_ALGORITHM; i++)
    {
        if (flg == 2)
        {
            alpha_max[i] = next_point.point.alpha[i] + jerk[i] * 2;
            alpha_min[i] = next_point.point.alpha[i] - jerk[i] * 2;
        }
        else
        {
            alpha_max[i] = next_point.point.alpha[i] + jerk[i];
            alpha_min[i] = next_point.point.alpha[i] - jerk[i];
        }
    }

    //FST_WARN("alpha:     %.4f, %.4f, %.4f,  %.4f, %.4f, %.4f",\
             next_point.point.alpha[0], next_point.point.alpha[1], next_point.point.alpha[2],\
             next_point.point.alpha[3], next_point.point.alpha[4], next_point.point.alpha[5]);
    //FST_WARN("alpha max: %.4f, %.4f, %.4f, %.4f, %.4f, %.4f",\
             alpha_max[0], alpha_max[1], alpha_max[2], alpha_max[3], alpha_max[4], alpha_max[5]);
    //FST_WARN("alpha min: %.4f, %.4f, %.4f, %.4f, %.4f, %.4f",\
             alpha_min[0], alpha_min[1], alpha_min[2], alpha_min[3], alpha_min[4], alpha_min[5]);
    //FST_WARN("this joint:%.4f, %.4f, %.4f,  %.4f, %.4f, %.4f",\
             this_point.point.joint[0], this_point.point.joint[1], this_point.point.joint[2],\
             this_point.point.joint[3], this_point.point.joint[4], this_point.point.joint[5]);    

    alpha_max[0] = alpha_max[0] < g_soft_constraint.j1.max_alpha ? alpha_max[0] : g_soft_constraint.j1.max_alpha;
    alpha_max[1] = alpha_max[1] < g_soft_constraint.j2.max_alpha ? alpha_max[1] : g_soft_constraint.j2.max_alpha;
    alpha_max[2] = alpha_max[2] < g_soft_constraint.j3.max_alpha ? alpha_max[2] : g_soft_constraint.j3.max_alpha;
    alpha_max[3] = alpha_max[3] < g_soft_constraint.j4.max_alpha ? alpha_max[3] : g_soft_constraint.j4.max_alpha;
    alpha_max[4] = alpha_max[4] < g_soft_constraint.j5.max_alpha ? alpha_max[4] : g_soft_constraint.j5.max_alpha;
    alpha_max[5] = alpha_max[5] < g_soft_constraint.j6.max_alpha ? alpha_max[5] : g_soft_constraint.j6.max_alpha;

    alpha_min[0] = alpha_min[0] > -g_soft_constraint.j1.max_alpha ? alpha_min[0] : -g_soft_constraint.j1.max_alpha;
    alpha_min[1] = alpha_min[1] > -g_soft_constraint.j2.max_alpha ? alpha_min[1] : -g_soft_constraint.j1.max_alpha;
    alpha_min[2] = alpha_min[2] > -g_soft_constraint.j3.max_alpha ? alpha_min[2] : -g_soft_constraint.j1.max_alpha;
    alpha_min[3] = alpha_min[3] > -g_soft_constraint.j4.max_alpha ? alpha_min[3] : -g_soft_constraint.j1.max_alpha;
    alpha_min[4] = alpha_min[4] > -g_soft_constraint.j5.max_alpha ? alpha_min[4] : -g_soft_constraint.j1.max_alpha;
    alpha_min[5] = alpha_min[5] > -g_soft_constraint.j6.max_alpha ? alpha_min[5] : -g_soft_constraint.j1.max_alpha;


    forwardMinimumDuration(this_point.point, next_point.point, alpha_min, alpha_max, t_min);

    //FST_INFO("min duration: %f,%f,%f,%f,%f,%f", t_min[0], t_min[1], t_min[2], t_min[3], t_min[4], t_min[5]);

    forwardMaximumDuration(this_point.point, next_point.point, alpha_min, alpha_max, t_max);

    //FST_INFO("max duration: %f,%f,%f,%f,%f,%f", t_max[0], t_max[1], t_max[2], t_max[3], t_max[4], t_max[5]);

    // determin duration Time
    double min_duration_max = t_min[0];
    double max_duration_min = t_max[0];
    size_t min_duration_limit_index  = 0;
    size_t max_duration_limit_index  = 0;
    
    for (int i = 1; i < AXIS_IN_ALGORITHM; i++)
    {
        if (t_min[i] > min_duration_max) {
            min_duration_limit_index = i;
            min_duration_max  = t_min[i];
        }
        if (t_max[i] < max_duration_min) {
            max_duration_limit_index = i;
            max_duration_min  = t_max[i];
        }
    }

    if (min_duration_max < max_duration_min)
    {
        if (flg == 0)
        {
            if (min_duration_max > next_point.expect_duration)
            {
                next_point.duration = min_duration_max;
            }
            else if (max_duration_min < next_point.expect_duration)
            {
                next_point.duration = max_duration_min;
            }
            else
            {
                next_point.duration = next_point.expect_duration;
            }
        }
        else if (flg == 1)
        {
            if (min_duration_max > next_point.expect_duration)
            {
                next_point.duration = min_duration_max;
            }
            else if (max_duration_min < next_point.expect_duration)
            {
                next_point.duration = max_duration_min;
            }
            else
            {
                next_point.duration = this_point.expect_duration;
            }
        }
        else if (flg == 2)
        {
            next_point.duration = max_duration_min;
        }
    }
    else
    {
        FST_ERROR("duration error, fail to create trajectory !!!");
        err = MOTION_INTERNAL_FAULT;
    }

    next_point.time_from_start = -1;

    next_point.point.alpha[0] = (this_point.point.joint[0] - next_point.point.joint[0] - next_point.point.omega[0] * next_point.duration) * 2 / next_point.duration / next_point.duration;
    next_point.point.alpha[1] = (this_point.point.joint[1] - next_point.point.joint[1] - next_point.point.omega[1] * next_point.duration) * 2 / next_point.duration / next_point.duration;
    next_point.point.alpha[2] = (this_point.point.joint[2] - next_point.point.joint[2] - next_point.point.omega[2] * next_point.duration) * 2 / next_point.duration / next_point.duration;
    next_point.point.alpha[3] = (this_point.point.joint[3] - next_point.point.joint[3] - next_point.point.omega[3] * next_point.duration) * 2 / next_point.duration / next_point.duration;
    next_point.point.alpha[4] = (this_point.point.joint[4] - next_point.point.joint[4] - next_point.point.omega[4] * next_point.duration) * 2 / next_point.duration / next_point.duration;
    next_point.point.alpha[5] = (this_point.point.joint[5] - next_point.point.joint[5] - next_point.point.omega[5] * next_point.duration) * 2 / next_point.duration / next_point.duration;

    memcpy(this_point.point.alpha, next_point.point.alpha, AXIS_IN_ALGORITHM * sizeof(double));


    this_point.point.omega[0] = next_point.point.omega[0] + next_point.point.alpha[0] * next_point.duration;
    this_point.point.omega[1] = next_point.point.omega[1] + next_point.point.alpha[1] * next_point.duration;
    this_point.point.omega[2] = next_point.point.omega[2] + next_point.point.alpha[2] * next_point.duration;
    this_point.point.omega[3] = next_point.point.omega[3] + next_point.point.alpha[3] * next_point.duration;
    this_point.point.omega[4] = next_point.point.omega[4] + next_point.point.alpha[4] * next_point.duration;
    this_point.point.omega[5] = next_point.point.omega[5] + next_point.point.alpha[5] * next_point.duration;

    next_point.point.alpha[0] = -next_point.point.alpha[0];
    next_point.point.alpha[1] = -next_point.point.alpha[1];
    next_point.point.alpha[2] = -next_point.point.alpha[2];
    next_point.point.alpha[3] = -next_point.point.alpha[3];
    next_point.point.alpha[4] = -next_point.point.alpha[4];
    next_point.point.alpha[5] = -next_point.point.alpha[5];

    next_point.point.omega[0] = - next_point.point.omega[0];
    next_point.point.omega[1] = - next_point.point.omega[1];
    next_point.point.omega[2] = - next_point.point.omega[2];
    next_point.point.omega[3] = - next_point.point.omega[3];
    next_point.point.omega[4] = - next_point.point.omega[4];
    next_point.point.omega[5] = - next_point.point.omega[5];

    JointState &p = next_point.point;
    //FST_INFO("stamp=%d, time_from_start=%.6f", next_point.path_point.stamp, next_point.time_from_start);
    //FST_INFO("min_duration_max=%.6f, max_duration_min=%.6f", min_duration_max, max_duration_min);
    //FST_INFO("duration=%.6f, exp duration=%.6f", next_point.duration, next_point.expect_duration);
    //FST_WARN("dis=%f, velocity=%.6f, exp velocity=%.6f", distance, distance / next_point.duration, distance / next_point.expect_duration);
    //FST_INFO("j1-j6: %.6f, %.6f, %.6f, %.6f, %.6f, %.6f", p.joint[0], p.joint[1], p.joint[2], p.joint[3], p.joint[4], p.joint[5]);
    //FST_INFO("w1-w6: %.6f, %.6f, %.6f, %.6f, %.6f, %.6f", p.omega[0], p.omega[1], p.omega[2], p.omega[3], p.omega[4], p.omega[5]);
    //FST_INFO("a1-a6: %.6f, %.6f, %.6f, %.6f, %.6f, %.6f", p.alpha[0], p.alpha[1], p.alpha[2], p.alpha[3], p.alpha[4], p.alpha[5]);

    return err;
}

ErrorCode createTrajFromPath(const ControlPoint &prev_point, ControlPoint &this_point)
{
    ErrorCode err;

    // determin expect duration time
    double distance;
    if (this_point.path_point.type == MOTION_LINE || this_point.path_point.type == MOTION_CIRCLE)
    {
        if (prev_point.path_point.type == MOTION_LINE || prev_point.path_point.type == MOTION_CIRCLE)
        {
            distance = getDistance(prev_point.path_point.pose, this_point.path_point.pose);
            this_point.expect_duration = distance / this_point.path_point.source->getCommandVelocity();
        }
        else
        {
            distance = getDistance(forwardKinematics(*((Joint*)prev_point.point.joint)), this_point.path_point.pose);
            this_point.expect_duration = distance / this_point.path_point.source->getCommandVelocity();
        }
    }

    // increase speed or constant speed
    double t_max[AXIS_IN_ALGORITHM];
    double t_min[AXIS_IN_ALGORITHM];
    double alpha_max[AXIS_IN_ALGORITHM];
    double alpha_min[AXIS_IN_ALGORITHM];

    // for test only
    double jerk[AXIS_IN_ALGORITHM] = {5.26, 4.44, 11.74, 197.64, 75.21, 105.61};

    for (int i = 0; i < AXIS_IN_ALGORITHM; i++)
    {
        alpha_max[i] = prev_point.point.alpha[i] + jerk[i];
        alpha_min[i] = prev_point.point.alpha[i] - jerk[i];
    }

    alpha_max[0] = alpha_max[0] < g_soft_constraint.j1.max_alpha ? alpha_max[0] : g_soft_constraint.j1.max_alpha;
    alpha_max[1] = alpha_max[1] < g_soft_constraint.j2.max_alpha ? alpha_max[1] : g_soft_constraint.j2.max_alpha;
    alpha_max[2] = alpha_max[2] < g_soft_constraint.j3.max_alpha ? alpha_max[2] : g_soft_constraint.j3.max_alpha;
    alpha_max[3] = alpha_max[3] < g_soft_constraint.j4.max_alpha ? alpha_max[3] : g_soft_constraint.j4.max_alpha;
    alpha_max[4] = alpha_max[4] < g_soft_constraint.j5.max_alpha ? alpha_max[4] : g_soft_constraint.j5.max_alpha;
    alpha_max[5] = alpha_max[5] < g_soft_constraint.j6.max_alpha ? alpha_max[5] : g_soft_constraint.j6.max_alpha;

    alpha_min[0] = alpha_min[0] > -g_soft_constraint.j1.max_alpha ? alpha_min[0] : -g_soft_constraint.j1.max_alpha;
    alpha_min[1] = alpha_min[1] > -g_soft_constraint.j2.max_alpha ? alpha_min[1] : -g_soft_constraint.j1.max_alpha;
    alpha_min[2] = alpha_min[2] > -g_soft_constraint.j3.max_alpha ? alpha_min[2] : -g_soft_constraint.j1.max_alpha;
    alpha_min[3] = alpha_min[3] > -g_soft_constraint.j4.max_alpha ? alpha_min[3] : -g_soft_constraint.j1.max_alpha;
    alpha_min[4] = alpha_min[4] > -g_soft_constraint.j5.max_alpha ? alpha_min[4] : -g_soft_constraint.j1.max_alpha;
    alpha_min[5] = alpha_min[5] > -g_soft_constraint.j6.max_alpha ? alpha_min[5] : -g_soft_constraint.j1.max_alpha;


    forwardMinimumDuration(this_point.point, prev_point.point, alpha_min, alpha_max, t_min);

    //FST_INFO("min duration: %f,%f,%f,%f,%f,%f", t_min[0], t_min[1], t_min[2], t_min[3], t_min[4], t_min[5]);

    forwardMaximumDuration(this_point.point, prev_point.point, alpha_min, alpha_max, t_max);

    //FST_INFO("max duration: %f,%f,%f,%f,%f,%f", t_max[0], t_max[1], t_max[2], t_max[3], t_max[4], t_max[5]);

    // determin duration Time
    double min_duration_max = t_min[0];
    double max_duration_min = t_max[0];
    size_t min_duration_limit_index  = 0;
    size_t max_duration_limit_index  = 0;
    
    for (int i = 1; i < AXIS_IN_ALGORITHM; i++)
    {
        if (t_min[i] > min_duration_max) {
            min_duration_limit_index = i;
            min_duration_max  = t_min[i];
        }
        if (t_max[i] < max_duration_min) {
            max_duration_limit_index = i;
            max_duration_min  = t_max[i];
        }
    }


    if (min_duration_max < max_duration_min)
    {
        if (min_duration_max > this_point.expect_duration * 1.1)
        {
            this_point.duration = min_duration_max;
        }
        else if (max_duration_min < this_point.expect_duration * 0.9)
        {
            this_point.duration = max_duration_min;
        }
        else
        {
            this_point.duration = (min_duration_max + max_duration_min) / 2;
        }
    }
    else
    {
        FST_ERROR("duration error, fail to create trajectory !!!");
    }

    this_point.time_from_start = prev_point.time_from_start + this_point.duration;

    this_point.point.alpha[0] = (this_point.point.joint[0] - prev_point.point.joint[0] - prev_point.point.omega[0] * this_point.duration) * 2 / this_point.duration / this_point.duration;
    this_point.point.alpha[1] = (this_point.point.joint[1] - prev_point.point.joint[1] - prev_point.point.omega[1] * this_point.duration) * 2 / this_point.duration / this_point.duration;
    this_point.point.alpha[2] = (this_point.point.joint[2] - prev_point.point.joint[2] - prev_point.point.omega[2] * this_point.duration) * 2 / this_point.duration / this_point.duration;
    this_point.point.alpha[3] = (this_point.point.joint[3] - prev_point.point.joint[3] - prev_point.point.omega[3] * this_point.duration) * 2 / this_point.duration / this_point.duration;
    this_point.point.alpha[4] = (this_point.point.joint[4] - prev_point.point.joint[4] - prev_point.point.omega[4] * this_point.duration) * 2 / this_point.duration / this_point.duration;
    this_point.point.alpha[5] = (this_point.point.joint[5] - prev_point.point.joint[5] - prev_point.point.omega[5] * this_point.duration) * 2 / this_point.duration / this_point.duration;

    this_point.point.omega[0] = prev_point.point.omega[0] + this_point.point.alpha[0] * this_point.duration;
    this_point.point.omega[1] = prev_point.point.omega[1] + this_point.point.alpha[1] * this_point.duration;
    this_point.point.omega[2] = prev_point.point.omega[2] + this_point.point.alpha[2] * this_point.duration;
    this_point.point.omega[3] = prev_point.point.omega[3] + this_point.point.alpha[3] * this_point.duration;
    this_point.point.omega[4] = prev_point.point.omega[4] + this_point.point.alpha[4] * this_point.duration;
    this_point.point.omega[5] = prev_point.point.omega[5] + this_point.point.alpha[5] * this_point.duration;

    this_point.time_from_start = prev_point.time_from_start + this_point.duration;
        
    JointState &p = this_point.point;
    FST_INFO("stamp=%d, time_from_start=%.6f", this_point.path_point.stamp, this_point.time_from_start);
    FST_INFO("min_duration_max=%.6f, max_duration_min=%.6f", min_duration_max, max_duration_min);
    FST_INFO("duration=%.6f, exp duration=%.6f", this_point.duration, this_point.expect_duration);
    FST_INFO("velocity=%.6f, exp velocity=%.6f", distance / this_point.duration, distance / this_point.expect_duration);
    FST_INFO("j1-j6: %.6f, %.6f, %.6f, %.6f, %.6f, %.6f", p.joint[0], p.joint[1], p.joint[2], p.joint[3], p.joint[4], p.joint[5]);
    FST_INFO("w1-w6: %.6f, %.6f, %.6f, %.6f, %.6f, %.6f", p.omega[0], p.omega[1], p.omega[2], p.omega[3], p.omega[4], p.omega[5]);
    FST_INFO("a1-a6: %.6f, %.6f, %.6f, %.6f, %.6f, %.6f", p.alpha[0], p.alpha[1], p.alpha[2], p.alpha[3], p.alpha[4], p.alpha[5]);
    //os << distance / this_point.duration << "," << distance / this_point.expect_duration << std::endl;

    return SUCCESS;
}

void computeDurationMax(Angle* start_joint_ptr, Angle* end_joint_ptr, Omega* start_omega_ptr, 
                                Alpha* acc_limit, Omega* velocity_limit, MotionTime& duration_max)
{
    MotionTime duration[AXIS_IN_ALGORITHM] = {0};
    double cond_part1 = 0, cond_part2 = 0;
    Omega fabs_start_omega;
    duration_max = 0;
    for(int i = 0; i < AXIS_IN_ALGORITHM; ++i)
    {
        fabs_start_omega = fabs(start_omega_ptr[i]);
        if(fabs(fabs_start_omega - velocity_limit[i]) < MINIMUM_E9
            || fabs_start_omega > velocity_limit[i]) // omega reach maximum
        {
            duration[i] = fabs(end_joint_ptr[i] - start_joint_ptr[i]) / fabs_start_omega;
        }
        else    // omega is under maximum
        {
            cond_part1 = pow(fabs_start_omega, 2);
            cond_part2 = 2 * acc_limit[i] * fabs(end_joint_ptr[i] - start_joint_ptr[i]);
            duration[i] = (-fabs_start_omega + sqrt(cond_part1 + cond_part2)) / acc_limit[i];
        }
        
        if(duration_max < duration[i])
        {
            duration_max = duration[i];
        }  
    }
/*if(is_pause)
    FST_INFO("duration: %f, %f, %f, %f, %f, %f", duration[0], duration[1], duration[2], duration[3], duration[4], duration[5]);*/
}

void computeDurationMin(Angle* start_joint_ptr, Angle* end_joint_ptr, Omega* start_omega_ptr, 
                                Alpha* acc_limit, Omega* velocity_limit, MotionTime& duration_min)
{
    MotionTime duration[AXIS_IN_ALGORITHM] = {0};
    double cond_part1 = 0, cond_part2 = 0;
    Omega fabs_start_omega;
    duration_min = DBL_MAX;
    for(int i = 0; i < AXIS_IN_ALGORITHM; ++i)
    {
        fabs_start_omega = fabs(start_omega_ptr[i]);
        cond_part1 = pow(fabs_start_omega, 2);
        cond_part2 = 2 * acc_limit[i] * fabs(end_joint_ptr[i] - start_joint_ptr[i]);
        if(cond_part1 >= cond_part2 
            && fabs(end_joint_ptr[i] - start_joint_ptr[i]) > MINIMUM_E9)
        {
            duration[i] = (fabs_start_omega - sqrt(cond_part1 - cond_part2)) / acc_limit[i];
        }
        else
        {
            duration[i] = DBL_MAX;
        }

        if(duration_min > duration[i])
        {
            duration_min = duration[i];
        }  
    }
}

void computeLastDurationMin(Angle* start_joint_ptr, Angle* end_joint_ptr, Omega* start_omega_ptr, MotionTime& duration_min)
{
    MotionTime duration[AXIS_IN_ALGORITHM] = {0};
    // if all axes reach the last step of pause, it might cause duration_min goes to DBL_MAX.
    // take the minimum duration of all axes needed to by applying under-limit acceleration.
    if(duration_min == DBL_MAX)
    {
        //duration_min = 0;
        for(int i = 0; i < AXIS_IN_ALGORITHM; ++i)
        {
            if(fabs(start_omega_ptr[i]) > MINIMUM_E9)
            {
                duration[i] = 2 * fabs((end_joint_ptr[i] - start_joint_ptr[i]) / start_omega_ptr[i]);
                if(duration_min > duration[i])
                {
                    duration_min = duration[i];
                }
            }
        }
    }
}


void computeTrajectory(bool is_pause, bool is_forward, size_t target_tick, Angle* start_joint_ptr, Angle* end_joint_ptr, Omega* start_omega_ptr, 
                            MotionTime duration, Alpha* acc_limit, Omega* velocity_limit, ControlPoint* target)
{
    Alpha acc = 0;
    for(int i = 0; i < AXIS_IN_ALGORITHM; ++i)
    {
        if(target->path_point.type == MOTION_JOINT)
        {
            target[target_tick].point.joint[i] = ((Angle*)&target[target_tick].path_point.joint)[i];
        }

        acc = 2 * (end_joint_ptr[i] - start_joint_ptr[i] - start_omega_ptr[i] * duration) / duration / duration;
        if(acc > acc_limit[i])
        {
            acc = acc_limit[i];
        }
        else if(acc < -acc_limit[i])
        {
            acc = -acc_limit[i];
        }
        else{}
        
        if(is_forward)
        {
            target[target_tick].point.alpha[i] = acc;
            target[target_tick].point.omega[i] = start_omega_ptr[i] + target[target_tick].point.alpha[i] * duration;
        }
        else
        {
            target[target_tick + 1].point.alpha[i] = -acc;
            target[target_tick].point.omega[i] = start_omega_ptr[i] - target[target_tick + 1].point.alpha[i] * duration;
        }

        if(target[target_tick].point.omega[i] > velocity_limit[i])
        {
            target[target_tick].point.omega[i] = velocity_limit[i];
        }
        else if(target[target_tick].point.omega[i] < -velocity_limit[i])
        {
            target[target_tick].point.omega[i] = -velocity_limit[i];
        }
        else{}
        
        if(is_pause)
        {
            // this condition maybe not reached for all axes at the same time, fix it later
            if(fabs(target[target_tick].point.omega[i]) < MINIMUM_E9
                || target[target_tick].point.omega[i] * target[target_tick - 1].point.omega[i] < 0)
            {
                target[target_tick].point.omega[i] = 0;
            }
        }
    }
}



}

