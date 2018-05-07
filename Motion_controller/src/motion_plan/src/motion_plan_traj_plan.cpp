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
    FST_INFO("js=%.6f, je=%.6f, w=%.6f, alimit=%.6f, wlimit=%.6f, duration_max=%.6f",
             start_joint_ptr[4], end_joint_ptr[4], start_omega_ptr[4], acc_limit[4], velocity_limit[4], duration_max);
/*if(is_pause)
    FST_INFO("duration: %f, %f, %f, %f, %f, %f", duration[0], duration[1], duration[2], duration[3], duration[4], duration[5]);*/
}

void computeDurationMin(Angle* start_joint_ptr, Angle* end_joint_ptr, Omega* start_omega_ptr, 
                                Alpha* acc_limit, Omega* velocity_limit, MotionTime& duration_min)
{
    MotionTime duration[AXIS_IN_ALGORITHM] = {0};
    double cond_part1 = 0, cond_part2 = 0;
    Omega fabs_start_omega;
    duration_min = 99.99;
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
            duration[i] = 99.99;
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
    if(duration_min > 99)
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


MotionTime computeDurationMax(Angle *js, Angle *je, Omega *ws, Alpha *acc_limit, Omega *vel_limit)
{
    double cond_part1 = 0, cond_part2 = 0;
    Omega fabs_start_omega;
    MotionTime duration_max = 0;
    MotionTime duration;
    //FST_INFO("js=%.6f,%.6f,%.6f,%.6f,%.6f,%.6f", js[0],js[1],js[2],js[3],js[4],js[5]);
    //FST_INFO("ws=%.6f,%.6f,%.6f,%.6f,%.6f,%.6f", ws[0],ws[1],ws[2],ws[3],ws[4],ws[5]);
    //FST_INFO("je=%.6f,%.6f,%.6f,%.6f,%.6f,%.6f", je[0],je[1],je[2],je[3],je[4],je[5]);
    //FST_INFO("al=%.6f,%.6f,%.6f,%.6f,%.6f,%.6f", acc_limit[0],acc_limit[1],acc_limit[2],acc_limit[3],acc_limit[4],acc_limit[5]);
    //FST_INFO("wl=%.6f,%.6f,%.6f,%.6f,%.6f,%.6f", vel_limit[0],vel_limit[1],vel_limit[2],vel_limit[3],vel_limit[4],vel_limit[5]);

    for(size_t i = 0; i < AXIS_IN_ALGORITHM; ++i)
    {
        fabs_start_omega = fabs(ws[i]);
        if(fabs(fabs_start_omega - vel_limit[i]) < MINIMUM_E9
            || fabs_start_omega > vel_limit[i]) // omega reach maximum
        {
            duration = fabs(je[i] - js[i]) / fabs_start_omega;
        }
        else    // omega is under maximum
        {
            cond_part1 = pow(fabs_start_omega, 2);
            cond_part2 = 2 * acc_limit[i] * fabs(je[i] - js[i]);
            duration = (-fabs_start_omega + sqrt(cond_part1 + cond_part2)) / acc_limit[i];
            //FST_INFO("max: part1=%.6f,part2=%.6f,duration=%.6f", cond_part1, cond_part2, duration);

            if (fabs_start_omega + acc_limit[i] * duration > vel_limit[i] + MINIMUM_E9)
            {
                duration = fabs(je[i] - js[i]) * 2 / (fabs_start_omega + vel_limit[i]);
            }
        }
        /*
        if (i == 4)
            FST_INFO("J5: js=%.6f, je=%.6f, ws=%.6f, a_lim=%.6f, w_lim=%.6f, part1=%.6f, part2=%.6f, duration=%.6f",
                     js[i], je[i], ws[i], acc_limit[i], vel_limit[i], cond_part1, cond_part2, duration);
        */
        if(duration_max < duration)
        {
            duration_max = duration;
        }
    }
    //FST_INFO("js=%.6f, je=%.6f, ws=%.6f, alimit=%.6f, wlimit=%.6f, duration_max=%.6f",
    //         js[4], je[4], ws[4], acc_limit[4], vel_limit[4], duration_max);
/*if(is_pause)
    FST_INFO("duration: %f, %f, %f, %f, %f, %f", duration[0], duration[1], duration[2], duration[3], duration[4], duration[5]);*/
    return duration_max;
}

MotionTime computeDurationMin(Angle *js, Angle *je, Omega *ws, Alpha *acc_limit, Omega *vel_limit)
{
    double cond_part1 = 0, cond_part2 = 0;
    Omega fabs_start_omega;
    Angle fabs_joint_trip;
    MotionTime duration_min = 99.99;
    MotionTime duration;

    for (size_t i = 0; i < AXIS_IN_ALGORITHM; ++i)
    {
        fabs_start_omega = fabs(ws[i]);
        fabs_joint_trip  = fabs(je[i] - js[i]);
        cond_part1 = pow(fabs_start_omega, 2);
        cond_part2 = 2 * acc_limit[i] * fabs_joint_trip;
        
        if (cond_part1 >= cond_part2 && fabs_joint_trip > MINIMUM_E9)
        {
            duration = (fabs_start_omega - sqrt(cond_part1 - cond_part2)) / acc_limit[i];
        }
        else
        {
            duration = 99.99;
        }
        //FST_INFO("min: part1=%.6f,part2=%.6f,duration=%.6f", cond_part1, cond_part2, duration);
        //FST_INFO("J%d: js=%.6f, je=%.6f, ws=%.6f, a_lim=%.6f, w_lim=%.6f", i + 1,
        //         js[i], je[i], ws[i], acc_limit[i], vel_limit[i]);

        if (duration_min > duration)
        {
            duration_min = duration;
        }  
    }

    return duration_min;
}


MotionTime computeDurationMax(Angle *js, Angle *je, Omega *ws, Alpha *a_top, Alpha *a_bottom, Omega *w_limit)
{
    //FST_INFO("js=%.6f,%.6f,%.6f,%.6f,%.6f,%.6f", js[0],js[1],js[2],js[3],js[4],js[5]);
    //FST_INFO("ws=%.6f,%.6f,%.6f,%.6f,%.6f,%.6f", ws[0],ws[1],ws[2],ws[3],ws[4],ws[5]);
    //FST_INFO("je=%.6f,%.6f,%.6f,%.6f,%.6f,%.6f", je[0],je[1],je[2],je[3],je[4],je[5]);
    //FST_INFO("al=%.6f,%.6f,%.6f,%.6f,%.6f,%.6f", acc_limit[0],acc_limit[1],acc_limit[2],acc_limit[3],acc_limit[4],acc_limit[5]);
    //FST_INFO("wl=%.6f,%.6f,%.6f,%.6f,%.6f,%.6f", vel_limit[0],vel_limit[1],vel_limit[2],vel_limit[3],vel_limit[4],vel_limit[5]);

    double alpha, delta, trip, duration;
    double duration_max = 0;

    for (size_t i = 0; i < AXIS_IN_ALGORITHM; ++i)
    {
        if (ws[i] > w_limit[i] - MINIMUM_E6 && je[i] > js[i] || ws[i] < MINIMUM_E6 -w_limit[i] && je[i] < js[i])
        {
            duration = (je[i] - js[i]) / ws[i];
        }
        else
        {
            trip = je[i] - js[i];
            if (fabs(trip) > MINIMUM_E9)
            {
                alpha = trip > 0 ? a_top[i] : a_bottom[i];
                delta = ws[i] * ws[i] + alpha * trip * 2;
                duration = alpha > 0 ? (sqrt(delta) - ws[i]) / alpha : (-sqrt(delta) - ws[i]) / alpha;

                if (fabs(ws[i] + alpha * duration) > w_limit[i])
                {
                    duration = trip * 2 / (trip > 0 ? ws[i] + w_limit[i] : ws[i] - w_limit[i]);
                }
            }
            else
            {
                duration = 0;
            }
        }

        if(duration_max < duration)
        {
            duration_max = duration;
        }
    }

    //FST_INFO("js=%.6f, je=%.6f, ws=%.6f, alimit=%.6f, wlimit=%.6f, duration_max=%.6f",
    //         js[4], je[4], ws[4], acc_limit[4], vel_limit[4], duration_max);
/*if(is_pause)
    FST_INFO("duration: %f, %f, %f, %f, %f, %f", duration[0], duration[1], duration[2], duration[3], duration[4], duration[5]);*/
    return duration_max;
}

MotionTime computeDurationMin(Angle *js, Angle *je, Omega *ws, Alpha *a_top, Alpha *a_bottom, Omega *w_limit)
{
    double alpha, delta, trip, duration;
    double duration_min = 99.99;

    for (size_t i = 0; i < AXIS_IN_ALGORITHM; ++i)
    {
        trip  = je[i] - js[i];
        if (trip > 0 && ws[i] < 0 || trip < 0 && ws[i] > 0 || fabs(ws[i]) < MINIMUM_E6 || fabs(trip) < MINIMUM_E6)
        {
            duration = 99.99;
        }
        else
        {
            alpha = trip < 0 ? a_top[i] : a_bottom[i];
            delta = ws[i] * ws[i] + alpha * trip * 2;
            if (delta < 0)
            {
                duration = 99.99;
            }
            else
            {
                duration = alpha > 0 ? (-ws[i] - sqrt(delta)) / alpha : (-ws[i] + sqrt(delta)) / alpha;
            }
        }

        if (duration_min > duration)
        {
            duration_min = duration;
        }  
    }

    return duration_min;
}

ErrorCode computeAlphaLimit(const double *joint, const double *omega, double *a_upper, double *a_lower)
{
    double alpha_limit[2][6];
    if (g_dynamics_interface.computeAccMax(joint, omega, alpha_limit))
    {
        a_upper[0] = alpha_limit[0][0] * 0.8;
        a_upper[1] = alpha_limit[0][1] * 0.8;
        a_upper[2] = alpha_limit[1][2] * 0.8;
        a_upper[3] = alpha_limit[0][3] * 0.8;
        a_upper[4] = alpha_limit[0][4] * 0.8;
        a_upper[5] = alpha_limit[0][5] * 0.8;
        a_lower[0] = alpha_limit[1][0] * 0.8;
        a_lower[1] = alpha_limit[1][1] * 0.8;
        a_lower[2] = alpha_limit[0][2] * 0.8;
        a_lower[3] = alpha_limit[1][3] * 0.8;
        a_lower[4] = alpha_limit[1][4] * 0.8;
        a_lower[5] = alpha_limit[1][5] * 0.8;

        //FST_INFO("alpha_upper: %.4f,%.4f,%.4f,%.4f,%.4f,%.4f",
        //        a_upper[0], a_upper[1], a_upper[2], a_upper[3], a_upper[4], a_upper[5]);
        //FST_INFO("alpha_lower: %.4f,%.4f,%.4f,%.4f,%.4f,%.4f",
        //        a_lower[0], a_lower[1], a_lower[2], a_lower[3], a_lower[4], a_lower[5]);

        return SUCCESS;
    }
    else
    {
        FST_ERROR("Fail to compute dynamics.");
        return MOTION_INTERNAL_FAULT;
    }
}


void createForwardTraj(ControlPoint &prev, ControlPoint &next)
{
    MotionTime d = next.duration;
    double alpha;

    for(int i = 0; i < AXIS_IN_ALGORITHM; ++i)
    {
        alpha = 2 * (next.point.joint[i] - prev.point.joint[i] - prev.point.omega[i] * d) / d / d;

        next.point.alpha[i] = alpha;
        next.point.omega[i] = prev.point.omega[i] + alpha * d;
    }
    /*
    FST_INFO("duration=%.6f", d);
    FST_INFO("omega - %.6f, %.6f, %.6f, %.6f, %.6f, %.6f",
             next.point.omega[0], next.point.omega[1], next.point.omega[2], 
             next.point.omega[3], next.point.omega[4], next.point.omega[5]);
    FST_INFO("alpha - %.6f, %.6f, %.6f, %.6f, %.6f, %.6f",
             next.point.alpha[0], next.point.alpha[1], next.point.alpha[2], 
             next.point.alpha[3], next.point.alpha[4], next.point.alpha[5]);
    */
}

void createBackwardTraj(ControlPoint &prev, ControlPoint &next)
{
    MotionTime d = next.duration;
    double alpha;

    for(int i = 0; i < AXIS_IN_ALGORITHM; ++i)
    {
        alpha = 2 * (prev.point.joint[i] - next.point.joint[i] + next.point.omega[i] * d) / d / d;

        next.point.alpha[i] = alpha;
        prev.point.omega[i] = next.point.omega[i] - alpha * d;
    }
    //FST_INFO("duration=%.6f", d);
    //FST_INFO("omega - %.6f, %.6f, %.6f, %.6f, %.6f, %.6f",
    //         prev.point.omega[0], prev.point.omega[1], prev.point.omega[2], 
    //         prev.point.omega[3], prev.point.omega[4], prev.point.omega[5]);
    //FST_INFO("alpha - %.6f, %.6f, %.6f, %.6f, %.6f, %.6f",
    //         next.point.alpha[0], next.point.alpha[1], next.point.alpha[2], 
    //         next.point.alpha[3], next.point.alpha[4], next.point.alpha[5]);
}

ErrorCode forwardTrajectory(ControlPoint &prev, ControlPoint &next,
                            MotionTime expect_duration, Omega *omega_limit)
{
    ErrorCode err = SUCCESS;
    MotionTime dmax, dmin;

    //FST_WARN("forward----------------------------");

    //double *a_upper = g_alpha_limit_upper;
    //double *a_lower = g_alpha_limit_lower;
    static size_t dynamics_update_cnt = 0;
    static double a_upper[6], a_lower[6];
    if (dynamics_update_cnt == 0)
    {
        computeAlphaLimit(prev.point.joint, prev.point.omega, a_upper, a_lower);
        dynamics_update_cnt ++;
        if (dynamics_update_cnt == 10)
        {
            dynamics_update_cnt = 0;
        }
    }
    
    dmin = computeDurationMax(prev.point.joint, next.point.joint, prev.point.omega, a_upper, a_lower, omega_limit);
    dmax = computeDurationMin(prev.point.joint, next.point.joint, prev.point.omega, a_upper, a_lower, omega_limit);

    if (dmax > dmin)
    {
        if (expect_duration < dmin)
        {
            next.duration = dmin;
        }
        else if (expect_duration > dmax)
        {
            next.duration = dmax;
        }
        else
        {
            next.duration = expect_duration;
        }

        //FST_INFO("stamp=%d, ID=%d  duration: min=%.4f, max=%.4f, expect=%.4f, res=%.6f",
        //         next.path_point.stamp, next.path_point.id, dmin, dmax, expect_duration, next.duration);

        createForwardTraj(prev, next);

        /*
        JointState *js = &prev.point;
        JointState *je = &next.point;
        FST_INFO("prev: joint - %.6f,%.6f,%.6f,%.6f,%.6f,%.6f",
                js->joint[0], js->joint[1], js->joint[2], js->joint[3], js->joint[4], js->joint[5]);
        FST_INFO("      omega - %.6f,%.6f,%.6f,%.6f,%.6f,%.6f",
                js->omega[0], js->omega[1], js->omega[2], js->omega[3], js->omega[4], js->omega[5]);
        FST_INFO("      alpha - %.6f,%.6f,%.6f,%.6f,%.6f,%.6f",
                js->alpha[0], js->alpha[1], js->alpha[2], js->alpha[3], js->alpha[4], js->alpha[5]);
        FST_INFO("next: joint - %.6f,%.6f,%.6f,%.6f,%.6f,%.6f",
                je->joint[0], je->joint[1], je->joint[2], je->joint[3], je->joint[4], je->joint[5]);
        FST_INFO("      omega - %.6f,%.6f,%.6f,%.6f,%.6f,%.6f",
                je->omega[0], je->omega[1], je->omega[2], je->omega[3], je->omega[4], je->omega[5]);
        FST_INFO("      alpha - %.6f,%.6f,%.6f,%.6f,%.6f,%.6f",
                je->alpha[0], je->alpha[1], je->alpha[2], je->alpha[3], je->alpha[4], je->alpha[5]);
        */
    }
    else
    {
        err = MOTION_INTERNAL_FAULT;
        FST_ERROR("stamp=%d, fore duration error: dmin=%.6f, dmax=%.6f", next.path_point.stamp, dmin, dmax);

        JointState *js = &prev.point;
        JointState *je = &next.point;
        FST_INFO("prev: joint - %.6f,%.6f,%.6f,%.6f,%.6f,%.6f",
                js->joint[0], js->joint[1], js->joint[2], js->joint[3], js->joint[4], js->joint[5]);
        FST_INFO("      omega - %.6f,%.6f,%.6f,%.6f,%.6f,%.6f",
                js->omega[0], js->omega[1], js->omega[2], js->omega[3], js->omega[4], js->omega[5]);
        FST_INFO("next: joint - %.6f,%.6f,%.6f,%.6f,%.6f,%.6f",
                je->joint[0], je->joint[1], je->joint[2], je->joint[3], je->joint[4], je->joint[5]);
        FST_INFO("omega limit - %.6f,%.6f,%.6f,%.6f,%.6f,%.6f",
                omega_limit[0], omega_limit[1], omega_limit[2], 
                omega_limit[3], omega_limit[4], omega_limit[5]);
        FST_INFO("alpha upper - %.6f,%.6f,%.6f,%.6f,%.6f,%.6f",
                a_upper[0], a_upper[1], a_upper[2], 
                a_upper[3], a_upper[4], a_upper[5]);
        FST_INFO("      lower - %.6f,%.6f,%.6f,%.6f,%.6f,%.6f",
                a_lower[0], a_lower[1], a_lower[2], 
                a_lower[3], a_lower[4], a_lower[5]);
    }

    return err;
}

ErrorCode backwardTrajectory(ControlPoint &prev, ControlPoint &next,
                             MotionTime expect_duration, Omega *omega_limit)
{
    ErrorCode err = SUCCESS;
    MotionTime dmax, dmin;

    //FST_WARN("backward---------------------------");

    //double *a_upper = g_alpha_limit_upper;
    //double *a_lower = g_alpha_limit_lower;

    double omega[6] = {-next.point.omega[0], -next.point.omega[1], -next.point.omega[2], 
                       -next.point.omega[3], -next.point.omega[4], -next.point.omega[5]};

    static size_t dynamics_update_cnt = 0;
    static double a_upper[6], a_lower[6];

    if (dynamics_update_cnt == 0)
    {
        computeAlphaLimit(prev.point.joint, omega, a_upper, a_lower);
        dynamics_update_cnt ++;
        if (dynamics_update_cnt == 10)
        {
            dynamics_update_cnt = 0;
        }
    }

    dmin = computeDurationMax(next.point.joint, prev.point.joint, omega, a_upper, a_lower, omega_limit);
    dmax = computeDurationMin(next.point.joint, prev.point.joint, omega, a_upper, a_lower, omega_limit);

    if (dmax > dmin)
    {
        if (expect_duration < dmin)
        {
            next.duration = dmin;
        }
        else if (expect_duration > dmax)
        {
            next.duration = dmax;
        }
        else
        {
            next.duration = expect_duration;
        }

        //FST_INFO("stamp=%d, ID=%d  duration: min=%.4f, max=%.4f, expect=%.4f, res=%.6f",
        //         next.path_point.stamp, next.path_point.id, dmin, dmax, expect_duration, next.duration);

        //FST_INFO("dmin=%.6f, dmax=%.6f, dexp=%.6f, dres=%.6f", dmin, dmax, next.expect_duration, next.duration);
        createBackwardTraj(prev, next);

        /*
        JointState *js = &prev.point;
        JointState *je = &next.point;
        FST_INFO("prev: joint - %.6f,%.6f,%.6f,%.6f,%.6f,%.6f",
                js->joint[0], js->joint[1], js->joint[2], js->joint[3], js->joint[4], js->joint[5]);
        FST_INFO("      omega - %.6f,%.6f,%.6f,%.6f,%.6f,%.6f",
                js->omega[0], js->omega[1], js->omega[2], js->omega[3], js->omega[4], js->omega[5]);
        FST_INFO("      alpha - %.6f,%.6f,%.6f,%.6f,%.6f,%.6f",
                js->alpha[0], js->alpha[1], js->alpha[2], js->alpha[3], js->alpha[4], js->alpha[5]);
        FST_INFO("next: joint - %.6f,%.6f,%.6f,%.6f,%.6f,%.6f",
                je->joint[0], je->joint[1], je->joint[2], je->joint[3], je->joint[4], je->joint[5]);
        FST_INFO("      omega - %.6f,%.6f,%.6f,%.6f,%.6f,%.6f",
                je->omega[0], je->omega[1], je->omega[2], je->omega[3], je->omega[4], je->omega[5]);
        FST_INFO("      alpha - %.6f,%.6f,%.6f,%.6f,%.6f,%.6f",
                je->alpha[0], je->alpha[1], je->alpha[2], je->alpha[3], je->alpha[4], je->alpha[5]);
        */
    }
    else
    {
        err = MOTION_INTERNAL_FAULT;
        FST_ERROR("stamp=%d, back duration error: dmin=%.6f, dmax=%.6f", next.path_point.stamp, dmin, dmax);
        JointState *js = &prev.point;
        JointState *je = &next.point;
        FST_INFO("prev: joint - %.6f,%.6f,%.6f,%.6f,%.6f,%.6f",
                js->joint[0], js->joint[1], js->joint[2], js->joint[3], js->joint[4], js->joint[5]);
        FST_INFO("      omega - %.6f,%.6f,%.6f,%.6f,%.6f,%.6f",
                js->omega[0], js->omega[1], js->omega[2], js->omega[3], js->omega[4], js->omega[5]);
        FST_INFO("      alpha - %.6f,%.6f,%.6f,%.6f,%.6f,%.6f",
                js->alpha[0], js->alpha[1], js->alpha[2], js->alpha[3], js->alpha[4], js->alpha[5]);
        FST_INFO("next: joint - %.6f,%.6f,%.6f,%.6f,%.6f,%.6f",
                je->joint[0], je->joint[1], je->joint[2], je->joint[3], je->joint[4], je->joint[5]);
        FST_INFO("      omega - %.6f,%.6f,%.6f,%.6f,%.6f,%.6f",
                je->omega[0], je->omega[1], je->omega[2], je->omega[3], je->omega[4], je->omega[5]);
        FST_INFO("      alpha - %.6f,%.6f,%.6f,%.6f,%.6f,%.6f",
                je->alpha[0], je->alpha[1], je->alpha[2], je->alpha[3], je->alpha[4], je->alpha[5]);
        FST_INFO("omega limit - %.6f,%.6f,%.6f,%.6f,%.6f,%.6f",
                omega_limit[0], omega_limit[1], omega_limit[2], 
                omega_limit[3], omega_limit[4], omega_limit[5]);
        FST_INFO("alpha upper - %.6f,%.6f,%.6f,%.6f,%.6f,%.6f",
                a_upper[0], a_upper[1], a_upper[2], 
                a_upper[3], a_upper[4], a_upper[5]);
        FST_INFO("      lower - %.6f,%.6f,%.6f,%.6f,%.6f,%.6f",
                a_lower[0], a_lower[1], a_lower[2], 
                a_lower[3], a_lower[4], a_lower[5]);
    }

    return err;
}



}
