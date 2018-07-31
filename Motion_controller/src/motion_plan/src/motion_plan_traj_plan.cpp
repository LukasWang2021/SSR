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


ErrorCode computeAlphaLimit(const double *joint, const double *omega, double *alpha_upper, double *alpha_lower)
{
    ErrorCode err = SUCCESS;
    double alpha_limit[2][6];

    if (g_dynamics_interface.computeAccMax(joint, omega, alpha_limit))
    {
        alpha_upper[0] = alpha_limit[0][0] > g_alpha_limit[0] * 0.8 ? alpha_limit[0][0] : g_alpha_limit[0] * 0.8;
        alpha_upper[1] = alpha_limit[0][1] > g_alpha_limit[1] * 0.8 ? alpha_limit[0][1] : g_alpha_limit[1] * 0.8;
        alpha_upper[2] = alpha_limit[0][2] > g_alpha_limit[2] * 0.8 ? alpha_limit[0][2] : g_alpha_limit[2] * 0.8;
        alpha_upper[3] = alpha_limit[0][3] > g_alpha_limit[3] * 0.8 ? alpha_limit[0][3] : g_alpha_limit[3] * 0.8;
        alpha_upper[4] = alpha_limit[0][4] > g_alpha_limit[4] * 0.8 ? alpha_limit[0][4] : g_alpha_limit[4] * 0.8;
        alpha_upper[5] = alpha_limit[0][5] > g_alpha_limit[5] * 0.8 ? alpha_limit[0][5] : g_alpha_limit[5] * 0.8;
        alpha_lower[0] = alpha_limit[1][0] < -g_alpha_limit[0] * 0.8 ? alpha_limit[1][0] : -g_alpha_limit[0] * 0.8;
        alpha_lower[1] = alpha_limit[1][1] < -g_alpha_limit[1] * 0.8 ? alpha_limit[1][1] : -g_alpha_limit[1] * 0.8;
        alpha_lower[2] = alpha_limit[1][2] < -g_alpha_limit[2] * 0.8 ? alpha_limit[1][2] : -g_alpha_limit[2] * 0.8;
        alpha_lower[3] = alpha_limit[1][3] < -g_alpha_limit[3] * 0.8 ? alpha_limit[1][3] : -g_alpha_limit[3] * 0.8;
        alpha_lower[4] = alpha_limit[1][4] < -g_alpha_limit[4] * 0.8 ? alpha_limit[1][4] : -g_alpha_limit[4] * 0.8;
        alpha_lower[5] = alpha_limit[1][5] < -g_alpha_limit[5] * 0.8 ? alpha_limit[1][5] : -g_alpha_limit[5] * 0.8;

        alpha_upper[0] *= g_global_acc_ratio;
        alpha_upper[1] *= g_global_acc_ratio;
        alpha_upper[2] *= g_global_acc_ratio;
        alpha_upper[3] *= g_global_acc_ratio;
        alpha_upper[4] *= g_global_acc_ratio;
        alpha_upper[5] *= g_global_acc_ratio;
        alpha_lower[0] *= g_global_acc_ratio;
        alpha_lower[1] *= g_global_acc_ratio;
        alpha_lower[2] *= g_global_acc_ratio;
        alpha_lower[3] *= g_global_acc_ratio;
        alpha_lower[4] *= g_global_acc_ratio;
        alpha_lower[5] *= g_global_acc_ratio;

        /*
        for (size_t i = 0; i < AXIS_IN_ALGORITHM; i++)
        {
            if (alpha_upper[i] < 0 || alpha_lower[i] > 0)
            {
                err = MOTION_INTERNAL_FAULT;
                break;
            }
        }

        if (err != SUCCESS)
        {
            FST_ERROR("ERROR while resolving dynamics:");
            FST_ERROR("input-joint: %.6f, %.6f, %.6f, %.6f, %.6f, %.6f",
                      joint[0], joint[1], joint[2], joint[3], joint[4], joint[5]);
            FST_ERROR("input-omega: %.6f, %.6f, %.6f, %.6f, %.6f, %.6f",
                      omega[0], omega[1], omega[2], omega[3], omega[4], omega[5]);
            FST_ERROR("output-alpha-upper: %.6f, %.6f, %.6f, %.6f, %.6f, %.6f",
                      alpha_upper[0], alpha_upper[1], alpha_upper[2], alpha_upper[3], alpha_upper[4], alpha_upper[5]);
            FST_ERROR("output-alpha-lower: %.6f, %.6f, %.6f, %.6f, %.6f, %.6f",
                      alpha_lower[0], alpha_lower[1], alpha_lower[2], alpha_lower[3], alpha_lower[4], alpha_lower[5]);
        }
         */

        return err;
    }
    else
    {
        FST_ERROR("Fail to compute dynamics.");
        return MOTION_INTERNAL_FAULT;
    }
}


MotionTime computeMinDuration(const Angle *js, const Angle *je, const Omega *ws,
                              const Alpha *a_upper, const Alpha *a_lower, char *fallback)
{
    //FST_LOG("compute d-min --------------------------");

    const double *w_limit = g_omega_limit;
    double  alpha, delta, duration;
    double  trip_max = 0;
    double  trip[AXIS_IN_ALGORITHM];
    double  dmax = 0;

    for (size_t i = 0; i < AXIS_IN_ALGORITHM; ++i)
    {
        trip[i] = je[i] - js[i];

        if (fabs(trip[i]) > trip_max)
        {
            trip_max = fabs(trip[i]);
        }
    }

    for (size_t i = 0; i < AXIS_IN_ALGORITHM; ++i)
    {
        if (ws[i] > w_limit[i] - MINIMUM_E3 && trip[i] > 0 || ws[i] < MINIMUM_E6 - w_limit[i] && trip[i] < 0)
        {
            duration = trip[i] / ws[i];
        }
        else
        {
            if (fabs(trip[i]) < trip_max * 0.05 && fabs(ws[i]) < 0.1)
            {
                fallback[i] = 1;
                duration = 0;
            }
            else
            {
                fallback[i] = 0;
                alpha = trip[i] > 0 ? a_upper[i] : a_lower[i];
                delta = ws[i] * ws[i] + alpha * trip[i] * 2;
                duration = alpha > 0 ? (sqrt(delta) - ws[i]) / alpha : (-sqrt(delta) - ws[i]) / alpha;

                if (fabs(ws[i] + alpha * duration) > w_limit[i])
                {
                    duration = trip[i] * 2 / (trip[i] > 0 ? ws[i] + w_limit[i] : ws[i] - w_limit[i]);
                }
            }
        }

        //FST_LOG("J%d: js=%.6f, je=%.6f, ws=%.6f, alpha=%.6f, delta=%.6f, duration=%.6f", i + 1,
        //         js[i], je[i], ws[i], alpha, delta, duration);

        if(dmax < duration)
        {
            dmax = duration;
        }
    }

    return dmax;
}


MotionTime computeMaxDuration(const Angle *js, const Angle *je, const Omega *ws,
                              const Alpha *a_upper, const Alpha *a_lower, char *fallback)
{
    //FST_LOG("compute d-max --------------------------");

    const double *w_limit = g_omega_limit;
    double alpha, delta, duration;
    double  trip_max = 0;
    double  trip[AXIS_IN_ALGORITHM];
    double dmin = 99.99;

    for (size_t i = 0; i < AXIS_IN_ALGORITHM; ++i)
    {
        trip[i] = je[i] - js[i];

        if (fabs(trip[i]) > trip_max)
        {
            trip_max = fabs(trip[i]);
        }
    }

    for (size_t i = 0; i < AXIS_IN_ALGORITHM; ++i)
    {
        if (trip[i] > 0 && ws[i] < 0 || trip[i] < 0 && ws[i] > 0 || fabs(ws[i]) < 0.1 && fabs(trip[i]) < trip_max * 0.05)
        {
            fallback[i] = 1;
            duration = 99.99;
        }
        else
        {
            fallback[i] = 0;
            alpha = trip < 0 ? a_upper[i] : a_lower[i];
            delta = ws[i] * ws[i] + alpha * trip[i] * 2;
            if (delta < 0)
            {
                duration = 99.99;
            }
            else
            {
                duration = alpha > 0 ? (-ws[i] - sqrt(delta)) / alpha : (-ws[i] + sqrt(delta)) / alpha;
            }
        }

        //FST_LOG("J%d: trip=%.6f, alpha=%.6f, ws2=%.6f, alpha*trip*2=%.6f, delta=%.6f, -ws[i] + sqrt(delta)=%.6f, duration=%.6f",
        //         i + 1, trip, alpha, ws[i] * ws[i], alpha * trip * 2, delta, -ws[i] + sqrt(delta), duration);

        if (dmin > duration)
        {
            dmin = duration;
        }
    }

    return dmin;
}


void createSpeedUpTraj(ControlPoint &prev, ControlPoint &next, const char *fallback)
{
    MotionTime d = next.forward_duration;
    double alpha;

    for (int i = 0; i < AXIS_IN_ALGORITHM; ++i)
    {
        if (fallback[i] == 0)
        {
            alpha = 2 * (next.forward_point.joint[i] - prev.forward_point.joint[i] - prev.forward_point.omega[i] * d) / d / d;

            next.forward_point.alpha[i] = alpha;
            next.forward_point.omega[i] = prev.forward_point.omega[i] + alpha * d;
        }
        else
        {
            next.forward_point.alpha[i] = 0;
            next.forward_point.omega[i] = (next.forward_point.joint[i] - prev.forward_point.joint[i]) / d;
        }
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


ErrorCode createSpeedUpTraj(ControlPoint &prev, ControlPoint &next, MotionTime expect_duration,
                            Alpha *alpha_upper, Alpha *alpha_lower)
{
    ErrorCode err = SUCCESS;
    char fallback[AXIS_IN_ALGORITHM];

    //FST_LOG("forward----------------------------");
    double *js = prev.forward_point.joint;
    double *ws = prev.forward_point.omega;
    double *je = next.forward_point.joint;

    /*
    FST_LOG("js=%.6f,%.6f,%.6f,%.6f,%.6f,%.6f", js[0],js[1],js[2],js[3],js[4],js[5]);
    FST_LOG("ws=%.6f,%.6f,%.6f,%.6f,%.6f,%.6f", ws[0],ws[1],ws[2],ws[3],ws[4],ws[5]);
    FST_LOG("je=%.6f,%.6f,%.6f,%.6f,%.6f,%.6f", je[0],je[1],je[2],je[3],je[4],je[5]);
    FST_LOG("wli=%.6f,%.6f,%.6f,%.6f,%.6f,%.6f", omega_limit[0],omega_limit[1],omega_limit[2],
            omega_limit[3],omega_limit[4],omega_limit[5]);
    FST_LOG("a-top=%.6f,%.6f,%.6f,%.6f,%.6f,%.6f",
            alpha_upper[0], alpha_upper[1], alpha_upper[2], alpha_upper[3], alpha_upper[4], alpha_upper[5]);
    FST_LOG("a-bot=%.6f,%.6f,%.6f,%.6f,%.6f,%.6f",
            alpha_lower[0], alpha_lower[1], alpha_lower[2], alpha_lower[3], alpha_lower[4], alpha_lower[5]);
    */

    double dmin = computeMinDuration(js, je, ws, alpha_upper, alpha_lower, fallback);


    if (expect_duration < dmin)
    {
        next.forward_duration = dmin;
    }
    else
    {
        next.forward_duration = expect_duration;
    }

    //FST_LOG("fore-stamp=%d, ID=%d  duration: min=%.4f, max=%.4f, expect=%.4f, res=%.6f",
    //         next.path_point.stamp, next.path_point.id, dmin, dmax, expect_duration, next.forward_duration);

    createSpeedUpTraj(prev, next, fallback);

    /*
    JointState *js = &prev.forward_point;
    JointState *je = &next.forward_point;
    FST_LOG("prev: joint - %.6f,%.6f,%.6f,%.6f,%.6f,%.6f",
            js->joint[0], js->joint[1], js->joint[2], js->joint[3], js->joint[4], js->joint[5]);
    FST_LOG("      omega - %.6f,%.6f,%.6f,%.6f,%.6f,%.6f",
            js->omega[0], js->omega[1], js->omega[2], js->omega[3], js->omega[4], js->omega[5]);
    FST_LOG("      alpha - %.6f,%.6f,%.6f,%.6f,%.6f,%.6f",
            js->alpha[0], js->alpha[1], js->alpha[2], js->alpha[3], js->alpha[4], js->alpha[5]);
    FST_LOG("next: joint - %.6f,%.6f,%.6f,%.6f,%.6f,%.6f",
            je->joint[0], je->joint[1], je->joint[2], je->joint[3], je->joint[4], je->joint[5]);
    FST_LOG("      omega - %.6f,%.6f,%.6f,%.6f,%.6f,%.6f",
            je->omega[0], je->omega[1], je->omega[2], je->omega[3], je->omega[4], je->omega[5]);
    FST_LOG("      alpha - %.6f,%.6f,%.6f,%.6f,%.6f,%.6f",
            je->alpha[0], je->alpha[1], je->alpha[2], je->alpha[3], je->alpha[4], je->alpha[5]);
    */

    return err;
}

ErrorCode createSpeedUpTraj(ControlPoint &prev, ControlPoint &next, MotionTime expect_duration)
{
    Alpha alpha_upper[AXIS_IN_ALGORITHM];
    Alpha alpha_lower[AXIS_IN_ALGORITHM];

    computeAlphaLimit(prev.forward_point.joint, prev.forward_point.omega, alpha_upper, alpha_lower);
    return createSpeedUpTraj(prev, next, expect_duration, alpha_upper, alpha_lower);
}


void createBackwardSpeedUpTraj(ControlPoint &prev, ControlPoint &next, const char *fallback)
{
    MotionTime d = next.backward_duration;
    double alpha;

    for (int i = 0; i < AXIS_IN_ALGORITHM; ++i)
    {
        alpha = 2 * (prev.backward_point.joint[i] - next.backward_point.joint[i] + next.backward_point.omega[i] * d) / d / d;

        next.backward_point.alpha[i] = alpha;
        prev.backward_point.omega[i] = next.backward_point.omega[i] - alpha * d;
    }
    //FST_INFO("duration=%.6f", d);
    //FST_INFO("omega - %.6f, %.6f, %.6f, %.6f, %.6f, %.6f",
    //         prev.point.omega[0], prev.point.omega[1], prev.point.omega[2],
    //         prev.point.omega[3], prev.point.omega[4], prev.point.omega[5]);
    //FST_INFO("alpha - %.6f, %.6f, %.6f, %.6f, %.6f, %.6f",
    //         next.point.alpha[0], next.point.alpha[1], next.point.alpha[2],
    //         next.point.alpha[3], next.point.alpha[4], next.point.alpha[5]);
}


ErrorCode createBackwardSpeedUpTraj(ControlPoint &prev, ControlPoint &next, MotionTime expect_duration,
                             Alpha *alpha_upper, Alpha *alpha_lower)
{
    ErrorCode err = SUCCESS;
    char fallback[AXIS_IN_ALGORITHM];

    //FST_WARN("backward---------------------------");

    double omega[6] = {-next.backward_point.omega[0], -next.backward_point.omega[1], -next.backward_point.omega[2],
                       -next.backward_point.omega[3], -next.backward_point.omega[4], -next.backward_point.omega[5]};
    double *js = next.backward_point.joint;
    double *je = prev.backward_point.joint;

    double dmin = computeMinDuration(js, je, omega, alpha_upper, alpha_lower, fallback);

    if (expect_duration < dmin)
    {
        next.backward_duration = dmin;
    }
    else
    {
        next.backward_duration = expect_duration;
    }

    //FST_LOG("back-stamp=%d, ID=%d  duration: min=%.4f, max=%.4f, expect=%.4f, res=%.6f",
    //         next.path_point.stamp, next.path_point.id, dmin, dmax, expect_duration, next.backward_duration);

    //FST_INFO("dmin=%.6f, dmax=%.6f, dexp=%.6f, dres=%.6f", dmin, dmax, next.expect_duration, next.duration);
    createBackwardSpeedUpTraj(prev, next, fallback);

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

    return err;
}


ErrorCode uniformTrajectory(ControlPoint &prev, ControlPoint &next, MotionTime expect_duration, TrajSegment &seg)
{
    double  trip[AXIS_IN_ALGORITHM];
    const double *omega_limit = g_omega_limit;
    MotionTime  duration = 0;

    for (size_t i = 0; i < AXIS_IN_ALGORITHM; i++)
    {
        trip[i] = next.forward_point.joint[i] - prev.forward_point.joint[i];

        if (fabs(trip[i]) / omega_limit[i] > duration)
        {
            duration = fabs(trip[i]) / omega_limit[i];
        }
    }

    if (duration < expect_duration)
    {
        duration = expect_duration;
    }

    //FST_INFO("forward-Uniform-Traj: exp-duration=%.6f, duration=%.6f", expect_duration, duration);

    for (size_t i = 0; i < AXIS_IN_ALGORITHM; i++)
    {
        seg.coeff[i][0] = prev.forward_point.joint[i];
        seg.coeff[i][1] = trip[i] / duration;
        seg.coeff[i][2] = 0;
        seg.coeff[i][3] = 0;
        seg.coeff[i][4] = 0;
        seg.coeff[i][5] = 0;
    }

    seg.duration = duration;
    next.forward_duration = duration;

    return SUCCESS;
}






















MotionTime computeDurationMax(Angle *js, Angle *je, Omega *ws, Alpha *a_top, Alpha *a_bottom, Omega *w_limit, char *fallback)
{
    //FST_LOG("compute d-min --------------------------");

    double  alpha, delta, duration;
    double  trip_max = 0;
    double  trip[AXIS_IN_ALGORITHM];
    double  duration_max = 0;

    for (size_t i = 0; i < AXIS_IN_ALGORITHM; ++i)
    {
        trip[i] = je[i] - js[i];
        if (fabs(trip[i]) > trip_max)
            trip_max = fabs(trip[i]);
    }

    for (size_t i = 0; i < AXIS_IN_ALGORITHM; ++i)
    {
        if (ws[i] > w_limit[i] - MINIMUM_E6 && trip[i] > 0 || ws[i] < MINIMUM_E6 -w_limit[i] && trip[i] < 0)
        {
            duration = trip[i] / ws[i];
        }
        else
        {
            if (fabs(trip[i]) < trip_max * 0.05 && fabs(ws[i]) < 0.1)
            {
                fallback[i] = 1;
                duration = 0;
            }
            else
            {
                fallback[i] = 0;
                alpha = trip[i] > 0 ? a_top[i] : a_bottom[i];
                delta = ws[i] * ws[i] + alpha * trip[i] * 2;
                duration = alpha > 0 ? (sqrt(delta) - ws[i]) / alpha : (-sqrt(delta) - ws[i]) / alpha;

                if (fabs(ws[i] + alpha * duration) > w_limit[i])
                {
                    duration = trip[i] * 2 / (trip[i] > 0 ? ws[i] + w_limit[i] : ws[i] - w_limit[i]);
                }
            }
        }
        
        //FST_LOG("J%d: js=%.6f, je=%.6f, ws=%.6f, alpha=%.6f, delta=%.6f, duration=%.6f", i + 1,
        //         js[i], je[i], ws[i], alpha, delta, duration);

        if(duration_max < duration)
        {
            duration_max = duration;
        }
    }

    return duration_max;
}

MotionTime computeDurationMin(Angle *js, Angle *je, Omega *ws, Alpha *a_top, Alpha *a_bottom, Omega *w_limit, char *fallback)
{
    //FST_LOG("compute d-max --------------------------");

    double alpha, delta, trip, duration;
    double duration_min = 99.99;

    for (size_t i = 0; i < AXIS_IN_ALGORITHM; ++i)
    {
        trip  = je[i] - js[i];
        if (trip > 0 && ws[i] < 0 || trip < 0 && ws[i] > 0 || fabs(ws[i]) < 0.1 && fabs(trip) < 0.0001)
        {
            fallback[i] = 1;
            duration = 99.99;
        }
        else
        {
            fallback[i] = 0;
            alpha = trip < 0 ? a_top[i] : a_bottom[i];
            delta = ws[i] * ws[i] + alpha * trip * 2;
            if (delta < 0)
            {
                duration = 99.99;
            }
            else
            {
                //if (fabs(delta / alpha / 2) < 0.005)
                //{
                //    double omg = sqrt(fabs(trip * alpha * 2));
                //    ws[i] = alpha > 0 ? MINIMUM_E6 - omg : omg - MINIMUM_E6;
                //    duration = 99.99;
               // }
                //else
                //{
                    duration = alpha > 0 ? (-ws[i] - sqrt(delta)) / alpha : (-ws[i] + sqrt(delta)) / alpha;
                //}
            }
        }

        //FST_LOG("J%d: trip=%.6f, alpha=%.6f, ws2=%.6f, alpha*trip*2=%.6f, delta=%.6f, -ws[i] + sqrt(delta)=%.6f, duration=%.6f",
        //         i + 1, trip, alpha, ws[i] * ws[i], alpha * trip * 2, delta, -ws[i] + sqrt(delta), duration);

        if (duration_min > duration)
        {
            duration_min = duration;
        }  
    }

    return duration_min;
}

MotionTime computeDurationMaxDebug(Angle *js, Angle *je, Omega *ws, Alpha *a_top, Alpha *a_bottom, Omega *w_limit)
{
    //FST_LOG("compute debug d-min --------------------------");

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

        FST_LOG("J%d: js=%.6f, je=%.6f, ws=%.6f, alpha=%.6f, delta=%.6f, duration=%.6f", i + 1,
                 js[i], je[i], ws[i], alpha, delta, duration);

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

MotionTime computeDurationMinDebug(Angle *js, Angle *je, Omega *ws, Alpha *a_top, Alpha *a_bottom, Omega *w_limit)
{
    //FST_LOG("compute debug d-max --------------------------");
    
    double alpha, delta, trip, duration;
    double duration_min = 99.99;

    for (size_t i = 0; i < AXIS_IN_ALGORITHM; ++i)
    {
        trip  = je[i] - js[i];
        if (trip > 0 && ws[i] < 0 || trip < 0 && ws[i] > 0 || fabs(ws[i]) < MINIMUM_E6 || fabs(trip) < 0.0001)
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

        FST_INFO("J%d: trip=%.6f, alpha=%.6f, ws2=%.6f, alpha*trip*2=%.6f, delta=%.6f, -ws[i] + sqrt(delta)=%.6f, duration=%.6f",
                 i + 1, trip, alpha, ws[i] * ws[i], alpha * trip * 2, delta, -ws[i] + sqrt(delta), duration);

        if (duration_min > duration)
        {
            duration_min = duration;
        }  
    }

    return duration_min;
}


void createForwardTraj(ControlPoint &prev, ControlPoint &next, char *fallback)
{
    MotionTime d = next.forward_duration;
    double alpha;

    for(int i = 0; i < AXIS_IN_ALGORITHM; ++i)
    {
        if (fallback[i] == 0)
        {
            alpha = 2 * (next.forward_point.joint[i] - prev.forward_point.joint[i] - prev.forward_point.omega[i] * d) / d / d;

            next.forward_point.alpha[i] = alpha;
            next.forward_point.omega[i] = prev.forward_point.omega[i] + alpha * d;
        }
        else
        {
            next.forward_point.alpha[i] = 0;
            next.forward_point.omega[i] = (next.forward_point.joint[i] - prev.forward_point.joint[i]) / d;
        }
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

void createBackwardTraj(ControlPoint &prev, ControlPoint &next, char *fallback)
{
    MotionTime d = next.backward_duration;
    double alpha;

    for(int i = 0; i < AXIS_IN_ALGORITHM; ++i)
    {
        alpha = 2 * (prev.backward_point.joint[i] - next.backward_point.joint[i] + next.backward_point.omega[i] * d) / d / d;

        next.backward_point.alpha[i] = alpha;
        prev.backward_point.omega[i] = next.backward_point.omega[i] - alpha * d;
    }
    //FST_INFO("duration=%.6f", d);
    //FST_INFO("omega - %.6f, %.6f, %.6f, %.6f, %.6f, %.6f",
    //         prev.point.omega[0], prev.point.omega[1], prev.point.omega[2], 
    //         prev.point.omega[3], prev.point.omega[4], prev.point.omega[5]);
    //FST_INFO("alpha - %.6f, %.6f, %.6f, %.6f, %.6f, %.6f",
    //         next.point.alpha[0], next.point.alpha[1], next.point.alpha[2], 
    //         next.point.alpha[3], next.point.alpha[4], next.point.alpha[5]);
}

ErrorCode forwardTrajectory(ControlPoint &prev, ControlPoint &next, MotionTime expect_duration,
                            Omega *omega_limit, Alpha *alpha_upper, Alpha *alpha_lower)
{
    ErrorCode err = SUCCESS;
    MotionTime dmax, dmin;
    char fallback[6];

    //FST_LOG("forward----------------------------");
    double *js = prev.forward_point.joint;
    double *ws = prev.forward_point.omega;
    double *je = next.forward_point.joint;

    /*
    FST_LOG("js=%.6f,%.6f,%.6f,%.6f,%.6f,%.6f", js[0],js[1],js[2],js[3],js[4],js[5]);
    FST_LOG("ws=%.6f,%.6f,%.6f,%.6f,%.6f,%.6f", ws[0],ws[1],ws[2],ws[3],ws[4],ws[5]);
    FST_LOG("je=%.6f,%.6f,%.6f,%.6f,%.6f,%.6f", je[0],je[1],je[2],je[3],je[4],je[5]);
    FST_LOG("wli=%.6f,%.6f,%.6f,%.6f,%.6f,%.6f", omega_limit[0],omega_limit[1],omega_limit[2],
            omega_limit[3],omega_limit[4],omega_limit[5]);
    FST_LOG("a-top=%.6f,%.6f,%.6f,%.6f,%.6f,%.6f",
            alpha_upper[0], alpha_upper[1], alpha_upper[2], alpha_upper[3], alpha_upper[4], alpha_upper[5]);
    FST_LOG("a-bot=%.6f,%.6f,%.6f,%.6f,%.6f,%.6f",
            alpha_lower[0], alpha_lower[1], alpha_lower[2], alpha_lower[3], alpha_lower[4], alpha_lower[5]);
    */

    dmin = computeDurationMax(prev.forward_point.joint, next.forward_point.joint, prev.forward_point.omega,
                              alpha_upper, alpha_lower, omega_limit, fallback);
    dmax = computeDurationMin(prev.forward_point.joint, next.forward_point.joint, prev.forward_point.omega,
                              alpha_upper, alpha_lower, omega_limit, fallback);

    //if (dmax * 1.05 > dmin)
    if (true)
    {
        if (expect_duration < dmin)
        {
            next.forward_duration = dmin;
        }
        else if (expect_duration > dmax)
        {
            next.forward_duration = dmax;
        }
        else
        {
            next.forward_duration = expect_duration;
        }

        //FST_LOG("fore-stamp=%d, ID=%d  duration: min=%.4f, max=%.4f, expect=%.4f, res=%.6f",
        //         next.path_point.stamp, next.path_point.id, dmin, dmax, expect_duration, next.forward_duration);

        createForwardTraj(prev, next, fallback);
        
        /*
        JointState *js = &prev.forward_point;
        JointState *je = &next.forward_point;
        FST_LOG("prev: joint - %.6f,%.6f,%.6f,%.6f,%.6f,%.6f",
                js->joint[0], js->joint[1], js->joint[2], js->joint[3], js->joint[4], js->joint[5]);
        FST_LOG("      omega - %.6f,%.6f,%.6f,%.6f,%.6f,%.6f",
                js->omega[0], js->omega[1], js->omega[2], js->omega[3], js->omega[4], js->omega[5]);
        FST_LOG("      alpha - %.6f,%.6f,%.6f,%.6f,%.6f,%.6f",
                js->alpha[0], js->alpha[1], js->alpha[2], js->alpha[3], js->alpha[4], js->alpha[5]);
        FST_LOG("next: joint - %.6f,%.6f,%.6f,%.6f,%.6f,%.6f",
                je->joint[0], je->joint[1], je->joint[2], je->joint[3], je->joint[4], je->joint[5]);
        FST_LOG("      omega - %.6f,%.6f,%.6f,%.6f,%.6f,%.6f",
                je->omega[0], je->omega[1], je->omega[2], je->omega[3], je->omega[4], je->omega[5]);
        FST_LOG("      alpha - %.6f,%.6f,%.6f,%.6f,%.6f,%.6f",
                je->alpha[0], je->alpha[1], je->alpha[2], je->alpha[3], je->alpha[4], je->alpha[5]);
        */
    }
    /*
    else
    {
        err = MOTION_INTERNAL_FAULT;
        FST_ERROR("stamp=%d, fore duration error: dmin=%.6f, dmax=%.6f", next.path_point.stamp, dmin, dmax);


        computeDurationMaxDebug(prev.forward_point.joint, next.forward_point.joint, prev.forward_point.omega, alpha_upper, alpha_lower, omega_limit);
        computeDurationMinDebug(prev.forward_point.joint, next.forward_point.joint, prev.forward_point.omega, alpha_upper, alpha_lower, omega_limit);

        JointState *js = &prev.forward_point;
        JointState *je = &next.forward_point;
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
                alpha_upper[0], alpha_upper[1], alpha_upper[2], 
                alpha_upper[3], alpha_upper[4], alpha_upper[5]);
        FST_INFO("      lower - %.6f,%.6f,%.6f,%.6f,%.6f,%.6f",
                alpha_lower[0], alpha_lower[1], alpha_lower[2], 
                alpha_lower[3], alpha_lower[4], alpha_lower[5]);

    }
    */

    return err;
}

ErrorCode backwardTrajectory(ControlPoint &prev, ControlPoint &next, MotionTime expect_duration,
                             Omega *omega_limit, Alpha *alpha_upper, Alpha *alpha_lower)
{
    ErrorCode err = SUCCESS;
    MotionTime dmax, dmin;
    char fallback[6];

    //FST_WARN("backward---------------------------");

    double omega[6] = {-next.backward_point.omega[0], -next.backward_point.omega[1], -next.backward_point.omega[2], 
                       -next.backward_point.omega[3], -next.backward_point.omega[4], -next.backward_point.omega[5]};

    dmin = computeDurationMax(next.backward_point.joint, prev.backward_point.joint,
                              omega, alpha_upper, alpha_lower, omega_limit, fallback);
    dmax = computeDurationMin(next.backward_point.joint, prev.backward_point.joint,
                              omega, alpha_upper, alpha_lower, omega_limit, fallback);

    //if (dmax * 1.05 > dmin)
    if (true)
    {
        if (expect_duration < dmin)
        {
            next.backward_duration = dmin;
        }
        else if (expect_duration > dmax)
        {
            next.backward_duration = dmax;
        }
        else
        {
            next.backward_duration = expect_duration;
        }

        //FST_LOG("back-stamp=%d, ID=%d  duration: min=%.4f, max=%.4f, expect=%.4f, res=%.6f",
        //         next.path_point.stamp, next.path_point.id, dmin, dmax, expect_duration, next.backward_duration);

        //FST_INFO("dmin=%.6f, dmax=%.6f, dexp=%.6f, dres=%.6f", dmin, dmax, next.expect_duration, next.duration);
        createBackwardTraj(prev, next, fallback);

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
    /*
    else
    {
        err = MOTION_INTERNAL_FAULT;
        FST_ERROR("stamp=%d, back duration error: dmin=%.6f, dmax=%.6f", next.path_point.stamp, dmin, dmax);
        JointState *js = &prev.backward_point;
        JointState *je = &next.backward_point;
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
                alpha_upper[0], alpha_upper[1], alpha_upper[2], 
                alpha_upper[3], alpha_upper[4], alpha_upper[5]);
        FST_INFO("      lower - %.6f,%.6f,%.6f,%.6f,%.6f,%.6f",
                alpha_lower[0], alpha_lower[1], alpha_lower[2], 
                alpha_lower[3], alpha_lower[4], alpha_lower[5]);
    }
     */

    return err;
}

ErrorCode forwardTrajectory(ControlPoint &prev, ControlPoint &next, MotionTime expect_duration,
                            Omega *omega_limit, TrajSegment &seg)
{
    Alpha alpha_upper[AXIS_IN_ALGORITHM];
    Alpha alpha_lower[AXIS_IN_ALGORITHM];

    computeAlphaLimit(prev.forward_point.joint, prev.forward_point.omega, alpha_upper, alpha_lower);
    return forwardTrajectory(prev, next, expect_duration, omega_limit, alpha_upper, alpha_lower);
}

ErrorCode forwardUniformTrajectory(ControlPoint &prev, ControlPoint &next, MotionTime expect_duration,
                                   Omega *omega_limit, TrajSegment &seg)
{
    double  trip[AXIS_IN_ALGORITHM];
    MotionTime duration = prev.forward_duration;

    for (size_t i = 0; i < AXIS_IN_ALGORITHM; i++)
    {
        trip[i] = next.forward_point.joint[i] - prev.forward_point.joint[i];
        if (fabs(trip[i]) / omega_limit[i] > duration)
        {
            duration = fabs(trip[i]) / omega_limit[i];
        }
    }

    //FST_INFO("forward-Uniform-Traj: exp-duration=%.6f, duration=%.6f", expect_duration, duration);

    for (size_t i = 0; i < AXIS_IN_ALGORITHM; i++)
    {
        seg.coeff[i][0] = prev.forward_point.joint[i];
        seg.coeff[i][1] = trip[i] / duration;
        seg.coeff[i][2] = 0;
        seg.coeff[i][3] = 0;
        seg.coeff[i][4] = 0;
        seg.coeff[i][5] = 0;
    }

    seg.duration = duration;

    return SUCCESS;
}

/*
ErrorCode forwardUniformTrajectory(ControlPoint &prev, ControlPoint &next, MotionTime expect_duration,
                                   Omega *omega_limit, TrajSegment &seg)
{
    //FST_INFO("forward-Uniform-Traj: exp-duration=%.6f", expect_duration);
    MotionTime duration;
    size_t  ind = 0;
    Omega   omg = prev.forward_point.omega[0];

    for (size_t i = 1; i < AXIS_IN_ALGORITHM; i++)
    {
        if (fabs(prev.forward_point.omega[i]) > fabs(omg))
        {
            ind = i;
            omg = prev.forward_point.omega[i];
        }
    }

    duration = (next.forward_point.joint[ind] - prev.forward_point.joint[ind]) / prev.forward_point.omega[ind];

    //FST_INFO("jnt_start=%.6f, jnt_end=%.6f, omega_start=%.6f, duration=%.6f, index=%d", prev.forward_point.joint[ind],
    //         next.forward_point.joint[ind], prev.forward_point.omega[ind], duration, ind);

    if (fabs(duration - expect_duration) < expect_duration * 0.05)
    {
        seg.duration = duration;

        seg.coeff[ind][0] = prev.forward_point.joint[ind];
        seg.coeff[ind][1] = prev.forward_point.omega[ind];
        seg.coeff[ind][2] = 0;
        seg.coeff[ind][3] = 0;
        seg.coeff[ind][4] = 0;
        seg.coeff[ind][5] = 0;

        double omge, trip;
        for (size_t i = 0; i < AXIS_IN_ALGORITHM; i++)
        {
            if (i != ind)
            {
                trip = next.forward_point.joint[i] - prev.forward_point.joint[i];
                omge = trip / expect_duration;
                seg.coeff[i][0] = prev.forward_point.joint[i];
                seg.coeff[i][1] = prev.forward_point.omega[i];
                seg.coeff[i][2] = (trip * 3 - (prev.forward_point.omega[i] * 2 + omge) * duration) / duration / duration;
                seg.coeff[i][3] = ((omge + prev.forward_point.omega[i]) * duration - trip * 2) / (duration * duration * duration);
                seg.coeff[i][4] = 0;
                seg.coeff[i][5] = 0;
            }
        }
    }
    else
    {
        Alpha alpha_upper[AXIS_IN_ALGORITHM];
        Alpha alpha_lower[AXIS_IN_ALGORITHM];
        computeAlphaLimit(prev.forward_point.joint, prev.forward_point.omega, alpha_upper, alpha_lower);

        forwardTrajectory(prev, next, expect_duration, omega_limit, alpha_upper, alpha_lower);

        seg.duration = next.forward_duration;

        for (size_t i = 0; i < AXIS_IN_ALGORITHM; i++)
        {
            seg.coeff[i][0] = prev.forward_point.joint[i];
            seg.coeff[i][1] = prev.forward_point.omega[i];
            seg.coeff[i][2] = next.forward_point.alpha[i] / 2;
            seg.coeff[i][3] = 0;
            seg.coeff[i][4] = 0;
            seg.coeff[i][5] = 0;
        }
    }

    return SUCCESS;
}
 */


}
