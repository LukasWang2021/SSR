#ifndef TRAJECTORY_ALG_H
#define TRAJECTORY_ALG_H

#include "base_datatype.h"
#include "error_code.h"

#define MAXAXES 6
#define minim 1e-12


ErrorCode forwardCycle(const fst_mc::JointPoint &start, const fst_mc::Joint &target, double exp_duration,
                       const fst_mc::Joint &alpha_upper, const fst_mc::Joint &alpha_lower,const fst_mc::Joint &jerk,
                       fst_mc::TrajSegment (&segment)[NUM_OF_JOINT]);
ErrorCode backwardCycle(const fst_mc::Joint &start, const fst_mc::JointPoint &target, double exp_duration,
                        const fst_mc::Joint &alpha_upper, const fst_mc::Joint &alpha_lower, const fst_mc::Joint &jerk,
                        fst_mc::TrajSegment (&segment)[NUM_OF_JOINT]);
ErrorCode computeDynamics(const fst_mc::Joint &angle, const fst_mc::Joint &omega,
                          fst_mc::Joint &alpha_upper, fst_mc::Joint &alpha_lower, fst_mc::DynamicsProduct &product);

ErrorCode smoothPoint2Point(const fst_mc::JointPoint &start, const fst_mc::JointPoint &target, double exp_duration,
                            const fst_mc::Joint &alpha_upper, const fst_mc::Joint &alpha_lower, const fst_mc::Joint &jerk,
                            fst_mc::TrajSegment (&segment)[NUM_OF_JOINT]);

ErrorCode forwardCycleTest(const fst_mc::JointPoint &start, const fst_mc::Joint &target, double exp_duration,
                           const fst_mc::Joint &alpha_upper, const fst_mc::Joint &alpha_lower,const fst_mc::Joint &jerk,
                           fst_mc::TrajSegment (&segment)[NUM_OF_JOINT])
{
    for (size_t i = 0; i < NUM_OF_JOINT; i++)
    {
        segment[i].duration[0] = exp_duration / 4;
        segment[i].duration[1] = exp_duration / 4;
        segment[i].duration[2] = exp_duration / 4;
        segment[i].duration[3] = exp_duration / 4;

        double v = (target[i] - start.angle[i]) / exp_duration;
        segment[i].coeff[0][3] = 0;
        segment[i].coeff[0][2] = 0;
        segment[i].coeff[0][1] = v;
        segment[i].coeff[0][0] = start.angle[i];
        segment[i].coeff[1][3] = 0;
        segment[i].coeff[1][2] = 0;
        segment[i].coeff[1][1] = v;
        segment[i].coeff[1][0] = start.angle[i] + v * segment[i].duration[0];
        segment[i].coeff[2][3] = 0;
        segment[i].coeff[2][2] = 0;
        segment[i].coeff[2][1] = v;
        segment[i].coeff[2][0] = start.angle[i] + v * (segment[i].duration[0] + segment[i].duration[1]);
        segment[i].coeff[3][3] = 0;
        segment[i].coeff[3][2] = 0;
        segment[i].coeff[3][1] = v;
        segment[i].coeff[3][0] = start.angle[i] + v * (segment[i].duration[0] + segment[i].duration[1] + segment[i].duration[2]);
    }

    return SUCCESS;
}

ErrorCode backwardCycleTest(const fst_mc::Joint &start, const fst_mc::JointPoint &target, double exp_duration,
                            const fst_mc::Joint &alpha_upper, const fst_mc::Joint &alpha_lower, const fst_mc::Joint &jerk,
                            fst_mc::TrajSegment (&segment)[NUM_OF_JOINT])
{
    for (size_t i = 0; i < NUM_OF_JOINT; i++)
    {
        segment[i].duration[0] = exp_duration / 4;
        segment[i].duration[1] = exp_duration / 4;
        segment[i].duration[2] = exp_duration / 4;
        segment[i].duration[3] = exp_duration / 4;

        double v = (target.angle[i] - start[i]) / exp_duration;
        segment[i].coeff[0][3] = 0;
        segment[i].coeff[0][2] = 0;
        segment[i].coeff[0][1] = v;
        segment[i].coeff[0][0] = start[i];
        segment[i].coeff[1][3] = 0;
        segment[i].coeff[1][2] = 0;
        segment[i].coeff[1][1] = v;
        segment[i].coeff[1][0] = start[i] + v * segment[i].duration[0];
        segment[i].coeff[2][3] = 0;
        segment[i].coeff[2][2] = 0;
        segment[i].coeff[2][1] = v;
        segment[i].coeff[2][0] = start[i] + v * (segment[i].duration[0] + segment[i].duration[1]);
        segment[i].coeff[3][3] = 0;
        segment[i].coeff[3][2] = 0;
        segment[i].coeff[3][1] = v;
        segment[i].coeff[3][0] = start[i] + v * (segment[i].duration[0] + segment[i].duration[1] + segment[i].duration[2]);

    }

    return SUCCESS;
}

#endif


