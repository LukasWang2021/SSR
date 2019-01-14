#ifndef TRAJECTORY_ALG_H
#define TRAJECTORY_ALG_H

#include "base_datatype.h"
#include "error_code.h"


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

void setJerk(const fst_mc::Joint &jerk);
void setMass(const fst_mc::Joint &mass);
void setTorque(const fst_mc::Joint &torque);
void setGearRatio(const fst_mc::Joint &ratio);
void setOmegaLimit(const fst_mc::Joint &omega);
void setAngleLimit(const fst_mc::Joint &lower_limit, const fst_mc::Joint &upper_limit);

#endif


