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

int calculateParam(double q0[MAXAXES], double dq0[MAXAXES], double ddq0[MAXAXES], double qf[MAXAXES], double maxdq[MAXAXES], 
                    double maxddq[MAXAXES], double Jm[MAXAXES], double Ti[MAXAXES], int type, double Ta[MAXAXES][4], 
                    double coeff[MAXAXES][4][4], double *maxT, double tqf[MAXAXES], double dqf[MAXAXES], double ddqf[MAXAXES]);

#endif


