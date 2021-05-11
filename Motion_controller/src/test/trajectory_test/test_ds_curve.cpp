#include <string>
#include <iostream>
#include <fstream>
#include <math.h>

#include "ds_planner/ds_planner.h"
#include "ds_planner/ds_planner_single_jerk.h"
#include "ds_planner/ds_planner_two_jerk.h"
#include "ds_planner/ds_planner_three_jerk.h"

bool test_ds_curve(int jerk_num, double vel_ratio)
{
   //  int jerk_num = 2;
    DSCurvePlanner *ds_curve;
    char* curve_file_name;
    switch (jerk_num)
    {
        case 1:
            ds_curve = new SingleJerkDSCurvePlanner();
            curve_file_name = "single_jerk_curve.csv";
        break;
        case 2: 
            ds_curve = new TwoJerkDSCurvePlanner();
            curve_file_name = "two_jerk_curve.csv";
        break; 
        case 3: 
            ds_curve = new ThreeJerkDSCurvePlanner();
            curve_file_name = "three_jerk_curve.csv";
        break; 
        default:
            return false;
    }

    double q0 = 0, q1 = 1;
    double jmax[3];
    jmax[0] = 0.49;
    jmax[1] = 0.7;
    jmax[2] = 0.9;

    jmax[0] = 1666.666667;
    jmax[1] = 500.000000;
    jmax[2] = 500.000000;

    double vmax = 0.5, amax = 0.49;
    vmax = 13.333333;
    amax = 83.333333;

    ds_curve->planDSCurve(q0, q1, vmax, amax, jmax, vel_ratio);

    ds_curve->outputDSCurve(0.001, curve_file_name);
}

#if 0
max_jerk[0] = 1666.666667
max_jerk[1] = 500.000000
max_jerk[2] = 500.000000
max_vel = 13.333333, max_acc = 83.333333, vel_ratio = 1.000000
#endif

int main(int argc, char* argv[])
{
    int jerk_num = atoi(argv[1]);
    double vel_ratio = atof(argv[2]);
    test_ds_curve(jerk_num, vel_ratio);
    return 0;
}