/**********************************************************************
    Copyright:	Foresight-Robotics
    File:   	fst_datatype.h
    Author:	    Dong Yi
    Data:	    2017.03.17
    Modify:	    2017.03.17
    Description:Define datatypes used in Trajectary Plan
    ******************************************************************/
#include "fst_datatype.h"

#ifndef FST_TRAJDATATYPE_H
#define FST_TRAJDATATYPE_H

namespace fst_controller
{
    /*Define Initial conditions of L command */
    struct LInitial
    {
        Pose    pose;
        double  velocity;
        double  vu;
        Pose    pose_prv;
    };

    /*Define Objective or preread L command*/
    struct LCommand
    {
        Pose    pose;
        double  velocity;
        int     cnt;
        double  smooth_radius;
    };

    /*Define Initial conditions of C command */
    struct CInitial
    {
        Pose    pose;
        double  velocity;
    };

    /*Define Objective or preread C command*/
    struct CCommand
    {
        Pose    pose1;
        Pose    pose2;
        double  velocity;
        int     cnt;
        double  smooth_radius;
    };

    /*Define Objective or preread J command*/
    struct JCommand
    {
        JointValues joints;
        double      v_percentage;
        int         cnt;
    };

    struct Arc
    {
        double ArcRot[4][4];
        double radius;
        double omega;
    };

    struct orienttrans
    {
        double q0[4];
        double q1[4];
        double s1[4];
        double s2[4];
        int modeflag;
        double theta_q;
        double theta_s;
    };

    struct ElementSmooth
    {
        double time[8];
        double Polynominal[7][4];
    };


    struct Parabola
    {
        double P0[3];
        double v0[3];
        double a0[3];
        double P2[3];
        double v2[3];
        double a2[3];
        double T;
        double lamda[3];
    };


}

#endif