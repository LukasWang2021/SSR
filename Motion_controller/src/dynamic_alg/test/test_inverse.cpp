#include <stdlib.h>
#include <iostream>
#include <string.h>
#include <fstream>
#include <vector>
#include "dynamic_alg_rtm.h"
#include <sys/time.h>

using namespace std;
using namespace basic_alg;
#define LINKS 6

int main(int argc, char** argv)
{
    printf("begin\n");
    DynamicAlgParam dynamics_alg_param_ptr[LINKS];
    dynamics_alg_param_ptr[0].ZZR = 4.4422;
    dynamics_alg_param_ptr[0].FS = 18.098;
    dynamics_alg_param_ptr[0].FV = 20.904;

    dynamics_alg_param_ptr[1].XXR = 2.8681;
    dynamics_alg_param_ptr[1].XY = -1.8825;
    dynamics_alg_param_ptr[1].XZR = -2.766;
    dynamics_alg_param_ptr[1].YZ = -0.30968;
    dynamics_alg_param_ptr[1].ZZR = 6.8069;
    dynamics_alg_param_ptr[1].MXR = -0.48049;
    dynamics_alg_param_ptr[1].MY = -5.3694;
    dynamics_alg_param_ptr[1].FS = 9.8725;
    dynamics_alg_param_ptr[1].FV = 48.124;

    dynamics_alg_param_ptr[2].XXR = 0.16869;
    dynamics_alg_param_ptr[2].XYR = -0.64756;
    dynamics_alg_param_ptr[2].XZ = -1.2098;
    dynamics_alg_param_ptr[2].YZ = -1.0284;
    dynamics_alg_param_ptr[2].ZZR = 1.8109; 
    dynamics_alg_param_ptr[2].MXR = -1.5038;
    dynamics_alg_param_ptr[2].MYR = -1.0835; 
    dynamics_alg_param_ptr[2].Im = 0.11902; 
    dynamics_alg_param_ptr[2].FS = 6.2768; 
    dynamics_alg_param_ptr[2].FV = 18.144;

    dynamics_alg_param_ptr[3].XXR = 0.75468; 
    dynamics_alg_param_ptr[3].XY = 0.77181; 
    dynamics_alg_param_ptr[3].XZ = -0.096175; 
    dynamics_alg_param_ptr[3].YZ = -1.0139; 
    dynamics_alg_param_ptr[3].ZZR = -1.5587; 
    dynamics_alg_param_ptr[3].MX = -0.049204; 
    dynamics_alg_param_ptr[3].MYR = -0.59913; 
    dynamics_alg_param_ptr[3].Im = 1.0685; 
    dynamics_alg_param_ptr[3].FS = 1.6369; 
    dynamics_alg_param_ptr[3].FV = 7.2242; 

    dynamics_alg_param_ptr[4].XXR = 2.011; 
    dynamics_alg_param_ptr[4].XY = 0.16856; 
    dynamics_alg_param_ptr[4].XZ = -0.16502;
    dynamics_alg_param_ptr[4].YZ = 0.65657; 
    dynamics_alg_param_ptr[4].ZZR = 1.892; 
    dynamics_alg_param_ptr[4].MX = 0.74715; 
    dynamics_alg_param_ptr[4].MYR = 0.7456; 
    dynamics_alg_param_ptr[4].Im = 1.3378; 
    dynamics_alg_param_ptr[4].FS = 3.0259; 
    dynamics_alg_param_ptr[4].FV = 10.963;

    dynamics_alg_param_ptr[5].XXR = -0.3696; 
    dynamics_alg_param_ptr[5].XY = -0.26786; 
    dynamics_alg_param_ptr[5].XZ = -0.28879; 
    dynamics_alg_param_ptr[5].YZ = -0.26229; 
    dynamics_alg_param_ptr[5].ZZ = 0.61266; 
    dynamics_alg_param_ptr[5].MX = 0.16604; 
    dynamics_alg_param_ptr[5].MY = -0.0917; 
    dynamics_alg_param_ptr[5].Im = -0.33699; 
    dynamics_alg_param_ptr[5].FS = 3.4088; 
    dynamics_alg_param_ptr[5].FV = 3.8218;

    DynamicAlgLoadParam load_param;
    memset(&load_param, 0 , sizeof(load_param));


    DynamicAlgRTM dyn;
    dyn.initDynamicAlgRTM("/root/install/share/runtime/axis_group/", dynamics_alg_param_ptr, LINKS);
    printf("valid = %d\n", dyn.isValid());
    dyn.updateLoadParam(load_param);
    unsigned long long err = 0;
    Joint joint;
    JointVelocity vel;
    JointAcceleration acc;
    JointTorque torque;

    joint.j1_ = -1.4828;
    joint.j2_ = -1.1403;
    joint.j3_ = 0.117;
    joint.j4_ = 2.0737;
    joint.j5_ = -0.58463;
    joint.j6_ = -0.27589;
    vel.v1_ = 1.1871;
    vel.v2_ = 0.55625;
    vel.v3_ = 0.15057;
    vel.v4_ = -0.0052203;
    vel.v5_ = -1.064;
    vel.v6_ = -0.14875;
    acc.a1_ = 3.2125;
    acc.a2_ = 0.72595;
    acc.a3_ = 0.61929;
    acc.a4_ = 0.12971;
    acc.a5_ = -1.2982;
    acc.a6_ = 0.49317;

    //compute time
    struct timeval t_start, t_end;
    long cost_time = 0;
    gettimeofday(&t_start, NULL);

    err = dyn.getTorqueInverseDynamics(joint, vel, acc, torque);
    if (err != 0)
    {
        printf("failed inverse dynamics\n");
        return 0;
    }

    gettimeofday(&t_end, NULL);
    cost_time = (t_end.tv_sec - t_start.tv_sec) * 1000000 + (t_end.tv_usec - t_start.tv_usec);
    printf("1: time = %d us\n", cost_time);

    printf("%f\n%f\n%f\n%f\n%f\n%f\n", torque.t1_, torque.t2_, torque.t3_, torque.t4_, torque.t5_, torque.t6_);


    joint.j1_ = -2.4462;
    joint.j2_ = -1.3379;
    joint.j3_ = 0.47139;
    joint.j4_ = 2.2053;
    joint.j5_ = 0.37932;
    joint.j6_ = -1.3666;
    vel.v1_ = 5.07E-08;
    vel.v2_ = 3.52E-09;
    vel.v3_ = -3.99E-08;
    vel.v4_ = 5.98E-09;
    vel.v5_ = -2.08E-08;
    vel.v6_ = 1.25E-07;
    acc.a1_ = -6.86E-05;
    acc.a2_ = -4.74E-06;
    acc.a3_ = 5.40E-05;
    acc.a4_ = -8.10E-06;
    acc.a5_ = 2.80E-05;
    acc.a6_ = -0.00016941;
    
    //compute time
    gettimeofday(&t_start, NULL);

    err = dyn.getTorqueInverseDynamics(joint, vel, acc, torque);
    if (err != 0)
    {
        printf("failed inverse dynamics\n");
        return 0;
    }

    gettimeofday(&t_end, NULL);
    cost_time = (t_end.tv_sec - t_start.tv_sec) * 1000000 + (t_end.tv_usec - t_start.tv_usec);
    printf("2: time = %d us\n", cost_time);

    printf("%f\n%f\n%f\n%f\n%f\n%f\n", torque.t1_, torque.t2_, torque.t3_, torque.t4_, torque.t5_, torque.t6_);


    printf("end\n");
    return 0;
}

