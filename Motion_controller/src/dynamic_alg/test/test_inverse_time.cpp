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
    //----------prepare param-----------//
/*    DynamicAlgParam dynamics_alg_param_ptr[LINKS];
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
*/
    //--------------init dynamics----------------------//
    DynamicAlgRTM dyn;
    dyn.initDynamicAlg("/root/install/share/runtime/axis_group/");
    printf("valid = %d\n", dyn.isValid());
    //dyn.updateLoadParam(load_param);
    bool err = false;
    Joint joint;
    JointVelocity vel;
    JointAcceleration acc;
    JointTorque torque;

    //--------read nposiont.txt, push to a vector------------//
    vector<double> position_vector;
    vector<double> velocity_vector;
    vector<double> accelerate_vector;
    ifstream position_file("nposition.txt");
    ifstream velocity_file("nvelocity.txt");
    ifstream accelerate_file("naccelerate.txt");
    ofstream outfile("output_inverse_tau.txt");
    string line;
    string num_str;

    // make loop count to be argv
    int count = 10;
    if (argc == 2)
    {
        count = atoi(argv[1]);
        if (count == 0)
            count = 10;
        printf("cout = %d\n", count);
    }
    //read nposition.txt
    for (int i = 0; i < count; ++i)
    {
        getline(position_file, line);
        num_str = ""; 
        for (int j = 0; j < line.length(); ++j)
        {
            if (line[j] == ',' || line[j] == ' ' || j == (line.length() - 1)) // can not read the final char "e-8"
            {
                if (j == (line.length() - 1))
                {
                    num_str += line[j];
                }
                position_vector.push_back(atof(num_str.c_str()));
                num_str = "";    
            }
            else
            {
                num_str += line[j];
            }
        }
    }
    printf("position size=%d\n", position_vector.size());
    //read nvelocity.txt
    for (int i = 0; i < count; ++i)
    {
        getline(velocity_file, line);
        num_str = ""; 
        for (int j = 0; j < line.length(); ++j)
        {
            if (line[j] == ',' || line[j] == ' ' || j == (line.length() - 1))
            {
                if (j == (line.length() - 1))
                {
                    num_str += line[j];
                }
                velocity_vector.push_back(atof(num_str.c_str()));
                num_str = "";    
            }
            else
            {
                num_str += line[j];
            }
        }
    }
    printf("velocity size=%d\n", velocity_vector.size());
    //read naccelerate.txt
    for (int i = 0; i < count; ++i)
    {
        getline(accelerate_file, line);
        num_str = ""; 
        for (int j = 0; j < line.length(); ++j)
        {
            if (line[j] == ',' || line[j] == ' ' || j == (line.length() - 1))
            {
                if (j == (line.length() - 1))
                {
                    num_str += line[j];
                }
                accelerate_vector.push_back(atof(num_str.c_str()));
                num_str = "";    
            }
            else
            {
                num_str += line[j];
            }
        }
    }
    printf("accelerate size=%d\n", accelerate_vector.size());
/*
    //print position, velocity, accelerate values
    for (int k = 0; k < position_vector.size(); ++k)
    {
        printf("%f, ", position_vector[k]);
        if ((k+1) % 6 == 0)
        {
            printf("\n");
        }
    }

    for (int k = 0; k < velocity_vector.size(); ++k)
    {
        printf("%f, ", velocity_vector[k]);
        if ((k+1) % 6 == 0)
        {
            printf("\n");
        }
    }
    for (int k = 0; k < accelerate_vector.size(); ++k)
    {
        printf("%f, ", accelerate_vector[k]);
        if ((k+1) % 6 == 0)
        {
            printf("\n");
        }
    }
*/  
    //compute time
    struct timeval t_start, t_end;
    long cost_time = 0;
    gettimeofday(&t_start, NULL);

    for (int i = 0; i < count; ++i)
    {
        for(int j = 0; j < 6; ++j)
        {
            joint[j] = position_vector[i*6 + j];
            vel[j] = velocity_vector[i*6 +j];
            acc[j] = accelerate_vector[i*6 + j];
        }
        err = dyn.getTorqueInverseDynamics(joint, vel, acc, torque);
        if (err == false)
        {
            printf("failed inverse dynamics\n");
            return 0;
        }
        /*
        if ( i == (count - 1))
        {
            printf("joint=%f, %f, %f, %f, %f, %f,\n",joint[0],joint[1],joint[2],joint[3],joint[4],joint[5]);
            printf("vel=%f, %f, %f, %f, %f, %f,\n",vel[0],vel[1],vel[2],vel[3],vel[4],vel[5]);
            printf("acc=%f, %f, %f, %f, %f, %f,\n",acc[0],acc[1],acc[2],acc[3],acc[4],acc[5]);
            printf("torque=%f, %f, %f, %f, %f, %f,\n",torque[0],torque[1],torque[2],torque[3],torque[4],torque[5]);
        }
*/
        outfile<<torque.t1_<<","<< torque.t2_<<","<< torque.t3_<<","<< torque.t4_<<","<< torque.t5_<<","<< torque.t6_<<endl;
        
    }
    
    gettimeofday(&t_end, NULL);
    cost_time = (t_end.tv_sec - t_start.tv_sec) * 1000000 + (t_end.tv_usec - t_start.tv_usec);
    printf("total_time = %d us, average_time = %d us\n", cost_time, cost_time/count);

    printf("end\n");
    return 0;
}

