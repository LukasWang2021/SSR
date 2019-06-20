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

int localSign(double value)
{
    if (value >= 0)
    {
        return 1;
    }
    else if (value < 0)
    {
        return -1;
    }
}

int main(int argc, char** argv)
{
    printf("begin\n");

    //--------------init dynamics----------------------//
    DynamicAlgRTM dyn;
    bool ret = dyn.initDynamicAlg("/root/install/share/runtime/axis_group/");
    printf("valid = %d\n", dyn.isValid());
    if(ret == false)
    {
        return -1;
    }
    //dyn.updateLoadParam(load_param);
    bool err = false;
    Joint joint;
    JointVelocity vel;
    JointAcceleration acc;
    JointAcceleration acc_pos;
    JointAcceleration acc_neg;
    JointTorque torque;
    JointTorque torque_max_pos;
    JointTorque torque_max_neg;

    //--------read nposiont.txt, push to a vector------------//
    vector<double> position_vector;
    vector<double> velocity_vector;
    vector<double> accelerate_vector;
    vector<double> tau_vector;
    ifstream position_file("nposition.txt");
    ifstream velocity_file("nvelocity.txt");
    ifstream accelerate_file("naccelerate.txt");

    ofstream outfile("output_acc_scale.txt");
    ofstream outfile2("output_torq_scale.txt");
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
            if (line[j] == ',' || line[j] == ' ' || j == (line.length() - 1)) 
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

  //print position, velocity, acc values
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
        }
        //err = dyn.getAccDirectDynamics(joint, vel, torque, acc);
        //dyn.getTorqueFromCurve(vel, torque);
        
        //outfile<<(30*vel.v1_*81/3.1415926)<<","<< (30*vel.v2_*101/3.1415926)<<","<< (30*vel.v3_*81/3.1415926)<<","<< (30*vel.v4_*60/3.1415926)<<","<< (30*vel.v5_*66.67/3.1415926)<<","<< (30*vel.v6_*44.64/3.1415926)<<","
        //       <<torque.t1_/81<<","<<torque.t2_/101<<","<<torque.t3_/81<<","<<torque.t4_/60<<","<<torque.t5_/66.67<<","<<torque.t6_/44.64<<","<<endl;
        
    }
/*
    //under max torque
    double scale[6] = {0}; 
    for (int i = 0; i < count; ++i)
    {
        for(int j = 0; j < 6; ++j)
        {
            joint[j] = position_vector[i*6 + j];
            vel[j] = velocity_vector[i*6 +j];
            acc[j] = accelerate_vector[i*6 + j];
        }
        err = dyn.getAccMax(joint, vel, acc_pos, acc_neg);
        if (err == false)
        {
            printf("failed direct dynamics\n");
            return 0;
        }

        for (int k = 0; k < 6; ++k)
        {
            if ( localSign(acc[k]) == localSign(acc_pos[k]))
            {
                scale[k] = acc_pos[k] / acc[k];
            }
            else if(localSign(acc[k]) == localSign(acc_neg[k]))
            {
                scale[k] = acc_neg[k] / acc[k];
            }
            else
            {
                scale[k] = -1;
            }
        }

        outfile<<acc[0]<<","<<acc[1]<<","<<acc[2]<<","<<acc[3]<<","<<acc[4]<<","<<acc[5]<<","
               <<acc_pos.a1_<<","<<acc_pos.a2_<<","<<acc_pos.a3_<<","<<acc_pos.a4_<<","<<acc_pos.a5_<<","<<acc_pos.a6_<<","
               <<acc_neg.a1_<<","<<acc_neg.a2_<<","<<acc_neg.a3_<<","<<acc_neg.a4_<<","<<acc_neg.a5_<<","<<acc_neg.a6_<<","
               <<scale[0]<<","<<scale[1]<<","<<scale[2]<<","<<scale[3]<<","<<scale[4]<<","<<scale[5]<<","<<endl;
        
    }
    

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
            printf("failed getTorqueInverseDynamics dynamics\n");
            return 0;
        }
        err = dyn.getTorqueMax(joint, vel, torque_max_pos, torque_max_neg);
        if (err == false)
        {
            printf("failed getTorqueMax dynamics\n");
            return 0;
        }

        for (int k = 0; k < 6; ++k)
        {
            if ( localSign(torque[k]) == localSign(torque_max_pos[k]))
            {
                scale[k] = torque_max_pos[k] / torque[k];
            }
            else if(localSign(torque[k]) == localSign(torque_max_neg[k]))
            {
                scale[k] = torque_max_neg[k] / torque[k];
            }
            else
            {
                scale[k] = -1;
            }
        }


        outfile2<<torque.t1_<<","<<torque.t2_<<","<<torque.t3_<<","<<torque.t4_<<","<<torque.t5_<<","<<torque.t6_<<","
               <<torque_max_pos.t1_<<","<<torque_max_pos.t2_<<","<<torque_max_pos.t3_<<","<<torque_max_pos.t4_<<","<<torque_max_pos.t5_<<","<<torque_max_pos.t6_<<","
               <<torque_max_neg.t1_<<","<<torque_max_neg.t2_<<","<<torque_max_neg.t3_<<","<<torque_max_neg.t4_<<","<<torque_max_neg.t5_<<","<<torque_max_neg.t6_<<","
               <<scale[0]<<","<<scale[1]<<","<<scale[2]<<","<<scale[3]<<","<<scale[4]<<","<<scale[5]<<","<<endl;
        
    }
*/

    gettimeofday(&t_end, NULL);
    cost_time = (t_end.tv_sec - t_start.tv_sec) * 1000000 + (t_end.tv_usec - t_start.tv_usec);
    printf("direct dynamics total_time = %d us, average_time = %d us\n", cost_time, cost_time/count);

    printf("end\n");
    return 0;
}

