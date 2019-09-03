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

    //--------read nposiont.txt, push to a vector------------//
    vector<double> position_vector;
    vector<double> velocity_vector;
    vector<double> tau_vector;
    ifstream position_file("nposition.txt");
    ifstream velocity_file("nvelocity.txt");
    ifstream tau_file("taucalcu.txt");
    ofstream outfile("output_direct_acc.txt");
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
    //read tau.txt
    for (int i = 0; i < count; ++i)
    {
        getline(tau_file, line);
        num_str = ""; 
        for (int j = 0; j < line.length(); ++j)
        {
            if (line[j] == ',' || line[j] == ' ' || j == (line.length() - 1))
            {
                if (j == (line.length() - 1))
                {
                    num_str += line[j];
                }
                tau_vector.push_back(atof(num_str.c_str()));
                num_str = "";    
            }
            else
            {
                num_str += line[j];
            }
        }
    }
    printf("tau size=%d\n", tau_vector.size());

  //print position, velocity, tau values
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
 /*   for (int k = 0; k < tau_vector.size(); ++k)
    {
        printf("%f,", tau_vector[k]);
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
/*
    for (int i = 0; i < count; ++i)
    {
        for(int j = 0; j < 6; ++j)
        {
            joint[j] = position_vector[i*6 + j];
            vel[j] = velocity_vector[i*6 +j];
            torque[j] = tau_vector[i*6 + j];
        }
        err = dyn.getAccDirectDynamics(joint, vel, torque, acc);
        if (err == false)
        {
            printf("failed direct dynamics\n");
            return 0;
        }
        
        outfile<<acc.a1_<<","<< acc.a2_<<","<< acc.a3_<<","<< acc.a4_<<","<< acc.a5_<<","<< acc.a6_<<endl;
        
    }
*/
    //under max torque
    for (int i = 0; i < count; ++i)
    {
        for(int j = 0; j < 6; ++j)
        {
            joint[j] = position_vector[i*6 + j];
            vel[j] = velocity_vector[i*6 +j];
            //torque[j] = tau_vector[i*6 + j];
        }
        err = dyn.getAccMax(joint, vel,acc_pos, acc_neg);
        if (err == false)
        {
            printf("failed direct dynamics\n");
            return 0;
        }
        
        outfile<<acc_pos.a1_<<","<<acc_neg.a1_<<","
               << acc_pos.a2_<<","<<acc_neg.a2_<<","
               << acc_pos.a3_<<","<<acc_neg.a3_<<","
               << acc_pos.a4_<<","<<acc_neg.a4_<<","
               << acc_pos.a5_<<","<<acc_neg.a5_<<","
               << acc_pos.a6_<<","<<acc_neg.a6_<<endl;
        
    }
    
    gettimeofday(&t_end, NULL);
    cost_time = (t_end.tv_sec - t_start.tv_sec) * 1000000 + (t_end.tv_usec - t_start.tv_usec);
    printf("direct dynamics total_time = %d us, average_time = %d us\n", cost_time, cost_time/count);

    printf("end\n");
    return 0;
}

