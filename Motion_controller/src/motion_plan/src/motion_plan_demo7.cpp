/*************************************************************************
	> File Name: motion_plan_demo1.cpp
	> Author: 
	> Mail: 
	> Created Time: 2018年01月09日 星期二 10时15分23秒
 ************************************************************************/

#include <iostream>
#include <string.h>
#include <cstring>
#include <fstream>
#include <motion_plan_frame_manager.h>
#include <motion_plan_reuse.h>
#include <motion_plan_variable.h>
#include <math.h>
#include <dynamics_interface.h>
#include <time.h>

using namespace std;
using namespace fst_controller;
using namespace fst_algorithm;

extern Matrix   fst_algorithm::g_user_frame;
extern Matrix   fst_algorithm::g_tool_frame;
extern Matrix   fst_algorithm::g_user_frame_inverse;
extern Matrix   fst_algorithm::g_tool_frame_inverse;

int main(int argc, char **argv)
{
    //std::cout<<"----------1"<<std::endl;
    clock_t start, finish;
    double Total_time;
    /* 测量一个事件持续的时间*/
    g_user_frame.identityMatrix();
    g_tool_frame.identityMatrix();
    g_user_frame_inverse.identityMatrix();
    g_tool_frame_inverse.identityMatrix();

    DynamicsInterface di;
    //jnt.j1 = 0.1;
    //jnt.j2 = -0.2;
    //jnt.j3 = 0.25;
    //jnt.j4 = 0.17;
    //jnt.j5 = -1.5708;
    //jnt.j6 = -0.38;
    double q[6]={0,0,0,0,-PI/2,0};
    double T[4][4];
    double s[8][6];
    int ns;
    double dq[6]={0,0,0,0,0,0};
    double ddq[6]={0,0,0,0,0,0};
    double maxddq[2][6];  
    //double it[6];
    double tau[6];
   
    //di.rne_tau(q,dq,ddq,tau);
    //double C[6][6][6];
    //double M[6][6];
    //double G[6];
    //start = clock();
    //di.getforwardkinematics(q,T);
 
    //di.getinversekinematics(T,s,ns);
    //di.getMiddleArray(q);
    //di.getM(4,2,q,M[0][0]);
    //FST_INFO("m11=%f\n",M[0][0]);
    di.computeAccMax(q,dq,maxddq);
    //di.getT(q,dq,ddq,tau);
    //for (int i=0;i<6;i++)
    //{
    //   FST_INFO("%f \n",tau[i]);
    //}
    //di.getInertia(q,it);
    //di.getCounterTorque(ddq,tau);
    //finish = clock();
    //Total_time = (double)(finish-start) / CLOCKS_PER_SEC;
    //FST_INFO( "%f seconds/n", Total_time);
#if 0
    FrameManager user_frame_manager("user_frame", MAX_USER_FRAME_NUM, "share/configuration/configurable/user_frame.yaml",
                                    g_user_frame, g_user_frame_inverse);
    FrameManager tool_frame_manager("tool_frame", MAX_TOOL_FRAME_NUM, "share/configuration/configurable/tool_frame.yaml",
                                    g_tool_frame, g_tool_frame_inverse);
    if(!user_frame_manager.isReady())
    {
        FST_ERROR("User frame parameters are invalid.");
        return -1;
    }
#if 1
    FST_INFO("--------------test 1-----------------");
    FST_INFO("initial activated frame: id = %d", user_frame_manager.getActivatedFrame());
    FST_INFO("--------------test 2-----------------");
    if(user_frame_manager.activateFrame(1))
    {
        FST_INFO("activate frame 1: success, expect failed");
    }
    else
    {
        FST_INFO("activate frame 1: failed, expect failed");
        FST_INFO("current activated frame: id = %d", user_frame_manager.getActivatedFrame());
    }
#endif    
#if 0    
    FST_INFO("--------------test 3-----------------");
    Frame new_frame;
    char* comment = "test frame";
    new_frame.id = 1;
    strcpy(new_frame.comment, comment);
    new_frame.data.position.x = 1;
    new_frame.data.position.y = 2;
    new_frame.data.position.z = 3;
    new_frame.data.orientation.a = 0.1;
    new_frame.data.orientation.b = 0.2;
    new_frame.data.orientation.c = 0.3;
    if(user_frame_manager.addFrame(new_frame))
    {
        FST_INFO("add new frame %d success", new_frame.id);
    }
    else
    {
        FST_INFO("add new frame %d failed", new_frame.id);
    }
#endif
#if 0
    FST_INFO("--------------test 4-----------------");
    Frame new_frame;
    char* comment = "test frame";
    new_frame.id = 1;
    strcpy(new_frame.comment, comment);
    new_frame.data.position.x = 1;
    new_frame.data.position.y = 2;
    new_frame.data.position.z = 5;
    new_frame.data.orientation.a = 0.1;
    new_frame.data.orientation.b = 0.2;
    new_frame.data.orientation.c = 0.3;
    if(user_frame_manager.updateFrame(new_frame))
    {
        FST_INFO("update frame %d success", new_frame.id);
    }
    else
    {
        FST_INFO("update frame %d failed", new_frame.id);
    }
#endif
    g_user_frame.printMatrix();
    g_user_frame_inverse.printMatrix();
#endif    
    return 0;
}

