/*************************************************************************
	> File Name: motion_plan_demo_manual.cpp
	> Author: 
	> Mail: 
	> Created Time: 2018年05月09日 星期三 09时05分59秒
 ************************************************************************/

#include <iostream>
#include <time.h>
#include <string.h>
#include <fstream>

#include <parameter_manager/parameter_manager_param_group.h>

#include <motion_plan_arm_group.h>
#include <motion_plan_matrix.h>
#include <motion_plan_kinematics.h>
#include <motion_plan_variable.h>
#include <motion_plan_motion_command.h>
#include <motion_plan_traj_plan.h>
#include <motion_plan_reuse.h>
#include <motion_plan_manual_teach.h>

using namespace std;
using namespace fst_controller;
using namespace fst_algorithm;


int main(int argc, char **argv)
{
    cout << "hello ... " << endl;
    cout << sizeof(ArmGroup) << endl;
    cout << sizeof(ControlPointCache) * MOTION_POOL_CAPACITY << endl;
    cout << sizeof(ControlPoint) << endl;
    ArmGroup arm;
    Joint start_joint;
    ErrorCode err = arm.initArmGroup();
    cout << "init " << endl;

    start_joint.j1 = 0;
    start_joint.j2 = 0;
    start_joint.j3 = 0;
    start_joint.j4 = 0;
    start_joint.j5 = -1.5708;
    start_joint.j6 = 0;
    arm.setStartState(start_joint);

    vector<JointOutput> points;
    vector<ManualDirection> dir;
    
    dir.resize(6);
    dir[0] = INCREASE;
    dir[1] = STANDBY;
    dir[2] = STANDBY;
    dir[3] = STANDBY;
    dir[4] = STANDBY;
    dir[5] = STANDBY;

    cout << "Manual Joint Step----------------------------" << endl;
    arm.setManualMode(STEP);
    arm.setManualFrame(JOINT);
    arm.manualMove(dir);

    arm.getPointFromFIFO(250, points);

    cout << "Manual Joint Continuous----------------------------" << endl;
    arm.setManualMode(CONTINUOUS);
    arm.setManualFrame(JOINT);
    arm.manualMove(dir);

    arm.getPointFromFIFO(50, points);

    dir[1] = DECREASE;
    arm.manualMove(dir);

    arm.getPointFromFIFO(50, points);

    dir[0] = STANDBY;
    arm.manualMove(dir);

    arm.getPointFromFIFO(50, points);

    dir[1] = STANDBY;
    arm.manualMove(dir);

    arm.getPointFromFIFO(200, points);

    cout << "Manual Joint Apoint----------------------------" << endl;
    Joint target;
    target.j1 = 0;
    target.j2 = 0;
    target.j3 = 0;
    target.j4 = 0;
    target.j5 = -1.5708;
    target.j6 = 0;

    arm.setManualMode(APOINT);
    arm.setManualFrame(JOINT);
    arm.manualMove(target);
    arm.getPointFromFIFO(350, points);

    cout << "Manual Cartesian Step----------------------------" << endl;
    dir[0] = INCREASE;
    dir[1] = STANDBY;
    dir[2] = STANDBY;
    dir[3] = STANDBY;
    dir[4] = STANDBY;
    dir[5] = STANDBY;

    arm.setManualMode(STEP);
    arm.setManualFrame(USER);
    arm.manualMove(dir);
    arm.getPointFromFIFO(350, points);

    dir[0] = DECREASE;
    dir[1] = STANDBY;
    dir[2] = INCREASE;
    dir[3] = INCREASE;
    dir[4] = DECREASE;
    dir[5] = STANDBY;

    arm.manualMove(dir);
    arm.getPointFromFIFO(350, points);

    arm.setManualMode(CONTINUOUS);
    arm.setManualFrame(USER);
    dir[0] = INCREASE;
    dir[1] = STANDBY;
    dir[2] = STANDBY;
    dir[3] = STANDBY;
    dir[4] = STANDBY;
    dir[5] = STANDBY;
    arm.manualMove(dir);
    arm.getPointFromFIFO(50, points);
    dir[0] = STANDBY;
    arm.manualMove(dir);
    arm.getPointFromFIFO(350, points);

    dir[0] = INCREASE;
    arm.manualMove(dir);
    arm.getPointFromFIFO(500, points);

    return 0;

}
