/*************************************************************************
	> File Name: manual_test.cpp
	> Author: 
	> Mail: 
	> Created Time: 2018年08月08日 星期三 17时27分48秒
 ************************************************************************/

#include <unistd.h>
#include <iostream>
#include <motion_control_arm_group.h>

using namespace std;
using namespace fst_mc;
using namespace fst_log;


int main(int argc, char **argv)
{
    size_t loop = 0;
    Logger log;
    log.initLogger("test");
    ArmGroup arm(&log);
    cout << "begin" << endl;

    arm.initGroup();
    cout << "reset group" << endl;
    arm.resetGroup();
    sleep(1);
    arm.setManualFrame(JOINT);
    /*
    ManualDirection dir[9] = {STANDING};
    dir[1] = DECREASE;
    arm.setManualMode(CONTINUOUS);
    //arm.setManualMode(STEP);
    arm.manualMove(dir);

    while (loop++ < 20)
    {
        arm.sendPoint();
        usleep(5 * 1000);
    }

    dir[0] = DECREASE;
    arm.manualMove(dir);
    while (loop++ < 30)
    {
        arm.sendPoint();
        usleep(5 * 1000);
    }

    dir[0] = STANDING;
    dir[1] = STANDING;
    arm.manualMove(dir);
    while (loop++ < 200)
    {
        arm.sendPoint();
        usleep(5 * 1000);
    }

    sleep(1);
    */

    arm.setManualMode(APOINT);
    Joint target;
    target.j1 = 0;
    target.j2 = 0;
    target.j3 = 0;
    target.j4 = 0;
    target.j5 = 0;
    target.j6 = 0;
    arm.manualMove(target);
    while (loop++ < 2000)
    {
        arm.sendPoint();
        usleep(5 * 1000);
    }

    sleep(1);
    arm.stopGroup();
    


    return 0;
}

