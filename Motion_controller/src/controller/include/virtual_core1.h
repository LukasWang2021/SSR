#ifndef VIRTUAL_CORE1_H
#define VIRTUAL_CORE1_H


#include <thread>
#include "base_datatype.h"
#include "common_log.h"
#include "thread_help.h"


namespace fst_ctrl
{
class VirtualCore1
{
public:
    VirtualCore1();
    ~VirtualCore1();

    void init(fst_log::Logger* log);

    /*
    This represents for if motor is running
    SERVO_READY = 1,
    SERVO_RUNNING = 2,
    */
    int getServoStatus();
    /*
    This represents for if core0 fifo is empty
    ARM_READY = 1,
    ARM_RUNNING = 2,
    */
    int getArmStatus();

    int getSafetyAlarm();

    void doEstop();
    void doReset();

    void threadFunc();
private:
    fst_log::Logger* log_;
    bool is_estop;
    bool is_reset;
    int servo_status_;
    int arm_status_;

    std::thread* thread_ptr_;
    fst_mc::Joint joint_cmd_;
    fst_mc::Joint joint_feedback_;
    fst_base::ThreadHelp thread_;
};

}

void virtualCore1ThreadFunc(void* arg);


#endif

