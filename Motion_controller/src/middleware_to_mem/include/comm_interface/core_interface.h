/**********************************************
Copyright © 2016 Foresight-Robotics Ltd. All rights reserved.
File:       core_interface.h
Author:     Feng.Wu 
Create:     04-Nov-2016
Modify:     08-Dec-2016
Summary:    lib to communicate with core1
**********************************************/

#ifndef MIDDLEWARE_TO_MEM_CORE_INTERFACE_H_
#define MIDDLEWARE_TO_MEM_CORE_INTERFACE_H_

#include <vector>
#include "error_code/error_code.h"
#include "middleware_to_mem/middleware_to_sharedmem.h"
#include "struct_to_mem/struct_joint_command.h"
#include "struct_to_mem/struct_trajectory_segment.h"
#include "struct_to_mem/struct_feedback_joint_states.h"

namespace fst_core_interface
{

//------------------------------------------------------------
// The interface classs for motion controller to send and receive
// data from BARE CORE.
// Sample usage:
//   CoreInterface comm;
//   ERROR_CODE_TYPE result = comm.init();
//   JointCommand jc = do something;
//   ERROR_CODE_TYPE send = comm.sendBareCore(jc);
//   FeedbackJointState fbjs;
//   ERROR_CODE_TYPE rec = comm.recvBareCore(fbjs);
//------------------------------------------------------------

class CoreInterface
{
public:
    //------------------------------------------------------------
    // Function:  CoreInterface
    // Summary: The constructor of class
    // In:      None
    // Out:     None
    // Return:  None 
    //------------------------------------------------------------
    CoreInterface();

    //------------------------------------------------------------
    // Function:  CoreInterface
    // Summary: The constructor of class
    // In:      None
    // Out:     None
    // Return:  None 
    //------------------------------------------------------------
    ~CoreInterface();

    //------------------------------------------------------------
    // Function:  init
    // Summary: Initialize the shared memory of cores
    // In:      None
    // Out:     None
    // Return:  0 -> succeed to initialize the shared memory.
    //          OPEN_CORE_MEM_FAIL -> failed 
    //------------------------------------------------------------
    ERROR_CODE_TYPE init(void);

    //------------------------------------------------------------
    // Function:  initTrajectory
    // Summary: Initialize the trajectory variable. 
    // In:      None
    // Out:     None
    // Return:  None
    //------------------------------------------------------------
    void initTrajectory(void);

    //------------------------------------------------------------
    // Function:  sendBareCore
    // Summary: Add the time stamp for each points. 
    //          Send the trajectory segments to CORE1.
    // In:      jc -> the desired values of all joints
    // Out:     None
    // Return:  0 -> success to send the trajectory to CORE1.
    //          WRITE_CORE_MEM_FAIL -> failed.
    //------------------------------------------------------------
    ERROR_CODE_TYPE sendBareCore(JointCommand jc);

    //------------------------------------------------------------
    // Function:  recvBareCore
    // Summary: Read the actual joint states from CORE1
    // In:      None
    // Out:     fbjs -> the actual joint states.
    // Return:  0 -> success to read the actual joint states.
    //          READ_CORE_MEM_FAIL -> failed. 
    //------------------------------------------------------------
    ERROR_CODE_TYPE recvBareCore(FeedbackJointState &fbjs);

    //------------------------------------------------------------
    // Function:  sendBareCoreFake
    // Summary: Send the trajectory segments to CORE1. (for fake test) 
    // In:      jc -> the desired values of all joints
    // Out:     None
    // Return:  0 -> success to send the trajectory to CORE1.
    //          WRITE_CORE_MEM_FAIL -> failed.
    //------------------------------------------------------------
    ERROR_CODE_TYPE sendBareCoreFake(JointCommand jc); // for fake test

    //------------------------------------------------------------
    // Function:  recvBareCoreFake
    // Summary: Read the actual joint states from CORE1 (for fake test)
    // In:      None
    // Out:     fbjs -> the actual joint states.
    // Return:  0 -> success to read the actural joint states.
    //------------------------------------------------------------
    ERROR_CODE_TYPE recvBareCoreFake(FeedbackJointState &fbjs); // for fake test

    //------------------------------------------------------------
    // Function:  setTimeStamp
    // Summary: set the time stamp. default is 0.001s 
    // In:      interval -> the unit is second.
    // Out:     None
    // Return:  None
    //------------------------------------------------------------
    void setTimeInterval(double interval);

    static const unsigned int SEC_TO_NSEC = 1000000000; //converting from second to microsecond
    
private:
    // The handle for sharedmem handling.
    int handle_core_;

    // Record the error status.
    ERROR_CODE_TYPE error_flag_;

    // The trajectory structure being to the sharedmem.
    TrajectorySegment ts_;

    // For adding timestamp.
    unsigned int sec_;
    unsigned int nsec_;
    unsigned int prev_sec_;
    unsigned int prev_nsec_;
    unsigned int time_step_;

    // for fake test
    std::vector<Points> joints_in_fifo_;
    FeedbackJointState fbjs_;
};
} //namespace fst_core_interface

#endif //MIDDLEWARE_TO_MEM_CORE_INTERFACE_H_

