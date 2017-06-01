/**********************************************************************
  Copyright:   Foresight-Robotics
  File:        servo_diag.h
  Author:      Yan He
  Data:        Dec.1  2016
  Modify:      Dec.23 2016
  Description: ServoDiag--Class declaration.
**********************************************************************/
#include <servo_conf.h>
#include <servo_service.h>
#include "comm_interface/comm_interface.h"
#include <data_monitor.h>

#ifndef SERVO_DIAG_H
#define SERVO_DIAG_H

namespace fst_controller {
// Brief class for controller. This class include many default settings and functions to make life easier.
class ServoDiag {
  // -----------------------------public functions---------------------------------------------

  public:
    //------------------------------------------------------------
    // Function:    ServoDiag
    // Summary: The constructor of class
    // In:      None
    // Out:     None
    // Return:  None
    //------------------------------------------------------------
    ServoDiag();


    //------------------------------------------------------------
    // Function:    ~ServoDiag
    // Summary: The destructor of class
    // In:      None
    // Out:     None
    // Return:  None
    //------------------------------------------------------------
    ~ServoDiag();


    //------------------------------------------------------------
    // Function:    InitComm
    // Summary: Init communication for command with PC
    // In:      ip address, port
    // Out:     None
    // Return:  Error code
    //------------------------------------------------------------
    static ERROR_CODE_TYPE InitComm(const char *ip_address, int port);


    //------------------------------------------------------------
    // Function:    ServoDiag_Thread
    // Summary: communication thread for command with PC
    // In:      Servconf *servconf,DataMonitor *monitor,ServoService* service
    // Out:     None
    // Return:  None
    //------------------------------------------------------------
    static void ServoDiag_Thread(Servconf *servconf,
                         DataMonitor *monitor,
                         ServoService* service);
    //------------------------------------------------------------
    // Function:    Sig_handler
    // Summary: linux system signal handler
    // In:      system signal
    // Out:     None
    // Return:  None
    //------------------------------------------------------------
    static void Sig_handler( int sig);

    // -----------------------------private functions---------------------------------------------
  public:
    static int exit_flag_;
  private:

    // -----------------------------member variables---------------------------------------------

    static fst_comm_interface::CommInterface comm_;
};  // class ServoDiag
}   // namespace fst_controller


#endif  // #ifndef SERVO_CF_COMM_H
