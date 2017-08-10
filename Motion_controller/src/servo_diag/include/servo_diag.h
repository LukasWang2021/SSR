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
enum PC_DiagnoseID
{ 
  PC_STARTRECORD            = 0x01,  
  PC_STOPRECORD             = 0x02,
  PC_READINT                = 0x03,
  PC_SETTRIGGER             = 0x04,
  PC_READSERVODTC           = 0x07,
  PC_READSERVOPARA          = 0x11,
  PC_WRITESERVOPARA         = 0x12,
  PC_SERVOCMD               = 0x20,
};

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
    // Function:    initComm
    // Summary: Init communication for command with PC
    // In:      ip address, port
    // Out:     None
    // Return:  Error code
    //------------------------------------------------------------
    static ERROR_CODE_TYPE initComm(const char *ip_address, int port);


    //------------------------------------------------------------
    // Function:    servoDiagThread
    // Summary: communication thread for command with PC
    // In:      Servconf *servconf,DataMonitor *monitor,ServoService* service
    // Out:     None
    // Return:  None
    //------------------------------------------------------------
    static void servoDiagThread(Servconf *servconf,
                         DataMonitor *monitor,
                         ServoService* service);
    //------------------------------------------------------------
    // Function:    sigHandler
    // Summary: linux system signal handler
    // In:      system signal
    // Out:     None
    // Return:  None
    //------------------------------------------------------------
    static void sigHandler( int sig);

    // -----------------------------private functions---------------------------------------------
  public:
    static const int SERVODIAG_PORT = 5558;
    static const int DATAMONITOR_PORT = 5559;
    static int exit_flag_;
  private:

    // -----------------------------member variables---------------------------------------------

    static fst_comm_interface::CommInterface comm_;
};  // class ServoDiag
}   // namespace fst_controller


#endif  // #ifndef SERVO_CF_COMM_H
