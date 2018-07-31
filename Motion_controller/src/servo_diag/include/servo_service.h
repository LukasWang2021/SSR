/**********************************************************************
  Copyright:   Foresight-Robotics
  File:        servo_service.h
  Author:      Yan He
  Data:        Dec.1  2016
  Modify:      Dec.23 2016
  Description: ServcfComm--Class declaration.
**********************************************************************/
#include "comm_interface/comm_interface.h"
#include "struct_to_mem/struct_service_request.h"
#include "struct_to_mem/struct_service_response.h"
#include <mutex>
#include <vector>
#include "error_code/error_code.h"

#ifndef SERVO_SERVICE_H
#define SERVO_SERVICE_H


namespace fst_controller {
// Brief class for controller. This class include many default settings and functions to make life easier.

enum ServiceID
{ 
  READ_SERVO_DATA_BY_ADDR            =0x14,  
  WRITE_SERVO_DATA_BY_ADDR         = 0x24,
  READ_SERVO_DTC_SID                = 0x31,
  LOG_CONTROL_SID                   =0x40,
  LOG_GETLIST_SID                   =0x41,
  SERVO_CMD_SID                     =0x60
};


class ServoService {
  // -----------------------------public functions---------------------------------------------

  public:
    //------------------------------------------------------------
    // Function:    ServoService
    // Summary: The constructor of class
    // In:      
    // Out:     None
    // Return:  None
    //------------------------------------------------------------
    ServoService();


    //------------------------------------------------------------
    // Function:    ~ServoService
    // Summary: The destructor of class
    // In:      None
    // Out:     None
    // Return:  None
    //------------------------------------------------------------
    ~ServoService();

    //------------------------------------------------------------
    // Function:    startLog
    // Summary: Start the data log for Core1
    // In:      size of varable list, varable name list
    // Out:      type list
    // Return:  ERROR Code
    //------------------------------------------------------------

    ERROR_CODE_TYPE  startLog(int size_of_varlist,const char *varlist,std::vector<int>& t_list);
    
    //------------------------------------------------------------
    // Function:    stopLog
    // Summary: Stop the data log for Core1
    // In:      None
    // Out:      None
    // Return:  ERROR Code
    //------------------------------------------------------------
    ERROR_CODE_TYPE  stopLog(void);
    
    
    //------------------------------------------------------------
    // Function:    downloadParam
    // Summary: Download Servo Configuration Param
    // In:      Address, data, length
    // Out:      None
    // Return:  ERROR Code
    //------------------------------------------------------------
    ERROR_CODE_TYPE  downloadParam(unsigned int addr,const char *data,int length);
    
    
    //------------------------------------------------------------
    // Function:    uploadParam
    // Summary: Upload Servo Configuration Param
    // In:      Address, length
    // Out:      data
    // Return:  ERROR Code
    //------------------------------------------------------------
    ERROR_CODE_TYPE  uploadParam(unsigned int addr,char *data,int& length);
    
    
    //------------------------------------------------------------
    // Function:    readIntVar
    // Summary: Read Int Varable by name
    // In:      varname
    // Out:      res
    // Return:  ERROR Code
    //------------------------------------------------------------    
    ERROR_CODE_TYPE  readIntVar(int size_of_varlist,const char *varname,int* res);

    //------------------------------------------------------------
    // Function:    readErrCode
    // Summary: Read Errcode
    // In:      size_of_codelist
    // Out:      res,numofres
    // Return:  ERROR Code
    //------------------------------------------------------------   
    ERROR_CODE_TYPE readErrCode(int size_of_codelist,int* res,int* numofres);

    //------------------------------------------------------------
    // Function:    servoCmd
    // Summary: service defined by servo software
    // In:      unsigned int i: servo command id; 
    //          const char * req: request data
    //          int req_size:     request data size       
    //          int res_size      response buffer size
    // Out:      char* res:       response data
    // Return:  ERROR Code
    //------------------------------------------------------------    
    ERROR_CODE_TYPE  servoCmd(unsigned int id,const char * req,int req_size,char* res,int res_size);
    //------------------------------------------------------------
    // Function:    setTrig
    // Summary: Set log trigger function
    // In:      trigname ticks
    // Out:     res: 1 succeed
    // Return:  ERROR Code
    //------------------------------------------------------------  
    ERROR_CODE_TYPE setTrig(const char *trigname,unsigned short ticks,int* res);
    //------------------------------------------------------------
    // Function:    initComm
    // Summary: To Initialize communication with service manager
    // In:      channel name
    // Out:     None
    // Return:  ERROR Code
    //------------------------------------------------------------
    static ERROR_CODE_TYPE initComm(const char *channel);
    
  public:
    static const int SERVO_CONF_SEG = 512;

  private:

    //------------------------------------------------------------
    // Function:    sendNRecv
    // Summary: Send request to Core1 and Receive response
    // In:      pointer to ServoService object
    // Out:     None
    // Return:  ERROR Code
    //------------------------------------------------------------    
    static ERROR_CODE_TYPE sendNRecv(fst_controller::ServoService* serv);

    // -----------------------------private functions---------------------------------------------

  private:

    // -----------------------------member variables---------------------------------------------
    static int counter_;
    static std::mutex mtx_;
    static fst_comm_interface::CommInterface *p_comm_;
  private:
    
    ServiceRequest client_service_request_;
    ServiceResponse client_service_response_;     
};  // class ServoService
}   // namespace fst_controller


#endif  // #ifndef SERVO_SERVICE_H

