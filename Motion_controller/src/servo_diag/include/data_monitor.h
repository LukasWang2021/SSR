/**********************************************************************
  Copyright:   Foresight-Robotics
  File:        data_monitor.h
  Author:      Yan He
  Data:        Dec.1  2016
  Modify:      Dec.23 2016
  Description: data_monitor--Class declaration.
**********************************************************************/


#ifndef DATA_MONITOR_H
#define DATA_MONITOR_H

#include "logfifo.h"
#include "servo_data_comm_struct.h"
#include "comm_interface/comm_interface.h"
#include <boost/lockfree/spsc_queue.hpp>
#include <vector>
#include "servo_data_comm_struct.h"
#include "LimitedFifo.h"

namespace fst_controller {
// Brief class for controller. This class include many default settings and functions to make life easier.

class DataMonitor {
  // -----------------------------public functions---------------------------------------------

  public:
    //------------------------------------------------------------
    // Function:    DataMonitor
    // Summary: The constructor of class
    // In:      ip_address: ip address; port: port number; 
    // Out:     None
    // Return:  None
    //------------------------------------------------------------
    DataMonitor(const char *ip_addr, int port);


    //------------------------------------------------------------
    // Function:    ~DataMonitor
    // Summary: The destructor of class
    // In:      None
    // Out:     None
    // Return:  None
    //------------------------------------------------------------
    ~DataMonitor();
    //------------------------------------------------------------
    // Function:   initDataMonitor
    // Summary: Init data monitor thread
    // In:      None
    // Out:     None
    // Return:  None
    //------------------------------------------------------------
    void initDataMonitor(void);
    //------------------------------------------------------------
    // Function:    startMonitor
    // Summary: Start Monitor
    // In:      None
    // Out:     size : snapshot size
    // Return:  None
    //------------------------------------------------------------
    static void startMonitor(DataMonitor* moni,std::vector<int>& t_list,unsigned int &ss_size);

    //------------------------------------------------------------
    // Function:    setSnapshotSize
    // Summary: Set Snapshot Size
    // In:      None
    // Out:     None
    // Return:  None
    //------------------------------------------------------------    
    static void setSnapshotSize(DataMonitor* moni,unsigned int ss_size);
    //------------------------------------------------------------
    // Function:    stopMonitor
    // Summary: Stop Monitor
    // In:      None
    // Out:     None
    // Return:  None
    //------------------------------------------------------------
    static void stopMonitor(DataMonitor* moni);
    //------------------------------------------------------------
    // Function:    pcComm_Thread
    // Summary: To start server to communicate with PC
    // In:      None
    // Out:     None
    // Return:  None
    //------------------------------------------------------------
    static void pcComm_Thread(DataMonitor* moni);
  private:
    // -----------------------------private functions---------------------------------------------
    static bool alignNcheck(int & k,int bytesize,int totalsize);

    //------------------------------------------------------------
    // Function:    logdata2Databuf
    // Summary: put the Log data to Data buffer for TCP
    // In:      
    // Out:     None
    // Return:  int
    //------------------------------------------------------------
    static int logdata2Databuf(char *databuf,int pos,const data64b_t* record_data,int type);

    //------------------------------------------------------------
    // Function:    dataMonitor_Thread
    // Summary: To start data monitor for core1
    // In:      None
    // Out:     None
    // Return:  None
    //------------------------------------------------------------
    static void dataMonitor_Thread(DataMonitor* moni);
    //------------------------------------------------------------
    // Function:   sendResponse
    // Summary: send Response
    // In:      None
    // Out:     None
    // Return:  None
    //------------------------------------------------------------

    static bool sendResponse(fst_comm_interface::CommInterface* pcomm,const void *buf, int buf_size);

    //------------------------------------------------------------
    // Function:    onRecordRequest
    // Summary: on data record request
    // In:      None
    // Out:     None
    // Return:  None
    //------------------------------------------------------------
    static bool onRecordRequest(DataMonitor* moni);
    //------------------------------------------------------------
    // Function:    onGetdataRequest
    // Summary: on get data request
    // In:      None
    // Out:     None
    // Return:  None
    //------------------------------------------------------------
    static int onGetdataRequest(unsigned char seq,DataMonitor* moni);
    //------------------------------------------------------------
    // Function:    onFinishRecord
    // Summary: on Finish Record
    // In:      None
    // Out:     None
    // Return:  None
    //------------------------------------------------------------
    static void onFinishRecord(DataMonitor* moni);

  private:

    // -----------------------------member variables---------------------------------------------

    LimitedFifo<Servo_Data_Record_t> *record_fifo_;
    fst_comm_interface::CommInterface* p_comm_;
    volatile bool start_monitor_;
    Servo_Data_Package_t data_package_;
    std::vector<int> t_list_;
    int data_state_;// -1: record data not required 0: record  is required 1: record fifo is locked
    unsigned int snapshot_size_;
};  // class ServcfComm
}   // namespace fst_controller


#endif  // #ifndef DATA_MONITOR_H
