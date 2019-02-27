/**********************************************
Copyright © 2016 Foresight-Robotics Ltd. All rights reserved.
File:       service_manager.h
Author:     Feng.Wu 
Create:     07-Nov-2016
Modify:     08-Dec-2016
Summary:    dealing with service
**********************************************/

#ifndef SERVICE_MANAGER_SERVICE_MANAGER_H_
#define SERVICE_MANAGER_SERVICE_MANAGER_H_

#include <vector>
#include "error_code.h"
#include "middleware_to_mem/middleware_to_sharedmem.h"
#include "comm_interface/comm_interface.h"
#include "struct_to_mem/struct_service_request.h"
#include "struct_to_mem/struct_service_response.h"
#include "service_actions/response_actions.h"


namespace fst_service_manager
{

enum ChannelStatus
{
    LOCAL_CHANNEL = 1,
    MCS_CHANNEL = 2,
    PARAM_CHANNEL = 3,
    TEST_CHANNEL = 4,
    TP_HEARTBEAT_CHANNEL = 5,
};

class ServiceManager
{
public:
    //------------------------------------------------------------
    // Function:  ServiceManager
    // Summary: The constructor of class
    // In:      None
    // Out:     None
    // Return:  None 
    //------------------------------------------------------------
    ServiceManager();

    //------------------------------------------------------------
    // Function:  ~ServiceManager
    // Summary: The destructor of class
    // In:      None
    // Out:     None
    // Return:  None 
    //------------------------------------------------------------
    ~ServiceManager();

    //------------------------------------------------------------
    // Function:    init
    // Summary: Initialize the communication channel and the sharedmem.
    // In:      None
    // Out:     None
    // Return:  0 -> success.
    //          ERROR_CODE -> failed.
    //------------------------------------------------------------
    ErrorCode init(void);

    //------------------------------------------------------------
    // Function:  receiveRequest
    // Summary: Receive a request. 
    // In:      None
    // Out:     None
    // Return:  true -> a request needs to be deal with.
    //          false -> did nothing. 
    //------------------------------------------------------------
    bool receiveRequest(void);

    //------------------------------------------------------------
    // Function:  addRequest
    // Summary: Setup the heartbeat request to BARE CORE if a number loop. 
    //          setup the stop request to BARE CORE if heartbeat_mcs missing.
    // In:      None
    // Out:     None
    // Return:  true -> a request to BARE CORE is setup.
    //          false -> didn't setup request.
    //------------------------------------------------------------
    bool addRequest(void);

    //------------------------------------------------------------
    // Function:  dtcSurvey
    // Summary: Create diagnostic request if there are errors in BARE CORE. 
    // In:      None
    // Out:     None
    // Return:  true -> error codes are available.
    //          false -> no error codes.
    //------------------------------------------------------------
    bool dtcSurvey(void);

    //------------------------------------------------------------
    // Function:  checkRequest
    // Summary: Mainly check if the request id is defined. 
    // In:      None
    // Out:     None
    // Return:  true -> the request id is available.
    //          false -> invailid request id.
    //------------------------------------------------------------
    bool checkRequest(ServiceRequest req);

    //------------------------------------------------------------
    // Function:  isLocalRequest
    // Summary: push local heartbeat request into local fifo if receive it.
    //          Fill the local heartbeart response with error_fifo_.
    // In:      None
    // Out:     None
    // Return:  true -> receive a heartbeat request.
    //          false -> not heartbeat request.
    //------------------------------------------------------------loop_count_core_
    bool isLocalRequest(ServiceRequest req);

    //------------------------------------------------------------
    // Function:  fillLocalHeartbeat
    // Summary: error codes are filled into the heartbeat response.
    // In:      None
    // Out:     None
    // Return:  true -> error codes are filled into the response.
    //          false -> No error codes to be filled.
    //------------------------------------------------------------
    bool fillLocalHeartbeat(void);

    //------------------------------------------------------------
    // Function:  sendRequest
    // Summary: send the service request to BARE CORE. 
    // In:      None
    // Out:     None
    // Return:  true -> success.
    //          false -> failed. 
    //------------------------------------------------------------
    bool sendRequest(void);

    //------------------------------------------------------------
    // Function:  getResponse
    // Summary: get the service response from BARE CORE. 
    // In:      None
    // Out:     Noneloop_count_core_
    // Return:  true -> success.
    //          false -> failed. 
    //------------------------------------------------------------
    bool getResponse(void);

    //------------------------------------------------------------
    // Function:  runService
    // Summary: Send the request and wait for the response from BARE CORE, including heartbeat_core_. 
    //          Create timeout error if abnormal.
    // In:      None
    // Out:     None
    // Return:  0 -> success to send a request and receive a response.
    //          BARE_CORE_TIMEOUT -> timeout. 
    //------------------------------------------------------------
    ErrorCode interactBareCore(void);

    //------------------------------------------------------------
    // Function:  storeError
    // Summary: Push the error code into error_fifo_. 
    // In:      None
    // Out:     None
    // Return:  true -> A error is pushed into error_fifo_.
    //          false -> did nothing. 
    //------------------------------------------------------------
    bool storeError(ErrorCode error);

    //------------------------------------------------------------
    // Function:  doesErrorExist
    // Summary: Check whether the error code is stored in error_fifo_. 
    // In:      None
    // Out:     None
    // Return:  true -> the error code exists in fifo.
    //          false -> the error code doesn't exist in fifo. 
    //------------------------------------------------------------
    bool doesErrorExist(ErrorCode error);

    //------------------------------------------------------------
    // Function:  manageResponse
    // Summary: deal with the service response from BARE CORE. 
    //          Some responses are deal with locally. git@git.foresight-robotics.cn:MC_System/Application.git
    //          Some are sent to the other processes.
    // In:      None
    // Out:     None
    // Return:  true -> a response is disposed.
    //          false -> did nothing. 
    //------------------------------------------------------------
    bool manageResponse(void);

    //------------------------------------------------------------
    // Function:  manageLocalResponse
    // Summary: deal with local responses. 
    // In:      None
    // Out:     None
    // Return:  true -> the local response is disposed.
    //          false -> did nothing. 
    //------------------------------------------------------------
    bool manageLocalResponse(void);

    //------------------------------------------------------------
    // Function:  transmitResponse
    // Summary: feedback the response to the other process. 
    // In:      comm -> the object used to send/recv msg.
    // Out:     None
    // Return:  true -> success to send msg.
    //          false -> fail to send msg.
    //------------------------------------------------------------
    bool transmitResponse(fst_comm_interface::CommInterface &comm);

    //------------------------------------------------------------
    // Function:  extractErrorCode
    // Summary: Extract error codes from dtc response.. 
    // In:      None
    // Out:     None
    // Return:  true -> success.
    //          false -> did nothing. 
    //------------------------------------------------------------
    bool extractErrorCode(ServiceResponse resp);


    //------------------------------------------------------------
    // Function:  sendRequestToBareCore
    // Summary: send the request to BARE CORE. 
    // In:      None
    // Out:     None
    // Return:  true -> success.
    //          false -> failed. 
    //------------------------------------------------------------
    bool sendRequestToBareCore(ServiceRequest &req);

    //------------------------------------------------------------
    // Function:  recvResponseFromBareCore
    // Summary: get the response from BARE CORE. 
    // In:      None
    // Out:     None
    // Return:  true -> success.
    //          false -> failed. 
    //------------------------------------------------------------
    bool recvResponseFromBareCore(ServiceResponse &resp);

    //------------------------------------------------------------
    // Function:  deleteFirstElement
    // Summary: delete the first element of a fifo. 
    // In:      None
    // Out:     None
    // Return:  true -> success.
    //          false -> failed. 
    //------------------------------------------------------------
    template<typename T>
    bool deleteFirstElement(T *fifo);
    
    //------------------------------------------------------------
    // Function:  runLoop
    // Summary: The main loop to run this process. 
    // In:      None.
    // Out:     None.
    // Return:  None.
    //------------------------------------------------------------
    void runLoop(void);

    // The max number of loops to send heartbeat request to BARE CORE.
    static const int HEARTBEAT_INTERVAL_CORE = 100; // 1ms * 100.

    // The BARE CORE timeout is 50ms. The unit is usec.
    //static const int HEARTBEAT_CORE_TIMEOUT = 50000;
    // The attempt number to communicate with barecore.
    static const int HEARTBEAT_CORE_TIMEOUT_COUNT = 5;

    // The max number of loops to recv heartbeat request from MCS.
    static const int HEARTBEAT_INTERVAL_MCS= 100; // 1ms * 100.

    // The attempt time to send response to other processes.
    static const int SEND_RESP_ATTEMPTS = 10;

    // The converting number from second to microsecond.
    static const unsigned int SEC_TO_USEC = 1000000;  

    // The cycle time of the main loop. Then unit is usec.
    static const unsigned int LOOP_TIME = 1000;

    static const int BYTE_LEN = 8;
private:

    // Used to manipulate shared memory of cores.
    int handle_core_;

    // The number counts every loop
    // To set the heartbeat interval of BARE CORE.
    int loop_count_core_;

    // The number counts every loop
    // To set the heartbeat interval limit of motion controller.
    int loop_count_mcs_;

    // To be true when motion controller send the first heartbeat.
    bool check_mcs_enable_;

    // The service ID which is dealing with.
    int running_sid_;

    // To indicate the response should be sent up or handled locally.
    ChannelStatus channel_flag_;

    // Used to communicate with other processes.
    fst_comm_interface::CommInterface comm_mcs_;
    fst_comm_interface::CommInterface comm_param_;
    fst_comm_interface::CommInterface comm_tp_heartbeat_;
    fst_comm_interface::CommInterface comm_test_;

    // The heartbeat request to BARE CORE.
    ServiceRequest heartbeat_core_;

    // The heartbeat response from this process.
    ServiceResponse heartbeat_local_;

    // The fifo is to deal with services related to BARE CORE.
    std::vector<ServiceRequest> request_fifo_;
    std::vector<ServiceResponse> response_fifo_;

    // The fifo is to store error codes.
    std::vector<ErrorCode> error_fifo_;

    // Used to respond local services from BARE CORE.
    fst_response_action::ResponseAction response_action_;
};
} //namespace fst_service_manager

#endif //SERVICE_MANAGER_SERVICE_MANAGER_H_
