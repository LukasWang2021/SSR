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
#include "middleware_to_mem/middleware_to_sharedmem.h"
#include "comm_interface/comm_interface.h"
#include "struct_to_mem/struct_service_request.h"
#include "struct_to_mem/struct_service_response.h"
#include "common_enum.h"
#include "error_code.h"
#include "common_log.h"
#include "service_manager_param.h"

namespace fst_service_manager
{

enum ChannelStatus
{
    LOCAL_CHANNEL = 1,
    MCS_CHANNEL = 2,
    CONTROLLER_HEARTBEAT_CHANNEL = 3,
    SERVO_DIAG_CHANNEL = 4,
    PARAM_CHANNEL = 5,
};

class ServiceManager
{
public:
    
    ServiceManager();

    ~ServiceManager();

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
    // Function:  interactBareCore
    // Summary: Send the request and wait for the response from BARE CORE, including heartbeat_core_req_. 
    //          Create timeout error if abnormal.
    // In:      None
    // Out:     None
    // Return:  0 -> success to send a request and receive a response.
    //          BARE_CORE_TIMEOUT -> timeout. 
    //------------------------------------------------------------
    ErrorCode interactBareCore(void);

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
    // Function:  runLoop
    // Summary: The main loop to run this process. 
    // In:      None.
    // Out:     None.
    // Return:  None.
    //------------------------------------------------------------
    void runLoop(void);

    void setExit(void);
    bool isExit(void);

    // The converting number from second to microsecond.
    static const unsigned int SEC_TO_USEC = 1000000;  

private:
     
    fst_log::Logger* log_ptr_;
    ServiceManagerParam* param_ptr_;
    int cycle_time_;
    int max_barecore_timeout_count_;
    int heartbeat_with_barecore_count_;
    int max_send_resp_count_;

    bool is_exit_;

    // Used to manipulate shared memory of cores.
    int handle_core_;

    // The number counts every loop to set the heartbeat interval of BARE CORE.
    int loop_count_core_;

    bool dtc_flag_;

    // The service ID which is dealing with.
    int running_sid_;

    // To indicate the response should be sent up or handled locally.
    ChannelStatus channel_flag_;

    // Used to communicate with other processes.
    fst_comm_interface::CommInterface comm_mcs_;
    fst_comm_interface::CommInterface comm_controller_heartbeat_;
    fst_comm_interface::CommInterface comm_servo_diag_;
    fst_comm_interface::CommInterface comm_param_;//todo delete

    // The heartbeat request to BARE CORE.
    ServiceRequest heartbeat_core_req_;

    // The heartbeat response from this process.
    ServiceResponse heartbeat_local_resp_;

    // The fifo is to deal with services related to BARE CORE.
    std::vector<ServiceRequest> request_fifo_;
    std::vector<ServiceResponse> response_fifo_;

    // The fifo is to store error codes.
    std::vector<ErrorCode> error_fifo_;

     //------------------------------------------------------------
    // Function:  addRequest
    // Summary: Setup the heartbeat request to BARE CORE if a number loop. 
    //          setup the stop request to BARE CORE if heartbeat_mcs missing.
    // In:      None
    // Out:     None
    // Return:  true -> a request to BARE CORE is setup.
    //          false -> didn't setup request.
    //------------------------------------------------------------
    bool addBareCoreHeartbeatRequest(void);

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
    // Function:  addRequest
    // Summary: Mainly check if the request id is defined. 
    // In:      None
    // Out:     None
    // Return:  true -> the request id is available.
    //          false -> invailid request id.
    //------------------------------------------------------------
    bool addRequest(ServiceRequest req);

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
    bool sendBareCoreRequest(void);

    //------------------------------------------------------------
    // Function:  getResponse
    // Summary: get the service response from BARE CORE. 
    // In:      None
    // Out:     Noneloop_count_core_
    // Return:  true -> success.
    //          false -> failed. 
    //------------------------------------------------------------
    bool getBareCoreResponse(void);

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

};
} //namespace fst_service_manager

#endif //SERVICE_MANAGER_SERVICE_MANAGER_H_
