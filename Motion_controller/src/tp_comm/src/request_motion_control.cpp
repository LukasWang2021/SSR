#include <pb_encode.h>
#include <pb_decode.h>
#include <pb_common.h>

#include "common_error_code.h"
#include "tp_comm.h"

using namespace base_space;
using namespace user_space;
using namespace log_space;

//"/rpc/motion_control/setGlobalVelRatio"
void TpComm::handleRequest0x000005EF(int recv_bytes)
{
    // create object for request and response package
    RequestMessageType_Double *request_data_ptr = new RequestMessageType_Double;
    if (request_data_ptr == NULL)
    {
        ErrorQueue::instance().push(TP_COMM_MEMORY_OPERATION_FAILED);
        LogProducer::error("comm", "handleRequest: can't allocate memory for request_data");
        return;
    }
    ResponseMessageType_Uint64 *response_data_ptr = new ResponseMessageType_Uint64;
    if (response_data_ptr == NULL)
    {
        ErrorQueue::instance().push(TP_COMM_MEMORY_OPERATION_FAILED);
        LogProducer::error("comm", "handleRequest: can't allocate memory for response_data");
        delete request_data_ptr;
        return;
    }

    handleRequestPackage(0x000005EF, (void *)request_data_ptr, (void *)response_data_ptr,
                         recv_bytes, RequestMessageType_Double_fields, -1);
}

//"/rpc/motion_control/getGlobalVelRatio"
void TpComm::handleRequest0x0001578F(int recv_bytes)
{
    // create object for request and response package
    RequestMessageType_Void *request_data_ptr = new RequestMessageType_Void;
    if (request_data_ptr == NULL)
    {
        ErrorQueue::instance().push(TP_COMM_MEMORY_OPERATION_FAILED);
        LogProducer::error("comm", "handleRequest: can't allocate memory for request_data");
        return;
    }
    ResponseMessageType_Uint64_Double *response_data_ptr = new ResponseMessageType_Uint64_Double;
    if (response_data_ptr == NULL)
    {
        ErrorQueue::instance().push(TP_COMM_MEMORY_OPERATION_FAILED);
        LogProducer::error("comm", "handleRequest: can't allocate memory for response_data");
        delete request_data_ptr;
        return;
    }

    handleRequestPackage(0x0001578F, (void *)request_data_ptr, (void *)response_data_ptr,
                         recv_bytes, RequestMessageType_Void_fields, -1);
}

//"/rpc/motion_control/setGlobalAccRatio",	,	"RequestMessageType.Double",	"ResponseMessageType.Uint64",
void TpComm::handleRequest0x0000271F(int recv_bytes)
{
    // create object for request and response package
    RequestMessageType_Double *request_data_ptr = new RequestMessageType_Double;
    if (request_data_ptr == NULL)
    {
        ErrorQueue::instance().push(TP_COMM_MEMORY_OPERATION_FAILED);
        LogProducer::error("comm", "handleRequest: can't allocate memory for request_data");
        return;
    }
    ResponseMessageType_Uint64 *response_data_ptr = new ResponseMessageType_Uint64;
    if (response_data_ptr == NULL)
    {
        ErrorQueue::instance().push(TP_COMM_MEMORY_OPERATION_FAILED);
        LogProducer::error("comm", "handleRequest: can't allocate memory for response_data");
        delete request_data_ptr;
        return;
    }

    handleRequestPackage(0x0000271F, (void *)request_data_ptr, (void *)response_data_ptr,
                         recv_bytes, RequestMessageType_Double_fields, -1);
}

//"/rpc/motion_control/getGlobalAccRatio"
void TpComm::handleRequest0x00016D9F(int recv_bytes)
{
    // create object for request and response package
    RequestMessageType_Void *request_data_ptr = new RequestMessageType_Void;
    if (request_data_ptr == NULL)
    {
        ErrorQueue::instance().push(TP_COMM_MEMORY_OPERATION_FAILED);
        LogProducer::error("comm", "handleRequest: can't allocate memory for request_data");
        return;
    }
    ResponseMessageType_Uint64_Double *response_data_ptr = new ResponseMessageType_Uint64_Double;
    if (response_data_ptr == NULL)
    {
        ErrorQueue::instance().push(TP_COMM_MEMORY_OPERATION_FAILED);
        LogProducer::error("comm", "handleRequest: can't allocate memory for response_data");
        delete request_data_ptr;
        return;
    }

    handleRequestPackage(0x00016D9F, (void *)request_data_ptr, (void *)response_data_ptr,
                         recv_bytes, RequestMessageType_Void_fields, -1);
}

//"/rpc/motion_control/axis_group/doStepManualMove"
void TpComm::handleRequest0x000085D5(int recv_bytes)
{
    // create object for request and response package
    RequestMessageType_Int32_Int32List *request_data_ptr = new RequestMessageType_Int32_Int32List;
    if (request_data_ptr == NULL)
    {
        ErrorQueue::instance().push(TP_COMM_MEMORY_OPERATION_FAILED);
        LogProducer::error("comm", "handleRequest: can't allocate memory for request_data");
        return;
    }
    ResponseMessageType_Uint64 *response_data_ptr = new ResponseMessageType_Uint64;
    if (response_data_ptr == NULL)
    {
        ErrorQueue::instance().push(TP_COMM_MEMORY_OPERATION_FAILED);
        LogProducer::error("comm", "handleRequest: can't allocate memory for response_data");
        delete request_data_ptr;
        return;
    }

    handleRequestPackage(0x000085D5, (void *)request_data_ptr, (void *)response_data_ptr,
                         recv_bytes, RequestMessageType_Int32_Int32List_fields, -1);
}

//"/rpc/motion_control/axis_group/doContinusManulMove"
void TpComm::handleRequest0x0000D3F5(int recv_bytes)
{
    // create object for request and response package
    RequestMessageType_Int32_Int32List *request_data_ptr = new RequestMessageType_Int32_Int32List;
    if (request_data_ptr == NULL)
    {
        ErrorQueue::instance().push(TP_COMM_MEMORY_OPERATION_FAILED);
        LogProducer::error("comm", "handleRequest: can't allocate memory for request_data");
        return;
    }
    ResponseMessageType_Uint64 *response_data_ptr = new ResponseMessageType_Uint64;
    if (response_data_ptr == NULL)
    {
        ErrorQueue::instance().push(TP_COMM_MEMORY_OPERATION_FAILED);
        LogProducer::error("comm", "handleRequest: can't allocate memory for response_data");
        delete request_data_ptr;
        return;
    }

    handleRequestPackage(0x0000D3F5, (void *)request_data_ptr, (void *)response_data_ptr,
                         recv_bytes, RequestMessageType_Int32_Int32List_fields, -1);
}

//"/rpc/motion_control/axis_group/doGotoCartesianPointManualMove"
void TpComm::handleRequest0x00010C05(int recv_bytes)
{
    // create object for request and response package
    RequestMessageType_Int32_UFTF_PoseAndPosture *request_data_ptr = new RequestMessageType_Int32_UFTF_PoseAndPosture;
    if (request_data_ptr == NULL)
    {
        ErrorQueue::instance().push(TP_COMM_MEMORY_OPERATION_FAILED);
        LogProducer::error("comm", "handleRequest: can't allocate memory for request_data");
        return;
    }
    ResponseMessageType_Uint64 *response_data_ptr = new ResponseMessageType_Uint64;
    if (response_data_ptr == NULL)
    {
        ErrorQueue::instance().push(TP_COMM_MEMORY_OPERATION_FAILED);
        LogProducer::error("comm", "handleRequest: can't allocate memory for response_data");
        delete request_data_ptr;
        return;
    }

    handleRequestPackage(0x00010C05, (void *)request_data_ptr, (void *)response_data_ptr,
                         recv_bytes, RequestMessageType_Int32_UFTF_PoseAndPosture_fields, -1);
}

//"/rpc/motion_control/axis_group/doGotoJointPointManualMove"
void TpComm::handleRequest0x00008075(int recv_bytes)
{
    // create object for request and response package
    RequestMessageType_Int32_DoubleList *request_data_ptr = new RequestMessageType_Int32_DoubleList;
    if (request_data_ptr == NULL)
    {
        ErrorQueue::instance().push(TP_COMM_MEMORY_OPERATION_FAILED);
        LogProducer::error("comm", "handleRequest: can't allocate memory for request_data");
        return;
    }
    ResponseMessageType_Uint64 *response_data_ptr = new ResponseMessageType_Uint64;
    if (response_data_ptr == NULL)
    {
        ErrorQueue::instance().push(TP_COMM_MEMORY_OPERATION_FAILED);
        LogProducer::error("comm", "handleRequest: can't allocate memory for response_data");
        delete request_data_ptr;
        return;
    }

    handleRequestPackage(0x00008075, (void *)request_data_ptr, (void *)response_data_ptr,
                         recv_bytes, RequestMessageType_Int32_DoubleList_fields, -1);
}

//"/rpc/motion_control/axis_group/doManualStop"
void TpComm::handleRequest0x0000A9A0(int recv_bytes)
{
    // create object for request and response package
    RequestMessageType_Int32 *request_data_ptr = new RequestMessageType_Int32;
    if (request_data_ptr == NULL)
    {
        ErrorQueue::instance().push(TP_COMM_MEMORY_OPERATION_FAILED);
        LogProducer::error("comm", "handleRequest: can't allocate memory for request_data");
        return;
    }
    ResponseMessageType_Uint64 *response_data_ptr = new ResponseMessageType_Uint64;
    if (response_data_ptr == NULL)
    {
        ErrorQueue::instance().push(TP_COMM_MEMORY_OPERATION_FAILED);
        LogProducer::error("comm", "handleRequest: can't allocate memory for response_data");
        delete request_data_ptr;
        return;
    }

    handleRequestPackage(0x0000A9A0, (void *)request_data_ptr, (void *)response_data_ptr,
                         recv_bytes, RequestMessageType_Int32_fields, -1);
}

//"/rpc/motion_control/axis_group/getJointsFeedBack"
void TpComm::handleRequest0x0000DFBB(int recv_bytes)
{
    // create object for request and response package
    RequestMessageType_Int32 *request_data_ptr = new RequestMessageType_Int32;
    if (request_data_ptr == NULL)
    {
        ErrorQueue::instance().push(TP_COMM_MEMORY_OPERATION_FAILED);
        LogProducer::error("comm", "handleRequest: can't allocate memory for request_data");
        return;
    }
    ResponseMessageType_Uint64_DoubleList *response_data_ptr = new ResponseMessageType_Uint64_DoubleList;
    if (response_data_ptr == NULL)
    {
        ErrorQueue::instance().push(TP_COMM_MEMORY_OPERATION_FAILED);
        LogProducer::error("comm", "handleRequest: can't allocate memory for response_data");
        delete request_data_ptr;
        return;
    }

    handleRequestPackage(0x0000DFBB, (void *)request_data_ptr, (void *)response_data_ptr,
                         recv_bytes, RequestMessageType_Int32_fields, -1);
}

//"/rpc/motion_control/axis_group/setUserSoftLimit"
void TpComm::handleRequest0x000114A4(int recv_bytes)
{
    // create object for request and response package
    RequestMessageType_Int32_JointLimit *request_data_ptr = new RequestMessageType_Int32_JointLimit;
    if (request_data_ptr == NULL)
    {
        ErrorQueue::instance().push(TP_COMM_MEMORY_OPERATION_FAILED);
        LogProducer::error("comm", "handleRequest: can't allocate memory for request_data");
        return;
    }
    ResponseMessageType_Uint64 *response_data_ptr = new ResponseMessageType_Uint64;
    if (response_data_ptr == NULL)
    {
        ErrorQueue::instance().push(TP_COMM_MEMORY_OPERATION_FAILED);
        LogProducer::error("comm", "handleRequest: can't allocate memory for response_data");
        delete request_data_ptr;
        return;
    }

    handleRequestPackage(0x000114A4, (void *)request_data_ptr, (void *)response_data_ptr,
                         recv_bytes, RequestMessageType_Int32_JointLimit_fields, -1);
}

//"/rpc/motion_control/axis_group/getUserSoftLimit"
void TpComm::handleRequest0x0000C764(int recv_bytes)
{
    // create object for request and response package
    RequestMessageType_Int32 *request_data_ptr = new RequestMessageType_Int32;
    if (request_data_ptr == NULL)
    {
        ErrorQueue::instance().push(TP_COMM_MEMORY_OPERATION_FAILED);
        LogProducer::error("comm", "handleRequest: can't allocate memory for request_data");
        return;
    }
    ResponseMessageType_Uint64_JointLimit *response_data_ptr = new ResponseMessageType_Uint64_JointLimit;
    if (response_data_ptr == NULL)
    {
        ErrorQueue::instance().push(TP_COMM_MEMORY_OPERATION_FAILED);
        LogProducer::error("comm", "handleRequest: can't allocate memory for response_data");
        delete request_data_ptr;
        return;
    }

    handleRequestPackage(0x0000C764, (void *)request_data_ptr, (void *)response_data_ptr,
                         recv_bytes, RequestMessageType_Int32_fields, -1);
}

//"/rpc/motion_control/axis_group/setManuSoftLimit"
void TpComm::handleRequest0x000108E4(int recv_bytes)
{
    // create object for request and response package
    RequestMessageType_Int32_JointLimit *request_data_ptr = new RequestMessageType_Int32_JointLimit;
    if (request_data_ptr == NULL)
    {
        ErrorQueue::instance().push(TP_COMM_MEMORY_OPERATION_FAILED);
        LogProducer::error("comm", "handleRequest: can't allocate memory for request_data");
        return;
    }
    ResponseMessageType_Uint64 *response_data_ptr = new ResponseMessageType_Uint64;
    if (response_data_ptr == NULL)
    {
        ErrorQueue::instance().push(TP_COMM_MEMORY_OPERATION_FAILED);
        LogProducer::error("comm", "handleRequest: can't allocate memory for response_data");
        delete request_data_ptr;
        return;
    }

    handleRequestPackage(0x000108E4, (void *)request_data_ptr, (void *)response_data_ptr,
                         recv_bytes, RequestMessageType_Int32_JointLimit_fields, -1);
}

//"/rpc/motion_control/axis_group/getManuSoftLimit"
void TpComm::handleRequest0x0000C244(int recv_bytes)
{
    // create object for request and response package
    RequestMessageType_Int32 *request_data_ptr = new RequestMessageType_Int32;
    if (request_data_ptr == NULL)
    {
        ErrorQueue::instance().push(TP_COMM_MEMORY_OPERATION_FAILED);
        LogProducer::error("comm", "handleRequest: can't allocate memory for request_data");
        return;
    }
    ResponseMessageType_Uint64_JointLimit *response_data_ptr = new ResponseMessageType_Uint64_JointLimit;
    if (response_data_ptr == NULL)
    {
        ErrorQueue::instance().push(TP_COMM_MEMORY_OPERATION_FAILED);
        LogProducer::error("comm", "handleRequest: can't allocate memory for response_data");
        delete request_data_ptr;
        return;
    }

    handleRequestPackage(0x0000C244, (void *)request_data_ptr, (void *)response_data_ptr,
                         recv_bytes, RequestMessageType_Int32_fields, -1);
}



//"/rpc/motion_control/axis_group/setHardLimit"
void TpComm::handleRequest0x0000C454(int recv_bytes)
{
    // create object for request and response package
    RequestMessageType_Int32_JointLimit *request_data_ptr = new RequestMessageType_Int32_JointLimit;
    if (request_data_ptr == NULL)
    {
        ErrorQueue::instance().push(TP_COMM_MEMORY_OPERATION_FAILED);
        LogProducer::error("comm", "handleRequest: can't allocate memory for request_data");
        return;
    }
    ResponseMessageType_Uint64 *response_data_ptr = new ResponseMessageType_Uint64;
    if (response_data_ptr == NULL)
    {
        ErrorQueue::instance().push(TP_COMM_MEMORY_OPERATION_FAILED);
        LogProducer::error("comm", "handleRequest: can't allocate memory for response_data");
        delete request_data_ptr;
        return;
    }

    handleRequestPackage(0x0000C454, (void *)request_data_ptr, (void *)response_data_ptr,
                         recv_bytes, RequestMessageType_Int32_JointLimit_fields, -1);
}

//"/rpc/motion_control/axis_group/getHardLimit"
void TpComm::handleRequest0x00013394(int recv_bytes)
{
    // create object for request and response package
    RequestMessageType_Int32 *request_data_ptr = new RequestMessageType_Int32;
    if (request_data_ptr == NULL)
    {
        ErrorQueue::instance().push(TP_COMM_MEMORY_OPERATION_FAILED);
        LogProducer::error("comm", "handleRequest: can't allocate memory for request_data");
        return;
    }
    ResponseMessageType_Uint64_JointLimit *response_data_ptr = new ResponseMessageType_Uint64_JointLimit;
    if (response_data_ptr == NULL)
    {
        ErrorQueue::instance().push(TP_COMM_MEMORY_OPERATION_FAILED);
        LogProducer::error("comm", "handleRequest: can't allocate memory for response_data");
        delete request_data_ptr;
        return;
    }

    handleRequestPackage(0x00013394, (void *)request_data_ptr, (void *)response_data_ptr,
                         recv_bytes, RequestMessageType_Int32_fields, -1);
}

//"/rpc/motion_control/axis_group/setCoordinate"
void TpComm::handleRequest0x0000A845(int recv_bytes)
{
    // create object for request and response package
    RequestMessageType_Int32List *request_data_ptr = new RequestMessageType_Int32List;
    if (request_data_ptr == NULL)
    {
        ErrorQueue::instance().push(TP_COMM_MEMORY_OPERATION_FAILED);
        LogProducer::error("comm", "handleRequest: can't allocate memory for request_data");
        return;
    }
    ResponseMessageType_Uint64 *response_data_ptr = new ResponseMessageType_Uint64;
    if (response_data_ptr == NULL)
    {
        ErrorQueue::instance().push(TP_COMM_MEMORY_OPERATION_FAILED);
        LogProducer::error("comm", "handleRequest: can't allocate memory for response_data");
        delete request_data_ptr;
        return;
    }

    handleRequestPackage(0x0000A845, (void *)request_data_ptr, (void *)response_data_ptr,
                         recv_bytes, RequestMessageType_Int32List_fields, -1);
}

//"/rpc/motion_control/axis_group/getCoordinate"
void TpComm::handleRequest0x00008595(int recv_bytes)
{
    // create object for request and response package
    RequestMessageType_Int32 *request_data_ptr = new RequestMessageType_Int32;
    if (request_data_ptr == NULL)
    {
        ErrorQueue::instance().push(TP_COMM_MEMORY_OPERATION_FAILED);
        LogProducer::error("comm", "handleRequest: can't allocate memory for request_data");
        return;
    }
    ResponseMessageType_Uint64_Int32 *response_data_ptr = new ResponseMessageType_Uint64_Int32;
    if (response_data_ptr == NULL)
    {
        ErrorQueue::instance().push(TP_COMM_MEMORY_OPERATION_FAILED);
        LogProducer::error("comm", "handleRequest: can't allocate memory for response_data");
        delete request_data_ptr;
        return;
    }

    handleRequestPackage(0x00008595, (void *)request_data_ptr, (void *)response_data_ptr,
                         recv_bytes, RequestMessageType_Int32_fields, -1);
}

//"/rpc/motion_control/axis_group/setUserCoordId"
void TpComm::handleRequest0x00005CF4(int recv_bytes)
{
    // create object for request and response package
    RequestMessageType_Int32List *request_data_ptr = new RequestMessageType_Int32List;
    if (request_data_ptr == NULL)
    {
        ErrorQueue::instance().push(TP_COMM_MEMORY_OPERATION_FAILED);
        LogProducer::error("comm", "handleRequest: can't allocate memory for request_data");
        return;
    }
    ResponseMessageType_Uint64 *response_data_ptr = new ResponseMessageType_Uint64;
    if (response_data_ptr == NULL)
    {
        ErrorQueue::instance().push(TP_COMM_MEMORY_OPERATION_FAILED);
        LogProducer::error("comm", "handleRequest: can't allocate memory for response_data");
        delete request_data_ptr;
        return;
    }

    handleRequestPackage(0x00005CF4, (void *)request_data_ptr, (void *)response_data_ptr,
                         recv_bytes, RequestMessageType_Int32List_fields, -1);
}

//"/rpc/motion_control/axis_group/getUserCoordId"
void TpComm::handleRequest0x00005BB4(int recv_bytes)
{
    // create object for request and response package
    RequestMessageType_Int32 *request_data_ptr = new RequestMessageType_Int32;
    if (request_data_ptr == NULL)
    {
        ErrorQueue::instance().push(TP_COMM_MEMORY_OPERATION_FAILED);
        LogProducer::error("comm", "handleRequest: can't allocate memory for request_data");
        return;
    }
    ResponseMessageType_Uint64_Int32 *response_data_ptr = new ResponseMessageType_Uint64_Int32;
    if (response_data_ptr == NULL)
    {
        ErrorQueue::instance().push(TP_COMM_MEMORY_OPERATION_FAILED);
        LogProducer::error("comm", "handleRequest: can't allocate memory for response_data");
        delete request_data_ptr;
        return;
    }

    handleRequestPackage(0x00005BB4, (void *)request_data_ptr, (void *)response_data_ptr,
                         recv_bytes, RequestMessageType_Int32_fields, -1);
}

//"/rpc/motion_control/axis_group/setTool"
void TpComm::handleRequest0x0001581C(int recv_bytes)
{
    // create object for request and response package
    RequestMessageType_Int32List *request_data_ptr = new RequestMessageType_Int32List;
    if (request_data_ptr == NULL)
    {
        ErrorQueue::instance().push(TP_COMM_MEMORY_OPERATION_FAILED);
        LogProducer::error("comm", "handleRequest: can't allocate memory for request_data");
        return;
    }
    ResponseMessageType_Uint64 *response_data_ptr = new ResponseMessageType_Uint64;
    if (response_data_ptr == NULL)
    {
        ErrorQueue::instance().push(TP_COMM_MEMORY_OPERATION_FAILED);
        LogProducer::error("comm", "handleRequest: can't allocate memory for response_data");
        delete request_data_ptr;
        return;
    }

    handleRequestPackage(0x0001581C, (void *)request_data_ptr, (void *)response_data_ptr,
                         recv_bytes, RequestMessageType_Int32List_fields, -1);
}

//"/rpc/motion_control/axis_group/getTool"
void TpComm::handleRequest0x0001354C(int recv_bytes)
{
    // create object for request and response package
    RequestMessageType_Int32 *request_data_ptr = new RequestMessageType_Int32;
    if (request_data_ptr == NULL)
    {
        ErrorQueue::instance().push(TP_COMM_MEMORY_OPERATION_FAILED);
        LogProducer::error("comm", "handleRequest: can't allocate memory for request_data");
        return;
    }
    ResponseMessageType_Uint64_Int32 *response_data_ptr = new ResponseMessageType_Uint64_Int32;
    if (response_data_ptr == NULL)
    {
        ErrorQueue::instance().push(TP_COMM_MEMORY_OPERATION_FAILED);
        LogProducer::error("comm", "handleRequest: can't allocate memory for response_data");
        delete request_data_ptr;
        return;
    }

    handleRequestPackage(0x0001354C, (void *)request_data_ptr, (void *)response_data_ptr,
                         recv_bytes, RequestMessageType_Int32_fields, -1);
}

//"/rpc/motion_control/axis_group/convertCartToJoint"
void TpComm::handleRequest0x00010FD4(int recv_bytes)
{
    // create object for request and response package
    RequestMessageType_Int32_UFTF_PoseAndPosture *request_data_ptr = new RequestMessageType_Int32_UFTF_PoseAndPosture;
    if (request_data_ptr == NULL)
    {
        ErrorQueue::instance().push(TP_COMM_MEMORY_OPERATION_FAILED);
        LogProducer::error("comm", "handleRequest: can't allocate memory for request_data");
        return;
    }
    ResponseMessageType_Uint64_DoubleList *response_data_ptr = new ResponseMessageType_Uint64_DoubleList;
    if (response_data_ptr == NULL)
    {
        ErrorQueue::instance().push(TP_COMM_MEMORY_OPERATION_FAILED);
        LogProducer::error("comm", "handleRequest: can't allocate memory for response_data");
        delete request_data_ptr;
        return;
    }

    handleRequestPackage(0x00010FD4, (void *)request_data_ptr, (void *)response_data_ptr,
                         recv_bytes, RequestMessageType_Int32_UFTF_PoseAndPosture_fields, -1);
}

//"/rpc/motion_control/axis_group/convertJointToCart",
void TpComm::handleRequest0x0000B6D4(int recv_bytes)
{
    // create object for request and response package
    RequestMessageType_Int32List_DoubleList *request_data_ptr = new RequestMessageType_Int32List_DoubleList;
    if (request_data_ptr == NULL)
    {
        ErrorQueue::instance().push(TP_COMM_MEMORY_OPERATION_FAILED);
        LogProducer::error("comm", "handleRequest: can't allocate memory for request_data");
        return;
    }
    ResponseMessageType_Uint64_PoseAndPosture *response_data_ptr = new ResponseMessageType_Uint64_PoseAndPosture;
    if (response_data_ptr == NULL)
    {
        ErrorQueue::instance().push(TP_COMM_MEMORY_OPERATION_FAILED);
        LogProducer::error("comm", "handleRequest: can't allocate memory for response_data");
        delete request_data_ptr;
        return;
    }

    handleRequestPackage(0x0000B6D4, (void *)request_data_ptr, (void *)response_data_ptr,
                         recv_bytes, RequestMessageType_Int32List_DoubleList_fields, -1);
}

//"/rpc/motion_control/axis_group/ignoreLostZeroError"
void TpComm::handleRequest0x00014952(int recv_bytes)
{
    // create object for request and response package
    RequestMessageType_Int32 *request_data_ptr = new RequestMessageType_Int32;
    if (request_data_ptr == NULL)
    {
        ErrorQueue::instance().push(TP_COMM_MEMORY_OPERATION_FAILED);
        LogProducer::error("comm", "handleRequest: can't allocate memory for request_data");
        return;
    }
    ResponseMessageType_Uint64 *response_data_ptr = new ResponseMessageType_Uint64;
    if (response_data_ptr == NULL)
    {
        ErrorQueue::instance().push(TP_COMM_MEMORY_OPERATION_FAILED);
        LogProducer::error("comm", "handleRequest: can't allocate memory for response_data");
        delete request_data_ptr;
        return;
    }

    handleRequestPackage(0x00014952, (void *)request_data_ptr, (void *)response_data_ptr,
                         recv_bytes, RequestMessageType_Int32_fields, -1);
}

//"/rpc/motion_control/axis_group/setSingleZeroPointOffset",	"RequestMessageType.Int32List_Double(count=2)",	"ResponseMessageType.Uint64"
void TpComm::handleRequest0x00012404(int recv_bytes)
{
    // create object for request and response package
    RequestMessageType_Int32List_Double *request_data_ptr = new RequestMessageType_Int32List_Double;
    if (request_data_ptr == NULL)
    {
        ErrorQueue::instance().push(TP_COMM_MEMORY_OPERATION_FAILED);
        LogProducer::error("comm", "handleRequest: can't allocate memory for request_data");
        return;
    }
    ResponseMessageType_Uint64 *response_data_ptr = new ResponseMessageType_Uint64;
    if (response_data_ptr == NULL)
    {
        ErrorQueue::instance().push(TP_COMM_MEMORY_OPERATION_FAILED);
        LogProducer::error("comm", "handleRequest: can't allocate memory for response_data");
        delete request_data_ptr;
        return;
    }

    handleRequestPackage(0x00012404, (void *)request_data_ptr, (void *)response_data_ptr,
                         recv_bytes, RequestMessageType_Int32List_Double_fields, -1);
}

//"/rpc/motion_control/axis_group/setAllZeroPointOffsets"
void TpComm::handleRequest0x00011853(int recv_bytes)
{
    // create object for request and response package
    RequestMessageType_Int32_DoubleList *request_data_ptr = new RequestMessageType_Int32_DoubleList;
    if (request_data_ptr == NULL)
    {
        ErrorQueue::instance().push(TP_COMM_MEMORY_OPERATION_FAILED);
        LogProducer::error("comm", "handleRequest: can't allocate memory for request_data");
        return;
    }
    ResponseMessageType_Uint64 *response_data_ptr = new ResponseMessageType_Uint64;
    if (response_data_ptr == NULL)
    {
        ErrorQueue::instance().push(TP_COMM_MEMORY_OPERATION_FAILED);
        LogProducer::error("comm", "handleRequest: can't allocate memory for response_data");
        delete request_data_ptr;
        return;
    }

    handleRequestPackage(0x00011853, (void *)request_data_ptr, (void *)response_data_ptr,
                         recv_bytes, RequestMessageType_Int32_DoubleList_fields, -1);
}

//"/rpc/motion_control/axis_group/getAllZeroPointOffsets"
void TpComm::handleRequest0x00012353(int recv_bytes)
{
    // create object for request and response package
    RequestMessageType_Int32 *request_data_ptr = new RequestMessageType_Int32;
    if (request_data_ptr == NULL)
    {
        ErrorQueue::instance().push(TP_COMM_MEMORY_OPERATION_FAILED);
        LogProducer::error("comm", "handleRequest: can't allocate memory for request_data");
        return;
    }
    ResponseMessageType_Uint64_DoubleList *response_data_ptr = new ResponseMessageType_Uint64_DoubleList;
    if (response_data_ptr == NULL)
    {
        ErrorQueue::instance().push(TP_COMM_MEMORY_OPERATION_FAILED);
        LogProducer::error("comm", "handleRequest: can't allocate memory for response_data");
        delete request_data_ptr;
        return;
    }

    handleRequestPackage(0x00012353, (void *)request_data_ptr, (void *)response_data_ptr,
                         recv_bytes, RequestMessageType_Int32_fields, -1);
}

//"/rpc/motion_control/axis_group/getAllZeroErrorMaskStatus"
void TpComm::handleRequest0x0000C183(int recv_bytes)
{
    // create object for request and response package
    RequestMessageType_Int32 *request_data_ptr = new RequestMessageType_Int32;
    if (request_data_ptr == NULL)
    {
        ErrorQueue::instance().push(TP_COMM_MEMORY_OPERATION_FAILED);
        LogProducer::error("comm", "handleRequest: can't allocate memory for request_data");
        return;
    }
    ResponseMessageType_Uint64_Int32List *response_data_ptr = new ResponseMessageType_Uint64_Int32List;
    if (response_data_ptr == NULL)
    {
        ErrorQueue::instance().push(TP_COMM_MEMORY_OPERATION_FAILED);
        LogProducer::error("comm", "handleRequest: can't allocate memory for response_data");
        delete request_data_ptr;
        return;
    }

    handleRequestPackage(0x0000C183, (void *)request_data_ptr, (void *)response_data_ptr,
                         recv_bytes, RequestMessageType_Int32_fields, -1);
}

//"/rpc/motion_control/axis_group/setSingleZeroPointStatus"
void TpComm::handleRequest0x00010E43(int recv_bytes)
{
    // create object for request and response package
    RequestMessageType_Int32List *request_data_ptr = new RequestMessageType_Int32List;
    if (request_data_ptr == NULL)
    {
        ErrorQueue::instance().push(TP_COMM_MEMORY_OPERATION_FAILED);
        LogProducer::error("comm", "handleRequest: can't allocate memory for request_data");
        return;
    }
    ResponseMessageType_Uint64 *response_data_ptr = new ResponseMessageType_Uint64;
    if (response_data_ptr == NULL)
    {
        ErrorQueue::instance().push(TP_COMM_MEMORY_OPERATION_FAILED);
        LogProducer::error("comm", "handleRequest: can't allocate memory for response_data");
        delete request_data_ptr;
        return;
    }

    handleRequestPackage(0x00010E43, (void *)request_data_ptr, (void *)response_data_ptr,
                         recv_bytes, RequestMessageType_Int32List_fields, -1);
}

//"/rpc/motion_control/axis_group/getAllZeroPointStatus"
void TpComm::handleRequest0x000102F3(int recv_bytes)
{
    RequestMessageType_Int32 *request_data_ptr = new RequestMessageType_Int32;
    if (request_data_ptr == NULL)
    {
        ErrorQueue::instance().push(TP_COMM_MEMORY_OPERATION_FAILED);
        LogProducer::error("comm", "handleRequest: can't allocate memory for request_data");
        return;
    }
    ResponseMessageType_Uint64_Int32List *response_data_ptr = new ResponseMessageType_Uint64_Int32List;
    if (response_data_ptr == NULL)
    {
        ErrorQueue::instance().push(TP_COMM_MEMORY_OPERATION_FAILED);
        LogProducer::error("comm", "handleRequest: can't allocate memory for response_data");
        delete request_data_ptr;
        return;
    }

    handleRequestPackage(0x000102F3, (void *)request_data_ptr, (void *)response_data_ptr,
                         recv_bytes, RequestMessageType_Int32_fields, -1);
}


//"/rpc/motion_control/axis_group/setJointManualStep"
void TpComm::handleRequest0x00018470(int recv_bytes)
{
    // create object for request and response package
    RequestMessageType_Int32_DoubleList *request_data_ptr = new RequestMessageType_Int32_DoubleList;
    if (request_data_ptr == NULL)
    {
        ErrorQueue::instance().push(TP_COMM_MEMORY_OPERATION_FAILED);
        LogProducer::error("comm", "handleRequest: can't allocate memory for request_data");
        return;
    }
    ResponseMessageType_Uint64 *response_data_ptr = new ResponseMessageType_Uint64;
    if (response_data_ptr == NULL)
    {
        ErrorQueue::instance().push(TP_COMM_MEMORY_OPERATION_FAILED);
        LogProducer::error("comm", "handleRequest: can't allocate memory for response_data");
        delete request_data_ptr;
        return;
    }

    handleRequestPackage(0x00018470, (void *)request_data_ptr, (void *)response_data_ptr,
                         recv_bytes, RequestMessageType_Int32_DoubleList_fields, -1);
}

//"/rpc/motion_control/axis_group/getJointManualStep"
void TpComm::handleRequest0x00006D10(int recv_bytes)
{
    // create object for request and response package
    RequestMessageType_Int32 *request_data_ptr = new RequestMessageType_Int32;
    if (request_data_ptr == NULL)
    {
        ErrorQueue::instance().push(TP_COMM_MEMORY_OPERATION_FAILED);
        LogProducer::error("comm", "handleRequest: can't allocate memory for request_data");
        return;
    }
    ResponseMessageType_Uint64_DoubleList *response_data_ptr = new ResponseMessageType_Uint64_DoubleList;
    if (response_data_ptr == NULL)
    {
        ErrorQueue::instance().push(TP_COMM_MEMORY_OPERATION_FAILED);
        LogProducer::error("comm", "handleRequest: can't allocate memory for response_data");
        delete request_data_ptr;
        return;
    }

    handleRequestPackage(0x00006D10, (void *)request_data_ptr, (void *)response_data_ptr,
                         recv_bytes, RequestMessageType_Int32_fields, -1);
}

//"/rpc/motion_control/axis_group/setCartesianManualStep"
void TpComm::handleRequest0x0000A420(int recv_bytes)
{
    // create object for request and response package
    RequestMessageType_Int32_Double *request_data_ptr = new RequestMessageType_Int32_Double;
    if (request_data_ptr == NULL)
    {
        ErrorQueue::instance().push(TP_COMM_MEMORY_OPERATION_FAILED);
        LogProducer::error("comm", "handleRequest: can't allocate memory for request_data");
        return;
    }
    ResponseMessageType_Uint64 *response_data_ptr = new ResponseMessageType_Uint64;
    if (response_data_ptr == NULL)
    {
        ErrorQueue::instance().push(TP_COMM_MEMORY_OPERATION_FAILED);
        LogProducer::error("comm", "handleRequest: can't allocate memory for response_data");
        delete request_data_ptr;
        return;
    }

    handleRequestPackage(0x0000A420, (void *)request_data_ptr, (void *)response_data_ptr,
                         recv_bytes, RequestMessageType_Int32_Double_fields, -1);
}

//"/rpc/motion_control/axis_group/getCartesianManualStep"
void TpComm::handleRequest0x0000EAC0(int recv_bytes)
{
    // create object for request and response package
    RequestMessageType_Int32 *request_data_ptr = new RequestMessageType_Int32;
    if (request_data_ptr == NULL)
    {
        ErrorQueue::instance().push(TP_COMM_MEMORY_OPERATION_FAILED);
        LogProducer::error("comm", "handleRequest: can't allocate memory for request_data");
        return;
    }
    ResponseMessageType_Uint64_Double *response_data_ptr = new ResponseMessageType_Uint64_Double;
    if (response_data_ptr == NULL)
    {
        ErrorQueue::instance().push(TP_COMM_MEMORY_OPERATION_FAILED);
        LogProducer::error("comm", "handleRequest: can't allocate memory for response_data");
        delete request_data_ptr;
        return;
    }

    handleRequestPackage(0x0000EAC0, (void *)request_data_ptr, (void *)response_data_ptr,
                         recv_bytes, RequestMessageType_Int32_fields, -1);
}

//"/rpc/motion_control/axis_group/setOrientationManualStep"
void TpComm::handleRequest0x00002940(int recv_bytes)
{
    // create object for request and response package
    RequestMessageType_Int32_Double *request_data_ptr = new RequestMessageType_Int32_Double;
    if (request_data_ptr == NULL)
    {
        ErrorQueue::instance().push(TP_COMM_MEMORY_OPERATION_FAILED);
        LogProducer::error("comm", "handleRequest: can't allocate memory for request_data");
        return;
    }
    ResponseMessageType_Uint64 *response_data_ptr = new ResponseMessageType_Uint64;
    if (response_data_ptr == NULL)
    {
        ErrorQueue::instance().push(TP_COMM_MEMORY_OPERATION_FAILED);
        LogProducer::error("comm", "handleRequest: can't allocate memory for response_data");
        delete request_data_ptr;
        return;
    }

    handleRequestPackage(0x00002940, (void *)request_data_ptr, (void *)response_data_ptr,
                         recv_bytes, RequestMessageType_Int32_Double_fields, -1);
}

//"/rpc/motion_control/axis_group/getOrientationManualStep"
void TpComm::handleRequest0x00016D20(int recv_bytes)
{
    // create object for request and response package
    RequestMessageType_Int32 *request_data_ptr = new RequestMessageType_Int32;
    if (request_data_ptr == NULL)
    {
        ErrorQueue::instance().push(TP_COMM_MEMORY_OPERATION_FAILED);
        LogProducer::error("comm", "handleRequest: can't allocate memory for request_data");
        return;
    }
    ResponseMessageType_Uint64_Double *response_data_ptr = new ResponseMessageType_Uint64_Double;
    if (response_data_ptr == NULL)
    {
        ErrorQueue::instance().push(TP_COMM_MEMORY_OPERATION_FAILED);
        LogProducer::error("comm", "handleRequest: can't allocate memory for response_data");
        delete request_data_ptr;
        return;
    }

    handleRequestPackage(0x00016D20, (void *)request_data_ptr, (void *)response_data_ptr,
                         recv_bytes, RequestMessageType_Int32_fields, -1);
}

//"/rpc/motion_control/axis_group/getFcpBasePose"
void TpComm::handleRequest0x000016B5(int recv_bytes)
{
    // create object for request and response package
    RequestMessageType_Int32 *request_data_ptr = new RequestMessageType_Int32;
    if (request_data_ptr == NULL)
    {
        ErrorQueue::instance().push(TP_COMM_MEMORY_OPERATION_FAILED);
        LogProducer::error("comm", "handleRequest: can't allocate memory for request_data");
        return;
    }
    ResponseMessageType_Uint64_DoubleList *response_data_ptr = new ResponseMessageType_Uint64_DoubleList;
    if (response_data_ptr == NULL)
    {
        ErrorQueue::instance().push(TP_COMM_MEMORY_OPERATION_FAILED);
        LogProducer::error("comm", "handleRequest: can't allocate memory for response_data");
        delete request_data_ptr;
        return;
    }

    handleRequestPackage(0x000016B5, (void *)request_data_ptr, (void *)response_data_ptr,
                         recv_bytes, RequestMessageType_Int32_fields, -1);
}

//"/rpc/motion_control/axis_group/getTcpCurrentPose"
void TpComm::handleRequest0x00003B45(int recv_bytes)
{
    // create object for request and response package
    RequestMessageType_Int32List *request_data_ptr = new RequestMessageType_Int32List;
    if (request_data_ptr == NULL)
    {
        ErrorQueue::instance().push(TP_COMM_MEMORY_OPERATION_FAILED);
        LogProducer::error("comm", "handleRequest: can't allocate memory for request_data");
        return;
    }
    ResponseMessageType_Uint64_DoubleList *response_data_ptr = new ResponseMessageType_Uint64_DoubleList;
    if (response_data_ptr == NULL)
    {
        ErrorQueue::instance().push(TP_COMM_MEMORY_OPERATION_FAILED);
        LogProducer::error("comm", "handleRequest: can't allocate memory for response_data");
        delete request_data_ptr;
        return;
    }

    handleRequestPackage(0x00003B45, (void *)request_data_ptr, (void *)response_data_ptr,
                         recv_bytes, RequestMessageType_Int32List_fields, -1);
}

//"/rpc/motion_control/getPostureByJoint"
void TpComm::handleRequest0x0000EC64(int recv_bytes)
{
    // create object for request and response package
    RequestMessageType_Int32_DoubleList *request_data_ptr = new RequestMessageType_Int32_DoubleList;
    if (request_data_ptr == NULL)
    {
        ErrorQueue::instance().push(TP_COMM_MEMORY_OPERATION_FAILED);
        LogProducer::error("comm", "handleRequest: can't allocate memory for request_data");
        return;
    }
    ResponseMessageType_Uint64_Posture *response_data_ptr = new ResponseMessageType_Uint64_Posture;
    if (response_data_ptr == NULL)
    {
        ErrorQueue::instance().push(TP_COMM_MEMORY_OPERATION_FAILED);
        LogProducer::error("comm", "handleRequest: can't allocate memory for response_data");
        delete request_data_ptr;
        return;
    }

    handleRequestPackage(0x0000EC64, (void *)request_data_ptr, (void *)response_data_ptr,
                         recv_bytes, RequestMessageType_Int32_DoubleList_fields, -1);
}
/********rpc/motion_control/axis_group/convertEulerTraj2JointFile, RequestMessageType_String**********/	
void TpComm::handleRequest0x0000E375(int recv_bytes)
{
    // create object for request and response package
    RequestMessageType_String *request_data_ptr = new RequestMessageType_String;
    if (request_data_ptr == NULL)
    {
        ErrorQueue::instance().push(TP_COMM_MEMORY_OPERATION_FAILED);
        LogProducer::error("comm", "handleRequest: can't allocate memory for request_data");
        return;
    }
    ResponseMessageType_Uint64* response_data_ptr = new ResponseMessageType_Uint64;
    if (response_data_ptr == NULL)
    {
        ErrorQueue::instance().push(TP_COMM_MEMORY_OPERATION_FAILED);
        LogProducer::error("comm", "handleRequest: can't allocate memory for response_data");
        delete request_data_ptr;
        return;
    }
    handleRequestPackage(0x0000E375, (void *)request_data_ptr, (void *)response_data_ptr,
                         recv_bytes, RequestMessageType_String_fields, -1);
}

//"/rpc/motion_control/axis_group/setOfflineTrajectoryFile"
void TpComm::handleRequest0x00011275(int recv_bytes)
{
    // create object for request and response package
    RequestMessageType_String *request_data_ptr = new RequestMessageType_String;
    if (request_data_ptr == NULL)
    {
        ErrorQueue::instance().push(TP_COMM_MEMORY_OPERATION_FAILED);
        LogProducer::error("comm", "handleRequest: can't allocate memory for request_data");
        return;
    }
    ResponseMessageType_Uint64* response_data_ptr = new ResponseMessageType_Uint64;
    if (response_data_ptr == NULL)
    {
        ErrorQueue::instance().push(TP_COMM_MEMORY_OPERATION_FAILED);
        LogProducer::error("comm", "handleRequest: can't allocate memory for response_data");
        delete request_data_ptr;
        return;
    }

    handleRequestPackage(0x00011275, (void *)request_data_ptr, (void *)response_data_ptr,
                         recv_bytes, RequestMessageType_String_fields, -1);
}

//"/rpc/motion_control/axis_group/PrepareOfflineTrajectory"
void TpComm::handleRequest0x000051E9(int recv_bytes)
{
    // create object for request and response package
    RequestMessageType_Void *request_data_ptr = new RequestMessageType_Void;
    if (request_data_ptr == NULL)
    {
        ErrorQueue::instance().push(TP_COMM_MEMORY_OPERATION_FAILED);
        LogProducer::error("comm", "handleRequest: can't allocate memory for request_data");
        return;
    }
    ResponseMessageType_Uint64* response_data_ptr = new ResponseMessageType_Uint64;
    if (response_data_ptr == NULL)
    {
        ErrorQueue::instance().push(TP_COMM_MEMORY_OPERATION_FAILED);
        LogProducer::error("comm", "handleRequest: can't allocate memory for response_data");
        delete request_data_ptr;
        return;
    }

    handleRequestPackage(0x000051E9, (void *)request_data_ptr, (void *)response_data_ptr,
                         recv_bytes, RequestMessageType_Void_fields, -1);
}

//"/rpc/motion_control/axis_group/moveOfflineTrajectory"
void TpComm::handleRequest0x0000C4D9(int recv_bytes)
{
    // create object for request and response package
    RequestMessageType_Void *request_data_ptr = new RequestMessageType_Void;
    if (request_data_ptr == NULL)
    {
        ErrorQueue::instance().push(TP_COMM_MEMORY_OPERATION_FAILED);
        LogProducer::error("comm", "handleRequest: can't allocate memory for request_data");
        return;
    }
    ResponseMessageType_Uint64* response_data_ptr = new ResponseMessageType_Uint64;
    if (response_data_ptr == NULL)
    {
        ErrorQueue::instance().push(TP_COMM_MEMORY_OPERATION_FAILED);
        LogProducer::error("comm", "handleRequest: can't allocate memory for response_data");
        delete request_data_ptr;
        return;
    }

    handleRequestPackage(0x0000C4D9, (void *)request_data_ptr, (void *)response_data_ptr,
                         recv_bytes, RequestMessageType_Void_fields, -1);
}

