#include <pb_encode.h>
#include <pb_decode.h>
#include <pb_common.h>

#include "error_monitor.h"
#include "error_code.h"
#include "tp_comm.h"

using namespace fst_base;
using namespace fst_comm;

//"/rpc/motion_control/stop"
void TpComm::handleRequest0x00001E70(int recv_bytes)
{
    // create object for request and response package
    RequestMessageType_Void *request_data_ptr = new RequestMessageType_Void;
    if (request_data_ptr == NULL)
    {
        ErrorMonitor::instance()->add(TP_COMM_MEMORY_OPERATION_FAILED);
        FST_ERROR("handleRequest: can't allocate memory for request_data");
        return;
    }
    ResponseMessageType_Uint64 *response_data_ptr = new ResponseMessageType_Uint64;
    if (response_data_ptr == NULL)
    {
        ErrorMonitor::instance()->add(TP_COMM_MEMORY_OPERATION_FAILED);
        FST_ERROR("handleRequest: can't allocate memory for response_data");
        delete request_data_ptr;
        return;
    }

    handleRequestPackage(0x00001E70, (void *)request_data_ptr, (void *)response_data_ptr,
                         recv_bytes, RequestMessageType_Void_fields, -1);
}

//"/rpc/motion_control/reset"
void TpComm::handleRequest0x00001D14(int recv_bytes)
{
    // create object for request and response package
    RequestMessageType_Void *request_data_ptr = new RequestMessageType_Void;
    if (request_data_ptr == NULL)
    {
        ErrorMonitor::instance()->add(TP_COMM_MEMORY_OPERATION_FAILED);
        FST_ERROR("handleRequest: can't allocate memory for request_data");
        return;
    }
    ResponseMessageType_Uint64 *response_data_ptr = new ResponseMessageType_Uint64;
    if (response_data_ptr == NULL)
    {
        ErrorMonitor::instance()->add(TP_COMM_MEMORY_OPERATION_FAILED);
        FST_ERROR("handleRequest: can't allocate memory for response_data");
        delete request_data_ptr;
        return;
    }

    handleRequestPackage(0x00001D14, (void *)request_data_ptr, (void *)response_data_ptr,
                         recv_bytes, RequestMessageType_Void_fields, -1);
}

//"/rpc/motion_control/axis_group/doStepManualMove"
void TpComm::handleRequest0x000085D5(int recv_bytes)
{
    // create object for request and response package
    RequestMessageType_Int32_Int32List *request_data_ptr = new RequestMessageType_Int32_Int32List;
    if (request_data_ptr == NULL)
    {
        ErrorMonitor::instance()->add(TP_COMM_MEMORY_OPERATION_FAILED);
        FST_ERROR("handleRequest: can't allocate memory for request_data");
        return;
    }
    ResponseMessageType_Uint64 *response_data_ptr = new ResponseMessageType_Uint64;
    if (response_data_ptr == NULL)
    {
        ErrorMonitor::instance()->add(TP_COMM_MEMORY_OPERATION_FAILED);
        FST_ERROR("handleRequest: can't allocate memory for response_data");
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
        ErrorMonitor::instance()->add(TP_COMM_MEMORY_OPERATION_FAILED);
        FST_ERROR("handleRequest: can't allocate memory for request_data");
        return;
    }
    ResponseMessageType_Uint64 *response_data_ptr = new ResponseMessageType_Uint64;
    if (response_data_ptr == NULL)
    {
        ErrorMonitor::instance()->add(TP_COMM_MEMORY_OPERATION_FAILED);
        FST_ERROR("handleRequest: can't allocate memory for response_data");
        delete request_data_ptr;
        return;
    }

    handleRequestPackage(0x0000D3F5, (void *)request_data_ptr, (void *)response_data_ptr,
                         recv_bytes, RequestMessageType_Int32_Int32List_fields, -1);
}

//"/rpc/motion_control/axis_group/getJointsFeedBack"
void TpComm::handleRequest0x0000DFBB(int recv_bytes)
{
    // create object for request and response package
    RequestMessageType_Int32 *request_data_ptr = new RequestMessageType_Int32;
    if (request_data_ptr == NULL)
    {
        ErrorMonitor::instance()->add(TP_COMM_MEMORY_OPERATION_FAILED);
        FST_ERROR("handleRequest: can't allocate memory for request_data");
        return;
    }
    ResponseMessageType_Uint64_DoubleList *response_data_ptr = new ResponseMessageType_Uint64_DoubleList;
    if (response_data_ptr == NULL)
    {
        ErrorMonitor::instance()->add(TP_COMM_MEMORY_OPERATION_FAILED);
        FST_ERROR("handleRequest: can't allocate memory for response_data");
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
        ErrorMonitor::instance()->add(TP_COMM_MEMORY_OPERATION_FAILED);
        FST_ERROR("handleRequest: can't allocate memory for request_data");
        return;
    }
    ResponseMessageType_Uint64 *response_data_ptr = new ResponseMessageType_Uint64;
    if (response_data_ptr == NULL)
    {
        ErrorMonitor::instance()->add(TP_COMM_MEMORY_OPERATION_FAILED);
        FST_ERROR("handleRequest: can't allocate memory for response_data");
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
        ErrorMonitor::instance()->add(TP_COMM_MEMORY_OPERATION_FAILED);
        FST_ERROR("handleRequest: can't allocate memory for request_data");
        return;
    }
    ResponseMessageType_Uint64_JointLimit *response_data_ptr = new ResponseMessageType_Uint64_JointLimit;
    if (response_data_ptr == NULL)
    {
        ErrorMonitor::instance()->add(TP_COMM_MEMORY_OPERATION_FAILED);
        FST_ERROR("handleRequest: can't allocate memory for response_data");
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
        ErrorMonitor::instance()->add(TP_COMM_MEMORY_OPERATION_FAILED);
        FST_ERROR("handleRequest: can't allocate memory for request_data");
        return;
    }
    ResponseMessageType_Uint64 *response_data_ptr = new ResponseMessageType_Uint64;
    if (response_data_ptr == NULL)
    {
        ErrorMonitor::instance()->add(TP_COMM_MEMORY_OPERATION_FAILED);
        FST_ERROR("handleRequest: can't allocate memory for response_data");
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
        ErrorMonitor::instance()->add(TP_COMM_MEMORY_OPERATION_FAILED);
        FST_ERROR("handleRequest: can't allocate memory for request_data");
        return;
    }
    ResponseMessageType_Uint64_JointLimit *response_data_ptr = new ResponseMessageType_Uint64_JointLimit;
    if (response_data_ptr == NULL)
    {
        ErrorMonitor::instance()->add(TP_COMM_MEMORY_OPERATION_FAILED);
        FST_ERROR("handleRequest: can't allocate memory for response_data");
        delete request_data_ptr;
        return;
    }

    handleRequestPackage(0x0000C244, (void *)request_data_ptr, (void *)response_data_ptr,
                         recv_bytes, RequestMessageType_Int32_fields, -1);
}

//"/rpc/motion_control/axis_group/doGotoCartesianPointManualMove"
void TpComm::handleRequest0x00010C05(int recv_bytes)
{
    // create object for request and response package
    RequestMessageType_Int32_DoubleList *request_data_ptr = new RequestMessageType_Int32_DoubleList;
    if (request_data_ptr == NULL)
    {
        ErrorMonitor::instance()->add(TP_COMM_MEMORY_OPERATION_FAILED);
        FST_ERROR("handleRequest: can't allocate memory for request_data");
        return;
    }
    ResponseMessageType_Uint64 *response_data_ptr = new ResponseMessageType_Uint64;
    if (response_data_ptr == NULL)
    {
        ErrorMonitor::instance()->add(TP_COMM_MEMORY_OPERATION_FAILED);
        FST_ERROR("handleRequest: can't allocate memory for response_data");
        delete request_data_ptr;
        return;
    }

    handleRequestPackage(0x00010C05, (void *)request_data_ptr, (void *)response_data_ptr,
                         recv_bytes, RequestMessageType_Int32_DoubleList_fields, -1);
}

//"/rpc/motion_control/axis_group/doGotoJointPointManualMove"
void TpComm::handleRequest0x00008075(int recv_bytes)
{
    // create object for request and response package
    RequestMessageType_Int32_DoubleList *request_data_ptr = new RequestMessageType_Int32_DoubleList;
    if (request_data_ptr == NULL)
    {
        ErrorMonitor::instance()->add(TP_COMM_MEMORY_OPERATION_FAILED);
        FST_ERROR("handleRequest: can't allocate memory for request_data");
        return;
    }
    ResponseMessageType_Uint64 *response_data_ptr = new ResponseMessageType_Uint64;
    if (response_data_ptr == NULL)
    {
        ErrorMonitor::instance()->add(TP_COMM_MEMORY_OPERATION_FAILED);
        FST_ERROR("handleRequest: can't allocate memory for response_data");
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
        ErrorMonitor::instance()->add(TP_COMM_MEMORY_OPERATION_FAILED);
        FST_ERROR("handleRequest: can't allocate memory for request_data");
        return;
    }
    ResponseMessageType_Uint64 *response_data_ptr = new ResponseMessageType_Uint64;
    if (response_data_ptr == NULL)
    {
        ErrorMonitor::instance()->add(TP_COMM_MEMORY_OPERATION_FAILED);
        FST_ERROR("handleRequest: can't allocate memory for response_data");
        delete request_data_ptr;
        return;
    }

    handleRequestPackage(0x0000A9A0, (void *)request_data_ptr, (void *)response_data_ptr,
                         recv_bytes, RequestMessageType_Int32_fields, -1);
}

//"/rpc/motion_control/setGlobalVelRatio"
void TpComm::handleRequest0x000005EF(int recv_bytes)
{
    // create object for request and response package
    RequestMessageType_Double *request_data_ptr = new RequestMessageType_Double;
    if (request_data_ptr == NULL)
    {
        ErrorMonitor::instance()->add(TP_COMM_MEMORY_OPERATION_FAILED);
        FST_ERROR("handleRequest: can't allocate memory for request_data");
        return;
    }
    ResponseMessageType_Uint64 *response_data_ptr = new ResponseMessageType_Uint64;
    if (response_data_ptr == NULL)
    {
        ErrorMonitor::instance()->add(TP_COMM_MEMORY_OPERATION_FAILED);
        FST_ERROR("handleRequest: can't allocate memory for response_data");
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
        ErrorMonitor::instance()->add(TP_COMM_MEMORY_OPERATION_FAILED);
        FST_ERROR("handleRequest: can't allocate memory for request_data");
        return;
    }
    ResponseMessageType_Uint64_Double *response_data_ptr = new ResponseMessageType_Uint64_Double;
    if (response_data_ptr == NULL)
    {
        ErrorMonitor::instance()->add(TP_COMM_MEMORY_OPERATION_FAILED);
        FST_ERROR("handleRequest: can't allocate memory for response_data");
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
        ErrorMonitor::instance()->add(TP_COMM_MEMORY_OPERATION_FAILED);
        FST_ERROR("handleRequest: can't allocate memory for request_data");
        return;
    }
    ResponseMessageType_Uint64 *response_data_ptr = new ResponseMessageType_Uint64;
    if (response_data_ptr == NULL)
    {
        ErrorMonitor::instance()->add(TP_COMM_MEMORY_OPERATION_FAILED);
        FST_ERROR("handleRequest: can't allocate memory for response_data");
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
        ErrorMonitor::instance()->add(TP_COMM_MEMORY_OPERATION_FAILED);
        FST_ERROR("handleRequest: can't allocate memory for request_data");
        return;
    }
    ResponseMessageType_Uint64_Double *response_data_ptr = new ResponseMessageType_Uint64_Double;
    if (response_data_ptr == NULL)
    {
        ErrorMonitor::instance()->add(TP_COMM_MEMORY_OPERATION_FAILED);
        FST_ERROR("handleRequest: can't allocate memory for response_data");
        delete request_data_ptr;
        return;
    }

    handleRequestPackage(0x00016D9F, (void *)request_data_ptr, (void *)response_data_ptr,
                         recv_bytes, RequestMessageType_Void_fields, -1);
}

//"/rpc/motion_control/axis_group/setHardLimit"
void TpComm::handleRequest0x0000C454(int recv_bytes)
{
    // create object for request and response package
    RequestMessageType_Int32_JointLimit *request_data_ptr = new RequestMessageType_Int32_JointLimit;
    if (request_data_ptr == NULL)
    {
        ErrorMonitor::instance()->add(TP_COMM_MEMORY_OPERATION_FAILED);
        FST_ERROR("handleRequest: can't allocate memory for request_data");
        return;
    }
    ResponseMessageType_Uint64 *response_data_ptr = new ResponseMessageType_Uint64;
    if (response_data_ptr == NULL)
    {
        ErrorMonitor::instance()->add(TP_COMM_MEMORY_OPERATION_FAILED);
        FST_ERROR("handleRequest: can't allocate memory for response_data");
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
        ErrorMonitor::instance()->add(TP_COMM_MEMORY_OPERATION_FAILED);
        FST_ERROR("handleRequest: can't allocate memory for request_data");
        return;
    }
    ResponseMessageType_Uint64_JointLimit *response_data_ptr = new ResponseMessageType_Uint64_JointLimit;
    if (response_data_ptr == NULL)
    {
        ErrorMonitor::instance()->add(TP_COMM_MEMORY_OPERATION_FAILED);
        FST_ERROR("handleRequest: can't allocate memory for response_data");
        delete request_data_ptr;
        return;
    }

    handleRequestPackage(0x00013394, (void *)request_data_ptr, (void *)response_data_ptr,
                         recv_bytes, RequestMessageType_Int32_fields, -1);
}

//"/rpc/motion_control/axis_group/getCoordinate"
void TpComm::handleRequest0x00008595(int recv_bytes)
{
    // create object for request and response package
    RequestMessageType_Int32 *request_data_ptr = new RequestMessageType_Int32;
    if (request_data_ptr == NULL)
    {
        ErrorMonitor::instance()->add(TP_COMM_MEMORY_OPERATION_FAILED);
        FST_ERROR("handleRequest: can't allocate memory for request_data");
        return;
    }
    ResponseMessageType_Uint64_Int32 *response_data_ptr = new ResponseMessageType_Uint64_Int32;
    if (response_data_ptr == NULL)
    {
        ErrorMonitor::instance()->add(TP_COMM_MEMORY_OPERATION_FAILED);
        FST_ERROR("handleRequest: can't allocate memory for response_data");
        delete request_data_ptr;
        return;
    }

    handleRequestPackage(0x00008595, (void *)request_data_ptr, (void *)response_data_ptr,
                         recv_bytes, RequestMessageType_Int32_fields, -1);
}

//"/rpc/motion_control/axis_group/setCoordinate"
void TpComm::handleRequest0x0000A845(int recv_bytes)
{
    // create object for request and response package
    RequestMessageType_Int32List *request_data_ptr = new RequestMessageType_Int32List;
    if (request_data_ptr == NULL)
    {
        ErrorMonitor::instance()->add(TP_COMM_MEMORY_OPERATION_FAILED);
        FST_ERROR("handleRequest: can't allocate memory for request_data");
        return;
    }
    ResponseMessageType_Uint64 *response_data_ptr = new ResponseMessageType_Uint64;
    if (response_data_ptr == NULL)
    {
        ErrorMonitor::instance()->add(TP_COMM_MEMORY_OPERATION_FAILED);
        FST_ERROR("handleRequest: can't allocate memory for response_data");
        delete request_data_ptr;
        return;
    }

    handleRequestPackage(0x0000A845, (void *)request_data_ptr, (void *)response_data_ptr,
                         recv_bytes, RequestMessageType_Int32List_fields, -1);
}

//"/rpc/motion_control/axis_group/getUserCoordId"
void TpComm::handleRequest0x00005BB4(int recv_bytes)
{
    // create object for request and response package
    RequestMessageType_Int32 *request_data_ptr = new RequestMessageType_Int32;
    if (request_data_ptr == NULL)
    {
        ErrorMonitor::instance()->add(TP_COMM_MEMORY_OPERATION_FAILED);
        FST_ERROR("handleRequest: can't allocate memory for request_data");
        return;
    }
    ResponseMessageType_Uint64_Int32 *response_data_ptr = new ResponseMessageType_Uint64_Int32;
    if (response_data_ptr == NULL)
    {
        ErrorMonitor::instance()->add(TP_COMM_MEMORY_OPERATION_FAILED);
        FST_ERROR("handleRequest: can't allocate memory for response_data");
        delete request_data_ptr;
        return;
    }

    handleRequestPackage(0x00005BB4, (void *)request_data_ptr, (void *)response_data_ptr,
                         recv_bytes, RequestMessageType_Int32_fields, -1);
}

//"/rpc/motion_control/axis_group/setUserCoordId"
void TpComm::handleRequest0x00005CF4(int recv_bytes)
{
    // create object for request and response package
    RequestMessageType_Int32List *request_data_ptr = new RequestMessageType_Int32List;
    if (request_data_ptr == NULL)
    {
        ErrorMonitor::instance()->add(TP_COMM_MEMORY_OPERATION_FAILED);
        FST_ERROR("handleRequest: can't allocate memory for request_data");
        return;
    }
    ResponseMessageType_Uint64 *response_data_ptr = new ResponseMessageType_Uint64;
    if (response_data_ptr == NULL)
    {
        ErrorMonitor::instance()->add(TP_COMM_MEMORY_OPERATION_FAILED);
        FST_ERROR("handleRequest: can't allocate memory for response_data");
        delete request_data_ptr;
        return;
    }

    handleRequestPackage(0x00005CF4, (void *)request_data_ptr, (void *)response_data_ptr,
                         recv_bytes, RequestMessageType_Int32List_fields, -1);
}

//"/rpc/motion_control/axis_group/getTool"
void TpComm::handleRequest0x0001354C(int recv_bytes)
{
    // create object for request and response package
    RequestMessageType_Int32 *request_data_ptr = new RequestMessageType_Int32;
    if (request_data_ptr == NULL)
    {
        ErrorMonitor::instance()->add(TP_COMM_MEMORY_OPERATION_FAILED);
        FST_ERROR("handleRequest: can't allocate memory for request_data");
        return;
    }
    ResponseMessageType_Uint64_Int32 *response_data_ptr = new ResponseMessageType_Uint64_Int32;
    if (response_data_ptr == NULL)
    {
        ErrorMonitor::instance()->add(TP_COMM_MEMORY_OPERATION_FAILED);
        FST_ERROR("handleRequest: can't allocate memory for response_data");
        delete request_data_ptr;
        return;
    }

    handleRequestPackage(0x0001354C, (void *)request_data_ptr, (void *)response_data_ptr,
                         recv_bytes, RequestMessageType_Int32_fields, -1);
}

//"/rpc/motion_control/axis_group/setTool"
void TpComm::handleRequest0x0001581C(int recv_bytes)
{
    // create object for request and response package
    RequestMessageType_Int32List *request_data_ptr = new RequestMessageType_Int32List;
    if (request_data_ptr == NULL)
    {
        ErrorMonitor::instance()->add(TP_COMM_MEMORY_OPERATION_FAILED);
        FST_ERROR("handleRequest: can't allocate memory for request_data");
        return;
    }
    ResponseMessageType_Uint64 *response_data_ptr = new ResponseMessageType_Uint64;
    if (response_data_ptr == NULL)
    {
        ErrorMonitor::instance()->add(TP_COMM_MEMORY_OPERATION_FAILED);
        FST_ERROR("handleRequest: can't allocate memory for response_data");
        delete request_data_ptr;
        return;
    }

    handleRequestPackage(0x0001581C, (void *)request_data_ptr, (void *)response_data_ptr,
                         recv_bytes, RequestMessageType_Int32List_fields, -1);
}

//"/rpc/motion_control/axis_group/convertCartToJoint"
void TpComm::handleRequest0x00010FD4(int recv_bytes)
{
    // create object for request and response package
    RequestMessageType_Int32List_DoubleList *request_data_ptr = new RequestMessageType_Int32List_DoubleList;
    if (request_data_ptr == NULL)
    {
        ErrorMonitor::instance()->add(TP_COMM_MEMORY_OPERATION_FAILED);
        FST_ERROR("handleRequest: can't allocate memory for request_data");
        return;
    }
    ResponseMessageType_Uint64_DoubleList *response_data_ptr = new ResponseMessageType_Uint64_DoubleList;
    if (response_data_ptr == NULL)
    {
        ErrorMonitor::instance()->add(TP_COMM_MEMORY_OPERATION_FAILED);
        FST_ERROR("handleRequest: can't allocate memory for response_data");
        delete request_data_ptr;
        return;
    }

    handleRequestPackage(0x00010FD4, (void *)request_data_ptr, (void *)response_data_ptr,
                         recv_bytes, RequestMessageType_Int32List_DoubleList_fields, -1);
}

//"/rpc/motion_control/axis_group/convertJointToCart",
void TpComm::handleRequest0x0000B6D4(int recv_bytes)
{
    // create object for request and response package
    RequestMessageType_Int32List_DoubleList *request_data_ptr = new RequestMessageType_Int32List_DoubleList;
    if (request_data_ptr == NULL)
    {
        ErrorMonitor::instance()->add(TP_COMM_MEMORY_OPERATION_FAILED);
        FST_ERROR("handleRequest: can't allocate memory for request_data");
        return;
    }
    ResponseMessageType_Uint64_DoubleList *response_data_ptr = new ResponseMessageType_Uint64_DoubleList;
    if (response_data_ptr == NULL)
    {
        ErrorMonitor::instance()->add(TP_COMM_MEMORY_OPERATION_FAILED);
        FST_ERROR("handleRequest: can't allocate memory for response_data");
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
        ErrorMonitor::instance()->add(TP_COMM_MEMORY_OPERATION_FAILED);
        FST_ERROR("handleRequest: can't allocate memory for request_data");
        return;
    }
    ResponseMessageType_Uint64 *response_data_ptr = new ResponseMessageType_Uint64;
    if (response_data_ptr == NULL)
    {
        ErrorMonitor::instance()->add(TP_COMM_MEMORY_OPERATION_FAILED);
        FST_ERROR("handleRequest: can't allocate memory for response_data");
        delete request_data_ptr;
        return;
    }

    handleRequestPackage(0x00014952, (void *)request_data_ptr, (void *)response_data_ptr,
                         recv_bytes, RequestMessageType_Int32_fields, -1);
}

//"/rpc/motion_control/axis_group/setAllZeroPointOffsets"
void TpComm::handleRequest0x00008AB4(int recv_bytes)
{
    // create object for request and response package
    RequestMessageType_Int32_DoubleList *request_data_ptr = new RequestMessageType_Int32_DoubleList;
    if (request_data_ptr == NULL)
    {
        ErrorMonitor::instance()->add(TP_COMM_MEMORY_OPERATION_FAILED);
        FST_ERROR("handleRequest: can't allocate memory for request_data");
        return;
    }
    ResponseMessageType_Uint64 *response_data_ptr = new ResponseMessageType_Uint64;
    if (response_data_ptr == NULL)
    {
        ErrorMonitor::instance()->add(TP_COMM_MEMORY_OPERATION_FAILED);
        FST_ERROR("handleRequest: can't allocate memory for response_data");
        delete request_data_ptr;
        return;
    }

    handleRequestPackage(0x00008AB4, (void *)request_data_ptr, (void *)response_data_ptr,
                         recv_bytes, RequestMessageType_Int32_DoubleList_fields, -1);
}

//"/rpc/motion_control/axis_group/getAllZeroPointOffsets"
void TpComm::handleRequest0x00012353(int recv_bytes)
{
    // create object for request and response package
    RequestMessageType_Int32 *request_data_ptr = new RequestMessageType_Int32;
    if (request_data_ptr == NULL)
    {
        ErrorMonitor::instance()->add(TP_COMM_MEMORY_OPERATION_FAILED);
        FST_ERROR("handleRequest: can't allocate memory for request_data");
        return;
    }
    ResponseMessageType_Uint64_DoubleList *response_data_ptr = new ResponseMessageType_Uint64_DoubleList;
    if (response_data_ptr == NULL)
    {
        ErrorMonitor::instance()->add(TP_COMM_MEMORY_OPERATION_FAILED);
        FST_ERROR("handleRequest: can't allocate memory for response_data");
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
        ErrorMonitor::instance()->add(TP_COMM_MEMORY_OPERATION_FAILED);
        FST_ERROR("handleRequest: can't allocate memory for request_data");
        return;
    }
    ResponseMessageType_Uint64_Int32List *response_data_ptr = new ResponseMessageType_Uint64_Int32List;
    if (response_data_ptr == NULL)
    {
        ErrorMonitor::instance()->add(TP_COMM_MEMORY_OPERATION_FAILED);
        FST_ERROR("handleRequest: can't allocate memory for response_data");
        delete request_data_ptr;
        return;
    }

    handleRequestPackage(0x0000C183, (void *)request_data_ptr, (void *)response_data_ptr,
                         recv_bytes, RequestMessageType_Int32_fields, -1);
}

//"/rpc/motion_control/axis_group/saveAllZeroPointOffsets"
void TpComm::handleRequest0x000171D3(int recv_bytes)
{
    // create object for request and response package
    RequestMessageType_Int32 *request_data_ptr = new RequestMessageType_Int32;
    if (request_data_ptr == NULL)
    {
        ErrorMonitor::instance()->add(TP_COMM_MEMORY_OPERATION_FAILED);
        FST_ERROR("handleRequest: can't allocate memory for request_data");
        return;
    }
    ResponseMessageType_Uint64 *response_data_ptr = new ResponseMessageType_Uint64;
    if (response_data_ptr == NULL)
    {
        ErrorMonitor::instance()->add(TP_COMM_MEMORY_OPERATION_FAILED);
        FST_ERROR("handleRequest: can't allocate memory for response_data");
        delete request_data_ptr;
        return;
    }

    handleRequestPackage(0x000171D3, (void *)request_data_ptr, (void *)response_data_ptr,
                         recv_bytes, RequestMessageType_Int32_fields, -1);
}

//"/rpc/motion_control/axis_group/setSingleZeroPointStatus"
void TpComm::handleRequest0x00010E43(int recv_bytes)
{
    // create object for request and response package
    RequestMessageType_Int32List *request_data_ptr = new RequestMessageType_Int32List;
    if (request_data_ptr == NULL)
    {
        ErrorMonitor::instance()->add(TP_COMM_MEMORY_OPERATION_FAILED);
        FST_ERROR("handleRequest: can't allocate memory for request_data");
        return;
    }
    ResponseMessageType_Uint64 *response_data_ptr = new ResponseMessageType_Uint64;
    if (response_data_ptr == NULL)
    {
        ErrorMonitor::instance()->add(TP_COMM_MEMORY_OPERATION_FAILED);
        FST_ERROR("handleRequest: can't allocate memory for response_data");
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
        ErrorMonitor::instance()->add(TP_COMM_MEMORY_OPERATION_FAILED);
        FST_ERROR("handleRequest: can't allocate memory for request_data");
        return;
    }
    ResponseMessageType_Uint64_Int32List *response_data_ptr = new ResponseMessageType_Uint64_Int32List;
    if (response_data_ptr == NULL)
    {
        ErrorMonitor::instance()->add(TP_COMM_MEMORY_OPERATION_FAILED);
        FST_ERROR("handleRequest: can't allocate memory for response_data");
        delete request_data_ptr;
        return;
    }

    handleRequestPackage(0x000102F3, (void *)request_data_ptr, (void *)response_data_ptr,
                         recv_bytes, RequestMessageType_Int32_fields, -1);
}

//"/rpc/motion_control/axis_group/calibrateAllZeroPointOffsets"
void TpComm::handleRequest0x00011B03(int recv_bytes)
{
    // create object for request and response package
    RequestMessageType_Int32 *request_data_ptr = new RequestMessageType_Int32;
    if (request_data_ptr == NULL)
    {
        ErrorMonitor::instance()->add(TP_COMM_MEMORY_OPERATION_FAILED);
        FST_ERROR("handleRequest: can't allocate memory for request_data");
        return;
    }
    ResponseMessageType_Uint64 *response_data_ptr = new ResponseMessageType_Uint64;
    if (response_data_ptr == NULL)
    {
        ErrorMonitor::instance()->add(TP_COMM_MEMORY_OPERATION_FAILED);
        FST_ERROR("handleRequest: can't allocate memory for response_data");
        delete request_data_ptr;
        return;
    }

    handleRequestPackage(0x00011B03, (void *)request_data_ptr, (void *)response_data_ptr,
                         recv_bytes, RequestMessageType_Int32_fields, -1);
}

//"/rpc/motion_control/axis_group/calibrateSingleZeroPointOffset"
void TpComm::handleRequest0x000131D4(int recv_bytes)
{
    // create object for request and response package
    RequestMessageType_Int32List *request_data_ptr = new RequestMessageType_Int32List;
    if (request_data_ptr == NULL)
    {
        ErrorMonitor::instance()->add(TP_COMM_MEMORY_OPERATION_FAILED);
        FST_ERROR("handleRequest: can't allocate memory for request_data");
        return;
    }
    ResponseMessageType_Uint64 *response_data_ptr = new ResponseMessageType_Uint64;
    if (response_data_ptr == NULL)
    {
        ErrorMonitor::instance()->add(TP_COMM_MEMORY_OPERATION_FAILED);
        FST_ERROR("handleRequest: can't allocate memory for response_data");
        delete request_data_ptr;
        return;
    }

    handleRequestPackage(0x000131D4, (void *)request_data_ptr, (void *)response_data_ptr,
                         recv_bytes, RequestMessageType_Int32List_fields, -1);
}

//"/rpc/motion_control/axis_group/calibrateZeroPointOffsets"
void TpComm::handleRequest0x00005AE3(int recv_bytes)
{
    // create object for request and response package
    RequestMessageType_Int32_Int32List *request_data_ptr = new RequestMessageType_Int32_Int32List;
    if (request_data_ptr == NULL)
    {
        ErrorMonitor::instance()->add(TP_COMM_MEMORY_OPERATION_FAILED);
        FST_ERROR("handleRequest: can't allocate memory for request_data");
        return;
    }
    ResponseMessageType_Uint64 *response_data_ptr = new ResponseMessageType_Uint64;
    if (response_data_ptr == NULL)
    {
        ErrorMonitor::instance()->add(TP_COMM_MEMORY_OPERATION_FAILED);
        FST_ERROR("handleRequest: can't allocate memory for response_data");
        delete request_data_ptr;
        return;
    }

    handleRequestPackage(0x00005AE3, (void *)request_data_ptr, (void *)response_data_ptr,
                         recv_bytes, RequestMessageType_Int32_Int32List_fields, -1);
}

//"/rpc/motion_control/axis_group/isReferencePointExist"
void TpComm::handleRequest0x0000D344(int recv_bytes)
{
    // create object for request and response package
    RequestMessageType_Int32 *request_data_ptr = new RequestMessageType_Int32;
    if (request_data_ptr == NULL)
    {
        ErrorMonitor::instance()->add(TP_COMM_MEMORY_OPERATION_FAILED);
        FST_ERROR("handleRequest: can't allocate memory for request_data");
        return;
    }
    ResponseMessageType_Uint64_Bool *response_data_ptr = new ResponseMessageType_Uint64_Bool;
    if (response_data_ptr == NULL)
    {
        ErrorMonitor::instance()->add(TP_COMM_MEMORY_OPERATION_FAILED);
        FST_ERROR("handleRequest: can't allocate memory for response_data");
        delete request_data_ptr;
        return;
    }

    handleRequestPackage(0x0000D344, (void *)request_data_ptr, (void *)response_data_ptr,
                         recv_bytes, RequestMessageType_Int32_fields, -1);
}

//"/rpc/motion_control/axis_group/deleteReferencePoint"
void TpComm::handleRequest0x00008744(int recv_bytes)
{
    // create object for request and response package
    RequestMessageType_Int32 *request_data_ptr = new RequestMessageType_Int32;
    if (request_data_ptr == NULL)
    {
        ErrorMonitor::instance()->add(TP_COMM_MEMORY_OPERATION_FAILED);
        FST_ERROR("handleRequest: can't allocate memory for request_data");
        return;
    }
    ResponseMessageType_Uint64 *response_data_ptr = new ResponseMessageType_Uint64;
    if (response_data_ptr == NULL)
    {
        ErrorMonitor::instance()->add(TP_COMM_MEMORY_OPERATION_FAILED);
        FST_ERROR("handleRequest: can't allocate memory for response_data");
        delete request_data_ptr;
        return;
    }

    handleRequestPackage(0x00008744, (void *)request_data_ptr, (void *)response_data_ptr,
                         recv_bytes, RequestMessageType_Int32_fields, -1);
}

//"/rp/motion_control/axis_group/saveReferencePoint"
void TpComm::handleRequest0x00006744(int recv_bytes)
{
    // create object for request and response package
    RequestMessageType_Int32 *request_data_ptr = new RequestMessageType_Int32;
    if (request_data_ptr == NULL)
    {
        ErrorMonitor::instance()->add(TP_COMM_MEMORY_OPERATION_FAILED);
        FST_ERROR("handleRequest: can't allocate memory for request_data");
        return;
    }
    ResponseMessageType_Uint64 *response_data_ptr = new ResponseMessageType_Uint64;
    if (response_data_ptr == NULL)
    {
        ErrorMonitor::instance()->add(TP_COMM_MEMORY_OPERATION_FAILED);
        FST_ERROR("handleRequest: can't allocate memory for response_data");
        delete request_data_ptr;
        return;
    }

    handleRequestPackage(0x00006744, (void *)request_data_ptr, (void *)response_data_ptr,
                         recv_bytes, RequestMessageType_Int32_fields, -1);
}

//"/rpc/motion_control/axis_group/fastCalibrateAllZeroPointOffsets"
void TpComm::handleRequest0x0000E913(int recv_bytes)
{
    // create object for request and response package
    RequestMessageType_Int32 *request_data_ptr = new RequestMessageType_Int32;
    if (request_data_ptr == NULL)
    {
        ErrorMonitor::instance()->add(TP_COMM_MEMORY_OPERATION_FAILED);
        FST_ERROR("handleRequest: can't allocate memory for request_data");
        return;
    }
    ResponseMessageType_Uint64 *response_data_ptr = new ResponseMessageType_Uint64;
    if (response_data_ptr == NULL)
    {
        ErrorMonitor::instance()->add(TP_COMM_MEMORY_OPERATION_FAILED);
        FST_ERROR("handleRequest: can't allocate memory for response_data");
        delete request_data_ptr;
        return;
    }

    handleRequestPackage(0x0000E913, (void *)request_data_ptr, (void *)response_data_ptr,
                         recv_bytes, RequestMessageType_Int32_fields, -1);
}

//"/rpc/motion_control/axis_group/fastCalibrateSingleZeroPointOffset"
void TpComm::handleRequest0x00004754(int recv_bytes)
{
    // create object for request and response package
    RequestMessageType_Int32List *request_data_ptr = new RequestMessageType_Int32List;
    if (request_data_ptr == NULL)
    {
        ErrorMonitor::instance()->add(TP_COMM_MEMORY_OPERATION_FAILED);
        FST_ERROR("handleRequest: can't allocate memory for request_data");
        return;
    }
    ResponseMessageType_Uint64 *response_data_ptr = new ResponseMessageType_Uint64;
    if (response_data_ptr == NULL)
    {
        ErrorMonitor::instance()->add(TP_COMM_MEMORY_OPERATION_FAILED);
        FST_ERROR("handleRequest: can't allocate memory for response_data");
        delete request_data_ptr;
        return;
    }

    handleRequestPackage(0x00004754, (void *)request_data_ptr, (void *)response_data_ptr,
                         recv_bytes, RequestMessageType_Int32List_fields, -1);
}

//"/rpc/motion_control/axis_group/fastCalibrateZeroPointOffsets"
void TpComm::handleRequest0x00007EC3(int recv_bytes)
{
    // create object for request and response package
    RequestMessageType_Int32_Int32List *request_data_ptr = new RequestMessageType_Int32_Int32List;
    if (request_data_ptr == NULL)
    {
        ErrorMonitor::instance()->add(TP_COMM_MEMORY_OPERATION_FAILED);
        FST_ERROR("handleRequest: can't allocate memory for request_data");
        return;
    }
    ResponseMessageType_Uint64 *response_data_ptr = new ResponseMessageType_Uint64;
    if (response_data_ptr == NULL)
    {
        ErrorMonitor::instance()->add(TP_COMM_MEMORY_OPERATION_FAILED);
        FST_ERROR("handleRequest: can't allocate memory for response_data");
        delete request_data_ptr;
        return;
    }

    handleRequestPackage(0x00007EC3, (void *)request_data_ptr, (void *)response_data_ptr,
                         recv_bytes, RequestMessageType_Int32_Int32List_fields, -1);
}

//"/rpc/motion_control/getAxisGroupInfoList"
void TpComm::handleRequest0x00010F54(int recv_bytes)
{
    // create object for request and response package
    RequestMessageType_Void *request_data_ptr = new RequestMessageType_Void;
    if (request_data_ptr == NULL)
    {
        ErrorMonitor::instance()->add(TP_COMM_MEMORY_OPERATION_FAILED);
        FST_ERROR("handleRequest: can't allocate memory for request_data");
        return;
    }
    ResponseMessageType_Uint64_AxisGroupInfoList *response_data_ptr = new ResponseMessageType_Uint64_AxisGroupInfoList;
    if (response_data_ptr == NULL)
    {
        ErrorMonitor::instance()->add(TP_COMM_MEMORY_OPERATION_FAILED);
        FST_ERROR("handleRequest: can't allocate memory for response_data");
        delete request_data_ptr;
        return;
    }

    handleRequestPackage(0x00010F54, (void *)request_data_ptr, (void *)response_data_ptr,
                         recv_bytes, RequestMessageType_Void_fields, -1);
}

//"/rpc/motion_control/axis_group/getUserSoftLimitWithUnit",
void TpComm::handleRequest0x00008ED4(int recv_bytes)
{
    // create object for request and response package
    RequestMessageType_Int32 *request_data_ptr = new RequestMessageType_Int32;
    if (request_data_ptr == NULL)
    {
        ErrorMonitor::instance()->add(TP_COMM_MEMORY_OPERATION_FAILED);
        FST_ERROR("handleRequest: can't allocate memory for request_data");
        return;
    }
    ResponseMessageType_Uint64_JointLimitWithUnit *response_data_ptr = new ResponseMessageType_Uint64_JointLimitWithUnit;
    if (response_data_ptr == NULL)
    {
        ErrorMonitor::instance()->add(TP_COMM_MEMORY_OPERATION_FAILED);
        FST_ERROR("handleRequest: can't allocate memory for response_data");
        delete request_data_ptr;
        return;
    }

    handleRequestPackage(0x00008ED4, (void *)request_data_ptr, (void *)response_data_ptr,
                         recv_bytes, RequestMessageType_Int32_fields, -1);
}

//"/rpc/motion_control/axis_group/getManuSoftLimitWithUnit"
void TpComm::handleRequest0x000124E4(int recv_bytes)
{
    // create object for request and response package
    RequestMessageType_Int32 *request_data_ptr = new RequestMessageType_Int32;
    if (request_data_ptr == NULL)
    {
        ErrorMonitor::instance()->add(TP_COMM_MEMORY_OPERATION_FAILED);
        FST_ERROR("handleRequest: can't allocate memory for request_data");
        return;
    }
    ResponseMessageType_Uint64_JointLimitWithUnit *response_data_ptr = new ResponseMessageType_Uint64_JointLimitWithUnit;
    if (response_data_ptr == NULL)
    {
        ErrorMonitor::instance()->add(TP_COMM_MEMORY_OPERATION_FAILED);
        FST_ERROR("handleRequest: can't allocate memory for response_data");
        delete request_data_ptr;
        return;
    }

    handleRequestPackage(0x000124E4, (void *)request_data_ptr, (void *)response_data_ptr,
                         recv_bytes, RequestMessageType_Int32_fields, -1);
}

//"/rpc/motion_control/axis_group/getHardLimitWithUnit"
void TpComm::handleRequest0x000092B4(int recv_bytes)
{
    // create object for request and response package
    RequestMessageType_Int32 *request_data_ptr = new RequestMessageType_Int32;
    if (request_data_ptr == NULL)
    {
        ErrorMonitor::instance()->add(TP_COMM_MEMORY_OPERATION_FAILED);
        FST_ERROR("handleRequest: can't allocate memory for request_data");
        return;
    }
    ResponseMessageType_Uint64_JointLimitWithUnit *response_data_ptr = new ResponseMessageType_Uint64_JointLimitWithUnit;
    if (response_data_ptr == NULL)
    {
        ErrorMonitor::instance()->add(TP_COMM_MEMORY_OPERATION_FAILED);
        FST_ERROR("handleRequest: can't allocate memory for response_data");
        delete request_data_ptr;
        return;
    }

    handleRequestPackage(0x000092B4, (void *)request_data_ptr, (void *)response_data_ptr,
                         recv_bytes, RequestMessageType_Int32_fields, -1);
}

//"/rpc/motion_control/axis_group/setRotateManualStep"
void TpComm::handleRequest0x00005290(int recv_bytes)
{
    // create object for request and response package
    RequestMessageType_Int32_Double *request_data_ptr = new RequestMessageType_Int32_Double;
    if (request_data_ptr == NULL)
    {
        ErrorMonitor::instance()->add(TP_COMM_MEMORY_OPERATION_FAILED);
        FST_ERROR("handleRequest: can't allocate memory for request_data");
        return;
    }
    ResponseMessageType_Uint64 *response_data_ptr = new ResponseMessageType_Uint64;
    if (response_data_ptr == NULL)
    {
        ErrorMonitor::instance()->add(TP_COMM_MEMORY_OPERATION_FAILED);
        FST_ERROR("handleRequest: can't allocate memory for response_data");
        delete request_data_ptr;
        return;
    }

    handleRequestPackage(0x00005290, (void *)request_data_ptr, (void *)response_data_ptr,
                         recv_bytes, RequestMessageType_Int32_Double_fields, -1);
}

//"/rpc/motion_control/axis_group/getRotateManualStep"
void TpComm::handleRequest0x00003000(int recv_bytes)
{
    // create object for request and response package
    RequestMessageType_Int32 *request_data_ptr = new RequestMessageType_Int32;
    if (request_data_ptr == NULL)
    {
        ErrorMonitor::instance()->add(TP_COMM_MEMORY_OPERATION_FAILED);
        FST_ERROR("handleRequest: can't allocate memory for request_data");
        return;
    }
    ResponseMessageType_Uint64_Double *response_data_ptr = new ResponseMessageType_Uint64_Double;
    if (response_data_ptr == NULL)
    {
        ErrorMonitor::instance()->add(TP_COMM_MEMORY_OPERATION_FAILED);
        FST_ERROR("handleRequest: can't allocate memory for response_data");
        delete request_data_ptr;
        return;
    }

    handleRequestPackage(0x00003000, (void *)request_data_ptr, (void *)response_data_ptr,
                         recv_bytes, RequestMessageType_Int32_fields, -1);
}

//"/rpc/motion_control/axis_group/setPrismaticManualStep"
void TpComm::handleRequest0x0000B640(int recv_bytes)
{
    // create object for request and response package
    RequestMessageType_Int32_Double *request_data_ptr = new RequestMessageType_Int32_Double;
    if (request_data_ptr == NULL)
    {
        ErrorMonitor::instance()->add(TP_COMM_MEMORY_OPERATION_FAILED);
        FST_ERROR("handleRequest: can't allocate memory for request_data");
        return;
    }
    ResponseMessageType_Uint64 *response_data_ptr = new ResponseMessageType_Uint64;
    if (response_data_ptr == NULL)
    {
        ErrorMonitor::instance()->add(TP_COMM_MEMORY_OPERATION_FAILED);
        FST_ERROR("handleRequest: can't allocate memory for response_data");
        delete request_data_ptr;
        return;
    }

    handleRequestPackage(0x0000B640, (void *)request_data_ptr, (void *)response_data_ptr,
                         recv_bytes, RequestMessageType_Int32_Double_fields, -1);
}

//"/rpc/motion_control/axis_group/getPrismaticManualStep"
void TpComm::handleRequest0x0000FCE0(int recv_bytes)
{
    // create object for request and response package
    RequestMessageType_Int32 *request_data_ptr = new RequestMessageType_Int32;
    if (request_data_ptr == NULL)
    {
        ErrorMonitor::instance()->add(TP_COMM_MEMORY_OPERATION_FAILED);
        FST_ERROR("handleRequest: can't allocate memory for request_data");
        return;
    }
    ResponseMessageType_Uint64_Double *response_data_ptr = new ResponseMessageType_Uint64_Double;
    if (response_data_ptr == NULL)
    {
        ErrorMonitor::instance()->add(TP_COMM_MEMORY_OPERATION_FAILED);
        FST_ERROR("handleRequest: can't allocate memory for response_data");
        delete request_data_ptr;
        return;
    }

    handleRequestPackage(0x0000FCE0, (void *)request_data_ptr, (void *)response_data_ptr,
                         recv_bytes, RequestMessageType_Int32_fields, -1);
}

//"/rpc/motion_control/axis_group/setCartesianManualStep"
void TpComm::handleRequest0x0000A420(int recv_bytes)
{
    // create object for request and response package
    RequestMessageType_Int32_Double *request_data_ptr = new RequestMessageType_Int32_Double;
    if (request_data_ptr == NULL)
    {
        ErrorMonitor::instance()->add(TP_COMM_MEMORY_OPERATION_FAILED);
        FST_ERROR("handleRequest: can't allocate memory for request_data");
        return;
    }
    ResponseMessageType_Uint64 *response_data_ptr = new ResponseMessageType_Uint64;
    if (response_data_ptr == NULL)
    {
        ErrorMonitor::instance()->add(TP_COMM_MEMORY_OPERATION_FAILED);
        FST_ERROR("handleRequest: can't allocate memory for response_data");
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
        ErrorMonitor::instance()->add(TP_COMM_MEMORY_OPERATION_FAILED);
        FST_ERROR("handleRequest: can't allocate memory for request_data");
        return;
    }
    ResponseMessageType_Uint64_Double *response_data_ptr = new ResponseMessageType_Uint64_Double;
    if (response_data_ptr == NULL)
    {
        ErrorMonitor::instance()->add(TP_COMM_MEMORY_OPERATION_FAILED);
        FST_ERROR("handleRequest: can't allocate memory for response_data");
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
        ErrorMonitor::instance()->add(TP_COMM_MEMORY_OPERATION_FAILED);
        FST_ERROR("handleRequest: can't allocate memory for request_data");
        return;
    }
    ResponseMessageType_Uint64 *response_data_ptr = new ResponseMessageType_Uint64;
    if (response_data_ptr == NULL)
    {
        ErrorMonitor::instance()->add(TP_COMM_MEMORY_OPERATION_FAILED);
        FST_ERROR("handleRequest: can't allocate memory for response_data");
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
        ErrorMonitor::instance()->add(TP_COMM_MEMORY_OPERATION_FAILED);
        FST_ERROR("handleRequest: can't allocate memory for request_data");
        return;
    }
    ResponseMessageType_Uint64_Double *response_data_ptr = new ResponseMessageType_Uint64_Double;
    if (response_data_ptr == NULL)
    {
        ErrorMonitor::instance()->add(TP_COMM_MEMORY_OPERATION_FAILED);
        FST_ERROR("handleRequest: can't allocate memory for response_data");
        delete request_data_ptr;
        return;
    }

    handleRequestPackage(0x00016D20, (void *)request_data_ptr, (void *)response_data_ptr,
                         recv_bytes, RequestMessageType_Int32_fields, -1);
}

////"/rpc/motion_control/axis_group/setSingleZeroPointOffset",	"RequestMessageType.Int32List_Double(count=2)",	"ResponseMessageType.Uint64"
void TpComm::handleRequest0x00012404(int recv_bytes)
{
    // create object for request and response package
    RequestMessageType_Int32List_Double *request_data_ptr = new RequestMessageType_Int32List_Double;
    if (request_data_ptr == NULL)
    {
        ErrorMonitor::instance()->add(TP_COMM_MEMORY_OPERATION_FAILED);
        FST_ERROR("handleRequest: can't allocate memory for request_data");
        return;
    }
    ResponseMessageType_Uint64 *response_data_ptr = new ResponseMessageType_Uint64;
    if (response_data_ptr == NULL)
    {
        ErrorMonitor::instance()->add(TP_COMM_MEMORY_OPERATION_FAILED);
        FST_ERROR("handleRequest: can't allocate memory for response_data");
        delete request_data_ptr;
        return;
    }

    handleRequestPackage(0x00012404, (void *)request_data_ptr, (void *)response_data_ptr,
                         recv_bytes, RequestMessageType_Int32List_Double_fields, -1);
}

//"/rpc/motion_control/getPostureByJoint"
void TpComm::handleRequest0x0000EC64(int recv_bytes)
{
    // create object for request and response package
    RequestMessageType_Int32_DoubleList *request_data_ptr = new RequestMessageType_Int32_DoubleList;
    if (request_data_ptr == NULL)
    {
        ErrorMonitor::instance()->add(TP_COMM_MEMORY_OPERATION_FAILED);
        FST_ERROR("handleRequest: can't allocate memory for request_data");
        return;
    }
    ResponseMessageType_Uint64_Int32List *response_data_ptr = new ResponseMessageType_Uint64_Int32List;
    if (response_data_ptr == NULL)
    {
        ErrorMonitor::instance()->add(TP_COMM_MEMORY_OPERATION_FAILED);
        FST_ERROR("handleRequest: can't allocate memory for response_data");
        delete request_data_ptr;
        return;
    }

    handleRequestPackage(0x0000EC64, (void *)request_data_ptr, (void *)response_data_ptr,
                         recv_bytes, RequestMessageType_Int32_DoubleList_fields, -1);
}

//"/rpc/motion_control/axis_group/setJointManualStep"
void TpComm::handleRequest0x00018470(int recv_bytes)
{
    // create object for request and response package
    RequestMessageType_Int32_DoubleList *request_data_ptr = new RequestMessageType_Int32_DoubleList;
    if (request_data_ptr == NULL)
    {
        ErrorMonitor::instance()->add(TP_COMM_MEMORY_OPERATION_FAILED);
        FST_ERROR("handleRequest: can't allocate memory for request_data");
        return;
    }
    ResponseMessageType_Uint64 *response_data_ptr = new ResponseMessageType_Uint64;
    if (response_data_ptr == NULL)
    {
        ErrorMonitor::instance()->add(TP_COMM_MEMORY_OPERATION_FAILED);
        FST_ERROR("handleRequest: can't allocate memory for response_data");
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
        ErrorMonitor::instance()->add(TP_COMM_MEMORY_OPERATION_FAILED);
        FST_ERROR("handleRequest: can't allocate memory for request_data");
        return;
    }
    ResponseMessageType_Uint64_DoubleList *response_data_ptr = new ResponseMessageType_Uint64_DoubleList;
    if (response_data_ptr == NULL)
    {
        ErrorMonitor::instance()->add(TP_COMM_MEMORY_OPERATION_FAILED);
        FST_ERROR("handleRequest: can't allocate memory for response_data");
        delete request_data_ptr;
        return;
    }

    handleRequestPackage(0x00006D10, (void *)request_data_ptr, (void *)response_data_ptr,
                         recv_bytes, RequestMessageType_Int32_fields, -1);
}

//"/rpc/motion_control/axis_group/getFcpBasePose"
void TpComm::handleRequest0x000016B5(int recv_bytes)
{
    // create object for request and response package
    RequestMessageType_Int32 *request_data_ptr = new RequestMessageType_Int32;
    if (request_data_ptr == NULL)
    {
        ErrorMonitor::instance()->add(TP_COMM_MEMORY_OPERATION_FAILED);
        FST_ERROR("handleRequest: can't allocate memory for request_data");
        return;
    }
    ResponseMessageType_Uint64_DoubleList *response_data_ptr = new ResponseMessageType_Uint64_DoubleList;
    if (response_data_ptr == NULL)
    {
        ErrorMonitor::instance()->add(TP_COMM_MEMORY_OPERATION_FAILED);
        FST_ERROR("handleRequest: can't allocate memory for response_data");
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
        ErrorMonitor::instance()->add(TP_COMM_MEMORY_OPERATION_FAILED);
        FST_ERROR("handleRequest: can't allocate memory for request_data");
        return;
    }
    ResponseMessageType_Uint64_DoubleList *response_data_ptr = new ResponseMessageType_Uint64_DoubleList;
    if (response_data_ptr == NULL)
    {
        ErrorMonitor::instance()->add(TP_COMM_MEMORY_OPERATION_FAILED);
        FST_ERROR("handleRequest: can't allocate memory for response_data");
        delete request_data_ptr;
        return;
    }

    handleRequestPackage(0x00003B45, (void *)request_data_ptr, (void *)response_data_ptr,
                         recv_bytes, RequestMessageType_Int32List_fields, -1);
}

