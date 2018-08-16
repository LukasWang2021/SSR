#include "tp_comm.h"
#include <pb_encode.h>
#include <pb_decode.h>
#include <pb_common.h>

using namespace fst_comm;

//"/rpc/motion_control/stop"
void TpComm::handleRequest0x00001E70(int recv_bytes)
{
    // create object for request and response package
    RequestMessageType_Void* request_data_ptr = new RequestMessageType_Void;
    if(request_data_ptr == NULL)
    {
        FST_ERROR("handleRequest: can't allocate memory for request_data");
        return;
    }
    ResponseMessageType_Bool* response_data_ptr = new ResponseMessageType_Bool;
    if(response_data_ptr == NULL)
    {
        FST_ERROR("handleRequest: can't allocate memory for response_data");
        delete request_data_ptr;
        return;
    }
    
    handleRequestPackage(0x00001E70, (void*)request_data_ptr, (void*)response_data_ptr, 
        recv_bytes, RequestMessageType_Void_fields, -1);
}

//"/rpc/motion_control/reset"
void TpComm::handleRequest0x00001D14(int recv_bytes)
{
    // create object for request and response package
    RequestMessageType_Void* request_data_ptr = new RequestMessageType_Void;
    if(request_data_ptr == NULL)
    {
        FST_ERROR("handleRequest: can't allocate memory for request_data");
        return;
    }
    ResponseMessageType_Bool* response_data_ptr = new ResponseMessageType_Bool;
    if(response_data_ptr == NULL)
    {
        FST_ERROR("handleRequest: can't allocate memory for response_data");
        delete request_data_ptr;
        return;
    }
    
    handleRequestPackage(0x00001D14, (void*)request_data_ptr, (void*)response_data_ptr, 
        recv_bytes, RequestMessageType_Void_fields, -1);
}

//"/rpc/motion_control/axis_group/setManualFrame"
void TpComm::handleRequest0x00009D05(int recv_bytes)
{
    // create object for request and response package
    RequestMessageType_Int32List* request_data_ptr = new RequestMessageType_Int32List;
    if(request_data_ptr == NULL)
    {
        FST_ERROR("handleRequest: can't allocate memory for request_data");
        return;
    }
    ResponseMessageType_Bool* response_data_ptr = new ResponseMessageType_Bool;
    if(response_data_ptr == NULL)
    {
        FST_ERROR("handleRequest: can't allocate memory for response_data");
        delete request_data_ptr;
        return;
    }
    
    handleRequestPackage(0x00009D05, (void*)request_data_ptr, (void*)response_data_ptr, 
        recv_bytes, RequestMessageType_Int32List_fields, -1);
}

//"/rpc/motion_control/axis_group/doStepManualMove"
void TpComm::handleRequest0x000085D5(int recv_bytes)
{
    // create object for request and response package
    RequestMessageType_Int32_Int32List* request_data_ptr = new RequestMessageType_Int32_Int32List;
    if(request_data_ptr == NULL)
    {
        FST_ERROR("handleRequest: can't allocate memory for request_data");
        return;
    }
    ResponseMessageType_Bool* response_data_ptr = new ResponseMessageType_Bool;
    if(response_data_ptr == NULL)
    {
        FST_ERROR("handleRequest: can't allocate memory for response_data");
        delete request_data_ptr;
        return;
    }
    
    handleRequestPackage(0x000085D5, (void*)request_data_ptr, (void*)response_data_ptr, 
        recv_bytes, RequestMessageType_Int32_Int32List_fields, -1);
}

//"/rpc/motion_control/axis_group/doContinusManulMove"
void TpComm::handleRequest0x0000D3F5(int recv_bytes)
{
    // create object for request and response package
    RequestMessageType_Int32_Int32List* request_data_ptr = new RequestMessageType_Int32_Int32List;
    if(request_data_ptr == NULL)
    {
        FST_ERROR("handleRequest: can't allocate memory for request_data");
        return;
    }
    ResponseMessageType_Bool* response_data_ptr = new ResponseMessageType_Bool;
    if(response_data_ptr == NULL)
    {
        FST_ERROR("handleRequest: can't allocate memory for response_data");
        delete request_data_ptr;
        return;
    }
    
    handleRequestPackage(0x0000D3F5, (void*)request_data_ptr, (void*)response_data_ptr, 
        recv_bytes, RequestMessageType_Int32_Int32List_fields, -1);
}

//"/rpc/motion_control/axis_group/getJointsFeedBack"
void TpComm::handleRequest0x0000DFBB(int recv_bytes)
{
    // create object for request and response package
    RequestMessageType_Int32* request_data_ptr = new RequestMessageType_Int32;
    if(request_data_ptr == NULL)
    {
        FST_ERROR("handleRequest: can't allocate memory for request_data");
        return;
    }
    ResponseMessageType_Bool_DoubleList* response_data_ptr = new ResponseMessageType_Bool_DoubleList;
    if(response_data_ptr == NULL)
    {
        FST_ERROR("handleRequest: can't allocate memory for response_data");
        delete request_data_ptr;
        return;
    }
    
    handleRequestPackage(0x0000DFBB, (void*)request_data_ptr, (void*)response_data_ptr, 
        recv_bytes, RequestMessageType_Int32_fields, -1);
}



//"/rpc/motion_control/axis_group/setUserSoftLimit"
void TpComm::handleRequest0x000114A4(int recv_bytes)
{
    // create object for request and response package
    RequestMessageType_Int32_JointLimit* request_data_ptr = new RequestMessageType_Int32_JointLimit;
    if(request_data_ptr == NULL)
    {
        FST_ERROR("handleRequest: can't allocate memory for request_data");
        return;
    }
    ResponseMessageType_Bool* response_data_ptr = new ResponseMessageType_Bool;
    if(response_data_ptr == NULL)
    {
        FST_ERROR("handleRequest: can't allocate memory for response_data");
        delete request_data_ptr;
        return;
    }
    
    handleRequestPackage(0x000114A4, (void*)request_data_ptr, (void*)response_data_ptr, 
        recv_bytes, RequestMessageType_Int32_JointLimit_fields, -1);
}

//"/rpc/motion_control/axis_group/getUserSoftLimit"
void TpComm::handleRequest0x0000C764(int recv_bytes)
{
    // create object for request and response package
    RequestMessageType_Int32* request_data_ptr = new RequestMessageType_Int32;
    if(request_data_ptr == NULL)
    {
        FST_ERROR("handleRequest: can't allocate memory for request_data");
        return;
    }
    ResponseMessageType_Bool_JointLimit* response_data_ptr = new ResponseMessageType_Bool_JointLimit;
    if(response_data_ptr == NULL)
    {
        FST_ERROR("handleRequest: can't allocate memory for response_data");
        delete request_data_ptr;
        return;
    }
    
    handleRequestPackage(0x0000C764, (void*)request_data_ptr, (void*)response_data_ptr, 
        recv_bytes, RequestMessageType_Int32_fields, -1);
}

//"/rpc/motion_control/axis_group/setManuSoftLimit"
void TpComm::handleRequest0x000108E4(int recv_bytes)
{
    // create object for request and response package
    RequestMessageType_Int32_JointLimit* request_data_ptr = new RequestMessageType_Int32_JointLimit;
    if(request_data_ptr == NULL)
    {
        FST_ERROR("handleRequest: can't allocate memory for request_data");
        return;
    }
    ResponseMessageType_Bool* response_data_ptr = new ResponseMessageType_Bool;
    if(response_data_ptr == NULL)
    {
        FST_ERROR("handleRequest: can't allocate memory for response_data");
        delete request_data_ptr;
        return;
    }
    
    handleRequestPackage(0x000108E4, (void*)request_data_ptr, (void*)response_data_ptr, 
        recv_bytes, RequestMessageType_Int32_JointLimit_fields, -1);
}

//"/rpc/motion_control/axis_group/getManuSoftLimit"
void TpComm::handleRequest0x0000C244(int recv_bytes)
{
    // create object for request and response package
    RequestMessageType_Int32* request_data_ptr = new RequestMessageType_Int32;
    if(request_data_ptr == NULL)
    {
        FST_ERROR("handleRequest: can't allocate memory for request_data");
        return;
    }
    ResponseMessageType_Bool_JointLimit* response_data_ptr = new ResponseMessageType_Bool_JointLimit;
    if(response_data_ptr == NULL)
    {
        FST_ERROR("handleRequest: can't allocate memory for response_data");
        delete request_data_ptr;
        return;
    }
    
    handleRequestPackage(0x0000C244, (void*)request_data_ptr, (void*)response_data_ptr, 
        recv_bytes, RequestMessageType_Int32_fields, -1);
}

//"/rpc/motion_control/axis_group/doGotoCartesianPointManualMove"
void TpComm::handleRequest0x00010C05(int recv_bytes)
{
    // create object for request and response package
    RequestMessageType_Int32_DoubleList* request_data_ptr = new RequestMessageType_Int32_DoubleList;
    if(request_data_ptr == NULL)
    {
        FST_ERROR("handleRequest: can't allocate memory for request_data");
        return;
    }
    ResponseMessageType_Bool* response_data_ptr = new ResponseMessageType_Bool;
    if(response_data_ptr == NULL)
    {
        FST_ERROR("handleRequest: can't allocate memory for response_data");
        delete request_data_ptr;
        return;
    }
    
    handleRequestPackage(0x00010C05, (void*)request_data_ptr, (void*)response_data_ptr, 
        recv_bytes, RequestMessageType_Int32_DoubleList_fields, -1);
}

//"/rpc/motion_control/axis_group/doGotoJointPointManualMove"
void TpComm::handleRequest0x00008075(int recv_bytes)
{
    // create object for request and response package
    RequestMessageType_Int32_DoubleList* request_data_ptr = new RequestMessageType_Int32_DoubleList;
    if(request_data_ptr == NULL)
    {
        FST_ERROR("handleRequest: can't allocate memory for request_data");
        return;
    }
    ResponseMessageType_Bool* response_data_ptr = new ResponseMessageType_Bool;
    if(response_data_ptr == NULL)
    {
        FST_ERROR("handleRequest: can't allocate memory for response_data");
        delete request_data_ptr;
        return;
    }
    
    handleRequestPackage(0x00008075, (void*)request_data_ptr, (void*)response_data_ptr, 
        recv_bytes, RequestMessageType_Int32_DoubleList_fields, -1);
}

//"/rpc/motion_control/axis_group/doManualStop"
void TpComm::handleRequest0x0000A9A0(int recv_bytes)
{
    // create object for request and response package
    RequestMessageType_Int32* request_data_ptr = new RequestMessageType_Int32;
    if(request_data_ptr == NULL)
    {
        FST_ERROR("handleRequest: can't allocate memory for request_data");
        return;
    }
    ResponseMessageType_Bool* response_data_ptr = new ResponseMessageType_Bool;
    if(response_data_ptr == NULL)
    {
        FST_ERROR("handleRequest: can't allocate memory for response_data");
        delete request_data_ptr;
        return;
    }
    
    handleRequestPackage(0x0000A9A0, (void*)request_data_ptr, (void*)response_data_ptr, 
        recv_bytes, RequestMessageType_Int32_fields, -1);
}

