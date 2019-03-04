#include "tp_comm.h"

using namespace fst_comm;

//"/rpc/motion_control/stop"
void TpComm::handleResponse0x00001E70(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size)
{
    if(!encodeResponsePackage(task->hash, ResponseMessageType_Uint64_fields, task->response_data_ptr, send_buffer_size))
    {
        FST_ERROR("handleResponse: failed to encode response package");// send
    }
    if(task->request_data_ptr != NULL)
    {
        delete (RequestMessageType_Void*)task->request_data_ptr;
    }
    if(task->response_data_ptr != NULL)
    {
        delete (ResponseMessageType_Uint64*)task->response_data_ptr;
    }
}

// "/rpc/motion_control/reset"
void TpComm::handleResponse0x00001D14(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size)
{
    if(!encodeResponsePackage(task->hash, ResponseMessageType_Uint64_fields, task->response_data_ptr, send_buffer_size))
    {
        FST_ERROR("handleResponse: failed to encode response package");// send
    }
    if(task->request_data_ptr != NULL)
    {
        delete (RequestMessageType_Void*)task->request_data_ptr;
    }
    if(task->response_data_ptr != NULL)
    {
        delete (ResponseMessageType_Uint64*)task->response_data_ptr;
    }
}

//"/rpc/motion_control/axis_group/doStepManualMove"
void TpComm::handleResponse0x000085D5(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size)
{
    if(!encodeResponsePackage(task->hash, ResponseMessageType_Uint64_fields, task->response_data_ptr, send_buffer_size))
    {
        FST_ERROR("handleResponse: failed to encode response package");// send
    }
    if(task->request_data_ptr != NULL)
    {
        delete (RequestMessageType_Int32_Int32List*)task->request_data_ptr;
    }
    if(task->response_data_ptr != NULL)
    {
        delete (ResponseMessageType_Uint64*)task->response_data_ptr;
    }
}

//"/rpc/motion_control/axis_group/doContinusManulMove"
void TpComm::handleResponse0x0000D3F5(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size)
{
    if(!encodeResponsePackage(task->hash, ResponseMessageType_Uint64_fields, task->response_data_ptr, send_buffer_size))
    {
        FST_ERROR("handleResponse: failed to encode response package");// send
    }
    if(task->request_data_ptr != NULL)
    {
        delete (RequestMessageType_Int32_Int32List*)task->request_data_ptr;
    }
    if(task->response_data_ptr != NULL)
    {
        delete (ResponseMessageType_Uint64*)task->response_data_ptr;
    }
}

//"/rpc/motion_control/axis_group/getJointsFeedBack"
void TpComm::handleResponse0x0000DFBB(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size)
{
    if(!encodeResponsePackage(task->hash, ResponseMessageType_Uint64_DoubleList_fields, task->response_data_ptr, send_buffer_size))
    {
        FST_ERROR("handleResponse: failed to encode response package");// send
    }
    if(task->request_data_ptr != NULL)
    {
        delete (RequestMessageType_Int32*)task->request_data_ptr;
    }
    if(task->response_data_ptr != NULL)
    {
        delete (ResponseMessageType_Uint64_DoubleList*)task->response_data_ptr;
    }
}




// "/rpc/motion_control/axis_group/setUserSoftLimit",
void TpComm::handleResponse0x000114A4(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size)
{
    if(!encodeResponsePackage(task->hash, ResponseMessageType_Uint64_fields, task->response_data_ptr, send_buffer_size))
    {
        FST_ERROR("handleResponse: failed to encode response package");// send
    }
    if(task->request_data_ptr != NULL)
    {
        delete (RequestMessageType_Int32_JointLimit*)task->request_data_ptr;
    }
    if(task->response_data_ptr != NULL)
    {
        delete (ResponseMessageType_Uint64*)task->response_data_ptr;
    }
}

//"/rpc/motion_control/axis_group/getUserSoftLimit",
void TpComm::handleResponse0x0000C764(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size)
{
    if(!encodeResponsePackage(task->hash, ResponseMessageType_Uint64_JointLimit_fields, task->response_data_ptr, send_buffer_size))
    {
        FST_ERROR("handleResponse: failed to encode response package");// send
    }
    if(task->request_data_ptr != NULL)
    {
        delete (RequestMessageType_Int32*)task->request_data_ptr;
    }
    if(task->response_data_ptr != NULL)
    {
        delete (ResponseMessageType_Uint64_JointLimit*)task->response_data_ptr;
    }
}

//"/rpc/motion_control/axis_group/setManuSoftLimit"
void TpComm::handleResponse0x000108E4(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size)
{
    if(!encodeResponsePackage(task->hash, ResponseMessageType_Uint64_fields, task->response_data_ptr, send_buffer_size))
    {
        FST_ERROR("handleResponse: failed to encode response package");// send
    }
    if(task->request_data_ptr != NULL)
    {
        delete (RequestMessageType_Int32_JointLimit*)task->request_data_ptr;
    }
    if(task->response_data_ptr != NULL)
    {
        delete (ResponseMessageType_Uint64*)task->response_data_ptr;
    }
}

//"/rpc/motion_control/axis_group/getManuSoftLimit"
void TpComm::handleResponse0x0000C244(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size)
{
    if(!encodeResponsePackage(task->hash, ResponseMessageType_Uint64_JointLimit_fields, task->response_data_ptr, send_buffer_size))
    {
        FST_ERROR("handleResponse: failed to encode response package");// send
    }
    if(task->request_data_ptr != NULL)
    {
        delete (RequestMessageType_Int32*)task->request_data_ptr;
    }
    if(task->response_data_ptr != NULL)
    {
        delete (ResponseMessageType_Uint64_JointLimit*)task->response_data_ptr;
    }
}

//"/rpc/motion_control/axis_group/doGotoCartesianPointManualMove"
void TpComm::handleResponse0x00010C05(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size)
{
    if(!encodeResponsePackage(task->hash, ResponseMessageType_Uint64_fields, task->response_data_ptr, send_buffer_size))
    {
        FST_ERROR("handleResponse: failed to encode response package");// send
    }
    if(task->request_data_ptr != NULL)
    {
        delete (RequestMessageType_Int32_DoubleList*)task->request_data_ptr;
    }
    if(task->response_data_ptr != NULL)
    {
        delete (ResponseMessageType_Uint64*)task->response_data_ptr;
    }
}

//"/rpc/motion_control/axis_group/doGotoJointPointManualMove"
void TpComm::handleResponse0x00008075(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size)
{
    if(!encodeResponsePackage(task->hash, ResponseMessageType_Uint64_fields, task->response_data_ptr, send_buffer_size))
    {
        FST_ERROR("handleResponse: failed to encode response package");// send
    }
    if(task->request_data_ptr != NULL)
    {
        delete (RequestMessageType_Int32_DoubleList*)task->request_data_ptr;
    }
    if(task->response_data_ptr != NULL)
    {
        delete (ResponseMessageType_Uint64*)task->response_data_ptr;
    }
}

//"/rpc/motion_control/axis_group/doManualStop"
void TpComm::handleResponse0x0000A9A0(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size)
{
    if(!encodeResponsePackage(task->hash, ResponseMessageType_Uint64_fields, task->response_data_ptr, send_buffer_size))
    {
        FST_ERROR("handleResponse: failed to encode response package");// send
    }
    if(task->request_data_ptr != NULL)
    {
        delete (RequestMessageType_Int32*)task->request_data_ptr;
    }
    if(task->response_data_ptr != NULL)
    {
        delete (ResponseMessageType_Uint64*)task->response_data_ptr;
    }
}


//"/rpc/motion_control/setGlobalVelRatio",
void TpComm::handleResponse0x000005EF(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size)
{
    if(!encodeResponsePackage(task->hash, ResponseMessageType_Uint64_fields, task->response_data_ptr, send_buffer_size))
    {
        FST_ERROR("handleResponse: failed to encode response package");// send
    }
    if(task->request_data_ptr != NULL)
    {
        delete (RequestMessageType_Double*)task->request_data_ptr;
    }
    if(task->response_data_ptr != NULL)
    {
        delete (ResponseMessageType_Uint64*)task->response_data_ptr;
    }
}

//"/rpc/motion_control/getGlobalVelRatio",
void TpComm::handleResponse0x0001578F(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size)
{
    if(!encodeResponsePackage(task->hash, ResponseMessageType_Uint64_Double_fields, task->response_data_ptr, send_buffer_size))
    {
        FST_ERROR("handleResponse: failed to encode response package");// send
    }
    if(task->request_data_ptr != NULL)
    {
        delete (RequestMessageType_Void*)task->request_data_ptr;
    }
    if(task->response_data_ptr != NULL)
    {
        delete (ResponseMessageType_Uint64_Double*)task->response_data_ptr;
    }
}

//"/rpc/motion_control/setGlobalAccRatio"
void TpComm::handleResponse0x0000271F(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size)
{
    if(!encodeResponsePackage(task->hash, ResponseMessageType_Uint64_fields, task->response_data_ptr, send_buffer_size))
    {
        FST_ERROR("handleResponse: failed to encode response package");// send
    }
    if(task->request_data_ptr != NULL)
    {
        delete (RequestMessageType_Double*)task->request_data_ptr;
    }
    if(task->response_data_ptr != NULL)
    {
        delete (ResponseMessageType_Uint64*)task->response_data_ptr;
    }
}

//"/rpc/motion_control/getGlobalAccRatio",
void TpComm::handleResponse0x00016D9F(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size)
{
    if(!encodeResponsePackage(task->hash, ResponseMessageType_Uint64_Double_fields, task->response_data_ptr, send_buffer_size))
    {
        FST_ERROR("handleResponse: failed to encode response package");// send
    }
    if(task->request_data_ptr != NULL)
    {
        delete (RequestMessageType_Void*)task->request_data_ptr;
    }
    if(task->response_data_ptr != NULL)
    {
        delete (ResponseMessageType_Uint64_Double*)task->response_data_ptr;
    }
}

//"/rpc/motion_control/axis_group/setHardLimit"
void TpComm::handleResponse0x0000C454(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size)
{
    if(!encodeResponsePackage(task->hash, ResponseMessageType_Uint64_fields, task->response_data_ptr, send_buffer_size))
    {
        FST_ERROR("handleResponse: failed to encode response package");// send
    }
    if(task->request_data_ptr != NULL)
    {
        delete (RequestMessageType_Int32_JointLimit*)task->request_data_ptr;
    }
    if(task->response_data_ptr != NULL)
    {
        delete (ResponseMessageType_Uint64*)task->response_data_ptr;
    }
}

//"/rpc/motion_control/axis_group/getHardLimit"
void TpComm::handleResponse0x00013394(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size)
{
    if(!encodeResponsePackage(task->hash, ResponseMessageType_Uint64_JointLimit_fields, task->response_data_ptr, send_buffer_size))
    {
        FST_ERROR("handleResponse: failed to encode response package");// send
    }
    if(task->request_data_ptr != NULL)
    {
        delete (RequestMessageType_Int32*)task->request_data_ptr;
    }
    if(task->response_data_ptr != NULL)
    {
        delete (ResponseMessageType_Uint64_JointLimit*)task->response_data_ptr;
    }
}

//"/rpc/motion_control/axis_group/getCoordinate"
void TpComm::handleResponse0x00008595(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size)
{
    if(!encodeResponsePackage(task->hash, ResponseMessageType_Uint64_Int32List_fields, task->response_data_ptr, send_buffer_size))
    {
        FST_ERROR("handleResponse: failed to encode response package");// send
    }
    if(task->request_data_ptr != NULL)
    {
        delete (RequestMessageType_Int32*)task->request_data_ptr;
    }
    if(task->response_data_ptr != NULL)
    {
        delete (ResponseMessageType_Uint64_Int32List*)task->response_data_ptr;
    }
}

//"/rpc/motion_control/axis_group/setCoordinate"
void TpComm::handleResponse0x0000A845(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size)
{
    if(!encodeResponsePackage(task->hash, ResponseMessageType_Uint64_fields, task->response_data_ptr, send_buffer_size))
    {
        FST_ERROR("handleResponse: failed to encode response package");// send
    }
    if(task->request_data_ptr != NULL)
    {
        delete (RequestMessageType_Int32List*)task->request_data_ptr;
    }
    if(task->response_data_ptr != NULL)
    {
        delete (ResponseMessageType_Uint64*)task->response_data_ptr;
    }
}

//"/rpc/motion_control/axis_group/getUserCoordId"
void TpComm::handleResponse0x00005BB4(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size)
{
    if(!encodeResponsePackage(task->hash, ResponseMessageType_Uint64_Int32_fields, task->response_data_ptr, send_buffer_size))
    {
        FST_ERROR("handleResponse: failed to encode response package");// send
    }
    if(task->request_data_ptr != NULL)
    {
        delete (RequestMessageType_Int32*)task->request_data_ptr;
    }
    if(task->response_data_ptr != NULL)
    {
        delete (ResponseMessageType_Uint64_Int32*)task->response_data_ptr;
    }
}

//"/rpc/motion_control/axis_group/setUserCoordId"
void TpComm::handleResponse0x00005CF4(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size)
{
    if(!encodeResponsePackage(task->hash, ResponseMessageType_Uint64_fields, task->response_data_ptr, send_buffer_size))
    {
        FST_ERROR("handleResponse: failed to encode response package");// send
    }
    if(task->request_data_ptr != NULL)
    {
        delete (RequestMessageType_Int32List*)task->request_data_ptr;
    }
    if(task->response_data_ptr != NULL)
    {
        delete (ResponseMessageType_Uint64*)task->response_data_ptr;
    }
}

//"/rpc/motion_control/axis_group/getTool"
void TpComm::handleResponse0x0001354C(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size)
{
    if(!encodeResponsePackage(task->hash, ResponseMessageType_Uint64_Int32_fields, task->response_data_ptr, send_buffer_size))
    {
        FST_ERROR("handleResponse: failed to encode response package");// send
    }
    if(task->request_data_ptr != NULL)
    {
        delete (RequestMessageType_Int32*)task->request_data_ptr;
    }
    if(task->response_data_ptr != NULL)
    {
        delete (ResponseMessageType_Uint64_Int32*)task->response_data_ptr;
    }
}

//"/rpc/motion_control/axis_group/setTool",	0x0001581C,	"RequestMessageType.Int32List(count = 2) ",	"ResponseMessageType.Uint64
void TpComm::handleResponse0x0001581C(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size)
{
    if(!encodeResponsePackage(task->hash, ResponseMessageType_Uint64_fields, task->response_data_ptr, send_buffer_size))
    {
        FST_ERROR("handleResponse: failed to encode response package");// send
    }
    if(task->request_data_ptr != NULL)
    {
        delete (RequestMessageType_Int32List*)task->request_data_ptr;
    }
    if(task->response_data_ptr != NULL)
    {
        delete (ResponseMessageType_Uint64*)task->response_data_ptr;
    }
}

//"/rpc/motion_control/axis_group/convertCartToJoint"
void TpComm::handleResponse0x00010FD4(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size)
{
    if(!encodeResponsePackage(task->hash, ResponseMessageType_Uint64_DoubleList_fields, task->response_data_ptr, send_buffer_size))
    {
        FST_ERROR("handleResponse: failed to encode response package");// send
    }
    if(task->request_data_ptr != NULL)
    {
        delete (RequestMessageType_Int32List_DoubleList*)task->request_data_ptr;
    }
    if(task->response_data_ptr != NULL)
    {
        delete (ResponseMessageType_Uint64_DoubleList*)task->response_data_ptr;
    }
}

//"/rpc/motion_control/axis_group/convertJointToCart"
void TpComm::handleResponse0x0000B6D4(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size)
{
    if(!encodeResponsePackage(task->hash, ResponseMessageType_Uint64_DoubleList_fields, task->response_data_ptr, send_buffer_size))
    {
        FST_ERROR("handleResponse: failed to encode response package");// send
    }
    if(task->request_data_ptr != NULL)
    {
        delete (RequestMessageType_Int32List_DoubleList*)task->request_data_ptr;
    }
    if(task->response_data_ptr != NULL)
    {
        delete (ResponseMessageType_Uint64_DoubleList*)task->response_data_ptr;
    }
}

//"/rpc/motion_control/axis_group/ignoreLostZeroError"
void TpComm::handleResponse0x00014952(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size)
{
    if(!encodeResponsePackage(task->hash, ResponseMessageType_Uint64_fields, task->response_data_ptr, send_buffer_size))
    {
        FST_ERROR("handleResponse: failed to encode response package");// send
    }
    if(task->request_data_ptr != NULL)
    {
        delete (RequestMessageType_Int32*)task->request_data_ptr;
    }
    if(task->response_data_ptr != NULL)
    {
        delete (ResponseMessageType_Uint64*)task->response_data_ptr;
    }
}

//"/rpc/motion_control/axis_group/setAllZeroPointOffsets",	
void TpComm::handleResponse0x00008AB4(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size)
{
    if(!encodeResponsePackage(task->hash, ResponseMessageType_Uint64_fields, task->response_data_ptr, send_buffer_size))
    {
        FST_ERROR("handleResponse: failed to encode response package");// send
    }
    if(task->request_data_ptr != NULL)
    {
        delete (RequestMessageType_Int32_DoubleList*)task->request_data_ptr;
    }
    if(task->response_data_ptr != NULL)
    {
        delete (ResponseMessageType_Uint64*)task->response_data_ptr;
    }
}

//"/rpc/motion_control/axis_group/getAllZeroPointOffsets",	
void TpComm::handleResponse0x00012353(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size)
{
    if(!encodeResponsePackage(task->hash, ResponseMessageType_Uint64_DoubleList_fields, task->response_data_ptr, send_buffer_size))
    {
        FST_ERROR("handleResponse: failed to encode response package");// send
    }
    if(task->request_data_ptr != NULL)
    {
        delete (RequestMessageType_Int32*)task->request_data_ptr;
    }
    if(task->response_data_ptr != NULL)
    {
        delete (ResponseMessageType_Uint64_DoubleList*)task->response_data_ptr;
    }
}

//"/rpc/motion_control/axis_group/getAllZeroErrorMaskStatus",	
void TpComm::handleResponse0x0000C183(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size)
{
    if(!encodeResponsePackage(task->hash, ResponseMessageType_Uint64_Int32List_fields, task->response_data_ptr, send_buffer_size))
    {
        FST_ERROR("handleResponse: failed to encode response package");// send
    }
    if(task->request_data_ptr != NULL)
    {
        delete (RequestMessageType_Int32*)task->request_data_ptr;
    }
    if(task->response_data_ptr != NULL)
    {
        delete (ResponseMessageType_Uint64_Int32List*)task->response_data_ptr;
    }
}

//"/rpc/motion_control/axis_group/saveAllZeroPointOffsets",
void TpComm::handleResponse0x000171D3(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size)
{
    if(!encodeResponsePackage(task->hash, ResponseMessageType_Uint64_fields, task->response_data_ptr, send_buffer_size))
    {
        FST_ERROR("handleResponse: failed to encode response package");// send
    }
    if(task->request_data_ptr != NULL)
    {
        delete (RequestMessageType_Int32*)task->request_data_ptr;
    }
    if(task->response_data_ptr != NULL)
    {
        delete (ResponseMessageType_Uint64*)task->response_data_ptr;
    }
}

//"/rpc/motion_control/axis_group/setSingleZeroPointStatus"
void TpComm::handleResponse0x00010E43(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size)
{
    if(!encodeResponsePackage(task->hash, ResponseMessageType_Uint64_fields, task->response_data_ptr, send_buffer_size))
    {
        FST_ERROR("handleResponse: failed to encode response package");// send
    }
    if(task->request_data_ptr != NULL)
    {
        delete (RequestMessageType_Int32List*)task->request_data_ptr;
    }
    if(task->response_data_ptr != NULL)
    {
        delete (ResponseMessageType_Uint64*)task->response_data_ptr;
    }
}

//"/rpc/motion_control/axis_group/getAllZeroPointStatus"
void TpComm::handleResponse0x000102F3(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size)
{
    if(!encodeResponsePackage(task->hash, ResponseMessageType_Uint64_Int32List_fields, task->response_data_ptr, send_buffer_size))
    {
        FST_ERROR("handleResponse: failed to encode response package");// send
    }
    if(task->request_data_ptr != NULL)
    {
        delete (RequestMessageType_Int32*)task->request_data_ptr;
    }
    if(task->response_data_ptr != NULL)
    {
        delete (ResponseMessageType_Uint64_Int32List*)task->response_data_ptr;
    }
}

//"/rpc/motion_control/axis_group/calibrateAllZeroPointOffsets"
void TpComm::handleResponse0x00011B03(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size)
{
    if(!encodeResponsePackage(task->hash, ResponseMessageType_Uint64_fields, task->response_data_ptr, send_buffer_size))
    {
        FST_ERROR("handleResponse: failed to encode response package");// send
    }
    if(task->request_data_ptr != NULL)
    {
        delete (RequestMessageType_Int32*)task->request_data_ptr;
    }
    if(task->response_data_ptr != NULL)
    {
        delete (ResponseMessageType_Uint64*)task->response_data_ptr;
    }
}

//"/rpc/motion_control/axis_group/calibrateSingleZeroPointOffset"
void TpComm::handleResponse0x000131D4(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size)
{
    if(!encodeResponsePackage(task->hash, ResponseMessageType_Uint64_fields, task->response_data_ptr, send_buffer_size))
    {
        FST_ERROR("handleResponse: failed to encode response package");// send
    }
    if(task->request_data_ptr != NULL)
    {
        delete (RequestMessageType_Int32List*)task->request_data_ptr;
    }
    if(task->response_data_ptr != NULL)
    {
        delete (ResponseMessageType_Uint64*)task->response_data_ptr;
    }
}

//"/rpc/motion_control/axis_group/calibrateZeroPointOffsets"
void TpComm::handleResponse0x00005AE3(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size)
{
    if(!encodeResponsePackage(task->hash, ResponseMessageType_Uint64_fields, task->response_data_ptr, send_buffer_size))
    {
        FST_ERROR("handleResponse: failed to encode response package");// send
    }
    if(task->request_data_ptr != NULL)
    {
        delete (RequestMessageType_Int32_Int32List*)task->request_data_ptr;
    }
    if(task->response_data_ptr != NULL)
    {
        delete (ResponseMessageType_Uint64*)task->response_data_ptr;
    }
}

//"/rpc/motion_control/axis_group/isReferencePointExist"
void TpComm::handleResponse0x0000D344(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size)
{
    if(!encodeResponsePackage(task->hash, ResponseMessageType_Uint64_Bool_fields, task->response_data_ptr, send_buffer_size))
    {
        FST_ERROR("handleResponse: failed to encode response package");// send
    }
    if(task->request_data_ptr != NULL)
    {
        delete (RequestMessageType_Int32*)task->request_data_ptr;
    }
    if(task->response_data_ptr != NULL)
    {
        delete (ResponseMessageType_Uint64_Bool*)task->response_data_ptr;
    }
}

//"/rpc/motion_control/axis_group/deleteReferencePoint"
void TpComm::handleResponse0x00008744(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size)
{
    if(!encodeResponsePackage(task->hash, ResponseMessageType_Uint64_fields, task->response_data_ptr, send_buffer_size))
    {
        FST_ERROR("handleResponse: failed to encode response package");// send
    }
    if(task->request_data_ptr != NULL)
    {
        delete (RequestMessageType_Int32*)task->request_data_ptr;
    }
    if(task->response_data_ptr != NULL)
    {
        delete (ResponseMessageType_Uint64*)task->response_data_ptr;
    }
}

//"/rpc/motion_control/axis_group/saveReferencePoint"
void TpComm::handleResponse0x00006744(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size)
{
    if(!encodeResponsePackage(task->hash, ResponseMessageType_Uint64_fields, task->response_data_ptr, send_buffer_size))
    {
        FST_ERROR("handleResponse: failed to encode response package");// send
    }
    if(task->request_data_ptr != NULL)
    {
        delete (RequestMessageType_Int32*)task->request_data_ptr;
    }
    if(task->response_data_ptr != NULL)
    {
        delete (ResponseMessageType_Uint64*)task->response_data_ptr;
    }
}

//"/rpc/motion_control/axis_group/fastCalibrateAllZeroPointOffsets"
void TpComm::handleResponse0x0000E913(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size)
{
    if(!encodeResponsePackage(task->hash, ResponseMessageType_Uint64_fields, task->response_data_ptr, send_buffer_size))
    {
        FST_ERROR("handleResponse: failed to encode response package");// send
    }
    if(task->request_data_ptr != NULL)
    {
        delete (RequestMessageType_Int32*)task->request_data_ptr;
    }
    if(task->response_data_ptr != NULL)
    {
        delete (ResponseMessageType_Uint64*)task->response_data_ptr;
    }
}

//"/rpc/motion_control/axis_group/fastCalibrateSingleZeroPointOffset"
void TpComm::handleResponse0x00004754(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size)
{
    if(!encodeResponsePackage(task->hash, ResponseMessageType_Uint64_fields, task->response_data_ptr, send_buffer_size))
    {
        FST_ERROR("handleResponse: failed to encode response package");// send
    }
    if(task->request_data_ptr != NULL)
    {
        delete (RequestMessageType_Int32List*)task->request_data_ptr;
    }
    if(task->response_data_ptr != NULL)
    {
        delete (ResponseMessageType_Uint64*)task->response_data_ptr;
    }
}

//"/rpc/motion_control/axis_group/fastCalibrateZeroPointOffsets"
void TpComm::handleResponse0x00007EC3(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size)
{
    if(!encodeResponsePackage(task->hash, ResponseMessageType_Uint64_fields, task->response_data_ptr, send_buffer_size))
    {
        FST_ERROR("handleResponse: failed to encode response package");// send
    }
    if(task->request_data_ptr != NULL)
    {
        delete (RequestMessageType_Int32_Int32List*)task->request_data_ptr;
    }
    if(task->response_data_ptr != NULL)
    {
        delete (ResponseMessageType_Uint64*)task->response_data_ptr;
    }
}

//"/rpc/motion_control/getAxisGroupInfoList"
void TpComm::handleResponse0x00010F54(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size)
{
    if(!encodeResponsePackage(task->hash, ResponseMessageType_Uint64_AxisGroupInfoList_fields, task->response_data_ptr, send_buffer_size))
    {
        FST_ERROR("handleResponse: failed to encode response package");// send
    }
    if(task->request_data_ptr != NULL)
    {
        delete (RequestMessageType_Void*)task->request_data_ptr;
    }
    if(task->response_data_ptr != NULL)
    {
        delete (ResponseMessageType_Uint64_AxisGroupInfoList*)task->response_data_ptr;
    }
}

//"/rpc/motion_control/axis_group/getUserSoftLimitWithUnit"
void TpComm::handleResponse0x00008ED4(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size)
{
    if(!encodeResponsePackage(task->hash, ResponseMessageType_Uint64_JointLimitWithUnit_fields, task->response_data_ptr, send_buffer_size))
    {
        FST_ERROR("handleResponse: failed to encode response package");// send
    }
    if(task->request_data_ptr != NULL)
    {
        delete (RequestMessageType_Int32*)task->request_data_ptr;
    }
    if(task->response_data_ptr != NULL)
    {
        delete (ResponseMessageType_Uint64_JointLimitWithUnit*)task->response_data_ptr;
    }
}

//"/rpc/motion_control/axis_group/getManuSoftLimitWithUnit"
void TpComm::handleResponse0x000124E4(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size)
{
    if(!encodeResponsePackage(task->hash, ResponseMessageType_Uint64_JointLimitWithUnit_fields, task->response_data_ptr, send_buffer_size))
    {
        FST_ERROR("handleResponse: failed to encode response package");// send
    }
    if(task->request_data_ptr != NULL)
    {
        delete (RequestMessageType_Int32*)task->request_data_ptr;
    }
    if(task->response_data_ptr != NULL)
    {
        delete (ResponseMessageType_Uint64_JointLimitWithUnit*)task->response_data_ptr;
    }
}

//"/rpc/motion_control/axis_group/getHardLimitWithUnit"
void TpComm::handleResponse0x000092B4(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size)
{
    if(!encodeResponsePackage(task->hash, ResponseMessageType_Uint64_JointLimitWithUnit_fields, task->response_data_ptr, send_buffer_size))
    {
        FST_ERROR("handleResponse: failed to encode response package");// send
    }
    if(task->request_data_ptr != NULL)
    {
        delete (RequestMessageType_Int32*)task->request_data_ptr;
    }
    if(task->response_data_ptr != NULL)
    {
        delete (ResponseMessageType_Uint64_JointLimitWithUnit*)task->response_data_ptr;
    }
}

//"/rpc/motion_control/axis_group/setRotateManualStep"
void TpComm::handleResponse0x00005290(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size)
{
    if(!encodeResponsePackage(task->hash, ResponseMessageType_Uint64_fields, task->response_data_ptr, send_buffer_size))
    {
        FST_ERROR("handleResponse: failed to encode response package");// send
    }
    if(task->request_data_ptr != NULL)
    {
        delete (RequestMessageType_Int32_Double*)task->request_data_ptr;
    }
    if(task->response_data_ptr != NULL)
    {
        delete (ResponseMessageType_Uint64*)task->response_data_ptr;
    }
}

//"/rpc/motion_control/axis_group/getRotateManualStep"
void TpComm::handleResponse0x00003000(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size)
{
    if(!encodeResponsePackage(task->hash, ResponseMessageType_Uint64_Double_fields, task->response_data_ptr, send_buffer_size))
    {
        FST_ERROR("handleResponse: failed to encode response package");// send
    }
    if(task->request_data_ptr != NULL)
    {
        delete (RequestMessageType_Int32*)task->request_data_ptr;
    }
    if(task->response_data_ptr != NULL)
    {
        delete (ResponseMessageType_Uint64_Double*)task->response_data_ptr;
    }
}

//"/rpc/motion_control/axis_group/setPrismaticManualStep"
void TpComm::handleResponse0x0000B640(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size)
{
    if(!encodeResponsePackage(task->hash, ResponseMessageType_Uint64_fields, task->response_data_ptr, send_buffer_size))
    {
        FST_ERROR("handleResponse: failed to encode response package");// send
    }
    if(task->request_data_ptr != NULL)
    {
        delete (RequestMessageType_Int32_Double*)task->request_data_ptr;
    }
    if(task->response_data_ptr != NULL)
    {
        delete (ResponseMessageType_Uint64*)task->response_data_ptr;
    }
}

//"/rpc/motion_control/axis_group/getPrismaticManualStep"
void TpComm::handleResponse0x0000FCE0(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size)
{
    if(!encodeResponsePackage(task->hash, ResponseMessageType_Uint64_Double_fields, task->response_data_ptr, send_buffer_size))
    {
        FST_ERROR("handleResponse: failed to encode response package");// send
    }
    if(task->request_data_ptr != NULL)
    {
        delete (RequestMessageType_Int32*)task->request_data_ptr;
    }
    if(task->response_data_ptr != NULL)
    {
        delete (ResponseMessageType_Uint64_Double*)task->response_data_ptr;
    }
}

//"/rpc/motion_control/axis_group/setCartesianManualStep"
void TpComm::handleResponse0x0000A420(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size)
{
    if(!encodeResponsePackage(task->hash, ResponseMessageType_Uint64_fields, task->response_data_ptr, send_buffer_size))
    {
        FST_ERROR("handleResponse: failed to encode response package");// send
    }
    if(task->request_data_ptr != NULL)
    {
        delete (RequestMessageType_Int32_Double*)task->request_data_ptr;
    }
    if(task->response_data_ptr != NULL)
    {
        delete (ResponseMessageType_Uint64*)task->response_data_ptr;
    }
}

//"/rpc/motion_control/axis_group/getCartesianManualStep",
void TpComm::handleResponse0x0000EAC0(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size)
{
    if(!encodeResponsePackage(task->hash, ResponseMessageType_Uint64_Double_fields, task->response_data_ptr, send_buffer_size))
    {
        FST_ERROR("handleResponse: failed to encode response package");// send
    }
    if(task->request_data_ptr != NULL)
    {
        delete (RequestMessageType_Int32*)task->request_data_ptr;
    }
    if(task->response_data_ptr != NULL)
    {
        delete (ResponseMessageType_Uint64_Double*)task->response_data_ptr;
    }
}

//"/rpc/motion_control/axis_group/setOrientationManualStep"
void TpComm::handleResponse0x00002940(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size)
{
    if(!encodeResponsePackage(task->hash, ResponseMessageType_Uint64_fields, task->response_data_ptr, send_buffer_size))
    {
        FST_ERROR("handleResponse: failed to encode response package");// send
    }
    if(task->request_data_ptr != NULL)
    {
        delete (RequestMessageType_Int32_Double*)task->request_data_ptr;
    }
    if(task->response_data_ptr != NULL)
    {
        delete (ResponseMessageType_Uint64*)task->response_data_ptr;
    }
}

//"/rpc/motion_control/axis_group/getOrientationManualStep"
void TpComm::handleResponse0x00016D20(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size)
{
    if(!encodeResponsePackage(task->hash, ResponseMessageType_Uint64_Double_fields, task->response_data_ptr, send_buffer_size))
    {
        FST_ERROR("handleResponse: failed to encode response package");// send
    }
    if(task->request_data_ptr != NULL)
    {
        delete (RequestMessageType_Int32*)task->request_data_ptr;
    }
    if(task->response_data_ptr != NULL)
    {
        delete (ResponseMessageType_Uint64_Double*)task->response_data_ptr;
    }
}

//"/rpc/motion_control/axis_group/setSingleZeroPointOffset"
void TpComm::handleResponse0x00012404(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size)
{
    if(!encodeResponsePackage(task->hash, ResponseMessageType_Uint64_fields, task->response_data_ptr, send_buffer_size))
    {
        FST_ERROR("handleResponse: failed to encode response package");// send
    }
    if(task->request_data_ptr != NULL)
    {
        delete (RequestMessageType_Int32List_Double*)task->request_data_ptr;
    }
    if(task->response_data_ptr != NULL)
    {
        delete (ResponseMessageType_Uint64*)task->response_data_ptr;
    }
}

//"/rpc/motion_control/getPostureByJoint"
void TpComm::handleResponse0x0000EC64(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size)
{
    if(!encodeResponsePackage(task->hash, ResponseMessageType_Uint64_Int32List_fields, task->response_data_ptr, send_buffer_size))
    {
        FST_ERROR("handleResponse: failed to encode response package");// send
    }
    if(task->request_data_ptr != NULL)
    {
        delete (RequestMessageType_Int32_DoubleList*)task->request_data_ptr;
    }
    if(task->response_data_ptr != NULL)
    {
        delete (ResponseMessageType_Uint64_Int32List*)task->response_data_ptr;
    }
}
