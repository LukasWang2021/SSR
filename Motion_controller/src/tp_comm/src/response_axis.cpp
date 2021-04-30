#include "tp_comm.h"

using namespace user_space;
using namespace std;
using namespace log_space;


/********rpc/axis/mcPower, ResponseMessageType_Uint64**********/   
void TpComm::handleResponse0x000053E2(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size)
{
    if(!encodeResponsePackage(task->hash, ResponseMessageType_Uint64_fields, task->response_data_ptr, send_buffer_size))
    {
        LogProducer::error("rpc", "handleResponse0x000053E2: failed to encode response package");
    }
    if(task->request_data_ptr != NULL)
    {
        delete (RequestMessageType_Int32_Bool*)task->request_data_ptr;
    }
    if(task->response_data_ptr != NULL)
    {
        delete (ResponseMessageType_Uint64*)task->response_data_ptr;
    }
}

/********rpc/axis/mcReset, ResponseMessageType_Uint64**********/   
void TpComm::handleResponse0x000180C4(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size)
{
    if(!encodeResponsePackage(task->hash, ResponseMessageType_Uint64_fields, task->response_data_ptr, send_buffer_size))
    {
        LogProducer::error("rpc", "handleResponse0x000180C4: failed to encode response package");
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

/********rpc/axis/mcStop, ResponseMessageType_Uint64**********/
void TpComm::handleResponse0x00002820(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size)
{
    if(!encodeResponsePackage(task->hash, ResponseMessageType_Uint64_fields, task->response_data_ptr, send_buffer_size))
    {
        LogProducer::error("rpc", "handleResponse0x00002820: failed to encode response package");
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

/********rpc/axis/mcHalt, ResponseMessageType_Uint64**********/ 
void TpComm::handleResponse0x00004BB4(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size)
{
    if(!encodeResponsePackage(task->hash, ResponseMessageType_Uint64_fields, task->response_data_ptr, send_buffer_size))
    {
        LogProducer::error("rpc", "handleResponse0x00004BB4: failed to encode response package");
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

/********rpc/axis/mcSetPosition, ResponseMessageType_Uint64**********/  
void TpComm::handleResponse0x0001798E(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size)
{
    if(!encodeResponsePackage(task->hash, ResponseMessageType_Uint64_fields, task->response_data_ptr, send_buffer_size))
    {
        LogProducer::error("rpc", "handleResponse0x0001798E: failed to encode response package");
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

/********rpc/axis/mcReadParameter, ResponseMessageType_Uint64_Int32**********/ 
void TpComm::handleResponse0x00016BF2(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size)
{
    if(!encodeResponsePackage(task->hash, ResponseMessageType_Uint64_Int32_fields, task->response_data_ptr, send_buffer_size))
    {
        LogProducer::error("rpc", "handleResponse0x00016BF2: failed to encode response package");
    }
    if(task->request_data_ptr != NULL)
    {
        delete (RequestMessageType_Int32List*)task->request_data_ptr;
    }
    if(task->response_data_ptr != NULL)
    {
        delete (ResponseMessageType_Uint64_Int32*)task->response_data_ptr;
    }
}

/********rpc/axis/mcWriteParameter, ResponseMessageType_Uint64**********/  
void TpComm::handleResponse0x00005732(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size)
{
    if(!encodeResponsePackage(task->hash, ResponseMessageType_Uint64_fields, task->response_data_ptr, send_buffer_size))
    {
        LogProducer::error("rpc", "handleResponse0x00005732: failed to encode response package");
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

/********rpc/axis/mcMoveAbsolute, ResponseMessageType_Uint64**********/ 
void TpComm::handleResponse0x000051F5(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size)
{
    if(!encodeResponsePackage(task->hash, ResponseMessageType_Uint64_fields, task->response_data_ptr, send_buffer_size))
    {
        LogProducer::error("rpc", "handleResponse0x000051F5: failed to encode response package");
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

/********rpc/axis/mcMoveVelocity, ResponseMessageType_Uint64**********/ 
void TpComm::handleResponse0x00016CF9(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size)
{
    if(!encodeResponsePackage(task->hash, ResponseMessageType_Uint64_fields, task->response_data_ptr, send_buffer_size))
    {
        LogProducer::error("rpc", "handleResponse0x00016CF9: failed to encode response package");
    }
    if(task->request_data_ptr != NULL)
    {
        delete (RequestMessageType_Int32List_DoubleList*)task->request_data_ptr;
    }
    if(task->response_data_ptr != NULL)
    {
        delete (ResponseMessageType_Uint64*)task->response_data_ptr;
    }
}

/********rpc/axis/mcReadActualPosition, ResponseMessageType_Uint64_Double**********/   
void TpComm::handleResponse0x000012BE(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size)
{
    if(!encodeResponsePackage(task->hash, ResponseMessageType_Uint64_Double_fields, task->response_data_ptr, send_buffer_size))
    {
        LogProducer::error("rpc", "handleResponse0x000012BE: failed to encode response package");
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

/********rpc/axis/mcReadActualVelocity, ResponseMessageType_Uint64_Double**********/  
void TpComm::handleResponse0x00002EA9(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size)
{
    if(!encodeResponsePackage(task->hash, ResponseMessageType_Uint64_Double_fields, task->response_data_ptr, send_buffer_size))
    {
        LogProducer::error("rpc", "handleResponse0x00002EA9: failed to encode response package");
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

/********rpc/axis/mcReadActualTorque, ResponseMessageType_Uint64_Double**********/ 
void TpComm::handleResponse0x00014265(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size)
{
    if(!encodeResponsePackage(task->hash, ResponseMessageType_Uint64_Double_fields, task->response_data_ptr, send_buffer_size))
    {
        LogProducer::error("rpc", "handleResponse0x00014265: failed to encode response package");
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

/********rpc/axis/mcReadAxisInfo, ResponseMessageType_Uint64_AxisInfo**********/  
void TpComm::handleResponse0x0000314F(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size)
{
    if(!encodeResponsePackage(task->hash, ResponseMessageType_Uint64_AxisInfo_fields, task->response_data_ptr, send_buffer_size))
    {
        LogProducer::error("rpc", "handleResponse0x0000314F: failed to encode response package");
    }
    if(task->request_data_ptr != NULL)
    {
        delete (RequestMessageType_Int32*)task->request_data_ptr;
    }
    if(task->response_data_ptr != NULL)
    {
        delete (ResponseMessageType_Uint64_AxisInfo*)task->response_data_ptr;
    }
}

/********rpc/axis/mcReadStatus, ResponseMessageType_Uint64_AxisStatus**********/   
void TpComm::handleResponse0x00003E53(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size)
{
    if(!encodeResponsePackage(task->hash, ResponseMessageType_Uint64_AxisStatus_fields, task->response_data_ptr, send_buffer_size))
    {
        LogProducer::error("rpc", "handleResponse0x00003E53: failed to encode response package");
    }
    if(task->request_data_ptr != NULL)
    {
        delete (RequestMessageType_Int32*)task->request_data_ptr;
    }
    if(task->response_data_ptr != NULL)
    {
        delete (ResponseMessageType_Uint64_AxisStatus*)task->response_data_ptr;
    }
}

/********rpc/axis/mcReadAxisError, ResponseMessageType_Uint64_Uint64**********/  
void TpComm::handleResponse0x000063C2(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size)
{
    if(!encodeResponsePackage(task->hash, ResponseMessageType_Uint64_Uint64_fields, task->response_data_ptr, send_buffer_size))
    {
        LogProducer::error("rpc", "handleResponse0x000063C2: failed to encode response package");
    }
    if(task->request_data_ptr != NULL)
    {
        delete (RequestMessageType_Int32*)task->request_data_ptr;
    }
    if(task->response_data_ptr != NULL)
    {
        delete (ResponseMessageType_Uint64_Uint64*)task->response_data_ptr;
    }
}

/********rpc/axis/mcReadAxisErrorHistory, ResponseMessageType_Uint64_Uint64List(count=8)**********/ 
void TpComm::handleResponse0x00018469(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size)
{
    if(!encodeResponsePackage(task->hash, ResponseMessageType_Uint64_Uint64List_fields, task->response_data_ptr, send_buffer_size))
    {
        LogProducer::error("rpc", "handleResponse0x00018469: failed to encode response package");
    }
    if(task->request_data_ptr != NULL)
    {
        delete (RequestMessageType_Int32*)task->request_data_ptr;
    }
    if(task->response_data_ptr != NULL)
    {
        delete (ResponseMessageType_Uint64_Uint64List*)task->response_data_ptr;
    }
}

/********rpc/axis/mcMoveRelative, ResponseMessageType_Uint64**********/	
void TpComm::handleResponse0x0000CC85(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size)
{
    if(!encodeResponsePackage(task->hash, ResponseMessageType_Uint64_fields, task->response_data_ptr, send_buffer_size))
    {
        LogProducer::error("rpc", "handleResponse0x0000CC85: failed to encode response package");
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
/********rpc/axis/mcHome, ResponseMessageType_Uint64**********/	
void TpComm::handleResponse0x000059B5(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size)
{
    if(!encodeResponsePackage(task->hash, ResponseMessageType_Uint64_fields, task->response_data_ptr, send_buffer_size))
    {
        LogProducer::error("rpc", "handleResponse0x000059B5: failed to encode response package");
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

/********rpc/axis/rtmAbortHoming, ResponseMessageType_Uint64**********/	
void TpComm::handleResponse0x0000E4B7(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size)
{
    if(!encodeResponsePackage(task->hash, ResponseMessageType_Uint64_fields, task->response_data_ptr, send_buffer_size))
    {
        LogProducer::error("rpc", "handleResponse0x0000E4B7: failed to encode response package");
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

/********rpc/axis/rtmReadAxisFdbPdoPtr, ResponseMessageType_Uint64_String**********/	
void TpComm::handleResponse0x0000A632(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size)
{
    if(!encodeResponsePackage(task->hash, ResponseMessageType_Uint64_Int32List_fields, task->response_data_ptr, send_buffer_size))
    {
        LogProducer::error("rpc", "handleResponse0x0000A632: failed to encode response package");
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

/********rpc/axis/rtmResetEncoder, ResponseMessageType_Uint64**********/
void TpComm::handleResponse0x00000BA2(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size)
{
    if(!encodeResponsePackage(task->hash, ResponseMessageType_Uint64_fields, task->response_data_ptr, send_buffer_size))
    {
        LogProducer::error("rpc", "handleResponse0x00000BA2: failed to encode response package");
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
