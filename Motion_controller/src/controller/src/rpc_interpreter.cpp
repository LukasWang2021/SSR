#include "controller_rpc.h"
#include <sys/time.h>
#include <unistd.h>
#include <list>
#include "interpreter_control.h"

using namespace user_space;
using namespace base_space;
using namespace log_space;


//"/rpc/interpreter/start"	
void ControllerRpc::handleRpc0x00006154(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_String* rq_data_ptr = static_cast<RequestMessageType_String*>(request_data_ptr);
    ResponseMessageType_Uint64* rs_data_ptr = static_cast<ResponseMessageType_Uint64*>(response_data_ptr);

    rs_data_ptr->data.data = InterpCtrl::instance().start(rq_data_ptr->data.data);
    
    LogProducer::info("rpc", "/rpc/interpreter/start %s", rq_data_ptr->data.data);
}

//"/rpc/interpreter/pause"	
void ControllerRpc::handleRpc0x0000BA55(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_Int32* rq_data_ptr = static_cast<RequestMessageType_Int32*>(request_data_ptr);
    ResponseMessageType_Uint64* rs_data_ptr = static_cast<ResponseMessageType_Uint64*>(response_data_ptr);

    rs_data_ptr->data.data = InterpCtrl::instance().pause(rq_data_ptr->data.data);
    
    LogProducer::info("rpc", "/rpc/interpreter/pause %d", rq_data_ptr->data.data);
}

//"/rpc/interpreter/resume"	
void ControllerRpc::handleRpc0x0000CF55(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_Int32* rq_data_ptr = static_cast<RequestMessageType_Int32*>(request_data_ptr);
    ResponseMessageType_Uint64* rs_data_ptr = static_cast<ResponseMessageType_Uint64*>(response_data_ptr);

    rs_data_ptr->data.data = InterpCtrl::instance().resume(rq_data_ptr->data.data);
    
    LogProducer::info("rpc", "/rpc/interpreter/resume %d", rq_data_ptr->data.data);
}

//"/rpc/interpreter/abort"	
void ControllerRpc::handleRpc0x000086F4(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_Int32* rq_data_ptr = static_cast<RequestMessageType_Int32*>(request_data_ptr);
    ResponseMessageType_Uint64* rs_data_ptr = static_cast<ResponseMessageType_Uint64*>(response_data_ptr);

    rs_data_ptr->data.data = InterpCtrl::instance().abort(rq_data_ptr->data.data);
    
    LogProducer::info("rpc", "/rpc/interpreter/abort %d", rq_data_ptr->data.data);
}

//"/rpc/interpreter/forward"	
void ControllerRpc::handleRpc0x0000D974(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_Int32* rq_data_ptr = static_cast<RequestMessageType_Int32*>(request_data_ptr);
    ResponseMessageType_Uint64* rs_data_ptr = static_cast<ResponseMessageType_Uint64*>(response_data_ptr);

    rs_data_ptr->data.data = InterpCtrl::instance().forward(rq_data_ptr->data.data);
    
    LogProducer::info("rpc", "/rpc/interpreter/forward %d", rq_data_ptr->data.data);
}

//"/rpc/interpreter/backward"	
void ControllerRpc::handleRpc0x00008E74(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_Int32* rq_data_ptr = static_cast<RequestMessageType_Int32*>(request_data_ptr);
    ResponseMessageType_Uint64* rs_data_ptr = static_cast<ResponseMessageType_Uint64*>(response_data_ptr);

    rs_data_ptr->data.data = InterpCtrl::instance().forward(rq_data_ptr->data.data);
    
    LogProducer::info("rpc", "/rpc/interpreter/forward %d", rq_data_ptr->data.data);
}

//"/rpc/interpreter/jump"	
void ControllerRpc::handleRpc0x00015930(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_Int32* rq_data_ptr = static_cast<RequestMessageType_Int32*>(request_data_ptr);
    ResponseMessageType_Uint64* rs_data_ptr = static_cast<ResponseMessageType_Uint64*>(response_data_ptr);

    rs_data_ptr->data.data = InterpCtrl::instance().jumpLine(rq_data_ptr->data.data);
    
    LogProducer::info("rpc", "/rpc/interpreter/jump %d", rq_data_ptr->data.data);
}
