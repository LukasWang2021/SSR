#include "controller_rpc.h"

using namespace fst_ctrl;

//"/rpc/io_manager/getDIByBit"	
void ControllerRpc::handleRpc0x0000BFE4(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_Int32List* rq_data_ptr = static_cast<RequestMessageType_Int32List*>(request_data_ptr);
    ResponseMessageType_Uint64_Int32* rs_data_ptr = static_cast<ResponseMessageType_Uint64_Int32*>(response_data_ptr);

    PhysicsID id;
    memset(&id, 0, sizeof(id));
    id.info.dev_type = rq_data_ptr->data.data[0];
    id.info.address = rq_data_ptr->data.data[1];
    id.info.port_type = MessageType_IoType_DI;
    id.info.port = rq_data_ptr->data.data[2];
    uint8_t value = 0;

    rs_data_ptr->error_code.data = io_manager_ptr_->getBitValue(id, value);
    if (rs_data_ptr->error_code.data ==  SUCCESS)
    {
        rs_data_ptr->data.data = value;
    }
    
    FST_INFO("/rpc/io_manager/getDIByBit, dev_type=%d, address=%d, port_type=%d, port_offset=%d, ret = 0x%llx", 
                 id.info.dev_type, id.info.address, id.info.port_type, id.info.port, rs_data_ptr->error_code.data);
}

//"/rpc/io_manager/setDIByBit"	
void ControllerRpc::handleRpc0x00018684(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_Int32List* rq_data_ptr = static_cast<RequestMessageType_Int32List*>(request_data_ptr);
    ResponseMessageType_Uint64* rs_data_ptr = static_cast<ResponseMessageType_Uint64*>(response_data_ptr);

    PhysicsID id;
    memset(&id, 0, sizeof(id));
    id.info.dev_type = rq_data_ptr->data.data[0];
    id.info.address = rq_data_ptr->data.data[1];
    id.info.port_type = MessageType_IoType_DI;
    id.info.port = rq_data_ptr->data.data[2];
    uint8_t value = rq_data_ptr->data.data[3];

    rs_data_ptr->data.data = io_manager_ptr_->setBitValue(id, value);

    FST_INFO("/rpc/io_manager/setDIByBit, dev_type=%d, address=%d, port_type=%d, port_offset=%d, ret = 0x%llx", 
                 id.info.dev_type, id.info.address, id.info.port_type, id.info.port, rs_data_ptr->data.data);
}

//"/rpc/io_manager/getDOByBit"	
void ControllerRpc::handleRpc0x0000B4C4(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_Int32List* rq_data_ptr = static_cast<RequestMessageType_Int32List*>(request_data_ptr);
    ResponseMessageType_Uint64_Int32* rs_data_ptr = static_cast<ResponseMessageType_Uint64_Int32*>(response_data_ptr);
    
    PhysicsID id;
    memset(&id, 0, sizeof(id));
    id.info.dev_type = rq_data_ptr->data.data[0];
    id.info.address = rq_data_ptr->data.data[1];
    id.info.port_type = MessageType_IoType_DO;
    id.info.port = rq_data_ptr->data.data[2];
    uint8_t value = 0;

    rs_data_ptr->error_code.data = io_manager_ptr_->getBitValue(id, value);
    if (rs_data_ptr->error_code.data ==  SUCCESS)
    {
        rs_data_ptr->data.data = value;
    }

    FST_INFO("/rpc/io_manager/getDOyBit, dev_type=%d, address=%d, port_type=%d, port_offset=%d, ret = 0x%llx", 
                 id.info.dev_type, id.info.address, id.info.port_type, id.info.port, rs_data_ptr->error_code.data);
}
//"/rpc/io_manager/setDOByBit"	
void ControllerRpc::handleRpc0x00017B64(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_Int32List* rq_data_ptr = static_cast<RequestMessageType_Int32List*>(request_data_ptr);
    ResponseMessageType_Uint64* rs_data_ptr = static_cast<ResponseMessageType_Uint64*>(response_data_ptr);

    PhysicsID id;
    memset(&id, 0, sizeof(id));
    id.info.dev_type = rq_data_ptr->data.data[0];
    id.info.address = rq_data_ptr->data.data[1];
    id.info.port_type = MessageType_IoType_DO;
    id.info.port = rq_data_ptr->data.data[2];
    uint8_t value = rq_data_ptr->data.data[3];

    rs_data_ptr->data.data = io_manager_ptr_->setBitValue(id, value);

    FST_INFO("/rpc/io_manager/setDOByBit, dev_type=%d, address=%d, port_type=%d, port_offset=%d, ret = 0x%llx", 
                 id.info.dev_type, id.info.address, id.info.port_type, id.info.port, rs_data_ptr->data.data);
}

