#include "controller_rpc.h"

using namespace fst_ctrl;

// "/rpc/io_mapping/getDIByBit"
void ControllerRpc::handleRpc0x000050B4(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_Int32* rq_data_ptr = static_cast<RequestMessageType_Int32*>(request_data_ptr);
    ResponseMessageType_Uint64_Int32* rs_data_ptr = static_cast<ResponseMessageType_Uint64_Int32*>(response_data_ptr);

    uint8_t value = 0;
    rs_data_ptr->error_code.data = io_mapping_ptr_->getDIByBit(rq_data_ptr->data.data, value);
    FST_INFO("rpc-getDIByBit: user_port=%d, value=%d, ret=%llx\n", rq_data_ptr->data.data, value, rs_data_ptr->error_code.data);

    if(rs_data_ptr->error_code.data == SUCCESS)
    {
        rs_data_ptr->data.data = value;
    }
    else
    {
        memset(&rs_data_ptr->data, 0, sizeof(MessageType_Int32));
    }

    if (rs_data_ptr->error_code.data != SUCCESS)
        recordLog(IO_MAPPING_LOG, rs_data_ptr->error_code.data, std::string("/rpc/io_mapping/getDIByBit"));
}

// "/rpc/io_mapping/setDIByBit"
void ControllerRpc::handleRpc0x00011754(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_Int32List* rq_data_ptr = static_cast<RequestMessageType_Int32List*>(request_data_ptr);
    ResponseMessageType_Uint64* rs_data_ptr = static_cast<ResponseMessageType_Uint64*>(response_data_ptr);

    if(rq_data_ptr->data.data_count == 2)
    {
        rs_data_ptr->data.data = SUCCESS;
    }
    else
    {
        rs_data_ptr->data.data = INVALID_PARAMETER;
    }

    rs_data_ptr->data.data = io_mapping_ptr_->setDIByBit(rq_data_ptr->data.data[0], rq_data_ptr->data.data[1]);
    FST_INFO("rpc-setDIByBit: user_port=%d, value=%d, ret =%x\n", rq_data_ptr->data.data[0], rq_data_ptr->data.data[1], rs_data_ptr->data.data);

    if (rs_data_ptr->data.data != SUCCESS)
        recordLog(IO_MAPPING_LOG, rs_data_ptr->data.data, std::string("/rpc/io_mapping/setDIByBit"));
}

// "/rpc/io_mapping/getDOByBit"
void ControllerRpc::handleRpc0x00013074(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_Int32* rq_data_ptr = static_cast<RequestMessageType_Int32*>(request_data_ptr);
    ResponseMessageType_Uint64_Int32* rs_data_ptr = static_cast<ResponseMessageType_Uint64_Int32*>(response_data_ptr);

    uint8_t value = 0;
    rs_data_ptr->error_code.data =io_mapping_ptr_->getDOByBit(rq_data_ptr->data.data, value);
    FST_INFO("rpc-getDOByBit: user_port=%d, value=%d, ret =%x\n", rq_data_ptr->data.data, value, rs_data_ptr->error_code.data);
    if(rs_data_ptr->error_code.data == SUCCESS)
    {
        rs_data_ptr->data.data = value;
    }
    else
    {
        memset(&rs_data_ptr->data, 0, sizeof(MessageType_Int32));
    }

    if (rs_data_ptr->error_code.data != SUCCESS)
        recordLog(IO_MAPPING_LOG, rs_data_ptr->error_code.data, std::string("/rpc/io_mapping/getDOByBit"));
}

// "/rpc/io_mapping/setDOByBit"
void ControllerRpc::handleRpc0x00007074(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_Int32List* rq_data_ptr = static_cast<RequestMessageType_Int32List*>(request_data_ptr);
    ResponseMessageType_Uint64* rs_data_ptr = static_cast<ResponseMessageType_Uint64*>(response_data_ptr);

    if(rq_data_ptr->data.data_count == 2)
    {
        rs_data_ptr->data.data = SUCCESS;
    }
    else
    {
        rs_data_ptr->data.data = INVALID_PARAMETER;
    }

    //TP can not control ouput if the setting of isAvailableInAutoMode is false.
    if ((!io_mapping_ptr_->isEnableInAutoMode()) && (state_machine_ptr_->getUserOpMode() == USER_OP_MODE_AUTO))
    {
        rs_data_ptr->data.data = CONTROLLER_INVALID_OPERATION;
    }
    else
    {
        rs_data_ptr->data.data = io_mapping_ptr_->setDOByBit(rq_data_ptr->data.data[0], rq_data_ptr->data.data[1]);
        FST_INFO("rpc-setDOByBit: user_port=%d, value=%d, ret=%x\n", rq_data_ptr->data.data[0], rq_data_ptr->data.data[1],rs_data_ptr->data.data);
    }
    
    if (rs_data_ptr->data.data != SUCCESS)
        recordLog(IO_MAPPING_LOG, rs_data_ptr->data.data, std::string("/rpc/io_mapping/setDOByBit"));
}


// "/rpc/io_mapping/getRIByBit"
void ControllerRpc::handleRpc0x00000684(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_Int32* rq_data_ptr = static_cast<RequestMessageType_Int32*>(request_data_ptr);
    ResponseMessageType_Uint64_Int32* rs_data_ptr = static_cast<ResponseMessageType_Uint64_Int32*>(response_data_ptr);

    uint8_t value = 0;
    rs_data_ptr->error_code.data = io_mapping_ptr_->getRIByBit(rq_data_ptr->data.data, value);
    FST_INFO("rpc-getRIByBit: user_port=%d, value=%d, ret =%x\n", rq_data_ptr->data.data, value, rs_data_ptr->error_code.data);
    if(rs_data_ptr->error_code.data == SUCCESS)
    {
        rs_data_ptr->data.data = value;
    }
    else
    {
        memset(&rs_data_ptr->data, 0, sizeof(MessageType_Int32));
    }

    if (rs_data_ptr->error_code.data != SUCCESS)
        recordLog(IO_MAPPING_LOG, rs_data_ptr->error_code.data, std::string("/rpc/io_mapping/getRIByBit"));
}

// "/rpc/io_mapping/setRIByBit"
void ControllerRpc::handleRpc0x0000CD24(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_Int32List* rq_data_ptr = static_cast<RequestMessageType_Int32List*>(request_data_ptr);
    ResponseMessageType_Uint64* rs_data_ptr = static_cast<ResponseMessageType_Uint64*>(response_data_ptr);

    if(rq_data_ptr->data.data_count == 2)
    {
        rs_data_ptr->data.data = SUCCESS;
    }
    else
    {
        rs_data_ptr->data.data = INVALID_PARAMETER;
    }

    rs_data_ptr->data.data = io_mapping_ptr_->setRIByBit(rq_data_ptr->data.data[0], rq_data_ptr->data.data[1]);
    FST_INFO("rpc-setRIByBit: user_port=%d, value=%d, ret =%x\n", rq_data_ptr->data.data[0], rq_data_ptr->data.data[1], rs_data_ptr->data.data);

    if (rs_data_ptr->data.data != SUCCESS)
        recordLog(IO_MAPPING_LOG, rs_data_ptr->data.data, std::string("/rpc/io_mapping/setRIByBit"));
}

// "/rpc/io_mapping/getROByBit"
void ControllerRpc::handleRpc0x00005BD4(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_Int32* rq_data_ptr = static_cast<RequestMessageType_Int32*>(request_data_ptr);
    ResponseMessageType_Uint64_Int32* rs_data_ptr = static_cast<ResponseMessageType_Uint64_Int32*>(response_data_ptr);

    uint8_t value = 0;
    rs_data_ptr->error_code.data =io_mapping_ptr_->getROByBit(rq_data_ptr->data.data, value);
    FST_INFO("rpc-getROByBit: user_port=%d, value=%d, ret =%x\n", rq_data_ptr->data.data, value, rs_data_ptr->error_code.data);
    if(rs_data_ptr->error_code.data == SUCCESS)
    {
        rs_data_ptr->data.data = value;
    }
    else
    {
        memset(&rs_data_ptr->data, 0, sizeof(MessageType_Int32));
    }

    if (rs_data_ptr->error_code.data != SUCCESS)
        recordLog(IO_MAPPING_LOG, rs_data_ptr->error_code.data, std::string("/rpc/io_mapping/getROByBit"));
}

// "/rpc/io_mapping/setROByBit"
void ControllerRpc::handleRpc0x00012274(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_Int32List* rq_data_ptr = static_cast<RequestMessageType_Int32List*>(request_data_ptr);
    ResponseMessageType_Uint64* rs_data_ptr = static_cast<ResponseMessageType_Uint64*>(response_data_ptr);

    if(rq_data_ptr->data.data_count == 2)
    {
        rs_data_ptr->data.data = SUCCESS;
    }
    else
    {
        rs_data_ptr->data.data = INVALID_PARAMETER;
    }

    //TP can not control ouput if the setting of isAvailableInAutoMode is false.
    if ((!io_mapping_ptr_->isEnableInAutoMode()) && (state_machine_ptr_->getUserOpMode() == USER_OP_MODE_AUTO))
    {
        rs_data_ptr->data.data = CONTROLLER_INVALID_OPERATION;
    }
    else
    {
        rs_data_ptr->data.data = io_mapping_ptr_->setROByBit(rq_data_ptr->data.data[0], rq_data_ptr->data.data[1]);
        FST_INFO("rpc-setROByBit: user_port=%d, value=%d, ret =%x\n", rq_data_ptr->data.data[0], rq_data_ptr->data.data[1], rs_data_ptr->data.data);
    }
    
    if (rs_data_ptr->data.data != SUCCESS)
        recordLog(IO_MAPPING_LOG, rs_data_ptr->data.data, std::string("/rpc/io_mapping/setROByBit"));
}

//"/rpc/io_mapping/getUIByBit"
void ControllerRpc::handleRpc0x0000A9A4(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_Int32* rq_data_ptr = static_cast<RequestMessageType_Int32*>(request_data_ptr);
    ResponseMessageType_Uint64_Int32* rs_data_ptr = static_cast<ResponseMessageType_Uint64_Int32*>(response_data_ptr);

    uint8_t value = 0;
    rs_data_ptr->error_code.data = io_mapping_ptr_->getUIByBit(rq_data_ptr->data.data, value);
    FST_INFO("rpc-getUIByBit: user_port=%d, value=%d, ret=%llx\n", rq_data_ptr->data.data, value, rs_data_ptr->error_code.data);

    if(rs_data_ptr->error_code.data == SUCCESS)
    {
        rs_data_ptr->data.data = value;
    }
    else
    {
        memset(&rs_data_ptr->data, 0, sizeof(MessageType_Int32));
    }

    if (rs_data_ptr->error_code.data != SUCCESS)
        recordLog(IO_MAPPING_LOG, rs_data_ptr->error_code.data, std::string("/rpc/io_mapping/getUIByBit"));
}

//"/rpc/io_mapping/setUIByBit"
void ControllerRpc::handleRpc0x00017044(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_Int32List* rq_data_ptr = static_cast<RequestMessageType_Int32List*>(request_data_ptr);
    ResponseMessageType_Uint64* rs_data_ptr = static_cast<ResponseMessageType_Uint64*>(response_data_ptr);

    if(rq_data_ptr->data.data_count == 2)
    {
        rs_data_ptr->data.data = SUCCESS;
    }
    else
    {
        rs_data_ptr->data.data = INVALID_PARAMETER;
    }

    rs_data_ptr->data.data = io_mapping_ptr_->setUIByBit(rq_data_ptr->data.data[0], rq_data_ptr->data.data[1]);
    FST_INFO("rpc-setUIByBit: user_port=%d, bypass or not=%d, ret =%x\n", rq_data_ptr->data.data[0], rq_data_ptr->data.data[1], rs_data_ptr->data.data);

    if (rs_data_ptr->data.data != SUCCESS)
        recordLog(IO_MAPPING_LOG, rs_data_ptr->data.data, std::string("/rpc/io_mapping/setUIByBit"));
}

//"/rpc/io_mapping/getUOByBit"
void ControllerRpc::handleRpc0x000002C4(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_Int32* rq_data_ptr = static_cast<RequestMessageType_Int32*>(request_data_ptr);
    ResponseMessageType_Uint64_Int32* rs_data_ptr = static_cast<ResponseMessageType_Uint64_Int32*>(response_data_ptr);

    uint8_t value = 0;
    rs_data_ptr->error_code.data =io_mapping_ptr_->getUOByBit(rq_data_ptr->data.data, value);
    FST_INFO("rpc-getUOByBit: user_port=%d, value=%d, ret =%x\n", rq_data_ptr->data.data, value, rs_data_ptr->error_code.data);
    if(rs_data_ptr->error_code.data == SUCCESS)
    {
        rs_data_ptr->data.data = value;
    }
    else
    {
        memset(&rs_data_ptr->data, 0, sizeof(MessageType_Int32));
    }

    if (rs_data_ptr->error_code.data != SUCCESS)
        recordLog(IO_MAPPING_LOG, rs_data_ptr->error_code.data, std::string("/rpc/io_mapping/getUOByBit"));
}


// "/rpc/io_mapping/syncFileIoStatus"
void ControllerRpc::handleRpc0x0000BA73(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_String* rq_data_ptr = static_cast<RequestMessageType_String*>(request_data_ptr);
    ResponseMessageType_Uint64* rs_data_ptr = static_cast<ResponseMessageType_Uint64*>(response_data_ptr);

    rs_data_ptr->data.data =io_mapping_ptr_->updateSimFile();

    if (rs_data_ptr->data.data != SUCCESS)
        recordLog(IO_MAPPING_LOG, rs_data_ptr->data.data, std::string("/rpc/io_mapping/syncFileIoMapping"));
}

// "/rpc/io_mapping/syncFileIoMapping"
void ControllerRpc::handleRpc0x0000C2A7(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_StringList* rq_data_ptr = static_cast<RequestMessageType_StringList*>(request_data_ptr);
    ResponseMessageType_Uint64* rs_data_ptr = static_cast<ResponseMessageType_Uint64*>(response_data_ptr);

    rs_data_ptr->data.data =io_mapping_ptr_->updateMappingFile();

    if (rs_data_ptr->data.data != SUCCESS)
        recordLog(IO_MAPPING_LOG, rs_data_ptr->data.data, std::string("/rpc/io_mapping/syncFileIoMapping"));
}
