#include "controller_rpc.h"
#include "error_code.h"


using namespace fst_ctrl;

// "/rpc/reg_manager/r/addReg"
void ControllerRpc::handleRpc0x00004FF7(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_RRegData* rq_data_ptr = static_cast<RequestMessageType_RRegData*>(request_data_ptr);
    ResponseMessageType_Uint64* rs_data_ptr = static_cast<ResponseMessageType_Uint64*>(response_data_ptr);

    if (false == state_machine_ptr_->getState())
    {
        rs_data_ptr->data.data = CONTROLLER_INVALID_OPERATION;
        FST_INFO("/rpc/reg_manager/r/addReg can't run when backup/restore, ret = %llx\n", rs_data_ptr->data.data);
    }

    RRegData reg;
    reg.id = rq_data_ptr->data.id;
    reg.name = rq_data_ptr->data.name;
    reg.comment = rq_data_ptr->data.comment;
    reg.value = rq_data_ptr->data.value;
    rs_data_ptr->data.data = reg_manager_ptr_->addRReg(&reg);

    if (rs_data_ptr->data.data != SUCCESS)
        recordLog(REG_MANAGER_LOG, rs_data_ptr->data.data, std::string("/rpc/reg_manager/r/addReg"));
}

// "/rpc/reg_manager/r/deleteReg"
void ControllerRpc::handleRpc0x000012F7(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_Int32* rq_data_ptr = static_cast<RequestMessageType_Int32*>(request_data_ptr);
    ResponseMessageType_Uint64* rs_data_ptr = static_cast<ResponseMessageType_Uint64*>(response_data_ptr);

    if (false == state_machine_ptr_->getState())
    {
        rs_data_ptr->data.data = CONTROLLER_INVALID_OPERATION;
        FST_INFO("/rpc/reg_manager/r/deleteReg can't run when backup/restore, ret = %llx\n", rs_data_ptr->data.data);
    }

    rs_data_ptr->data.data = reg_manager_ptr_->deleteRReg(rq_data_ptr->data.data);

    if (rs_data_ptr->data.data != SUCCESS)
        recordLog(REG_MANAGER_LOG, rs_data_ptr->data.data, std::string("/rpc/reg_manager/r/deleteReg"));
}

// "/rpc/reg_manager/r/updateReg"
void ControllerRpc::handleRpc0x00005757(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_RRegData* rq_data_ptr = static_cast<RequestMessageType_RRegData*>(request_data_ptr);
    ResponseMessageType_Uint64* rs_data_ptr = static_cast<ResponseMessageType_Uint64*>(response_data_ptr);

    if (false == state_machine_ptr_->getState())
    {
        rs_data_ptr->data.data = CONTROLLER_INVALID_OPERATION;
        FST_INFO("/rpc/reg_manager/r/updateReg can't run when backup/restore, ret = %llx\n", rs_data_ptr->data.data);
    }

    RRegData reg;
    reg.id = rq_data_ptr->data.id;
    reg.name = rq_data_ptr->data.name;
    reg.comment = rq_data_ptr->data.comment;
    reg.value = rq_data_ptr->data.value;
    rs_data_ptr->data.data = reg_manager_ptr_->updateRReg(&reg);

    if (rs_data_ptr->data.data != SUCCESS)
        recordLog(REG_MANAGER_LOG, rs_data_ptr->data.data, std::string("/rpc/reg_manager/r/updateReg"));
}

// "/rpc/reg_manager/r/getReg"
void ControllerRpc::handleRpc0x0000EAB7(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_Int32* rq_data_ptr = static_cast<RequestMessageType_Int32*>(request_data_ptr);
    ResponseMessageType_Uint64_RRegData* rs_data_ptr = static_cast<ResponseMessageType_Uint64_RRegData*>(response_data_ptr);

    RRegData reg;
    rs_data_ptr->error_code.data = reg_manager_ptr_->getRReg(rq_data_ptr->data.data, &reg);
    if(rs_data_ptr->error_code.data == SUCCESS)
    {
        rs_data_ptr->data.id = reg.id;
        strncpy(rs_data_ptr->data.name, reg.name.c_str(), 31);
        rs_data_ptr->data.name[31] = 0;
        strncpy(rs_data_ptr->data.comment, reg.comment.c_str(), 255);
        rs_data_ptr->data.comment[255] = 0;
        rs_data_ptr->data.value = reg.value;
    }
    else
    {
        memset(&rs_data_ptr->data, 0, sizeof(MessageType_RRegData));
    }

    if (rs_data_ptr->error_code.data != SUCCESS)
        recordLog(REG_MANAGER_LOG, rs_data_ptr->error_code.data, std::string("/rpc/reg_manager/r/getReg"));
}

// "/rpc/reg_manager/r/moveReg"
void ControllerRpc::handleRpc0x0000C877(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_Int32List* rq_data_ptr = static_cast<RequestMessageType_Int32List*>(request_data_ptr);
    ResponseMessageType_Uint64* rs_data_ptr = static_cast<ResponseMessageType_Uint64*>(response_data_ptr);

    if (false == state_machine_ptr_->getState())
    {
        rs_data_ptr->data.data = CONTROLLER_INVALID_OPERATION;
        FST_INFO("/rpc/reg_manager/r/moveReg can't run when backup/restore, ret = %llx\n", rs_data_ptr->data.data);
    }

    if(rq_data_ptr->data.data_count == 2)
    {
        rs_data_ptr->data.data = reg_manager_ptr_->moveRReg(rq_data_ptr->data.data[0], rq_data_ptr->data.data[1]);
    }
    else
    {
        rs_data_ptr->data.data = REG_MANAGER_INVALID_ARG;
    }

    if (rs_data_ptr->data.data != SUCCESS)
    recordLog(REG_MANAGER_LOG, rs_data_ptr->data.data, std::string("/rpc/reg_manager/r/moveReg"));
}

// "/rpc/reg_manager/r/getChangedList"
void ControllerRpc::handleRpc0x0000A904(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_Int32List* rq_data_ptr = static_cast<RequestMessageType_Int32List*>(request_data_ptr);
    ResponseMessageType_Uint64_BaseRegSummaryList* rs_data_ptr = static_cast<ResponseMessageType_Uint64_BaseRegSummaryList*>(response_data_ptr);

    if(rq_data_ptr->data.data_count == 2)
    {
        std::vector<BaseRegSummary> summary_list;
        summary_list = reg_manager_ptr_->getRRegChangedList(rq_data_ptr->data.data[0], rq_data_ptr->data.data[1]);
        for(unsigned int i = 0; i < summary_list.size(); ++i)
        {
            rs_data_ptr->data.summary[i].id = summary_list[i].id;
            strncpy(rs_data_ptr->data.summary[i].name, summary_list[i].name.c_str(), 31);
            rs_data_ptr->data.summary[i].name[31] = 0;
            strncpy(rs_data_ptr->data.summary[i].comment, summary_list[i].comment.c_str(), 255);
            rs_data_ptr->data.summary[i].comment[255] = 0;
        }
        rs_data_ptr->data.summary_count = summary_list.size();
        rs_data_ptr->error_code.data = SUCCESS;
    }
    else
    {
        rs_data_ptr->data.summary_count = 0;
        rs_data_ptr->error_code.data = REG_MANAGER_INVALID_ARG;
    }

    if (rs_data_ptr->error_code.data != SUCCESS)
        recordLog(REG_MANAGER_LOG, rs_data_ptr->error_code.data, std::string("/rpc/reg_manager/r/getChangedList"));
}

// "/rpc/reg_manager/r/getValidList"
void ControllerRpc::handleRpc0x00008CE4(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_Int32List* rq_data_ptr = static_cast<RequestMessageType_Int32List*>(request_data_ptr);
    ResponseMessageType_Uint64_BaseRegSummaryList* rs_data_ptr = static_cast<ResponseMessageType_Uint64_BaseRegSummaryList*>(response_data_ptr);

    if(rq_data_ptr->data.data_count == 2)
    {
        std::vector<BaseRegSummary> summary_list;
        summary_list = reg_manager_ptr_->getRRegValidList(rq_data_ptr->data.data[0], rq_data_ptr->data.data[1]);
        for(unsigned int i = 0; i < summary_list.size(); ++i)
        {
            rs_data_ptr->data.summary[i].id = summary_list[i].id;
            strncpy(rs_data_ptr->data.summary[i].name, summary_list[i].name.c_str(), 31);
            rs_data_ptr->data.summary[i].name[31] = 0;
            strncpy(rs_data_ptr->data.summary[i].comment, summary_list[i].comment.c_str(), 255);
            rs_data_ptr->data.summary[i].comment[255] = 0;
        }
        rs_data_ptr->data.summary_count = summary_list.size();
        rs_data_ptr->error_code.data = SUCCESS;
    }
    else
    {
        rs_data_ptr->data.summary_count = 0;
        rs_data_ptr->error_code.data = REG_MANAGER_INVALID_ARG;
    }

    if (rs_data_ptr->error_code.data != SUCCESS)
        recordLog(REG_MANAGER_LOG, rs_data_ptr->error_code.data, std::string("/rpc/reg_manager/r/getValidList"));
}

// "/rpc/reg_manager/mr/addReg"
void ControllerRpc::handleRpc0x000097E7(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_MrRegData* rq_data_ptr = static_cast<RequestMessageType_MrRegData*>(request_data_ptr);
    ResponseMessageType_Uint64* rs_data_ptr = static_cast<ResponseMessageType_Uint64*>(response_data_ptr);
    
    if (false == state_machine_ptr_->getState())
    {
        rs_data_ptr->data.data = CONTROLLER_INVALID_OPERATION;
        FST_INFO("/rpc/reg_manager/mr/addReg can't run when backup/restore, ret = %llx\n", rs_data_ptr->data.data);
    }

    MrRegData reg;
    reg.id = rq_data_ptr->data.id;
    reg.name = rq_data_ptr->data.name;
    reg.comment = rq_data_ptr->data.comment;
    reg.value = rq_data_ptr->data.value;
    rs_data_ptr->data.data = reg_manager_ptr_->addMrReg(&reg);

    if (rs_data_ptr->data.data != SUCCESS)
        recordLog(REG_MANAGER_LOG, rs_data_ptr->data.data, std::string("/rpc/reg_manager/mr/addReg"));
}

// "/rpc/reg_manager/mr/deleteReg"
void ControllerRpc::handleRpc0x0000E5D7(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_Int32* rq_data_ptr = static_cast<RequestMessageType_Int32*>(request_data_ptr);
    ResponseMessageType_Uint64* rs_data_ptr = static_cast<ResponseMessageType_Uint64*>(response_data_ptr);

    if (false == state_machine_ptr_->getState())
    {
        rs_data_ptr->data.data = CONTROLLER_INVALID_OPERATION;
        FST_INFO("/rpc/reg_manager/mr/deleteReg can't run when backup/restore, ret = %llx\n", rs_data_ptr->data.data);
    }

    rs_data_ptr->data.data = reg_manager_ptr_->deleteMrReg(rq_data_ptr->data.data);

    if (rs_data_ptr->data.data != SUCCESS)
        recordLog(REG_MANAGER_LOG, rs_data_ptr->data.data, std::string("/rpc/reg_manager/mr/deleteReg"));
}

// "/rpc/reg_manager/mr/updateReg"
void ControllerRpc::handleRpc0x0000E9B7(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_MrRegData* rq_data_ptr = static_cast<RequestMessageType_MrRegData*>(request_data_ptr);
    ResponseMessageType_Uint64* rs_data_ptr = static_cast<ResponseMessageType_Uint64*>(response_data_ptr);

    if (false == state_machine_ptr_->getState())
    {
        rs_data_ptr->data.data = CONTROLLER_INVALID_OPERATION;
        FST_INFO("/rpc/reg_manager/mr/updateReg can't run when backup/restore, ret = %llx\n", rs_data_ptr->data.data);
    }

    MrRegData reg;
    reg.id = rq_data_ptr->data.id;
    reg.name = rq_data_ptr->data.name;
    reg.comment = rq_data_ptr->data.comment;
    reg.value = rq_data_ptr->data.value;
    rs_data_ptr->data.data = reg_manager_ptr_->updateMrReg(&reg);

    if (rs_data_ptr->data.data != SUCCESS)
        recordLog(REG_MANAGER_LOG, rs_data_ptr->data.data, std::string("/rpc/reg_manager/mr/updateReg"));
}

// "/rpc/reg_manager/mr/getReg"
void ControllerRpc::handleRpc0x0000B507(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_Int32* rq_data_ptr = static_cast<RequestMessageType_Int32*>(request_data_ptr);
    ResponseMessageType_Uint64_MrRegData* rs_data_ptr = static_cast<ResponseMessageType_Uint64_MrRegData*>(response_data_ptr);

    MrRegData reg;
    rs_data_ptr->error_code.data = reg_manager_ptr_->getMrReg(rq_data_ptr->data.data, &reg);
    if(rs_data_ptr->error_code.data == SUCCESS)
    {
        rs_data_ptr->data.id = reg.id;
        strncpy(rs_data_ptr->data.name, reg.name.c_str(), 31);
        rs_data_ptr->data.name[31] = 0;
        strncpy(rs_data_ptr->data.comment, reg.comment.c_str(), 31);
        rs_data_ptr->data.comment[255] = 0;
        rs_data_ptr->data.value = reg.value;
    }
    else
    {
        memset(&rs_data_ptr->data, 0, sizeof(MessageType_MrRegData));
    }

    if (rs_data_ptr->error_code.data != SUCCESS)
        recordLog(REG_MANAGER_LOG, rs_data_ptr->error_code.data, std::string("/rpc/reg_manager/mr/getReg"));
}

// "/rpc/reg_manager/mr/moveReg"
void ControllerRpc::handleRpc0x00015BA7(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_Int32List* rq_data_ptr = static_cast<RequestMessageType_Int32List*>(request_data_ptr);
    ResponseMessageType_Uint64* rs_data_ptr = static_cast<ResponseMessageType_Uint64*>(response_data_ptr);

    if (false == state_machine_ptr_->getState())
    {
        rs_data_ptr->data.data = CONTROLLER_INVALID_OPERATION;
        FST_INFO("/rpc/reg_manager/mr/moveReg can't run when backup/restore, ret = %llx\n", rs_data_ptr->data.data);
    }

    if(rq_data_ptr->data.data_count == 2)
    {
        rs_data_ptr->data.data = reg_manager_ptr_->moveMrReg(rq_data_ptr->data.data[0], rq_data_ptr->data.data[1]);
    }
    else
    {
        rs_data_ptr->data.data = REG_MANAGER_INVALID_ARG;
    }

    if (rs_data_ptr->data.data != SUCCESS)
        recordLog(REG_MANAGER_LOG, rs_data_ptr->data.data, std::string("/rpc/reg_manager/mr/moveReg"));
}

// "/rpc/reg_manager/mr/getChangedList"
void ControllerRpc::handleRpc0x00001774(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_Int32List* rq_data_ptr = static_cast<RequestMessageType_Int32List*>(request_data_ptr);
    ResponseMessageType_Uint64_BaseRegSummaryList* rs_data_ptr = static_cast<ResponseMessageType_Uint64_BaseRegSummaryList*>(response_data_ptr);

    if(rq_data_ptr->data.data_count == 2)
    {
        std::vector<BaseRegSummary> summary_list;
        summary_list = reg_manager_ptr_->getMrRegChangedList(rq_data_ptr->data.data[0], rq_data_ptr->data.data[1]);
        for(unsigned int i = 0; i < summary_list.size(); ++i)
        {
            rs_data_ptr->data.summary[i].id = summary_list[i].id;
            strncpy(rs_data_ptr->data.summary[i].name, summary_list[i].name.c_str(), 31);
            rs_data_ptr->data.summary[i].name[31] = 0;
            strncpy(rs_data_ptr->data.summary[i].comment, summary_list[i].comment.c_str(), 255);
            rs_data_ptr->data.summary[i].comment[255] = 0;
        }
        rs_data_ptr->data.summary_count = summary_list.size();
        rs_data_ptr->error_code.data = SUCCESS;
    }
    else
    {
        rs_data_ptr->data.summary_count = 0;
        rs_data_ptr->error_code.data = REG_MANAGER_INVALID_ARG;
    }

    if (rs_data_ptr->error_code.data != SUCCESS)
        recordLog(REG_MANAGER_LOG, rs_data_ptr->error_code.data, std::string("/rpc/reg_manager/mr/getChangedList"));
}

// "/rpc/reg_manager/mr/getValidList"
void ControllerRpc::handleRpc0x00015CF4(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_Int32List* rq_data_ptr = static_cast<RequestMessageType_Int32List*>(request_data_ptr);
    ResponseMessageType_Uint64_BaseRegSummaryList* rs_data_ptr = static_cast<ResponseMessageType_Uint64_BaseRegSummaryList*>(response_data_ptr);

    if(rq_data_ptr->data.data_count == 2)
    {
        std::vector<BaseRegSummary> summary_list;
        summary_list = reg_manager_ptr_->getMrRegValidList(rq_data_ptr->data.data[0], rq_data_ptr->data.data[1]);
        for(unsigned int i = 0; i < summary_list.size(); ++i)
        {
            rs_data_ptr->data.summary[i].id = summary_list[i].id;
            strncpy(rs_data_ptr->data.summary[i].name, summary_list[i].name.c_str(), 31);
            rs_data_ptr->data.summary[i].name[31] = 0;
            strncpy(rs_data_ptr->data.summary[i].comment, summary_list[i].comment.c_str(), 255);
            rs_data_ptr->data.summary[i].comment[255] = 0;
        }
        rs_data_ptr->data.summary_count = summary_list.size();
        rs_data_ptr->error_code.data = SUCCESS;
    }
    else
    {
        rs_data_ptr->data.summary_count = 0;
        rs_data_ptr->error_code.data = REG_MANAGER_INVALID_ARG;
    }

    if (rs_data_ptr->error_code.data != SUCCESS)
        recordLog(REG_MANAGER_LOG, rs_data_ptr->error_code.data, std::string("/rpc/reg_manager/mr/getValidList"));
}

// "/rpc/reg_manager/sr/addReg"
void ControllerRpc::handleRpc0x000161E7(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_SrRegData* rq_data_ptr = static_cast<RequestMessageType_SrRegData*>(request_data_ptr);
    ResponseMessageType_Uint64* rs_data_ptr = static_cast<ResponseMessageType_Uint64*>(response_data_ptr);
    
    if (false == state_machine_ptr_->getState())
    {
        rs_data_ptr->data.data = CONTROLLER_INVALID_OPERATION;
        FST_INFO("/rpc/reg_manager/sr/addReg can't run when backup/restore, ret = %llx\n", rs_data_ptr->data.data);
    }

    SrRegData reg;
    reg.id = rq_data_ptr->data.id;
    reg.name = rq_data_ptr->data.name;
    reg.comment = rq_data_ptr->data.comment;
    reg.value = rq_data_ptr->data.value;
    rs_data_ptr->data.data = reg_manager_ptr_->addSrReg(&reg);

    if (rs_data_ptr->data.data != SUCCESS)
        recordLog(REG_MANAGER_LOG, rs_data_ptr->data.data, std::string("/rpc/reg_manager/sr/addReg"));
}

// "/rpc/reg_manager/sr/deleteReg"
void ControllerRpc::handleRpc0x0000B817(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_Int32* rq_data_ptr = static_cast<RequestMessageType_Int32*>(request_data_ptr);
    ResponseMessageType_Uint64* rs_data_ptr = static_cast<ResponseMessageType_Uint64*>(response_data_ptr);

    if (false == state_machine_ptr_->getState())
    {
        rs_data_ptr->data.data = CONTROLLER_INVALID_OPERATION;
        FST_INFO("/rpc/reg_manager/sr/deleteReg can't run when backup/restore, ret = %llx\n", rs_data_ptr->data.data);
    }

    rs_data_ptr->data.data = reg_manager_ptr_->deleteSrReg(rq_data_ptr->data.data);

    if (rs_data_ptr->data.data != SUCCESS)
        recordLog(REG_MANAGER_LOG, rs_data_ptr->data.data, std::string("/rpc/reg_manager/sr/deleteReg"));
}

// "/rpc/reg_manager/sr/updateReg"
void ControllerRpc::handleRpc0x000119F7(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_SrRegData* rq_data_ptr = static_cast<RequestMessageType_SrRegData*>(request_data_ptr);
    ResponseMessageType_Uint64* rs_data_ptr = static_cast<ResponseMessageType_Uint64*>(response_data_ptr);

    if (false == state_machine_ptr_->getState())
    {
        rs_data_ptr->data.data = CONTROLLER_INVALID_OPERATION;
        FST_INFO("/rpc/reg_manager/sr/updateReg can't run when backup/restore, ret = %llx\n", rs_data_ptr->data.data);
    }

    SrRegData reg;
    reg.id = rq_data_ptr->data.id;
    reg.name = rq_data_ptr->data.name;
    reg.comment = rq_data_ptr->data.comment;
    reg.value = rq_data_ptr->data.value;
    rs_data_ptr->data.data = reg_manager_ptr_->updateSrReg(&reg);

    if (rs_data_ptr->data.data != SUCCESS)
        recordLog(REG_MANAGER_LOG, rs_data_ptr->data.data, std::string("/rpc/reg_manager/sr/updateReg"));
}

// "/rpc/reg_manager/sr/getReg"
void ControllerRpc::handleRpc0x00017F07(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_Int32* rq_data_ptr = static_cast<RequestMessageType_Int32*>(request_data_ptr);
    ResponseMessageType_Uint64_SrRegData* rs_data_ptr = static_cast<ResponseMessageType_Uint64_SrRegData*>(response_data_ptr);

    SrRegData reg;
    rs_data_ptr->error_code.data = reg_manager_ptr_->getSrReg(rq_data_ptr->data.data, &reg);
    if(rs_data_ptr->error_code.data == SUCCESS)
    {
        rs_data_ptr->data.id = reg.id;
        strncpy(rs_data_ptr->data.name, reg.name.c_str(), 31);
        rs_data_ptr->data.name[31] = 0;
        strncpy(rs_data_ptr->data.comment, reg.comment.c_str(), 255);
        rs_data_ptr->data.comment[255] = 0;
        strncpy(rs_data_ptr->data.value, reg.value.c_str(), 255);
        rs_data_ptr->data.value[255] = 0;
    }
    else
    {
        memset(&rs_data_ptr->data, 0, sizeof(MessageType_SrRegData));
    }

    if (rs_data_ptr->error_code.data != SUCCESS)
        recordLog(REG_MANAGER_LOG, rs_data_ptr->error_code.data, std::string("/rpc/reg_manager/sr/getReg"));
}

// "/rpc/reg_manager/sr/moveReg"
void ControllerRpc::handleRpc0x00002127(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_Int32List* rq_data_ptr = static_cast<RequestMessageType_Int32List*>(request_data_ptr);
    ResponseMessageType_Uint64* rs_data_ptr = static_cast<ResponseMessageType_Uint64*>(response_data_ptr);

    if (false == state_machine_ptr_->getState())
    {
        rs_data_ptr->data.data = CONTROLLER_INVALID_OPERATION;
        FST_INFO("/rpc/reg_manager/sr/moveReg can't run when backup/restore, ret = %llx\n", rs_data_ptr->data.data);
    }

    if(rq_data_ptr->data.data_count == 2)
    {
        rs_data_ptr->data.data = reg_manager_ptr_->moveSrReg(rq_data_ptr->data.data[0], rq_data_ptr->data.data[1]);
    }
    else
    {
        rs_data_ptr->data.data = REG_MANAGER_INVALID_ARG;
    }

    if (rs_data_ptr->data.data != SUCCESS)
        recordLog(REG_MANAGER_LOG, rs_data_ptr->data.data, std::string("/rpc/reg_manager/sr/moveReg"));
}

// "/rpc/reg_manager/sr/getChangedList"
void ControllerRpc::handleRpc0x00004834(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_Int32List* rq_data_ptr = static_cast<RequestMessageType_Int32List*>(request_data_ptr);
    ResponseMessageType_Uint64_BaseRegSummaryList* rs_data_ptr = static_cast<ResponseMessageType_Uint64_BaseRegSummaryList*>(response_data_ptr);

    if(rq_data_ptr->data.data_count == 2)
    {
        std::vector<BaseRegSummary> summary_list;
        summary_list = reg_manager_ptr_->getSrRegChangedList(rq_data_ptr->data.data[0], rq_data_ptr->data.data[1]);
        for(unsigned int i = 0; i < summary_list.size(); ++i)
        {
            rs_data_ptr->data.summary[i].id = summary_list[i].id;
            strncpy(rs_data_ptr->data.summary[i].name, summary_list[i].name.c_str(), 31);
            rs_data_ptr->data.summary[i].name[31] = 0;
            strncpy(rs_data_ptr->data.summary[i].comment, summary_list[i].comment.c_str(), 255);
            rs_data_ptr->data.summary[i].comment[255] = 0;
        }
        rs_data_ptr->data.summary_count = summary_list.size();
        rs_data_ptr->error_code.data = SUCCESS;
    }
    else
    {
        rs_data_ptr->data.summary_count = 0;
        rs_data_ptr->error_code.data = REG_MANAGER_INVALID_ARG;
    }

    if (rs_data_ptr->error_code.data != SUCCESS)
        recordLog(REG_MANAGER_LOG, rs_data_ptr->error_code.data, std::string("/rpc/reg_manager/sr/getChangedList"));
}

// "/rpc/reg_manager/sr/getValidList"
void ControllerRpc::handleRpc0x00009854(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_Int32List* rq_data_ptr = static_cast<RequestMessageType_Int32List*>(request_data_ptr);
    ResponseMessageType_Uint64_BaseRegSummaryList* rs_data_ptr = static_cast<ResponseMessageType_Uint64_BaseRegSummaryList*>(response_data_ptr);

    if(rq_data_ptr->data.data_count == 2)
    {
        std::vector<BaseRegSummary> summary_list;
        summary_list = reg_manager_ptr_->getSrRegValidList(rq_data_ptr->data.data[0], rq_data_ptr->data.data[1]);
        for(unsigned int i = 0; i < summary_list.size(); ++i)
        {
            rs_data_ptr->data.summary[i].id = summary_list[i].id;
            strncpy(rs_data_ptr->data.summary[i].name, summary_list[i].name.c_str(), 31);
            rs_data_ptr->data.summary[i].name[31] = 0;
            strncpy(rs_data_ptr->data.summary[i].comment, summary_list[i].comment.c_str(), 255);
            rs_data_ptr->data.summary[i].comment[255] = 0;
        }
        rs_data_ptr->data.summary_count = summary_list.size();
        rs_data_ptr->error_code.data = SUCCESS;
    }
    else
    {
        rs_data_ptr->data.summary_count = 0;
        rs_data_ptr->error_code.data = REG_MANAGER_INVALID_ARG;
    }

    if (rs_data_ptr->error_code.data != SUCCESS)
        recordLog(REG_MANAGER_LOG, rs_data_ptr->error_code.data, std::string("/rpc/reg_manager/sr/getValidList"));
}

// "/rpc/reg_manager/pr/addReg"
void ControllerRpc::handleRpc0x000154E7(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_PrRegData* rq_data_ptr = static_cast<RequestMessageType_PrRegData*>(request_data_ptr);
    ResponseMessageType_Uint64* rs_data_ptr = static_cast<ResponseMessageType_Uint64*>(response_data_ptr);

    if (false == state_machine_ptr_->getState())
    {
        rs_data_ptr->data.data = CONTROLLER_INVALID_OPERATION;
        FST_INFO("/rpc/reg_manager/pr/addReg can't run when backup/restore, ret = %llx\n", rs_data_ptr->data.data);
    }

    PrRegData reg;
    reg.id = rq_data_ptr->data.id;
    reg.name = rq_data_ptr->data.name;
    reg.comment = rq_data_ptr->data.comment;
    reg.value.pos_type = rq_data_ptr->data.pos_type;
    reg.value.group_id = rq_data_ptr->data.group_id;
    memcpy(reg.value.pos, rq_data_ptr->data.pos.data, 9*sizeof(double));

    reg.value.posture[3] = rq_data_ptr->data.posture.wrist_flip;
    reg.value.posture[2] = rq_data_ptr->data.posture.arm_up_down;
    reg.value.posture[1] = rq_data_ptr->data.posture.arm_back_front;
    reg.value.posture[0] = rq_data_ptr->data.posture.arm_left_right;

    if (rq_data_ptr->data.posture.turn_cycle.data_count != 9)
    {
        rs_data_ptr->data.data = CONTROLLER_INVALID_OPERATION;
        return;
    }
    memcpy(reg.value.turn, rq_data_ptr->data.posture.turn_cycle.data, 9*sizeof(int));
    rs_data_ptr->data.data = reg_manager_ptr_->addPrReg(&reg);

    if (rs_data_ptr->data.data != SUCCESS)
        recordLog(REG_MANAGER_LOG, rs_data_ptr->data.data, std::string("/rpc/reg_manager/pr/addReg"));
}

// "/rpc/reg_manager/pr/deleteReg"
void ControllerRpc::handleRpc0x00001097(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_Int32* rq_data_ptr = static_cast<RequestMessageType_Int32*>(request_data_ptr);
    ResponseMessageType_Uint64* rs_data_ptr = static_cast<ResponseMessageType_Uint64*>(response_data_ptr);

    if (false == state_machine_ptr_->getState())
    {
        rs_data_ptr->data.data = CONTROLLER_INVALID_OPERATION;
        FST_INFO("/rpc/reg_manager/pr/deleteReg can't run when backup/restore, ret = %llx\n", rs_data_ptr->data.data);
    }

    rs_data_ptr->data.data = reg_manager_ptr_->deletePrReg(rq_data_ptr->data.data);

    if (rs_data_ptr->data.data != SUCCESS)
        recordLog(REG_MANAGER_LOG, rs_data_ptr->data.data, std::string("/rpc/reg_manager/pr/deleteReg"));
}

// "/rpc/reg_manager/pr/updateReg"
void ControllerRpc::handleRpc0x00009EF7(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_PrRegData* rq_data_ptr = static_cast<RequestMessageType_PrRegData*>(request_data_ptr);
    ResponseMessageType_Uint64* rs_data_ptr = static_cast<ResponseMessageType_Uint64*>(response_data_ptr);

    if (false == state_machine_ptr_->getState())
    {
        rs_data_ptr->data.data = CONTROLLER_INVALID_OPERATION;
        FST_INFO("/rpc/reg_manager/pr/updateReg can't run when backup/restore, ret = %llx\n", rs_data_ptr->data.data);
    }

    PrRegData reg;
    reg.id = rq_data_ptr->data.id;
    reg.name = rq_data_ptr->data.name;
    reg.comment = rq_data_ptr->data.comment;
    reg.value.pos_type = rq_data_ptr->data.pos_type;
    reg.value.group_id = rq_data_ptr->data.group_id;
    memcpy(reg.value.pos, rq_data_ptr->data.pos.data, 9*sizeof(double));

    reg.value.posture[3] = rq_data_ptr->data.posture.wrist_flip;
    reg.value.posture[2] = rq_data_ptr->data.posture.arm_up_down;
    reg.value.posture[1] = rq_data_ptr->data.posture.arm_back_front;
    reg.value.posture[0] = rq_data_ptr->data.posture.arm_left_right;

    if (rq_data_ptr->data.posture.turn_cycle.data_count != 9)
    {
        rs_data_ptr->data.data = CONTROLLER_INVALID_OPERATION;
        return;
    }
    memcpy(reg.value.turn, rq_data_ptr->data.posture.turn_cycle.data, 9*sizeof(int));
    rs_data_ptr->data.data = reg_manager_ptr_->updatePrReg(&reg);

    if (rs_data_ptr->data.data != SUCCESS)
        recordLog(REG_MANAGER_LOG, rs_data_ptr->data.data, std::string("/rpc/reg_manager/pr/updateReg"));
}

// "/rpc/reg_manager/pr/getReg"
void ControllerRpc::handleRpc0x00017207(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_Int32* rq_data_ptr = static_cast<RequestMessageType_Int32*>(request_data_ptr);
    ResponseMessageType_Uint64_PrRegData* rs_data_ptr = static_cast<ResponseMessageType_Uint64_PrRegData*>(response_data_ptr);

    PrRegData reg;
    rs_data_ptr->error_code.data = reg_manager_ptr_->getPrReg(rq_data_ptr->data.data, &reg);
    if(rs_data_ptr->error_code.data == SUCCESS)
    {
        rs_data_ptr->data.id = reg.id;
        strncpy(rs_data_ptr->data.name, reg.name.c_str(), 31);
        rs_data_ptr->data.name[31] = 0;
        strncpy(rs_data_ptr->data.comment, reg.comment.c_str(), 255);
        rs_data_ptr->data.comment[255] = 0;
        rs_data_ptr->data.group_id = reg.value.group_id;
        rs_data_ptr->data.pos_type = reg.value.pos_type;
        rs_data_ptr->data.pos.data_count = 9;
        memcpy(rs_data_ptr->data.pos.data, reg.value.pos, 9*sizeof(double));

        rs_data_ptr->data.posture.wrist_flip = reg.value.posture[3];        // wrist
        rs_data_ptr->data.posture.arm_up_down = reg.value.posture[2];       // elbow
        rs_data_ptr->data.posture.arm_back_front = reg.value.posture[1];    // arm
        rs_data_ptr->data.posture.arm_left_right = reg.value.posture[0];    // flip

        rs_data_ptr->data.posture.turn_cycle.data_count = 9;
        memcpy(rs_data_ptr->data.posture.turn_cycle.data, reg.value.turn, 9*sizeof(int));
    }
    else
    {
        memset(&rs_data_ptr->data, 0, sizeof(MessageType_PrRegData));
    }

    if (rs_data_ptr->error_code.data != SUCCESS)
        recordLog(REG_MANAGER_LOG, rs_data_ptr->error_code.data, std::string("/rpc/reg_manager/pr/getReg"));
}

// "/rpc/reg_manager/pr/moveReg"
void ControllerRpc::handleRpc0x0000D7C7(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_Int32List* rq_data_ptr = static_cast<RequestMessageType_Int32List*>(request_data_ptr);
    ResponseMessageType_Uint64* rs_data_ptr = static_cast<ResponseMessageType_Uint64*>(response_data_ptr);

    if (false == state_machine_ptr_->getState())
    {
        rs_data_ptr->data.data = CONTROLLER_INVALID_OPERATION;
        FST_INFO("/rpc/reg_manager/pr/moveReg can't run when backup/restore, ret = %llx\n", rs_data_ptr->data.data);
    }

    if(rq_data_ptr->data.data_count == 2)
    {
        rs_data_ptr->data.data = reg_manager_ptr_->movePrReg(rq_data_ptr->data.data[0], rq_data_ptr->data.data[1]);
    }
    else
    {
        rs_data_ptr->data.data = REG_MANAGER_INVALID_ARG;
    }

    if (rs_data_ptr->data.data != SUCCESS)
        recordLog(REG_MANAGER_LOG, rs_data_ptr->data.data, std::string("/rpc/reg_manager/pr/moveReg"));
}

// "/rpc/reg_manager/pr/getChangedList"
void ControllerRpc::handleRpc0x0000B454(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_Int32List* rq_data_ptr = static_cast<RequestMessageType_Int32List*>(request_data_ptr);
    ResponseMessageType_Uint64_BaseRegSummaryList* rs_data_ptr = static_cast<ResponseMessageType_Uint64_BaseRegSummaryList*>(response_data_ptr);

    if(rq_data_ptr->data.data_count == 2)
    {
        std::vector<BaseRegSummary> summary_list;
        summary_list = reg_manager_ptr_->getPrRegChangedList(rq_data_ptr->data.data[0], rq_data_ptr->data.data[1]);
        for(unsigned int i = 0; i < summary_list.size(); ++i)
        {
            rs_data_ptr->data.summary[i].id = summary_list[i].id;
            strncpy(rs_data_ptr->data.summary[i].name, summary_list[i].name.c_str(), 31);
            rs_data_ptr->data.summary[i].name[31] = 0;
            strncpy(rs_data_ptr->data.summary[i].comment, summary_list[i].comment.c_str(), 255);
            rs_data_ptr->data.summary[i].comment[255] = 0;
        }
        rs_data_ptr->data.summary_count = summary_list.size();
        rs_data_ptr->error_code.data = SUCCESS;
    }
    else
    {
        rs_data_ptr->data.summary_count = 0;
        rs_data_ptr->error_code.data = REG_MANAGER_INVALID_ARG;
    }

    if (rs_data_ptr->error_code.data != SUCCESS)
        recordLog(REG_MANAGER_LOG, rs_data_ptr->error_code.data, std::string("/rpc/reg_manager/pr/getChangedList"));
}

// "/rpc/reg_manager/pr/getValidList"
void ControllerRpc::handleRpc0x00009354(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_Int32List* rq_data_ptr = static_cast<RequestMessageType_Int32List*>(request_data_ptr);
    ResponseMessageType_Uint64_BaseRegSummaryList* rs_data_ptr = static_cast<ResponseMessageType_Uint64_BaseRegSummaryList*>(response_data_ptr);

    if(rq_data_ptr->data.data_count == 2)
    {
        std::vector<BaseRegSummary> summary_list;
        summary_list = reg_manager_ptr_->getPrRegValidList(rq_data_ptr->data.data[0], rq_data_ptr->data.data[1]);
        for(unsigned int i = 0; i < summary_list.size(); ++i)
        {
            rs_data_ptr->data.summary[i].id = summary_list[i].id;
            strncpy(rs_data_ptr->data.summary[i].name, summary_list[i].name.c_str(), 31);
            rs_data_ptr->data.summary[i].name[31] = 0;
            strncpy(rs_data_ptr->data.summary[i].comment, summary_list[i].comment.c_str(), 255);
            rs_data_ptr->data.summary[i].comment[255] = 0;
        }
        rs_data_ptr->data.summary_count = summary_list.size();
        rs_data_ptr->error_code.data = SUCCESS;
    }
    else
    {
        rs_data_ptr->data.summary_count = 0;
        rs_data_ptr->error_code.data = REG_MANAGER_INVALID_ARG;
    }

    if (rs_data_ptr->error_code.data != SUCCESS)
        recordLog(REG_MANAGER_LOG, rs_data_ptr->error_code.data, std::string("/rpc/reg_manager/pr/getValidList"));
}

// "/rpc/reg_manager/hr/addReg"
void ControllerRpc::handleRpc0x00016CE7(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_HrRegData* rq_data_ptr = static_cast<RequestMessageType_HrRegData*>(request_data_ptr);
    ResponseMessageType_Uint64* rs_data_ptr = static_cast<ResponseMessageType_Uint64*>(response_data_ptr);

    if (false == state_machine_ptr_->getState())
    {
        rs_data_ptr->data.data = CONTROLLER_INVALID_OPERATION;
        FST_INFO("/rpc/reg_manager/hr/addReg can't run when backup/restore, ret = %llx\n", rs_data_ptr->data.data);
    }

    HrRegData reg;
    reg.id = rq_data_ptr->data.id;
    reg.name = rq_data_ptr->data.name;
    reg.comment = rq_data_ptr->data.comment;
    reg.value.group_id = rq_data_ptr->data.group_id;
    memcpy(reg.value.joint_pos, rq_data_ptr->data.joints.data, 9*sizeof(double));
    memcpy(reg.value.diff_pos, rq_data_ptr->data.diffs.data, 9*sizeof(double));
    rs_data_ptr->data.data = reg_manager_ptr_->addHrReg(&reg);

    if (rs_data_ptr->data.data != SUCCESS)
        recordLog(REG_MANAGER_LOG, rs_data_ptr->data.data, std::string("/rpc/reg_manager/hr/addReg"));
}

// "/rpc/reg_manager/hr/deleteReg"
void ControllerRpc::handleRpc0x00003D17(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_Int32* rq_data_ptr = static_cast<RequestMessageType_Int32*>(request_data_ptr);
    ResponseMessageType_Uint64* rs_data_ptr = static_cast<ResponseMessageType_Uint64*>(response_data_ptr);

    if (false == state_machine_ptr_->getState())
    {
        rs_data_ptr->data.data = CONTROLLER_INVALID_OPERATION;
        FST_INFO("/rpc/reg_manager/hr/deleteReg can't run when backup/restore, ret = %llx\n", rs_data_ptr->data.data);
    }

    rs_data_ptr->data.data = reg_manager_ptr_->deleteHrReg(rq_data_ptr->data.data);

    if (rs_data_ptr->data.data != SUCCESS)
        recordLog(REG_MANAGER_LOG, rs_data_ptr->data.data, std::string("/rpc/reg_manager/hr/deleteReg"));
}

// "/rpc/reg_manager/hr/updateReg"
void ControllerRpc::handleRpc0x0000CB77(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_HrRegData* rq_data_ptr = static_cast<RequestMessageType_HrRegData*>(request_data_ptr);
    ResponseMessageType_Uint64* rs_data_ptr = static_cast<ResponseMessageType_Uint64*>(response_data_ptr);

    if (false == state_machine_ptr_->getState())
    {
        rs_data_ptr->data.data = CONTROLLER_INVALID_OPERATION;
        FST_INFO("/rpc/reg_manager/hr/updateReg can't run when backup/restore, ret = %llx\n", rs_data_ptr->data.data);
    }

    HrRegData reg;
    reg.id = rq_data_ptr->data.id;
    reg.name = rq_data_ptr->data.name;
    reg.comment = rq_data_ptr->data.comment;
    reg.value.group_id = rq_data_ptr->data.group_id;
    memcpy(reg.value.joint_pos, rq_data_ptr->data.joints.data, 9*sizeof(double));
    memcpy(reg.value.diff_pos, rq_data_ptr->data.diffs.data, 9*sizeof(double));
    rs_data_ptr->data.data = reg_manager_ptr_->updateHrReg(&reg);

    if (rs_data_ptr->data.data != SUCCESS)
        recordLog(REG_MANAGER_LOG, rs_data_ptr->data.data, std::string("/rpc/reg_manager/hr/updateReg"));
}

// "/rpc/reg_manager/hr/getReg"
void ControllerRpc::handleRpc0x00000367(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_Int32* rq_data_ptr = static_cast<RequestMessageType_Int32*>(request_data_ptr);
    ResponseMessageType_Uint64_HrRegData* rs_data_ptr = static_cast<ResponseMessageType_Uint64_HrRegData*>(response_data_ptr);

    HrRegData reg;
    rs_data_ptr->error_code.data = reg_manager_ptr_->getHrReg(rq_data_ptr->data.data, &reg);
    if(rs_data_ptr->error_code.data == SUCCESS)
    {
        rs_data_ptr->data.id = reg.id;
        strncpy(rs_data_ptr->data.name, reg.name.c_str(), 31);
        rs_data_ptr->data.name[31] = 0;
        strncpy(rs_data_ptr->data.comment, reg.comment.c_str(), 255);
        rs_data_ptr->data.comment[255] = 0;
        rs_data_ptr->data.group_id = reg.value.group_id;
        rs_data_ptr->data.joints.data_count = 9;
        memcpy(rs_data_ptr->data.joints.data, reg.value.joint_pos, 9*sizeof(double));
        rs_data_ptr->data.diffs.data_count = 9;
        memcpy(rs_data_ptr->data.diffs.data, reg.value.diff_pos, 9*sizeof(double));
    }
    else
    {
        memset(&rs_data_ptr->data, 0, sizeof(MessageType_HrRegData));
    }

    if (rs_data_ptr->error_code.data != SUCCESS)
        recordLog(REG_MANAGER_LOG, rs_data_ptr->error_code.data, std::string("/rpc/reg_manager/hr/getReg"));
}

// "/rpc/reg_manager/hr/moveReg"
void ControllerRpc::handleRpc0x00014A87(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_Int32List* rq_data_ptr = static_cast<RequestMessageType_Int32List*>(request_data_ptr);
    ResponseMessageType_Uint64* rs_data_ptr = static_cast<ResponseMessageType_Uint64*>(response_data_ptr);

    if (false == state_machine_ptr_->getState())
    {
        rs_data_ptr->data.data = CONTROLLER_INVALID_OPERATION;
        FST_INFO("/rpc/reg_manager/hr/moveReg can't run when backup/restore, ret = %llx\n", rs_data_ptr->data.data);
    }

    if(rq_data_ptr->data.data_count == 2)
    {
        rs_data_ptr->data.data = reg_manager_ptr_->moveHrReg(rq_data_ptr->data.data[0], rq_data_ptr->data.data[1]);
    }
    else
    {
        rs_data_ptr->data.data = REG_MANAGER_INVALID_ARG;
    }

    if (rs_data_ptr->data.data != SUCCESS)
        recordLog(REG_MANAGER_LOG, rs_data_ptr->data.data, std::string("/rpc/reg_manager/hr/moveReg"));
}

// "/rpc/reg_manager/hr/getChangedList"
void ControllerRpc::handleRpc0x00012974(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_Int32List* rq_data_ptr = static_cast<RequestMessageType_Int32List*>(request_data_ptr);
    ResponseMessageType_Uint64_BaseRegSummaryList* rs_data_ptr = static_cast<ResponseMessageType_Uint64_BaseRegSummaryList*>(response_data_ptr);

    if(rq_data_ptr->data.data_count == 2)
    {
        std::vector<BaseRegSummary> summary_list;
        summary_list = reg_manager_ptr_->getHrRegChangedList(rq_data_ptr->data.data[0], rq_data_ptr->data.data[1]);
        for(unsigned int i = 0; i < summary_list.size(); ++i)
        {
            rs_data_ptr->data.summary[i].id = summary_list[i].id;
            strncpy(rs_data_ptr->data.summary[i].name, summary_list[i].name.c_str(), 31);
            rs_data_ptr->data.summary[i].name[31] = 0;
            strncpy(rs_data_ptr->data.summary[i].comment, summary_list[i].comment.c_str(), 255);
            rs_data_ptr->data.summary[i].comment[255] = 0;
        }
        rs_data_ptr->data.summary_count = summary_list.size();
        rs_data_ptr->error_code.data = SUCCESS;
    }
    else
    {
        rs_data_ptr->data.summary_count = 0;
        rs_data_ptr->error_code.data = REG_MANAGER_INVALID_ARG;
    }

    if (rs_data_ptr->error_code.data != SUCCESS)
        recordLog(REG_MANAGER_LOG, rs_data_ptr->error_code.data, std::string("/rpc/reg_manager/hr/getChangedList"));
}

// "/rpc/reg_manager/hr/getValidList"
void ControllerRpc::handleRpc0x00006B54(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_Int32List* rq_data_ptr = static_cast<RequestMessageType_Int32List*>(request_data_ptr);
    ResponseMessageType_Uint64_BaseRegSummaryList* rs_data_ptr = static_cast<ResponseMessageType_Uint64_BaseRegSummaryList*>(response_data_ptr);

    if(rq_data_ptr->data.data_count == 2)
    {
        std::vector<BaseRegSummary> summary_list;
        summary_list = reg_manager_ptr_->getHrRegValidList(rq_data_ptr->data.data[0], rq_data_ptr->data.data[1]);
        for(unsigned int i = 0; i < summary_list.size(); ++i)
        {
            rs_data_ptr->data.summary[i].id = summary_list[i].id;
            strncpy(rs_data_ptr->data.summary[i].name, summary_list[i].name.c_str(), 31);
            rs_data_ptr->data.summary[i].name[31] = 0;
            strncpy(rs_data_ptr->data.summary[i].comment, summary_list[i].comment.c_str(), 255);
            rs_data_ptr->data.summary[i].comment[255] = 0;
        }
        rs_data_ptr->data.summary_count = summary_list.size();
        rs_data_ptr->error_code.data = SUCCESS;
    }
    else
    {
        rs_data_ptr->data.summary_count = 0;
        rs_data_ptr->error_code.data = REG_MANAGER_INVALID_ARG;
    }

    if (rs_data_ptr->error_code.data != SUCCESS)
        recordLog(REG_MANAGER_LOG, rs_data_ptr->error_code.data, std::string("/rpc/reg_manager/hr/getValidList"));
}

