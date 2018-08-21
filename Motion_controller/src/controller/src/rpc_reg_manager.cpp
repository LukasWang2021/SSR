#include "controller_rpc.h"

using namespace fst_ctrl;

// "/rpc/reg_manager/r/addReg"
void ControllerRpc::handleRpc0x00004FF7(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_RRegData* rq_data_ptr = static_cast<RequestMessageType_RRegData*>(request_data_ptr);
    ResponseMessageType_Bool* rs_data_ptr = static_cast<ResponseMessageType_Bool*>(response_data_ptr);

    RRegData reg;
    reg.id = rq_data_ptr->data.id;
    reg.name = rq_data_ptr->data.name;
    reg.comment = rq_data_ptr->data.comment;
    reg.value = rq_data_ptr->data.value;
    rs_data_ptr->data.data = reg_manager_ptr_->addRReg(&reg);
}

// "/rpc/reg_manager/r/deleteReg"
void ControllerRpc::handleRpc0x000012F7(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_Int32* rq_data_ptr = static_cast<RequestMessageType_Int32*>(request_data_ptr);
    ResponseMessageType_Bool* rs_data_ptr = static_cast<ResponseMessageType_Bool*>(response_data_ptr);

    rs_data_ptr->data.data = reg_manager_ptr_->deleteRReg(rq_data_ptr->data.data);
}

// "/rpc/reg_manager/r/updateReg"
void ControllerRpc::handleRpc0x00005757(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_RRegData* rq_data_ptr = static_cast<RequestMessageType_RRegData*>(request_data_ptr);
    ResponseMessageType_Bool* rs_data_ptr = static_cast<ResponseMessageType_Bool*>(response_data_ptr);

    RRegData reg;
    reg.id = rq_data_ptr->data.id;
    reg.name = rq_data_ptr->data.name;
    reg.comment = rq_data_ptr->data.comment;
    reg.value = rq_data_ptr->data.value;
    rs_data_ptr->data.data = reg_manager_ptr_->updateRReg(&reg);
}

// "/rpc/reg_manager/r/getReg"
void ControllerRpc::handleRpc0x0000EAB7(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_Int32* rq_data_ptr = static_cast<RequestMessageType_Int32*>(request_data_ptr);
    ResponseMessageType_Bool_RRegData* rs_data_ptr = static_cast<ResponseMessageType_Bool_RRegData*>(response_data_ptr);

    RRegData reg;
    rs_data_ptr->success.data = reg_manager_ptr_->getRReg(rq_data_ptr->data.data, &reg);
    if(rs_data_ptr->success.data)
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
}

// "/rpc/reg_manager/r/moveReg"
void ControllerRpc::handleRpc0x0000C877(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_Int32List* rq_data_ptr = static_cast<RequestMessageType_Int32List*>(request_data_ptr);
    ResponseMessageType_Bool* rs_data_ptr = static_cast<ResponseMessageType_Bool*>(response_data_ptr);

    if(rq_data_ptr->data.data_count == 2)
    {
        rs_data_ptr->data.data = reg_manager_ptr_->moveRReg(rq_data_ptr->data.data[0], rq_data_ptr->data.data[1]);
    }
    else
    {
        rs_data_ptr->data.data = false;
    }
}

// "/rpc/reg_manager/r/getChangedList"
void ControllerRpc::handleRpc0x0000A904(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_Int32List* rq_data_ptr = static_cast<RequestMessageType_Int32List*>(request_data_ptr);
    ResponseMessageType_BaseRegSummaryList* rs_data_ptr = static_cast<ResponseMessageType_BaseRegSummaryList*>(response_data_ptr);

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
            rs_data_ptr->data.summary[i].comment[31] = 0;
        }
        rs_data_ptr->data.summary_count = summary_list.size();
    }
    else
    {
        rs_data_ptr->data.summary_count = 0;
    }
}

// "/rpc/reg_manager/r/getValidList"
void ControllerRpc::handleRpc0x00008CE4(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_Int32List* rq_data_ptr = static_cast<RequestMessageType_Int32List*>(request_data_ptr);
    ResponseMessageType_BaseRegSummaryList* rs_data_ptr = static_cast<ResponseMessageType_BaseRegSummaryList*>(response_data_ptr);

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
            rs_data_ptr->data.summary[i].comment[31] = 0;
        }
        rs_data_ptr->data.summary_count = summary_list.size();
    }
    else
    {
        rs_data_ptr->data.summary_count = 0;
    }
}

// "/rpc/reg_manager/mr/addReg"
void ControllerRpc::handleRpc0x000097E7(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_MrRegData* rq_data_ptr = static_cast<RequestMessageType_MrRegData*>(request_data_ptr);
    ResponseMessageType_Bool* rs_data_ptr = static_cast<ResponseMessageType_Bool*>(response_data_ptr);

    MrRegData reg;
    reg.id = rq_data_ptr->data.id;
    reg.name = rq_data_ptr->data.name;
    reg.comment = rq_data_ptr->data.comment;
    reg.value = rq_data_ptr->data.value;
    rs_data_ptr->data.data = reg_manager_ptr_->addMrReg(&reg);
}

// "/rpc/reg_manager/mr/deleteReg"
void ControllerRpc::handleRpc0x0000E5D7(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_Int32* rq_data_ptr = static_cast<RequestMessageType_Int32*>(request_data_ptr);
    ResponseMessageType_Bool* rs_data_ptr = static_cast<ResponseMessageType_Bool*>(response_data_ptr);

    rs_data_ptr->data.data = reg_manager_ptr_->deleteMrReg(rq_data_ptr->data.data);
}

// "/rpc/reg_manager/mr/updateReg"
void ControllerRpc::handleRpc0x0000E9B7(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_MrRegData* rq_data_ptr = static_cast<RequestMessageType_MrRegData*>(request_data_ptr);
    ResponseMessageType_Bool* rs_data_ptr = static_cast<ResponseMessageType_Bool*>(response_data_ptr);

    MrRegData reg;
    reg.id = rq_data_ptr->data.id;
    reg.name = rq_data_ptr->data.name;
    reg.comment = rq_data_ptr->data.comment;
    reg.value = rq_data_ptr->data.value;
    rs_data_ptr->data.data = reg_manager_ptr_->updateMrReg(&reg);
}

// "/rpc/reg_manager/mr/getReg"
void ControllerRpc::handleRpc0x0000B507(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_Int32* rq_data_ptr = static_cast<RequestMessageType_Int32*>(request_data_ptr);
    ResponseMessageType_Bool_MrRegData* rs_data_ptr = static_cast<ResponseMessageType_Bool_MrRegData*>(response_data_ptr);

    MrRegData reg;
    rs_data_ptr->success.data = reg_manager_ptr_->getMrReg(rq_data_ptr->data.data, &reg);
    if(rs_data_ptr->success.data)
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
}

// "/rpc/reg_manager/mr/moveReg"
void ControllerRpc::handleRpc0x00015BA7(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_Int32List* rq_data_ptr = static_cast<RequestMessageType_Int32List*>(request_data_ptr);
    ResponseMessageType_Bool* rs_data_ptr = static_cast<ResponseMessageType_Bool*>(response_data_ptr);

    if(rq_data_ptr->data.data_count == 2)
    {
        rs_data_ptr->data.data = reg_manager_ptr_->moveMrReg(rq_data_ptr->data.data[0], rq_data_ptr->data.data[1]);
    }
    else
    {
        rs_data_ptr->data.data = false;
    }
}

// "/rpc/reg_manager/mr/getChangedList"
void ControllerRpc::handleRpc0x00001774(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_Int32List* rq_data_ptr = static_cast<RequestMessageType_Int32List*>(request_data_ptr);
    ResponseMessageType_BaseRegSummaryList* rs_data_ptr = static_cast<ResponseMessageType_BaseRegSummaryList*>(response_data_ptr);

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
            rs_data_ptr->data.summary[i].comment[31] = 0;
        }
        rs_data_ptr->data.summary_count = summary_list.size();
    }
    else
    {
        rs_data_ptr->data.summary_count = 0;
    }
}

// "/rpc/reg_manager/mr/getValidList"
void ControllerRpc::handleRpc0x00015CF4(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_Int32List* rq_data_ptr = static_cast<RequestMessageType_Int32List*>(request_data_ptr);
    ResponseMessageType_BaseRegSummaryList* rs_data_ptr = static_cast<ResponseMessageType_BaseRegSummaryList*>(response_data_ptr);

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
            rs_data_ptr->data.summary[i].comment[31] = 0;
        }
        rs_data_ptr->data.summary_count = summary_list.size();
    }
    else
    {
        rs_data_ptr->data.summary_count = 0;
    }
}

// "/rpc/reg_manager/sr/addReg"
void ControllerRpc::handleRpc0x000161E7(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_SrRegData* rq_data_ptr = static_cast<RequestMessageType_SrRegData*>(request_data_ptr);
    ResponseMessageType_Bool* rs_data_ptr = static_cast<ResponseMessageType_Bool*>(response_data_ptr);

    SrRegData reg;
    reg.id = rq_data_ptr->data.id;
    reg.name = rq_data_ptr->data.name;
    reg.comment = rq_data_ptr->data.comment;
    reg.value = rq_data_ptr->data.value;
    rs_data_ptr->data.data = reg_manager_ptr_->addSrReg(&reg);
}

// "/rpc/reg_manager/sr/deleteReg"
void ControllerRpc::handleRpc0x0000B817(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_Int32* rq_data_ptr = static_cast<RequestMessageType_Int32*>(request_data_ptr);
    ResponseMessageType_Bool* rs_data_ptr = static_cast<ResponseMessageType_Bool*>(response_data_ptr);

    rs_data_ptr->data.data = reg_manager_ptr_->deleteSrReg(rq_data_ptr->data.data);
}

// "/rpc/reg_manager/sr/updateReg"
void ControllerRpc::handleRpc0x000119F7(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_SrRegData* rq_data_ptr = static_cast<RequestMessageType_SrRegData*>(request_data_ptr);
    ResponseMessageType_Bool* rs_data_ptr = static_cast<ResponseMessageType_Bool*>(response_data_ptr);

    SrRegData reg;
    reg.id = rq_data_ptr->data.id;
    reg.name = rq_data_ptr->data.name;
    reg.comment = rq_data_ptr->data.comment;
    reg.value = rq_data_ptr->data.value;
    rs_data_ptr->data.data = reg_manager_ptr_->updateSrReg(&reg);
}

// "/rpc/reg_manager/sr/getReg"
void ControllerRpc::handleRpc0x00017F07(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_Int32* rq_data_ptr = static_cast<RequestMessageType_Int32*>(request_data_ptr);
    ResponseMessageType_Bool_SrRegData* rs_data_ptr = static_cast<ResponseMessageType_Bool_SrRegData*>(response_data_ptr);

    SrRegData reg;
    rs_data_ptr->success.data = reg_manager_ptr_->getSrReg(rq_data_ptr->data.data, &reg);
    if(rs_data_ptr->success.data)
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
}

// "/rpc/reg_manager/sr/moveReg"
void ControllerRpc::handleRpc0x00002127(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_Int32List* rq_data_ptr = static_cast<RequestMessageType_Int32List*>(request_data_ptr);
    ResponseMessageType_Bool* rs_data_ptr = static_cast<ResponseMessageType_Bool*>(response_data_ptr);

    if(rq_data_ptr->data.data_count == 2)
    {
        rs_data_ptr->data.data = reg_manager_ptr_->moveSrReg(rq_data_ptr->data.data[0], rq_data_ptr->data.data[1]);
    }
    else
    {
        rs_data_ptr->data.data = false;
    }
}

// "/rpc/reg_manager/sr/getChangedList"
void ControllerRpc::handleRpc0x00004834(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_Int32List* rq_data_ptr = static_cast<RequestMessageType_Int32List*>(request_data_ptr);
    ResponseMessageType_BaseRegSummaryList* rs_data_ptr = static_cast<ResponseMessageType_BaseRegSummaryList*>(response_data_ptr);

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
            rs_data_ptr->data.summary[i].comment[31] = 0;
        }
        rs_data_ptr->data.summary_count = summary_list.size();
    }
    else
    {
        rs_data_ptr->data.summary_count = 0;
    }
}

// "/rpc/reg_manager/sr/getValidList"
void ControllerRpc::handleRpc0x00009854(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_Int32List* rq_data_ptr = static_cast<RequestMessageType_Int32List*>(request_data_ptr);
    ResponseMessageType_BaseRegSummaryList* rs_data_ptr = static_cast<ResponseMessageType_BaseRegSummaryList*>(response_data_ptr);

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
            rs_data_ptr->data.summary[i].comment[31] = 0;
        }
        rs_data_ptr->data.summary_count = summary_list.size();
    }
    else
    {
        rs_data_ptr->data.summary_count = 0;
    }
}

// "/rpc/reg_manager/pr/addReg"
void ControllerRpc::handleRpc0x000154E7(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_PrRegData* rq_data_ptr = static_cast<RequestMessageType_PrRegData*>(request_data_ptr);
    ResponseMessageType_Bool* rs_data_ptr = static_cast<ResponseMessageType_Bool*>(response_data_ptr);

    PrRegData reg;
    reg.id = rq_data_ptr->data.id;
    reg.name = rq_data_ptr->data.name;
    reg.comment = rq_data_ptr->data.comment;
    reg.value.pos_type = rq_data_ptr->data.pos_type;
    reg.value.group_id = rq_data_ptr->data.group_id;
    memcpy(reg.value.pos, rq_data_ptr->data.pos.data, 9*sizeof(double));
    memcpy(reg.value.posture, rq_data_ptr->data.posture.data, 4*sizeof(bool));
    rs_data_ptr->data.data = reg_manager_ptr_->addPrReg(&reg);
}

// "/rpc/reg_manager/pr/deleteReg"
void ControllerRpc::handleRpc0x00001097(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_Int32* rq_data_ptr = static_cast<RequestMessageType_Int32*>(request_data_ptr);
    ResponseMessageType_Bool* rs_data_ptr = static_cast<ResponseMessageType_Bool*>(response_data_ptr);

    rs_data_ptr->data.data = reg_manager_ptr_->deletePrReg(rq_data_ptr->data.data);
}

// "/rpc/reg_manager/pr/updateReg"
void ControllerRpc::handleRpc0x00009EF7(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_PrRegData* rq_data_ptr = static_cast<RequestMessageType_PrRegData*>(request_data_ptr);
    ResponseMessageType_Bool* rs_data_ptr = static_cast<ResponseMessageType_Bool*>(response_data_ptr);

    PrRegData reg;
    reg.id = rq_data_ptr->data.id;
    reg.name = rq_data_ptr->data.name;
    reg.comment = rq_data_ptr->data.comment;
    reg.value.pos_type = rq_data_ptr->data.pos_type;
    reg.value.group_id = rq_data_ptr->data.group_id;
    memcpy(reg.value.pos, rq_data_ptr->data.pos.data, 9*sizeof(double));
    memcpy(reg.value.posture, rq_data_ptr->data.posture.data, 4*sizeof(double));
    rs_data_ptr->data.data = reg_manager_ptr_->updatePrReg(&reg);
}

// "/rpc/reg_manager/pr/getReg"
void ControllerRpc::handleRpc0x00017207(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_Int32* rq_data_ptr = static_cast<RequestMessageType_Int32*>(request_data_ptr);
    ResponseMessageType_Bool_PrRegData* rs_data_ptr = static_cast<ResponseMessageType_Bool_PrRegData*>(response_data_ptr);

    PrRegData reg;
    rs_data_ptr->success.data = reg_manager_ptr_->getPrReg(rq_data_ptr->data.data, &reg);
    if(rs_data_ptr->success.data)
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
        rs_data_ptr->data.posture.data_count = 4;
        memcpy(rs_data_ptr->data.posture.data, reg.value.posture, 4*sizeof(double));
    }
    else
    {
        memset(&rs_data_ptr->data, 0, sizeof(MessageType_PrRegData));
    }
}

// "/rpc/reg_manager/pr/moveReg"
void ControllerRpc::handleRpc0x0000D7C7(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_Int32List* rq_data_ptr = static_cast<RequestMessageType_Int32List*>(request_data_ptr);
    ResponseMessageType_Bool* rs_data_ptr = static_cast<ResponseMessageType_Bool*>(response_data_ptr);

    if(rq_data_ptr->data.data_count == 2)
    {
        rs_data_ptr->data.data = reg_manager_ptr_->movePrReg(rq_data_ptr->data.data[0], rq_data_ptr->data.data[1]);
    }
    else
    {
        rs_data_ptr->data.data = false;
    }
}

// "/rpc/reg_manager/pr/getChangedList"
void ControllerRpc::handleRpc0x0000B454(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_Int32List* rq_data_ptr = static_cast<RequestMessageType_Int32List*>(request_data_ptr);
    ResponseMessageType_BaseRegSummaryList* rs_data_ptr = static_cast<ResponseMessageType_BaseRegSummaryList*>(response_data_ptr);

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
            rs_data_ptr->data.summary[i].comment[31] = 0;
        }
        rs_data_ptr->data.summary_count = summary_list.size();
    }
    else
    {
        rs_data_ptr->data.summary_count = 0;
    }
}

// "/rpc/reg_manager/pr/getValidList"
void ControllerRpc::handleRpc0x00009354(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_Int32List* rq_data_ptr = static_cast<RequestMessageType_Int32List*>(request_data_ptr);
    ResponseMessageType_BaseRegSummaryList* rs_data_ptr = static_cast<ResponseMessageType_BaseRegSummaryList*>(response_data_ptr);

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
            rs_data_ptr->data.summary[i].comment[31] = 0;
        }
        rs_data_ptr->data.summary_count = summary_list.size();
    }
    else
    {
        rs_data_ptr->data.summary_count = 0;
    }
}

// "/rpc/reg_manager/hr/addReg"
void ControllerRpc::handleRpc0x00016CE7(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_HrRegData* rq_data_ptr = static_cast<RequestMessageType_HrRegData*>(request_data_ptr);
    ResponseMessageType_Bool* rs_data_ptr = static_cast<ResponseMessageType_Bool*>(response_data_ptr);

    HrRegData reg;
    reg.id = rq_data_ptr->data.id;
    reg.name = rq_data_ptr->data.name;
    reg.comment = rq_data_ptr->data.comment;
    reg.value.group_id = rq_data_ptr->data.group_id;
    memcpy(reg.value.joint_pos, rq_data_ptr->data.joints.data, 9*sizeof(double));
    memcpy(reg.value.diff_pos, rq_data_ptr->data.diffs.data, 9*sizeof(double));
    rs_data_ptr->data.data = reg_manager_ptr_->addHrReg(&reg);
}

// "/rpc/reg_manager/hr/deleteReg"
void ControllerRpc::handleRpc0x00003D17(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_Int32* rq_data_ptr = static_cast<RequestMessageType_Int32*>(request_data_ptr);
    ResponseMessageType_Bool* rs_data_ptr = static_cast<ResponseMessageType_Bool*>(response_data_ptr);

    rs_data_ptr->data.data = reg_manager_ptr_->deleteHrReg(rq_data_ptr->data.data);
}

// "/rpc/reg_manager/hr/updateReg"
void ControllerRpc::handleRpc0x0000CB77(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_HrRegData* rq_data_ptr = static_cast<RequestMessageType_HrRegData*>(request_data_ptr);
    ResponseMessageType_Bool* rs_data_ptr = static_cast<ResponseMessageType_Bool*>(response_data_ptr);

    HrRegData reg;
    reg.id = rq_data_ptr->data.id;
    reg.name = rq_data_ptr->data.name;
    reg.comment = rq_data_ptr->data.comment;
    reg.value.group_id = rq_data_ptr->data.group_id;
    memcpy(reg.value.joint_pos, rq_data_ptr->data.joints.data, 9*sizeof(double));
    memcpy(reg.value.diff_pos, rq_data_ptr->data.diffs.data, 9*sizeof(double));
    rs_data_ptr->data.data = reg_manager_ptr_->updateHrReg(&reg);
}

// "/rpc/reg_manager/hr/getReg"
void ControllerRpc::handleRpc0x00000367(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_Int32* rq_data_ptr = static_cast<RequestMessageType_Int32*>(request_data_ptr);
    ResponseMessageType_Bool_HrRegData* rs_data_ptr = static_cast<ResponseMessageType_Bool_HrRegData*>(response_data_ptr);

    HrRegData reg;
    rs_data_ptr->success.data = reg_manager_ptr_->getHrReg(rq_data_ptr->data.data, &reg);
    if(rs_data_ptr->success.data)
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
}

// "/rpc/reg_manager/hr/moveReg"
void ControllerRpc::handleRpc0x00014A87(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_Int32List* rq_data_ptr = static_cast<RequestMessageType_Int32List*>(request_data_ptr);
    ResponseMessageType_Bool* rs_data_ptr = static_cast<ResponseMessageType_Bool*>(response_data_ptr);

    if(rq_data_ptr->data.data_count == 2)
    {
        rs_data_ptr->data.data = reg_manager_ptr_->moveHrReg(rq_data_ptr->data.data[0], rq_data_ptr->data.data[1]);
    }
    else
    {
        rs_data_ptr->data.data = false;
    }
}

// "/rpc/reg_manager/hr/getChangedList"
void ControllerRpc::handleRpc0x00012974(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_Int32List* rq_data_ptr = static_cast<RequestMessageType_Int32List*>(request_data_ptr);
    ResponseMessageType_BaseRegSummaryList* rs_data_ptr = static_cast<ResponseMessageType_BaseRegSummaryList*>(response_data_ptr);

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
            rs_data_ptr->data.summary[i].comment[31] = 0;
        }
        rs_data_ptr->data.summary_count = summary_list.size();
    }
    else
    {
        rs_data_ptr->data.summary_count = 0;
    }
}

// "/rpc/reg_manager/hr/getValidList"
void ControllerRpc::handleRpc0x00006B54(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_Int32List* rq_data_ptr = static_cast<RequestMessageType_Int32List*>(request_data_ptr);
    ResponseMessageType_BaseRegSummaryList* rs_data_ptr = static_cast<ResponseMessageType_BaseRegSummaryList*>(response_data_ptr);

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
            rs_data_ptr->data.summary[i].comment[31] = 0;
        }
        rs_data_ptr->data.summary_count = summary_list.size();
    }
    else
    {
        rs_data_ptr->data.summary_count = 0;
    }
}

