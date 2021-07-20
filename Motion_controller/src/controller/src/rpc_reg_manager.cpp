#include "controller_rpc.h"

using namespace fst_ctrl;
using namespace user_space;
using namespace log_space;
using namespace group_space;


//"/rpc/reg_manager/pr/addReg"	
void ControllerRpc::handleRpc0x000154E7(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_PrRegData* rq_data_ptr = static_cast<RequestMessageType_PrRegData*>(request_data_ptr);
    ResponseMessageType_Uint64* rs_data_ptr = static_cast<ResponseMessageType_Uint64*>(response_data_ptr);

    PrRegData reg;
    reg.id = rq_data_ptr->data.id;
    reg.name = rq_data_ptr->data.name;
    reg.comment = rq_data_ptr->data.comment;
    reg.value.pos_type = static_cast<uint8_t>(rq_data_ptr->data.pos_type);
    reg.value.group_id = rq_data_ptr->data.group_id;
    memcpy(reg.value.pos, rq_data_ptr->data.pos.data, 9*sizeof(double));

    reg.value.posture[3] = static_cast<int8_t>(rq_data_ptr->data.posture.wrist_flip);
    reg.value.posture[2] = static_cast<int8_t>(rq_data_ptr->data.posture.arm_up_down);
    reg.value.posture[1] = static_cast<int8_t>(rq_data_ptr->data.posture.arm_back_front);
    reg.value.posture[0] = static_cast<int8_t>(rq_data_ptr->data.posture.arm_left_right);
    reg.value.turn[0] = static_cast<int8_t>(rq_data_ptr->data.posture.turn_cycle.data[0]);
    reg.value.turn[1] = static_cast<int8_t>(rq_data_ptr->data.posture.turn_cycle.data[1]);
    reg.value.turn[2] = static_cast<int8_t>(rq_data_ptr->data.posture.turn_cycle.data[2]);
    reg.value.turn[3] = static_cast<int8_t>(rq_data_ptr->data.posture.turn_cycle.data[3]);
    reg.value.turn[4] = static_cast<int8_t>(rq_data_ptr->data.posture.turn_cycle.data[4]);
    reg.value.turn[5] = static_cast<int8_t>(rq_data_ptr->data.posture.turn_cycle.data[5]);
    reg.value.turn[6] = static_cast<int8_t>(rq_data_ptr->data.posture.turn_cycle.data[6]);
    reg.value.turn[7] = static_cast<int8_t>(rq_data_ptr->data.posture.turn_cycle.data[7]);
    reg.value.turn[8] = static_cast<int8_t>(rq_data_ptr->data.posture.turn_cycle.data[8]);

    if (rq_data_ptr->data.posture.turn_cycle.data_count != 9
        || reg_manager_ptr_->getNameLengthLimit() < reg.name.size()
        || reg_manager_ptr_->getCommentLengthLimit() < reg.comment.size())
    {
        rs_data_ptr->data.data = CONTROLLER_INVALID_OPERATION;
        LogProducer::error("rpc", "/rpc/reg_manager/pr/addReg invalid inputs, ret = %llx\n", rs_data_ptr->data.data);
        return;
    }

    rs_data_ptr->data.data = reg_manager_ptr_->addPrReg(&reg);


    if (rs_data_ptr->data.data == SUCCESS)
        LogProducer::info("rpc", "/rpc/reg_manager/pr/addReg id=%d success", reg.id);
    else
        LogProducer::error("rpc", "/rpc/reg_manager/pr/addReg id=%d failed. Error = 0x%llx", reg.id, rs_data_ptr->data.data);
}
//"/rpc/reg_manager/pr/deleteReg"	
void ControllerRpc::handleRpc0x00001097(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_Int32* rq_data_ptr = static_cast<RequestMessageType_Int32*>(request_data_ptr);
    ResponseMessageType_Uint64* rs_data_ptr = static_cast<ResponseMessageType_Uint64*>(response_data_ptr);

    rs_data_ptr->data.data = reg_manager_ptr_->deletePrReg(rq_data_ptr->data.data);

    if (rs_data_ptr->data.data == SUCCESS)
        LogProducer::info("rpc", "/rpc/reg_manager/pr/deleteReg id=%d success", rq_data_ptr->data.data);
    else
        LogProducer::error("rpc", "/rpc/reg_manager/pr/deleteReg id=%d failed. Error = 0x%llx", rq_data_ptr->data.data, rs_data_ptr->data.data);
}
//"/rpc/reg_manager/pr/updateReg"	
void ControllerRpc::handleRpc0x00009EF7(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_PrRegData* rq_data_ptr = static_cast<RequestMessageType_PrRegData*>(request_data_ptr);
    ResponseMessageType_Uint64* rs_data_ptr = static_cast<ResponseMessageType_Uint64*>(response_data_ptr);

    PrRegData reg;
    reg.id = rq_data_ptr->data.id;
    reg.name = rq_data_ptr->data.name;
    reg.comment = rq_data_ptr->data.comment;
    reg.value.pos_type = static_cast<uint8_t>(rq_data_ptr->data.pos_type);
    reg.value.group_id = rq_data_ptr->data.group_id;
    memcpy(reg.value.pos, rq_data_ptr->data.pos.data, 9*sizeof(double));

    reg.value.posture[3] = static_cast<int8_t>(rq_data_ptr->data.posture.wrist_flip);
    reg.value.posture[2] = static_cast<int8_t>(rq_data_ptr->data.posture.arm_up_down);
    reg.value.posture[1] = static_cast<int8_t>(rq_data_ptr->data.posture.arm_back_front);
    reg.value.posture[0] = static_cast<int8_t>(rq_data_ptr->data.posture.arm_left_right);
    reg.value.turn[0] = static_cast<int8_t>(rq_data_ptr->data.posture.turn_cycle.data[0]);
    reg.value.turn[1] = static_cast<int8_t>(rq_data_ptr->data.posture.turn_cycle.data[1]);
    reg.value.turn[2] = static_cast<int8_t>(rq_data_ptr->data.posture.turn_cycle.data[2]);
    reg.value.turn[3] = static_cast<int8_t>(rq_data_ptr->data.posture.turn_cycle.data[3]);
    reg.value.turn[4] = static_cast<int8_t>(rq_data_ptr->data.posture.turn_cycle.data[4]);
    reg.value.turn[5] = static_cast<int8_t>(rq_data_ptr->data.posture.turn_cycle.data[5]);
    reg.value.turn[6] = static_cast<int8_t>(rq_data_ptr->data.posture.turn_cycle.data[6]);
    reg.value.turn[7] = static_cast<int8_t>(rq_data_ptr->data.posture.turn_cycle.data[7]);
    reg.value.turn[8] = static_cast<int8_t>(rq_data_ptr->data.posture.turn_cycle.data[8]);

    if (rq_data_ptr->data.posture.turn_cycle.data_count != 9
        || reg_manager_ptr_->getNameLengthLimit() < reg.name.size()
        || reg_manager_ptr_->getCommentLengthLimit() < reg.comment.size())
    {
        rs_data_ptr->data.data = CONTROLLER_INVALID_OPERATION;
        LogProducer::error("rpc", "/rpc/reg_manager/pr/updateReg invalid inputs, ret = %llx\n", rs_data_ptr->data.data);
        return;
    }

    rs_data_ptr->data.data = reg_manager_ptr_->updatePrReg(&reg);

    if (rs_data_ptr->data.data == SUCCESS)
        LogProducer::info("rpc", "/rpc/reg_manager/pr/updateReg id=%d success", reg.id );
    else
        LogProducer::error("rpc", "/rpc/reg_manager/pr/updateReg id=%d failed. Error = 0x%llx", reg.id, rs_data_ptr->data.data);
}
//"/rpc/reg_manager/pr/getReg"	
void ControllerRpc::handleRpc0x00017207(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_Int32* rq_data_ptr = static_cast<RequestMessageType_Int32*>(request_data_ptr);
    ResponseMessageType_Uint64_PrRegData* rs_data_ptr = static_cast<ResponseMessageType_Uint64_PrRegData*>(response_data_ptr);

    PrRegData reg;
    rs_data_ptr->error_code.data = reg_manager_ptr_->getPrReg(rq_data_ptr->data.data, &reg);
    if(rs_data_ptr->error_code.data == SUCCESS)
    {
        rs_data_ptr->data.id = reg.id;
        strncpy(rs_data_ptr->data.name, reg.name.c_str(), reg.name.size());
        rs_data_ptr->data.name[reg.name.size()] = 0;
        strncpy(rs_data_ptr->data.comment, reg.comment.c_str(), reg.comment.size());
        rs_data_ptr->data.comment[reg.comment.size()] = 0;
        rs_data_ptr->data.group_id = reg.value.group_id;
        rs_data_ptr->data.pos_type = reg.value.pos_type;
        rs_data_ptr->data.pos.data_count = 9;
        memcpy(rs_data_ptr->data.pos.data, reg.value.pos, 9*sizeof(double));

        rs_data_ptr->data.posture.wrist_flip = reg.value.posture[3];        // wrist
        rs_data_ptr->data.posture.arm_up_down = reg.value.posture[2];       // elbow
        rs_data_ptr->data.posture.arm_back_front = reg.value.posture[1];    // arm
        rs_data_ptr->data.posture.arm_left_right = reg.value.posture[0];    // reserved for diff model

        rs_data_ptr->data.posture.turn_cycle.data_count = 9;
        rs_data_ptr->data.posture.turn_cycle.data[0] = reg.value.turn[0];
        rs_data_ptr->data.posture.turn_cycle.data[1] = reg.value.turn[1];
        rs_data_ptr->data.posture.turn_cycle.data[2] = reg.value.turn[2];
        rs_data_ptr->data.posture.turn_cycle.data[3] = reg.value.turn[3];
        rs_data_ptr->data.posture.turn_cycle.data[4] = reg.value.turn[4];
        rs_data_ptr->data.posture.turn_cycle.data[5] = reg.value.turn[5];
        rs_data_ptr->data.posture.turn_cycle.data[6] = reg.value.turn[6];
        rs_data_ptr->data.posture.turn_cycle.data[7] = reg.value.turn[7];
        rs_data_ptr->data.posture.turn_cycle.data[8] = reg.value.turn[8];
        LogProducer::info("rpc", "/rpc/reg_manager/pr/getReg id=%d success", reg.id);
    }
    else
    {
        memset(&rs_data_ptr->data, 0, sizeof(MessageType_PrRegData));
        LogProducer::error("rpc", "/rpc/reg_manager/pr/getReg id=%d failed. Error = 0x%llx", reg.id, rs_data_ptr->error_code.data);
    }
}
//"/rpc/reg_manager/pr/moveReg"	
void ControllerRpc::handleRpc0x0000D7C7(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_Int32List* rq_data_ptr = static_cast<RequestMessageType_Int32List*>(request_data_ptr);
    ResponseMessageType_Uint64* rs_data_ptr = static_cast<ResponseMessageType_Uint64*>(response_data_ptr);

    if(rq_data_ptr->data.data_count == 2)
    {
        rs_data_ptr->data.data = reg_manager_ptr_->movePrReg(rq_data_ptr->data.data[0], rq_data_ptr->data.data[1]);
    }
    else
    {
        rs_data_ptr->data.data = REG_MANAGER_INVALID_ARG;
    }

    if (rs_data_ptr->data.data == SUCCESS)
        LogProducer::info("rpc", "/rpc/reg_manager/pr/moveReg %d to %d success", rq_data_ptr->data.data[1], rq_data_ptr->data.data[0]);
    else
        LogProducer::error("rpc", "/rpc/reg_manager/pr/moveReg %d to %d failed. Error = 0x%llx", rq_data_ptr->data.data[1], rq_data_ptr->data.data[0], rs_data_ptr->data.data);
}
//"/rpc/reg_manager/pr/getChangedList"	
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
            strncpy(rs_data_ptr->data.summary[i].name, summary_list[i].name.c_str(), summary_list[i].name.size());
            rs_data_ptr->data.summary[i].name[summary_list[i].name.size()] = 0;
            strncpy(rs_data_ptr->data.summary[i].comment, summary_list[i].comment.c_str(), summary_list[i].comment.size());
            rs_data_ptr->data.summary[i].comment[summary_list[i].comment.size()] = 0;
        }
        rs_data_ptr->data.summary_count = summary_list.size();
        rs_data_ptr->error_code.data = SUCCESS;
    }
    else
    {
        rs_data_ptr->data.summary_count = 0;
        rs_data_ptr->error_code.data = REG_MANAGER_INVALID_ARG;
    }
    if (rs_data_ptr->error_code.data == SUCCESS)
        LogProducer::info("rpc", "/rpc/reg_manager/pr/getChangedList success");
    else
        LogProducer::error("rpc", "/rpc/reg_manager/pr/getChangedList failed. Error = 0x%llx", rs_data_ptr->error_code.data);
}
//"/rpc/reg_manager/pr/getValidList"	
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
            strncpy(rs_data_ptr->data.summary[i].name, summary_list[i].name.c_str(), summary_list[i].name.size());
            rs_data_ptr->data.summary[i].name[summary_list[i].name.size()] = 0;
            strncpy(rs_data_ptr->data.summary[i].comment, summary_list[i].comment.c_str(), summary_list[i].comment.size());
            rs_data_ptr->data.summary[i].comment[summary_list[i].comment.size()] = 0;
        }
        rs_data_ptr->data.summary_count = summary_list.size();
        rs_data_ptr->error_code.data = SUCCESS;
    }
    else
    {
        rs_data_ptr->data.summary_count = 0;
        rs_data_ptr->error_code.data = REG_MANAGER_INVALID_ARG;
    }
    if (rs_data_ptr->error_code.data == SUCCESS)
        LogProducer::info("rpc", "/rpc/reg_manager/pr/getValidList success");
    else
        LogProducer::error("rpc", "/rpc/reg_manager/pr/getValidList failed. Error = 0x%llx", rs_data_ptr->error_code.data);
}



