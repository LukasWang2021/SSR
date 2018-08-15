#include "controller_ipc.h"
#include "reg_manager.h"
#include <unistd.h>
#include <stdlib.h>
#include <cstring>


using namespace fst_ctrl;
using namespace std;


void ControllerIpc::handleIpcSetPrReg(void* request_data_ptr, void* response_data_ptr)
{
    PrRegDataIpc* rq_data_ptr = static_cast<PrRegDataIpc*>(request_data_ptr);
    bool* rs_data_ptr = static_cast<bool*>(response_data_ptr);

    PrRegData data;
    data.id = rq_data_ptr->id;
    data.name = rq_data_ptr->name;
    data.comment = rq_data_ptr->comment;
    memcpy(&data.value, &rq_data_ptr->value, sizeof(PrValue));
    *rs_data_ptr = reg_manager_ptr_->updatePrReg(&data);
}

void ControllerIpc::handleIpcSetHrReg(void* request_data_ptr, void* response_data_ptr)
{
    HrRegDataIpc* rq_data_ptr = static_cast<HrRegDataIpc*>(request_data_ptr);
    bool* rs_data_ptr = static_cast<bool*>(response_data_ptr);

    HrRegData data;
    data.id = rq_data_ptr->id;
    data.name = rq_data_ptr->name;
    data.comment = rq_data_ptr->comment;
    memcpy(&data.value, &rq_data_ptr->value, sizeof(HrValue));
    *rs_data_ptr = reg_manager_ptr_->updateHrReg(&data);
}

void ControllerIpc::handleIpcSetMrReg(void* request_data_ptr, void* response_data_ptr)
{
    MrRegDataIpc* rq_data_ptr = static_cast<MrRegDataIpc*>(request_data_ptr);
    bool* rs_data_ptr = static_cast<bool*>(response_data_ptr);

    MrRegData data;
    data.id = rq_data_ptr->id;
    data.name = rq_data_ptr->name;
    data.comment = rq_data_ptr->comment;
    data.value = rq_data_ptr->value;
    *rs_data_ptr = reg_manager_ptr_->updateMrReg(&data);
}

void ControllerIpc::handleIpcSetSrReg(void* request_data_ptr, void* response_data_ptr)
{
    SrRegDataIpc* rq_data_ptr = static_cast<SrRegDataIpc*>(request_data_ptr);
    bool* rs_data_ptr = static_cast<bool*>(response_data_ptr);

    SrRegData data;
    data.id = rq_data_ptr->id;
    data.name = rq_data_ptr->name;
    data.comment = rq_data_ptr->comment;
    data.value = rq_data_ptr->value;
    *rs_data_ptr = reg_manager_ptr_->updateSrReg(&data);
}

void ControllerIpc::handleIpcSetRReg(void* request_data_ptr, void* response_data_ptr)
{
    RRegDataIpc* rq_data_ptr = static_cast<RRegDataIpc*>(request_data_ptr);
    bool* rs_data_ptr = static_cast<bool*>(response_data_ptr);

    RRegData data;
    data.id = rq_data_ptr->id;
    data.name = rq_data_ptr->name;
    data.comment = rq_data_ptr->comment;
    data.value = rq_data_ptr->value;
    *rs_data_ptr = reg_manager_ptr_->updateRReg(&data);
}

void ControllerIpc::handleIpcGetPrReg(void* request_data_ptr, void* response_data_ptr)
{
    int* rq_data_ptr = static_cast<int*>(request_data_ptr);
    PrRegDataIpc* rs_data_ptr = static_cast<PrRegDataIpc*>(response_data_ptr);

    PrRegData data;
    if(!reg_manager_ptr_->getPrReg(*rq_data_ptr, &data))
    {
        rs_data_ptr->id = 0;
    }
    else
    {
        rs_data_ptr->id = data.id;
        strncpy(rs_data_ptr->name, data.name.c_str(), 31);
        rs_data_ptr->name[31] = 0;
        strncpy(rs_data_ptr->comment, data.comment.c_str(), 255);
        rs_data_ptr->comment[255] = 0;
        rs_data_ptr->value = data.value;
    }
}

void ControllerIpc::handleIpcGetHrReg(void* request_data_ptr, void* response_data_ptr)
{
    int* rq_data_ptr = static_cast<int*>(request_data_ptr);
    HrRegDataIpc* rs_data_ptr = static_cast<HrRegDataIpc*>(response_data_ptr);

    HrRegData data;
    if(!reg_manager_ptr_->getHrReg(*rq_data_ptr, &data))
    {
        rs_data_ptr->id = 0;
    }
    else
    {
        rs_data_ptr->id = data.id;
        strncpy(rs_data_ptr->name, data.name.c_str(), 31);
        rs_data_ptr->name[31] = 0;
        strncpy(rs_data_ptr->comment, data.comment.c_str(), 255);
        rs_data_ptr->comment[255] = 0;
        rs_data_ptr->value = data.value;
    }    
}

void ControllerIpc::handleIpcGetMrReg(void* request_data_ptr, void* response_data_ptr)
{
    int* rq_data_ptr = static_cast<int*>(request_data_ptr);
    MrRegDataIpc* rs_data_ptr = static_cast<MrRegDataIpc*>(response_data_ptr);

    MrRegData data;
    if(!reg_manager_ptr_->getMrReg(*rq_data_ptr, &data))
    {
        rs_data_ptr->id = 0;
    }
    else
    {
        rs_data_ptr->id = data.id;
        strncpy(rs_data_ptr->name, data.name.c_str(), 31);
        rs_data_ptr->name[31] = 0;
        strncpy(rs_data_ptr->comment, data.comment.c_str(), 255);
        rs_data_ptr->comment[255] = 0;
        rs_data_ptr->value = data.value;
    }  
}

void ControllerIpc::handleIpcGetSrReg(void* request_data_ptr, void* response_data_ptr)
{
    int* rq_data_ptr = static_cast<int*>(request_data_ptr);
    SrRegDataIpc* rs_data_ptr = static_cast<SrRegDataIpc*>(response_data_ptr);

    SrRegData data;
    if(!reg_manager_ptr_->getSrReg(*rq_data_ptr, &data))
    {
        rs_data_ptr->id = 0;
    }
    else
    {
        rs_data_ptr->id = data.id;
        strncpy(rs_data_ptr->name, data.name.c_str(), 31);
        rs_data_ptr->name[31] = 0;
        strncpy(rs_data_ptr->comment, data.comment.c_str(), 255);
        rs_data_ptr->comment[255] = 0;
        strncpy(rs_data_ptr->value, data.value.c_str(), 255);
        rs_data_ptr->value[255] = 0;
    }      
}

void ControllerIpc::handleIpcGetRReg(void* request_data_ptr, void* response_data_ptr)
{
    int* rq_data_ptr = static_cast<int*>(request_data_ptr);
    RRegDataIpc* rs_data_ptr = static_cast<RRegDataIpc*>(response_data_ptr);

    RRegData data;
    if(!reg_manager_ptr_->getRReg(*rq_data_ptr, &data))
    {
        rs_data_ptr->id = 0;
    }
    else
    {
        rs_data_ptr->id = data.id;
        strncpy(rs_data_ptr->name, data.name.c_str(), 31);
        rs_data_ptr->name[31] = 0;
        strncpy(rs_data_ptr->comment, data.comment.c_str(), 255);
        rs_data_ptr->comment[255] = 0;
        rs_data_ptr->value = data.value;
    } 
}


