#include "controller_ipc.h"
#include "reg_manager.h"

using namespace fst_ctrl;


void ControllerIpc::handleIpcSetPrReg(void* request_data_ptr, void* response_data_ptr)
{
    PrRegData* rq_data_ptr = static_cast<PrRegData*>(request_data_ptr);
    bool* rs_data_ptr = static_cast<bool*>(response_data_ptr);
    *rs_data_ptr = reg_manager_ptr_->updatePrReg(rq_data_ptr);
}

void ControllerIpc::handleIpcSetHrReg(void* request_data_ptr, void* response_data_ptr)
{
    HrRegData* rq_data_ptr = static_cast<HrRegData*>(request_data_ptr);
    bool* rs_data_ptr = static_cast<bool*>(response_data_ptr);
    *rs_data_ptr = reg_manager_ptr_->updateHrReg(rq_data_ptr);
}

void ControllerIpc::handleIpcSetMrReg(void* request_data_ptr, void* response_data_ptr)
{
    MrRegData* rq_data_ptr = static_cast<MrRegData*>(request_data_ptr);
    bool* rs_data_ptr = static_cast<bool*>(response_data_ptr);
    *rs_data_ptr = reg_manager_ptr_->updateMrReg(rq_data_ptr);
}

void ControllerIpc::handleIpcSetSrReg(void* request_data_ptr, void* response_data_ptr)
{
    SrRegData* rq_data_ptr = static_cast<SrRegData*>(request_data_ptr);
    bool* rs_data_ptr = static_cast<bool*>(response_data_ptr);
    *rs_data_ptr = reg_manager_ptr_->updateSrReg(rq_data_ptr);
}

void ControllerIpc::handleIpcSetRReg(void* request_data_ptr, void* response_data_ptr)
{
    RRegData* rq_data_ptr = static_cast<RRegData*>(request_data_ptr);
    bool* rs_data_ptr = static_cast<bool*>(response_data_ptr);
    *rs_data_ptr = reg_manager_ptr_->updateRReg(rq_data_ptr);
}

void ControllerIpc::handleIpcGetPrReg(void* request_data_ptr, void* response_data_ptr)
{
    int* rq_data_ptr = static_cast<int*>(request_data_ptr);
    PrRegData* rs_data_ptr = static_cast<PrRegData*>(response_data_ptr);

    if(!reg_manager_ptr_->getPrReg(*rq_data_ptr, rs_data_ptr))
    {
        rs_data_ptr->id = 0;
    }
}

void ControllerIpc::handleIpcGetHrReg(void* request_data_ptr, void* response_data_ptr)
{
    int* rq_data_ptr = static_cast<int*>(request_data_ptr);
    HrRegData* rs_data_ptr = static_cast<HrRegData*>(response_data_ptr);

    if(!reg_manager_ptr_->getHrReg(*rq_data_ptr, rs_data_ptr))
    {
        rs_data_ptr->id = 0;
    }
}

void ControllerIpc::handleIpcGetMrReg(void* request_data_ptr, void* response_data_ptr)
{
    int* rq_data_ptr = static_cast<int*>(request_data_ptr);
    MrRegData* rs_data_ptr = static_cast<MrRegData*>(response_data_ptr);

    if(!reg_manager_ptr_->getMrReg(*rq_data_ptr, rs_data_ptr))
    {
        rs_data_ptr->id = 0;
    }
}

void ControllerIpc::handleIpcGetSrReg(void* request_data_ptr, void* response_data_ptr)
{
    int* rq_data_ptr = static_cast<int*>(request_data_ptr);
    SrRegData* rs_data_ptr = static_cast<SrRegData*>(response_data_ptr);

    if(!reg_manager_ptr_->getSrReg(*rq_data_ptr, rs_data_ptr))
    {
        rs_data_ptr->id = 0;
    }
}

void ControllerIpc::handleIpcGetRReg(void* request_data_ptr, void* response_data_ptr)
{
    int* rq_data_ptr = static_cast<int*>(request_data_ptr);
    RRegData* rs_data_ptr = static_cast<RRegData*>(response_data_ptr);

    if(!reg_manager_ptr_->getRReg(*rq_data_ptr, rs_data_ptr))
    {
        rs_data_ptr->id = 0;
    }
}


