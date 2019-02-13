#include "controller_ipc.h"
#include "reg_manager.h"
#include <unistd.h>
#include <stdlib.h>
#include <cstring>

using namespace fst_ctrl;
using namespace std;


void ControllerIpc::handleIpcSetPrRegPos(void* request_data_ptr, void* response_data_ptr)
{
    PrRegDataIpc* rq_data_ptr = static_cast<PrRegDataIpc*>(request_data_ptr);
    bool* rs_data_ptr = static_cast<bool*>(response_data_ptr);

    *rs_data_ptr = reg_manager_ptr_->updatePrRegPos(rq_data_ptr);
}

void ControllerIpc::handleIpcSetHrRegJointPos(void* request_data_ptr, void* response_data_ptr)
{
    HrRegDataIpc* rq_data_ptr = static_cast<HrRegDataIpc*>(request_data_ptr);
    bool* rs_data_ptr = static_cast<bool*>(response_data_ptr);

    *rs_data_ptr = reg_manager_ptr_->updateHrRegJointPos(rq_data_ptr);
}

void ControllerIpc::handleIpcSetMrRegValue(void* request_data_ptr, void* response_data_ptr)
{
    MrRegDataIpc* rq_data_ptr = static_cast<MrRegDataIpc*>(request_data_ptr);
    bool* rs_data_ptr = static_cast<bool*>(response_data_ptr);

    *rs_data_ptr = reg_manager_ptr_->updateMrRegValue(rq_data_ptr);
}

void ControllerIpc::handleIpcSetSrRegValue(void* request_data_ptr, void* response_data_ptr)
{
    SrRegDataIpc* rq_data_ptr = static_cast<SrRegDataIpc*>(request_data_ptr);
    bool* rs_data_ptr = static_cast<bool*>(response_data_ptr);

    *rs_data_ptr = reg_manager_ptr_->updateSrRegValue(rq_data_ptr);
}

void ControllerIpc::handleIpcSetRRegValue(void* request_data_ptr, void* response_data_ptr)
{
    RRegDataIpc* rq_data_ptr = static_cast<RRegDataIpc*>(request_data_ptr);
    bool* rs_data_ptr = static_cast<bool*>(response_data_ptr);

    *rs_data_ptr = reg_manager_ptr_->updateRRegValue(rq_data_ptr);   
}

void ControllerIpc::handleIpcGetPrRegPos(void* request_data_ptr, void* response_data_ptr)
{
    int* rq_data_ptr = static_cast<int*>(request_data_ptr);
    PrRegDataIpc* rs_data_ptr = static_cast<PrRegDataIpc*>(response_data_ptr);
    
    if(!reg_manager_ptr_->getPrRegPos(*rq_data_ptr, rs_data_ptr))
    {
        rs_data_ptr->id = 0;
    }
}

void ControllerIpc::handleIpcGetHrRegJointPos(void* request_data_ptr, void* response_data_ptr)
{
    int* rq_data_ptr = static_cast<int*>(request_data_ptr);
    HrRegDataIpc* rs_data_ptr = static_cast<HrRegDataIpc*>(response_data_ptr);

    if(!reg_manager_ptr_->getHrRegJointPos(*rq_data_ptr, rs_data_ptr))
    {
        rs_data_ptr->id = 0;
    }  
}

void ControllerIpc::handleIpcGetMrRegValue(void* request_data_ptr, void* response_data_ptr)
{
    int* rq_data_ptr = static_cast<int*>(request_data_ptr);
    MrRegDataIpc* rs_data_ptr = static_cast<MrRegDataIpc*>(response_data_ptr);

    if(!reg_manager_ptr_->getMrRegValue(*rq_data_ptr, rs_data_ptr))
    {
        rs_data_ptr->id = 0;
    }
}

void ControllerIpc::handleIpcGetSrRegValue(void* request_data_ptr, void* response_data_ptr)
{
    int* rq_data_ptr = static_cast<int*>(request_data_ptr);
    SrRegDataIpc* rs_data_ptr = static_cast<SrRegDataIpc*>(response_data_ptr);

    if(!reg_manager_ptr_->getSrRegValue(*rq_data_ptr, rs_data_ptr))
    {
        rs_data_ptr->id = 0;
    }     
}

void ControllerIpc::handleIpcGetRRegValue(void* request_data_ptr, void* response_data_ptr)
{
    int* rq_data_ptr = static_cast<int*>(request_data_ptr);
    RRegDataIpc* rs_data_ptr = static_cast<RRegDataIpc*>(response_data_ptr);

    if(!reg_manager_ptr_->getRRegValue(*rq_data_ptr, rs_data_ptr))
    {
        rs_data_ptr->id = 0;
    }
}

void ControllerIpc::handleIpcGetMiValue(void* request_data_ptr, void* response_data_ptr)
{
    int* rq_data_ptr = static_cast<int*>(request_data_ptr);
    fst_base::MiDataIpc* rs_data_ptr = static_cast<fst_base::MiDataIpc*>(response_data_ptr);

    uint16_t reg[2] = {0, 0}; 

    if (modbus_manager_ptr_->readInputRegs(0, *rq_data_ptr, 2, &reg[0]) == SUCCESS)
    {
        int32_t h_reg = reg[1]<<16;
        rs_data_ptr->value = reg[0] + h_reg;
        rs_data_ptr->id = *rq_data_ptr;
    }
    else
    {
        rs_data_ptr->id = 0;
    }
    
}



