#include "reg_manager.h"
#include "common_file_path.h"
#include <unistd.h>
#include <fstream>

using namespace std;
using namespace fst_ctrl;
using namespace rtm_nvram;
using namespace log_space;

RegManager::RegManager():
    param_ptr_(NULL)
{
    param_ptr_ = new RegManagerParam();
    nvram_ptr_ = new NvramHandler();
    for(int i=0; i < REG_TYPE_MAX; ++i)
    {
        reg_ptr_[i] = NULL;
    }
}

RegManager::~RegManager()
{
    for(int i = 0; i < REG_TYPE_MAX; ++i)
    {
        if (reg_ptr_[i] != NULL)
        {
            delete reg_ptr_[i];
            reg_ptr_[i] = NULL;
        }
    }

    if(nvram_ptr_ != NULL)
    {
        delete nvram_ptr_;
        nvram_ptr_ = NULL;
    }

    if(param_ptr_ != NULL)
    {
        delete param_ptr_;
        param_ptr_ = NULL;
    }
}

ErrorCode RegManager::init()
{
    if (!param_ptr_->loadParam())
    {
        return REG_MANAGER_LOAD_PARAM_FAILED;
    }

    bool res = nvram_ptr_->init();

    if (!res)
    {
        LogProducer::error("reg_manager", "Fail to inti nvram handler");
        return REG_MANAGER_INIT_OBJECT_FAILED;
    }

    reg_ptr_[REG_TYPE_PR] = new PrReg(param_ptr_, nvram_ptr_);
    reg_ptr_[REG_TYPE_HR] = new HrReg(param_ptr_, nvram_ptr_);
    reg_ptr_[REG_TYPE_SR] = new SrReg(param_ptr_, nvram_ptr_);
    reg_ptr_[REG_TYPE_MR] = new MrReg(param_ptr_, nvram_ptr_);
    reg_ptr_[REG_TYPE_R] = new RReg(param_ptr_, nvram_ptr_);

    for (int i = 0; i < REG_TYPE_MAX; ++i)
    {
        if (reg_ptr_[i] != NULL)
        {
            ErrorCode error_code = reg_ptr_[i]->init();
            
            if (error_code != SUCCESS)
            {
 				LogProducer::error("reg_manager", "RegManager::init[%d] error_code = 0x%llx", i, error_code);
                return error_code;
            }
        }
        else
        {
			LogProducer::error("reg_manager", "RegManager::init REG_MANAGER_INIT_OBJECT_FAILED");
            return REG_MANAGER_INIT_OBJECT_FAILED;
        }
    }

    return SUCCESS;
}

size_t RegManager::getNameLengthLimit()
{
    return param_ptr_->name_length_limit_;
}
size_t RegManager::getCommentLengthLimit()
{
    return param_ptr_->comment_length_limit_;
}
size_t RegManager::getSrValueLengthLimit()
{
    return param_ptr_->sr_value_length_limit_;
}

ErrorCode RegManager::backupNvram()
{
    string backup_file = NVRAM_DIR;
    backup_file = backup_file + "nvram.backup";
    ofstream output_handler(backup_file, ios::binary);
    uint8_t data[PR_DATA_SIZE + HR_DATA_SIZE + SR_DATA_SIZE + MR_DATA_SIZE + R_DATA_SIZE];

    if (!nvram_ptr_->readNvram(0, data, sizeof(data)))
    {
        return REG_MANAGER_OPERATE_NVRAM_FAILED;
    }

    output_handler.write(reinterpret_cast<char*>(data), sizeof(data));
    output_handler.flush();
    output_handler.close();
    return SUCCESS;
}

ErrorCode RegManager::restoreNvram()
{
    string backup_file = NVRAM_DIR;
    backup_file = backup_file + "nvram.backup";
    ifstream input_handler(backup_file, ios::binary);
    uint8_t data[PR_DATA_SIZE + HR_DATA_SIZE + SR_DATA_SIZE + MR_DATA_SIZE + R_DATA_SIZE];
    input_handler.read(reinterpret_cast<char*>(data), sizeof(data));
    input_handler.close();
    return nvram_ptr_->writeNvram(0, data, sizeof(data)) ? SUCCESS : REG_MANAGER_OPERATE_NVRAM_FAILED;
}

bool RegManager::isRegValid(RegType reg_type, int reg_index)
{
    switch(reg_type)
    {
        case REG_TYPE_PR:   return reg_ptr_[REG_TYPE_PR]->isRegValid(reg_index);
        case REG_TYPE_HR:   return reg_ptr_[REG_TYPE_HR]->isRegValid(reg_index);
        case REG_TYPE_SR:   return reg_ptr_[REG_TYPE_SR]->isRegValid(reg_index);
        case REG_TYPE_MR:   return reg_ptr_[REG_TYPE_MR]->isRegValid(reg_index);
        case REG_TYPE_R:    return reg_ptr_[REG_TYPE_R]->isRegValid(reg_index);
        default:            return false;
    }
}

ErrorCode RegManager::addPrReg(PrRegData* data_ptr)
{
    return reg_ptr_[REG_TYPE_PR]->addReg((void*)data_ptr);
}

ErrorCode RegManager::deletePrReg(int id)
{
    return reg_ptr_[REG_TYPE_PR]->deleteReg(id);
}

ErrorCode RegManager::getPrReg(int id, PrRegData* data_ptr)
{
    return reg_ptr_[REG_TYPE_PR]->getReg(id, (void*)data_ptr);
}

ErrorCode RegManager::updatePrReg(PrRegData* data_ptr)
{
    return reg_ptr_[REG_TYPE_PR]->updateReg((void*)data_ptr);
}

ErrorCode RegManager::movePrReg(int expect_id, int original_id)
{
    return reg_ptr_[REG_TYPE_PR]->moveReg(expect_id, original_id);
}

std::vector<BaseRegSummary> RegManager::getPrRegChangedList(int start_id, int size)
{
    return reg_ptr_[REG_TYPE_PR]->getChangedList(start_id, size);
}

std::vector<BaseRegSummary> RegManager::getPrRegValidList(int start_id, int size)
{
    return reg_ptr_[REG_TYPE_PR]->getValidList(start_id, size);
}

ErrorCode RegManager::addHrReg(HrRegData* data_ptr)
{
    return reg_ptr_[REG_TYPE_HR]->addReg((void*)data_ptr);
}

ErrorCode RegManager::deleteHrReg(int id)
{
    return reg_ptr_[REG_TYPE_HR]->deleteReg(id);
}

ErrorCode RegManager::getHrReg(int id, HrRegData* data_ptr)
{
    return reg_ptr_[REG_TYPE_HR]->getReg(id, (void*)data_ptr);
}

ErrorCode RegManager::updateHrReg(HrRegData* data_ptr)
{
    return reg_ptr_[REG_TYPE_HR]->updateReg((void*)data_ptr);
}

ErrorCode RegManager::moveHrReg(int expect_id, int original_id)
{
    return reg_ptr_[REG_TYPE_HR]->moveReg(expect_id, original_id);
}

std::vector<BaseRegSummary> RegManager::getHrRegChangedList(int start_id, int size)
{
    return reg_ptr_[REG_TYPE_HR]->getChangedList(start_id, size);
}

std::vector<BaseRegSummary> RegManager::getHrRegValidList(int start_id, int size)
{
    return reg_ptr_[REG_TYPE_HR]->getValidList(start_id, size);
}

ErrorCode RegManager::addMrReg(MrRegData* data_ptr)
{
    return reg_ptr_[REG_TYPE_MR]->addReg((void*)data_ptr);
}

ErrorCode RegManager::deleteMrReg(int id)
{
    return reg_ptr_[REG_TYPE_MR]->deleteReg(id);
}

ErrorCode RegManager::getMrReg(int id, MrRegData* data_ptr)
{
    return reg_ptr_[REG_TYPE_MR]->getReg(id, (void*)data_ptr);
}

ErrorCode RegManager::updateMrReg(MrRegData* data_ptr)
{
    return reg_ptr_[REG_TYPE_MR]->updateReg((void*)data_ptr);
}

ErrorCode RegManager::moveMrReg(int expect_id, int original_id)
{
    return reg_ptr_[REG_TYPE_MR]->moveReg(expect_id, original_id);
}

std::vector<BaseRegSummary> RegManager::getMrRegChangedList(int start_id, int size)
{
    return reg_ptr_[REG_TYPE_MR]->getChangedList(start_id, size);
}

std::vector<BaseRegSummary> RegManager::getMrRegValidList(int start_id, int size)
{
    return reg_ptr_[REG_TYPE_MR]->getValidList(start_id, size);
}

ErrorCode RegManager::addSrReg(SrRegData* data_ptr)
{
    return reg_ptr_[REG_TYPE_SR]->addReg((void*)data_ptr);
}

ErrorCode RegManager::deleteSrReg(int id)
{
    return reg_ptr_[REG_TYPE_SR]->deleteReg(id);
}

ErrorCode RegManager::getSrReg(int id, SrRegData* data_ptr)
{
    return reg_ptr_[REG_TYPE_SR]->getReg(id, (void*)data_ptr);
}

ErrorCode RegManager::updateSrReg(SrRegData* data_ptr)
{
    return reg_ptr_[REG_TYPE_SR]->updateReg((void*)data_ptr);
}

ErrorCode RegManager::moveSrReg(int expect_id, int original_id)
{
    return reg_ptr_[REG_TYPE_SR]->moveReg(expect_id, original_id);
}

std::vector<BaseRegSummary> RegManager::getSrRegChangedList(int start_id, int size)
{
    return reg_ptr_[REG_TYPE_SR]->getChangedList(start_id, size);
}

std::vector<BaseRegSummary> RegManager::getSrRegValidList(int start_id, int size)
{
    return reg_ptr_[REG_TYPE_SR]->getValidList(start_id, size);
}

ErrorCode RegManager::addRReg(RRegData* data_ptr)
{
    return reg_ptr_[REG_TYPE_R]->addReg((void*)data_ptr);
}

ErrorCode RegManager::deleteRReg(int id)
{
    return reg_ptr_[REG_TYPE_R]->deleteReg(id);
}

ErrorCode RegManager::getRReg(int id, RRegData* data_ptr)
{
    return reg_ptr_[REG_TYPE_R]->getReg(id, (void*)data_ptr);
}

ErrorCode RegManager::updateRReg(RRegData* data_ptr)
{
    return reg_ptr_[REG_TYPE_R]->updateReg((void*)data_ptr);
}

ErrorCode RegManager::moveRReg(int expect_id, int original_id)
{
    return reg_ptr_[REG_TYPE_R]->moveReg(expect_id, original_id);
}

std::vector<BaseRegSummary> RegManager::getRRegChangedList(int start_id, int size)
{
    return reg_ptr_[REG_TYPE_R]->getChangedList(start_id, size);
}

std::vector<BaseRegSummary> RegManager::getRRegValidList(int start_id, int size)
{
    return reg_ptr_[REG_TYPE_R]->getValidList(start_id, size);
}

bool RegManager::getPrRegValueById(int id, PrValue& pr_value)
{
    PrReg* reg_ptr = dynamic_cast<PrReg*>(reg_ptr_[REG_TYPE_PR]);
    return reg_ptr->getRegValueById(id, pr_value);
}

bool RegManager::getHrRegValueById(int id, HrValue& hr_value)
{
    HrReg* reg_ptr = dynamic_cast<HrReg*>(reg_ptr_[REG_TYPE_HR]);
    return reg_ptr->getRegValueById(id, hr_value);
}

bool RegManager::getMrRegValueById(int id, MrValue& mr_value)
{
    MrReg* reg_ptr = dynamic_cast<MrReg*>(reg_ptr_[REG_TYPE_MR]);
    return reg_ptr->getRegValueById(id, mr_value);
}

bool RegManager::getSrRegValueById(int id, SrValue& sr_value)
{
    SrReg* reg_ptr = dynamic_cast<SrReg*>(reg_ptr_[REG_TYPE_SR]);
    return reg_ptr->getRegValueById(id, sr_value);
}

bool RegManager::getRRegValueById(int id, RValue& r_value)
{
    RReg* reg_ptr = dynamic_cast<RReg*>(reg_ptr_[REG_TYPE_R]);
    return reg_ptr->getRegValueById(id, r_value);
}

bool RegManager::updatePrRegPos(PrRegDataIpc* data_ptr)
{
    PrReg* reg_ptr = dynamic_cast<PrReg*>(reg_ptr_[REG_TYPE_PR]);
    return reg_ptr->updateRegPos(data_ptr);
}

bool RegManager::updateHrRegJointPos(HrRegDataIpc* data_ptr)
{
    HrReg* reg_ptr = dynamic_cast<HrReg*>(reg_ptr_[REG_TYPE_HR]);
    return reg_ptr->updateRegJointPos(data_ptr);
}

bool RegManager::updateMrRegValue(MrRegDataIpc* data_ptr)
{
    MrReg* reg_ptr = dynamic_cast<MrReg*>(reg_ptr_[REG_TYPE_MR]);
    return reg_ptr->updateRegValue(data_ptr);
}

bool RegManager::updateSrRegValue(SrRegDataIpc* data_ptr)
{
    SrReg* reg_ptr = dynamic_cast<SrReg*>(reg_ptr_[REG_TYPE_SR]);
    return reg_ptr->updateRegValue(data_ptr);
}

bool RegManager::updateRRegValue(RRegDataIpc* data_ptr)
{
    RReg* reg_ptr = dynamic_cast<RReg*>(reg_ptr_[REG_TYPE_R]);
    return reg_ptr->updateRegValue(data_ptr);
}

bool RegManager::getPrRegPos(int id, PrRegDataIpc* data_ptr)
{
    PrReg* reg_ptr = dynamic_cast<PrReg*>(reg_ptr_[REG_TYPE_PR]);
    return reg_ptr->getRegPos(id, data_ptr);
}

bool RegManager::getHrRegJointPos(int id, HrRegDataIpc* data_ptr)
{
    HrReg* reg_ptr = dynamic_cast<HrReg*>(reg_ptr_[REG_TYPE_HR]);
    return reg_ptr->getRegJointPos(id, data_ptr);
}

bool RegManager::getMrRegValue(int id, MrRegDataIpc* data_ptr)
{
    MrReg* reg_ptr = dynamic_cast<MrReg*>(reg_ptr_[REG_TYPE_MR]);
    return reg_ptr->getRegValue(id, data_ptr);
}

bool RegManager::getSrRegValue(int id, SrRegDataIpc* data_ptr)
{
    SrReg* reg_ptr = dynamic_cast<SrReg*>(reg_ptr_[REG_TYPE_SR]);
    return reg_ptr->getRegValue(id, data_ptr);
}

bool RegManager::getRRegValue(int id, RRegDataIpc* data_ptr)
{
    RReg* reg_ptr = dynamic_cast<RReg*>(reg_ptr_[REG_TYPE_R]);
    return reg_ptr->getRegValue(id, data_ptr);
}

