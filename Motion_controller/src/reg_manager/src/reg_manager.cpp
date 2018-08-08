#include "reg_manager.h"


using namespace fst_ctrl;


RegManager::RegManager():
    log_ptr_(NULL),
    param_ptr_(NULL)
{
    log_ptr_ = new fst_log::Logger();
    param_ptr_ = new RegManagerParam();
    for(int i=0; i < REG_TYPE_MAX; ++i)
    {
        reg_ptr_[i] = NULL;
    }
    FST_LOG_INIT("RegManager");
    FST_LOG_SET_LEVEL((fst_log::MessageLevel)param_ptr_->log_level_);
}

RegManager::~RegManager()
{

}

bool RegManager::init()
{
    if(!param_ptr_->loadParam())
    {
        FST_ERROR("Failed to load reg manager component parameters");
        return false;
    } 
    FST_LOG_SET_LEVEL((fst_log::MessageLevel)param_ptr_->log_level_);   

    reg_ptr_[REG_TYPE_PR] = new PrReg(param_ptr_);
    reg_ptr_[REG_TYPE_HR] = new HrReg(param_ptr_);
    reg_ptr_[REG_TYPE_SR] = new SrReg(param_ptr_);
    reg_ptr_[REG_TYPE_MR] = new MrReg(param_ptr_);
    reg_ptr_[REG_TYPE_R] = new RReg(param_ptr_);

    for(int i = 0; i < REG_TYPE_MAX; ++i)
    {
        if(reg_ptr_[i] == NULL
            || !reg_ptr_[i]->init())
        {
            return false;
        }
    }
    return true;    
}

bool RegManager::addPrReg(PrRegData* data_ptr)
{
    return reg_ptr_[REG_TYPE_PR]->addReg((void*)data_ptr);
}

bool RegManager::deletePrReg(int id)
{
    return reg_ptr_[REG_TYPE_PR]->deleteReg(id);
}

bool RegManager::getPrReg(int id, PrRegData* data_ptr)
{
    return reg_ptr_[REG_TYPE_PR]->getReg(id, (void*)data_ptr);
}

bool RegManager::setPrReg(PrRegData* data_ptr)
{
    return reg_ptr_[REG_TYPE_PR]->setReg((void*)data_ptr);
}

bool RegManager::movePrReg(int expect_id, int original_id)
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

bool RegManager::addHrReg(HrRegData* data_ptr)
{
    return reg_ptr_[REG_TYPE_HR]->addReg((void*)data_ptr);
}

bool RegManager::deleteHrReg(int id)
{
    return reg_ptr_[REG_TYPE_HR]->deleteReg(id);
}

bool RegManager::getHrReg(int id, HrRegData* data_ptr)
{
    return reg_ptr_[REG_TYPE_HR]->getReg(id, (void*)data_ptr);
}

bool RegManager::setHrReg(HrRegData* data_ptr)
{
    return reg_ptr_[REG_TYPE_HR]->setReg((void*)data_ptr);
}

bool RegManager::moveHrReg(int expect_id, int original_id)
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

bool RegManager::addMrReg(MrRegData* data_ptr)
{
    return reg_ptr_[REG_TYPE_MR]->addReg((void*)data_ptr);
}

bool RegManager::deleteMrReg(int id)
{
    return reg_ptr_[REG_TYPE_MR]->deleteReg(id);
}

bool RegManager::getMrReg(int id, MrRegData* data_ptr)
{
    return reg_ptr_[REG_TYPE_MR]->getReg(id, (void*)data_ptr);
}

bool RegManager::setMrReg(MrRegData* data_ptr)
{
    return reg_ptr_[REG_TYPE_MR]->setReg((void*)data_ptr);
}

bool RegManager::moveMrReg(int expect_id, int original_id)
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

bool RegManager::addSrReg(SrRegData* data_ptr)
{
    return reg_ptr_[REG_TYPE_SR]->addReg((void*)data_ptr);
}

bool RegManager::deleteSrReg(int id)
{
    return reg_ptr_[REG_TYPE_SR]->deleteReg(id);
}

bool RegManager::getSrReg(int id, SrRegData* data_ptr)
{
    return reg_ptr_[REG_TYPE_SR]->getReg(id, (void*)data_ptr);
}

bool RegManager::setSrReg(SrRegData* data_ptr)
{
    return reg_ptr_[REG_TYPE_SR]->setReg((void*)data_ptr);
}

bool RegManager::moveSrReg(int expect_id, int original_id)
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

bool RegManager::addRReg(RRegData* data_ptr)
{
    return reg_ptr_[REG_TYPE_R]->addReg((void*)data_ptr);
}

bool RegManager::deleteRReg(int id)
{
    return reg_ptr_[REG_TYPE_R]->deleteReg(id);
}

bool RegManager::getRReg(int id, RRegData* data_ptr)
{
    return reg_ptr_[REG_TYPE_R]->getReg(id, (void*)data_ptr);
}

bool RegManager::setRReg(RRegData* data_ptr)
{
    return reg_ptr_[REG_TYPE_R]->setReg((void*)data_ptr);
}

bool RegManager::moveRReg(int expect_id, int original_id)
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

void* RegManager::getPrRegValueById(int id)
{
    return reg_ptr_[REG_TYPE_PR]->getRegValueById(id);
}

void* RegManager::getHrRegValueById(int id)
{
    return reg_ptr_[REG_TYPE_HR]->getRegValueById(id);
}

void* RegManager::getMrRegValueById(int id)
{
    return reg_ptr_[REG_TYPE_MR]->getRegValueById(id);
}

void* RegManager::getSrRegValueById(int id)
{
    return reg_ptr_[REG_TYPE_SR]->getRegValueById(id);
}

void* RegManager::getRRegValueById(int id)
{
    return reg_ptr_[REG_TYPE_R]->getRegValueById(id);
}


