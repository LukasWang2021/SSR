#include "reg_manager.h"
#include "error_code.h"
#include <unistd.h>

using namespace fst_ctrl;


RegManager::RegManager():
    log_ptr_(NULL),
    param_ptr_(NULL),
    nvram_obj_(NVRAM_AREA_LENGTH)
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
    if(log_ptr_ != NULL)
    {
        delete log_ptr_;
        log_ptr_ = NULL;
    }
    if(param_ptr_ != NULL)
    {
        delete param_ptr_;
        param_ptr_ = NULL;
    }
}

void RegManager::initNVRam()
{
	char mr_buf[sizeof(NVRamMrRegData)];
	char r_buf[sizeof(NVRamRRegData)];
	char pr_buf[sizeof(NVRamPrRegData)];
	
	char * pWriteAddr = NVRAM_HEAD ;
	unsigned int uiWrite = NVRAM_Magic_NUM;
	
    FST_INFO("RegManager::initNVRam write = %08X at %08X", uiWrite, pWriteAddr);
	nvram_obj_.writeSync((uint8_t*)&uiWrite, pWriteAddr, sizeof(unsigned int));
	usleep(30000);
	uiWrite = 0 ;
    FST_INFO("RegManager::initNVRam reset to = %08X", uiWrite);
	nvram_obj_.read((uint8_t*)&uiWrite, NVRAM_HEAD, sizeof(unsigned int));
	if(uiWrite != NVRAM_Magic_NUM)
	{
		FST_INFO("RegManager::initNVRam read NG = %08X", uiWrite);
		return ;
	}
	
	uiWrite = NVRAM_VERSION;
	pWriteAddr += sizeof(unsigned int);
	nvram_obj_.write((uint8_t*)&uiWrite, pWriteAddr, sizeof(unsigned int));
	usleep(30000);

    FST_INFO("RegManager::initNVRam Starting 30% ...... ");
	memset(mr_buf, 0x00, sizeof(NVRamMrRegData));
	for(int i =0; i < NVRAM_MR_NUM; i++)
	{
		nvram_obj_.writeSync((uint8_t*)mr_buf, pWriteAddr, sizeof(NVRamMrRegData));
		usleep(10000);
		pWriteAddr += sizeof(NVRamMrRegData);
	}
	
    FST_INFO("RegManager::initNVRam Starting 60% ...... ");
	memset(r_buf, 0x00, sizeof(NVRamRRegData));
	for(int i =0; i < NVRAM_R_NUM; i++)
	{
		nvram_obj_.writeSync((uint8_t*)r_buf, pWriteAddr, sizeof(NVRamRRegData));
		usleep(10000);
		pWriteAddr += sizeof(NVRamRRegData);
	}
	
    FST_INFO("RegManager::initNVRam Starting 100% ...... ");
//  These code would crash the data at NVRAM_HEAD
//	memset(pr_buf, 0x00, sizeof(NVRamPrRegData));
//	for(int i =0; i < NVRAM_PR_NUM; i++)
//	{
//		nvram_obj_.writeSync((uint8_t*)pr_buf, pWriteAddr, sizeof(NVRamPrRegData));
//		usleep(10000);
//		pWriteAddr += sizeof(NVRamPrRegData);
//	}
//    FST_INFO("RegManager::initNVRam Starting 100% ...... ");
	
	uiWrite = 0 ;
    FST_INFO("RegManager::initNVRam NVRamPrRegData reset to = %08X", uiWrite);
	nvram_obj_.read((uint8_t*)&uiWrite, NVRAM_HEAD, sizeof(unsigned int));
    FST_INFO("RegManager::initNVRam read = %08X at %08X", uiWrite, pWriteAddr);
}

ErrorCode RegManager::init()
{
	unsigned int uiMagic ;
    if(!param_ptr_->loadParam())
    {
        FST_ERROR("Failed to load reg manager component parameters");
        return REG_MANAGER_LOAD_PARAM_FAILED;
    } 
    FST_LOG_SET_LEVEL((fst_log::MessageLevel)param_ptr_->log_level_);   

    use_nvram_ = param_ptr_->use_nvram_;
	FST_INFO("RegManager::init use_nvram_ = %d", use_nvram_);
	if(use_nvram_ == REG_USE_NVRAM)
	{
		ErrCode error = nvram_obj_.openNvram();
		FST_INFO("RegManager::init nvram_obj_.openNvram = %08X", error);
		if(error == FST_NVRAM_OK)
		{
			error = nvram_obj_.isNvramReady();
			if(error == FST_NVRAM_OK)
			{
				char * pReadAddr = NVRAM_HEAD ;
				ErrCode error = nvram_obj_.read(
					(uint8_t*)&uiMagic, pReadAddr, sizeof(unsigned int));
			    FST_INFO("RegManager::init nvram_obj_.read = %08X at %08X return %08X",
					uiMagic, pReadAddr, error);
				if(NVRAM_Magic_NUM != uiMagic)
				{
					initNVRam();
			    	FST_INFO("RegManager::init initNVRam = %08X", uiMagic);
			    //    return REG_MANAGER_LOAD_NVRAM_FAILED;
				}
			}
		}
	}
	FST_INFO("RegManager::init param_ptr_->use_nvram_ = %d", param_ptr_->use_nvram_);

    reg_ptr_[REG_TYPE_PR] = new PrReg(param_ptr_);
    reg_ptr_[REG_TYPE_HR] = new HrReg(param_ptr_);
    reg_ptr_[REG_TYPE_SR] = new SrReg(param_ptr_);
    reg_ptr_[REG_TYPE_MR] = new MrReg(param_ptr_);
    reg_ptr_[REG_TYPE_R] = new RReg(param_ptr_);

    ErrorCode error_code;
    for(int i = 0; i < REG_TYPE_MAX; ++i)
    {
        if(reg_ptr_[i] != NULL)
        {
            error_code = reg_ptr_[i]->init();
            if(error_code != SUCCESS)
            {
 				FST_INFO("RegManager::init[%d] error_code = %08X", i, error_code);
                return error_code;
            }
        }
        else
        {
			FST_INFO("RegManager::init REG_MANAGER_INIT_OBJECT_FAILED");
            return REG_MANAGER_INIT_OBJECT_FAILED;
        }
    }
    return SUCCESS;
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

bool RegManager::updatePrRegPos(PrRegDataIpc* data_ptr)
{
    PrReg* reg_ptr = static_cast<PrReg*>(reg_ptr_[REG_TYPE_PR]);
    return reg_ptr->updateRegPos(data_ptr);
}

bool RegManager::updateHrRegJointPos(HrRegDataIpc* data_ptr)
{
    HrReg* reg_ptr = static_cast<HrReg*>(reg_ptr_[REG_TYPE_HR]);
    return reg_ptr->updateRegJointPos(data_ptr);
}

bool RegManager::updateMrRegValue(MrRegDataIpc* data_ptr)
{
    MrReg* reg_ptr = static_cast<MrReg*>(reg_ptr_[REG_TYPE_MR]);
    return reg_ptr->updateRegValue(data_ptr);
}

bool RegManager::updateSrRegValue(SrRegDataIpc* data_ptr)
{
    SrReg* reg_ptr = static_cast<SrReg*>(reg_ptr_[REG_TYPE_SR]);
    return reg_ptr->updateRegValue(data_ptr);
}

bool RegManager::updateRRegValue(RRegDataIpc* data_ptr)
{
    RReg* reg_ptr = static_cast<RReg*>(reg_ptr_[REG_TYPE_R]);
    return reg_ptr->updateRegValue(data_ptr);
}

bool RegManager::getPrRegPos(int id, PrRegDataIpc* data_ptr)
{
    PrReg* reg_ptr = static_cast<PrReg*>(reg_ptr_[REG_TYPE_PR]);
    return reg_ptr->getRegPos(id, data_ptr);
}

bool RegManager::getHrRegJointPos(int id, HrRegDataIpc* data_ptr)
{
    HrReg* reg_ptr = static_cast<HrReg*>(reg_ptr_[REG_TYPE_HR]);
    return reg_ptr->getRegJointPos(id, data_ptr);
}

bool RegManager::getMrRegValue(int id, MrRegDataIpc* data_ptr)
{
    MrReg* reg_ptr = static_cast<MrReg*>(reg_ptr_[REG_TYPE_MR]);
    return reg_ptr->getRegValue(id, data_ptr);
}

bool RegManager::getSrRegValue(int id, SrRegDataIpc* data_ptr)
{
    SrReg* reg_ptr = static_cast<SrReg*>(reg_ptr_[REG_TYPE_SR]);
    return reg_ptr->getRegValue(id, data_ptr);
}

bool RegManager::getRRegValue(int id, RRegDataIpc* data_ptr)
{
    RReg* reg_ptr = static_cast<RReg*>(reg_ptr_[REG_TYPE_R]);
    return reg_ptr->getRegValue(id, data_ptr);
}

