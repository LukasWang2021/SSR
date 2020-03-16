#include "anybus_manager.h"
#include <iostream>

using namespace fst_base;
using namespace fst_anybus;

#define ABIP_LINUX_USE_MII_ETH_IF   1
#define ABIP_LINUX_MII_ETH_IF       "eth1"

volatile unsigned char* pAbcc;
volatile unsigned char* pMemSys;

AnybusManager::AnybusManager():
    param_ptr_(NULL),
    log_ptr_(NULL)
{
    log_ptr_ = new fst_log::Logger();
    param_ptr_ = new AnybusManagerParam();
    FST_LOG_INIT("AnybusManager");
    FST_LOG_SET_LEVEL((fst_log::MessageLevel)param_ptr_->log_level_);

    is_running_ = false;
    abcc_handler_status_ = APPL_MODULE_NO_ERROR;
    safety_enable_ = false;
}

AnybusManager::~AnybusManager()
{
    this->closeServer();

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


unsigned long long int AnybusManager::initAnybus()
{
    // char* pc_args_info = "";
    if (safety_enable_)
    {
        char pc_args_info[] = "(T100 Safety enabled)";
        //SAFE_SetSafetyEnable( false );
        FST_INFO("%s %s\n", ABCC_STARTER_KIT_VER_STRING, pc_args_info);
    }
    else 
    {
        char pc_args_info[] = "";
        FST_INFO("%s %s\n", ABCC_STARTER_KIT_VER_STRING, pc_args_info);
    }

    int fd = open("/dev/mem", O_RDWR);
    if( fd < 1 )
    {
        FST_ERROR("Failed to open /dev/mem");
        unsigned long long int open_dev_mem_failed = 0x01;
        return open_dev_mem_failed;
    }

    unsigned int page_addr = (abcc_page_addr_ & ~(abcc_page_size_ - 1));
    void* p_mem;
    p_mem = mmap(NULL, abcc_page_size_, PROT_READ|PROT_WRITE, MAP_SHARED, fd, page_addr);

    if (p_mem == (void *)-1)
    {
        FST_ERROR("%s(%d)-%s: Failed to mmap /dev/mem .\n", __FILE__,__LINE__,__FUNCTION__);
        unsigned long long int open_dev_mem_failed = 0x01;
        return open_dev_mem_failed;
    }

    pAbcc = (unsigned char*)p_mem;

    //Get pointer to system controller
    int fd_sys = open("/dev/mem", O_RDWR);
    if (fd_sys < 1)
    {
        FST_ERROR("Failed to open /dev/mem for system controler");
        unsigned long long int open_dev_mem_failed = 0x01;
        return open_dev_mem_failed;
    }

    int size = 0x1000;
    unsigned int addr = 0xff300020;
    page_addr = (addr & ~(size - 1));

    void *pMemSys1 = mmap(NULL, size, PROT_READ|PROT_WRITE, MAP_SHARED, fd_sys, page_addr);

    if (pMemSys1 == (void *)-1)
    {
        FST_ERROR("%s(%d)-%s: Failed to mmap /dev/mem .\n", __FILE__,__LINE__,__FUNCTION__);
        unsigned long long int open_dev_mem_failed = 0x01;
        return open_dev_mem_failed;
    }
    pMemSys = (unsigned char*)pMemSys1;
    pMemSys = pMemSys + 0x20;

    if( ABCC_HwInit() != ABCC_EC_NO_ERROR )
    {
        FST_ERROR( "Failed init hardware");
        unsigned long long int init_hardware_failed = 0x01;
        return init_hardware_failed;
    }

    ABCC_CbfUserInitReq();

    FST_INFO("Success to init anybus manager");
    return SUCCESS;
}

unsigned long long int AnybusManager::init()
{
    unsigned long long int anybus_manager_load_param_failed = 0x01;
    if(!param_ptr_->loadParam()) return anybus_manager_load_param_failed;

    operation_mode_ = param_ptr_->operation_mode_;
    cycle_time_ = param_ptr_->cycle_time_;
    abcc_page_size_ = param_ptr_->abcc_page_size_;
    abcc_page_addr_ = param_ptr_->abcc_page_addr_;
    appl_page_size_ = param_ptr_->appl_page_size_;
    appl_page_addr_ = param_ptr_->appl_page_addr_;
    safety_enable_ = param_ptr_->safety_enable_;

    return SUCCESS;
}

unsigned long long int AnybusManager::openServer()
{
    is_running_ = true;

    if(!thread_ptr_.run(&anybusManagerRoutineThreadFunc, this, 50))
    {
        FST_ERROR("Failed to open AnybusManager");
        unsigned long long int anybus_manager_open_failed = 0x01;
        return anybus_manager_open_failed;
    }

    return SUCCESS;
}

void AnybusManager::closeServer()
{
    is_running_ = false;
    thread_ptr_.join();
}

void* AnybusManager::anybusManagerThreadFunc()
{
    // UINT8 test1 = *( (volatile UINT8 *)pAbcc + 0x3FF0);
    // FST_INFO("test1 = 0x%x\n", test1);
    abcc_handler_status_ = APPL_HandleAbcc();

    switch(abcc_handler_status_)
    {
        case APPL_MODULE_NO_ERROR:
            //FST_INFO("APPL_MODULE_NO_ERROR");
            break;
        case APPL_MODULE_RESET:
        {
            APPL_RestartAbcc();
            FST_INFO("APPL_MODULE_RESET");
        }
           break;
        default:
        {
            is_running_ = true;
            FST_INFO("abcc handler status is not exist");
        }
           break;
    }

    usleep(1000);
    ABCC_RunTimerSystem(1);

    return NULL;
}

bool AnybusManager::isRunning()
{
    return is_running_;
}

void* anybusManagerRoutineThreadFunc(void* arg)
{
    AnybusManager* anybus_manager = static_cast<AnybusManager*>(arg);
    while(anybus_manager->isRunning())
    {
        anybus_manager->anybusManagerThreadFunc();
    }

    return NULL;
}
