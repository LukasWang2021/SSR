#include "controller.h"


using namespace fst_ctrl;


Controller* Controller::instance_ = NULL;

Controller::Controller():
    is_exit_(false)
{   
    FST_LOG_INIT("controller");
    FST_LOG_SET_LEVEL(param_.log_level_);
}


Controller::~Controller()
{

}

Controller* Controller::getInstance()
{
    if(instance_ == NULL)
    {
        instance_ = new Controller();
    }
    return instance_;
}

bool Controller::init()
{
    if(!param_.loadParam())
    {
        FST_ERROR("Failed to load controller component parameters");
        return false;
    }    
    FST_LOG_SET_LEVEL(param_.log_level_);    

    if(!tp_comm_.init()
        || !tp_comm_.open())
    {
        return false;
    }
    else
    {
        return true;
    }
}

bool Controller::isExit()
{
    return is_exit_;
}

