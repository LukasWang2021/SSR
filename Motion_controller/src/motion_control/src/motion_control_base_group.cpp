/*************************************************************************
	> File Name: motion_control_base_group.cpp
	> Author: 
	> Mail: 
	> Created Time: 2018年08月07日 星期二 11时25分56秒
 ************************************************************************/

#include <unistd.h>
#include <string.h>

#include <motion_control_base_group.h>

//////////////////// FIXME //////////////////
#define BARE_CORE_TIMEOUT 0x6666666600000066
/////////////////////////////////////////////

namespace fst_mc
{

static const size_t MAX_ATTEMPTS = 100;


BaseGroup::BaseGroup(fst_log::Logger* plog)
{
    group_state_ = STANDBY;
    log_ptr_ = plog;
    auto_cache_ = NULL;
    manual_cache_ = NULL;
    auto_time_ = 0;
    manual_time_ = 0;
    pthread_mutex_init(&auto_mutex_, NULL);
    pthread_mutex_init(&manual_mutex_, NULL);
}

BaseGroup::~BaseGroup()
{}

ErrorCode BaseGroup::resetGroup(void)
{
    return bare_core_.resetBareCore() == true ? SUCCESS : BARE_CORE_TIMEOUT;
}

ErrorCode BaseGroup::stopGroup(void)
{
    return bare_core_.stopBareCore() == true ? SUCCESS : BARE_CORE_TIMEOUT;
}




}

