#include "controller.h"
#include <unistd.h>
#include <iostream>
#include <signal.h>
#include "common_error_code.h"
#include "init_protector.h"

using namespace std;
using namespace user_space;

#define CONTROLLER_PROCESS_NAME "CONTROLLER"

Controller* g_controller_ptr_ = NULL;

void onExit(int dunno)
{
    std::cout<<"on exiting controller process"<<std::endl;
    g_controller_ptr_->setExit();
    user_space::init_clean();
}

int main(int argc, char **argv)
{
    // init protection
    if(!user_space::init_protect(CONTROLLER_PROCESS_NAME))
    {
        cout<<endl<<"INIT_PROTECTOR -> ERROR: "<<CONTROLLER_PROCESS_NAME<<" initialization failed"<<endl;
        return -1;
    }


    Controller* controller_ptr = Controller::getInstance();
    if(controller_ptr != NULL)
    {
        ErrorCode error_code = controller_ptr->init();
        if(error_code != SUCCESS)
        {
            std::cout<<"failed to init controller: "<<std::hex<<error_code<<std::endl;
            return -1;
        }
        else
        {
            g_controller_ptr_ = controller_ptr;
            signal(SIGINT, onExit);
            signal(SIGTERM, onExit);  
            signal(SIGHUP, onExit); 
            signal(SIGQUIT, onExit); 
            while(!controller_ptr->isExit())
            {
                usleep(500000);
            }
        }
    }
    delete controller_ptr;
    std::cout<<"controller exit"<<std::endl;
    return 0;
}


