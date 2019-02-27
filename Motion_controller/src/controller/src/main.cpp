#include "controller.h"
#include <unistd.h>
#include <iostream>
#include <signal.h>
#include "error_code.h"
//#include "base_datatype.h"

using namespace std;
using namespace fst_ctrl;

Controller* g_controller_ptr_ = NULL;

void onExit(int dunno)
{
    std::cout<<"on exiting controller process"<<std::endl;
    g_controller_ptr_->setExit();
}

int main(int argc, char **argv)
{
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


