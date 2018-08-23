#include "controller.h"
#include <unistd.h>
#include <iostream>
#include "error_code.h"

using namespace std;
using namespace fst_ctrl;

int main(int argc, char **argv)
{
    Controller* controller_ptr = Controller::getInstance();
    if(controller_ptr != NULL)
    {
        if(controller_ptr->init() != SUCCESS)
        {
            std::cout<<"failed to init controller"<<std::endl;
            return -1;
        }
        else
        {
            while(!controller_ptr->isExit())
            {
                usleep(500000);
            }
        }
    }

    return 0;
}

