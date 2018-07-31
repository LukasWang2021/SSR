#include "controller.h"
#include <unistd.h>

using namespace fst_ctrl;

int main(int argc, char **argv)
{
    Controller* controller_ptr = Controller::getInstance();
    if(controller_ptr != NULL)
    {
        if(!controller_ptr->init())
        {
            return -1;
        }
        else
        {
            while(!controller_ptr->isExit())
            {
                usleep(100);
            }
        }
    }

    return 0;
}

