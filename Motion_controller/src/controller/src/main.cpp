#include "controller.h"
#include <unistd.h>
#include <iostream>
#include "error_code.h"
#include "base_datatype.h"

using namespace std;
using namespace fst_ctrl;

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
            while(!controller_ptr->isExit())
            {
                usleep(500000);
            }
        }
    }

    return 0;
}

