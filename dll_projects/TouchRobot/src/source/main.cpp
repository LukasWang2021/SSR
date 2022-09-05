#include <stdint.h>
#include <stdio.h>
#include <Windows.h>
#include "touch_interface.h"

int err_info(void* usr, void* sys)
{
    printf("%s\n", (char*)sys);
    return 0;
}

int main(int argc, char **argv)
{
    int ret = 0;
    double euler[6];
    TouchCallbackRegister(TOUCH_EVENT_ERROR_EXIST, err_info, NULL);

    ret = TouchEnable();

    if (ret < 0)
    {
        printf("Touch Enable Failed!\n");
    }
    else {
        printf("Touch Enable Succeed!\n");
    }
    TouchForceFeedCtrl(1);
    while (1)
    {
        Sleep(1000);
        //if (TouchXyzAbcPull(euler, 6 * sizeof(double)) < 0)
        //    printf("err: TouchXyzAbcPull!\n");
        //else
        //    printf("xyzabc: %lf %lf %lf %lf %lf %lf\n", euler[0], euler[1], euler[2], euler[3], euler[4], euler[5]);
    }

    return 0;
}
