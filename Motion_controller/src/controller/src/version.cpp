#include "stdio.h"
#include "version.h"
#include "controller_version.h"
#include <string>
#include <string.h>

#define MAIN_VERSION 1

char *get_version()
{
    static char version_all[64] = "";

#ifndef MAIN_VERSION
    sprintf(version_all, "%d.%d.%d.%s.%s", controller_VERSION_MAJOR,
            controller_VERSION_MINOR, controller_VERSION_PATCH,
            controller_BUILD_DATE,controller_VERSION_COMMIT);

#else 
    sprintf(version_all, "%d.%d", controller_VERSION_MAJOR,controller_VERSION_MINOR);

#endif

    printf("controller_version:%s\n", version_all);
    return version_all;
}  
  
char *get_build_date()
{
    return controller_BUILD_DATE;
}
  
char *get_build_time()
{
    return controller_BUILD_TIME;
}
  
int get_ver_major()
{
    return controller_VERSION_MAJOR;
}
  
int get_ver_minor()
{
    return controller_VERSION_MINOR;
}
  
int get_ver_rev()
{
    return controller_VERSION_PATCH;
}
  
