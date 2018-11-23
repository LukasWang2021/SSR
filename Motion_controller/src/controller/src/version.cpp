#include "stdio.h"
#include "version.h"
#include "controller_version.h"

//#define VER_MAJOR 2
//#define VER_MINOR 0
//#define VER_REVISION 5
//#define VER_BUILD_DATE "20181031"
//#define VER_BUILD_TIME "08:45:20"
//#define VERSION_ALL  "2.0.5.20181031"
  
char *get_version()
{
    char version_all[64];
    sprintf(version_all, "%d.%d.%d.%s.%s", controller_VERSION_MAJOR,
            controller_VERSION_MINOR, controller_VERSION_PATCH,
            controller_BUILD_DATE,controller_VERSION_COMMIT);
    printf("controller_version:%s\n", version_all);
    return version_all;
}  
  
char *get_build_date()
{
    return controller_BUILD_DATE;
    //return VER_BUILD_DATE;
}
  
char *get_build_time()
{
    return controller_BUILD_TIME;
    //return VER_BUILD_TIME;
}
  
int get_ver_major()
{
    return controller_VERSION_MAJOR;
    //return VER_MAJOR;
}
  
int get_ver_minor()
{
    return controller_VERSION_MINOR;
    //return VER_MINOR;
}
  
int get_ver_rev()
{
    return controller_VERSION_PATCH;
    //return VER_REVISION;
}
  
