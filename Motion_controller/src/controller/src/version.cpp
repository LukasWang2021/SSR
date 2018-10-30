#include "stdio.h"
#include "version.h"

#define VER_MAJOR 2
#define VER_MINOR 0
#define VER_REVISION 5
  
#define VER_BUILD_DATE "20181030"  
#define VER_BUILD_TIME "09:49:45"    
  
#define VERSION_ALL  "2.0.5.20181030"  
  
char *get_version()
{
    return VERSION_ALL;
}  
  
char *get_build_date()
{
    return VER_BUILD_DATE;
}
  
char *get_build_time()
{
    return VER_BUILD_TIME;
}
  
int get_ver_major()
{
    return VER_MAJOR;
}
  
int get_ver_minor()
{
    return VER_MINOR;
}
  
int get_ver_rev()
{  
    return VER_REVISION;
}
  
