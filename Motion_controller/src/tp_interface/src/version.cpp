#include "stdio.h"
#include "version.h"

#define VER_MAJOR 1
#define VER_MINOR 2
#define VER_REVISION 3
  
#define VER_BUILD_DATE "20170120"  
#define VER_BUILD_TIME "20:29:42"    
  
#define VERSION_ALL  "1.2.3.20170120"  
  
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
  
