#include <stdio.h>
#include <tp_interface_version.h>

char ver_buf[128];

char *get_version()
{
    snprintf(ver_buf, sizeof(ver_buf), "%d.%d.%d", 
            tp_interface_VERSION_MAJOR, tp_interface_VERSION_MINOR, tp_interface_VERSION_PATCH);
    return ver_buf;
}  
  
int get_ver_major()
{
    return tp_interface_VERSION_MAJOR;
}
  
int get_ver_minor()
{
    return tp_interface_VERSION_MINOR;
}
  
int get_ver_rev()
{  
    return tp_interface_VERSION_PATCH;
}
  
