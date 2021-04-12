#!/bin/bash  
  
# parameter check  
if [ $# -ne 3 ]
then
    echo "usage($#): $0 major_num minor_num revise_num"
exit
fi
# function  
version()  
{  
    BUILD_DATE=`date "+%Y%m%d"`
    BUILD_TIME=`date "+%R:%S"`
  
    cat > src/controller/src/version.cpp <<EEEEEEE
#include "stdio.h"
#include "version.h"

#define VER_MAJOR $1
#define VER_MINOR $2
#define VER_REVISION $3
  
#define VER_BUILD_DATE "$BUILD_DATE"  
#define VER_BUILD_TIME "$BUILD_TIME"    
  
#define VERSION_ALL  "$1.$2.$3.${BUILD_DATE}"  
  
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
  
EEEEEEE
    
    cat > src/controller/include/version.h <<EEEEEEE 
#ifndef CONTROLLER_VERSION_H_
#define CONTROLLER_VERSION_H_ 

char *get_version();  

char *get_build_date();

char *get_build_time();

int get_ver_major();

int get_ver_minor();

int get_ver_rev();

#endif /*CONTROLLER_VERSION_H_ */  
EEEEEEE
}

# print
echo "###############################"
echo "######### $0 "
echo "######### Major Number: $1"
echo "######### Minor Number: $2"
echo "######### Revise_Num: $3" 
  
version $1 $2 $3
  
if [ $? -eq 0 ]
then
    echo "######### Done! "  
else
    "######### Failed! "
fi
echo "###############################"
