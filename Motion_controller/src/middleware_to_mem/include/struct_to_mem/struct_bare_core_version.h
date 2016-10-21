#ifndef MIDDLEWARE_TO_MEM_STRUCT_BARE_CORE_VERSION_H_
#define MIDDLEWARE_TO_MEM_STRUCT_BARE_CORE_VERSION_H_

typedef struct
{
    int year;
    int month;
    int day;
    int major; 
    int minor;
    int revision;
    char description[20];
    char servoVersion[40];
}BareCoreVersion;


#endif //MIDDLEWARE_TO_MEM_STRUCT_BARE_CORE_VERSION_H_

