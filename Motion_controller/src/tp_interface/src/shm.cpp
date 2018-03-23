#include <map>
#include <string>
#include <string.h>
#include <unistd.h> 
#include <sys/types.h>
#include <fcntl.h>
#include <sys/mman.h>
#include "shm.h"

using namespace std;

#define MEM_FALSE 0
#define MEM_TRUE 1
#define MEM_WRITE_TURN 11
#define MEM_READ_TURN 12
#define MEM_READ_ALREADY 0
#define MEM_WRITE_ALREADY 1

map<string, ShmData> g_mapper;
FuncTable table;

int createShm(const char* name, int size)
{
    int fd = shm_open(name,  O_CREAT|O_RDWR, 00777);
    if (fd == -1)
    {
        perror("failed on creating sharedmem\n");
        close(fd);
        return -1;
    }
    ShmData data;
    ftruncate(fd, size);
    data.ptr = mmap(NULL, size, PROT_READ|PROT_WRITE, MAP_SHARED, fd, 0);
    if (data.ptr == MAP_FAILED) 
    {
        close(fd);
        perror("failed on mapping process sharedmem\n");
        return -1;
    }
 
    memset(data.ptr, 0, size);
    g_mapper.insert(map<string, ShmData>::value_type(name, data));

    return 0;
}

int openShm(const char* name, int size)
{
    int fd = shm_open(name,  O_RDWR, 00777);
    if (fd == -1)
    {
        perror("failed on creating sharedmem\n");
        close(fd);
        return -1;
    }
    ShmData data;
    data.ptr = mmap(NULL, size, PROT_READ|PROT_WRITE, MAP_SHARED, fd, 0);
    if (data.ptr == MAP_FAILED) 
    {
        close(fd);
        perror("failed on mapping process sharedmem\n");
        return -1;
    }

    g_mapper.insert(map<string, ShmData>::value_type(name, data));

    return 0;   
}

ShmData *getShm(const char* name)
{
    map<string, ShmData>::iterator it = g_mapper.find(name);
    if (it == g_mapper.end())
        return NULL;

    ShmData *data = &it->second;
    return data;
}

void readShm(const char* name, int offset,  void*buffer, int size)
{
    void* ptr = getShm(name)->ptr;

    memcpy(buffer, (char*)ptr+offset, size);
}


void writeShm(const char* name, int offset, void*buffer, int size)
{
    void* ptr = getShm(name)->ptr;
    memcpy((char*)ptr+offset, buffer, size); 
}

bool tryWrite(const char* name, int offset, void*buffer, int size)
{
    ShmData *data = getShm(name);
    if (data == NULL)
    {
        printf("can't find name:%s\n", name);
        return false;
    }
    volatile unsigned int *ptr_read, *ptr_write, *ptr_turn, *ptr_latest;
    AcceFlag *flag = (AcceFlag *)&(((FuncTable*)data->ptr)->offset_flag);
    ptr_read = &(flag->read);
    ptr_write = &(flag->write);
    ptr_turn = &(flag->turn);
    ptr_latest = &(flag->latest);

  //  printf("read:%d, write:%d, turn:%d, latest:%d\n", flag->read, flag->write, flag->turn, flag->latest);

    if (*(ptr_latest) == MEM_WRITE_ALREADY){return false;}

    if (*(ptr_read) == MEM_FALSE && *(ptr_write)  == MEM_FALSE)
    {
        *(ptr_write) = MEM_TRUE;  //Indicating that it is going to be the writing state
        *(ptr_turn) = MEM_WRITE_TURN; //mark that which process is using the memory
        if (*(ptr_read) == MEM_TRUE &&  *(ptr_turn) != MEM_WRITE_TURN )
        {
            *(ptr_write) = MEM_FALSE; 
            return false;
        } //check if writing process is working on.
        memcpy(((FuncTable*)data->ptr)->param + offset, buffer, size);
        *(ptr_latest) = MEM_WRITE_ALREADY;
        *(ptr_write) = MEM_FALSE;       
    } 
    else
    {
        return false;
    }


    return true;
}


bool lockRead(const char* name, int offset, void*buffer, int size)
{
    ShmData *data = getShm(name);
    if (data == NULL)
        return false;
    volatile unsigned int *ptr_read, *ptr_write, *ptr_turn, *ptr_latest;
    AcceFlag *flag = (AcceFlag *)&(((FuncTable*)data->ptr)->offset_flag);
    ptr_read = &(flag->read);
    ptr_write = &(flag->write);
    ptr_turn = &(flag->turn);
    ptr_latest = &(flag->latest);
   // printf("read:%d, write:%d, turn:%d, latest:%d\n", flag->read, flag->write, flag->turn, flag->latest);

    if (*(ptr_latest) == MEM_READ_ALREADY){return false;}

    if (*(ptr_read) == MEM_FALSE && *(ptr_write)  ==  MEM_FALSE)
    {
        *(ptr_read) = MEM_TRUE;  //Indicating that it is going to be the reading state
        *(ptr_turn) = MEM_READ_TURN; //mark that which process is using the memory
        if (*(ptr_write) == MEM_TRUE &&  *(ptr_turn) != MEM_READ_TURN )
        {
            *(ptr_read) = MEM_FALSE; 
            return false;
        } //check if writing process is working on.
        memcpy(buffer, ((FuncTable*)data->ptr)->param + offset, size);
        *(ptr_latest) = MEM_READ_ALREADY;
       // *(ptr_read) = MEM_FALSE;
    }

    return true; 
}

bool unlockRead(const char* name)
{
    ShmData *data = getShm(name);
    if (data == NULL)
        return false;
    volatile unsigned int *ptr_read, *ptr_write, *ptr_turn, *ptr_latest;
    AcceFlag *flag = (AcceFlag *)&(((FuncTable*)data->ptr)->offset_flag);
    if (flag->read == MEM_TRUE)
        flag->read = MEM_FALSE;
    return true;
}

bool tryRead(const char* name, int offset, void*buffer, int size)
{
    ShmData *data = getShm(name);
    if (data == NULL)
        return false;
    volatile unsigned int *ptr_read, *ptr_write, *ptr_turn, *ptr_latest;
    AcceFlag *flag = (AcceFlag *)&(((FuncTable*)data->ptr)->offset_flag);
    ptr_read = &(flag->read);
    ptr_write = &(flag->write);
    ptr_turn = &(flag->turn);
    ptr_latest = &(flag->latest);
  //  printf("read:%d, write:%d, turn:%d, latest:%d\n", flag->read, flag->write, flag->turn, flag->latest);

    if (*(ptr_latest) == MEM_READ_ALREADY){return false;}

    if (*(ptr_read) == MEM_FALSE && *(ptr_write)  ==  MEM_FALSE)
    {
        *(ptr_read) = MEM_TRUE;  //Indicating that it is going to be the reading state
        *(ptr_turn) = MEM_READ_TURN; //mark that which process is using the memory
        if (*(ptr_write) == MEM_TRUE &&  *(ptr_turn) != MEM_READ_TURN )
        {
            *(ptr_read) = MEM_FALSE; 
            return false;
        } //check if writing process is working on.
        memcpy(buffer, ((FuncTable*)data->ptr)->param + offset, size);
        *(ptr_latest) = MEM_READ_ALREADY;
        *(ptr_read) = MEM_FALSE;
    }

    return true;    
}

bool isEmpty(const char* name)
{
    ShmData *data = getShm(name);
}

