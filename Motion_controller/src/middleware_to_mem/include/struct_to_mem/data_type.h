#ifndef _DATA_TYPE_H_
#define _DATA_TYPE_H_

#define MEM_NO_HANDSHAKE 0
#define MEM_HANDSHAKE 1

typedef struct
{ 
    unsigned int offset;
    unsigned int offset_flag;
    unsigned int size;
    unsigned int handshake;
    char *name;
}FunctionTable;

typedef struct
{
    unsigned int read;
    unsigned int write;
    unsigned int turn;
    unsigned int latest;
}AccessFlag;

#endif
