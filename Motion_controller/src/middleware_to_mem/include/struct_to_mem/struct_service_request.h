#ifndef MIDDLEWARE_TO_MEM_STRUCT_SERVICE_REQUEST_H_
#define MIDDLEWARE_TO_MEM_STRUCT_SERVICE_REQUEST_H_

#define REQ_BUFF_LEN 1024

typedef struct 
{
    int req_id;
    char req_buff[REQ_BUFF_LEN];
}ServiceRequest;


#endif //MIDDLEWARE_TO_MEM_STRUCT_SERVICE_REQUEST_H_
