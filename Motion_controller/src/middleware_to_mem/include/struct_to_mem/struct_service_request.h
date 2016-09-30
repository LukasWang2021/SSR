#ifndef STRUCT_SERVICE_REQUEST_H_
#define STRUCT_SERVICE_REQUEST_H_

#define REQ_BUFF_LEN 1024

typedef struct 
{
    int req_id;
    char req_buff[REQ_BUFF_LEN];
}ServiceRequest;


#endif //STRUCT_SERVICE_REQUEST_H_
