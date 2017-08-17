#ifndef MIDDLEWARE_TO_MEM_STRUCT_SERVICE_RESPONSE_H_
#define MIDDLEWARE_TO_MEM_STRUCT_SERVICE_RESPONSE_H_

#define RES_BUFF_LEN 1024

typedef struct 
{
    int res_id;
    char res_buff[RES_BUFF_LEN];
}ServiceResponse;


#endif //MIDDLEWARE_TO_MEM_STRUCT_SERVICE_RESPONSE_H_
