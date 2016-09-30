#ifndef STRUCT_SERVICE_RESPONSE_H_
#define STRUCT_SERVICE_RESPONSE_H_

#define RES_BUFF_LEN 1024

typedef struct 
{
    int res_id;
    char res_buff[RES_BUFF_LEN];

}ServiceResponse;


#endif //STRUCT_SERVICE_RESPONSE_H_
