#ifndef SERVO_DATA_COMM_STRUCT_H_
#define SERVO_DATA_COMM_STRUCT_H_

#define  PACKAGE_SIZE 12500
#define  SNAPSHOT_SIZE 50000
#define  RECORD_SIZE 64
#define  SERVO_DATA_SEG_LENGTH 512
#define  SERVO_CMD_SEG_LENGTH 1024
typedef enum
{
    TYPE_UINT8 = 0,
    TYPE_INT8 = 1,
    TYPE_UINT16 = 2,
    TYPE_INT16 = 3,
    TYPE_UINT32 = 4,
    TYPE_INT32 = 5,
    TYPE_FLOAT32 = 6,
    TYPE_FLOAT64 = 7,
    TYPE_INT64 = 8,
}E_DATA_TYPE_t;

typedef struct 
{
    int64_t flag;//for 8 bytes align
    char data[RECORD_SIZE];
}Servo_Data_Record_t;

typedef struct 
{
    int length;
    int seq;
    Servo_Data_Record_t record[PACKAGE_SIZE];
}Servo_Data_Package_t;

typedef struct 
{
    int64_t id;//for 8 bytes align
    char data[SERVO_CMD_SEG_LENGTH];
}ServoCommand_Pkg_t;


#endif //SERVO_DATA_COMM_STRUCT_H_
