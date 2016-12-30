/**
 * @file fst_error.h
 * @brief 
 * @author WangWei
 * @version 1.0.0
 * @date 2016-08-29
 */
#ifndef TP_INTERFACE_FST_ERROR_H_
#define TP_INTERFACE_FST_ERROR_H_

#define SET_MODE_FAILED                         (unsigned long long int)0x0001000200670001   /*set current mode failed*/
#define INVALID_ACTION_IN_CURRENT_MODE          (unsigned long long int)0x0001000200670002   /*cant action in current mode*/
#define SET_STATE_FAILED                        (unsigned long long int)0x0001000200670003   /*set current state failed*/
#define INVALID_ACTION_IN_CURRENT_STATE         (unsigned long long int)0x0001000200670004   /*cant action in current state*/
#define FIND_MOVE_INSTRUCTION_FAILED            (unsigned long long int)0x0001000200670005   /*when smoothing, can't find next move instruction*/
#define CONTINUE_MOVING_AS_PLAN_FAILED          (unsigned long long int)0x0001000200670006   /*can't continue moving as plan*/
#define PAUSE_AS_PLAN_FAILED                    (unsigned long long int)0x0001000400670001   /*can't pause motion as plan*/
#define READ_SHARE_MEMORY_TIMEOUT               (unsigned long long int)0x0001000600700001   /*read share memory timeout*/
#define WRITE_SHARE_MEMORY_TIMEOUT              (unsigned long long int)0x0001000600700002   /*write share memory timeout*/
#define ENCODE_MESSAGE_FAILED                   (unsigned long long int)0x0001000200830001   /*encode controller message failed*/
#define DECODE_MESSAGE_FAILED                   (unsigned long long int)0x0001000200830002   /*decode controller message failed*/
#define INVALID_PARAM_FROM_TP                   (unsigned long long int)0x0002000300830003   /*tp sent invalid parameters*/
#define READ_SHARE_MEMORY_FAILED                (unsigned long long int)0x0000000200700001   /*read share memory failed*/
#define WRITE_SHARE_MEMORY_FAILED               (unsigned long long int)0x0000000200700002   /*write share memory failed*/
#define WRONG_FIFO_STATE                        (unsigned long long int)0x0000000200670001   /*traj_len is not 0 but joints_len is 0 */
#define ENCODER_DATA_CHANGED                    (unsigned long long int)0x0000000200670002   /*data from encoder changed*/
#define CREATE_ARM_GROUP_FAILED                 (unsigned long long int)0x0001000200970001   /*create arm group failed*/
#define CREATE_PARAM_GROUP_FAILED               (unsigned long long int)0x0001000200970002   /*create_param_group_failed*/
#define PARAMETER_NOT_UPDATED                   (unsigned long long int)0x0000000200720001   /*the parameter hasn't updated yet*/
#define SERVO_ESTOP                             (unsigned long long int)0x0001000600700003   /*received estop signal from safety board*/







typedef unsigned long long int U64;
//#define INT64 ErrorCode


#define FST_SUCCESS				(0)



#endif
