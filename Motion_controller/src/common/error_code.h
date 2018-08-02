#ifndef COMMON_ERROR_CODE_H_
#define COMMON_ERROR_CODE_H_

#define INIT_CONTROLLER_FAILED                  (unsigned long long int)0x0011000200AB0001   /*initialization controller failed*/
#define WRITE_SERVICE_TIMETOUT                  (unsigned long long int)0x00010006006F0001   /*write service timeout*/
#define READ_SERVICE_TIMEOUT                    (unsigned long long int)0x00010006006F0002   /*read service timeout*/
#define SERVICE_DISABLE_FAILED                  (unsigned long long int)0x00010006006F0003   /*write disable service to core1 failed*/
#define SERVICE_RESET_FAILED                    (unsigned long long int)0x00010006006F0004   /*write reset service to core1 failed*/

#define SET_MODE_FAILED                         (unsigned long long int)0x0001000200670001   /*set current mode failed*/
#define INVALID_ACTION_IN_CURRENT_MODE          (unsigned long long int)0x0001000200670002   /*cant action in current mode*/
#define SET_STATE_FAILED                        (unsigned long long int)0x0001000200670003   /*set current state failed*/
#define INVALID_ACTION_IN_CURRENT_STATE         (unsigned long long int)0x0001000200670004   /*cant action in current state*/
#define FIND_MOVE_INSTRUCTION_FAILED            (unsigned long long int)0x0001000200670005   /*when smoothing, can't find next move instruction*/
#define CONTINUE_MOVING_AS_PLAN_FAILED          (unsigned long long int)0x0001000200670006   /*can't continue moving as plan*/
#define INVALID_ACTION_IN_LIMITED_STATE         (unsigned long long int)0x0001000200670007   /*cant caction in limited running state*/
#define PAUSE_AS_PLAN_FAILED                    (unsigned long long int)0x0001000600670008   /*error accurred during suspending*/
#define PARSE_IO_PATH_FAILED                    (unsigned long long int)0x0001000400670009   /*cant use current path to set IO*/

#define READ_SHARE_MEMORY_TIMEOUT               (unsigned long long int)0x0001000600700001   /*read share memory timeout*/
#define WRITE_SHARE_MEMORY_TIMEOUT              (unsigned long long int)0x0001000600700002   /*write share memory timeout*/
#define FAILED_TO_SET_TEMP_ZERO                 (unsigned long long int)0x000200020070000A   /*failed to set temp zero*/

#define ENCODE_MESSAGE_FAILED                   (unsigned long long int)0x0001000200830001   /*encode controller message failed*/
#define DECODE_MESSAGE_FAILED                   (unsigned long long int)0x0001000200830002   /*decode controller message failed*/
#define INVALID_PARAM_FROM_TP                   (unsigned long long int)0x0001000200830003   /*tp sent invalid parameters*/
#define INVALID_PATH_FROM_TP                    (unsigned long long int)0x0001000200830004   /*tp sent invalid path*/
#define INVALID_ID_FROM_TP                      (unsigned long long int)0x0001000200830005   /*tp sent invalid id*/
#define READ_SHARE_MEMORY_FAILED                (unsigned long long int)0x0000000200700001   /*read share memory failed*/
#define WRITE_SHARE_MEMORY_FAILED               (unsigned long long int)0x0000000200700002   /*write share memory failed*/
#define WRONG_FIFO_STATE                        (unsigned long long int)0x0000000200670001   /*traj_len is not 0 but joints_len is 0 */
#define ENCODER_DATA_CHANGED                    (unsigned long long int)0x0000000200670002   /*data from encoder changed, this is not an error*/
#define CREATE_ARM_GROUP_FAILED                 (unsigned long long int)0x0001000200970001   /*create arm group failed*/
#define CREATE_PARAM_GROUP_FAILED               (unsigned long long int)0x0001000200970002   /*create_param_group_failed*/
#define PARAMETER_NOT_UPDATED                   (unsigned long long int)0x0000000200720001   /*the parameter hasn't updated yet*/
#define SERVO_ESTOP                             (unsigned long long int)0x0001000600700003   /*received estop signal from safety board*/

#define ZERO_OFFSET_LOST                        (unsigned long long int)0x00010004006607D2   /*one or more axis lost its zero offset*/
#define ZERO_OFFSET_DEVIATE                     (unsigned long long int)0x00010004006607D3   /*axis zero offset deviated*/
#define NEED_CALIBRATION                        (unsigned long long int)0x0001000400660412   /*ArmGroup need to calibrate*/

#define FAIL_GET_REGISTER_TYPE                  (unsigned long long int)0x00010002006707EF   /*fail get register type*/
#define FAIL_GET_REGISTER_ID                    (unsigned long long int)0x00010002006707F0   /*fail get register id*/
#define FAIL_SET_REGISTER_TYPE                  (unsigned long long int)0x00010002006707F1   /*fail set register type*/
#define FAIL_SET_REGISTER_ID                    (unsigned long long int)0x00010002006707F2   /*fail set register id*/

#define FALT_GET_FRAME                          (unsigned long long int)0x00010002006707F9   /*fail get frame*/
#define FALT_UPDATE_FRAME                       (unsigned long long int)0x00010002006707FA   /*fail update frame*/
#define FALT_DELETE_FRAME                       (unsigned long long int)0x00010002006707FB   /*fail delete frame*/
#define FALT_ADD_FRAME                          (unsigned long long int)0x00010002006707FC   /*fail_add_frame*/
#define FALT_ACTIVATE_FRAME                     (unsigned long long int)0x00010002006707FD   /*fail_activate_frame*/
#define FALT_INIT_USER_FRAME                    (unsigned long long int)0x00110001006707FE   /*fail_init_user_frame*/
#define FALT_INIT_TOOL_FRAME                    (unsigned long long int)0x00110001006707FF   /*fail_init_tool_frame*/
#define FALT_SET_FRAME                          (unsigned long long int)0x0001000200670800   /*fail_set_frame*/

#define FAIL_INTERPRETER_BASE                       (unsigned long long int)0x0001000900B50000   /*fail to dump parameter into a file*/
#define FAIL_INTERPRETER_SYNTAX_ERROR               (unsigned long long int)0x0001000900B50001 
#define FAIL_INTERPRETER_UNBALANCED_PARENTHESES     (unsigned long long int)0x0001000900B50002 
#define FAIL_INTERPRETER_NO_EXPRESSION_PRESENT      (unsigned long long int)0x0001000900B50003 
#define FAIL_INTERPRETER_EQUALS_SIGN_EXPECTED       (unsigned long long int)0x0001000900B50004 
#define FAIL_INTERPRETER_NOT_A_VARIABLE             (unsigned long long int)0x0001000900B50005 
#define FAIL_INTERPRETER_LABEL_TABLE_FULL           (unsigned long long int)0x0001000900B50006 
#define FAIL_INTERPRETER_DUPLICATE_SUB_LABEL        (unsigned long long int)0x0001000900B50007 
#define FAIL_INTERPRETER_UNDEFINED_SUB_LABEL        (unsigned long long int)0x0001000900B50008
#define FAIL_INTERPRETER_THEN_EXPECTED              (unsigned long long int)0x0001000900B50009 
#define FAIL_INTERPRETER_TO_EXPECTED                (unsigned long long int)0x0001000900B5000A 
#define FAIL_INTERPRETER_TOO_MANY_NESTED_FOR_LOOPS  (unsigned long long int)0x0001000900B5000B 
#define FAIL_INTERPRETER_NEXT_WITHOUT_FOR           (unsigned long long int)0x0001000900B5000C 
#define FAIL_INTERPRETER_TOO_MANY_NESTED_GOSUB      (unsigned long long int)0x0001000900B5000D 
#define FAIL_INTERPRETER_RETURN_WITHOUT_GOSUB       (unsigned long long int)0x0001000900B5000E 
#define FAIL_INTERPRETER_FILE_NOT_FOUND             (unsigned long long int)0x0001000900B5000F 
#define FAIL_INTERPRETER_MOVL_WITH_JOINT            (unsigned long long int)0x0001000900B50010
#define FAIL_INTERPRETER_MOVJ_WITH_POINT            (unsigned long long int)0x0001000900B50011 
#define FAIL_INTERPRETER_ILLEGAL_LINE_NUMBER        (unsigned long long int)0x0001000900B50012 

#define FAIL_INTERPRETER_ALARM_EXEC_BASE            (unsigned long long int)0x0001000900B50100 
#define FAIL_INTERPRETER_USER_ALARM1                (unsigned long long int)0x0001000900B50101
#define FAIL_INTERPRETER_USER_ALARM2                (unsigned long long int)0x0001000900B50102
#define FAIL_INTERPRETER_USER_ALARM3                (unsigned long long int)0x0001000900B50103
#define FAIL_INTERPRETER_USER_ALARM4                (unsigned long long int)0x0001000900B50104
#define FAIL_INTERPRETER_USER_ALARM5                (unsigned long long int)0x0001000900B50105
#define FAIL_INTERPRETER_USER_ALARM6                (unsigned long long int)0x0001000900B50106
#define FAIL_INTERPRETER_USER_ALARM7                (unsigned long long int)0x0001000900B50107
#define FAIL_INTERPRETER_USER_ALARM8                (unsigned long long int)0x0001000900B50108
#define FAIL_INTERPRETER_USER_ALARM9                (unsigned long long int)0x0001000900B50109
#define FAIL_INTERPRETER_USER_ALARM10               (unsigned long long int)0x0001000900B5010A
#define FAIL_INTERPRETER_NOT_IN_PAUSE               (unsigned long long int)0x0001000900B5010B



#define FALT_SET_STEP                           (unsigned long long int)0x000100020067041B   /*fail_set_step*/
#define FALT_SET_TARGET                         (unsigned long long int)0x000100020067041C   /*fail_set_target*/

#define TPI_SUCCESS				(0)

#endif

