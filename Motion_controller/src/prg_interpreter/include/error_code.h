/**
 * @file error_code.h
 * @brief 
 * @author WangWei
 * @version 1.0.0
 * @date 2016-08-29
 */
#ifndef INTERPRETER_ERROR_H_
#define INTERPRETER_ERROR_H_


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
#define FAIL_INTERPRETER_FUNC_PARAMS_MISMATCH       (unsigned long long int)0x0001000900B50013 

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



#define TPI_SUCCESS				(0)
//#define INT64 ErrorCode


#endif
