#ifndef INTERPRETER_COMMON_H_
#define INTERPRETER_COMMON_H_

#include "fst_datatype.h"
#include "stdint.h"
using namespace fst_controller;

#define ADD_INFO_NUM    10

#define TP_XPATH_LEN     1024

#define USE_XPATH

typedef enum _AdditionalInfomationType
{
    ACC, 
    CONDITION,
    CONDITION_OFFSET ,
    OFFSET ,
    TOOL_OFFSET, 
    EV,
    IND_EV,
    TB,
    TA,
    DB,
} AdditionalInfomationType;

typedef enum _AdditionalOffsetInfomationType
{
    PR, 
    PR_UF, 
    C_VEC ,
    C_VEC_UF ,
    J_VEC, 
    J_VEC_UF, 
} AdditionalOffsetInfomationType;

typedef enum __AdditionalExecuteInfomationType
{
    ASSIGNMENT, 
    SUBROUTINE ,
} AdditionalExecuteInfomationType;

typedef enum _InstType
{
    COMMON = 1,
    LOGIC_TOK,
    END_TOK,
    MOTION,
    END_PROG,
}InstType;

typedef enum _InterpreterState
{
    IDLE_R      = 0,    
    EXECUTE_R   = 1,
    PAUSED_R    = 2,
//    WAITING_R   = 3,

    IDLE_TO_EXECUTE_T   = 101,
    EXECUTE_TO_PAUSE_T  = 102,
    PAUSE_TO_IDLE_T     = 103,
    PAUSE_TO_EXECUTE_T  = 104,
	
    ERROR_EXEC_BASE_T    = 200,                 
    ERROR_SYNTAX_ERROR_T = 201,                // 0 
    ERROR_UNBALANCED_PARENTHESES_T = 202,      // 1 
    ERROR_NO_EXPRESSION_PRESENT_T = 203,       // 2 
    ERROR_EQUALS_SIGN_EXPECTED_T = 204,        // 3 
    ERROR_NOT_VARIABLE_T = 205,                // 4 
    ERROR_LABEL_TABLE_FULL_T = 206,            // 5 
    ERROR_DUPLICATE_SUB_LABEL_T = 207,         // 6 
    ERROR_UNDEFINED_SUB_LABEL_T = 208,         // 7 
    ERROR_THEN_EXPECTED_T = 209,               // 8 
    ERROR_TO_EXPECTED_T = 210,                 // 9 
    ERROR_TOO_MANY_NESTED_FOR_LOOPS_T = 211,   // 10 
    ERROR_NEXT_WITHOUT_FOR_T = 212,            // 11
    ERROR_TOO_MANY_NESTED_GOSUBS_T = 213,      // 12 
    ERROR_RETURN_WITHOUT_GOSUB_T = 214,        // 13 
    ERROR_FILE_NOT_FOUND_T = 215               // 14

}InterpreterState;

typedef enum _InterpreterCommand
{
    LOAD    = 101,
    JUMP    = 102,
    START   = 103,
    DEBUG   = 104,
    FORWARD  = 105,
    BACKWARD = 106,
    CONTINUE = 107,
    PAUSE   = 108,
    ABORT   = 109,
		
    MOD_REG  = 201,
	READ_REG = 202,
    MOD_IO   = 203,
    READ_IO  = 204,
    
    READ_SMLT_STS  = 205,
	MOD_SMLT_STS   = 206,
    MOD_SMLT_VAL   = 207,
    
    READ_CHG_PR_LST   = 208,
    READ_CHG_SR_LST   = 209,
    READ_CHG_R_LST   = 210,
    READ_CHG_MR_LST   = 211,

}InterpreterCommand;


typedef enum _RegDIOType
{
    NOTDEFINE = 0,
    REG = 1,
    DIO = 2,
}RegDIOType;


typedef enum _DIOType
{
    UNDEFINE = 0,
    DI = 1,
    DO = 2,
}DIOType;

typedef enum _RegType
{
    POSE_REG           = 101,
    POSE_REG_POSE      = 102,
    POSE_REG_JOINT     = 103,
    POSE_REG_TYPE      = 104,
    POSE_REG_ID        = 105,
    POSE_REG_COMMENT   = 106,
	// 6
    STR_REG            = 201,
    STR_REG_VALUE      = 202,
    STR_REG_ID         = 203,
    STR_REG_COMMENT    = 204,
    // 6 + 4 = 10
    NUM_REG            = 301,
    NUM_REG_VALUE      = 302,
    NUM_REG_ID         = 303,
    NUM_REG_COMMENT    = 304,
	// 10 + 4 = 14
    MOT_REG            = 401,
    MOT_REG_VALUE      = 402,
    MOT_REG_ID         = 403,
    MOT_REG_COMMENT    = 404,
	// 14 + 4 = 18
    UF_REG             = 501,
    UF_REG_COORD       = 502,
    UF_REG_ID          = 503,
    UF_REG_COMMENT     = 504,
	// 18 + 4 = 22
    TF_REG             = 601,
    TF_REG_COORD       = 602,
    TF_REG_ID          = 603,
    TF_REG_COMMENT     = 604,
	// 22 + 4 = 26
    PL_REG             = 701,
    PL_REG_POSE        = 702,
    PL_REG_PALLET      = 703,
    PL_REG_FLAG        = 704,
    PL_REG_ID          = 705,
    PL_REG_COMMENT     = 706,
	// 26 + 6 = 32
}RegType;

#define REG_TYPE_NUM    32

/*
typedef struct _DIOMap
{
    int     card_seq;
    DIOType type;
    int     port_seq;
    bool has_value;
    int     value;
}DIOMap;
*/

typedef struct _IOMapPortInfo
{
    uint32_t    msg_id;
    uint32_t    dev_id;
    int         port_type;
    int         port_index;
    int         bytes_len;
    char        value;
}IOMapPortInfo;

typedef struct _IOPathInfo
{
    char        dio_path[128];
    char        value;
}IOPathInfo;

typedef struct _Reg
{
    RegType type;
    int     index;
}Reg;

typedef struct _RegMap
{
    RegType type;
    int     index;
    char    value[512];
}RegMap;

typedef enum _UserOpMode
{
    NONE_U                  = 0,
    AUTO_MODE_U             = 1,
    SLOWLY_MANUAL_MODE_U    = 2,
    UNLIMITED_MANUAL_MODE_U = 3,
}UserOpMode;



typedef struct _StartCtrl
{
    char        file_name[128];
}StartCtrl;

typedef struct _InterpreterControl
{
    InterpreterCommand cmd;
    union
    {
        StartCtrl   start_ctrl;
        // int         id;
#ifdef USE_XPATH
        char           line[TP_XPATH_LEN];
#else
    int             line;
#endif
        RegMap      reg;
        // IOMapPortInfo  dio;
        IOPathInfo  dioPathInfo;
    };
}InterpreterControl;


typedef struct _CtrlStatus
{
    bool        is_permitted;  //if sending next move command is permitted
    UserOpMode  user_op_mode;
    bool        is_data_ready;  //if sending next move command is permitted
}CtrlStatus;

typedef struct _IntprtStatus
{
#ifdef USE_XPATH
    char            line[TP_XPATH_LEN];
#else
    int             line;
#endif
    InterpreterState    state;
#ifdef WIN32
	__int64         warn;
#else
    long long int   warn;
#endif
}IntprtStatus;

typedef struct _AdditionalOffsetInfomation
{
    AdditionalOffsetInfomationType        type;
    union {
        Reg             pr_reg;
        PoseEuler       pose_target;
        Joint           joint_target;
    };
    Reg                 uf_reg;
} AdditionalOffsetInfomation;

typedef struct _AdditionalExecuteInfomation
{
    AdditionalExecuteInfomationType type;
    double    range ;
    union {
        RegMap          assignment;
        char            fname[128];
    };
} AdditionalExecuteInfomation;

typedef struct _AdditionalInfomation
{
    AdditionalInfomationType        type;
    union 
    {
    	int acc_speed ;                         // used for AAC, EV, IND_EV
    	AdditionalOffsetInfomation offset ; // used for OFFSET, TOOL_OFFSET
    	AdditionalExecuteInfomation execute ; // used for TB, TA, DB
    };
} AdditionalInfomation;

typedef struct _Instruction
{
#ifdef USE_XPATH
    char            line[TP_XPATH_LEN];
#else
    int             line;
#endif
    InstType        type;
    union 
    {
        MotionTarget    target;
        int             loop_cnt;
    };
    bool is_additional;
    int add_num;
#ifdef WIN32
    char additional; //malloc other memory
#else
    char additional[0]; //malloc other memory
#endif
    //add some other additional statement
    //.....
    //.....
}Instruction;

#endif
