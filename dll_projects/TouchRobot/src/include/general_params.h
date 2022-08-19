#ifndef GENERAL_PARAMS_H_
#define GENERAL_PARAMS_H_

#ifdef __cplusplus
extern "C" {
#endif

#define _TOUCH_DLL_VERSION_ "TouchDLL-V2.0"
#ifdef _WIN_PLAT
#define M_E        2.71828182845904523536
#define M_LOG2E    1.44269504088896340736
#define M_LOG10E   0.434294481903251827651
#define M_LN2      0.693147180559945309417
#define M_LN10     2.30258509299404568402
#define M_PI       3.14159265358979323846
#define M_PI_2     1.57079632679489661923
#define M_PI_4     0.785398163397448309616
#define M_1_PI     0.318309886183790671538
#define M_2_PI     0.636619772367581343076
#define M_2_SQRTPI 1.12837916709551257390
#define M_SQRT2    1.41421356237309504880
#define M_SQRT1_2  0.707106781186547524401
#endif

    typedef struct {
        int freq_touch; 			           // touch的采样频率 //2ms
        int filter_enum_xyz;				   // 0-butter_worth; 1-mean_window; 2-both	
        int filter_enum_abc;
        int filter_fd_xyz;					   // 截止频率
        int filter_fd_abc;
        int filter_mean_xyz_nums;			   // 滑窗点数
        int filter_mean_abc_nums;
        int rsv_touch[3];
        char* controller_ip;				   // controller ip
        int online_trj_flag;		           // 在线轨迹flag
        int cycle_time_controller;             // controller的通信周期 //10ms
        int point_nums;                        // 每次给controller下发的轨迹点数

    }TouchParams_t;

    typedef enum {
        INIT_STAT = 0,
        IDEL_STAT = 1,
        PRE_WORK_STAT = 2,
        WORK_STAT = 3,
        ERR_STAT = 4,
    }TouchState_e;

    typedef enum {
        SEND_START = 0,
        SEND_WORK = 1,
        SEND_END = 2,
    }ssr_touch_sendst_e;

#ifdef __cplusplus
}
#endif


#endif