#ifndef _TOUCH_TEST_H_
#define _TOUCH_TEST_H_

#include <iostream>
#include <mutex>
#include <thread>
#include <Windows.h>
#include <mmsystem.h>
#include "buffer_manager.h"
#include "general_params.h"
#include "dlist_help.h"
#include "rpc_help.h"
#include <touch_interface.h>

#define RELAY_BUFF_SIZE (ONLINETRJ_FRAME_SIZE*4096)
#define LOG_BUFF_SIZE (1024*1024)
#define OFFLINE_FILE_PATH "offline_trj.csv"
#define TICK_BUFF_NUM 1000


struct TickManger {
    int index;
    double tick_buff[TICK_BUFF_NUM];
};
class TouchTest
{
public:
    /**
     * @brief Get the single object of TouchTest.
     * @retval Single object of TouchTest.
     */
    static TouchTest& getInstance()
    {
        static TouchTest a;
        return a;
    }
    /**
     * @brief Destructor of TouchTest.
     * @retval void
     */
    ~TouchTest();

    /**
    * @brief Enable the touch solfware.
    * @retval void.
    */
    void ServiceEnable(void);

    /**
    * @brief Disable the touch solfware.
    * @retval void.
    */
    void ServiceDisable(void);

    /**
    * @brief Set fault of the touch solfware.
    * @retval void.
    */
    void ServiceSetFault(void);

    /**
    * @brief Resetfault of the touch solfware.
    * @retval void.
    */
    void ServiceResetFault(void);
    /**
    * @brief Get state of the touch solfware.
    * @retval TouchState_e.
    *   - SHUTDOWN_STAT = -1,
    *   - INIT_STAT = 0,
    *   - IDEL_STAT = 1,
    *   - PRE_WORK_STAT = 2,
    *   - WORK_STAT = 3,
    *   - ERR_STAT = 4,
    */
    int ServiceGetState(void);

    /**
    * @brief Service to read sample data from touch.
    * @retval void.
    */
    void ServiceReadTouch(void);

    /**
    * @brief Service to send sample data to controller.
    * @retval void.
    */
    void ServiceWriteController(void);

    /**
    * @brief Service to operate the state machine of touch software.
    * @retval void.
    */
    void ServiceStatemachine(void);

    /**
    * @brief Service to get tick of read thread and send thread.
    * @retval void.
    */
    void ServiceTickRecord(void);
private:

    TouchTest();

    bool InitHighTimer(void);
    void FreeHighTimer(void);

    void TouchInit(void);
    bool TouchReInit(void);

    bool TouchCfgInit(void);
    bool TouchDevInit(void);
    bool CtrlCommInit(void);

public:
    bool is_enable_;
    bool is_exit_;
    bool is_error_;

    bool is_config_init_;
    bool is_touch_init_;
    bool is_rpc_init_;
    bool is_timer_init_;

    int sig_read_start_ = 0;
    int sig_write_start_ = 0;

    TouchParams_t touch_params_;
    buffer_info_t relay_buff_info_;
    char relay_buff[RELAY_BUFF_SIZE];

    TouchState_e work_state_;
private:
    //timer params
    MMRESULT g_mmTimerId = 0;
    UINT g_wAccuracy = 0;

    //tick record
    std::mutex tick_mutex_;
    dlist_node_t* tick_node_;

    std::thread touch_read_thread_;
    std::thread statemachine_thread_;
    std::thread tick_record_thread_;
};

int UsSleep(int us);
void TimerTask_RpcSend(void);
char* TouchErrInfoGet(void);

#endif


