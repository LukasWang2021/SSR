#ifndef TOUCH_INTERFACE_H_
#define TOUCH_INTERFACE_H_
#ifdef __cplusplus 
extern "C" {
#endif

//#define _MAKE_DLL_					//open when compile the DLL
#define _RPC_OPEN_	
//#define	_TICK_RECORD_

#define ERR_INFO_BYTES 1024

typedef int(*TOUCH_CALLBACK)(void *usr, void *sys) ;

typedef struct {
	TOUCH_CALLBACK fct;
	void* param;
}Touch_Callbak_Manager_t;

typedef enum {
	TOUCH_EVENT_ERROR_EXIST = 0,
	TOUCH_EVENT_KEY_SINGLE_PRESS = 1,
	TOUCH_EVENT_KEY_DOUBLE_PRESS = 2,
	TOUCH_EVENT_KEY_LONG_PRESS = 3,
	TOUCH_EVENT_KEY_LONG_PRESS_RELEASE = 4,
	TOUCH_EVENT_MAX = 5,
}TouchCallbackOp_e;

/**
 * @brief Enable touch software.
 * @retval 0 Operation is successful.
 * @retval -1 Operation is failed.
 */
extern "C" _declspec(dllexport) int TouchEnable(void);

/**
 * @brief Disable touch software..
 * @retval 0 Operation is successful.
 * @retval -1 Operation is failed.
 */
extern "C" _declspec(dllexport) int TouchDisable(void);

/**
 * @brief Init Set state to fault.
 * @retval 0 Operation is successful.
 * @retval -1 Operation is failed.
 */
extern "C" _declspec(dllexport) int TouchSetFault(void);

/**
 * @brief Reset state from fault to enable.
 * @retval 0 Operation is successful.
 * @retval -1 Operation is failed.
 */
extern "C" _declspec(dllexport) int TouchResetFault(void);

/**
 * @brief Interface of register callback function .
 * @param [in] index of callback function.
 * @param [in] callback function pointer.
 * @param [in] self-defined parameter of callback function.
 * @retval 0 Operation is successful.
 * @retval -1 Operation is failed.
 */
extern "C" _declspec(dllexport) int TouchCallbackRegister(int index, TOUCH_CALLBACK fcn, void *usr);

/**
 * @brief Get touch software's state.
 * @retval  TouchState_e.
 * - INIT_STAT = 0,
 * - IDEL_STAT = 1,
 * - PRE_WORK_STAT = 2,
 * - WORK_STAT = 3,
 * -  ERR_STAT = 4,
 */
extern "C" _declspec(dllexport) int TouchStateGet(void);

/**
 * @brief Get transformation matrix from touch device.
 * @param [in] buffer of transformation matrix.
 * @param [in] size of buffer.
 * @retval 0 Operation is successful.
 * @retval -1 Operation is failed.
 */
extern "C" _declspec(dllexport) int TouchTransfromationMatrixPull(double* tf_mtx_ptr, int size);

extern "C" _declspec(dllexport) char* TouchTestLoopBack(const char* str, int size);
extern "C" _declspec(dllexport) void TouchGetVersion(char* str, int size);
extern "C" _declspec(dllexport) int TouchGetErrInfo(char* str, int size);
extern "C" _declspec(dllexport) int TouchXyzAbcPull(double* point, int size);

extern "C" _declspec(dllexport) void TouchForceFeedCtrl(int open_flag);
/************************************************************************************/
extern int TouchCallbackRun(int index, void* param);
extern int TouchTransfromationMatrixGet(double* tf_mtx_ptr);


#ifdef __cplusplus 
}
#endif
#endif