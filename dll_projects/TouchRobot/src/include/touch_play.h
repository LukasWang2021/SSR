#ifndef TOUCHPLAY_H__
#define TOUCHPLAY_H__
#ifdef __cplusplus 
extern "C" { 
#endif


#define KEY_NUMS 2
#define KEY_PRESSED_KEEP_CNT 60 // need 0.5s
//#define KEY_PRESSED_RELEASE_KEEP_CNT 70 // need 0.5s

typedef enum{
	KEY_A = 0,
	KEY_B = 1,
}Key_e;
	
typedef enum{
	TRIGGER = 1,
	RELEASE = 2,
	SINGLE_CLICK = 3,
	T_DOUBLE_CLICK = 4,
	LONG_PRESS = 5,
	NOT_PRESS = 6,
	DOUBLE_CLICK_CHECK = 7,
}KeyState_e;

/**
 * @brief Init touch device.
 * @retval 0 Operation is successful.
 * @retval -1 Operation is failed.
 */
int DeviceInit(void);

/**
 * @brief Calibrate touch device.
 * @retval 0 Operation is successful.
 * @retval -1 Operation is failed.
 */
int DeviceCalib(void);

/**
 * @brief Config sample rate of touch device by configuration file.
 * @param [in] sample frequency.
 * @return void
 */
void DeviceCommConfig(int freq);

/**
 * @brief Start touch device to sample data.
 * @retval 0 Operation is successful.
 * @retval -1 Operation is failed.
 */
int DeviceStart(void);

/**
 * @brief Pull sample data of touch device.
 * @param [in] data buffer
 * @param [in] sizeof data buffer
 * @retval 0 Operation is successful.
 * @retval -1 Operation is failed.
 */
int DevicePullData(char *buff, int size);

/**
 * @brief Stop touch device to sample data.
 * @retval 0 Operation is successful.
 * @retval -1 Operation is failed.
 */
int DeviceStop(void);

/**
 * @brief Disable touch device.
 * @retval 0 Operation is successful.
 * @retval -1 Operation is failed.
 */
int DeviceDisable(void);

/**
 * @brief Get key envent.
 * @param [in] which key, KEY_A=0 or KEY_B=1.
 * @retval 	KeyState_e
 *	- TRIGGER = 1,
 *	- RELEASE = 2,
 *	- SINGLE_CLICK = 3,
 *	- T_DOUBLE_CLICK = 4,
 *  - LONG_PRESS = 5,
 *  - NOT_PRESS = 6,
 *  - DOUBLE_CLICK_CHECK = 7,
 */
int DeviceKeyState_Get(int);

/**
 * @brief Reset temp value of key pressed check program.
 * @return void
 */
void DeviceKeyState_Reset(void);

/**
 * @brief Get error information of touch_play module.
 * @retval String of error information.
 */
//char* DeviceErrInfoGet();

/**
 * @brief Force feedback control.
 * @param [in] 0:disable, 1:enable.
 * @retval 	void
 */
void DeviceForceFeedCtrl(int);
#ifdef __cplusplus 
} 
#endif

#endif

