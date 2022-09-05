#ifndef RPC_HELP_H
#define RPC_HELP_H
#include <stdint.h>
#ifdef __cplusplus 
extern "C" { 
#endif
#define ONLINETRJ_FRAME_HEAD_NUM  1
#define ONLINETRJ_FRAME_POINT_NUM 5
#define ONLINETRJ_FRAME_TF_NUM    16
#define ONLINETRJ_FRAME_NUM       ( (ONLINETRJ_FRAME_HEAD_NUM + ONLINETRJ_FRAME_TF_NUM) * ONLINETRJ_FRAME_POINT_NUM )

#define ONLINETRJ_FRAME_HEAD_SIZE 8       // sizeof(double)
#define ONLINETRJ_FRAME_TF_SIZE 128       // 16*sizeof(double)
#define ONLINETRJ_FRAME_SIZE (ONLINETRJ_FRAME_HEAD_SIZE+ONLINETRJ_FRAME_TF_SIZE)

#define MAX_TF_NUMS 32

typedef struct{
	int head;// 0-begin , 1-keep , 2-end
	int rsv_dat;
	double tf[ONLINETRJ_FRAME_TF_NUM];
}online_trj_frame_t;

typedef struct {
	int send_nums;
	int state[MAX_TF_NUMS];
	double tf_mtx[MAX_TF_NUMS][ONLINETRJ_FRAME_TF_NUM];
}online_trj_send_t;

//1-auto, 2-manaul_slow,3-manual,4-online
typedef enum{
	CONTROLLER_WORK_MODE_AUTO,
	CONTROLLER_WORK_MODE_MANUAL_SLOW,
	CONTROLLER_WORK_MODE_MANUAL,
	CONTROLLER_WORK_MODE_ONLINE,
}controller_work_mode_e;

/**
 * @brief Init communication of rpc_help module.
 * @param [in] ip address of controller.
 * @retval 0 Operation is successful.
 * @retval -1 Operation is failed.
 */
extern int rpc_help_comm_init(char *ip);

/**
 * @brief Called when not used rpc_help module.
 * @retval 0 Operation is successful.
 * @retval -1 Operation is failed.
 */
extern int rpc_help_comm_end(void);

/**
 * @brief Set work mode of controller.
 * @param [in] work mode, 4-online mode.
 * @retval 0 Operation is successful.
 * @retval -1 Operation is failed.
 */
extern int rpc_help_setMode(uint32_t mode);

/**
 * @brief Send transmatrix and state to controller.
 * @param [in] transmatrix arrays
 * @param [in] state, 0-begin, 1-middle, 2-end
 * @param [in] count of transmatrixs
 * @retval 0 Operation is successful.
 * @retval -1 Operation is failed.
 */
extern int rpc_help_sendOnlineTrajectory(double traj[], int state[], int cnt);

/**
 * @brief Get force of sensor.
 * @param [in] force buffer address.
 * @param [in] force buffer size address.
 * @retval 0 Operation is successful.
 * @retval -1 Operation is failed.
 */
extern int rpc_help_getForce(double* force, int32_t size);

#ifdef __cplusplus 
} 
#endif
#endif

