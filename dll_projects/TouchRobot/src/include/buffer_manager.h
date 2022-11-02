#ifndef __BUFFER_MANAGER_H__
#define __BUFFER_MANAGER_H__
#include "stdint.h"
#include <mutex>
#ifdef __cplusplus
extern "C" {
#endif

    typedef struct
    {
        int32_t tail_index_offset;
        int32_t head_index_offset;
        int32_t element_length;
        int32_t buff_length;
        int32_t safety_resevd;
        int32_t semaphore;
        //int32_t write_lock;
        std::mutex mtx;
        //char * buff;
    }buffer_info_t;

    /**
    * @brief Push one item to circle buffer.
    * @param [in] circle buffer information.
    * @param [in] circle buffer address.
    * @param [in] local buffer address.
    * @retval -1 Push failed.
    * @retval >0 Push item counters.
    */
    extern int32_t push_circle_buff_item(buffer_info_t* circle_buff_info, char* circle_buff, char* push_ptr);


    /**
    * @brief Push items to circle buffer.
    * @param [in] circle buffer information.
    * @param [in] circle buffer address.
    * @param [in] souce buffer information.
    * @param [in] source circle buffer address.
    * @retval -1 Push failed.
    * @retval >0 Push item counters.
    */
    extern int32_t push_circle_buff_bundle(buffer_info_t* circle_buff_info, char* circle_buff,
        buffer_info_t* local_buff_info, char* local_buff);

    /**
    * @brief Pull one item to local circle buffer.
    * @param [in] circle buffer information.
    * @param [in] circle buffer address.
    * @param [in] local buffer address.
    * @retval 0 Operation is successful.
    * @retval -1 Operation is failed.
    */
    extern int32_t pull_circle_buff_item(buffer_info_t* circle_buff_info, char* circle_buff, char* pull_ptr);

    /**
    * @brief Pull items to local circle buffer.
    * @param [in] circle buffer information.
    * @param [in] circle buffer address.
    * @param [in] locak circle buffer information.
    * @param [in] local buffer address.
    * @retval >0 Pull item counters.
    * @retval -1 Operation is failed.
    */
    extern int32_t pull_circle_buff_bundle(buffer_info_t* circle_buff_info, char* circle_buff,
        buffer_info_t* local_buff_info, char* local_buff);

    /**
    * @brief Pull items to local buffer.
    * @param [in] circle buffer information.
    * @param [in] circle buffer address.
    * @param [in] local buffer address.
    * @param [in] request items numbers.
    * @retval >0 Pull item counters.
    * @retval -1 Operation is failed.
    */
    extern int32_t pull_circle_buff_all(buffer_info_t* circle_buff_info, char* circle_buff, char* pull_ptr, int32_t req_nums);

    /**
     * @brief Get items numbers in circle buffer. 
     * @param [in] circle buffer information.
     * @retval items numbers in circle buffer.
     */
    extern int32_t get_circle_buff_occupied(buffer_info_t* circle_buff_info);

    /**
     * @brief Clear circle buffer.
     * @param [in]  circle buffer information.
     * @retval 0 Operation is successful.
     * @retval -1 Operation is failed.
     */
    extern int buffer_clear(buffer_info_t *circle_buff_info);

    /**
     * @brief Whether the circle buffer is full.
     * @param [in]  circle buffer information.
     * @retval 0 Not full.
     * @retval 1 Full.
     */
    extern int32_t is_buff_full(buffer_info_t*);

    /**
     * @brief Whether the circle buffer is empty.
     * @param [in]  circle buffer information.
     * @retval 0 Not empty.
     * @retval 1 Empty.
     */
    extern int32_t is_buff_empty(buffer_info_t*);

    /**
     * @brief Lock the circle buffer.
     * @param [in]  circle buffer information.
     * @retval 0 Succeed.
     * @retval -1 Failed.
     */
    extern int buffer_lock(buffer_info_t*);

    /**
     * @brief Unlock the circle buffer.
     * @param [in]  circle buffer information.
     */
    extern void buffer_unlock(buffer_info_t*);

#ifdef __cplusplus
}
#endif
#endif
