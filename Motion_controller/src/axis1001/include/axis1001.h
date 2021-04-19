#ifndef AXIS_1001_H
#define AXIS_1001_H

/**
 * @file axis1001.h
 * @brief The file is the header file of class "axis1001" for stepper motor.
 * @author youming.wu
 */

#include "axis.h"
#include "error_queue.h"

namespace axis_space {

/**
 * @brief Axis1001 is the stepper motor class.
 * @details It is the subclass of Axis. \n
 *          Mostly it handles the Function Block which generates the process data objects(PDO) of a motion.\n
 *          Different axis type might have different implementations of the applications.\n
 *          The differences might be the axis path algorithms or trajectory data structure.\n
 */
class Axis1001 : public Axis
{
  public:
    /**
     * @brief Constructor of the class.
     * @param [in] id The reference to axis.
     */
    Axis1001(int32_t id);
    /**
     * @brief Destructor of the class. 
     */ 
    virtual ~Axis1001(void);

    /**
     * @brief Initialize the axis application.
     * @details Load the path algorithm.\n
     *          Create the function block queue and trajectory block queue.\n
     * @retval false Failed.
     * @retval true Success.
     */
    virtual bool initApplication(void);

    /**
     * @brief Load the model parameters of the axis application.
     * @details Load the parameters which take effect after power on.\n
     * @retval false Failed.
     * @retval true Success.
     */
    virtual bool reloadSystemModel(void);

    /**
     * @brief Control the power stage(On or Off).
     * @param [in] enable
     * - false Power is being enabled,
     * - true Power is being disabled,
     * @return error_code.
     */
    virtual ErrorCode mcPower(bool enable);

    /**
     * @brief Push a Function Block to the queue.
     * @details The function block might comes from RPC or program text.
     * @retval false Failed to push a FB if no algorithm is assigned or under an invalid status of the axis.
     * @retval true Success.
     */
    virtual bool pushBackFB(void* fb_ptr);

    /**
     * @brief Get the status of the Function Block Queue.
     * @details 
     * @return 
     * - FBQ_STATUS_EMPTY = 0,      
     * - FBQ_STATUS_NOT_FULL = 1,
     * - FBQ_STATUS_FULL = 2,      
     */
    virtual base_space::FBQueueStatus_e getFBQStatus();

    /**
     * @brief Compute the Function Block.
     * @details The function block will be processed to generate the trajectory block by the assigned algorithm.
     * @return void
     */
    virtual void processFBQ();

    /**
     * @brief Send the Trajectory Block queue to the servo.
     * @details The trajectory will be passed to the servo side.
     * @return void
     */
    virtual void processTBQ();

    /**
     * @brief Clear the function block queue and the trajectory block queue.
     * @details It is called under power_on, reset and stop.\n       
     * @return void
     */
    virtual void clearBQ();

  private:    
  //
};

}
#endif
