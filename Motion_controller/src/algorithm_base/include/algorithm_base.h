#ifndef ALGORITHM_BASE_H
#define ALGORITHM_BASE_H

/**
 * @file algorithm_base.h
 * @brief The file is the header file of class "AlgorithmBase".
 * @author zhengyu.shen
 */

#include "model_base.h"
#include "common_error_code.h"
#include <stdint.h>
#include <string>

/**
 * @brief base_space includes all foundational definitions and realizations.
 */
namespace base_space
{
/**
 * @brief AlgorithmBase is the base class for all algorithms.
 * @details All algorithms run in Axis.processFBQ() and Group.processFBQ() should inherit the base class.
 */
class AlgorithmBase
{
public:
    /**
     * @brief Constructor of the class.
     * @param [in] name The name of the algorithm.
     */
    AlgorithmBase(std::string name);
    /**
     * @brief Destructor of the class. 
     */
    virtual ~AlgorithmBase();
    /**
     * @brief Set external source for the algorithm.
     * @details Pure virtual function. It should be realized in its child class.
     * @param [in] source_type The type of the source.
     * @param [in] source_ptr The pointer of the source.
     * @retval true Set external source succeed.
     * @retval false Set external source failed.
     */
    virtual bool setExtSource(uint32_t source_type, void* source_ptr) = 0;
    /**
     * @brief Synchronize the algorithm object by related model parameters.
     * @details Pure virtual function. It should be realized in its child class.
     * @param [in] model_ptr Pointer of the related model.
     * @retval true Operation succeed.
     * @retval false Operation failed.
     */
    virtual bool syncModel(void* model_ptr) = 0;
    /**
     * @brief Reset the algorithm state to initial.
     * @details Pure virtual function. It should be realized in its child class.
     * @return void
     */
    virtual void reset() = 0;
    /**
     * @brief Run the algorithm , calculate something.
     * @details Pure virtual function. It should be realized in its child class.
     * @param [in] raw_fbq_ptr Pointer of the function block queue.
     * @param [in] raw_tbq_ptr Pointer of the trajectory block queue.
     * @param [in] raw_data_ptr Pointer of any other support data.
     * @return If no error happen, return SUCCESS, otherwise return some error code.
     */
    virtual ErrorCode compute(void* raw_fbq_ptr, void* raw_tbq_ptr, void* raw_data_ptr) = 0;
    /**
     * @brief Get the name of the algorithm object.
     * @return Name string.
     */
    std::string getName();
    
private:
    AlgorithmBase();

    std::string name_;  /**< The name of the object*/
};

}


#endif
