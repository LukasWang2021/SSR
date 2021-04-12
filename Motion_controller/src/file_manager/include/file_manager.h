#ifndef FILE_MANAGER_H
#define FILE_MANAGER_H

/**
 * @file file_manager.h
 * @brief The file is the header file of class "FileManager".
 * @author Feng.Wu
 */

#include <stdint.h>
#include "common_error_code.h"

/**
 * @brief base_space includes all foundational definitions and realizations
 */
namespace base_space
{

/**
 * @brief FielManager can be used to read or write a text file.
 */
class FileManager
{
public:
    /**
     * @brief Constructor of the class.
     */
    FileManager();
    /**
     * @brief Destructor of the class. 
     */ 
    ~FileManager();

    /**
     * @brief read the text.
     * @param [in] file_path The full path and the name of the text file.
     * @param [out] ptr The pointer to the data stream.
     * @param [out] length the length of the data stream.
     * @return ErrorCode.
     */
    ErrorCode readFileStream(uint8_t* &ptr, long &length, const char* file_path);

    /**
     * @brief write the text to a file.
     * @param [in] ptr The pointer to the data stream.
     * @param [in] length the length of the data stream.
     * @param [in] file_path The full path and the name of the text file.
     * @return ErrorCode.
     */
    ErrorCode writeFileStream(void* ptr, long length, const char* file_path);

private:
    uint8_t buff_[65535];
};

}

#endif

