//-----------------------------------------------------------------------------
// Copyright(c) 2019-2050 Foresight-Robotics
// All rights reserved.
//
// File: nvram.h
// Version: 0.2.01
// 
// Create: Jan.07 2019
// Author: Shuguo, Zhang
// Summary: This file provides some functions to access to NvRam.
//-----------------------------------------------------------------------------

#ifndef _FST_NVRAM_H_
#define _FST_NVRAM_H_

#include "fst_spi.h"
#include <stdint.h>
#include <cstdbool>

typedef uint64_t ErrCode;

class Nvram
{
private:
    // spi bus
    FstSpi* spi_;

    uint32_t data_len_default_;
    uint32_t data_len_;
    uint32_t additional_len_; // additional_len_ is 4 bytes, which is the total size of a instruction and a address.
    uint32_t address_;
    
    // the structure of datas as a group that is transfer between a master and a nvram.
    struct NvramData
    {
        uint8_t instruction;
        uint8_t address[3];
        uint8_t data[0];
    };
    NvramData *xmit_;
    NvramData *recv_;

	// lujiaming tries to remove static at 0320
    static bool is_Checked_Ready_;

    void writeRoutine();
    void readRoutine();
    uint8_t bcc(uint8_t* data, uint32_t length);
    void setAddress(uint32_t address);

public:
    Nvram(uint32_t length);
    ~Nvram();

    /* read from or write to Nvram */
    ErrCode read(uint8_t* dest, uint32_t addr, uint32_t length);
    void write(uint8_t* src, uint32_t addr, uint32_t length);
    bool writeSync(uint8_t* src, uint32_t addr, uint32_t length);

    /* functions in a initialization-step */
    ErrCode openNvram();
    ErrCode isNvramReady();

    /* calculate the check_sum of the data transmitted/received */
    uint8_t checkSumOfData(uint8_t* data, uint32_t length);
};

enum InstructionSet {
    NVRAM_INS_READ = 0x03,
    NVRAM_INS_WRITE = 0x02
};

enum NvramMapping {
    /* The 23LCV1024 contains 1M bits, which says 128K bytes.
     * 1M bits = 128K bytes = 0x20000 bytes
     */
    NVRAM_MAX_SIZE = 0x20000, 
    NVRAM_BLK_SIZE = 0x10000,
    NVRAM_BLK_1_BASE = 0x0,
    NVRAM_BLK_2_BASE = NVRAM_BLK_1_BASE + NVRAM_BLK_SIZE,

    POWER_ON_TEST = 0x0,
    POWER_ON_TEST_LEN = 0x10,
    POWER_DOWN_CHK =  POWER_ON_TEST + POWER_ON_TEST_LEN,
    POWER_DOWN_CHK_LEN = 0x10,

    ZERO_BLK = POWER_DOWN_CHK + POWER_DOWN_CHK_LEN,
    ZERO_BLK_LEN = 0x100,

    REG_BLK = ZERO_BLK + ZERO_BLK_LEN,
    REG_BLK_LEN = NVRAM_BLK_SIZE - REG_BLK,
};

enum NvramError {
    FST_NVRAM_OK = 0,
    FST_NVRAM_ALLOC_F = 0x0011000200AE0005,
    FST_NVRAM_BADDATA_F = 0x0001000200AE0006,
    FST_NVRAM_R_TIMEOUT_F = 0x0001000200AE0007,
    FST_NVRAM_OVERSIZE_F = 0x0011000200AE0008,
    FST_NVRAM_NOT_READY_F = 0x0011000200AE0009,
    FST_NVRAM_ZERO_BAD_F = 0x0011000200AE000A,
};

#endif // _FST_NVRAM_H_
