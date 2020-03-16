//-----------------------------------------------------------------------------
// Copyright(c) 2019-2050 Foresight-Robotics
// All rights reserved.
//
// File: fst_spi.h
// Version: 0.2.01
// 
// Create: Jan.07 2019
// Author: Shuguo, Zhang
// Summary: This file defines interfaces as a master that access SPI with 
//          Avalon interface. The SPI peripheral is a slave.
//-----------------------------------------------------------------------------

#ifndef _FST_SPI_H_
#define _FST_SPI_H_

#include <fcntl.h>
#include <stdint.h>
#include <sys/mman.h>
#include <stddef.h>
#include <mutex>
#include <cstdbool>

typedef uint64_t ErrCode;

class FstSpi {
    public:
        static FstSpi* instance_;
        static FstSpi* getInstance();
    public:
        ErrCode transferData(uint8_t *tx, uint8_t *rx, uint32_t len);
        void lockSpi();
        void unlockSpi();
    private:
        FstSpi();
        ~FstSpi();
    private:
        ErrCode initSpi();
        ErrCode openSpi();
        ErrCode mapSpiReg();
    private:
        uint32_t getStatus();
        void printAllRegs();
        bool isRxReady();
        bool isTxReady();
        bool isRxOverRun();
        bool isTxOverRun();
        bool isError();
        bool isEndOfPacket();
    private:
        void setSso();
        void clearSso();

        ErrCode setEopValue(uint32_t len);
        uint32_t getEopValue();

        ErrCode selectSlaveN(uint32_t slave);
        void releaseSlaveN();
    private:
        int fd_;
        void *spi_base_;
        std::mutex spi_mutex_;
        const uint32_t timeout_trylock_spi_ = 5000; // the uinit is microsecond.
        
};

enum FST_SPI_MAP {
    FST_SPI_BASE = 0xFF250000,
    FST_SPI_REG_LEN = (7 * sizeof(uint32_t)),
    FST_SPI_MAP_PROT = (PROT_READ | PROT_WRITE),
    FST_SPI_MAP_FLAG = MAP_SHARED,
    FST_SPI_MAP_OFST = FST_SPI_BASE,
};

enum AVALON_SPI_REG_MAPPING {
    AVALON_SPI_RXDATA = 0,
    AVALON_SPI_TXDATA = 1,
    AVALON_SPI_STATUS = 2,
    AVALON_SPI_CONTROL = 3,
    AVALON_SPI_RESERVED = 4,
    AVALON_SPI_SLAVESELECT = 5,
    AVALON_SPI_EOP_VALUE = 6,
};

enum AVALON_SPI_STATUS_BITS {
    AVALON_SPI_ROE = 0x8,
    AVALON_SPI_TOE = 0x10,
    AVALON_SPI_TMT = 0x20,
    AVALON_SPI_TRDY = 0x40,
    AVALON_SPI_RRDY = 0x80,
    AVALON_SPI_E = 0x100,
    AVALON_SPI_EOP = 0x200,
};

enum AVALON_SPI_CONTROL_BITS {
    AVALON_SPI_IROE = 0x8,
    AVALON_SPI_ITOE = 0x10,
    AVALON_SPI_ITRDY = 0x40,
    AVALON_SPI_IRRDY = 0x80,
    AVALON_SPI_IE = 0x100,
    AVALON_SPI_IEOP = 0x200,
    AVALON_SPI_SSO = 0x400,
};

enum FST_SPI_SLAVE {
    FST_SPI_SLAVE_1 = 1,
};

enum FST_SPI_ERROR {
    FST_SPI_OK = 0,
    FST_SPI_OPEN_F = 0x0001000B00AE0001, // SPI module can't open file "/dev/mem".
    FST_SPI_MAP_F = 0X0001000B00AE0002, // SPI module can't map the address to SPI registers.
    FST_SPI_EOP_VALUE_F = 0x0001000200AE0003, // SPI module can't set eop_value because it is transferring datas.
    FST_SPI_RX_OVERRUN_F = 0x0001000200AE0004, // SPI's rxdata regitster is overran and received datas are unavailable.

};

#endif // _FST_SPI_H_