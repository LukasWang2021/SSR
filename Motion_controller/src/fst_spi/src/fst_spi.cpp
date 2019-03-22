//-----------------------------------------------------------------------------
// Copyright(c) 2019-2050 Foresight-Robotics
// All rights reserved.
//
// File: fst_spi.cpp
// Version: 0.2.01
// 
// Create: Jan.07 2019
// Author: Shuguo, Zhang
// Summary: This file defines interfaces as a master that access SPI with 
//          Avalon interface. The SPI peripheral is a slave.
//-----------------------------------------------------------------------------

#include "fst_spi.h"
#include <iostream>
#include <sys/mman.h>
#include <chrono>
#include <thread>

FstSpi* FstSpi::instance_ = NULL;

FstSpi* FstSpi::getInstance() {
    if(instance_ == NULL) {
        instance_ = new FstSpi;
    }

    return instance_;
}

FstSpi::FstSpi() {
    initSpi();
}

FstSpi::~FstSpi() {

}

ErrCode FstSpi::openSpi() {
    fd_ = open("/dev/mem", O_RDWR);
    if(fd_ == -1) {
        return FST_SPI_OPEN_F;
    }
    return FST_SPI_OK;
}

ErrCode FstSpi::mapSpiReg() {
    spi_base_ = mmap(NULL, FST_SPI_REG_LEN, FST_SPI_MAP_PROT, FST_SPI_MAP_FLAG, fd_, FST_SPI_MAP_OFST);
    if(spi_base_ ==MAP_FAILED) {
        return FST_SPI_MAP_F;
    }
    std::cout << "BASE: " << spi_base_ << std::endl;
    return FST_SPI_OK;
}

ErrCode FstSpi::initSpi() {
    ErrCode ret;
    ret = FST_SPI_OK;
    
    ret = openSpi();
    if(ret == FST_SPI_OK) {
        ret = mapSpiReg();
    }
    
    return ret;
}

bool FstSpi::trylockSpi() {
    std::chrono::microseconds sleep_time(1000);
    auto start = std::chrono::high_resolution_clock::now();
    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::micro> interval;
    while(!spi_mutex_.try_lock()) {
        end = std::chrono::high_resolution_clock::now();
        interval = end - start;
        if(interval.count() > timeout_trylock_spi_) {
            return false;
        }
        std::this_thread::sleep_for(sleep_time);       
    }
    return true;
}

void FstSpi::unlockSpi() {
    spi_mutex_.unlock();
}

#define FIND_SPI_REG(reg) ((uint32_t *)((uint32_t *)spi_base_ + (reg)))
#define IS_BIT_TRUE(reg, bit) ((((reg) & (bit)) == 0) ? false : true)

bool FstSpi::isRxReady() {
    volatile uint32_t *status;
    status = FIND_SPI_REG(AVALON_SPI_STATUS);
    return IS_BIT_TRUE(*status, AVALON_SPI_RRDY);
}

bool FstSpi::isTxReady() {
    volatile uint32_t *status;
    status = FIND_SPI_REG(AVALON_SPI_STATUS);
    return IS_BIT_TRUE(*status, AVALON_SPI_TRDY);
}

bool FstSpi::isRxOverRun() {
    volatile uint32_t *status;
    status = FIND_SPI_REG(AVALON_SPI_STATUS);
    return IS_BIT_TRUE(*status, AVALON_SPI_ROE);
}

bool FstSpi::isTxOverRun() {
    volatile uint32_t *status;
    status = FIND_SPI_REG(AVALON_SPI_STATUS);
    return IS_BIT_TRUE(*status, AVALON_SPI_TOE);
}

bool FstSpi::isError() {
    volatile uint32_t *status;
    status = FIND_SPI_REG(AVALON_SPI_STATUS);
    return IS_BIT_TRUE(*status, AVALON_SPI_E);
}

bool FstSpi::isEndOfPacket() {
    volatile uint32_t *status;
    status = FIND_SPI_REG(AVALON_SPI_STATUS);
    return IS_BIT_TRUE(*status, AVALON_SPI_EOP);
}

void FstSpi::setSso() {
    volatile uint32_t *control;
    control = FIND_SPI_REG(AVALON_SPI_CONTROL);
    *control |= AVALON_SPI_SSO;
}

void FstSpi::clearSso() {
    volatile uint32_t *control;
    control = FIND_SPI_REG(AVALON_SPI_CONTROL);
    *control &= ~AVALON_SPI_SSO;
}

ErrCode FstSpi::setEopValue(uint32_t len) {
    volatile uint32_t *eop_value;
    eop_value = FIND_SPI_REG(AVALON_SPI_EOP_VALUE);

    volatile uint32_t *slave;
    slave = FIND_SPI_REG(AVALON_SPI_SLAVESELECT);

    volatile uint32_t *control;
    control = FIND_SPI_REG(AVALON_SPI_CONTROL);

    if((*slave != 0) & ((*control & AVALON_SPI_SSO) != 0)) {
        return FST_SPI_EOP_VALUE_F;
    }

    *eop_value = len;
    return FST_SPI_OK;
}

uint32_t FstSpi::getEopValue() {
    return *FIND_SPI_REG(AVALON_SPI_EOP_VALUE);
}

ErrCode FstSpi::selectSlaveN(uint32_t slave) {
    ErrCode ret;
    ret = FST_SPI_OK;

    *FIND_SPI_REG(AVALON_SPI_SLAVESELECT) = slave;

    return ret;
}

void FstSpi::releaseSlaveN() {
    *FIND_SPI_REG(AVALON_SPI_SLAVESELECT) = 0;
}

uint32_t FstSpi::getStatus() {
    volatile uint32_t status;
    status = *FIND_SPI_REG(AVALON_SPI_STATUS);
//    std::cout << "status: " << status << std::endl;
    return status;
}

void FstSpi::printAllRegs() {
    std::cout<< "rxdat = " << *FIND_SPI_REG(AVALON_SPI_RXDATA) << std::endl;
    std::cout<< "txdat = " << *FIND_SPI_REG(AVALON_SPI_TXDATA) << std::endl;
    std::cout<< "status = " << *FIND_SPI_REG(AVALON_SPI_STATUS) << std::endl;
    std::cout<< "control = " << *FIND_SPI_REG(AVALON_SPI_CONTROL) << std::endl;
    std::cout<< "slaveselect = " << *FIND_SPI_REG(AVALON_SPI_SLAVESELECT) << std::endl;
    std::cout<< "eop_value = " << *FIND_SPI_REG(AVALON_SPI_EOP_VALUE) << std::endl;
}

ErrCode FstSpi::transferData(uint8_t *tx, uint8_t *rx, uint32_t len) {
    ErrCode ret;
    ret = FST_SPI_OK;

    ret = setEopValue(len);
    if(ret == FST_SPI_OK) {
        setSso();
        ret = selectSlaveN(FST_SPI_SLAVE_1);
    }

    if(ret == FST_SPI_OK) {        
        volatile uint32_t *txdata;
        txdata = FIND_SPI_REG(AVALON_SPI_TXDATA);
    //    std::cout<< "txdata: "<< txdata << std::endl;

        volatile uint32_t *rxdata;
        rxdata = FIND_SPI_REG(AVALON_SPI_RXDATA);
    //    std::cout<< "rxdata: "<< rxdata << std::endl;

        uint8_t *tx_p;
        tx_p = tx;
        uint8_t *rx_p;
        rx_p = rx;

        uint32_t cnt = 0;

        // while(!isEndOfPacket()) {
        while(cnt < len) {
            while(!isTxReady());
            if(isTxReady()) {
       //         std::cout<<"Write"<<std::endl;
                getStatus();
                *txdata = *tx_p;
                ++tx_p;
                ++cnt;
            }

            while(!isRxReady()){
       //         std::cout<<"Idle"<<std::endl;
                getStatus();
                };
            if(isRxReady()) {
       //         std::cout<<"Read"<<std::endl;
                getStatus();
                *rx_p = *rxdata;
                getStatus();
                ++rx_p;
            }

        }
    }

    releaseSlaveN();
    clearSso();

    return ret;
}