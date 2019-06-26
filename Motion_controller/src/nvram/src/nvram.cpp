//-----------------------------------------------------------------------------
// Copyright(c) 2019-2050 Foresight-Robotics
// All rights reserved.
//
// File: nvram.cpp
// Version: 0.2.01
// 
// Create: Jan.07 2019
// Author: Shuguo, Zhang
// Summary: 
//-----------------------------------------------------------------------------
// defined(_GLIBCXX_HAS_GTHREADS) && defined(_GLIBCXX_USE_C99_STDINT_TR1)
#ifndef _GLIBCXX_HAS_GTHREADS
#define _GLIBCXX_HAS_GTHREADS
#endif //_GLIBCXX_HAS_GTHREADS

#ifndef _GLIBCXX_USE_C99_STDINT_TR1
#define _GLIBCXX_USE_C99_STDINT_TR1
#endif //_GLIBCXX_USE_C99_STDINT_TR1

#include "nvram.h"
#include <stdlib.h>
#include <time.h>
#include <thread>
#include <utility>
#include <pthread.h>

bool Nvram::is_Checked_Ready_ = false;

Nvram::Nvram(uint32_t length) {
    data_len_default_ = length;
    additional_len_ = sizeof(NvramData);
}

Nvram::~Nvram() {
    delete[] xmit_;
    delete[] recv_;
}

ErrCode Nvram::openNvram() {
    spi_ = FstSpi::getInstance();
    
    xmit_ = (NvramData *)new uint8_t[data_len_default_ + additional_len_];
    if(xmit_ == NULL) {
        return FST_NVRAM_ALLOC_F;
    }
    recv_ = (NvramData *)new uint8_t[data_len_default_ + additional_len_];
    if(recv_ == NULL) {
        return FST_NVRAM_ALLOC_F;
    }
	return FST_NVRAM_OK ;
}

void Nvram::setAddress(uint32_t addr) {
    xmit_->address[0] = (uint8_t)(addr >> 16);
    xmit_->address[1] = (uint8_t)(addr >> 8);
    xmit_->address[2] = (uint8_t)addr;
}

uint8_t Nvram::bcc(uint8_t* data, uint32_t length) {
    uint8_t sum;
    sum = 0;

    uint32_t cnt;
    for(cnt = 0; cnt < length; ++cnt)
    {
        /* code */
        sum ^= data[cnt];
    }
    
    return sum;
}

uint8_t Nvram::checkSumOfData(uint8_t *data, uint32_t length) {
    return bcc(data, length);
}

ErrCode Nvram::isNvramReady() {
    ErrCode ret;
    ret = FST_NVRAM_OK;

    if(is_Checked_Ready_) {
        return ret;
    }

    srand((int)time(0));
    uint32_t input_random = (uint32_t)(rand()%1000);
    uint32_t output_random = input_random + 0xa;
    
    uint32_t temp_data_len = data_len_;
    data_len_ = sizeof(uint32_t);

    xmit_->instruction = NVRAM_INS_WRITE;
    setAddress((uint32_t)POWER_ON_TEST);
    *(uint32_t *)xmit_->data = input_random;
    
    if(spi_->trylockSpi()) {
        spi_->transferData((uint8_t *)xmit_, (uint8_t *)recv_, data_len_ + additional_len_);
        xmit_->instruction = NVRAM_INS_READ;
        spi_->transferData((uint8_t *)xmit_, (uint8_t *)recv_, data_len_ + additional_len_);
        spi_->unlockSpi();
    }else
    {
        ret = FST_NVRAM_R_TIMEOUT_F;
    }
    
    if(*(uint32_t *)recv_->data != input_random) {
        ret = FST_NVRAM_NOT_READY_F;
    }
    else
    {
        is_Checked_Ready_ = true;
    }

    data_len_ = temp_data_len;
    return ret;
}

ErrCode Nvram::read(uint8_t* dest, uint32_t addr, uint32_t length) {
    ErrCode ret;
    ret = FST_NVRAM_OK;

    data_len_ = length;
    address_ = addr;

    uint32_t len;
    len = data_len_ + additional_len_;

    xmit_->instruction = NVRAM_INS_READ;
    setAddress(address_);
    
    if(!spi_->trylockSpi()) {
        ret = FST_NVRAM_R_TIMEOUT_F;
    }

    if (ret == FST_NVRAM_OK) {
        spi_->transferData((uint8_t *)xmit_, (uint8_t *)recv_, len);
        spi_->unlockSpi();
        for(uint32_t cnt = 0; cnt < data_len_; ++cnt)
        {
            dest[cnt] = recv_->data[cnt];
        }
    }
    
    return ret;
}

void Nvram::writeRoutine() {
    uint32_t len;
    len = data_len_ + additional_len_;

    if(spi_->trylockSpi()) {
        spi_->transferData((uint8_t *)xmit_, (uint8_t *)recv_, len);
        setAddress((uint32_t)NVRAM_BLK_2_BASE + address_);
        spi_->transferData((uint8_t *)xmit_, (uint8_t *)recv_, len);
        spi_->unlockSpi();
    }
	else  {
	    printf("writeRoutine spi_->trylockSpi() failed \n");
    }
}

bool Nvram::writeSync(uint8_t *src, uint32_t addr, uint32_t length) {
    data_len_ = length;
    address_ = addr;

    for(uint32_t cnt = 0; cnt < data_len_; ++cnt)
    {
        xmit_->data[cnt] = src[cnt];
    }

    xmit_->instruction = NVRAM_INS_WRITE;
    setAddress(address_);

//    std::thread w_routin(&Nvram::writeRoutine, this);
//    w_routin.detach();
    uint32_t len;
    len = data_len_ + additional_len_;

    if(spi_->trylockSpi()) {
        spi_->transferData((uint8_t *)xmit_, (uint8_t *)recv_, len);
        setAddress((uint32_t)NVRAM_BLK_2_BASE + address_);
        spi_->transferData((uint8_t *)xmit_, (uint8_t *)recv_, len);
        spi_->unlockSpi();
		return true ;
    }
	else  {
	    printf("writeSync spi_->trylockSpi() failed \n");
		return false ;
    }
}

void Nvram::write(uint8_t *src, uint32_t addr, uint32_t length) {
    data_len_ = length;
    address_ = addr;

    for(uint32_t cnt = 0; cnt < data_len_; ++cnt)
    {
        xmit_->data[cnt] = src[cnt];
    }

    xmit_->instruction = NVRAM_INS_WRITE;
    setAddress(address_);

    std::thread w_routin(&Nvram::writeRoutine, this);
    w_routin.detach();
}
