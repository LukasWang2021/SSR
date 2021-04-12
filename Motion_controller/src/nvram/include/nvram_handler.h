/*************************************************************************
	> File Name: nvram_handler.h
	> Author: 
	> Mail: 
	> Created Time: 2020年07月03日 星期五 13时41分54秒
 ************************************************************************/

#ifndef _NVRAM_HANDLER_H
#define _NVRAM_HANDLER_H

#include "nvram.h"

namespace rtm_nvram
{

class NvramHandler
{
public:
	NvramHandler();
	~NvramHandler();
	bool init(void);
	bool readNvram(uint32_t address, uint8_t *data, uint32_t length);
	bool writeNvram(uint32_t address, const uint8_t *data, uint32_t length);
private:
	NvramImage *image_ptr_;
};

}
#endif
