#pragma once
#include "v8stdint.h"
#include <sys/time.h>
#include <time.h>
#include <unistd.h>

static inline void delay(uint32_t ms){
	while (ms>=1000){
		usleep(1000*1000);
		ms-=1000;
	};
	if (ms!=0){
		usleep(ms*1000);
	}
}


namespace impl{
	uint64_t f_getus();
	uint32_t f_getms();
}

#define getms() impl::f_getms()
