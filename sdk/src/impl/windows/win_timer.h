#pragma once
#include <windows.h>
#include "v8stdint.h"

#define BEGIN_STATIC_CODE( _blockname_ ) \
	static class _static_code_##_blockname_ {   \
	public:     \
	_static_code_##_blockname_ ()


#define END_STATIC_CODE( _blockname_ ) \
	}   _instance_##_blockname_;

#define delay(x)   ::Sleep(x)

namespace impl{
	void HPtimer_reset();
	uint32_t getHDTimer(uint32_t div = 1000);
}

#define getms()   impl::getHDTimer()
#define getus()   impl::getHDTimer(1)

