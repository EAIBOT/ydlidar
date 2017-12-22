#if !defined(_WIN32)
#include "unix_timer.h"

namespace impl{
	uint64_t f_getus() {
		struct timespec t;
		t.tv_sec = t.tv_nsec = 0;
		clock_gettime(CLOCK_MONOTONIC, &t);
		return t.tv_sec*1000000LL + t.tv_nsec/1000;
	}
	uint32_t f_getms() {
		struct timespec t;
		t.tv_sec = t.tv_nsec = 0;
		clock_gettime(CLOCK_MONOTONIC, &t);
		return t.tv_sec*1000L + t.tv_nsec/1000000L;
	}
}
#endif