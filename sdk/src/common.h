#pragma once

#if defined(_WIN32)
#include "impl\windows\win.h"
#include "impl\windows\win_serial.h"
#include "impl\windows\win_timer.h"
#elif defined(__GNUC__)
#include "impl/unix/unix.h"
#include "impl/unix/unix_serial.h"
#include "impl/unix/unix_timer.h"
#else
#error "unsupported target"
#endif

#include "locker.h"
#include "serial.h"
#include "thread.h"

#define SDKVerision "1.2.3"
