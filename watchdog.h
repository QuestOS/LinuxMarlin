#ifndef WATCHDOG_H
#define WATCHDOG_H

#include "Marlin.h"

//If we do not have a watchdog, then we can have empty functions which are optimized away.
FORCE_INLINE void watchdog_init() {};
FORCE_INLINE void watchdog_reset() {};

#endif
