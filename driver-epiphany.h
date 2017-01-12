#ifndef DEVICE_EPI_H
#define DEVICE_EPI_H

#include "miner.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <stdint.h>
#include <unistd.h>
#include <signal.h>

#include <sys/stat.h>
#include <sys/types.h>

#ifndef WIN32
  #include <sys/wait.h>
  #include <sys/resource.h>
#endif

#include <libgen.h>

#include "compat.h"
#include "miner.h"
#include "bench_block.h"

#if defined(unix)
	#include <errno.h>
	#include <fcntl.h>
#endif

#ifdef USE_EPIPHANY
  #include <e-hal.h>
  #define _BufOffset (0x01000000)
  extern struct device_drv epiphany_drv;
#endif

#endif /* __DEVICE_EPI_H__ */
