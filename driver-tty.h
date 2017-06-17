#ifndef DEVICE_TTY_H
#define DEVICE_TTY_H

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

extern void print_tty_ndevs(int *ndevs);

extern struct device_drv tty_drv;



#endif /* __DEVICE_TTY_H__ */
