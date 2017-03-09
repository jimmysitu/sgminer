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

#include <e-hal.h>
#include <e-loader.h>

extern void print_epi_ndevs(int *ndevs);

#define SHARED_DRAM (0x01000000)
extern struct device_drv epiphany_drv;

typedef struct _shared_buf_t {
	uint8_t data[256];
  uint64_t target;
  uint32_t offset;                // work start offset
	volatile uint32_t nonce;
	volatile uint8_t start;         // Start flag, set by host
	volatile uint8_t found;         // Nonce found flag, set by device
	volatile uint8_t done;          // Hash job done flag, set by device
} shared_buf_t;


#endif /* __DEVICE_EPI_H__ */
