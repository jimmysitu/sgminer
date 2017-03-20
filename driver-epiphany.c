/*
 * Copyright 2013-2013 Rafael Waldo Delgado Doblas
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the Free
 * Software Foundation; either version 3 of the License, or (at your option)
 * any later version.  See COPYING for more details.
 */

#include "compat.h"
#include "miner.h"
#include "config_parser.h"
#include "driver-epiphany.h"
#include "findnonce.h"
#include "util.h"


/* TODO: cleanup externals ********************/
extern void submit_work_async(struct work *work_in, struct timeval *tv_work_found);
extern int dev_from_id(int thr_id);

void print_epi_ndevs(int *ndevs)
{
  opt_verbose = true;
  epiphany_drv.drv_detect();
  applog(LOG_INFO, "%i EPI devices max detected", *ndevs);
}

struct cgpu_info epis[MAX_EPIDEVICES]; /* Maximum number apparently possible */
struct cgpu_info *cpus;

struct epiphany_thread_data {
  int(*queue_kernel_parameters)(e_epiphany_t *, dev_blk_ctx *, unsigned, unsigned);
  uint32_t *res;
};

static void epiphany_detect()
{
	e_platform_t platform;

	if (e_init(NULL) == E_ERR)
		return;

	if (e_reset_system() == E_ERR)
		return;

	if (e_get_platform_info(&platform) == E_ERR)
		return;

	struct cgpu_info *cgpu = malloc(sizeof(struct cgpu_info));

	if (unlikely(!cgpu))
		quit(1, "Failed to malloc epiphany");

  nDevs = 1;    // Support only 1 epiphany for now
	cgpu->drv = &epiphany_drv;
	cgpu->deven = DEV_ENABLED;
	cgpu->threads = 1;
	cgpu->epiphany_rows = platform.rows;
	cgpu->epiphany_cols = platform.cols;
    
  cgpu->virtual_epi = 0;
  cgpu->algorithm = default_profile.algorithm;
	add_cgpu(cgpu);

}

static bool epiphany_thread_prepare(struct thr_info *thr)
{
  struct timeval now;
  struct cgpu_info *cgpu = thr->cgpu;

  int i = thr->id;
  int epi = cgpu->device_id;
  int virtual_epi = cgpu->virtual_epi;
	e_epiphany_t *dev = &cgpu->epiphany_dev;
	e_mem_t *emem = &cgpu->epiphany_emem;
	unsigned rows = cgpu->epiphany_rows;
	unsigned cols = cgpu->epiphany_cols;

	char *source_fullpath = alloca(PATH_MAX);
	char *kernel_fullpath = alloca(PATH_MAX);
	char source_filename[256];
	char kernel_filename[256];
	char compiler_cmd[512];
  FILE* checkf;

  // Allocate a share buffer with Epiphany device
	if (e_alloc(emem, SHARED_DRAM, rows * cols * sizeof(shared_buf_t)) == E_ERR) {
		applog(LOG_ERR, "Error: Could not alloc shared Epiphany memory.");
		return false;
	}

	if (e_open(dev, 0, 0, rows, cols) == E_ERR) {
		applog(LOG_ERR, "Error: Could not start Epiphany cores.");
		return false;
	}
	if (e_reset_group(dev) != E_OK) {
		applog(LOG_ERR, "Error: Could not reset Epiphany working group.");
  }

  // Check kernel source code
  sprintf(source_filename, "%s.epi.c", (!empty_string(epis[virtual_epi].algorithm.kernelfile) ? epis[virtual_epi].algorithm.kernelfile : epis[virtual_epi].algorithm.name));

  applog(LOG_DEBUG, "Using source file %s", source_filename);
	strcpy(source_fullpath, sgminer_path);
	strcat(source_fullpath, "/kernel/");
	strcat(source_fullpath, source_filename);
	checkf = fopen(source_fullpath, "r");
	if (!checkf) {
		thr->cgpu->status = LIFE_SICK;
		applog(LOG_ERR, "Error: Could not find epiphany source: %s", source_fullpath);
		return false;
	}
	fclose(checkf);

  char *esdk;
  esdk = getenv("EPIPHANY_HOME");
  if(!esdk){
    applog(LOG_ERR, "Environment variable ${EPIPHANY_HOME} not found");
		return false;
  }

  sprintf(kernel_filename, "%s.elf", (!empty_string(epis[virtual_epi].algorithm.kernelfile) ? epis[virtual_epi].algorithm.kernelfile : epis[virtual_epi].algorithm.name));
  strcpy(kernel_fullpath, sgminer_path);
  strcat(kernel_fullpath, "/kernel/");
  strcat(kernel_fullpath, kernel_filename);
  sprintf(compiler_cmd, "e-gcc -D GAP=%d -O2 -le-lib -T %s/bsps/current/internal.ldf -o %s %s", GAP, esdk, kernel_fullpath, source_fullpath);

  applog(LOG_DEBUG, "Compiling EPI kernel file\n\t%s", compiler_cmd);
  system(compiler_cmd);

  applog(LOG_INFO, "Init EPI thread %i EPI %i virtual EPI %i", i, epi, virtual_epi);
  applog(LOG_DEBUG, "Using EPI kernel file %s", kernel_filename);

	checkf = fopen(kernel_fullpath, "r");
	if (!checkf) {
		thr->cgpu->status = LIFE_SICK;
		applog(LOG_ERR, "Error: Could not find epiphany kernel: %s", kernel_fullpath);
		return false;
	}
	fclose(checkf);

	if (e_load_group(kernel_fullpath, dev, 0, 0, rows, cols, E_FALSE) == E_ERR) {
		applog(LOG_ERR, "Error: Could not load %s on Epiphany.", kernel_fullpath);
		return false;
	}
	if (e_start_group(dev) != E_OK) {
		applog(LOG_ERR, "Error: Could not start Epiphany working group.");
  }

  cgtime(&now);
  get_datestamp(cgpu->init, sizeof(cgpu->init), &now);

	return true;
}

static bool epiphany_thread_init(struct thr_info *thr)
{
  const int thr_id = thr->id;
  struct cgpu_info *cgpu = thr->cgpu;
  struct epiphany_thread_data *thrdata;
  thrdata = (struct epiphany_thread_data *)calloc(1, sizeof(*thrdata));
  thr->cgpu_data = thrdata;
  int buffersize = BUFFERSIZE;
  
  thrdata->queue_kernel_parameters = cgpu->algorithm.queue_kernel;
  
  if (!thrdata) {
    applog(LOG_ERR, "Failed to calloc in epiphany_thread_init");
    return false;
  }

  thrdata->queue_kernel_parameters = cgpu->algorithm.queue_kernel;
  thrdata->res = (uint32_t *)calloc(buffersize, 1);

  if (!thrdata->res) {
    free(thrdata);
    applog(LOG_ERR, "Failed to calloc in epiphany_thread_init");
    return false;
  }

  cgpu->status = LIFE_WELL;

  cgpu->device_last_well = time(NULL);

  return true;
}

static bool epiphany_prepare_work(struct thr_info __maybe_unused *thr, struct work *work)
{
  work->blk.work = work;
  if (work->pool->algorithm.prepare_work) work->pool->algorithm.prepare_work(&work->blk, (uint32_t *)(work->midstate), (uint32_t *)(work->data));
  thr->pool_no = work->pool->pool_no;
  return true;
}

static int64_t epiphany_scanhash(struct thr_info *thr, struct work *work,
  int64_t __maybe_unused max_nonce)
{
  struct cgpu_info *cgpu = thr->cgpu;
	const int thr_id = thr->id;
  struct epiphany_thread_data *thrdata = (struct epiphany_thread_data *)thr->cgpu_data;
	e_epiphany_t *dev = &cgpu->epiphany_dev;
	unsigned rows = cgpu->epiphany_rows;
	unsigned cols = cgpu->epiphany_cols;
  uint32_t found_idx = cgpu->algorithm.found_idx;
  int buffersize = BUFFERSIZE;

  uint32_t first_nonce = work->blk.nonce;
	uint32_t last_nonce;
	int status;

  int64_t hashes;

  status = thrdata->queue_kernel_parameters(dev, &work->blk, rows, cols);
  if (unlikely(status != 0)) {
    applog(LOG_ERR, "Error queue_kernel_parameters failed.");
    return -1;
  }

  int i, j;
  uint8_t start = 1;
  for(i = 0; i < rows; i++){
    for(j = 0; j < cols; j++){
      e_write(dev, i, j, 0x7104, &first_nonce, sizeof(uint32_t));   // offset
      e_write(dev, i, j, 0x710C, &start, sizeof(uint8_t));          // start
    }
  }
  applog(LOG_DEBUG, "[EPI] Started %d*%d e-cores.", i, j);
  applog(LOG_DEBUG, "[EPI] Started with offset 0x%08x.", first_nonce);

  thrdata->res[found_idx] = 0;
  uint32_t nonce;
  uint32_t working = (1 << (rows*cols)) - 1;
  while(working){
    for(i = 0; i < rows; i++){
      for(j = 0; j < cols; j++){
        if(e_read(dev, i, j, 0x710C, &start, sizeof(uint8_t)) != sizeof(uint8_t))   // check if stop
          applog(LOG_ERR, "[EPI] Failed to read start flag on e-core (%d,%d)", i, j);

        if(!start){
          if(working != (working & (~(1 << (i * cols + j))))){
            working = working & (~(1 << (i * cols + j)));
            applog(LOG_DEBUG, "[EPI] All job on e-core (%d, %d) done, working cores: %x", i, j, working);
          }
          uint8_t found = 0;
          e_read(dev, i, j, 0x710D, &found, sizeof(uint8_t));         // check if found
          e_read(dev, i, j, 0x7108, &last_nonce, sizeof(uint32_t));   // get last nonce before stop 
          if(found){
            thrdata->res[thrdata->res[found_idx]] = last_nonce;  // get golden nonce 
            thrdata->res[found_idx]++;
            applog(LOG_DEBUG, "[EPI] e-core (%d, %d) found something", i, j);
            goto found_nonces; 
          }
        }
      }
    }
  } // while

found_nonces:
  /* found entry is used as a counter to say how many nonces exist */
  if (thrdata->res[found_idx]) {
    applog(LOG_DEBUG, "EPI %d found something?", cgpu->device_id);
    postcalc_hash_async(thr, work, thrdata->res);
    memset(thrdata->res, 0, buffersize);
  }

  work->blk.nonce += (1 << GAP) * 16;
  hashes = (1 << GAP) * 16;
	return hashes;
}

static void epiphany_thread_shutdown(__maybe_unused struct thr_info *thr)
{
	e_epiphany_t *dev = &thr->cgpu->epiphany_dev;
	e_mem_t *emem = &thr->cgpu->epiphany_emem;

	e_close(dev);
	e_free(emem);
	e_finalize();

}

static uint64_t epiphany_can_limit_work(struct thr_info __maybe_unused *thr)
{
	return 0x1ff;
}

struct device_drv epiphany_drv = {
	/*.drv_id = */              DRIVER_epiphany,
	/*.dname = */               "epiphany",
	/*.name = */                "EPI",
	/*.drv_detect = */          epiphany_detect,
  /*.reinit_device = */       NULL,
  /*.get_statline_before = */ NULL,
  /*.get_statline = */        NULL,
  /*.api_data = */            NULL,
  /*.get_stats = */           NULL,
  /*.identify_device = */     NULL,
  /*.set_device = */          NULL,
	/*.thread_prepare = */      epiphany_thread_prepare,
  /*.can_limit_work = */      epiphany_can_limit_work,
  /*.thread_init = */         epiphany_thread_init,
  /*.prepare_work = */        epiphany_prepare_work,
  /*.hash_work = */           NULL,
	/*.scanhash = */            epiphany_scanhash,
  /*.scanwork = */            NULL,
  /*.queue_full = */          NULL,
  /*.flush_work = */          NULL,
  /*.update_work = */         NULL,
  /*.hw_error = */            NULL,
	/*.thread_shutdown = */     epiphany_thread_shutdown,
  /*.thread_enable =*/        NULL,
  false,
  0,
  0
};





