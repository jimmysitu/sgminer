/*
 * Copyright 2013-2013 Rafael Waldo Delgado Doblas
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the Free
 * Software Foundation; either version 3 of the License, or (at your option)
 * any later version.  See COPYING for more details.
 */

#include "config_parser.h"
#include "driver-epiphany.h"


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

	char *fullpath = alloca(PATH_MAX);
	char filename[256];

	if (e_alloc(emem, _BufOffset, rows * cols * sizeof(shared_buf_t)) == E_ERR) {
		applog(LOG_ERR, "Error: Could not alloc shared Epiphany memory.");
		return false;
	}

	if (e_open(dev, 0, 0, rows, cols) == E_ERR) {
		applog(LOG_ERR, "Error: Could not start Epiphany cores.");
		return false;
	}
  
  // TODO: Jimmy, add compile flow here
  
  applog(LOG_INFO, "Init EPI thread %i EPI %i virtual GPU %i", i, epi, virtual_epi);

  sprintf(filename, "%s.srec", (!empty_string(epis[virtual_epi].algorithm.kernelfile) ? epis[virtual_epi].algorithm.kernelfile : epis[virtual_epi].algorithm.name));
  applog(LOG_DEBUG, "Using source file %s", filename);

	strcpy(fullpath, sgminer_path);
	strcat(fullpath, filename);
	FILE* checkf = fopen(fullpath, "r");
	if (!checkf) {
		thr->cgpu->status = LIFE_SICK;
		applog(LOG_ERR, "Error: Could not find epiphany kernel: %s", fullpath);
		return false;
	}
	fclose(checkf);

	if (e_load_group(fullpath, dev, 0, 0, rows, cols, E_FALSE) == E_ERR) {
		applog(LOG_ERR, "Error: Could not load %s on Epiphany.", fullpath);
		return false;
	}

  cgtime(&now);
  get_datestamp(cgpu->init, sizeof(cgpu->init), &now);

	return true;
}

static bool epiphany_thread_init(struct thr_info *thr)
{
  const int thr_id = thr->id;
  struct cgpu_info *gpu = thr->cgpu;
  
  // TODO: do some initial here

  gpu->status = LIFE_WELL;

  gpu->device_last_well = time(NULL);

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
	const int thr_id = thr->id;
	unsigned char hash1[64];
	uint32_t first_nonce = work->blk.nonce;
	uint32_t last_nonce;
	bool rc;

  int64_t hashes = 100;

  //TODO: Add real hash code here
  
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





