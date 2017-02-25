/*
 * Copyright 2013-2013 Rafael Waldo Delgado Doblas
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the Free
 * Software Foundation; either version 3 of the License, or (at your option)
 * any later version.  See COPYING for more details.
 */

#include "driver-epiphany.h"

#ifdef USE_EPIPHANY

/* TODO: cleanup externals ********************/
extern void submit_work_async(struct work *work_in, struct timeval *tv_work_found);
extern int dev_from_id(int thr_id);



static void epiphany_detect()
{
	e_platform_t platform;

	if (e_init(NULL) == E_ERR)
		return;

	if (e_reset_system() == E_ERR)
		return;

	if (e_get_platform_info(&platform) == E_ERR)
		return;

	struct cgpu_info *epiphany = malloc(sizeof(struct cgpu_info));

	if (unlikely(!epiphany))
		quit(1, "Failed to malloc epiphany");

	epiphany->drv = &epiphany_drv;
	epiphany->deven = DEV_ENABLED;
	epiphany->threads = 1;
	epiphany->epiphany_rows = platform.rows;
	epiphany->epiphany_cols = platform.cols;
	add_cgpu(epiphany);

}

static bool epiphany_thread_prepare(struct thr_info *thr)
{
	e_epiphany_t *dev = &thr->cgpu->epiphany_dev;
	e_mem_t *emem = &thr->cgpu->epiphany_emem;
	unsigned rows = thr->cgpu->epiphany_rows;
	unsigned cols = thr->cgpu->epiphany_cols;

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

  sprintf(filename, "%s.srec", (!empty_string(cgpu->algorithm.kernelfile) ? cgpu->algorithm.kernelfile : cgpu->algorithm.name));
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

	thread_reportin(thr);

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
static bool epiphany_scrypt(struct thr_info *thr, const unsigned char __maybe_unused *pmidstate,
		     unsigned char *pdata, unsigned char __maybe_unused *phash1,
		     unsigned char __maybe_unused *phash, const unsigned char *ptarget,
		     uint32_t max_nonce, uint32_t *last_nonce, uint32_t n)
{

	uint32_t i;
	e_epiphany_t *dev = &thr->cgpu->epiphany_dev;
	e_mem_t *emem = &thr->cgpu->epiphany_emem;
	unsigned rows = thr->cgpu->epiphany_rows;
	unsigned cols = thr->cgpu->epiphany_cols;

	uint8_t *core_working = calloc(rows*cols, sizeof(uint8_t));
	uint32_t *core_nonce = calloc(rows*cols, sizeof(uint32_t));
	uint32_t cores_working = 0;

	uint32_t *nonce = (uint32_t *)(pdata + 76);

	uint32_t ostate;
	uint32_t data[20];
	const uint8_t core_go = 1;

	uint32_t tmp_hash7;
	uint32_t Htarg = ((const uint32_t *)ptarget)[7];
	bool ret = false;

	be32enc_vect(data, (const uint32_t *)pdata, 19);

	if (e_start_group(dev) == E_ERR) {
		applog(LOG_ERR, "Error: Could not start Epiphany cores.");
		return false;
	}

	off_t offdata = offsetof(shared_buf_t, data);
	off_t offostate = offsetof(shared_buf_t, ostate);
	off_t offcorego = offsetof(shared_buf_t, go);
	off_t offcoreend = offsetof(shared_buf_t, working);
	off_t offcore;

#ifdef EPIPHANY_DEBUG
	#define SCRATCHBUF_SIZE	(131584)
	uint32_t ostate2[8];
	char *scratchbuf = malloc(SCRATCHBUF_SIZE);
	uint32_t *ostatearm = calloc(rows*cols, sizeof(uint32_t));
#endif

	i = 0;
	while(1) {

		offcore = i * sizeof(shared_buf_t);

		if ((!core_working[i]) && (n < max_nonce)) {

			*nonce = ++n;
			data[19] = n;
			core_working[i] = 1;
			cores_working++;
			core_nonce[i] = n;

#ifdef EPIPHANY_DEBUG
			scrypt_1024_1_1_256_sp(data, scratchbuf, ostate2);
			ostatearm[i] = ostate2[7];
#endif

			e_write(emem, 0, 0, offcore + offdata, (void *) data, sizeof(data));
			e_write(emem, 0, 0, offcore + offcoreend, (void *) &core_working[i], sizeof(core_working[i]));
			e_write(emem, 0, 0, offcore + offcorego, (void *) &core_go, sizeof(core_go));

		}

		e_read(emem, 0, 0, offcore + offcoreend, (void *) &(core_working[i]), sizeof(core_working[i]));

		if (!core_working[i]) {

			e_read(emem, 0, 0, offcore + offostate, (void *) &(ostate), sizeof(ostate));
#ifdef EPIPHANY_DEBUG
			applog(LOG_DEBUG, "CORE %u - EPI HASH %u - ARM HASH %u - %s", i, ostate, ostatearm[i], ((ostate==ostatearm[i])? "OK":"FAIL"));
#endif
			tmp_hash7 = be32toh(ostate);
			cores_working--;
			if (unlikely(tmp_hash7 <= Htarg)) {
				((uint32_t *)pdata)[19] = htobe32(core_nonce[i]);
				*last_nonce = core_nonce[i];
#ifdef EPIPHANY_DEBUG
				free(scratchbuf);
#endif
				return true;
			}

		}

		if (unlikely(((n >= max_nonce) && !cores_working) || thr->work_restart)) {
			*last_nonce = n;
#ifdef EPIPHANY_DEBUG
			free(scratchbuf);
#endif
			return false;
		}

		i++;
		i %= rows * cols;

	}

#ifdef EPIPHANY_DEBUG
	free(scratchbuf);
#endif

	return false;
}

static int64_t epiphany_scanhash(struct thr_info *thr, struct work *work,
  int64_t __maybe_unused max_nonce)
{
	const int thr_id = thr->id;
	unsigned char hash1[64];
	uint32_t first_nonce = work->blk.nonce;
	uint32_t last_nonce;
	bool rc;

	hex2bin(hash1, "00000000000000000000000000000000000000000000000000000000000000000000008000000000000000000000000000000000000000000000000000010000", 64);
EPIPHANYSearch:
	last_nonce = first_nonce;
	rc = false;

	/* scan nonces for a proof-of-work hash */
	{
		rc = epiphany_scrypt(
			thr,
			work->midstate,
			work->data,
			hash1,
			work->hash,
			work->target,
			max_nonce,
			&last_nonce,
			work->blk.nonce
		);
	}
	/* if nonce found, submit work */
	if (unlikely(rc)) {
		applog(LOG_DEBUG, "EPIPHANY %d found something?", dev_from_id(thr_id));
		submit_work_async(work, NULL);
		work->blk.nonce = last_nonce + 1;
		goto EPIPHANYSearch;
	}
	else
	if (unlikely(last_nonce == first_nonce))
		return 0;

	work->blk.nonce = last_nonce + 1;
	return last_nonce - first_nonce + 1;
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


#endif



