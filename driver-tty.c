/*
 * Copyright 2013-2013 Rafael Waldo Delgado Doblas
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the Free
 * Software Foundation; either version 3 of the License, or (at your option)
 * any later version.  See COPYING for more details.
 */
#include "config.h"

#ifdef HAVE_CURSES
#include <curses.h>
#endif

#include <string.h>
#include <stdbool.h>
#include <stdint.h>
#include <signal.h>
#include <sys/types.h>

#ifndef WIN32
#include <sys/resource.h>
#endif
#include <ccan/opt/opt.h>


#include "compat.h"
#include "miner.h"
#include "config_parser.h"
#include "driver-tty.h"
#include "findnonce.h"
#include "util.h"


/* TODO: cleanup externals ********************/

#ifdef HAVE_CURSES
extern WINDOW *mainwin, *statuswin, *logwin;
extern void enable_curses(void);
#endif

extern int ttyr_thr_id;

extern void submit_work_async(struct work *work_in, struct timeval *tv_work_found);
extern int dev_from_id(int thr_id);
extern void *miner_thread(void *userdata);

void print_tty_ndevs(int *ndevs)
{
  opt_verbose = true;
  tty_drv.drv_detect();
  applog(LOG_INFO, "%i tty devices max detected", *ndevs);
}

struct cgpu_info ttys[MAX_TTYDEVICES]; /* Maximum number apparently possible */
struct cgpu_info *cpus;

struct tty_thread_data {
  int(*queue_kernel_parameters)(int *, dev_blk_ctx *);
  uint32_t *res;
};

static bool ttyStates[MAX_TTYDEVICES];

#define BAUDRATE B115200
#define TTYDEVICE "/dev/ttyPS1"

bool initTty(unsigned int tty, char *name, size_t nameSize, algorithm_t *algorithm)
{
	struct cgpu_info *cgpu = &ttys[tty];
	int *dev = &cgpu->tty_dev;

  struct termios options;

  applog(LOG_DEBUG, "initTty() started, selected ttys[%d]", tty);

  *dev = open(TTYDEVICE, O_RDWR | O_NOCTTY | O_NDELAY | O_NONBLOCK);
  if(*dev == -1){
    applog(LOG_ERR, "Failed to open tty device");
		return false;
  }
  //fcntl(*dev, F_SETFL, 0);

  // Get current options
  tcgetattr(*dev, &options);

  // Setting baudrate
  cfsetispeed(&options, BAUDRATE);
  cfsetospeed(&options, BAUDRATE);

  // Enable the receiver and set local mode, 8N1
  options.c_cflag |= (CLOCAL | CREAD);
  options.c_cflag &= ~PARENB;
  options.c_cflag &= ~CSTOPB;
  options.c_cflag &= ~CSIZE;
  options.c_cflag |= CS8;

  // no flow control
  options.c_cflag &= ~CRTSCTS;

  // Misc setting
  options.c_iflag &= ~(BRKINT | ICRNL | IMAXBEL);
  options.c_oflag &= ~(OPOST | ONLCR);
  options.c_lflag &= ~(ISIG | ICANON | IEXTEN | ECHO | ECHOE | ECHOK | ECHOCTL | ECHOKE);

  options.c_cc[VMIN]  = 0;
  options.c_cc[VTIME] = 10;

  // Setting configuration
  tcflush(*dev, TCIFLUSH);
  if(tcsetattr(*dev, TCSANOW, &options) != 0){
    applog(LOG_INFO, "[TTY] initTty() failed to setting configuration with device %d", *dev);
  }
  strcpy(name, TTYDEVICE);
  applog(LOG_INFO, "[TTY] initTty() done with device %d", *dev);
  return true;

}

/* We have only one thread that ever re-initialises TTYs, thus if any TTY
 * init command fails due to a completely wedged TTY, the thread will never
 * return, unable to harm other GPUs. If it does return, it means we only had
 * a soft failure and then the reinit_tty thread is ready to tackle another
 * TTY */
void *reinit_tty(void *userdata)
{
  struct thr_info *mythr = (struct thr_info *)userdata;
  struct cgpu_info *cgpu;
  struct thr_info *thr;
  struct timeval now;
  char name[256];
  int thr_id;
  int tty;

  pthread_detach(pthread_self());
  
  strcpy(name, "");

select_cgpu:
  cgpu = (struct cgpu_info *)tq_pop(mythr->q, NULL);
  if (!cgpu)
    goto out;

  tty = cgpu->device_id;

  rd_lock(&mining_thr_lock);
  for (thr_id = 0; thr_id < mining_threads; ++thr_id) {
    thr = mining_thr[thr_id];
    cgpu = thr->cgpu;
    if (cgpu->drv->drv_id != DRIVER_tty)
      continue;
    if (dev_from_id(thr_id) != tty)
      continue;

    thr->rolling = thr->cgpu->rolling = 0;
    /* Reports the last time we tried to revive a sick TTY */
    cgtime(&thr->sick);
    if (!pthread_kill(thr->pth, 0)) {
      applog(LOG_WARNING, "Thread %d still exists, killing it off", thr_id);
      cg_completion_timeout(&thr_info_cancel_join, thr, 5000);
      thr->cgpu->drv->thread_shutdown(thr);
    }
    else
      applog(LOG_WARNING, "Thread %d no longer exists", thr_id);
  }
  rd_unlock(&mining_thr_lock);

  rd_lock(&mining_thr_lock);
  for (thr_id = 0; thr_id < mining_threads; ++thr_id) {
    int virtual_tty;

    thr = mining_thr[thr_id];
    cgpu = thr->cgpu;
    if (cgpu->drv->drv_id != DRIVER_tty)
      continue;
    if (dev_from_id(thr_id) != tty)
      continue;

    virtual_tty = cgpu->virtual_tty;

    thr->q = tq_new();
    if (!thr->q)
      quit(1, "Failed to tq_new in reinit_tty");

    applog(LOG_INFO, "Reinit TTY thread %d", thr_id);
    ttyStates[thr_id] = initTty(virtual_tty, name, sizeof(name), &cgpu->algorithm);
    if (!ttyStates[thr_id]) {
      applog(LOG_ERR, "Failed to reinit TTY thread %d", thr_id);
      goto select_cgpu;
    }
    applog(LOG_INFO, "initTty() finished. Found %s", name);

    if (unlikely(thr_info_create(thr, NULL, miner_thread, thr))) {
      applog(LOG_ERR, "thread %d create failed", thr_id);
      return NULL;
    }
    applog(LOG_WARNING, "Thread %d restarted", thr_id);
  }
  rd_unlock(&mining_thr_lock);

  cgtime(&now);
  get_datestamp(cgpu->init, sizeof(cgpu->init), &now);

  rd_lock(&mining_thr_lock);
  for (thr_id = 0; thr_id < mining_threads; ++thr_id) {
    thr = mining_thr[thr_id];
    cgpu = thr->cgpu;
    if (cgpu->drv->drv_id != DRIVER_tty)
      continue;
    if (dev_from_id(thr_id) != tty)
      continue;

    cgsem_post(&thr->sem);
  }
  rd_unlock(&mining_thr_lock);

  goto select_cgpu;
out:
  return NULL;
}

static void reinit_tty_device(struct cgpu_info *tty)
{
  tq_push(control_thr[ttyr_thr_id].q, tty);
}

static void tty_detect()
{
  int fd;
  int *dev = &fd;
  struct termios options;

  *dev = open(TTYDEVICE, O_RDWR | O_NOCTTY | O_NDELAY | O_NONBLOCK);
  if(*dev == -1){
    applog(LOG_ERR, "Failed to open tty device");
		return;
  }
  //fcntl(*dev, F_SETFL, 0);

  // Get current options
  tcgetattr(*dev, &options);

  // Setting baudrate
  cfsetispeed(&options, BAUDRATE);
  cfsetospeed(&options, BAUDRATE);

  // Enable the receiver and set local mode, 8N1
  options.c_cflag |= (CLOCAL | CREAD);
  options.c_cflag &= ~PARENB;
  options.c_cflag &= ~CSTOPB;
  options.c_cflag &= ~CSIZE;
  options.c_cflag |= CS8;

  // Misc setting
  options.c_iflag &= ~(BRKINT | ICRNL | IMAXBEL);
  options.c_oflag &= ~(OPOST | ONLCR);
  options.c_lflag &= ~(ISIG | ICANON | IEXTEN | ECHO | ECHOE | ECHOK | ECHOCTL | ECHOKE);

  options.c_cc[VMIN]  = 4;
  options.c_cc[VTIME] = 10;

  // Setting configuration
  //tcsetattr(*dev, TCSANOW, &options);
  tcsetattr(*dev, TCSAFLUSH, &options);

  uint8_t loop_test[4] = {0xAA, 0x01, 0x01, 0x00};
  uint8_t loop_ack[4];

  int t;
  t = write(*dev, loop_test, 4);
  if(4 != t){
    applog(LOG_ERR, "Loop test error, wrote %d", t);
  }

  sleep(2);

  t = read(*dev, loop_ack, 4);
  if(4 != t){
    applog(LOG_ERR, "Loop ack error, read %d", t);
  }

  if(1 != (loop_ack[3] - loop_test[3])){
    applog(LOG_ERR, "Loop test fail, ack btye error\n");
  }

  applog(LOG_DEBUG, "Detected tty device %d", *dev);
  // close tty device
  close(*dev);


  nDevs = 1;    // Support only 1 tty for now

  struct cgpu_info *cgpu;
  cgpu = &ttys[0];
	cgpu->drv = &tty_drv;
	cgpu->deven = DEV_ENABLED;
	cgpu->threads = 1;

  cgpu->virtual_tty = 0;
  cgpu->hw_errors = 0;
  cgpu->algorithm = default_profile.algorithm;
	add_cgpu(cgpu);

}

static bool tty_thread_prepare(struct thr_info *thr)
{
  char name[256];
  struct timeval now;
  struct cgpu_info *cgpu = thr->cgpu;

  int i = thr->id;
  static bool failmessage = false;
  int tty = cgpu->device_id;
  int virtual_tty = cgpu->virtual_tty;
	int *dev = &cgpu->tty_dev;

	char *source_fullpath = alloca(PATH_MAX);
	char *kernel_fullpath = alloca(PATH_MAX);
	char source_filename[256];
	char kernel_filename[256];
	char compiler_cmd[512];

  strcpy(name, "");
  // Open and setting the tty device
  ttyStates[i] = initTty(virtual_tty, name, sizeof(name), &cgpu->algorithm);
  if (!ttyStates[i]){
#ifdef HAVE_CURSES
    if (use_curses)
      enable_curses();
#endif
    applog(LOG_ERR, "Failed to init TTY thread %d, disabling device %d", i, tty);
    if (!failmessage) {
      applog(LOG_ERR, "Restarting the GPU from the menu will not fix this.");
      applog(LOG_ERR, "Re-check your configuration and try restarting.");
      failmessage = true;
#ifdef HAVE_CURSES
      char *buf;
      if (use_curses) {
        buf = curses_input("Press enter to continue");
        if (buf)
          free(buf);
      }
#endif
    }
    cgpu->deven = DEV_DISABLED;
    cgpu->status = LIFE_NOSTART;

    dev_error(cgpu, REASON_DEV_NOSTART);
    return false;
  }

  if (!cgpu->name)
    cgpu->name = TTYDEVICE ;

  applog(LOG_INFO, "initTty() finished. Found %s", name);
  cgtime(&now);
  get_datestamp(cgpu->init, sizeof(cgpu->init), &now);

	return true;
}

static bool tty_thread_init(struct thr_info *thr)
{
  const int thr_id = thr->id;
  struct cgpu_info *cgpu = thr->cgpu;
  struct tty_thread_data *thrdata;
  thrdata = (struct tty_thread_data *)calloc(1, sizeof(*thrdata));
  thr->cgpu_data = thrdata;
  int buffersize = BUFFERSIZE;

  thrdata->queue_kernel_parameters = cgpu->algorithm.queue_kernel;

  if (!thrdata) {
    applog(LOG_ERR, "Failed to calloc in tty_thread_init");
    return false;
  }

  thrdata->queue_kernel_parameters = cgpu->algorithm.queue_kernel;
  thrdata->res = (uint32_t *)calloc(buffersize, 1);

  if (!thrdata->res) {
    free(thrdata);
    applog(LOG_ERR, "Failed to calloc in tty_thread_init");
    return false;
  }

  cgpu->status = LIFE_WELL;

  cgpu->device_last_well = time(NULL);

  return true;
}

static bool tty_prepare_work(struct thr_info __maybe_unused *thr, struct work *work)
{
  work->blk.work = work;
  if (work->pool->algorithm.prepare_work) work->pool->algorithm.prepare_work(&work->blk, (uint32_t *)(work->midstate), (uint32_t *)(work->data));
  thr->pool_no = work->pool->pool_no;
  return true;
}

static int64_t tty_scanhash(struct thr_info *thr, struct work *work,
  int64_t __maybe_unused max_nonce)
{
  struct cgpu_info *cgpu = thr->cgpu;
	const int thr_id = thr->id;
  struct tty_thread_data *thrdata = (struct tty_thread_data *)thr->cgpu_data;
	int *dev = &cgpu->tty_dev;
  uint32_t found_idx = cgpu->algorithm.found_idx;
  int buffersize = BUFFERSIZE;

  uint32_t first_nonce = work->blk.nonce;
	uint32_t last_nonce;
	int status;

  int64_t hashes = 0;

  applog(LOG_DEBUG, "[TTY] tty_scanhash started with device %d", *dev);

  status = thrdata->queue_kernel_parameters(dev, &work->blk);
  if (unlikely(status != 0)) {
    applog(LOG_ERR, "Error queue_kernel_parameters failed.");
    return -1;
  }
  applog(LOG_DEBUG, "[TTY] Started with offset 0x%08x.", first_nonce);

  thrdata->res[found_idx] = 0;
  uint8_t msg[7];

  // Wait and check if any nonce is found
  int cnt = 10;
  while(cnt > 0){
    sleep(1);
    // Read tty device to get the golden nonces
    applog(LOG_DEBUG, "[TTY] tty device try to read 7 bytes");
    int rd = read(*dev, msg, 7);
//    if(rd > 0 & rd < 7){
//      applog(LOG_DEBUG, "[TTY] tty device read %d bytes, read once more", rd);
//      rd = rd + read(*dev, &msg[rd], (7-rd));
//    }

    if(7 == rd){
      applog(LOG_DEBUG, "[TTY] tty device read %d btyes, header: 0x%02X, 0x%02X, 0x%02X", 
             rd, msg[0], msg[1], msg[2]);
      uint32_t nonce = *((uint32_t*)&msg[3]);
      thrdata->res[thrdata->res[found_idx]] = nonce;  // get golden nonce 
      thrdata->res[found_idx]++;
      applog(LOG_DEBUG, "[TTY] tty device found something, nonce: 0x%08x", nonce);

      last_nonce = swab32(nonce);

      hashes = last_nonce - first_nonce;
      goto found_nonces;

    }else if(0 == rd){
      applog(LOG_DEBUG, "[TTY] tty device read %d btyes, waiting %d", rd, cnt);
      hashes = hashes + 20000000;
      cnt--;
    }else{
      applog(LOG_DEBUG, "[TTY] tty device read %d btyes, continue %d", rd, cnt);
      hashes = hashes + 20000000;
      cnt--;
      //break;
    }
  }


found_nonces:
  /* found entry is used as a counter to say how many nonces exist */
  if (thrdata->res[found_idx]) {
    applog(LOG_DEBUG, "TTY %d found something?", cgpu->device_id);
    postcalc_hash_async(thr, work, thrdata->res);
    memset(thrdata->res, 0, buffersize);
  }

  work->blk.nonce += hashes;
  work->blk.nonce += 1;
	return hashes;
}

static void tty_thread_shutdown(__maybe_unused struct thr_info *thr)
{
	int *dev = &thr->cgpu->tty_dev;
  if(-1 == close(*dev)){
    switch(errno){
    case EBADF:
      applog(LOG_DEBUG, "Close tty device %d fail, maybe it is not open", *dev);
      break;
    case EINTR:
      applog(LOG_DEBUG, "Close tty device %d fail, interrupted by a signal", *dev);
      break;
    case EIO:
      applog(LOG_DEBUG, "Close tty device %d fail, I/O error occurred", *dev);
      break;
    default:
      applog(LOG_DEBUG, "Close tty device %d fail, unknown reason", *dev);
    }
  }else{
    applog(LOG_DEBUG, "[TTY] Closed tty device %d", *dev);
  }

  free(((struct tty_thread_data *)thr->cgpu_data)->res);
  free(thr->cgpu_data);
  thr->cgpu_data = NULL;
}

static uint64_t tty_can_limit_work(struct thr_info __maybe_unused *thr)
{
	return 0x1ff;
}

struct device_drv tty_drv = {
	/*.drv_id = */              DRIVER_tty,
	/*.dname = */               "tty",
	/*.name = */                "TTY",
	/*.drv_detect = */          tty_detect,
  /*.reinit_device = */       reinit_tty_device,
  /*.get_statline_before = */ NULL,
  /*.get_statline = */        NULL,
  /*.api_data = */            NULL,
  /*.get_stats = */           NULL,
  /*.identify_device = */     NULL,
  /*.set_device = */          NULL,
  /*.thread_prepare = */      tty_thread_prepare,
  /*.can_limit_work = */      tty_can_limit_work,
  /*.thread_init = */         tty_thread_init,
  /*.prepare_work = */        tty_prepare_work,
  /*.hash_work = */           NULL,
	/*.scanhash = */            tty_scanhash,
  /*.scanwork = */            NULL,
  /*.queue_full = */          NULL,
  /*.flush_work = */          NULL,
  /*.update_work = */         NULL,
  /*.hw_error = */            NULL,
	/*.thread_shutdown = */     tty_thread_shutdown,
  /*.thread_enable =*/        NULL,
  false,
  0,
  0
};





