/*-
 * Copyright 2017 Jimmy web@jimmystone.cn
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 */

#include <stdint.h>
#include <e_lib.h>

typedef struct _shared_buf_t {
	uint8_t data[256];
  uint32_t target;
  uint32_t offset;                // work start offset
	volatile uint32_t nonce;
	volatile uint8_t start;         // Start flag, set by host
	volatile uint8_t found;         // Nonce found flag, set by device
	volatile uint8_t done;          // Hash job done flag, set by device
} shared_buf_t;

volatile shared_buf_t SharedBuf[16] SECTION("shared_dram"); // Share buffer with host in dram

volatile uint8_t  *data   = (void *) 0x7000;
volatile uint32_t *target = (void *) 0x7100;
volatile uint32_t *offset = (void *) 0x7104;
volatile uint32_t *nonce  = (void *) 0x7108;
volatile uint8_t  *start  = (void *) 0x710C;
volatile uint8_t  *found  = (void *) 0x710D;
volatile uint8_t  *done   = (void *) 0x710E;



