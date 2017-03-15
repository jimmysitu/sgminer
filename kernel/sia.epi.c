/*-
 * Copyright 2017, Jimmy Situ, web@jimmystone.cn
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

#include <string.h>
#include <stdint.h>
#include <e_lib.h>

//#define DEBUG 1

#define ROTR64(x, y)  (((x) >> (y)) ^ ((x) << (64 - (y))))

#define G(r,i,a,b,c,d) \
{ \
  a = a + b + m[ blake2b_sigma[r][2*i] ]; \
  d = ROTR64(d ^ a, 32); \
  c = c + d; \
  b = ROTR64(b ^ c, 24); \
  a = a + b + m[ blake2b_sigma[r][2*i+1] ]; \
  d = ROTR64(d ^ a, 16); \
  c = c + d; \
  b = ROTR64(b ^ c, 63); \
}

#define ROUND(r) \
	G(r,0,v[ 0],v[ 4],v[ 8],v[12]); \
	G(r,1,v[ 1],v[ 5],v[ 9],v[13]); \
	G(r,2,v[ 2],v[ 6],v[10],v[14]); \
	G(r,3,v[ 3],v[ 7],v[11],v[15]); \
	G(r,4,v[ 0],v[ 5],v[10],v[15]); \
	G(r,5,v[ 1],v[ 6],v[11],v[12]); \
	G(r,6,v[ 2],v[ 7],v[ 8],v[13]); \
	G(r,7,v[ 3],v[ 4],v[ 9],v[14]); \

#define SWAP4(x) \
   ((((uint32_t)(x) >> 24) & 0x000000FF) | \
    (((uint32_t)(x) >> 8 ) & 0x0000FF00) | \
    (((uint32_t)(x) << 8 ) & 0x00FF0000) | \
    (((uint32_t)(x) << 24) & 0xFF000000)   \
   )

#define SWAP8(x) \
   ((((uint64_t)(x) >> 56) & 0x00000000000000FF) | \
    (((uint64_t)(x) >> 40) & 0x000000000000FF00) | \
    (((uint64_t)(x) >> 24) & 0x0000000000FF0000) | \
    (((uint64_t)(x) >> 8 ) & 0x00000000FF000000) | \
    (((uint64_t)(x) << 8 ) & 0x000000FF00000000) | \
    (((uint64_t)(x) << 24) & 0x0000FF0000000000) | \
    (((uint64_t)(x) << 40) & 0x00FF000000000000) | \
    (((uint64_t)(x) << 56) & 0xFF00000000000000)   \
   )

#ifdef DEBUG
typedef struct _shared_buf_t {
	uint8_t data[256];
  uint32_t target;
  uint32_t offset;                // work start offset
  volatile uint32_t nonce;
  volatile uint8_t start;         // Start flag, set by host, clear by device
  volatile uint8_t found;         // Nonce found flag, set/clear by device
} shared_buf_t;

volatile shared_buf_t SharedBuf[16] SECTION("shared_dram"); // Share buffer with host in dram
#endif

volatile uint8_t  *data   = (void *) 0x7000;
volatile uint32_t *target = (void *) 0x7100;
volatile uint32_t *offset = (void *) 0x7104;
volatile uint32_t *nonce  = (void *) 0x7108;
volatile uint8_t  *start  = (void *) 0x710C;
volatile uint8_t  *found  = (void *) 0x710D;
volatile uint64_t *dbg  = (void *) 0x7110;

const uint8_t blake2b_sigma[12][16] = {
	{ 0,  1,  2,  3,  4,  5,  6,  7,  8,  9,  10, 11, 12, 13, 14, 15 } ,
	{ 14, 10, 4,  8,  9,  15, 13, 6,  1,  12, 0,  2,  11, 7,  5,  3  } ,
	{ 11, 8,  12, 0,  5,  2,  15, 13, 10, 14, 3,  6,  7,  1,  9,  4  } ,
	{ 7,  9,  3,  1,  13, 12, 11, 14, 2,  6,  5,  10, 4,  0,  15, 8  } ,
	{ 9,  0,  5,  7,  2,  4,  10, 15, 14, 1,  11, 12, 6,  8,  3,  13 } ,
	{ 2,  12, 6,  10, 0,  11, 8,  3,  4,  13, 7,  5,  15, 14, 1,  9  } ,
	{ 12, 5,  1,  15, 14, 13, 4,  10, 0,  7,  6,  3,  9,  2,  8,  11 } ,
	{ 13, 11, 7,  14, 12, 1,  3,  9,  5,  0,  15, 4,  8,  6,  2,  10 } ,
	{ 6,  15, 14, 9,  11, 3,  0,  8,  12, 2,  13, 7,  1,  4,  10, 5  } ,
	{ 10, 2,  8,  4,  7,  6,  1,  5,  15, 11, 9,  14, 3,  12, 13, 0  } ,
	{ 0,  1,  2,  3,  4,  5,  6,  7,  8,  9,  10, 11, 12, 13, 14, 15 } ,
	{ 14, 10, 4,  8,  9,  15, 13, 6,  1,  12, 0,  2,  11, 7,  5,  3  } };
	
const uint64_t iv[16] = {
  0x6a09e667f2bdc928, 0xbb67ae8584caa73b, 0x3c6ef372fe94f82b, 0xa54ff53a5f1d36f1,
  0x510e527fade682d1, 0x9b05688c2b3e6c1f, 0x1f83d9abfb41bd6b, 0x5be0cd19137e2179,
  0x6a09e667f3bcc908, 0xbb67ae8584caa73b, 0x3c6ef372fe94f82b, 0xa54ff53a5f1d36f1,
  0x510e527fade68281, 0x9b05688c2b3e6c1f, 0xe07c265404be4294, 0x5be0cd19137e2179
};

uint64_t m[16], v[16];

void Round(uint8_t r){
	G(r,0,v[ 0],v[ 4],v[ 8],v[12]);
	G(r,1,v[ 1],v[ 5],v[ 9],v[13]);
	G(r,2,v[ 2],v[ 6],v[10],v[14]);
	G(r,3,v[ 3],v[ 7],v[11],v[15]);
	G(r,4,v[ 0],v[ 5],v[10],v[15]);
	G(r,5,v[ 1],v[ 6],v[11],v[12]);
	G(r,6,v[ 2],v[ 7],v[ 8],v[13]);
	G(r,7,v[ 3],v[ 4],v[ 9],v[14]);
}

#define GAP   20

int main(){
  int i, j;
  uint32_t count;
  uint32_t coreid;
  unsigned row, col;

  while(1)
  {
    count = 0;
    coreid = (uint32_t)e_get_coreid();
    e_coords_from_coreid(coreid, &row, &col);
    uint32_t idx = (row << 2) + col;

    // wait until host start e-core
    while(!(*start));

    m[0] = *((uint64_t*)(data + 0 ));
    m[1] = *((uint64_t*)(data + 8 ));
    m[2] = *((uint64_t*)(data + 16));
    m[3] = *((uint64_t*)(data + 24));
    m[4] = (uint64_t)(*offset) + (uint64_t)((1 << GAP) * idx);
    m[5] = *((uint64_t*)(data + 40));
    m[6] = *((uint64_t*)(data + 48));
    m[7] = *((uint64_t*)(data + 56));
    m[8] = *((uint64_t*)(data + 64));
    m[9] = *((uint64_t*)(data + 72));
    m[10] = m[11] = m[12] = m[13] = m[14] = m[15] = 0;


    while(*start){
      for(i=0; i<16; i++){
        v[i] = iv[i];
      }

      Round( 0 );
      Round( 1 );
      Round( 2 );
      Round( 3 );
      Round( 4 );
      Round( 5 );
      Round( 6 );
      Round( 7 );
      Round( 8 );
      Round( 9 );
      Round( 10 );
      Round( 11 );

      (*found) = (SWAP8(0x6a09e667f2bdc928 ^ v[0] ^ v[8]) <= (uint64_t)(*target));
      (*nonce) = SWAP4((uint32_t)m[4]);
      count++;
#ifdef DEBUG
      memcpy(&SharedBuf[idx], data, sizeof(shared_buf_t));
#endif
      if(*found){
        (*start) = 0;
      }else{
        m[4]++;
        if(count>>GAP){
          (*start) = 0;
        }
      }
    } // end of while(start))
  } // end of while(1)

  return 0;
}

