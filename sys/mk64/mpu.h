/*
 * Copyright (c) 2024 <esp@iki.fi>
 *
 * Permission to use, copy, modify, and distribute this software for any
 * purpose with or without fee is hereby granted, provided that the above
 * copyright notice and this permission notice appear in all copies.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 * WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 * ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 * WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 * ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 * OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 */

#ifndef _MACHINE_MPU_H
#define _MACHINE_MPU_H

#include <machine/kinetis.h>

/* user mode permission bits */
#define MPU_RGD_WORD2_UM_R           0x4 /* read */
#define MPU_RGD_WORD2_UM_W           0x2 /* write */
#define MPU_RGD_WORD2_UM_X           0x1 /* execute */

/* supervisor mode permission bits */
#define MPU_RGD_WORD2_SM_RWX         0
#define MPU_RGD_WORD2_SM_RX          (0x1 << 3)
#define MPU_RGD_WORD2_SM_RW          (0x2 << 3)
#define MPU_RGD_WORD2_SM_UM          (0x3 << 3) /* use user mode permissions */

/* process id match on/off toggle */
#define MPU_RGD_WORD2_PE             (1 << 5)

#define MPU_RGD_WORD2_ALLOW_SDHC     (0x3 << 26)
#define MPU_RGD_WORD2_ALLOW_DEBUGGER (0x1f << 6)

#define MPU_RGD_WORD3_VALID          ((uint32_t) 1)

#define MPU_RGD_WORD2_FIELDS(cr, dbg, dma, enet, usb, sdhc)            \
    (cr) | (dbg << 6) | (dma << 12) | (enet << 18) | (usb << 24)       \
        | (sdhc << 26)

#define MPU_WORD2_LOBM_FMT "u:%c%c%c s:%3s p%c"

#define MPU_PARSE_WORD2_LOBM(bmbits)                                   \
    (bmbits & 0x4) ? 'r' : '-', (bmbits & 0x2) ? 'w' : '-',            \
        (bmbits & 0x1) ? 'x' : '-',                                    \
        (((bmbits & 0x18) == 0x18)   ? "usr"                           \
         : ((bmbits & 0x18) == 0x10) ? "rw-"                           \
         : ((bmbits & 0x18) == 0x8)  ? "r-x"                           \
                                     : "rwx"),                          \
        (bmbits & 0x20) ? '1' : '0'


void mpu_init(void);
void mpu_stat(int verbose);

#endif /* _MACHINE_MPU_H */
