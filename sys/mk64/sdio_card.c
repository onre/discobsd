/*
 * Copyright (c) 2021 Christopher Hettrick <chris@structfoo.com>
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

#include <sys/param.h>
#include <sys/systm.h>

#include <machine/debug.h>
#include <machine/sd.h>
#include <machine/sdio_card.h>
#include <machine/mk6x_sdio.h>
#include "sd.h"
#include "mk6x_sdio.h"


#define SECTSIZE        512

extern struct disk sddrives[NSD];

/*
 * Initialize a card.
 * Return nonzero if successful.
 */
int
card_init(int unit)
{
    struct disk *du = &sddrives[unit];

    du->card_type = mk6x_sdio_cardtype();

    return 1;
}

/*
 * Get disk size in 512-byte sectors.
 * Return nonzero if successful.
 */
int
card_size(int unit)
{
    return mk6x_sdio_sectorCount();
}

/*
 * Read a block of data.
 * Return nonzero if successful.
 */
int
card_read(int unit, unsigned int offset, char *data, unsigned int bcount)
{
    int nblocks;

    if ((bcount % SECTSIZE) == 0) {
        nblocks = bcount / SECTSIZE;

    } else {
        nblocks = (bcount / SECTSIZE) + 1;
    }

#if 0
    DEBUG("card_read:  bcount: %d\tnblocks: %d\tbcount \% %d: %d\n",
      bcount, nblocks, SECTSIZE, bcount % SECTSIZE);
#endif

    if (!mk6x_sdio_readSectors(offset<<1, data, nblocks)) {
	printf("card_read: read failed: %d\n", mk6x_sdio_errorCode());
        return 0;
    }

    return 1;
}

/*
 * Write a block of data.
 * Return nonzero if successful.
 */
int
card_write(int unit, unsigned offset, char *data, unsigned bcount)
{
    int nblocks, state;

    if ((bcount % SECTSIZE) == 0) {
        nblocks = bcount / SECTSIZE;
    } else {
        nblocks = (bcount / SECTSIZE) + 1;
    }

#if 0
    DEBUG("card_write: bcount: %d\tnblocks: %d\tbcount \% %d: %d\n",
      bcount, nblocks, SECTSIZE, bcount % SECTSIZE);
#endif
    return mk6x_sdio_writeSectors(offset<<1, (void *)data, nblocks);

#if 0
    /* Wait for write completion. */
    int x = spl0();
    while (BSP_SD_GetCardState() != BSP_SD_OK)
        ;
    splx(x);

    if (SD_state != BSP_SD_OK) {
        printf("card_write: write failed\n");
        return 0;
    }
#endif
    
    return 1;
}

/*
 * Disable the SD card.
 */
void
card_release(int unit)
{

}