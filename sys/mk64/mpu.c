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

#ifdef KERNEL

#include <sys/types.h>
#include <sys/systm.h>

#include <machine/kinetis.h>
#include <machine/mpu.h>

void mpu_init(void) {
    /**
     *
     * TODO: take full advantage of the MPU.
     *
     * MPU_CESR value as freshly reset is 0x00815101.
     *
     * crossbar switch master assignments:
     *
     *   0  core code bus       3  enet
     *   1  core system bus     4  usb fs/ls otg
     *   2  dma/ezport          5  sdhc
     *
     * slave port assignments from Kinetis K64 sub-family reference manual:
     *
     *   crossbar slave port 0  ->  mpu slave port 0  ->  flash controller 
     *   crossbar slave port 1  ->  mpu slave port 1  ->  SRAM backdoor
     *       code bus (SRAM_L)  ->  mpu slave port 2  ->  SRAM_L backdoor
     *     system bus (SRAM_U)  ->  mpu slave port 3  ->  SRAM_U backdoor
     *   crossbar slave port 4  ->  mpu slave port 4  ->  flexbus
     *
     * logical bus master assigments (important for setting RGDAACn and friends):
     *
     *   0  core         4  usb
     *   1  debugger     5  sdhc
     *   2  dma/ezport   6  (none)
     *   3  enet         7  (none)
     *
     * interestingly enough, only the debugger may change any
     * register. core is not permitted to touch RGD0 other than set
     * permissions via RGDAAC0 and even then changing M1SM or M1UM is
     * not allowed as blocking the debugger is out of question.
     *
     * system memory map:
     *
     *        range                  what                    access
     *  0000 0000-07ff ffff   flash                        read only, all masters
     *  0800 0000-0fff ffff   flexbus alias for 88*        core only
     *  1000 0000-13ff ffff   flexnvm (emulated eeprom)    all masters
     *  1400 0000-17ff ffff   flexram (not very useful)    all masters
     *  1800 0000-1bff ffff   flexbus alias for 98*        core only
     *  1fff 0000-1fff ffff   SRAM_L: lower SRAM, "code"   all masters
     *  2000 0000-2002 ffff   SRAM_U: upper SRAM, "system" all masters
     *                        _bitband region_, wtf
     *  2010 0000-21ff ffff   reserved                     -
     *  2200 0000-23ff ffff   TCMU SRAM bitband alias      core only
     *  2400 0000-2fff ffff   reserved                     -
     *  4000                  AIPS0 bitband (peripherals)
     *  4008                  AIPS1 bitband (more peripherals)
     *  400f                  GPIO bitband
     *     (...continues...)
     *
     * also, misaligned access over the lower/upper boundary is not allowed.
     *
     * when access permission for a region is evaluated, the most
     * permissive rule wins. i.e. we can't allow everyone write access
     * everywhere and then use more specific rules to deny access to
     * specific areas.
     *
     * this, combined with how the K64 starts up with the MPU active
     * and one huge region configured, means we have to set up the
     * regions one by one and as the last step change the access rule
     * for the huge region.
     *
     * ...or how the reference manual phrases it:
     */

    /**
     * K64 Sub-Family Reference Manual, Rev. 4, Oct 2019, p. 440
     * Chapter 19                    Memory Protection Unit (MPU)
     * 
     * 19.4.2 Putting it all together and error terminations
     *
     * For each slave port monitored, the MPU performs a reduction-AND
     * of all the individual terms from each access evaluation
     * macro. This expression then terminates the bus cycle with an
     * error and reports a protection error for three conditions:
     *
     * • If the access does not hit in any region descriptor, a protection
     *   error is reported.
     * • If the access hits in a single region descriptor and that region
     *   signals a protection violation, a protection error is reported.
     * • If the access hits in multiple (overlapping) regions and all regions
     *   signal protection violations, a protection error is reported.
     *
     * As shown in the third condition, granting permission is a
     * higher priority than denying access for overlapping
     * regions. This approach is more flexible to system software in
     * region descriptor assignments. For an example of the use of
     * overlapping region descriptors, see Application information.
    */

    /**
     * plan:
     *
     * - flash is kernel text, only accessible from handler mode
     * - lowest 64 k RAM segment is eXecute Never, also off-limits for
     *   userspace
     * - the rest of RAM is free for all
     *
     * -> we have 32-bit granularity and 12 regions, thus it would be entirely
     *    possible to do things like (mostly) isolate in-core processes
     *
     * -? does the user structure need to be accessible from userspace?
     *
     * MPU supports 8-bit process identifiers. these are set by the cleverly
     * named Miscellaneous Control Module via the MCM_PID register. additionally,
     * masters 0-3 have separate supervisor/user (handler/thread, I presume)
     * access controls.
     *
     * core privileged   = access all areas
     * core unprivileged = flash (kernel) XN, SRAM_L off limits except u
     * core unprivileged = use pids?
     *
     * MPU_RGDx_WORD2 M0-3:
     *
     * bits 0-2          3-4         5
     * MxUM              MxSM        MxPE
     *
     * 1 user execute    00 rwx      0 ignore process identifier
     * 2 user write      01 rx       1 check process identifier against WORD3
     * 4 user read       10 rw
     *                   11 use UM
     *
     * M4-7:
     *
     * bit 0                         bit 1
     * RE - read enable 0/1          WE - write enable 0/1
     *
     *
     */

    /* kernel text in flash */
    MPU_RGD1_WORD0 = 0;
    MPU_RGD1_WORD1 = 0x07ffffff & ~(0x1f);
    MPU_RGD1_WORD2 = MPU_RGD_WORD2_SM_RWX;
    MPU_RGD1_WORD3 |= MPU_RGD_WORD3_VALID;

    /* kernel data + u + kernel stack */
    MPU_RGD2_WORD0 = 0x1fff0000;
    MPU_RGD2_WORD1 = 0x1fffffff & ~(0x1f);
    MPU_RGD2_WORD2 = MPU_RGD_WORD2_SM_RW | MPU_RGD_WORD2_ALLOW_SDHC;
    MPU_RGD2_WORD3 |= MPU_RGD_WORD3_VALID;

    /* userland */
    MPU_RGD3_WORD0 = 0x20000000;
    MPU_RGD3_WORD1 = 0x2002ffff & ~(0x1f);
    MPU_RGD3_WORD2 = MPU_RGD_WORD2_FIELDS(0x1f, 0x1f, 0, 0, 0, 0);
    MPU_RGD3_WORD3 |= MPU_RGD_WORD3_VALID;

    /* that one piece of code just after main() call, from where we
     * jump into userland for the first time.
     *
     * TODO: improve.
     */
    MPU_RGD4_WORD0 = 0x300;
    MPU_RGD4_WORD1 = 0x3ff & ~(0x1f);
    MPU_RGD4_WORD2 = MPU_RGD_WORD2_FIELDS(0x1f, 0x1f, 0, 0, 0, 0);
    MPU_RGD4_WORD3 |= MPU_RGD_WORD3_VALID;

    /* disallow non-privileged access for region 0, and do it using
     * RGDAAC0 instead of RGD0_WORD2 to avoid invalidating the
     * region and possibly ending up with a state where we'd need to
     * reset the MCU to be able to write to the MPU config regs again.
     *
     * TODO: this could be eXecute Never?
     */
    MPU_RGDAAC0 = MPU_RGD_WORD2_SM_RWX | MPU_RGD_WORD2_ALLOW_DEBUGGER;
}

/**
 * print the current MPU status and configuration to the console.
 *
 * TODO: include the less-interesting bus masters too.
 */
void mpu_stat(int verbose) {
    int mpuregcnt;

    if (!(MPU_CESR & 1)) {
        printf("mpu: disabled\n");
        return;
    }

    mpuregcnt =
        (MPU_CESR & (1 << 9)) ? 16 : ((MPU_CESR & (1 << 8)) ? 12 : 8);
    printf("mpu: enabled, %d regions supported\n", mpuregcnt);

    if (!verbose)
        return;

    for (int n = 0; n < mpuregcnt; n++) {
        unsigned int *regstart, *regend, *regword2, *regword3, *rgdaac;
        /**
	 * MPU_CESR       =  0x4000D000 
	 * MPU_RGDn_WORD0 =  MPU_CESR + 0x400 + (0x10 * n) 
	 * MPU_RGDn_WORD1 =  MPU_CESR + 0x404 + (0x10 * n) 
	 * MPU_RGDn_WORD2 =  MPU_CESR + 0x408 + (0x10 * n) 
	 * MPU_RGDn_WORD3 =  MPU_CESR + 0x40C + (0x10 * n) 
	 * MPU_RGDAACn    =  MPU_CESR + 0x800 + (0x04 * n)
	 */

        regstart = (unsigned int *) (0x4000D000 + 0x400 + (0x10 * n));
        regend   = (unsigned int *) (0x4000D000 + 0x404 + (0x10 * n));
        regword2 = (unsigned int *) (0x4000D000 + 0x408 + (0x10 * n));
        regword3 = (unsigned int *) (0x4000D000 + 0x40C + (0x10 * n));
        rgdaac   = (unsigned int *) (0x4000D000 + 0x800 + (0x04 * n));

        /* there might be a corner case where this is a legitimately
	 * useful MPU region configuration, but this'll do for now.
	 */
        if (*regstart == *regend && !*regword3)
            continue;

        printf("mpu: region %2d: 0x%08x-0x%08x, flags 0x%08x, %s\n", n,
               *regstart, *regend | 0x1f, *rgdaac,
               (*regword3 & 1) ? "valid" : "invalid");
        if (*regword3 & 1) {
            for (int bm = 0; bm < 4; bm++) {
                printf(" bm%d:" MPU_WORD2_LOBM_FMT, bm,
                       MPU_PARSE_WORD2_LOBM((*regword2 >> bm * 6)));
            }
            printf("\n");
        }
    }
}

#endif
