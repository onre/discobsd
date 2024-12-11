/* Teensyduino Core Library
 * http://www.pjrc.com/teensy/
 * Copyright (c) 2017 PJRC.COM, LLC.
 *
 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files (the
 * "Software"), to deal in the Software without restriction, including
 * without limitation the rights to use, copy, modify, merge, publish,
 * distribute, sublicense, and/or sell copies of the Software, and to
 * permit persons to whom the Software is furnished to do so, subject to
 * the following conditions:
 *
 * 1. The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * 2. If the Software is incorporated into a build system that allows
 * selection among a list of target devices, then similar target
 * devices manufactured by PJRC.COM must be included in the list of
 * target devices and selectable in the same manner.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS
 * BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN
 * ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#ifdef KERNEL

#include <machine/mk64fx512.h>

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
     * core unprivileged = pids?
     *
     * regions:
     * 1 kernel text
     * 2 sram_L except u
     * 3 u
     * 4 userland
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
     * firstmost first step: configure the regions.
     *
     */

#define MPU_RGD_WORD2_UM_R           0x4 /* read */
#define MPU_RGD_WORD2_UM_W           0x2 /* write */
#define MPU_RGD_WORD2_UM_X           0x1 /* execute */

#define MPU_RGD_WORD2_SM_RWX         0   /* used to signify intent */
#define MPU_RGD_WORD2_SM_RX          (0x1 << 3)
#define MPU_RGD_WORD2_SM_RW          (0x2 << 3)
#define MPU_RGD_WORD2_SM_UM          (0x3 << 3) /* use user mode permissions */

#define MPU_RGD_WORD2_PE             (1 << 5)

#define MPU_RGD_WORD2_ALLOW_SDHC     (0x3 << 26)
#define MPU_RGD_WORD2_ALLOW_DEBUGGER (0x1f << 6)
#define MPU_RGD_VALID                ((uint32_t) 1)

#define MPU_RGD_WORD2_FIELDS(cr, dbg, dma, enet, usb, sdhc)            \
    (cr) | (dbg << 6) | (dma << 12) | (enet << 18) | (usb << 24)       \
        | (sdhc << 26)

    /* kernel text in flash. only core supervisor mode access permitted. */
    MPU_RGD1_WORD0 = 0;
    MPU_RGD1_WORD1 = 0x07ffffff & ~(0x1f);
    MPU_RGD1_WORD2 = MPU_RGD_WORD2_SM_RWX;
    MPU_RGD1_WORD3 |= MPU_RGD_VALID;

    /* kernel data + u + kernel stack. */
    MPU_RGD2_WORD0 = 0x1fff0000;
    MPU_RGD2_WORD1 = 0x1fffffff & ~(0x1f);
    MPU_RGD2_WORD2 = MPU_RGD_WORD2_SM_RW | MPU_RGD_WORD2_ALLOW_SDHC;
    MPU_RGD2_WORD3 |= MPU_RGD_VALID;

    /* userland. */
    MPU_RGD3_WORD0 = 0x20000000;
    MPU_RGD3_WORD1 = 0x2002ffff & ~(0x1f);
    MPU_RGD3_WORD2 = MPU_RGD_WORD2_FIELDS(0x1f, 0x1f, 0, 0, 0, 0);
    MPU_RGD3_WORD3 |= MPU_RGD_VALID;

    /* that one piece of code just after main() call, from where we
     * jump into userland for the first time.
     */
    MPU_RGD4_WORD0 = 0x300;
    MPU_RGD4_WORD1 = 0x3ff & ~(0x1f);
    MPU_RGD4_WORD2 = MPU_RGD_WORD2_FIELDS(0x1f, 0x1f, 0, 0, 0, 0);
    MPU_RGD4_WORD3 |= MPU_RGD_VALID;

    /* disallow non-privileged access for region 0, and do it using
     * RGDAAC0 instead of MPU_RGD0_WORD2 to avoid invalidating the
     * region and possibly ending up with a state where we'd need to
     * reset the MCU to be able to write to the MPU config regs again.
     */
    MPU_RGDAAC0 = MPU_RGD_WORD2_SM_RWX | MPU_RGD_WORD2_ALLOW_DEBUGGER;
}


unsigned long rtc_get(void) { return RTC_TSR; }

void rtc_set(unsigned long t) {
    RTC_SR  = 0;
    RTC_TPR = 0;
    RTC_TSR = t;
    RTC_SR  = RTC_SR_TCE;
}

#pragma GCC diagnostic pop

int nvic_execution_priority(void) {
    uint32_t priority = 256;
    uint32_t primask, faultmask, basepri, ipsr;

    // full algorithm in ARM DDI0403D, page B1-639
    // this isn't quite complete, but hopefully good enough
    asm volatile("mrs %0, faultmask\n" : "=r"(faultmask)::);
    if (faultmask)
        return -1;
    asm volatile("mrs %0, primask\n" : "=r"(primask)::);
    if (primask)
        return 0;
    asm volatile("mrs %0, ipsr\n" : "=r"(ipsr)::);
    if (ipsr) {
        if (ipsr < 16)
            priority = 0; // could be non-zero
        else
            priority = NVIC_GET_PRIORITY(ipsr - 16);
    }
    asm volatile("mrs %0, basepri\n" : "=r"(basepri)::);
    if (basepri > 0 && basepri < priority)
        priority = basepri;
    return priority;
}


#endif /* KERNEL */
