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

#    include <machine/mk64fx512.h>

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
     * interestingly enough, only debugger may change any register. core is not
     * permitted to touch RGD0 other than set permissions via RGDAAC0 and even
     * then changing M1SM or M1UM is not allowed as blocking the debugger is out
     * of question.
     *
     * system memory map:
     *
     *        range                  what                    access
     *  0000 0000-07ff ffff   flash                        read only, all masters
     *  0800 0000-0fff ffff   flexbus alias for 88*        core only
     *  1000 0000-13ff ffff   flexnvm (emulated eeprom)    all masters
     *  1400 0000-17ff ffff   flexram (not very useful)    all masters
     *  1800 0000-1bff ffff   flexbus alias for 98*        core only
     *  1fff 0000-1fff ffff   SRAM_L: lower SRAM           all masters
     *  2000 0000-2002 ffff   SRAM_U: upper SRAM           all masters
     *                        _bitband region_, wtf
     *  2010 0000-21ff ffff   reserved                     -
     *  2200 0000-23ff ffff   TCMU SRAM bitband alias      core only
     *  2400 0000-2fff ffff   reserved                     -
     *  4000                  AIPS0 bitband (peripherals)
     *  4008                  AIPS1 bitband (more peripherals)
     *  400f                  GPIO bitband
     *     (...higher up there is more stuff...)
     *
     * also, misaligned access over the lower/upper boundary is not allowed.
     *
     */

#if 0
    t = MPU_RGDAAC0;
    t |= 0x0F7DF7DF;
    MPU_RGDAAC0 = t;
#endif
}


unsigned long
    rtc_get(void) {
    return RTC_TSR;
}

void rtc_set(unsigned long t) {
    RTC_SR  = 0;
    RTC_TPR = 0;
    RTC_TSR = t;
    RTC_SR  = RTC_SR_TCE;
}

#pragma GCC diagnostic pop

int nvic_execution_priority(void)
{
	uint32_t priority=256;
	uint32_t primask, faultmask, basepri, ipsr;

	// full algorithm in ARM DDI0403D, page B1-639
	// this isn't quite complete, but hopefully good enough
	__asm__ volatile("mrs %0, faultmask\n" : "=r" (faultmask)::);
	if (faultmask) return -1;
	__asm__ volatile("mrs %0, primask\n" : "=r" (primask)::);
	if (primask) return 0;
	__asm__ volatile("mrs %0, ipsr\n" : "=r" (ipsr)::);
	if (ipsr) {
		if (ipsr < 16) priority = 0; // could be non-zero
		else priority = NVIC_GET_PRIORITY(ipsr - 16);
	}
	__asm__ volatile("mrs %0, basepri\n" : "=r" (basepri)::);
	if (basepri > 0 && basepri < priority) priority = basepri;
	return priority;
}


#endif /* KERNEL */
