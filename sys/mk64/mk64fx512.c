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
