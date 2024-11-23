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

#ifndef _MACHINE_MK64FX512_H_
#define _MACHINE_MK64FX512_H_

#ifdef KERNEL

#include <machine/kinetis.h>

static inline void arm_attach_isr(enum IRQ_NUMBER_t irq,
				  void (*function)(void)) {
    _VectorsRam[irq + 16] = function;
}

static __inline unsigned int
get_basepri(void) {
    unsigned int result;

    asm volatile("mrs %0, basepri" : "=r" (result));
    return result;
}

static __inline void
set_basepri(unsigned int value) {
    asm volatile("msr basepri, %0" :: "r" (value) : "memory");
}

static __inline void
set_basepri_max(unsigned int value) {
    asm volatile("msr basepri_max, %0" :: "r" (value) : "memory");
}

static __inline void isb(void) {
    asm volatile("isb 0xF" ::: "memory");
}

static __inline void dsb(void) {
    asm volatile("dsb");
}

static __inline void wfi(void) {
    asm volatile("wfi");
}

#define RESTART_ADDR 0xE000ED0C
#define READ_RESTART() (*(volatile uint32_t *) RESTART_ADDR)
#define WRITE_RESTART(val)                                         \
        ((*(volatile uint32_t *) RESTART_ADDR) = (val))

#define SCB_SHCSR_MEMFAULTENA ((unsigned int) 1 << 16)
#define SCB_SHCSR_BUSFAULTENA ((unsigned int) 1 << 17)
#define SCB_SHCSR_USGFAULTENA ((unsigned int) 1 << 18)

#define SCB_ICSR_PENDSVSET_MASK ((unsigned int) 1 << 28)
#define SCB_CCR_STKALIGN_MASK ((unsigned int) 1 << 9)

#define SVCALL_IRQ 11
#define PENDSV_IRQ 14
#define SYSTICK_IRQ 15
#define IRQ_SYSTICK 15 /* d'oh */

int nvic_execution_priority(void);
unsigned long rtc_get(void);
void rtc_set(unsigned long);
void mpuinit(void);

#define __NVIC_PRIO_BITS 4

#endif /* KERNEL */

#endif /* !_MACHINE_MK64FX512_H_ */
