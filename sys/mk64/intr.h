/*
 * Copyright (c) 2022 Christopher Hettrick <chris@structfoo.com>
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

#ifndef _MACHINE_INTR_H_
#define _MACHINE_INTR_H_

#ifdef KERNEL

#include <machine/atomic.h>
#include <machine/teensy.h>
#include <machine/mk64fx512.h>
#include <machine/machparam.h>
#include <machine/kinetis.h>

/**
 * DDI0403D chapter B1.5, or how I understood it:
 *
 * in ARM, interrupt priority 0 is the highest. Kinetis K series have
 * four bits of priority in an eight-bit field. the high nibble is the
 * significant one.  thus, we end up with the following series as the
 * possible priorities, from highest to lowest:
 *
 *  0, 16, 32, 48, 64, 80, 96, 112, 128, 144, 160, 176, 192, 208, 224, 240
 *
 * this does allow for some creativity when mapping the seven 2.11-BSD
 * priority levels. let's try with:
 *
 *  spl0           240
 *  splsoftclock   192
 *  splnet         128
 *  splbio          64
 *  spltty          32
 *  splclock        16
 *  splhigh          0
 *
 * BASEPRI changes the priority of the currently executing thread in
 * regard to interrupts. this makes it effectively an interrupt mask,
 * and BASEPRI_MAX is an alias which prevents one from changing it "in
 * the wrong direction" - as in, through that alias it can only be
 * decreased in value, which means that the mask will match more
 * exceptions than it did before the change. thus BSD spl...()
 * functions should be easy to implement using it.
 *
 * PRIMASK is a one-bit register preventing all exceptions with
 * configurable priority from executing. it also has some interesting
 * properties outlined in "Priority escalation" section of the document,
 * such as turning SVC calls into hard faults.
 *
 * setting FAULTMASK raises priority to -1, making the current
 * execution thread essentially a hard fault.
 *
 * BONUS: other listed cases of exception priority escalation:
 *
 *  - non-enabled configurable-priority fault firing
 *  - exception handler for a configurable-priority fault causes
 *    another exception of equal or higher (logical) priority
 *  - exception handler for a configurable-priority fault causes
 *    an exception of the type it is currently handling
 *
 */

#ifdef HARDMODE

#define SPL_NONE      240

#define SPL_LEAST     224
#define SPL_SOFTCLOCK 192
#define SPL_NET       160

#define SPL_TTY       128
#define SPL_BIO       96

#define SPL_CLOCK     64
#define SPL_HIGH      32
#define SPL_TOP       16

#define splusb()      spltty()

#endif /* HARDMODE */

static inline int arm_set_system_handler_prio(int handler, int prio) {
    /**
     * each priority register consists of four 8-bit values for respective
     * system handlers. there is no reg 0 as handler/exception 0 is not defined
     * and 1-3 are not configurable.
     *
     * SCB_SHPR1 = 7-4   (reserved, busfault, memmanage, usagefault)
     * SCB_SHPR2 = 11-8  (svcall, reserved, reserved, reserved)
     * SCB_SHPR3 = 15-12 (systick, pendsv, reserved, debugmon)
     */

    int c;

    switch (handler) {
    case SVCALL_HANDLER: /* SCB_SHPR2 bits 31-24 */
        c = SCB_SHPR2;
        c &= ~0xff000000;
        c |= (prio << 24) & 0xff000000;
        SCB_SHPR2 = c;
        break;
    case PENDSV_HANDLER: /* SCB_SHPR3 bits 23-16 */
        c = SCB_SHPR3;
        c &= ~0xff0000;
        c |= (prio << 16) & 0xff0000;
        SCB_SHPR3 = c;
        break;
    case SYSTICK_HANDLER: /* SCB_SHPR3 bits 31-24*/
        c = SCB_SHPR3;
        c &= ~0xff000000;
        c |= (prio << 24) & 0xff000000;
        SCB_SHPR3 = c;
        break;
    default:
        return -1;
    }
    return c;
}

static inline int arm_enable_interrupts() {
    uint32_t primask;
    primask = __get_primask();
    __enable_irq_set_barrier();
    led_intr(1);
    return primask;
}

static inline int arm_disable_interrupts() {
    uint32_t primask;
    primask = __get_primask();
    __disable_irq_set_barrier();
    led_intr(0);
    return primask;
}

static inline void arm_restore_interrupts(int s) {
    __set_primask(s);
    __set_barrier();
    led_intr(1);
}

static inline void arm_disable_irq(int irq) {
    NVIC_DISABLE_IRQ(irq);
    __set_barrier();
}

static inline void arm_enable_irq(int irq) {
    NVIC_ENABLE_IRQ(irq);
    __set_barrier();
}

#ifdef HARDMODE
/**
 * prio is an ARM-style priority mask.
 */
static inline void arm_set_irq_prio(int irq, int prio) {
    NVIC_SET_PRIORITY(irq, prio);
}

static inline int splraise(int new) {
    int old;

    old = nvic_execution_priority();
    set_basepri_max(new);
    __set_barrier();
    if (new == SPL_LEAST)
        teensy_gpio_led_spl(7);
    else
        teensy_gpio_led_spl(~(new >> 5));
    return old;
}

static inline void splx(int s) {
    set_basepri(s);
    __set_barrier();
    if (s == SPL_LEAST)
        teensy_gpio_led_spl(7);
    else
        teensy_gpio_led_spl(~(s >> 5));
}

static inline int spl0(void) {
    int old;

    old = nvic_execution_priority();
    set_basepri(0);
    __set_barrier();
    teensy_gpio_led_spl(0);

    return old;
}

#define splhigh()      splraise(SPL_HIGH)
#define splclock()     splraise(SPL_CLOCK)
#define spltty()       splraise(SPL_TTY)
#define splbio()       splraise(SPL_BIO)
#define splnet()       splraise(SPL_NET)
#define splsoftclock() splraise(SPL_SOFTCLOCK)

#else

#define splhigh()      arm_disable_interrupts()
#define splclock()     arm_disable_interrupts()
#define spltty()       arm_disable_interrupts()
#define splbio()       arm_disable_interrupts()
#define splnet()       arm_disable_interrupts()

#define splsoftclock() arm_enable_interrupts()
#define spl0()         arm_enable_interrupts()

#define splx(s)        arm_restore_interrupts(s)

#define arm_set_irq_prio(irq, prio)

#endif /* HARDMODE */

#endif /* KERNEL */

#endif /* !_MACHINE_INTR_H_ */
