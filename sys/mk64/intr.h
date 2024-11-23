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
#include <machine/mk64fx512.h>

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
 * BASEPRI is an interrupt mask, and BASEPRI_MAX is an alias which
 * prevents one from changing it "in the wrong direction" - as in,
 * through that alias it can only be decreased in value, which means
 * that the mask will match more exceptions than it did before the
 * change. thus BSD spl...() functions should be easy to implement
 * using it.
 *
 * PRIMASK is a one-bit register preventing all exceptions with
 * configurable priority from executing. it is primarily used for
 * turning off exceptions. it also has some interesting properties
 * outlined in "Priority escalation" section of the document, such as
 * turning SVC calls into hard faults.
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

#define SPL_LEAST     240
#define SPL_SOFTCLOCK 224
#define SPL_NET       192
#define SPL_BIO       160
#define SPL_TTY       128
#define SPL_CLOCK     32
#define SPL_HIGH      0

static inline void arm_enable_interrupts() {
    __enable_irq();
    isb();
}

static inline void arm_disable_interrupts() {
    __disable_irq();
    isb();
}

static inline void arm_disable_irq(int irq) {
    NVIC_DISABLE_IRQ(irq);
}

static inline void arm_enable_irq(int irq) {
    NVIC_ENABLE_IRQ(irq);
}

/**
 * prio is an ARM-style priority mask.
 */
static inline void arm_set_irq_prio(int irq, int prio) {
    NVIC_SET_PRIORITY(irq, prio);
}

static inline int splraise(int new) {
    int old;

    old = get_basepri();
    set_basepri_max(new);
    isb();
    return old;
}

static inline void splx(int s) {
    set_basepri(s);
    isb();
}

static inline int spl0(void) {
    int old;

    old = get_basepri();
    set_basepri(SPL_LEAST);
    isb();

    return old;
}

#define splhigh()      splraise(SPL_HIGH)
#define splclock()     splraise(SPL_CLOCK)
#define spltty()       splraise(SPL_TTY)
#define splbio()       splraise(SPL_BIO)
#define splnet()       splraise(SPL_NET)
#define splsoftclock() splraise(SPL_SOFTCLOCK)

#endif /* KERNEL */

#endif /* !_MACHINE_INTR_H_ */
