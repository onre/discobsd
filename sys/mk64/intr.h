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

#ifndef SIMPLE_INTERRUPTS
#define HARDMODE
#endif

#include <machine/atomic.h>
#include <machine/teensy.h>
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
 * BASEPRI changes the priority of the currently executing thread in
 * regard to interrupts. this makes it effectively an interrupt mask,
 * and BASEPRI_MAX is an alias which prevents one from changing it "in
 * the wrong direction" - as in, through that alias it can only be
 * decreased in value, which means that the mask will match more
 * exceptions than it did before the change. thus BSD spl...()
 * functions should be easy to implement using it.
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

#define SPL_LEAST     224
#define SPL_SOFTCLOCK 176
#define SPL_NET       128
#define SPL_TTY       80
#define SPL_BIO       64
#define SPL_CLOCK     48
#define SPL_HIGH      32
#define SPL_TOP       16

static inline int arm_enable_interrupts() {
    uint32_t primask;
    primask = __get_primask();
    __enable_irq_set_barrier();
    led_intr_ena_on();
    return primask;
}

static inline int arm_disable_interrupts() {
    uint32_t primask;
    primask = __get_primask();
    __disable_irq_set_barrier();
    led_intr_ena_off();
    return primask;
}

static inline void arm_restore_interrupts(int s) {
    __set_primask(s);
    __set_barrier();
    led_intr_ena_on();
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
    return old;
}

static inline void splx(int s) {
    set_basepri(s);
    __set_barrier();
    switch (s) {
    case SPL_HIGH:
        led_spl_high();
        break;
    case SPL_CLOCK:
        led_spl_clock();
        break;
    case SPL_TTY:
        led_spl_tty();
        break;
    case SPL_BIO:
        led_spl_bio();
        break;
    case SPL_NET:
        led_spl_net();
        break;
    case SPL_SOFTCLOCK:
        led_spl_softclock();
        break;
    case SPL_LEAST:
        led_spl_least();
        break;
    }
}

static inline int spl0(void) {
    int old;

    old = nvic_execution_priority();
    set_basepri(SPL_LEAST);
    __set_barrier();

    led_spl_least();

    return old;
}

#define splhigh()                                                      \
    splraise(SPL_HIGH);                                                \
    led_spl_high();
#define splclock()                                                     \
    splraise(SPL_CLOCK);                                               \
    led_spl_clock();
#define spltty()                                                       \
    splraise(SPL_TTY);                                                 \
    led_spl_tty();
#define splbio()                                                       \
    splraise(SPL_BIO);                                                 \
    led_spl_bio();
#define splnet()                                                       \
    splraise(SPL_NET);                                                 \
    led_spl_net();
#define splsoftclock()                                                 \
    splraise(SPL_SOFTCLOCK);                                           \
    led_spl_softclock();

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
