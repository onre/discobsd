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

#    include <machine/mk64fx512.h>
#    include <machine/teensy.h>

#    ifdef TEENSY35

/**
 * this is a      ######   #####  #######
 * honest attempt #     # #     # #
 * at trying to   #     # #       #
 * not constantly ######  #        #####
 * forget which   #       #             #
 * pin the LED is #       #     # #     #
 * connected to   #        #####   #####
 */

void teensy_led_init(void) {
    PORTC_PCR5 = PORT_PCR_MUX(1) | PORT_PCR_DSE | PORT_PCR_SRE;
    GPIOC_PDDR |= TEENSY_LED_PIN_MASK;
    GPIOC_PSOR |= TEENSY_LED_PIN_MASK;
}

void teensy_led_on(void) { GPIOC_PSOR = TEENSY_LED_PIN_MASK; }

void teensy_led_off(void) { GPIOC_PCOR = TEENSY_LED_PIN_MASK; }

#    endif /* TEENSY35 */

#endif /* KERNEL */
