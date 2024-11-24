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
}

void teensy_led_on(void) { GPIOC_PSOR = TEENSY_LED_PIN_MASK; }

void teensy_led_off(void) { GPIOC_PCOR = TEENSY_LED_PIN_MASK; }

void ledinit(void) {
    LED_PIN_INIT(LED0_PORT, LED0_PIN);
    LED_PIN_INIT(LED1_PORT, LED1_PIN);
    LED_PIN_INIT(LED2_PORT, LED2_PIN);
    LED_PIN_INIT(LED3_PORT, LED3_PIN);
    LED_PIN_INIT(LED4_PORT, LED4_PIN);
    LED_PIN_INIT(LED5_PORT, LED5_PIN);
    LED_PIN_INIT(LED6_PORT, LED6_PIN);
    LED_PIN_INIT(LED7_PORT, LED7_PIN);
    LED_PIN_INIT(LED8_PORT, LED8_PIN);

    LED_ON(LED_FAULT);
    mdelay(50);
    LED_OFF(LED_FAULT);
}

#    endif /* TEENSY35 */

#endif /* KERNEL */
