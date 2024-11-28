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

#include <machine/mk64fx512.h>
#include <machine/teensy.h>
#include <machine/intr.h>

#ifdef TEENSY35

/**
 * this is a      ######   #####  #######
 * honest attempt #     # #     # #
 * at trying to   #     # #       #
 * not constantly ######  #        #####
 * forget which   #       #             #
 * pin the LED is #       #     # #     #
 * connected to   #        #####   #####
 */

/*  RED   YEL   YEL   YEL   GRN   GRN   GRN   GRN
 *   B     B     B     B     A     A     A     A
 *  11    10    19    18    16    15    14     5
 *
 * 4 bits to greens:
 *
 * GPIOA_PCOR = (7 << 14)|(1 << 5);
 * GPIOA_PSOR = (((num<<1) & 0x7) << 14)|((num & 1) & (1 << 5); 
 *
 * 3 bits to yellows:
 *
 * GPIOB_PCOR = (3 << 18)|(1 << 10);
 * GPIOB_PSOR = ((num & 0x4) << 18)|(((num>>2) & 1) & (1 << 10)):
 *
 */

void led_g4bit(int val) {
    GPIOA_PCOR = ((7 << 14) | (1 << 5));
    GPIOA_PSOR = ((((val >> 1) & 0x7) << 14) | ((val & 1) << 5));
}

void led_y3bit(int val) {
    GPIOB_PCOR = ((3 << 18) | (1 << 10));
    GPIOB_PSOR = (((val & 0x3) << 18) | (((val >> 2) & 1) << 10));
}

void led_init(void) {
    LED_PIN_INIT(LED0_PORT, LED0_PIN);
    LED_PIN_INIT(LED1_PORT, LED1_PIN);
    LED_PIN_INIT(LED2_PORT, LED2_PIN);
    LED_PIN_INIT(LED3_PORT, LED3_PIN);
    LED_PIN_INIT(LED4_PORT, LED4_PIN);
    LED_PIN_INIT(LED5_PORT, LED5_PIN);
    LED_PIN_INIT(LED6_PORT, LED6_PIN);
    LED_PIN_INIT(LED7_PORT, LED7_PIN);
    LED_PIN_INIT(LED8_PORT, LED8_PIN);
}

void led_test(void) {
    volatile int j;
    for (int i = 0; i < 128; i++) {
        led_g4bit(i & 0xF);
        led_y3bit((i >> 4) & 0x7);

        for (j = 0; j < 0x8ffff; j++)
            ;
    }
}

void led_fault(int num) {
    if (num)
        LED_ON(LED_FAULT);
    else
        LED_OFF(LED_FAULT);
}

void led_intr(int num) {
    if (num)
        LED_ON(LED_INTR);
    else
        LED_OFF(LED_INTR);
}

void ftm0_isr(void) {
    volatile static unsigned int n;
    int s;

    s = splbio();

    FTM0_SC &= ~FTM_SC_TOF;
    FTM0_CNT = 0;

    n++;

    if (n % 3)
        GPIOB_PCOR = (1 << 11);
    else
        GPIOB_PSOR = (1 << 11);

    splx(s);
}


#endif /* TEENSY35 */

#endif /* KERNEL */
