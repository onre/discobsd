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

teensy_gpio_pin teensy_gpio_pins[NPINS] = {
    {{25, TG_GPIO_A, 5, TG_OUTPUT, 0}},  /* fault */

    {{26, TG_GPIO_A, 14, TG_OUTPUT, 0}}, /* spl b0 */
    {{27, TG_GPIO_A, 15, TG_OUTPUT, 0}}, /* spl b1 */
    {{28, TG_GPIO_A, 16, TG_OUTPUT, 0}}, /* spl b2 */

    {{32, TG_GPIO_B, 11, TG_OUTPUT, 0}}, /* value b3 */
    {{31, TG_GPIO_B, 10, TG_OUTPUT, 0}}, /* spl b0 */
    {{30, TG_GPIO_B, 19, TG_OUTPUT, 0}}, /* spl b1 */
    {{29, TG_GPIO_B, 18, TG_OUTPUT, 0}}, /* spl b2 */

    {{19, TG_GPIO_B, 2, TG_OUTPUT, 0}},  /* value b4 */
    {{18, TG_GPIO_B, 3, TG_OUTPUT, 0}},  /* value b5 */
    {{17, TG_GPIO_B, 1, TG_OUTPUT, 0}},  /* value b6 */
    {{16, TG_GPIO_B, 0, TG_OUTPUT, 0}},  /* value b7 */
    
    {{13, TG_GPIO_C, 5, TG_OUTPUT, 0}},  /* onboard led */

    {{14, TG_GPIO_D, 1, TG_INPUT, TG_PULLUP}},  /* button */
};


/**
 *  RED   YEL   YEL   YEL
 *   A     A     A     A
 *  16    15    14     5  
 *
 *  GRN   GRN   GRN   GRN   GRN   GRN   GRN   GRN
 *   B     B     B     B     B     B     B     B
 *  11    10    19    18     2     3     1     0
 *
 * (val & 0x2) | ((val >> 1) & 0x4) | ((val << 1) & 0x8) | ((val >> 14) & 0x30) | ((val >> 3) & 0xc0)
 */

void teensy_gpio_init_pin(teensy_gpio_pin *pin) {
    volatile unsigned int *pddr, *pcr, *pcor;

    pcr = (unsigned int *) (0x40049000 + (pin->port * 0x1000) + (pin->pin * 4));
    pcor = (unsigned int *) (0x400FF000 + (pin->port * 0x40) + 0x8);
    pddr = (unsigned int *) (0x400FF000 + (pin->port * 0x40) + 0x14);

    if (pin->direction == TG_OUTPUT) {
	*pddr |= (1 << pin->pin);
	*pcr = TG_PCR_PIN_OUTPUT | (pin->interrupt << 16);
    } else {
	*pddr &= ~(1 << pin->pin);
        if (pin->pullup == TG_PULLUP) {
	    *pcr = TG_PCR_PIN_INPUT | TG_PCR_PIN_PULLUP | (pin->interrupt << 16);
        } else {
	    *pcr = (TG_PCR_PIN_INPUT & ~PORT_PCR_PE) | (pin->interrupt << 16);
        }
    }
    *pcor = (1 << pin->pin);
}

void teensy_gpio_led_value(int val) {
    val &= 0xff;
    GPIOB_PCOR = ((3 << 18) | (3 << 10) | 0xF);

    GPIOB_PSOR = ((val & 0x3) | ((val >> 1) & 0x4) | ((val << 1) & 0x8) | ((val << 14) & (0x3 << 18)) | ((val << 4) & (0x3 << 10)));
}

void teensy_gpio_led_spl(int val) {
    val &= 0x7;
    GPIOA_PCOR = (7 << 14);
    GPIOA_PSOR = (val << 14);
}

void teensy_gpio_init(void) {
    for (int i = 0; i < NPINS; i++)
	teensy_gpio_init_pin(&teensy_gpio_pins[i]);
}

void teensy_gpio_led_test(void) {
    volatile int j;

    for (int i = 0; i < 2; i++) {
        led_fault(1);
	teensy_gpio_led_value(0xff);
	teensy_gpio_led_spl(0xf);

        for (j = 0; j < 0xfffff; j++)
            ;

        led_fault(0);
        teensy_gpio_led_value(0);
        teensy_gpio_led_spl(0);

        for (j = 0; j < 0xfffff; j++)
            ;

    }
}


void ftm0_isr(void) {
    volatile static unsigned int n;
    int s;

    s = splbio();

    FTM0_SC &= ~FTM_SC_TOF;
    FTM0_CNT = 0;

    n++;

    if (n % 3)
	led_fault(1);
    else
	led_fault(0);

    splx(s);
}





#endif /* TEENSY35 */

#endif /* KERNEL */
