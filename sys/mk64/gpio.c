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

#include <sys/param.h>
#include <sys/conf.h>
#include <sys/kconfig.h>
#include <sys/types.h>
#include <sys/systm.h>

#include <sys/vmmeter.h>

#include <machine/mk64fx512.h>
#include <machine/teensy.h>
#include <machine/gpio.h>
#include <machine/intr.h>
#include <machine/uart.h>
#include <machine/systick.h>

teensy_gpio_pin teensy_gpio_pins[NPINS] = {
    /* tpin, port, pin, dir, pullup, irq trigger, reserved, is led? (early init) */
    {{25, TG_GPIO_A, 5, TG_OUTPUT, 0, 0, 0, 1}},  /* fault */

    {{26, TG_GPIO_A, 14, TG_OUTPUT, 0, 0, 0, 1}}, /* spl b0 */
    {{27, TG_GPIO_A, 15, TG_OUTPUT, 0, 0, 0, 1}}, /* spl b1 */
    {{28, TG_GPIO_A, 16, TG_OUTPUT, 0, 0, 0, 1}}, /* spl b2 */

    {{32, TG_GPIO_B, 11, TG_OUTPUT, 0, 0, 0, 1}}, /* value b3 */
    {{31, TG_GPIO_B, 10, TG_OUTPUT, 0, 0, 0, 1}}, /* spl b0 */
    {{30, TG_GPIO_B, 19, TG_OUTPUT, 0, 0, 0, 1}}, /* spl b1 */
    {{29, TG_GPIO_B, 18, TG_OUTPUT, 0, 0, 0, 1}}, /* spl b2 */

    {{19, TG_GPIO_B, 2, TG_OUTPUT, 0, 0, 0, 1}},  /* value b4 */
    {{18, TG_GPIO_B, 3, TG_OUTPUT, 0, 0, 0, 1}},  /* value b5 */
    {{17, TG_GPIO_B, 1, TG_OUTPUT, 0, 0, 0, 1}},  /* value b6 */
    {{16, TG_GPIO_B, 0, TG_OUTPUT, 0, 0, 0, 1}},  /* value b7 */
    
    {{13, TG_GPIO_C, 5, TG_OUTPUT, 0, 0, 0, 1}},  /* onboard led */

    {{14, TG_GPIO_D, 1, TG_INPUT, TG_PULLUP, TG_PCR_IRQ_HIGH, 0, 0}},  /* button */
};

static u_int gpioisrpin[TG_NGPIO];
static void port_isr(int port);
static int late;


void portd_isr(void) {
    port_isr(TG_GPIO_D);
}

static void port_isr(int port) {
    volatile u_int *isfr;
    u_int pins;
    int s, c = 0;

    s = splgpio();
    isfr = TG_ISFR(port);

    pins = *isfr & gpioisrpin[port];

    if (!pins)
	goto out;
    
    do {
	if (*isfr & (1 << c)) {
	    /* printf("gpio: pin %c%d: interrupt\n", 0x41 + port, c);*/
	    /* printf("%8d si %d so %d \n", systick_ms, cnt.v_swpin, cnt.v_swpout); */
	    printf("\n");
	    mpustat(1);
	    sync();
	    *isfr |= (1 << c);
	}
    } while (c++ < TG_GPIO_PINMAX);

 out:
    splx(s);
}

/** Set GPIO port ISR.
 *
 * port = TG_GPIO_{A,B,C,D,E}
 *  pin = 0..31
 * mode = TG_PCR_IRQ_{OFF,LOW,RISING,FALLING,EITHER,HIGH}
 */
void teensy_gpio_set_isr(int port, int pin, int mode) {
    volatile u_int *pcr;
    int s;

    s = splgpio();

    arm_disable_irq(TG_GPIO_IRQ(port));
    
    if (mode && !(mode & 0x8)) {
	printf("gpio: unsupported mode\n");
    }

    pcr = TG_PCR(port, pin);

    *pcr &= TG_PCR_IRQ_OFF;
    gpioisrpin[port] &= ~(1 << pin);
	
    if (mode) {
	*pcr |= (mode << 16);
	printf("gpio: pin %c%d: interrupt enabled\n", 0x41 + port, pin);
	gpioisrpin[port] |= (1 << pin);
    }

    /* no point in hammering the isr if there are no
     * active pins, so only re-enable if it makes sense.
     */
    if (gpioisrpin[port]) {
        arm_enable_irq(TG_GPIO_IRQ(port));
    }

    splx(s);
}

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

    if (!late && !pin->is_led)
	return;
    if (late && pin->is_led)
	return;
    
    pcor = TG_PCOR(pin->port);
    pddr = TG_PDDR(pin->port);
    pcr = TG_PCR(pin->port, pin->pin);

    if (pin->direction == TG_OUTPUT) {
	*pddr |= (1 << pin->pin);
	*pcr = TG_PCR_PIN_OUTPUT;
	*pcor = (1 << pin->pin);
    } else {
	*pddr &= ~(1 << pin->pin);
        if (pin->pullup == TG_PULLUP) {
	    *pcr = TG_PCR_PIN_PULLUP;
        } else {
	    *pcr = TG_PCR_PIN_INPUT;
        }
    }

    if (pin->interrupt)
	teensy_gpio_set_isr(pin->port, pin->pin, pin->interrupt);
}

void teensy_gpio_init(void) {
    if (late)
	for (int i = 0; i < TG_NGPIO; i++)
	    arm_set_irq_prio(TG_GPIO_IRQ(i), SPL_GPIO);

    for (int i = 0; i < NPINS; i++)
            teensy_gpio_init_pin(&teensy_gpio_pins[i]);

    late = 1;
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

#if 0
void ftm0_isr(void) {
    volatile static unsigned int n;
    int s;

    s = splgpio();

    FTM0_SC &= ~FTM_SC_TOF;
    FTM0_CNT = 0;

    n++;

    if (n % 3)
	led_fault(1);
    else
	led_fault(0);

    splx(s);
}
#endif

static int
gpioprobe(config)
     struct conf_device *config;
{
    int unit = config->dev_unit;
    int flags = config->dev_flags;

    if (unit != 0)
	return 0;

    printf("gpio: alive\n");
    
    teensy_gpio_init();
    return 1;
}

struct driver gpiodriver = {
    "gpio", gpioprobe,
};

#endif
