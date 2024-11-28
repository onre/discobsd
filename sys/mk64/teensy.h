#ifndef _MACHINE_TEENSY_H_
#define _MACHINE_TEENSY_H_

#include <machine/kinetis.h>

extern uint32_t _edata;
extern uint32_t _etext;
extern void (*_VectorsRam[102])(void);
extern uint32_t _sdata;


#ifdef TEENSY35

#define NPINS 14

#define TEENSY_LED_PIN_MASK   (1 << 5)

#define TG_GPIO_A 0
#define TG_GPIO_B 1
#define TG_GPIO_C 2
#define TG_GPIO_D 3
#define TG_GPIO_E 4

#define TG_INPUT 0
#define TG_OUTPUT 1

#define TG_NORMAL 0
#define TG_PULLUP 1

#define TG_PCR_PIN_OUTPUT (PORT_PCR_MUX(1) | PORT_PCR_DSE | PORT_PCR_SRE)
#define TG_PCR_PIN_INPUT (PORT_PCR_MUX(1))
#define TG_PCR_PIN_PULLUP (PORT_PCR_PS | PORT_PCR_PE)

typedef union teensy_gpio_pin {
    struct {
	unsigned int tpin:8;
        unsigned int port:3;
	unsigned int pin:5;
	unsigned int direction:1; /* 0 = input, 1 = output */
	unsigned int pullup:1;
	unsigned int interrupt:4; /* 0000 = disabled, see k64 reference manual p. 283 for more */
	unsigned int reserved:10;
    };
    unsigned int raw;
} teensy_gpio_pin;

extern teensy_gpio_pin teensy_gpio_pins[NPINS];

#define TG_PSOR(tgprt) *((unsigned int *) (0x400FF000 + (tgprt * 0x40) + 0x04))
#define TG_PCOR(tgprt) *((unsigned int *) (0x400FF000 + (tgprt * 0x40) + 0x08))
#define TG_PTOR(tgprt) *((unsigned int *) (0x400FF000 + (tgprt * 0x40) + 0x0C))
#define TG_PDIR(tgprt) *((unsigned int *) (0x400FF000 + (tgprt * 0x40) + 0x10))
#define TG_PDDR(tgprt) *((unsigned int *) (0x400FF000 + (tgprt * 0x40) + 0x14))

#define LED_FAULT teensy_gpio_pins[0]
#define LED_ONBOARD teensy_gpio_pins[NPINS-2]
#define BUTTON_USER teensy_gpio_pins[NPINS-1]

void teensy_gpio_init(void);
void teensy_gpio_led_value(int val);
void teensy_gpio_led_spl(int val);

static inline void led_fault(int val) {
    if (val) {
        TG_PSOR(LED_FAULT.port) = (1 << LED_FAULT.pin);
    } else {
        TG_PCOR(LED_FAULT.port) = (1 << LED_FAULT.pin);
    }
}

static inline void led_intr(int val) {
    if (val) {
        TG_PSOR(LED_ONBOARD.port) = (1 << LED_ONBOARD.pin);
    } else {
        TG_PCOR(LED_ONBOARD.port) = (1 << LED_ONBOARD.pin);
    }
}

#endif

#endif /* _MACHINE_TEENSY_H_ */
