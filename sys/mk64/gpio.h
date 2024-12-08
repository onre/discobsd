#ifndef _MACHINE_GPIO_H
#define _MACHINE_GPIO_H

#include <sys/types.h>
#include <machine/kinetis.h>

#define NPINS           14


#define SPL_GPIO        SPL_BIO
#define splgpio()       splbio()

#define TG_NGPIO        5

#define TG_GPIO_A       0
#define TG_GPIO_B       1
#define TG_GPIO_C       2
#define TG_GPIO_D       3
#define TG_GPIO_E       4

#define TG_GPIO_PINMAX  31

#define TG_GPIO_IRQ(pn) (59 + pn)

#define TG_INPUT        0
#define TG_OUTPUT       1

#define TG_NORMAL       0
#define TG_PULLUP       1

#define TG_PCR_PIN_OUTPUT                                              \
    (PORT_PCR_MUX(1) | PORT_PCR_DSE | PORT_PCR_SRE)
#define TG_PCR_PIN_INPUT   (PORT_PCR_MUX(1))
#define TG_PCR_PIN_PULLUP  (PORT_PCR_MUX(1) | PORT_PCR_PS | PORT_PCR_PE)

#define TG_PCR_IRQ_OFF     ~(0xf << 16)
#define TG_PCR_IRQ_LOW     0x8
#define TG_PCR_IRQ_RISING  0x9
#define TG_PCR_IRQ_FALLING 0xa
#define TG_PCR_IRQ_EITHER  0xb
#define TG_PCR_IRQ_HIGH    0xc

#define TG_PCR_MASK(x)     (x << 16)

typedef union teensy_gpio_pin {
    struct {
        unsigned int tpin : 8;
        unsigned int port : 3;
        unsigned int pin : 5;
        unsigned int direction : 1; /* 0 = input, 1 = output */
        unsigned int pullup : 1;
        unsigned int interrupt : 4; /* TG_PCR_IRQ_* macros */
        unsigned int reserved : 9;
        unsigned int is_led : 1;
    };
    unsigned int raw;
} teensy_gpio_pin;

extern teensy_gpio_pin teensy_gpio_pins[NPINS];

#define TG_PORT_BASE 0x400FF000
#define TG_PCR_BASE  0x40049000

#define TG_PSOR(ptnm)                                                  \
    (volatile u_int *) (TG_PORT_BASE + (ptnm * 0x40) + 0x04)
#define TG_PCOR(ptnm)                                                  \
    (volatile u_int *) (TG_PORT_BASE + (ptnm * 0x40) + 0x08)
#define TG_PTOR(ptnm)                                                  \
    (volatile u_int *) (TG_PORT_BASE + (ptnm * 0x40) + 0x0C)
#define TG_PDIR(ptnm)                                                  \
    (volatile u_int *) (TG_PORT_BASE + (ptnm * 0x40) + 0x10)
#define TG_PDDR(ptnm)                                                  \
    (volatile u_int *) (TG_PORT_BASE + (ptnm * 0x40) + 0x14)
#define TG_PCR(ptnm, pnnm)                                             \
    (volatile u_int *) (TG_PCR_BASE + (ptnm * 0x1000) + (pnnm * 4))
#define TG_ISFR(ptnm)                                                  \
    (volatile u_int *) (TG_PCR_BASE + (ptnm * 0x1000) + 0xa0)
void teensy_gpio_init(void);
void teensy_gpio_set_isr(int port, int pin, int mode);
void portd_isr(void);

#define LED_FAULT   teensy_gpio_pins[0]
#define LED_ONBOARD teensy_gpio_pins[NPINS - 2]
#define BUTTON_USER teensy_gpio_pins[NPINS - 1]

static inline void led_fault(int val) {
    *(val ? (TG_PSOR(LED_FAULT.port)) : (TG_PCOR(LED_FAULT.port))) =
        (1 << LED_FAULT.pin);
}

static inline void led_intr(int val) {
    *(val ? (TG_PSOR(LED_ONBOARD.port)) : (TG_PCOR(LED_ONBOARD.port))) =
        (1 << LED_ONBOARD.pin);
}

static inline void teensy_gpio_led_value(int val) {
    val &= 0xff;
    GPIOB_PCOR = ((3 << 18) | (3 << 10) | 0xF);

    GPIOB_PSOR =
        ((val & 0x3) | ((val >> 1) & 0x4) | ((val << 1) & 0x8) |
         ((val << 14) & (0x3 << 18)) | ((val << 4) & (0x3 << 10)));
}

static inline void teensy_gpio_led_spl(int val) {
    val &= 0x7;
    GPIOA_PCOR = (7 << 14);
    GPIOA_PSOR = (val << 14);
}



#endif
