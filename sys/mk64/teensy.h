#ifndef _MACHINE_TEENSY_H_
#define _MACHINE_TEENSY_H_

#include <machine/kinetis.h>

extern unsigned long _edata;
extern unsigned long _etext;
extern void (*_VectorsRam[102])(void);
extern unsigned long _sdata;


#ifdef TEENSY35

#define TEENSY_LED_PIN_MASK (1 << 5)



void teensy_led_init(void);
void teensy_led_on(void);
void teensy_led_off(void);

#define LED0_PORT             B
#define LED0_PIN              11
#define LED1_PORT             B
#define LED1_PIN              10
#define LED2_PORT             B
#define LED2_PIN              19
#define LED3_PORT             B
#define LED3_PIN              18
#define LED4_PORT             A
#define LED4_PIN              16
#define LED5_PORT             A
#define LED5_PIN              15
#define LED6_PORT             A
#define LED6_PIN              14
#define LED7_PORT             A
#define LED7_PIN              5
#define LED8_PORT             C
#define LED8_PIN              5

#define LED_FAULT             LED0
#define LED_SPL_HIGH          LED1
#define LED_SPL_CLOCK         LED2
#define LED_SPL_BIO           LED3
#define LED_SPL_TTY           LED4
#define LED_SPL_NET           LED5
#define LED_SPL_SOFTCLOCK     LED6
#define LED_SPL_LEAST         LED7
#define LED_INTR_ENA          LED8

#define _LED_PDDR(PRT, PN)    GPIO##PRT##_PDDR
#define _LED_PSOR(PRT, PN)    GPIO##PRT##_PSOR
#define _LED_PCOR(PRT, PN)    GPIO##PRT##_PCOR
#define _LED_PCR(PRT, PN)     PORT##PRT##_PCR##PN

#define _LED_BITMASK(PRT, PN) (1 << PN)

#define _LED_ON(PRT, PN)      _LED_PSOR(PRT, PN) |= _LED_BITMASK(PRT, PN)
#define _LED_OFF(PRT, PN)     _LED_PCOR(PRT, PN) |= _LED_BITMASK(PRT, PN)

#define _LED_ON_PORT(PRT)     PRT##_PORT
#define _LED_ON_PIN(PN)       PN##_PIN

#define LED_PIN_INIT(PRT, PN)                                          \
    _LED_PCR(PRT, PN) = PORT_PCR_MUX(1) | PORT_PCR_DSE | PORT_PCR_SRE; \
    _LED_PDDR(PRT, PN) |= _LED_BITMASK(PRT, PN);                       \
    _LED_ON(PRT, PN);                                                  \
    mdelay(20);                                                        \
    _LED_OFF(PRT, PN);

#define LED_ON(LEDNM)  _LED_ON(_LED_ON_PORT(LEDNM), _LED_ON_PIN(LEDNM))
#define LED_OFF(LEDNM) _LED_OFF(_LED_ON_PORT(LEDNM), _LED_ON_PIN(LEDNM))

#define LED_SPL_OFF                                                    \
    LED_OFF(LED1);                                                     \
    LED_OFF(LED2);                                                     \
    LED_OFF(LED3);                                                     \
    LED_OFF(LED4);                                                     \
    LED_OFF(LED5);                                                     \
    LED_OFF(LED6);                                                     \
    LED_OFF(LED7)

#define LED_SPL_ON                                                     \
    LED_ON(LED1);                                                     \
    LED_ON(LED2);                                                     \
    LED_ON(LED3);                                                     \
    LED_ON(LED4);                                                     \
    LED_ON(LED5);                                                     \
    LED_ON(LED6);                                                     \
    LED_ON(LED7)

static __inline void led_spl_least(void) {
    LED_SPL_OFF;
    LED_ON(LED_SPL_LEAST);
}

static __inline void led_spl_softclock(void) {
    LED_SPL_OFF;
    LED_ON(LED_SPL_SOFTCLOCK);
}

static __inline void led_spl_net(void) {
    LED_SPL_OFF;
    LED_ON(LED_SPL_NET);
}

static __inline void led_spl_tty(void) {
    LED_SPL_OFF;
    LED_ON(LED_SPL_TTY);
}

static __inline void led_spl_bio(void) {
    LED_SPL_OFF;
    LED_ON(LED_SPL_BIO);
}

static __inline void led_spl_clock(void) {
    LED_SPL_OFF;
    LED_ON(LED_SPL_CLOCK);
}

static __inline void led_spl_high(void) {
    LED_SPL_OFF;
    LED_ON(LED_SPL_HIGH);
}

static __inline void led_spl_top(void) {
    LED_SPL_ON;
}

static __inline void led_fault_on(void) { LED_ON(LED_FAULT); }

static __inline void led_fault_off(void) { LED_OFF(LED_FAULT); }

static __inline void led_intr_ena_on(void) { LED_ON(LED_INTR_ENA); }

static __inline void led_intr_ena_off(void) { LED_OFF(LED_INTR_ENA); }


void ledinit(void);

/* once unix has booted up enough to have interrupts configured,
 * this is called to prevent the interrupt service routine from exploding.
 */
void usb_enable_spl(void);


#endif

#endif /* _MACHINE_TEENSY_H_ */
