#ifndef _MACHINE_TEENSY_H_
#define _MACHINE_TEENSY_H_

#include <machine/kinetis.h>

extern unsigned long _edata;
extern unsigned long _etext;
extern void (*_VectorsRam[102])(void);
extern unsigned long _sdata;


#ifdef TEENSY35

#define TEENSY_LED_PIN_MASK   (1 << 5)

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

#define LED_Y2                LED1
#define LED_Y1                LED2
#define LED_Y0                LED3
#define LED_G0                LED7

#define LED_SPL_BIT2          LED4
#define LED_SPL_BIT1          LED5
#define LED_SPL_BIT0          LED6

#define LED_INTR              LED8

#define _LED_PDDR(PRT, PN)    GPIO##PRT##_PDDR
#define _LED_PSOR(PRT, PN)    GPIO##PRT##_PSOR
#define _LED_PCOR(PRT, PN)    GPIO##PRT##_PCOR
#define _LED_PCR(PRT, PN)     PORT##PRT##_PCR##PN

#define _LED_BITMASK(PRT, PN) (1 << PN)

#define _LED_ON(PRT, PN)      _LED_PSOR(PRT, PN) = _LED_BITMASK(PRT, PN)
#define _LED_OFF(PRT, PN)     _LED_PCOR(PRT, PN) = _LED_BITMASK(PRT, PN)

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

void led_init(void);
void led_test(void);

void led_g4bit(int val);
void led_y3bit(int val);

void led_fault(int val);
void led_intr(int val);

/* once unix has booted up enough to have interrupts configured,
 * this is called to prevent the interrupt service routine from exploding.
 */
void usb_enable_spl(void);


#endif

#endif /* _MACHINE_TEENSY_H_ */
