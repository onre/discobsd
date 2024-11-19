#ifndef _MACHINE_TEENSY_H_
#define _MACHINE_TEENSY_H_

#ifdef TEENSY35

#        define TEENSY_LED_PIN_MASK (1 << 5)

void teensy_led_init(void);
void teensy_led_on(void);
void teensy_led_off(void);

#endif

#endif /* _MACHINE_TEENSY_H_ */
