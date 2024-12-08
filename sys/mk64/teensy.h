#include <sys/types.h>

#include <machine/intr.h>
#include <machine/kinetis.h>

#ifndef _MACHINE_TEENSY_H_
#define _MACHINE_TEENSY_H_


extern uint32_t _edata;
extern uint32_t _etext;
extern void (*_VectorsRam[102])(void);
extern uint32_t _sdata;


#ifdef TEENSY35

#define IDLE_LED_SHOW 0 

#define TEENSY_LED_PIN_MASK (1 << 5)


#endif

#endif /* _MACHINE_TEENSY_H_ */
